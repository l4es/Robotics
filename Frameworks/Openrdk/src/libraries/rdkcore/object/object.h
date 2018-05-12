/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007, 2008  Daniele Calisi, Andrea Censi (<first_name>.<last_name>@dis.uniroma1.it)
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef RDK2_OBJECT_OBJECT
#define RDK2_OBJECT_OBJECT

#include <list>
#include <vector>
#include <map>
#include <string>
#include <rdkcore/demangle/demangle.h>
#include <rdkcore/serialization/read.h>
#include <rdkcore/serialization/write.h>
#include <rdkcore/time/time.h>

namespace RDK2 { namespace Meta {
		class ObjectDiff;
}}

namespace RDK2 {

	/// @brief La nuova e purificata RDK2::Object.
	///
	/// @par Rules and conventions about RDK2::Objects:
	///
	/// Every subclass should implement clone().
	///
	/// Every subclass which is to be added in a factory must implement
	/// a constructor with no arguments.
	///
	/// Copying of RDK2::Objects occurs only with clone(), please do not rely
	/// on copy constructors, operator=, casts or other means.
	/// RDK2::Object are usually passed around as pointers.
	///

	class Object: public RDK2::Serialization::Readable,
	               public RDK2::Serialization::Writable,
	               public RDK2::Demangle::Reflective
	{
	public:
		Object() : creationTimestamp(), upTimestamp() { }
		virtual ~Object() {}

		/// Returns a copy of the object.
		/// @return 0 if not implemented
		virtual Object* clone() const;

		#define RDK2_DEFAULT_CLONE(Klass) \
			virtual RDK2::Object* clone() const { return new Klass(*this); }

	protected:
		/// This member is the timestamp when the object is created
		/// it is const and the default constructor initialize it to the current timestamp
		RDK2::Time::Timestamp creationTimestamp;

		/// This member is the name of the creator of this object (e.g., the session that created this object)
		/// this has to be set by hand
		string creatorName;

		/// This member is the timestamp of last update performed on object,
		/// should be updated every time a property of the object changes
		RDK2::Time::Timestamp upTimestamp;

		/// This member is the name of the last updater of this object (e.g., the session the modified this object)
		/// this has to be set using the functions updateTimestamp with the second parameter
		string lastUpdaterName;

	public:
		inline const RDK2::Time::Timestamp& getCreationTimestamp() const { return creationTimestamp; }
		inline const RDK2::Time::Timestamp& getUpdateTimestamp() const { return upTimestamp; }
		inline const string& getCreatorName() const { return creatorName; }
		inline const string& getLastUpdaterName() const { return lastUpdaterName; }
		void updateTimestamp(const std::string& updaterName = "") { upTimestamp.setToNow(); lastUpdaterName = updaterName; }
		void updateTimestamp(const RDK2::Time::Timestamp& t, const std::string& updaterName = "")
			{ upTimestamp = t; lastUpdaterName = updaterName; }
		void setCreatorName(const std::string& creatorName);

		/// @name Serialization
		/// Functions for dealing with serialization..
		//@{
		/// These default implementations of Writable and Readable serialize
		/// empty objects.
		virtual void read(RDK2::Serialization::Reader*r) throw (ReadingException);
		virtual void write(RDK2::Serialization::Writer*w) const throw (WritingException);
		/// These functions are helper for derived class
		/// They do should contain real reading and writing of object member
		virtual void readObjectBody(RDK2::Serialization::Reader*r) throw (ReadingException);
		virtual void writeObjectBody(RDK2::Serialization::Writer*w) const throw (WritingException);
		//@}

		/// @name Diffs
		/// Functions for big objects; known big objects: Vector(s), RImage, RMapImage
		//@{
		virtual bool knowsDiffs() { return false; }
		virtual bool applyDiff(const RDK2::Meta::ObjectDiff*) { return false; }
		virtual vector<RDK2::Meta::ObjectDiff*> splitInDiffs(size_t) {
			vector<RDK2::Meta::ObjectDiff*> v; return v; }
		//@}

		/// @name StringRepresentationsAndVisualizations
		/// Functions for dealing with string representation of the robjects..
		//@{
		/// Ritorna true se ha una rappresentazione "semplice" come stringa.
		virtual bool hasStringRepresentation() const { return false; }

		/// Ritorna la rappresentazione come stringa.
		virtual std::string getStringRepresentation() const;

		/// Inversa della precedente.
		virtual bool loadFromStringRepresentation(const std::string&) { return false; }

		/// Returns true if this object has a string that can be visualized.
		virtual bool hasStringForVisualization() const { return false; }

		/// Returns a string to be used to visualize the object.
		virtual std::string getStringForVisualization() const;
		//@}

	/// TODO:
	///
	/// @name TestSupport
	/// These functions are used by the built-in test system..
	//@{

		/// Get a vector of test cases.
		virtual std::vector<Object*> getTestCases() const {
			return std::vector<Object*>(); }

		/// Compare this and another object.
		/// @note For clarity, we decided not to use operator=.
		///
		/// @return   true   if they are equivalent
		virtual bool equals(const Object*) const { return false; };

	//@}

	};

	/// To add a factory, just add a line like this:
	///	RDK2_FACTORY(MyClass)
	/// in a cpp file. This creates a static object of class Factory (see below)
	/// whose constructor will be executed at init. time and will put
	/// a prototype in the database.

	#define RDK2_FACTORY(classe) \
		RDK2::Meta::Factory factory_ ## classe (new classe());

	namespace Meta {
		class Factory {
			public:
				Factory(const RDK2::Object*prototype);
		};

		// Static object used for holding data
        	typedef std::map<std::string, const RDK2::Object*> Class2prot;

        	Class2prot& getClass2prot();

	}


} // namespace


#endif
