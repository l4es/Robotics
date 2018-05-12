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

#ifndef RSTRING_H
#define RSTRING_H
// Ruby ha una struct chiamata "RString"
#ifndef SWIGRUBY

#include <rdkcore/object/object.h>
#include <string>

namespace RDK2 { namespace RPrimitive {

	// TODO: pensare all'escaping
	// FIXME: DC: non ho capito perch� non discende da RPrimitive
	class RString: public RDK2::Object {
		public:
			RString(std::string text="");

			bool hasStringRepresentation() const;
			std::string getStringRepresentation() const;
			bool loadFromStringRepresentation(const std::string&);
			bool hasStringForVisualization() const { return true; }

			void read(RDK2::Reader*r) throw (RDK2::ReadingException);
			void write(RDK2::Writer*w) const  throw (RDK2::WritingException);
			virtual Object* clone() const;

			operator const std::string&() const { return value; }

			std::vector<Object*> getTestCases() const;
			bool equals(const Object*) const;

		public:
			std::string value;
	};

}}

// #ifndef SWIGRUBY
#endif

#endif
