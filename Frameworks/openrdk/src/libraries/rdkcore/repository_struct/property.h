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

#ifndef RDK2_REPOSITORY_PROPERTY
#define RDK2_REPOSITORY_PROPERTY

#include <set>

#include <rdkcore/posixconstructs/posixmutex.h>
#include <rdkcore/object/object.h>
#include <rdkcore/rprimitive/rdouble.h>
#include <rdkcore/repository_struct/robjectqueue.h>

#include "url.h"

#define QUEUE_CLASSNAME "RObjectPipeQueue"

namespace RDK2 { namespace RepositoryNS {
	using namespace PosixConstructs;

class Session;

struct PropertyEnumItem {
	PropertyEnumItem(uint value, string name, string description) :
		name(name), description(description), value(value) { }

	string name;
	string description;
	uint value;
};

// FIXME deprecated
enum PropertyOptions {
	NORMAL = 0x00,
	READ_ONLY = 0x01,
	DONT_SAVE = 0x02,
	INFO = READ_ONLY | DONT_SAVE,
	NOT_PERSISTENT = DONT_SAVE		// deprecated
};

enum PropertyOptionsEnum {
	// persistence options (first two bits)
	PERSISTENT = 0x00,	// the property will be loaded from the file and then saved to it (default)
	VOLATILE = 0x01,	// the property will NEVER be loaded, and will be saved NULL
	KEEP_THIS = 0x02,	// the property will be loaded, but NEVER overwritten
	// rconsole options (third bit)
	EDITABLE = 0x00,	// rconsole operator can edit this property (default)
	NOT_EDITABLE = 0x04	// rconsole operator can only see this property
};

class PropertyDef {	// FIXME da seguire QUESTA strada
protected:
	std::string objectClassName;
	std::string description;
	
	Url linkTo;
	
	int persistenceOptions;
	bool rconsoleEditable;
	string externalFileName;
	bool externalFileBinary;
	
public:
	PropertyDef(int poe = (PERSISTENT | EDITABLE)) : 
		linkTo(""), persistenceOptions(PERSISTENT), rconsoleEditable(true),
		externalFileName(""), externalFileBinary(false)
	{ 
		if (poe & (KEEP_THIS | VOLATILE)) {
			persistenceOptions |= KEEP_THIS;	// defensive programming
			throw InvalidOperation("May a thunder catch you, you cannot set a property "
				"to be both KEEP_THIS and VOLATILE! (" + description + ")");
		}
		else {
			if (poe & VOLATILE) persistenceOptions |= VOLATILE;
			if (poe & KEEP_THIS) persistenceOptions |= KEEP_THIS;
		}
		if (poe & NOT_EDITABLE) rconsoleEditable = false;
	}
	
	// link
	inline Url getLinkTo() const { return linkTo; }
	inline bool isLink() const { return linkTo != ""; }
	inline void setLinkTo(CUrl url) { linkTo = url; }
	
	// description and class name
	inline string getDescription() const { return description; }
	inline void setDescription(cstr s) { description = s; }
	string getObjectClassName() const;
	void setObjectClassName(cstr s) throw (InvalidOperation);
	
	// persistence
	inline bool isPersistent() const { return (persistenceOptions & (VOLATILE | KEEP_THIS)) == 0; }
	inline bool isVolatile() const { return persistenceOptions & VOLATILE; }
	inline bool isKeepThis() const { return persistenceOptions & KEEP_THIS; }
	inline void setPersistent() { persistenceOptions = persistenceOptions & ~(VOLATILE | KEEP_THIS); }
	inline void setVolatile() { persistenceOptions = (persistenceOptions & ~KEEP_THIS) | VOLATILE; }
	inline void setKeepThis() { persistenceOptions = (persistenceOptions & ~VOLATILE) | KEEP_THIS; }
	
	// persistence / external file name
	inline string getExternalFileName() const { return externalFileName; }
	inline void setExternalFileName(cstr fileName) { externalFileName = fileName; }
	inline bool isExternalSaved() const { return externalFileName != ""; }
	inline bool isExternalFileBinary() const { return externalFileBinary; }
	inline bool isExternalFileXml() const { return !externalFileBinary; }
	inline void setExternalFileBinary() { externalFileBinary = true; }
	inline void setExternalFileXml() { externalFileBinary = false; }
	
	// rconsole options
	inline bool isEditable() const { return rconsoleEditable; }
	inline bool isNotEditable() const { return !rconsoleEditable; }
	inline void setEditable() { rconsoleEditable = true; }
	inline void setNotEditable() { rconsoleEditable = false; }
	
	// static members
	static bool isClassPersistent(cstr objectClassName);
	static bool isClassVolatile(cstr objectClassName);
	static bool isClassKeepThis(cstr objectClassName);
};

class Property : public PosixConstructs::PosixMutex {
protected:
	std::string objectClass;
public:	// FIXME
	RDK2::Object* object;
	const RDK2::Object* defaultObject;
	Url url;
	
	/** this is the current definition of this property */
	PropertyDef def;
	
	/** this is the default definition of this property, decided in initConfigurationProperties */
	PropertyDef defaultDef;

	std::string description;

	PropertyOptions options;

	PropertyDef getPropertyDef() { return def; }
	PropertyDef getDefaultPropertyDef() { return defaultDef; }
	void saveDefaultPropertyDef() { defaultDef = def; }
	
	/// Valid when property is an Enum
	bool isEnumB;    // XXX non mi piace granch�
	vector<PropertyEnumItem> enumItems;
	
	/// Valid when property is a link
	bool isLinkB;
	Url linkTo;
	set<Url> linkedBy;
	
	bool deleted;

	/// Valid when property has to be saved externally
	bool savedExternally;
	string externalFilename;
	
	/// Valid when property is a double
	RDK2::RPrimitive::RDouble::Unit defaultDoubleUnit;        // XXX

	/// Stuff for people listening to this property's diffs
	RObjectPipeQueue diffsQueue;
	set<Session*> listeningSessions;

public:
	Property(Url url, cstr objectClass, RDK2::Object* defaultObject = 0) :
		objectClass(objectClass),
		object(0),
		defaultObject(defaultObject),
		url(url),
		options(NORMAL),
		isEnumB(false),
		isLinkB(false),
		linkTo(""),
		deleted(false),
		savedExternally(false),
		externalFilename(""),
		diffsQueue("diffsQueue")
	{ def.setObjectClassName(objectClass); }
	virtual ~Property();

	void setObject(RDK2::Object* object) throw (WrongType);
	void setObjectL(RDK2::Object* object) throw (WrongType);

	RDK2::Object* getObjectL();
	const RDK2::Object* getObjectLC();
	template <typename T> T* getObjectAsL() throw (WrongType, ValueNotSet);
	template <typename T> const T* getObjectAsLC() throw (WrongType, ValueNotSet);

	bool isQueue();
	bool isStorage();
	bool isReadonly();
	void setReadonly(bool readonly);
	bool isPersistent();
	void setPersistent(bool persistent);
	bool isEnum();
	bool isLink();

	void setObjectClassName(cstr objectClass);
	
	/// FIXME: referenza circolare: rforeignproperties ha
	/// rpropertydef che deriva da Property, quindi il link non puo riuscire
	/// workaround: mettere questa funzione nel .h
	string getObjectClassName() const;

	friend class Repository;
	friend class Session;
};

#include "property.hpp"

}} // namespaces

#endif
