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

#include <rdkcore/posixconstructs/stack.h>

#include "property.h"


namespace RDK2 { namespace RepositoryNS {
	Property::~Property() {
		// FIXME: devo cancellare?
	}

	#define SET_OPTION(opt, yesorno) if (yesorno) options = (PropertyOptions) (options | opt); \
		else options = (PropertyOptions) (options & ~opt)

	bool Property::isQueue() { return objectClass == QUEUE_CLASSNAME; }
	bool Property::isStorage() { return objectClass != QUEUE_CLASSNAME; }
	bool Property::isReadonly() { return options & READ_ONLY; }
	void Property::setReadonly(bool readonly) { 
		lock(HERE); SET_OPTION(READ_ONLY, readonly); unlock(); }
	bool Property::isPersistent() { return !(options & NOT_PERSISTENT); }
	void Property::setPersistent(bool persistent) { 
		lock(HERE); SET_OPTION(NOT_PERSISTENT, !persistent); unlock(); }
	bool Property::isEnum() { return isEnumB; }
	bool Property::isLink() { return isLinkB; }

	const RDK2::Object* Property::getObjectLC() { 
		return object ? object : defaultObject; 
	}
	
	RDK2::Object* Property::getObjectL() { 
		return object ? object : 
			(defaultObject ? (object = defaultObject->clone(), object) : 0); 
	}

	void Property::setObject(RDK2::Object* object) throw (WrongType) {
		lock(WHERE("Setting object"));
		setObjectL(object);
		unlock();
	}

	void Property::setObjectL(RDK2::Object* object) throw (WrongType) {
		if (objectClass == "") 
			setObjectClassName(object->getClassName());  // XXX are we sure?
		else { 
			if (objectClass != object->getClassName()) {
				throw WrongType(
					string() + "Object of wrong type for '" + url + "' (should be '"
				+ objectClass + "', is '" + object->getClassName() + "')");
			}

			delete this->object;    // "delete 0" is safe (Stroustrup docet)
			this->object = object;
		}
	}



void Property::setObjectClassName(cstr objectClass) {
	// NOTE: objectClass is "" for remote properties that are still not updated for the first time
	if (this->objectClass == "") this->objectClass = objectClass;
	else throw InvalidOperation("You cannot change object class");
}
string Property::getObjectClassName() const { return objectClass; }

// PropertyDef

void PropertyDef::setObjectClassName(cstr objectClassName) throw (InvalidOperation)
{
	// NOTE: objectClass is "" for remote properties that are still not updated for the first time
	if (this->objectClassName == "") {
		this->objectClassName = objectClassName;
		// setting default options
		if (isClassPersistent(objectClassName)) setPersistent();
		else setVolatile();
	}
	else throw InvalidOperation("You cannot change object class once it is set");
}

bool PropertyDef::isClassPersistent(cstr objectClassName)
{
		if (objectClassName == "RDouble" || objectClassName == "RInt"
		|| objectClassName == "RString" || objectClassName == "RPoint2od"
		|| objectClassName == "RBool" || objectClassName == "RC8Set") {
			return true;
		}
		else {
			return false;
		}
}

bool PropertyDef::isClassKeepThis(cstr /*objectClassName*/)
{
	return false;
}

bool PropertyDef::isClassVolatile(cstr objectClassName)
{
	return !isClassPersistent(objectClassName);
}

string PropertyDef::getObjectClassName() const { return objectClassName; }

}}



