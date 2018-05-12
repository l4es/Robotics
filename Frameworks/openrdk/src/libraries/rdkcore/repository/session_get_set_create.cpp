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

#include "session.h"

namespace RDK2 { namespace RepositoryNS {

// CREATE functions

void Session::createBool(CUrl url, cstr description, PropertyOptions options)
	throw (InvalidOperation)
{ createStorage("RBool", url, description); setPropertyOptions(url, options); }

void Session::createBool(CUrl url, cstr description, bool defaultValue, PropertyOptions options)
	throw (InvalidOperation)
{ createStorage("RBool", url, description, new RBool(defaultValue)); setPropertyOptions(url, options); }

void Session::createInt(CUrl url, cstr description, PropertyOptions options)
	throw (InvalidOperation)
{ createStorage("RInt", url, description); setPropertyOptions(url, options); }

void Session::createInt(CUrl url, cstr description, int defaultValue, PropertyOptions options)
	throw (InvalidOperation)
{ createStorage("RInt", url, description, new RInt(defaultValue)); setPropertyOptions(url, options); }

void Session::createString(CUrl url, cstr description, PropertyOptions options)
	throw (InvalidOperation)
{ createStorage("RString", url, description); setPropertyOptions(url, options); }

void Session::createString(CUrl url, cstr description, cstr defaultValue, PropertyOptions options)
	throw (InvalidOperation)
{ createStorage("RString", url, description, new RString(defaultValue)); setPropertyOptions(url, options); }

void Session::createDouble(CUrl url, cstr description, RDouble::Unit unit, PropertyOptions options)
	throw (InvalidOperation)
{
	createStorage("RDouble", url, description);
	getProperty(url)->defaultDoubleUnit = unit;
	setPropertyOptions(url, options);
}

void Session::createDouble(CUrl url, cstr description, RDouble::Unit unit,
	double defaultValue, PropertyOptions options)
	throw (InvalidOperation)
{
	createStorage("RDouble", url, description, new RDouble(defaultValue, unit));
	getProperty(url)->defaultDoubleUnit = unit;
	setPropertyOptions(url, options);
}

void Session::createPose(CUrl url, cstr description)
	throw (InvalidOperation)
{ createStorage("RPoint2od", url, description); }

void Session::createPose(CUrl url, cstr description, const Point2od& defaultValue)
	throw (InvalidOperation)
{ createStorage("RPoint2od", url, description, new RPoint2od(defaultValue)); }


void Session::createPoint(CUrl url, cstr description)
	throw (InvalidOperation)
{ createStorage("RPoint2d", url, description); }

void Session::createPoint(CUrl url, cstr description, const Point2d& defaultValue)
	throw (InvalidOperation)
{ createStorage("RPoint2d", url, description, new RPoint2d(defaultValue)); }


void Session::createPose3(CUrl url, cstr description)
	throw (InvalidOperation)
{ createStorage("RPoint3od", url, description); }

void Session::createPose3(CUrl url, cstr description, const Point3od& defaultValue)
	throw (InvalidOperation)
{ createStorage("RPoint3od", url, description, new RPoint3od(defaultValue)); }


void Session::createPoint3(CUrl url, cstr description)
	throw (InvalidOperation)
{ createStorage("RPoint3d", url, description); }

void Session::createPoint3(CUrl url, cstr description, const Point3d& defaultValue)
	throw (InvalidOperation)
{ createStorage("RPoint3d", url, description, new RPoint3d(defaultValue)); }


// GET functions

bool Session::getBool(CUrl url)
	throw (InvalidOperation, NoSuchProperty, WrongType, ValueNotSet)
{ return getValue<bool, RBool>(url); }

int Session::getInt(CUrl url)
	throw (InvalidOperation, NoSuchProperty, WrongType, ValueNotSet)
{ return getValue<int, RInt>(url); }

double Session::getDouble(CUrl url)
	throw (InvalidOperation, NoSuchProperty, WrongType, ValueNotSet)
{ return getValue<double, RDouble>(url); }

string Session::getString(CUrl url)
	throw (InvalidOperation, NoSuchProperty, WrongType, ValueNotSet)
{ return getValue<string, RString>(url); }

Point2od Session::getPose(CUrl url)
	throw (InvalidOperation, NoSuchProperty, WrongType, ValueNotSet)
{ return getValue<Point2od, RPoint2od>(url); }

Point3od Session::getPose3(CUrl url)
	throw (InvalidOperation, NoSuchProperty, WrongType, ValueNotSet)
{ return getValue<Point3od, RPoint3od>(url); }

Point2d Session::getPoint(CUrl url)
	throw (InvalidOperation, NoSuchProperty, WrongType, ValueNotSet)
{ return getValue<Point2d, RPoint2d>(url); }

Point3d Session::getPoint3(CUrl url)
	throw (InvalidOperation, NoSuchProperty, WrongType, ValueNotSet)
{ return getValue<Point3d, RPoint3d>(url); }

// SET functions

void Session::setBool(CUrl url, bool i)
	throw (InvalidOperation, NoSuchProperty, WrongType)
{ setObject(url, new RBool(i)); }

void Session::setInt(CUrl url, int i)
	throw (InvalidOperation, NoSuchProperty, WrongType)
{ setObject(url, new RInt(i)); }

void Session::setDouble(CUrl url, double i)
	throw (InvalidOperation, NoSuchProperty, WrongType)
{ setObject(url, new RDouble(i, getProperty(url)->defaultDoubleUnit)); }

void Session::setString(CUrl url, const string& i)
	throw (InvalidOperation, NoSuchProperty, WrongType)
{ setObject(url, new RString(i)); }

void Session::setPose(CUrl url, const Point2od& i)
	throw (InvalidOperation, NoSuchProperty, WrongType)
{ setObject(url, new RPoint2od(i)); }

void Session::setPose3(CUrl url, const Point3od& i)
	throw (InvalidOperation, NoSuchProperty, WrongType)
{ setObject(url, new RPoint3od(i)); }

void Session::setPoint(CUrl url, const Point2d& i)
	throw (InvalidOperation, NoSuchProperty, WrongType)
{ setObject(url, new RPoint2d(i)); }

void Session::setPoint3(CUrl url, const Point3d& i)
	throw (InvalidOperation, NoSuchProperty, WrongType)
{ setObject(url, new RPoint3d(i)); }


// ENUMS

void Session::createEnum(CUrl url, cstr description,
	const vector<PropertyEnumItem>& items,
	uint defaultValue, PropertyOptions options)
	throw (InvalidOperation)
{
	createStorage("RInt", url, description, new RInt(defaultValue));
	Property* p = getProperty(url);
	Profiler::lock((sessionName + ":" + url).c_str());
	p->lock(HERE);
	p->enumItems = items;
	p->isEnumB = true;
	p->unlock();
	Profiler::unlock((sessionName + ":" + url).c_str());
	setPropertyOptions(url, options);
}

void Session::setEnum(CUrl url, uint value) throw (InvalidOperation, NoSuchProperty, WrongType)
{
	setObject(url, new RInt(value));
}

bool Session::isEnum(CUrl url) throw (InvalidOperation, NoSuchProperty)
{
	return getProperty(url)->isEnum();
}

string Session::getEnumString(CUrl url)
	throw (InvalidOperation, NoSuchProperty, WrongType, ValueNotSet)
{
	Property* p = getProperty(url);
	Profiler::lock((sessionName + ":" + url).c_str());
	p->lock(HERE);
	uint v;
	try {
		v = p->getObjectAsLC<RInt>()->value;
	}
	catch (const SessionException& e) {
		p->unlock();
		Profiler::unlock((sessionName + ":" + url).c_str());
		throw e;
	}
	for (size_t i = 0; i < p->enumItems.size(); i++) {
		if (p->enumItems[i].value == v) {
			p->unlock();
			Profiler::unlock((sessionName + ":" + url).c_str());
			return p->enumItems[i].name;
		}
	}
	p->unlock();
	Profiler::unlock((sessionName + ":" + url).c_str());
	return "<unknown>";
}

}} // namespaces
