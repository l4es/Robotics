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

#include <cstdio>
#include "rstring.h"

namespace RDK2 { namespace RPrimitive {

	RDK2_FACTORY(RString);

	using RDK2::Object;
	using namespace std;

	RString::RString(string text) : value(text) { }


	Object* RString::clone() const{
		return new RString(value);
	}

	bool RString::hasStringRepresentation() const {
		return true;
	}

	std::string RString::getStringRepresentation() const {
		return value;
	}

	bool RString::loadFromStringRepresentation(const std::string&t) {
		value = t;
		return true;
	}

	void RString::read(RDK2::Serialization::Reader*r) throw (ReadingException) {
		r->startReading(getClassName());
			value = r->readString();
		r->doneReading();
	}

	void RString::write(RDK2::Serialization::Writer*w) const throw (WritingException)  {
		w->startWriting(getClassName());
			w->writeString(value);
		w->doneWriting();
	}

	std::vector<Object*> RString::getTestCases() const {
		std::vector<Object*> v;
		v.push_back(new RString("Hello"));
		v.push_back(new RString("Hello> >> ><a></b>"));
		v.push_back(new RString(" "));
		v.push_back(new RString(""));
		return v;
	}

	bool RString::equals(const Object*ob) const {
		const RString * s = dynamic_cast<const RString*>(ob);
		return s && (s->value == value);
	}

}}








