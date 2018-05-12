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

#ifndef H_RAGENT_CONFIG_PAIR
#define H_RAGENT_CONFIG_PAIR

#include <rdkcore/container/container.h>

namespace RDK2 { namespace RAgent {

	using namespace RDK2::Containers;

	struct Pair: public RDK2::Object  {
		string url;
		RDK2::Object *value;
		
		Pair() : url("undefined"), value(0) {}
		
		Pair(cstr s, RDK2::Object*v) : url(s), value(v) {}
		
		void read(Reader*r) throw (ReadingException) {
			r->startReading("Pair");
			url = r->readString();
			value = static_cast<Object*>(r->readObject());
			r->doneReading();
		}
		
		void write(Writer*w) const throw (WritingException) {
			if(!value)
				throw WritingException("Could not serialize Pair with null value.");

			w->startWriting("Pair");
				w->writeString(url);
				w->writeObject(true, value);
			w->doneWriting();
		}
		
		RDK2::Object * clone() const { return new Pair(*this); }
	};
	
	typedef RDK2::Containers::Vector<Pair> PropertyList;
		
}} // namespace RDK2::RAgent

#endif
