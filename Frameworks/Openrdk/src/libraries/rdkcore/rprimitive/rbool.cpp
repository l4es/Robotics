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

#include "rbool.h"

namespace RDK2 { namespace RPrimitive {
	
	RDK2_FACTORY(RBool);
	
	static const char * d = "RBool";
	const char * RBool::myClassName() const {
		return d;
	}
	
	
	void RBool::read(RDK2::Serialization::Reader*r) throw (ReadingException) {
		r->startReading(getClassName());
			value = r->read_i8();
		r->doneReading();
	}

	void RBool::write(RDK2::Serialization::Writer*w) const throw (WritingException)  {
		w->startWriting(getClassName());
			w->write_i8(value?1:0);
		w->doneWriting();
	}


	std::string RBool::getStringRepresentation() const {
		return value ? "Yes" : "No"; 
	}
	
	bool RBool::loadFromStringRepresentation(const std::string& s) {
		if (s == "1" || s == "Yes" || s == "yes" || s == "True" || s == "true") {
			value = true;
			return true;
		}
		else if (s == "0" || s == "No" || s == "no" || s == "False" || s == "false") {
			value = false;
			return true;
		}
		else return false;
	}
	
}}
