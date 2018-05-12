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

#ifndef R_INT_H
#define R_INT_H

#include "rprimitive.h"

namespace RDK2 { namespace RPrimitive {
	
	struct RInt: public RPrimitive<int> {
		RInt(int value=0) : RPrimitive<int>(value) {}	
	
		// Interfaccia RPrimitive
		const char* myClassName() const { return "RInt"; };
		
		bool hasStringForVisualization() const { return true; }

		void read(RDK2::Reader*r) throw (RDK2::ReadingException);
		void write(RDK2::Writer*w) const  throw (RDK2::WritingException);
		RDK2_DEFAULT_CLONE(RInt);
	};

}}


#endif
