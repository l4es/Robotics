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

#include "rdkmath.h"
#include <cstdio>

namespace RDK2 {

bool string2float(const std::string& s, float &v)
{
	if(s==RDK_INF_STRING) { v = RDK_INF32; return true; } 
	if(s==RDK_NAN_STRING) { v = RDK_NAN; return true; } 
	return 1 == sscanf(s.c_str(), "%f", &v);
}

bool string2float(const std::string& s, double &v) 
{
	if(s==RDK_INF_STRING) { v = RDK_INF64; return true; } 
	if(s==RDK_NAN_STRING) { v = RDK_NAN; return true; } 
	return 1 == sscanf(s.c_str(), "%lf", &v);
}

}
