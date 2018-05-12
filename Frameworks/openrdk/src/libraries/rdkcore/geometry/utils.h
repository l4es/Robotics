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

#ifndef RDK2_GEOMETRY_UTILS
#define RDK2_GEOMETRY_UTILS

#include <sstream>
#include <string>
#include <cmath>

//#include "HomogTrans.h"
//#include "Location.h"

namespace RDK2 { namespace Geometry {

inline double deg2rad(double d ) { return d * M_PI / 180; }
inline double rad2deg(double d ) { return d * 180 / M_PI; }
inline double    m2mm(double m ) { return      m *  1000; }
inline double    mm2m(double mm) { return     mm * 0.001; }

}}

#endif
