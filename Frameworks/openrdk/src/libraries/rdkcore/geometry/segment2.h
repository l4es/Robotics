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

#ifndef RDK2_GEOMETRY_SEGMENT2
#define RDK2_GEOMETRY_SEGMENT2

#include <string>
#include <sstream>
#include <assert.h>
#include <cmath>

#include "point.h"

namespace RDK2 { namespace Geometry {

using std::sqrt;
using std::cos;
using std::sin;

/**
 * A generic vector with two components. 
 *
 * Note: by design, none of the methods changes the values of the members.
 *
 * @author  Andrea Censi  <andrea.censi@dis.uniroma1.it>
 * @author Daniele Calisi <calisi@dis.uniroma1.it>
 * @author Alberto Ingenito (serialize/deserialize) <alberto.ing@gmail.com>
*/

template <typename Numeric> 
struct Segment2 {
	Point2<Numeric> p0;
	Point2<Numeric> p1;

	Segment2() : p0(0, 0), p1(0, 0) { }
	Segment2(Point2<Numeric> p0, Point2<Numeric> p1) : p0(p0), p1(p1) { }
	Segment2(Numeric p0x, Numeric p0y, Numeric p1x, Numeric p1y) : p0(p0x, p0y), p1(p1x, p1y) { }

}; // template Point2

}} // ns

#endif
