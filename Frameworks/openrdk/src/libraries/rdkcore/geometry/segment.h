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

#ifndef RDK2_GEOMETRY_SEGMENT
#define RDK2_GEOMETRY_SEGMENT

#include "segment2.h"
#include "point.h"

namespace RDK2 { namespace Geometry {

typedef Segment2<int>    Segment2i;
typedef Segment2<double> Segment2d;

bool sedgewickIntersect(const Point2d& s0p0, const Point2d& s0p1, const Point2d& s1p0, const Point2d& s1p1);

}} // ns

#endif
