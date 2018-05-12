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

#include "segment.h"

namespace RDK2 { namespace Geometry {

// Sedgewick's algorithm (segments intersection)
int sedgewick_ccw(const Point2d& p0, const Point2d& p1, const Point2d& p2)
{
	double dx1 = p1.x - p0.x;
	double dy1 = p1.y - p0.y;
	double dx2 = p2.x - p0.x; 
	double dy2 = p2.y - p0.y;
	if (dx1*dy2 > dy1*dx2) return +1;
	if (dx1*dy2 < dy1*dx2) return -1;
	if ((dx1*dx2 < 0) || (dy1*dy2 < 0)) return -1;
	if ((dx1*dx1+dy1*dy1) < (dx2*dx2+dy2*dy2)) return +1;
	return 0;
}

// Sedgewick's algorithm (segments intersection)
bool sedgewickIntersect(const Point2d& s0p0, const Point2d& s0p1, const Point2d& s1p0, const Point2d& s1p1)
{
	return ((sedgewick_ccw(s0p0, s0p1, s1p0) * sedgewick_ccw(s0p0, s0p1, s1p1)) <= 0)
	    && ((sedgewick_ccw(s1p0, s1p1, s0p0) * sedgewick_ccw(s1p0, s1p1, s0p1)) <= 0);
}

}} // namespaces
