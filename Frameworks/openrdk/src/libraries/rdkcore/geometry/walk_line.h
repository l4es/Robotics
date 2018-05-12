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

#ifndef RDK2_GEOMETRY_WALK_LINE
#define RDK2_GEOMETRY_WALK_LINE

/*
 * Author: Daniele Calisi <calisi@dis.uniroma1.it>
 */

#include <cmath>
#include <rdkcore/geometry/point.h>

namespace RDK2 { namespace Geometry {

template <typename Numeric>
class Line2 {
public:
	Line2() : p0(0, 0), p1(0, 0) { }
	Line2(Numeric x0, Numeric y0, Numeric x1, Numeric y1) : p0(x0, y0), p1(x1, y1) { }
	Line2(Point2<Numeric> p0, Point2<Numeric> p1) : p0(p0), p1(p1) { }
	
	Point2<Numeric> p0, p1;
};

typedef Line2<double> Line2d;
typedef Line2<int> Line2i;

class LineWalk
{
public:
	LineWalk(Point2i p0, Point2i p1);
	
	bool next();
	
	void rewind();
	
	Point2i getPoint() const;
		
//protected:
	int vx, vy;			// directions
	double dx, dy;			// increments
	Point2i p0, p1;			// points
	unsigned int steps;		// max(pixels_x, pixels_y)
	unsigned int currentStep;
};

}} // ns

#endif
