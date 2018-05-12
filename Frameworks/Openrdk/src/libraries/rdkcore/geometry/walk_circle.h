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

#ifndef RDK2_GEOMETRY_WALK_CIRCLE
#define RDK2_GEOMETRY_WALK_CIRCLE

#include <cmath>
#include <list>
#include <rdkcore/geometry/point.h>

namespace RDK2 { namespace Geometry {

class CircleWalk{
public:
	CircleWalk(Point2i c, double ra);
	
	bool next();
	
	void rewind();
	
	Point2i getPoint() const;

	void createCirclePoints();
		
	std::vector<Point2i> circlePoints;
	Point2i center;
	int radius;
};

}} // ns

#endif
