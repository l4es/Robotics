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

#include "walk_circle.h"

namespace RDK2 { namespace Geometry {

CircleWalk::CircleWalk(Point2i c, double ra)
{
	center = c;
	radius = (int)ra;
	if (radius >0)
		createCirclePoints();
}
	
bool CircleWalk::next() {
	circlePoints.pop_back();
	return !circlePoints.empty();
}
	
void CircleWalk::rewind() { 
	if (radius >0)
		createCirclePoints();
}

Point2i CircleWalk::getPoint() const { return circlePoints.back(); }

void CircleWalk::createCirclePoints() {
	int xQuadInscribed = (int)(radius / sqrt(2.));
	for (int x = 0; x < radius; x++)
		for(int y = 0; y <= x; y++)
			//the first check condition makes us avoid forwarding calculations
			if ( x < xQuadInscribed || (x*x + y*y) < (radius*radius)) {
				circlePoints.push_back(Point2i(center.x + x, center.y + y));
				//simmetric addition of points on the other quadrants
				circlePoints.push_back(Point2i(center.x + x, center.y - y));
				circlePoints.push_back(Point2i(center.x - x, center.y + y));
				circlePoints.push_back(Point2i(center.x - x, center.y - y));
			
				//simmetric addition of points on the other otcants
				circlePoints.push_back(Point2i(center.x + y, center.y + x));
				circlePoints.push_back(Point2i(center.x + y, center.y - x));
				circlePoints.push_back(Point2i(center.x - y, center.y + x));
				circlePoints.push_back(Point2i(center.x - y, center.y - x));
			}
/*
	//old method, non optimized
	for (int x = -radius; x < radius; x++)
		for(int y = -radius ; y < radius; y++) {
			//the first check condition makes us avoid forwarding calculations
			if ((abs(x)+abs(y))<radius
					||(x*x + y*y) < (radius*radius))
				circlePoints.push_back(Point2i(center.x + x, center.y + y));
		}
*/
}

}} // ns
