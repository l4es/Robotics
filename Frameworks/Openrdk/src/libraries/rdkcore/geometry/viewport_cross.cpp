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

#include "viewport.h"
#include "segment.h"

namespace RDK2 { namespace Geometry {

	bool Viewport::crossesSegment(const Point2d& p0, const Point2d& p1) const {
		if( isInside(p0) || isInside(p1) ) 
				return true;
		
		if (sedgewickIntersect(p0, p1, rmin, Point2d(rmin.x, rmax.y))
		|| sedgewickIntersect(p0, p1, Point2d(rmax.x, rmin.y), rmax)
		|| sedgewickIntersect(p0, p1, rmin, Point2d(rmax.x, rmin.y))
		|| sedgewickIntersect(p0, p1, Point2d(rmin.x, rmax.y), rmax))
			return true;
			
		return false;
	}

}}

