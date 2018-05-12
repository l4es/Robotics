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

#include "angle.h"
#include "robotmodels.h"

#include <rdkcore/rdkmath/rdkmath.h>

namespace RDK2 { namespace Geometry {

bool diff(const Point2od& pose_old, const Point2od& pose_new, double &speed, double &jog) {
	jog   = angleDiff(pose_new.theta , pose_old.theta); // FIXME

	speed = sqrt( (pose_new.x - pose_old.x)*(pose_new.x - pose_old.x) +
				  (pose_new.y - pose_old.y)*(pose_new.y - pose_old.y) );
		
	int sign =
	  RDK2::sgn (  cos(pose_old.theta) * (pose_new.x - pose_old.x) +
			 sin(pose_old.theta) * (pose_new.y - pose_old.y)  );

	speed *= sign;
	
	#define SMALL 1e-30
	/*if( (SMALL>abs(pose_old.x-pose_new.x))
	 && (SMALL>abs(pose_old.x-pose_new.x)) )
	  speed=0;*/
	 if(fabs(speed)<SMALL) speed = 0;
	 if(fabs(jog)<SMALL) jog = 0;
	 
	return speed != 0 || jog != 0;
}

	Point2od difference(Point2od p0, Point2od p1) {
		Point2d ds = (p1-p0).rot(-p0.theta);
		double dtheta = p1.theta - p0.theta;
		return Point2od(ds.x,ds.y,dtheta);
	}
	
	Point2od add(Point2od p0, Point2od diff) {
		Point2d p1 = (Point2d)p0 + diff.rot(p0.theta);
		return Point2od(p1.x,p1.y, p0.theta+diff.theta);	
	}
	
}} // ns
