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

#ifndef RDK2_GEOMETRY_POINT
#define RDK2_GEOMETRY_POINT

#include <string>
#include <sstream>
#include <assert.h>
#include <cmath>
#include <rdkcore/textutils/textutils.h>

#include "point2.h"
#include "point2o.h"
#include "point3.h"
#include "point3o.h"

namespace RDK2 { namespace Geometry {

using std::sqrt;
using std::cos;
using std::sin;

typedef Point2<int>    Point2i;
typedef Point2<double> Point2d;
typedef Point3<double> Point3d;
typedef Point2o<double> Point2od;
typedef Point3o<double> Point3od;


	bool parsePoint( 
		const std::string& sx, const std::string& sy, const std::string& stheta, 
		Point2od& p, std::string*error=NULL);
	
	

	/** Line fitting: doesn't use error (=0) yet  */
	void regression(
		const std::vector<RDK2::Geometry::Point2d>& points, 
		double&theta, double&rho, double&error);

		Point2od mm2m(const Point2od& p);
		Point3od mm2m(const Point3od& p);
		
		/*
		 * @brief this function returns the 3d coordinates of a 3d point defined in a local reference system given by pose into the reference system in which pose is defined */
		 
		Point3d Trans3d(const Point3od& pose, const Point3d& point);  
		
			
}} // ns

#endif
