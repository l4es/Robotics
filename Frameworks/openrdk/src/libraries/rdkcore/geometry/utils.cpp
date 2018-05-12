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

/**
 * @file
 *
 * @brief Miscellaneous utility functions
 */

#include "dmatrix.h"
#include "utils.h"
#include "point.h"

#include <rdkcore/textutils/textutils.h>
#include <rdkcore/rdkmath/rdkmath.h>

#include <string>
#include <cstdio>

using namespace RDK2::TextUtils;
using namespace std;

namespace RDK2 { namespace Geometry {

	Point2od mm2m(const Point2od& p) { 
		return Point2od( mm2m(p.x), mm2m(p.y), p.theta); 
	}
	
	Point3od mm2m(const Point3od& p){
		return Point3od( mm2m(p.x), mm2m(p.y), mm2m(p.z), p.theta, p.phi, p.gamma); 
	}	
	
Point3d Trans3d(const Point3od& /*pose*/, const Point3d&/* point*/)
{
#if 0
	HomogTrans h;
	double x=pose.x;
	double y=pose.y;
	double z=pose.z;
	double roll=pose.theta;
	double pitch=pose.phi;
	double yaw=pose.gamma;
	h.DirectTrans(x,y,z,roll,pitch,yaw);
	
	Location l0(point.x,point.y,point.z);
	Location l;
	l=h*l0;
	
	Point3d trans_point(l.vect[0],l.vect[1],l.vect[2]);
	return trans_point;
#endif
	return Point3d(0., 0., 0.);	
}

/** Line fitting: doesn't use error (=0) yet  */
void regression(
	const std::vector<Point2d>& points, double&theta, double&rho, double&error) {
	int n = points.size(); assert(n);
	double mu_x = 0;
	double mu_y = 0;
	for(int a=0;a<n;a++) {
		mu_x += points[a].x / n;
		mu_y += points[a].y / n;
	}
	double s_x2 = 0;
	double s_y2 = 0;
	double s_xy = 0;
	for(int a=0;a<n;a++) {
		s_x2 += square(points[a].x-mu_x);
		s_y2 += square(points[a].y-mu_y);
		s_xy += (points[a].y-mu_y)*(points[a].x-mu_x);
	}
	
	theta = 0.5 * atan2(-2*s_xy, s_y2-s_x2);
	rho = mu_x * cos(theta) + mu_y * sin(theta);
	if(rho>0) {
		rho = - rho;
		theta = theta + M_PI;
	}
	error = 0;
}



	bool parseMatrix(
		const vector<string>& sv, int start_index, int rows, int columns,
		DMatrix<double> &m, string*error) {
		assert(rows>=1);
		assert(columns>=1);
		
		m = DMatrixD(rows, columns);
		
		for(int i=0;i<rows;i++)
		for(int j=0;j<columns;j++) {
			std::string token = sv[start_index + i*columns +j];
			if(!RDK2::TextUtils::parse<double>(token, m[i][j])) {
				if(error) *error = std::string(
					"While reading matrix element i,j=") + toString(i) +"," + toString(j)
				 + "Could not parse token '"+ token+"'.";
				return false;
			}
		}
		return true;
	}
	
	bool parsePoint(
		const string& sx, const string& sy, const string& stheta,
		Point2od& p, std::string*error) {

		if( !RDK2::TextUtils::parse(sx, p.x) ||
				!RDK2::TextUtils::parse(sy, p.y) ||
		!RDK2::TextUtils::parse(stheta, p.theta)) {
				if(error) *error = string("Could not parse point from: ") + sx + "; "
						+ sy + "; "+stheta;
				return false;
		}

		return true;
	}
	

}} // ns
