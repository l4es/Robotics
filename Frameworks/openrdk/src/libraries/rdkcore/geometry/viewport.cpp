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

#include <sstream>
#include "viewport.h"

namespace RDK2 { namespace Geometry {

///////////////////
// A couple of very important functions
	void basic_w2b(
				   const Point2d& rmin, const Point2d& rmax, const double& x, const double& y,
				   const int bufferWidth, const int bufferHeight, int&bufferX, int&bufferY,
				   bool exception_if_outside)
	throw(std::invalid_argument)
{
		double realWidth  = rmax.x-rmin.x;
		double realHeight = rmax.y-rmin.y;
		
		// u,v \in [0,1]
		double u = (x-rmin.x)/realWidth;
		double v = -(y-rmax.y)/realHeight;
		
		bufferX = (int)floor(u*bufferWidth);
		bufferY = (int)floor(v*bufferHeight);
		
		if(u==1) bufferX =  bufferWidth-1;
		if(v==1) bufferY =  bufferHeight-1;
		
		if(exception_if_outside)
			if( (bufferX<0) || (bufferX>=bufferWidth) || (bufferY<0) || (bufferY>=bufferHeight) ) {
				std::string error = std::string("Point ")+RDK2::TextUtils::toString(x)+","+RDK2::TextUtils::toString(y)
				+ " is outside " + rmin.toString() + "-" + rmax.toString();
				throw std::invalid_argument(error);
			}
}


void basic_b2w(const Point2d&rmin, const Point2d&rmax, const int bx, const int by,
			   const int bufferWidth, const int bufferHeight, double&wx, double&wy) {

	int u = bx;
	int v = bufferHeight-1-by;
	double xcellSize = (rmax.x-rmin.x) / bufferWidth;
	double ycellSize = (rmax.y-rmin.y) / bufferHeight;
	
	wx = rmin.x + xcellSize*0.5 + xcellSize * u;
	wy = rmin.y + ycellSize*0.5 + ycellSize * v;
}

Viewport basic_cellBB(
	const Point2d&rmin, const Point2d&rmax, const int bx, const int by,
	const int bufferWidth, const int bufferHeight) {
		int u = bx;
		int v = bufferHeight-1-by;
		double xcellSize = (rmax.x-rmin.x) / bufferWidth;
		double ycellSize = (rmax.y-rmin.y) / bufferHeight;
		
		Point2d mmin = rmin + Point2d(xcellSize * u, ycellSize * v);
		Point2d mmax = rmin + Point2d(xcellSize * (u+1), ycellSize * (v+1));
		return Viewport(mmin,mmax);
}

///////////////


	Viewport::Viewport(const Point2d& rmin, const Point2d& rmax)
	throw(std::invalid_argument)
	: rmin(rmin), rmax(rmax) {
		if(!(rmin.x < rmax.x)) {
			std::string error = std::string("rmin.x = ") + RDK2::TextUtils::toString(rmin.x)
				+ " >= rmax.x = " + RDK2::TextUtils::toString(rmax.x);
			throw std::invalid_argument(error);
		}
		if(!(rmin.y < rmax.y)) {
			std::string error = std::string("rmin.y = ") + RDK2::TextUtils::toString(rmin.y) 
				+ " >= rmax.y = " + RDK2::TextUtils::toString(rmax.y);
			throw std::invalid_argument(error);
		}
	}
	
	
	Point2<int> Viewport::w2b(const Point2d&world, int bufferWidth, int bufferHeight ) const 
		throw(std::invalid_argument)
	{
		Point2i result;
		basic_w2b(rmin,rmax,world.x,world.y, bufferWidth, bufferHeight, result.x, result.y, true);
		return result;
	}
	
	Point2<int> Viewport::w2b_ani(const Point2d&world, int bufferWidth, int bufferHeight ) const
		throw() 
	{
		Point2i result;
		basic_w2b(rmin,rmax,world.x,world.y, bufferWidth, bufferHeight, result.x, result.y, false);
		return result;
	}
	
		
	bool Viewport::isInside(double x, double y) const {
		return x>=rmin.x && x<rmax.x
		&&     y>=rmin.y && y<rmax.y;
	}

	
	Viewport Viewport::grow(double percentIncrement) const {
		percentIncrement/=2;
		double w = width()*percentIncrement;
		double h = height()*percentIncrement;
		Point2d rmin1(rmin), rmax1(rmax);
		rmin1.x -= w;
		rmin1.y -= h;
		rmax1.x += w;
		rmax1.y += h;
		return Viewport(rmin1,rmax1);
	}
	
	
	std::string Viewport::toString() throw() {
		std::ostringstream oss;
		oss << "Viewport{Min-max: " << rmin.toString() << " - " << rmax.toString() << "}";
		return oss.str();	
	}
	
}} // ns
