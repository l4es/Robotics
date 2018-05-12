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

/*made by Giuseppe Paolo Settembre
  report bugs at <giusset@hotmail.com>
  this library draw quadrilaterals, triangles, lines, and points 
  */

#ifndef RDK2_GEOMETRY_WALK_QUAD
#define RDK2_GEOMETRY_WALK_QUAD

#include "point.h"
#include "walk_line.h"

namespace RDK2 { namespace Geometry {

class QuadWalk {
	public:	
		QuadWalk(Point2i p1, Point2i p2, Point2i p3, Point2i p4);
		bool next();
		Point2i getPoint() const;
		void rewind();

	private:
		//alignment case
		bool quad; //is really a quad
		bool triangle; //is a triangle
		bool line; //is a line
		LineWalk *ptr;  //used only if quad is a line
		
		//extreme
		int minI;
		int minJ;
		int maxI;
		int maxJ;
		
		//variables for scan
		int j1, j2, j3, j4;
		float t1, t2, t3, t4;
		bool sup1,sup2,sup3,sup4;
		int conta;
		int i,j;
		
		//working methods
		bool appartiene();
		void costrTri(Point2i &p1, Point2i &p2, Point2i &p3);
			
		//parameters of the 4 contour lines of the filling quad 
		float a1; float a2; float a3; float a4;
		float b1; float b2; float b3; float b4;
		float p1x; float p2x; float p3x; float p4x;
		float p1y; float p2y; float p3y; float p4y;
};

}} // ns

#endif

