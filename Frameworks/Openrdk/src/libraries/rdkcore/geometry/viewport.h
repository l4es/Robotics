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

#ifndef RDK2_GEOMETRY_VIEWPORT
#define RDK2_GEOMETRY_VIEWPORT

#include <string>
#include <algorithm>
#include <stdexcept>
#include <rdkcore/textutils/textutils.h>
#include "dmatrix.h"
#include "point.h"

namespace RDK2 { namespace Geometry {

/** 
	Provides a viewport (??? trasformation) over a dmatrix.
	Storage is delegated to DMatrix without crippling that class
	with coordinates methods.
	
	Note: it is *very* important that the two mappings defined by 
	b2w and w2b be consistent.
	
	All the w2b throws invalid_argument if the coordinates are not valid. 

*/

class Viewport 
{
	private:
		Point2d rmin;
		Point2d rmax;
	
	public:
		
		Viewport(const Point2d& rmin, const Point2d& rmax)
			throw(std::invalid_argument);
		
	/// Utils
	//@{
		/** Returns true if the point is inside the bounding box. */
		bool isInside(double wx, double wy) const;
		
		/** @see isInside. Allows to use a point type (x=col=j,y=row=i) for coordinates. */
		template<class PointType>
		bool isInside(const PointType& p) const ;
	///@}
	
		/// Checks if data is valid
		bool isValid() const { return width() > 0 && height() > 0; }
		
		double  width() const { return rmax.x - rmin.x; };
		double height() const { return rmax.y - rmin.y; };
		
		/// Returns a descriptive string.
		std::string toString() throw();
		
		const Point2d & getMin() const { return rmin; }
		const Point2d & getMax() const { return rmax; }
		
		
	/// World to buffer methods
	/// There is a little confusion with names.
	//@{
		
		// Trasformare le seguenti in "el"
		/** Get dmatrix element corresponding to world coordinates x,y. */
		template<class X>
		X& w2b(DMatrix<X>&m, double x, double y) const throw(std::invalid_argument);
		
		/** Get dmatrix element corresponding to world coordinates x,y 
			(const version). */
		template<class X>
		const X& w2b(const DMatrix<X>&m, double x, double y) const throw(std::invalid_argument);
		
		/** Get dmatrix element corresponding to world coordinates p.x,p.y. */
		template<class X, class PointType>
		X& w2b(DMatrix<X>&m, const PointType& p) const throw(std::invalid_argument);
		
		/** Get dmatrix element corresponding to world coordinates p.x,p.y 
			(const version). */
		template<class X, class PointType>
		const X& w2b(const DMatrix<X>&m, const PointType& p) const throw(std::invalid_argument);
		
		
		/// @throw invalid_argument  resulting point is not inside the buffer
		Point2i w2b(const Point2d&world, int columns, int rows ) const
			throw(std::invalid_argument);
	
		/// This does not throw if point is outside buffer
		Point2i w2b_ani(const Point2d&world, int columns, int rows ) const
			throw();
			
/*		
		/// Get cell indexes corresponding to world coordinates x,y. 
		template<class X>
		Point2i world2buffer(DMatrix<X>&m, const Point2d& world) 
			const throw(std::invalid_argument);*/

	//@}
	
	/// Buffer to world methods
	//@{
		
		/// Get cell size in world units
		template<class X>
		Point2<double> getCellSize(const DMatrix<X>&m);
		
		/// Buffer to world (pay attention to the order of the arguments! (m[row=y][column=x])
		/// Returns the world-coordinates of the center of the cell (row,column)
		template<class X>
		Point2<double> b2w(const DMatrix<X>&m, int row, int column) const;

		template<class X>
		Point2<double> b2w(const DMatrix<X>&m, Point2i p) const {
			return b2w(m, p.y, p.x);
		}
	
		/// Returns the world bounding box for cell (row=y=i,column=x=j)
		template<class X>
		Viewport getCellBoundingBox(const DMatrix<X>&m, int row, int column) const;
		template<class X>
		Viewport getCellBoundingBox(const DMatrix<X>&m, Point2i p) const {
			return getCellBoundingBox(m, p.y, p.x);
		}

	//@}

	/// Check whether the segment p0-p1 crosses this rectangle.
	/// It crosses if p0 or p1 are inside, or p0-p1 crosses any of the sides. 	
	bool crossesSegment(const Point2d& p0, const Point2d& p1) const;


	/// deprecated
		/** Grow the bounds of the bounding box by percentIncrement. 
			Should it return a new object? */
		Viewport grow(double percentIncrement) const;
	
};

#include "viewport.hpp"

/// If exception_if_outside, an exception is thrown if it is outside :-)
void basic_w2b(
			   const Point2d& rmin, const Point2d& rmax, const double& x, const double& y,
			   const int bufferWidth, const int bufferHeight, int&bufferX, int&bufferY,
			   bool exception_if_outside)
throw(std::invalid_argument);

void basic_b2w(const Point2d&rmin, const Point2d&rmax, const int bx, const int by,
			   const int bufferWidth, const int bufferHeight, double&wx, double&wy);

Viewport basic_cellBB(const Point2d&rmin, const Point2d&rmax, const int bx, const int by,
			  const int bufferWidth, const int bufferHeight);


/** A uni-dimensional Viewport */

template<class D=double>
class AffineView {
	D rmin, rmax;
	int imin, imax;
	
	public:
	
	AffineView(D rmin, D rmax, int imin, int imax):
		rmin(rmin), rmax(rmax), imin(imin), imax(imax) {
		assert(rmax>rmin);
	}
	
	bool w2b(double r, int& i) {
		if( r<rmin || r>rmax) return false;
		i = (int) (imin + (imax-imin) * (r-rmin) / (rmax-rmin));
		i = std::max( std::min(i,imax), imin);
		return true;
	}
};

}} // ns

#endif
