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

#ifndef RDK2_GEOMETRY_POINT2O
#define RDK2_GEOMETRY_POINT2O

#include "point2.h"
#include "angle.h"

namespace RDK2 { namespace Geometry {

/**
	Beware that the operators + - * uses also the "theta" component.

*/

template <typename Numeric>
class Point2o : public Point2<Numeric> {
public:
	double theta;
	
/// @name Constructors
//@{
	Point2o() : Point2<Numeric>(), theta(0) { }
	Point2o(Numeric x, Numeric y, double theta) : Point2<Numeric>(x, y), theta(theta) { }
	explicit Point2o(const Point2<Numeric>& p, double theta=0) : Point2<Numeric>(p), theta(theta) { }
	Point2o(const Point2o<Numeric>& p) : Point2<Numeric>(p.x, p.y), theta(p.theta) { }
//@}
	
/// @name Operators
//@{
	/// Subtracts two vectors
	Point2o<Numeric>  operator- (const Point2o<Numeric>&  p) const { 
		return Point2o<Numeric>(this->x - p.x, this->y - p.y, this->theta-p.theta); 
	}
	
	/// Adds two vectors 
	Point2o<Numeric> operator+ (const Point2o<Numeric>&  p) const { 
		return Point2o<Numeric>(this->x + p.x, this->y + p.y,theta+p.theta); 
	}
	
	/// Multiplies components by n
	Point2o<Numeric>  operator* (const Numeric& n) const { 
		return Point2o<Numeric>(  this->x * n,   this->y * n, this->theta*n); 
	}
	
	/// Rotate this vector of @param thetar radians around point @param p
	Point2o rotate(double thetar, const Point2<Numeric>& p = Point2<Numeric>(0, 0)) const 
	{
		double x = this->x, y = this->y;
		x -= p.x;
		y -= p.y;
		double newx = (double) x  * cos(thetar) - y * sin(thetar);
		double newy = (double) x  * sin(thetar) + y * cos(thetar);
		double newt = theta + thetar;
		newx += p.x;
		newy += p.y;
		return Point2o((Numeric) newx, (Numeric) newy, newt);
	}

	/// Change the reference system of the point, @param localReferenceSystem is the origin
	/// of the local reference system in global coordinates (theta points towards x axis)
	Point2o toLocalReferenceSystem(const Point2o<Numeric>& localReferenceSystem) const
	{
		Point2<Numeric> tempp(this->x - localReferenceSystem.x, this->y - localReferenceSystem.y);
		tempp = tempp.rot(-localReferenceSystem.theta);
		return Point2o(tempp.x, tempp.y, this->theta - localReferenceSystem.theta);
	}

	/// Change back to global reference system, @param localReferenceSystem is the origin
	/// of the local reference system in global coordinates (theta points towards x axis)
	Point2o toGlobalReferenceSystem(const Point2o<Numeric>& localReferenceSystem) const
	{
		Point2<Numeric> tempp(this->x, this->y);
		tempp = tempp.rot(localReferenceSystem.theta);
		tempp.x = tempp.x + localReferenceSystem.x;
		tempp.y = tempp.y + localReferenceSystem.y;
		return Point2o(tempp.x, tempp.y, this->theta + localReferenceSystem.theta);
	}
	
	/// Generates a point2 from the x,y components. 
	Point2<Numeric> point2() const { return Point2<Numeric>(this->x,this->y); }

//@}
	
/// @name Tests
//@{
	
	bool operator==(const Point2o<Numeric>& p) const {
		return this->x == p.x && this->y == p.y && theta == p.theta; }
	
	bool operator!=(const Point2o<Numeric>& p) const {
		return !(p == *this);
	}
//@}
	
/// @name Output functions
//@{
	
	/// A string representation, with \f$ @theta \f$ expressed in
	/// degrees. Example: "(1,2;90)"
	std::string toString() const
	{
		std::ostringstream oss;
		oss << "(" << this->x << "," << this->y << ";" << rad2deg(theta) << ")";
		return oss.str();
	}
	
	/// A string representation. Example: " x y theta "
	std::string outputSpaceSeparated() const {
		std::ostringstream oss;
		oss << " " << this->x << " " << this->y << " " << theta << " ";
		return oss.str();
	}
	
//@}
	
};

template <class Numeric>
Point2o<Numeric> weightedMean(const Point2o<Numeric>& p1, const Point2o<Numeric>& p2, double alpha){
	double beta=1-alpha;
	Numeric x = (Numeric)(beta*p1.x+alpha*p2.x);
	Numeric y = (Numeric)(beta*p1.y+alpha*p2.y);
	double theta = angleWeightedMean(p1.theta,p2.theta,alpha);
	return Point2o<Numeric>(x,y,theta);
}

/// @name Standard I/O as space separated
//@{

template <class Numeric>
std::ostream& operator<<( std::ostream&os, const Point2o<Numeric>&p) {
	return os << " " << p.x << " " << p.y << " " << p.theta << " ";
}

template <class Numeric>
std::istream& operator>>( std::istream&is, Point2o<Numeric>&p) {
	return is >> p.x >> p.y >> p.theta;
}

//@}

}} // ns

#endif
