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

#ifndef RDK2_GEOMETRY_POINT2
#define RDK2_GEOMETRY_POINT2

#include <string>
#include <sstream>
#include <assert.h>
#include <cmath>

namespace RDK2 { namespace Geometry {

using std::sqrt;
using std::cos;
using std::sin;

/**
 * A generic vector with two components.
 *
 * Note: by design, none of the methods changes the values of the members.
 *
 * @author Andrea Censi  <andrea.censi@dis.uniroma1.it>
 * @author Daniele Calisi <calisi@dis.uniroma1.it>
*/

template <typename Numeric>
struct Point2 {

	Numeric x, y;

	Point2() : x(0), y(0) { }
	Point2(Numeric x, Numeric y) : x(x), y(y) { }
	Point2(const Point2<Numeric>& p) : x(p.x), y(p.y) { }

/// Operators
	/** Subtracts two vectors. */
	Point2  operator- (const Point2&  p) const { return Point2(x - p.x, y - p.y); }

	/** Adds two vectors. */
	Point2  operator+ (const Point2&  p) const { return Point2(x + p.x, y + p.y); }

	/** Subtracts two vectors, unary version . */
	Point2&  operator-= (const Point2&  p) { x -= p.x; y -= p.y; return *this; }

	/** Adds two vectors, unary version. */
	Point2&  operator+= (const Point2&  p) { x += p.x; y += p.y; return *this; }

	/** Multiplies components by n. */
	Point2  operator* (const Numeric& n) const { return Point2(  x * n,   y * n); }

	/** Divides components by n. */
	Point2  operator/ (const Numeric& n) const { return Point2(  x / n,   y / n); }

	/** Scalar product. */
	Numeric operator* (const Point2&  p) const { return    x * p.x + y * p.y;     }

	/** Checks for equality   (not a safe way for not-integer Numeric types:
		can you feel the evil in this function?) */
	bool operator==(const Point2&  p) const { return x == p.x && y == p.y;     }

	/** Checks for inequality (not a safe way: see above) */
	bool operator!=(const Point2&  p) const { return !(p == *this);            }

	/** This only have sense if you want to have a std::map<Point2<WhatYouLike>, <WhatYouWant> >
	    or something like this */
	bool operator<(const Point2& p) const { return x < p.x || (x == p.x && y < p.y); }

/// Methods
	/** Returns L2-norm of vector.*/
	double mod()  const { return sqrt(mod2());  }

	/** Returns L2-norm of vector, squared. */
	double mod2() const { return x * x + y * y; }

	/** Returns L2-norm of vector.*/
	double abs() const { return mod(); }

	/** Returns L2-norm of vector.*/
	double norm() const { return mod(); }

	/** Returns the squared distance between this point and p. */
	double distTo2(const Point2& p) const { return ((*this) - p).mod2(); }

	/** Returns the distance between this point and p. */
	double distTo(const Point2& p) const { return ((*this) - p).mod(); }

	/** Returns the angle of the line that connects this point and p. */
	double thetaWith(const Point2& p) const { return atan2(p.y - y, p.x - x); }

	/** Returns a vector perpendicular to this, with the same module. */
	Point2 perp() const { return Point2(y,-x); }

	/** Returns a versor (= norm 1) parallel to this. */
	Point2 vers() const { return vers(theta()); }

	/** Returns angle w/x axis, that is the angle of the polar coordinates.
	 *  This function asserts if both x and y are 0. */
	double theta() const {
		return (x!=0||y!=0) ? atan2(y, x) : 0;
	}

	/** Rotate this vector of  @param thetar radians around center @param center. */
	Point2 rot(double thetar, const Point2<Numeric>& center = Point2<Numeric>(0, 0)) const
	{
		double newx = (double) (this->x - center.x) * cos(thetar) - (this->y - center.y) * sin(thetar) + center.x;
		double newy = (double) (this->x - center.x) * sin(thetar) + (this->y - center.y) * cos(thetar) + center.y;
		return Point2((Numeric) newx, (Numeric) newy);
	}

	/** A string representation. Example: "(1,2)". */
	std::string toString () const
	{
		std::ostringstream oss;
		oss << "(" << x << "," << y << ")";
		return oss.str();
	}

	/** Another string representation. Example: " 1 2 ". */
	std::string outputSpaceSeparated() const {
		std::ostringstream oss;
		oss << " " << this->x << " " << this->y << " ";
		return oss.str();
	}

	/// Static methods

	/** Returns a versor of the specified angle */
	static Point2 vers(double theta)  {
		return Point2(cos(theta), sin(theta));
	}

	/** Returns a versor with the minima of the components of @a and @b.*/
	static Point2 min(const Point2&a, const Point2&b)  {
		return Point2( std::min(a.x,b.x), std::min(a.y,b.y));
	}

	/** Returns a versor with the maxima of the components of @a and @b.*/
	static Point2 max(const Point2&a, const Point2&b)  {
		return Point2( std::max(a.x,b.x), std::max(a.y,b.y));
	}

}; // template Point2


template <class Numeric>
std::ostream& operator<<( std::ostream&os, const Point2<Numeric>&p) {
	return os << " " << p.x << " " << p.y << " ";
}

template <class Numeric>
std::istream& operator>>( std::istream&is, Point2<Numeric>&p) {
	return is >> p.x >> p.y;
}

}} // ns

#endif
