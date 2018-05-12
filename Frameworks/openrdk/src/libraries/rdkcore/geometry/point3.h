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

#ifndef RDK2_GEOMETRY_POINT3
#define RDK2_GEOMETRY_POINT3

#include "point2.h"

namespace RDK2 { namespace Geometry {

template <typename Numeric>
class Point3 : public Point2<Numeric> {
public:
	Numeric z;
	
	Point3() : Point2<Numeric>(), z(0) { }
	Point3(Numeric x, Numeric y, Numeric z) : Point2<Numeric>(x, y), z(z) { }
	Point3(const Point3<Numeric>& p) : Point2<Numeric>(p.x, p.y), z(p.z) { }
		
	inline Point3 operator-(const Point3<Numeric>& p) const { 
		return Point3(this->x - p.x, this->y - p.y, this->z - p.z); }
	inline Point3 operator+(const Point3<Numeric>& p) const { 
		return Point3(this->x + p.x, this->y + p.y, this->z + p.z); }

	inline Point3 operator*(const Numeric& n) const { return Point3(this->x * n, this->y * n, this->z * n); }
	inline Point3 operator/(const Numeric& n) const { if (n == 0) return *this; return Point3(this->x / n, this->y / n, this->z / n); }
	
	inline Numeric operator*(const Point3& p) const { return this->x * p.x + this->y * p.y + this->z * p.z; }
	inline bool    operator==(const Point3& p) const { return this->x == p.x && this->y == p.y && this->z == p.z; }
	inline bool    operator!=(const Point3& p) const { return !(p == *this); }
	
	/** Returns vector product **/
	inline Point3<Numeric> cross(const Point3<Numeric>& p) const { 
		return Point3<Numeric>(this->y*p.z - this->z*p.y, this->z*p.x - this->x*p.z, this->x*p.y - this->y*p.x);
	}
	
	/** Returns L2-norm of vector. */
	inline Numeric mod() const { return sqrt(mod2()); }
	
	/** Returns L2-norm of vector, squared. */
	inline Numeric mod2() const { return this->x * this->x + this->y *this-> y + this->z * this->z; }
	
	/** Normalize the vector */
	inline Point3 normalize() { Numeric length = mod(); if (length == 0) return *this; return Point3(*this) / length; }
	
	inline Numeric distTo(const Point3<Numeric>& p) const { return ((*this) - p).mod(); }
	
	std::string toString() const
	{
		std::ostringstream oss;
		oss << "(" << this->x << "," << this->y << "," << this->z << ")";
		return oss.str();
	}
};

}} // ns

#endif
