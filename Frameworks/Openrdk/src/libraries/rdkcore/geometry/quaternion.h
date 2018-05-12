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

#ifndef RDK2_QUATERNION_H
#define RDK2_QUATERNION_H

#include <cmath>

#include "point2o.h"
#include "point3.h"

namespace RDK2 { namespace Geometry {

template <typename Numeric>
class Quaternion {
protected:
	Point3<Numeric> v;
	
public:
	double &x, &y, &z, w;
	
/// @name Constructors
//@{
	Quaternion():v(0,0,0),x(v.x),y(v.y),z(v.z),w(1){ }
	Quaternion(Numeric _w, Numeric _x, Numeric _y, Numeric _z): v(_x,_y,_z),x(v.x),y(v.y),z(v.z),w(_w) { }

	/// WARNING: the @a _w parameter is the fourth
	Quaternion(const Point3<Numeric>& p, Numeric _w = 1): v(p.x, p.y,p.z),x(v.x),y(v.y),z(v.z),w(_w) { }
	Quaternion(const Quaternion<Numeric> &q): v(q.x,q.y,q.z),x(v.x),y(v.y),z(v.z),w(q.w) { }
//@}
	
/// @name Utility Functions
//@{
	inline Point3<Numeric>& getVector() const {return v;}
//@}
	
/// @name Unary Operators
//@{
	/// Returns quaternion conjugate
	inline Quaternion<Numeric> conjugate() const {
		return Quaternion<Numeric>(v *-1, w);
	}
	
	/// Returns the L-2 norm, squared
	inline Numeric mod2() const {
		return this->x * this->x + this->y *this-> y + this->z * this->z + this->w * this->w;
	}
	
	/// Returns the L-2 norm
	inline Numeric norm() const {
		return sqrt(mod2());
	}
	
	/// Returns the quaternion inverse
	inline Quaternion<Numeric> inverse() const {
		return conjugate() * 1.0/mod2();
	}
//@}
	
/// @name Binary Operators
//@{
	/// Adds two quaternions
	inline Quaternion<Numeric> operator+(const Quaternion<Numeric>& q) const {
		return Quaternion<Numeric>(x + q.x, y + q.y, z + q.z, w + q.w);
	}
	
	/// Adds to a quaternion another quaternion
	inline Quaternion<Numeric>& operator+=(const Quaternion<Numeric>& q) {
		x += q.x ;
		y += q.y ;
		z += q.z ;
		w += q.w ;
		return *this;
	}
	
	/// Subtract two quaternions
	inline Quaternion<Numeric> operator-(const Quaternion<Numeric>& q) const {
		return Quaternion<Numeric>(x - q.x, y - q.y, z - q.z, w - q.w);
	}
	
	/// Multiplies by a scalar
	inline Quaternion<Numeric> operator*(const Numeric& s) const{
		return Quaternion<Numeric>(s * x, s * y, s * z, s * w);
	}
		
	/// Multiplies by a quaternion
	Quaternion<Numeric> operator*(const Quaternion<Numeric>& q) const {
		Point3<Numeric> vector = v.cross(q.getVector()) + q.getVector() * w + v * q.w;
		Numeric scalar = w * q.w - v * q.getVector();
		return Quaternion<Numeric>(vector, scalar);
	}

	/// Divides by a scalar
	inline Quaternion<Numeric> operator/(const Numeric& s) const{
		return Quaternion<Numeric>(s / x, s / y, s / z, s / w);
	}

	/// Euclidean product
	inline Quaternion<Numeric> euclidean(const Quaternion<Numeric>& q) const {
		Point3<Numeric> vector = v.cross(q.getVector()) * -1 + q.getVector() * w - v * q.w;
		Numeric scalar = w * q.w + v * q.getVector();
		return Quaternion<Numeric>(vector, scalar);
	}
	
	/// Rotate the quaternion by @param alpha radians around the given @param axis of rotation
	/// The quaternion is considered as a 3D point in its homogenous coordinate
	Quaternion<Numeric> rotate(const Point3<Numeric>& axis, const double alpha) const {
		Quaternion<Numeric> z;
		z.w = std::cos(alpha * 0.5);
		z.v = axis * (1./axis.mod()) * std::sin(alpha * 0.5);
		return z * *this * z.conjugate();
	}
//@}
	
/// @name Comparators
//@{
	
	/// Equality
	inline bool operator==(const Quaternion<Numeric>& q) const {
		return v == q.getVector() && w == w;
	}
	
	/// Disequality
	inline bool operator!=(const Quaternion<Numeric>& q) const {
		return !(q == *this);
	}
//@}

};

typedef Quaternion< double > Quaterniond;

}}

#endif
