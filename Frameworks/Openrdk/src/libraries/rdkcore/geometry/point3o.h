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

#ifndef RDK2_GEOMETRY_POINT3O
#define RDK2_GEOMETRY_POINT3O

#include "point2o.h"
#include "angle.h"

namespace RDK2 { namespace Geometry {

/**
	Beware that the operators + - * uses also the "theta", "phi"
	and "gamma" components.

*/

template <typename Numeric>
class Point3o : public Point2o<Numeric> {
private:
	// We re-define this method, inherited from Point2o, as
	// private, so we make it unaccessible from outside.
	Point2o<Numeric> rotate(double thetar);
public:
	Numeric z;
	/// theta=roll, phi=pitch, gamma=yaw
	// FIXME: DC: non sono d'accordo: theta = yaw (cioe' intorno all'asse z, generalizzazione del 2D)
	// phi=pitch, cioe' intorno all'asse y, gamma = roll, cioe' intorno all'asse x
	double phi, gamma;
	
/// @name Constructors
//@{
	Point3o() : Point2o<Numeric>(), z(0), phi(0), gamma(0) { }
	Point3o(Numeric x, Numeric y, Numeric z, double theta, double phi, double gamma) : Point2o<Numeric>(x, y, theta), z(z), phi(phi), gamma(gamma) { }
	explicit Point3o(const Point2o<Numeric>& p, Numeric z=0, double phi=0, double gamma=0) : Point2o<Numeric>(p), z(z), phi(phi), gamma(gamma) { }
	Point3o(const Point3o<Numeric>& p) : Point2o<Numeric>(p.x, p.y, p.theta), z(p.z), phi(p.phi), gamma(p.gamma) { }
//@}
	
/// @name Operators
//@{
	/// Subtracts two vectors
	Point3o<Numeric>  operator- (const Point3o<Numeric>&  p) const { 
		return Point3o<Numeric>(this->x - p.x, this->y - p.y, this->z - p.z, this->theta-p.theta, this->phi - p.phi, this->gamma - p.gamma); 
	}
	
	/// Adds two vectors 
	Point3o<Numeric> operator+ (const Point3o<Numeric>&  p) const { 
		return Point3o<Numeric>(this->x + p.x, this->y + p.y, this->z + p.z, this->theta + p.theta, this->phi + p.phi, this->gamma + p.gamma); 
	}
	
	/// Multiplies components by n. 
	Point3o<Numeric>  operator* (const Numeric& n) const { 
		return Point3o<Numeric>(  this->x * n,   this->y * n, this->z * n, this->theta*n, this->phi * n, this->gamma * n); 
	}


	// Returns the matrix unfolded: (m11 m12 m13 m21 m22 m23 m31 m32 m33)
	// See http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	static double *rollPitchYawMatrix(double roll, double pitch, double yaw) {
		double *temp = new double[9];
		// We cache our cosines and sines
		double sintheta = sin(pitch);
		double costheta = cos(pitch);
		double sinpsi = sin(yaw);
		double cospsi = cos(yaw);
		double cosphi = cos(roll);
		double sinphi = sin(roll);
		temp[0] = costheta * cospsi;
		temp[1] = -cosphi * sinpsi + sinphi * sintheta * cospsi;
		temp[2] = sinphi * sinpsi + cosphi * sintheta * cospsi;
		temp[3] = costheta * sinpsi;
		temp[4] = cosphi * cospsi + sinphi * sintheta * sinpsi;
		temp[5] = -sinphi * cospsi + cosphi * sintheta * sinpsi;
		temp[6] = -sintheta;
		temp[7] = sinphi * costheta;
		temp[8] = cosphi * costheta;
		return temp;
	}

	/// Generates a point2o from the x,y,theta components. 
	Point2o<Numeric> point2o() const { return Point2o<Numeric>(this->x,this->y, this->theta); }
//@}
	
/// @name Tests
//@{
	
	bool operator==(const Point3o<Numeric>& p) const {
		return this->x == p.x && this->y == p.y && this->z == p.z && this->theta == p.theta && this->phi == p.phi && this->gamma == p.gamma; }
	
	bool operator!=(const Point3o<Numeric>& p) const {
		return !(p == *this);
	}
//@}
	
/// @name Output functions
//@{
	
	/**
	 * @brief String representation (x,y,z;theta,gamma,phi)
	 *
	 * A string representation, with \f$ @theta \f$, \f$ @gamma \f$
	 * and \f$ @phi \f$ expressed in degrees. Example:
	 *
	 * "(1,2,3;90�,3�,45�)"
	 */
	std::string toString() const
	{
		std::ostringstream oss;
		oss << "(" << this->x << "," << this->y << "," << this->z << ";" << rad2deg(this->theta) << "," << rad2deg(phi) << "," << rad2deg(gamma) << ")";
		return oss.str();
	}
	
	/// A string representation. Example: " x y z theta phi gamma"
	std::string outputSpaceSeparated() const {
		std::ostringstream oss;
		oss << " " << this->x << " " << this->y << " " << z << " "  << this->theta << " " << phi << " " << gamma;
		return oss.str();
	}
	
//@}
	
};

// FIXME: fa quello che dovrebbe? L'ho adattata "a naso" (Arrigo)
template <class Numeric>
Point3o<Numeric> weightedMean(const Point3o<Numeric>& p1, const Point3o<Numeric>& p2, double alpha){
	double beta=1-alpha;
	Numeric x = (Numeric)(beta*p1.x+alpha*p2.x);
	Numeric y = (Numeric)(beta*p1.y+alpha*p2.y);
	Numeric z = (Numeric)(beta*p1.z+alpha*p2.z);
	double theta = angleWeightedMean(p1.theta,p2.theta,alpha);
	double phi = angleWeightedMean(p1.phi,p2.phi,alpha);
	double gamma = angleWeightedMean(p1.theta,p2.theta,gamma);
	return Point3o<Numeric>(x,y,z,theta,phi,gamma);
}

/// @name Standard I/O as space separated
//@{

template <class Numeric>
std::ostream& operator<<( std::ostream&os, const Point3o<Numeric>&p) {
	return os << " " << p.x << " " << p.y << " " << p.z << " " << p.theta << " " << p.phi << " " << p.gamma;
}

template <class Numeric>
std::istream& operator>>( std::istream&is, Point3o<Numeric>&p) {
	return is >> p.x >> p.y >> p.z >> p.theta >> p.phi >> p.gamma;
}

//@}

}} // ns

#endif
