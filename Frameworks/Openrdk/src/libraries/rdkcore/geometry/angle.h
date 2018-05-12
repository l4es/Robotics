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

/*
 * Author: Daniele Calisi <calisi@dis.uniroma1.it>
 */

#ifndef RDK2_GEOMETRY_ANGLE
#define RDK2_GEOMETRY_ANGLE

#include <cmath>
#include "utils.h"

namespace RDK2 { namespace Geometry {

#define M_2PI   (2*M_PI)

/** Returns an angle that is 0 <= angle < 2 * PI */
inline double angNorm2PiUnsig(double ang)
{
	
	double r = std::fmod(ang, 2 * M_PI);
	if (r < 0) r = 2 * M_PI + r;
	return r;
}

/** returns an angle that is -PI <= angle <= PI */
inline double angNormPiSig(double ang)
{
	
	double r = std::fmod(ang, 2 * M_PI);
	if (r > M_PI) r = -(2 * M_PI - r);
	else if (r < -M_PI) r = 2 * M_PI + r;
	return r;
}

inline double angDiff(double ang1, double ang2) { 
	return angNormPiSig(angNorm2PiUnsig(ang1) - angNorm2PiUnsig(ang2)); 
}

// AC: il Censi mette qua i suoi, almeno stanno tutti dalla stessa parte. 
// prima di unificarli voglio una verifica formale della equivalenza altrimenti
// e' il caos

// AC: e con caos intendo il vero caos tipo rane che cadono dal cielo

inline double norm(double t, double range) {
	 return fmod(fmod(t,range)+range, range);
}

/** Between 0 and 2pi */ 
inline double  norm2PI(double t) { return norm(       t, M_2PI); }

/** Between 0 and pi */
inline double   normPI(double t) { return norm(       t, M_PI ); }

/** Between -pi/2 and pi/2 */
inline double  normPIX(double t) { return norm(t+M_PI/2, M_PI )- M_PI/2; }

/** Between -pi and pi */
inline double norm2PIX(double t) { return norm(  t+M_PI, M_2PI)- M_PI;   }

/** You want the difference to be normalized between -pi and pi */
inline double angleDiff(double Angle1, double Angle2)
{
	return norm2PIX(norm2PI(Angle1) - norm2PI(Angle2));
}

inline double angleWeightedMean(double Angle1, double Angle2, double alpha)
{
	return atan2((1-alpha)*sin(Angle1)+alpha*sin(Angle2),(1-alpha)*cos(Angle1)+alpha*cos(Angle2));
}


}} // namespaces

#endif
