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

#ifndef RDK_MATH_H
#define RDK_MATH_H

#include <cmath>
#include <cstdlib>
#include <string>
#include <sstream>

namespace RDK2 {

template<class X> inline X square(const X& x) { return x*x; }

inline double rand1u() { return (double) rand() / RAND_MAX; }
inline double rand1s() { return (double) rand() / RAND_MAX * 2 - 1.0; }

/*
 * Return a random number within [-r/2,r/2)
 */
inline double randr(double r) { return (rand1u() - .5) * r; }

/*
 * Return a random number within [-r/2,r/2) centered in c
 */
inline double randrc(double r, double c) { return (rand1u() - .5) * r + c; }

template<typename X>
inline X sgn(X v) { return (v == 0. ? 0. : (v > 0. ? 1. : -1.)); }

#ifdef CYGWIN
	#define RDK_INF32 ((float)infinity())
	#define RDK_INF64 (infinity())
	#define RDK_NAN (nan(0))
#else
	#define RDK_INF32 ((float)INFINITY) 
	#define RDK_INF64 INFINITY
	#define RDK_NAN NAN
#endif

#define RDK_NAN_STRING "nan"
#define RDK_INF_STRING "inf"
	
template<typename T>
inline bool RDK_ISNAN(T a) { return a != a; }

inline bool RDK_ISINF(double a) { return a == RDK_INF64; }
inline bool RDK_ISINF(float  a) { return a == RDK_INF32; }

template<typename T>
bool sameFloat(T a, T b, T epsilon = 0)
{
	if (RDK_ISNAN(a) && RDK_ISNAN(b)) return true;
	if (RDK_ISNAN(a) || RDK_ISNAN(b)) return false;
	return (a == b) || (fabs(a - b) < epsilon);
}

bool string2float(const std::string& s, double &v);	
bool string2float(const std::string& s, float &v);	

template<typename T>
std::string float2string(T v)
{
	if (RDK_ISNAN(v)) return  RDK_NAN_STRING;
	if (RDK_ISINF(v)) return  RDK_INF_STRING;
	std::ostringstream oss;
	oss << v;
	return oss.str();
}

} // namespace

#endif
