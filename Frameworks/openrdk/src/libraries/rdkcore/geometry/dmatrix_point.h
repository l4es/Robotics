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

#ifndef RDK2_DMATRIX_POINT
#define RDK2_DMATRIX_POINT

#include "dmatrix.h"
#include "point2.h"
#include "point3.h"

namespace RDK2
{

namespace Geometry
{

template <class X, class Y>
Point2<X> operator* ( const DMatrix<Y>& m, const Point2<X>&p )
{
	assert ( m.columns() == 2 );
	assert ( m.rows() == 2 );
	return Point2<X> (
					 m[0][0] * p.x + m[0][1] * p.y,
					 m[1][0] * p.x + m[1][1] * p.y
				 );
}

template <class X, class Y>
Point3<X> operator* ( const DMatrix<Y>& m, const Point3<X>&p )
{
	assert ( m.columns() == 3 );
	assert ( m.rows() == 3 );
	return Point3<X> (
					 m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z,
					 m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z,
					 m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z
				 );
}

}
} // namespaces

#endif
