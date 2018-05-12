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

#ifndef RDK2_GEOMETRY_ROBOTMODELS
#define RDK2_GEOMETRY_ROBOTMODELS

#include "dmatrix.h"
#include "point.h"

namespace RDK2 { namespace Geometry {

	/** Decomposes robot path as 1 translation + 1 rotation (first translation, then rotation) */
	Point2od difference(Point2od p0, Point2od p1);
	/** Inverse of difference: diff is a translation+rotation applied to p0 */
	Point2od add(Point2od p0, Point2od diff);

	/** DONT USE, use difference()*/
	bool diff(const Point2od& pose_old, const Point2od& pose_new, double &speed, double &jog);

}} // ns

#endif
