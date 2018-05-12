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

#include "reference.h"

namespace RDK2 { namespace Geometry {
	
// Ruota di ds.theta e trasla di ds.x,ds.y
PolarPoint translateTo(PolarPoint p, Point2od ds) {
	return PolarPoint( p.getCartesian().rot(ds.theta) + Point2d(ds));
}


// Ruota di ds.theta e trasla di ds.x,ds.y
PPVec translateTo(const PPVec&vec, Point2od ds) {
	PPVec temp;
	for(size_t a=0;a<vec.size();a++) 
		temp.push_back(translateTo(vec[a], ds));
	return temp;
}

Point2d changeRef(Point2od newReference, Point2d p) {
	return (p-newReference).rot(-newReference.theta);
}

PolarPoint changeRef(Point2od newReference, PolarPoint pp) {
	return PolarPoint(changeRef(newReference, pp.getCartesian()));
}

AppliedVector changeRef(Point2od newReference, AppliedVector ov) {
	AppliedVector av;
	av.where = changeRef(newReference, ov.where);
	av.weight = ov.weight;
	av.theta = ov.theta - newReference.theta;
	return av;
}

Point2od local2world(Point2od ref, Point2od p) {
	Point2d c = ref.point2() + p.rot(ref.theta);
	double  t = ref.theta + p.theta;
	return Point2od(c,t);
}

AppliedVector local2world(Point2od ref, AppliedVector p) {
	AppliedVector av;
	av.where = ref.point2() + p.where.rot(ref.theta);
	av.weight = p.weight;
	av.theta = p.theta + ref.theta;
	return av;
}

}} // ns
