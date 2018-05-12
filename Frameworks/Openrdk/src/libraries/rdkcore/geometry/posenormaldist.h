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

#ifndef RDK2_GEOMETRY_POSENORMALDIST
#define RDK2_GEOMETRY_POSENORMALDIST

#include "dmatrix.h"
#include "point.h"

namespace RDK2 { namespace Geometry {

struct PoseNormalDist {
	Point2od mu;
	DMatrix<double> sigma;
	
	PoseNormalDist(): sigma(3,3) {}
	PoseNormalDist(const Point2od& mu, const DMatrix<double>& xy_cov, double th_cov);
	
	double getThetaCov() { return sigma[2][2]; }
	DMatrix<double> getXYCov() { return sigma.aPieceOfYou(0,0,2,2); }
};

PoseNormalDist addNormalDist(const PoseNormalDist&, const PoseNormalDist&);

}} // ns

#endif

