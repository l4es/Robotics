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

#include "posenormaldist.h"
#include "robotmodels.h"
#include "dmatrix_algebra.h"

namespace RDK2 { namespace Geometry {

	PoseNormalDist::PoseNormalDist(const Point2od& mu, const DMatrix<double>& xy_cov, double th_cov) {
		this-> mu = mu;
		this->sigma=matrix33<double>(xy_cov, th_cov, 0);
	}
		
	PoseNormalDist addNormalDist(
		const PoseNormalDist&start, 
		const PoseNormalDist&increment) 
	{
		PoseNormalDist result;
		result.mu = add(start.mu, increment.mu);
		result.sigma = start.sigma +
			rot3z(start.mu.theta)*increment.sigma*rot3z(-start.mu.theta);
		return result;
	}

}} // ns

