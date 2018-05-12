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

#ifndef RDK2_GEOMETRY_OTHERPOINTS
#define RDK2_GEOMETRY_OTHERPOINTS

#include <vector>
#include <rdkcore/geometry/viewport.h>
#include "point.h"
#include <cstdio>
// AC: altre strutture point-like in attesa forse di un po' di standardizzazione
//     e di documentazione.

namespace RDK2 { namespace Geometry {

struct PolarPoint {
	double theta, rho;

	PolarPoint(double theta=0, double rho=0) : theta(theta), rho(rho) {}
	PolarPoint(const Point2d&p) {
		theta = p.theta();
		rho   = p.abs();
	}

	Point2d getCartesian() const {
		return Point2d::vers(theta) * rho;
	}

	bool operator < (const PolarPoint&m) const {
		return theta < m.theta;
	}

	PolarPoint operator+ (const PolarPoint&  p) const
	{
		double x = rho*cos(theta) + p.rho*cos(p.theta);
		double y = rho*sin(theta) + p.rho*sin(p.theta);

		if(x==0 && y==0)
		{
			return PolarPoint(0,0);
		}
		else
		{
			return PolarPoint( atan2(y,x), sqrt(x*x+y*y));
		}
	}

	PolarPoint operator- (const PolarPoint&  p) const
	{
		PolarPoint np(p);
		np.theta -= M_PI;
		return *this + np;
	}

	bool operator== (const PolarPoint& p)
	{
		return theta == p.theta
						&& rho == p.rho;
	}

};

	typedef std::vector<PolarPoint> PPVec;
	typedef std::vector<Point2od> OPVec;

	struct AppliedVector {
		Point2d where;
		double theta;
		double weight;

		double theta2;
	};

	typedef std::vector<AppliedVector> AVV;

	/// Filters out points whose norm is < min
	AVV filterMin(const AVV&v, double min);

	/// Filters out points whose norm is < max
	AVV filterMax(const AVV&, double max);
	double getMaxRho(const AVV&v);

	Viewport getBounds(AVV&v);

	Point2d getMin(AVV&v);
	Point2d getMax(AVV&v);

std::ostream& operator<<( std::ostream&os, const PolarPoint&p);

std::istream& operator>>( std::istream&is, PolarPoint&p);

}} // end namespaces

#endif
