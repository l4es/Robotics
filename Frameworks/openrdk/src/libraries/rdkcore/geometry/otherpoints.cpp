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

#include "otherpoints.h"

namespace RDK2 { namespace Geometry {

double getMaxRho(const AVV&v) {
	double maxRho = 0;
	for(int a=0;a<(int)v.size();a++)
		maxRho = std::max(maxRho, v[a].where.norm());
	return maxRho;
}

AVV filterMax(const AVV&v, double max) {
	AVV r;
	for(int a=0;a<(int)v.size();a++)
		if(v[a].where.norm()<max)
			r.push_back(v[a]);
	return r;
}

AVV filterMin(const AVV&v, double min) {
	AVV r;
	for(int a=0;a<(int)v.size();a++)
		if(v[a].where.norm()>min)
			r.push_back(v[a]);
	return r;
}
	
Point2d min(AVV&v) {
	assert(v.size());
	Point2d m = v[0].where;
	for(size_t i=0;i<v.size();++i)
		m = Point2d::min(m,v.at(i).where);
	return m;
}

Point2d max(AVV&v) {
	assert(v.size());
	Point2d m = v[0].where;
	for(size_t i=0;i<v.size();++i)
		m = Point2d::max(m,v.at(i).where);
	return m;
}

Viewport getBounds(AVV&v) {
	return Viewport(min(v),max(v));	
}


std::ostream& operator<<( std::ostream&os, const PolarPoint&p) {
	return os << " " << p.theta << " " << p.rho << " ";
}

std::istream& operator>>( std::istream&is, PolarPoint&p) {
	return is >> p.theta >> p.rho;
}

}} // ns

