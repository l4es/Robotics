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

#include "rpoint2od.h"

#include <rdkcore/geometry/utils.h>
#include <rdkcore/rdkmath/rdkmath.h>

#include <sstream>
#include <iomanip>
#include <float.h>
#include <cstdlib>

using namespace RDK2::Geometry;
using namespace std;

namespace RDK2 { namespace RGeometry {

	RDK2_FACTORY(RPoint2od);

	RDK2::Object* RPoint2od::clone() const
	{
		return new RPoint2od(*this);
	}

	std::string RPoint2od::getStringRepresentation() const
	{
		ostringstream oss;
		oss << x << " " << y << " " << rad2deg(theta);
		return oss.str();
	}

	bool RPoint2od::loadFromStringRepresentation(const std::string& s)
	{
		istringstream iss(s);
		string a;
		iss >> x >> y >> a;
		if (a == "inf") theta = DBL_MAX;
		else theta = deg2rad(atof(a.c_str()));
		return iss;
	}

	std::string RPoint2od::getStringForVisualization() const
	{
		ostringstream oss;
		oss << setiosflags(ios::fixed) << setprecision(1);
		oss << x << " " << y << " " << rad2deg(theta) << "";
		return oss.str();
	}

	void RPoint2od::read(RDK2::Serialization::Reader*r)
		throw (RDK2::ReadingException)
	{
		r->startReading(getClassName());
			RDK2::Object::readObjectBody ( r );
			x     = r->read_f64();
			y     = r->read_f64();
			theta = r->read_f64();
		r->doneReading();
	}

	void RPoint2od::write(RDK2::Serialization::Writer*w) const
		throw (RDK2::WritingException)
	{
		w->startWriting(getClassName());
			RDK2::Object::writeObjectBody ( w );
			w->write_f64(x,"x");
			w->write_f64(y,"y");
			w->write_f64(theta,"theta");
		w->doneWriting();
	}

	std::vector<Object*>  RPoint2od::getTestCases() const {
		std::vector<Object*> v;
		v.push_back(new RPoint2od(1,0,0));
		v.push_back(new RPoint2od(2,0,RDK_INF64));
		v.push_back(new RPoint2od(3,0,RDK_NAN));
		v.push_back(new RPoint2od(4,RDK_NAN,0));
		return v;
	}

	// A NaN does not compare equal to any floating-point number or NaN, even if the latter
	// has an identical representation. One can therefore test whether a variable has a NaN value
	// by comparing it to itself (i.e. if x != x then x is NaN).

	bool  RPoint2od::equals(const Object*p) const {
		const RPoint2od * o = dynamic_cast<const RPoint2od*>(p);
		return o
			&& ( (x!=x && (o->x!=o->x)) || (o->x == x))
			&& ( (y!=y && (o->y!=o->y)) || (o->y == y))
			&& ( ((theta!=theta) && (o->theta!=o->theta)) || (o->theta == theta));
	}

}} // namespaces

