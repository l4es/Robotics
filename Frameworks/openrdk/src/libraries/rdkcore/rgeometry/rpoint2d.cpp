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

#include "rpoint2d.h"

namespace RDK2 { namespace RGeometry {

RDK2_FACTORY(RPoint2d);

RPoint2d::RPoint2d(double x, double y ) 
	: RDK2::Geometry::Point2d(x, y) { }

RDK2::Object* RPoint2d::clone() const
{
	return new RPoint2d(*this);
}

void RPoint2d::read(RDK2::Serialization::Reader*r) 
	throw (RDK2::ReadingException) 
{
	r->startReading(getClassName());
		x = r->read_f64();
		y = r->read_f64();
	r->doneReading();
}

void RPoint2d::write(RDK2::Serialization::Writer*w) const
	throw (RDK2::WritingException) 
{
	w->startWriting(getClassName());
		w->write_f64(x);
		w->write_f64(y);
	w->doneWriting();
}

}}

