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

#include "rpoint3od.h"

#include <sstream>
using namespace std;

namespace RDK2 { namespace RGeometry {
	
RDK2_FACTORY(RPoint3od);

Object* RPoint3od::clone() const
{
	return new RPoint3od(*this);
}

void RPoint3od::read(RDK2::Serialization::Reader*r) 
	throw (RDK2::ReadingException) 
{
	r->startReading(getClassName());
		x = r->read_f64();
		y = r->read_f64();
		z = r->read_f64();
		theta = r->read_f64();
		phi = r->read_f64();
		gamma = r->read_f64();
	r->doneReading();
}

void RPoint3od::write(RDK2::Serialization::Writer*w) const 
	throw (RDK2::WritingException) 
{
	w->startWriting(getClassName());
		w->write_f64(x,"x");
		w->write_f64(y,"y");
		w->write_f64(z,"z");
		w->write_f64(theta,"theta");
		w->write_f64(phi,"phi");
		w->write_f64(gamma,"gamma");
	w->doneWriting();
}
	

}} // namespace 
