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

#include <sstream>
using namespace std;

#include "rquaterniond.h"

namespace RDK2 { namespace RGeometry {
	
	RDK2_FACTORY(RQuaterniond);
	
		
		
	Object* RQuaterniond::clone() const
	{
		return new RQuaterniond(*this);
	}

	void RQuaterniond::read(RDK2::Serialization::Reader*r) 
		throw (RDK2::Serialization::ReadingException) 
	{
		r->startReading(getClassName());
			x = r->read_f64();
			y = r->read_f64();
			z = r->read_f64();
			w = r->read_f64();
		r->doneReading();
	}

	void RQuaterniond::write(RDK2::Serialization::Writer*wr) const 
		throw (RDK2::Serialization::WritingException) 
	{
		wr->startWriting(getClassName());
			wr->write_f64(x,"x");
			wr->write_f64(y,"y");
			wr->write_f64(z,"z");
			wr->write_f64(w,"w");
		wr->doneWriting();
	}
	

}} // namespace 
