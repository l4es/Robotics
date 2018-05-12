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

#include "rpoint2i.h"

namespace RDK2 { namespace RGeometry {
		
	RDK2_FACTORY(RPoint2i);
	
	RDK2::Object* RPoint2i::clone() const
	{
		return new RPoint2i(*this);
	}

	void RPoint2i::read(RDK2::Serialization::Reader*r) 
		throw (RDK2::ReadingException) 
	{
		r->startReading(getClassName());
			size_t n; int32_t*values;
			r->read_i32(&values,&n);
			x = values[0];
			y = values[1];
			delete[] values;
		r->doneReading();
	}

	void RPoint2i::write(RDK2::Serialization::Writer*w) const 
		throw (RDK2::WritingException) 
	{
		w->startWriting(getClassName());
			int32_t values[2] = {x,y};
			w->write_i32(values, 2);
		w->doneWriting();
	}
	
	std::string RPoint2i::getStringRepresentation() const
	{
		ostringstream oss;
		oss << x << " " << y;
		return oss.str();
	}
	
	bool RPoint2i::loadFromStringRepresentation(const std::string& s)
	{
		istringstream iss(s);
		return iss >> x >> y;
	}
	
	std::string RPoint2i::getStringForVisualization() const
	{
		ostringstream oss;
		oss << x << " " << y;
		return oss.str();
	}

}} // namespaces	

