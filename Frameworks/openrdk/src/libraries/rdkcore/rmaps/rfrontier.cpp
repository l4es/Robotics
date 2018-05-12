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

#include "rfrontier.h"

namespace RDK2 { namespace RMaps {

	RDK2_FACTORY(RFrontier);
	RDK2_FACTORY(RFrontierVector);
	
	using namespace std;
	
	void RFrontier::write(Writer* w) const throw (WritingException)
	{
		w->startWriting(getClassName());
			w->write_i32(points.size());
			for (vector<Point2i>::const_iterator it = points.begin(); 
				it != points.end(); ++it) 
			{
				w->write_i32(it->x);
				w->write_i32(it->y);
			}
		w->doneWriting();
	}
	
	void RFrontier::read(Reader* r) throw (ReadingException)
	{
		r->startReading(getClassName());
			points.clear();
			size_t ptsize = r->read_i32();
			for (size_t i = 0; i < ptsize; i++) {
				Point2i p(r->read_i32(), r->read_i32());
				points.push_back(p);
			}
		r->doneReading();
	}

}} // namespace

