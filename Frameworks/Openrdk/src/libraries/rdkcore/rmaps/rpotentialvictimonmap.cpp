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

#include "rpotentialvictimonmap.h"

namespace RDK2 { namespace RMaps {

	RDK2_FACTORY(RPotentialVictimOnMap);
	
	void RPotentialVictimOnMap::write(Writer* w) const throw (WritingException)
	{
		w->startWriting(getClassName());
			w->write_f32(position.x);
			w->write_f32(position.y);
						 w->write_i8(id);
			w->write_i8(visited);
		w->doneWriting();
	}
	
	void RPotentialVictimOnMap::read(Reader* r) throw (ReadingException)
	{
		r->startReading(getClassName());
			position.x = r->read_f32();
			position.y = r->read_f32();
						 id = r->read_i8();
			visited = r->read_i8();
		r->doneReading();
	}

}} // namespace
