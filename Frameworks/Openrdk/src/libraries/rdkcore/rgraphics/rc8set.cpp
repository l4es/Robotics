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

#include "rc8set.h"

namespace RDK2 { namespace RGraphics {

	RDK2_FACTORY(RC8Set);
	
	void RC8Set::read(Reader* r) throw (ReadingException)
	{
		r->startReading(getClassName());
			mask = r->read_u8();
		r->doneReading();
	}
	
	void RC8Set::write(Writer* w) const throw (WritingException)
	{
		w->startWriting(getClassName());
			w->write_u8(mask);
		w->doneWriting();
	}

}} // namespaces
