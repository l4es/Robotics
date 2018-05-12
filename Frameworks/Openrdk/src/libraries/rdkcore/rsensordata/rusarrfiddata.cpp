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

#include "rusarrfiddata.h"

namespace RDK2 { namespace RSensorData {

	RDK2_FACTORY(RUsarRfidData);
	RDK2_FACTORY(RUsarRfidDataVector);
		
	using namespace std;
	using namespace RDK2::Serialization;
	
	void RUsarRfidData::write(Writer* w) const throw (WritingException)
	{
		w->startWriting(getClassName());
			w->writeString(ipc_hostname);
			w->writeString(name);
			w->write_i32(id);
			w->write_f32(location.x);
			w->write_f32(location.y);
			w->write_f32(location.z);
			w->write_f32(timestamp.getSeconds());
//			w->writeString(data);
		w->doneWriting();
	}
	
	void RUsarRfidData::read(Reader* r) throw (ReadingException)
	{
		r->startReading(getClassName());
			ipc_hostname=r->readString();
			name = r->readString();
			id = r->read_i32();
			location.x=r->read_f32();
			location.y=r->read_f32();
			location.z=r->read_f32();
			timestamp.setSeconds(r->read_f32());
//			data = r->readString();
		r->doneReading();
	}
	
}} // namespaces

