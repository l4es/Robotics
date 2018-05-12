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

#include "rxsensdata.h"

namespace RDK2 { namespace RSensorData {

	RDK2_FACTORY(RXsensData);
	
	using namespace RDK2::SensorData;
	using namespace std;
	using namespace RDK2::Serialization;
	
	void RXsensData::read(Reader* r) throw (ReadingException)
	{
		r->startReading(getClassName());
			timestamp= r->read_f32();
			ipc_timestamp = r->read_f32();
			tag = r->readString();
			ipc_hostname = r->readString();
			odometryPose.x = r->read_f32();
			odometryPose.y = r->read_f32();
			odometryPose.theta = r->read_f32();
			estimated.x = r->read_f32();
			estimated.y = r->read_f32();
			estimated.theta = r->read_f32();
			accX = r->read_f32();
			accY = r->read_f32();
			accZ = r->read_f32();
			q.x = r->read_f32();
			q.y = r->read_f32();
			q.z = r->read_f32();
			q.w = r->read_f32();
		r->doneReading();
	}
	
	void RXsensData::write(Writer* w) const throw (WritingException)
	{
		w->startWriting(getClassName());
			w->write_f32(timestamp);
			w->write_f32(ipc_timestamp);
			w->writeString(tag);
			w->writeString(ipc_hostname);
			w->write_f32(odometryPose.x);
			w->write_f32(odometryPose.y);
			w->write_f32(odometryPose.theta);
			w->write_f32(estimated.x);
			w->write_f32(estimated.y);
			w->write_f32(estimated.theta);
			w->write_f32(accX);
			w->write_f32(accY);
			w->write_f32(accZ);
			w->write_f32(q.x);
			w->write_f32(q.y);
			w->write_f32(q.z);
			w->write_f32(q.w);
		w->doneWriting();
	}



	
}} // namespaces

