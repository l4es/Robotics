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

#include "rblobdata.h"

namespace RDK2 { namespace RSensorData {
	
	RDK2_FACTORY(RBlobData);

	void RBlobData::read(Reader* r) throw (ReadingException)
	{
		r->startReading(getClassName());
			tag = r->readString();
			timestamp= r->read_f32();
			sensorName = r->readString();
			ipc_hostname = r->readString();
			ipc_timestamp = r->read_f32();
			odometryPose.x = r->read_f32();
			odometryPose.y = r->read_f32();
			odometryPose.theta = r->read_f32();
			estimatedPose.x = r->read_f32();
			estimatedPose.y = r->read_f32();
			estimatedPose.theta = r->read_f32();
			panAngle = r->read_f32();
			tiltAngle = r->read_f32();
			zoomAngle = r->read_f32();
			width = r->read_u8();
			height = r->read_u8(); 
			size_t size = r->read_u8();
			blobSet.reserve(size);
			for (size_t i=0; i<size; ++i)
			{
				Blob b;
				b.id     = r->read_i32();
				b.color  = r->read_i32();
				b.area   = r->read_i32();
				b.x      = r->read_i32();
				b.y      = r->read_i32();
				b.left   = r->read_i32();
				b.right  = r->read_i32();
				b.top    = r->read_i32();
				b.bottom = r->read_i32();
				b.range  = r->read_f32();
				blobSet.push_back(b);
			}
		r->doneReading();
	}
	
	void RBlobData::write(Writer* w) const throw (WritingException)
	{
		w->startWriting(getClassName());
			w->writeString(tag);
			w->write_f32(timestamp);
			w->writeString(sensorName);
			w->writeString(ipc_hostname);
			w->write_f32(ipc_timestamp);
			w->write_f32(odometryPose.x);
			w->write_f32(odometryPose.y);
			w->write_f32(odometryPose.theta);
			w->write_f32(estimatedPose.x);
			w->write_f32(estimatedPose.y);
			w->write_f32(estimatedPose.theta);
			w->write_f32(panAngle);
			w->write_f32(tiltAngle);
			w->write_f32(zoomAngle);
			w->write_u8(width);
			w->write_u8(height); 
			w->write_u8(blobSet.size());
			for (size_t i=0; i<blobSet.size(); ++i)
			{
				const Blob& b = blobSet[i];
				w->write_i32(b.id);
				w->write_i32(b.color);
				w->write_i32(b.area);
				w->write_i32(b.x);
				w->write_i32(b.y);
				w->write_i32(b.left);
				w->write_i32(b.right);
				w->write_i32(b.top);
				w->write_i32(b.bottom);
				w->write_f32(b.range);
			}
		w->doneWriting();
	}


	
}} // namespaces

