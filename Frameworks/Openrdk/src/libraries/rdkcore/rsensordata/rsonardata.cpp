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

#include "rsonardata.h"

namespace RDK2 { namespace RSensorData {

void RSonarData::read(Reader* r) throw (ReadingException)
{
	r->startReading(getClassName());
	minReading = r->read_f32();
	maxReading = r->read_f32();
	size_t sz = r->read_i32();
	devices.resize(sz);
	for (size_t i = 0; i < sz; i++) {
		devices[i].devicePose.x = r->read_f32();
		devices[i].devicePose.y = r->read_f32();
		devices[i].devicePose.theta = r->read_f32();
		devices[i].reading = r->read_f32();
		devices[i].counter = r->read_i32();
		devices[i].new_reading = r->read_i8();
	}
	r->doneReading();
}

void RSonarData::write(Writer* w) const throw (WritingException)
{
	w->startWriting(getClassName());
	w->write_f32(minReading);
	w->write_f32(maxReading);
	w->write_i32((int) devices.size());
	for (size_t i = 0; i < devices.size(); i++) {
		w->write_f32(devices[i].devicePose.x);
		w->write_f32(devices[i].devicePose.y);
		w->write_f32(devices[i].devicePose.theta);
		w->write_f32(devices[i].reading);
		w->write_i32(devices[i].counter);
		w->write_i8(devices[i].new_reading);
	}
	w->doneWriting();
}

RDK2_FACTORY(RSonarData);
	
}} // namespaces

