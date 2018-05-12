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

#include "rtouchsensordata.h"

namespace RDK2 { namespace RSensorData {

	bool RTouchSensorData::hasStringForVisualization() const {
		return true;
	}

	string RTouchSensorData::getStringForVisualization() const {
		map<string, bool>::const_iterator i;
		ostringstream oss;
		for (i = sensors.begin(); i != sensors.end(); i++) {
			if (i != sensors.begin()) oss << ", ";
			oss << i->first << ": " << i->second;
		}
		return oss.str();
	}

	void RTouchSensorData::read(Reader* r) throw (ReadingException)
	{
		int size, i;
		r->startReading(getClassName());
		timestamp.setSeconds(r->read_f32());
		size = r->read_i32();
		for (i = 0; i < size; i++) {
			string name = r->readString();
			int value = r->read_i8();
			sensors[name] = (value == 1);
		}
		r->doneReading();
	}
	
	void RTouchSensorData::write(Writer* w) const throw (WritingException)
	{
		map<string, bool>::const_iterator i;
		w->startWriting(getClassName());
		w->write_f32(timestamp.getSeconds());
		w->write_i32(sensors.size());
		for (i = sensors.begin(); i != sensors.end(); i++) {
			w->writeString(i->first);
			if (i->second)
				w->write_i8(1);
			else
				w->write_i8(0);
		}
		w->doneWriting();
	}

	RDK2_FACTORY(RTouchSensorData);
	
}} // namespace

