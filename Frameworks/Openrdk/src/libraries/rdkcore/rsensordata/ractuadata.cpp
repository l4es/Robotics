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

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "RActuaData"

#include "ractuadata.h"

namespace RDK2 { namespace RSensorData {

	RDK2_FACTORY(RActuaData);
	
	using namespace RDK2::SensorData;
	using namespace std;
	using namespace RDK2::Serialization;
	
	void RActuaData::read(Reader* r) throw (ReadingException)
	{
		r->startReading(getClassName());
			timestamp= r->read_f32();
			ipc_timestamp = r->read_f32();
			tag = r->readString();
			ipc_hostname = r->readString();
			odometryPose.x = r->read_f32();
			odometryPose.y = r->read_f32();
			odometryPose.theta = r->read_f32();
			estimatedPose.x = r->read_f32();
			estimatedPose.y = r->read_f32();
			estimatedPose.theta = r->read_f32();
			accX = r->read_f32();
			accY = r->read_f32();
		r->doneReading();
	}
	
	void RActuaData::write(Writer* w) const throw (WritingException)
	{
		w->startWriting(getClassName());
			w->write_f32(timestamp.getSeconds());
			w->write_f32(ipc_timestamp.getSeconds());
			w->writeString(tag);
			w->writeString(ipc_hostname);
			w->write_f32(odometryPose.x);
			w->write_f32(odometryPose.y);
			w->write_f32(odometryPose.theta);
			w->write_f32(estimatedPose.x);
			w->write_f32(estimatedPose.y);
			w->write_f32(estimatedPose.theta);
			w->write_f32(accX);
			w->write_f32(accY);
		w->doneWriting();
	}

	string RActuaData::getLogLine() const {
		ostringstream oss;
		oss << logtag
			<< accX << " " << accY << " "
			<< odometryPose.x << " " << odometryPose.y << " " << odometryPose.theta << " "
			<< estimatedPose.x << " " << estimatedPose.y << " " << estimatedPose.theta << " "
			<< timestamp.getMsFromMidnight() << " "
			<< ipc_timestamp.getMsFromMidnight() << " "
			<< ipc_hostname << " " << tag;
		return oss.str();
	}

	bool RActuaData::loadFromLogLine(cstr line) {
		istringstream iss(line);
		string name;
		iss >> name;
		if (name != logtag) {
			RDK_ERROR_PRINTF("Error, the first token must be ACTUA");
			return false;
		}
		iss >> accX >> accY;
		iss >> odometryPose.x >> odometryPose.y >> odometryPose.theta;
		iss >> estimatedPose.x >> estimatedPose.y >> estimatedPose.theta;
		unsigned long a;
		iss >> a >> a;	// XXX
		timestamp.setMsFromMidnight(a);
		ipc_timestamp = timestamp;
		iss >> ipc_hostname >> tag;
		return true;
	}
	
}} // namespaces

