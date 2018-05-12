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

#ifndef H_SENSORDATA
#define H_SENSORDATA

#include <string>
#include <rdkcore/time/time.h>
#include <rdkcore/geometry/point.h>

namespace RDK2 { namespace SensorData {

using namespace RDK2::Time;
using namespace RDK2::Geometry;

struct Parser;

struct BaseSensorData {
	/*const*/ std::string logtag; // LM: why logtag should be const?
	std::string tag;	// DEPRECATED
	
	Timestamp timestamp;
	std::string sensorName;
	std::string ipc_hostname;
	Timestamp ipc_timestamp;

	Point2od odometryPose;	//< odometry pose when the data has been taken (OF THE SENSOR, NOT OF THE ROBOT)
	Point2od estimatedPose;	//< estimated pose when the data has been taken (OF THE SENSOR, NOT OF THE ROBOT)

	BaseSensorData(const std::string& logtag = "UNKNOWN") : 
		logtag(logtag),
		tag("<uninitialised>"), // DEPRECATED
		timestamp(-1),
		sensorName("<unnamed>"),
		ipc_hostname("localhost"),
		ipc_timestamp(-1),
		odometryPose(0., 0., 0.),
		estimatedPose(0., 0., 0.)
	{
	}

	virtual ~BaseSensorData() {}
	
	virtual Parser* getParser() const { return 0; }		// DEPRECATED

	virtual string getLogLine() const
	{
		return "ERROR I don't know how to log this sensor";
	}
	
	virtual bool loadFromLogLine(const string&) {
		return false;
	}
};


struct Unknown: public BaseSensorData {
	std::string line;
};


}}

#endif
