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

#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <rdkcore/geometry/point.h>
#include "sensordata.h"




namespace RDK2 { namespace SensorData {

struct SonarDevice {
	SonarDevice() : reading(0.0), counter(0), new_reading(false) { }
	std::string name;
	RDK2::Geometry::Point2od devicePose; 
	double reading;
	int counter;
	bool new_reading;
};


struct SonarData: public BaseSensorData {
	SonarData() : minReading(0.0), maxReading(0.0) { }
	/** Minimum and maximum sensor range. Points above and below are invalid. */
	double minReading, maxReading;
	typedef std::vector<SonarDevice> SonarDevices;
	SonarDevices devices;
};

}} // end namespace

#endif

