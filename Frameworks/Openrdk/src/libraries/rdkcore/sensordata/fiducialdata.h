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

#ifndef H_FIDUCIALDATA
#define H_FIDUCIALDATA

#include <rdkcore/geometry/point.h>
#include "sensordata.h"

namespace RDK2 { namespace SensorData {

struct FiducialData: public BaseSensorData {
	/** *structors */
	FiducialData(const std::string& name="FIDUCIAL") : BaseSensorData(name) { }
	virtual ~FiducialData() { }

	struct Tag: RDK2::Geometry::Point3od
	{
		int id;
		Point3od uncertainty;
	};
	
	/* :TODO:08/11/2009 11:36:33 AM:lm: not available for player 2.0.4 */
	/** Minimum and maximum sensor range. Points above and below are invalid. */
	//double minReading, maxReading;

	/** Maximum range at which the sensor can detect the ID of a fiducial, in meters. */
	//double maxReadingID;
	
	/** Angular field of view of the scanner, in rad */
	//int fov;

	typedef std::vector<Tag> TagSet;
	TagSet tagSet;
};

}} // end namespace

#endif
