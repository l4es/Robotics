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

#ifndef SENSORDATA_LASERDATA_TRACKER_H
#define SENSORDATA_LASERDATA_TRACKER_H

#include <rdkcore/geometry/point2.h>
#include <rdkcore/sensordata/laserdata.h>

namespace RDK2 { namespace SensorData {

using namespace RDK2::Geometry;

class LaserDataTracker {
public:
	/** @brief Resets the object, you need to call \c update to have a valid object. */
	void reset();
	
	/** @brief Updates the laser data structure according with \c laserData and \c robotPose.
	 *  @param laserData the current laser data.
	 *  @param robotPose the pose of the sensor when \c laserData has been taken.
	 */
	void update(const LaserData& laserData, const Point2od& sensorPose);
	
	/** @brief Returns the tracked laser data.
	 */
	inline const LaserData& getLaserData() const { return extLaserData; }
	
protected:
	LaserData extLaserData;
	Point2od lastSensorPose;
};

}} // namespaces

#endif
