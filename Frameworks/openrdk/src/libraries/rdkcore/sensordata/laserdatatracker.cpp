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

#include <float.h>
#include <sys/types.h>

#include "laserdatatracker.h"

namespace RDK2 { namespace SensorData {

void LaserDataTracker::reset()
{
	extLaserData.points.clear();
}

void LaserDataTracker::update(const LaserData& laserData, const Point2od& sensorPose)
{
	if (extLaserData.points.size() == 0) {
		double rayWidth = laserData.getRayWidth();
		int trackedRaysCount = (int) (M_PI * 2 / rayWidth + 1);
		extLaserData.minTheta = -((double)trackedRaysCount/2)*rayWidth;
		extLaserData.maxTheta = -extLaserData.minTheta;
		extLaserData.points.resize(trackedRaysCount);	// default constructor of LaserPoint sets reading to 0
		extLaserData.updateThetas();
		for (size_t i = 0; i < laserData.points.size(); i++) {
			int index = extLaserData.getNearestIndex(laserData.points[i].theta);
			extLaserData.points[index].reading = (laserData.points[i].isValid() ? laserData.points[i].reading : DBL_MAX);
		}
	}
	else {
		double ax = sensorPose.x - lastSensorPose.x;
		double ay = sensorPose.y - lastSensorPose.y;
		double alpha = sensorPose.theta - lastSensorPose.theta;
		double tx = ax * cos(-lastSensorPose.theta) - ay * sin(-lastSensorPose.theta);
		double ty = ax * sin(-lastSensorPose.theta) + ay * cos(-lastSensorPose.theta);
		
		vector<LaserData::LaserPoint> trackedRays;
		trackedRays.resize(extLaserData.points.size());
		for (size_t i = 0; i < extLaserData.points.size(); i++) trackedRays[i] = extLaserData.points[i];
		
		for (size_t i = 0; i < trackedRays.size(); i++) {
			// cartesian coordinates of the point in old reference system
			Point2d p(trackedRays[i].reading * cos(trackedRays[i].theta),
				trackedRays[i].reading * sin(trackedRays[i].theta));
				
			// intermediate reference system
			p.x -= tx;
			p.y -= ty;
			
			// cartesian coordinates of the point in the new reference system
			p = p.rot(-alpha);
			
			// polar coordinates in the new reference system (null rays are not updated)
			if (trackedRays[i].isValid()) {
				trackedRays[i].reading = Point2d(0., 0.).distTo(p);
				trackedRays[i].theta = Point2d(0., 0.).thetaWith(p);
			}
			if (trackedRays[i].reading < 0.01) trackedRays[i].markInvalid();
		}
		
		for (size_t i = 0; i < extLaserData.points.size(); i++) extLaserData.points[i].reading = DBL_MAX;
		
		// track old rays
		for (size_t i = 0; i < trackedRays.size(); i++) {
			bool updateIt = false;
			int idx = extLaserData.getNearestIndex(trackedRays[i].theta);
			if (trackedRays[i].isValid()) {
				if (!extLaserData.points[idx].isValid() || extLaserData.points[idx].reading < 0.01) updateIt = true;
				else if (trackedRays[i].reading < extLaserData.points[idx].reading) updateIt = true;
			}
			if (updateIt) extLaserData.points[idx].reading = trackedRays[i].reading;
		}
		
		// update with new rays
		for (size_t i = 0; i < laserData.points.size(); i++) {
			if (laserData.points[i].reading < 0.01) continue;
			if (laserData.points[i].isValid()) {
				int idx = extLaserData.getNearestIndex(laserData.points[i].theta);
				if (laserData.points[i].isValid()) extLaserData.points[idx].reading = laserData.points[i].reading;
			}
		}
		
		// interpolate invalid rays
		bool wasInvalid = true;
		int lastValidIdx = -1;
		for (size_t i = 0; i < extLaserData.points.size(); i++) {
			if (extLaserData.points[i].isValid()) {
				if (wasInvalid && lastValidIdx != -1) {
					if (i - lastValidIdx < 5) {		// FIXME why 5?
						for (size_t j = lastValidIdx + 1; j < i; j++) {
							// FIXME a linear interpolation should be better than radial interpolation
							double a = (double) (j - lastValidIdx) / (i - lastValidIdx);
							extLaserData.points[j].reading = (1-a) * extLaserData.points[lastValidIdx].reading
								+ a * extLaserData.points[i].reading;
						}
					}
				}
				wasInvalid = false;
				lastValidIdx = i;
			}
			else {
				wasInvalid = true;
			}
		}
	}
	
	lastSensorPose = sensorPose;
}

}} // namespaces
