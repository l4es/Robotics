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

#include <math.h>
#include <sys/types.h>

#include <rdkcore/geometry/robotmodels.h>
#include <rdkcore/geometry/point.h>

#include "laserdata.h"

namespace RDK2 { namespace SensorData {

using namespace RDK2::Geometry;

 void laserdata2avv(const LaserData& scan, int npoints, double maxDist, AVV&avv) {
	for(int a=0;a<(int)scan.points.size();a++) {
		int from = std::max(0, a-npoints);
		int to = std::min((int)scan.points.size()-1, a+npoints);
		
		Point2d center = scan.points[a].ppoint().getCartesian();
		std::vector<Point2d> points;		
		for(int i=from;i<=to;i++) {
			Point2d point = scan.points[i].ppoint().getCartesian();
			if( (center-point).abs() <= maxDist) points.push_back(point);
		}
		AppliedVector av;
		av.where = center;
		double error, rho;
		regression(points, av.theta, rho, error);
		av.weight = 1;
		avv.push_back(av);
	}
}

void LaserData::updateThetas()
{
	double rayWidth = getRayWidth();
	for (size_t i = 0; i < points.size(); i++) points[i].theta = minTheta + i * rayWidth;
}

int LaserData::numValidPoints() const {
	int num=0;
	for(int a=0;a<(int)points.size();a++)
		if(points[a].reading != 0) num++;
	return num;
}

bool LaserData::raysEqualTo(const LaserData&ld) {
		if(points.size() != ld.points.size()) {
			return false;	
		}
		
		for(int a=0;a<(int)points.size();a++)
			if(points[a].reading != ld.points[a].reading)
				return false;
		
		return true;
	}

bool LaserData::isInsideClearArea(Point2d world) const {
	Point2od o = add(estimatedPose, laserPose);
	Point2d relative = (world-o).rot(-o.theta);
	double reading = relative.abs();
	if(reading<0.01) return false;
	double theta = norm2PIX(relative.theta());
	int ray = (int)((theta-minTheta) / (maxTheta-minTheta) * points.size());
	if(ray<0 || ray >= (int)points.size()) 
		return false;
	return points[ray].reading > reading && reading > minReading;
}


/// Returns minimum range (values of 0 are ignored)
double LaserData::minRange() const {
	bool set = false;
	double result = 1000;
	for(size_t a=0;a<points.size();a++) {
		if(points[a].reading<0) continue;
		if(!set) { result = points[a].reading; set = true;}
		result = std::min(result, points[a].reading);
	}
	return result;
}

int LaserData::getMinAngle()
{
	bool set = false;
	double min_ang = 1000;
	double result = 1000;
	for(size_t a=0;a<points.size();a++) {
		if(points[a].reading<0) continue;
		if(!set) { result = points[a].reading; set = true;}
		if (points[a].reading < result)
			min_ang=points[a].theta;
	}
	return min_ang;
	
}

void LaserData::copy(const LaserData&l) {
	laserPose = l.laserPose;
	minReading = l.minReading;
	maxReading = l.maxReading;
	minTheta = l.minTheta;
	maxTheta = l.maxTheta;
	odometryPose = l.odometryPose;
	estimatedPose = l.estimatedPose;
	
	for(int a=0;a<(int)l.points.size();a++) {
		LaserPoint lp;
		lp.reading = l.points[a].reading;
		lp.theta = l.points[a].theta;
		lp.intensity= l.points[a].intensity;
		lp.trueAlpha= l.points[a].trueAlpha;
		
		points.push_back(lp);
	}
	
	tag=l.tag;
	timestamp=l.timestamp;
	ipc_hostname=l.ipc_hostname;
	ipc_timestamp=l.ipc_timestamp;
}

void LaserData::mm2m() {
	laserPose = RDK2::Geometry::mm2m(laserPose);
	minReading = RDK2::Geometry::mm2m(minReading);
	maxReading = RDK2::Geometry::mm2m(maxReading);
	odometryPose = RDK2::Geometry::mm2m(odometryPose);
	estimatedPose = RDK2::Geometry::mm2m(estimatedPose);
	for(int a=0;a<(int)points.size();a++)
		points[a].reading = RDK2::Geometry::mm2m(points[a].reading);
}

}}
