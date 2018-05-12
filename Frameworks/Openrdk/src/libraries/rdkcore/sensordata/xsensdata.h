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

#ifndef H_XSENSDATA
#define H_XSENSDATA
#include <rdkcore/geometry/point.h>
#include <rdkcore/geometry/quaternion.h>
#include "sensordata.h"


namespace RDK2 { namespace SensorData {

struct XsensData: public BaseSensorData {
	XsensData(): odometry(0.,0.,0.), estimated(0.,0.,0.), accX(0.), accY(0.), accZ(0.), q(0.,0.,0.,1.) {
		tag = "XSENS";
	}
	RDK2::Geometry::Point2od odometry;
	RDK2::Geometry::Point2od estimated;
	double accX, accY, accZ;
	RDK2::Geometry::Quaterniond q;
	
	virtual ~XsensData(){}
};

}} // end namespaces

#endif
