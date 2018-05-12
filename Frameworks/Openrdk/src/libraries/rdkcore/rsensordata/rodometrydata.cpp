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
#define LOGGING_MODULE "ROdometryData"

#include "rodometrydata.h"

namespace RDK2 { namespace RSensorData {

RDK2_FACTORY(ROdometryData);
	
string ROdometryData::getLogLine() const
{
	ostringstream oss;
	oss << "ODOM " << odometryPose.x << " " << odometryPose.y << " " << odometryPose.theta << " ";
	oss << estimatedPose.x << " " << estimatedPose.y << " " << estimatedPose.theta << " ";
	oss << timestamp.getMsFromMidnight() << " " 
		<< ipc_hostname << " " << ipc_timestamp.getMsFromMidnight();
	return oss.str();
}

bool ROdometryData::loadFromLogLine(cstr line)
{
	istringstream iss(line);
	string name;
	iss >> name;
	if (name != "ODOM") {
		RDK_ERROR_PRINTF("The first token in the line must be ODOM");
		return false;
	}
	iss >> odometryPose.x >> odometryPose.y >> odometryPose.theta;
	iss >> estimatedPose.x >> estimatedPose.y >> estimatedPose.theta;
	unsigned long a;
	iss >> a;
	timestamp.setMsFromMidnight(a);
	iss >> ipc_hostname;
	iss >> a;
	ipc_timestamp.setMsFromMidnight(a);
	return true;
}
	
}} // namespaces

