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
#define LOGGING_MODULE "RStereoImageHeightsData"

#include "rstereoimageheightsdata.h"

namespace RDK2 { namespace RSensorData {

bool RStereoImageHeightsData::loadFromLogLine(cstr line)
{
	istringstream iss(line);
	string rtag;
	iss >> rtag;
	if (rtag != "STEREOIMAGEHEIGHTS") {
		RDK_ERROR_PRINTF("The log line must start with STEREOIMAGEHEIGHTS");
		return false;
	}
	int pts;
	iss >> pts;
	heights.resize(pts);
	for (int i = 0; i < pts; i++) {
		iss >> heights[i].p.x >> heights[i].p.y >> heights[i].height;
	}
	iss >> odometryPose.x >> odometryPose.y >> odometryPose.theta;
	iss >> estimatedPose.x >> estimatedPose.y >> estimatedPose.theta;
	int ts;
	iss >> ts;
	timestamp.setMsFromMidnight(ts);
	iss >> ipc_hostname;
	ipc_timestamp = timestamp;
	return true;
}

string RStereoImageHeightsData::getLogLine() const
{
	ostringstream oss;
	oss << "STEREOIMAGEHEIGHTS " << heights.size() << " ";
	for (size_t i = 0; i < heights.size(); i++) {
		oss << heights[i].p.x << " " << heights[i].p.y << " " << heights[i].height << " ";
	}
	oss << odometryPose.x << " ";
	oss << odometryPose.y << " ";
	oss << odometryPose.theta << " ";
	oss << estimatedPose.x << " ";
	oss << estimatedPose.y << " ";
	oss << estimatedPose.theta << " ";
	oss << timestamp.getMsFromMidnight() << " ";
	oss << ipc_hostname << " ";
	oss << ipc_timestamp.getMsFromMidnight();
	return oss.str();
}

}} // namespaces
