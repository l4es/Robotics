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
#define LOGGING_MODULE "RLaserData"

#include <rdkcore/rprimitive/rstring.h>
#include <sstream>
#include "rlaserdata.h"

namespace RDK2 { namespace RSensorData {

	RDK2_FACTORY(RLaserData);

	using namespace RDK2::SensorData;
	using namespace std;
	using namespace RDK2::Serialization;

	void RLaserData::read(Reader* r) throw (ReadingException)
	{
		r->startReading(getClassName());
		laserPose.x = r->read_f32();
		laserPose.y = r->read_f32();
		laserPose.z = r->read_f32();
		laserPose.theta = r->read_f32();
		laserPose.phi = r->read_f32();
		laserPose.gamma = r->read_f32();
		odometryPose.x = r->read_f32();
		odometryPose.y = r->read_f32();
		odometryPose.theta = r->read_f32();
		estimatedPose.x = r->read_f32();
		estimatedPose.y = r->read_f32();
		estimatedPose.theta = r->read_f32();
		minReading = r->read_f32();
		maxReading = r->read_f32();
		minTheta = r->read_f32();
		maxTheta = r->read_f32();
		timestamp= r->read_f32();
		ipc_timestamp = r->read_f32();
		tag = r->readString();
		sensorName = r->readString();
		ipc_hostname = r->readString();
		uint nrays = (uint) r->read_i32();
		//if (nrays > 400) {
		//  r->doneReading();
		//  throw ReadingException("What the devil?");
		//}
		points.resize(nrays);
		double* buf;
		size_t buflen;
		r->readBytes((void**)&buf, &buflen);
		/*		minTheta = -M_PI;
					maxTheta = M_PI;*/
		//RDK_DEBUG_PRINTF("Dio %.2f %.2f %d", minTheta, maxTheta, nrays);
		double step = (maxTheta - minTheta) / nrays;
		for (size_t i = 0; i < nrays; i++) {
			points[i].theta = step * i + minTheta; //JMK Q(o), minTheta here is NEEDED serialzation sucked!!!!!!!!!!
			points[i].reading = buf[i];
			//RDK_DEBUG_PRINTF("A %d %.2f", i, buf[i]);
			points[i].intensity = 1.;
		}
		delete[] buf;
		r->readBytes((void**)&buf, &buflen);
		delete[] buf;
		r->doneReading();
	}

	void RLaserData::write(Writer* w) const throw (WritingException)
	{
		w->startWriting(getClassName());
		w->write_f32(laserPose.x);
		w->write_f32(laserPose.y);
		w->write_f32(laserPose.z);
		w->write_f32(laserPose.theta);
		w->write_f32(laserPose.phi);
		w->write_f32(laserPose.gamma);
		w->write_f32(odometryPose.x);
		w->write_f32(odometryPose.y);
		w->write_f32(odometryPose.theta);
		w->write_f32(estimatedPose.x);
		w->write_f32(estimatedPose.y);
		w->write_f32(estimatedPose.theta);
		w->write_f32(minReading);
		w->write_f32(maxReading);
		w->write_f32(minTheta);
		w->write_f32(maxTheta);
		w->write_f32(timestamp.getSeconds());
		w->write_f32(ipc_timestamp.getSeconds());
		w->writeString(tag);
		w->writeString(sensorName);
		w->writeString(ipc_hostname);
		w->write_i32((int) points.size());
		double* buf = new double[points.size()];
		for (size_t i = 0; i < points.size(); i++) buf[i] = points[i].reading;
		w->writeBytes((void*)buf, (points.size()) * sizeof(double));
		w->writeBytes((void*)buf, (110) * sizeof(double));
		delete[] buf;
		//cout << "dio" << endl;
		w->doneWriting();
	}

	string RLaserData::getLogLine() const
	{
		ostringstream oss;
		if (logtag == "FLASER")
		{
			oss << logtag << " " << points.size() << " ";
			for (size_t i = 0; i < points.size(); i++) 
			{
				oss << points[i].reading << " ";
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
		else if (logtag == "ROBOTLASER1")
		{
			oss << logtag << " ";
			oss << getStringRepresentation();
			return oss.str();
		}
		else
		{
			RDK_DEBUG_PRINTF("Error on laserdata format");
			return "";
		}
	}

	bool RLaserData::loadFromLogLine(cstr line)
	{
		istringstream iss(line);
		string name;
		iss >> name;
		if (name == "FLASER")
		{
			uint pointSize;
			iss >> pointSize;
			points.resize(pointSize);
			// XXX guessing... one ray for each degree
			double dd = pointSize * M_PI / 180;
			minReading = 0.02;
			maxReading = 80.;
			minTheta = -dd/2;
			maxTheta = dd/2;
			double da = dd/pointSize;
			for (size_t i = 0; i < points.size(); i++) {
				iss >> points[i].reading;
				points[i].theta = minTheta + da*i;
			}
			iss >> odometryPose.x >> odometryPose.y >> odometryPose.theta;
			iss >> estimatedPose.x >> estimatedPose.y >> estimatedPose.theta;
			double a;
			iss >> a;
			timestamp = a;
			iss >> ipc_hostname;
			iss >> a;
			ipc_timestamp = a;
			return true;
		}
		else if (name == "ROBOTLASER1")
		{
			logtag = name;
			getline(iss,name);
			loadFromStringRepresentation(name);
			return true;
		}
		else
		{
			RDK_ERROR_PRINTF("The log line must start with FLASER or ROBOTLASER1");
			return false;
		}
		return false;
	}

	std::string RLaserData::getCarmenLogLine() const
	{
		std::ostringstream oss;
		oss << 99 << " " // carmen(0.7.4-beta) laser_messages.h:UNKNOWN_PROXIMITY_SENSOR
			<< minTheta << " "
			<< (maxTheta-minTheta) << " "
			<< getRayWidth() << " "
			<< maxReading << " "
			<< 0.001 << " " // carmen(0.7.4-beta) SICK=0.01 Hokuyo=0.001
			<< 0 << " " // carmen(0.7.4-beta) remission mode (NONE=0, DIRECT=1, NORMALIZED=2
			<< points.size() << " ";
		//copy(points.begin(),points.end(),std::ostream_iterator<double>(oss," "));
		for (size_t i = 0; i < points.size(); i++)
		{
			oss << points[i].reading << " ";
		}
		
		Point2od globalLaserPose = laserPose.toGlobalReferenceSystem(estimatedPose);
				
		oss << 0 << " " // remission size
			// #remission > 0 then fix the lines below
			//for (size_t i = 0; i < remissions.size(); i++)
			//{
			//  oss << remissions[i] << " ";
			//}
			<< globalLaserPose.x << " "
			<< globalLaserPose.y << " "
			<< globalLaserPose.theta << " "
			<< estimatedPose.x << " "
			<< estimatedPose.y << " "
			<< estimatedPose.theta << " "
			<< 0. << " " // carmen(0.7.4-beta) ??? tv
			<< 0. << " " // carmen(0.7.4-beta) ??? rv
			<< 0. << " " // carmen(0.7.4-beta) ??? forward safetydist
			<< 0. << " " // carmen(0.7.4-beta) ??? side safetydist
			<< 0. << " " // carmen(0.7.4-beta) ??? turnaxis
			<< timestamp.getMsFromMidnight() << " "
			<< ipc_hostname << " "
			<< ipc_timestamp.getMsFromMidnight() << " ";
		return oss.str();
	}

	bool RLaserData::loadFromCarmenLogLine(const std::string& cstr)
	{
		std::istringstream iss(cstr);
		int iDiscardedValue,iToParseValue;
		double dDiscardedValue,dToParseValue; 
		unsigned long lToParseValue;
		iss >> iDiscardedValue // carmen(0.7.4-beta) laser_messages.h:UNKNOWN_PROXIMITY_SENSOR (type)
			>> minTheta
			>> dToParseValue;
		maxTheta = minTheta + dToParseValue;
		iss >> dDiscardedValue // angular_resolution is computed by getRayWidth
			>> maxReading
			>> dDiscardedValue // carmen(0.7.4-beta) accuracy: SICK=0.01 Hokuyo=0.001
			>> iDiscardedValue // carmen(0.7.4-beta) remission mode: (NONE=0, DIRECT=1, NORMALIZED=2
			>> iToParseValue; // points.size
		points.resize(iToParseValue);
		updateThetas();
		//copy(points.begin(),points.end(),std::ostream_iterator<double>(oss," "));
		for (int i = 0; i < iToParseValue; i++)
		{
			iss >> points[i].reading;
		}
		iss >> iToParseValue; // remission size
		// #remission > 0 then fix the lines below
		for (int i = 0; i < iToParseValue; i++)
		{
			iss >> dDiscardedValue;
		}
		Point2od globalLaserPose;
		iss >> globalLaserPose.x
			>> globalLaserPose.y
			>> globalLaserPose.theta
			>> estimatedPose.x
			>> estimatedPose.y
			>> estimatedPose.theta
			>> dDiscardedValue // carmen(0.7.4-beta) ??? tv
			>> dDiscardedValue // carmen(0.7.4-beta) ??? rv
			>> dDiscardedValue // carmen(0.7.4-beta) ??? forward safetydist
			>> dDiscardedValue // carmen(0.7.4-beta) ??? side safetydist
			>> dDiscardedValue // carmen(0.7.4-beta) ??? turnaxis
			>> lToParseValue;
		Point2od localLaserPose = globalLaserPose.toLocalReferenceSystem(estimatedPose); // RDK wants a local pose
		laserPose.x = localLaserPose.x;
		laserPose.y = localLaserPose.y;
		laserPose.theta = localLaserPose.theta;
		timestamp.setMsFromMidnight(lToParseValue);
		iss >> ipc_hostname
				>> lToParseValue;
		ipc_timestamp.setMsFromMidnight(lToParseValue);
		return !iss.fail();
	}

	
	std::string RLaserData::getStringRepresentation() const
	{
		std::ostringstream oss;
		oss << getCarmenLogLine() << " "
		    << "# "
		    << minReading         << " "
		    << roll               << " "
		    << pitch              << " "
		    << laserPan           << " "
				<< sensorName         << " "
				<< odometryPose       << " ";
		return oss.str();
	}

	bool RLaserData::loadFromStringRepresentation(const std::string& cstr)
	{
		std::istringstream iss(cstr.substr(cstr.find_last_of("#")+1));
		iss >> minReading
		    >> roll
		    >> pitch
		    >> laserPan
		    >> sensorName
		    >> odometryPose;
		return loadFromCarmenLogLine(cstr) && !iss.fail();
	}

}} // namespaces

