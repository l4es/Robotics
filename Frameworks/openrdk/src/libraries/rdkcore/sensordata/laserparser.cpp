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

#include <rdkcore/textutils/linestream.h>
#include <rdkcore/textutils/textutils.h>
#include <rdkcore/geometry/point.h>
#include "laserparser.h"
#include "laserdata.h"

using namespace RDK2::TextUtils;
using namespace std;
using namespace RDK2::Geometry;


namespace RDK2 { namespace SensorData {

	LaserParser LaserData::laserParser;

	 Parser* LaserData::getParser() const {
		 return &laserParser;
	 }
	 
	void  LaserParser::setParam(const std::string& , const std::string&) {
		
	}
	
	BaseSensorData * LaserParser::parse(
		const std::string& data,
		std::string * error) 
	{
		LaserData * ld = new LaserData();
		
		StringVector sv = tokenize(data);
		
		int nrays;
		if(!RDK2::TextUtils::parse<int>(sv[1], nrays)) {
			if(error) *error = string("Could not parse number of rays: ")+sv[1];
			delete ld;
			return 0;
		}
		
		if(nrays<=0) {
			if(error) *error = string("Warning, no rays?: l = ")+data;
			delete ld;
			return 0;
		}
		
		int expected = 2+nrays+9;
		if( (int)sv.size() < expected) {
			if(error) *error = string("I expected ") + toString(expected) 
				+ " tokens instead of " + toString(sv.size());
			delete ld;
			return 0;
		}
		
		for(int a=0;a<nrays;a++) {
			double theta, rho, intensity;
			theta = -M_PI/2 + a * M_PI / nrays;
			intensity = 0;
			string t = sv[2+a];
			if(!RDK2::TextUtils::parse<double>(t, rho)) {
				if(error) *error = string("Could not parse ray #")+toString(a)+": "+t;
				delete ld;
				return 0;
			}
			LaserData::LaserPoint p;
			p.theta =theta;
			p.reading = rho;
			p.intensity= intensity;
			ld->points.push_back(p);
		}
	
		ld->tag = getTag();
		// FIXME: parametri
		ld->laserPose  = Point3od(0,0,0,0,0,0);
		ld->minReading = 0.1;
		ld->maxReading = 49;
		ld->minTheta = -M_PI/2;
		ld->maxTheta =  M_PI/2;
	
		if (
		   !parsePoint(sv[2+nrays+0], sv[2+nrays+1], sv[2+nrays+2], ld->estimatedPose, error)
		|| !parsePoint(sv[2+nrays+3], sv[2+nrays+4], sv[2+nrays+5], ld->odometryPose, error) ) {
			delete ld;
			return 0;
		}
		
		//double d;
		if(!RDK2::TextUtils::parse(sv[2+nrays+6], ld->ipc_timestamp)) {
			if(error) *error = string("Could not parse ipc_timestamp:")+sv[2+nrays+6];
			delete ld;
			return 0;
		}
		
		ld->ipc_hostname = sv[2+nrays+7];
		
		if(!RDK2::TextUtils::parse(sv[2+nrays+8], ld->timestamp)) {
			if(error) *error = string("Could not parse ipc_timestamp:")+sv[2+nrays+8];
			delete ld;
			return 0;
		}
		
		return ld;
	}

	bool LaserParser::write(
		const BaseSensorData * input,
		std::string& data,
		std::string* error) {
		
		const LaserData * ld = dynamic_cast<const LaserData*>(input);
		if(!ld) {
			if(error) *error = string("Could not convert to LaserData data tagged as ")+
				input->tag;
			return false;
		}
		
		ostringstream os;
		
		os << ld->tag <<  " ";
		
		os << ld->points.size();	
		
		for(int a=0;a<(int)ld->points.size();a++) {
			os << " " << ld->points[a].reading;
		}
		
		os 
		<< " " << ld->estimatedPose.x   << " " << ld->estimatedPose.y   << " " << ld->estimatedPose.theta 
		<< " " << ld->odometryPose.x << " " << ld->odometryPose.y << " " << ld
		->odometryPose.theta;
		
		os.precision(20);
		
		os
		<< " " << ld->ipc_timestamp.getSeconds()
		<< " " << ld->ipc_hostname
		<< " " << ld->timestamp.getSeconds();
		
		data = os.str();
		return true;
	}
		
		
}} // end namespace 

