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

#include "rfiducialdata.h"

namespace RDK2 { namespace RSensorData {
	
	RDK2_FACTORY(RFiducialData);

	void RFiducialData::read(Reader* r) throw (ReadingException)
	{
		r->startReading(getClassName());
			tag = r->readString();
			logtag = r->readString();
			timestamp= r->read_f32();
			sensorName = r->readString();
			ipc_hostname = r->readString();
			ipc_timestamp = r->read_f32();
			odometryPose.x = r->read_f32();
			odometryPose.y = r->read_f32();
			odometryPose.theta = r->read_f32();
			estimatedPose.x = r->read_f32();
			estimatedPose.y = r->read_f32();
			estimatedPose.theta = r->read_f32();
			size_t size = r->read_u8();
			tagSet.reserve(size);
			for (size_t i=0; i<size; ++i)
			{
				Tag t;
				t.id     = r->read_i32();
				t.x  = r->read_f32();
				t.y  = r->read_f32();
				t.z  = r->read_f32();
				t.theta  = r->read_f32();
				t.phi  = r->read_f32();
				t.gamma  = r->read_f32();
				t.uncertainty.x  = r->read_f32();
				t.uncertainty.y  = r->read_f32();
				t.uncertainty.z  = r->read_f32();
				t.uncertainty.theta  = r->read_f32();
				t.uncertainty.phi  = r->read_f32();
				t.uncertainty.gamma  = r->read_f32();
				tagSet.push_back(t);
			}
		r->doneReading();
	}
	
	void RFiducialData::write(Writer* w) const throw (WritingException)
	{
		w->startWriting(getClassName());
			w->writeString(tag);
			w->writeString(logtag);
			w->write_f32(timestamp);
			w->writeString(sensorName);
			w->writeString(ipc_hostname);
			w->write_f32(ipc_timestamp);
			w->write_f32(odometryPose.x);
			w->write_f32(odometryPose.y);
			w->write_f32(odometryPose.theta);
			w->write_f32(estimatedPose.x);
			w->write_f32(estimatedPose.y);
			w->write_f32(estimatedPose.theta);
			w->write_u8(tagSet.size());
			for (size_t i=0; i<tagSet.size(); ++i)
			{
				const Tag& t = tagSet[i];
				w->write_i32(t.id);
				w->write_f32(t.x);
				w->write_f32(t.y);
				w->write_f32(t.z);
				w->write_f32(t.theta);
				w->write_f32(t.phi);
				w->write_f32(t.gamma);
				w->write_f32(t.uncertainty.x);
				w->write_f32(t.uncertainty.y);
				w->write_f32(t.uncertainty.z);
				w->write_f32(t.uncertainty.theta);
				w->write_f32(t.uncertainty.phi);
				w->write_f32(t.uncertainty.gamma);
			}
		w->doneWriting();
	}

	std::string RFiducialData::getStringRepresentation() const
	{
		std::ostringstream oss;
		oss << logtag << " "
		    << timestamp.getSeconds() << " "
		    //<< sensorName << " "
		    << ipc_hostname << " "
		    << ipc_timestamp.getSeconds() << " "
		    << odometryPose.x << " "
		    << odometryPose.y << " "
		    << odometryPose.theta << " "
		    << estimatedPose.x << " "
		    << estimatedPose.y << " "
		    << estimatedPose.theta << " "
		    << tagSet.size()                     << " " ;
		for (size_t i=0; i<tagSet.size(); ++i)
		{
			const Tag& t = tagSet[i];
			oss << t.id            << " "
			    << t.x             << " "
			    << t.y             << " "
			    << t.z             << " "
			    << t.theta         << " "
			    << t.phi           << " "
			    << t.gamma         << " "
			    << t.uncertainty.x << " "
			    << t.uncertainty.y << " "
			    << t.uncertainty.z << " "
			    << t.uncertainty.theta << " "
			    << t.uncertainty.phi << " "
			    << t.uncertainty.gamma << " ";
		}
		return oss.str();
	}

	bool RFiducialData::loadFromStringRepresentation(const std::string& cstr)
	{
		std::istringstream iss(cstr);
		double dToParseValue;
		iss >> logtag
				>> dToParseValue;
		timestamp = dToParseValue;
		iss //>> sensorName
				>> ipc_hostname
				>> dToParseValue;
		ipc_timestamp = dToParseValue;
		iss >> odometryPose.x  >> odometryPose.y  >> odometryPose.theta
		    >> estimatedPose.x >> estimatedPose.y >> estimatedPose.theta;
		size_t size;
		iss >> size;
		for (size_t i=0; i<size; ++i)
		{
			Tag t;
			iss >> t.id
			    >> t.x
			    >> t.y
			    >> t.z
			    >> t.theta
			    >> t.phi
			    >> t.gamma
			    >> t.uncertainty.x
			    >> t.uncertainty.y
			    >> t.uncertainty.z
			    >> t.uncertainty.theta
			    >> t.uncertainty.phi
			    >> t.uncertainty.gamma;
			tagSet.push_back(t);
		}
		return true;
	}

	
}} // namespaces

