#include "rpercept.h"

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "RPercept"

#include <cstring>

namespace RDK2 { namespace RSensorData
{

	RDK2_FACTORY(RPercept);

	void RPercept::read(Reader* r) throw (ReadingException)
	{
		r->startReading(getClassName());
			timestamp= r->read_f32();
			ipc_timestamp = r->read_f32();
			tag = r->readString();
			logtag = r->readString();
			ipc_hostname = r->readString();
			odometryPose.x = r->read_f32();
			odometryPose.y = r->read_f32();
			odometryPose.theta = r->read_f32();
			estimatedPose.x = r->read_f32();
			estimatedPose.y = r->read_f32();
			estimatedPose.theta = r->read_f32();
			rho = r->read_f32();
			theta = r->read_f32();
			cov.xx = r->read_f32();
			cov.yy = r->read_f32();
			cov.xy = r->read_f32();
		r->doneReading();
	}
	
	void RPercept::write(Writer* w) const throw (WritingException)
	{
		w->startWriting(getClassName());
			w->write_f32(timestamp.getSeconds());
			w->write_f32(ipc_timestamp.getSeconds());
			w->writeString(tag);
			w->writeString(logtag);
			w->writeString(ipc_hostname);
			w->write_f32(odometryPose.x);
			w->write_f32(odometryPose.y);
			w->write_f32(odometryPose.theta);
			w->write_f32(estimatedPose.x);
			w->write_f32(estimatedPose.y);
			w->write_f32(estimatedPose.theta);
			w->write_f32(rho);
			w->write_f32(theta);
			w->write_f32(cov.xx);
			w->write_f32(cov.yy);
			w->write_f32(cov.xy);
		w->doneWriting();
	}

	string RPercept::getLogLine() const
	{
		ostringstream oss;
		oss << logtag
				<< rho    << " " << theta  << " "
				<< cov.xx << " " << cov.yy << " " << cov.xy << " "
				<< odometryPose.x << " " << odometryPose.y << " " << odometryPose.theta << " "
				<< estimatedPose.x << " " << estimatedPose.y << " " << estimatedPose.theta << " "
				<< timestamp.getMsFromMidnight() << " "
				<< ipc_timestamp.getMsFromMidnight() << " "
				<< ipc_hostname << " " << tag;
		return oss.str();
	}

	bool RPercept::loadFromLogLine(cstr line)
	{
		istringstream iss(line);
		string name;
		iss >> name;
		if (strncmp(name.c_str(),logtag.substr(0,4).c_str(),4) != 0)
		{
			RDK_ERROR_STREAM("Error, the first token must be PCPT");
			return false;
		}
		iss >> rho >> theta >> cov.xx >> cov.yy >> cov.xy;
		iss >> odometryPose.x >> odometryPose.y >> odometryPose.theta;
		iss >> estimatedPose.x >> estimatedPose.y >> estimatedPose.theta;
		unsigned long a;
		iss >> a;
		timestamp.setMsFromMidnight(a);
		iss >> a;
		ipc_timestamp.setMsFromMidnight(a);
		iss >> ipc_hostname >> tag;
		return true;
	}
	
	RDK2_FACTORY(RPerceptVector);

}} // namespaces

