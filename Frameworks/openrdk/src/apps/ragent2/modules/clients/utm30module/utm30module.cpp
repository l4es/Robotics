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

#define MODULE_NAME "UTM30Module"

#include <cstring>

#include <rdkcore/geometry/utils.h>
#include <rdkcore/geometry/robotmodels.h>
#include <rdkcore/rsensordata/rodometrydata.h>
#include <rdkcore/sensordata/laserdata.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE MODULE_NAME

#include "utm30module.h"


#define PROPERTY_LASER_PORT "params/laserPort"
#define PROPERTY_LASER_TYPE "params/laserType"
#define PROPERTY_LASER_POSE "params/laserPose"
#define PROPERTY_LASER_DATA "out/laserData"
#define PROPERTY_LASER_DATA_QUEUE "out/laserDataQueue"
#define PROPERTY_ROBOT_POSE "in/robotPose"



namespace RDK2 { namespace RAgent {

bool UTM30Module::initConfigurationProperties()
{
	SESSION_TRY_START(session)

		Common::createDefaultProperties(session, true);
		session->createString(PROPERTY_LASER_PORT, "Laser port", "/dev/ttyACM0", READ_ONLY);
		session->createString(PROPERTY_LASER_TYPE, "Laser type", "UTM-30LX", READ_ONLY);
		session->createPose(PROPERTY_LASER_POSE, "Laser pose wrt the base platform", Point2od(0., 0., 0.));
		session->createPose(PROPERTY_ROBOT_POSE, "Robot pose", Point2od(0., 0., 0.));
		session->createStorage("RLaserData",PROPERTY_LASER_DATA, "Laser data");
		session->createQueue(PROPERTY_LASER_DATA_QUEUE, "Laser data queue");

		
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

bool UTM30Module::init()
{
	SESSION_TRY_START(session)

	string port = session->getString(PROPERTY_LASER_PORT);		
	string type = session->getString(PROPERTY_LASER_TYPE);
	
	RDK_INFO_STREAM("Connecting to laser " << type << " at " << port);

	int o=hokuyo_open_usb(&urg,port.c_str());
	if (o<=0) {
	      RDK_ERROR_STREAM("Cannot connect to laser " << type << " at " << port  );
	      return false;
	}

	o=hokuyo_init(&urg,1);
	if (o<=0) {
	      RDK_ERROR_STREAM("Cannot connect to laser " << type << " at " << port  );
	      return false;
	}
  
	o=hokuyo_startContinuous(&urg, 0, urg.maxBeams, 0, 0);
	if (o<=0){
	      RDK_ERROR_STREAM("Cannot connect to laser " << type << " at " << port  );
	      return false;
	}
	
	
	
	// open laser
	      
	RDK_INFO_PRINTF("Laser enabled");
	      
	
	SESSION_END_CATCH_TERMINATE(session)
	return true;
}

void UTM30Module::exec()
{
	while (session->dontWait(), !exiting) {

	      SESSION_TRY_START(session)

	      readLaser();
	      
	      SESSION_END_CATCH_TERMINATE(session)
	      
	      usleep(5000);
	}
}


void UTM30Module::readLaser() throw(SessionException)
{
  
    hokuyo_readPacket(&urg, buf, HOKUYO_BUFSIZE,10);
    HokuyoRangeReading reading;
    hokuyo_parseReading(&reading, buf, 0);
    
    RLaserData* rld = new RLaserData;
    rld->points.resize(reading.n_ranges);
    
    double initialAngle=-(0.5*urg.angularResolution*urg.maxBeams);
    
    for (int i = 0; i < reading.n_ranges; i++) {
	LaserData::LaserPoint &lp = rld->points[i];
	
	lp.reading = reading.ranges[i]/1000.;
	lp.theta = initialAngle*M_PI+urg.angularResolution*i;
    }
    
    	rld->tag = "FLASER";
	rld->timestamp.setToNow();
	rld->ipc_timestamp.setToNow();
	rld->laserPose.x = session->getPose(PROPERTY_LASER_POSE).x;
	rld->laserPose.y = session->getPose(PROPERTY_LASER_POSE).y;
	rld->laserPose.theta = session->getPose(PROPERTY_LASER_POSE).theta;
	rld->minReading = 0.02;	
	rld->maxReading = 30.0;
	rld->minTheta = initialAngle;
	rld->maxTheta = -initialAngle;
	
	Point2od globalRobotPose;
	
	globalRobotPose.x = session->getPose(PROPERTY_ROBOT_POSE).x;
	globalRobotPose.y = session->getPose(PROPERTY_ROBOT_POSE).y;
	globalRobotPose.theta = session->getPose(PROPERTY_ROBOT_POSE).theta;
	
	Point2od globalLaserPose = rld->laserPose.toGlobalReferenceSystem(globalRobotPose);

	rld->odometryPose = globalLaserPose;
	rld->estimatedPose = globalLaserPose;

	session->queuePush(PROPERTY_LASER_DATA_QUEUE, rld->clone());
	session->setObject(PROPERTY_LASER_DATA, rld);

}


void UTM30Module::cleanup()
{
	hokuyo_close(&urg);
}

MODULE_FACTORY(UTM30Module);

}} // namespace
