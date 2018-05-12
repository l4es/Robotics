#define MODULE_NAME "WebotsClientModule"

#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE MODULE_NAME

#include <cstring>

#include "webotsclientmodule.h"
#include "webots.h"
#include <nao/object/rnaojoints.h>

#define PROPERTY_WEBOTS_IN_DESIRED_JOINTS "in/desiredJoints"
#define PROPERTY_WEBOTS_OUT_BODY_JOINTS "out/bodyJoints"
#define PROPERTY_WEBOTS_OUT_IMAGE "out/image"
#define PROPERTY_WEBOTS_OUT_GPSPOSE "out/gpspose" // not yet implemented
#define PROPERTY_WEBOTS_OUT_GPSEULERANGLES "out/gpseuler" // not yet implemented
#define PROPERTY_WEBOTS_OUT_ESTIMATED_POSE "out/estimatedPose" // not yet implemented
#define PROPERTY_WEBOTS_OUT_TIMESTAMP "out/timestamp"

#define PROPERTY_WEBOTS_HOST "params/serverHost"
#define PROPERTY_WEBOTS_PORT "params/serverPort"

#define PROPERTY_PLAYER_NUMBER "params/playerNumber"

using namespace RDK2::Geometry;
using namespace RDK2::RGeometry;
using namespace RDK2::RGraphics;

namespace RDK2 { namespace RAgent {

bool WebotsClientModule::initConfigurationProperties() {
	SESSION_TRY_START(session)

	Common::createDefaultProperties(session, true);

	session->createString(PROPERTY_WEBOTS_HOST, "Webots host", WEBOTSHOST, READ_ONLY);
	session->createInt(PROPERTY_WEBOTS_PORT, "Webots port", WEBOTSPORT, READ_ONLY);
	session->createDouble(PROPERTY_WEBOTS_OUT_TIMESTAMP, "Webots timestamp", RDouble::SEC, 0.0, READ_ONLY);

	session->createInt(PROPERTY_PLAYER_NUMBER, "Player number (for multi-robot worlds)", 1);
	
	session->createStorage("RNaoJoints", PROPERTY_WEBOTS_IN_DESIRED_JOINTS, "Joints to send to simulator", new RNaoJoints());

	session->createStorage("RNaoJoints", PROPERTY_WEBOTS_OUT_BODY_JOINTS, "Current Joints configuration from the simulator", new RNaoJoints());

	// not yet implemented
	//session->createStorage("GPSPose3d",PROPERTY_WEBOTS_OUT_GPSPOSE, "GPS 3d pose of the robot",new GPSPose3d());
	//session->setObject(PROPERTY_WEBOTS_OUT_GPSPOSE,new GPSPose3d());

	// not yet implemented
	// session->createStorage("RPoint2od",PROPERTY_WEBOTS_OUT_ESTIMATED_POSE, "RPoint2od pose of the robot",new RPoint2od());
	// session->setObject(PROPERTY_WEBOTS_OUT_ESTIMATED_POSE,new RPoint2od());

	// not yet implemented
	//session->createStorage("GPSPose3d",PROPERTY_WEBOTS_OUT_GPSEULERANGLES, "GPS Euler Angles of the robot pose ",new GPSPose3d());
//	session->setObject(PROPERTY_WEBOTS_OUT_GPSEULERANGLES,new GPSPose3d());

	session->createImage(PROPERTY_WEBOTS_OUT_IMAGE, "Webots Image RGB24", WEBOTS_CAMERA_W, WEBOTS_CAMERA_H, RImage::RGB24);

	SESSION_END(session);
	return true;
	SESSION_CATCH_TERMINATE(session);
	return false;
}

bool WebotsClientModule::init() {
	SESSION_TRY_START(session)

	playerNumber = session->getInt(PROPERTY_PLAYER_NUMBER);

	string host = session->getString(PROPERTY_WEBOTS_HOST);
	int port = session->getInt(PROPERTY_WEBOTS_PORT);

	if (!wc.connect(playerNumber,host,port)) {
		RDK_DEBUG_PRINTF("Unable to connect to Webots Simulator", getModuleName().c_str());
		return false;
	}
	
	SESSION_END(session);
	return true;
	SESSION_CATCH_TERMINATE(session);
	return false;
}

void  WebotsClientModule::doExec() {	
	
	session->lock(PROPERTY_WEBOTS_IN_DESIRED_JOINTS, HERE);
	RNaoJoints* desjoints = session->getObjectAsL<RNaoJoints>(PROPERTY_WEBOTS_IN_DESIRED_JOINTS);
	session->unlock(PROPERTY_WEBOTS_IN_DESIRED_JOINTS);
	while (!wc.sendCmd(*desjoints)) 
		RDK_INFO_STREAM( " WARNING : it seems that webots cannot accept your command..." );
	
	session->valueChanged(PROPERTY_WEBOTS_IN_DESIRED_JOINTS);
	
	if (!wc.receive(wdata)) // HERE THIS MODULE REALLY WAIT...
	{
		RDK_INFO_STREAM("Something goes wrong receiving data...");
	}
	session->setDouble(PROPERTY_WEBOTS_OUT_TIMESTAMP,wdata.timestamp);

	session->lock(PROPERTY_WEBOTS_OUT_IMAGE, HERE);
	RImage *img = session->getObjectAsL<RImage>(PROPERTY_WEBOTS_OUT_IMAGE);
	memcpy(img->getBuffer(), wdata.image, WEBOTS_CAMERA_W*WEBOTS_CAMERA_H*3);
	session->unlock(PROPERTY_WEBOTS_OUT_IMAGE);
	session->valueChanged(PROPERTY_WEBOTS_OUT_IMAGE);


	session->lock(PROPERTY_WEBOTS_OUT_BODY_JOINTS, HERE);
	RNaoJoints* joints = session->getObjectAsL<RNaoJoints>(PROPERTY_WEBOTS_OUT_BODY_JOINTS);
	vector<float> &values = joints->getValues();
	values.clear();
	copy(wdata.joints,wdata.joints+RNaoJoints::NAO_JOINTS_COUNT,back_inserter(values));
	session->unlock(PROPERTY_WEBOTS_OUT_BODY_JOINTS);
}

void WebotsClientModule::exec() {
	while (session->dontWait(), !exiting) {
		SESSION_TRY_START(session)
		doExec();
		SESSION_END_CATCH_TERMINATE(session)
	}
}

void WebotsClientModule::exitRequested() {
	wc.close();
}

MODULE_FACTORY(WebotsClientModule);

}} // namespace
