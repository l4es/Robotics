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

#define MODULE_NAME "Player2Module"

#include <rdkcore/geometry/utils.h>
#include <rdkcore/geometry/robotmodels.h>
#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/rsensordata/rodometrydata.h>
#include <rdkcore/rsensordata/rblobdata.h>
#include <rdkcore/rsensordata/rfiducialdata.h>
#include <rdkcore/sensordata/laserdata.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE MODULE_NAME

#include "player2module.h"
#include <cstring>

#define PROPERTY_ROBOT_CAMERA_DATA "cameraData"
#define PROPERTY_ROBOT_TAG_DATA "tagData"
#define PROPERTY_PLAYERSERVER_HOST "params/serverHost"
#define PROPERTY_PLAYERSERVER_PORT "params/serverPort"
#define PROPERTY_SIMSERVER_HOST "params/simServerHost"
#define PROPERTY_SIMSERVER_PORT "params/simServerPort"
#define PROPERTY_SLEEP "params/sleepTime"
#define PROPERTY_ROBOT_CLASS "params/robotClass"
#define PROPERTY_ENABLE_LASER "params/enableLaser"
#define PROPERTY_ENABLE_SONAR "params/enableSonar"
#define PROPERTY_ENABLE_CAMERA "params/enableCamera"
#define PROPERTY_ENABLE_FIDUCIAL "params/enableFiducial"
#define PROPERTY_ENABLE_MOTORS "params/enableMotors"
#define PROPERTY_ENABLE_GROUND_TRUTH "params/enableGroundTruth"
#define PROPERTY_GROUND_TRUTH_POSE "out/groundTruthPose"
#define PROPERTY_ROBOT_NAME "params/robotName"
#define PROPERTY_WORLD_CENTER "params/centerOfWorld"

#define PROPERTY_CMD_SIMULATION_SET_POSE "cmds/simulationSetPose"
#define PROPERTY_CMD_SIMULATION_SET_POSE_ARG "cmds/simulationSetPoseArg"

#define PROPERTY_USE_GROUND_TRUTH_FOR_LASER "params/useGroundTruthForLaser"

namespace RDK2 { namespace RAgent {

bool Player2Module::initConfigurationProperties()
{
	SESSION_TRY_START(session)

		Common::createDefaultProperties(session, true);
		Common::createRobotProperties(session);
		session->createString(PROPERTY_PLAYERSERVER_HOST, "Player server host", "localhost", READ_ONLY);
		session->createInt(PROPERTY_PLAYERSERVER_PORT, "Player server port", PLAYER_PORTNUM, READ_ONLY);
		session->createString(PROPERTY_SIMSERVER_HOST, "Simulation server (Player) host", "localhost", READ_ONLY);
		session->createInt(PROPERTY_SIMSERVER_PORT, "Simulation server (Player) port", PLAYER_PORTNUM, READ_ONLY);
		session->createString(PROPERTY_ROBOT_CLASS, "RobotClass","PatrolBot", READ_ONLY);
		session->createStorage("RBlobData", PROPERTY_ROBOT_CAMERA_DATA, "Camera data (aka virtual images)");
		session->createStorage("RFiducialData", PROPERTY_ROBOT_TAG_DATA, "Tag data (from fiducial sensor)");

		session->createBool(PROPERTY_ENABLE_LASER, "Enable laser client", true);
		session->createBool(PROPERTY_ENABLE_SONAR, "Enable sonar client", false);
		session->createBool(PROPERTY_ENABLE_MOTORS, "Enable motors client", true);
		session->createBool(PROPERTY_ENABLE_CAMERA, "Enable camera client", false);
		session->createBool(PROPERTY_ENABLE_FIDUCIAL, "Enable fiducial client", false);
		session->createBool(PROPERTY_ENABLE_GROUND_TRUTH, "Enable ground truth pose", false);
		session->createPose(PROPERTY_GROUND_TRUTH_POSE, "Ground truth pose", Point2od(0., 0., 0.));
		session->createDouble(PROPERTY_SLEEP, "Time to wait between read()s. ", RDouble::SEC, 0.010);

		session->createBool(PROPERTY_CMD_SIMULATION_SET_POSE, "Reset robot pose in simulation", false);
		session->createPose(PROPERTY_CMD_SIMULATION_SET_POSE_ARG, "Pose to reset in simulation", Point2od(0., 0., 0.));
		session->createBool(PROPERTY_USE_GROUND_TRUTH_FOR_LASER, "Put ground truth pose in laser scans", false);
		session->createPose(PROPERTY_WORLD_CENTER, "Center of the simulated world", Point2od(0., 0., 0.));

		session->createString(PROPERTY_ROBOT_NAME, "Robot name used within Player/Stage world", "robot1");

	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

bool Player2Module::init()
{
	SESSION_TRY_START(session)

		string host = session->getString(PROPERTY_PLAYERSERVER_HOST);
		int port = session->getInt(PROPERTY_PLAYERSERVER_PORT);
		RDK_DEBUG_STREAM("Connecting to Player server at " << host << ":" << port);

		try {

			playerClient = new PlayerClient(host, port);
			if (!playerClient) {
				RDK_ERROR_PRINTF("Cannot connect to Player server at %s:%d", host.c_str(), port);
				return false;
			}

			playerClient->SetDataMode(PLAYER_DATAMODE_PULL);
			playerClient->SetReplaceRule(true, -1, -1, -1);

			RDK_TRACE_STREAM("Available devices:");
			playerClient->RequestDeviceList();
			list<playerc_device_info_t> devs = playerClient->GetDeviceList();
			for (list<playerc_device_info_t>::iterator it = devs.begin();
			it != devs.end(); ++it) {
				RDK_TRACE_PRINTF("Driver '%s', interface: %s, index %d",
								it->drivername, playerClient->LookupName(it->addr.interf).c_str(), it->addr.index);
			}		
	
			if (session->getBool(PROPERTY_ENABLE_GROUND_TRUTH)) {
				simClient = new PlayerClient(
													session->getString(PROPERTY_SIMSERVER_HOST)
													,
													session->getInt(PROPERTY_SIMSERVER_PORT));
				if (!simClient) {
					RDK_ERROR_PRINTF("Cannot connect to Simulation server at %s:%d", host.c_str(), port);
					return false;
				}
				simulationProxy = new SimulationProxy(simClient, 0);
				RDK_DEBUG_PRINTF("Simulation proxy enabled");
			}

			if (session->getBool(PROPERTY_ENABLE_MOTORS)) {
				positionProxy = new Position2dProxy(playerClient, 0);
				positionProxy->SetMotorEnable(true);
				RDK_DEBUG_PRINTF("Motors enabled");
			}
			
			if (session->getBool(PROPERTY_ENABLE_LASER)) {
				laserProxy = new LaserProxy(playerClient, 0);
				// LM: the player driver for Sick PLS has some trouble on
				// configuring the laser
				// thus we will avoid this and then correct the data later
				if (session->getString(PROPERTY_ROBOT_CLASS)!="SickPLS")
				{
					laserProxy->RequestConfigure();
				}
				RDK_DEBUG_PRINTF("Laser enabled");
			}

			if (session->getBool(PROPERTY_ENABLE_SONAR)) {
				sonarProxy= new SonarProxy(playerClient, 0);
				sonarProxy->RequestGeom();
				RDK_DEBUG_PRINTF("Sonar enabled");
			}

			if (session->getBool(PROPERTY_ENABLE_CAMERA)) {
#if defined PLAYER_VERSION_LT_2_1 || defined PLAYER_VERSION_LT_3
				ptzProxy = new PlayerCc::PtzProxy ( playerClient, 0);
#endif

				blobfinderProxy = new PlayerCc::BlobfinderProxy ( playerClient );
				RDK_DEBUG_PRINTF("Camera enabled");
			}

			if (session->getBool(PROPERTY_ENABLE_FIDUCIAL)) {
				fiducialProxy = new PlayerCc::FiducialProxy ( playerClient, 0);
				fiducialProxy->RequestGeometry();
				RDK_DEBUG_PRINTF("Fiducial enabled");
			}

			RDK_INFO_STREAM("Connected to player server at " << host << ":" << port);
		}
		catch (PlayerCc::PlayerError e) {
			ostringstream oss;
			oss << e;
			RDK_ERROR_PRINTF("Cannot connect to player server: %s", oss.str().c_str());
			return false;
//			return true;
		}
	SESSION_END_CATCH_TERMINATE(session)
	return true;
}

void Player2Module::exec()
{
	while (session->dontWait(), !exiting) {
		try {
			playerClient->Read();
		}
		catch (const PlayerCc::PlayerError&) {
			RDK_ERROR_PRINTF("Error in reading from server, retrying in 1s...");
			sleep(1);
			continue;
		}
		double sleepTime = 0.0;

		SESSION_TRY_START(session)

			if (session->getBool(PROPERTY_ENABLE_MOTORS))       readOdometry();
			if (session->getBool(PROPERTY_ENABLE_LASER))        readLaser();
			if (session->getBool(PROPERTY_ENABLE_SONAR))        readSonar();
			if (session->getBool(PROPERTY_ENABLE_CAMERA))       readImage();
			if (session->getBool(PROPERTY_ENABLE_FIDUCIAL))     readFiducial();
			if (session->getBool(PROPERTY_ENABLE_GROUND_TRUTH)) readGroundTruthPose();	

			if (session->getBool(PROPERTY_ENABLE_MOTORS)) updatePlayerSpeed(session);

			if (session->getBool(PROPERTY_CMD_SIMULATION_SET_POSE)) {
				session->setBool(PROPERTY_CMD_SIMULATION_SET_POSE, false);
				if (simulationProxy) {
					Point2od poseToSet = session->getPose(PROPERTY_CMD_SIMULATION_SET_POSE_ARG);
					char * cstr;
					string name = session->getString(PROPERTY_ROBOT_NAME);
					cstr = new char [name.size()+1];
					strcpy (cstr, name.c_str());
					poseToSet = add(session->getPose(PROPERTY_WORLD_CENTER),poseToSet);
					RDK_WARNING_STREAM("SimPOse " << poseToSet);
					simulationProxy->SetPose2d(cstr, poseToSet.x, poseToSet.y, poseToSet.theta);
				}
				else {
					RDK_ERROR_PRINTF("No simulation proxy available for resetting pose");
				}
			}
			sleepTime = session->getDouble(PROPERTY_SLEEP);

		SESSION_END_CATCH_TERMINATE(session)

		usleep((uint) (sleepTime * 1000 * 1000));
	}
}

void Player2Module::readGroundTruthPose() throw (SessionException)
{
//	if (!simulationProxy->IsValid()) return;	// NOTE: for some reason, IsValid will always be false for simulationProxy (ask Player mailing list)
	SESSION_TRY(session)
		Point2od pose;
		char * cstr;
		string name = session->getString(PROPERTY_ROBOT_NAME);
		cstr = new char [name.size()+1];
		strcpy (cstr, name.c_str());
		simulationProxy->GetPose2d(cstr, pose.x, pose.y, pose.theta);
		pose = difference(session->getPose(PROPERTY_WORLD_CENTER),pose);
		session->setPose(PROPERTY_GROUND_TRUTH_POSE, pose);
		delete [] cstr;
	SESSION_CATCH_RETHROW(session)
}

void Player2Module::readOdometry() throw (SessionException)
{
	if (!positionProxy->IsValid()) return;

	ROdometryData * rod = new ROdometryData();

	rod->timestamp.setToNow();
	rod->ipc_timestamp.setToNow();

	Point2od pose;
	pose.x = positionProxy->GetXPos();
	pose.y = positionProxy->GetYPos();
	pose.theta = positionProxy->GetYaw();

	pose = difference(session->getPose(PROPERTY_WORLD_CENTER),pose);

	rod->odometryPose.x = pose.x;
	rod->odometryPose.y = pose.y;
	rod->odometryPose.theta = pose.theta;
	rod->estimatedPose = rod->odometryPose;
	rod->leftStalled = positionProxy->GetStall();
	rod->rightStalled = positionProxy->GetStall();
	rod->speed = positionProxy->GetXSpeed();
	rod->jog = positionProxy->GetYawSpeed();

	session->queuePush(PROPERTY_ROBOT_DATA_QUEUE, rod);
	session->setPose(PROPERTY_ROBOT_ODOMETRY_POSE, pose);
}

void Player2Module::readLaser() throw(SessionException)
{
	if (!laserProxy->IsValid()) return;

	RLaserData* rld = new RLaserData;
	int n = laserProxy->GetCount();

	for (int i = 0; i < n; i++){
		LaserData::LaserPoint lp;
		lp.theta = laserProxy->GetBearing(i);
		if (session->getString(PROPERTY_ROBOT_CLASS)=="PatrolBot" && (i==15 || i==165)){
			lp.reading=(laserProxy->GetRange(i-1)+laserProxy->GetRange(i+1))/2.0;
		}
		// LM: the player driver for sickpls returns some weird parameters
		// I am putting here the necessary conversion
		else if (session->getString(PROPERTY_ROBOT_CLASS)=="SickPLS")
		{
			lp.reading = laserProxy->GetRange(i)/100;
			lp.theta    = deg2rad(lp.theta/100);
		}
		else
		{
			lp.reading = laserProxy->GetRange(i);
		}
		lp.intensity  = laserProxy->GetIntensity(i);
		rld->points.push_back(lp);
	}

	rld->tag = "FLASER";
	rld->timestamp.setToNow();
	rld->ipc_timestamp.setToNow();
	rld->laserPose.x = session->getPose(PROPERTY_ROBOT_LASER_POSE).x;
	rld->laserPose.y = session->getPose(PROPERTY_ROBOT_LASER_POSE).y;
	rld->laserPose.theta = session->getPose(PROPERTY_ROBOT_LASER_POSE).theta;
	rld->minReading = 0.0;	// FIXME LaserProxy lacks of GetMinRange()
	rld->maxReading = laserProxy->GetMaxRange();
	rld->minTheta = laserProxy->GetMinAngle();
	rld->maxTheta = laserProxy->GetMaxAngle();

	Point2od globalRobotPose;
	if (session->getBool(PROPERTY_USE_GROUND_TRUTH_FOR_LASER)) {
		char * cstr;
		string name = session->getString(PROPERTY_ROBOT_NAME);
		cstr = new char [name.size()+1];
		strcpy (cstr, name.c_str());
		simulationProxy->GetPose2d(cstr, globalRobotPose.x, globalRobotPose.y, globalRobotPose.theta);
	}
	else {
		globalRobotPose.x = positionProxy->GetXPos();
		globalRobotPose.y = positionProxy->GetYPos();
		globalRobotPose.theta = positionProxy->GetYaw();
	}

	Point2od globalLaserPose = rld->laserPose.toGlobalReferenceSystem(globalRobotPose);

	rld->odometryPose = globalLaserPose;
	rld->estimatedPose = globalLaserPose;

	session->queuePush(PROPERTY_ROBOT_DATA_QUEUE, rld->clone());
	session->setObject(PROPERTY_ROBOT_LASER_DATA, rld);
}

void Player2Module::readSonar() throw(SessionException)
{
	if (!sonarProxy->IsValid()) return;

	RSonarData * rsd = new RSonarData;
	int n = sonarProxy->GetCount();

	for (int i = 0; i < n; i++){
		SensorData::SonarDevice sd;
		sd.devicePose.x=sonarProxy->GetPose(i).px;
		sd.devicePose.y=sonarProxy->GetPose(i).py;
#ifdef PLAYER_VERSION_LT_2_1
		sd.devicePose.theta=sonarProxy->GetPose(i).pa;
#else
		sd.devicePose.theta=sonarProxy->GetPose(i).pyaw;
#endif
		sd.reading = sonarProxy->GetScan(i);
		sd.new_reading=true;
		if (sd.reading >0.001) rsd->devices.push_back(sd); //if is <0.001 means there is no real reading
	}

	rsd->minReading = 0.0;	// FIXME SonarProxy lacks of GetMinRange()
	rsd->maxReading = 8.0;  // FIXME this should be provided by Player

	session->queuePush(PROPERTY_ROBOT_DATA_QUEUE, rsd->clone());
	session->setObject(PROPERTY_ROBOT_SONAR_DATA, rsd);
}


void Player2Module::readImage() throw ( SessionException )
{
	if ( blobfinderProxy == 0 || !blobfinderProxy->IsValid() ) return;

	uint32_t bCount = blobfinderProxy->GetCount();
	if (bCount == 0)
		return;

	RBlobData* rbd = new RBlobData();

	rbd->timestamp.setToNow();
	rbd->ipc_timestamp.setToNow();
	Point2od globalRobotPose;
	globalRobotPose.x = positionProxy->GetXPos();
	globalRobotPose.y = positionProxy->GetYPos();
	globalRobotPose.theta = positionProxy->GetYaw();
	rbd->odometryPose = globalRobotPose;
	rbd->estimatedPose = globalRobotPose;

	rbd->width = blobfinderProxy->GetWidth();
	rbd->height = blobfinderProxy->GetHeight();
	if (!( ptzProxy == 0 || !ptzProxy->IsValid() ))
	{
		rbd->panAngle  = ptzProxy->GetPan();
		rbd->tiltAngle = ptzProxy->GetTilt();
		rbd->zoomAngle = ptzProxy->GetZoom();
	}
	else
	{
		//rbd->panAngle  = ptzProxy->GetPan();
		//rbd->tiltAngle = ptzProxy->GetTilt();
		rbd->zoomAngle = 1;
	}

	for (uint32_t i=0; i<bCount; ++i)
	{
		playerc_blobfinder_blob_t blob = blobfinderProxy->GetBlob(i);
		BlobData::Blob b;
		b.id     = blob.id;
		b.color  = blob.color;
		b.area   = blob.area;
		b.x      = blob.x;
		b.y      = blob.y;
		b.left   = blob.left;
		b.right  = blob.right;
		b.top    = blob.top;
		b.bottom = blob.bottom;
		b.range  = blob.range/1000; // there is a bug in stage here: range = (int)range*1000; so I will normalize it to meters
		rbd->blobSet.push_back(b);
	}

	session->queuePush(PROPERTY_ROBOT_DATA_QUEUE, rbd->clone());
	session->setObject(PROPERTY_ROBOT_CAMERA_DATA, rbd);
}

void Player2Module::readFiducial() throw ( SessionException )
{
	if ( fiducialProxy == 0 || !fiducialProxy->IsValid() ) return;


	RFiducialData* rfd = new RFiducialData();

	rfd->timestamp.setToNow();
	rfd->ipc_timestamp.setToNow();
	Point2od globalRobotPose;
	globalRobotPose.x = positionProxy->GetXPos();
	globalRobotPose.y = positionProxy->GetYPos();
	globalRobotPose.theta = positionProxy->GetYaw();
	rfd->odometryPose = globalRobotPose;
	rfd->estimatedPose = globalRobotPose;

	/* :TODO:08/11/2009 11:36:33 AM:lm: not available for player 2.0.4 */
	//rfd->minReading = ;
	//rfd->maxReading = ;
	//rfd->maxReadingID = ;
	//rfd->fov = ;


	uint32_t fCount = fiducialProxy->GetCount();
	for (uint32_t i=0; i<fCount; ++i)
	{
		player_fiducial_item_t f = fiducialProxy->GetFiducialItem(i);
		FiducialData::Tag t;
		t.id            = f.id;
		t.x             = f.pose.px;
		t.y             = f.pose.py;
		t.z             = f.pose.pz;
		//using DC convention. See rdkcore/geometry/point3.h
		t.theta = f.pose.pyaw;
		t.phi   = f.pose.ppitch;
		t.gamma = f.pose.proll;
		t.uncertainty.x = f.upose.px;
		t.uncertainty.y = f.upose.py;
		t.uncertainty.z = f.upose.pz;
		t.uncertainty.theta = f.upose.pyaw;
		t.uncertainty.phi   = f.upose.ppitch;
		t.uncertainty.gamma = f.upose.proll;
		rfd->tagSet.push_back(t);
	}
	session->queuePush(PROPERTY_ROBOT_DATA_QUEUE, rfd->clone());
	session->setObject(PROPERTY_ROBOT_TAG_DATA, rfd);
}

void Player2Module::updatePlayerSpeed(Session* session) throw (SessionException)
{
	if (!positionProxy) return;

	double desiredSpeed = session->getDouble(PROPERTY_ROBOT_DESIRED_SPEED);
	double desiredJog = session->getDouble(PROPERTY_ROBOT_DESIRED_JOG);

	positionProxy->SetSpeed(desiredSpeed, desiredJog);

	double robotSpeed = positionProxy->GetXSpeed();
	double robotJog = positionProxy->GetYawSpeed();

	session->setDouble(PROPERTY_ROBOT_SPEED, robotSpeed);
	session->setDouble(PROPERTY_ROBOT_JOG, robotJog);
}

#define COMMAND_ROBOT_SET_SPEED "setSpeed"
#define COMMAND_ROBOT_SET_JOG "setJog"
#define COMMAND_ROBOT_SET_SPEED_AND_JOG "setSpeedAndJog"

bool Player2Module::parseRobotCmd(cstr cmd, Session* session)
{
	bool r = false;

	float arg1, arg2;

	if (0<sscanf(cmd.c_str(), COMMAND_ROBOT_SET_SPEED " %g", &arg1)){
		session->setDouble(PROPERTY_ROBOT_DESIRED_SPEED, arg1);
		r = true;
	}

	if (0<sscanf(cmd.c_str(), COMMAND_ROBOT_SET_JOG " %g", &arg1)){
		session->setDouble(PROPERTY_ROBOT_DESIRED_JOG, arg1);
		r = true;
	}

	if (0<sscanf(cmd.c_str(), COMMAND_ROBOT_SET_SPEED_AND_JOG " %g %g", &arg1, &arg2)){
		session->setDouble(PROPERTY_ROBOT_DESIRED_SPEED, arg1);
		session->setDouble(PROPERTY_ROBOT_DESIRED_JOG, arg2);
		r = true;
	}

	return r;
}

void Player2Module::asyncAgentCmd(cstr cmd)
{
	SESSION_TRY_START(asyncSession)
		if (parseRobotCmd(cmd, asyncSession)) updatePlayerSpeed(asyncSession);
	SESSION_END_CATCH_TERMINATE(asyncSession)
}

void Player2Module::cleanup()
{
	delete simulationProxy;
	delete positionProxy;
	delete fiducialProxy;
	delete blobfinderProxy;
	delete ptzProxy;
	delete sonarProxy;
	delete laserProxy;
	delete playerClient;
}

MODULE_FACTORY(Player2Module);

}} // namespace
