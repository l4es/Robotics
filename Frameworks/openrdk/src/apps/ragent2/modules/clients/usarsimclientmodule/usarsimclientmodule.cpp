/**
 * This file contains some methods of the class
 * UsarSimClientModule. Other methods and utility functions are contained
 * in the other files in the same directory.
 */

#include <float.h>

//#define ENABLE_LOG

//Max number of time we allow to try a re-connection with UsarSim server
#define USBOT_STARTUP_CONN_LIMIT 60

#include <rdkcore/textutils/textutils.h>
#include <rdkcore/rmaps/rline2donmap.h>
#include "usarsimclientmodule.h"
#include "parsing.h"
#include "parser.h"

#include <sys/socket.h>
#include <sys/types.h>
#include <errno.h>
#include <netdb.h> /* for gethostbyaddr(3) */
#include <fcntl.h>  /* for fcntl(2) */
#include <netinet/in.h> /* for struct sockaddr_in, SOCK_STREAM */
#include <cstring>
#include <rdkcore/textutils/textutils.h>
#include <rdkcore/rsensordata/rusarrfiddata.h>
#include <rdkcore/rsensordata/rtouchsensordata.h>

using namespace RDK2::Geometry;

namespace RDK2 { namespace RAgent {
using namespace RDK2::RMaps;
using namespace RDK2::Geometry;


bool UsarSimClientModule::initConfigurationProperties()
{
	SESSION_TRY_START(session)
		Common::createDefaultProperties(session,true);
		Common::createRobotProperties(session);

		// Parameters
		session->createString(PROPERTY_ROBOT_CLASS, "Robot class", "USARBot.Zerg");
		session->createString(PROPERTY_USARSIM_ADDRESS, "USARSim server address", "127.0.0.1");
		session->createInt(PROPERTY_USARSIM_PORT, "USARSim server port", 3000);	// FIXME only one property 127.0.0.1:3000
		session->createString(PROPERTY_IMAGESERVER_ADDRESS, "Image server address", "127.0.0.1"); // as above
	
		// Start pose (FIXME use only one property of type Pose)
		session->createString(PROPERTY_START_POSE_NAME,
			"Start pose name (so you don't need to set numbers, leave blank to use numbers)", "");
		session->createDouble(PROPERTY_START_X,  "Start x position, Meters.", RDouble::M, 4.9);
		session->createDouble(PROPERTY_START_Y,  "Start y position, Meters.", RDouble::M, 1.9);
		session->createDouble(PROPERTY_START_Z,  "Start z position, Meters.", RDouble::M, 1.8);
		session->createDouble(PROPERTY_START_PITCH,  "Start Pitch, Rad.", RDouble::RAD, deg2rad(0.));
		session->createDouble(PROPERTY_START_ROLL,  "Start y position, Rad.", RDouble::RAD, deg2rad(0.));
		session->createDouble(PROPERTY_START_YAW,  "Start z position, Rad.", RDouble::RAD, deg2rad(0.));

		// Information
		session->createPose(PROPERTY_INITIAL_POSE_2D, "Initial pose 2D", Point2od(0., 0., 0.));
		session->setNotEditable(PROPERTY_INITIAL_POSE_2D);
		session->createPose3(PROPERTY_INITIAL_POSE_3D, "Initial pose 3D", Point3od(0., 0., 0., 0., 0., 0.));
		session->setNotEditable(PROPERTY_INITIAL_POSE_3D);
   		session->createInt(PROPERTY_MULTIVIEW_NUMBER, "Multiview subview number", -1);
   		session->setVolatile(PROPERTY_MULTIVIEW_NUMBER);
   		session->createInt(PROPERTY_BATTERY_LIFE, "Battery life", 0);
   		session->setVolatile(PROPERTY_BATTERY_LIFE);
   		session->createString(PROPERTY_VEHICLE_TYPE, "Vehicle type", "");
		session->setVolatile(PROPERTY_VEHICLE_TYPE);
		session->createDouble(PROPERTY_USARSIM_TIME, "USARSim time from the STA message", RDouble::SEC, 0.0);
		session->setVolatile(PROPERTY_USARSIM_TIME);
		
		// Encoders
		session->createString(PROPERTY_ENCODERS_CONFIG, "Configuration string for the encoders", "");

		// GPS sensor
		session->createBool(PROPERTY_GPS_ENABLED, "GPS sensor enabled", false);

		// Ground Truth
		session->createBool(PROPERTY_GROUND_TRUTH_POSE_ENABLED, "Ground truth pose enabled", false);

		// INS sensor
		session->createBool(PROPERTY_INS_ENABLED, "INS enabled", false);

		// Touch sensor
		session->createString(PROPERTY_TOUCH_SENSORS_CONFIG, "Configuration string for the touch sensors", "");

		// Speed/jog control
		session->createDouble(PROPERTY_CMD_SPEED, "Commanded speed", RDouble::M_SEC, 0.0);
		session->setVolatile(PROPERTY_CMD_SPEED);
		session->createDouble(PROPERTY_CMD_JOG, "Commanded jog", RDouble::RAD_SEC, 0.0);
		session->setVolatile(PROPERTY_CMD_JOG);
		
		// Flipper controllers config
		session->createString(PROPERTY_FLIPPER_CTRLS_CONFIG, "Flipper controllers configuration", "");

		
		
		/** DEPRECATED **/
		session->createStorage("RUsarInuData",PROPERTY_INU_SENSORDATA, 
				       "INU sensor data");
//		session->setPersistent(PROPERTY_INU_SENSORDATA, false);
		session->createBool(PROPERTY_USE_DEVICE_LASER, "Use device Laser", true);
		session->createBool(PROPERTY_USE_GROUND_TRUE_POSE, "Use real pose", false);
		session->createBool(PROPERTY_USE_RFID_LOCATION, "Use Location information for RFID", true);
		session->createBool(PROPERTY_USE_DEVICE_SONAR, "Use device Sonar", true);
		//session->createBool(PROPERTY_USE_DEVICE_PANTILT, "Use device PanTilt", true);
		session->createBool(PROPERTY_USE_DEVICE_IRCAMERA, "Use device IRCamera", true);
		session->createBool(PROPERTY_USE_USARSIM_TIMESTAMPS, "Use UsarSim timestamps", false);

		session->createDouble(PROPERTY_EXPANSION_SIZE,"Link to obstacle expander property ",RDouble::M,0.25);

		session->createBool(PROPERTY_SAVE_DATA,"check whether we have to write on log",false);

		// Pan Tilt "Mission Package" properties
		session->createBool(PROPERTY_PANTILT_ENABLED, "PanTilt device enabled", true);
		session->createString(PROPERTY_PANTILT_MISSION_PACKAGE_NAME, "PanTilt mission package name", "CameraPanTilt");
		session->createDouble(PROPERTY_PANTILT_PAN, "PanTilt current pan position", RDouble::RAD, 0.0);
		session->setVolatile(PROPERTY_PANTILT_PAN);
		session->createDouble(PROPERTY_PANTILT_TILT, "PanTilt current tilt position", RDouble::RAD, 0.0);
		session->setVolatile(PROPERTY_PANTILT_TILT);
		session->createDouble(PROPERTY_PANTILT_DESIRED_PAN, "PanTilt desired pan", RDouble::RAD, 0.0);
		session->createDouble(PROPERTY_PANTILT_DESIRED_TILT, "PanTilt desired tilt", RDouble::RAD, 0.0);
		session->createVector<RItemOnMap>(PROPERTY_VIEW_PANTILT_DIRECTION, "PanTilt direction");
		session->createDouble(PROPERTY_VIEW_PANTILT_DIRECTION_FOV, "PanTilt direction FOV (only display purpose)",
			RDouble::RAD, deg2rad(45.));
		session->createDouble(PROPERTY_VIEW_PANTILT_DIRECTION_DIST, "PanTilt direction distance (only display purpose)",
			RDouble::M, 2.);

		// Victim Sensor properties
		session->createBool(PROPERTY_VICTIM_SENSOR_ENABLED, "Victim Sensor device enabled", true);
		session->createInt(PROPERTY_VICTIM_SENSOR_PARTS_SEEN, "Victim parts currently seen", 0);
		session->setVolatile(PROPERTY_VICTIM_SENSOR_PARTS_SEEN);
		session->createDouble(PROPERTY_VICTIM_SENSOR_PARTS_SEEN_INTERVAL, "Interval to compute victim parts currently seen",
			RDouble::SEC, 1.);
		
		// Victim RFID Sensor properties
		session->createBool(PROPERTY_VICTIM_RFID_SENSOR_ENABLED, "Victim RFID Sensor device enabled", true);
		session->createDouble(PROPERTY_VICTIM_RFID_SENSOR_FOV, "Victim RFID Sensor FOV (filter)", RDouble::RAD, 2 * M_PI);
		session->createDouble(PROPERTY_VICTIM_RFID_SENSOR_MAX_DIST, 
			"Victim RFID Sensor max distance (filter)", RDouble::M, 10000.);
		session->createBool(PROPERTY_VICTIM_RFID_SENSOR_USE_RELIABILITY, "Victim RFID Sensor use reliability", false);

		session->createVector<RItemOnMap>(PROPERTY_RFID_TAGS, "Rfid tags to draw");
		session->createVector<RItemOnMap>(PROPERTY_ITEMS, "Rfid victims to draw");
		session->createVector<RItemOnMap>(PROPERTY_CURRENT_VICTIM_VIEW, "Current victim");
		
		session->createBool(PROPERTY_COPY_ODOMETRY, "Copy odometry pose to estimated pose "
			"(to by-pass the scanmatcher process)", false);
		
		session->createString(PROPERTY_REMOTE_MAPPER, "Remote Mapper Name", "Jamaiko");
		session->createBool(PROPERTY_REMOTE_WRITE,"Remote writing",false);
		
		session->createDouble(PROPERTY_LASER_MIN_RANGE,"Min range for laser reading",RDouble::M,0.04);
		session->createDouble(PROPERTY_LASER_MAX_RANGE,"Max range for laser reading",RDouble::M,20.);

		session->createDouble(PROPERTY_VICTIM_X,"Victim x",RDouble::M,0.);
		session->createDouble(PROPERTY_VICTIM_Y,"Victim y",RDouble::M,0.);
		session->createDouble(PROPERTY_VICTIM_Z,"Victim z",RDouble::M,0.);

		session->createString(PROPERTY_VICTIM_NAME, "Victim name" ,"No_victim");

		session->createPose(PROPERTY_BALL_POSITION, "Ball position", Point2od(DBL_MAX, DBL_MAX, DBL_MAX));
		session->setVolatile(PROPERTY_BALL_POSITION);
		session->createBool(PROPERTY_BALL_VISIBLE, "Ball is visible", false, INFO);
		
		session->createBool(PROPERTY_RFID_WRITE_FILE, "Write RFID to file", false);	// FIXME READ_ONLY
/*		session->createStorage("RTouchSensorData", 
				       PROPERTY_TOUCH_SENSORS, 
				       "Touch sensors' data");*/
		
		
		//FIXME this queue is created only for the hokuyo lrf. It represents
		//a temporary solution, since THE queue has not being built thinking
		//about the possibility of two lrf's in the same robot...
		session->createQueue(PROPERTY_HOKUYO_DATA_QUEUE,"HOKUYO QUEUE"); 
		
//		session->createQueue(PROPERTY_RFID_QUEUE,"Rfid fake queue for vision map builder");
/*		session->createDouble(PROPERTY_RFID_DIST,"Victim Rfid perceive distance",RDouble::M,2.0);
		session->createInt(PROPERTY_RFID_NUMRAYS,"Victim Rfid ray number",360);*/
		// Tarantula-specific properties
		session->createDouble(PROPERTY_ROBOT_FRONT_FLIPPER_POSITION, "Front flipper angle", RDouble::RAD, 0.);
		session->createDouble(PROPERTY_ROBOT_REAR_FLIPPER_POSITION, "Rear flipper angle", RDouble::RAD, 0.);
		session->createDouble(PROPERTY_ROBOT_DESIRED_FRONT_FLIPPER_SPEED, "Front flipper desired speed", RDouble::RAD_SEC, 0.);
		session->createDouble(PROPERTY_ROBOT_DESIRED_REAR_FLIPPER_SPEED, "Rear flipper desired speed",  RDouble::RAD_SEC, 0.);
		session->createBool(PROPERTY_ROBOT_HAS_PWM_FLIPPERS, "Tarantula flippers are controlled by PWM", false);
		session->createBool(PROPERTY_ROBOT_HAS_PWM_TRACKS, "Tarantula tracks are controlled by PWM", false);
		session->createDouble(PROPERTY_ROBOT_MAX_FLIPPER_SPEED, "Maximum flipper speed", RDouble::RAD_SEC, M_PI / 4);
		session->createDouble(PROPERTY_ROBOT_DESIRED_LEFT_TRACK_SPEED, "Left track desired speed", RDouble::RAD_SEC, 0.);
		session->createDouble(PROPERTY_ROBOT_DESIRED_RIGHT_TRACK_SPEED, "Right track desired speed", RDouble::RAD_SEC, 0.);
		session->createBool(PROPERTY_ROBOT_CONTROL_TRACKS, "Robot is controlled through left and right track instead of speed and jog", false);

		// QuadRotor specific values
		session->createDouble(PROPERTY_ROBOT_ALTITUDE_VELOCITY,"QuadRotor altitude velocity", RDouble::M_SEC, 0.0);
		session->createDouble(PROPERTY_ROBOT_LINEAR_VELOCITY,"QuadRotor linear velocity", RDouble::M_SEC, 0.0); 
		session->createDouble(PROPERTY_ROBOT_LATERAL_VELOCITY,"QuadRotor lateral velocity", RDouble::M_SEC, 0.0);
		session->createDouble(PROPERTY_ROBOT_ROTATIONAL_VELOCITY,"QuadRotor rotational velocity", RDouble::RAD_SEC, 0.0);

		session->createPose3(PROPERTY_ROBOT_POSE3, "QuadRotor current 3d pose", Point3od(0.0,0.0,0.0, 0.0, 0.0, 0.0 ));  
		session->createPose(PROPERTY_ROBOT_POSE_2D, "Quadorotor current 2D pose", Point2od(0.0, 0.0, 0.0));
		
		session->createDouble(PROPERTY_ROBOT_CURRENT_X,"QuadRotor current x coordinate", RDouble::M, 0.0); 
		session->createDouble(PROPERTY_ROBOT_CURRENT_Y,"Quadrotor current y coordinate", RDouble::M, 0.0);
		session->createDouble(PROPERTY_ROBOT_CURRENT_Z,"Quadrotor current z coordinate", RDouble::M, 0.0);
		session->createDouble(PROPERTY_ROBOT_YAW,"Quadrotor current yaw coordinate", RDouble::RAD_SEC, 0.0);
		
		// Interleaved view
		session->createBool(PROPERTY_ROBOT_INTERLEAVED_VIEW_ENABLED, "Enable interleaved view",false);
		session->createBool(PROPERTY_ROBOT_INTERLEAVED_VIEW_SCREENSHOT_ENABLED, "Screenshot enabled (this bool is enabled when is the robot's turn)",false);
		session->createInt(PROPERTY_ROBOT_INTERLEAVED_VIEW_TURN,"View order (turn of this robot)",0);
		session->createInt(PROPERTY_ROBOT_INTERLEAVED_VIEW_NUMBER_OF_ROBOTS,"Number of robots sharing camera",2);
		session->createDouble(PROPERTY_ROBOT_INTERLEAVED_VIEW_TIME_INTERVAL,"Time interval to change camera",RDouble::REAL,1.0);
				
				
		// Inputs
		session->createInt(PROPERTY_RFID_ID_TO_READ, "ID of RFID to read", 0);
		session->createInt(PROPERTY_RFID_ID_TO_WRITE, "ID of RFID to write into", 0);
		session->createString(PROPERTY_RFID_DATA_TO_WRITE, "Data to write into RFID");
		// Outputs
		session->createBool(PROPERTY_LAST_RFID_DROP_SUCCESSFUL, "Last RFID drop was succesful", false, INFO);
		session->createVector<RUsarRfidData>(PROPERTY_RFID_DETECTED_IDS, "IDs of last detected RFIDs");
		session->createBool(PROPERTY_LAST_RFID_READ_SUCCESSFUL, "Last RFID reading was succesful", false, INFO);
		session->createString(PROPERTY_RFID_READING_DATA, "Data contained in last read RFID", "", INFO);
		session->createBool(PROPERTY_LAST_RFID_WRITE_SUCCESSFUL, "Last RFID writing was succesful", false, INFO);
		// Commands
		session->createBool(PROPERTY_CMD_DROP_RFID, "Drop RFID", false);
		session->setVolatile(PROPERTY_CMD_DROP_RFID);
		session->createBool(PROPERTY_CMD_READ_RFID, "Read RFID", false);
		session->setVolatile(PROPERTY_CMD_READ_RFID);
		session->createBool(PROPERTY_CMD_WRITE_RFID, "Write RFID", false);
		session->setVolatile(PROPERTY_CMD_WRITE_RFID);
		
		session->createStorage("RLaserData", PROPERTY_HOKUYO_LASER_DATA, "Second laser data");
		session->createString(PROPERTY_HOKUYO_SENSOR_NAME, "Hokuyo sensor name", "Scanner2");
		session->createStorage("RSonarData", PROPERTY_SONAR_DATA, "Sonar data");	
	
	SESSION_END(session)

	cmdSent = false;
	waitingForRFIDResult = false;

	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

bool UsarSimClientModule::initInterfaceProperties()
{
	string cfg;
	vector<string> v;
	SESSION_TRY_START(session)
		// Encoders
		cfg = session->getString(PROPERTY_ENCODERS_CONFIG);
		v = TextUtils::tokenize(cfg, " ");
		for (size_t i = 0; i < v.size(); i++) {
			session->createDouble(PROPERTY_ENCODERS_FOLDER + v[i] + PROPERTY_ENCODERS_POS_SUFFIX, "Encoder " + v[i], RDouble::RAD, 0.0);
		}
		// GPS sensor
		if (session->getBool(PROPERTY_GPS_ENABLED)) {
			session->createString(PROPERTY_GPS_NAME, "Name of the last GPS sensor acquired", "");
			session->createString(PROPERTY_GPS_LATITUDE, "Latitude", "");
			session->createString(PROPERTY_GPS_LONGITUDE, "Longitude", "");
			session->createBool(PROPERTY_GPS_FIX, "Indicates if GPS has acquired a position or not", false);
			session->createInt(PROPERTY_GPS_SATELLITES, "Number of satellites tracked by the GPS", 0);
			session->createPose(PROPERTY_GPS_POSE, "Pose computed from GPS data", Point2od(0.0, 0.0, 0.0));
		}
		// Ground truth pose
		if (session->getBool(PROPERTY_GROUND_TRUTH_POSE_ENABLED)) {
			session->createPose(PROPERTY_GROUND_TRUTH_POSE, "Ground truth pose", Point2od(0.0, 0.0, 0.0));
		}
		// INS sensor
		if (session->getBool(PROPERTY_INS_ENABLED)) {
			session->createDouble(PROPERTY_INS_X, "X from the INS", RDouble::M, 0.0);
			session->createDouble(PROPERTY_INS_Y, "Y from the INS", RDouble::M, 0.0);
			session->createDouble(PROPERTY_INS_Z, "Z from the INS", RDouble::M, 0.0);
			session->createDouble(PROPERTY_INS_ROLL, "Roll from the INS", RDouble::RAD, 0.0);
			session->createDouble(PROPERTY_INS_PITCH, "Pitch from the INS", RDouble::RAD, 0.0);
			session->createDouble(PROPERTY_INS_YAW, "Yaw from the INS", RDouble::RAD, 0.0);
		}
		// Touch sensors
		cfg = session->getString(PROPERTY_TOUCH_SENSORS_CONFIG);
		v = TextUtils::tokenize(cfg, " ");
		for (size_t i = 0; i < v.size(); i++) {
			session->createBool(PROPERTY_TOUCH_SENSORS_FOLDER + v[i], "Touch sensor " + v[i], false);
		}
		// Flipper controllers
		cfg = session->getString(PROPERTY_FLIPPER_CTRLS_CONFIG);
		v = TextUtils::tokenize(cfg, " ");
		for (size_t i = 0; i < v.size(); i++) {
			session->createDouble(PROPERTY_FLIPPER_CTRLS_FOLDER + v[i] + PROPERTY_FLIPPER_CTRLS_POS_SUFFIX,
				"Flipper " + v[i] + " position", RDouble::RAD, 0.0);
		}
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

bool UsarSimClientModule::init()
{
	SESSION_TRY_START(session)
		//trying to connect with UsarSim Srever
		connected = false;
		spawnedBot = false;
		connectionAttempts = 0;
		isComStation = false;
		if (session->getBool(PROPERTY_RFID_WRITE_FILE))
			rfidStream.open(("Rfid_"+session->getRepositoryName()).c_str());
		
		std::string robotClass = session->getString(PROPERTY_ROBOT_CLASS);
		if (robotClass.compare("USARBot.Zerg")==0){
			session->setDouble(PROPERTY_ROBOT_WIDTH,0.3);
			session->setDouble(PROPERTY_ROBOT_HEIGHT,0.3);
			session->setDouble(PROPERTY_EXPANSION_SIZE,0.15);
		} else if (robotClass.compare("USARBot.P2DX")==0){
			session->setDouble(PROPERTY_ROBOT_WIDTH,0.40);
			session->setDouble(PROPERTY_ROBOT_HEIGHT,0.50);
			session->setDouble(PROPERTY_EXPANSION_SIZE,0.25);
		} else if (robotClass.compare("USARBot.P2AT")==0){
			session->setDouble(PROPERTY_ROBOT_WIDTH,0.40);
			session->setDouble(PROPERTY_ROBOT_HEIGHT,0.50);
			session->setDouble(PROPERTY_EXPANSION_SIZE,0.25);
                } else if ((robotClass.compare("USARBot.Tarantula") == 0) ||
			   (robotClass.compare("USARBot.TarantulaSIED") == 0)) {
			session->setDouble(PROPERTY_ROBOT_WIDTH, 0.44);
			session->setDouble(PROPERTY_ROBOT_HEIGHT, 0.70);
			session->setDouble(PROPERTY_EXPANSION_SIZE, 0.); // XXX
		} else if ((robotClass.compare("USARBot.AirRobot") == 0)) {
			session->setDouble(PROPERTY_ROBOT_WIDTH, 0.999 );
			session->setDouble(PROPERTY_ROBOT_HEIGHT, 0.999);
			session->setDouble(PROPERTY_EXPANSION_SIZE, 0.25); 

		} else if (robotClass.find("ComStation") != string::npos) {
			session->setDouble(PROPERTY_ROBOT_WIDTH, 1.);
			session->setDouble(PROPERTY_ROBOT_HEIGHT, 1.);
			session->setDouble(PROPERTY_EXPANSION_SIZE, 0.);
			isComStation = true;
		}
		else {
			RDK_ERROR_STREAM("Cannot set measures for such robot= " << robotClass);
		}

                session->setString(PROPERTY_VICTIM_NAME,"No_victim");

		session->lock(PROPERTY_RFID_TAGS, HERE);
			RItemOnMapVector* v1 = session->getObjectAsL<RItemOnMapVector>(PROPERTY_RFID_TAGS);
			v1->clear();
		session->unlock(PROPERTY_RFID_TAGS);

		session->lock(PROPERTY_ITEMS, HERE);
			RItemOnMapVector* v2 = session->getObjectAsL<RItemOnMapVector>(PROPERTY_ITEMS);
			v2->clear();
		session->unlock(PROPERTY_ITEMS);

		session->setDouble(PROPERTY_LASER_MAX_RANGE,20.0);
		session->setDouble(PROPERTY_LASER_MIN_RANGE,0.04);
		
		while (!(spawnedBot && connected)) {
			if (connectionAttempts == USBOT_STARTUP_CONN_LIMIT){
				//We should send a message to the operator somehow
				RDK_ERROR_STREAM("Connection with UsarSim could not "
						 "be established for "<< 
						 USBOT_STARTUP_CONN_LIMIT << " times");
				return false;
			}
			if (us_connected()) {
				spawnedBot=spawnBot();				
			} else {
				us_connect();
				connectionAttempts++;
			}			
		}
		// here you can declare the events you are waiting in
		// the main exec() loop, for example:
		//session->listenToTimer(30.);	// FIXME: DC perché???
	SESSION_END(session)

	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}



void UsarSimClientModule::commandRobot()
{	
	// Get robot class
	std::string robotClass = session->getString(PROPERTY_ROBOT_CLASS);
	
	// Speed/jog control
	double lSpeed = 0., rSpeed = 0.;
	double cmdSpeed = session->getDouble(PROPERTY_CMD_SPEED);
	double cmdJog = session->getDouble(PROPERTY_CMD_JOG);
	computeWheelSpeed(session, cmdSpeed, cmdJog, lSpeed, rSpeed);
	if (lSpeed == 0.) {
		// setting speed to 0. has no effect in usarsim (che dio li fulmini tutti)
		double jdm = ((double) (rand() % 1000)) / 1000.;
		lSpeed = jdm * 0.001 - 0.0005;
	}
	if (rSpeed == 0.) {
		// setting speed to 0. has no effect in usarsim (che dio li fulmini tutti)
		double jdm = ((double) (rand() % 1000)) / 1000.;
		rSpeed = jdm * 0.001 - 0.0005;
	}
	ostringstream oss;
	oss << "DRIVE {Left " << lSpeed << "} {Right " << rSpeed << "}\r\n";
	if (addCommand(oss.str().c_str()) < 0) RDK_ERROR_STREAM("Can't send commands speed/jog");

	// Flipper controllers
	if (session->getString(PROPERTY_FLIPPER_CTRLS_CONFIG) != "") {
		vector<string> v = TextUtils::tokenize(session->getString(PROPERTY_FLIPPER_CTRLS_CONFIG));
		ostringstream oss;
		oss << "MULTIDRIVE ";
		for (size_t i = 0; i < v.size(); i++) {
			static TimerR tmr;
			map<string, double>::iterator itflipperpos = cmdFlipperPositions.find(v[i]);
			double oldPos = 0.0;
			if (itflipperpos == cmdFlipperPositions.end()) cmdFlipperPositions.insert(make_pair(v[i], oldPos));
			else oldPos = itflipperpos->second;
			double newPos = session->getDouble(PROPERTY_FLIPPER_CTRLS_FOLDER + v[i] + PROPERTY_FLIPPER_CTRLS_POS_SUFFIX);
			double ad = newPos - oldPos, bd = angDiff(newPos, oldPos);
			if (fabs(ad) > fabs(bd)) newPos = oldPos + bd;	// "smart" positioning: go to position using the shortest angle
			oss << "{" << v[i] << " " << newPos << "} ";
			cmdFlipperPositions[v[i]] = newPos;
		}
		oss << "\r\n";
		if (addCommand(oss.str().c_str()) < 0) RDK_ERROR_STREAM("Can't send commands to flippers");
	}
}

// comando principale
void UsarSimClientModule::exec()
{	
	bool cameraChanged = false;
	while (session->dontWait(), !exiting) {
		SESSION_TRY_START(session);
		
		// Read data from USARSim
		if (!isComStation) parseUsarData();
		else sleep(1);
		// Check if we have commands to execute
		if ((session->getBool(PROPERTY_CMD_DROP_RFID)) && !waitingForRFIDResult)
			dropRFID();
		else if (session->getBool(PROPERTY_CMD_READ_RFID) && !waitingForRFIDResult)
			readRFID();
		else if (session->getBool(PROPERTY_CMD_WRITE_RFID) && !waitingForRFIDResult)
			writeRFID();
		if(session->getBool(PROPERTY_ROBOT_INTERLEAVED_VIEW_ENABLED))
			changeCamera(&cameraChanged);		
		commandRobot();	  // command to robot
		refreshPanTiltDirection(session);
		updateCurrentVictimPartsSeen(session);
		
		SESSION_END_CATCH_TERMINATE(session)
	}
}

void UsarSimClientModule::updateCurrentVictimPartsSeen(Session* session) throw (SessionException)
{
	if (!session->getBool(PROPERTY_VICTIM_SENSOR_ENABLED)) {
		session->setInt(PROPERTY_VICTIM_SENSOR_PARTS_SEEN, 0);
		curVictimPartsSeen.clear();
		return;
	}
	if (curVictimPartsSeenTimer.getSeconds() > session->getDouble(PROPERTY_VICTIM_SENSOR_PARTS_SEEN_INTERVAL)) {
		session->setInt(PROPERTY_VICTIM_SENSOR_PARTS_SEEN, curVictimPartsSeen.size());
		curVictimPartsSeen.clear();
		curVictimPartsSeenTimer.start();
	}
}

void UsarSimClientModule::refreshPanTiltDirection(Session* session) throw (SessionException)
{
	double fov = session->getDouble(PROPERTY_VIEW_PANTILT_DIRECTION_FOV);
	double dist = session->getDouble(PROPERTY_VIEW_PANTILT_DIRECTION_DIST);
	double pan = session->getDouble(PROPERTY_PANTILT_PAN);
	Point2od robotPose = session->getPose(PROPERTY_ROBOT_ESTIMATED_POSE);
	session->lock(PROPERTY_VIEW_PANTILT_DIRECTION, HERE);
	Vector<RItemOnMap>* v = session->getObjectAsL<Vector<RItemOnMap> >(PROPERTY_VIEW_PANTILT_DIRECTION);
	v->clear();
	v->push_back(new RLine2dOnMap(robotPose.x, robotPose.y,
		cos(robotPose.theta + pan - fov/2) * dist + robotPose.x, sin(robotPose.theta + pan - fov/2) * dist + robotPose.y,
		RGB_ORANGE, 2.));
	v->push_back(new RLine2dOnMap(robotPose.x, robotPose.y,
		cos(robotPose.theta + pan + fov/2) * dist + robotPose.x, sin(robotPose.theta + pan + fov/2) * dist + robotPose.y,
		RGB_ORANGE, 2.));
	session->unlock(PROPERTY_VIEW_PANTILT_DIRECTION);
	session->valueChanged(PROPERTY_VIEW_PANTILT_DIRECTION);
}

int UsarSimClientModule::addCommand(const char* command) {
	#ifdef MACOSX
	if (!::send(m_sock,command,strlen(command), SO_NOSIGPIPE)) {
		// RDK_ERROR_STREAM("write() "<<command);
		return false;
	}
	#elif LINUX
	if (!::send(m_sock,command,strlen(command), MSG_NOSIGNAL)) {
		// RDK_ERROR_STREAM("write() "<<command);
		return false;
	}
	#elif CYGWIN
	if (!::send(m_sock,command,strlen(command), MSG_NOSIGNAL)) {
		return false;
	}
	#else
	#warning Platform not implemented
	#endif
	// RDK_DEBUG_STREAM("write() "<<command);
	return true;
}

void UsarSimClientModule::dropRFID() {
	ostringstream cmdToSend;
	// This check is needed because this method can be called
	// asynchronously
	if (!waitingForRFIDResult) {
		cmdToSend << "SET {Type RFIDReleaser} {Name Gun} " <<
			"{Opcode Release}";
		if (addCommand((char *)cmdToSend.str().c_str()) < 0) {
			RDK_ERROR_STREAM("RFID release command not sent");
		} else { // Command sent succesfully
			waitingForRFIDResult = true;
			session->setBool(PROPERTY_CMD_DROP_RFID, true);
		}
	} else RDK_INFO_STREAM("Can't drop an RFID now! Please try later.");
}

void UsarSimClientModule::readRFID() {
	ostringstream cmdToSend;
	// This check is needed because this method can be called
	// asynchronously
	if (!waitingForRFIDResult) {
		cmdToSend << "SET {Type RFID} {Name RFID} " <<
			"{Opcode Read} {Params " << 
			session->getInt(PROPERTY_RFID_ID_TO_READ) << "}";
		// RDK_DEBUG_STREAM(cmdToSend.str());
		if (addCommand((char *)cmdToSend.str().c_str()) < 0) {
			RDK_ERROR_STREAM("RFID read command not sent");
		} else { // Command sent succesfully
			waitingForRFIDResult = true;
			session->setBool(PROPERTY_CMD_READ_RFID, true);
		}
	} else RDK_INFO_STREAM("Can't read an RFID now! Please try later.");
}

void UsarSimClientModule::writeRFID() {
	ostringstream cmdToSend;
	// This check is needed because this method can be called
	// asynchronously
	if (!waitingForRFIDResult) {
		cmdToSend << "SET {Type RFID} {Name RFID} " <<
			"{Opcode Write} {Params " << 
			session->getInt(PROPERTY_RFID_ID_TO_WRITE) << " " <<
			session->getString(PROPERTY_RFID_DATA_TO_WRITE) <<
			"}";
		// RDK_DEBUG_STREAM(cmdToSend.str());
		if (addCommand((char *)cmdToSend.str().c_str()) < 0) {
			RDK_ERROR_STREAM("RFID write command not sent");
		} else { // Command sent succesfully
			waitingForRFIDResult = true;
			session->setBool(PROPERTY_CMD_WRITE_RFID, true);
		}
	} else RDK_INFO_STREAM("Can't write to a RFID now! Please try later.");
}


void UsarSimClientModule::cleanup() {
#ifdef ENABLE_LOG
	log.close();        
	logApprendimento.close();
#endif
	SESSION_TRY_START(session)
	if (session->getBool(PROPERTY_RFID_WRITE_FILE)) {
		rfidStream << "END" << std::endl;
		rfidStream.close();
	}
	SESSION_END_CATCH_TERMINATE(session)
}

// TODO: implement commands: readRFID, writeRFID
void UsarSimClientModule::asyncAgentCmd(cstr command)
{
	//float arg;
	float spqrSpeed = 0., spqrJog = 0.;
	double spqrFFlip = 0., spqrRFlip = 0.;
	bool updateSpeedJog = false;
	//bool usePantilt = false;
	bool pantiltEnabled = false;
	bool useFlippers = false;
	char cmdBuff[USBOT_MAX_CMD_LEN];
	std::string botName, botHost, imageHost;
	int cameraNum;
        istringstream ss(command);
        ostringstream cmdToSend;
        string cmd;

        // Parse the first word, and mark it if it's recognized
        ss >> cmd;

	cmdBuff[0] = 0;
        // RDK_INFO_STREAM("Received command: \"" << cmd.c_str() << "\"")

	SESSION_TRY_START(asyncSession);
	// Only tarantula has flippers
	std::string robotClass = asyncSession->getString(PROPERTY_ROBOT_CLASS);
	useFlippers = ((robotClass.compare("USARBot.Tarantula") == 0) ||
		       (robotClass.compare("USARBot.TarantulaSIED") == 0));
	//usePantilt = asyncSession->getBool(PROPERTY_USE_DEVICE_PANTILT);
	pantiltEnabled = asyncSession->getBool(PROPERTY_PANTILT_ENABLED);
	botName = asyncSession->getRepositoryName();
	botHost = asyncSession->getString(PROPERTY_USARSIM_ADDRESS);
	imageHost = asyncSession->getString(PROPERTY_IMAGESERVER_ADDRESS);

	RDK_DEBUG_PRINTF("%s", cmd.c_str());       
 
	if (cmd == "setSpeed") {
		ss >> spqrSpeed;
		RDK_INFO_STREAM("speedControl..." << spqrSpeed);
		updateSpeedJog = true;
	}
	else if (cmd == "setJog") {
		ss >> spqrJog;
		// RDK_INFO_STREAM("jogControl..." << spqrJog);
		updateSpeedJog = true;
	}
	else if (cmd == "setSpeedAndJog") {
		ss >> spqrSpeed >> spqrJog;
		// RDK_INFO_PRINTF("speedAndJogControl... %f %f", spqrSpeed, spqrJog);
		updateSpeedJog = true;
	}
	else if (cmd == "reset") {
		RDK_ERROR_STREAM("Speed Jog Reset... DEPRECATED!");
		spqrSpeed = 0.;
		spqrJog = 0.;
		updateSpeedJog = true;
	} else if (cmd == "setCamera") {
		ss >> cameraNum;
		snprintf(cmdBuff, sizeof(cmdBuff), "SET {Type Camera}{Robot %s}{Name Camera}{Client %s}\r\n",botName.c_str(), imageHost.c_str());
		RDK_INFO_PRINTF("[i] setCamera: %s", cmdBuff);
		if (addCommand(cmdBuff)<0)
			RDK_ERROR_STREAM("Robot camera not set");
	}
	/***************************/
	/* PANTILT MISSION PACKAGE */
	/***************************/
	else if (pantiltEnabled && cmd == "setPan") {
		double desiredPan;
		ss >> desiredPan;
		asyncSession->setDouble(PROPERTY_PANTILT_DESIRED_PAN, desiredPan);
	}
	else if (pantiltEnabled && cmd == "setTilt") {
		double desiredTilt;
		ss >> desiredTilt;
		asyncSession->setDouble(PROPERTY_PANTILT_DESIRED_TILT, desiredTilt);
	}
	else if (pantiltEnabled && cmd == "setPanAndTilt") {
		double desiredPan, desiredTilt;
		ss >> desiredPan >> desiredTilt;
		asyncSession->setDouble(PROPERTY_PANTILT_DESIRED_PAN, desiredPan);
		asyncSession->setDouble(PROPERTY_PANTILT_DESIRED_TILT, desiredTilt);
	}
#if 0
	} else if (usePantilt && (cmd == "setPan")) {
		ss >> spqrPan;
		// RDK_INFO_STREAM("panControl..." << spqrPan);
		snprintf(cmdBuff, sizeof(cmdBuff), "MISPKG {Name Cam1}{Rotation 0,0,%f}\r\n",spqrPan);
		if (addCommand(cmdBuff)<0) {
			RDK_ERROR_STREAM("PanTilt Pan setting not send");
		} else panTiltUpdate[PAN] = false;
	} else if (usePantilt && (cmd == "setTilt")) {
		ss >> spqrTilt;
		// RDK_INFO_STREAM("tiltControl..." << spqrTilt);
		snprintf(cmdBuff, sizeof(cmdBuff), "MISPKG {Name Cam1}{Rotation 0,%f,0}\r\n",spqrTilt);
		if (addCommand(cmdBuff)<0) {
			RDK_ERROR_STREAM("PanTilt Tilt setting not send");
		} else panTiltUpdate[PAN] = false;
	}
	//NOTE: This command actually does nothing because angles are relative.
	//      Actually it stops camera movement, but the camera itself is
	//      not reset in its initial position. (NdZaratti)
	else if(usePantilt && (cmd == "panTiltReset")) {
		RDK_INFO_STREAM("Pan Tilt Reset...");
		spqrPan = 0.;
		spqrTilt = 0.;
		snprintf(cmdBuff, sizeof(cmdBuff), "MISPKG {Name Cam1}{Rotation 0,0,0}\r\n");
		if (addCommand(cmdBuff)<0) {
			RDK_ERROR_STREAM("PanTilt Reset setting not send");
		} else panTiltUpdate[PAN] = false;
#endif
	else if (useFlippers && (cmd == "setFrontFlip")) {
		ss >> spqrFFlip;
		//saturateNonPWMFlippers(spqrFFlip);
		// RDK_INFO_STREAM("setFrontFlip..." << spqrFFlip); 
		// Sadly, USARSim allows us only to drive one flipper a
		// time. Therefore, we need to send two commands.	
		asyncSession->setDouble(PROPERTY_ROBOT_DESIRED_FRONT_FLIPPER_SPEED, 
					spqrFFlip);
		cmdToSend << "DRIVE {Name FLFlipper}{Order 1}{Value " 
			  << spqrFFlip << "}\r\n";
		cmdToSend << "DRIVE {Name FRFlipper}{Order 1}{Value " 
			  << spqrFFlip << "}\r\n";
		if (addCommand((char *)cmdToSend.str().c_str())<0)
			RDK_ERROR_STREAM("Front flipper setting not sent");
        } else if (useFlippers && (cmd == "setRearFlip")) {
		ss >> spqrRFlip;
		//saturateNonPWMFlippers(spqrRFlip);
		asyncSession->setDouble(PROPERTY_ROBOT_DESIRED_REAR_FLIPPER_SPEED, 
					spqrRFlip);
		// RDK_INFO_STREAM("setRearFlip..." << spqrRFlip);
		cmdToSend << "DRIVE {Name RLFlipper}{Order 1}{Value " 
			  << spqrRFlip << "}\r\n";
		cmdToSend << "DRIVE {Name RRFlipper}{Order 1}{Value " 
			  << spqrRFlip << "}\r\n";
		if (addCommand((char *)cmdToSend.str().c_str())<0)
			RDK_ERROR_STREAM("Rear flipper setting not sent")
        } else if (cmd == "dropRFID") {
		dropRFID();
	}
	if (updateSpeedJog) {

		asyncSession->setDouble(PROPERTY_ROBOT_DESIRED_SPEED,spqrSpeed);
		asyncSession->setDouble(PROPERTY_ROBOT_DESIRED_JOG,spqrJog);
// 		RDK_INFO_STREAM("Desired speed: " << spqrSpeed << " - jog: " <<
// 				spqrJog);

//		double lSpeed = (spqrSpeed / 8.75) - (RDK2::Geometry::rad2deg(spqrJog) / 1.52);
//		double rSpeed = (spqrSpeed / 8.75) + (RDK2::Geometry::rad2deg(spqrJog) / 1.52);
// 		mi ruba 0.25 m/s ogni 1 m/s
// 		double lSpeed = (spqrSpeed / 7.0f) - (spqrJog / 1.52f);
// 		double rSpeed = (spqrSpeed / 7.0f) + (spqrJog / 1.52f);

//right now spqrSpeed is in mm/sec we want it in m/sec
//		spqrSpeed /=1000;

		double lSpeed, rSpeed;
		computeWheelSpeed(asyncSession, spqrSpeed, spqrJog, lSpeed, rSpeed);

		//RDK_INFO_STREAM("linearSpeed" << spqrSpeed << " Jog"<<spqrJog << " => rspeed = " << rSpeed << " lspeed = " << lSpeed);


		char cmd[USBOT_MAX_CMD_LEN];
		cmd[0] = 0;
		snprintf(cmd, sizeof(cmd), "DRIVE {Left %f} {Right %f}\r\n",
			 lSpeed, rSpeed);
		if (session->getBool(PROPERTY_ROBOT_CONTROL_TRACKS)) {
			// We have to set also the left & right track
			// desired speed 
			session->setDouble(PROPERTY_ROBOT_DESIRED_LEFT_TRACK_SPEED,
					   lSpeed);
			session->setDouble(PROPERTY_ROBOT_DESIRED_RIGHT_TRACK_SPEED,
					   rSpeed);
		}
		// RDK_INFO_STREAM("Sent command: " << cmd);
		if (addCommand(cmd)<0)
			RDK_ERROR_STREAM("ConfString not send");
	}
	SESSION_END_CATCH_TERMINATE(asyncSession);
}

void UsarSimClientModule::changeCamera(bool* cameraChanged) {
	
	ostringstream cmd;
	int myTurn = session->getInt(PROPERTY_ROBOT_INTERLEAVED_VIEW_TURN);
	double interval = session->getDouble(PROPERTY_ROBOT_INTERLEAVED_VIEW_TIME_INTERVAL);
	int robots = session->getInt(PROPERTY_ROBOT_INTERLEAVED_VIEW_NUMBER_OF_ROBOTS);
	double time = session->getDouble(PROPERTY_USARSIM_TIME);
	
	double period = robots * interval;
	int currentTurn = (int) (fmod(time, period)/interval);
	if(currentTurn == myTurn) {
	
		if(*cameraChanged == false) {
			
			// It's my turn
			*cameraChanged = true;
					
			// Detach camera first
			cmd << "SET {Type Camera} {Client " << (session->getString(PROPERTY_USARSIM_ADDRESS)).c_str() << "}\r\n";
			if (!addCommand((char *)cmd.str().c_str()))
				RDK_ERROR_PRINTF("Failed detaching camera");						
			
			// Change camera now
			cmd.str("");
			cmd << "SET {Type Camera} {Robot " << session->getRepositoryName() << "} {Name Camera} {Client " << (session->getString(PROPERTY_USARSIM_ADDRESS)).c_str() << "}\r\n";
			if (!addCommand((char *)cmd.str().c_str()))
				RDK_ERROR_PRINTF("Failed changing camera");
			
			// Enable screenshot for this robot	
			session->setBool(PROPERTY_ROBOT_INTERLEAVED_VIEW_SCREENSHOT_ENABLED,true);			
		}
	
	} else {
		
		*cameraChanged = false;
		session->setBool(PROPERTY_ROBOT_INTERLEAVED_VIEW_SCREENSHOT_ENABLED,false);
	}	
	
}

bool UsarSimClientModule::us_connected(){
/*//	struct timeval tv;
	fd_set readfds;

//	tv.tv_sec = 2;
//	tv.tv_usec = 500000;

	FD_ZERO(&exceptfds);
	FD_SET(m_sock, &exceptfds);

	select(STDIN+1, NULL, NULL, exceptfds, NULL);

	if (FD_ISSET(m_sock, &exceptfds))
		printf("exception in connection!\n");
	else
		printf("Timed out.\n");*/

	return connected;

}


bool UsarSimClientModule::us_connect() throw(SessionException) {
	static struct sockaddr_in server; // connectors address information
	struct hostent* entp;

	const char * host = (session->getString(PROPERTY_USARSIM_ADDRESS)).c_str();
	int port = session->getInt(PROPERTY_USARSIM_PORT);

	RDK_INFO_PRINTF("Connection to Gamebots server (%s:%d)...", host, port);
	fflush(stdout);

	server.sin_family = PF_INET; // host byte order
	if((entp = gethostbyname(host)) == NULL) {
		RDK_ERROR_PRINTF("Setup(): \"%s\" is unknown host; can't connect to gamebots\n", host);
		return false;
	}
	memcpy(&server.sin_addr, entp->h_addr_list[0], entp->h_length);
	server.sin_port = htons(port);	// short, network byte order

	if((m_sock = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
		RDK_ERROR_STREAM("socket creation failed");
		return false;
	}
	int conn = connect(m_sock, (struct sockaddr*)&server, sizeof(server));
	if (conn == 0){
		connected = true;
		RDK_INFO_STREAM("connection with usarSim server established.");
	}
	else {
		connected = false;
		RDK_ERROR_STREAM("could not esablish connection " << conn);
	}

/*	if(fcntl(m_sock,F_SETFL,O_NONBLOCK) < 0) {	  // make it nonblocking
		RDK_ERROR_STREAM("fcntl(2) failed");
		return false;
	}*/
	return connected;
}

string substring(string s, size_t start, size_t end)
{
        return s.substr(start, end-start);
}

double robotClassDeltaZeta(const string& robotClass)
{
	// NOTE: values retrieved from http://digilander.libero.it/windflow/
	double startZ = 0.;
	if (robotClass == "USARBot.P2AT" || robotClass == "USARBot.StereoP2AT" || 
	    robotClass == "USARBot.P2AT_ATLANTA") startZ += -0.07;
	else if (robotClass == "USARBot.P2DX") startZ += -0.01;
	else if (robotClass == "USARBot.Zerg") startZ += 0.13;
	else if (robotClass == "USARBot.Talon") startZ += 0.06;
	else if (robotClass == "USARBot.ATRVjr") startZ += -0.20;
	else if (robotClass == "USARBot.ComStation") startZ += 0.;
	else if (robotClass == "USARBot.AirRobot") startZ += 0.0;
	else {
		RDK_ERROR_PRINTF("Unknown bot class (%s) for delta Z, refer to " 
			"http://digilander.libero.it/windflow/ to add the proper delta Z "
			"(or ask Marco)", robotClass.c_str());
	}
	return startZ;
}

bool UsarSimClientModule::spawnBot()
{
	string startPoseName = session->getString(PROPERTY_START_POSE_NAME);
	int startPoseNameIndex = 0;
	string robotClass = session->getString(PROPERTY_ROBOT_CLASS);
	string botName = session->getRepositoryName();
	
	double rdkStartX = 0., rdkStartY = 0., rdkStartZ = 0., rdkStartRoll = 0., rdkStartPitch = 0., rdkStartYaw = 0.;
	double usStartX = 0.,  usStartY = 0.,  usStartZ = 0.,  usStartRoll = 0.,  usStartPitch = 0.,  usStartYaw = 0.;
	
	if (startPoseName == "") {
		rdkStartX = session->getDouble(PROPERTY_START_X);
		rdkStartY = session->getDouble(PROPERTY_START_Y);
		rdkStartZ = session->getDouble(PROPERTY_START_Z);
		rdkStartRoll = session->getDouble(PROPERTY_START_ROLL);
		rdkStartPitch = session->getDouble(PROPERTY_START_PITCH);
		rdkStartYaw = session->getDouble(PROPERTY_START_YAW);
		usStartX = rdkStartX; 			usStartY = -rdkStartY; 				usStartZ = -rdkStartZ;
		usStartRoll = rdkStartRoll; 	usStartPitch = -rdkStartPitch; 		usStartYaw = -rdkStartYaw;
	}
	else {
		addCommand("GETSTARTPOSES\r\n");
		string ln;
		int r = readDataFromServer(m_sock, ln);
		r = r;
		//if (r)
		//{
		//  // please do something with r
		//}
		if (ln.substr(0, 3) != "NFO") return false;
		ln = ln.substr(4);
		string a = substring(ln, ln.find_first_of("{")+1, ln.find_first_of("}"));
		if (a.substr(0, 10) != "StartPoses") return false;
		int b = atoi(a.substr(11).c_str());
		ln = ln.substr(a.size()+2);
		a = substring(ln, ln.find_first_of("{")+1, ln.find_first_of("}"));
		istringstream iss(a);
		bool startPoseFound = false;
		int startPoseIndex = 0;
		if (startPoseName.find(",")) {
			startPoseNameIndex = atoi(startPoseName.substr(startPoseName.find(",") + 1).c_str());
			startPoseName = startPoseName.substr(0, startPoseName.find(","));
		}
		for (size_t i = 0; i < (uint) b; i++) {
			string name, position, orientation;
			iss >> name >> position >> orientation;
			if (name == startPoseName) {
				if (startPoseIndex == startPoseNameIndex) {
					vector<string> v = tokenize(position, ",");
					usStartX = atof(v[0].c_str());
					usStartY = atof(v[1].c_str());
					usStartZ = atof(v[2].c_str());
					usStartZ += robotClassDeltaZeta(robotClass);
					v = tokenize(orientation, ",");
					usStartRoll = atof(v[1].c_str());
					usStartPitch = atof(v[0].c_str());
					usStartYaw = atof(v[2].c_str());
					startPoseFound = true;
					rdkStartX = usStartX; 			rdkStartY = -usStartY; 				rdkStartZ = -usStartZ;
					rdkStartRoll = usStartRoll; 	rdkStartPitch = -usStartPitch; 		rdkStartYaw = -usStartYaw;
					break;
				}
				else startPoseIndex++;
			}
		}
		if (!startPoseFound) {
			RDK_ERROR_PRINTF("Cannot find start pose named '%s' (index %d) in this map",
				startPoseName.c_str(), startPoseNameIndex);
			return false;
		}
	}

	RDK_INFO_PRINTF("Spawning bot '%s', class '%s'", botName.c_str(), robotClass.c_str());
	if (startPoseName != "") RDK_INFO_PRINTF("Start pose name: %s,%d", startPoseName.c_str(), startPoseNameIndex);
	RDK_INFO_PRINTF("RDK Location: %.2f %.2f yaw = %.2f (z = %.2f, pitch = %.2f, roll = %.2f)",
		rdkStartX, rdkStartY, rdkStartYaw, rdkStartZ, rdkStartPitch, rdkStartRoll);
	RDK_INFO_PRINTF("USARSim Location: %.2f %.2f %.2f (z = %.2f, pitch = %.2f, roll = %.2f)",
		usStartX, usStartY, usStartYaw, usStartZ, usStartPitch, usStartRoll);
	
	ostringstream cmd;
	
	cmd << "INIT {ClassName " << robotClass.c_str() << "}"
		<< "{Name " << botName.c_str() << "}"
		<< "{Location " << usStartX << "," << usStartY << "," << usStartZ << "}"
		<< "{Rotation " << usStartRoll << "," << usStartPitch << "," << usStartYaw << "}\r\n";

	if (!addCommand((char *)cmd.str().c_str())) {
		RDK_ERROR_STREAM("Failed spawning bot");
		return false;
	}

	Point2od initialPose2D(rdkStartX, rdkStartY, rdkStartYaw);
	Point3od initialPose3D(rdkStartX, rdkStartY, rdkStartZ, rdkStartYaw, rdkStartPitch, rdkStartRoll);

	session->setPose(PROPERTY_INITIAL_POSE_2D, initialPose2D);
	session->setPose3(PROPERTY_INITIAL_POSE_3D, initialPose3D);

	return true;
}

void UsarSimClientModule::exitRequested()
{	
	ostringstream cmd;
	// Detach camera before exit
	cmd << "SET {Type Camera} {Client " << (session->getString(PROPERTY_USARSIM_ADDRESS)).c_str() << "}\r\n";
	if (!addCommand((char *)cmd.str().c_str()))
		RDK_ERROR_PRINTF("Failed detaching camera");		
		
	int a;
	if (connected) {
		a = shutdown(m_sock, SHUT_RDWR);
		if (a == -1) {
			RDK_ERROR_PRINTF("Unable to shut socket down: %s", 
					 strerror(errno));
		}
		a = close(m_sock);
		if (a == -1) {
			RDK_ERROR_PRINTF("Unable to close socket: %s", 
					 strerror(errno));
		}
	}
}


void UsarSimClientModule::logVictim(RMaps::RItemOnMapVector* items){
	victimStream.open(("Victim_"+session->getRepositoryName()).c_str() );		
	if (!victimStream.good()){
		RDK_DEBUG_STREAM("FILE NOT GOOD******************");
	}
        RItemOnMapVector& itemsVector = *items;
	for(RItemOnMapVector::iterator it =itemsVector.begin(); it!=itemsVector.end(); it++){
		RVictimOnMap* cvom = dynamic_cast<RVictimOnMap*>(*it);
		if(cvom){				
			victimStream << cvom->idAI << ", " << cvom->pose.x
						   << ", " << cvom->pose.y
						   << ", " << cvom->zPosition
						   << ", " << cvom->statusAI << std::endl;
		}
	}
	victimStream << "END" << std::endl;
	victimStream.close();
}

MODULE_FACTORY(UsarSimClientModule);

}} // namespace
