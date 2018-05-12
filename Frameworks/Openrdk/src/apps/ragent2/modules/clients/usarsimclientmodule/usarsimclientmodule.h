/**
 * This file describes the UsarSimClientModule class
 */

/** Some hints for when you develop in this module
1) always normalize angles, both in sensor readings and in controllers (usually, use angNormPiSig)
*/

#ifndef RDK2_MODULE_USARSIMCLIENTMODULE
#define RDK2_MODULE_USARSIMCLIENTMODULE

#define MODULE_NAME "UsarSimClientModule"

#include <rdkcore/modules/module.h>
#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/rsensordata/rodometrydata.h>
#include <rdkcore/rsensordata/rlaserdata.h>
#include <rdkcore/rmaps/ritemonmapvector.h>
#include <rdkcore/rmaps/rfidtagonmap.h>
#include <rdkcore/rmaps/rvictimonmap.h>
#include <rdkcore/rsensordata/rusarvictimrfiddata.h>
#include <rdkcore/rsensordata/rusarrfiddata.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE MODULE_NAME

#include <map>
#include <fstream>

/** NEW DEFINES */

// Global parameters
#define PROPERTY_ROBOT_CLASS         "params/robotClass"
#define PROPERTY_USARSIM_ADDRESS     "params/usarsimAddress"
#define PROPERTY_USARSIM_PORT        "params/usarsimPort"
#define PROPERTY_IMAGESERVER_ADDRESS "params/imageServerAddress"

// Start pose
#define PROPERTY_START_POSE_NAME "params/startPose/name"
#define PROPERTY_START_X         "params/startPose/x"
#define PROPERTY_START_Y         "params/startPose/y"
#define PROPERTY_START_Z         "params/startPose/z"
#define PROPERTY_START_PITCH     "params/startPose/pitch"
#define PROPERTY_START_ROLL      "params/startPose/roll"
#define PROPERTY_START_YAW       "params/startPose/yaw"

// Information and status
#define PROPERTY_INITIAL_POSE_2D  "info/initialPose2D"
#define PROPERTY_INITIAL_POSE_3D  "info/initialPose3D"
#define PROPERTY_VEHICLE_TYPE	  "info/vehicleType"
#define PROPERTY_MULTIVIEW_NUMBER "status/multiViewNumber"
#define PROPERTY_BATTERY_LIFE	  "status/batteryLife"
#define PROPERTY_USARSIM_TIME     "status/usarsimTime"

// Encoders (configurable: <name1> <name2> ...)
#define PROPERTY_ENCODERS_CONFIG          "params/encodersConfig"
#define PROPERTY_ENCODERS_FOLDER          "sensors/encoders/"
#define PROPERTY_ENCODERS_POS_SUFFIX      "Pos"

// GPS sensor (not configurable)
#define PROPERTY_GPS_ENABLED    "params/gpsEnabled"
#define PROPERTY_GPS_NAME       "sensors/gps/name"
#define PROPERTY_GPS_LATITUDE   "sensors/gps/latitude"
#define PROPERTY_GPS_LONGITUDE  "sensors/gps/longitude"
#define PROPERTY_GPS_FIX        "sensors/gps/fix"
#define PROPERTY_GPS_SATELLITES "sensors/gps/satellites"
#define PROPERTY_GPS_POSE       "sensors/gps/pose"

// Ground Truth sensor (not configurable)
#define PROPERTY_GROUND_TRUTH_POSE_ENABLED "params/groundTruthPoseEnabled"
#define PROPERTY_GROUND_TRUTH_POSE         "sensors/groundTruthPose"

// INS sensor (not configurable)
#define PROPERTY_INS_ENABLED "params/insEnabled"
#define PROPERTY_INS_ROLL    "sensors/ins/roll"
#define PROPERTY_INS_PITCH   "sensors/ins/pitch"
#define PROPERTY_INS_YAW     "sensors/ins/yaw"
#define PROPERTY_INS_X       "sensors/ins/x"
#define PROPERTY_INS_Y       "sensors/ins/y"
#define PROPERTY_INS_Z       "sensors/ins/z"

// Touch sensor (configurable: <name1> <name2> <name3> ...)
#define PROPERTY_TOUCH_SENSORS_CONFIG "params/touchSensorsConfig"
#define PROPERTY_TOUCH_SENSORS_FOLDER "sensors/touches/"

// Speed/jog control
#define PROPERTY_CMD_SPEED "in/cmdSpeed"
#define PROPERTY_CMD_JOG "in/cmdJog"

// Flipper controllers (configurable: <name1> <name2> ...)
#define PROPERTY_FLIPPER_CTRLS_CONFIG     "params/flipperCtrlsConfig"
#define PROPERTY_FLIPPER_CTRLS_FOLDER     "ctrls/flippers/"
#define PROPERTY_FLIPPER_CTRLS_POS_SUFFIX "Pos"

/** OLD AND DEPRECATED DEFINES */

/// @name Robot sizes 
//@{
// FIXME: this should be more elegantly embedded into robot class
// definition
// FIXME DC: diameter means distance between wheels, i.e. width of the robot
// DC: if you look for "diameter" in a dictionary, you will find it has a different meaning
#define P2AT_DIAMETER 0.49 
#define P2AT_WHEEL_RADIUS 0.11 

// FIXME: not real values, but used just to shut up warnings
#define P2DX_DIAMETER 0.49 
#define P2DX_WHEEL_RADIUS 0.11 

#define ZERG_WHEEL_DISTANCE 0.4153
#define ZERG_WHEEL_RADIUS 0.06

#define TARANTULA_LENGTH 0.44
// Tarantula's trackers are made out of wheels. We work with their
// dimensions. FIXME: check the wheel radius
#define TARANTULA_MIN_WHEEL_RADIUS 0.05
//@}

#define USBOT_MAX_QUE_LEN 32
#define USBOT_MAX_MSG_LEN 4096 
#define USBOT_MAX_CMD_LEN 1024

/**
 * @name Sensor types
 *
 * These costants are for internal use. They identify the type of
 * sensor reading.
 */
//@{
#define US_DATA_POSITION	0x01
#define US_DATA_POSITION3D	0x01<<1
#define US_DATA_LASER		0x01<<2
#define US_DATA_SONAR		0x01<<3
#define US_DATA_PTZ		0x01<<4
#define US_GEOM_LASER		0x01<<5
#define US_GEOM_SONAR		0x01<<6
#define US_CONF_LASER		0x01<<7
#define US_CONF_SONAR		0x01<<8
#define US_ODOMETRY		0x01<<9
#define US_INU			0x01<<10
#define US_IRCAMERA		0x01<<11
#define US_RFIDTAG		0x01<<12
#define US_VICTRFID		0x01<<14
#define US_3DSCANNER		0x01<<15
#define US_BALL			0x01<<16
#define US_ENCODER		0x01<<17
#define US_RFID_RELEASER	0x01<<18
#define US_RFID_DATA_RESULT	0x01<<19
#define US_VICTDATA		0x01<<20
#define US_TOUCH		0x01<<21
#define US_GROUND_TRUTH		0x01<<22
#define US_STA                  0x01<<23
//@}

#define IRCAM_WIDTH  0.16
#define IRCAM_HEIGHT 0.12
#define IRCAM_MAX_RANGE 1


#define PROPERTY_INU_SENSORDATA "inuSensorData"

#define PROPERTY_DEVICE_LASER_X "deviceLaserX"
#define PROPERTY_DEVICE_LASER_Y "deviceLaserY"

#define PROPERTY_USE_GROUND_TRUE_POSE "useRealPose"
#define PROPERTY_USE_DEVICE_LASER "useDeviceLaser"
#define PROPERTY_USE_DEVICE_SONAR "useDeviceSonar"
#define PROPERTY_USE_DEVICE_IRCAMERA "useDeviceIRCamera"
#define PROPERTY_USE_USARSIM_TIMESTAMPS "useUsarSimTimestamps"
#define PROPERTY_USE_RFID_LOCATION "params/useRFIDLocation"


/// Pan Tilt "Mission Package" properties
#define PROPERTY_PANTILT_ENABLED "devices/pantiltEnabled"
#define PROPERTY_PANTILT_MISSION_PACKAGE_NAME "pantilt/missionPackageName"
#define PROPERTY_PANTILT_PAN "pantilt/pan"
#define PROPERTY_PANTILT_TILT "pantilt/tilt"
#define PROPERTY_PANTILT_DESIRED_PAN "pantilt/desiredPan"
#define PROPERTY_PANTILT_DESIRED_TILT "pantilt/desiredTilt"
#define PROPERTY_VIEW_PANTILT_DIRECTION "pantilt/view/direction"
#define PROPERTY_VIEW_PANTILT_DIRECTION_FOV "pantilt/view/directionFov"
#define PROPERTY_VIEW_PANTILT_DIRECTION_DIST "pantilt/view/directionDist"

#define PROPERTY_ROBOT_VIEWNUMBER "viewNum"

/// Victim Sensor properties
#define PROPERTY_VICTIM_SENSOR_ENABLED "devices/victimSensorEnabled"
#define PROPERTY_VICTIM_SENSOR_PARTS_SEEN "victimSensor/partsSeen"
#define PROPERTY_VICTIM_SENSOR_PARTS_SEEN_INTERVAL "victimSensor/partsSeenInterval"

/// Victim RFID Sensor properties
#define PROPERTY_VICTIM_RFID_SENSOR_ENABLED "devices/victimRfidSensorEnabled"
#define PROPERTY_VICTIM_RFID_SENSOR_FOV "victimRfidSensor/fov"
#define PROPERTY_VICTIM_RFID_SENSOR_MAX_DIST "victimRfidSensor/maxDist"
#define PROPERTY_VICTIM_RFID_SENSOR_USE_RELIABILITY "victimRfidSensor/useReliability"

#define PROPERTY_COPY_ODOMETRY "copyOdometry"

#define PROPERTY_EXPANSION_SIZE "expansionSize"
#define PROPERTY_RFID_TAGS "tags"
#define PROPERTY_ITEMS "itemOnMap"

// please FIXME
#define PROPERTY_HOKUYO_LASER_DATA "rangeScanners/hokuyo/laserData"
#define PROPERTY_HOKUYO_SENSOR_NAME "rangeScanners/hokuyo/sensorName"

#define PROPERTY_REMOTE_MAPPER "remoteMapperName"
#define PROPERTY_REMOTE_WRITE "remoteWrite"

#define PROPERTY_LASER_MAX_RANGE "laserMaxRange"
#define PROPERTY_LASER_MIN_RANGE "laserMinRange"

#define PROPERTY_VICTIM_NAME "victimName"
#define PROPERTY_VICTIM_X "victimX"
#define PROPERTY_VICTIM_Y "victimY"
#define PROPERTY_VICTIM_Z "victimZ"

#define PROPERTY_SAVE_DATA "saveData"

#define PROPERTY_BALL_POSITION "ball/position"
#define PROPERTY_BALL_VISIBLE "ball/visible"

#define PROPERTY_RFID_WRITE_FILE "rfidWriteFile"

// Touch sensor data
#define PROPERTY_TOUCH_SENSORS "out/touchSensors"

// Tarantula
#define PROPERTY_ROBOT_FRONT_FLIPPER_POSITION "tarantula/frontFlipperPosition"
#define PROPERTY_ROBOT_REAR_FLIPPER_POSITION "tarantula/rearFlipperPosition"
#define PROPERTY_ROBOT_DESIRED_FRONT_FLIPPER_SPEED "tarantula/desiredFrontFlipperSpeed"
#define PROPERTY_ROBOT_DESIRED_REAR_FLIPPER_SPEED "tarantula/desiredRearFlipperSpeed"
#define PROPERTY_ROBOT_HAS_PWM_FLIPPERS "tarantula/hasPWMFlippers"
#define PROPERTY_ROBOT_HAS_PWM_TRACKS "tarantula/hasPWMTracks"
#define PROPERTY_ROBOT_MAX_FLIPPER_SPEED "tarantula/maxFlipperSpeed"
#define PROPERTY_ROBOT_DESIRED_LEFT_TRACK_SPEED "tarantula/desiredLeftTrackSpeed"
#define PROPERTY_ROBOT_DESIRED_RIGHT_TRACK_SPEED "tarantula/desiredRightTrackSpeed"
#define PROPERTY_ROBOT_CONTROL_TRACKS "tarantula/controlTracks"

// QuadRotor
#define PROPERTY_ROBOT_ALTITUDE_VELOCITY   "quadrotor/altitudeVelocity"
#define PROPERTY_ROBOT_LINEAR_VELOCITY 	   "quadrotor/linearVelocity"
#define PROPERTY_ROBOT_LATERAL_VELOCITY    "quadrotor/lateralVelocity"
#define PROPERTY_ROBOT_ROTATIONAL_VELOCITY "quadrotor/rotationalVelocity"
#define PROPERTY_ROBOT_POSE3   	           "quadrotor/pose3"
#define PROPERTY_ROBOT_POSE_2D				"quadrotor/pose2"

#define PROPERTY_ROBOT_YAW                 "quadrotor/yaw"
#define PROPERTY_ROBOT_CURRENT_X   	   "quadrotor/current_x"
#define PROPERTY_ROBOT_CURRENT_Y 	   "quadrotor/current_y"
#define PROPERTY_ROBOT_CURRENT_Z           "quadrotor/current_z"

// Interleaved view
#define PROPERTY_ROBOT_INTERLEAVED_VIEW_ENABLED "interleavedView/enabled"
#define PROPERTY_ROBOT_INTERLEAVED_VIEW_TURN "interleavedView/turn"
#define PROPERTY_ROBOT_INTERLEAVED_VIEW_NUMBER_OF_ROBOTS "interleavedView/numberOfRobots"
#define PROPERTY_ROBOT_INTERLEAVED_VIEW_TIME_INTERVAL "interleavedView/timeInterval"
#define PROPERTY_ROBOT_INTERLEAVED_VIEW_SCREENSHOT_ENABLED "interleavedView/screenshotEnabled"

// RFIDs
#define PROPERTY_LAST_RFID_DROP_SUCCESSFUL "out/lastRFIDDropSuccesful"
#define PROPERTY_LAST_RFID_READ_SUCCESSFUL "out/lastRFIDReadingSuccesful"
#define PROPERTY_LAST_RFID_WRITE_SUCCESSFUL "out/lastRFIDWritingSuccesful"
#define PROPERTY_RFID_DETECTED_IDS "out/lastDetectedRFIDIDs"
#define PROPERTY_RFID_READING_DATA "out/RFIDData"

#define PROPERTY_RFID_ID_TO_READ "in/RFIDIDToRead"
#define PROPERTY_RFID_ID_TO_WRITE "in/RFIDIDToWrite"
#define PROPERTY_RFID_DATA_TO_WRITE "in/RFIDDataToWrite"

#define PROPERTY_CMD_DROP_RFID "cmds/dropRFID"
#define PROPERTY_CMD_READ_RFID "cmds/readRFID"
#define PROPERTY_CMD_WRITE_RFID "cmds/writeRFID"

#define PROPERTY_CURRENT_VICTIM_VIEW "view/currentVictim"
//#define PROPERTY_SCREENSHOT_VICTIM "rdk://IC/imageClient/takeScreenshot"

#define PROPERTY_HOKUYO_DATA_QUEUE "rangeScanners/hokuyo/dataQueue"
#define PROPERTY_SONAR_DATA "sonarData2"

// #define PROPERTY_RFID_QUEUE "RFIDQueue"
// #define PROPERTY_RFID_DIST "RFIDist"
// #define PROPERTY_RFID_NUMRAYS "RfidNRays"

/// What do display in the "data" field when we have a reading with no
/// data

enum State {INACTIVE, FAILED, COMPUTING, REFINING, DONE };
enum UsarSensor {LASER, SONAR, PANTILT, ODOMETRY, INU, SOUND, IRCAMERA, SENSOR_END};
enum PanTiltCommands {PAN, TILT, PTRESET, PANTILT_END};

#define MAX_NUM_RAYS 360

namespace RDK2 { namespace RAgent {

using namespace RDK2::RSensorData;

class UsarSimClientModule : public Module {
public:

	virtual ~UsarSimClientModule() { }

	bool initConfigurationProperties();
	bool initInterfaceProperties();
	bool init();
	void exec();
	void cleanup();
	void asyncAgentCmd(cstr cmd);
	void exitRequested();
	
protected:	
	/**
	 * Read data from USARSim and update our knowledge of the world.
	 */
	void parseUsarData();	// OLD FUNCTION
	bool readFromServer(const std::string& l);

	/**
	* @brief Scans s for elements in the form {tag content}
	*
	* Tag and content are separated by a space.
	*
	* @param s the string to search. It is modified, so that subsequent
	* calls to this function on the same string return subsequent pairs
	* (tag, content)
	*
	* @param tag will contain the tag
	*
	* @param content wll contain the content
	*
	* @return true if a tag + content pair is found, false otherwise
	*/
	static bool getNextSegmentContent(string& s, string& tag, string& content);

	/// Spawn a robot
	bool spawnBot();
	/// Send a command to USARSim
	int addCommand(const char* command);
	void fillRFIDQueue();
	std::ofstream rfidStream;
	std::ofstream victimStream;
	void logVictim(RMaps::RItemOnMapVector* items);	

private:
	bool us_connect() throw(SessionException);
	bool us_connected();
	void changeCamera(bool* cameraChanged);
	/**
	 * For internal use: sets the local pose in the local
	 * properties, and pushes an event in the data queue.
	 */
	void setRobotPose(double xPos, double yPos, double yaw, 
			  double timeact);

	/// @name Sensor readings
	//@{
	void parseStaData(char* pBody);
	void parseGroundTruthData(char *pBody);
	void parsePositionData(char* pBody);
	void parsePanTiltData(char* pBody);
	void parseLaserConfigure(char* pBody);
	void parseLaserData(char* pBody);
	void parseOdometryData(char* pBody);
	void parseRfidTagData(char* pBody);
	void parseVictimRfidData(char* pBody);
	void parseVictimData(char* pBody);
	void parseSonarData(char* pBody);
	
	/** New parser functions **/
	void parseRfid(std::string& s);
	void parseGroundTruth(std::string& s);
	void parseRangeScanner(std::string& s);
	void parseGPS(std::string& s);
	void parseIns(std::string& s);
	void parseTouch(std::string& s);
	void parseEncoder(std::string& s);
	
	/**
	 * Add a RFID data reading to the repository, to the mapper,
	 * send it to remote hosts...
	 */
	void recordNewRFID(RUsarRfidData* rfd);
	/**
	 * Parse INS sensor reading (the name is for backward
	 * compatibility). The result is pushed into
	 * PROPERTY_ROBOT_DATA_QUEUE.
	 *
	 * This method needs the session to be started.
	 */
	void parseInuData(char* pBody);
	void parseBallData(char* pBody);
	/**
	 * Parse encoder reading. The result is pushed into
	 * PROPERTY_ROBOT_DATA_QUEUE.
	 *
	 * This method needs the session to be started.
	 */
	void parseEncoderData(char *pBody);
	/**
	 * Parse the answer to a "drop RFID" command.
	 *
	 * If the dropping was succesful, then the property
	 * out/lastRFIDDropSuccesful is set to true; otherwise, it's
	 * set to FALSE.
	 */
	void parseRfidReleaserData(char *pBody);
	/**
	 * Parse the answer to a "read RFID" or "write RFID" command.
	 *
	 * If the reading/writing was succesful, then the property
	 * out/lastRFIDReadingSuccesful (or
	 * out/lastRFIDReadingSuccesful) is set to true; otherwise,
	 * it's set to FALSE.
	 *
	 * The reading/writing command is also disabled.
	 *
	 * WARNING: the system is unable to distinguish between a read
	 * failure and a write failure! Please don't read and write
	 * RFID concurrently, to avoid possible problems.
	 */
	void parseRfidDataResult(char *pBody);
	/**
	 * Parse the touch sensor reading
	 */
	void parseTouchSensorData(const char* pBody);
	//@}

	/**
	 * Calculates wheel speed based on the robot class and the
	 * desired speed and jog.
	 *
	 * This function also saturates tracks' speed on a non-PWM
	 * driven Tarantula
	 */
        void computeWheelSpeed(Session *session, double cmdSpeed, double cmdJog, double& leftWheelSpeed, double& rightWheelSpeed);

	/// Send the "drop RFID" command.
	/**
	 * This methods sets also the command property. After calling
	 * it, you should wait for the command property to be unset.
	 * The property out/lastRFIDDropSuccesful will then be
	 * set. All this is done by parseRFIDReleaserData(), when
	 * USARSim sends a RES message.
	 */
	void dropRFID();

	/// Send the "read RFID" command.
	/**
	 * This methods sets also the command property. After calling
	 * it, you should wait for the command property to be unset.
	 * The property out/lastRFIDReadSuccesful will then be
	 * set. All this is done by parseRFIDDataResult(), when
	 * USARSim sends a RES message.
	 */
	void readRFID();

	/// Send the "write RFID" command.
	/**
	 * This methods sets also the command property. Before calling
	 * it, you should set the in/RFIDIDToWrite property to the
	 * RFID ID you want to access.
	 *
	 * After calling, wait for the command property to be unset.
	 * The property out/lastRFIDWriteSuccesful will then be
	 * set. All this is done by parseRFIDDataResult(), when
	 * USARSim sends a RES message.
	 */
	void writeRFID();

	/// Send commands to robot, based on the module properties
	/**
	 * Send all the commands needed to set speed, jog, pantilt,
	 * flippers, etc.
	 *
	 * All the data are read from the repository.
	 *
	 * This method is supposed to be called by exec(). It needs a
	 * started session.
	 */
	void commandRobot();
	
	map<string, double> cmdFlipperPositions;
	
	/// Gamebots socket
	int m_sock;
	unsigned int connectionAttempts;
	bool connected, spawnedBot;	

	/*debug*/
	ofstream log;
	ofstream logApprendimento;	
	
	/*To Check*/
	bool panTiltUpdate[PANTILT_END];
	double usarCameraPan, usarCameraTilt;	//in rad
	float spqrPan, spqrTilt;			//in deg
	
	double mySpeedJogTime, mySpeed, myJog;

	void refreshPanTiltDirection(Session* session) throw (SessionException);

	bool cmdSent;
	/// Have we dropped a RFID, and are we waiting for the RES
	/// message about it?
	bool waitingForRFIDResult;
	
	void updateCurrentVictimPartsSeen(Session* session) throw (SessionException);

	typedef std::map<std::string,RUsarVictimRfidData>::iterator rfidIt; 
	std::map<std::string,RUsarVictimRfidData> rfidMap;
	
	Point2od initialPose;
	set<string> curVictimPartsSeen;
	TimerR curVictimPartsSeenTimer;
	
	/// this is needed because ComStation robot does not receive anything, it
	/// should not be waiting on the socket (RDK event queue will become too big)
	bool isComStation;
};

}} // namespace

#endif
