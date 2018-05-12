	/**
 * This file contains functions and methods of UsarSimClientModule class
 * that parse USARSim messages.
 */

#include <float.h> // for DBL_MAX
#include <errno.h>
#include <cstring>

#include <rdkcore/rsensordata/rusarinudata.h>
#include <rdkcore/rsensordata/rusarvictimdata.h>
#include <rdkcore/rsensordata/rtouchsensordata.h>
#include <rdkcore/rmaps/rline2donmap.h>
#include "usarsimclientmodule.h"
#include "parsing.h"

#include <rdkcore/rsensordata/rsonardata.h>
#include <rdkcore/geometry/point3.h>

using namespace std;

//Funzioni brutte da rifare con le stringhe !!!!!
bool us_getData(int sock,char* data,unsigned int data_ln){
    unsigned int numread = 0;
    char c = 0;
    int res=0;
    while (c!='\n' && numread<data_ln) {	// read a line
		res=read(sock,&c,1);
		if (res == 0) return false;	// connection closed
		else if(res == -1) { 			// read one byte
			if(errno != EAGAIN) {
				RDK_ERROR_STREAM("read() failed for code: " << errno << " exiting.");
				return false;
    			}
	 	}
		else data[numread++]=c;
    }
    if (res==-1){
	if(errno != EAGAIN) {
		RDK_ERROR_STREAM("read() failed for code: " << errno << " exiting.");
		return false;
	}    
    }
    data[numread]='\0';
//     std::ofstream outfile;
//     outfile.open("outputFile2");
//     outfile << "index  = " << numread << "data lengh = " << data_ln << std::endl;
//     outfile << "index  = " << data << std::endl;     
    return true;
}

/*bool us_getLine(char* data,char* line,unsigned int data_ln){
    unsigned int numread = 0;
    char c = 0;
    int res=0;
    while (c!='\n' && numread<data_ln && c!='\0') {	// read a line
		c = data[numread]; 
		line[numread++]=c;
    }
    line[numread]='\0';
//    std::ofstream outfile;
//    outfile.open("outputFile2");
//    outfile << "index  = " << numread << "data lengh = " << data_ln << std::endl;
//    outfile << "index  = " << data << std::endl;     
    return numread;
}*/


/*bool us_getData(int sock,char* data,unsigned int data_ln){
    int res=0;
    res=read(sock,data,data_ln);
    //RDK_DEBUG_STREAM("Data read res = " << res);
    if (res==-1){
	if(errno != EAGAIN) {
		RDK_ERROR_STREAM("read() failed for code: " << errno << " exiting.");
		return false;
	}    
	//RDK_DEBUG_STREAM("errorcode = " << errno);
    	data[0] = '\0';
	return true;
    }
    int index = res<data_ln ? res:data_ln;
    data[index]='\0';
//     std::ofstream outfile;
//     outfile.open("outputFile");
//     outfile << "index  = " << index << "data lengh = " << data_ln << std::endl;
//     outfile << "index  = " << data << std::endl;     
    //RDK_DEBUG_STREAM("Data read = "<< data);
    return true;
}*/

int us_get_word(char* data, int pos, char* word) {
  char *p, *pNext;
  int next;

  if (data==NULL || pos<0 || pos>=(int)strlen(data)) return -1;
  for (p=data+pos;*p==' ';p++)
	{}
  pNext = strchr(p,' ');
  if (pNext == NULL) pNext = data + strlen(data);
  next = pNext - p;
  if (word!=NULL) {
    strncpy(word,p,next);
    word[next]=0;
  }
  return next + pos;
}

int us_get_type(char* data, char** ppBody) {
	string head, check, type;
	int res = 0;
	// if (data==NULL) return -1; // Who would ever do this? (Arrigo)
	istringstream line(data);
	// Almost all the message types need the first three words to
	// be identified. Don't forget the ending "}"!
	line >> head >> check >> type;
	*ppBody = data + head.length();
	if (check == "{Time") { // Some sensors give the Time
		// Ignore it and read next token
		line >> check >> type;
	}
	if (head == "STA") {
		res |= US_STA;
	} else if (head == "SEN") {
		if (check == "{Type") { // Sanity check
			if (type == "Range}") res |= US_DATA_SONAR;
			else if (type == "Sonar}") res |= US_DATA_SONAR;
			else if (type == "RangeScanner}") res |= US_DATA_LASER;
			else if (type == "IRCamera}") res |= US_IRCAMERA;
			else if (type == "Odometry}") res |= US_ODOMETRY;
			// once upon a time, there was INU. Now, there's INS.
			else if (type == "INS}") res |= US_INU;
			else if (type == "RFID}") res |= US_RFIDTAG;
			else if (type == "VictRFID}") res |= US_VICTRFID;
			else if (type == "VictSensor}") res |= US_VICTDATA;
			else if (type == "3DRangeScanner}")
				res |= US_3DSCANNER;
			else if (type == "Helper}") res |= US_BALL;
			else if (type == "Encoder}") res |= US_ENCODER;
			else if (type == "Touch}") res |= US_TOUCH;
			else if (type == "GroundTruth}") {
				res |= US_GROUND_TRUTH | US_DATA_POSITION | 
					US_DATA_POSITION3D;
			} else {
//  				RDK_ERROR_STREAM("Unhandled SEN message: " <<
//  						 data);
				res = -1;
			}
		} else {
			RDK_ERROR_STREAM("Bad SEN message from USARSim: " <<
					 "check : \"" << check << "\"" << data);
			res = -1;
		}
	}
	else if (head == "MISSTA") {
		if (check == "{Name") {
			if (type == "CameraPanTilt}"
				/*session->getString(PROPERTY_PANTILT_MISSION_PACKAGE_NAME) FIXME + "}"*/) {
				res |= US_DATA_PTZ;
			}
		}
	}
	else if (head == "MIS") {
		if (check == "{Name") { // Sanity check
			if (type == "Cam1}") res |= US_DATA_PTZ;
			else {
				RDK_ERROR_STREAM("Unhandled MIS message: " <<
						 data);
				res = -1;
			}
		} else {
			RDK_ERROR_STREAM("Bad MIS message from USARSim: " <<
					 data);
			res = -1;
		}
	} else if (head == "GEO") {
		if (check == "{Type") { // Sanity ckeck
			if (type == "Range}") res |= US_GEOM_SONAR;
			else if (type == "RangeScanner}") res |= US_GEOM_LASER;
			else {
				RDK_ERROR_STREAM("Unhandled GEO message: " <<
						 data);
				res = -1;
			}
		} else {
			RDK_ERROR_STREAM("Bad GEO message from USARSim: " <<
					 data);
			res = -1;
		}
	} else if (head == "CONF") {
		if (check == "{Type") { // Sanity check
			if (type == "Range}") res |= US_CONF_SONAR;
			else if (type == "RangeScanner}") res |= US_CONF_LASER;
			else {
				RDK_ERROR_STREAM("Unhandled CONF message: " <<
						 data);
				res = -1;
			}
		} else {
			RDK_ERROR_STREAM("Bad CONF message from USARSim: " <<
					 data);
			res = -1;
		}
	} else if (head == "RES") {
		if (check == "{Type") { // Sanity check
			if (type == "RFIDReleaser}") res |= US_RFID_RELEASER;
			else if (type == "RFID}") {
				// RDK_DEBUG_PRINTF(data);
				res |= US_RFID_DATA_RESULT;
			} else {
				RDK_DEBUG_PRINTF("Unhandled RES message: %s",
						 data);
				res = -1;
			}
		} else {
			RDK_ERROR_STREAM("Bad RES message from USARSim: " <<
					 data);
			res = -1;
		}

	} else res = -1;
	return res;
}


int us_get_segmentByName(char* data, int pos, char* name, char* segment) {
  char *p1, *p2;
  char tmpStr[128];

  if (data==NULL || name==NULL || pos<0 || pos>=(int)strlen(data)) return -1;
  tmpStr[0]='{';
  strncpy(tmpStr+1,name,sizeof(tmpStr));
  if ((p1 = strstr(data+pos,tmpStr))==NULL) return -1;
  p1 += 1;
  if ((p2 = strchr(p1,'}'))==NULL) return -1;
  if (segment!=NULL) {
    strncpy(segment,p1,p2-p1);
    segment[p2-p1]=0;
  }
  return p2-data+1;
}

int us_get_value(char* segment, char* name, char* value) {
  char *p;
  if (segment==NULL || name==NULL) return -1;
  if ((p = strstr(segment,name))==NULL) return -1;
  return us_get_word(segment,p+strlen(name)-segment,value);
}

int us_get_value2(char* data, const char* name, char* value) {
  char *p1, *p2;
  char tmpStr[128];
  int pos = 0;

  if (data==NULL || name==NULL) return -1;
  tmpStr[0]='{'; 
//  RDK_DEBUG_STREAM("us_get_value2 with name: " << name);
  strncpy(tmpStr+1, name, sizeof(tmpStr) - 2);
  if ((p1 = strstr(data+pos,tmpStr))==NULL) return -1;
  p1 += strlen(tmpStr);
  if ((p2 = strchr(p1,' '))==NULL) return -1;
  pos = us_get_word(data,p2-data+1,value);
  if (data[pos-1]=='}') {
    pos-=1;
    value[strlen(value)-1]=0;
  }
  return pos;
}

using namespace RDK2::Geometry;

namespace RDK2 { namespace RAgent {
using namespace RDK2::RMaps;
using namespace RDK2::Geometry;

void UsarSimClientModule::parseUsarData(){
	char data[USBOT_MAX_MSG_LEN+1];
    	connected = false;
	connected = us_getData(m_sock,data,sizeof(data));

if (readFromServer(string(data))) return; 

#ifdef ENABLE_LOG
	if (! log.good()){
		RDK_DEBUG_STREAM("Could not open log");
	}
	log << data << endl;
#endif
//     char* line;
//	while (us_getLine(data,line,data_ln)){
		
//	} 

// 	RDK_INFO_STREAM("USARSim data: " << data);
	char *pBody;
	int type = us_get_type(data,&pBody);	//pBody = data without type string
	if (type==-1) return;
	bool useLaser = false;
	bool useSonar = false;
	//bool usePantilt = false;
	bool useIRCamera = false;
	bool useRealPose = session->getBool(PROPERTY_USE_GROUND_TRUE_POSE);
	useLaser = session->getBool(PROPERTY_USE_DEVICE_LASER);
	useSonar = session->getBool(PROPERTY_USE_DEVICE_SONAR);
	//usePantilt = session->getBool(PROPERTY_USE_DEVICE_PANTILT);
	useIRCamera = session->getBool(PROPERTY_USE_DEVICE_IRCAMERA);
	useIRCamera = useIRCamera;
	
	bool pantiltEnabled = session->getBool(PROPERTY_PANTILT_ENABLED);
	bool victimSensorEnabled = session->getBool(PROPERTY_VICTIM_SENSOR_ENABLED);
	bool victimRfidSensorEnabled = session->getBool(PROPERTY_VICTIM_RFID_SENSOR_ENABLED);

	//RDK_DEBUG_PRINTF("Parsing Messages");
	if (type & US_ODOMETRY && !useRealPose) {
		parseOdometryData(pBody);
	} else if (type & US_STA) {
		parseStaData(pBody);
	} else if (useRealPose && (type & US_DATA_POSITION) &&
		   (type & US_GROUND_TRUTH)) {
		parseGroundTruthData(pBody);
	} else if (useLaser && (type & US_DATA_LASER)) {
		parseLaserData(pBody);
	} else if (useLaser && (type & US_CONF_LASER)) {
		parseLaserConfigure(pBody);
	} else if (pantiltEnabled && (type & US_DATA_PTZ))
		parsePanTiltData(pBody);
	else if (useSonar && (type & US_DATA_SONAR)) 
		parseSonarData(pBody);
	else if (victimSensorEnabled && (type & US_VICTDATA)) parseVictimData(pBody);
	else if (type & US_RFIDTAG)
		{ parseRfidTagData(pBody);  }
	else if (type & US_INU)
		{ parseInuData(pBody);  }
	else if (type & US_BALL)
		{ parseBallData(pBody); }
	else if (type & US_ENCODER)
		parseEncoderData(pBody);
	else if (type & US_RFID_RELEASER)
		parseRfidReleaserData(pBody);
	else if (type & US_RFID_DATA_RESULT)
		parseRfidDataResult(pBody);
	else if (victimRfidSensorEnabled && (type & US_VICTRFID)) parseVictimRfidData(pBody);
	else if (type & US_3DSCANNER){
		RDK_DEBUG_STREAM("parsing");
		if (! logApprendimento.good()){
			RDK_DEBUG_STREAM("Could not open log");
		}
		if (session->getBool(PROPERTY_SAVE_DATA)){
			logApprendimento << data << endl;
			//Write robot Pose
//			logApprendimento << 
			session->setBool(PROPERTY_SAVE_DATA,false);
		}
		RDK_DEBUG_STREAM("wrote");
		cmdSent = false;
	} else if (type & US_TOUCH) {
		parseTouchSensorData(pBody);
	}
}

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
bool getNextSegmentContent(string& s, string& tag, string& content)
{
	if (s.size() == 0) { tag = ""; content = ""; return false; }
	size_t a = s.find_first_of("{"), b = s.find_first_of("}");
	if (a == string::npos || b == string::npos) { tag = ""; content = ""; return false; }
	string c = s.substr(a + 1, b - a - 1);
	size_t d = c.find_first_of(" ");
	if (d == string::npos) { tag = ""; content = ""; return false; }
	tag = c.substr(0, d);
	content = c.substr(d + 1);
	s = s.substr(b + 1);
	return true;
}

void UsarSimClientModule::parsePanTiltData(char* pBody)
{
	string s(pBody), tag, content;
	while (getNextSegmentContent(s, tag, content)) {
		if (tag == "Link" && content == "1") {
			while (getNextSegmentContent(s, tag, content) && tag != "Value") ;
			if (tag == "Value") session->setDouble(PROPERTY_PANTILT_PAN, atof(content.c_str()));
		}
		else if (tag == "Link" && content == "2") {
			while (getNextSegmentContent(s, tag, content) && tag != "Value") ;
			if (tag == "Value") session->setDouble(PROPERTY_PANTILT_TILT, atof(content.c_str()));
		}
	}
}

void UsarSimClientModule::parseVictimData(char* pBody)
{
	bool useUsarSimTimestamps = session->getBool(PROPERTY_USE_USARSIM_TIMESTAMPS);
	
	string s(pBody), tag, content;
	Timestamp ts;
	bool timestampFound = false;
	string partName, sensorName;
	Point3d location;
	while (getNextSegmentContent(s, tag, content)) {
		if (tag == "Time" && useUsarSimTimestamps) {
			timestampFound = true;
			ts.setSeconds(atof(content.c_str()));
		}
		else if (tag == "Name") sensorName = content;
		else if (tag == "PartName") partName = content;
		else if (tag == "Location") {
			vector<string> v = TextUtils::tokenize(content, ",");
			if (v.size() != 3) {
				RDK_ERROR_PRINTF("Malformed location segment in victim sensor data '%s'", pBody);
			}
			else {
				location.x = atof(v[0].c_str());
				location.y = -atof(v[1].c_str());
				location.z = -atof(v[2].c_str());
				RUsarVictimData* vd = new RUsarVictimData();
				vd->timestamp = ts;
				vd->location = location;
				vd->partName = partName;
				curVictimPartsSeen.insert(partName);
				vd->sensorName = sensorName;
				vd->estimatedPose = session->getPose(PROPERTY_ROBOT_ESTIMATED_POSE);
				vd->odometryPose = session->getPose(PROPERTY_ROBOT_ODOMETRY_POSE);
/*				RDK_INFO_PRINTF("Found victim sensor data. "
					"Sensor name: '%s', Part: '%s', Location: %.2f %.2f %.2f",
					sensorName.c_str(), partName.c_str(), location.x, location.y, location.z);*/
				session->queuePush(PROPERTY_ROBOT_DATA_QUEUE, vd);
			}
		}
		else if (tag == "Status") {
			if (content == "NoVictims") return;
		}
	}
	if (useUsarSimTimestamps && !timestampFound) {
		RDK_ERROR_PRINTF("You want to use USARSim timestamps, but I cannot find timestamp in sensor message '%s'", pBody);
	}
}

void UsarSimClientModule::parseVictimRfidData(char* pBody)
{
	bool useUsarSimTimestamps = session->getBool(PROPERTY_USE_USARSIM_TIMESTAMPS);
	Point2od robotPose = session->getPose(PROPERTY_ROBOT_ESTIMATED_POSE);
	
	string s(pBody), tag, content;
	Timestamp ts;
	bool timestampFound = false;
	string sensorName = "", victimId = "", victimName = "", victimStatus = "";
	Point3d location;
	while (getNextSegmentContent(s, tag, content)) {
		if (tag == "Time" && useUsarSimTimestamps) {
			timestampFound = true;
			ts.setSeconds(atof(content.c_str()));
		}
		else if (tag == "Name") sensorName = content;
		else if (tag == "VictimId") victimId = content;
		else if (tag == "VictimName") {
			victimName = content;
			if (victimName.substr(0, 4) == "Fire") victimName = "Fire";	// FIXME
		}
		else if (tag == "VictimStatus") victimStatus = content;
		else if (tag == "Location") {
			vector<string> v = TextUtils::tokenize(content, ",");
			if (v.size() != 3) {
				RDK_ERROR_PRINTF("Malformed location segment in victim sensor data '%s'",
					pBody);
			}
			else {
				// FIXME fare una funzione
				location.x = atof(v[0].c_str());
				location.y = -atof(v[1].c_str());
				location.z = atof(v[2].c_str());
				Point2d p = location.rot(robotPose.theta);
				p = p + robotPose;
				location.x = p.x;
				location.y = p.y;
			}
		}
	}
	if (useUsarSimTimestamps && !timestampFound) {
		RDK_ERROR_PRINTF("You want to use USARSim timestamps, "
			"but I cannot find timestamp in sensor message '%s'", pBody);
	}
	
	// check if it should be filtered for FOV or max distance
	double victimSensorFov = session->getDouble(PROPERTY_VICTIM_RFID_SENSOR_FOV);
	double victimSensorMaxDist = session->getDouble(PROPERTY_VICTIM_RFID_SENSOR_MAX_DIST);	
	double pan = session->getDouble(PROPERTY_PANTILT_PAN);
	bool shouldBeFilteredOut = false;
	if (robotPose.distTo(location) > victimSensorMaxDist) shouldBeFilteredOut = true;
	else {
		double rfidAngleWithPan = angDiff(robotPose.thetaWith(location), robotPose.theta);
		rfidAngleWithPan = angDiff(rfidAngleWithPan, pan);
		if (fabs(rfidAngleWithPan) > victimSensorFov / 2) shouldBeFilteredOut = true;
	}
	if (victimName == "Fire") shouldBeFilteredOut = false;
	
	if (!shouldBeFilteredOut) {
		RUsarVictimRfidData* vd = new RUsarVictimRfidData();
		vd->timestamp = ts;
		vd->location = location;
		vd->id = victimId;
		vd->name = victimName;
		vd->status = victimStatus;
/*		RDK_INFO_PRINTF("Found victim RFID sensor data: "
			"Sensor name: '%s', VictimId: '%s', VictimName: '%s', "
			"VictimStatus: '%s', Location: %.2f %.2f %.2f",
			sensorName.c_str(), victimId.c_str(), victimName.c_str(),
			victimStatus.c_str(), location.x, location.y, location.z);*/
		session->queuePush(PROPERTY_ROBOT_DATA_QUEUE, vd);
	}
}

#if 0
void UsarSimClientModule::parsePanTiltData(char* pBody){

	RDK2::TextUtils::StringVector tokens, vals;
	std::string str;

	tokens = RDK2::TextUtils::tokenize(pBody);

	//Parse Pan
	if(tokens.size() > 9)
	{
		vals = RDK2::TextUtils::tokenize(tokens[9], ",");
		usarCameraPan = atof(vals[2].c_str());
		session->setDouble(PROPERTY_ROBOT_CAMERA_PAN, usarCameraPan);
	}
	//Parse Tilt
	if(tokens.size() > 15)
	{
		vals = RDK2::TextUtils::tokenize(tokens[15], ",");
		usarCameraTilt = atof(vals[1].c_str());
		session->setDouble(PROPERTY_ROBOT_CAMERA_TILT, usarCameraTilt);
	}
}
#endif

void UsarSimClientModule::parseLaserConfigure(char* pBody){

    char val[128];
    if (us_get_value2(pBody,"Resolution",val)==-1) return;
//     double resolution = atoi(val);
    if (us_get_value2(pBody,"Fov",val)==-1) return;
// 	double fov = atoi(val);
    if (us_get_value2(pBody,"MinRange",val)==-1) return;
// 	double min_range = atof(val)*US_UU_MM;
    if (us_get_value2(pBody,"MaxRange",val)==-1) return;
// 	double max_range = atof(val)*US_UU_MM;

// 	int nrays        = (int)(fov / resolution);
// 	double min_theta = GRAD2RAD((fov / -2)*US_UU_DEG);
// 	double max_theta = GRAD2RAD((fov / 2)*US_UU_DEG);
//     usarRangeFinder->parserConfigure(nrays, min_theta, max_theta, min_range, max_range);
}

void UsarSimClientModule::parseLaserData(char* pBody){
	//RDK_DEBUG_STREAM("Parsing Laser Reading");
	char tmp[128], *p1, *p2;

	RLaserData * rld = new RLaserData;

	// Using the real odometry in the odometry field of laserData JMK
	RDK2::Geometry::Point2od estimated =
	  session->getPose(PROPERTY_ROBOT_ESTIMATED_POSE);
	RDK2::Geometry::Point2od odometryPose =
	  session->getPose(PROPERTY_ROBOT_ODOMETRY_POSE);

	bool useUsarSimTimestamps = session->getBool(PROPERTY_USE_USARSIM_TIMESTAMPS);

	//RDK_DEBUG_PRINTF("ODOMETRY POSE: %f, %f, %f", odometryPose.x, odometryPose.y, odometryPose.theta);
	//RDK_DEBUG_PRINTF("ESTIMATED POSE: %f, %f, %f", estimated.x, estimated.y, estimated.theta);

	
	//FIXME temporary solution for handling hokuyo data...
	bool main_queue=true;
	if (us_get_value2(pBody,"Name",tmp)==-1){
	 return;
	}
//	RDK_DEBUG_PRINTF("Laser name: '%s'", tmp);
// 	int additional_rays=0;

	string hokuyoSensorName = session->getString(PROPERTY_HOKUYO_SENSOR_NAME);

	if (hokuyoSensorName == string(tmp)) {
		main_queue=false;
	}
	
	
	double timeact;
	if (useUsarSimTimestamps) {

		//RDK_DEBUG_STREAM("waiting for time");
		if (us_get_value2(pBody,"Time",tmp)==-1) return;
		double time = atof(tmp);
		//RDK_DEBUG_STREAM("got time");
		timeact = time;

	} else {
		struct timeval tv;
		gettimeofday(&tv, NULL);
		timeact = tv.tv_sec + 0.000001 * tv.tv_usec;
	}

	char ranges[2048];
	double newData[MAX_NUM_RAYS];
	int count = 0;
	if (us_get_value2(pBody, "Range", ranges)==-1) return;
	p1 = ranges;

/*	if(!main_queue){//Skips first 45 rays
		int skip_45=0;
		while ((p2 = strchr(p1,','))>0 && skip_45<45)	{
      *p2 = 0;
    	p1 = p2+1;
			skip_45++;
		}
	}*/
	while ((p2 = strchr(p1,','))>0)	{
      *p2 = 0;
      newData[count++] = atof(p1);
      p1 = p2+1;
	  if (count >= MAX_NUM_RAYS) break; //control that there are no more of numRays
	}
	newData[count++] = atof(p1);
// 	RDK_DEBUG_PRINTF("count : %d",count); 
	//now initialize the (eventual) rays from which we can't catch data
// 	while (count < numRays)
// 		newData[count++] = 0;

	double minRange = session->getDouble(PROPERTY_LASER_MIN_RANGE);
	double maxRange = session->getDouble(PROPERTY_LASER_MAX_RANGE);
 	int n = count;

	rld->points.resize(n);

	double angle=-M_PI/2;
	if (!main_queue) angle = deg2rad(-135.0);
	
	double step=M_PI/(n-1);
	if (!main_queue) step = deg2rad(270.0) / (n-1);
	
	for (int i=0; i<n; i++){
		SensorData::LaserData::LaserPoint lp;
		lp.theta      = angle;
		lp.reading  = newData[i];
// 		RDK_DEBUG_PRINTF("angle reading %f %f",angle,newData[i]);

		angle+=step;
 		//RDK_DEBUG_STREAM("lp.reading="<<lp.reading);
		lp.intensity  = 0;
		rld->points[i]=lp;
	}

// 	double laserX = 0;
// 	double laserY = 0;
// 	laserX = session->getDouble(PROPERTY_DEVICE_LASER_X);
// 	laserY = session->getDouble(PROPERTY_DEVICE_LASER_Y);


	// Using the real odometry in th odometry field of laserData JMK
	Point2od lp = session->getPose(PROPERTY_ROBOT_LASER_POSE);
	rld->laserPose.x = lp.x;
	rld->laserPose.y = lp.y;
	rld->laserPose.theta = lp.theta;
	//rld->laserPose = session->getPose(PROPERTY_ROBOT_LASER_POSE);
	rld->ipc_hostname  = /*"USARLaser"*/session->getRepositoryName();
	rld->timestamp.setSeconds(timeact);	// Non basta :)
	rld->ipc_timestamp.setSeconds(timeact);
	rld->minReading    = minRange;
	rld->maxReading    = maxRange;
	rld->minTheta      = -M_PI/2; //FIXME
	rld->maxTheta      = M_PI/2; //FIXME
	rld->odometryPose      = odometryPose;
	rld->estimatedPose      = estimated;

	// FIXME temporary solution for the hokuyo data...
	if (main_queue) {
		session->queuePush(PROPERTY_ROBOT_DATA_QUEUE, rld->clone());
		session->setObject(PROPERTY_ROBOT_LASER_DATA, rld);
	}
	else {
		session->queuePush(PROPERTY_HOKUYO_DATA_QUEUE, rld->clone());
		session->setObject(PROPERTY_HOKUYO_LASER_DATA, rld);
	}

	//queue->push_back(rld);
	//RDK_DEBUG_STREAM("rld points size = "<< rld->points.size() << " timestamp = " << rld->timestamp);
	//RDK_DEBUG_PRINTF("LASER PARSING");
}

void UsarSimClientModule::setRobotPose(double xPos, double yPos, double yaw,
				    double timeact) {
	ROdometryData * rod = new ROdometryData();
	yaw=-yaw; //  The z axis is flipped in USARsim JMK
	yPos *= -1.0; // The y axis is flipped

	session->setPose(PROPERTY_ROBOT_ODOMETRY_POSE,
			 Point2od(xPos, yPos, yaw));
	if (session->getBool(PROPERTY_COPY_ODOMETRY)){
		//to bypass the scanmatcher process
		session->setPose(PROPERTY_ROBOT_ESTIMATED_POSE,
				 Point2od(xPos, yPos, yaw));
	}

	rod->odometryPose.x = xPos;
	rod->odometryPose.y = yPos;
	rod->odometryPose.theta = yaw;
	rod->estimatedPose.x = xPos;
	rod->estimatedPose.y = yPos;
	rod->estimatedPose.theta = yaw;
	
	rod->leftStalled = rod->rightStalled = 0; //USARSIM has no stall values
	rod->speed = mySpeed;
	rod->jog   = myJog;
	//FIXME rod->counter? rod->ipc_name?
	rod->ipc_hostname  = "USARPosition";
	
	rod->timestamp.setSeconds(timeact);
	rod->ipc_timestamp = rod->timestamp;
	session->queuePush(PROPERTY_ROBOT_DATA_QUEUE,rod);

	//RDK_DEBUG_PRINTF("ROBOT POSE");
}

void UsarSimClientModule::parseStaData(char* pBody) { //Q(o), anzi OCROPOID
	char tmp[128];
	istringstream *iss;
//	RDK_DEBUG_STREAM("STA data: " << pBody);
/*	
	bool useUsarSimTimestamps = session->getBool(PROPERTY_USE_USARSIM_TIMESTAMPS);
	double tisession->queuePush(PROPERTY_ROBOT_DATA_QUEUE, rid);meact;
	char junk;
	if (useUsarSimTimestamps) {
		// RDK_DEBUG_STREAM("waiting for time in: " << pBody);
		if (us_get_value2(pBody,"Time",tmp)==-1) return;
		double time = atof(tmp);
		timeact = time;
	} else {
		struct timeval tv;
		//RDK_DEBUG_STREAM("Getting time of day...");
		gettimeofday(&tv, NULL);
		timeact = tv.tv_sec + 0.000001 * tv.tv_usec;
	}
*/	
	//=== Parse MULTIVIEW view number ===
	int viewNum;
	// RDK_DEBUG_PRINTF("Parsing STA View");
	if (us_get_value2(pBody, "View", tmp)==-1) { 
		RDK_ERROR_STREAM("Error reading View number"); 
		return;
	}
	iss = new istringstream(tmp);
	// sscanf(tmp,"%f,%f,%f",&roll,&pitch,&yaw);
	*iss >> viewNum;
	delete iss;

	session->setInt(PROPERTY_ROBOT_VIEWNUMBER, viewNum);
	//===================================

	//======= Parse BATTERY life ========
	int batteryLife;
	// RDK_DEBUG_PRINTF("Parsing BatteryLife");
	if (us_get_value2(pBody, "Battery", tmp)==-1) { 
		RDK_ERROR_STREAM("Error reading Battery life"); 
		return;
	}
	iss = new istringstream(tmp);
	// sscanf(tmp,"%f,%f,%f",&roll,&pitch,&yaw);
	*iss >> batteryLife;
	delete iss;

//	session->setInt(PROPERTY_ROBOT_BATTERYLIFE, batteryLife);
	//===================================
}

/*
 * Example message: 
 * SEN {Time 1939.1141} {Type GroundTruth} {Name GroundTruth} 
 *             {Location 0.75,-7.28,-0.26} {Orientation 0.00,6.28,0.00} ...
 */
void UsarSimClientModule::parseGroundTruthData(char* pBody) {
	string s(pBody), tag, content;
	double timeact;
	double xPos, yPos, zPos, roll, pitch, yaw;
	bool haveTime, useUsarSimTimestamps;
	struct timeval tv;
	useUsarSimTimestamps = 
		session->getBool(PROPERTY_USE_USARSIM_TIMESTAMPS);
	// We prepare the timestamp nevertheless; in case, we'll use
	// the one provided by USARSim.
	gettimeofday(&tv, NULL);
	timeact = tv.tv_sec + 0.000001 * tv.tv_usec;
	haveTime = !useUsarSimTimestamps;
	while (getNextSegmentContent(s, tag, content)) {
		if (tag == "Time") {
			if (useUsarSimTimestamps) {
				timeact = atof(content.c_str());
				haveTime = true;
			}
		} else if (tag == "Name") {
			// ignore it
		} else if (tag == "Type") {
			// ignore it
		} else if (tag == "Location") {
			char junk;
			istringstream iss(content);
			iss >> xPos >> junk >> yPos >> junk >> zPos;
		} else if (tag == "Orientation") {
			char junk;
			istringstream iss(content);
			iss >> roll >> junk >> pitch >> junk >> yaw;
		} else {
 			RDK_ERROR_STREAM("Unknown tag: " << tag << 
 					 " in GroundTruth sensor data: " << 
 					 pBody);
		}
	} // cycle all tags
	if (!haveTime) {
		RDK_ERROR_STREAM("Error: USARSim didn't send timestamp. "
				 "Using local one.");
	}
	setRobotPose(xPos, yPos, yaw, timeact);
//  	RDK_INFO_STREAM("GroundTruth: x=" << xPos << ", y=" << yPos << 
//  			", roll=" << roll << ", pitch=" << pitch << 
//  			", yaw=" << yaw);
}


void UsarSimClientModule::parseOdometryData(char* pBody)
{
	char tmp[128];

	bool useUsarSimTimestamps = session->getBool(PROPERTY_USE_USARSIM_TIMESTAMPS);

	double timeact;
	if (useUsarSimTimestamps) {
		if (us_get_value2(pBody,"Time",tmp)==-1) return;
		double time = atof(tmp);
		timeact = time;
	}
	else {
		struct timeval tv;
		gettimeofday(&tv, NULL);
		timeact = tv.tv_sec + 0.000001 * tv.tv_usec;
	}

	float xPos, yPos, yaw=0;

	//x and y are exchanged
	if (us_get_value2(pBody, "Pose", tmp)==-1){ RDK_DEBUG_STREAM("error reading x and y!!!!!!!!"); return;}
	sscanf(tmp,"%f,%f,%f",&xPos,&yPos,&yaw);

	yaw = -yaw; 	// The z axis is flipped in USARsim JMK
	yPos = -yPos; 	// The y axis is flipped

	ROdometryData * rod = new ROdometryData();

	Point2od odopose;
	odopose.x = cos(initialPose.theta) * xPos - sin(initialPose.theta) * yPos + initialPose.x;
	odopose.y = sin(initialPose.theta) * xPos + cos(initialPose.theta) * yPos + initialPose.y;
	odopose.theta = initialPose.theta + yaw;

	rod->odometryPose = odopose;
	rod->estimatedPose = odopose;

	rod->leftStalled = rod->rightStalled = 0; // USARSIM has no stall values
	rod->speed = mySpeed;
	rod->jog   = myJog;
	rod->ipc_hostname  = "USARPosition";

	rod->timestamp.setSeconds(timeact);
	rod->ipc_timestamp = rod->timestamp;

	session->queuePush(PROPERTY_ROBOT_DATA_QUEUE, rod);

	//RDK_DEBUG_PRINTF("QUAD HERE??");
	
	session->setPose(PROPERTY_ROBOT_ODOMETRY_POSE, odopose);
	if (session->getBool(PROPERTY_COPY_ODOMETRY)) {
		session->setPose(PROPERTY_ROBOT_ESTIMATED_POSE, odopose);
	}
}


void UsarSimClientModule::recordNewRFID(RUsarRfidData* rfd) {
	double locx, locy, locz;
	locx = rfd->location.x;
	// Resetting locx, locy and locz is a dirty trick. But it's
	// needed to keep this function separate. (Arrigo)
	locy = -rfd->location.y;
	locz = rfd->location.z - session->getDouble(PROPERTY_START_Z);
	// Load values into repository
	// session->setString(PROPERTY_RFID_READING_DATA, data);
	// Build the entry for the map
	// !!!!!!!!!!!!!!!! SEND rfd TO REMOTE HOST  !!!!!!!!!!!!!!!!
	if(session->getBool(PROPERTY_REMOTE_WRITE)){
		string mapper=session->getString(PROPERTY_REMOTE_MAPPER);
		string myname=session->getRepositoryName();
		RNetMessage* message=new RNetMessage(mapper,
						     myname,
						     RNetMessage::MAPPERDATA,
						     Network::TCP_IP,
						     rfd->clone()); 
		session->queuePush(PROPERTY_OUTBOX, message);
// 		RDK_DEBUG_PRINTF("Remote RFID Inserted");
	}
	
	session->lock(PROPERTY_RFID_TAGS, HERE);
		
	RItemOnMapVector* v = 
		session->getObjectAsL<RItemOnMapVector>(PROPERTY_RFID_TAGS);
	Point2od estimated_pose =
		session->getPose(PROPERTY_ROBOT_ESTIMATED_POSE);
	double robot_theta = estimated_pose.theta;
	double robot_x=estimated_pose.x;
	double robot_y=estimated_pose.y;
	double robot_z=session->getDouble(PROPERTY_START_Z);
	double tagAbsX=robot_x + locx*cos(robot_theta) + 
		locy*sin(robot_theta);
	double tagAbsY=robot_y + locx*sin(robot_theta) - 
		locy*cos(robot_theta);
	double tagAbsZ=robot_z+locz;
	
	RfidTagOnMap* newTag = new RfidTagOnMap(tagAbsX, tagAbsY, tagAbsZ,
						RGB_RED, 0.5, rfd->id);

	bool alreadySeen = false;
	for (size_t i = 0; i < v->size(); i++) {
		RfidTagOnMap* rf = dynamic_cast<RfidTagOnMap*>((*v)[i]);
		if (rf && rf->id == rfd->id) {
			alreadySeen = true;
			break;
		}
	}
 
	if (!alreadySeen) {
		RDK_DEBUG_STREAM("Adding RfidTAG " << newTag->id << " in position "
				 << newTag->x << ", " << newTag->y << ", " << newTag->z);

		// XXX ask jamaiko if this is ok
		if (session->getBool(PROPERTY_RFID_WRITE_FILE)) {
			rfidStream << newTag->id << ", " << newTag->x
				   << ", " << newTag->y
				   << ", " << newTag->z << std::endl;
		}
		v->push_back(newTag);
	}

	session->unlock(PROPERTY_RFID_TAGS);
	session->valueChanged(PROPERTY_RFID_TAGS);
	session->queuePush(PROPERTY_ROBOT_DATA_QUEUE, rfd);
}

/**
 * Example line: SEN {Type RFID} {Name RFIDReader} {ID 118} 
 *                   {Location -0.09,-0.00,0.04} {Mem 0} {ID ...} ...
 */
void UsarSimClientModule::parseRfidTagData(char* pBody){
	string Name, data;
	double locx, locy, locz;
	int id = 0;
	string line(pBody), temp1, temp2, backup;
	char junk;
	double timeact;
	struct timeval tv;
	RUsarRfidData* rfd = NULL;
	// closeRFIDs will contain all the RUsarRfidData objects we
	// are creating now
	Vector<RUsarRfidData> *closeRFIDs = new Vector<RUsarRfidData>;

	// RDK_DEBUG_STREAM("RFID Tag received " << pBody << " parsing...");
	// Sanity check
	if (!getNextSegmentContent(line, temp1, temp2) || (temp1 != "Type") || 
	    (temp2 != "RFID" )) {
		RDK_ERROR_STREAM("Bad RFID tag received: " << pBody);
		return;
	}
	
	// RFID tags don't provide time information any more!
// 	bool useUsarSimTimestamps = 
// 		session->getBool(PROPERTY_USE_USARSIM_TIMESTAMPS);
// 	if (useUsarSimTimestamps) {
// 		//RDK_DEBUG_STREAM("waiting for time");
// 		if (us_get_value2(pBody,"Time",tmp)==-1) return;
// 		double time = atof(tmp);
// 		//RDK_DEBUG_STREAM("got time");
// 		timeact = time;
// 	} else {
	gettimeofday(&tv, NULL);
	timeact = tv.tv_sec + 0.000001 * tv.tv_usec;
	// Read Name (another sanity check)
	if (!getNextSegmentContent(line, temp1, Name) || (temp1 != "Name")) {
		RDK_DEBUG_STREAM("error reading RFID name!!!!!!!");
		return;
	}
	// Read all IDs and their information
	while (getNextSegmentContent(line, temp1, temp2)) {
		if ((temp1 == "Location") && 
		    session->getBool(PROPERTY_USE_RFID_LOCATION)) {
			istringstream loc;
			loc.str(temp2);
			// Commas are junk
			loc >> locx >> junk >> locy >> junk >> locz;
			rfd->location.x = locx;
			// JMK: Remeber USARSim has a rotated
			// reference system
			rfd->location.y = -locy; 
			//JMK so I have the real z in the reading:
			rfd->location.z = locz + 
				session->getDouble(PROPERTY_START_Z);
		} else if (temp1 == "Mem") {
			// We don't support this
			RDK_ERROR_STREAM("Warning: RFIDS are sending data "
					 "without request!");
 		} else if (temp1 == "ID") { // This starts RFID info
			istringstream iss;
			iss.str(temp2);
			iss >> id;
			if (rfd != NULL) { // Another RFID in the
					   // list. Save the previous.
				closeRFIDs->push_back((RUsarRfidData *)
						      rfd->clone());
				recordNewRFID(rfd);
			}
			rfd = new RUsarRfidData();
			rfd->ipc_hostname=session->getRepositoryName();
			rfd->timestamp.setSeconds(timeact);
			rfd->name = Name;
			rfd->id = id;
		}
	} // Cycle on all RFID IDs
	if (rfd != NULL) { // We have the last RFID read to be saved
		closeRFIDs->push_back((RUsarRfidData *)
				      rfd->clone());
		recordNewRFID(rfd);
	}
	// Now, let's store the list of close RFIDs
	session->lock(PROPERTY_RFID_DETECTED_IDS, HERE);
	session->setObject(PROPERTY_RFID_DETECTED_IDS, closeRFIDs);
	session->unlock(PROPERTY_RFID_DETECTED_IDS);
}

#if 0
void UsarSimClientModule::parseVictimRfidData(char* pBody){
	//RDK_DEBUG_STREAM("Vict RFID received " << pBody << " parsing data...");

	char tmp[128];

	char Name[10] = "unset";
	char id[80] = "unset";
	char status[80] = "unset";
	float locx, locy, locz=0.0;

	//RDK_DEBUG_STREAM("RFID Tag received " << pBody << " parsing...");

	
	bool useUsarSimTimestamps = session->getBool(PROPERTY_USE_USARSIM_TIMESTAMPS);

	double timeact;
	if (useUsarSimTimestamps) {
		//RDK_DEBUG_STREAM("waiting for time");
		double time = -1.;
		if (us_get_value2(pBody,"Time",tmp)!=-1){
			time = atof(tmp);
		}
		//RDK_DEBUG_STREAM("got time");
		timeact = time;
	} else {
		struct timeval tv;
		gettimeofday(&tv, NULL);
		timeact = tv.tv_sec + 0.000001 * tv.tv_usec;
	}

	//x and y are exchanged
	if (us_get_value2(pBody, "VictimName", tmp)==-1){ RDK_DEBUG_STREAM("error reading RFID name!!!!!!!!"); return;}
	strncpy(Name,tmp,sizeof(Name));

	if (us_get_value2(pBody, "VictimId", tmp)==-1){ RDK_DEBUG_STREAM("error reading RFID name!!!!!!!!"); return;}
	strncpy(id,tmp,sizeof(id));

	if (us_get_value2(pBody, "VictimStatus", tmp)==-1){ RDK_DEBUG_STREAM("error reading RFID name!!!!!!!!"); return;}
	strncpy(status,tmp,sizeof(status));

	if (us_get_value2(pBody, "Location", tmp)==-1){ RDK_DEBUG_STREAM("error reading RFID name!!!!!!!!"); return;}
	sscanf(tmp,"%f,%f,%f",&locx,&locy,&locz);

// 	RDK_DEBUG_STREAM("Got Victim RFID value= Time " << timeact << " Name " << Name << " ID " << id << " State " << status << " 		Location " << locx << ", "<< locy << ", "<< locz);

	RDK_DEBUG_PRINTF("%s", pBody);
	RDK_DEBUG_PRINTF("Reading RFID Victim data: name = '%s', id = '%s', status = '%s', location = %.2f %.2f %.2f",
		Name, id, status, locx, locy, locz);
	
	Point2od estimated_pose=session->getPose(PROPERTY_ROBOT_ESTIMATED_POSE);
	
	double reliability = -1.; 
	char relstr[3];
	char name[10];
	if (Name[0] >= '0' && Name[0] <= '9') {
		if (Name[0] == '1'){
			sprintf(relstr,"%c%c\n",Name[1],Name[2]);
			reliability = atof(relstr);
			sprintf(name,"%c%c",Name[3],Name[4]);
		} else if (Name[1] == '0'){
			sprintf(relstr,"%c%c\n",Name[1],Name[2]);
			reliability = 100 - atof(relstr);	
			sprintf(name,"%c%c",Name[3],Name[4]);
		}
		else if (Name[0] == '2' || Name[0] == '3') {
			strcpy(name, "Fire");
			reliability = 100;
		}
	}
	else {
		strcpy(name, Name);
		reliability = 100;
	}
	
// 	RDK_DEBUG_STREAM(" reliability: string = " << relstr << " val = " << reliability);	
/*	if (!strcmp(Name,"VictRFID")){
		RDK_DEBUG_STREAM("Wrong Data Format... returning");
		return;
	}*/
	
	RUsarVictimRfidData* rfd = new RUsarVictimRfidData();
	rfd->ipc_hostname=session->getRepositoryName();
	rfd->timestamp.setSeconds(timeact);
	rfd->name = name;
	rfd->id = id;
	rfd->status = status;
 	rfd->reliability = reliability/100;
	rfd->location.x = locx;
	rfd->location.y = -locy; //JMK: Remeber USARSim has a rotated reference system
	rfd->location.z = locz+session->getDouble(PROPERTY_START_Z); //JMK so I have the real z in the reading
	Point2d ptmp = rfd->location.rot(estimated_pose.theta);
	ptmp = ptmp + estimated_pose;
	rfd->location.x = ptmp.x;
	rfd->location.y = ptmp.y;
	
	// check if it should be filtered for FOV or max distance
	double victimSensorFov = session->getDouble(PROPERTY_VICTIM_RFID_SENSOR_FOV);
	double victimSensorMaxDist = session->getDouble(PROPERTY_VICTIM_RFID_SENSOR_MAX_DIST);	
	double pan = session->getDouble(PROPERTY_PANTILT_PAN);
	bool shouldBeFilteredOut = false;
	const Point2od& robotPose = estimated_pose;
	if (robotPose.distTo(rfd->location) > victimSensorMaxDist) shouldBeFilteredOut = true;
	else {
		double rfidAngleWithPan = angDiff(robotPose.thetaWith(rfd->location), robotPose.theta);
		rfidAngleWithPan = angDiff(rfidAngleWithPan, pan);
		if (fabs(rfidAngleWithPan) > victimSensorFov / 2) shouldBeFilteredOut = true;
	}
	if (rfd->name == "Fire") shouldBeFilteredOut = false;
	
	if (rfd->status != "None"){
		//RDK_DEBUG_STREAM("Searching for id: " << rfd->id);
		//Now we are sure about the RFID information
		rfidIt rit = rfidMap.find(std::string(rfd->name)); 
		if ( rit == rfidMap.end()){
			//found a new rfid with status
			double res = (rand()%10000)/100.;
			if (!session->getBool(PROPERTY_VICTIM_RFID_SENSOR_USE_RELIABILITY)) res = 0.;
			RDK_DEBUG_STREAM("");
			RDK_DEBUG_STREAM("*****************************************************************");
			RDK_DEBUG_STREAM(" name" << rfd->name << "reliability: string = " << relstr << " val = " << rfd->reliability*100 << " random number " << res);	
			if (res > rfd->reliability*100){
				if (rfd->id != "FalsePositive"){
					rfd->id = "FalsePositive";
					rfd->status = "FalsePositive";
				} else {
					rfd->id = "FakePippo";
					rfd->status = "BlamingGod";
				}	
			}
			rfd->reliability = 1.;
			RDK_DEBUG_STREAM("discovered new RFID: Name " << rfd->name << " ID " << rfd->id << " State " << rfd->status << " reliability " << rfd->reliability*100 << " random value " << res );
			rfidMap.insert(make_pair(rfd->name,*rfd));	
			RDK_DEBUG_STREAM("*****************************************************************");
			RDK_DEBUG_STREAM("");
		} else {
/*			RDK_DEBUG_STREAM("returnig known RFID before updating status and id: Name " << rfd->name << " ID " << rfd->id << " State " << rfd->status << " reliability " << rfd->reliability);*/
			rfd->status = rit->second.status;
			rfd->id = rit->second.id;
/*			RDK_DEBUG_STREAM("returnig known RFID after updating status and id: Name " << rfd->name << " ID " << rfd->id << " State " << rfd->status << " reliability " << rfd->reliability);*/
		}
	}
			
	//drawing current victim on map
	session->lock(PROPERTY_CURRENT_VICTIM_VIEW, HERE);
	RItemOnMapVector* iomv = session->getObjectAsL<RItemOnMapVector>(PROPERTY_CURRENT_VICTIM_VIEW);
	iomv->clear();
	iomv->push_back(new RLine2dOnMap(rfd->location.x - 0.25, rfd->location.y - 0.25,
		rfd->location.x + 0.25, rfd->location.y - 0.25, RGB_ORANGE, 2.));
	iomv->push_back(new RLine2dOnMap(rfd->location.x + 0.25, rfd->location.y - 0.25,
		rfd->location.x + 0.25, rfd->location.y + 0.25, RGB_ORANGE, 2.));
	iomv->push_back(new RLine2dOnMap(rfd->location.x + 0.25, rfd->location.y + 0.25,
		rfd->location.x - 0.25, rfd->location.y + 0.25, RGB_ORANGE, 2.));
	iomv->push_back(new RLine2dOnMap(rfd->location.x - 0.25, rfd->location.y + 0.25,
		rfd->location.x - 0.25, rfd->location.y - 0.25, RGB_ORANGE, 2.));
	session->unlock(PROPERTY_CURRENT_VICTIM_VIEW);
	
	
	const std::string lastid = session->getString(PROPERTY_VICTIM_NAME);
	const std::string newid = rfd->id;
        if (lastid.compare(newid)) session->setString(PROPERTY_VICTIM_NAME, newid);	

	double victimAbsX=0.;
	double victimAbsY=0.;
	double victimAbsZ=0.;
	
	//putting victims on the map item vector
	if (newid.compare("FalsePositive") && newid.compare("None")) {
		session->lock(PROPERTY_ITEMS, HERE);
			RItemOnMapVector* v = session->getObjectAsL<RItemOnMapVector>(PROPERTY_ITEMS);
			bool found= false;
			bool changed = false;
			const std::string status = rfd->status;
			
			RItemOnMapVector& featureVector = *v;
			for(RItemOnMapVector::iterator it =featureVector.begin();((it!=featureVector.end())&&(!found)); it++){
			//for( int i = 0; i < featureVector.size(); i++ )
				RVictimOnMap* cvom = dynamic_cast<RVictimOnMap*>(*it);
				if(cvom){
					const std::string actualVictim = cvom->idAI;
					if (actualVictim.compare(newid) == 0){
						
						victimAbsX=cvom->pose.x;
						victimAbsY=cvom->pose.y;
						victimAbsZ=cvom->zPosition;						
						//RDK_DEBUG_STREAM("Found victim: " << actualVictim);
						const std::string actualstatus = cvom->statusAI;
						
						if ((!status.empty())&&(actualstatus.empty()||(status.compare("None")&&
						status.compare(actualstatus)))) {
							                                                        
							cvom->statusAI = status;
							//cvom->color = RGB_GREEN;
							changed = true;
							
							RDK_DEBUG_STREAM("Found victim: " << cvom->idAI);
							RDK_DEBUG_STREAM("Updated status to: " << cvom->statusAI);
							
							//session->setBool(PROPERTY_SCREENSHOT_VICTIM, true);

							// !!!!!!!!!!!!!!!!!!!! SEND rfd TO REMOTE HOST  !!!!!!!!!!!!!!!!!!!!
							if(session->getBool(PROPERTY_REMOTE_WRITE)){
								string mapper=session->getString(PROPERTY_REMOTE_MAPPER);
								string myname=session->getRepositoryName();
								RNetMessage* message=new RNetMessage(mapper,
												myname,
												RNetMessage::MAPPERDATA,
												Network::TCP_IP,
												rfd->clone()); 
								session->queuePush(PROPERTY_OUTBOX, message);
// 								RDK_DEBUG_PRINTF("Remote VictimRFID Inserted");
							}

						} //else RDK_DEBUG_STREAM("Confirming status to: " << actualstatus);
						found= true;
					}
				}
			}
			
			if (!found){
 				
				RVictimOnMap* newVictim = new RVictimOnMap();

				newVictim->idAI = newid;
				newVictim->statusAI = status;
				
				Point2od estimated_pose=session->getPose(PROPERTY_ROBOT_ESTIMATED_POSE);

				double robot_theta = estimated_pose.theta;
				double robot_x=estimated_pose.x;
				double robot_y=estimated_pose.y;
				double robot_z=session->getDouble(PROPERTY_START_Z);
				victimAbsX=robot_x + locx*cos(robot_theta) + locy*sin(robot_theta);
				victimAbsY=robot_y + locx*sin(robot_theta) - locy*cos(robot_theta);
				victimAbsZ=robot_z+locz;
					
				newVictim->pose=Point2od(victimAbsX,victimAbsY,0.);
				newVictim->zPosition=victimAbsZ;
				if (!status.empty()){
					//if (status.compare("None")==0) newVictim->color=RGB_ORANGE;
					//else newVictim->color=RGB_GREEN;  
				}
				RDK_DEBUG_STREAM("Introducing victim " << newVictim->idAI << " at status " 
					<< newVictim->statusAI << " in position " << newVictim->pose.x 
					<< " , " << newVictim->pose.y << " , " << newVictim->zPosition);

				v->push_back(newVictim);
				changed=true;

				// !!!!!!!!!!!!!!!!!!!! SEND rfd TO REMOTE HOST  !!!!!!!!!!!!!!!!!!
				if(session->getBool(PROPERTY_REMOTE_WRITE)){
					string mapper=session->getString(PROPERTY_REMOTE_MAPPER);
					string myname=session->getRepositoryName();
					RNetMessage* message=new RNetMessage(mapper,
									myname,
									RNetMessage::MAPPERDATA,
									Network::TCP_IP,
									rfd->clone()); 
					session->queuePush(PROPERTY_OUTBOX, message);
// 					RDK_DEBUG_PRINTF("Remote RFID Inserted");
				}


			}
				
// 		if (changed) logVictim(v);
		session->unlock(PROPERTY_ITEMS);			
		
		if (changed) session->valueChanged(PROPERTY_ITEMS);	

		session->setDouble(PROPERTY_VICTIM_X, victimAbsX);
		session->setDouble(PROPERTY_VICTIM_Y, victimAbsY);
		session->setDouble(PROPERTY_VICTIM_Z, victimAbsZ);	
	}
	
// 	RDK_DEBUG_STREAM("Inserting RFID value=  Name " << rfd->name << " ID " << rfd->id << " State " << rfd->status << " reliability " << rfd->reliability);
	if (shouldBeFilteredOut) {
/*		RDK_DEBUG_PRINTF("The RFID is not within the FOV and Max Distance of the sensor: "
			"it will be filtered out");*/
	}
	else session->queuePush(PROPERTY_ROBOT_DATA_QUEUE, rfd);
}
#endif

// FIXME: this old function, parsing INU data and not INS, may be useless. (Arrigo)
// void UsarSimClientModule::parseInuData(char* pBody){
// //	RDK_DEBUG_STREAM("INU Data received " << pBody << " parsing data...");

// 	char tmp[128];

// 	char Name[10] = "unset";
// 	float orx, ory, orz=0.0;

	
// 	//RDK_DEBUG_STREAM("INU Data time received");
// 	bool useUsarSimTimestamps = session->getBool(PROPERTY_USE_USARSIM_TIMESTAMPS);

// 	double timeact;
// 	if (useUsarSimTimestamps) {
// 		//RDK_DEBUG_STREAM("waiting for time");
// 		if (us_get_value2(pBody,"Time",tmp)==-1) return;
// 		double time = atof(tmp);
// 		//RDK_DEBUG_STREAM("got time");
// 		timeact = time;
// 	} else {
// 		struct timeval tv;
// 		gettimeofday(&tv, NULL);
// 		timeact = tv.tv_sec + 0.000001 * tv.tv_usec;
// 	}

// 	//x and y are exchanged
// 	if (us_get_value2(pBody, "Name", tmp)==-1){ RDK_DEBUG_STREAM("error reading RFID name!!!!!!!!"); return;}
// 	strncpy(Name,tmp,sizeof(Name));

// 	if (us_get_value2(pBody, "Orientation", tmp)==-1){ RDK_DEBUG_STREAM("error reading RFID name!!!!!!!!"); return;}
// 	sscanf(tmp,"%f,%f,%f",&orx,&ory,&orz);

// //	RDK_DEBUG_STREAM("Got INU Data = Time " << timeact << " Name " << Name << " Location " << orx << ", "<< ory << ", "<< orz);
// 	RUsarInuData* rid = new RUsarInuData();
// 	rid->timestamp.setSeconds(timeact);
// 	rid->name = Name;
// 	rid->location.x = orx;
// 	rid->location.y = ory;
// 	rid->location.z = orz;
// 	session->queuePush(PROPERTY_ROBOT_DATA_QUEUE, rid);
// }

void UsarSimClientModule::parseInuData(char* pBody){
	return;
	string sensorName;
	float loc_x, loc_y, loc_z;	 // location coordinates
	float or_x, or_y, or_z;		 // orientation coordinates
	char tmp[200];
	struct timeval tv;
	double timeact;
	istringstream *loc_tmp, *or_tmp; // temp input stream
	char junk;
	Point3d position;	
	loc_x=0.0, loc_y=0.0, loc_z=0.0;
	or_x=0.0, or_y=0.0, or_z=0.0;	

  	//RDK_DEBUG_STREAM("INS Data received " << pBody << ". Parsing data...");
	// Old INU sensors gave also time information. INS sensors
	// don't, so we use our system clock.
	gettimeofday(&tv, NULL);
	timeact = tv.tv_sec + 0.000001 * tv.tv_usec;
	//x and y are exchanged (FIXME: check if they really are)
	
	// reading INS sensor <Name> value	
	if (us_get_value2(pBody, "Name", tmp) == -1) { 
		RDK_ERROR_STREAM("error reading INU sensor name!!!!!!!!"); 
		return;
	}
	sensorName = tmp;
	
	// reading INS sensor <Location> values (loc_x, loc_y, loc_z)
	if (us_get_value2(pBody, "Location", tmp) == -1) { 
		RDK_ERROR_STREAM("error reading location!!!!!!!!"); 
		return;
	}
	loc_tmp = new istringstream(tmp);
	*loc_tmp >> loc_x >> junk >> loc_y >> junk >> loc_z;	// commas are junk
	delete loc_tmp;
	
	// reading INS sensor <Orientation> values (or_x, or_y, or_z)
	if (us_get_value2(pBody, "Orientation", tmp) == -1) { 
		RDK_ERROR_STREAM("error reading orientation!!!!!!!!"); 
		return;
	}
	or_tmp = new istringstream(tmp);
	*or_tmp >> or_x >> junk >> or_y >> junk >> or_z;	// commas are junk
	delete or_tmp;
	
	RUsarInuData* rid = new RUsarInuData();
	rid->timestamp.setSeconds(timeact);
	rid->name = sensorName;
	rid->location.x = loc_x;
	rid->location.y = -loc_y;
	rid->location.z = -loc_z;
	position.x = loc_x;
	position.y = -loc_y;
	position.z = -loc_z;
	
	// write extimeted coordinates in repository
	session->setDouble(PROPERTY_ROBOT_CURRENT_X,position.x);
	session->setDouble(PROPERTY_ROBOT_CURRENT_Y,position.y);
	session->setDouble(PROPERTY_ROBOT_CURRENT_Z,position.z);
	
	// Set also the local property
	rid->location.theta = -or_z;
	rid->location.phi   = -or_y;
	rid->location.gamma = or_x;

	session->setDouble(PROPERTY_ROBOT_YAW,  rid->location.theta);

	session->setDouble(PROPERTY_INS_ROLL, rid->location.gamma);
	session->setDouble(PROPERTY_INS_PITCH, rid->location.phi);

	session->setPose3(PROPERTY_ROBOT_POSE3, Point3od(position.x, position.y, position.z, rid->location.theta, rid->location.phi, rid->location.gamma) );
	session->setPose(PROPERTY_ROBOT_POSE_2D, Point2od(position.x, position.y, rid->location.gamma));
	session->setObject(PROPERTY_INU_SENSORDATA, (RUsarInuData *) rid->clone());
	session->queuePush(PROPERTY_ROBOT_DATA_QUEUE, rid); 
	//RDK_DEBUG_PRINTF("%.2f %.2f %.2f", orx, ory, orz);
}


void UsarSimClientModule::parseBallData(char* pBody)
{
/*	double timeact;
	if (session->getBool(PROPERTY_USE_USARSIM_TIMESTAMPS)) {
		if (us_get_value2(pBody,"Time",tmp)==-1) return;
		double time = atof(tmp);
		timeact = time;
	}
	else {
		struct timeval tv;
		gettimeofday(&tv, NULL);
		timeact = tv.tv_sec + 0.000001 * tv.tv_usec;
	}
*/
	char tmp[128];
	
	char name[128] = "unset";
	if (us_get_value2(pBody, "Name", tmp) == -1)
		{ RDK_DEBUG_STREAM("error reading Ball name!!!!!!!!"); return; }
	strncpy(name,tmp,sizeof(name));

	float x, y, z;
	if (us_get_value2(pBody, "Pos3D", tmp)==-1)
		{ RDK_DEBUG_STREAM("error reading ball position!!!!!!!!"); return; }
	sscanf(tmp,"%f,%f,%f",&x,&y,&z);

	bool ballVisible;
	if (us_get_value2(pBody, "Visible", tmp)==-1)
		{ RDK_DEBUG_STREAM("error reading visibility of the ball"); return; }
	ballVisible = (strncmp(tmp, "true", 4)==0);

	// note: y is flipped, damned usarsim! FIXME!
	session->setPose(PROPERTY_BALL_POSITION, Point2od(x, -y-2.5, DBL_MAX));
	session->setBool(PROPERTY_BALL_VISIBLE, ballVisible);
}

/* Example line:
   SEN {Type Encoder} {Name Encoder1 Tick deg1} {Name Encoder2 Tick deg2} ...
*/
void UsarSimClientModule::parseEncoderData(char* pBody){
	string sensorName, tmp;
	double value;	
	string tag, content, data = pBody;
//  	RDK_DEBUG_STREAM("Encoder data received " << pBody << 
// 			 ". Parsing data...");
	while (getNextSegmentContent(data, tag, content)) {
		if (tag == "Name") {
			istringstream tmp2(content);
			tmp2 >> sensorName;
// 			RDK_DEBUG_PRINTF("Encoder name: %s",sensorName.c_str());
			tmp2 >> tmp;
			if (tmp != "Tick") {
				RDK_ERROR_PRINTF("Strange encoder data: %s",
						 pBody);
				return;
			}
			tmp2 >> value;
// 			RDK_DEBUG_PRINTF("Got encoder data = Name %s, "
// 					 "Reading: %f degrees", 
// 					 sensorName.c_str(), value);
			if (sensorName.compare("FFlipperEncoder") == 0)
				session->setDouble(PROPERTY_ROBOT_FRONT_FLIPPER_POSITION,
						   deg2rad(value));
			else if (sensorName.compare("RFlipperEncoder") == 0)
				session->setDouble(PROPERTY_ROBOT_REAR_FLIPPER_POSITION,
						   deg2rad(value));
			else {
			//	RDK_INFO_PRINTF("Unknown encoder: %s", sensorName.c_str());
			}
		} else if (tag == "Type") {
			// It's ok
		} else {
			RDK_ERROR_PRINTF("Invalid tag (%s) in encoder data: %s",
					 tag.c_str(), pBody);
		}
	} // while getNextSegmentContent()
}

/* 
   Example line: RES {Time ...} {Type RFIDReleaser} {Name Gun} {status OK}
*/
void UsarSimClientModule::parseRfidReleaserData(char *pBody) {
	string message = pBody;
	if (TextUtils::endsWith(message, "OK}\r\n")) {
		RDK_INFO_STREAM("RFID dropped succesfully");
		session->setBool(PROPERTY_LAST_RFID_DROP_SUCCESSFUL, true);
	} else {
		RDK_INFO_STREAM("RFID not dropped! :-(");
		session->setBool(PROPERTY_LAST_RFID_DROP_SUCCESSFUL, false);
	}
	waitingForRFIDResult = false;
	session->setBool(PROPERTY_CMD_DROP_RFID, false);
}

/* 
   Example line: RES {Time ...} {Type RFID} {Name RFID} {status OK} 
                     {ID RFIDTagID} {Mem MemoryContent} (read ok)
		     
		 RES {Time ...} {Type RFID} {Name RFID} {status OK} 
		                                        (write ok)

		 RES {Time ...} {Type RFID} {Name RFID} {status Failed} 
		                                        (read/write error)

*/
void UsarSimClientModule::parseRfidDataResult(char *pBody) {
	istringstream *iss = new istringstream(pBody);
	string temp1="", temp2;
	// Find our status
	while (temp1 != "{status")
		*iss >> temp1 >> temp2;
	if (temp2 == "OK}") { // Reading or writing is succesful!
		*iss >> temp1 >> temp2;
		if (temp1 == "{ID") { // We have a succesful reading
			*iss >> temp1; // Reads {Mem
			temp2 = "";
			// Ugly parsing: reads until '}'. This is
			// required because the data may contain
			// spaces
			while(temp1[temp1.length() - 1] != '}') {
				*iss >> temp1;
				temp2 += temp1 + " ";
			}
			// Take away the last " }"
			temp1 = temp2.substr(0, temp2.length() - 2);
			session->setString(PROPERTY_RFID_READING_DATA, temp1);
			// RDK_INFO_STREAM("RFID read succesfully");
			session->setBool(PROPERTY_LAST_RFID_READ_SUCCESSFUL, 
					 true);
			session->setBool(PROPERTY_CMD_READ_RFID, false);
		} else { // We have a succesful writing
			// RDK_INFO_STREAM("RFID written succesfully");
			session->setBool(PROPERTY_LAST_RFID_WRITE_SUCCESSFUL, 
					 true);
			session->setBool(PROPERTY_CMD_WRITE_RFID, false);
		}
	} else { // Reading or writing is unsuccesful!
		// Too bad, we can't distinguish. So, we set the one
		// who has the CMD set.
		if (session->getBool(PROPERTY_CMD_READ_RFID)) { // Reading
			// RDK_INFO_STREAM("RFID not read! :-(");
			session->setBool(PROPERTY_LAST_RFID_READ_SUCCESSFUL, 
					 false);
			session->setBool(PROPERTY_CMD_READ_RFID, false);
		} else { // Writing
			// RDK_INFO_STREAM("RFID not written! :-(");
			session->setBool(PROPERTY_LAST_RFID_WRITE_SUCCESSFUL, 
					 false);
			session->setBool(PROPERTY_CMD_WRITE_RFID, false);
		}
	}
	waitingForRFIDResult = false;
	delete iss;
}

/* Example line:
   SEN {Type Touch} {Name Sensor1 Touch False} {Name Sensor2 Touch False} ...

   TODO: add support for USARSim-provided timestamps
*/
void UsarSimClientModule::parseTouchSensorData(const char* pBody) {
	string s(pBody), tag, content;
	struct timeval tv;
	double timeact;
	RTouchSensorData *data = new RTouchSensorData;
	// Prepare locally-generated timestamp
	gettimeofday(&tv, NULL);
	timeact = tv.tv_sec + 0.000001 * tv.tv_usec;
	data->timestamp.setSeconds(timeact);
	while (getNextSegmentContent(s, tag, content)) {
		if (tag == "Name") {
			string sensorName, temp, sensorData;
			bool sensorStatus;
			istringstream iss(content);
			iss >> sensorName >> temp >> sensorData;
			if (temp == "Touch") { // Sanity check
				sensorStatus = (sensorData != "False");
				data->sensors[sensorName] = sensorStatus;
				// FIXME
				if (session->propertyExists("touch/" + sensorName)) {
					session->setBool("touch/" + sensorName, sensorStatus);
				}
//  				RDK_INFO_STREAM("Touch: " << sensorName <<
//  						" is " << sensorStatus);
			} else {
				RDK_ERROR_STREAM("Strange output from touch "
						 "sensor: " << pBody);
			}
		} else if (tag == "Type") {
			// ignore it
		} else {
			RDK_ERROR_STREAM("Unknown tag: " << tag << " in touch "
					 "sensor data: " << pBody);
		}
	} // cycle all tags
	// In the end, all sensor readings must go in the queue
//	RDK_INFO_STREAM("Touch sensors: " << data->getStringForVisualization());
	session->queuePush(PROPERTY_ROBOT_DATA_QUEUE, data->clone());
	session->setObject(PROPERTY_TOUCH_SENSORS, data);
}


/* 
	Sonar Parser 

	Example line:
	{Time 497.3063} {Type Sonar} {Name F1 Range 1.4505}
*/
void UsarSimClientModule::parseSonarData(char *pBody) {
	//RDK_DEBUG_STREAM("Sonar Data received " << pBody << ". Parsing data...");
	RSonarData* rSonarData;
	struct timeval tv;
	double timeact;
	string s(pBody), tag, content;
	SonarDevice sd;	
	
	/* prepare locally-generated timestamp */
	gettimeofday(&tv, NULL);
	timeact = tv.tv_sec + 0.000001 * tv.tv_usec;
	rSonarData = new RSonarData();
	rSonarData->timestamp.setSeconds(timeact);

	/* parsing couples name-value */
	while (getNextSegmentContent(s, tag, content)) {
			
		if (tag == "Name") {
			string sensorName, temp, sensorData;
			double sensorValue;
			istringstream iss(content);
			iss >> sensorName >> temp >> sensorData;
			sd.name = sensorName;	// adding name info into Device object

			if (temp == "Range") { // Sanity check
				istringstream converted(sensorData);
				converted >> sensorValue;
				sd.reading = sensorValue;	// adding value info into Device object
				//RDK_DEBUG_PRINTF(sd.name);
				//RDK_DEBUG_PRINTF("Sonar value: %f",sd.reading);
			} else {
				RDK_ERROR_STREAM("Sonar Error on reading");
			}
		}
		/* adding device */
		rSonarData->devices.push_back(sd);
	}
	/* store property */
	session->lock(PROPERTY_SONAR_DATA, HERE);
	session->setObject(PROPERTY_SONAR_DATA, rSonarData);
	session->unlock(PROPERTY_SONAR_DATA);
}


}} // Namespaces
