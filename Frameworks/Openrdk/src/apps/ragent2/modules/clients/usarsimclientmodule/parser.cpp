#include "usarsimclientmodule.h"

#include <rdkcore/rsensordata/rtouchsensordata.h>

#include <float.h>
#include <errno.h>

namespace RDK2 { namespace RAgent {

bool readDataFromServer(int sock, string& data)
{
	char c = 0;
	size_t readBytes = 0;
	data = "";
	while (c != '\n') {
		int r = read(sock, &c, 1);
		if (r == 0) return false;
		else if (r == -1) {
			if (errno != EAGAIN) {
				RDK_ERROR_PRINTF("Error while reading from the server: (%d)", errno);
				return false;
			}
		}
		else {
			data = data + string(1, c);
			readBytes += r;
		}
	}
	while (data.size() && (data[data.size()-1] == '\n' || data[data.size()-1] == '\r')) data = data.substr(0, data.size()-1);
	return true;
}

bool UsarSimClientModule::getNextSegmentContent(string& s, string& tag, string& content)
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

bool UsarSimClientModule::readFromServer(const string& l)
{
	bool parsedSomething = false;
	string s = l, tag, content;
	string s2 = l;
	// FIXME after thrashing the old functions, we should put here the socket reading stuff
	istringstream iss(l); string dataType; iss >> dataType; s = s.substr(dataType.length());
	if (dataType == "STA") {
		while (getNextSegmentContent(s, tag, content)) {
			if (tag == "Type") session->setString(PROPERTY_VEHICLE_TYPE, content);
			else if (tag == "Battery") session->setInt(PROPERTY_BATTERY_LIFE, atoi(content.c_str()));
			else if (tag == "View") session->setInt(PROPERTY_MULTIVIEW_NUMBER, atoi(content.c_str()));
			else if (tag == "Time") session->setDouble(PROPERTY_USARSIM_TIME, atof(content.c_str()));
			else if (tag == "FrontSteer") {
				// 
			}
		}
		parsedSomething = true;
	}
	else if (dataType == "MISSTA") {
	}
	else if (dataType == "SEN") {
		string type = "";
		while (getNextSegmentContent(s, tag, content)) {
			if (tag == "Type") { type = content; break; }
		}
		if (type == "") { RDK_ERROR_PRINTF("Sensor message without a type: '%s'", l.c_str()); }
		else if (type == "RFID")        { parseRfid(s);                 parsedSomething = true; }
		else if (type == "GroundTruth") { parseGroundTruth(s);          parsedSomething = true; }
		else if (type == "GPS")         { parseGPS(s);                  parsedSomething = true; }
		else if (type == "Touch")       { parseTouch(s);                parsedSomething = true; }
		else if (type == "INS")         { parseIns(s);                  parsedSomething = true; }
		else if (type == "Encoder")     { parseEncoder(s);              parsedSomething = true; }
		//else if (type == "RangeScanner") { parseRangeScanner(s); parsedSomething = true; } FIXME scanmatcher segment fault?
	}
	else if (dataType == "GEO") {
	}
	else if (dataType == "CONF") {
	}
	else if (dataType == "RES") {
	}
	else if (dataType == "NFO") {
	}
	else {
		RDK_ERROR_STREAM("Unknown message type from server: '" << dataType << "' in '" << s2 << "'");
	}
	return parsedSomething;
}

struct TempRfid {
	string id;
	Point3d location;
	double dist;
};

Point3d parse3DLocation(const string& s)
{
	vector<string> v = TextUtils::tokenize(s, ",");
	if (v.size() != 3) {
		RDK_ERROR_PRINTF("Cannot parse 3D location from '%s'", s.c_str());
		return Point3d(0.0, 0.0, 0.0);
	}
	else {
		return Point3d(atof(v[0].c_str()), -atof(v[1].c_str()), -atof(v[2].c_str()));
	}
}

void parse3DOrientation(const string& s, double& roll, double& pitch, double& yaw)
{
	vector<string> v = TextUtils::tokenize(s, ",");
	if (v.size() != 3) {
		roll = 0.0; pitch = 0.0; yaw = 0.0;
		RDK_ERROR_PRINTF("Cannot parse 3D location from '%s'", s.c_str());
	}
	else {
		roll = angNormPiSig(atof(v[0].c_str()));
		pitch = angNormPiSig(-atof(v[1].c_str()));
		yaw = angNormPiSig(-atof(v[2].c_str()));
	}
}

void UsarSimClientModule::parseRfid(string& s)
{
	string tag, content;
	vector<TempRfid> v;
	TempRfid tr;
	while (getNextSegmentContent(s, tag, content)) {
		if (tag == "ID") {
			if (tr.id != "") v.push_back(tr);
			tr.id = content;
		}
		else if (tag == "Location") tr.location = parse3DLocation(content);
		else if (tag == "Distance") tr.dist = atof(content.c_str());
	}
	if (tr.id != "") v.push_back(tr);
	if (v.size() > 0) {
		size_t nearestIdx = 0;
		for (size_t i = 0; i < v.size(); i++) {
			if (v[i].dist < v[nearestIdx].dist) nearestIdx = i;
		}
		if (session->getBool("cmds/logRfidForVito")) {
			ofstream ofs("RFIDs.log", ios_base::out | ios_base::app);
			Timestamp ts;
			ofs << ts.getMsFromMidnight() << " " << v[nearestIdx].id << " "
				<< v[nearestIdx].location.x << " " << v[nearestIdx].location.y << endl;
			session->setBool("cmds/logRfidForVito", false);
		}
	}
}

void UsarSimClientModule::parseRangeScanner(string& s)
{
	string tag, content;
	double fov = 0.0, resolution = 0.0;
	RLaserData* rld = new RLaserData;
	rld->sensorName = "Unknown";
	while (getNextSegmentContent(s, tag, content)) {
		if (tag == "Name") rld->sensorName = content;
		else if (tag == "FOV") fov = atof(content.c_str());
		else if (tag == "Resolution") resolution = atof(content.c_str());
		else if (tag == "Range") {
			size_t numRays = (size_t) (fov / resolution);
			rld->points.resize(numRays+1);
			char buf[10];
			size_t c = 0, r = 0;
			for (size_t i = 0; i <= content.length(); i++) {
				if (i == content.length() || content[i] == ',') {
					buf[c] = '\0';
					if (r >= numRays) {
						RDK_ERROR_PRINTF("There should be %zu rays in the message, but now I'm reading the %d", numRays, r);
						break;
					}
					rld->points[r].reading = atof(buf);
					c = 0;
				}
				else buf[c] = content[i];
			}
		}
	}
	Point2od lp = session->getPose(PROPERTY_ROBOT_LASER_POSE);
	rld->laserPose.x = lp.x;
	rld->laserPose.y = lp.y;
	rld->laserPose.theta = lp.theta;
	//rld->laserPose = session->getPose(PROPERTY_ROBOT_LASER_POSE);
	rld->timestamp.setToNow();	// FIXME
	rld->minReading = session->getDouble(PROPERTY_LASER_MIN_RANGE);	// FIXME
	rld->maxReading = session->getDouble(PROPERTY_LASER_MAX_RANGE);	// FIXME
	rld->minTheta = -fov/2;
	rld->maxTheta = fov/2;
	rld->roll = 0.0;			// FIXME
	rld->pitch = 0.0;			// FIXME
	
	RDK_DEBUG_PRINTF("%s %.2f %.2f %d", rld->sensorName.c_str(), rad2deg(rld->minTheta), rad2deg(rld->maxTheta), rld->points.size());
	
	// FIXME temporary solution for hokuyo
	if (rld->sensorName == session->getString(PROPERTY_HOKUYO_SENSOR_NAME)) {
		session->queuePush(PROPERTY_HOKUYO_DATA_QUEUE, rld->clone());
		session->setObject(PROPERTY_HOKUYO_LASER_DATA, rld);
	}
	else {
		session->queuePush(PROPERTY_ROBOT_DATA_QUEUE, rld->clone());
		session->setObject(PROPERTY_ROBOT_LASER_DATA, rld);
	}
}

void UsarSimClientModule::parseIns(string& s)
{
	if (!session->getBool(PROPERTY_INS_ENABLED)) return;
	string tag, content;
	double insX = 0.0, insY = 0.0, insZ = 0.0, insRoll = 0.0, insPitch = 0.0, insYaw = 0.0;
	while (getNextSegmentContent(s, tag, content)) {
		if (tag == "Location") {
			Point3d p = parse3DLocation(content);
			insX = p.x; insY = p.y; insZ = p.z;
		}
		else if (tag == "Orientation") {
			parse3DOrientation(content, insRoll, insPitch, insYaw);
		}
	}
	session->setDouble(PROPERTY_INS_X, insX);
	session->setDouble(PROPERTY_INS_Y, insY);
	session->setDouble(PROPERTY_INS_Z, insZ);
	session->setDouble(PROPERTY_INS_ROLL, insRoll);
	session->setDouble(PROPERTY_INS_PITCH, insPitch);
	session->setDouble(PROPERTY_INS_YAW, insYaw);
}

void UsarSimClientModule::parseGroundTruth(string& s)
{
	if (!session->getBool(PROPERTY_GROUND_TRUTH_POSE_ENABLED)) return;
	string tag, content;
	Point2od gt;
	while (getNextSegmentContent(s, tag, content)) {
		if (tag == "Location") {
			Point3d p = parse3DLocation(content);
			gt.x = p.x; gt.y = p.y;
		}
		else if (tag == "Orientation") {
			double roll, pitch;
			parse3DOrientation(content, roll, pitch, gt.theta);
			gt.theta = -gt.theta;
		}
	}
	ROdometryData* rod = new ROdometryData();
	rod->timestamp.setToNow();
	rod->ipc_timestamp.setToNow();
	rod->odometryPose = gt;
	rod->estimatedPose = gt;
	session->setPose(PROPERTY_GROUND_TRUTH_POSE, gt);
}

void UsarSimClientModule::parseTouch(string& s)
{
	if (session->getString(PROPERTY_TOUCH_SENSORS_CONFIG) == "") return;
	RTouchSensorData *data = new RTouchSensorData;
	string tag, content;
	while (getNextSegmentContent(s, tag, content)) {
		if (tag == "Name") {
			string sensorName, temp, sensorData;
			istringstream iss(content);
			iss >> sensorName >> temp >> sensorData;
			if (session->propertyExists(PROPERTY_TOUCH_SENSORS_FOLDER + sensorName)
			&& temp == "Touch") {	// sanity check
				data->sensors[sensorName] = (sensorData != "False");
				session->setBool(PROPERTY_TOUCH_SENSORS_FOLDER + sensorName, (sensorData != "False"));
			}
		}
		else if (tag == "Type") {
			// ignore it
		}
		else {
			RDK_ERROR_PRINTF("Unknown tag '%s' in touch sensor data", tag.c_str());
		}
	}
	data->timestamp.setToNow();
	session->queuePush(PROPERTY_ROBOT_DATA_QUEUE, data);
}

void UsarSimClientModule::parseEncoder(string& s)
{
	if (session->getString(PROPERTY_ENCODERS_CONFIG) == "") return;
	string tag, content;
	while (getNextSegmentContent(s, tag, content)) {
		if (tag == "Name") {
			string sensorName, temp, sensorData;
			istringstream iss(content);
			iss >> sensorName >> temp >> sensorData;
			if (session->propertyExists(PROPERTY_ENCODERS_FOLDER + sensorName + PROPERTY_ENCODERS_POS_SUFFIX)
			&& temp == "Tick") {	// sanity check
				int t = atoi(sensorData.c_str());
				double rad = -angNormPiSig(t * 0.01745);		// FIXME can we get it from configuration? is it right to put "-" here?
				session->setDouble(PROPERTY_ENCODERS_FOLDER + sensorName + PROPERTY_ENCODERS_POS_SUFFIX, rad);
			}
		}
		else {
			RDK_ERROR_PRINTF("Unknown tag '%s' in touch sensor data", tag.c_str());
		}
	}
}

/*
* arcLengthOfMeridian (from http://home.hiwaay.net/~taylorc/toolbox/geography/geoutm.html)
*
* Computes the ellipsoidal distance from the equator to a point at a
* given latitude.
*
* Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
* GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
*
* Inputs:
*     phi - Latitude of the point, in radians.
*
* Globals:
*     SM_A - Ellipsoid model major axis.
*     SM_B - Ellipsoid model minor axis.
*
* Returns:
*     The ellipsoidal distance of the point from the equator, in meters.
*
*/
#define SM_A 6378137.0
#define SM_B 6356752.314

double arcLengthOfMeridian(double lat)
{
	/* Precalculate n */
	double n = (SM_A - SM_B) / (SM_A + SM_B);

	/* Precalculate alpha */
	double alpha = ((SM_A + SM_B) / 2.0)
		* (1.0 + (pow(n, 2.0) / 4.0) + (pow(n, 4.0) / 64.0));

	/* Precalculate beta */
	double beta = (-3.0 * n / 2.0) + (9.0 * pow(n, 3.0) / 16.0)
		+ (-3.0 * pow(n, 5.0) / 32.0);

	/* Precalculate gamma */
	double gamma = (15.0 * pow(n, 2.0) / 16.0)
		+ (-15.0 * pow(n, 4.0) / 32.0);

	/* Precalculate delta */
	double delta = (-35.0 * pow(n, 3.0) / 48.0)
		+ (105.0 * pow(n, 5.0) / 256.0);

	/* Precalculate epsilon */
	double epsilon = (315.0 * pow(n, 4.0) / 512.0);

	/* Now calculate the sum of the series and return */
	return alpha
		* (lat + (beta * sin(2.0 * lat))
			+ (gamma * sin(4.0 * lat))
			+ (delta * sin(6.0 * lat))
			+ (epsilon * sin(8.0 * lat)));
}


/*
* mapLatLonToXY (from http://home.hiwaay.net/~taylorc/toolbox/geography/geoutm.html)
*
* Converts a latitude/longitude pair to x and y coordinates in the
* Transverse Mercator projection.  Note that Transverse Mercator is not
* the same as UTM; a scale factor is required to convert between them.
*
* Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
* GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
*
* Inputs:
*    phi - Latitude of the point, in radians.
*    lambda - Longitude of the point, in radians.
*    lambda0 - Longitude of the central meridian to be used, in radians.
*
* Outputs:
*    xy - A 2-element array containing the x and y coordinates
*         of the computed point.
*
* Returns:
*    The function does not return a value.
*
*/

void mapLatLonToXY(double /*phi*/ lat, double /*lambda*/ lon, double /*lambda0*/ lon0, double& x, double& y)
{
	/* Precalculate ep2 */
	double ep2 = (SM_A * SM_A - SM_B * SM_B) / SM_B * SM_B;

	/* Precalculate nu2 */
	double coslat = cos(lat);
	double nu2 = ep2 * coslat * coslat;

	/* Precalculate N */
	double N = (SM_A * SM_A) / (SM_B * sqrt(1 + nu2));

	/* Precalculate t */
	double t = tan(lat);
	double t2 = t * t;

	/* Precalculate l */
	double l = lon - lon0;

	/* Precalculate coefficients for l**n in the equations below
		so a normal human being can read the expressions for easting
		and northing
		-- l**1 and l**2 have coefficients of 1.0 */
	double l3coef = 1.0 - t2 + nu2;

	double l4coef = 5.0 - t2 + 9 * nu2 + 4.0 * (nu2 * nu2);

	double l5coef = 5.0 - 18.0 * t2 + (t2 * t2) + 14.0 * nu2
		- 58.0 * t2 * nu2;

	double l6coef = 61.0 - 58.0 * t2 + (t2 * t2) + 270.0 * nu2
		- 330.0 * t2 * nu2;

	double l7coef = 61.0 - 479.0 * t2 + 179.0 * (t2 * t2) - (t2 * t2 * t2);

	double l8coef = 1385.0 - 3111.0 * t2 + 543.0 * (t2 * t2) - (t2 * t2 * t2);

	/* Calculate easting (x) */
	x = N * cos(lat) * l
		+ (N / 6.0 * pow(cos(lat), 3.0) * l3coef * pow(l, 3.0))
		+ (N / 120.0 * pow(cos(lat), 5.0) * l5coef * pow(l, 5.0))
		+ (N / 5040.0 * pow(cos(lat), 7.0) * l7coef * pow(l, 7.0));

	/* Calculate northing (y) */
	y = arcLengthOfMeridian(lat)
		+ (t / 2.0 * N * pow(cos(lat), 2.0) * pow(l, 2.0))
		+ (t / 24.0 * N * pow(cos(lat), 4.0) * l4coef * pow(l, 4.0))
		+ (t / 720.0 * N * pow(cos(lat), 6.0) * l6coef * pow(l, 6.0))
		+ (t / 40320.0 * N * pow(cos(lat), 8.0) * l8coef * pow(l, 8.0));

	return;
}

void UsarSimClientModule::parseGPS(string& s)
{
	if (!session->getBool(PROPERTY_GPS_ENABLED)) return;
	RDK_DEBUG_PRINTF("GPS: %s", s.c_str());
	string tag = "", content = "";
	string name = "";
	string latitude = "", longitude = "";
	bool fix = false;
	int satellites = 0;
	double latVal = 0.0, longVal = 0.0;
	while (getNextSegmentContent(s, tag, content)) {
		if (tag == "Name") name = content;
		else if (tag == "Latitude" || tag == "Longitude") {
			vector<string> v = tokenize(content, ",");
			if (v.size() == 3) {
				int deg = atoi(v[0].c_str());
				double minute = atof(v[1].c_str());
				char c = (v[2].length() > 0 ? v[2][0] : '\0');
				ostringstream oss;
				oss << deg << "° " << minute << "'" << c;
				if (tag == "Latitude") latitude = oss.str();
				else if (tag == "Longitude") longitude = oss.str();
				double d = deg2rad((double) deg + minute / 60.0) * ((c == 'W' || c == 'S') ? -1 : 1);
				if (tag == "Longitude") longVal = d;
				else if (tag == "Latitude") latVal = d;
			}
			else {
				RDK_ERROR_PRINTF("Malformed server response '{%s %s}'", tag.c_str(), content.c_str());
			}
		}
		else if (tag == "Fix") fix = (content == "1" ? true : false);
		else if (tag == "Satellites") satellites = atoi(content.c_str());
	}
	session->setString(PROPERTY_GPS_NAME, name);
	session->setString(PROPERTY_GPS_LATITUDE, latitude);
	session->setString(PROPERTY_GPS_LONGITUDE, longitude);
	session->setBool(PROPERTY_GPS_FIX, fix);
	session->setInt(PROPERTY_GPS_SATELLITES, satellites);
	double x = 0.0, y = 0.0;
	mapLatLonToXY(latVal, longVal, 0.0, x, y);
	session->setPose(PROPERTY_GPS_POSE, Point2od(x, y, DBL_MAX));
}

}} // ns
