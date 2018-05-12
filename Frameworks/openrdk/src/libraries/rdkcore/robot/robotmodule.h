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

/**
 * This file contains some macros and functions useful for robot
 * modules
 */

#ifndef RDK2_COMMON_ROBOTMODULE
#define RDK2_COMMON_ROBOTMODULE

#define PROPERTY_MODULE_ENABLED "enabled"
#define PROPERTY_MODULE_STATE "state"
#define PROPERTY_MODULE_ACTIVITY_COUNTER "stats/activityCounter"
#define PROPERTY_MODULE_LAST_ITERATION_TIMER "stats/iterationDuration/last"
#define PROPERTY_MODULE_ITERATION_DURATION_MEAN "stats/iterationDuration/mean"
#define PROPERTY_MODULE_SCHEDULE_INTERVAL_MEAN "stats/scheduleInterval/mean"
#define PROPERTY_MODULE_SCHEDULE_MIN_INTERVAL "sys/scheduleMinInterval"

#define ROBOT_PREFIX "/robot/"

#define PROPERTY_ROBOT_ODOMETRY_POSE "odometryPose"
#define PROPERTY_ROBOT_ESTIMATED_POSE "estimatedPose"
#define PROPERTY_ROBOT_TARGET_POSE "targetPose"
#define PROPERTY_ROBOT_WIDTH "width"
#define PROPERTY_ROBOT_HEIGHT "height"
#define PROPERTY_ROBOT_RADIUS "radius"
#define PROPERTY_ROBOT_SHAPE "shape"
#define PROPERTY_ROBOT_STALLED "stalled"
#define PROPERTY_ROBOT_SPEED "speed"
#define PROPERTY_ROBOT_JOG "jog"
#define PROPERTY_ROBOT_PATH_DONE "pathDone"
#define PROPERTY_ROBOT_WHICH_STALLED "whichStalled"
#define PROPERTY_ROBOT_DESIRED_SPEED "desiredSpeed"
#define PROPERTY_ROBOT_MAX_WHEEL_SPEED "maxWheelSpeed"
#define PROPERTY_ROBOT_DESIRED_JOG "desiredJog"
#define PROPERTY_ROBOT_LASER_DATA "laserData"
#define PROPERTY_ROBOT_SONAR_DATA "sonarData"
#define PROPERTY_ROBOT_DATA_QUEUE "dataQueue"
#define PROPERTY_ROBOT_LASER_POSE "laserPose"

#define SSTR(x) #x

#define CREATE_SERVER_PROPERTIES(server, host, port)\
	session->createString(SSTR(server##ServerHost), #server" server host", #host);\
	session->createInt(SSTR(server##ServerPort), #server" server port", port);\
	Network::UDPSocket* server##Socket = new UDPSocket();\
	serverMap.insert(std::make_pair(#server, server##Socket));
	
#define SELECT_CHECK(server, parseFunction)\
	ServerAddressMap::iterator server##_it = serverMap.find(#server);\
	if(server##_it != serverMap.end() && FD_ISSET(server##_it->second->getSocket(), &readSet)){\
		std::string data;\
		InetAddress server##Addr;\
		int server##result = server##_it->second->recv(data, server##Addr);\
		parsedData = parseFunction(data);\
	}




#include <rdkcore/repository/session.h>

// FIXME
#include <rdkcore/rsensordata/rusarvictimrfiddata.h>
#include <rdkcore/posixconstructs/posixmutex.h>

namespace RDK2 { namespace /*(FIXME: )RAgent*/Common {

using namespace RepositoryNS;

enum RobotShape { CIRCULAR = 0, RECTANGULAR = 1 };
enum RobotWhicStalled { NONE = 0, LEFT = 1, RIGHT = 2, BOTH = 3 };

/**
 * Creates the properties every robot should have
 */
void createRobotProperties(Session* session) throw (SessionException);


// XXX forse pi chiaro mettere DEFAULT_ENABLED | CREATE_STATE_PROPERTY, anche se ad Andrea non piacciono :-)
/**
 * Create the properties every module should have
 */
void createDefaultProperties(Session* session, bool defaultEnabled, bool createStateProperty = true)
	throw (SessionException);

/**
 * This enum defines the possible states that a "working" module can
 * assume.
 */
enum ModuleState { STATE_INACTIVE = 0, STATE_COMPUTING = 1, STATE_DONE = 2, STATE_FAILED = 3 };
/**
 * Create a property that holds a ModuleState
 */
void createStateProperty(Session* session, string propertyName) throw (SessionException);

void getRobotSizes(Session* session, double& robotMinSize, double& robotMaxSize) throw (SessionException);

// FIXME AAAAAAAAAAAAAAAAHHH
extern vector<RDK2::RSensorData::RUsarVictimRfidData*> cheIlCieloMiPerdoni;
extern PosixConstructs::PosixMutex cheIlCieloMiFulmini;

}} // namespaces

#endif
