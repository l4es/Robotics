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

#include <rdkcore/repository/repository.h>
#include <rdkcore/rmaps/ritemonmapvector.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "RobotModule"

// FIXME
#include <rdkcore/rsensordata/rusarvictimrfiddata.h>
#include <rdkcore/posixconstructs/posixmutex.h>

#include "robotmodule.h"

namespace RDK2 { namespace Common {

// FIXME!!!
vector<RDK2::RSensorData::RUsarVictimRfidData*> cheIlCieloMiPerdoni;
PosixConstructs::PosixMutex cheIlCieloMiFulmini;

void createDefaultProperties(Session* session, bool defaultEnabled, bool haveToCreateStateProperty)
	throw (SessionException)
{
	SESSION_TRY(session)
		session->createBool(PROPERTY_MODULE_ENABLED, "Module is enabled", defaultEnabled);
		if (haveToCreateStateProperty) createStateProperty(session, PROPERTY_MODULE_STATE);
		session->createInt(PROPERTY_MODULE_ACTIVITY_COUNTER,
			"Activity counter (increments for each iteration)", 0, INFO);
		session->setVolatile(PROPERTY_MODULE_ACTIVITY_COUNTER);
		session->createDouble(PROPERTY_MODULE_LAST_ITERATION_TIMER,
			"Duration of last iteration", RDouble::SEC, 0., INFO);
		session->setVolatile(PROPERTY_MODULE_LAST_ITERATION_TIMER);
		session->createDouble(PROPERTY_MODULE_ITERATION_DURATION_MEAN, "Duration of iterations: mean", RDouble::SEC, 0., INFO);
		session->setVolatile(PROPERTY_MODULE_ITERATION_DURATION_MEAN);
		session->createDouble(PROPERTY_MODULE_SCHEDULE_INTERVAL_MEAN, "Schedulation interval: mean", RDouble::SEC, 0., INFO);
		session->setVolatile(PROPERTY_MODULE_SCHEDULE_INTERVAL_MEAN);
		session->createDouble(PROPERTY_MODULE_SCHEDULE_MIN_INTERVAL, "Minimum scheduling interval", RDouble::SEC, 0.010);
	SESSION_CATCH_RETHROW(session)
}

void createStateProperty(Session* session, string propertyName) throw (SessionException)
{
	ENUM_CREATE(state);
	ENUM_ITEM(state, STATE_INACTIVE, "Inactive", "Module is inactive");
	ENUM_ITEM(state, STATE_COMPUTING, "Computing", "Module is computing");
	ENUM_ITEM(state, STATE_FAILED, "Failed", "Module has failed computation");
	ENUM_ITEM(state, STATE_DONE, "Done", "Module has done its task");
	session->createEnum(propertyName, "Module state", state, STATE_INACTIVE, INFO);
}

void createRobotProperties(Session* session) throw (SessionException)
{
	SESSION_TRY(session)
		session->createPose(PROPERTY_ROBOT_ODOMETRY_POSE, "Pose of robot (odometry)");
		session->createPose(PROPERTY_ROBOT_ESTIMATED_POSE, "Pose of robot (localization/slam estimate)");
		session->createPose(PROPERTY_ROBOT_TARGET_POSE, "Target pose (gotopos)", RPoint2od(0.,0.,0.));
	
		session->createStorage("RLaserData", PROPERTY_ROBOT_LASER_DATA, "Laser data");
		session->createStorage("RSonarData", PROPERTY_ROBOT_SONAR_DATA, "Sonar data");

		session->createDouble(PROPERTY_ROBOT_RADIUS, "Radius of robot (circular robot)", RDouble::M, 0.300);
		session->createDouble(PROPERTY_ROBOT_WIDTH, "Width of robot (y, rectangular robot)", RDouble::M, 0.400);
		session->createDouble(PROPERTY_ROBOT_HEIGHT, "Depth of robot (x, rectangular robot)", RDouble::M, 0.500);
		
		ENUM_CREATE(shapes);
		ENUM_ITEM(shapes, CIRCULAR, "Circular", "Circular shape (Pioneer 2)");
		ENUM_ITEM(shapes, RECTANGULAR, "Rectangular", "Rectangular shape (Pioneer 3AT)");
		session->createEnum(PROPERTY_ROBOT_SHAPE, "Shape of robot", shapes, CIRCULAR);
	
		session->createBool(PROPERTY_ROBOT_STALLED, "Robot stalled", false, INFO);

		ENUM_CREATE(whichStalled);
		ENUM_ITEM(whichStalled, NONE, "None", "No wheel stalled");
		ENUM_ITEM(whichStalled, LEFT, "Left", "Left wheels stalled");
		ENUM_ITEM(whichStalled, RIGHT, "Right", "Right wheels stalled");
		ENUM_ITEM(whichStalled, BOTH, "Both", "Full robot stall");
		session->createEnum(PROPERTY_ROBOT_WHICH_STALLED, "Which robot wheel stalled", whichStalled, NONE, INFO);
	
		session->createDouble(PROPERTY_ROBOT_SPEED, "Robot speed", RDouble::M_SEC, 0., INFO);
		session->createDouble(PROPERTY_ROBOT_JOG, "Robot jog", RDouble::RAD_SEC, 0., INFO);
		session->createDouble(PROPERTY_ROBOT_DESIRED_SPEED, "Robot desired speed", RDouble::M_SEC, 0., NOT_PERSISTENT);
		session->createDouble(PROPERTY_ROBOT_MAX_WHEEL_SPEED, "Maximum speed for single wheel", RDouble::M_SEC, 2 * M_PI);
		session->createDouble(PROPERTY_ROBOT_DESIRED_JOG, "Robot desired jog", RDouble::RAD_SEC, 0., NOT_PERSISTENT);
	
		session->createQueue(PROPERTY_ROBOT_DATA_QUEUE, "Robot data queue");
		session->createPose(PROPERTY_ROBOT_LASER_POSE, "Laser pose on robot", Point2od(0., 0., 0.));
	SESSION_CATCH_RETHROW(session)
}

void getRobotSizes(Session* session, double& robotMinSize, double& robotMaxSize) throw (SessionException)
{
	Common::RobotShape robotShape =
		(Common::RobotShape) session->getEnum(ROBOT_PREFIX PROPERTY_ROBOT_SHAPE);
	if (robotShape == Common::CIRCULAR) {
		robotMinSize = robotMaxSize = session->getDouble(ROBOT_PREFIX PROPERTY_ROBOT_RADIUS) * 2;
	}
	else {
		double robotWidth = session->getDouble(ROBOT_PREFIX PROPERTY_ROBOT_WIDTH);
		double robotHeight = session->getDouble(ROBOT_PREFIX PROPERTY_ROBOT_HEIGHT);
		robotMinSize = (robotWidth < robotHeight ? robotWidth : robotHeight);
		robotMaxSize = (robotWidth > robotHeight ? robotWidth : robotHeight);
	}
}

}} // namespaces
