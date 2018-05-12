#define MODULE_NAME "SimpleMapperModule"

#include <rdkcore/sensordata/laserdata.h>
#include <rdkcore/rsensordata/rlaserdata.h>

#include <rdkcore/geometry/point2.h>
#include <rdkcore/geometry/walk_line.h>

#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/rmaps/rmapimage.h>
#include <rdkcore/rmaps/rmapimagediffpoints.h>
#include <rdkcore/rmaps/rfeatureonmapvector.h>
#include <rdkcore/logging/logging.h>
#include <rdkcore/rgraphics/rc8set.h>
#define LOGGING_MODULE MODULE_NAME

#include "simplemappermodule.h"

#define PROPERTY_MAP "out/map"
#define PROPERTY_MAP_X "params/initMapX"
#define PROPERTY_MAP_Y "params/initMapY"
#define PROPERTY_MAP_WIDTH "params/initMapWidth"
#define PROPERTY_MAP_HEIGHT "params/initMapHeight"
#define PROPERTY_MAP_RESOLUTION "params/initMapResolution"
#define PROPERTY_LASER_DATA "in/laserData"
#define PROPERTY_ROBOT_POSE "in/robotPose"
#define PROPERTY_MODE "mode"
#define PROPERTY_QUEUE "in/dataQueue"
#define PROPERTY_CMD_RESET "cmds/reset"
#define PROPERTY_CLEAR_MAP_ON_INIT "params/clearMapOnInit"
#define PROPERTY_MAX_LASER_DIST "params/maxLaserDist"
#define PROPERTY_DONT_TOUCH_COLORS "params/dontTouchColors"

#define PROPERTY_LASER_PLACEMENT "params/laserPlacement"

namespace RDK2 { namespace RAgent {

using namespace RDK2::SensorData;
using namespace RDK2::RMaps;
using namespace RDK2::RGraphics;
using namespace RDK2::Geometry;
using namespace RDK2::RSensorData;

bool SimpleMapperModule::initConfigurationProperties()
{
	SESSION_TRY_START(session)	
		Common::createDefaultProperties(session, true, false);
		
		ENUM_CREATE(mode);
		ENUM_ITEM(mode, USE_OBJECTS, "Use objects", "Get pose and laserData from storage objects");
		ENUM_ITEM(mode, USE_QUEUE, "Use queue", "Get pose and laserData from queue");
		session->createEnum(PROPERTY_MODE, "Mapper mode (0 = laserData/robotPose, 1 = dataQueue)", mode, USE_QUEUE);

		session->createDouble(PROPERTY_MAX_LASER_DIST, "Max laser distance", RDouble::M, 20.);

		session->createMap(PROPERTY_MAP, "Map", 0., 0., 0., 5., 200, 200);
		//session->setPersistent(PROPERTY_MAP, false);
		session->createDouble(PROPERTY_MAP_X, "X coord of map.", RDouble::M, -5.);
		session->createDouble(PROPERTY_MAP_Y, "Y coord of map.", RDouble::M, 10.);
		session->createDouble(PROPERTY_MAP_WIDTH,  "Map width.", RDouble::M, 20.);
		session->createDouble(PROPERTY_MAP_HEIGHT, "Map height.",RDouble::M, 20.);
		session->createDouble(PROPERTY_MAP_RESOLUTION, "Map resolution (pixel/m).", RDouble::REAL, 60.);

		session->createPose(PROPERTY_LASER_PLACEMENT, "Laser placement w.r.t. the robot", Point2od(0.0, 0.0, 0.0));
	
		session->createStorage("RC8Set", PROPERTY_DONT_TOUCH_COLORS, "Colors that cannot be modified by this module", new RC8Set());

		session->createStorage("RLaserData", PROPERTY_LASER_DATA, "Laser data");
		session->createPose(PROPERTY_ROBOT_POSE, "Robot pose (please link)");
		session->createQueue(PROPERTY_QUEUE, "Robot data queue");

		session->createBool(PROPERTY_CMD_RESET, "Reset the map", false);

		session->createBool(PROPERTY_CLEAR_MAP_ON_INIT, "Clear map on initialization", true);
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

void SimpleMapperModule::initMap(Session* session) throw (SessionException)
{
	double x = session->getDouble(PROPERTY_MAP_X);
	double y = session->getDouble(PROPERTY_MAP_Y);
	double width = session->getDouble(PROPERTY_MAP_WIDTH);
	double height = session->getDouble(PROPERTY_MAP_HEIGHT);
	double resolution = session->getDouble(PROPERTY_MAP_RESOLUTION);

	session->lock(PROPERTY_MAP, HERE);

	RMapImage* rmi = session->getObjectAsL<RMapImage>(PROPERTY_MAP);
	rmi->x = x;
	rmi->y = y;
	rmi->realWidth= width;

	delete rmi->image;
	int pixel_width = (int)(width * resolution);
	int pixel_height = (int)(height * resolution);
	
	RImage* image = new RImage( pixel_width, pixel_height, RImage::C8 );
	unsigned char* buf = image->getBuffer();
	for(int b = 0; b < (int)image->getHeight(); b++) 
		for(int a = 0; a < (int)image->getWidth(); a++) {
			buf[b * image->getWidth() + a] = C8Unknown;
		}
	rmi->image = image;

	session->unlock(PROPERTY_MAP);
}

bool SimpleMapperModule::init()
{
	SESSION_TRY_START(session)
		if (session->getBool(PROPERTY_CLEAR_MAP_ON_INIT)) initMap(session);
		session->queueSubscribe(PROPERTY_QUEUE);
		session->listen(PROPERTY_QUEUE);
		session->listen(PROPERTY_ROBOT_POSE);
		session->listen(PROPERTY_LASER_DATA);
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

void SimpleMapperModule::exec()
{
	while (session->wait(), !exiting) {
		SESSION_TRY_START(session)
		if (session->getBool(PROPERTY_MODULE_ENABLED)) {
			if (session->getBool(PROPERTY_CMD_RESET)) {
				initMap(session);
				session->setBool(PROPERTY_CMD_RESET, false);
			}
			Point2od laserPose = session->getPose(PROPERTY_LASER_PLACEMENT);
			RC8Set* dontTouchColors = session->getObjectCloneAs<RC8Set>(PROPERTY_DONT_TOUCH_COLORS);
			if (session->getInt(PROPERTY_MODE) == (int) USE_OBJECTS) {
				Point2od pose = session->getPose(PROPERTY_ROBOT_POSE) + laserPose;	// FIXME not exactly "+"
				RLaserData* laserData = session->getObjectCloneAs<RLaserData>(PROPERTY_LASER_DATA); 
				updateMap(session, pose, *laserData, dontTouchColors);
				delete laserData;
				session->queueFreeze(PROPERTY_QUEUE);
			}
			else {
				vector<const RLaserData*> data = session->queueFreezeAs<RLaserData>(PROPERTY_QUEUE);
				for (size_t i = 0; i < data.size(); i++) {
					updateMap(session, data[i]->estimatedPose + laserPose, *(data[i]), dontTouchColors);	// FIXME not exactly "+"
				}
			}
			delete dontTouchColors;
		}
		else session->queueFreeze(PROPERTY_QUEUE);
		SESSION_END_CATCH_TERMINATE(session)
	}
}

void SimpleMapperModule::updateMap(Session* session, const Point2od& pose, const LaserData& rld, const RC8Set* dontTouchColors)
{
	SESSION_TRY(session)

	session->lock(PROPERTY_MAP, HERE);
	RMapImage* rmi = session->getObjectAsL<RMapImage>(PROPERTY_MAP);

	double maxRange = rld.maxReading;
	double maxLaserToUse = session->getDouble(PROPERTY_MAX_LASER_DIST);
	
	int n = rld.points.size();
	
	vector<Point2i> modifiedPoints;

	for(int a=0; a<n; a++) {
		int msx, msy;
		bool putObstacle = true;
		
		rmi->world2buf(pose.x, pose.y, msx, msy);
		Point2i p0(msx, msy);
		
		double reading  = (rld.points[a].reading);
		if (reading > maxRange || reading > maxLaserToUse) {
			reading = std::min(maxRange, maxLaserToUse);
			putObstacle = false;
		}
		else if (reading < 0.01) {
			continue;
		}
		
		double angle = pose.theta + rld.laserPose.theta + rld.points[a].theta;
		double wex = pose.x + reading*cos(angle);
		double wey = pose.y + reading*sin(angle);
		int mex, mey;
		rmi->world2buf(wex, wey, mex, mey);
		Point2i p1(mex, mey);
		
		LineWalk whiteLine(p0, p1);
		do {
			Point2i pp = whiteLine.getPoint();
			updatePixel(rmi->image, dontTouchColors, pp.x, pp.y, C8Free, modifiedPoints);
		} while (whiteLine.next());
	
		if (putObstacle) {
			for (int i = 0; i < 3; i++) {
				updatePixel(rmi->image, dontTouchColors, mex+i%2, mey+i/2, C8Obstacle, modifiedPoints);
			}
		}
	}
		
	session->declareDiffL(PROPERTY_MAP, new RMapImageDiffPoints(rmi, modifiedPoints));
	session->unlock(PROPERTY_MAP);
	session->valueChanged(PROPERTY_MAP);

	SESSION_CATCH_RETHROW(session)
}

MODULE_FACTORY(SimpleMapperModule);

}} // namespace
