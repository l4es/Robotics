#define MODULE_NAME "DummyLocalizerModule"

#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE MODULE_NAME

#include "dummylocalizermodule.h"

#define PROPERTY_ODOMETRY_POSE "in/odometryPose"
#define PROPERTY_ESTIMATED_POSE "out/estimatedPose"

namespace RDK2 { namespace RAgent {

bool DummyLocalizerModule::initConfigurationProperties()
{
	SESSION_TRY_START(session)
		// put false in the second parameter if you want the module disabled for default
		// add a third parameter valued false if you don't want the state property
		Common::createDefaultProperties(session, true);
		
		// input
		session->createPose(PROPERTY_ODOMETRY_POSE, "Odometry Pose (please link)", Point2od(0., 0., 0.));

		// output
		session->createPose(PROPERTY_ESTIMATED_POSE, "Estimated Pose", Point2od(0., 0., 0.));

	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

bool DummyLocalizerModule::init()
{
	// in the init() function you should initialize everything your module needs (drivers initialization,
	// sockets, and so on); all modules wait for all init() before they start
	SESSION_TRY_START(session)
		session->listen(PROPERTY_ODOMETRY_POSE);
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

void DummyLocalizerModule::exec()
{
	while (session->wait(), !exiting) {
		SESSION_TRY_START(session)
			Point2od odometryPose = session->getPose(PROPERTY_ODOMETRY_POSE);
			session->setPose(PROPERTY_ESTIMATED_POSE, odometryPose);
		SESSION_END_CATCH_TERMINATE(session)
	}
}

MODULE_FACTORY(DummyLocalizerModule);

}}

