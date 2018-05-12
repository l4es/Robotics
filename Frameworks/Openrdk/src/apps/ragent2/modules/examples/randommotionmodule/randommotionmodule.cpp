#define MODULE_NAME "RandomMotionModule"

#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE MODULE_NAME

#include "randommotionmodule.h"

#define PROPERTY_FORWARD_SPEED "params/forwardSpeed"
#define PROPERTY_TURNING_JOG "params/jog"
#define PROPERTY_MIN_TURNING_DURATION "params/minTurningDuration"
#define PROPERTY_MAX_TURNING_DURATION "params/maxTurningDuration"
#define PROPERTY_MIN_WALKING_DURATION "params/minWalkingDuration"
#define PROPERTY_MAX_WALKING_DURATION "params/maxWalkingDuration"

#define PROPERTY_DESIRED_SPEED "out/desiredSpeed"
#define PROPERTY_DESIRED_JOG "out/desiredJog"

namespace RDK2 { namespace RAgent {

bool RandomMotionModule::initConfigurationProperties()
{
	SESSION_TRY_START(session)
		// put false in the second parameter if you want the module disabled for default
		// add a third parameter valued false if you don't want the state property
		Common::createDefaultProperties(session, false);
		session->createDouble(PROPERTY_FORWARD_SPEED, "Speed when walking forward", RDouble::M_SEC, 0.5);
		session->createDouble(PROPERTY_TURNING_JOG, "Jog when turning", RDouble::RAD_SEC, deg2rad(10.0));
		session->createDouble(PROPERTY_MIN_TURNING_DURATION, "Minimum turning duration", RDouble::SEC, 1.0);
		session->createDouble(PROPERTY_MAX_TURNING_DURATION, "Maximum turning duration", RDouble::SEC, 4.0);
		session->createDouble(PROPERTY_MIN_WALKING_DURATION, "Minimum forward walking duration", RDouble::SEC, 1.0);
		session->createDouble(PROPERTY_MAX_WALKING_DURATION, "Maximum forward walking duration", RDouble::SEC, 3.0);

		session->createDouble(PROPERTY_DESIRED_SPEED, "Desired robot speed (please link)", RDouble::M_SEC, 0.0);
		session->createDouble(PROPERTY_DESIRED_JOG, "Desired robot jog (please link)", RDouble::RAD_SEC, 0.0);

	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

bool RandomMotionModule::init()
{
	SESSION_TRY_START(session)
		session->listenToTimer(100.);
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

void RandomMotionModule::exec()
{
	TimerR tmr;
	int currentAction = 0;
	double currentDuration = 0.0;
	bool wasEnabled = false;
	while (session->wait(), !exiting) {
		SESSION_TRY_START(session)
			bool enabled = session->getBool(PROPERTY_MODULE_ENABLED);
			if (!enabled && wasEnabled) {
				session->setDouble(PROPERTY_DESIRED_SPEED, 0.);
				session->setDouble(PROPERTY_DESIRED_JOG, 0.);
			}
			else if (enabled) {
				if (tmr.getSeconds() > currentDuration) currentAction = 0;
				if (currentAction == 0) {
					// action choice
					currentAction = rand() % 3 + 1;
					currentDuration = (double) (rand() % 1000) / 1000.;
					double minDur = 0., maxDur = 0.;
					if (currentAction == 1) {	// go forward
						minDur = session->getDouble(PROPERTY_MIN_WALKING_DURATION);
						maxDur = session->getDouble(PROPERTY_MAX_WALKING_DURATION);
					}
					else {	// turn
						minDur = session->getDouble(PROPERTY_MIN_TURNING_DURATION);
						maxDur = session->getDouble(PROPERTY_MAX_TURNING_DURATION);
					}
					currentDuration = currentDuration * (maxDur - minDur) + minDur;
					RDK_DEBUG_PRINTF("Performing action %d for %.2f seconds", currentAction, currentDuration);
					tmr.start();
				}

				if (currentAction == 1) {
					session->setDouble(PROPERTY_DESIRED_SPEED, session->getDouble(PROPERTY_FORWARD_SPEED));
					session->setDouble(PROPERTY_DESIRED_JOG, 0.);
				}
				else {
					session->setDouble(PROPERTY_DESIRED_SPEED, 0.);
					session->setDouble(PROPERTY_DESIRED_JOG, session->getDouble(PROPERTY_TURNING_JOG) * (currentAction == 2 ? -1 : 1));
				}
			}
			wasEnabled = enabled;
		SESSION_END_CATCH_TERMINATE(session)
	}
}

MODULE_FACTORY(RandomMotionModule);

}} // namespace
