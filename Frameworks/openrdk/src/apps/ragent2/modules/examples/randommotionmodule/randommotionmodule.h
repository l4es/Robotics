/**
 * @file
 * @brief This file contains the declaration of the RandomMotionModule module
 */

#ifndef RDK2_MODULE_RANDOMMOTIONMODULE
#define RDK2_MODULE_RANDOMMOTIONMODULE

#include <rdkcore/modules/module.h>

namespace RDK2 { namespace RAgent {

/**
 * @brief This module performs a random motion for a wheeled robot.
 *
 * When it is activated, this module moves randomly the robot, it does not do any obstacle avoidance.
 *
 * @ingroup RAgentModules
 */

class RandomMotionModule : public Module {
public:
	RandomMotionModule() { }
	virtual ~RandomMotionModule() { }

	bool initConfigurationProperties();
	bool init();
	void exec();
};

}} // namespace

#endif
