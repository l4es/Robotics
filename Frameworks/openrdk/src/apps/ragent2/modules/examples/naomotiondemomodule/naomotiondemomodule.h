/**
 * @file
 * @brief This file contains the declaration of VisionDemoModule
 */

#ifndef RDK_MODULE_NAOMOTIONDEMO
#define RDK_MODULE_NAOMOTIONDEMO

#include <rdkcore/modules/module.h>

namespace RDK2 { namespace RAgent {

/**
 * @brief Please write a SHORT description of NaoMotionDemoModule here.
 *
 * Please write a description of NaoMotionDemoModule here.
 *
 * @ingroup RAgentModules
 */

class NaoMotionDemoModule : public Module {
public:
	NaoMotionDemoModule() { }
	virtual ~NaoMotionDemoModule() { }

	bool initConfigurationProperties();
	// bool initInterfaceProperties();
	bool init();
	void exec();
	void exitRequested();
	// void cleanup();
	// void asyncAgentCmd(cstr cmd);
};

}} // namespace

#endif
