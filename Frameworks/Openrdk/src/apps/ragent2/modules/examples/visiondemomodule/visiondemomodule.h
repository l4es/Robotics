/**
 * @file
 * @brief This file contains the declaration of VisionDemoModule
 */

#ifndef RDK_MODULE_TEMPLATEMODULE
#define RDK_MODULE_TEMPLATEMODULE

#include <rdkcore/modules/module.h>

namespace RDK2 { namespace RAgent {

/**
 * @brief Please write a SHORT description of VisionDemoModule here.
 *
 * Please write a description of VisionDemoModule here.
 *
 * @ingroup RAgentModules
 */

class VisionDemoModule : public Module {
public:
	VisionDemoModule() { }
	virtual ~VisionDemoModule() { }

	bool initConfigurationProperties();
	// bool initInterfaceProperties();
	bool init();
	void exec();
	// void exitRequested();
	// void cleanup();
	// void asyncAgentCmd(cstr cmd);
};

}} // namespace

#endif
