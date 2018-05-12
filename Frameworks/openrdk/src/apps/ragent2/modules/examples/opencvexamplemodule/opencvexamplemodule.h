/**
 * @file
 * @brief This file contains the declaration of OpenCVExampleModule
 */

#ifndef RDK_MODULE_OPENCVEXAMPLEMODULE
#define RDK_MODULE_OPENCVEXAMPLEMODULE

#include <rdkcore/modules/module.h>

namespace RDK2 { namespace RAgent {

/**
 * @brief Please write a SHORT description of OpenCVExampleModule here.
 *
 * Please write a description of OpenCVExampleModule here.
 *
 * @ingroup RAgentModules
 */

#define EVENT_HANDLER_CLASS OpenCVExampleModule

class OpenCVExampleModule : public Module {
public:
	OpenCVExampleModule() { }
	virtual ~OpenCVExampleModule() { }

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
