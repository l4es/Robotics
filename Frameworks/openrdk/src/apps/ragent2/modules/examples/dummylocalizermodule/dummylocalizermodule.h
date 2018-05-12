/**
 * @file
 * @brief This file contains the declaration of the Template module
 */

#ifndef RDK2_MODULE_DUMMYLOCALIZERMODULE
#define RDK2_MODULE_DUMMYLOCALIZERMODULE

#include <rdkcore/modules/module.h>

namespace RDK2 { namespace RAgent {

/**
 * @brief Please write a SHORT description of your module here.
 *
 * Please write a description of your module here.
 *
 * @ingroup RAgentModules
 */

class DummyLocalizerModule : public Module {
public:
	DummyLocalizerModule() { }
	virtual ~DummyLocalizerModule() { }

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
