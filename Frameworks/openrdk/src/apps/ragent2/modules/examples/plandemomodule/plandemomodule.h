/**
 * @file
 * @brief This file contains the declaration of PlanDemoModule
 */

#ifndef RDK_MODULE_PLANDEMO
#define RDK_MODULE_PLANDEMO

#include <rdkcore/modules/module.h>

namespace RDK2 { namespace RAgent {

/**
 * @brief Please write a SHORT description of PlanDemoModule here.
 *
 * Please write a description of PlanDemoModule here.
 *
 * @ingroup RAgentModules
 */

class PlanDemoModule : public Module {
public:
	PlanDemoModule() { }
	virtual ~PlanDemoModule() { }

	bool initConfigurationProperties();
	// bool initInterfaceProperties();
	bool init();
	void exec();
	// void exitRequested();
	// void cleanup();
	// void asyncAgentCmd(cstr cmd);

private:
	double lastY;
	int state;

};

}} // namespace

#endif
