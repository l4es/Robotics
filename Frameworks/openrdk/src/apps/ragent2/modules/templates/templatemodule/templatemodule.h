/**
 * @file
 * @brief This file contains the declaration of TemplateModule
 */

#ifndef RDK_MODULE_TEMPLATEMODULE
#define RDK_MODULE_TEMPLATEMODULE

#include <rdkcore/modules/module.h>

namespace RDK2 { namespace RAgent {

/**
 * @brief Please write a SHORT description of TemplateModule here.
 *
 * Please write a description of TemplateModule here.
 *
 * @ingroup RAgentModules
 */

class TemplateModule : public Module {
public:
	TemplateModule() { }
	virtual ~TemplateModule() { }

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
