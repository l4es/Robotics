#ifndef RDK_MODULE_TEMPLATEMODULE
#define RDK_MODULE_TEMPLATEMODULE

#include <rdkcore/modules/module.h>

namespace RDK2 { namespace RAgent {

#define EVENT_HANDLER_CLASS TemplateModule

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
