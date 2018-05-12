#ifndef RDK_MODULE_DIOPSISTASKMANAGERMODULE
#define RDK_MODULE_DIOPSISTASKMANAGERMODULE

#include <rdkcore/modules/module.h>

#include "dsp/DSPAbstract_TaskManager.h"
#include "dsp/FreeRTOS_include/task.h"

namespace RDK2 { namespace RAgent {

#define EVENT_HANDLER_CLASS DiopsisTaskmanagerModule

class DiopsisTaskmanagerModule : public Module {
public:
	DiopsisTaskmanagerModule():timeout(1000000)
		{ }
	virtual ~DiopsisTaskmanagerModule() { }

	bool initConfigurationProperties();
	// bool initInterfaceProperties();
	bool init();
	void exec();
	// void exitRequested();
	// void cleanup();
	// void asyncAgentCmd(cstr cmd);
private:
	DA_Msg msg;
	DA_TLCreateTaskArgs* mArgs;
	DA_long larg;

	DA_TLTasks *magicTasks;
	DA_TLParams* magicParams;

	DA_double* task1Var;
	DA_double* task2Var;

	DA_int timeout;
	xTaskHandle TLHandle;

	string currentTaskBasename;
	string phase;

	bool doInit();
	bool startTask(string basename);
	void checkTasks();

	bool propertyEventVariable(const Event* e);
	DECLARE_EVENT_HANDLER(propertyEventVariable);
};

}} // namespace

#endif
