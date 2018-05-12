/**
 * @file
 * @brief This file contains the declaration of EventHandlerExampleModule
 */

#ifndef RDK_MODULE_EVENTHANDLEREXAMPLEMODULE
#define RDK_MODULE_EVENTHANDLEREXAMPLEMODULE

#include <rdkcore/modules/module.h>

namespace RDK2 { namespace RAgent {

/**
 * @brief Please write a SHORT description of EventHandlerExampleModule here.
 *
 * Please write a description of EventHandlerExampleModule here.
 *
 * @ingroup RAgentModules
 */

#define EVENT_HANDLER_CLASS EventHandlerExampleModule

class EventHandlerExampleModule : public Module {
public:
	EventHandlerExampleModule() { }
	virtual ~EventHandlerExampleModule() { }

	bool initConfigurationProperties();
	bool init();
	void exec();

private:
	bool timerEvent(const Event* e);			DECLARE_EVENT_HANDLER(timerEvent);
	bool propertyUpdateEvent(const Event* e);	DECLARE_EVENT_HANDLER(propertyUpdateEvent);
	bool propertyUpdateEvent2(const Event* e);	DECLARE_EVENT_HANDLER(propertyUpdateEvent2);
};

}} // namespace

#endif
