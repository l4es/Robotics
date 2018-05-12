#define MODULE_NAME "EventHandlerExampleModule"

#include "eventhandlerexamplemodule.h"
#include "eventhandlerexamplemodule_names.h"

#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/logging/logging.h>
#include <rdkcore/ns.h>
#define LOGGING_MODULE MODULE_NAME

namespace RDK2 { namespace RAgent {

bool EventHandlerExampleModule::initConfigurationProperties()
{
	SESSION_TRY_START(session)
		// put false in the second parameter if you want the module disabled for default
		// add a third parameter valued false if you don't want the state property
		Common::createDefaultProperties(session, true);
		session->createBool(PROPERTY_BOOL_ONE, "Bool 1", false);
		session->createBool(PROPERTY_BOOL_TWO, "Bool 2", false);
		session->createBool(PROPERTY_BAAL_THREE, "Baal 3", false);
		session->createBool(PROPERTY_BEEL_FOUR, "Beel 4", false);
		session->createInt(PROPERTY_INT_ONE, "Int 1", 0);
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

bool EventHandlerExampleModule::init()
{
	SESSION_TRY_START(session)
		//session->listenToTimer(10000.);
		session->listen(PROPERTY_BOOL_ONE);
		session->listen(PROPERTY_BOOL_TWO);
		session->listen(PROPERTY_BAAL_THREE);
		session->listen(PROPERTY_INT_ONE);
		
		//session->registerTimerEventHandler(SESSION_EVENT(timerEvent));
		session->registerPropertyUpdateEventHandler(SESSION_EVENT(propertyUpdateEvent), PROPERTY_BOOL_ONE);
		session->registerPropertyUpdateEventHandler(SESSION_EVENT(propertyUpdateEvent2), "test/bool");
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

void EventHandlerExampleModule::exec()
{
	while (session->wait(), !exiting) {
		SESSION_TRY_START(session)
			RDK_DEBUG_PRINTF("%zu events in queue", session->events().size());
			session->processEvents();
		SESSION_END_CATCH_TERMINATE(session)
	}
}

bool EventHandlerExampleModule::propertyUpdateEvent(const Event* e)
{
	const EventPropertyUpdate* event = dynamic_cast<const EventPropertyUpdate*>(e);
	if (event) {
		RDK_DEBUG_PRINTF("Handler for property 'test/bool1' update event (url: '%s')", event->propertyUrl.c_str());
	}
	else {
		RDK_DEBUG_PRINTF("Something wrong, this seems not to be an EventPropertyUpdate object (it is of class '%s')", e->type.c_str());
	}
	return true;
}

bool EventHandlerExampleModule::propertyUpdateEvent2(const Event* e)
{
	const EventPropertyUpdate* event = dynamic_cast<const EventPropertyUpdate*>(e);
	if (event) {
		RDK_DEBUG_PRINTF("Handler for property prefix 'test/bool' update event (url: '%s')", event->propertyUrl.c_str());
	}
	else {
		RDK_DEBUG_PRINTF("Something wrong, this seems not to be an EventPropertyUpdate object (it is of class '%s')", e->type.c_str());
	}
	return true;
}

bool EventHandlerExampleModule::timerEvent(const Event*)
{
	RDK_DEBUG_PRINTF("Handler for timer event");
	return true;
}

MODULE_FACTORY(EventHandlerExampleModule);

}} // namespace
