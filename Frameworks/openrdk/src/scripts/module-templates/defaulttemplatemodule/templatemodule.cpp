#define MODULE_NAME "TemplateModule"

#include "templatemodule.h"
#include "templatemodule_names.h"

#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/logging/logging.h>
#include <rdkcore/ns.h>
#define LOGGING_MODULE MODULE_NAME

namespace RDK2 { namespace RAgent {

bool TemplateModule::initConfigurationProperties()
{
	SESSION_TRY_START(session)
		// put false in the second parameter if you want the module disabled for default
		// add a third parameter valued false if you don't want the state property
		Common::createDefaultProperties(session, true);
		// here you declare the properties of your module
		// in the repository they have the name of this module as prefix, e.g. /yourModuleName/maxSpeed
		// session->createString(PROPERTY_MY_BEAUTIFUL_STRING, "My beautiful string", "Default value");
		// session->createDouble(PROPERTY_MAX_SPEED, "Maximum speed", RDouble::MM_SEC, 100.);
		// // example of enum creation
		// ENUM_CREATE(whichStalled);
		// ENUM_ITEM(whichStalled, NONE, "None", "No wheel stalled");
		// ENUM_ITEM(whichStalled, LEFT, "Left", "Left wheels stalled");
		// ENUM_ITEM(whichStalled, RIGHT, "Right", "Right wheels stalled");
		// ENUM_ITEM(whichStalled, BOTH, "Both", "Full robot stall");
		// session->createEnum(PROPERTY_ROBOT_WHICH_STALLED, "Which robot wheel stalled", whichStalled, NONE, INFO);
		// // example of image creation
		// session->createImage(PROPERTY_MY_IMAGE, "My pretty image", width, height, type);
		// // example of map creation
		// session->createMap(PROPERTY_MY_MAP, "My ugly map", x, y, theta, realWidth, bitmapWidth, bitmapHeight);
		// // example of vector creation
		// session->createVector<RType>(PROPERTY_MY_VECTOR, "My vector");

	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

// bool TemplateModule::initInterfaceProperties() { }

bool TemplateModule::init()
{
	// in the init() function you should initialize everything your module needs (drivers initialization,
	// sockets, and so on); all modules wait for all init() before they start
	SESSION_TRY_START(session)
		// here you can declare the events you are waiting in the main exec() loop,
		// for example:
		session->listenToTimer(500.);
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

void TemplateModule::exec()
{
	while (session->wait(), !exiting) {
		SESSION_TRY_START(session)
			RDK_DEBUG_PRINTF("I'm '%s', I will do something", getModuleName().c_str());
		SESSION_END_CATCH_TERMINATE(session)
	}
}

// implement this if you need to force the exec to go out of the loop
// if the thread may be waiting not for the session->wait() semaphore:
// on closing, the main thread will set the exiting variables, call this exitRequested()
// function and then signal the session semaphores
// void TemplateModule::exitRequested() { }

// implement this if you need to clean up things after the exec has exited
// void TemplateModule::cleanup() { }

// void TemplateModule::asyncAgentCmd(cstr cmd)
// {
//	SESSION_TRY_START(asyncSession)
//	// here you can parse 'cmd'
//	SESSION_END_CATCH_TERMINATE(asyncSession)
// }

MODULE_FACTORY(TemplateModule);

}} // namespace
