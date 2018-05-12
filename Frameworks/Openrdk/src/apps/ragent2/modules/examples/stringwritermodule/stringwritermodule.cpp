#define MODULE_NAME "StringWriterModule"

#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE MODULE_NAME

#include "stringwritermodule.h"

#define PROPERTY_STRING_TO_WRITE "params/stringToWrite"

namespace RDK2 { namespace RAgent {

bool StringWriterModule::initConfigurationProperties()
{
	SESSION_TRY_START(session)
		// put false in the second parameter if you want the module disabled for default
		// add a third parameter valued false if you don't want the state property
		Common::createDefaultProperties(session, true);
		session->createString(PROPERTY_STRING_TO_WRITE, "String to print", "This is a string");

	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

bool StringWriterModule::init()
{
	SESSION_TRY_START(session)
		session->listenToTimer(1000.);
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

void StringWriterModule::exec()
{
	while (session->wait(), !exiting) {
		SESSION_TRY_START(session)
			string myString = session->getString(PROPERTY_STRING_TO_WRITE);
			RDK_DEBUG_PRINTF("I'm '%s', this is the string: '%s'", getModuleName().c_str(), myString.c_str());
		SESSION_END_CATCH_TERMINATE(session)
	}
}

MODULE_FACTORY(StringWriterModule);

}} // namespace
