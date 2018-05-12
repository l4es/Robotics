#define MODULE_NAME "NaoQiAudioModule"

#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE MODULE_NAME

#include "naoqiaudiomodule.h"
#include "naoqiaudiomodule_names.h"

#include <cstring>

namespace RDK2 { namespace RAgent {

bool NaoQiAudioModule::initConfigurationProperties()
{
	SESSION_TRY_START(session)

	Common::createDefaultProperties(session, true);

	session->createString(PROPERTY_IN_TEXT, "Text to say", "");
	session->setVolatile(PROPERTY_IN_TEXT);
	session->createInt(PROPERTY_VOLUME, "Volume (0-100)", 50);

	SESSION_END(session);
	return true;
	SESSION_CATCH_TERMINATE(session);
	return false;
}

bool NaoQiAudioModule::init()
{
	SESSION_TRY_START(session)

	shmem = new SharedMemory(SharedMemory::AUDIO, sizeof(RdkAudioData));

	RdkAudioData* data = static_cast<RdkAudioData*>(shmem->getEntry());
	sayToShm(data);

	session->registerPropertyUpdateEventHandler(SESSION_EVENT(doSayToShm));
	session->listen(PROPERTY_IN_TEXT);

	shmem->signal(NAOQI);

	SESSION_END(session);
	return true;
	SESSION_CATCH_TERMINATE(session);
	return false;
}

void NaoQiAudioModule::exec()
{
	while (session->wait(), !exiting)
	{
		SESSION_TRY_START(session)
			session->processEvents();
		SESSION_END_CATCH_TERMINATE(session);
	}
}

void NaoQiAudioModule::exitRequested()
{
	if (shmem == 0)
		return;
	shmem->wait(RDK);

	RdkAudioData* data = static_cast<RdkAudioData*>(shmem->getEntry());
	strcpy(data->text,"Bye Bye");

	shmem->signal(NAOQI);
}

bool NaoQiAudioModule::doSayToShm(const Event* /*e*/)
{
	shmem->wait(RDK);
	RdkAudioData* data = static_cast<RdkAudioData*>(shmem->getEntry());
	sayToShm(data);
	shmem->signal(NAOQI);
	return true;
}

void NaoQiAudioModule::sayToShm(RdkAudioData* data)
{
	if (session->getString(PROPERTY_IN_TEXT) != "")
	{
		strncpy(data->text,session->getString(PROPERTY_IN_TEXT).c_str(),sizeof(data->text));
		data->volume = session->getInt(PROPERTY_VOLUME);
		session->setString(PROPERTY_IN_TEXT,"");
	}
}

//void NaoQiAudioModule::cleanup() { }

MODULE_FACTORY(NaoQiAudioModule);

}} // namespace
