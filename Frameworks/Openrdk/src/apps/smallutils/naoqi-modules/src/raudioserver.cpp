/// <summary>
/// OpenRDK Audio server module.
/// </summary>
/// <remarks> OpenRDK Developers </remarks>

#include "raudioserver.h"
#include <alproxy.h>
#include <alptr.h>
#include <albroker.h>
#include <almodule.h>

#include <nao-objects/sensordatashm.h>
#include <rdkcore/config.h>
#include <string>

/// <summary>
/// OpenRDK Audio server module.
/// </summary>
/// <param name="broker"> A smart pointer to the broker.</param> 
/// <param name="name">   The name of the module. </param> 
RAudioServer::RAudioServer(AL::ALPtr<AL::ALBroker> broker, const std::string& name ): AL::ALModule(broker, name ), shm(NULL)
{
	setModuleDescription( "Simple Audio Server for handling Nao audio device in OpenRDK." );

	functionName( "isPlugged", getName(), "Check if the Shared Memory is correctly plugged in." );
	setReturn( "isPlugged", "true if the shared memory is connected.");
	BIND_METHOD( RAudioServer::isPlugged );
}

/// <summary>
/// Destructor
/// </summary>
RAudioServer::~RAudioServer()
{
	if (shm != NULL)
	{
		delete shm;
	}
	shm = NULL;
}

/// <summary>
/// Initialization method
/// </summary>
void RAudioServer::init()
{
	// Create a proxy to the logger module
	// If this fails, we throw an exception and the module will not be registered.
	try
	{
		fLogProxy = getParentBroker()->getLoggerProxy();
	}
	catch (const AL::ALError& e)
	{
		throw AL::ALError(getName(),"RAudioServer()","Fail to create a proxy to ALLogger module. Msg " + e.toString() + "\nModule will abort." );
	}

	// Create a proxy to the text-to-speech module
	// If this fails, we throw an exception and the module will not be registered.
	try
	{
		fTTSProxy = getParentBroker()->getProxy("ALTextToSpeech");
	}
	catch ( AL::ALError& e )
	{
#ifdef OpenRDK_ARCH_GENERIC
		fLogProxy->warn(getName(),"generic platform detected: Could not create a proxy to ALTextToSpeech module");
#else
		throw AL::ALError(getName(),"RAudioServer()","Fail to create a proxy to ALTextToSpeech module. Msg " + e.toString() + "\nModule will abort." );
#endif
	}

	shm = new RDK2::SharedMemory(RDK2::SharedMemory::AUDIO, sizeof(RdkAudioData));
	if (shm->pluggingSuccessful())
	{
		fLogProxy->lowInfo(getName(),"Succesfully plugged in Shared Memory for AUDIO");
	}
	else
	{
		fLogProxy->error(getName(),"Cannot create ShM AUDIO");
	}

	RdkAudioData *command = static_cast<RdkAudioData*>(shm->getEntry());
#ifdef OpenRDK_ARCH_GENERIC
	fLogProxy->warn(getName(),"generic platform detected: fTTSProxy->call<float>(\"getVolume\") not implemented");
#else
	command->volume  = static_cast<int>(fTTSProxy->call<float>("getVolume"));
#endif
	command->text[0] = '\0';

	pthread_create(
			&senderThreadId,
			NULL,
			(void*(*)(void*))senderThreadFn,
			this );

	fLogProxy->info(getName()," Init AUDIO done.");
}

#ifdef OpenRDK_ARCH_GENERIC
#define SAY(toSay) \
	fLogProxy->warn(getName(),"generic platform detected: on Nao I would say \"" + toSay + "\"");
#else
#define SAY(toSay) \
	fTTSProxy->callVoid("say", toSay);
#endif

/// <summary>
/// Shared memory handler function
/// </summary>
void RAudioServer::sender()
{
	usleep(5e6);
	std::string toSay="";

	toSay = "Open RDK - NaoQi module initialized.";
	SAY(toSay)

	while (true)
	{
		shm->wait(NAOQI);

		RdkAudioData *command = static_cast<RdkAudioData*>(shm->getEntry());
		if (command->text[0] != '\0')
		{
			//it block everything, TODO
//#ifdef OpenRDK_ARCH_GENERIC
//      fLogProxy->warn(getName(),"fTTSProxy->callVoid(\"setVolume\") not implemented for platform: generic");
//#else
//      //fTTSProxy->callVoid("setVolume",0.5);
//#endif
			
			toSay = command->text;
			SAY(toSay)
			command->text[0]='\0';
		}

		shm->signal(RDK);
	}
}

