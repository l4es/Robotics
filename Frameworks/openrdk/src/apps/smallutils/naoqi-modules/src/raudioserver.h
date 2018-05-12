/// <summary>
/// OpenRDK Audio server module.
/// </summary>
/// <remarks> OpenRDK Developers </remarks>

#ifndef OPENRDK_RAUDIOSERVER_H
#define OPENRDK_RAUDIOSERVER_H

#include <alloggerproxy.h>
#include <alptr.h>
#include <almodule.h>
#include <alproxies/altexttospeechproxy.h>
#include <rdkcore/sharedmemory/sharedmemory.h>

namespace AL
{
	class ALBroker;
}

/// <summary>
/// OpenRDK Audio server module.
/// </summary>
class RAudioServer : public AL::ALModule
{

	public:

		/// <summary>
		/// Simple Audio Server for handling Nao audio device in OpenRDK.
		/// </summary>
		RAudioServer(AL::ALPtr<AL::ALBroker> pBroker, const std::string& pName );

		/// <summary>
		/// Destructor
		/// </summary>
		virtual ~RAudioServer();

		/// <summary>
		/// Initialization method
		/// </summary>
		virtual void init();

		/// <summary>
		/// Shared memory handler thread
		/// </summary>
		static void* senderThreadFn( RAudioServer* me ) { me->sender(); return NULL; }

		/// <summary>
		/// Check if the Shared Memory is correctly plugged in.
		/// </summary>
		/// <returns> true if the shared memory is connected. </returns>
		bool isPlugged() { return shm->pluggingSuccessful(); }

	protected :
		/// <summary>
		/// Shared memory handler function
		/// </summary>
		void sender();

		/// <summary>
		/// Proxy to the logger module.
		/// </summary>
		AL::ALPtr<AL::ALLoggerProxy> fLogProxy;

		/// <summary>
		/// Proxy to the text-to-speech module.
		/// </summary>
		AL::ALPtr<AL::ALProxy>  fTTSProxy;

		/// <summary>
		/// Shared memory pointer
		/// </summary>
		RDK2::SharedMemory *shm;

		/// <summary>
		/// Shared memory handler thread pointer
		/// </summary>
		pthread_t senderThreadId;
};

#endif  // OPENRDK_RAUDIOSERVER_H

