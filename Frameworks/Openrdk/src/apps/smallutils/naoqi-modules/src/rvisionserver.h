/// <summary>
/// OpenRDK Vision server module.
/// </summary>
/// <remarks> OpenRDK Developers </remarks>

#ifndef OPENRDK_RVISIONSERVER_H
#define OPENRDK_RVISIONSERVER_H

#include <alptr.h>
#include <almodule.h>

#include <rdkcore/sharedmemory/sharedmemory.h>

namespace AL
{
	class ALBroker;
	class ALLoggerProxy;
}

/// <summary>
/// OpenRDK Vision server module.
/// </summary>
class RVisionServer : public AL::ALModule
{

	public:

		/// <summary>
		/// Simple Vision Server for handling Nao video device in OpenRDK.
		/// </summary>
		RVisionServer(AL::ALPtr<AL::ALBroker> pBroker, const std::string& pName );

		/// <summary>
		/// Desctructor
		/// </summary>
		virtual ~RVisionServer();

		/// <summary>
		/// Initialization method
		/// </summary>
		virtual void init();

		/// <summary>
		/// Register to the V.I.M.
		/// </summary>
		/// <param name="pResolution"> Resolution requested. </param> 
		/// <param name="pColorSpace"> Colorspace requested. </param> 
		void registerToVIM(const int &pResolution, const int &pColorSpace);

		/// <summary>
		/// Unregister from the V.I.M.
		/// </summary>
		void unRegisterFromVIM();

		/// <summary>
		/// Shared memory handler thread
		/// </summary>
		static void* visionTaskThreaFn( RVisionServer* me ) { me->visionTask(); return NULL; }

		/// <summary>
		/// Check if the Shared Memory is correctly plugged in.
		/// </summary>
		/// <returns> true if the shared memory is connected. </returns>
		bool isPlugged() { return shmimage->pluggingSuccessful(); }

	private:
		/// <summary>
		/// Process commands from OpenRDK module
		/// </summary>
		void processCommands();

		/// <summary>
		/// Shared memory handler function
		/// </summary>
		void visionTask();

		/// <summary>
		/// Proxy to the logger module.
		/// </summary>
		AL::ALPtr<AL::ALLoggerProxy> fLogProxy;

		/// <summary>
		/// Proxy to the video device module.
		/// </summary>
		AL::ALPtr<AL::ALProxy>  fCamProxy;
		std::string fGvmName;
		bool fRegisteredToVim;

		/// <summary>
		/// Shared memory pointer
		/// </summary>
		RDK2::SharedMemory *shmimage;

		/// <summary>
		/// Shared memory handler thread pointer
		/// </summary>
		pthread_t visionTaskThreadId;
};

#endif  // OPENRDK_RVISIONSERVER_H

