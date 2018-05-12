#ifndef OPENRDK_RSENSORSSERVER_H
#define OPENRDK_RSENSORSSERVER_H

#include <alptr.h>
#include <almodule.h>
#include <rdkcore/sharedmemory/sharedmemory.h>
#include <nao-objects/sensordatashm.h>

namespace AL
{
	class ALBroker;
	class ALMemoryFastAccess;
	class ALLoggerProxy;
}

class RSensorsServer : public AL::ALModule
{
	public:
		/// <summary>
		/// Simple Sensor Server for handling Nao sensors in OpenRDK.
		/// </summary>
		RSensorsServer(AL::ALPtr<AL::ALBroker> pBroker, const std::string& pName );

		/// <summary>
		/// Desctructor
		/// </summary>
		virtual ~RSensorsServer();
		
		/// <summary>
		/// Initialization method
		/// </summary>
		virtual void init();
		
		/// <summary>
		/// Register to the V.I.M.
		/// </summary>
		/// <param name="pResolution"> Resolution requested. </param> 
		/// <param name="pColorSpace"> Colorspace requested. </param> 
		/// <param name="pFps"> Frame per second requested. </param> 
		void registerToVIM(const int &pResolution, const int &pColorSpace, const int& pFps);

		/// <summary>
		/// Unregister from the V.I.M.
		/// </summary>
		void unRegisterFromVIM();

		/// <summary>
		/// Shared memory handler thread for images
		/// </summary>
		static void* visionTaskThreaFn( RSensorsServer* me ) { me->visionTask(); return NULL; }

		/// <summary>
		/// Shared memory handler thread for sensors
		/// </summary>
		static void* sensorThreadFn( RSensorsServer* me ) { me->sensorTask(); return NULL; }

		/// <summary>
		/// Shared memory handler thread for sensors
		/// </summary>
		static void* allSensorThreadFn( RSensorsServer* me ) { me->allSensorTask(); return NULL; }

		/// <summary>
		/// Shared memory handler thread for battery level handling
		/// </summary>
		static void* batteryThreadFn( RSensorsServer* me ) { me->batteryManager(); return NULL; }
		
		/// <summary>
		/// Check if the Shared Memory is correctly plugged in.
		/// </summary>
		/// <returns> true if the shared memory is connected. </returns>
		bool isPlugged()
		{
			return shmsensor->pluggingSuccessful() &&
				shmimage->pluggingSuccessful();
		}

	private:
		/// <summary>
		/// Proxy to the logger module.
		/// </summary>
		AL::ALPtr<AL::ALLoggerProxy> fLogProxy;

		/// <summary>
		/// Proxy to the led module.
		/// </summary>
		AL::ALPtr<AL::ALProxy> fLedProxy;
		
		/// <summary>
		/// Proxy to sentinel proxy.
		/// </summary>		
		AL::ALPtr<AL::ALProxy> xsentinel_proxy;
		
		/// <summary>
		/// Proxy to naomotionmodule proxy.
		/// </summary>		
		AL::ALPtr<AL::ALProxy> naomotionmodule_proxy;
		
		/// <summary>
		/// Proxy to the fast memory module.
		/// </summary>
		AL::ALPtr<AL::ALMemoryFastAccess> fMemoryFastAccess;
		std::vector<float> sensorValues;
		std::vector<std::string> fSensorKeys;
		
		/// <summary>
		/// Proxy to the video device module.
		/// </summary>
		AL::ALPtr<AL::ALProxy>  fCamProxy;
		std::string fGvmName;
		bool fRegisteredToVim;

		/// <summary>
		/// Shared memory pointer for sensors
		/// </summary>
		RDK2::SharedMemory *shmsensor;

		/// <summary>
		/// Shared memory pointer for image
		/// </summary>
		RDK2::SharedMemory *shmimage;

		/// <summary>
		/// Shared memory pointer for all sensors
		/// </summary>
		RDK2::SharedMemory *shmallsensor;

		//////////////////// Thread functions ////////////////////
		/// <summary>
		/// Shared memory handler function
		/// </summary>
		void visionTask();

		/// <summary>
		/// Read sensor values to shared memory
		/// </summary>
		void sensorTask();

		/// <summary>
		/// Read all sensor values (including images) to shared memory
		/// Combine visionTask and sensorTask
		/// </summary>
		void allSensorTask();

		/// <summary>
		/// Battery level manager
		/// </summary>
		void batteryManager();
		
		/// <summary>
		/// Shared memory handler thread pointer for sensors
		/// </summary>
		pthread_t sensorThreadId;

		/// <summary>
		/// Shared memory handler thread pointer for all sensors
		/// </summary>
		pthread_t allSensorThreadId;

		/// <summary>
		/// Shared memory handler thread pointer for images
		/// </summary>
		pthread_t visionTaskThreadId;
		
		/// <summary>
		/// Shared memory handler thread pointer for battery
		/// </summary>
		pthread_t batteryThreadId;

		/// <summary>
		/// Read image into the shared memory
		/// </summary>
		void imageReader(RdkImageData& imageData);

		/// <summary>
		/// Process commands from OpenRDK module
		/// </summary>
		void processCommands(RdkImageData& imageData);

		/// <summary>
		/// Read sensor values into the shared memory
		/// </summary>
		void sensorReader(RdkSensorData& sensorData);

		int currentResolution;
		int currentColorspace;
		int currentFps;
		int currentCamera;
};

#endif
