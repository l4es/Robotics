/// <summary>
/// OpenRDK Vision server module.
/// </summary>
/// <remarks> OpenRDK Developers </remarks>

#include "rsensorsserver.h"
#include <alproxy.h>
#include <alloggerproxy.h>
#include <alptr.h>
#include <albroker.h>
#include <almodule.h>

#include <almemoryfastaccess.h>
#include <alvisiondefinitions.h>
#include <alloggerproxy.h>
#include <alimage.h>
#include <nao-objects/naojoints.h>

#include <rdkcore/config.h>
#include <string>

using namespace std;
using namespace AL;

// this is used to enable a joint shared memory for sensors and images
// instead separate ones
#define ENABLE_JOINT_SHM

// this define needs if you want to enable experimental feature,
// like double buffer image or fast switching cameras
// uncomment to enable
//#define EXPERIMENTAL_FEATURE

enum
{
	// Joints Sensor list
	JHeadPitch = 0,
	JHeadYaw,
	JLAnklePitch,
	JLAnkleRoll,
	JLElbowRoll,
	JLElbowYaw,
	JLHand,
	JLHipPitch,
	JLHipRoll,
	JLHipYawPitch,
	JLKneePitch,
	JLShoulderPitch,
	JLShoulderRoll,
	JLWristYaw,
	JRAnklePitch,
	JRAnkleRoll,
	JRElbowRoll,
	JRElbowYaw,
	JRHand,
	JRHipPitch,
	JRHipRoll,
	JRKneePitch,
	JRShoulderPitch,
	JRShoulderRoll,
	JRWristYaw,

	// Inertial sensors
	ISAccX,
	ISAccY,
	ISAccZ,
	ISGyrX,
	ISGyrY,
	ISGyrRef,
	ISAngleX,
	ISAngleY,

	// Some FSR sensors
	FSRLCenterOfPressureX,
	FSRLCenterOfPressureY,
	FSRLTotalWeight,
	FSRRCenterOfPressureX,
	FSRRCenterOfPressureY,
	FSRRTotalWeight,

	// Ultrasound
	USLeft,
	USRight,

	// Bumpers
	BumperLLeft,
	BumperLRight,
	BumperRLeft,
	BumperRRight,

	// Miscellaneous
	BatteryCharge,
	BatteryCurrent,
	BatteryTemperature,

	MAX_NUMBER_OF_VALUES
};

/// <summary>
/// OpenRDK Sensor server module.
/// </summary>
/// <param name="broker"> A smart pointer to the broker.</param> 
/// <param name="name">   The name of the module. </param> 
RSensorsServer::RSensorsServer(AL::ALPtr<AL::ALBroker> broker, const std::string& name ): AL::ALModule(broker, name ),
	fRegisteredToVim(false),
	shmsensor(NULL),
	shmimage(NULL),
	shmallsensor(NULL)
{
	setModuleDescription("Simple Sensor Server for handling Nao sensors (joints, inertial, fsr, ultrasound, images) in OpenRDK.");

	functionName( "registerToVIM", getName(), "Register to the V.I.M." );
	addParam("resolution", "Resolution requested.");
	addParam("colorSpace", "Colorspace requested.");
	BIND_METHOD( RSensorsServer::registerToVIM );

	functionName( "unRegisterFromVIM", getName(), "Unregister from the V.I.M." );
	BIND_METHOD( RSensorsServer::unRegisterFromVIM );

	functionName("isPlugged",getName(),"Check if the Shared Memories are correctly plugged in.");
	setReturn("isPlugged","true if the shared memory is connected.");

	BIND_METHOD(RSensorsServer::isPlugged);
}

/// <summary>
/// Destructor
/// </summary>
RSensorsServer::~RSensorsServer()
{
	if (shmsensor != NULL)
	{
		delete shmsensor;
	}
	shmsensor = NULL;
	
	this->unRegisterFromVIM();
	if (shmimage != NULL)
	{
		delete shmimage;
	}
	shmimage = NULL;

	if (shmallsensor != NULL)
	{
		delete shmallsensor;
	}
	shmallsensor = NULL;
}

/// <summary>
/// Initialization method
/// </summary>
void RSensorsServer::init()
{
	// Create a proxy to the logger module.
	// If this fails, we throw an exception and the module will not be registered.
	try
	{
		fLogProxy = getParentBroker()->getLoggerProxy();
	}
	catch (const AL::ALError& e)
	{
		throw AL::ALError(getName(),"RSensorsServer()","Fail to create a proxy to ALLogger module. Msg " + e.toString() + "\nModule will abort.");
	}

	// Create a proxy to led devices
	// If this fails, we throw an exception and the module will not be registered
	try
	{
		fLedProxy = getParentBroker()->getProxy("ALLeds");
	}
	catch (ALError& e)
	{
#ifdef OpenRDK_ARCH_GENERIC
		fLogProxy->warn(getName(),"Could not create a proxy to ALLeds module");
#else
		throw ALError(getName(),"RSensorsServer()","Fail to create a proxy to ALLeds module. Msg " + e.toString() + "\nModule will abort.");
#endif
	}

	// Create a proxy to memory fast access module
	// If this fails, we throw an exception and the module will not be registered
	try
	{
		fMemoryFastAccess = AL::ALPtr<AL::ALMemoryFastAccess>(new AL::ALMemoryFastAccess());
		fSensorKeys.clear();
		fSensorKeys.resize(MAX_NUMBER_OF_VALUES);

		// Joints Sensor list
		fSensorKeys[JHeadPitch] = std::string("Device/SubDeviceList/HeadPitch/Position/Sensor/Value");
		fSensorKeys[JHeadYaw] = std::string("Device/SubDeviceList/HeadYaw/Position/Sensor/Value");
		fSensorKeys[JLAnklePitch] = std::string("Device/SubDeviceList/LAnklePitch/Position/Sensor/Value");
		fSensorKeys[JLAnkleRoll] = std::string("Device/SubDeviceList/LAnkleRoll/Position/Sensor/Value");
		fSensorKeys[JLElbowRoll] = std::string("Device/SubDeviceList/LElbowRoll/Position/Sensor/Value");
		fSensorKeys[JLElbowYaw] = std::string("Device/SubDeviceList/LElbowYaw/Position/Sensor/Value");
		fSensorKeys[JLHand] = std::string("Device/SubDeviceList/LHand/Position/Sensor/Value");
		fSensorKeys[JLHipPitch] = std::string("Device/SubDeviceList/LHipPitch/Position/Sensor/Value");
		fSensorKeys[JLHipRoll] = std::string("Device/SubDeviceList/LHipRoll/Position/Sensor/Value");
		fSensorKeys[JLHipYawPitch] = std::string("Device/SubDeviceList/LHipYawPitch/Position/Sensor/Value");
		fSensorKeys[JLKneePitch] = std::string("Device/SubDeviceList/LKneePitch/Position/Sensor/Value");
		fSensorKeys[JLShoulderPitch] = std::string("Device/SubDeviceList/LShoulderPitch/Position/Sensor/Value");
		fSensorKeys[JLShoulderRoll] = std::string("Device/SubDeviceList/LShoulderRoll/Position/Sensor/Value");
		fSensorKeys[JLWristYaw] = std::string("Device/SubDeviceList/LWristYaw/Position/Sensor/Value");
		fSensorKeys[JRAnklePitch] = std::string("Device/SubDeviceList/RAnklePitch/Position/Sensor/Value");
		fSensorKeys[JRAnkleRoll] = std::string("Device/SubDeviceList/RAnkleRoll/Position/Sensor/Value");
		fSensorKeys[JRElbowRoll] = std::string("Device/SubDeviceList/RElbowRoll/Position/Sensor/Value");
		fSensorKeys[JRElbowYaw] = std::string("Device/SubDeviceList/RElbowYaw/Position/Sensor/Value");
		fSensorKeys[JRHand] = std::string("Device/SubDeviceList/RHand/Position/Sensor/Value");
		fSensorKeys[JRHipPitch] = std::string("Device/SubDeviceList/RHipPitch/Position/Sensor/Value");
		fSensorKeys[JRHipRoll] = std::string("Device/SubDeviceList/RHipRoll/Position/Sensor/Value");
		fSensorKeys[JRKneePitch] = std::string("Device/SubDeviceList/RKneePitch/Position/Sensor/Value");
		fSensorKeys[JRShoulderPitch] = std::string("Device/SubDeviceList/RShoulderPitch/Position/Sensor/Value");
		fSensorKeys[JRShoulderRoll] = std::string("Device/SubDeviceList/RShoulderRoll/Position/Sensor/Value");
		fSensorKeys[JRWristYaw] = std::string("Device/SubDeviceList/RWristYaw/Position/Sensor/Value");

		// Inertial sensors
		fSensorKeys[ISAccX] = std::string("Device/SubDeviceList/InertialSensor/AccX/Sensor/Value");
		fSensorKeys[ISAccY] = std::string("Device/SubDeviceList/InertialSensor/AccY/Sensor/Value");
		fSensorKeys[ISAccZ] = std::string("Device/SubDeviceList/InertialSensor/AccZ/Sensor/Value");
		fSensorKeys[ISGyrX] = std::string("Device/SubDeviceList/InertialSensor/GyrX/Sensor/Value");
		fSensorKeys[ISGyrY] = std::string("Device/SubDeviceList/InertialSensor/GyrY/Sensor/Value");
		fSensorKeys[ISGyrRef] = std::string("Device/SubDeviceList/InertialSensor/GyrRef/Sensor/Value");
		fSensorKeys[ISAngleX] = std::string("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value");
		fSensorKeys[ISAngleY] = std::string("Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value");

		// Some FSR sensors
		fSensorKeys[FSRLCenterOfPressureX] = std::string("Device/SubDeviceList/LFoot/FSR/CenterOfPressure/X/Sensor/Value");
		fSensorKeys[FSRLCenterOfPressureY] = std::string("Device/SubDeviceList/LFoot/FSR/CenterOfPressure/Y/Sensor/Value");
		fSensorKeys[FSRLTotalWeight] = std::string("Device/SubDeviceList/LFoot/FSR/TotalWeight/Sensor/Value");
		fSensorKeys[FSRRCenterOfPressureX] = std::string("Device/SubDeviceList/RFoot/FSR/CenterOfPressure/X/Sensor/Value");
		fSensorKeys[FSRRCenterOfPressureY] = std::string("Device/SubDeviceList/RFoot/FSR/CenterOfPressure/Y/Sensor/Value");
		fSensorKeys[FSRRTotalWeight] = std::string("Device/SubDeviceList/RFoot/FSR/TotalWeight/Sensor/Value");

		// Ultrasound
		fSensorKeys[USLeft] = std::string("Device/SubDeviceList/US/Left/Sensor/Value");
		fSensorKeys[USRight] = std::string("Device/SubDeviceList/US/Right/Sensor/Value");

		// Bumper
		fSensorKeys[BumperLLeft] = std::string("Device/SubDeviceList/LFoot/Bumber/Left/Sensor/Value");
		fSensorKeys[BumperLRight] = std::string("Device/SubDeviceList/LFoot/Bumber/Right/Sensor/Value");
		fSensorKeys[BumperRLeft] = std::string("Device/SubDeviceList/RFoot/Bumber/Left/Sensor/Value");
		fSensorKeys[BumperRRight] = std::string("Device/SubDeviceList/RFoot/Bumber/Right/Sensor/Value");

		// Miscellaneous
		fSensorKeys[BatteryCharge] = std::string("Device/SubDeviceList/Battery/Charge/Sensor/Value");
		fSensorKeys[BatteryCurrent] = std::string("Device/SubDeviceList/Battery/ElectricCurrent/Sensor/Value");
		fSensorKeys[BatteryTemperature] = std::string("Device/SubDeviceList/Battery/Temperature/Sensor/Value");

		sensorValues.resize(fSensorKeys.size());

		fMemoryFastAccess->ConnectToVariables(getParentBroker(),fSensorKeys,false);
	}
	catch (const AL::ALError& e)
	{
		throw AL::ALError(getName(),"RSensorsServer()","Fail to create a proxy to ALMemory module. Msg " + e.toString() + "\nModule will abort." );
	}

	currentResolution = AL::kQVGA;
	currentColorspace = AL::kYUV422InterlacedColorSpace;
	currentFps        = 30;
	// Create a proxy to video input device
	// If this fails, we throw an exception and the module will not be registered
	try
	{
		fCamProxy = getParentBroker()->getProxy("ALVideoDevice");
		this->registerToVIM(currentResolution,currentColorspace,currentFps);
	}
	catch ( AL::ALError& e )
	{
#ifdef OpenRDK_ARCH_GENERIC
		fLogProxy->warn(getName(),"Could not create a proxy to ALVideoDevice module");
#else
		throw AL::ALError(getName(),"RSensorsSever()","Fail to create a proxy to ALVideoDevice module. Msg " + e.toString() + "\nModule will abort." );
#endif
	}

	pthread_create( &batteryThreadId,NULL,(void*(*)(void*))batteryThreadFn,this );

#ifdef ENABLE_JOINT_SHM
	shmallsensor = new RDK2::SharedMemory(RDK2::SharedMemory::ALLSENSOR, sizeof(RdkAllSensorData));
	if (shmallsensor->pluggingSuccessful())
	{
		fLogProxy->lowInfo(getName(),"Succesfully plugged in Shared Memory for ALLSENSOR");
	}
	else
	{
		fLogProxy->error(getName(),"Cannot create ShM ALLSENSOR");
	}

	pthread_create( &allSensorThreadId,NULL,(void*(*)(void*))allSensorThreadFn,this );
	fLogProxy->info(getName()," Init ALLSENSOR done.");
#else
	shmsensor = new RDK2::SharedMemory(RDK2::SharedMemory::SENSOR, sizeof(RdkSensorData));
	if (shmsensor->pluggingSuccessful())
	{
		fLogProxy->lowInfo(getName(),"Succesfully plugged in Shared Memory for SENSOR");
	}
	else
	{
		fLogProxy->error(getName(),"Cannot create ShM SENSOR");
	}

	shmimage = new RDK2::SharedMemory(RDK2::SharedMemory::IMAGE, sizeof(RdkImageData));
	if (shmimage->pluggingSuccessful())
	{
		fLogProxy->lowInfo(getName(),"Succesfully plugged in Shared Memory for IMAGE");
	}
	else
	{
		fLogProxy->error(getName(),"Cannot create ShM IMAGE");
	}
	pthread_create( &visionTaskThreadId,NULL,(void*(*)(void*))visionTaskThreaFn,this );
	pthread_create( &sensorThreadId,NULL,(void*(*)(void*))sensorThreadFn,this );
	fLogProxy->info(getName()," Init SENSOR done.");
#endif
}

/// <summary>
/// Register to the V.I.M.
/// </summary>
/// <param name="pResolution"> Resolution requested. </param> 
/// <param name="pColorSpace"> Colorspace requested. </param> 
void RSensorsServer::registerToVIM(const int &pResolution, const int &pColorSpace, const int& pFps) {

	// If we've already registered a module, we need to unregister it first !
	if (fRegisteredToVim) {
		throw AL::ALError(getName(), "registerToVIM()", "A video module has already been "
				"registered. Call unRegisterFromVIM() before trying to register a new module.");
	}

	// GVM Name that we're going to use to register.
	const std::string kOriginalName = "OpenRDK_GVM";
	int imgWidth = 0;
	int imgHeight = 0;
	int imgNbLayers = 0;

	AL::getSizeFromResolution(pResolution, imgWidth, imgHeight);
	imgNbLayers = AL::getNumLayersInColorSpace(pColorSpace);

	if (imgWidth == -1 || imgWidth == -1 || imgNbLayers == -1) {
		throw AL::ALError(getName(), "registerToVIM()", "Invalid resolution or color space.");
	}

	// Call the "subscribe" function with the given parameters.
	fGvmName = fCamProxy->call<std::string>("subscribe", kOriginalName,
			pResolution, pColorSpace, pFps );

	fLogProxy->info(getName(), " module registered as " + fGvmName);

	// Registration is successful, set fRegisteredToVim to true.
	fRegisteredToVim = true;
	currentFps = pFps;
	currentResolution = pResolution;
	currentColorspace = pColorSpace;
}


/// <summary>
/// Unregister from the V.I.M.
/// </summary>
void RSensorsServer::unRegisterFromVIM() {

	if (!fRegisteredToVim) {
		throw AL::ALError(getName(), "unRegisterFromVIM()", "No video module is currently "
				"registered! Call registerToVIM first.");
	}

	fLogProxy->info(getName(), "try to unregister " + fGvmName + " module." );
	fCamProxy->callVoid("unsubscribe", fGvmName);
	fLogProxy->info(getName(), "Done.");

	// UnRegistration is successful, set fRegisteredToVim to false.
	fRegisteredToVim = false;
}

/// <summary>
/// Shared memory handler function for battery management
/// </summary>
void RSensorsServer::batteryManager()
{
	string led_before;
	string led_after;
	float battery_level;

	while (true)
	{
		fMemoryFastAccess->GetValues(sensorValues);

		battery_level = sensorValues[BatteryCharge];

		led_before = "Ears/Led/Left/";
		led_after = "Deg/Actuator/Value";

		if (battery_level >= 0.9)
		{
			// With this level of the battery the "EarsLed" must be turned on with ten leds.
			fLedProxy->callVoid("on",led_before + "324" + led_after);
		}
		else fLedProxy->callVoid("off",led_before + "324" + led_after);

		if (battery_level >= 0.8)
		{
			// With this level of the battery the "EarsLed" must be turned on with nine leds.
			fLedProxy->callVoid("on",led_before + "288" + led_after);
		}
		else fLedProxy->callVoid("off",led_before + "288" + led_after);

		if (battery_level >= 0.7)
		{
			// With this level of the battery the "EarsLed" must be turned on with eight leds.
			fLedProxy->callVoid("on",led_before + "252" + led_after);
		}
		else fLedProxy->callVoid("off",led_before + "252" + led_after);

		if (battery_level >= 0.6)
		{
			// With this level of the battery the "EarsLed" must be turned on with seven leds.
			fLedProxy->callVoid("on",led_before + "216" + led_after);
		}
		else fLedProxy->callVoid("off",led_before + "216" + led_after);

		if (battery_level >= 0.5)
		{
			// With this level of the battery the "EarsLed" must be turned on with six leds.
			fLedProxy->callVoid("on",led_before + "180" + led_after);
		}
		else fLedProxy->callVoid("off",led_before + "180" + led_after);

		if (battery_level >= 0.4)
		{
			// With this level of the battery the "EarsLed" must be turned on with five leds.
			fLedProxy->callVoid("on",led_before + "144" + led_after);
		}
		else fLedProxy->callVoid("off",led_before + "144" + led_after);

		if (battery_level >= 0.3)
		{
			// With this level of the battery the "EarsLed" must be turned on with four leds.
			fLedProxy->callVoid("on",led_before + "108" + led_after);
		}
		else fLedProxy->callVoid("off",led_before + "108" + led_after);

		if (battery_level >= 0.2)
		{
			// With this level of the battery the "EarsLed" must be turned on with three leds.
			fLedProxy->callVoid("on",led_before + "72" + led_after);
		}
		else fLedProxy->callVoid("off",led_before + "72" + led_after);

		if (battery_level >= 0.1)
		{
			// With this level of the battery the "EarsLed" must be turned on with two leds.
			fLedProxy->callVoid("on",led_before + "36" + led_after);
		}
		else fLedProxy->callVoid("off",led_before + "36" + led_after);

		// With this level of the battery the "EarsLed" must be turned on with only one led.
		fLedProxy->callVoid("on",led_before + "0" + led_after);

		usleep(10000000);
	}
}

/// <summary>
/// Shared memory handler function for vision task
/// </summary>
void RSensorsServer::visionTask() // to rdk module
{
	while (true)
	{
		shmimage->wait(NAOQI);
#ifdef OpenRDK_ARCH_GENERIC
		usleep(1000);
#endif

		RdkAllSensorData *sensordata = static_cast<RdkAllSensorData*>(shmallsensor->getEntry());
		imageReader(sensordata->vision);
		processCommands(sensordata->vision);

		shmimage->signal(RDK);
	}
}

/// <summary>
/// Shared memory handler function for sensor reading
/// </summary>
void RSensorsServer::sensorTask()
{
	while (true)
	{
		shmsensor->wait(NAOQI);
		RdkAllSensorData *sensordata = static_cast<RdkAllSensorData*>(shmallsensor->getEntry());
		sensorReader(sensordata->sensors);
		shmsensor->signal(RDK);
	}
}

/// <summary>
/// Shared memory handler function for sensor reading
/// </summary>
void RSensorsServer::allSensorTask()
{
	while (true)
	{
		shmallsensor->wait(NAOQI);

		RdkAllSensorData *sensordata = static_cast<RdkAllSensorData*>(shmallsensor->getEntry());
		sensorReader(sensordata->sensors);
		imageReader(sensordata->vision);
		processCommands(sensordata->vision);
		shmallsensor->signal(RDK);
	}
}

/// <summary>
/// Read image into the shared memory
/// </summary>
void RSensorsServer::imageReader(RdkImageData& imageData)
{
	if (!fRegisteredToVim)
	{
#ifndef OpenRDK_ARCH_GENERIC
		throw AL::ALError(getName(), "imageReader()",  "No video module is currently "
				"registered! Call registerToVIM() first.");
#endif
	}
	// Now you can get the pointer to the video structure.
#ifdef OpenRDK_ARCH_GENERIC
	//fLogProxy->info(getName(), "Taken a new image");
	bzero(imageData.data,sizeof(imageData.data));
	int imgWidth = 0;
	int imgHeight = 0;
	int imgNbLayers = 0;

	AL::getSizeFromResolution(AL::kQVGA, imgWidth, imgHeight);
	imgNbLayers = AL::getNumLayersInColorSpace(AL::kYUV422InterlacedColorSpace);
	imageData.width  = imgWidth;
	imageData.height = imgHeight;
	imageData.bpp    = imgNbLayers;
#else
	//ostringstream oss;
	//oss << "Current Camera #" << fCamProxy->call<int>( "getParam", AL::kCameraSelectID);
	//fLogProxy->info(getName(), oss.str());

	AL::ALImage* image = NULL;
	//image = (AL::ALImage*) (fCamProxy->call<int>("getDirectRawImageLocal", fGvmName)); /// test
	image = (AL::ALImage*) (fCamProxy->call<int>("getImageLocal", fGvmName));
	char* dataPointer = NULL;
	if (image != NULL)
	{
		dataPointer = (char*)image->getFrame();
		memcpy(imageData.data,dataPointer,image->getSize());
		imageData.width  = image->fWidth;
		imageData.height = image->fHeight;
		imageData.bpp    = image->fNbLayers;
	}
	fCamProxy->call<int>("releaseImage", fGvmName);
	//fCamProxy->call<int>("releaseDirectRawImage", fGvmName);
	//currentCamera = fCamProxy->call<int>("getParam", AL::kCameraSelectID);

	//imageData.selectedCameraID = currentCamera;
#endif
}

/// <summary>
/// Process commands from OpenRDK module
/// </summary>
void RSensorsServer::processCommands(RdkImageData& imageData)
{
#ifdef OpenRDK_ARCH_GENERIC
	usleep(1000);
#endif

	if (imageData.changeParams)
	{
		imageData.changeParams = false;
		fLogProxy->info(getName(), "OpenRDK: changing camera params");
#ifdef EXPERIMENTAL_FEATURE
		if (imageData.cameraParams[BOTTOMCAMERA].kCameraResolutionID != currentResolution
				||
				imageData.cameraParams[BOTTOMCAMERA].kCameraColorSpaceID != currentColorspace
				||
				imageData.cameraParams[BOTTOMCAMERA].kCameraFrameRateID != currentFps)
		{
			fLogProxy->warn(getName(), "Changing main camera parameters (resolution, colorspace or fps)");
			unRegisterFromVIM();
			registerToVIM(
					imageData.cameraParams[BOTTOMCAMERA].kCameraResolutionID,
					imageData.cameraParams[BOTTOMCAMERA].kCameraColorSpaceID,
					imageData.cameraParams[BOTTOMCAMERA].kCameraFrameRateID);
			try
			{
				fCamProxy->callVoid( "setParam", AL::kCameraSelectID, imageData.selectedCameraID );
				currentCamera = imageData.selectedCameraID;
			}
			catch (const AL::ALError& e)
			{
				std::cout << "error: AL::kCameraSelectID" << AL::kCameraSelectID << std::endl;
			}
		}
#endif

		if (currentCamera != imageData.selectedCameraID)
		{
			ostringstream oss;
			oss << "Changing selected camera, was #" << currentCamera << " would be #" << imageData.selectedCameraID << endl;
			fLogProxy->warn(getName(), oss.str());
			try
			{
				fCamProxy->callVoid( "setParam", AL::kCameraSelectID, imageData.selectedCameraID );
				currentCamera = imageData.selectedCameraID;
			}
			catch (const AL::ALError& e)
			{
				std::cout << "error: AL::kCameraSelectID" << AL::kCameraSelectID << std::endl;
			}
		}

#ifdef EXPERIMENTAL_FEATURE
		if (imageData.fastSwitchCamera)
		{
			try
			{
				fLogProxy->warn(getName(), "Fast switching camera");
				fCamProxy->callVoid( "setParam", AL::kCameraFastSwitchID);
				imageData.fastSwitchCamera = false;
			}
			catch (const AL::ALError& e)
			{
				std::cout << "error: AL::kCameraFastSwitchID" << std::endl;
			}
		}
#endif
		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraBrightnessID, imageData.cameraParams[imageData.selectedCameraID].kCameraBrightnessID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraBrightnessID" << AL::kCameraBrightnessID << std::endl;
		}
		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraContrastID, imageData.cameraParams[imageData.selectedCameraID].kCameraContrastID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraContrastID" << AL::kCameraContrastID << std::endl;
		}
		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraSaturationID, imageData.cameraParams[imageData.selectedCameraID].kCameraSaturationID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraSaturationID" << AL::kCameraSaturationID << std::endl;
		}
		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraHueID, imageData.cameraParams[imageData.selectedCameraID].kCameraHueID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraHueID" << AL::kCameraHueID << std::endl;
		}
		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraRedChromaID, imageData.cameraParams[imageData.selectedCameraID].kCameraRedChromaID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraRedChromaID" << AL::kCameraRedChromaID << std::endl;
		}
		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraBlueChromaID, imageData.cameraParams[imageData.selectedCameraID].kCameraBlueChromaID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraBlueChromaID" << AL::kCameraBlueChromaID << std::endl;
		}
		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraHFlipID, imageData.cameraParams[imageData.selectedCameraID].kCameraHFlipID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraHFlipID" << AL::kCameraHFlipID << std::endl;
		}
		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraVFlipID, imageData.cameraParams[imageData.selectedCameraID].kCameraVFlipID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraVFlipID" << AL::kCameraVFlipID << std::endl;
		}
		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraLensXID, imageData.cameraParams[imageData.selectedCameraID].kCameraLensXID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraLensXID" << AL::kCameraLensXID << std::endl;
		}
		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraLensYID, imageData.cameraParams[imageData.selectedCameraID].kCameraLensYID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraLensYID" << AL::kCameraLensYID << std::endl;
		}
		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraAutoGainID, imageData.cameraParams[imageData.selectedCameraID].kCameraAutoGainID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraAutoGainID" << AL::kCameraAutoGainID << std::endl;
		}

		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraGainID, imageData.cameraParams[imageData.selectedCameraID].kCameraGainID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraGainID" << AL::kCameraGainID << std::endl;
		}

		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraAutoExpositionID, imageData.cameraParams[imageData.selectedCameraID].kCameraAutoExpositionID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraAutoExpositionID" << AL::kCameraAutoExpositionID << std::endl;
		}

		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraExposureCorrectionID, imageData.cameraParams[imageData.selectedCameraID].kCameraExposureCorrectionID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraExposureCorrectionID" << AL::kCameraExposureCorrectionID << std::endl;
		}

		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraExposureID, imageData.cameraParams[imageData.selectedCameraID].kCameraExposureID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraExposureID" << AL::kCameraExposureID << std::endl;
		}

		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraAutoWhiteBalanceID, imageData.cameraParams[imageData.selectedCameraID].kCameraAutoWhiteBalanceID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraAutoWhiteBalanceID" << AL::kCameraAutoWhiteBalanceID << std::endl;
		}
	}
}

/// <summary>
/// Read sensor values into the shared memory
/// </summary>
void RSensorsServer::sensorReader(RdkSensorData& sensorData)
{
	fMemoryFastAccess->GetValues(sensorValues);

	sensorData.timestamp = 0;//getTimeStamp();

	sensorData.jointsValues[Nao::NaoJoints::HEAD_PITCH] = sensorValues[JHeadPitch];
	sensorData.jointsValues[Nao::NaoJoints::HEAD_YAW] = sensorValues[JHeadYaw];
	sensorData.jointsValues[Nao::NaoJoints::L_ANKLE_PITCH] = sensorValues[JLAnklePitch];
	sensorData.jointsValues[Nao::NaoJoints::L_ANKLE_ROLL] = sensorValues[JLAnkleRoll];
	sensorData.jointsValues[Nao::NaoJoints::L_ELBOW_ROLL] = sensorValues[JLElbowRoll];
	sensorData.jointsValues[Nao::NaoJoints::L_ELBOW_YAW] = sensorValues[JLElbowYaw];
	//sensorData.jointsValues[Nao::NaoJoints::] = sensorValues[LHand];
	sensorData.jointsValues[Nao::NaoJoints::L_HIP_PITCH] = sensorValues[JLHipPitch];
	sensorData.jointsValues[Nao::NaoJoints::L_HIP_ROLL] = sensorValues[JLHipRoll];
	sensorData.jointsValues[Nao::NaoJoints::L_HIP_YAW_PITCH] = sensorValues[JLHipYawPitch];
	sensorData.jointsValues[Nao::NaoJoints::L_KNEE_PITCH] = sensorValues[JLKneePitch];
	sensorData.jointsValues[Nao::NaoJoints::L_SHOULDER_PITCH] = sensorValues[JLShoulderPitch];
	sensorData.jointsValues[Nao::NaoJoints::L_SHOULDER_ROLL] = sensorValues[JLShoulderRoll];
	//sensorData.jointsValues[Nao::NaoJoints::] = sensorValues[LWristYaw];
	sensorData.jointsValues[Nao::NaoJoints::R_ANKLE_PITCH] = sensorValues[JRAnklePitch];
	sensorData.jointsValues[Nao::NaoJoints::R_ANKLE_ROLL] = sensorValues[JRAnkleRoll];
	sensorData.jointsValues[Nao::NaoJoints::R_ELBOW_ROLL] = sensorValues[JRElbowRoll];
	sensorData.jointsValues[Nao::NaoJoints::R_ELBOW_YAW] = sensorValues[JRElbowYaw];
	//sensorData.jointsValues[Nao::NaoJoints::] = sensorValues[RHand];
	sensorData.jointsValues[Nao::NaoJoints::R_HIP_PITCH] = sensorValues[JRHipPitch];
	sensorData.jointsValues[Nao::NaoJoints::R_HIP_ROLL] = sensorValues[JRHipRoll];
	sensorData.jointsValues[Nao::NaoJoints::R_KNEE_PITCH] = sensorValues[JRKneePitch];
	sensorData.jointsValues[Nao::NaoJoints::R_SHOULDER_PITCH] = sensorValues[JRShoulderPitch];
	sensorData.jointsValues[Nao::NaoJoints::R_SHOULDER_ROLL] = sensorValues[JRShoulderRoll];
	//sensorData.jointsValues[Nao::NaoJoints::] = sensorValues[RWristYaw];

	sensorData.acc.x = sensorValues[ISAccX];
	sensorData.acc.y = sensorValues[ISAccY];
	sensorData.acc.z = sensorValues[ISAccZ];

	sensorData.gyr.x = sensorValues[ISGyrX];
	sensorData.gyr.y = sensorValues[ISGyrY];
	sensorData.gyr.ref = sensorValues[ISGyrRef];
	//sensorData.gyr[2] = sensorValues[ISGyrZ];//fMemoryFastAccess->call<AL::ALValue>("getData",std::string("Device/SubDeviceList/InertialSensor/GyrRef/Sensor/Value"),0);

	sensorData.angle.x = sensorValues[ISAngleX];
	sensorData.angle.y = sensorValues[ISAngleY];

	sensorData.battery.charge      = sensorValues[BatteryCharge];
	sensorData.battery.current     = sensorValues[BatteryCurrent];
	sensorData.battery.temperature = sensorValues[BatteryTemperature];

	sensorData.us.left             = sensorValues[USLeft];
	sensorData.us.right            = sensorValues[USRight];

	sensorData.fsr.left.x          = sensorValues[FSRLCenterOfPressureX];
	sensorData.fsr.left.y          = sensorValues[FSRLCenterOfPressureY];
	sensorData.fsr.left.weight     = sensorValues[FSRLTotalWeight];
	sensorData.fsr.right.x         = sensorValues[FSRRCenterOfPressureX];
	sensorData.fsr.right.y         = sensorValues[FSRRCenterOfPressureY];
	sensorData.fsr.right.weight    = sensorValues[FSRRTotalWeight];

	sensorData.bumpers.left.left = sensorValues[BumperLLeft];
	sensorData.bumpers.left.right = sensorValues[BumperLRight];
	sensorData.bumpers.right.left = sensorValues[BumperRLeft];
	sensorData.bumpers.right.right = sensorValues[BumperRRight];
	
	// The feedbacksensor from xsentinel
	if (! xsentinel_proxy)
	{
		xsentinel_proxy = getParentBroker()->getProxy("XSentinelModule");
		if (! xsentinel_proxy)
		{
			fLogProxy->lowInfo(getName(), "Can not get XSentinelModule");
		}
	}
	std::string state = "Initial";
	bool isBlueTeam = true;
	if (xsentinel_proxy)
	{
		state = xsentinel_proxy->call<std::string>("GetGameState");
		isBlueTeam = xsentinel_proxy->call<bool>("IsBlueTeam");
		
		std::stringstream ss;
		ss << "State: " << state << std::endl;
		ss << (isBlueTeam?"Blue" : "Red") << std::endl;
		fLogProxy->lowInfo(getName(),ss.str());
	}
	strcpy(sensorData.feedbackSensor.gamestateByXSentinel, state.c_str());
	sensorData.feedbackSensor.isBlueTeamByXSentinel = isBlueTeam;
	
	
	// The feedbacksensor from naomotionmodule
	if (! naomotionmodule_proxy)
	{
		naomotionmodule_proxy = getParentBroker()->getProxy("NaoMotionModule");
		if (! naomotionmodule_proxy)
		{
			fLogProxy->lowInfo(getName(), "Can not get NaoMotionModule");
		}
	}
	
	std::string motionModuleTask = "Stand";
	bool isHeadMoving = true;
	double odometryX = 0;
	double odometryY = 0;
	double odometryTheta = 0;
	if (naomotionmodule_proxy)
	{
		motionModuleTask = naomotionmodule_proxy->call<std::string>("Feedback_GetCurrentTask");
		isHeadMoving = naomotionmodule_proxy->call<bool>("IsHeadCommandExecuting");
		
		bool isUseSensorValue = true;
		AL::ALValue ret = naomotionmodule_proxy->call<AL::ALValue>("ALMotion_getRobotPosition", AL::ALValue(isUseSensorValue));
		odometryX = ret[0];
		odometryY = ret[1];
		odometryTheta = ret[2];
		
		std::stringstream ss;
		ss << "Task: " << motionModuleTask << std::endl;
		ss << (isHeadMoving?"HeadMoving" : "HeadStopped") << std::endl;
		ss << "ODOMETRY: " << odometryX << ", " << odometryY << ", " << odometryTheta << std::endl;
		fLogProxy->lowInfo(getName(),ss.str());
	}
	strcpy(sensorData.feedbackSensor.motionModuleTaskByNaoMotionModule, motionModuleTask.c_str());
	sensorData.feedbackSensor.isHeadCommandExecutingByNaoMotionModule = isHeadMoving;
	sensorData.feedbackSensor.odometryXByNaoMotionModule = odometryX;
	sensorData.feedbackSensor.odometryYByNaoMotionModule = odometryY;
	sensorData.feedbackSensor.odometryThetaByNaoMotionModule = odometryTheta;
}
