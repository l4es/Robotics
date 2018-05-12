/// <summary>
/// OpenRDK Vision server module.
/// </summary>
/// <remarks> OpenRDK Developers </remarks>

#include "rvisionserver.h"
#include <alproxy.h>
#include <alptr.h>
#include <albroker.h>
#include <almodule.h>

#include <alvisiondefinitions.h>
#include <alloggerproxy.h>
#include <alimage.h>

#include <rdkcore/sharedmemory/sensordatashm.h>
#include <rdkcore/config.h>

#include <sstream>

/// <summary>
/// OpenRDK Vision server module.
/// </summary>
/// <param name="broker"> A smart pointer to the broker.</param> 
/// <param name="name">   The name of the module. </param> 
RVisionServer::RVisionServer(AL::ALPtr<AL::ALBroker> broker, const std::string& name ): AL::ALModule(broker, name ),
	fRegisteredToVim(false),
	shmimage(NULL)
{
	setModuleDescription( "Simple Vision Server for handling Nao video device in OpenRDK." );

	// Define bound methods with their description.
	functionName( "registerToVIM", getName(), "Register to the V.I.M." );
	addParam("resolution", "Resolution requested.");
	addParam("colorSpace", "Colorspace requested.");
	BIND_METHOD( RVisionServer::registerToVIM );

	functionName( "unRegisterFromVIM", getName(), "Unregister from the V.I.M." );
	BIND_METHOD( RVisionServer::unRegisterFromVIM );

	functionName( "isPlugged", getName(), "Check if the Shared Memory is correctly plugged in." );
	setReturn( "isPlugged", "true if the shared memory is connected.");
	BIND_METHOD( RVisionServer::isPlugged );

}

/// <summary>
/// Destructor
/// </summary>
RVisionServer::~RVisionServer()
{
	unRegisterFromVIM();
	if (shmimage != NULL)
		delete shmimage;
	shmimage = NULL;
}

/// <summary>
/// Initialization method
/// </summary>
void RVisionServer::init()
{
	// Create a proxy to the logger module
	// If this fails, we throw an exception and the module will not be registered.
	try
	{
		fLogProxy = getParentBroker()->getLoggerProxy();
	}
	catch (const AL::ALError& e)
	{
		throw AL::ALError(getName(),"RVisionServer()","Fail to create a proxy to ALLogger module. Msg " + e.toString() + "\nModule will abort." );
	}

	// Create a proxy to video input device
	// If this fails, we throw an exception and the module will not be registered
	try
	{
		fCamProxy = getParentBroker()->getProxy("ALVideoDevice");
		registerToVIM(AL::kQVGA,AL::kYUV422InterlacedColorSpace);
	}
	catch ( AL::ALError& e )
	{
#ifdef OpenRDK_ARCH_GENERIC
		fLogProxy->warn(getName(),"Could not create a proxy to ALVideoDevice module");
#else
		throw AL::ALError(getName(),"RVisionSever()","Fail to create a proxy to ALVideoDevice module. Msg " + e.toString() + "\nModule will abort." );
#endif
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

	fLogProxy->info(getName()," Init IMAGE done.");
}

/// <summary>
/// Register to the V.I.M.
/// </summary>
/// <param name="pResolution"> Resolution requested. </param> 
/// <param name="pColorSpace"> Colorspace requested. </param> 
void RVisionServer::registerToVIM(const int &pResolution, const int &pColorSpace) {

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
	//const int kImgDepth = 8;
	const int kFps = 30;

	AL::getSizeFromResolution(pResolution, imgWidth, imgHeight);
	imgNbLayers = AL::getNumLayersInColorSpace(pColorSpace);

	if (imgWidth == -1 || imgWidth == -1 || imgNbLayers == -1) {
		throw AL::ALError(getName(), "registerToVIM()", "Invalid resolution or color space.");
	}

	// Call the "subscribe" function with the given parameters.
	fGvmName = fCamProxy->call<std::string>("subscribe", kOriginalName,
			pResolution, pColorSpace, kFps );

	fLogProxy->info(getName(), " module registered as " + fGvmName);

	// Registration is successful, set fRegisteredToVim to true.
	fRegisteredToVim = true;
}


/// <summary>
/// Unregister from the V.I.M.
/// </summary>
void RVisionServer::unRegisterFromVIM() {

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
/// Process commands from OpenRDK module
/// </summary>
void RVisionServer::processCommands()
{
#ifdef OpenRDK_ARCH_GENERIC
	usleep(1000);
#endif
	RdkImageData *imageData = static_cast<RdkImageData*>(shmimage->getEntry());
	if (imageData->changeParams)
	{
		imageData->changeParams = false;
		std::cout << "OpenRDK: changing camera params" << std::endl;
		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraSelectID, imageData->selectedCameraID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraSelectID" << AL::kCameraSelectID << std::endl;
		}
		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraBrightnessID, imageData->cameraParams[imageData->selectedCameraID].kCameraBrightnessID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraBrightnessID" << AL::kCameraBrightnessID << std::endl;
		}
		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraContrastID, imageData->cameraParams[imageData->selectedCameraID].kCameraContrastID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraContrastID" << AL::kCameraContrastID << std::endl;
		}
		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraSaturationID, imageData->cameraParams[imageData->selectedCameraID].kCameraSaturationID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraSaturationID" << AL::kCameraSaturationID << std::endl;
		}
		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraHueID, imageData->cameraParams[imageData->selectedCameraID].kCameraHueID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraHueID" << AL::kCameraHueID << std::endl;
		}
		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraRedChromaID, imageData->cameraParams[imageData->selectedCameraID].kCameraRedChromaID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraRedChromaID" << AL::kCameraRedChromaID << std::endl;
		}
		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraBlueChromaID, imageData->cameraParams[imageData->selectedCameraID].kCameraBlueChromaID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraBlueChromaID" << AL::kCameraBlueChromaID << std::endl;
		}
		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraHFlipID, imageData->cameraParams[imageData->selectedCameraID].kCameraHFlipID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraHFlipID" << AL::kCameraHFlipID << std::endl;
		}
		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraVFlipID, imageData->cameraParams[imageData->selectedCameraID].kCameraVFlipID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraVFlipID" << AL::kCameraVFlipID << std::endl;
		}
		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraLensXID, imageData->cameraParams[imageData->selectedCameraID].kCameraLensXID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraLensXID" << AL::kCameraLensXID << std::endl;
		}
		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraLensYID, imageData->cameraParams[imageData->selectedCameraID].kCameraLensYID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraLensYID" << AL::kCameraLensYID << std::endl;
		}
		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraAutoGainID, imageData->cameraParams[imageData->selectedCameraID].kCameraAutoGainID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraAutoGainID" << AL::kCameraAutoGainID << std::endl;
		}

		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraGainID, imageData->cameraParams[imageData->selectedCameraID].kCameraGainID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraGainID" << AL::kCameraGainID << std::endl;
		}

		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraAutoExpositionID, imageData->cameraParams[imageData->selectedCameraID].kCameraAutoExpositionID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraAutoExpositionID" << AL::kCameraAutoExpositionID << std::endl;
		}

		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraExposureCorrectionID, imageData->cameraParams[imageData->selectedCameraID].kCameraExposureCorrectionID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraExposureCorrectionID" << AL::kCameraExposureCorrectionID << std::endl;
		}

		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraExposureID, imageData->cameraParams[imageData->selectedCameraID].kCameraExposureID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraExposureID" << AL::kCameraExposureID << std::endl;
		}

		try
		{
			fCamProxy->callVoid( "setParam", AL::kCameraAutoWhiteBalanceID, imageData->cameraParams[imageData->selectedCameraID].kCameraAutoWhiteBalanceID );
		}
		catch (const AL::ALError& e)
		{
			std::cout << "error: AL::kCameraAutoWhiteBalanceID" << AL::kCameraAutoWhiteBalanceID << std::endl;
		}
	}
}

/// <summary>
/// Shared memory handler function for vision task
/// </summary>
void RVisionServer::visionTask() // to rdk module
{
	while (true)
	{
		shmimage->wait(NAOQI);
#ifdef OpenRDK_ARCH_GENERIC
		usleep(1000);
#endif

		if (!fRegisteredToVim)
		{
#ifndef OpenRDK_ARCH_GENERIC
			throw AL::ALError(getName(), "visionTask()",  "No video module is currently "
					"registered! Call registerToVIM() first.");
#endif
		}
		// Now you can get the pointer to the video structure.
#ifdef OpenRDK_ARCH_GENERIC
		//fLogProxy->info(getName(), "Taken a new image");
#else
		AL::ALImage* image = NULL;
		image = (AL::ALImage*) (fCamProxy->call<int>("getImageLocal", fGvmName));
#endif

		RdkImageData *imageData        = static_cast<RdkImageData*>(shmimage->getEntry());
#ifdef OpenRDK_ARCH_GENERIC
		bzero(imageData->data,sizeof(imageData->data));
		int imgWidth = 0;
		int imgHeight = 0;
		int imgNbLayers = 0;

		AL::getSizeFromResolution(AL::kQVGA, imgWidth, imgHeight);
		imgNbLayers = AL::getNumLayersInColorSpace(AL::kYUV422InterlacedColorSpace);
		imageData->width  = imgWidth;
		imageData->height = imgHeight;
		imageData->bpp    = imgNbLayers;
#else
		char* dataPointer = NULL;
		if (image != NULL)
		{
			dataPointer = (char*)image->getFrame();
			memcpy(imageData->data,dataPointer,image->getSize());
			imageData->width  = image->fWidth;
			imageData->height = image->fHeight;
			imageData->bpp    = image->fNbLayers;
		}
		fCamProxy->call<int>("releaseImage", fGvmName);

		processCommands();
#endif
		shmimage->signal(RDK);
	}
}
