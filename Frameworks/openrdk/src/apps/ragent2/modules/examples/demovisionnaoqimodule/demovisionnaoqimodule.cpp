#define MODULE_NAME "DemoVisionNaoQiModule"
#define OpenRDK_ARCH_NOTLINUX OpenRDK_ARCH_GEODE || OpenRDK_ARCH_ATOM
#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/time/time.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE MODULE_NAME

#include "demovisionnaoqimodule.h"

#include <alvisiondefinitions.h>
#include <alerror.h>
#include <vector>

//NaoQi SDK 1.2.0 compatibility
#if(NaoQi_VERSION_MAJOR == 1 && NaoQi_VERSION_MINOR == 2)
#include <alvisionimage.h>
#else
#include <alimage.h>
#endif

const std::string PARENT_BROKER_ADDRESS = "127.0.0.1";

// see alvisiondefinitions.h


namespace RDK2 { namespace RAgent
{

	bool DemoVisionNaoQiModule::initConfigurationProperties()
	{
		SESSION_TRY_START(session)
			// put false in the second parameter if you want the module disabled for default
			// add a third parameter valued false if you don't want the state property
			Common::createDefaultProperties(session, true);
		session->createBool(PROPERTY_CMD_CONNECT, "Connect to NaoQi", false);
		session->setVolatile(PROPERTY_CMD_CONNECT);
		session->createBool(PROPERTY_CMD_CHANGE_CAMERA_PARAMS, "Reload camera parameters", false);
		session->setVolatile(PROPERTY_CMD_CHANGE_CAMERA_PARAMS);

		// connection stuffs
		session->createString(PROPERTY_PARAMS_CONNECT_PARENT_BROKER, "NaoQi parent broker address",PARENT_BROKER_ADDRESS);
		session->createInt(PROPERTY_PARAMS_CONNECT_PARENT_BROKER_PORT, "NaoQi parent broker port", kBrokerPort); // defined in altypes.h
		session->createString(PROPERTY_PARAMS_CONNECT_BROKER, "NaoQi broker address", "");
		session->createInt(PROPERTY_PARAMS_CONNECT_BROKER_PORT, "NaoQi broker port", 0);

		////////////////// camera parameters
		// resolution
		ENUM_CREATE(resolution);
		ENUM_ITEM(resolution, kVGA, "High Resolution", "Use the highest resolution (640 * 480)");
		ENUM_ITEM(resolution, kQVGA, "Medium Resolution", "Use the medium resolution (320 * 240)");
		ENUM_ITEM(resolution, kQQVGA, "Low Resolution", "Use the lowest resolution (160 * 120)");
		ostringstream oss;
		oss << "Resolution ("
		    << kVGA << "=high (640x480), "
				<< kQVGA << "=mid (320x240), "
				<< kQQVGA << "=low (160x120) )";
		session->createEnum(PROPERTY_PARAMS_Resolution, oss.str(), resolution, kQVGA);

		// colorspace
		ENUM_CREATE(colorspace);
		ENUM_ITEM(colorspace, kYuvColorSpace, "kYuvColorSpace", "kYuvColorSpace");
		ENUM_ITEM(colorspace, kyUvColorSpace, "kyUvColorSpace", "kyUvColorSpace");
		ENUM_ITEM(colorspace, kyuVColorSpace, "kyuVColorSpace", "kyuVColorSpace");
		ENUM_ITEM(colorspace, kRgbColorSpace, "kRgbColorSpace", "kRgbColorSpace");
		ENUM_ITEM(colorspace, krGbColorSpace, "krGbColorSpace", "krGbColorSpace");
		ENUM_ITEM(colorspace, krgBColorSpace, "krgBColorSpace", "krgBColorSpace");
//NaoQi SDK 1.2.0 compatibility
#if(NaoQi_VERSION_MAJOR == 1 && NaoQi_VERSION_MINOR == 2)
		ENUM_ITEM(colorspace, kHsvColorSpace, "kHsvColorSpace", "kHsvColorSpace");
		ENUM_ITEM(colorspace, khSvColorSpace, "khSvColorSpace", "khSvColorSpace");
		ENUM_ITEM(colorspace, khsVColorSpace, "khsVColorSpace", "khsVColorSpace");
#else
		ENUM_ITEM(colorspace, kHsyColorSpace, "kHsyColorSpace", "kHsyColorSpace");
		ENUM_ITEM(colorspace, khSyColorSpace, "khSyColorSpace", "khSyColorSpace");
		ENUM_ITEM(colorspace, khsYColorSpace, "khsYColorSpace", "khsYColorSpace");
#endif
		ENUM_ITEM(colorspace, kYUV422InterlacedColorSpace, "kYUV422InterlacedColorSpace (raw)", "kYUV422InterlacedColorSpace");
		ENUM_ITEM(colorspace, kYUVColorSpace, "kYUVColorSpace", "kYUVColorSpace");
		ENUM_ITEM(colorspace, kRGBColorSpace, "kRGBColorSpace", "kRGBColorSpace");
//NaoQi SDK 1.2.0 compatibility
#if(NaoQi_VERSION_MAJOR == 1 && NaoQi_VERSION_MINOR == 2)
		ENUM_ITEM(colorspace, kHSVColorSpace, "kHSVColorSpace", "kHSVColorSpace");
#else
		ENUM_ITEM(colorspace, kHSYColorSpace, "kHSYColorSpace", "kHSYColorSpace");
#endif
		ENUM_ITEM(colorspace, kBGRColorSpace, "kBGRColorSpace", "kBGRColorSpace"); //for opencv ease of use
		ENUM_ITEM(colorspace, kYYCbCrColorSpace, "kYYCbCrColorSpace", "kYYCbCrColorSpace"); //for tiff io implementation
		ENUM_ITEM(colorspace, kH2RGBColorSpace, "kH2RGBColorSpace", "kH2RGBColorSpace"); //H from HSV to RGB in fake colors
		ENUM_ITEM(colorspace, kHSMixedColorSpace, "kHSMixedColorSpace", "kHSMixedColorSpace"); //HS and (H +S)/2
		oss.str("");
		oss << "Colorspace ("
			<< kYuvColorSpace << "=Yuv, "
			<< kyUvColorSpace << "=yUv, "
			<< kyuVColorSpace << "=yuV, "
			<< kRgbColorSpace << "=Rgb, "
			<< krGbColorSpace << "=rGb, "
			<< krgBColorSpace << "=rgB, "
//NaoQi SDK 1.2.0 compatibility
#if(NaoQi_VERSION_MAJOR == 1 && NaoQi_VERSION_MINOR == 2)
			<< kHsvColorSpace << "=Hsv, "
			<< khSvColorSpace << "=hSv, "
			<< khsVColorSpace << "=hsV, "
#else
			<< kHsyColorSpace << "=Hsy, "
			<< khSyColorSpace << "=hSy, "
			<< khsYColorSpace << "=hsY, "
#endif
			<< kYUV422InterlacedColorSpace << "=YUV422Interlaced, "
			<< kYUVColorSpace << "=YUV, "
			<< kRGBColorSpace << "=RGB, "
//NaoQi SDK 1.2.0 compatibility
#if(NaoQi_VERSION_MAJOR == 1 && NaoQi_VERSION_MINOR == 2)
			<< kHSVColorSpace << "=HSV, "
#else
			<< kHSYColorSpace << "=HSY, "
#endif
			<< kBGRColorSpace << "=BGR, " //for opencv ease of use
			<< kYYCbCrColorSpace << "=YYCbCr, " //for tiff io implementation 
			<< kH2RGBColorSpace << "=H2RGB, " //H from HSV to RGB in fake colors
			<< kHSMixedColorSpace << "=HSMixed, " //HS and (H +S)/2
			<< ")";
		session->createEnum(PROPERTY_PARAMS_Colorspace, oss.str(), colorspace, kYUV422InterlacedColorSpace);
		session->createInt(PROPERTY_PARAMS_FrameRate, "Frame rate: 5, 10, 15, 30", 10);

		// scaling method
		ENUM_CREATE(scale);
		ENUM_ITEM(scale, kSimpleScaleMethod, "kSimpleScaleMethod", "kSimpleScaleMethod");
		ENUM_ITEM(scale, kAverageScaleMethod, "kAverageScaleMethod", "kAverageScaleMethod");
		ENUM_ITEM(scale, kQualityScaleMethod, "kQualityScaleMethod", "kQualityScaleMethod");
		ENUM_ITEM(scale, kNoScaling, "kNoScaling", "kNoScaling");

		// other parameters
		session->createInt(PROPERTY_PARAMS_CAMERA_Brightness, "Brightness [0..255]", 128);
		session->createInt(PROPERTY_PARAMS_CAMERA_Contrast, "Contrast [0..127]", 64);
		session->createInt(PROPERTY_PARAMS_CAMERA_Saturation, "Saturation [0..255]", 128);
		session->createInt(PROPERTY_PARAMS_CAMERA_Hue, "Hue [-180..180]", 0);
		session->createInt(PROPERTY_PARAMS_CAMERA_RedChroma, "RedChroma [0..255]", 128);
		session->createInt(PROPERTY_PARAMS_CAMERA_BlueChroma, "BlueChroma [0..255]", 128);
		session->createInt(PROPERTY_PARAMS_CAMERA_Gain, "Gain [0..255]", 128);
		session->createInt(PROPERTY_PARAMS_CAMERA_HFlip, "HFlip [0,1]", 0);
		session->createInt(PROPERTY_PARAMS_CAMERA_VFlip, "VFlip [0,1]", 0);
		session->createInt(PROPERTY_PARAMS_CAMERA_LensX, "LensX [0..255]", 0);
		session->createInt(PROPERTY_PARAMS_CAMERA_LensY, "LensY [0..255]", 0);
		session->createInt(PROPERTY_PARAMS_CAMERA_AutoExposition, "AutoExposition [0,1]", 0);
		session->createInt(PROPERTY_PARAMS_CAMERA_AutoWhiteBalance, "AutoWhiteBalance [0,1]", 0);
		session->createInt(PROPERTY_PARAMS_CAMERA_AutoGain, "AutoGain [0,1]", 0);
		session->createInt(PROPERTY_PARAMS_CAMERA_Exposure, "Exposure (time in ms = value x 33/480) [0..480 (number of lines of the sensor)]", 240);
		session->createInt(PROPERTY_PARAMS_CAMERA_ExposureCorrection, "ExposureCorrection", 0);
		session->createInt(PROPERTY_PARAMS_CAMERA_Select, "0=Top, 1=Bottom", 1);

		// FIXME remove magic number
		session->createImage(PROPERTY_OUT_IMAGE, "Current Image", 320, 240, RDK2::RGraphics::RImage::YUYV);

		SESSION_END(session)
			return true;
		SESSION_CATCH_TERMINATE(session)
			return false;
	}

	// bool DemoVisionNaoQiModule::initInterfaceProperties() { }

	bool DemoVisionNaoQiModule::init()
	{
		// in the init() function you should initialize everything your module needs (drivers initialization,
		// sockets, and so on); all modules wait for all init() before they start
		SESSION_TRY_START(session)

		session->listen(PROPERTY_CMD_CONNECT);
		session->listen(PROPERTY_CMD_CHANGE_CAMERA_PARAMS);
		double fr = 1000.0/session->getInt(PROPERTY_PARAMS_FrameRate);
		RDK_INFO_STREAM("Listeninig to " << fr << " ms" << session->getInt(PROPERTY_PARAMS_FrameRate));
		session->listenToTimer(fr);
#ifdef OpenRDK_ARCH_NOTLINUX
		shmimage = new SharedMemory(SharedMemory::IMAGE, sizeof(RdkImageData));
#endif

//		session->registerTimerEventHandler(SESSION_EVENT(timerEvent));
		session->registerPropertyUpdateEventHandler(SESSION_EVENT(doConnection), PROPERTY_CMD_CONNECT);
		session->registerPropertyUpdateEventHandler(SESSION_EVENT(doChangeCameraParams), PROPERTY_CMD_CHANGE_CAMERA_PARAMS);
		//session->registerPropertyUpdateEventHandler(SESSION_EVENT(doChangeResolution), PROPERTY_PARAMS_Resolution);
		//session->registerPropertyUpdateEventHandler(SESSION_EVENT(doChangeColorspace), PROPERTY_PARAMS_Colorspace);
		//session->registerPropertyUpdateEventHandler(SESSION_EVENT(doChangeFrameRate), PROPERTY_PARAMS_FrameRate);
		
		saveCameraParams(TOPCAMERA);
		saveCameraParams(BOTTOMCAMERA);
		connected = false;
		selectedCameraID = 1;
#ifdef OpenRDK_ARCH_NOTLINUX
		shmimage->signal(NAOQI);
#endif

		session->setBool(PROPERTY_CMD_CONNECT, true);
		SESSION_END(session)
			return true;
		SESSION_CATCH_TERMINATE(session)
			return false;
	}

	void DemoVisionNaoQiModule::exec()
	{
#ifdef OpenRDK_ARCH_NOTLINUX
		while (session->dontWait(), !exiting)
		{
			SESSION_TRY_START(session)
			shmimage->wait(RDK);
#else
		while (session->wait(), !exiting)
		{
			SESSION_TRY_START(session);
#endif
			session->processEvents();

			if (connected)
			{
				Timestamp t;
#ifdef OpenRDK_ARCH_NOTLINUX
				//if (!session->isLocked(PROPERTY_OUT_IMAGE))
				{
					RdkImageData *image         = static_cast<RdkImageData*>(shmimage->getEntry());
					unsigned char * dataPointer = image->data;
					bufferSize                  = image->width*image->height*image->bpp;
#else
				ALValue image;
				image.arraySetSize(7);

				//Now you can get the data.
				image = cameraProxy->call<ALValue>( "getImageRemote", pID );

				// You can get the pointer to the image data and its size
				const char* dataPointer =  static_cast<const char*>(image[6].GetBinary());
#endif
					session->lock(PROPERTY_OUT_IMAGE, HERE);
					RImage *rimage = session->getObjectAsL<RImage>(PROPERTY_OUT_IMAGE);
					if (dataPointer != NULL)
					{
						if (bufferSize != rimage->getBufferSize())
						{
							RDK_ERROR_STREAM("Trying to put image of size " << bufferSize << " in a buffer of size " << rimage->getBufferSize());
						}
						else
						{
							unsigned char *buf = rimage->getBuffer();
							memcpy(buf,dataPointer,bufferSize);
						}
					}
					else
					{
						RDK_ERROR_STREAM("Null datapointer");
					}

					session->unlock(PROPERTY_OUT_IMAGE);
					session->valueChanged(PROPERTY_OUT_IMAGE);
					//RDK_TRACE_STREAM("Retrieving image time " << Timestamp().getMsFromMidnight()-t.getMsFromMidnight());

#ifdef OpenRDK_ARCH_NOTLINUX
				} //if (!session->isLocked(PROPERTY_OUT_IMAGE))
				shmimage->signal(NAOQI);
#else
				cameraProxy->call<int>( "releaseImage", pID);
#endif
			} //if (connected)

			SESSION_END_CATCH_TERMINATE(session)
		}
	}

	bool DemoVisionNaoQiModule::doConnection(const Event* e)
	{
		const EventPropertyUpdate* event = dynamic_cast<const EventPropertyUpdate*>(e);
		if (!event)
		{
			RDK_DEBUG_PRINTF("Something wrong, this seems not to be an EventPropertyUpdate object (it is of class '%s')", e->type.c_str());
			return false;
		}
		RDK_INFO_STREAM("Trying to connect");

#ifdef OpenRDK_ARCH_NOTLINUX
		RdkImageData *image = static_cast<RdkImageData*>(shmimage->getEntry());
		image->cameraParams[TOPCAMERA]    = cameraParams[TOPCAMERA];
		image->cameraParams[BOTTOMCAMERA] = cameraParams[BOTTOMCAMERA];
		connected   = true;
		session->setBool(PROPERTY_CMD_CHANGE_CAMERA_PARAMS, true);
#else

		if (connected)
		{
			connected = false;
			// FIXME disconnect handling
			session->setBool(PROPERTY_CMD_CONNECT, false);
			return true;
		}

		try
		{
			int brokerPort = session->getInt(PROPERTY_PARAMS_CONNECT_BROKER_PORT);
			std::string brokerIP = session->getString(PROPERTY_PARAMS_CONNECT_BROKER);
			if (brokerPort == 0)
			{
//NaoQi SDK 1.2.0 compatibility
#if(NaoQi_VERSION_MAJOR == 1 && NaoQi_VERSION_MINOR == 2)
				brokerPort = FindFreePort(brokerIP);
#endif
			}

			ALBroker::Ptr broker = AL::ALBroker::createBroker("SPQRBroker", brokerIP, brokerPort, session->getString(PROPERTY_PARAMS_CONNECT_PARENT_BROKER), session->getInt(PROPERTY_PARAMS_CONNECT_PARENT_BROKER_PORT));
			cameraProxy = broker->getProxy( "NaoCam" );
#if(! (NaoQi_VERSION_MAJOR == 1 && NaoQi_VERSION_MINOR == 2))
			brokerPort = broker->getPort();
			session->setInt(PROPERTY_PARAMS_CONNECT_BROKER_PORT,brokerPort);
#endif
			RDK_INFO_STREAM("Connected to NaoQi Brokers: "

					<< "[parent " << session->getString(PROPERTY_PARAMS_CONNECT_PARENT_BROKER) << ":"
					<< session->getInt(PROPERTY_PARAMS_CONNECT_PARENT_BROKER_PORT) << "] "
					<< "[main " << brokerIP <<  ":"
					<< brokerPort << "]"
					);
		}
		catch ( AL::ALError e )
		{
			RDK_ERROR_STREAM ( "Cannot connect to NaoQi: " << e.toString() );
			connected = false;
		}
		// register Generic Video Module to Video Input Module
		try
		{
			pID = cameraProxy->call<std::string>("register", "SPQR-GVM", session->getInt(PROPERTY_PARAMS_Resolution), session->getInt(PROPERTY_PARAMS_Colorspace), session->getInt(PROPERTY_PARAMS_FrameRate));
			connected = true;
			RDK_INFO_STREAM("Registering " << pID);
		}
		catch (const ALError&)
		{
			RDK_ERROR_STREAM("Could not call the register method.");
		}
		session->setBool(PROPERTY_CMD_CHANGE_CAMERA_PARAMS, true);
#endif
		return connected;
	}

	bool DemoVisionNaoQiModule::doChangeCameraParams(const Event* e)
	{
		const EventPropertyUpdate* event = dynamic_cast<const EventPropertyUpdate*>(e);
		if (!session->getBool(PROPERTY_CMD_CHANGE_CAMERA_PARAMS))
		{
			return false;
		}
		session->setBool(PROPERTY_CMD_CHANGE_CAMERA_PARAMS, false);
		if (!event)
		{
			RDK_DEBUG_PRINTF("Something wrong, this seems not to be an EventPropertyUpdate object (it is of class '%s')", e->type.c_str());
			return false;
		}

		if (selectedCameraID != session->getInt(PROPERTY_PARAMS_CAMERA_Select))
		{
			saveCameraParams(selectedCameraID);
			selectedCameraID = 1;//session->getInt(PROPERTY_PARAMS_CAMERA_Select);
			restoreCameraParams(session->getInt(PROPERTY_PARAMS_CAMERA_Select));
#ifndef OpenRDK_ARCH_NOTLINUX
			cameraProxy->callVoid( "setParam", kCameraSelectID, selectedCameraID );
#endif
		}

#ifdef OpenRDK_ARCH_NOTLINUX
		RdkImageData *image = static_cast<RdkImageData*>(shmimage->getEntry());
		image->cameraParams[TOPCAMERA]    = cameraParams[TOPCAMERA];
		image->cameraParams[BOTTOMCAMERA] = cameraParams[BOTTOMCAMERA];
		image->selectedCameraID = selectedCameraID;
		image->changeParams = true;
#else
		cameraProxy->callVoid( "setParam", kCameraBrightnessID, session->getInt(PROPERTY_PARAMS_CAMERA_Brightness) );
		cameraProxy->callVoid( "setParam", kCameraContrastID, session->getInt(PROPERTY_PARAMS_CAMERA_Contrast) );
		cameraProxy->callVoid( "setParam", kCameraSaturationID, session->getInt(PROPERTY_PARAMS_CAMERA_Saturation) );
		cameraProxy->callVoid( "setParam", kCameraHueID, session->getInt(PROPERTY_PARAMS_CAMERA_Hue) );
		cameraProxy->callVoid( "setParam", kCameraRedChromaID, session->getInt(PROPERTY_PARAMS_CAMERA_RedChroma) );
		cameraProxy->callVoid( "setParam", kCameraBlueChromaID, session->getInt(PROPERTY_PARAMS_CAMERA_BlueChroma) );
		cameraProxy->callVoid( "setParam", kCameraHFlipID, session->getInt(PROPERTY_PARAMS_CAMERA_HFlip) );
		cameraProxy->callVoid( "setParam", kCameraVFlipID, session->getInt(PROPERTY_PARAMS_CAMERA_VFlip) );
		cameraProxy->callVoid( "setParam", kCameraLensXID, session->getInt(PROPERTY_PARAMS_CAMERA_LensX) );
		cameraProxy->callVoid( "setParam", kCameraLensYID, session->getInt(PROPERTY_PARAMS_CAMERA_LensY) );
		cameraProxy->callVoid( "setParam", kCameraAutoGainID, session->getInt(PROPERTY_PARAMS_CAMERA_AutoGain) );
		cameraProxy->callVoid( "setParam", kCameraGainID, session->getInt(PROPERTY_PARAMS_CAMERA_Gain) );
		cameraProxy->callVoid( "setParam", kCameraAutoExpositionID, session->getInt(PROPERTY_PARAMS_CAMERA_AutoExposition) );
		cameraProxy->callVoid( "setParam", kCameraExposureCorrectionID, session->getInt(PROPERTY_PARAMS_CAMERA_ExposureCorrection) );
		cameraProxy->callVoid( "setParam", kCameraExposureID, session->getInt(PROPERTY_PARAMS_CAMERA_Exposure) );
		cameraProxy->callVoid( "setParam", kCameraAutoWhiteBalanceID, session->getInt(PROPERTY_PARAMS_CAMERA_AutoWhiteBalance) );
#endif
		sleep(10);
		RDK_INFO_STREAM("Changing camera parameters.");
		return true;
	}

	bool DemoVisionNaoQiModule::doChangeResolution(const Event* e)
	{
		const EventPropertyUpdate* event = dynamic_cast<const EventPropertyUpdate*>(e);
		if (!event)
		{
			RDK_DEBUG_PRINTF("Something wrong, this seems not to be an EventPropertyUpdate object (it is of class '%s')", e->type.c_str());
			return false;
		}

		if (!cameraProxy->call<bool>( "setResolution", pID, session->getInt(PROPERTY_PARAMS_Resolution) ))
		{
			RDK_ERROR_STREAM("Error setting " << session->getInt(PROPERTY_PARAMS_Resolution) << " resolution");
			return false;
		}

		cameraParams[selectedCameraID].kCameraResolutionID = session->getInt(PROPERTY_PARAMS_Resolution);

		int w,h,l;
		getSizeFromResolution(cameraParams[selectedCameraID].kCameraResolutionID, w, h);
		l = getNumLayersInColorSpace(session->getInt(PROPERTY_PARAMS_Colorspace));
		bufferSize = w*h*l;
		

		session->lock(PROPERTY_OUT_IMAGE, HERE);
		RImage *rimage = session->getObjectAsL<RImage>(PROPERTY_OUT_IMAGE);
		rimage->canvasResize(w,h,0,0,0);
		session->unlock(PROPERTY_OUT_IMAGE);


		return true;
	}

	bool DemoVisionNaoQiModule::doChangeColorspace(const Event* e)
	{
		const EventPropertyUpdate* event = dynamic_cast<const EventPropertyUpdate*>(e);
		if (!event)
		{
			RDK_DEBUG_PRINTF("Something wrong, this seems not to be an EventPropertyUpdate object (it is of class '%s')", e->type.c_str());
			return false;
		}

		RDK_DEBUG_STREAM("Colorspace conversion not implemented yet");
		session->setInt(PROPERTY_PARAMS_Colorspace, kYUV422InterlacedColorSpace);
		return true;

		//bool res = cameraProxy->call<bool>( "setColorSpace", pID, session->getInt(PROPERTY_PARAMS_Colorspace) );
		//if (!res)
		//{
		//  RDK_ERROR_STREAM("Error setting " << session->getInt(PROPERTY_PARAMS_Colorspace) << " colorspace");
		//  return false;
		//}
	}

	bool DemoVisionNaoQiModule::doChangeFrameRate(const Event* e)
	{
		const EventPropertyUpdate* event = dynamic_cast<const EventPropertyUpdate*>(e);
		if (!event)
		{
			RDK_DEBUG_PRINTF("Something wrong, this seems not to be an EventPropertyUpdate object (it is of class '%s')", e->type.c_str());
			return false;
		}

		if (!cameraProxy->call<bool>( "setFrameRate", pID, session->getInt(PROPERTY_PARAMS_FrameRate) ))
		{
			RDK_ERROR_STREAM("Error setting " << session->getInt(PROPERTY_PARAMS_FrameRate) << " frame rate");
			return false;
		}

		cameraParams[selectedCameraID].kCameraFrameRateID = session->getInt(PROPERTY_PARAMS_FrameRate);
		double fr = 1000.0/cameraParams[selectedCameraID].kCameraFrameRateID;
		RDK_DEBUG_STREAM("Listening to " << fr << "ms");
		session->listenToTimer(fr);

		return true;
	}
	
	void DemoVisionNaoQiModule::saveCameraParams(int index)
	{
		if (index >= NUMBEROFCAMERAS)
		{
			RDK_ERROR_STREAM("Error trying to save unexisting camera, leaving unchanged id = " << index )
			index = 1;
		}
		cameraParams[index].kCameraBrightnessID = session->getInt(PROPERTY_PARAMS_CAMERA_Brightness);
		cameraParams[index].kCameraContrastID = session->getInt(PROPERTY_PARAMS_CAMERA_Contrast);
		cameraParams[index].kCameraSaturationID = session->getInt(PROPERTY_PARAMS_CAMERA_Saturation);
		cameraParams[index].kCameraHueID = session->getInt(PROPERTY_PARAMS_CAMERA_Hue);
		cameraParams[index].kCameraRedChromaID = session->getInt(PROPERTY_PARAMS_CAMERA_RedChroma);
		cameraParams[index].kCameraBlueChromaID = session->getInt(PROPERTY_PARAMS_CAMERA_BlueChroma);
		cameraParams[index].kCameraHFlipID = session->getInt(PROPERTY_PARAMS_CAMERA_HFlip);
		cameraParams[index].kCameraVFlipID = session->getInt(PROPERTY_PARAMS_CAMERA_VFlip);
		cameraParams[index].kCameraLensXID = session->getInt(PROPERTY_PARAMS_CAMERA_LensX);
		cameraParams[index].kCameraLensYID = session->getInt(PROPERTY_PARAMS_CAMERA_LensY);
		cameraParams[index].kCameraAutoGainID = session->getInt(PROPERTY_PARAMS_CAMERA_AutoGain);
		cameraParams[index].kCameraGainID = session->getInt(PROPERTY_PARAMS_CAMERA_Gain);
		cameraParams[index].kCameraAutoExpositionID = session->getInt(PROPERTY_PARAMS_CAMERA_AutoExposition);
		cameraParams[index].kCameraExposureCorrectionID = session->getInt(PROPERTY_PARAMS_CAMERA_ExposureCorrection);
		cameraParams[index].kCameraExposureID = session->getInt(PROPERTY_PARAMS_CAMERA_Exposure);
		cameraParams[index].kCameraAutoWhiteBalanceID = session->getInt(PROPERTY_PARAMS_CAMERA_AutoWhiteBalance);
		cameraParams[index].kCameraResolutionID = session->getInt(PROPERTY_PARAMS_Resolution);
		cameraParams[index].kCameraFrameRateID = session->getInt(PROPERTY_PARAMS_FrameRate);
		cameraParams[index].kCameraColorSpaceID = session->getInt(PROPERTY_PARAMS_Colorspace);
//		cameraParams[index].kCameraBufferSizeID = session->getInt(PROPERTY_PARAMS_CAMERA_BufferSize);
	}

	void DemoVisionNaoQiModule::restoreCameraParams(int index)
	{
		if (index >= NUMBEROFCAMERAS)
		{
			RDK_ERROR_STREAM("Error trying to restore unexisting camera, leaving unchanged.")
			return;
		}
		session->setInt(PROPERTY_PARAMS_CAMERA_Brightness,cameraParams[index].kCameraBrightnessID);
		session->setInt(PROPERTY_PARAMS_CAMERA_Contrast,cameraParams[index].kCameraContrastID);
		session->setInt(PROPERTY_PARAMS_CAMERA_Saturation,cameraParams[index].kCameraSaturationID);
		session->setInt(PROPERTY_PARAMS_CAMERA_Hue,cameraParams[index].kCameraHueID);
		session->setInt(PROPERTY_PARAMS_CAMERA_RedChroma,cameraParams[index].kCameraRedChromaID);
		session->setInt(PROPERTY_PARAMS_CAMERA_BlueChroma,cameraParams[index].kCameraBlueChromaID);
		session->setInt(PROPERTY_PARAMS_CAMERA_HFlip,cameraParams[index].kCameraHFlipID);
		session->setInt(PROPERTY_PARAMS_CAMERA_VFlip,cameraParams[index].kCameraVFlipID);
		session->setInt(PROPERTY_PARAMS_CAMERA_LensX,cameraParams[index].kCameraLensXID);
		session->setInt(PROPERTY_PARAMS_CAMERA_LensY,cameraParams[index].kCameraLensYID);
		session->setInt(PROPERTY_PARAMS_CAMERA_AutoGain,cameraParams[index].kCameraAutoGainID);
		session->setInt(PROPERTY_PARAMS_CAMERA_Gain,cameraParams[index].kCameraGainID);
		session->setInt(PROPERTY_PARAMS_CAMERA_AutoExposition,cameraParams[index].kCameraAutoExpositionID);
		session->setInt(PROPERTY_PARAMS_CAMERA_ExposureCorrection,cameraParams[index].kCameraExposureCorrectionID);
		session->setInt(PROPERTY_PARAMS_CAMERA_Exposure,cameraParams[index].kCameraExposureID);
		session->setInt(PROPERTY_PARAMS_CAMERA_AutoWhiteBalance,cameraParams[index].kCameraAutoWhiteBalanceID);
		session->setInt(PROPERTY_PARAMS_Resolution,cameraParams[index].kCameraResolutionID);
		session->setInt(PROPERTY_PARAMS_FrameRate,cameraParams[index].kCameraFrameRateID);
		session->setInt(PROPERTY_PARAMS_Colorspace,cameraParams[index].kCameraColorSpaceID);
		//session->setInt(PROPERTY_PARAMS_CAMERA_BufferSize,cameraParams[index].kCameraBufferSizeID);
	}

	// implement this if you need to force the exec to go out of the loop
	// if the thread may be waiting not for the session->wait() semaphore:
	// on closing, the main thread will set the exiting variables, call this exitRequested()
	// function and then signal the session semaphores
	// void DemoVisionNaoQiModule::exitRequested() { }

	// implement this if you need to clean up things after the exec has exited
	//void DemoVisionNaoQiModule::cleanup()
	//{}

	// void DemoVisionNaoQiModule::asyncAgentCmd(cstr cmd)
	// {
	//	SESSION_TRY_START(asyncSession)
	//	// here you can parse 'cmd'
	//	SESSION_END_CATCH_TERMINATE(asyncSession)
	// }

	MODULE_FACTORY(DemoVisionNaoQiModule);

	}} // namespace
