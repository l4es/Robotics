#define MODULE_NAME "NaoQiVisionModule"

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE MODULE_NAME

#include "naoqivisionmodule.h"
#include "naoqivisionmodule_names.h"
#include <rdkcore/sharedmemory/sensordatashm.h>

#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/time/time.h>

#include <nao/object/naocameraparams.h>

#include <alvisiondefinitions.h>
#include <alerror.h>

#include <alimage.h>

#include <vector>
#include <cstring>

namespace RDK2 { namespace RAgent
{

	bool NaoQiVisionModule::initConfigurationProperties()
	{
		SESSION_TRY_START(session)
			// put false in the second parameter if you want the module disabled for default
			// add a third parameter valued false if you don't want the state property
		Common::createDefaultProperties(session, true);
		session->createBool(PROPERTY_CMD_CHANGE_CAMERA_PARAMS, "Reload camera parameters", false);
		session->setVolatile(PROPERTY_CMD_CHANGE_CAMERA_PARAMS);
		session->createBool(PROPERTY_CMD_FORCE_CHANGE, "Force reloading camera parameters", false);
		session->setVolatile(PROPERTY_CMD_FORCE_CHANGE);

		////////////////// camera parameters
		// resolution
		ENUM_CREATE(resolution);
		ENUM_ITEM(resolution, AL::kVGA, "High Resolution", "Use the highest resolution (640 * 480)");
		ENUM_ITEM(resolution, AL::kQVGA, "Medium Resolution", "Use the medium resolution (320 * 240)");
		ENUM_ITEM(resolution, AL::kQQVGA, "Low Resolution", "Use the lowest resolution (160 * 120)");
		ostringstream oss;
		oss << "Resolution ("
		    << AL::kVGA << "=high (640x480), "
				<< AL::kQVGA << "=mid (320x240), "
				<< AL::kQQVGA << "=low (160x120) )";
		session->createEnum(PROPERTY_PARAMS_Resolution, oss.str(), resolution, AL::kQVGA);

		// colorspace
		ENUM_CREATE(colorspace);
		ENUM_ITEM(colorspace, AL::kYuvColorSpace, "kYuvColorSpace", "kYuvColorSpace");
		ENUM_ITEM(colorspace, AL::kyUvColorSpace, "kyUvColorSpace", "kyUvColorSpace");
		ENUM_ITEM(colorspace, AL::kyuVColorSpace, "kyuVColorSpace", "kyuVColorSpace");
		ENUM_ITEM(colorspace, AL::kRgbColorSpace, "kRgbColorSpace", "kRgbColorSpace");
		ENUM_ITEM(colorspace, AL::krGbColorSpace, "krGbColorSpace", "krGbColorSpace");
		ENUM_ITEM(colorspace, AL::krgBColorSpace, "krgBColorSpace", "krgBColorSpace");
		ENUM_ITEM(colorspace, AL::kHsyColorSpace, "kHsyColorSpace", "kHsyColorSpace");
		ENUM_ITEM(colorspace, AL::khSyColorSpace, "khSyColorSpace", "khSyColorSpace");
		ENUM_ITEM(colorspace, AL::khsYColorSpace, "khsYColorSpace", "khsYColorSpace");
		ENUM_ITEM(colorspace, AL::kYUV422InterlacedColorSpace, "kYUV422InterlacedColorSpace (raw)", "kYUV422InterlacedColorSpace");
		ENUM_ITEM(colorspace, AL::kYUVColorSpace, "kYUVColorSpace", "kYUVColorSpace");
		ENUM_ITEM(colorspace, AL::kRGBColorSpace, "kRGBColorSpace", "kRGBColorSpace");
		ENUM_ITEM(colorspace, AL::kHSYColorSpace, "kHSYColorSpace", "kHSYColorSpace");
		ENUM_ITEM(colorspace, AL::kBGRColorSpace, "kBGRColorSpace", "kBGRColorSpace"); //for opencv ease of use
		ENUM_ITEM(colorspace, AL::kYYCbCrColorSpace, "kYYCbCrColorSpace", "kYYCbCrColorSpace"); //for tiff io implementation
		ENUM_ITEM(colorspace, AL::kH2RGBColorSpace, "kH2RGBColorSpace", "kH2RGBColorSpace"); //H from HSV to RGB in fake colors
		ENUM_ITEM(colorspace, AL::kHSMixedColorSpace, "kHSMixedColorSpace", "kHSMixedColorSpace"); //HS and (H +S)/2
		oss.str("");
		oss << "Colorspace ("
			<< AL::kYuvColorSpace << "=Yuv, "
			<< AL::kyUvColorSpace << "=yUv, "
			<< AL::kyuVColorSpace << "=yuV, "
			<< AL::kRgbColorSpace << "=Rgb, "
			<< AL::krGbColorSpace << "=rGb, "
			<< AL::krgBColorSpace << "=rgB, "
			<< AL::kHsyColorSpace << "=Hsy, "
			<< AL::khSyColorSpace << "=hSy, "
			<< AL::khsYColorSpace << "=hsY, "
			<< AL::kYUV422InterlacedColorSpace << "=YUV422Interlaced, "
			<< AL::kYUVColorSpace << "=YUV, "
			<< AL::kRGBColorSpace << "=RGB, "
			<< AL::kHSYColorSpace << "=HSY, "
			<< AL::kBGRColorSpace << "=BGR, " //for opencv ease of use
			<< AL::kYYCbCrColorSpace << "=YYCbCr, " //for tiff io implementation 
			<< AL::kH2RGBColorSpace << "=H2RGB, " //H from HSV to RGB in fake colors
			<< AL::kHSMixedColorSpace << "=HSMixed, " //HS and (H +S)/2
			<< ")";
		session->createEnum(PROPERTY_PARAMS_Colorspace, oss.str(), colorspace, AL::kYUV422InterlacedColorSpace);
		session->createInt(PROPERTY_PARAMS_FrameRate, "Frame rate: 5, 10, 15, 30", 10);

		// scaling method
		ENUM_CREATE(scale);
		ENUM_ITEM(scale, AL::kSimpleScaleMethod, "kSimpleScaleMethod", "kSimpleScaleMethod");
		ENUM_ITEM(scale, AL::kAverageScaleMethod, "kAverageScaleMethod", "kAverageScaleMethod");
		ENUM_ITEM(scale, AL::kQualityScaleMethod, "kQualityScaleMethod", "kQualityScaleMethod");
		ENUM_ITEM(scale, AL::kNoScaling, "kNoScaling", "kNoScaling");

		// other parameters
		// TOP CAMERA
		session->createInt(PROPERTY_PARAMS_CAMERA_TOP_Brightness, "Brightness [0..255]", 128);
		session->createInt(PROPERTY_PARAMS_CAMERA_TOP_Contrast, "Contrast [0..127]", 64);
		session->createInt(PROPERTY_PARAMS_CAMERA_TOP_Saturation, "Saturation [0..255]", 128);
		session->createInt(PROPERTY_PARAMS_CAMERA_TOP_Hue, "Hue [-180..180]", 0);
		session->createInt(PROPERTY_PARAMS_CAMERA_TOP_RedChroma, "RedChroma [0..255]", 128);
		session->createInt(PROPERTY_PARAMS_CAMERA_TOP_BlueChroma, "BlueChroma [0..255]", 128);
		session->createInt(PROPERTY_PARAMS_CAMERA_TOP_Gain, "Gain [0..255]", 128);
		session->createInt(PROPERTY_PARAMS_CAMERA_TOP_HFlip, "HFlip [0,1]", 0);
		session->createInt(PROPERTY_PARAMS_CAMERA_TOP_VFlip, "VFlip [0,1]", 0);
		session->createInt(PROPERTY_PARAMS_CAMERA_TOP_LensX, "LensX [0..255]", 0);
		session->createInt(PROPERTY_PARAMS_CAMERA_TOP_LensY, "LensY [0..255]", 0);
		session->createInt(PROPERTY_PARAMS_CAMERA_TOP_AutoExposition, "AutoExposition [0,1]", 0);
		session->createInt(PROPERTY_PARAMS_CAMERA_TOP_AutoWhiteBalance, "AutoWhiteBalance [0,1]", 0);
		session->createInt(PROPERTY_PARAMS_CAMERA_TOP_AutoGain, "AutoGain [0,1]", 0);
		session->createInt(PROPERTY_PARAMS_CAMERA_TOP_Exposure, "Exposure (time in ms = value x 33/480) [0..480 (number of lines of the sensor)]", 240);
		session->createInt(PROPERTY_PARAMS_CAMERA_TOP_ExposureCorrection, "ExposureCorrection", 0);
		// BOTTOM CAMERA
		session->createInt(PROPERTY_PARAMS_CAMERA_BOTTOM_Brightness, "Brightness [0..255]", 128);
		session->createInt(PROPERTY_PARAMS_CAMERA_BOTTOM_Contrast, "Contrast [0..127]", 64);
		session->createInt(PROPERTY_PARAMS_CAMERA_BOTTOM_Saturation, "Saturation [0..255]", 128);
		session->createInt(PROPERTY_PARAMS_CAMERA_BOTTOM_Hue, "Hue [-180..180]", 0);
		session->createInt(PROPERTY_PARAMS_CAMERA_BOTTOM_RedChroma, "RedChroma [0..255]", 128);
		session->createInt(PROPERTY_PARAMS_CAMERA_BOTTOM_BlueChroma, "BlueChroma [0..255]", 128);
		session->createInt(PROPERTY_PARAMS_CAMERA_BOTTOM_Gain, "Gain [0..255]", 128);
		session->createInt(PROPERTY_PARAMS_CAMERA_BOTTOM_HFlip, "HFlip [0,1]", 0);
		session->createInt(PROPERTY_PARAMS_CAMERA_BOTTOM_VFlip, "VFlip [0,1]", 0);
		session->createInt(PROPERTY_PARAMS_CAMERA_BOTTOM_LensX, "LensX [0..255]", 0);
		session->createInt(PROPERTY_PARAMS_CAMERA_BOTTOM_LensY, "LensY [0..255]", 0);
		session->createInt(PROPERTY_PARAMS_CAMERA_BOTTOM_AutoExposition, "AutoExposition [0,1]", 0);
		session->createInt(PROPERTY_PARAMS_CAMERA_BOTTOM_AutoWhiteBalance, "AutoWhiteBalance [0,1]", 0);
		session->createInt(PROPERTY_PARAMS_CAMERA_BOTTOM_AutoGain, "AutoGain [0,1]", 0);
		session->createInt(PROPERTY_PARAMS_CAMERA_BOTTOM_Exposure, "Exposure (time in ms = value x 33/480) [0..480 (number of lines of the sensor)]", 240);
		session->createInt(PROPERTY_PARAMS_CAMERA_BOTTOM_ExposureCorrection, "ExposureCorrection", 0);

		session->createInt(PROPERTY_PARAMS_CAMERA_Select, "0=Top, 1=Bottom", 1);
		session->createBool(PROPERTY_PARAMS_LINK_PARAMS, "Use same parameters for all cameras (default are the Bottom Camera ones)", true);

		// FIXME remove magic number
		session->createImage(PROPERTY_OUT_IMAGE, "Current Image", 320, 240, RDK2::RGraphics::RImage::YUYV);

		session->createBool(PROPERTY_DBG_ENABLED, "Enable dbg",false);
		session->setVolatile(PROPERTY_DBG_ENABLED);
		session->createImage(PROPERTY_DBG_IMAGE, "Current Image", 80, 60, RDK2::RGraphics::RImage::RGB32);

		SESSION_END(session)
			return true;
		SESSION_CATCH_TERMINATE(session)
			return false;
	}

	// bool NaoQiVisionModule::initInterfaceProperties() { }

	bool NaoQiVisionModule::init()
	{
		// in the init() function you should initialize everything your module needs (drivers initialization,
		// sockets, and so on); all modules wait for all init() before they start
		SESSION_TRY_START(session)

		CAMERA_PREFIX[TOPCAMERA] = CAMERA_TOP_PREFIX;
		CAMERA_PREFIX[BOTTOMCAMERA] = CAMERA_BOTTOM_PREFIX;

		shmimage = new SharedMemory(SharedMemory::IMAGE, sizeof(RdkImageData));

		//session->registerPropertyUpdateEventHandler(SESSION_EVENT(doChangeCameraParams), PROPERTY_CMD_CHANGE_CAMERA_PARAMS);
		//session->listen(PROPERTY_CMD_CHANGE_CAMERA_PARAMS);

		saveCameraParams(TOPCAMERA);
		saveCameraParams(BOTTOMCAMERA);
		selectedCameraID = -1;

		RdkImageData *image = static_cast<RdkImageData*>(shmimage->getEntry());
		image->cameraParams[TOPCAMERA]    = cameraParams[TOPCAMERA];
		image->cameraParams[BOTTOMCAMERA] = cameraParams[BOTTOMCAMERA];

		session->setBool(PROPERTY_CMD_CHANGE_CAMERA_PARAMS, true);

		shmimage->signal(NAOQI);

		SESSION_END(session)
			return true;
		SESSION_CATCH_TERMINATE(session)
			return false;
	}

	void NaoQiVisionModule::exec()
	{
		while (session->dontWait(), !exiting)
		{
			SESSION_TRY_START(session)
			shmimage->wait(RDK);
#ifdef OPENRDK_ARCH_GENERIC
			usleep(100*10e6);
#else
			if (session->getBool(PROPERTY_CMD_CHANGE_CAMERA_PARAMS))
				doChangeCameraParams(new EventProperty(PROPERTY_CMD_CHANGE_CAMERA_PARAMS));

			Timestamp t;
			RdkImageData *imageData     = static_cast<RdkImageData*>(shmimage->getEntry());
			unsigned char * dataPointer = imageData->data;
			bufferSize                  = imageData->width*imageData->height*imageData->bpp;
			session->lock(PROPERTY_OUT_IMAGE, HERE);
			RImage *rimage = session->getObjectAsL<RImage>(PROPERTY_OUT_IMAGE);
			// TODO test this
			//if (imageData->width != static_cast<int>(rimage->getWidth())
			//    ||
			//    imageData->height != static_cast<int>(rimage->getHeight()))
			//{
			//  RDK_DEBUG_STREAM("Changing image size from " << rimage->getWidth() << "x" << rimage->getHeight() << " to " << imageData->width << "x" << imageData->height);
			//  rimage->setSizeAndType(imageData->width, imageData->height, rimage->getType());
			//}
			//RDK_WARNING_STREAM(imageData->width << "x" << imageData->height);
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
				if (session->getBool(PROPERTY_DBG_ENABLED))
				{
					session->lock(PROPERTY_DBG_IMAGE, HERE);
					RImage* rimg = static_cast<RImage*>(rimage->clone())->convertTo(RImage::RGB32);
					rimg->imageResize(80,60);
					session->setObject(PROPERTY_DBG_IMAGE, rimg);
					session->unlock(PROPERTY_DBG_IMAGE);
					session->valueChanged(PROPERTY_DBG_IMAGE);
				}
			}
			else
			{
				RDK_ERROR_STREAM("Null datapointer");
			}
			session->unlock(PROPERTY_OUT_IMAGE);
			session->valueChanged(PROPERTY_OUT_IMAGE);
			RDK_TRACE_STREAM("Retrieving image time " << Timestamp().getMsFromMidnight()-t.getMsFromMidnight());

			if (session->getBool(PROPERTY_CMD_FORCE_CHANGE))
			{
				session->setBool(PROPERTY_CMD_CHANGE_CAMERA_PARAMS, true);
				doChangeCameraParams(new EventProperty(PROPERTY_CMD_FORCE_CHANGE));
			}

#endif
			shmimage->signal(NAOQI);
			SESSION_END_CATCH_TERMINATE(session)
		}
	}

	bool NaoQiVisionModule::doChangeCameraParams(const Event* e)
	{
		const EventProperty* event = dynamic_cast<const EventProperty*>(e);
		if (!event)
		{
			RDK_DEBUG_PRINTF("Something wrong, this seems not to be an EventProperty object (it is of class '%s')", e->type.c_str());
			return false;
		}
		int newSelectedCameraID = session->getInt(PROPERTY_PARAMS_CAMERA_Select);
		if (event->propertyUrl == PROPERTY_CMD_CHANGE_CAMERA_PARAMS)
		{
			session->setBool(PROPERTY_CMD_CHANGE_CAMERA_PARAMS, false);
		}
		else if (event->propertyUrl == PROPERTY_CMD_FORCE_CHANGE)
		{
			RDK_INFO_STREAM("Force reloading parameters");
			/* :WORKAROUND:29/12/09 14:41:23:lm: selecting different camera force parameters reloading */
			newSelectedCameraID = selectedCameraID == TOPCAMERA ? BOTTOMCAMERA : TOPCAMERA;
		}
		
		if (selectedCameraID != newSelectedCameraID)
		{
			if (!saveCameraParams(newSelectedCameraID))
			{
				RDK_ERROR_STREAM("Error trying to save unexisting camera, leaving unchanged [id = " << newSelectedCameraID << "]");
			}
			else
			{
				if (!restoreCameraParams(newSelectedCameraID))
				{
					RDK_ERROR_STREAM("Error trying to restore unexisting camera, leaving unchanged [id = " << newSelectedCameraID << "]");
				}
				else
				{
					selectedCameraID = newSelectedCameraID;
				}
			}
		}

		RdkImageData *image = static_cast<RdkImageData*>(shmimage->getEntry());
		image->cameraParams[TOPCAMERA]    = cameraParams[TOPCAMERA];
		image->cameraParams[BOTTOMCAMERA] = cameraParams[BOTTOMCAMERA];
		image->selectedCameraID = selectedCameraID;
		image->changeParams = true;

		if (event->propertyUrl == PROPERTY_CMD_FORCE_CHANGE)
		{
			RDK_INFO_STREAM("Force Changing camera parameters. 2nd call required");
			session->setBool(PROPERTY_CMD_FORCE_CHANGE, false);
		}
		else
		{
			RDK_INFO_STREAM("Changing camera parameters.");
		}
		return true;
	}

	bool NaoQiVisionModule::doChangeResolution(const Event* e)
	{
		const EventPropertyUpdate* event = dynamic_cast<const EventPropertyUpdate*>(e);
		if (!event)
		{
			RDK_DEBUG_PRINTF("Something wrong, this seems not to be an EventPropertyUpdate object (it is of class '%s')", e->type.c_str());
			return false;
		}


		cameraParams[selectedCameraID].kCameraResolutionID = session->getInt(PROPERTY_PARAMS_Resolution);

		int w,h,l;
		AL::getSizeFromResolution(cameraParams[selectedCameraID].kCameraResolutionID, w, h);
		l = AL::getNumLayersInColorSpace(session->getInt(PROPERTY_PARAMS_Colorspace));
		bufferSize = w*h*l;
		

		session->lock(PROPERTY_OUT_IMAGE, HERE);
		RImage *rimage = session->getObjectAsL<RImage>(PROPERTY_OUT_IMAGE);
		rimage->canvasResize(w,h,0,0,0);
		session->unlock(PROPERTY_OUT_IMAGE);


		return true;
	}

	bool NaoQiVisionModule::doChangeColorspace(const Event* e)
	{
		const EventPropertyUpdate* event = dynamic_cast<const EventPropertyUpdate*>(e);
		if (!event)
		{
			RDK_DEBUG_PRINTF("Something wrong, this seems not to be an EventPropertyUpdate object (it is of class '%s')", e->type.c_str());
			return false;
		}

		RDK_DEBUG_STREAM("Colorspace conversion not implemented yet");
		session->setInt(PROPERTY_PARAMS_Colorspace, AL::kYUV422InterlacedColorSpace);
		return true;

		//bool res = cameraProxy->call<bool>( "setColorSpace", pID, session->getInt(PROPERTY_PARAMS_Colorspace) );
		//if (!res)
		//{
		//  RDK_ERROR_STREAM("Error setting " << session->getInt(PROPERTY_PARAMS_Colorspace) << " colorspace");
		//  return false;
		//}
	}

	bool NaoQiVisionModule::doChangeFrameRate(const Event* e)
	{
		const EventPropertyUpdate* event = dynamic_cast<const EventPropertyUpdate*>(e);
		if (!event)
		{
			RDK_DEBUG_PRINTF("Something wrong, this seems not to be an EventPropertyUpdate object (it is of class '%s')", e->type.c_str());
			return false;
		}

		cameraParams[selectedCameraID].kCameraFrameRateID = session->getInt(PROPERTY_PARAMS_FrameRate);
		double fr = 1000.0/cameraParams[selectedCameraID].kCameraFrameRateID;
		RDK_DEBUG_STREAM("Listening to " << fr << "ms");
		session->listenToTimer(fr);

		return true;
	}
	
	bool NaoQiVisionModule::saveCameraParams(int index)
	{
		if (index >= NUMBEROFCAMERAS)
		{
			return false;
		}
		if (session->getBool(PROPERTY_PARAMS_LINK_PARAMS))
		{
			index = BOTTOMCAMERA;
		}
		string url = CAMERA_PREFIX[index] + "/";
		cameraParams[index].kCameraBrightnessID = session->getInt(url+PROPERTY_SUFFIX_Brightness);
		cameraParams[index].kCameraContrastID = session->getInt(url+PROPERTY_SUFFIX_Contrast);
		cameraParams[index].kCameraSaturationID = session->getInt(url+PROPERTY_SUFFIX_Saturation);
		cameraParams[index].kCameraHueID = session->getInt(url+PROPERTY_SUFFIX_Hue);
		cameraParams[index].kCameraRedChromaID = session->getInt(url+PROPERTY_SUFFIX_RedChroma);
		cameraParams[index].kCameraBlueChromaID = session->getInt(url+PROPERTY_SUFFIX_BlueChroma);
		cameraParams[index].kCameraHFlipID = session->getInt(url+PROPERTY_SUFFIX_HFlip);
		cameraParams[index].kCameraVFlipID = session->getInt(url+PROPERTY_SUFFIX_VFlip);
		cameraParams[index].kCameraLensXID = session->getInt(url+PROPERTY_SUFFIX_LensX);
		cameraParams[index].kCameraLensYID = session->getInt(url+PROPERTY_SUFFIX_LensY);
		cameraParams[index].kCameraAutoGainID = session->getInt(url+PROPERTY_SUFFIX_AutoGain);
		cameraParams[index].kCameraGainID = session->getInt(url+PROPERTY_SUFFIX_Gain);
		cameraParams[index].kCameraAutoExpositionID = session->getInt(url+PROPERTY_SUFFIX_AutoExposition);
		cameraParams[index].kCameraExposureCorrectionID = session->getInt(url+PROPERTY_SUFFIX_ExposureCorrection);
		cameraParams[index].kCameraExposureID = session->getInt(url+PROPERTY_SUFFIX_Exposure);
		cameraParams[index].kCameraAutoWhiteBalanceID = session->getInt(url+PROPERTY_SUFFIX_AutoWhiteBalance);
		cameraParams[index].kCameraResolutionID = session->getInt(PROPERTY_PARAMS_Resolution);
		cameraParams[index].kCameraFrameRateID = session->getInt(PROPERTY_PARAMS_FrameRate);
		cameraParams[index].kCameraColorSpaceID = session->getInt(PROPERTY_PARAMS_Colorspace);
//		cameraParams[index].kCameraBufferSizeID = session->getInt(url+PROPERTY_SUFFIX_BufferSize);
		if (session->getBool(PROPERTY_PARAMS_LINK_PARAMS))
		{
			for (int i=0; i<NUMBEROFCAMERAS; ++i)
				cameraParams[i] = cameraParams[BOTTOMCAMERA];
		}
		return true;
	}

	bool NaoQiVisionModule::restoreCameraParams(int index)
	{
		if (index >= NUMBEROFCAMERAS)
		{
			return false;
		}
		string url = CAMERA_PREFIX[index] + "/";
		session->setInt(url+PROPERTY_SUFFIX_Brightness,cameraParams[index].kCameraBrightnessID);
		session->setInt(url+PROPERTY_SUFFIX_Contrast,cameraParams[index].kCameraContrastID);
		session->setInt(url+PROPERTY_SUFFIX_Saturation,cameraParams[index].kCameraSaturationID);
		session->setInt(url+PROPERTY_SUFFIX_Hue,cameraParams[index].kCameraHueID);
		session->setInt(url+PROPERTY_SUFFIX_RedChroma,cameraParams[index].kCameraRedChromaID);
		session->setInt(url+PROPERTY_SUFFIX_BlueChroma,cameraParams[index].kCameraBlueChromaID);
		session->setInt(url+PROPERTY_SUFFIX_HFlip,cameraParams[index].kCameraHFlipID);
		session->setInt(url+PROPERTY_SUFFIX_VFlip,cameraParams[index].kCameraVFlipID);
		session->setInt(url+PROPERTY_SUFFIX_LensX,cameraParams[index].kCameraLensXID);
		session->setInt(url+PROPERTY_SUFFIX_LensY,cameraParams[index].kCameraLensYID);
		session->setInt(url+PROPERTY_SUFFIX_AutoGain,cameraParams[index].kCameraAutoGainID);
		session->setInt(url+PROPERTY_SUFFIX_Gain,cameraParams[index].kCameraGainID);
		session->setInt(url+PROPERTY_SUFFIX_AutoExposition,cameraParams[index].kCameraAutoExpositionID);
		session->setInt(url+PROPERTY_SUFFIX_ExposureCorrection,cameraParams[index].kCameraExposureCorrectionID);
		session->setInt(url+PROPERTY_SUFFIX_Exposure,cameraParams[index].kCameraExposureID);
		session->setInt(url+PROPERTY_SUFFIX_AutoWhiteBalance,cameraParams[index].kCameraAutoWhiteBalanceID);
		session->setInt(PROPERTY_PARAMS_Resolution,cameraParams[index].kCameraResolutionID);
		session->setInt(PROPERTY_PARAMS_FrameRate,cameraParams[index].kCameraFrameRateID);
		session->setInt(PROPERTY_PARAMS_Colorspace,cameraParams[index].kCameraColorSpaceID);
		//session->setInt(url+PROPERTY_SUFFIX_BufferSize,cameraParams[index].kCameraBufferSizeID);
		return true;
	}

	// implement this if you need to force the exec to go out of the loop
	// if the thread may be waiting not for the session->wait() semaphore:
	// on closing, the main thread will set the exiting variables, call this exitRequested()
	// function and then signal the session semaphores
void NaoQiVisionModule::exitRequested()
{
	if (shmimage == 0)
		return;
	shmimage->signal(NAOQI);
}

	// implement this if you need to clean up things after the exec has exited
	//void NaoQiVisionModule::cleanup()
	//{}

	// void NaoQiVisionModule::asyncAgentCmd(cstr cmd)
	// {
	//	SESSION_TRY_START(asyncSession)
	//	// here you can parse 'cmd'
	//	SESSION_END_CATCH_TERMINATE(asyncSession)
	// }

	MODULE_FACTORY(NaoQiVisionModule);

	}} // namespace
