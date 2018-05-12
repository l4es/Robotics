#define MODULE_NAME "NaoQiAllSensorsModule"

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE MODULE_NAME

#include "naoqiallsensorsmodule.h"
#include "naoqiallsensorsmodule_names.h"

#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/time/time.h>

#include <nao-objects/naocameraparams.h>
#include <nao-objects/rnaojoints.h>
#include <nao-objects/rnaosensors.h>

#if NaoQi_VERSION_MAJOR == 1 && NaoQi_VERSION_MINOR >= 10
#include <alvision/alvisiondefinitions.h>
#include <alcore/alerror.h>
#include <alvision/alimage.h>
#include <alcommon/albroker.h>
#include <alcommon/alproxy.h>
#else
#include <alvisiondefinitions.h>
#include <alerror.h>
#include <alimage.h>
#include <albroker.h>
#include <alproxy.h>
#endif

#include <vector>
#include <cstring>

using namespace RDK2::Time;

// this define needs if you want to enable experimental feature,
// like double buffer image or fast switching cameras
// uncomment to enable
//#define EXPERIMENTAL_FEATURE


namespace RDK2 { namespace RAgent {

	static const int DEBUG_IMAGE_WIDTH  = 80;
	static const int DEBUG_IMAGE_HEIGHT = 60;

	bool NaoQiAllSensorsModule::initConfigurationProperties()
	{
		SESSION_TRY_START(session)
			// put false in the second parameter if you want the module disabled for default
			// add a third parameter valued false if you don't want the state property
			Common::createDefaultProperties(session, true);


		// FIXME remove magic number
#ifdef EXPERIMENTAL_FEATURE
		session->createBool(PROPERTY_PARAMS_USE_DOUBLE_BUFFER, "Use double buffer for image retrieving", false);
		session->createImage(PROPERTY_OUT_IMAGE2, "Buffer Image #2", 320, 240, RDK2::RGraphics::RImage::YUYV);
		session->createInt(PROPERTY_IN_CURRENT_BUFFER_IN_USE, "Index of buffer used by image processor", 2);
		session->setVolatile(PROPERTY_IN_CURRENT_BUFFER_IN_USE);
#endif
		session->createImage(PROPERTY_OUT_IMAGE1, "Buffer Image #1", 320, 240, RDK2::RGraphics::RImage::YUYV);

		///// timing
		session->createDouble(PROPERTY_TIME_FPS, "Current estimated frame rate of image acquisition [fps]",RDouble::OTHER, 0);
		session->setVolatile(PROPERTY_TIME_FPS);
		session->createInt(PROPERTY_TIME_LAST_IMAGE_READING, "Timestamp of last image received (in ms)", 0);
		session->setVolatile(PROPERTY_TIME_LAST_IMAGE_READING);
		session->createInt(PROPERTY_TIME_LAST_SENSOR_READING, "Timestamp of last sensor reading (in ms)", 0);
		session->setVolatile(PROPERTY_TIME_LAST_SENSOR_READING);
		session->createInt(PROPERTY_TIME_LAST_CALL, "Timestamp of last call (shm waking up) (in ms)", 0);
		session->setVolatile(PROPERTY_TIME_LAST_CALL);

		////////////////// camera parameters
		// frame rate
		session->createInt(PROPERTY_PARAMS_FrameRate, "Frame rate: 5, 10, 15, 30", 10);

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

		// colorspace ///not implemented yet
		ENUM_CREATE(colorspace);
		//ENUM_ITEM(colorspace, AL::kYuvColorSpace, "kYuvColorSpace", "kYuvColorSpace");
		//ENUM_ITEM(colorspace, AL::kyUvColorSpace, "kyUvColorSpace", "kyUvColorSpace");
		//ENUM_ITEM(colorspace, AL::kyuVColorSpace, "kyuVColorSpace", "kyuVColorSpace");
		//ENUM_ITEM(colorspace, AL::kRgbColorSpace, "kRgbColorSpace", "kRgbColorSpace");
		//ENUM_ITEM(colorspace, AL::krGbColorSpace, "krGbColorSpace", "krGbColorSpace");
		//ENUM_ITEM(colorspace, AL::krgBColorSpace, "krgBColorSpace", "krgBColorSpace");
		//ENUM_ITEM(colorspace, AL::kHsyColorSpace, "kHsyColorSpace", "kHsyColorSpace");
		//ENUM_ITEM(colorspace, AL::khSyColorSpace, "khSyColorSpace", "khSyColorSpace");
		//ENUM_ITEM(colorspace, AL::khsYColorSpace, "khsYColorSpace", "khsYColorSpace");
		ENUM_ITEM(colorspace, AL::kYUV422InterlacedColorSpace, "kYUV422InterlacedColorSpace (raw)", "kYUV422InterlacedColorSpace");
		//ENUM_ITEM(colorspace, AL::kYUVColorSpace, "kYUVColorSpace", "kYUVColorSpace");
		//ENUM_ITEM(colorspace, AL::kRGBColorSpace, "kRGBColorSpace", "kRGBColorSpace");
		//ENUM_ITEM(colorspace, AL::kHSYColorSpace, "kHSYColorSpace", "kHSYColorSpace");
		//ENUM_ITEM(colorspace, AL::kBGRColorSpace, "kBGRColorSpace", "kBGRColorSpace"); //for opencv ease of use
		//ENUM_ITEM(colorspace, AL::kYYCbCrColorSpace, "kYYCbCrColorSpace", "kYYCbCrColorSpace"); //for tiff io implementation
		//ENUM_ITEM(colorspace, AL::kH2RGBColorSpace, "kH2RGBColorSpace", "kH2RGBColorSpace"); //H from HSV to RGB in fake colors
		//ENUM_ITEM(colorspace, AL::kHSMixedColorSpace, "kHSMixedColorSpace", "kHSMixedColorSpace"); //HS and (H +S)/2
		oss.str("");
		oss << "Colorspace ("
			//<< AL::kYuvColorSpace << "=Yuv, "
			//<< AL::kyUvColorSpace << "=yUv, "
			//<< AL::kyuVColorSpace << "=yuV, "
			//<< AL::kRgbColorSpace << "=Rgb, "
			//<< AL::krGbColorSpace << "=rGb, "
			//<< AL::krgBColorSpace << "=rgB, "
			//<< AL::kHsyColorSpace << "=Hsy, "
			//<< AL::khSyColorSpace << "=hSy, "
			//<< AL::khsYColorSpace << "=hsY, "
			<< AL::kYUV422InterlacedColorSpace << "=YUV422Interlaced, "
			//<< AL::kYUVColorSpace << "=YUV, "
			//<< AL::kRGBColorSpace << "=RGB, "
			//<< AL::kHSYColorSpace << "=HSY, "
			//<< AL::kBGRColorSpace << "=BGR, " //for opencv ease of use
			//<< AL::kYYCbCrColorSpace << "=YYCbCr, " //for tiff io implementation 
			//<< AL::kH2RGBColorSpace << "=H2RGB, " //H from HSV to RGB in fake colors
			//<< AL::kHSMixedColorSpace << "=HSMixed, " //HS and (H +S)/2
			<< ")";
		session->createEnum(PROPERTY_PARAMS_Colorspace, oss.str(), colorspace, AL::kYUV422InterlacedColorSpace);

		// scaling method
		//ENUM_CREATE(scale);
		//ENUM_ITEM(scale, AL::kSimpleScaleMethod, "kSimpleScaleMethod", "kSimpleScaleMethod");
		//ENUM_ITEM(scale, AL::kAverageScaleMethod, "kAverageScaleMethod", "kAverageScaleMethod");
		//ENUM_ITEM(scale, AL::kQualityScaleMethod, "kQualityScaleMethod", "kQualityScaleMethod");
		//ENUM_ITEM(scale, AL::kNoScaling, "kNoScaling", "kNoScaling");

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

		/////commands
		session->createBool(PROPERTY_CMD_CHANGE_CAMERA_PARAMS, "Reload camera parameters", false);
		session->setVolatile(PROPERTY_CMD_CHANGE_CAMERA_PARAMS);
		session->createBool(PROPERTY_CMD_FORCE_CHANGE, "Force reloading camera parameters", false);
		session->setVolatile(PROPERTY_CMD_FORCE_CHANGE);
#ifdef EXPERIMENTAL_FEATURE
		session->createBool(PROPERTY_CMD_SWITCH_CAMERA, "Switch camera (will use fast mechanism)", false);
		session->setVolatile(PROPERTY_CMD_SWITCH_CAMERA);
#endif

		/////////////////// Sensors /////////////////////////
		session->createStorage("RNaoJoints", PROPERTY_OUT_JOINTS1, "Nao joints buffer #1");
		session->setObject(PROPERTY_OUT_JOINTS1, new RNaoJoints());
		session->createStorage("RNaoSensors", PROPERTY_OUT_SENSORS1, "Nao sensor buffer #1");
		session->setObject(PROPERTY_OUT_SENSORS1, new RNaoSensors());
#ifdef EXPERIMENTAL_FEATURE
		session->createStorage("RNaoJoints", PROPERTY_OUT_JOINTS2, "Nao joints buffer #2");
		session->setObject(PROPERTY_OUT_JOINTS2, new RNaoJoints());
		session->createStorage("RNaoSensors", PROPERTY_OUT_SENSORS2, "Nao sensor buffer #2");
		session->setObject(PROPERTY_OUT_SENSORS2, new RNaoSensors());
#endif

		session->createDouble(PROPERTY_OUT_BATTERY, "Battery charge (0.0-1.0)", RDouble::REAL, -1);
		session->setVolatile(PROPERTY_OUT_BATTERY);

		session->createInt(PROPERTY_OUT_CAMERA_IN_USE, "Camera used for images");
		session->setVolatile(PROPERTY_OUT_CAMERA_IN_USE);


		//// debug
		session->createBool(PROPERTY_DBG_ENABLED, "Enable debug output (use RDK_LOG=trace)", false);
		session->setVolatile(PROPERTY_DBG_ENABLED);
		session->createImage(PROPERTY_DBG_IMAGE, "Current Image", DEBUG_IMAGE_WIDTH, DEBUG_IMAGE_HEIGHT, RDK2::RGraphics::RImage::RGB32);
		session->createString(PROPERTY_OUT_ALLDATA, "All data in string format", "N/A");
		session->setVolatile(PROPERTY_OUT_ALLDATA);

		session->createString(PROPERTY_OUT_POSE, "Pose (body posture) string format", "");
		session->setVolatile(PROPERTY_OUT_POSE);


		/////////////////// Feedback Sensors /////////////////////////
		session->createString(PROPERTY_OUT_MOTION_TASK_FROM_NMM, "feedback of motion task from NaoMotionModule", "Stand");
		session->setVolatile(PROPERTY_OUT_MOTION_TASK_FROM_NMM);

		session->createBool(PROPERTY_OUT_IS_HEAD_MOVING, "feedback indicating whether is moving from NaoMotionModule", false);
		session->setVolatile(PROPERTY_OUT_IS_HEAD_MOVING);

		session->createPose(PROPERTY_OUT_ODOMETRY_POSE, "feedback of odometry from NaoMotionModule", Point2od());
		session->setVolatile(PROPERTY_OUT_ODOMETRY_POSE);

		/////////////////// Game Controller /////////////////////////
		session->createString(PROPERTY_IN_GAMESTATE, "One in {initial, ready, set, play, finish", "initial");
		session->setVolatile(PROPERTY_IN_GAMESTATE);
		session->createInt(PROPERTY_IN_TEAMCOLOR, "Team Color (red:1 or blue:0)", 0, DONT_SAVE);
		session->createInt(PROPERTY_IN_PENALTY, "0 if unpenalized, the code of the penalty otherwise", 0);
		session->setVolatile(PROPERTY_IN_PENALTY);

		session->createString(PROPERTY_OUT_FINAL_GAMESTATE, "One in {initial, ready, set, play, finish", "initial");
		session->setVolatile(PROPERTY_OUT_FINAL_GAMESTATE);

		session->createInt(PROPERTY_OUT_FINAL_TEAMCOLOR, "Team Color (red:1 or blue:0)", 0);
		session->setVolatile(PROPERTY_OUT_FINAL_TEAMCOLOR);

		session->createInt(PROPERTY_OUT_FINAL_PENALIZE, "0 if unpenalized, the code of the penalty otherwise", 0);
		session->setVolatile(PROPERTY_OUT_FINAL_PENALIZE);

		session->createInt(PROPERTY_IN_UNIFORM_NUMBER, "my uniform number", 1);

		/////////////////// Miscs /////////////////////////
		session->createDouble(PROPERTY_PARAM_DEFAULT_PENALIZE_TIME, "time of penalty", RDouble::SEC, 30.0);
		session->createString(PROPERTY_OUT_TTS_TEXT, "TTS Text to be sent to Audio module", "");
		session->setVolatile(PROPERTY_OUT_TTS_TEXT);


		mIsTheFirstTime = true;
		mGameStateFromXSentinel = "Initial";
		mIsBlueTeamFromXSentinel = true;
		mLastPenalizedTimeValid = false;
		mLastPenalizedTime = 0;

		SESSION_END(session)
			return true;
		SESSION_CATCH_TERMINATE(session)
			return false;
	}

	// bool NaoQiAllSensorsModule::initInterfaceProperties() { }

	bool NaoQiAllSensorsModule::init()
	{
		// in the init() function you should initialize everything your module needs (drivers initialization,
		// sockets, and so on); all modules wait for all init() before they start
		SESSION_TRY_START(session)

		shm = new SharedMemory(SharedMemory::ALLSENSOR, sizeof(RdkAllSensorData));

		CAMERA_PREFIX[TOPCAMERA] = CAMERA_TOP_PREFIX;
		CAMERA_PREFIX[BOTTOMCAMERA] = CAMERA_BOTTOM_PREFIX;

		saveCameraParams(TOPCAMERA);
		saveCameraParams(BOTTOMCAMERA);

		selectedCameraID = -1;

		RdkAllSensorData *sensordata = static_cast<RdkAllSensorData*>(shm->getEntry());
		RdkImageData& image = sensordata->vision;
		image.cameraParams[TOPCAMERA]    = cameraParams[TOPCAMERA];
		image.cameraParams[BOTTOMCAMERA] = cameraParams[BOTTOMCAMERA];
#ifdef EXPERIMENTAL_FEATURE
		image.fastSwitchCamera = false;
#endif
		session->setBool(PROPERTY_CMD_CHANGE_CAMERA_PARAMS, true);

		shm->signal(NAOQI);

		SESSION_END(session)
			return true;
		SESSION_CATCH_TERMINATE(session)
			return false;
	}

	void NaoQiAllSensorsModule::exec()
	{
		while (session->dontWait(), !exiting)
		{
			SESSION_TRY_START(session)
				
				shm->wait(RDK);
			unsigned long currentTimestamp = Timestamp().getMsFromMidnight();
			session->setDouble(PROPERTY_TIME_FPS, 1000. / (currentTimestamp - currentTs.getMsFromMidnight()));
			currentTs.setMsFromMidnight(currentTimestamp);
			RDK_TRACE_STREAM("Current FPS (sensors): " << session->getDouble(PROPERTY_TIME_FPS));
			session->setInt(PROPERTY_TIME_LAST_CALL,currentTimestamp);

			if (session->getBool(PROPERTY_CMD_CHANGE_CAMERA_PARAMS))
				doChangeCameraParams(new EventProperty(PROPERTY_CMD_CHANGE_CAMERA_PARAMS));
			RdkAllSensorData *sensordata = static_cast<RdkAllSensorData*>(shm->getEntry());

			int currentBuffer = 1;
#ifdef EXPERIMENTAL_FEATURE
			session->lock(PROPERTY_IN_CURRENT_BUFFER_IN_USE, HERE);
			RInt* currentBufferInUse = session->getObjectAsL<RInt>(PROPERTY_IN_CURRENT_BUFFER_IN_USE);
			currentBuffer = currentBufferInUse->value;
#endif
			readImage(sensordata->vision,currentBuffer);
			readSensors(sensordata->sensors,currentBuffer);
			computeStateFromSensors(sensordata->sensors);
#ifdef EXPERIMENTAL_FEATURE
			session->unlock(PROPERTY_IN_CURRENT_BUFFER_IN_USE);
			if (session->getBool(PROPERTY_CMD_SWITCH_CAMERA))
			{
				sensordata->vision.fastSwitchCamera = true;
				sensordata->vision.changeParams = true;
				session->setBool(PROPERTY_CMD_SWITCH_CAMERA,false);
			}
#endif
			shm->signal(NAOQI);

			SESSION_END_CATCH_TERMINATE(session)
		}
	}

	void NaoQiAllSensorsModule::readImage(RdkImageData& imageData, int currentBufferInUse)
	{
		unsigned char * dataPointer = imageData.data;
		size_t bufferSize = imageData.width*imageData.height*imageData.bpp;

		session->setInt(PROPERTY_OUT_CAMERA_IN_USE, imageData.selectedCameraID);

		RImage *rimage         = NULL;
#ifdef EXPERIMENTAL_FEATURE
		if (session->getBool(PROPERTY_PARAMS_USE_DOUBLE_BUFFER))
		{
			if (currentBufferInUse == 1)
			{
				session->lock(PROPERTY_OUT_IMAGE2, HERE);
				rimage = session->getObjectAsL<RImage>(PROPERTY_OUT_IMAGE2);
			}
			else
			{
				session->lock(PROPERTY_OUT_IMAGE1, HERE);
				rimage = session->getObjectAsL<RImage>(PROPERTY_OUT_IMAGE1); 
			}
		}
		else
#endif
		{
			session->lock(PROPERTY_OUT_IMAGE1, HERE);
			rimage = session->getObjectAsL<RImage>(PROPERTY_OUT_IMAGE1);
		}

		// TODO test this
#ifdef EXPERIMENTAL_FEATURE
		if (imageData.width != static_cast<int>(rimage->getWidth())
				||
				imageData.height != static_cast<int>(rimage->getHeight()))
		{
			RDK_DEBUG_STREAM("Camera #" << selectedCameraID << " Changing image size from " << rimage->getWidth() << "x" << rimage->getHeight() << " to " << imageData.width << "x" << imageData.height);
			rimage->canvasResize(imageData.width, imageData.height);
		}
		//RDK_WARNING_STREAM(imageData.width << "x" << imageData.height);
#endif
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
				RImage* rimg = rimage->convertTo(RImage::RGB32);
				rimg->imageResize(DEBUG_IMAGE_WIDTH,DEBUG_IMAGE_HEIGHT);
				session->setObject(PROPERTY_DBG_IMAGE, rimg);
				session->unlock(PROPERTY_DBG_IMAGE);
				session->valueChanged(PROPERTY_DBG_IMAGE);
			}
		}
		else
		{
			RDK_ERROR_STREAM("Null datapointer");
		}

		session->setInt(PROPERTY_TIME_LAST_IMAGE_READING, currentTs.getMsFromMidnight());

#ifdef EXPERIMENTAL_FEATURE
		if (session->getBool(PROPERTY_PARAMS_USE_DOUBLE_BUFFER))
		{
			if (currentBufferInUse == 1)
			{
				session->unlock(PROPERTY_OUT_IMAGE2);
				session->valueChanged(PROPERTY_OUT_IMAGE2/*, currentTs*/);
			}
			else
			{
				session->unlock(PROPERTY_OUT_IMAGE1);
				session->valueChanged(PROPERTY_OUT_IMAGE1/*, currentTs*/);
			}
		}
		else
#endif
		{
			session->unlock(PROPERTY_OUT_IMAGE1);
			session->valueChanged(PROPERTY_OUT_IMAGE1);
		}

		if (session->getBool(PROPERTY_CMD_FORCE_CHANGE))
		{
			session->setBool(PROPERTY_CMD_CHANGE_CAMERA_PARAMS, true);
			doChangeCameraParams(new EventProperty(PROPERTY_CMD_FORCE_CHANGE));
		}
	}

	void NaoQiAllSensorsModule::readSensors(RdkSensorData& sensors, int /*currentBufferInUse*/)
	{
		RNaoJoints *currentjoints         = NULL;
		RNaoSensors* currentsensors = NULL;

#ifdef EXPERIMENTAL_FEATURE
		if (session->getBool(PROPERTY_PARAMS_USE_DOUBLE_BUFFER))
		{
			if (currentBufferInUse == 1)
			{
				session->lock(PROPERTY_OUT_JOINTS2, HERE);
				currentjoints = session->getObjectAsL<RNaoJoints>(PROPERTY_OUT_JOINTS2);
				session->lock(PROPERTY_OUT_SENSORS2, HERE);
				currentsensors = session->getObjectAsL<RNaoSensors>(PROPERTY_OUT_SENSORS2);
			}
			else
			{
				session->lock(PROPERTY_OUT_JOINTS1, HERE);
				currentjoints = session->getObjectAsL<RNaoJoints>(PROPERTY_OUT_JOINTS1);
				session->lock(PROPERTY_OUT_SENSORS1, HERE);
				currentsensors = session->getObjectAsL<RNaoSensors>(PROPERTY_OUT_SENSORS1);
			}
		}
		else
#endif
		{
			session->lock(PROPERTY_OUT_JOINTS1, HERE);
			currentjoints = session->getObjectAsL<RNaoJoints>(PROPERTY_OUT_JOINTS1);
			session->lock(PROPERTY_OUT_SENSORS1, HERE);
			currentsensors = session->getObjectAsL<RNaoSensors>(PROPERTY_OUT_SENSORS1);
		}

		vector<float>& joints = currentjoints->getValues();
		for (size_t i = 0; i < joints.size(); i++)
			joints[i] = sensors.jointsValues[i];

		session->setDouble(PROPERTY_OUT_BATTERY,sensors.battery.charge);

		currentsensors->sensors = sensors;

		if (session->getBool(PROPERTY_DBG_ENABLED))
		{
			ostringstream ss;
			ss << *currentsensors << endl;
			session->setString(PROPERTY_OUT_ALLDATA,ss.str());

			ostringstream ss2;
			for (size_t k=0; k<JOINTS_VALUES_SIZE; k++) { 
				ss2 << currentsensors->sensors.jointsValues[k] << " "; 
			}
			session->setString(PROPERTY_OUT_POSE,ss2.str());

			RDK_TRACE_STREAM(currentsensors->getStringRepresentation());
			RDK_TRACE_STREAM(currentjoints->getStringRepresentation());
		}

		session->setInt(PROPERTY_TIME_LAST_SENSOR_READING, currentTs.getMsFromMidnight());

#ifdef EXPERIMENTAL_FEATURE
		if (session->getBool(PROPERTY_PARAMS_USE_DOUBLE_BUFFER))
		{
			if (currentBufferInUse == 1)
			{
				session->unlock(PROPERTY_OUT_SENSORS2);
				session->valueChanged(PROPERTY_OUT_SENSORS2,currentTs);
				session->unlock(PROPERTY_OUT_JOINTS2);
				session->valueChanged(PROPERTY_OUT_JOINTS2,currentTs);
			}
			else
			{
				session->unlock(PROPERTY_OUT_SENSORS1);
				session->valueChanged(PROPERTY_OUT_SENSORS1,currentTs);
				session->unlock(PROPERTY_OUT_JOINTS1);
				session->valueChanged(PROPERTY_OUT_JOINTS1,currentTs);
			}
		}
		else
#endif
		{
			session->unlock(PROPERTY_OUT_SENSORS1);
			session->valueChanged(PROPERTY_OUT_SENSORS1/*,currentTs*/);
			session->unlock(PROPERTY_OUT_JOINTS1);
			session->valueChanged(PROPERTY_OUT_JOINTS1/*,currentTs*/);
		}
	}

		void NaoQiAllSensorsModule::computeStateFromSensors(RdkSensorData& sensors)
		{

		/**************************************************************************************/
		/** Handle the button sensor feedback from xsentinel,
		 *   we don't care which state the xsentinel provided, only care the changes
		 */
		std::string tmpgamestate = sensors.feedbackSensor.gamestateByXSentinel;
		bool tmpIsBlueTeam = sensors.feedbackSensor.isBlueTeamByXSentinel;

		std::string gamestateFromGC = session->getString(PROPERTY_IN_GAMESTATE);
		int penalizedFromGC = session->getInt(PROPERTY_IN_PENALTY);
		int colorFromGC = session->getInt(PROPERTY_IN_TEAMCOLOR);

		std::string realgamestate = session->getString(PROPERTY_OUT_FINAL_GAMESTATE);
		int realPenalize = session->getInt(PROPERTY_OUT_FINAL_PENALIZE);
		int realColor = session->getInt(PROPERTY_OUT_FINAL_TEAMCOLOR);

		mMyUNum = session->getInt(PROPERTY_IN_UNIFORM_NUMBER);

		/*
		std::stringstream ss;
		ss << "------------------\n";
		ss << "mGameStateFromXSentinel: " << mGameStateFromXSentinel << "\n";
		ss << "tmpgamestate:            " << tmpgamestate << "\n";
		ss << "gamestateFromGC:         " << gamestateFromGC << "\n";
		ss << "penalizedFromGC:         " << penalizedFromGC << "\n";
		ss << "colorFromGC:             " << colorFromGC << "\n";

		RDK_INFO_PRINTF(ss.str().c_str());
		*/

		if (mIsTheFirstTime)
		{
			mGameStateFromXSentinel = tmpgamestate;

			mIsTheFirstTime = false;
		}

		// button pressed		
		if (mGameStateFromXSentinel != tmpgamestate)
		{
			if (realPenalize != 0)
			{
				realPenalize = 0;
			}
			else
			{	
				realPenalize = 1;

				// If the realgamestate belongs to "initial, ready, set", go to play, regardless there is a GC or not.
				if ((realgamestate == "initial" || realgamestate == "ready" || realgamestate == "set"))
				{
					realgamestate = "play";
				}
			}

			mGameStateFromXSentinel = tmpgamestate;
		}
		else
		{
			// If penalized, don't listen to the gamecontroller
			// two way to release it:
			// 1. button
			// 2. timeout, and received unpenalized from the gamecontroller
			if (gamestateFromGC != "nogamestate")
			{
				// obey gamecontroller
				realgamestate = gamestateFromGC;
				realPenalize = penalizedFromGC;
			}
			else if (realPenalize != 0)
			{
				//if (gamestateFromGC == "play" && 
				//	penalizedFromGC == 0 &&
				//	mLastPenalizedTimeValid)
				if (mLastPenalizedTimeValid)
				{
					int diff = session->getDouble(PROPERTY_PARAM_DEFAULT_PENALIZE_TIME) - (GetSeconds() - mLastPenalizedTime);
					//RDK_INFO_PRINTF("=============\nDiff: %i\n", diff);
					if (diff < 0)
					{
						// There is a game controller, remind the referee, orelse release automaticlly(not anymore)
						if (gamestateFromGC == "nogamestate")
						{
							// no release
							// Auto release
							//realPenalize = 0;

							if (mMyUNum != 1) // goalie does not have this mechanism...
							{
								// Remind
								std::stringstream ss;
								ss << "Please unpenalize me";
								session->setString(PROPERTY_OUT_TTS_TEXT, ss.str());
								RDK_INFO_PRINTF("Say: %s", ss.str().c_str());
							}
						}
						else
						{
							if (penalizedFromGC != 0)
							{
								if (mMyUNum != 1)	// goalie does not have this mechanism...
								{
									// Remind
									std::stringstream ss;
									ss << "Please unpenalize me";
									session->setString(PROPERTY_OUT_TTS_TEXT, ss.str());
									RDK_INFO_PRINTF("Say: %s", ss.str().c_str());
								}
							}
							else
							{
								// obey game controller
								realgamestate = gamestateFromGC;
								realPenalize = penalizedFromGC;
							}
						}
					}
					else if (diff <= 5 && diff >= 1)
					{
						if (mHaveSaidTime < 0 || (mHaveSaidTime - diff) == 1)
						{
							if (mMyUNum != 1)	// goalie does not have this mechanism...
							{
								std::stringstream ss;
								//ss << "unpenalized in " << diff << (diff == 1? " second" : " seconds");
								ss << diff;
								session->setString(PROPERTY_OUT_TTS_TEXT, ss.str());
								RDK_INFO_PRINTF("say: %s", ss.str().c_str());
								mHaveSaidTime = diff;
							}
						}
					}
				}
				else
				{
					// ignore
					//RDK_INFO_PRINTF("=============\nError Detected\n");
				}
			}
		}

		if (realgamestate == "initial")
		{
			if (gamestateFromGC != "nogamestate")
			{
				// obey game controller
				realColor = colorFromGC;
			}
			else if (mIsBlueTeamFromXSentinel != tmpIsBlueTeam)
			{
				realColor = (realColor != 0 ? 0 : 1);
				mIsBlueTeamFromXSentinel = tmpIsBlueTeam;
			}

		}

		// detect the penalized change
		if (realPenalize == 0) mLastPenalizedTimeValid = false;
		else if (session->getInt(PROPERTY_OUT_FINAL_PENALIZE) == 0)
		{
			// just penalized
			mLastPenalizedTime = GetSeconds();
			mLastPenalizedTimeValid = true;
			mHaveSaidTime = -1;
		}

		// write
		session->setString(PROPERTY_OUT_FINAL_GAMESTATE, realgamestate);
		session->setInt(PROPERTY_OUT_FINAL_PENALIZE, realPenalize);
		session->setInt(PROPERTY_OUT_FINAL_TEAMCOLOR, realColor);

		/*
		RDK_INFO_PRINTF("\n===========================");
		RDK_INFO_PRINTF("Timestamp:    %f", GetSeconds());	
		RDK_INFO_PRINTF("GSFromGC:     %s", gamestateFromGC.c_str());
		RDK_INFO_PRINTF("PenFromGC:    %i", penalizedFromGC);
		RDK_INFO_PRINTF("GameState:    %s", realgamestate.c_str());
		RDK_INFO_PRINTF("realPenalize: %i", realPenalize);
		RDK_INFO_PRINTF("realColor:    %i", realColor);			
		RDK_INFO_PRINTF("PenaValid:    %i", mLastPenalizedTimeValid);	
		RDK_INFO_PRINTF("Pena Time:    %f", mLastPenalizedTime);
		int tmp = (int)session->getDouble(PROPERTY_PARAM_DEFAULT_PENALIZE_TIME);		
		RDK_INFO_PRINTF("TIMEOUT:      %i", tmp);	
		*/

		/** set the feedback data */
		session->setString(PROPERTY_OUT_MOTION_TASK_FROM_NMM, sensors.feedbackSensor.motionModuleTaskByNaoMotionModule);
		session->setBool(PROPERTY_OUT_IS_HEAD_MOVING, sensors.feedbackSensor.isHeadCommandExecutingByNaoMotionModule);
		session->setPose(PROPERTY_OUT_ODOMETRY_POSE, Point2od(
								sensors.feedbackSensor.odometryXByNaoMotionModule,
								sensors.feedbackSensor.odometryYByNaoMotionModule,
								sensors.feedbackSensor.odometryThetaByNaoMotionModule));
	}

	bool NaoQiAllSensorsModule::doChangeCameraParams(const Event* e)
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

		RdkAllSensorData *sensordata = static_cast<RdkAllSensorData*>(shm->getEntry());
		RdkImageData& image = sensordata->vision;
		image.cameraParams[TOPCAMERA]    = cameraParams[TOPCAMERA];
		image.cameraParams[BOTTOMCAMERA] = cameraParams[BOTTOMCAMERA];
		image.selectedCameraID = selectedCameraID;
		image.changeParams = true;

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


	bool NaoQiAllSensorsModule::saveCameraParams(int index)
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
		if (session->getBool(PROPERTY_PARAMS_LINK_PARAMS))
		{
			for (int i=0; i<NUMBEROFCAMERAS; ++i)
				cameraParams[i] = cameraParams[BOTTOMCAMERA];
		}
		return true;
	}

	bool NaoQiAllSensorsModule::restoreCameraParams(int index)
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
		return true;
	}

	// implement this if you need to force the exec to go out of the loop
	// if the thread may be waiting not for the session->wait() semaphore:
	// on closing, the main thread will set the exiting variables, call this exitRequested()
	// function and then signal the session semaphores
	void NaoQiAllSensorsModule::exitRequested()
	{
		if (shm == 0)
			return;
		shm->signal(NAOQI);
	}

	// implement this if you need to clean up things after the exec has exited
	//void NaoQiAllSensorsModule::cleanup()
	//{}

	// void NaoQiAllSensorsModule::asyncAgentCmd(cstr cmd)
	// {
	//	SESSION_TRY_START(asyncSession)
	//	// here you can parse 'cmd'
	//	SESSION_END_CATCH_TERMINATE(asyncSession)
	// }

	
	float NaoQiAllSensorsModule::GetSeconds()
	{
		return Timestamp().getMsFromMidnight() * 0.001;
	}

	MODULE_FACTORY(NaoQiAllSensorsModule);

}} // namespace
