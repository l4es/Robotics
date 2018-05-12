/**
 * @file
 * @brief This file contains the declaration of NaoQiAllSensorsModule
 */

#ifndef RDK_MODULE_NAOQIALLSENSORSMODULE
#define RDK_MODULE_NAOQIALLSENSORSMODULE

#include <rdkcore/modules/module.h>
#include <rdkcore/sharedmemory/sharedmemory.h>
#include <rdkcore/sharedmemory/sensordatashm.h>
namespace RDK2 { namespace RAgent {

/**
 * @brief Please write a SHORT description of NaoQiAllSensorsModule here.
 *
 * Please write a description of NaoQiAllSensorsModule here.
 *
 * @ingroup RAgentModules
 */

#define EVENT_HANDLER_CLASS NaoQiAllSensorsModule

class NaoQiAllSensorsModule : public Module {
public:
	NaoQiAllSensorsModule():shm(NULL) { }
	virtual ~NaoQiAllSensorsModule()
	{
		if (shm == NULL)
			return;
		delete shm;
	}

	bool initConfigurationProperties();
	// bool initInterfaceProperties();
	bool init();
	void exec();
	void exitRequested();

private:
	RDK2::SharedMemory::SharedMemory *shm;
	//std::string pID;
	unsigned int bufferSize;

	std::string CAMERA_PREFIX[NUMBEROFCAMERAS];

	NaoCameraParams cameraParams[NUMBEROFCAMERAS];
	int selectedCameraID;
	bool saveCameraParams(int index);
	bool restoreCameraParams(int index);

	bool doChangeCameraParams(const Event* e);
	DECLARE_EVENT_HANDLER(doChangeCameraParams);
	bool doChangeResolution(const Event* e);
	//DECLARE_EVENT_HANDLER(doChangeResolution);
	bool doChangeFrameRate(const Event* e);
	//DECLARE_EVENT_HANDLER(doChangeFrameRate);
	bool doChangeColorspace(const Event* e);
	//DECLARE_EVENT_HANDLER(doChangeColorspace);

	void readImage(RdkImageData& imageData);
	void readSensors(RdkSensorData& sensors);
};

}} // namespace

#endif
