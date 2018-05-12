/**
 * @file
 * @brief This file contains the declaration of NaoQiVisionModule
 */

#ifndef RDK_MODULE_NAOQIVISIONMODULE
#define RDK_MODULE_NAOQIVISIONMODULE

#include <rdkcore/modules/module.h>
#include <rdkcore/sharedmemory/sharedmemory.h>
#include <rdkcore/sharedmemory/sensordatashm.h>

namespace RDK2 { namespace RAgent {

/**
 * @brief Please write a SHORT description of NaoQiVisionModule here.
 *
 * Please write a description of NaoQiVisionModule here.
 *
 * @ingroup RAgentModules
 */

#define EVENT_HANDLER_CLASS NaoQiVisionModule

class NaoQiVisionModule : public Module {
public:
	NaoQiVisionModule():shmimage(NULL) { }
	virtual ~NaoQiVisionModule()
	{
		if (shmimage == NULL)
			return;
		delete shmimage;
	}

	bool initConfigurationProperties();
	// bool initInterfaceProperties();
	bool init();
	void exec();
	void exitRequested();

private:
	RDK2::SharedMemory::SharedMemory *shmimage;
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

	
};

}} // namespace

#endif
