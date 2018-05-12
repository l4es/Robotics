/**
 * @file
 * @brief This file contains the declaration of NaoQiAllSensorsModule
 */

#ifndef RDK_MODULE_NAOQIALLSENSORSMODULE
#define RDK_MODULE_NAOQIALLSENSORSMODULE

#include <rdkcore/modules/module.h>
#include <rdkcore/sharedmemory/sharedmemory.h>
#include <nao-objects/sensordatashm.h>

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
		if (shm != NULL)
		{
			delete shm;
			shm = NULL;
		}
	}

	bool initConfigurationProperties();
	// bool initInterfaceProperties();
	bool init();
	void exec();
	void exitRequested();

private:
	RDK2::SharedMemory *shm;

	void readImage(RdkImageData& imageData, int currentBufferInUse);
	void readSensors(RdkSensorData& sensors, int currentBufferInUse);
	void computeStateFromSensors(RdkSensorData& sensors);

	bool doChangeCameraParams(const Event* e);

	int selectedCameraID;
	bool saveCameraParams(int index);
	bool restoreCameraParams(int index);
	std::string CAMERA_PREFIX[NUMBEROFCAMERAS];
	NaoCameraParams cameraParams[NUMBEROFCAMERAS];
	
	Timestamp currentTs;

	bool mIsTheFirstTime;
	std::string mGameStateFromXSentinel;
	bool mIsBlueTeamFromXSentinel;
	
	bool mLastPenalizedTimeValid;
	float mLastPenalizedTime;
	int mHaveSaidTime;

	/** my uniform number, if I am the goalie, I don't have the 30's mechanism of reminding sb. to release the penelized state*/
	int mMyUNum;
	
	float GetSeconds();
};

}} // namespace

#endif
