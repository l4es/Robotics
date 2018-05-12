/**
 * @file
 * @brief This file contains the declaration of DemoVisionNaoQiModule
 */

#ifndef RDK_MODULE_DEMOVISIONNAOQIMODULE
#define RDK_MODULE_DEMOVISIONNAOQIMODULE

#include <rdkcore/modules/module.h>
#include <albroker.h>
#include <alproxy.h>
#include <rdkcore/sharedmemory/sharedmemory.h>
#include <rdkcore/sharedmemory/sensordatashm.h>
#include "demovisionnaoqimodule_names.h"


using namespace AL;

namespace RDK2 { namespace RAgent {

/**
 * @brief Please write a SHORT description of DemoVisionNaoQiModule here.
 *
 * Please write a description of DemoVisionNaoQiModule here.
 *
 * @ingroup RAgentModules
 */

#define EVENT_HANDLER_CLASS DemoVisionNaoQiModule

class DemoVisionNaoQiModule : public Module {
public:
	DemoVisionNaoQiModule() { }
	virtual ~DemoVisionNaoQiModule() { }

	bool initConfigurationProperties();
	// bool initInterfaceProperties();
	bool init();
	void exec();

private:

#define OpenRDK_ARCH_NOTLINUX OpenRDK_ARCH_GEODE || OpenRDK_ARCH_ATOM

#ifndef OpenRDK_ARCH_NOTLINUX
	ALPtr<ALBroker> broker;
#else
	SharedMemory *shmimage;
#endif
	ALPtr<ALProxy> cameraProxy;
	std::string pID;
	unsigned int bufferSize;

	//bool connect();
	bool connected;

	NaoCameraParams cameraParams[NUMBEROFCAMERAS];
	int selectedCameraID;
	void saveCameraParams(int index);
	void restoreCameraParams(int index);


	bool timerEvent(const Event* e);			DECLARE_EVENT_HANDLER(timerEvent);
	bool doConnection(const Event* e);	DECLARE_EVENT_HANDLER(doConnection);
	bool doChangeCameraParams(const Event* e);	DECLARE_EVENT_HANDLER(doChangeCameraParams);
	bool doChangeResolution(const Event* e);	DECLARE_EVENT_HANDLER(doChangeResolution);
	bool doChangeFrameRate(const Event* e);	DECLARE_EVENT_HANDLER(doChangeFrameRate);
	bool doChangeColorspace(const Event* e);	DECLARE_EVENT_HANDLER(doChangeColorspace);

	
};

}} // namespace

#endif
