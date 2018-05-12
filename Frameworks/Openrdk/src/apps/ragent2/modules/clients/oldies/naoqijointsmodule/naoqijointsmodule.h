#ifndef RDK2_MODULE_NAOQIJOINTSMODULE
#define RDK2_MODULE_NAOQIJOINTSMODULE

#include <rdkcore/modules/module.h>
#include <rdkcore/sharedmemory/sharedmemory.h>
#include <rdkcore/time/time.h>

#define EVENT_HANDLER_CLASS NaoQiJointsModule

namespace RDK2 { namespace RAgent {

class NaoQiJointsModule : public RDK2::RAgent::Module {
public:
	NaoQiJointsModule() { }
	virtual ~NaoQiJointsModule() { }

	bool initConfigurationProperties();
	bool init();
	void exec();
	void exitRequested();
	void cleanup();
	void closing();

private:
	Time::Timestamp timestamp;
	unsigned long lastts;
	int wait_for_sensor_reading;
	SharedMemory *shmmotion, *shmsensor;

	// added by massimo
	double oldStiffness,newStiffness;
	int oldStiffnessChain,newStiffnessChain;
	//
	//added by gianluca
	bool motionCmdUpdateEvent(const Event* e);	
	DECLARE_EVENT_HANDLER(motionCmdUpdateEvent);
};

}} // namespace

#endif
