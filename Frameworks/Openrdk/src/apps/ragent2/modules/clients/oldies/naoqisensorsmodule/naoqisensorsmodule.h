#ifndef RDK2_MODULE_NAOQISENSORSMODULE
#define RDK2_MODULE_NAOQISENSORSMODULE

#include <rdkcore/modules/module.h>
#include <rdkcore/sharedmemory/sharedmemory.h>

namespace RDK2 { namespace RAgent {

#define EVENT_HANDLER_CLASS NaoQiSensorsModule

class NaoQiSensorsModule : public Module {
public:
	NaoQiSensorsModule(): shmsensor(NULL) { }
	virtual ~NaoQiSensorsModule()
	{
		if (shmsensor == NULL)
			return;
		delete shmsensor;
	}

	bool initConfigurationProperties();
	bool init();
	void exec();
	void exitRequested();
	//void cleanup();

private:
	SharedMemory* shmsensor;
};

}} // namespace

#endif
