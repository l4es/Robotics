#ifndef RDK2_MODULE_NAOQIAUDIOMODULE
#define RDK2_MODULE_NAOQIAUDIOMODULE

#include <rdkcore/modules/module.h>
#include <rdkcore/sharedmemory/sharedmemory.h>
#include <rdkcore/sharedmemory/sensordatashm.h>

namespace RDK2 { namespace RAgent {

#define EVENT_HANDLER_CLASS NaoQiAudioModule

class NaoQiAudioModule : public Module {
public:
	NaoQiAudioModule(): shmem(NULL) { }
	virtual ~NaoQiAudioModule()
	{
		if (shmem == 0)
			return;
		delete shmem;
}
	

	bool initConfigurationProperties();
	bool init();
	void exec();
	void exitRequested();
	//void cleanup();

private:
	SharedMemory* shmem;

	bool doSayToShm(const Event* e);
	DECLARE_EVENT_HANDLER(doSayToShm);
	void sayToShm(RdkAudioData* data);
};

}} // namespace

#endif
