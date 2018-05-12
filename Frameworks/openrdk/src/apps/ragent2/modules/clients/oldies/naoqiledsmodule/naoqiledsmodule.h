#ifndef RDK_MODULE_NAOQILEDSMODULE
#define RDK_MODULE_NAOQILEDSMODULE
#define MODULE_NAME "NaoQiLedsModule"

#include <rdkcore/modules/module.h>
#include <rdkcore/sharedmemory/sharedmemory.h>

namespace RDK2
{
	namespace RAgent
	{
		#define EVENT_HANDLER_CLASS NaoQiLedsModule
		
		class NaoQiLedsModule : public Module
		{
			private:
				SharedMemory* shmem;
				
				bool propertyEventPropertyBallSeen(const Event*);
				bool propertyEventPropertyGameController(const Event*);
				bool propertyEventPropertyKickOff(const Event*);
				bool propertyEventPropertyPenalized(const Event*);
				bool propertyEventPropertyTeamColor(const Event*);
				bool timerEvent(const Event*);
				
				DECLARE_EVENT_HANDLER(propertyEventPropertyBallSeen);
				DECLARE_EVENT_HANDLER(propertyEventPropertyGameController);
				DECLARE_EVENT_HANDLER(propertyEventPropertyKickOff);
				DECLARE_EVENT_HANDLER(propertyEventPropertyPenalized);
				DECLARE_EVENT_HANDLER(propertyEventPropertyTeamColor);
				DECLARE_EVENT_HANDLER(timerEvent);
				
			public:
				NaoQiLedsModule() : shmem(NULL) {;}
				virtual ~NaoQiLedsModule()
				{
					if (shmem != NULL)
					{
						delete shmem;
						
						shmem = NULL;
					}
				}
				
				bool initConfigurationProperties();
				bool init();
				void exec();
		};
	}
}

#endif
