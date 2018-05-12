#ifndef OPENRDK_RLEDSSERVER_H
#define OPENRDK_RLEDSSERVER_H

#include <alloggerproxy.h>
#include <rdkcore/sharedmemory/sharedmemory.h>

namespace AL
{
	class ALBroker;
}

class RLedsServer : public AL::ALModule
{
	protected:
		AL::ALPtr<AL::ALLoggerProxy> logger_proxy;
		AL::ALPtr<AL::ALProxy> led_proxy;
		RDK2::SharedMemory* shm_led;
		pthread_t leds_manager_thread_id;
		
		void ledsManager();
		
	public:
		RLedsServer(AL::ALPtr<AL::ALBroker>,const std::string&);
		virtual ~RLedsServer();
		
		virtual void init();
		bool isPlugged() { return shm_led->pluggingSuccessful(); }
		static void* ledsManagerThread(RLedsServer* me) { me->ledsManager(); return NULL; }
		
		void setBallSeen(int);
		void setGameState(int);
		void setGoalSeen(bool,bool,bool,bool);
		void setKickOff(bool,int);
		void setPlayerRole(int);
		void setTeamColor(int);
};

#endif
