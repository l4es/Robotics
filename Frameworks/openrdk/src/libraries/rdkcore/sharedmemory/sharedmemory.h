#ifndef RDK2_SHAREDMEMORY_NAOQI
#define RDK2_SHAREDMEMORY_NAOQI

#include <vector>
#include <string>

#include "sharedmemory_traits.h"

namespace RDK2 {


class SharedMemory {
public:
	enum SharedInformationType { MOTION=0, SENSOR=1, AUDIO=2, LEDS=5, IMAGE=4, ALLSENSOR=6, DCM=7, GAMECONTROLLER=8 };

	SharedMemory(SharedInformationType type,size_t size);

	~SharedMemory();
	
	bool pluggingSuccessful() { return creationResultIsGood; }
	
	int signal(int semNum);
	int wait(int semNum);

	void * getEntry() { return memoria; }
	
	int getMemoryPlug() { return memoryPlug; }

private :
	
	void * memoria;

	int  trafficLigthNaoqi;
	int  trafficLigthRdk;
	int  memoryPlug;
 	bool creationResultIsGood;
	
	int createSharedMemory(size_t size);
	int destroySharedMemory();
	int unplugSharedMemory();
	int plugSharedMemory();

	SharedInformationType type;
};

}

#endif

