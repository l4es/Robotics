#include "sharedmemory.h"
#include "sharedmemory_traits.h"

//#include "motionshm.h"
//#include "sensordatashm.h"

#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/types.h>
#include <sys/shm.h>
#include <iostream>

namespace RDK2 {

	using namespace std;

	SharedMemory::SharedMemory(SharedInformationType t,size_t size):type(t){

		memoryPlug=createSharedMemory(size);
		creationResultIsGood=(memoryPlug!=-1);
		if(creationResultIsGood)
			creationResultIsGood=(plugSharedMemory()!=-1)?true:false;
		if(creationResultIsGood)
			creationResultIsGood =	
				((trafficLigthRdk=semget(TRAFFIC_RDK_NAOQI_KEY+type,2,IPC_CREAT|0666))>0) && 
				((trafficLigthNaoqi=semget(TRAFFIC_NAOQI_RDK_KEY+type,2,IPC_CREAT|0666))>0);
		if(creationResultIsGood)
			creationResultIsGood =  (semctl(trafficLigthRdk, 0, SETVAL, 0)!=-1) && 
				(semctl(trafficLigthNaoqi, 0, SETVAL, 0)!=-1);
	}

	SharedMemory::~SharedMemory(){
		//distruggi semaforo
		if(unplugSharedMemory()!=-1)
			destroySharedMemory();
	}

	int SharedMemory::wait(int semNumber) {
		sembuf subOne[1] = {{0,-1,0}};
		switch (semNumber) {
			case RDK   : return semop(trafficLigthRdk,subOne,1);
			case NAOQI : return semop(trafficLigthNaoqi,subOne,1);
			default    : return -1;
		}
	}

	int SharedMemory::signal(int semNumber){
		sembuf subOne[1] = {{0,1,0}};
		switch (semNumber) {
			case RDK   : return semop(trafficLigthRdk,subOne,1);
			case NAOQI : return semop(trafficLigthNaoqi,subOne,1);
			default    : return -1;
		}
	}

	int SharedMemory::createSharedMemory(size_t size)
	{
		cout << "RDKSHM: creating shmem key " << MEM_KEY+type << " size: " << size << endl;
		int sh_result = shmget(MEM_KEY+type, size, IPC_EXCL|0666);
		if (sh_result == -1)
			sh_result = shmget(MEM_KEY+type, size, IPC_CREAT|IPC_EXCL|0666);
		
		return sh_result;
	}

	int SharedMemory::plugSharedMemory()
	{
		int result;
		void *addr_ptr;
		addr_ptr = shmat(getMemoryPlug(), 0, SHM_R|SHM_W);

		if (addr_ptr == (void *) -1)
		{
			memoria   = NULL;
			result = -1;
		}
		else
		{
			memoria = addr_ptr;
			// memoria = new (addr_ptr) RdkMotionData();
			result = 0;
		}

		return result;
	}

	int SharedMemory::unplugSharedMemory()
	{
		return shmdt(memoria);
	}

	int SharedMemory::destroySharedMemory()
	{
		return shmctl(MEM_KEY+type, IPC_RMID, NULL);
	}

}
