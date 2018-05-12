#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include "DBIOS/DBIOS_MAGICV_MUTEX.h"
#include "FreeRTOS_include/queue.h"
#include "DSPAbstract_TaskManager.h"
#include "FreeRTOS_include/task.h"

//  Retrieve a message from a task
DA_RETCODE DSPAbstractTM_GetMsg(DA_TLTasks *pTask, DA_Msg * msg, DA_int iTimeout){
	DA_RETCODE retcode = DA_SUCCESS;
	portTickType currTime;
	portTickType initTime;
	if (iTimeout > 0)
		initTime = xTaskGetTickCount();
try_again: ;
	if (!D940F_MAGICV_MUTEX_Lock(_M_STRUCTI(DA_TLTasks, pTask)->iMutex)){
		retcode = DA_SUCCESS;
#ifdef __chess__
	        if ((int)xQueueReceive(_M_STRUCTI(DA_TLTasks, pTask)->hQueueRecv, (void *)msg, (portTickType)iTimeout) == (int)pdFALSE){
#else
	        if ((int)xQueueReceive(_M_STRUCTI(DA_TLTasks, pTask)->hQueueSend, (void *)msg, (portTickType)iTimeout) == (int)pdFALSE){
#endif
	        	retcode = DA_ETIMEOUT;
	        }
		D940F_MAGICV_MUTEX_Unlock(_M_STRUCTI(DA_TLTasks, pTask)->iMutex);
	}else if (iTimeout > 0){
		retcode = DA_ENOMUTEXLOCK;
#ifdef __chess__
		taskYIELD();
#else
		usleep(ARM_USLEEP_INTERVAL);
#endif
		currTime = xTaskGetTickCount();
		if ((currTime - initTime) <= iTimeout)
			goto try_again;
	}
endf:
	return retcode;
}

//  Send a message from this task to ARM 
DA_RETCODE DSPAbstractTM_PutMsg(DA_TLTasks *pTask, DA_Msg * msg, DA_int iTimeout){
	DA_RETCODE retcode = DA_SUCCESS;

	if (!D940F_MAGICV_MUTEX_Lock(_M_STRUCTI(DA_TLTasks, pTask)->iMutex)){
		printf("pm-- %d %ld\n", _M_STRUCTI(DA_TLTasks, pTask)->iMutex, _M_STRUCTI(DA_TLTasks, pTask)->hQueueRecv);
		printf("pmmsg-- %ld %ld\n", msg->iMsg, msg->lArg);

#ifdef __chess__
	        if (xQueueSend( _M_STRUCTI(DA_TLTasks, pTask)->hQueueSend, (void *)msg, (portTickType) iTimeout ) != pdPASS ){
#else
	        if (xQueueSend( _M_STRUCTI(DA_TLTasks, pTask)->hQueueRecv, (void *)msg, (portTickType) iTimeout ) != pdPASS ){
#endif
	        	retcode = DA_ETIMEOUT;
        	}
		D940F_MAGICV_MUTEX_Unlock(_M_STRUCTI(DA_TLTasks, pTask)->iMutex);
        }else{
		retcode = DA_ENOMUTEXLOCK;
        }
	return retcode;
}

//  Wait until a message has arrived or a datastream is ready 
DA_RETCODE DSPAbstractTM_Wait(){
	return DA_SUCCESS;
}

//  Allocate a buffer whose descriptor will be sent to ARM as a message 
DA_RETCODE DSPAbstractTM_AllocMsgBuf(){
	return DA_SUCCESS;
}

//  Free a buffer whose descriptor will be sent to ARM as a message 
DA_RETCODE DSPAbstractTM_FreeMsgBuf(){
	return DA_SUCCESS;
}
