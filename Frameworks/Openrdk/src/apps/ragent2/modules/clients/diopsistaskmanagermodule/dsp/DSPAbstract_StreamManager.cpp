#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include "DBIOS/DBIOS_MAGICV_MUTEX.h"
#include "FreeRTOS_include/queue.h"
#include "DSPAbstract_StreamManager.h"

//  Create a stream
DA_RETCODE DSPAbstractDM_Create(DA_SMStream *pStream, DA_int iDirection, DA_int iSize, DA_int iNumBuffer){
}

//  Delete a stream
DA_RETCODE DSPAbstractDM_Delete(DA_SMStream *pStream){
}

//  Issue a buffer on a stream
DA_RETCODE DSPAbstractDM_Issue(DA_SMStream *pStream, void *pBuff, DA_int iSize, DA_int iTimeout, DA_int *pBuffIndex){
}

//  Reclaim a buffer from a stream
DA_RETCODE DSPAbstractDM_Reclaim(DA_SMStream *pStream){
}

