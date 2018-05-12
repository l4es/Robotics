#ifndef DSPABSTRACT_DM_H
#define DSPABSTRACT_DM_H

#include "TaskLauncher.h"

enum {
	DA_SM_TOARM,
	DA_SM_FROMARM
};

typedef struct {
	DA_TLTasks *pTask;
	DA_long *lBuffer;
	DA_int iSize;
	DA_int iNumBuffer;
	DA_int iDirection;
	DA_int iMutex;
	DA_int iIndex;
} DA_SMStream;

//  Create a stream
DA_RETCODE DSPAbstractDM_Create(DA_SMStream *pStream, DA_int iDirection, DA_int iSize, DA_int iNumBuffer);
//  Delete a stream
DA_RETCODE DSPAbstractDM_Delete(DA_SMStream *pStream);
//  Issue a buffer on a stream
DA_RETCODE DSPAbstractDM_Issue(DA_SMStream *pStream, void *pBuff, DA_int iSize, DA_int iTimeout, DA_int *pBuffIndex);
//  Reclaim a buffer from a stream
DA_RETCODE DSPAbstractDM_Reclaim(DA_SMStream *pStream);

#endif
