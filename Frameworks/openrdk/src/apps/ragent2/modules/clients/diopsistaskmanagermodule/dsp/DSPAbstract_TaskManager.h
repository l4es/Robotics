#ifndef DSPABSTRACT_MSG_H
#define DSPABSTRACT_MSG_H

#include "TaskLauncher.h"

//Message structure: command + argument
typedef struct {
    DA_int	iMsg;
    DA_long	lArg;
} DA_Msg;

//  Retrieve a message from ARM for a task
DA_RETCODE DSPAbstractTM_GetMsg(DA_TLTasks *pTask, DA_Msg * msg, DA_int iTimeout);
//  Send a message from this task to ARM 
DA_RETCODE DSPAbstractTM_PutMsg(DA_TLTasks *pTask, DA_Msg * msg, DA_int iTimeout);
//  Wait until a message has arrived or a datastream is ready 
DA_RETCODE DSPAbstractTM_Wait();
//  Allocate a buffer whose descriptor will be sent to ARM as a message 
DA_RETCODE DSPAbstractTM_AllocMsgBuf();
//  Free a buffer whose descriptor will be sent to ARM as a message 
DA_RETCODE DSPAbstractTM_FreeMsgBuf();


#endif
