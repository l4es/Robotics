#ifndef TASKLAUNCHER_H
#define TASKLAUNCHER_H

#include "FreeRTOS_include/FreeRTOS.h"
#include "FreeRTOS_include/queue.h"
#include "DSPAbstract.h"

enum DA_TM_MSG_COMMANDS{
	DA_TM_MSG_CREATETASK,
	DA_TM_MSG_TASKCREATE_SUCCESS,
	DA_TM_MSG_TASKCREATE_FAILED,
	DA_TM_MSG_CREATESTREAM,
	DA_TM_MSG_STREAMCREATE_SUCCESS,
	DA_TM_MSG_STREAMCREATE_FAILED,
	DA_TM_MSG_STREAMISSUE,
	DA_TM_MSG_STREAMRECLAIM,
	DA_TM_MSG_ALLOCMEMORY,
	DA_TM_MSG_ALLOCMEMORY_DONE,
	DA_TM_MSG_FREEMEMORY,
	DA_TM_MSG_FREEMEMORY_DONE
};

//struct for storing the create task info
typedef struct {
	DA_char *pName;
	DA_long lAddress;
	DA_int iPriority;
	DA_int iQueueElemNum;
} DA_TLCreateTaskArgs;

//struct for storing the tasks info
typedef struct {
	DA_long lAddress;
	DA_int iIndex;
	DA_int iMutex;
	xQueueHandle hQueueSend;
	xQueueHandle hQueueRecv;
} DA_TLTasks;

//struct for sending a command to the TaskLauncher
typedef struct {
	enum DA_TM_MSG_COMMANDS iCode; //FIXME
	DA_long *pArgs;
} DA_TLcommand;

//struct for sending task creation data to the TaskLauncher
typedef struct {
	DA_char *pName;
	DA_int iPriority;
} DA_TLTaskData;

//struct for storing the TaskLauncher info
typedef struct {
	DA_int iMaxTaskNum;
	DA_int iRegisteredTasksNum;
	DA_int iRegisteredStreamsNum;
	DA_int iRegisteredMutexNum;
	DA_int iQueueElemNum;
	DA_TLTasks *pTasksInfo;
} DA_TLParams;

#endif
