#ifndef DSPABSTRACT_H
#define DSPABSTRACT_H
#include "FreeRTOS_include/FreeRTOS.h"

#define DA_char portCHAR
#define DA_int portSHORT
#define DA_long portLONG
#define DA_float portFLOAT
#define DA_double portDOUBLE

#define ARM_USLEEP_INTERVAL 1000


#define DA_PRIORITY_LOW tskIDLE_PRIORITY

//Max nuber of tasks
#define DA_MAX_TASK_NUM 4
//Max nuber of mutexes
#define DA_MAX_MUTEX_NUM DA_MAX_TASK_NUM
//Default mailbox queue length
#define DA_QUEUE_LENGTH 4


typedef enum {
	DA_SUCCESS = 0,
	DA_EGENERIC = -1,
	DA_ETOOMANYTASKS = -2,
	DA_EUNKNOWNMSG = -3,
	DA_ETIMEOUT = -4,
	DA_ENOMUTEXLOCK = -5,
	DA_ENOMAP = -6
} DA_RETCODE;

#define _M_STRUCTI(typ, name) ((typ *)(_DBIOS_DM_I_mmapped_base + (unsigned long)name))
#define _M_STRUCTF(typ, name) ((typ *)(_DBIOS_DM_F_mmapped_base + (unsigned long)name))

#endif
