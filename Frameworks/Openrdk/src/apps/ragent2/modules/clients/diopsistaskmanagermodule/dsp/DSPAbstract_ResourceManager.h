#ifndef DSPABSTRACTRM_H
#define DSPABSTRACTRM_H

#include <stdio.h>
#include <stdlib.h>
#include <sys/errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#ifndef __PC__
#include "mAgicV.h"
#endif
#include "DBIOS/DBIOS_macro.h"
#include "DBIOS/magicV_regs.h"
#include "DSPAbstract.h"
#include "DSPAbstract_ResourceManager.h"
#include "TaskLauncher.h"

void * DSPAbstractRM_AllocMemory(DA_int size);

DA_RETCODE DSPAbstractRM_FreeMemory(void *ptr);

DA_RETCODE DSPAbstractRM_Init();

#endif
