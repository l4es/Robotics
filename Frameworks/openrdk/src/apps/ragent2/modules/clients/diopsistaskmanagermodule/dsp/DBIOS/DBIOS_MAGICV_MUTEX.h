/* *****************************************************************************
                SOFTWARE API FOR MUTEX
   ***************************************************************************** */
#ifndef DBIOS_MAGICV_MUTEX_H
#define DBIOS_MAGICV_MUTEX_H


#include "LIBv3.h"
#include "DBIOS_macro.h"
#include "magicV_regs.h"

unsigned int D940F_MAGICV_MUTEX_Lock(unsigned int mutexNumber);
unsigned int D940F_MAGICV_MUTEX_Unlock(unsigned int mutexNumber);
unsigned int D940F_MAGICV_MUTEX_GetStatus(unsigned int mutexNumber);
unsigned int D940F_MAGICV_MUTEX_GetOwner(unsigned int mutexNumber);
unsigned int D940F_MAGICV_MUTEX_LockL(unsigned int mutexNumber,long timeout);

#endif

