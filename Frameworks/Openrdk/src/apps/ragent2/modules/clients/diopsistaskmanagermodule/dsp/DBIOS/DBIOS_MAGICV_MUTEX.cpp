/* *****************************************************************************
                SOFTWARE API FOR MUTEX
   ***************************************************************************** */

#include "DBIOS_macro.h"
#include "LIBv3.h"
#include "DBIOS_MAGICV_MUTEX.h"

/*!
\file DBIOS_MAGICV_MUTEX.c
\brief MAGICV MUTEX functions description
*/

//! \brief Lock a MUTEX
 unsigned int D940F_MAGICV_MUTEX_Lock(
	 unsigned int mutexNumber)			//!< Mutex index
{
	 long res;
	 _DBIOS_WriteMagicReg(D940_REG_MGCMUTEXCTL, (1u<<(mutexNumber&0xf)))
	//control 
	 _DBIOS_ReadMagicReg(D940_REG_MGCMUTEXCTL, res)
#ifdef __chess__
	 // if magic is owner and mutex control register is locked return 0 (ok)
	 if( (!(res&(1UL<<(mutexNumber&0xf)+16))) && (res&(1U<<(mutexNumber&0xf))) )
	    return 0;
#else
	 // if ARM is owner and mutex control register is locked return 0 (ok)
	 if( ((res&(1UL<<(mutexNumber&0xf)+16))) && (res&(1U<<(mutexNumber&0xf))) )
	    return 0;
#endif	 
	 return 1;//not ok 

 }

 //! \brief Unlock a MUTEX
 unsigned int D940F_MAGICV_MUTEX_Unlock(
	 unsigned int mutexNumber)		//!< Mutex index
	 {
	 int res;
	 _DBIOS_WriteMagicReg(D940_REG_MGCMUTEXCTL,(1uL<<(unsigned long)(((unsigned long)mutexNumber&0xful)+16ul)))
	 //control
	 _DBIOS_ReadMagicReg(D940_REG_MGCMUTEXCTL, res)
#ifdef __chess__
	 if(res&(1U<<(mutexNumber&0xf))) 
	     return res;  //stil locked, not ok
#else
	 if(!(res&(1U<<(mutexNumber&0xf)))) 
	     return res;  //stil locked, not ok
#endif	 
	 
	 return 0; //ok
 }

//! \brief Get the status of a MUTEX
unsigned int D940F_MAGICV_MUTEX_GetStatus(
	unsigned int mutexNumber)		//!< Mutex index
	{
	int res;
	_DBIOS_ReadMagicReg(D940_REG_MGCMUTEXCTL, res)
	return res&(1<<(mutexNumber&0xf));
	}

//! \brief Get the owner of a MUTEX
unsigned int D940F_MAGICV_MUTEX_GetOwner(
	unsigned int mutexNumber)	//!< Mutex index
	{  //return 0 if magic is owner
	long res;
	 _DBIOS_ReadMagicReg(D940_REG_MGCMUTEXCTL, res)
	return (res&(1L<<(mutexNumber&0xf)+16));
}

//! \brief Lock a MUTEX with timeout
unsigned int D940F_MAGICV_MUTEX_LockL(
	unsigned int mutexNumber, 		//!< Mutex index
	long timeout)			//!< timeout
	{
     long res;
	 long waiting_time=timeout;
	 do{
		 timeout--;
	 _DBIOS_WriteMagicReg(D940_REG_MGCMUTEXCTL, (1<<(mutexNumber&0xf)))
	//control 
	 _DBIOS_ReadMagicReg(D940_REG_MGCMUTEXCTL, res)}
	 // try to get lock while arm is owner or mutex is unlocked(maybe only the first check is necessary)
	 while( (((res&(1L<<(mutexNumber&0xf)+16))) || !(res&(1<<(mutexNumber&0xf))))&& timeout>0 );
	    
	 return 0;//OK
	
}
