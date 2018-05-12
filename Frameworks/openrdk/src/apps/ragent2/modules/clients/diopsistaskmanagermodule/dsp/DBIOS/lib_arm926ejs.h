// Alpha Release of CP15 functions for ARM926
#ifndef __LIB_ARM926EJS__
#define __LIB_ARM926EJS__
#ifdef __GCC__
#define __inline inline
#endif
// ****************************************************************************
// CP15 Register 0
// 	Read: ID code | cache type
//	Write: Unpredictable

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_ReadIDCode
//* \brief Read ID code register
//*----------------------------------------------------------------------------
__inline unsigned int D940F_ARM_ReadIDCode();

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_ReadCacheType
//* \brief Read cache type
//*----------------------------------------------------------------------------
__inline unsigned int D940F_ARM_ReadCacheType();

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_ReadTCMStatus
//* \brief Read TCM Status
//*----------------------------------------------------------------------------
__inline unsigned int D940F_ARM_ReadTCMStatus();

// ****************************************************************************
// CP15 Register 1
// 	Read: Control
//	Write: Control

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_ReadControl
//* \brief Read Control register
//*----------------------------------------------------------------------------
#ifdef __GCC__
__inline unsigned int D940F_ARM_ReadControl() __attribute__((always_inline));
#else
__inline unsigned int D940F_ARM_ReadControl();
#endif
//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_WriteControl
//* \brief Write Control register
//*----------------------------------------------------------------------------
#ifdef __GCC__
//__inline void D940F_ARM_WriteControl(unsigned int ctl) __attribute__((always_inline));
#define  D940F_ARM_WriteControl(ctl) asm volatile("MCR p15, 0, %0, c1, c0, 0"::"r" (ctl));
#else
__inline void D940F_ARM_WriteControl(unsigned int ctl);
#endif
// ****************************************************************************
// CP15 Register 2
// 	Read: Translation table Base
//	Write: Translation table Base

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_ReadTTB
//* \brief Read Translation table base register
//*----------------------------------------------------------------------------
__inline unsigned int D940F_ARM_ReadTTB();

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_WriteTTB
//* \brief Write Translation table base  register
//*----------------------------------------------------------------------------
__inline void D940F_ARM_WriteTTB(unsigned int ttb);

// ****************************************************************************
// CP15 Register 3
// 	Read: Read domain access control
//	Write: Write domain access control

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_ReadDomain
//* \brief Read domain access control
//*----------------------------------------------------------------------------
__inline unsigned int D940F_ARM_ReadDomain();
//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_WriteDomain
//* \brief Write domain access control
//*----------------------------------------------------------------------------
__inline void D940F_ARM_WriteDomain(unsigned int domain);

// ****************************************************************************
// CP15 Register 5
// 	Read: Read Fault Status
//	Write: Write Fault Status

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_ReadDataFSR
//* \brief Read data FSR value
//*----------------------------------------------------------------------------
__inline unsigned int D940F_ARM_ReadDataFSR();

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_WriteDataFSR
//* \brief Write data FSR value
//*----------------------------------------------------------------------------
__inline void D940F_ARM_WriteDataFSR(unsigned int dataFSR);

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_ReadPrefetchFSR
//* \brief Read prefetch FSR value
//*----------------------------------------------------------------------------
__inline unsigned int D940F_ARM_ReadPrefetchFSR();

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_WritePrefetchFSR
//* \brief Write prefetch FSR value
//*----------------------------------------------------------------------------
__inline void D940F_ARM_WritePrefetchFSR(unsigned int prefetchFSR);
// ****************************************************************************
// CP15 Register 6
// 	Read: Read Fault Address
//	Write: Write Fault Address

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_ReadFAR
//* \brief Read FAR data
//*----------------------------------------------------------------------------
__inline unsigned int D940F_ARM_ReadFAR();

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_WriteFAR
//* \brief Write FAR data
//*----------------------------------------------------------------------------
__inline void D940F_ARM_WriteFAR(unsigned int dataFAR);

// ****************************************************************************
// CP15 Register 7
// 	Read: Unpredictable
//	Write: Cache operations

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_InvalidateIDCache
//* \brief Invalidate ICache and DCache
//*----------------------------------------------------------------------------
__inline void D940F_ARM_InvalidateIDCache();

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_InvalidateICache
//* \brief Invalidate ICache
//*----------------------------------------------------------------------------
__inline void D940F_ARM_InvalidateICache();

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_InvalidateICacheMVA
//* \brief Invalidate ICache single entry (using MVA);
//*----------------------------------------------------------------------------
__inline void D940F_ARM_InvalidateICacheMVA(unsigned int mva);

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_PrefetchICacheLine
//* \brief Prefetch ICache line (using MVA)
//*----------------------------------------------------------------------------
__inline void D940F_ARM_PrefetchICacheLine(unsigned int mva);

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_InvalidateDCache
//* \brief Invalidate DCache
//*----------------------------------------------------------------------------
__inline void D940F_ARM_InvalidateDCache();

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_InvalidateDCacheMVA
//* \brief Invalidate DCache single entry (using MVA)
//*----------------------------------------------------------------------------
__inline void D940F_ARM_InvalidateDCacheMVA(unsigned int mva);

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_CleanDCacheMVA
//* \brief Clean DCache single entry (using MVA)
//*----------------------------------------------------------------------------
__inline void D940F_ARM_CleanDCacheMVA(unsigned int mva);

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_CleanInvalidateDCacheMVA
//* \brief Clean and Invalidate DCache single entry (using MVA)
//*----------------------------------------------------------------------------
__inline void D940F_ARM_CleanInvalidateDCacheMVA(unsigned int mva);

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_CleanDCacheIDX
//* \brief Clean DCache single entry (using index)
//*----------------------------------------------------------------------------
__inline void D940F_ARM_CleanDCacheIDX(unsigned int index);

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_CleanInvalidateDCacheIDX
//* \brief Clean and Invalidate DCache single entry (using index)
//*----------------------------------------------------------------------------
__inline void D940F_ARM_CleanInvalidateDCacheIDX(unsigned int index);

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_DrainWriteBuffer
//* \brief Drain Write Buffer
//*----------------------------------------------------------------------------
__inline void D940F_ARM_DrainWriteBuffer();

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_WaitForInterrupt
//* \brief Wait for interrupt
//*----------------------------------------------------------------------------
__inline void D940F_ARM_WaitForInterrupt();
// ****************************************************************************
// CP15 Register 8
// 	Read: Unpredictable
//	Write: TLB operations

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_InvalidateIDTLB
//* \brief Invalidate TLB(s)
//*----------------------------------------------------------------------------
__inline void D940F_ARM_InvalidateIDTLB();

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_InvalidateITLB
//* \brief Invalidate I TLB
//*----------------------------------------------------------------------------
__inline void D940F_ARM_InvalidateITLB();

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_InvalidateITLBMVA
//* \brief Invalidate I TLB single entry (using MVA)
//*----------------------------------------------------------------------------
__inline void D940F_ARM_InvalidateITLBMVA(unsigned int mva);

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_InvalidateDTLB
//* \brief Invalidate D TLB
//*----------------------------------------------------------------------------
__inline void D940F_ARM_InvalidateDTLB();

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_InvalidateDTLBMVA
//* \brief Invalidate D TLB single entry (using MVA)
//*----------------------------------------------------------------------------
__inline void D940F_ARM_InvalidateDTLBMVA(unsigned int mva);

// ****************************************************************************
// CP15 Register 9
// 	Read: Cache lockdown
//	Write: Cache lockdown

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_ReadDCacheLockdown
//* \brief Read D Cache lockdown
//*----------------------------------------------------------------------------
__inline unsigned int D940F_ARM_ReadDCacheLockdown();

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_WriteDCacheLockdown
//* \brief Write D Cache lockdown
//*----------------------------------------------------------------------------
__inline void D940F_ARM_WriteDCacheLockdown(
					    unsigned int index);

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_ReadICacheLockdown
//* \brief Read I Cache lockdown
//*----------------------------------------------------------------------------
__inline unsigned int D940F_ARM_ReadICacheLockdown();

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_WriteICacheLockdown
//* \brief Write I Cache lockdown
//*----------------------------------------------------------------------------
__inline void D940F_ARM_WriteICacheLockdown(
					    unsigned int index);
//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_WriteITCMCfg
//* \brief Write Instruction TCM Configuration
//*----------------------------------------------------------------------------
__inline void D940F_ARM_WriteITCMCfg(unsigned int conf);

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_WriteDTCMCfg
//* \brief Write Data TCM Configuration
//*----------------------------------------------------------------------------
__inline void D940F_ARM_WriteDTCMCfg(unsigned int conf);

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_ReadDTCMCfg
//* \brief Read Data TCM Configuration
//*----------------------------------------------------------------------------
__inline unsigned int D940F_ARM_ReadDTCMCfg();

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_ReadITCMCfg
//* \brief Read Inst TCM Configuration
//*----------------------------------------------------------------------------
__inline unsigned int D940F_ARM_ReadITCMCfg();

// ****************************************************************************
// CP15 Register 10
// 	Read: TLB lockdown
//	Write: TLB lockdown

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_ReadDTLBLockdown
//* \brief Read D TLB lockdown
//*----------------------------------------------------------------------------
__inline unsigned int D940F_ARM_ReadDTLBLockdown();

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_WriteDTLBLockdown
//* \brief Write D TLB lockdown
//*----------------------------------------------------------------------------
__inline void D940F_ARM_WriteDTLBLockdown(
					  unsigned int lockdown);

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_ReadITLBLockdown
//* \brief Read I TLB lockdown
//*----------------------------------------------------------------------------
__inline unsigned int D940F_ARM_ReadITLBLockdown();

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_WriteITLBLockdown
//* \brief Write I TLB lockdown
//*----------------------------------------------------------------------------
__inline void D940F_ARM_WriteITLBLockdown(
					  unsigned int lockdown);

// ****************************************************************************
// CP15 Register 13
// 	Read: Read FCSE PID
//	Write: Write FCSE PID

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_ReadFCSEPID
//* \brief Read FCSE PID
//*----------------------------------------------------------------------------
__inline unsigned int D940F_ARM_ReadFCSEPID();

//*----------------------------------------------------------------------------
//* \fn    D940F_ARM_WriteFCSEPID
//* \brief Write FCSE PID
//*----------------------------------------------------------------------------
__inline void D940F_ARM_WriteFCSEPID(unsigned int pid);
#endif

