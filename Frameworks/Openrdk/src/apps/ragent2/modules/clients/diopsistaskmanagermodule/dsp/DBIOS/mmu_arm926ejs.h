//*----------------------------------------------------------------------------
//*         ATMEL Microcontroller Software Support  -  ROUSSET  -
//*----------------------------------------------------------------------------
//* The software is delivered "AS IS" without warranty or condition of any
//* kind, either express, implied or statutory. This includes without
//* limitation any warranty or condition with respect to merchantability or
//* fitness for any particular purpose, or against the infringements of
//* intellectual property rights of others.
//*----------------------------------------------------------------------------
//* File Name           : mmu.h
//* Object              : Common MMU and CACHE operations
//* Creation            : ODi   Aug 22nd 02
//*
//*----------------------------------------------------------------------------
#ifndef MMU_H
#define MMU_H

#define SECTION_SBO           (1<<1 | 1<<4)  // Must always set in section descriptor

#define SECTION_NOCACH_NOBUFF 0
#define SECTION_BUFFERED      (1<<2)
#define SECTION_CACHABLE      (1<<3)
#define SECTION_CACH_AND_BUFF (3<<2)

#define SECTION_WRITE_THROUGH (1<<3)
#define SECTION_WRITE_BACK    (3<<2)

#define SECTION_DOMAIN0        ( 0<<5)
#define SECTION_DOMAIN1        ( 1<<5)
#define SECTION_DOMAIN2        ( 2<<5)
#define SECTION_DOMAIN3        ( 3<<5)
#define SECTION_DOMAIN4        ( 4<<5)
#define SECTION_DOMAIN5        ( 5<<5)
#define SECTION_DOMAIN6        ( 6<<5)
#define SECTION_DOMAIN7        ( 7<<5)
#define SECTION_DOMAIN8        ( 8<<5)
#define SECTION_DOMAIN9        ( 9<<5)
#define SECTION_DOMAIN10       (10<<5)
#define SECTION_DOMAIN11       (11<<5)
#define SECTION_DOMAIN12       (12<<5)
#define SECTION_DOMAIN13       (13<<5)
#define SECTION_DOMAIN14       (14<<5)
#define SECTION_DOMAIN15       (15<<5)

#define SECTION_AP_NO_ACCESS      (0<<10)  // This protection depends on bits S and R in the CP15 Control Register
#define SECTION_AP_SUPERV_ONLY    (1<<10)  // Only Supervisor can do Read Write Access
#define SECTION_AP_SUPERV_USER_RO (2<<10)  // User can do Read Only Accesses
#define SECTION_AP_SUPERV_USER_RW (2<<10)  // User can do Read Write Accesses

#define SECTION_SHIFT_FOR_ADR     20    // A section describes 1 MB area

#define DOMAIN_NO_ACCESS          0     // Any access generates a domain fault
#define DOMAIN_CLIENT             1     // Accesses are checked against the permission bits
#define DOMAIN_MANAGER            3     // Accesses are not checked

#define ITCM_ENABLE               1
#define DTCM_ENABLE               1

extern void D940F_CleanDCache(void); // Clean and invalidate D Cache

extern void D940F_ResetICache(void); // Reset I Cache (Should be run from a non cachable area)
extern void D940F_ResetDCache(void); // Reset D Cache (Should be run from a non cachable area)

extern void D940F_EnableMMU(void); // Enable MMU
extern void D940F_DisableMMU(void); // Disable MMU

extern void D940F_EnableICache(void); // Enable I Cache
extern void D940F_DisableICache(void); // Disable I Cache
extern void D940F_EnableDCache(void); // Enable D Cache
extern void D940F_DisableDCache(void); // Disable D Cache

extern void D940F_LockITLB(unsigned int address); // Lock one I TLB entry after entries previously locked
extern void D940F_LockICache(unsigned int startAddress, unsigned int size); // Write I TLB lockdown  (Should be run from a non cachable area)

extern unsigned int D940F_CheckTCM(void);  // Get TCM Status
extern void D940F_EnableITCM( unsigned int base_addr);  // Enable  I TCM
extern void D940F_DisableITCM(void);                    // Disable I TCM
extern void D940F_EnableDTCM(unsigned int base_addr);   // Enable  D TCM
extern void D940F_DisableDTCM(void);                    // Disable D TCM

#endif // MMU_H
