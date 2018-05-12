#ifndef __MAGICV_REGS_H__
#define __MAGICV_REGS_H__

#define D940_MAGIC_REG_BASE 0x680000
#define D940_MAGIC_RF_BASE  0x680400
#define D940_MAGIC_PM_BASE 0x600000
#define D940_MAGIC_DMI_BASE 0x620000
#define D940_MAGIC_DMF_BASE 0x640000
#define D940_MAGIC_DMD_BASE 0x660000

// ----- MGCSTAT ----
/** DESCRIPTION: status register */
/** OFFSET: 0x1 */
/** ACCESS: RO */
#define D940_REG_MGCSTAT  0x1 // status register
#define STAT_SLEEP (1<<0) //sleep mode
#define STAT_RUN (1<<1) //run mode
#define STAT_DEBUG (1<<2) //debug mode
#define STAT_START (1<<3) //start program, reset by run
#define STAT_STEP (1<<4) //step mode
#define STAT_EXCEPTION (1<<5) //fatal exception arised
#define STAT_ISR (1<<6) //interrupt mode
#define STAT_HALTED (1<<7) //halt instruction arosed, reset by start and cont
#define STAT_CROSS_TRIGGER (1<<8) //cross trigger on
#define STAT_STOP (1<<9) //program stopped, reset by start and cont
#define STAT_EXT_DBG_REQ (1<<10) //external debug request
#define STAT_BREAK (1<<11) //break point reached, reset by start and cont
#define STAT_WATCH (1<<12) //watch point reached, reset by start and cont
#define STAT_GBREAK (1<<13) //break request from external, reset by run
#define STAT_DBG_REQ_TO_ARM (1<<14) //debug request to ARM
#define STAT_DECOMP (1L<<15) //decompression enable
#define STAT_MMU (1L<<16) //mmu enable/disable (default)
#define STAT_SWPIPE (1L<<17) //software pipeline mode
#define STAT_PTY2BREAK (1L<<18) //parity is a break point (no exception)
#define STAT_PM_CHECK (1L<<19) //parity/edac check enable on PM
#define STAT_RPT (1L<<20) //single vliw repeating mode
#define STAT_LOCK_CTRL (1L<<21) //mgc control register locked
#define STAT_ALLEX_FATAL (1L<<22) //mgc control all exceptions are fatal
#define STAT_ALLEX_ISR (1L<<23) //mgc control all exceptions can be trapped
#define STAT_TICK_OVF (1L<<24) //mgc step counter overflow
#define D940_REGMASK_MGCSTAT (((1L<<25))-1)

void decode_mgcstat(char *ret, unsigned int val);

// ----- MGCCTRL ----
/** DESCRIPTION: control register */
/** OFFSET: 0x0 */
/** ACCESS: WO */
#define D940_REG_MGCCTRL  0x0 // control register
#define CTRL_SLEEP_ON (1<<0) //sleep mode ON
#define CTRL_SLEEP_OFF (1<<1) //sleep mode OFF
#define CTRL_CONT (1<<2) //continue an interrupted program
#define CTRL_STOP (1<<3) //stop a running program
#define CTRL_START (1<<4) //restart a program
#define CTRL_GBREAK_ON (1<<5) //stop from gui a running program
#define CTRL_GBREAK_OFF (1<<6) //continue from gui an interrupted program
#define CTRL_STEP_ON (1<<7) //step mode ON
#define CTRL_STEP_OFF (1<<8) //step mode OFF
#define CTRL_TRIGG_ON (1<<9) //trigger ARM/MAGIC debug signals ON
#define CTRL_TRIGG_OFF (1<<10) //trigger ARM/MAGIC debug signals OFF
#define CTRL_DECOMP_ON (1<<11) //decompressor ON
#define CTRL_DECOMP_OFF (1<<12) //decompressor OFF
#define CTRL_WATCH_ON (1<<13) //watchpoint ON
#define CTRL_WATCH_OFF (1<<14) //watchpoint OFF
#define CTRL_PTY2BREAK_ON (1L<<15) //pty error generate break ON
#define CTRL_PTY2BREAK_OFF (1L<<16) //pty error generate exception
#define CTRL_SWPIPE_ON (1L<<17) //software pipeline mode ON
#define CTRL_SWPIPE_OFF (1L<<18) //software pipeline mode OFF
#define CTRL_PMCHECK_ON (1L<<19) //program memory check ON
#define CTRL_PMCHECK_OFF (1L<<20) //program memory check OFF
#define CTRL_MMU_ON (1L<<21) //mmu enable
#define CTRL_MMU_OFF (1L<<22) //mmu disable (default)
#define CTRL_TICK_ON (1L<<23) //tick counter start
#define CTRL_TICK_OFF (1L<<24) //tick counter stop
#define CTRL_CTRLLOCK_ON (1L<<25) //lock control register
#define CTRL_CTRLLOCK_OFF (1L<<26) //unlock control register
#define CTRL_ALLFATAL_ON (1L<<27) //all fatal on
#define CTRL_ALLFATAL_OFF (1L<<28) //all fatal off
#define CTRL_FATAL2ISR_ON (1L<<29) //all fatal exceptions no block on
#define CTRL_FATAL2ISR_OFF (1L<<30) //all fatal exception no block off
#define D940_REGMASK_MGCCTRL (((1L<<31))-1)


// ----- MGCMASK ----
/** DESCRIPTION: mask register */
/** OFFSET: 0x2 */
/** ACCESS: RW */
#define D940_REG_MGCMASK  0x2 // mask register
#define GET_MASK(x) ((x>>0) & 268435455) // GET FIELD
#define SET_MASK(x) ((x & 268435455)<<0) // SET FIELD
#define D940_REGMASK_MGCMASK (((1L<<28))-1)


// ----- MGCEXC ----
/** DESCRIPTION: exception register */
/** OFFSET: 0x3 */
/** ACCESS: RC */
#define D940_REG_MGCEXC  0x3 // exception register
#define EX_BADIN (1<<0) //bad input operands
#define EX_BADOUT (1<<1) //bad output operands
#define EX_DIVZERO (1<<2) //division by zero
#define EX_ADDUNKN (1<<3) //fpu adder unknown operation
#define EX_AGU0_OVF (1<<4) //agu0 address overflow
#define EX_AGU0_PTY (1<<5) //agu0 bad address parity
#define EX_AGU0_ARITH_OVF (1<<6) //agu0 bad arithmetic overflow
#define EX_AGU1_OVF (1<<7) //agu1 address overflow
#define EX_AGU1_PTY (1<<8) //agu1 bad address parity
#define EX_AGU1_ARITH_OVF (1<<9) //agu0 bad arithmentic overflow
#define EX_SLV_ERR (1<<10) //slave error
#define EX_MST_ERR (1<<11) //master error
#define EX_SOFT_EX (1<<12) //software exception
#define EX_CTRL_DENIED (1<<13) //magic cannot access to ctrl register
#define EX_WRITE_READ_ERR0 (1<<14) //write read conflict on AGU0
#define EX_WRITE_READ_ERR1 (1L<<15) //write read conflict on AGU1
#define EX_RF7_CONFLICT (1L<<16) //write conflict on port RF7
#define EX_RF5_CONFLICT (1L<<17) //write conflict on port RF5
#define EX_ARF_CONFLICT (1L<<18) //write conflict on ARF REGS
#define EX_FLOWER_CONFLICT (1L<<19) //write conflict on FLOWER REGS
#define EX_DMA_CONFLICT (1L<<20) //write conflict on DMA REGS
#define EX_AGU0_UNKN (1L<<21) //agu0 unknown code
#define EX_AGU1_UNKN (1L<<22) //agu1 unknown code
#define EX_PTY_ERR (1L<<23) //parity error
#define EX_RTI_ERR (1L<<24) //invalid rti
#define EX_DOUBLE_WE_ERR (1L<<25) //magic double write same address
#define EX_PAGE_MISS (1L<<26) //mmu can't find a page to allocate
#define EX_MULUNKN (1L<<27) //fpu multiplier unknown operation
#define EX_DMA_BUSY (1L<<28) //new dma request over a pending chan
#define D940_REGMASK_MGCEXC (((1L<<29))-1)


// ----- MGCSTKY0 ----
/** DESCRIPTION: sticky register0 */
/** OFFSET: 0x4 */
/** ACCESS: RC */
#define D940_REG_MGCSTKY0  0x4 // sticky register0
#define GET_STKY0(x) ((x>>0) & 0) // GET FIELD
#define SET_STKY0(x) ((x & 0)<<0) // SET FIELD
#define D940_REGMASK_MGCSTKY0 0xffffffff


// ----- MGCSTKY1 ----
/** DESCRIPTION: sticky register1 */
/** OFFSET: 0x5 */
/** ACCESS: RC */
#define D940_REG_MGCSTKY1  0x5 // sticky register1
#define GET_STKY1(x) ((x>>0) & 0) // GET FIELD
#define SET_STKY1(x) ((x & 0)<<0) // SET FIELD
#define D940_REGMASK_MGCSTKY1 0xffffffff


// ----- MGCCONDITION ----
/** DESCRIPTION: Condition code register */
/** OFFSET: 0x6 */
/** ACCESS: RW */
#define D940_REG_MGCCONDITION  0x6 // Condition code register
#define CC_AGU0FL_C (1<<0) //agu0 condition flag
#define CC_AGU0FL_B (1<<1) //agu0 boundary flag
#define CC_AGU0FL_Z (1<<2) //agu0 zero flag
#define CC_AGU0FL_N (1<<3) //agu0 negative flag
#define CC_AGU1FL_C (1<<4) //agu1 condition flag
#define CC_AGU1FL_B (1<<5) //agu1 boundary flag
#define CC_AGU1FL_Z (1<<6) //agu1 zero flag
#define CC_AGU1FL_N (1<<7) //agu1 negative flag
#define CC_OPFL0 (1<<8) //operators condition flag I
#define CC_OPFL1 (1<<9) //operators condition flag Q
#define CC_TESTFL (1<<10) //test flag from write and test
#define CC_RPT_LOOP (1<<11) //repeat loop
#define CC_SIRQ2 (1<<12) //sirq2
#define CC_SIRQ3 (1<<13) //sirq3
#define D940_REGMASK_MGCCONDITION (((1<<14))-1)


// ----- MGCREPT ----
/** DESCRIPTION: Repeat loop address register */
/** OFFSET: 0x7 */
/** ACCESS: RW */
#define D940_REG_MGCREPT  0x7 // Repeat loop address register
#define GET_ENDLOOP(x) ((x>>0) & 65535) // GET FIELD
#define SET_ENDLOOP(x) ((x & 65535)<<0) // SET FIELD
#define GET_STARTLOOP(x) ((x>>16) & 65535) // GET FIELD
#define SET_STARTLOOP(x) ((x & 65535)<<16) // SET FIELD
#define D940_REGMASK_MGCREPT 0xffffffff


// ----- MGCPC ----
/** DESCRIPTION: program counter register */
/** OFFSET: 0x8 */
/** ACCESS: RW */
#define D940_REG_MGCPC  0x8 // program counter register
#define GET_PC(x) ((x>>0) & 65535) // GET FIELD
#define SET_PC(x) ((x & 65535)<<0) // SET FIELD
#define D940_REGMASK_MGCPC (((1L<<16))-1)


// ----- MGCSTKPC ----
/** DESCRIPTION: condition stack register */
/** OFFSET: 0x9 */
/** ACCESS: RW */
#define D940_REG_MGCSTKPC  0x9 // condition stack register
#define GET_STKPC(x) ((x>>0) & 65535) // GET FIELD
#define SET_STKPC(x) ((x & 65535)<<0) // SET FIELD
#define D940_REGMASK_MGCSTKPC (((1L<<16))-1)


// ----- MGCJMPREG ----
/** DESCRIPTION: branch register */
/** OFFSET: 0xA */
/** ACCESS: RW */
#define D940_REG_MGCJMPREG  0xA // branch register
#define GET_STKPC(x) ((x>>0) & 65535) // GET FIELD
#define SET_STKPC(x) ((x & 65535)<<0) // SET FIELD
#define D940_REGMASK_MGCJMPREG (((1L<<16))-1)


// ----- MGCPRED ----
/** DESCRIPTION: predication register */
/** OFFSET: 0xB */
/** ACCESS: RW */
#define D940_REG_MGCPRED  0xB // predication register
#define PRD0_I (1<<0) //predication register 0 I field
#define PRD0_Q (1<<1) //predication register 0 Q field
#define PRD0_G (1<<2) //predication register 0 G field
#define PRD1_I (1<<3) //predication register 1 I field
#define PRD1_Q (1<<4) //predication register 1 Q field
#define PRD1_G (1<<5) //predication register 1 G field
#define PRD2_I (1<<6) //predication register 2 I field
#define PRD2_Q (1<<7) //predication register 2 Q field
#define PRD2_G (1<<8) //predication register 2 G field
#define PRD3_I (1<<9) //predication register 3 I field
#define PRD3_Q (1<<10) //predication register 3 Q field
#define PRD3_G (1<<11) //predication register 3 G field
#define PRD_MODE0 (1<<12) //predication mode
#define PRD_MODE1 (1<<13) //predication mode
#define D940_REGMASK_MGCPRED (((1<<14))-1)


// ----- MGCSTEP ----
/** DESCRIPTION: steps register */
/** OFFSET: 0xC */
/** ACCESS: RW */
#define D940_REG_MGCSTEP  0xC // steps register
#define GET_STEP(x) ((x>>0) & 0) // GET FIELD
#define SET_STEP(x) ((x & 0)<<0) // SET FIELD
#define D940_REGMASK_MGCSTEP 0xffffffff


// ----- MGCWATCH ----
/** DESCRIPTION: watch register */
/** OFFSET: 0xD */
/** ACCESS: RW */
#define D940_REG_MGCWATCH  0xD // watch register
#define GET_WATCH(x) ((x>>0) & 65535) // GET FIELD
#define SET_WATCH(x) ((x & 65535)<<0) // SET FIELD
#define D940_REGMASK_MGCWATCH (((1L<<16))-1)


// ----- MGCMUTEXCTL ----
/** DESCRIPTION: mutex control register */
/** OFFSET: 0xE */
/** ACCESS: WO */
#define D940_REG_MGCMUTEXCTL  0xE // mutex control register
#define MTX_LOCK0 (1<<0) //lock mutex 0 if free
#define MTX_LOCK1 (1<<1) //lock mutex 1 if free
#define MTX_LOCK2 (1<<2) //lock mutex 2 if free
#define MTX_LOCK3 (1<<3) //lock mutex 3 if free
#define MTX_LOCK4 (1<<4) //lock mutex 4 if free
#define MTX_LOCK5 (1<<5) //lock mutex 5 if free
#define MTX_LOCK6 (1<<6) //lock mutex 6 if free
#define MTX_LOCK7 (1<<7) //lock mutex 7 if free
#define MTX_LOCK8 (1<<8) //lock mutex 8 if free
#define MTX_LOCK9 (1<<9) //lock mutex 9 if free
#define MTX_LOCK10 (1<<10) //lock mutex 10 if free
#define MTX_LOCK11 (1<<11) //lock mutex 11 if free
#define MTX_LOCK12 (1<<12) //lock mutex 12 if free
#define MTX_LOCK13 (1<<13) //lock mutex 13 if free
#define MTX_LOCK14 (1<<14) //lock mutex 14 if free
#define MTX_LOCK15 (1L<<15) //lock mutex 15 if free
#define MTX_UNLOCK0 (1L<<16) //unlock mutex 0 if owner
#define MTX_UNLOCK1 (1L<<17) //unlock mutex 1 if owner
#define MTX_UNLOCK2 (1L<<18) //unlock mutex 2 if owner
#define MTX_UNLOCK3 (1L<<19) //unlock mutex 3 if owner
#define MTX_UNLOCK4 (1L<<20) //unlock mutex 4 if owner
#define MTX_UNLOCK5 (1L<<21) //unlock mutex 5 if owner
#define MTX_UNLOCK6 (1L<<22) //unlock mutex 6 if owner
#define MTX_UNLOCK7 (1L<<23) //unlock mutex 7 if owner
#define MTX_UNLOCK8 (1L<<24) //unlock mutex 8 if owner
#define MTX_UNLOCK9 (1L<<25) //unlock mutex 9 if owner
#define MTX_UNLOCK10 (1L<<26) //unlock mutex 10 if owner
#define MTX_UNLOCK11 (1L<<27) //unlock mutex 11 if owner
#define MTX_UNLOCK12 (1L<<28) //unlock mutex 12 if owner
#define MTX_UNLOCK13 (1L<<29) //unlock mutex 13 if owner
#define MTX_UNLOCK14 (1L<<30) //unlock mutex 14 if owner
#define MTX_UNLOCK15 (1L<<31) //unlock mutex 15 if owner
#define D940_REGMASK_MGCMUTEXCTL 0xffffffff


// ----- MGCMUTEXSTAT ----
/** DESCRIPTION: mutex status register */
/** OFFSET: 0xE */
/** ACCESS: RO */
#define D940_REG_MGCMUTEXSTAT  0xE // mutex status register
#define MTX_LOCKED0 (1<<0) //mutex 0 lock status
#define MTX_LOCKED1 (1<<1) //mutex 1 lock status
#define MTX_LOCKED2 (1<<2) //mutex 2 lock status
#define MTX_LOCKED3 (1<<3) //mutex 3 lock status
#define MTX_LOCKED4 (1<<4) //mutex 4 lock status
#define MTX_LOCKED5 (1<<5) //mutex 5 lock status
#define MTX_LOCKED6 (1<<6) //mutex 6 lock status
#define MTX_LOCKED7 (1<<7) //mutex 7 lock status
#define MTX_LOCKED8 (1<<8) //mutex 8 lock status
#define MTX_LOCKED9 (1<<9) //mutex 9 lock status
#define MTX_LOCKED10 (1<<10) //mutex 10 lock status
#define MTX_LOCKED11 (1<<11) //mutex 11 lock status
#define MTX_LOCKED12 (1<<12) //mutex 12 lock status
#define MTX_LOCKED13 (1<<13) //mutex 13 lock status
#define MTX_LOCKED14 (1<<14) //mutex 14 lock status
#define MTX_LOCKED15 (1L<<15) //mutex 15 lock status
#define MTX_OWNER0 (1L<<16) //mutex owner 1=ARM 0=MAGIC
#define MTX_OWNER1 (1L<<17) //mutex owner 1=ARM 0=MAGIC
#define MTX_OWNER2 (1L<<18) //mutex owner 1=ARM 0=MAGIC
#define MTX_OWNER3 (1L<<19) //mutex owner 1=ARM 0=MAGIC
#define MTX_OWNER4 (1L<<20) //mutex owner 1=ARM 0=MAGIC
#define MTX_OWNER5 (1L<<21) //mutex owner 1=ARM 0=MAGIC
#define MTX_OWNER6 (1L<<22) //mutex owner 1=ARM 0=MAGIC
#define MTX_OWNER7 (1L<<23) //mutex owner 1=ARM 0=MAGIC
#define MTX_OWNER8 (1L<<24) //mutex owner 1=ARM 0=MAGIC
#define MTX_OWNER9 (1L<<25) //mutex owner 1=ARM 0=MAGIC
#define MTX_OWNER10 (1L<<26) //mutex owner 1=ARM 0=MAGIC
#define MTX_OWNER11 (1L<<27) //mutex owner 1=ARM 0=MAGIC
#define MTX_OWNER12 (1L<<28) //mutex owner 1=ARM 0=MAGIC
#define MTX_OWNER13 (1L<<29) //mutex owner 1=ARM 0=MAGIC
#define MTX_OWNER14 (1L<<30) //mutex owner 1=ARM 0=MAGIC
#define MTX_OWNER15 (1L<<31) //mutex owner 1=ARM 0=MAGIC
#define D940_REGMASK_MGCMUTEXSTAT 0xffffffff


void decode_mgcmutexstat(char*ret,unsigned val);

// ----- MGCLOOPCNT ----
/** DESCRIPTION: loop counter register */
/** OFFSET: 0xF */
/** ACCESS: RW */
#define D940_REG_MGCLOOPCNT  0xF // loop counter register
#define GET_COUNTER(x) ((x>>0) & 0) // GET FIELD
#define SET_COUNTER(x) ((x & 0)<<0) // SET FIELD
#define D940_REGMASK_MGCLOOPCNT 0xffffffff


// ----- MGCLOOPLEN ----
/** DESCRIPTION: loop len register */
/** OFFSET: 0x10 */
/** ACCESS: RW */
#define D940_REG_MGCLOOPLEN  0x10 // loop len register
#define GET_COUNTER(x) ((x>>0) & 0) // GET FIELD
#define SET_COUNTER(x) ((x & 0)<<0) // SET FIELD
#define D940_REGMASK_MGCLOOPLEN 0xffffffff


// ----- MGCRFINC ----
/** DESCRIPTION: Rf address increment register */
/** OFFSET: 0x11 */
/** ACCESS: RW */
#define D940_REG_MGCRFINC  0x11 // Rf address increment register
#define GET_RFINC(x) ((x>>0) & 0) // GET FIELD
#define SET_RFINC(x) ((x & 0)<<0) // SET FIELD
#define D940_REGMASK_MGCRFINC 0xffffffff


// ----- MGCRFMODM ----
/** DESCRIPTION: multiplier Rf address mask */
/** OFFSET: 0x12 */
/** ACCESS: RW */
#define D940_REG_MGCRFMODM  0x12 // multiplier Rf address mask
#define GET_RFMASKM(x) ((x>>0) & 0) // GET FIELD
#define SET_RFMASKM(x) ((x & 0)<<0) // SET FIELD
#define D940_REGMASK_MGCRFMODM 0xffffffff


// ----- MGCRFMODA ----
/** DESCRIPTION: adder Rf address mask */
/** OFFSET: 0x13 */
/** ACCESS: RW */
#define D940_REG_MGCRFMODA  0x13 // adder Rf address mask
#define GET_RFMASKA(x) ((x>>0) & 0) // GET FIELD
#define SET_RFMASKA(x) ((x & 0)<<0) // SET FIELD
#define D940_REGMASK_MGCRFMODA 0xffffffff


// ----- MGCDMAEXTADD ----
/** DESCRIPTION: Dma External Address */
/** OFFSET: 0x64 */
/** ACCESS: RW */
#define D940_REG_MGCDMAEXTADD  0x64 // Dma External Address
#define GET_EXTADDRESS(x) ((x>>0) & 268435455) // GET FIELD
#define SET_EXTADDRESS(x) ((x & 268435455)<<0) // SET FIELD
#define D940_REGMASK_MGCDMAEXTADD (((1L<<28))-1)


// ----- MGCDMAEXTCIRCLEN ----
/** DESCRIPTION: Dma External Circular Len */
/** OFFSET: 0x65 */
/** ACCESS: RW */
#define D940_REG_MGCDMAEXTCIRCLEN  0x65 // Dma External Circular Len
#define GET_EXTCIRCLEN(x) ((x>>0) & 16777215) // GET FIELD
#define SET_EXTCIRCLEN(x) ((x & 16777215)<<0) // SET FIELD
#define D940_REGMASK_MGCDMAEXTCIRCLEN (((1L<<24))-1)


// ----- MGCDMAEXTMOD ----
/** DESCRIPTION: Dma External Modifier */
/** OFFSET: 0x66 */
/** ACCESS: RW */
#define D940_REG_MGCDMAEXTMOD  0x66 // Dma External Modifier
#define GET_EXTMODIFIER(x) ((x>>0) & 65535) // GET FIELD
#define SET_EXTMODIFIER(x) ((x & 65535)<<0) // SET FIELD
#define D940_REGMASK_MGCDMAEXTMOD (((1L<<16))-1)


// ----- MGCDMAINTADD ----
/** DESCRIPTION: Dma Internal Address */
/** OFFSET: 0x67 */
/** ACCESS: RW */
#define D940_REG_MGCDMAINTADD  0x67 // Dma Internal Address
#define GET_INTADDRESS(x) ((x>>0) & 65535) // GET FIELD
#define SET_INTADDRESS(x) ((x & 65535)<<0) // SET FIELD
#define D940_REGMASK_MGCDMAINTADD (((1L<<16))-1)


// ----- MGCDMAINTCIRCLEN ----
/** DESCRIPTION: Dma Internal Circular Len */
/** OFFSET: 0x68 */
/** ACCESS: RW */
#define D940_REG_MGCDMAINTCIRCLEN  0x68 // Dma Internal Circular Len
#define GET_INTCIRCLEN(x) ((x>>0) & 65535) // GET FIELD
#define SET_INTCIRCLEN(x) ((x & 65535)<<0) // SET FIELD
#define D940_REGMASK_MGCDMAINTCIRCLEN (((1L<<16))-1)


// ----- MGCDMAINTMOD ----
/** DESCRIPTION: Dma Internal Modifier */
/** OFFSET: 0x69 */
/** ACCESS: RW */
#define D940_REG_MGCDMAINTMOD  0x69 // Dma Internal Modifier
#define GET_INTMODIFIER(x) ((x>>0) & 65535) // GET FIELD
#define SET_INTMODIFIER(x) ((x & 65535)<<0) // SET FIELD
#define D940_REGMASK_MGCDMAINTMOD (((1L<<16))-1)


// ----- MGCDMALEN ----
/** DESCRIPTION: Dma Burst Len */
/** OFFSET: 0x6A */
/** ACCESS: RW */
#define D940_REG_MGCDMALEN  0x6A // Dma Burst Len
#define GET_LEN(x) ((x>>0) & 65535) // GET FIELD
#define SET_LEN(x) ((x & 65535)<<0) // SET FIELD
#define D940_REGMASK_MGCDMALEN (((1L<<16))-1)


// ----- MGCDMASEG ----
/** DESCRIPTION: Dma Internal Segment Addresses */
/** OFFSET: 0x6B */
/** ACCESS: RW */
#define D940_REG_MGCDMASEG  0x6B // Dma Internal Segment Addresses
#define GET_INTSEG(x) ((x>>0) & 15) // GET FIELD internal segment 0:PM,1:DMI,2:DMF,3:DMD
#define SET_INTSEG(x) ((x & 15)<<0) // SET FIELD internal segment 0:PM,1:DMI,2:DMF,3:DMD
#define D940_REGMASK_MGCDMASEG (((1<<4))-1)


// ----- MGCDMASTAT ----
/** DESCRIPTION: dma status register */
/** OFFSET: 0x6F */
/** ACCESS: RO */
#define D940_REG_MGCDMASTAT  0x6F // dma status register
#define DMA_BUSY0 (1<<0) //dma channel0 busy or pending
#define DMA_BUSY1 (1<<1) //dma channel1 busy or pending
#define DMA_BUSY2 (1<<2) //dma channel2 busy or pending
#define DMA_BUSY3 (1<<3) //dma channel3 busy or pending
#define DMA_RUN0 (1<<4) //dma channel0 running
#define DMA_RUN1 (1<<5) //dma channel1 running
#define DMA_RUN2 (1<<6) //dma channel2 running
#define DMA_RUN3 (1<<7) //dma channel3 running
#define DMA_BUSY (1<<8) //dma busy
#define DMA_RUN (1<<9) //dma run
#define GET_DMA_CURR_CHAN(x) ((x>>10) & 3) // GET FIELD current channel specification
#define SET_DMA_CURR_CHAN(x) ((x & 3)<<10) // SET FIELD current channel specification
#define DMA_CONTINUE_INTERNAL (1<<12) //dma don't load internal parameters
#define DMA_CONTINUE_EXTERNAL (1<<13) //dma don't load external parameters
#define DMA_ERR (1<<14) //dma error
#define DMA_ABORTED (1L<<15) //dma abort command
#define DMA_EOT0 (1L<<16) //dma eot channel0
#define DMA_EOT1 (1L<<17) //dma eot channel1
#define DMA_EOT2 (1L<<18) //dma eot channel2
#define DMA_EOT3 (1L<<19) //dma eot channel3
#define D940_REGMASK_MGCDMASTAT (((1L<<20))-1)


void decode_mgcdmastat(char*ret,unsigned val);

// ----- MGCDMACTRL ----
/** DESCRIPTION: dma control register */
/** OFFSET: 0x6F */
/** ACCESS: WO */
#define D940_REG_MGCDMACTRL  0x6F // dma control register
#define DMA_WRITE (1<<0) //write
#define DMA_READ (1<<1) //read
#define DMA_ABORT (1<<2) //abort current dma
#define DMA_CLREOT (1<<3) //clear sticky eot
#define DMA_INT_CONTINUE_OFF (1<<4) //reload internal dma parameters
#define DMA_INT_CONTINUE_ON (1<<5) //preserve internal dma parameters
#define DMA_EXT_CONTINUE_OFF (1<<6) //reload external dma parameters
#define DMA_EXT_CONTINUE_ON (1<<7) //preserve external dma parameters
#define DMA_CHANGE_CHAN (1<<8) //change channel
#define GET_DMA_CHAN(x) ((x>>9) & 3) // GET FIELD channel specification
#define SET_DMA_CHAN(x) ((x & 3)<<9) // SET FIELD channel specification
#define DMA_ABORT_ALL (1<<11) //dma abort all
#define DMA_TEST_BUSY0 (1<<12) //test dma busy0
#define DMA_TEST_BUSY1 (1<<13) //test dma busy1
#define DMA_TEST_BUSY2 (1<<14) //test dma busy2
#define DMA_TEST_BUSY3 (1L<<15) //test dma busy3
#define DMA_TEST_RUN0 (1L<<16) //test dma run0
#define DMA_TEST_RUN1 (1L<<17) //test dma run1
#define DMA_TEST_RUN2 (1L<<18) //test dma run2
#define DMA_TEST_RUN3 (1L<<19) //test dma run3
#define DMA_TEST_BUSY (1L<<20) //test dma busy
#define DMA_TEST_RUN (1L<<21) //test dma run
#define DMA_LOCK (1L<<22) //lock AHB master
#define DMA_UNLOCK (1L<<23) //unlock AHB maste
#define DMA_TEST_ERR (1L<<26) //test dma err
#define DMA_TEST_ABORTED (1L<<27) //test dma aborted
#define DMA_TEST_EOT0 (1L<<28) //dma eot channel0
#define DMA_TEST_EOT1 (1L<<29) //dma eot channel1
#define DMA_TEST_EOT2 (1L<<30) //dma eot channel2
#define DMA_TEST_EOT3 (1L<<31) //dma eot channel3
#define D940_REGMASK_MGCDMACTRL 0xffffffff


// ----- MGCSTKIQ ----
/** DESCRIPTION: stack register STKIQ */
/** OFFSET: 0x70 */
/** ACCESS: RW */
#define D940_REG_MGCSTKIQ  0x70 // stack register STKIQ
#define GET_STKI(x) ((x>>0) & 65535) // GET FIELD stack i
#define SET_STKI(x) ((x & 65535)<<0) // SET FIELD stack i
#define GET_STKQ(x) ((x>>16) & 65535) // GET FIELD stack q
#define SET_STKQ(x) ((x & 65535)<<16) // SET FIELD stack q
#define D940_REGMASK_MGCSTKIQ 0xffffffff


// ----- MMUVIRT0 ----
/** DESCRIPTION: MMU virtual page register  */
/** OFFSET: 0x80 */
/** ACCESS: RW */
#define D940_REG_MMUVIRT0  0x80 // MMU virtual page register
#define MMU_VIRT_MAP (1<<0) //mapped page
#define MMU_VIRT_FIX (1<<1) //fixed page
#define MMU_VIRT_REF (1<<2) //recently referenced page
#define GET_PHYSPAGE(x) ((x>>3) & 7) // GET FIELD physical page
#define SET_PHYSPAGE(x) ((x & 7)<<3) // SET FIELD physical page
#define D940_REGMASK_MMUVIRT0 (((1<<6))-1)


// ----- MMUMAPPEDVIRT0 ----
/** DESCRIPTION: MMU virtual mapped page regs  */
/** OFFSET: 0xC0 */
/** ACCESS: RW */
#define D940_REG_MMUMAPPEDVIRT0  0xC0 // MMU virtual mapped page regs
#define GET_MMUMAPPEDVIRT0(x) ((x>>0) & 63) // GET FIELD
#define SET_MMUMAPPEDVIRT0(x) ((x & 63)<<0) // SET FIELD
#define D940_REGMASK_MMUMAPPEDVIRT0 (((1<<6))-1)


// ----- MMUFAULTREG ----
/** DESCRIPTION: MMU virtual fault address  */
/** OFFSET: 0xC8 */
/** ACCESS: RW */
#define D940_REG_MMUFAULTREG  0xC8 // MMU virtual fault address
#define GET_FAULTVIRTUALADDR(x) ((x>>0) & 65535) // GET FIELD
#define SET_FAULTVIRTUALADDR(x) ((x & 65535)<<0) // SET FIELD
#define D940_REGMASK_MMUFAULTREG (((1L<<16))-1)


// ----- MMUEXTADDREG ----
/** DESCRIPTION: MMU external address  */
/** OFFSET: 0xC9 */
/** ACCESS: RW */
#define D940_REG_MMUEXTADDREG  0xC9 // MMU external address
#define GET_EXTADDR(x) ((x>>0) & 255) // GET FIELD section to be used to place magic program external memory
#define SET_EXTADDR(x) ((x & 255)<<0) // SET FIELD section to be used to place magic program external memory
#define D940_REGMASK_MMUEXTADDREG (((1<<8))-1)


// ----- MMUCTLREG ----
/** DESCRIPTION: MMU control register  */
/** OFFSET: 0xCA */
/** ACCESS: WO */
#define D940_REG_MMUCTLREG  0xCA // MMU control register
#define MMU_CTL_SWRST (1<<0) //software reset
#define MMU_CTL_MASK_PREFETCH (1<<1) //mask program prefetch
#define MMU_CTL_UMASK_PREFETCH (1<<2) //unmask program prefetch
#define D940_REGMASK_MMUCTLREG (((1<<3))-1)


// ----- MMUSTATREG ----
/** DESCRIPTION: MMU control register  */
/** OFFSET: 0xCA */
/** ACCESS: RO */
#define D940_REG_MMUSTATREG  0xCA // MMU control register
#define MMU_BUSY (1<<0) //mmu busy reloading page
#define MMU_WAIT_ENDTX (1<<1) //mmu waiting master eot
#define MMU_WAIT_MST (1<<2) //mmu waiting master free
#define MMU_MASK_PREFETCH (1<<3) //mask prefetch micro instructions
#define MMU_DOUBLE_FAULT (1<<4) //two consecutive page faults
#define MMU_PAGE_FAULT (1<<5) //page fault
#define MMU_SUBPAGE_FAULT (1<<6) //chunk fault
#define MMU_PREFETCH_PAGE (1<<7) //prefetch page fault
#define MMU_PREFETCH_SUBPAGE (1<<8) //prefetch chunk fault
#define MMU_DOUBLE_CMD (1<<9) //warning mmu command during prefetch
#define MMU_FIXED_PAGE (1<<10) //page is temporary-fixed
#define D940_REGMASK_MMUSTATREG (((1<<11))-1)


// ----- MMUMISSCNT ----
/** DESCRIPTION: MMU miss counter register  */
/** OFFSET: 0xCB */
/** ACCESS: RW */
#define D940_REG_MMUMISSCNT  0xCB // MMU miss counter register
#define GET_MISSCNT(x) ((x>>0) & 0) // GET FIELD miss counter register
#define SET_MISSCNT(x) ((x & 0)<<0) // SET FIELD miss counter register
#define D940_REGMASK_MMUMISSCNT 0xffffffff


// ----- MGCINTSVR0 ----
/** DESCRIPTION: interrupt service routine0 */
/** OFFSET: 0x71 */
/** ACCESS: RW */
#define D940_REG_MGCINTSVR0  0x71 // interrupt service routine0
#define GET_SVR0(x) ((x>>0) & 65535) // GET FIELD
#define SET_SVR0(x) ((x & 65535)<<0) // SET FIELD
#define D940_REGMASK_MGCINTSVR0 (((1L<<16))-1)


// ----- MGCINTSVR1 ----
/** DESCRIPTION: interrupt service routine1 */
/** OFFSET: 0x72 */
/** ACCESS: RW */
#define D940_REG_MGCINTSVR1  0x72 // interrupt service routine1
#define GET_SVR1(x) ((x>>0) & 65535) // GET FIELD
#define SET_SVR1(x) ((x & 65535)<<0) // SET FIELD
#define D940_REGMASK_MGCINTSVR1 (((1L<<16))-1)


// ----- MGCINTSVR2 ----
/** DESCRIPTION: interrupt service routine2 */
/** OFFSET: 0x73 */
/** ACCESS: RW */
#define D940_REG_MGCINTSVR2  0x73 // interrupt service routine2
#define GET_SVR2(x) ((x>>0) & 65535) // GET FIELD
#define SET_SVR2(x) ((x & 65535)<<0) // SET FIELD
#define D940_REGMASK_MGCINTSVR2 (((1L<<16))-1)


// ----- MGCINTSVR3 ----
/** DESCRIPTION: interrupt service routine3 */
/** OFFSET: 0x74 */
/** ACCESS: RW */
#define D940_REG_MGCINTSVR3  0x74 // interrupt service routine3
#define GET_SVR3(x) ((x>>0) & 65535) // GET FIELD
#define SET_SVR3(x) ((x & 65535)<<0) // SET FIELD
#define D940_REGMASK_MGCINTSVR3 (((1L<<16))-1)


// ----- MGCINTSVR4 ----
/** DESCRIPTION: interrupt service routine4 */
/** OFFSET: 0x75 */
/** ACCESS: RW */
#define D940_REG_MGCINTSVR4  0x75 // interrupt service routine4
#define GET_SVR4(x) ((x>>0) & 65535) // GET FIELD
#define SET_SVR4(x) ((x & 65535)<<0) // SET FIELD
#define D940_REGMASK_MGCINTSVR4 (((1L<<16))-1)


// ----- MGCINTSVR5 ----
/** DESCRIPTION: interrupt service routine5 */
/** OFFSET: 0x76 */
/** ACCESS: RW */
#define D940_REG_MGCINTSVR5  0x76 // interrupt service routine5
#define GET_SVR5(x) ((x>>0) & 65535) // GET FIELD
#define SET_SVR5(x) ((x & 65535)<<0) // SET FIELD
#define D940_REGMASK_MGCINTSVR5 (((1L<<16))-1)


// ----- MGCINTSVR6 ----
/** DESCRIPTION: interrupt service routine6 */
/** OFFSET: 0x77 */
/** ACCESS: RW */
#define D940_REG_MGCINTSVR6  0x77 // interrupt service routine6
#define GET_SVR6(x) ((x>>0) & 65535) // GET FIELD
#define SET_SVR6(x) ((x & 65535)<<0) // SET FIELD
#define D940_REGMASK_MGCINTSVR6 (((1L<<16))-1)


// ----- MGCINTSVR7 ----
/** DESCRIPTION: interrupt service routine7 */
/** OFFSET: 0x78 */
/** ACCESS: RW */
#define D940_REG_MGCINTSVR7  0x78 // interrupt service routine7
#define GET_SVR7(x) ((x>>0) & 65535) // GET FIELD
#define SET_SVR7(x) ((x & 65535)<<0) // SET FIELD
#define D940_REGMASK_MGCINTSVR7 (((1L<<16))-1)


// ----- MGCINTMASK ----
/** DESCRIPTION: interrupt mask register */
/** OFFSET: 0x79 */
/** ACCESS: RW */
#define D940_REG_MGCINTMASK  0x79 // interrupt mask register
#define INTMSK0 (1<<0) //interrupt mask int 0 (group0)
#define INTMSK1 (1<<1) //interrupt mask int 1 (group1)
#define INTMSK2 (1<<2) //interrupt mask int 2
#define INTMSK3 (1<<3) //interrupt mask int 3
#define INTMSK4 (1<<4) //interrupt mask int 4
#define INTMSK5 (1<<5) //interrupt mask int 5
#define INTMSK6 (1<<6) //interrupt mask int 6
#define INTMSK7 (1<<7) //interrupt mask int 7
#define INTSHMSK00 (1<<8) //interrupt mask shared 0 group0
#define INTSHMSK01 (1<<9) //interrupt mask shared 1 group0
#define INTSHMSK02 (1<<10) //interrupt mask shared 2 group0
#define INTSHMSK03 (1<<11) //interrupt mask shared 3 group0
#define INTSHMSK10 (1<<12) //interrupt mask shared 0 group1
#define INTSHMSK11 (1<<13) //interrupt mask shared 1 group1
#define INTSHMSK12 (1<<14) //interrupt mask shared 2 group1
#define INTSHMSK13 (1L<<15) //interrupt mask shared 3 group1
#define D940_REGMASK_MGCINTMASK (((1L<<16))-1)


// ----- MGCINTSTAT ----
/** DESCRIPTION: interrupt status register */
/** OFFSET: 0x7A */
/** ACCESS: RO */
#define D940_REG_MGCINTSTAT  0x7A // interrupt status register
#define INTP0 (1<<0) //interrupt 0 pending
#define INTP1 (1<<1) //interrupt 1 pending
#define INTP2 (1<<2) //interrupt 2 pending
#define INTP3 (1<<3) //interrupt 3 pending
#define INTP4 (1<<4) //interrupt 4 pending
#define INTP5 (1<<5) //interrupt 5 pending
#define INTP6 (1<<6) //interrupt 6 pending
#define INTP7 (1<<7) //interrupt 7 pending
#define INTEN0 (1<<8) //interrupt 0 enabled
#define INTEN1 (1<<9) //interrupt 1 enabled
#define INTEN2 (1<<10) //interrupt 2 enabled
#define INTEN3 (1<<11) //interrupt 3 enabled
#define INTEN4 (1<<12) //interrupt 4 enabled
#define INTEN5 (1<<13) //interrupt 5 enabled
#define INTEN6 (1<<14) //interrupt 6 enabled
#define INTEN7 (1L<<15) //interrupt 7 enabled
#define D940_REGMASK_MGCINTSTAT (((1L<<16))-1)


void decode_mgcintstat(char*ret,unsigned val);

// ----- MGCINTGSTAT ----
/** DESCRIPTION: interrupt group status register */
/** OFFSET: 0x7B */
/** ACCESS: RO */
#define D940_REG_MGCINTGSTAT  0x7B // interrupt group status register
#define SHDG00 (1<<0) //interrupt status of line 0 of group0
#define SHDG01 (1<<1) //interrupt status of line 1 of group0
#define SHDG02 (1<<2) //interrupt status of line 2 of group0
#define SHDG03 (1<<3) //interrupt status of line 3 of group0
#define CSHDG00 (1<<4) //interrupt current status of line 0 of group0
#define CSHDG01 (1<<5) //interrupt current status of line 1 of group0
#define CSHDG02 (1<<6) //interrupt current status of line 2 of group0
#define CSHDG03 (1<<7) //interrupt current status of line 3 of group0
#define SHDG10 (1<<8) //interrupt status of line 0 of group1
#define SHDG11 (1<<9) //interrupt status of line 1 of group1
#define SHDG12 (1<<10) //interrupt status of line 2 of group1
#define SHDG13 (1<<11) //interrupt status of line 3 of group1
#define CSHDG10 (1<<12) //interrupt current status of line 0 of group1
#define CSHDG11 (1<<13) //interrupt current status of line 1 of group1
#define CSHDG12 (1<<14) //interrupt current status of line 2 of group1
#define CSHDG13 (1L<<15) //interrupt current status of line 3 of group1
#define D940_REGMASK_MGCINTGSTAT (((1L<<16))-1)


void decode_mgcintgstat(char*ret,unsigned val);

// ----- MGCINTCTRL ----
/** DESCRIPTION: interrupt control register */
/** OFFSET: 0x7C */
/** ACCESS: WO */
#define D940_REG_MGCINTCTRL  0x7C // interrupt control register
#define ENINT0 (1<<0) //interrupt 0 enable
#define DISINT0 (1<<1) //interrupt 0 disable
#define ENINT1 (1<<2) //interrupt 1 enable
#define DISINT1 (1<<3) //interrupt 1 disable
#define ENINT2 (1<<4) //interrupt 2 enable
#define DISINT2 (1<<5) //interrupt 2 disable
#define ENINT3 (1<<6) //interrupt 3 enable
#define DISINT3 (1<<7) //interrupt 3 disable
#define ENINT4 (1<<8) //interrupt 4 enable
#define DISINT4 (1<<9) //interrupt 4 disable
#define ENINT5 (1<<10) //interrupt 5 enable
#define DISINT5 (1<<11) //interrupt 5 disable
#define ENINT6 (1<<12) //interrupt 6 enable
#define DISINT6 (1<<13) //interrupt 6 disable
#define ENINT7 (1<<14) //interrupt 7 enable
#define DISINT7 (1L<<15) //interrupt 7 disable
#define D940_REGMASK_MGCINTCTRL (((1L<<16))-1)


// ----- MGCWKSTAT ----
/** DESCRIPTION: interrupt wake status register */
/** OFFSET: 0x7D */
/** ACCESS: RO */
#define D940_REG_MGCWKSTAT  0x7D // interrupt wake status register
#define WKST0 (1<<0) //wake on int 0 sharm0
#define WKST1 (1<<1) //wake on int 1 sharm1
#define WKST2 (1<<2) //wake on int 2 arm irq0
#define WKST3 (1<<3) //wake on int 3 arm irq1
#define WKST4 (1<<4) //wake on int 4 eot
#define WKST5 (1<<5) //wake on int 5 watch
#define WKST6 (1<<6) //wake on int 6 exception
#define WKST7 (1<<7) //wake on int 7 tick counter/SWI
#define WKEOT0 (1<<8) //wake on channel0 dma eot
#define WKEOT1 (1<<9) //wake on channel1 dma eot
#define WKEOT2 (1<<10) //wake on channel2 dma eot
#define WKEOT3 (1<<11) //wake on channel3 dma eot (mmu)
#define D940_REGMASK_MGCWKSTAT (((1<<12))-1)


void decode_mgcwkstat(char*ret,unsigned val);

// ----- MGCWKCTRL ----
/** DESCRIPTION: interrupt wake control register */
/** OFFSET: 0x7D */
/** ACCESS: WO */
#define D940_REG_MGCWKCTRL  0x7D // interrupt wake control register
#define ENWK0 (1<<0) //wake 0 enable
#define DISWK0 (1<<1) //wake 0 disable
#define ENWK1 (1<<2) //wake 1 enable
#define DISWK1 (1<<3) //wake 1 disable
#define ENWK2 (1<<4) //wake 2 enable
#define DISWK2 (1<<5) //wake 2 disable
#define ENWK3 (1<<6) //wake 3 enable
#define DISWK3 (1<<7) //wake 3 disable
#define ENWK4 (1<<8) //wake 4 enable
#define DISWK4 (1<<9) //wake 4 disable
#define ENWK5 (1<<10) //wake 5 enable
#define DISWK5 (1<<11) //wake 5 disable
#define ENWK6 (1<<12) //wake 6 enable
#define DISWK6 (1<<13) //wake 6 disable
#define ENWK7 (1<<14) //wake 7 enable
#define DISWK7 (1L<<15) //wake 7 disable
#define ENWK8 (1L<<16) //wake 8 enable
#define DISWK8 (1L<<17) //wake 8 disable
#define ENWK9 (1L<<18) //wake 9 enable
#define DISWK9 (1L<<19) //wake 9 disable
#define ENWK10 (1L<<20) //wake 10 enable
#define DISWK10 (1L<<21) //wake 10 disable
#define ENWK11 (1L<<22) //wake 11 enable
#define DISWK11 (1L<<23) //wake 11 disable
#define D940_REGMASK_MGCWKCTRL (((1L<<24))-1)


// ----- MGCINTSETRESET ----
/** DESCRIPTION: interrupt setreset register */
/** OFFSET: 0x7E */
/** ACCESS: WO */
#define D940_REG_MGCINTSETRESET  0x7E // interrupt setreset register
#define SETINT0 (1<<0) //set interrupt 0
#define CLRINT0 (1<<1) //clr interrupt 0
#define SETINT1 (1<<2) //set interrupt 1
#define CLRINT1 (1<<3) //clr interrupt 1
#define SETINT2 (1<<4) //set interrupt 2
#define CLRINT2 (1<<5) //clr interrupt 2
#define SETINT3 (1<<6) //set interrupt 3
#define CLRINT3 (1<<7) //clr interrupt 3
#define SETINT4 (1<<8) //set interrupt 4
#define CLRINT4 (1<<9) //clr interrupt 4
#define SETINT5 (1<<10) //set interrupt 5
#define CLRINT5 (1<<11) //clr interrupt 5
#define SETINT6 (1<<12) //set interrupt 6
#define CLRINT6 (1<<13) //clr interrupt 6
#define SETINT7 (1<<14) //set interrupt 7
#define CLRINT7 (1L<<15) //clr interrupt 7
#define D940_REGMASK_MGCINTSETRESET (((1L<<16))-1)


// ----- MGCINTPRIORITY ----
/** DESCRIPTION: interrupt priority register */
/** OFFSET: 0x7F */
/** ACCESS: RW */
#define D940_REG_MGCINTPRIORITY  0x7F // interrupt priority register
#define GET_PRIO0(x) ((x>>0) & 7) // GET FIELD
#define SET_PRIO0(x) ((x & 7)<<0) // SET FIELD
#define GET_PRIO1(x) ((x>>3) & 7) // GET FIELD
#define SET_PRIO1(x) ((x & 7)<<3) // SET FIELD
#define GET_PRIO2(x) ((x>>6) & 7) // GET FIELD
#define SET_PRIO2(x) ((x & 7)<<6) // SET FIELD
#define GET_PRIO3(x) ((x>>9) & 7) // GET FIELD
#define SET_PRIO3(x) ((x & 7)<<9) // SET FIELD
#define GET_PRIO4(x) ((x>>12) & 7) // GET FIELD
#define SET_PRIO4(x) ((x & 7)<<12) // SET FIELD
#define GET_PRIO5(x) ((x>>15) & 7) // GET FIELD
#define SET_PRIO5(x) ((x & 7)<<15) // SET FIELD
#define GET_PRIO6(x) ((x>>18) & 7) // GET FIELD
#define SET_PRIO6(x) ((x & 7)<<18) // SET FIELD
#define GET_PRIO7(x) ((x>>21) & 7) // GET FIELD
#define SET_PRIO7(x) ((x & 7)<<21) // SET FIELD
#define D940_REGMASK_MGCINTPRIORITY (((1L<<24))-1)

#endif
