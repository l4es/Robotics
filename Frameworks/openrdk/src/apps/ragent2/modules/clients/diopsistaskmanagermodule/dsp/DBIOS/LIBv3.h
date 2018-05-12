//  ----------------------------------------------------------------------------
//          ATMEL Microcontroller Software Support  -  ROUSSET  -
//  ----------------------------------------------------------------------------
//  DISCLAIMER:  THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
//  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
//  DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
//  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
//  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
//  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
//  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//  ----------------------------------------------------------------------------
// File Name           : D940HF.h
// Object              : D940HF definitions
// Generated       	    : D940 SW Application Group  12/19/2005 (12:19:58)
// D940PS_US
// CVS Reference       : /D940HF.pl/1.1/Tue Dec 28 09:56:37 2004//
// CVS Reference       : /SYS_SAM9261.pl/1.5/Thu Nov 18 13:22:33 2004//
// CVS Reference       : /HMATRIX1_SAM9260.pl/1.3/Thu Oct 13 12:44:25 2005//
// CVS Reference       : /PMC_D940HF.pl/1.3/Mon Jul 19 13:36:27 2004//
// CVS Reference       : /HSMC3_SAM9261.pl/1.1/Tue Nov 16 09:16:07 2004//
// CVS Reference       : /SHDWC_SAM9261.pl/1.1/Tue Mar  8 14:46:52 2005//
// CVS Reference       : /HSDRAMC1_6100A.pl/1.2/Mon Aug  9 10:52:25 2004//
// CVS Reference       : /AIC_6075A.pl/1.1/Mon Jul 12 17:04:01 2004//
// CVS Reference       : /PDC_6074C.pl/1.2/Thu Feb  3 09:02:11 2005//
// CVS Reference       : /DBGU_6059D.pl/1.1/Mon Jan 31 13:54:41 2005//
// CVS Reference       : /PIO_6057A.pl/1.2/Thu Feb  3 10:29:42 2005//
// CVS Reference       : /RSTC_6098A.pl/1.3/Thu Nov  4 13:57:00 2004//
// CVS Reference       : /RTTC_6081A.pl/1.2/Thu Nov  4 13:57:22 2004//
// CVS Reference       : /PITC_6079A.pl/1.2/Thu Nov  4 13:56:22 2004//
// CVS Reference       : /WDTC_6080A.pl/1.3/Thu Nov  4 13:58:52 2004//
// CVS Reference       : /TC_6082A.pl/1.7/Wed Mar  9 16:31:51 2005//
// CVS Reference       : /UDP_6083C.pl/1.1/Mon Jan 31 13:03:53 2005//
// CVS Reference       : /TWI_1761B.pl/1.4/Fri Feb  7 10:30:07 2003//
// CVS Reference       : /US_6089C.pl/1.1/Mon Jan 31 13:56:02 2005//
// CVS Reference       : /SSC_6078A.pl/1.1/Tue Jul 13 07:10:41 2004//
// CVS Reference       : /SPI_6088D.pl/1.2/Mon Feb 14 07:37:17 2005//
// CVS Reference       : /UHP_6127A.pl/1.1/Wed Feb 23 16:03:17 2005//
// CVS Reference       : /EMAC_1794A.pl/1.4/Fri Jan 17 12:11:54 2003//
// CVS Reference       : /CAN_D940HF.pl/1.7/Fri Jul  4 13:08:34 2003//
// CVS Reference       : /MCI_6101A.pl/1.1/Tue Jul 13 06:33:59 2004//
//  ----------------------------------------------------------------------------

#ifndef D940HF_H
#define D940HF_H

#ifdef __chess__
#define D940_REG volatile long
#define __MAGICV_EXTERNAL chess_storage(EXT_DATA)
#else
typedef volatile int D940_REG;// Hardware register definition
#define __MAGICV_EXTERNAL
#define __MAGICV_EXTMEM
#endif

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR System Peripherals
// *****************************************************************************
typedef struct _D940S_SYS {
	D940_REG	 SDRAMC_MR; 	//!< SDRAM Controller Mode Register
	D940_REG	 SDRAMC_TR; 	//!< SDRAM Controller Refresh Timer Register
	D940_REG	 SDRAMC_CR; 	//!< SDRAM Controller Configuration Register
	D940_REG	 SDRAMC_HSR; 	//!< SDRAM Controller High Speed Register
	D940_REG	 SDRAMC_LPR; 	//!< SDRAM Controller Low Power Register
	D940_REG	 SDRAMC_IER; 	//!< SDRAM Controller Interrupt Enable Register
	D940_REG	 SDRAMC_IDR; 	//!< SDRAM Controller Interrupt Disable Register
	D940_REG	 SDRAMC_IMR; 	//!< SDRAM Controller Interrupt Mask Register
	D940_REG	 SDRAMC_ISR; 	//!< SDRAM Controller Interrupt Mask Register
	D940_REG	 SDRAMC_MDR; 	//!< SDRAM Memory Device Register
	D940_REG	 Reserved0[118]; 	//!< 
	D940_REG	 SMC_SETUP0; 	//!<  Setup Register for CS 0
	D940_REG	 SMC_PULSE0; 	//!<  Pulse Register for CS 0
	D940_REG	 SMC_CYCLE0; 	//!<  Cycle Register for CS 0
	D940_REG	 SMC_CTRL0; 	//!<  Control Register for CS 0
	D940_REG	 SMC_SETUP1; 	//!<  Setup Register for CS 1
	D940_REG	 SMC_PULSE1; 	//!<  Pulse Register for CS 1
	D940_REG	 SMC_CYCLE1; 	//!<  Cycle Register for CS 1
	D940_REG	 SMC_CTRL1; 	//!<  Control Register for CS 1
	D940_REG	 SMC_SETUP2; 	//!<  Setup Register for CS 2
	D940_REG	 SMC_PULSE2; 	//!<  Pulse Register for CS 2
	D940_REG	 SMC_CYCLE2; 	//!<  Cycle Register for CS 2
	D940_REG	 SMC_CTRL2; 	//!<  Control Register for CS 2
	D940_REG	 SMC_SETUP3; 	//!<  Setup Register for CS 3
	D940_REG	 SMC_PULSE3; 	//!<  Pulse Register for CS 3
	D940_REG	 SMC_CYCLE3; 	//!<  Cycle Register for CS 3
	D940_REG	 SMC_CTRL3; 	//!<  Control Register for CS 3
	D940_REG	 SMC_SETUP4; 	//!<  Setup Register for CS 4
	D940_REG	 SMC_PULSE4; 	//!<  Pulse Register for CS 4
	D940_REG	 SMC_CYCLE4; 	//!<  Cycle Register for CS 4
	D940_REG	 SMC_CTRL4; 	//!<  Control Register for CS 4
	D940_REG	 SMC_SETUP5; 	//!<  Setup Register for CS 5
	D940_REG	 SMC_PULSE5; 	//!<  Pulse Register for CS 5
	D940_REG	 SMC_CYCLE5; 	//!<  Cycle Register for CS 5
	D940_REG	 SMC_CTRL5; 	//!<  Control Register for CS 5
	D940_REG	 SMC_SETUP6; 	//!<  Setup Register for CS 6
	D940_REG	 SMC_PULSE6; 	//!<  Pulse Register for CS 6
	D940_REG	 SMC_CYCLE6; 	//!<  Cycle Register for CS 6
	D940_REG	 SMC_CTRL6; 	//!<  Control Register for CS 6
	D940_REG	 SMC_SETUP7; 	//!<  Setup Register for CS 7
	D940_REG	 SMC_PULSE7; 	//!<  Pulse Register for CS 7
	D940_REG	 SMC_CYCLE7; 	//!<  Cycle Register for CS 7
	D940_REG	 SMC_CTRL7; 	//!<  Control Register for CS 7
	D940_REG	 Reserved1[96]; 	//!< 
	D940_REG	 MATRIX_MCFG0; 	//!<  Master Configuration Register 0 
	D940_REG	 MATRIX_MCFG1; 	//!<  Master Configuration Register 1 
	D940_REG	 MATRIX_MCFG2; 	//!<  Master Configuration Register 2 
	D940_REG	 MATRIX_MCFG3; 	//!<  Master Configuration Register 3 
	D940_REG	 MATRIX_MCFG4; 	//!<  Master Configuration Register 4 
	D940_REG	 MATRIX_MCFG5; 	//!<  Master Configuration Register 5 
	D940_REG	 Reserved2[10]; 	//!< 
	D940_REG	 MATRIX_SCFG0; 	//!<  Slave Configuration Register 0 
	D940_REG	 MATRIX_SCFG1; 	//!<  Slave Configuration Register 1 
	D940_REG	 MATRIX_SCFG2; 	//!<  Slave Configuration Register 2 
	D940_REG	 MATRIX_SCFG3; 	//!<  Slave Configuration Register 3 
	D940_REG	 MATRIX_SCFG4; 	//!<  Slave Configuration Register 4 
	D940_REG	 Reserved3[11]; 	//!< 
	D940_REG	 MATRIX_PRAS0; 	//!<  PRAS0 
	D940_REG	 Reserved4[1]; 	//!< 
	D940_REG	 MATRIX_PRAS1; 	//!<  PRAS1 
	D940_REG	 Reserved5[1]; 	//!< 
	D940_REG	 MATRIX_PRAS2; 	//!<  PRAS2 
	D940_REG	 Reserved6[1]; 	//!< 
	D940_REG	 MATRIX_PRAS3; 	//!<  PRAS3 
	D940_REG	 Reserved7[1]; 	//!< 
	D940_REG	 MATRIX_PRAS4; 	//!<  PRAS4 
	D940_REG	 Reserved8[23]; 	//!< 
	D940_REG	 MATRIX_MRCR; 	//!<  Master Remp Control Register 
	D940_REG	 Reserved9[63]; 	//!< 
	D940_REG	 AIC_SMR[32]; 	//!< Source Mode Register
	D940_REG	 AIC_SVR[32]; 	//!< Source Vector Register
	D940_REG	 AIC_IVR; 	//!< IRQ Vector Register
	D940_REG	 AIC_FVR; 	//!< FIQ Vector Register
	D940_REG	 AIC_ISR; 	//!< Interrupt Status Register
	D940_REG	 AIC_IPR; 	//!< Interrupt Pending Register
	D940_REG	 AIC_IMR; 	//!< Interrupt Mask Register
	D940_REG	 AIC_CISR; 	//!< Core Interrupt Status Register
	D940_REG	 Reserved10[2]; 	//!< 
	D940_REG	 AIC_IECR; 	//!< Interrupt Enable Command Register
	D940_REG	 AIC_IDCR; 	//!< Interrupt Disable Command Register
	D940_REG	 AIC_ICCR; 	//!< Interrupt Clear Command Register
	D940_REG	 AIC_ISCR; 	//!< Interrupt Set Command Register
	D940_REG	 AIC_EOICR; 	//!< End of Interrupt Command Register
	D940_REG	 AIC_SPU; 	//!< Spurious Vector Register
	D940_REG	 AIC_DCR; 	//!< Debug Control Register (Protect)
	D940_REG	 Reserved11[1]; 	//!< 
	D940_REG	 AIC_FFER; 	//!< Fast Forcing Enable Register
	D940_REG	 AIC_FFDR; 	//!< Fast Forcing Disable Register
	D940_REG	 AIC_FFSR; 	//!< Fast Forcing Status Register
	D940_REG	 Reserved12[45]; 	//!< 
	D940_REG	 DBGU_CR; 	//!< Control Register
	D940_REG	 DBGU_MR; 	//!< Mode Register
	D940_REG	 DBGU_IER; 	//!< Interrupt Enable Register
	D940_REG	 DBGU_IDR; 	//!< Interrupt Disable Register
	D940_REG	 DBGU_IMR; 	//!< Interrupt Mask Register
	D940_REG	 DBGU_CSR; 	//!< Channel Status Register
	D940_REG	 DBGU_RHR; 	//!< Receiver Holding Register
	D940_REG	 DBGU_THR; 	//!< Transmitter Holding Register
	D940_REG	 DBGU_BRGR; 	//!< Baud Rate Generator Register
	D940_REG	 Reserved13[7]; 	//!< 
	D940_REG	 DBGU_CIDR; 	//!< Chip ID Register
	D940_REG	 DBGU_EXID; 	//!< Chip ID Extension Register
	D940_REG	 DBGU_FNTR; 	//!< Force NTRST Register
	D940_REG	 Reserved14[45]; 	//!< 
	D940_REG	 DBGU_RPR; 	//!< Receive Pointer Register
	D940_REG	 DBGU_RCR; 	//!< Receive Counter Register
	D940_REG	 DBGU_TPR; 	//!< Transmit Pointer Register
	D940_REG	 DBGU_TCR; 	//!< Transmit Counter Register
	D940_REG	 DBGU_RNPR; 	//!< Receive Next Pointer Register
	D940_REG	 DBGU_RNCR; 	//!< Receive Next Counter Register
	D940_REG	 DBGU_TNPR; 	//!< Transmit Next Pointer Register
	D940_REG	 DBGU_TNCR; 	//!< Transmit Next Counter Register
	D940_REG	 DBGU_PTCR; 	//!< PDC Transfer Control Register
	D940_REG	 DBGU_PTSR; 	//!< PDC Transfer Status Register
	D940_REG	 Reserved15[54]; 	//!< 
	D940_REG	 PIOA_PER; 	//!< PIO Enable Register
	D940_REG	 PIOA_PDR; 	//!< PIO Disable Register
	D940_REG	 PIOA_PSR; 	//!< PIO Status Register
	D940_REG	 Reserved16[1]; 	//!< 
	D940_REG	 PIOA_OER; 	//!< Output Enable Register
	D940_REG	 PIOA_ODR; 	//!< Output Disable Registerr
	D940_REG	 PIOA_OSR; 	//!< Output Status Register
	D940_REG	 Reserved17[1]; 	//!< 
	D940_REG	 PIOA_IFER; 	//!< Input Filter Enable Register
	D940_REG	 PIOA_IFDR; 	//!< Input Filter Disable Register
	D940_REG	 PIOA_IFSR; 	//!< Input Filter Status Register
	D940_REG	 Reserved18[1]; 	//!< 
	D940_REG	 PIOA_SODR; 	//!< Set Output Data Register
	D940_REG	 PIOA_CODR; 	//!< Clear Output Data Register
	D940_REG	 PIOA_ODSR; 	//!< Output Data Status Register
	D940_REG	 PIOA_PDSR; 	//!< Pin Data Status Register
	D940_REG	 PIOA_IER; 	//!< Interrupt Enable Register
	D940_REG	 PIOA_IDR; 	//!< Interrupt Disable Register
	D940_REG	 PIOA_IMR; 	//!< Interrupt Mask Register
	D940_REG	 PIOA_ISR; 	//!< Interrupt Status Register
	D940_REG	 PIOA_MDER; 	//!< Multi-driver Enable Register
	D940_REG	 PIOA_MDDR; 	//!< Multi-driver Disable Register
	D940_REG	 PIOA_MDSR; 	//!< Multi-driver Status Register
	D940_REG	 Reserved19[1]; 	//!< 
	D940_REG	 PIOA_PPUDR; 	//!< Pull-up Disable Register
	D940_REG	 PIOA_PPUER; 	//!< Pull-up Enable Register
	D940_REG	 PIOA_PPUSR; 	//!< Pull-up Status Register
	D940_REG	 Reserved20[1]; 	//!< 
	D940_REG	 PIOA_ASR; 	//!< Select A Register
	D940_REG	 PIOA_BSR; 	//!< Select B Register
	D940_REG	 PIOA_ABSR; 	//!< AB Select Status Register
	D940_REG	 Reserved21[9]; 	//!< 
	D940_REG	 PIOA_OWER; 	//!< Output Write Enable Register
	D940_REG	 PIOA_OWDR; 	//!< Output Write Disable Register
	D940_REG	 PIOA_OWSR; 	//!< Output Write Status Register
	D940_REG	 Reserved22[85]; 	//!< 
	D940_REG	 PIOB_PER; 	//!< PIO Enable Register
	D940_REG	 PIOB_PDR; 	//!< PIO Disable Register
	D940_REG	 PIOB_PSR; 	//!< PIO Status Register
	D940_REG	 Reserved23[1]; 	//!< 
	D940_REG	 PIOB_OER; 	//!< Output Enable Register
	D940_REG	 PIOB_ODR; 	//!< Output Disable Registerr
	D940_REG	 PIOB_OSR; 	//!< Output Status Register
	D940_REG	 Reserved24[1]; 	//!< 
	D940_REG	 PIOB_IFER; 	//!< Input Filter Enable Register
	D940_REG	 PIOB_IFDR; 	//!< Input Filter Disable Register
	D940_REG	 PIOB_IFSR; 	//!< Input Filter Status Register
	D940_REG	 Reserved25[1]; 	//!< 
	D940_REG	 PIOB_SODR; 	//!< Set Output Data Register
	D940_REG	 PIOB_CODR; 	//!< Clear Output Data Register
	D940_REG	 PIOB_ODSR; 	//!< Output Data Status Register
	D940_REG	 PIOB_PDSR; 	//!< Pin Data Status Register
	D940_REG	 PIOB_IER; 	//!< Interrupt Enable Register
	D940_REG	 PIOB_IDR; 	//!< Interrupt Disable Register
	D940_REG	 PIOB_IMR; 	//!< Interrupt Mask Register
	D940_REG	 PIOB_ISR; 	//!< Interrupt Status Register
	D940_REG	 PIOB_MDER; 	//!< Multi-driver Enable Register
	D940_REG	 PIOB_MDDR; 	//!< Multi-driver Disable Register
	D940_REG	 PIOB_MDSR; 	//!< Multi-driver Status Register
	D940_REG	 Reserved26[1]; 	//!< 
	D940_REG	 PIOB_PPUDR; 	//!< Pull-up Disable Register
	D940_REG	 PIOB_PPUER; 	//!< Pull-up Enable Register
	D940_REG	 PIOB_PPUSR; 	//!< Pull-up Status Register
	D940_REG	 Reserved27[1]; 	//!< 
	D940_REG	 PIOB_ASR; 	//!< Select A Register
	D940_REG	 PIOB_BSR; 	//!< Select B Register
	D940_REG	 PIOB_ABSR; 	//!< AB Select Status Register
	D940_REG	 Reserved28[9]; 	//!< 
	D940_REG	 PIOB_OWER; 	//!< Output Write Enable Register
	D940_REG	 PIOB_OWDR; 	//!< Output Write Disable Register
	D940_REG	 PIOB_OWSR; 	//!< Output Write Status Register
	D940_REG	 Reserved29[85]; 	//!< 
	D940_REG	 PIOC_PER; 	//!< PIO Enable Register
	D940_REG	 PIOC_PDR; 	//!< PIO Disable Register
	D940_REG	 PIOC_PSR; 	//!< PIO Status Register
	D940_REG	 Reserved30[1]; 	//!< 
	D940_REG	 PIOC_OER; 	//!< Output Enable Register
	D940_REG	 PIOC_ODR; 	//!< Output Disable Registerr
	D940_REG	 PIOC_OSR; 	//!< Output Status Register
	D940_REG	 Reserved31[1]; 	//!< 
	D940_REG	 PIOC_IFER; 	//!< Input Filter Enable Register
	D940_REG	 PIOC_IFDR; 	//!< Input Filter Disable Register
	D940_REG	 PIOC_IFSR; 	//!< Input Filter Status Register
	D940_REG	 Reserved32[1]; 	//!< 
	D940_REG	 PIOC_SODR; 	//!< Set Output Data Register
	D940_REG	 PIOC_CODR; 	//!< Clear Output Data Register
	D940_REG	 PIOC_ODSR; 	//!< Output Data Status Register
	D940_REG	 PIOC_PDSR; 	//!< Pin Data Status Register
	D940_REG	 PIOC_IER; 	//!< Interrupt Enable Register
	D940_REG	 PIOC_IDR; 	//!< Interrupt Disable Register
	D940_REG	 PIOC_IMR; 	//!< Interrupt Mask Register
	D940_REG	 PIOC_ISR; 	//!< Interrupt Status Register
	D940_REG	 PIOC_MDER; 	//!< Multi-driver Enable Register
	D940_REG	 PIOC_MDDR; 	//!< Multi-driver Disable Register
	D940_REG	 PIOC_MDSR; 	//!< Multi-driver Status Register
	D940_REG	 Reserved33[1]; 	//!< 
	D940_REG	 PIOC_PPUDR; 	//!< Pull-up Disable Register
	D940_REG	 PIOC_PPUER; 	//!< Pull-up Enable Register
	D940_REG	 PIOC_PPUSR; 	//!< Pull-up Status Register
	D940_REG	 Reserved34[1]; 	//!< 
	D940_REG	 PIOC_ASR; 	//!< Select A Register
	D940_REG	 PIOC_BSR; 	//!< Select B Register
	D940_REG	 PIOC_ABSR; 	//!< AB Select Status Register
	D940_REG	 Reserved35[9]; 	//!< 
	D940_REG	 PIOC_OWER; 	//!< Output Write Enable Register
	D940_REG	 PIOC_OWDR; 	//!< Output Write Disable Register
	D940_REG	 PIOC_OWSR; 	//!< Output Write Status Register
	D940_REG	 Reserved36[213]; 	//!< 
	D940_REG	 PMC_SCER; 	//!< System Clock Enable Register
	D940_REG	 PMC_SCDR; 	//!< System Clock Disable Register
	D940_REG	 PMC_SCSR; 	//!< System Clock Status Register
	D940_REG	 Reserved37[1]; 	//!< 
	D940_REG	 PMC_PCER; 	//!< Peripheral Clock Enable Register
	D940_REG	 PMC_PCDR; 	//!< Peripheral Clock Disable Register
	D940_REG	 PMC_PCSR; 	//!< Peripheral Clock Status Register
	D940_REG	 Reserved38[1]; 	//!< 
	D940_REG	 PMC_MOR; 	//!< Main Oscillator Register
	D940_REG	 PMC_MCFR; 	//!< Main Clock  Frequency Register
	D940_REG	 PMC_PLLAR; 	//!< PLL A Register
	D940_REG	 PMC_PLLBR; 	//!< PLL B Register
	D940_REG	 PMC_MCKR; 	//!< Master Clock Register
	D940_REG	 Reserved39[3]; 	//!< 
	D940_REG	 PMC_PCKR[8]; 	//!< Programmable Clock Register
	D940_REG	 PMC_IER; 	//!< Interrupt Enable Register
	D940_REG	 PMC_IDR; 	//!< Interrupt Disable Register
	D940_REG	 PMC_SR; 	//!< Status Register
	D940_REG	 PMC_IMR; 	//!< Interrupt Mask Register
	D940_REG	 Reserved40[36]; 	//!< 
	D940_REG	 RSTC_RCR; 	//!< Reset Control Register
	D940_REG	 RSTC_RSR; 	//!< Reset Status Register
	D940_REG	 RSTC_RMR; 	//!< Reset Mode Register
	D940_REG	 Reserved41[1]; 	//!< 
	D940_REG	 SHDWC_SHCR; 	//!< Shut Down Control Register
	D940_REG	 SHDWC_SHMR; 	//!< Shut Down Mode Register
	D940_REG	 SHDWC_SHSR; 	//!< Shut Down Status Register
	D940_REG	 Reserved42[1]; 	//!< 
	D940_REG	 RTTC_RTMR; 	//!< Real-time Mode Register
	D940_REG	 RTTC_RTAR; 	//!< Real-time Alarm Register
	D940_REG	 RTTC_RTVR; 	//!< Real-time Value Register
	D940_REG	 RTTC_RTSR; 	//!< Real-time Status Register
	D940_REG	 PITC_PIMR; 	//!< Period Interval Mode Register
	D940_REG	 PITC_PISR; 	//!< Period Interval Status Register
	D940_REG	 PITC_PIVR; 	//!< Period Interval Value Register
	D940_REG	 PITC_PIIR; 	//!< Period Interval Image Register
	D940_REG	 WDTC_WDCR; 	//!< Watchdog Control Register
	D940_REG	 WDTC_WDMR; 	//!< Watchdog Mode Register
	D940_REG	 WDTC_WDSR; 	//!< Watchdog Status Register
	D940_REG	 Reserved43[1]; 	//!< 
	D940_REG	 SYS_GPBR0; 	//!< General Purpose Register 0
	D940_REG	 SYS_GPBR1; 	//!< General Purpose Register 1
	D940_REG	 SYS_GPBR2; 	//!< General Purpose Register 2
	D940_REG	 SYS_GPBR3; 	//!< General Purpose Register 3
} D940S_SYS,  __MAGICV_EXTERNAL *D940PS_SYS;

// -------- GPBR : (SYS Offset: 0x1350) GPBR General Purpose Register -------- 
// -------- GPBR : (SYS Offset: 0x1354) GPBR General Purpose Register -------- 
// -------- GPBR : (SYS Offset: 0x1358) GPBR General Purpose Register -------- 
// -------- GPBR : (SYS Offset: 0x135c) GPBR General Purpose Register -------- 

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR SDRAM Controller Interface
// *****************************************************************************
typedef struct _D940S_SDRAMC {
	D940_REG	 SDRAMC_MR; 	//!< SDRAM Controller Mode Register
	D940_REG	 SDRAMC_TR; 	//!< SDRAM Controller Refresh Timer Register
	D940_REG	 SDRAMC_CR; 	//!< SDRAM Controller Configuration Register
	D940_REG	 SDRAMC_HSR; 	//!< SDRAM Controller High Speed Register
	D940_REG	 SDRAMC_LPR; 	//!< SDRAM Controller Low Power Register
	D940_REG	 SDRAMC_IER; 	//!< SDRAM Controller Interrupt Enable Register
	D940_REG	 SDRAMC_IDR; 	//!< SDRAM Controller Interrupt Disable Register
	D940_REG	 SDRAMC_IMR; 	//!< SDRAM Controller Interrupt Mask Register
	D940_REG	 SDRAMC_ISR; 	//!< SDRAM Controller Interrupt Mask Register
	D940_REG	 SDRAMC_MDR; 	//!< SDRAM Memory Device Register
} D940S_SDRAMC,  __MAGICV_EXTERNAL *D940PS_SDRAMC;

// -------- SDRAMC_MR : (SDRAMC Offset: 0x0) SDRAM Controller Mode Register -------- 
#define D940C_SDRAMC_MODE     ((D940_REG) 0xF <<  0) //!< (SDRAMC) Mode
#define 	D940C_SDRAMC_MODE_NORMAL_CMD           ((D940_REG) 0x0) //!< (SDRAMC) Normal Mode
#define 	D940C_SDRAMC_MODE_NOP_CMD              ((D940_REG) 0x1) //!< (SDRAMC) Issue a NOP Command at every access
#define 	D940C_SDRAMC_MODE_PRCGALL_CMD          ((D940_REG) 0x2) //!< (SDRAMC) Issue a All Banks Precharge Command at every access
#define 	D940C_SDRAMC_MODE_LMR_CMD              ((D940_REG) 0x3) //!< (SDRAMC) Issue a Load Mode Register at every access
#define 	D940C_SDRAMC_MODE_RFSH_CMD             ((D940_REG) 0x4) //!< (SDRAMC) Issue a Refresh
#define 	D940C_SDRAMC_MODE_EXT_LMR_CMD          ((D940_REG) 0x5) //!< (SDRAMC) Issue an Extended Load Mode Register
#define 	D940C_SDRAMC_MODE_DEEP_CMD             ((D940_REG) 0x6) //!< (SDRAMC) Enter Deep Power Mode
// -------- SDRAMC_TR : (SDRAMC Offset: 0x4) SDRAMC Refresh Timer Register -------- 
#define D940C_SDRAMC_COUNT    ((D940_REG) 0xFFF <<  0) //!< (SDRAMC) Refresh Counter
// -------- SDRAMC_CR : (SDRAMC Offset: 0x8) SDRAM Configuration Register -------- 
#define D940C_SDRAMC_NC       ((D940_REG) 0x3 <<  0) //!< (SDRAMC) Number of Column Bits
#define 	D940C_SDRAMC_NC_8                    ((D940_REG) 0x0) //!< (SDRAMC) 8 Bits
#define 	D940C_SDRAMC_NC_9                    ((D940_REG) 0x1) //!< (SDRAMC) 9 Bits
#define 	D940C_SDRAMC_NC_10                   ((D940_REG) 0x2) //!< (SDRAMC) 10 Bits
#define 	D940C_SDRAMC_NC_11                   ((D940_REG) 0x3) //!< (SDRAMC) 11 Bits
#define D940C_SDRAMC_NR       ((D940_REG) 0x3 <<  2) //!< (SDRAMC) Number of Row Bits
#define 	D940C_SDRAMC_NR_11                   ((D940_REG) 0x0 <<  2) //!< (SDRAMC) 11 Bits
#define 	D940C_SDRAMC_NR_12                   ((D940_REG) 0x1 <<  2) //!< (SDRAMC) 12 Bits
#define 	D940C_SDRAMC_NR_13                   ((D940_REG) 0x2 <<  2) //!< (SDRAMC) 13 Bits
#define D940C_SDRAMC_NB       ((D940_REG) 0x1 <<  4) //!< (SDRAMC) Number of Banks
#define 	D940C_SDRAMC_NB_2_BANKS              ((D940_REG) 0x0 <<  4) //!< (SDRAMC) 2 banks
#define 	D940C_SDRAMC_NB_4_BANKS              ((D940_REG) 0x1 <<  4) //!< (SDRAMC) 4 banks
#define D940C_SDRAMC_CAS      ((D940_REG) 0x3 <<  5) //!< (SDRAMC) CAS Latency
#define 	D940C_SDRAMC_CAS_2                    ((D940_REG) 0x2 <<  5) //!< (SDRAMC) 2 cycles
#define 	D940C_SDRAMC_CAS_3                    ((D940_REG) 0x3 <<  5) //!< (SDRAMC) 3 cycles
#define D940C_SDRAMC_DBW      ((D940_REG) 0x1 <<  7) //!< (SDRAMC) Data Bus Width
#define 	D940C_SDRAMC_DBW_32_BITS              ((D940_REG) 0x0 <<  7) //!< (SDRAMC) 32 Bits datas bus
#define 	D940C_SDRAMC_DBW_16_BITS              ((D940_REG) 0x1 <<  7) //!< (SDRAMC) 16 Bits datas bus
#define D940C_SDRAMC_TWR      ((D940_REG) 0xF <<  8) //!< (SDRAMC) Number of Write Recovery Time Cycles
#define 	D940C_SDRAMC_TWR_0                    ((D940_REG) 0x0 <<  8) //!< (SDRAMC) Value :  0
#define 	D940C_SDRAMC_TWR_1                    ((D940_REG) 0x1 <<  8) //!< (SDRAMC) Value :  1
#define 	D940C_SDRAMC_TWR_2                    ((D940_REG) 0x2 <<  8) //!< (SDRAMC) Value :  2
#define 	D940C_SDRAMC_TWR_3                    ((D940_REG) 0x3 <<  8) //!< (SDRAMC) Value :  3
#define 	D940C_SDRAMC_TWR_4                    ((D940_REG) 0x4 <<  8) //!< (SDRAMC) Value :  4
#define 	D940C_SDRAMC_TWR_5                    ((D940_REG) 0x5 <<  8) //!< (SDRAMC) Value :  5
#define 	D940C_SDRAMC_TWR_6                    ((D940_REG) 0x6 <<  8) //!< (SDRAMC) Value :  6
#define 	D940C_SDRAMC_TWR_7                    ((D940_REG) 0x7 <<  8) //!< (SDRAMC) Value :  7
#define 	D940C_SDRAMC_TWR_8                    ((D940_REG) 0x8 <<  8) //!< (SDRAMC) Value :  8
#define 	D940C_SDRAMC_TWR_9                    ((D940_REG) 0x9 <<  8) //!< (SDRAMC) Value :  9
#define 	D940C_SDRAMC_TWR_10                   ((D940_REG) 0xA <<  8) //!< (SDRAMC) Value : 10
#define 	D940C_SDRAMC_TWR_11                   ((D940_REG) 0xB <<  8) //!< (SDRAMC) Value : 11
#define 	D940C_SDRAMC_TWR_12                   ((D940_REG) 0xC <<  8) //!< (SDRAMC) Value : 12
#define 	D940C_SDRAMC_TWR_13                   ((D940_REG) 0xD <<  8) //!< (SDRAMC) Value : 13
#define 	D940C_SDRAMC_TWR_14                   ((D940_REG) 0xE <<  8) //!< (SDRAMC) Value : 14
#define 	D940C_SDRAMC_TWR_15                   ((D940_REG) 0xF <<  8) //!< (SDRAMC) Value : 15
#define D940C_SDRAMC_TRC      ((D940_REG) 0xF << 12) //!< (SDRAMC) Number of RAS Cycle Time Cycles
#define 	D940C_SDRAMC_TRC_0                    ((D940_REG) 0x0 << 12) //!< (SDRAMC) Value :  0
#define 	D940C_SDRAMC_TRC_1                    ((D940_REG) 0x1 << 12) //!< (SDRAMC) Value :  1
#define 	D940C_SDRAMC_TRC_2                    ((D940_REG) 0x2 << 12) //!< (SDRAMC) Value :  2
#define 	D940C_SDRAMC_TRC_3                    ((D940_REG) 0x3 << 12) //!< (SDRAMC) Value :  3
#define 	D940C_SDRAMC_TRC_4                    ((D940_REG) 0x4 << 12) //!< (SDRAMC) Value :  4
#define 	D940C_SDRAMC_TRC_5                    ((D940_REG) 0x5 << 12) //!< (SDRAMC) Value :  5
#define 	D940C_SDRAMC_TRC_6                    ((D940_REG) 0x6 << 12) //!< (SDRAMC) Value :  6
#define 	D940C_SDRAMC_TRC_7                    ((D940_REG) 0x7 << 12) //!< (SDRAMC) Value :  7
#define 	D940C_SDRAMC_TRC_8                    ((D940_REG) 0x8 << 12) //!< (SDRAMC) Value :  8
#define 	D940C_SDRAMC_TRC_9                    ((D940_REG) 0x9 << 12) //!< (SDRAMC) Value :  9
#define 	D940C_SDRAMC_TRC_10                   ((D940_REG) 0xA << 12) //!< (SDRAMC) Value : 10
#define 	D940C_SDRAMC_TRC_11                   ((D940_REG) 0xB << 12) //!< (SDRAMC) Value : 11
#define 	D940C_SDRAMC_TRC_12                   ((D940_REG) 0xC << 12) //!< (SDRAMC) Value : 12
#define 	D940C_SDRAMC_TRC_13                   ((D940_REG) 0xD << 12) //!< (SDRAMC) Value : 13
#define 	D940C_SDRAMC_TRC_14                   ((D940_REG) 0xE << 12) //!< (SDRAMC) Value : 14
#define 	D940C_SDRAMC_TRC_15                   ((D940_REG) 0xF << 12) //!< (SDRAMC) Value : 15
#define D940C_SDRAMC_TRP      ((D940_REG) 0xF << 16) //!< (SDRAMC) Number of RAS Precharge Time Cycles
#define 	D940C_SDRAMC_TRP_0                    ((D940_REG) 0x0 << 16) //!< (SDRAMC) Value :  0
#define 	D940C_SDRAMC_TRP_1                    ((D940_REG) 0x1 << 16) //!< (SDRAMC) Value :  1
#define 	D940C_SDRAMC_TRP_2                    ((D940_REG) 0x2 << 16) //!< (SDRAMC) Value :  2
#define 	D940C_SDRAMC_TRP_3                    ((D940_REG) 0x3 << 16) //!< (SDRAMC) Value :  3
#define 	D940C_SDRAMC_TRP_4                    ((D940_REG) 0x4 << 16) //!< (SDRAMC) Value :  4
#define 	D940C_SDRAMC_TRP_5                    ((D940_REG) 0x5 << 16) //!< (SDRAMC) Value :  5
#define 	D940C_SDRAMC_TRP_6                    ((D940_REG) 0x6 << 16) //!< (SDRAMC) Value :  6
#define 	D940C_SDRAMC_TRP_7                    ((D940_REG) 0x7 << 16) //!< (SDRAMC) Value :  7
#define 	D940C_SDRAMC_TRP_8                    ((D940_REG) 0x8 << 16) //!< (SDRAMC) Value :  8
#define 	D940C_SDRAMC_TRP_9                    ((D940_REG) 0x9 << 16) //!< (SDRAMC) Value :  9
#define 	D940C_SDRAMC_TRP_10                   ((D940_REG) 0xA << 16) //!< (SDRAMC) Value : 10
#define 	D940C_SDRAMC_TRP_11                   ((D940_REG) 0xB << 16) //!< (SDRAMC) Value : 11
#define 	D940C_SDRAMC_TRP_12                   ((D940_REG) 0xC << 16) //!< (SDRAMC) Value : 12
#define 	D940C_SDRAMC_TRP_13                   ((D940_REG) 0xD << 16) //!< (SDRAMC) Value : 13
#define 	D940C_SDRAMC_TRP_14                   ((D940_REG) 0xE << 16) //!< (SDRAMC) Value : 14
#define 	D940C_SDRAMC_TRP_15                   ((D940_REG) 0xF << 16) //!< (SDRAMC) Value : 15
#define D940C_SDRAMC_TRCD     ((D940_REG) 0xF << 20) //!< (SDRAMC) Number of RAS to CAS Delay Cycles
#define 	D940C_SDRAMC_TRCD_0                    ((D940_REG) 0x0 << 20) //!< (SDRAMC) Value :  0
#define 	D940C_SDRAMC_TRCD_1                    ((D940_REG) 0x1 << 20) //!< (SDRAMC) Value :  1
#define 	D940C_SDRAMC_TRCD_2                    ((D940_REG) 0x2 << 20) //!< (SDRAMC) Value :  2
#define 	D940C_SDRAMC_TRCD_3                    ((D940_REG) 0x3 << 20) //!< (SDRAMC) Value :  3
#define 	D940C_SDRAMC_TRCD_4                    ((D940_REG) 0x4 << 20) //!< (SDRAMC) Value :  4
#define 	D940C_SDRAMC_TRCD_5                    ((D940_REG) 0x5 << 20) //!< (SDRAMC) Value :  5
#define 	D940C_SDRAMC_TRCD_6                    ((D940_REG) 0x6 << 20) //!< (SDRAMC) Value :  6
#define 	D940C_SDRAMC_TRCD_7                    ((D940_REG) 0x7 << 20) //!< (SDRAMC) Value :  7
#define 	D940C_SDRAMC_TRCD_8                    ((D940_REG) 0x8 << 20) //!< (SDRAMC) Value :  8
#define 	D940C_SDRAMC_TRCD_9                    ((D940_REG) 0x9 << 20) //!< (SDRAMC) Value :  9
#define 	D940C_SDRAMC_TRCD_10                   ((D940_REG) 0xA << 20) //!< (SDRAMC) Value : 10
#define 	D940C_SDRAMC_TRCD_11                   ((D940_REG) 0xB << 20) //!< (SDRAMC) Value : 11
#define 	D940C_SDRAMC_TRCD_12                   ((D940_REG) 0xC << 20) //!< (SDRAMC) Value : 12
#define 	D940C_SDRAMC_TRCD_13                   ((D940_REG) 0xD << 20) //!< (SDRAMC) Value : 13
#define 	D940C_SDRAMC_TRCD_14                   ((D940_REG) 0xE << 20) //!< (SDRAMC) Value : 14
#define 	D940C_SDRAMC_TRCD_15                   ((D940_REG) 0xF << 20) //!< (SDRAMC) Value : 15
#define D940C_SDRAMC_TRAS     ((D940_REG) 0xF << 24) //!< (SDRAMC) Number of RAS Active Time Cycles
#define 	D940C_SDRAMC_TRAS_0                    ((D940_REG) 0x0 << 24) //!< (SDRAMC) Value :  0
#define 	D940C_SDRAMC_TRAS_1                    ((D940_REG) 0x1 << 24) //!< (SDRAMC) Value :  1
#define 	D940C_SDRAMC_TRAS_2                    ((D940_REG) 0x2 << 24) //!< (SDRAMC) Value :  2
#define 	D940C_SDRAMC_TRAS_3                    ((D940_REG) 0x3 << 24) //!< (SDRAMC) Value :  3
#define 	D940C_SDRAMC_TRAS_4                    ((D940_REG) 0x4 << 24) //!< (SDRAMC) Value :  4
#define 	D940C_SDRAMC_TRAS_5                    ((D940_REG) 0x5 << 24) //!< (SDRAMC) Value :  5
#define 	D940C_SDRAMC_TRAS_6                    ((D940_REG) 0x6 << 24) //!< (SDRAMC) Value :  6
#define 	D940C_SDRAMC_TRAS_7                    ((D940_REG) 0x7 << 24) //!< (SDRAMC) Value :  7
#define 	D940C_SDRAMC_TRAS_8                    ((D940_REG) 0x8 << 24) //!< (SDRAMC) Value :  8
#define 	D940C_SDRAMC_TRAS_9                    ((D940_REG) 0x9 << 24) //!< (SDRAMC) Value :  9
#define 	D940C_SDRAMC_TRAS_10                   ((D940_REG) 0xA << 24) //!< (SDRAMC) Value : 10
#define 	D940C_SDRAMC_TRAS_11                   ((D940_REG) 0xB << 24) //!< (SDRAMC) Value : 11
#define 	D940C_SDRAMC_TRAS_12                   ((D940_REG) 0xC << 24) //!< (SDRAMC) Value : 12
#define 	D940C_SDRAMC_TRAS_13                   ((D940_REG) 0xD << 24) //!< (SDRAMC) Value : 13
#define 	D940C_SDRAMC_TRAS_14                   ((D940_REG) 0xE << 24) //!< (SDRAMC) Value : 14
#define 	D940C_SDRAMC_TRAS_15                   ((D940_REG) 0xF << 24) //!< (SDRAMC) Value : 15
#define D940C_SDRAMC_TXSR     ((D940_REG) 0xF << 28) //!< (SDRAMC) Number of Command Recovery Time Cycles
#define 	D940C_SDRAMC_TXSR_0                    ((D940_REG) 0x0 << 28) //!< (SDRAMC) Value :  0
#define 	D940C_SDRAMC_TXSR_1                    ((D940_REG) 0x1 << 28) //!< (SDRAMC) Value :  1
#define 	D940C_SDRAMC_TXSR_2                    ((D940_REG) 0x2 << 28) //!< (SDRAMC) Value :  2
#define 	D940C_SDRAMC_TXSR_3                    ((D940_REG) 0x3 << 28) //!< (SDRAMC) Value :  3
#define 	D940C_SDRAMC_TXSR_4                    ((D940_REG) 0x4 << 28) //!< (SDRAMC) Value :  4
#define 	D940C_SDRAMC_TXSR_5                    ((D940_REG) 0x5 << 28) //!< (SDRAMC) Value :  5
#define 	D940C_SDRAMC_TXSR_6                    ((D940_REG) 0x6 << 28) //!< (SDRAMC) Value :  6
#define 	D940C_SDRAMC_TXSR_7                    ((D940_REG) 0x7 << 28) //!< (SDRAMC) Value :  7
#define 	D940C_SDRAMC_TXSR_8                    ((D940_REG) 0x8 << 28) //!< (SDRAMC) Value :  8
#define 	D940C_SDRAMC_TXSR_9                    ((D940_REG) 0x9 << 28) //!< (SDRAMC) Value :  9
#define 	D940C_SDRAMC_TXSR_10                   ((D940_REG) 0xA << 28) //!< (SDRAMC) Value : 10
#define 	D940C_SDRAMC_TXSR_11                   ((D940_REG) 0xB << 28) //!< (SDRAMC) Value : 11
#define 	D940C_SDRAMC_TXSR_12                   ((D940_REG) 0xC << 28) //!< (SDRAMC) Value : 12
#define 	D940C_SDRAMC_TXSR_13                   ((D940_REG) 0xD << 28) //!< (SDRAMC) Value : 13
#define 	D940C_SDRAMC_TXSR_14                   ((D940_REG) 0xE << 28) //!< (SDRAMC) Value : 14
#define 	D940C_SDRAMC_TXSR_15                   ((D940_REG) 0xF << 28) //!< (SDRAMC) Value : 15
// -------- SDRAMC_HSR : (SDRAMC Offset: 0xc) SDRAM Controller High Speed Register -------- 
#define D940C_SDRAMC_DA       ((D940_REG) 0x1 <<  0) //!< (SDRAMC) Decode Cycle Enable Bit
#define 	D940C_SDRAMC_DA_DISABLE              ((D940_REG) 0x0) //!< (SDRAMC) Disable Decode Cycle
#define 	D940C_SDRAMC_DA_ENABLE               ((D940_REG) 0x1) //!< (SDRAMC) Enable Decode Cycle
// -------- SDRAMC_LPR : (SDRAMC Offset: 0x10) SDRAM Controller Low-power Register -------- 
#define D940C_SDRAMC_LPCB     ((D940_REG) 0x3 <<  0) //!< (SDRAMC) Low-power Configurations
#define 	D940C_SDRAMC_LPCB_DISABLE              ((D940_REG) 0x0) //!< (SDRAMC) Disable Low Power Features
#define 	D940C_SDRAMC_LPCB_SELF_REFRESH         ((D940_REG) 0x1) //!< (SDRAMC) Enable SELF_REFRESH
#define 	D940C_SDRAMC_LPCB_POWER_DOWN           ((D940_REG) 0x2) //!< (SDRAMC) Enable POWER_DOWN
#define 	D940C_SDRAMC_LPCB_DEEP_POWER_DOWN      ((D940_REG) 0x3) //!< (SDRAMC) Enable DEEP_POWER_DOWN
#define D940C_SDRAMC_PASR     ((D940_REG) 0x7 <<  4) //!< (SDRAMC) Partial Array Self Refresh (only for Low Power SDRAM)
#define D940C_SDRAMC_TCSR     ((D940_REG) 0x3 <<  8) //!< (SDRAMC) Temperature Compensated Self Refresh (only for Low Power SDRAM)
#define D940C_SDRAMC_DS       ((D940_REG) 0x3 << 10) //!< (SDRAMC) Drive Strenght (only for Low Power SDRAM)
#define D940C_SDRAMC_TIMEOUT  ((D940_REG) 0x3 << 12) //!< (SDRAMC) Time to define when Low Power Mode is enabled
#define 	D940C_SDRAMC_TIMEOUT_0_CLK_CYCLES         ((D940_REG) 0x0 << 12) //!< (SDRAMC) Activate SDRAM Low Power Mode Immediately
#define 	D940C_SDRAMC_TIMEOUT_64_CLK_CYCLES        ((D940_REG) 0x1 << 12) //!< (SDRAMC) Activate SDRAM Low Power Mode after 64 clock cycles after the end of the last transfer
#define 	D940C_SDRAMC_TIMEOUT_128_CLK_CYCLES       ((D940_REG) 0x2 << 12) //!< (SDRAMC) Activate SDRAM Low Power Mode after 64 clock cycles after the end of the last transfer
// -------- SDRAMC_IER : (SDRAMC Offset: 0x14) SDRAM Controller Interrupt Enable Register -------- 
#define D940C_SDRAMC_RES      ((D940_REG) 0x1 <<  0) //!< (SDRAMC) Refresh Error Status
// -------- SDRAMC_IDR : (SDRAMC Offset: 0x18) SDRAM Controller Interrupt Disable Register -------- 
// -------- SDRAMC_IMR : (SDRAMC Offset: 0x1c) SDRAM Controller Interrupt Mask Register -------- 
// -------- SDRAMC_ISR : (SDRAMC Offset: 0x20) SDRAM Controller Interrupt Status Register -------- 
// -------- SDRAMC_MDR : (SDRAMC Offset: 0x24) SDRAM Controller Memory Device Register -------- 
#define D940C_SDRAMC_MD       ((D940_REG) 0x3 <<  0) //!< (SDRAMC) Memory Device Type
#define 	D940C_SDRAMC_MD_SDRAM                ((D940_REG) 0x0) //!< (SDRAMC) SDRAM Mode
#define 	D940C_SDRAMC_MD_LOW_POWER_SDRAM      ((D940_REG) 0x1) //!< (SDRAMC) SDRAM Low Power Mode

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR Static Memory Controller Interface
// *****************************************************************************
typedef struct _D940S_SMC {
	D940_REG	 SMC_SETUP0; 	//!<  Setup Register for CS 0
	D940_REG	 SMC_PULSE0; 	//!<  Pulse Register for CS 0
	D940_REG	 SMC_CYCLE0; 	//!<  Cycle Register for CS 0
	D940_REG	 SMC_CTRL0; 	//!<  Control Register for CS 0
	D940_REG	 SMC_SETUP1; 	//!<  Setup Register for CS 1
	D940_REG	 SMC_PULSE1; 	//!<  Pulse Register for CS 1
	D940_REG	 SMC_CYCLE1; 	//!<  Cycle Register for CS 1
	D940_REG	 SMC_CTRL1; 	//!<  Control Register for CS 1
	D940_REG	 SMC_SETUP2; 	//!<  Setup Register for CS 2
	D940_REG	 SMC_PULSE2; 	//!<  Pulse Register for CS 2
	D940_REG	 SMC_CYCLE2; 	//!<  Cycle Register for CS 2
	D940_REG	 SMC_CTRL2; 	//!<  Control Register for CS 2
	D940_REG	 SMC_SETUP3; 	//!<  Setup Register for CS 3
	D940_REG	 SMC_PULSE3; 	//!<  Pulse Register for CS 3
	D940_REG	 SMC_CYCLE3; 	//!<  Cycle Register for CS 3
	D940_REG	 SMC_CTRL3; 	//!<  Control Register for CS 3
	D940_REG	 SMC_SETUP4; 	//!<  Setup Register for CS 4
	D940_REG	 SMC_PULSE4; 	//!<  Pulse Register for CS 4
	D940_REG	 SMC_CYCLE4; 	//!<  Cycle Register for CS 4
	D940_REG	 SMC_CTRL4; 	//!<  Control Register for CS 4
	D940_REG	 SMC_SETUP5; 	//!<  Setup Register for CS 5
	D940_REG	 SMC_PULSE5; 	//!<  Pulse Register for CS 5
	D940_REG	 SMC_CYCLE5; 	//!<  Cycle Register for CS 5
	D940_REG	 SMC_CTRL5; 	//!<  Control Register for CS 5
	D940_REG	 SMC_SETUP6; 	//!<  Setup Register for CS 6
	D940_REG	 SMC_PULSE6; 	//!<  Pulse Register for CS 6
	D940_REG	 SMC_CYCLE6; 	//!<  Cycle Register for CS 6
	D940_REG	 SMC_CTRL6; 	//!<  Control Register for CS 6
	D940_REG	 SMC_SETUP7; 	//!<  Setup Register for CS 7
	D940_REG	 SMC_PULSE7; 	//!<  Pulse Register for CS 7
	D940_REG	 SMC_CYCLE7; 	//!<  Cycle Register for CS 7
	D940_REG	 SMC_CTRL7; 	//!<  Control Register for CS 7
} D940S_SMC,   __MAGICV_EXTERNAL *D940PS_SMC;

// -------- SMC_SETUP : (SMC Offset: 0x0) Setup Register for CS x -------- 
#define D940C_SMC_NWESETUP    ((D940_REG) 0x3F <<  0) //!< (SMC) NWE Setup Length
#define D940C_SMC_NCSSETUPWR  ((D940_REG) 0x3F <<  8) //!< (SMC) NCS Setup Length in WRite Access
#define D940C_SMC_NRDSETUP    ((D940_REG) 0x3F << 16) //!< (SMC) NRD Setup Length
#define D940C_SMC_NCSSETUPRD  ((D940_REG) 0x3F << 24) //!< (SMC) NCS Setup Length in ReaD Access
// -------- SMC_PULSE : (SMC Offset: 0x4) Pulse Register for CS x -------- 
#define D940C_SMC_NWEPULSE    ((D940_REG) 0x7F <<  0) //!< (SMC) NWE Pulse Length
#define D940C_SMC_NCSPULSEWR  ((D940_REG) 0x7F <<  8) //!< (SMC) NCS Pulse Length in WRite Access
#define D940C_SMC_NRDPULSE    ((D940_REG) 0x7F << 16) //!< (SMC) NRD Pulse Length
#define D940C_SMC_NCSPULSERD  ((D940_REG) 0x7F << 24) //!< (SMC) NCS Pulse Length in ReaD Access
// -------- SMC_CYC : (SMC Offset: 0x8) Cycle Register for CS x -------- 
#define D940C_SMC_NWECYCLE    ((D940_REG) 0x1FF <<  0) //!< (SMC) Total Write Cycle Length
#define D940C_SMC_NRDCYCLE    ((D940_REG) 0x1FF << 16) //!< (SMC) Total Read Cycle Length
// -------- SMC_CTRL : (SMC Offset: 0xc) Control Register for CS x -------- 
#define D940C_SMC_READMODE    ((D940_REG) 0x1 <<  0) //!< (SMC) Read Mode
#define D940C_SMC_WRITEMODE   ((D940_REG) 0x1 <<  1) //!< (SMC) Write Mode
#define D940C_SMC_NWAITM      ((D940_REG) 0x3 <<  5) //!< (SMC) NWAIT Mode
#define 	D940C_SMC_NWAITM_NWAIT_DISABLE        ((D940_REG) 0x0 <<  5) //!< (SMC) External NWAIT disabled.
#define 	D940C_SMC_NWAITM_NWAIT_ENABLE_FROZEN  ((D940_REG) 0x2 <<  5) //!< (SMC) External NWAIT enabled in frozen mode.
#define 	D940C_SMC_NWAITM_NWAIT_ENABLE_READY   ((D940_REG) 0x3 <<  5) //!< (SMC) External NWAIT enabled in ready mode.
#define D940C_SMC_BAT         ((D940_REG) 0x1 <<  8) //!< (SMC) Byte Access Type
#define 	D940C_SMC_BAT_BYTE_SELECT          ((D940_REG) 0x0 <<  8) //!< (SMC) Write controled by ncs, nbs0, nbs1, nbs2, nbs3. Read controled by ncs, nrd, nbs0, nbs1, nbs2, nbs3.
#define 	D940C_SMC_BAT_BYTE_WRITE           ((D940_REG) 0x1 <<  8) //!< (SMC) Write controled by ncs, nwe0, nwe1, nwe2, nwe3. Read controled by ncs and nrd.
#define D940C_SMC_DBW         ((D940_REG) 0x3 << 12) //!< (SMC) Data Bus Width
#define 	D940C_SMC_DBW_WIDTH_EIGTH_BITS     ((D940_REG) 0x0 << 12) //!< (SMC) 8 bits.
#define 	D940C_SMC_DBW_WIDTH_SIXTEEN_BITS   ((D940_REG) 0x1 << 12) //!< (SMC) 16 bits.
#define 	D940C_SMC_DBW_WIDTH_THIRTY_TWO_BITS ((D940_REG) 0x2 << 12) //!< (SMC) 32 bits.
#define D940C_SMC_TDF         ((D940_REG) 0xF << 16) //!< (SMC) Data Float Time.
#define D940C_SMC_TDFEN       ((D940_REG) 0x1 << 20) //!< (SMC) TDF Enabled.
#define D940C_SMC_PMEN        ((D940_REG) 0x1 << 24) //!< (SMC) Page Mode Enabled.
#define D940C_SMC_PS          ((D940_REG) 0x3 << 28) //!< (SMC) Page Size
#define 	D940C_SMC_PS_SIZE_FOUR_BYTES      ((D940_REG) 0x0 << 28) //!< (SMC) 4 bytes.
#define 	D940C_SMC_PS_SIZE_EIGHT_BYTES     ((D940_REG) 0x1 << 28) //!< (SMC) 8 bytes.
#define 	D940C_SMC_PS_SIZE_SIXTEEN_BYTES   ((D940_REG) 0x2 << 28) //!< (SMC) 16 bytes.
#define 	D940C_SMC_PS_SIZE_THIRTY_TWO_BYTES ((D940_REG) 0x3 << 28) //!< (SMC) 32 bytes.
// -------- SMC_SETUP : (SMC Offset: 0x10) Setup Register for CS x -------- 
// -------- SMC_PULSE : (SMC Offset: 0x14) Pulse Register for CS x -------- 
// -------- SMC_CYC : (SMC Offset: 0x18) Cycle Register for CS x -------- 
// -------- SMC_CTRL : (SMC Offset: 0x1c) Control Register for CS x -------- 
// -------- SMC_SETUP : (SMC Offset: 0x20) Setup Register for CS x -------- 
// -------- SMC_PULSE : (SMC Offset: 0x24) Pulse Register for CS x -------- 
// -------- SMC_CYC : (SMC Offset: 0x28) Cycle Register for CS x -------- 
// -------- SMC_CTRL : (SMC Offset: 0x2c) Control Register for CS x -------- 
// -------- SMC_SETUP : (SMC Offset: 0x30) Setup Register for CS x -------- 
// -------- SMC_PULSE : (SMC Offset: 0x34) Pulse Register for CS x -------- 
// -------- SMC_CYC : (SMC Offset: 0x38) Cycle Register for CS x -------- 
// -------- SMC_CTRL : (SMC Offset: 0x3c) Control Register for CS x -------- 
// -------- SMC_SETUP : (SMC Offset: 0x40) Setup Register for CS x -------- 
// -------- SMC_PULSE : (SMC Offset: 0x44) Pulse Register for CS x -------- 
// -------- SMC_CYC : (SMC Offset: 0x48) Cycle Register for CS x -------- 
// -------- SMC_CTRL : (SMC Offset: 0x4c) Control Register for CS x -------- 
// -------- SMC_SETUP : (SMC Offset: 0x50) Setup Register for CS x -------- 
// -------- SMC_PULSE : (SMC Offset: 0x54) Pulse Register for CS x -------- 
// -------- SMC_CYC : (SMC Offset: 0x58) Cycle Register for CS x -------- 
// -------- SMC_CTRL : (SMC Offset: 0x5c) Control Register for CS x -------- 
// -------- SMC_SETUP : (SMC Offset: 0x60) Setup Register for CS x -------- 
// -------- SMC_PULSE : (SMC Offset: 0x64) Pulse Register for CS x -------- 
// -------- SMC_CYC : (SMC Offset: 0x68) Cycle Register for CS x -------- 
// -------- SMC_CTRL : (SMC Offset: 0x6c) Control Register for CS x -------- 
// -------- SMC_SETUP : (SMC Offset: 0x70) Setup Register for CS x -------- 
// -------- SMC_PULSE : (SMC Offset: 0x74) Pulse Register for CS x -------- 
// -------- SMC_CYC : (SMC Offset: 0x78) Cycle Register for CS x -------- 
// -------- SMC_CTRL : (SMC Offset: 0x7c) Control Register for CS x -------- 

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR AHB Matrix Interface
// *****************************************************************************
typedef struct _D940S_MATRIX {
	D940_REG	 MATRIX_MCFG0; 	//!<  Master Configuration Register 0 
	D940_REG	 MATRIX_MCFG1; 	//!<  Master Configuration Register 1 
	D940_REG	 MATRIX_MCFG2; 	//!<  Master Configuration Register 2 
	D940_REG	 MATRIX_MCFG3; 	//!<  Master Configuration Register 3 
	D940_REG	 MATRIX_MCFG4; 	//!<  Master Configuration Register 4 
	D940_REG	 MATRIX_MCFG5; 	//!<  Master Configuration Register 5 
	D940_REG	 Reserved0[10]; 	//!< 
	D940_REG	 MATRIX_SCFG0; 	//!<  Slave Configuration Register 0 
	D940_REG	 MATRIX_SCFG1; 	//!<  Slave Configuration Register 1 
	D940_REG	 MATRIX_SCFG2; 	//!<  Slave Configuration Register 2 
	D940_REG	 MATRIX_SCFG3; 	//!<  Slave Configuration Register 3 
	D940_REG	 MATRIX_SCFG4; 	//!<  Slave Configuration Register 4 
	D940_REG	 Reserved1[11]; 	//!< 
	D940_REG	 MATRIX_PRAS0; 	//!<  PRAS0 
	D940_REG	 Reserved2[1]; 	//!< 
	D940_REG	 MATRIX_PRAS1; 	//!<  PRAS1 
	D940_REG	 Reserved3[1]; 	//!< 
	D940_REG	 MATRIX_PRAS2; 	//!<  PRAS2 
	D940_REG	 Reserved4[1]; 	//!< 
	D940_REG	 MATRIX_PRAS3; 	//!<  PRAS3 
	D940_REG	 Reserved5[1]; 	//!< 
	D940_REG	 MATRIX_PRAS4; 	//!<  PRAS4 
	D940_REG	 Reserved6[23]; 	//!< 
	D940_REG	 MATRIX_MRCR; 	//!<  Master Remp Control Register 
} D940S_MATRIX, __MAGICV_EXTERNAL *D940PS_MATRIX;

// -------- MATRIX_MCFG0 : (MATRIX Offset: 0x0) Master Configuration Register 0 -------- 
#define D940C_MATRIX_ULBT     ((D940_REG) 0x7 <<  0) //!< (MATRIX) Undefined Length Burst Type
// -------- MATRIX_MCFG1 : (MATRIX Offset: 0x4) Master Configuration Register 1 -------- 
// -------- MATRIX_MCFG2 : (MATRIX Offset: 0x8) Master Configuration Register 2 -------- 
// -------- MATRIX_MCFG3 : (MATRIX Offset: 0xc) Master Configuration Register 3 -------- 
// -------- MATRIX_MCFG4 : (MATRIX Offset: 0x10) Master Configuration Register 4 -------- 
// -------- MATRIX_MCFG5 : (MATRIX Offset: 0x14) Master Configuration Register 5 -------- 
// -------- MATRIX_SCFG0 : (MATRIX Offset: 0x40) Slave Configuration Register 0 -------- 
#define D940C_MATRIX_SLOT_CYCLE ((D940_REG) 0xFF <<  0) //!< (MATRIX) Maximum Number of Allowed Cycles for a Burst
#define D940C_MATRIX_DEFMSTR_TYPE ((D940_REG) 0x3 << 16) //!< (MATRIX) Default Master Type
#define 	D940C_MATRIX_DEFMSTR_TYPE_NO_DEFMSTR           ((D940_REG) 0x0 << 16) //!< (MATRIX) No Default Master. At the end of current slave access, if no other master request is pending, the slave is deconnected from all masters. This results in having a one cycle latency for the first transfer of a burst.
#define 	D940C_MATRIX_DEFMSTR_TYPE_LAST_DEFMSTR         ((D940_REG) 0x1 << 16) //!< (MATRIX) Last Default Master. At the end of current slave access, if no other master request is pending, the slave stay connected with the last master having accessed it. This results in not having the one cycle latency when the last master re-trying access on the slave.
#define 	D940C_MATRIX_DEFMSTR_TYPE_FIXED_DEFMSTR        ((D940_REG) 0x2 << 16) //!< (MATRIX) Fixed Default Master. At the end of current slave access, if no other master request is pending, the slave connects with fixed which number is in FIXED_DEFMSTR field. This results in not having the one cycle latency when the fixed master re-trying access on the slave.
#define D940C_MATRIX_FIXED_DEFMSTR0 ((D940_REG) 0x7 << 18) //!< (MATRIX) Fixed Index of Default Master
#define 	D940C_MATRIX_FIXED_DEFMSTR0_ARM926I              ((D940_REG) 0x0 << 18) //!< (MATRIX) ARM926EJ-S Instruction Master is Default Master
#define 	D940C_MATRIX_FIXED_DEFMSTR0_ARM926D              ((D940_REG) 0x1 << 18) //!< (MATRIX) ARM926EJ-S Data Master is Default Master
#define 	D940C_MATRIX_FIXED_DEFMSTR0_PDC                  ((D940_REG) 0x2 << 18) //!< (MATRIX) PDC Master is Default Master
#define 	D940C_MATRIX_FIXED_DEFMSTR0_ISI                  ((D940_REG) 0x3 << 18) //!< (MATRIX) ISI Master is Default Master
#define 	D940C_MATRIX_FIXED_DEFMSTR0_EMAC                 ((D940_REG) 0x4 << 18) //!< (MATRIX) EMAC Master is Default Master
#define 	D940C_MATRIX_FIXED_DEFMSTR0_USB                  ((D940_REG) 0x5 << 18) //!< (MATRIX) USB Master is Default Master
#define D940C_MATRIX_ARBT     ((D940_REG) 0x3 << 24) //!< (MATRIX) Arbitration Type
// -------- MATRIX_SCFG1 : (MATRIX Offset: 0x44) Slave Configuration Register 1 -------- 
#define D940C_MATRIX_FIXED_DEFMSTR1 ((D940_REG) 0x7 << 18) //!< (MATRIX) Fixed Index of Default Master
#define 	D940C_MATRIX_FIXED_DEFMSTR1_ARM926I              ((D940_REG) 0x0 << 18) //!< (MATRIX) ARM926EJ-S Instruction Master is Default Master
#define 	D940C_MATRIX_FIXED_DEFMSTR1_ARM926D              ((D940_REG) 0x1 << 18) //!< (MATRIX) ARM926EJ-S Data Master is Default Master
#define 	D940C_MATRIX_FIXED_DEFMSTR1_PDC                  ((D940_REG) 0x2 << 18) //!< (MATRIX) PDC Master is Default Master
#define 	D940C_MATRIX_FIXED_DEFMSTR1_ISI                  ((D940_REG) 0x3 << 18) //!< (MATRIX) ISI Master is Default Master
#define 	D940C_MATRIX_FIXED_DEFMSTR1_EMAC                 ((D940_REG) 0x4 << 18) //!< (MATRIX) EMAC Master is Default Master
#define 	D940C_MATRIX_FIXED_DEFMSTR1_USB                  ((D940_REG) 0x5 << 18) //!< (MATRIX) USB Master is Default Master
// -------- MATRIX_SCFG2 : (MATRIX Offset: 0x48) Slave Configuration Register 2 -------- 
#define D940C_MATRIX_FIXED_DEFMSTR2 ((D940_REG) 0x1 << 18) //!< (MATRIX) Fixed Index of Default Master
#define 	D940C_MATRIX_FIXED_DEFMSTR2_ARM926I              ((D940_REG) 0x0 << 18) //!< (MATRIX) ARM926EJ-S Instruction Master is Default Master
#define 	D940C_MATRIX_FIXED_DEFMSTR2_ARM926D              ((D940_REG) 0x1 << 18) //!< (MATRIX) ARM926EJ-S Data Master is Default Master
#define 	D940C_MATRIX_FIXED_DEFMSTR2_PDC                  ((D940_REG) 0x2 << 18) //!< (MATRIX) PDC Master is Default Master
#define 	D940C_MATRIX_FIXED_DEFMSTR2_USB                  ((D940_REG) 0x5 << 18) //!< (MATRIX) USB Master is Default Master
// -------- MATRIX_SCFG3 : (MATRIX Offset: 0x4c) Slave Configuration Register 3 -------- 
#define D940C_MATRIX_FIXED_DEFMSTR3 ((D940_REG) 0x7 << 18) //!< (MATRIX) Fixed Index of Default Master
#define 	D940C_MATRIX_FIXED_DEFMSTR3_ARM926I              ((D940_REG) 0x0 << 18) //!< (MATRIX) ARM926EJ-S Instruction Master is Default Master
#define 	D940C_MATRIX_FIXED_DEFMSTR3_ARM926D              ((D940_REG) 0x1 << 18) //!< (MATRIX) ARM926EJ-S Data Master is Default Master
#define 	D940C_MATRIX_FIXED_DEFMSTR3_PDC                  ((D940_REG) 0x2 << 18) //!< (MATRIX) PDC Master is Default Master
#define 	D940C_MATRIX_FIXED_DEFMSTR3_ISI                  ((D940_REG) 0x3 << 18) //!< (MATRIX) ISI Master is Default Master
#define 	D940C_MATRIX_FIXED_DEFMSTR3_EMAC                 ((D940_REG) 0x4 << 18) //!< (MATRIX) EMAC Master is Default Master
#define 	D940C_MATRIX_FIXED_DEFMSTR3_USB                  ((D940_REG) 0x5 << 18) //!< (MATRIX) USB Master is Default Master
// -------- MATRIX_SCFG4 : (MATRIX Offset: 0x50) Slave Configuration Register 4 -------- 
#define D940C_MATRIX_FIXED_DEFMSTR4 ((D940_REG) 0x3 << 18) //!< (MATRIX) Fixed Index of Default Master
#define 	D940C_MATRIX_FIXED_DEFMSTR4_ARM926I              ((D940_REG) 0x0 << 18) //!< (MATRIX) ARM926EJ-S Instruction Master is Default Master
#define 	D940C_MATRIX_FIXED_DEFMSTR4_ARM926D              ((D940_REG) 0x1 << 18) //!< (MATRIX) ARM926EJ-S Data Master is Default Master
#define 	D940C_MATRIX_FIXED_DEFMSTR4_PDC                  ((D940_REG) 0x2 << 18) //!< (MATRIX) PDC Master is Default Master
#define 	D940C_MATRIX_FIXED_DEFMSTR4_USB                  ((D940_REG) 0x5 << 18) //!< (MATRIX) USB Master is Default Master
// -------- MATRIX_PRAS0 : (MATRIX Offset: 0x80) PRAS0 Register -------- 
#define D940C_MATRIX_M0PR     ((D940_REG) 0x3 <<  0) //!< (MATRIX) ARM926EJ-S Instruction priority
#define D940C_MATRIX_M1PR     ((D940_REG) 0x3 <<  4) //!< (MATRIX) ARM926EJ-S Data priority
#define D940C_MATRIX_M2PR     ((D940_REG) 0x3 <<  8) //!< (MATRIX) PDC priority
#define D940C_MATRIX_M3PR     ((D940_REG) 0x3 << 12) //!< (MATRIX) ISI priority
#define D940C_MATRIX_M4PR     ((D940_REG) 0x3 << 16) //!< (MATRIX) EMAC priority
#define D940C_MATRIX_M5PR     ((D940_REG) 0x3 << 20) //!< (MATRIX) USB priority
// -------- MATRIX_PRAS1 : (MATRIX Offset: 0x88) PRAS1 Register -------- 
// -------- MATRIX_PRAS2 : (MATRIX Offset: 0x90) PRAS2 Register -------- 
// -------- MATRIX_PRAS3 : (MATRIX Offset: 0x98) PRAS3 Register -------- 
// -------- MATRIX_PRAS4 : (MATRIX Offset: 0xa0) PRAS4 Register -------- 
// -------- MATRIX_MRCR : (MATRIX Offset: 0x100) MRCR Register -------- 
#define D940C_MATRIX_RCA926I  ((D940_REG) 0x1 <<  0) //!< (MATRIX) Remap Command Bit for ARM926EJ-S Instruction
#define D940C_MATRIX_RCA926D  ((D940_REG) 0x1 <<  1) //!< (MATRIX) Remap Command Bit for ARM926EJ-S Data
#define D940C_MATRIX_RCB2     ((D940_REG) 0x1 <<  2) //!< (MATRIX) Remap Command Bit for PDC
#define D940C_MATRIX_RCB3     ((D940_REG) 0x1 <<  3) //!< (MATRIX) Remap Command Bit for ISI
#define D940C_MATRIX_RCB4     ((D940_REG) 0x1 <<  4) //!< (MATRIX) Remap Command Bit for EMAC
#define D940C_MATRIX_RCB5     ((D940_REG) 0x1 <<  5) //!< (MATRIX) Remap Command Bit for USB

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR Advanced Interrupt Controller
// *****************************************************************************
typedef struct _D940S_AIC {
	D940_REG	 AIC_SMR[32]; 	//!< Source Mode Register
	D940_REG	 AIC_SVR[32]; 	//!< Source Vector Register
	D940_REG	 AIC_IVR; 	//!< IRQ Vector Register
	D940_REG	 AIC_FVR; 	//!< FIQ Vector Register
	D940_REG	 AIC_ISR; 	//!< Interrupt Status Register
	D940_REG	 AIC_IPR; 	//!< Interrupt Pending Register
	D940_REG	 AIC_IMR; 	//!< Interrupt Mask Register
	D940_REG	 AIC_CISR; 	//!< Core Interrupt Status Register
	D940_REG	 Reserved0[2]; 	//!< 
	D940_REG	 AIC_IECR; 	//!< Interrupt Enable Command Register
	D940_REG	 AIC_IDCR; 	//!< Interrupt Disable Command Register
	D940_REG	 AIC_ICCR; 	//!< Interrupt Clear Command Register
	D940_REG	 AIC_ISCR; 	//!< Interrupt Set Command Register
	D940_REG	 AIC_EOICR; 	//!< End of Interrupt Command Register
	D940_REG	 AIC_SPU; 	//!< Spurious Vector Register
	D940_REG	 AIC_DCR; 	//!< Debug Control Register (Protect)
	D940_REG	 Reserved1[1]; 	//!< 
	D940_REG	 AIC_FFER; 	//!< Fast Forcing Enable Register
	D940_REG	 AIC_FFDR; 	//!< Fast Forcing Disable Register
	D940_REG	 AIC_FFSR; 	//!< Fast Forcing Status Register
} D940S_AIC,   __MAGICV_EXTERNAL *D940PS_AIC;

// -------- AIC_SMR : (AIC Offset: 0x0) Control Register -------- 
#define D940C_AIC_PRIOR       ((D940_REG) 0x7 <<  0) //!< (AIC) Priority Level
#define 	D940C_AIC_PRIOR_LOWEST               ((D940_REG) 0x0) //!< (AIC) Lowest priority level
#define 	D940C_AIC_PRIOR_HIGHEST              ((D940_REG) 0x7) //!< (AIC) Highest priority level
#define D940C_AIC_SRCTYPE     ((D940_REG) 0x3 <<  5) //!< (AIC) Interrupt Source Type
#define 	D940C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE  ((D940_REG) 0x0 <<  5) //!< (AIC) Internal Sources Code Label Level Sensitive
#define 	D940C_AIC_SRCTYPE_INT_EDGE_TRIGGERED   ((D940_REG) 0x1 <<  5) //!< (AIC) Internal Sources Code Label Edge triggered
#define 	D940C_AIC_SRCTYPE_EXT_HIGH_LEVEL       ((D940_REG) 0x2 <<  5) //!< (AIC) External Sources Code Label High-level Sensitive
#define 	D940C_AIC_SRCTYPE_EXT_POSITIVE_EDGE    ((D940_REG) 0x3 <<  5) //!< (AIC) External Sources Code Label Positive Edge triggered
// -------- AIC_CISR : (AIC Offset: 0x114) AIC Core Interrupt Status Register -------- 
#define D940C_AIC_NFIQ        ((D940_REG) 0x1 <<  0) //!< (AIC) NFIQ Status
#define D940C_AIC_NIRQ        ((D940_REG) 0x1 <<  1) //!< (AIC) NIRQ Status
// -------- AIC_DCR : (AIC Offset: 0x138) AIC Debug Control Register (Protect) -------- 
#define D940C_AIC_DCR_PROT    ((D940_REG) 0x1 <<  0) //!< (AIC) Protection Mode
#define D940C_AIC_DCR_GMSK    ((D940_REG) 0x1 <<  1) //!< (AIC) General Mask

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR Peripheral DMA Controller
// *****************************************************************************
typedef struct _D940S_PDC {
	D940_REG	 PDC_RPR; 	//!< Receive Pointer Register
	D940_REG	 PDC_RCR; 	//!< Receive Counter Register
	D940_REG	 PDC_TPR; 	//!< Transmit Pointer Register
	D940_REG	 PDC_TCR; 	//!< Transmit Counter Register
	D940_REG	 PDC_RNPR; 	//!< Receive Next Pointer Register
	D940_REG	 PDC_RNCR; 	//!< Receive Next Counter Register
	D940_REG	 PDC_TNPR; 	//!< Transmit Next Pointer Register
	D940_REG	 PDC_TNCR; 	//!< Transmit Next Counter Register
	D940_REG	 PDC_PTCR; 	//!< PDC Transfer Control Register
	D940_REG	 PDC_PTSR; 	//!< PDC Transfer Status Register
} D940S_PDC,   __MAGICV_EXTERNAL *D940PS_PDC;

// -------- PDC_PTCR : (PDC Offset: 0x20) PDC Transfer Control Register -------- 
#define D940C_PDC_RXTEN       ((D940_REG) 0x1 <<  0) //!< (PDC) Receiver Transfer Enable
#define D940C_PDC_RXTDIS      ((D940_REG) 0x1 <<  1) //!< (PDC) Receiver Transfer Disable
#define D940C_PDC_TXTEN       ((D940_REG) 0x1 <<  8) //!< (PDC) Transmitter Transfer Enable
#define D940C_PDC_TXTDIS      ((D940_REG) 0x1 <<  9) //!< (PDC) Transmitter Transfer Disable
// -------- PDC_PTSR : (PDC Offset: 0x24) PDC Transfer Status Register -------- 

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR Debug Unit
// *****************************************************************************
typedef struct _D940S_DBGU {
	D940_REG	 DBGU_CR; 	//!< Control Register
	D940_REG	 DBGU_MR; 	//!< Mode Register
	D940_REG	 DBGU_IER; 	//!< Interrupt Enable Register
	D940_REG	 DBGU_IDR; 	//!< Interrupt Disable Register
	D940_REG	 DBGU_IMR; 	//!< Interrupt Mask Register
	D940_REG	 DBGU_CSR; 	//!< Channel Status Register
	D940_REG	 DBGU_RHR; 	//!< Receiver Holding Register
	D940_REG	 DBGU_THR; 	//!< Transmitter Holding Register
	D940_REG	 DBGU_BRGR; 	//!< Baud Rate Generator Register
	D940_REG	 Reserved0[7]; 	//!< 
	D940_REG	 DBGU_CIDR; 	//!< Chip ID Register
	D940_REG	 DBGU_EXID; 	//!< Chip ID Extension Register
	D940_REG	 DBGU_FNTR; 	//!< Force NTRST Register
	D940_REG	 Reserved1[45]; 	//!< 
	D940_REG	 DBGU_RPR; 	//!< Receive Pointer Register
	D940_REG	 DBGU_RCR; 	//!< Receive Counter Register
	D940_REG	 DBGU_TPR; 	//!< Transmit Pointer Register
	D940_REG	 DBGU_TCR; 	//!< Transmit Counter Register
	D940_REG	 DBGU_RNPR; 	//!< Receive Next Pointer Register
	D940_REG	 DBGU_RNCR; 	//!< Receive Next Counter Register
	D940_REG	 DBGU_TNPR; 	//!< Transmit Next Pointer Register
	D940_REG	 DBGU_TNCR; 	//!< Transmit Next Counter Register
	D940_REG	 DBGU_PTCR; 	//!< PDC Transfer Control Register
	D940_REG	 DBGU_PTSR; 	//!< PDC Transfer Status Register
} D940S_DBGU,   __MAGICV_EXTERNAL *D940PS_DBGU;

// -------- DBGU_CR : (DBGU Offset: 0x0) Debug Unit Control Register -------- 
#define D940C_US_RSTRX        ((D940_REG) 0x1 <<  2) //!< (DBGU) Reset Receiver
#define D940C_US_RSTTX        ((D940_REG) 0x1 <<  3) //!< (DBGU) Reset Transmitter
#define D940C_US_RXEN         ((D940_REG) 0x1 <<  4) //!< (DBGU) Receiver Enable
#define D940C_US_RXDIS        ((D940_REG) 0x1 <<  5) //!< (DBGU) Receiver Disable
#define D940C_US_TXEN         ((D940_REG) 0x1 <<  6) //!< (DBGU) Transmitter Enable
#define D940C_US_TXDIS        ((D940_REG) 0x1 <<  7) //!< (DBGU) Transmitter Disable
#define D940C_US_RSTSTA       ((D940_REG) 0x1 <<  8) //!< (DBGU) Reset Status Bits
// -------- DBGU_MR : (DBGU Offset: 0x4) Debug Unit Mode Register -------- 
#define D940C_US_PAR          ((D940_REG) 0x7 <<  9) //!< (DBGU) Parity type
#define 	D940C_US_PAR_EVEN                 ((D940_REG) 0x0 <<  9) //!< (DBGU) Even Parity
#define 	D940C_US_PAR_ODD                  ((D940_REG) 0x1 <<  9) //!< (DBGU) Odd Parity
#define 	D940C_US_PAR_SPACE                ((D940_REG) 0x2 <<  9) //!< (DBGU) Parity forced to 0 (Space)
#define 	D940C_US_PAR_MARK                 ((D940_REG) 0x3 <<  9) //!< (DBGU) Parity forced to 1 (Mark)
#define 	D940C_US_PAR_NONE                 ((D940_REG) 0x4 <<  9) //!< (DBGU) No Parity
#define 	D940C_US_PAR_MULTI_DROP           ((D940_REG) 0x6 <<  9) //!< (DBGU) Multi-drop mode
#define D940C_US_CHMODE       ((D940_REG) 0x3 << 14) //!< (DBGU) Channel Mode
#define 	D940C_US_CHMODE_NORMAL               ((D940_REG) 0x0 << 14) //!< (DBGU) Normal Mode: The USART channel operates as an RX/TX USART.
#define 	D940C_US_CHMODE_AUTO                 ((D940_REG) 0x1 << 14) //!< (DBGU) Automatic Echo: Receiver Data Input is connected to the TXD pin.
#define 	D940C_US_CHMODE_LOCAL                ((D940_REG) 0x2 << 14) //!< (DBGU) Local Loopback: Transmitter Output Signal is connected to Receiver Input Signal.
#define 	D940C_US_CHMODE_REMOTE               ((D940_REG) 0x3 << 14) //!< (DBGU) Remote Loopback: RXD pin is internally connected to TXD pin.
// -------- DBGU_IER : (DBGU Offset: 0x8) Debug Unit Interrupt Enable Register -------- 
#define D940C_US_RXRDY        ((D940_REG) 0x1 <<  0) //!< (DBGU) RXRDY Interrupt
#define D940C_US_TXRDY        ((D940_REG) 0x1 <<  1) //!< (DBGU) TXRDY Interrupt
#define D940C_US_ENDRX        ((D940_REG) 0x1 <<  3) //!< (DBGU) End of Receive Transfer Interrupt
#define D940C_US_ENDTX        ((D940_REG) 0x1 <<  4) //!< (DBGU) End of Transmit Interrupt
#define D940C_US_OVRE         ((D940_REG) 0x1 <<  5) //!< (DBGU) Overrun Interrupt
#define D940C_US_FRAME        ((D940_REG) 0x1 <<  6) //!< (DBGU) Framing Error Interrupt
#define D940C_US_PARE         ((D940_REG) 0x1 <<  7) //!< (DBGU) Parity Error Interrupt
#define D940C_US_TXEMPTY      ((D940_REG) 0x1 <<  9) //!< (DBGU) TXEMPTY Interrupt
#define D940C_US_TXBUFE       ((D940_REG) 0x1 << 11) //!< (DBGU) TXBUFE Interrupt
#define D940C_US_RXBUFF       ((D940_REG) 0x1 << 12) //!< (DBGU) RXBUFF Interrupt
#define D940C_US_COMM_TX      ((D940_REG) 0x1 << 30) //!< (DBGU) COMM_TX Interrupt
#define D940C_US_COMM_RX      ((D940_REG) 0x1 << 31) //!< (DBGU) COMM_RX Interrupt
// -------- DBGU_IDR : (DBGU Offset: 0xc) Debug Unit Interrupt Disable Register -------- 
// -------- DBGU_IMR : (DBGU Offset: 0x10) Debug Unit Interrupt Mask Register -------- 
// -------- DBGU_CSR : (DBGU Offset: 0x14) Debug Unit Channel Status Register -------- 
// -------- DBGU_FNTR : (DBGU Offset: 0x48) Debug Unit FORCE_NTRST Register -------- 
#define D940C_US_FORCE_NTRST  ((D940_REG) 0x1 <<  0) //!< (DBGU) Force NTRST in JTAG

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR Parallel Input Output Controler
// *****************************************************************************
typedef struct _D940S_PIO {
	D940_REG	 PIO_PER; 	//!< PIO Enable Register
	D940_REG	 PIO_PDR; 	//!< PIO Disable Register
	D940_REG	 PIO_PSR; 	//!< PIO Status Register
	D940_REG	 Reserved0[1]; 	//!< 
	D940_REG	 PIO_OER; 	//!< Output Enable Register
	D940_REG	 PIO_ODR; 	//!< Output Disable Registerr
	D940_REG	 PIO_OSR; 	//!< Output Status Register
	D940_REG	 Reserved1[1]; 	//!< 
	D940_REG	 PIO_IFER; 	//!< Input Filter Enable Register
	D940_REG	 PIO_IFDR; 	//!< Input Filter Disable Register
	D940_REG	 PIO_IFSR; 	//!< Input Filter Status Register
	D940_REG	 Reserved2[1]; 	//!< 
	D940_REG	 PIO_SODR; 	//!< Set Output Data Register
	D940_REG	 PIO_CODR; 	//!< Clear Output Data Register
	D940_REG	 PIO_ODSR; 	//!< Output Data Status Register
	D940_REG	 PIO_PDSR; 	//!< Pin Data Status Register
	D940_REG	 PIO_IER; 	//!< Interrupt Enable Register
	D940_REG	 PIO_IDR; 	//!< Interrupt Disable Register
	D940_REG	 PIO_IMR; 	//!< Interrupt Mask Register
	D940_REG	 PIO_ISR; 	//!< Interrupt Status Register
	D940_REG	 PIO_MDER; 	//!< Multi-driver Enable Register
	D940_REG	 PIO_MDDR; 	//!< Multi-driver Disable Register
	D940_REG	 PIO_MDSR; 	//!< Multi-driver Status Register
	D940_REG	 Reserved3[1]; 	//!< 
	D940_REG	 PIO_PPUDR; 	//!< Pull-up Disable Register
	D940_REG	 PIO_PPUER; 	//!< Pull-up Enable Register
	D940_REG	 PIO_PPUSR; 	//!< Pull-up Status Register
	D940_REG	 Reserved4[1]; 	//!< 
	D940_REG	 PIO_ASR; 	//!< Select A Register
	D940_REG	 PIO_BSR; 	//!< Select B Register
	D940_REG	 PIO_ABSR; 	//!< AB Select Status Register
	D940_REG	 Reserved5[9]; 	//!< 
	D940_REG	 PIO_OWER; 	//!< Output Write Enable Register
	D940_REG	 PIO_OWDR; 	//!< Output Write Disable Register
	D940_REG	 PIO_OWSR; 	//!< Output Write Status Register
} D940S_PIO,   __MAGICV_EXTERNAL *D940PS_PIO;


// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR Clock Generator Controler
// *****************************************************************************
typedef struct _D940S_CKGR {
	D940_REG	 CKGR_MOR; 	//!< Main Oscillator Register
	D940_REG	 CKGR_MCFR; 	//!< Main Clock  Frequency Register
	D940_REG	 CKGR_PLLAR; 	//!< PLL A Register
	D940_REG	 CKGR_PLLBR; 	//!< PLL B Register
} D940S_CKGR,   __MAGICV_EXTERNAL *D940PS_CKGR;

// -------- CKGR_MOR : (CKGR Offset: 0x0) Main Oscillator Register -------- 
#define D940C_CKGR_MOSCEN     ((D940_REG) 0x1 <<  0) //!< (CKGR) Main Oscillator Enable
#define D940C_CKGR_OSCBYPASS  ((D940_REG) 0x1 <<  1) //!< (CKGR) Main Oscillator Bypass
#define D940C_CKGR_OSCOUNT    ((D940_REG) 0xFF <<  8) //!< (CKGR) Main Oscillator Start-up Time
// -------- CKGR_MCFR : (CKGR Offset: 0x4) Main Clock Frequency Register -------- 
#define D940C_CKGR_MAINF      ((D940_REG) 0xFFFF <<  0) //!< (CKGR) Main Clock Frequency
#define D940C_CKGR_MAINRDY    ((D940_REG) 0x1 << 16) //!< (CKGR) Main Clock Ready
// -------- CKGR_PLLAR : (CKGR Offset: 0x8) PLL A Register -------- 
#define D940C_CKGR_DIVA       ((D940_REG) 0xFF <<  0) //!< (CKGR) Divider A Selected
#define 	D940C_CKGR_DIVA_0                    ((D940_REG) 0x0) //!< (CKGR) Divider A output is 0
#define 	D940C_CKGR_DIVA_BYPASS               ((D940_REG) 0x1) //!< (CKGR) Divider A is bypassed
#define D940C_CKGR_PLLACOUNT  ((D940_REG) 0x3F <<  8) //!< (CKGR) PLL A Counter
#define D940C_CKGR_OUTA       ((D940_REG) 0x3 << 14) //!< (CKGR) PLL A Output Frequency Range
#define 	D940C_CKGR_OUTA_0                    ((D940_REG) 0x0 << 14) //!< (CKGR) Please refer to the PLLA datasheet
#define 	D940C_CKGR_OUTA_1                    ((D940_REG) 0x1 << 14) //!< (CKGR) Please refer to the PLLA datasheet
#define 	D940C_CKGR_OUTA_2                    ((D940_REG) 0x2 << 14) //!< (CKGR) Please refer to the PLLA datasheet
#define 	D940C_CKGR_OUTA_3                    ((D940_REG) 0x3 << 14) //!< (CKGR) Please refer to the PLLA datasheet
#define D940C_CKGR_MULA       ((D940_REG) 0x7FF << 16) //!< (CKGR) PLL A Multiplier
#define D940C_CKGR_SRCA       ((D940_REG) 0x1 << 29) //!< (CKGR) 
// -------- CKGR_PLLBR : (CKGR Offset: 0xc) PLL B Register -------- 
#define D940C_CKGR_DIVB       ((D940_REG) 0xFF <<  0) //!< (CKGR) Divider B Selected
#define 	D940C_CKGR_DIVB_0                    ((D940_REG) 0x0) //!< (CKGR) Divider B output is 0
#define 	D940C_CKGR_DIVB_BYPASS               ((D940_REG) 0x1) //!< (CKGR) Divider B is bypassed
#define D940C_CKGR_PLLBCOUNT  ((D940_REG) 0x3F <<  8) //!< (CKGR) PLL B Counter
#define D940C_CKGR_OUTB       ((D940_REG) 0x3 << 14) //!< (CKGR) PLL B Output Frequency Range
#define 	D940C_CKGR_OUTB_0                    ((D940_REG) 0x0 << 14) //!< (CKGR) Please refer to the PLLB datasheet
#define 	D940C_CKGR_OUTB_1                    ((D940_REG) 0x1 << 14) //!< (CKGR) Please refer to the PLLB datasheet
#define 	D940C_CKGR_OUTB_2                    ((D940_REG) 0x2 << 14) //!< (CKGR) Please refer to the PLLB datasheet
#define 	D940C_CKGR_OUTB_3                    ((D940_REG) 0x3 << 14) //!< (CKGR) Please refer to the PLLB datasheet
#define D940C_CKGR_MULB       ((D940_REG) 0x7FF << 16) //!< (CKGR) PLL B Multiplier
#define D940C_CKGR_USBDIV     ((D940_REG) 0x3 << 28) //!< (CKGR) Divider for USB Clocks
#define 	D940C_CKGR_USBDIV_0                    ((D940_REG) 0x0 << 28) //!< (CKGR) Divider output is PLL clock output
#define 	D940C_CKGR_USBDIV_1                    ((D940_REG) 0x1 << 28) //!< (CKGR) Divider output is PLL clock output divided by 2
#define 	D940C_CKGR_USBDIV_2                    ((D940_REG) 0x2 << 28) //!< (CKGR) Divider output is PLL clock output divided by 4

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR Power Management Controler
// *****************************************************************************
typedef struct _D940S_PMC {
	D940_REG	 PMC_SCER; 	//!< System Clock Enable Register
	D940_REG	 PMC_SCDR; 	//!< System Clock Disable Register
	D940_REG	 PMC_SCSR; 	//!< System Clock Status Register
	D940_REG	 Reserved0[1]; 	//!< 
	D940_REG	 PMC_PCER; 	//!< Peripheral Clock Enable Register
	D940_REG	 PMC_PCDR; 	//!< Peripheral Clock Disable Register
	D940_REG	 PMC_PCSR; 	//!< Peripheral Clock Status Register
	D940_REG	 Reserved1[1]; 	//!< 
	D940_REG	 PMC_MOR; 	//!< Main Oscillator Register
	D940_REG	 PMC_MCFR; 	//!< Main Clock  Frequency Register
	D940_REG	 PMC_PLLAR; 	//!< PLL A Register
	D940_REG	 PMC_PLLBR; 	//!< PLL B Register
	D940_REG	 PMC_MCKR; 	//!< Master Clock Register
	D940_REG	 Reserved2[3]; 	//!< 
	D940_REG	 PMC_PCKR[8]; 	//!< Programmable Clock Register
	D940_REG	 PMC_IER; 	//!< Interrupt Enable Register
	D940_REG	 PMC_IDR; 	//!< Interrupt Disable Register
	D940_REG	 PMC_SR; 	//!< Status Register
	D940_REG	 PMC_IMR; 	//!< Interrupt Mask Register
} D940S_PMC,   __MAGICV_EXTERNAL *D940PS_PMC;

// -------- PMC_SCER : (PMC Offset: 0x0) System Clock Enable Register -------- 
#define D940C_PMC_PCK         ((D940_REG) 0x1 <<  0) //!< (PMC) Processor Clock
#define D940C_PMC_UHP         ((D940_REG) 0x1 <<  6) //!< (PMC) USB Host Port Clock
#define D940C_PMC_UDP         ((D940_REG) 0x1 <<  7) //!< (PMC) USB Device Port Clock
#define D940C_PMC_PCK0        ((D940_REG) 0x1 <<  8) //!< (PMC) Programmable Clock Output
#define D940C_PMC_PCK1        ((D940_REG) 0x1 <<  9) //!< (PMC) Programmable Clock Output
#define D940C_PMC_PCK2        ((D940_REG) 0x1 << 10) //!< (PMC) Programmable Clock Output
#define D940C_PMC_PCK3        ((D940_REG) 0x1 << 11) //!< (PMC) Programmable Clock Output
#define D940C_PMC_PCK4        ((D940_REG) 0x1 << 12) //!< (PMC) mAgicV Programmable Clock Output
#define D940C_PMC_HCK0        ((D940_REG) 0x1 << 16) //!< (PMC) AHB LCDCK Clock Output
#define D940C_PMC_HCK1        ((D940_REG) 0x1 << 17) //!< (PMC) AHB LCDCK Clock Output
#define D940C_PMC_HCK2        ((D940_REG) 0x1 << 18) //!< (PMC) AHB LCDCK Clock Output
// -------- PMC_SCDR : (PMC Offset: 0x4) System Clock Disable Register -------- 
// -------- PMC_SCSR : (PMC Offset: 0x8) System Clock Status Register -------- 
// -------- CKGR_MOR : (PMC Offset: 0x20) Main Oscillator Register -------- 
// -------- CKGR_MCFR : (PMC Offset: 0x24) Main Clock Frequency Register -------- 
// -------- CKGR_PLLAR : (PMC Offset: 0x28) PLL A Register -------- 
// -------- CKGR_PLLBR : (PMC Offset: 0x2c) PLL B Register -------- 
// -------- PMC_MCKR : (PMC Offset: 0x30) Master Clock Register -------- 
#define D940C_PMC_CSS         ((D940_REG) 0x3 <<  0) //!< (PMC) Programmable Clock Selection
#define 	D940C_PMC_CSS_SLOW_CLK             ((D940_REG) 0x0) //!< (PMC) Slow Clock is selected
#define 	D940C_PMC_CSS_MAIN_CLK             ((D940_REG) 0x1) //!< (PMC) Main Clock is selected
#define 	D940C_PMC_CSS_PLLA_CLK             ((D940_REG) 0x2) //!< (PMC) Clock from PLL A is selected
#define 	D940C_PMC_CSS_PLLB_CLK             ((D940_REG) 0x3) //!< (PMC) Clock from PLL B is selected
#define D940C_PMC_PRES        ((D940_REG) 0x7 <<  2) //!< (PMC) Programmable Clock Prescaler
#define 	D940C_PMC_PRES_CLK                  ((D940_REG) 0x0 <<  2) //!< (PMC) Selected clock
#define 	D940C_PMC_PRES_CLK_2                ((D940_REG) 0x1 <<  2) //!< (PMC) Selected clock divided by 2
#define 	D940C_PMC_PRES_CLK_4                ((D940_REG) 0x2 <<  2) //!< (PMC) Selected clock divided by 4
#define 	D940C_PMC_PRES_CLK_8                ((D940_REG) 0x3 <<  2) //!< (PMC) Selected clock divided by 8
#define 	D940C_PMC_PRES_CLK_16               ((D940_REG) 0x4 <<  2) //!< (PMC) Selected clock divided by 16
#define 	D940C_PMC_PRES_CLK_32               ((D940_REG) 0x5 <<  2) //!< (PMC) Selected clock divided by 32
#define 	D940C_PMC_PRES_CLK_64               ((D940_REG) 0x6 <<  2) //!< (PMC) Selected clock divided by 64
#define D940C_PMC_MDIV        ((D940_REG) 0x3 <<  8) //!< (PMC) Master Clock Division
#define 	D940C_PMC_MDIV_1                    ((D940_REG) 0x0 <<  8) //!< (PMC) The master clock and the processor clock are the same
#define 	D940C_PMC_MDIV_2                    ((D940_REG) 0x1 <<  8) //!< (PMC) The processor clock is twice as fast as the master clock
#define 	D940C_PMC_MDIV_3                    ((D940_REG) 0x2 <<  8) //!< (PMC) The processor clock is four times faster than the master clock
// -------- PMC_PCKR : (PMC Offset: 0x40) Programmable Clock Register -------- 
// -------- PMC_IER : (PMC Offset: 0x60) PMC Interrupt Enable Register -------- 
#define D940C_PMC_MOSCS       ((D940_REG) 0x1 <<  0) //!< (PMC) MOSC Status/Enable/Disable/Mask
#define D940C_PMC_LOCKA       ((D940_REG) 0x1 <<  1) //!< (PMC) PLL A Status/Enable/Disable/Mask
#define D940C_PMC_LOCKB       ((D940_REG) 0x1 <<  2) //!< (PMC) PLL B Status/Enable/Disable/Mask
#define D940C_PMC_MCKRDY      ((D940_REG) 0x1 <<  3) //!< (PMC) Master Clock Status/Enable/Disable/Mask
#define D940C_PMC_PCK0RDY     ((D940_REG) 0x1 <<  8) //!< (PMC) PCK0_RDY Status/Enable/Disable/Mask
#define D940C_PMC_PCK1RDY     ((D940_REG) 0x1 <<  9) //!< (PMC) PCK1_RDY Status/Enable/Disable/Mask
#define D940C_PMC_PCK2RDY     ((D940_REG) 0x1 << 10) //!< (PMC) PCK2_RDY Status/Enable/Disable/Mask
#define D940C_PMC_PCK3RDY     ((D940_REG) 0x1 << 11) //!< (PMC) PCK3_RDY Status/Enable/Disable/Mask
#define D940C_PMC_PCK4RDY     ((D940_REG) 0x1 << 12) //!< (PMC) PCK4_RDY Status/Enable/Disable/Mask
// -------- PMC_IDR : (PMC Offset: 0x64) PMC Interrupt Disable Register -------- 
// -------- PMC_SR : (PMC Offset: 0x68) PMC Status Register -------- 
// -------- PMC_IMR : (PMC Offset: 0x6c) PMC Interrupt Mask Register -------- 

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR Reset Controller Interface
// *****************************************************************************
typedef struct _D940S_RSTC {
	D940_REG	 RSTC_RCR; 	//!< Reset Control Register
	D940_REG	 RSTC_RSR; 	//!< Reset Status Register
	D940_REG	 RSTC_RMR; 	//!< Reset Mode Register
} D940S_RSTC,   __MAGICV_EXTERNAL *D940PS_RSTC;

// -------- RSTC_RCR : (RSTC Offset: 0x0) Reset Control Register -------- 
#define D940C_RSTC_PROCRST    ((D940_REG) 0x1 <<  0) //!< (RSTC) Processor Reset
#define D940C_RSTC_ICERST     ((D940_REG) 0x1 <<  1) //!< (RSTC) ICE Interface Reset
#define D940C_RSTC_PERRST     ((D940_REG) 0x1 <<  2) //!< (RSTC) Peripheral Reset
#define D940C_RSTC_EXTRST     ((D940_REG) 0x1 <<  3) //!< (RSTC) External Reset
#define D940C_RSTC_KEY        ((D940_REG) 0xFF << 24) //!< (RSTC) Password
// -------- RSTC_RSR : (RSTC Offset: 0x4) Reset Status Register -------- 
#define D940C_RSTC_URSTS      ((D940_REG) 0x1 <<  0) //!< (RSTC) User Reset Status
#define D940C_RSTC_RSTTYP     ((D940_REG) 0x7 <<  8) //!< (RSTC) Reset Type
#define 	D940C_RSTC_RSTTYP_GENERAL              ((D940_REG) 0x0 <<  8) //!< (RSTC) General reset. Both VDDCORE and VDDBU rising.
#define 	D940C_RSTC_RSTTYP_WAKEUP               ((D940_REG) 0x1 <<  8) //!< (RSTC) WakeUp Reset. VDDCORE rising.
#define 	D940C_RSTC_RSTTYP_WATCHDOG             ((D940_REG) 0x2 <<  8) //!< (RSTC) Watchdog Reset. Watchdog overflow occured.
#define 	D940C_RSTC_RSTTYP_SOFTWARE             ((D940_REG) 0x3 <<  8) //!< (RSTC) Software Reset. Processor reset required by the software.
#define 	D940C_RSTC_RSTTYP_USER                 ((D940_REG) 0x4 <<  8) //!< (RSTC) User Reset. NRST pin detected low.
#define D940C_RSTC_NRSTL      ((D940_REG) 0x1 << 16) //!< (RSTC) NRST pin level
#define D940C_RSTC_SRCMP      ((D940_REG) 0x1 << 17) //!< (RSTC) Software Reset Command in Progress.
// -------- RSTC_RMR : (RSTC Offset: 0x8) Reset Mode Register -------- 
#define D940C_RSTC_URSTEN     ((D940_REG) 0x1 <<  0) //!< (RSTC) User Reset Enable
#define D940C_RSTC_URSTIEN    ((D940_REG) 0x1 <<  4) //!< (RSTC) User Reset Interrupt Enable
#define D940C_RSTC_ERSTL      ((D940_REG) 0xF <<  8) //!< (RSTC) User Reset Enable

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR Shut Down Controller Interface
// *****************************************************************************
typedef struct _D940S_SHDWC {
	D940_REG	 SHDWC_SHCR; 	//!< Shut Down Control Register
	D940_REG	 SHDWC_SHMR; 	//!< Shut Down Mode Register
	D940_REG	 SHDWC_SHSR; 	//!< Shut Down Status Register
} D940S_SHDWC,   __MAGICV_EXTERNAL *D940PS_SHDWC;

// -------- SHDWC_SHCR : (SHDWC Offset: 0x0) Shut Down Control Register -------- 
#define D940C_SHDWC_SHDW      ((D940_REG) 0x1 <<  0) //!< (SHDWC) Processor Reset
#define D940C_SHDWC_KEY       ((D940_REG) 0xFF << 24) //!< (SHDWC) Shut down KEY Password
// -------- SHDWC_SHMR : (SHDWC Offset: 0x4) Shut Down Mode Register -------- 
#define D940C_SHDWC_WKMODE0   ((D940_REG) 0x3 <<  0) //!< (SHDWC) Wake Up 0 Mode Selection
#define 	D940C_SHDWC_WKMODE0_NONE                 ((D940_REG) 0x0) //!< (SHDWC) None. No detection is performed on the wake up input.
#define 	D940C_SHDWC_WKMODE0_HIGH                 ((D940_REG) 0x1) //!< (SHDWC) High Level.
#define 	D940C_SHDWC_WKMODE0_LOW                  ((D940_REG) 0x2) //!< (SHDWC) Low Level.
#define 	D940C_SHDWC_WKMODE0_ANYLEVEL             ((D940_REG) 0x3) //!< (SHDWC) Any level change.
#define D940C_SHDWC_CPTWK0    ((D940_REG) 0xF <<  4) //!< (SHDWC) Counter On Wake Up 0
#define D940C_SHDWC_RTTWKEN   ((D940_REG) 0x1 << 16) //!< (SHDWC) Real Time Timer Wake Up Enable
// -------- SHDWC_SHSR : (SHDWC Offset: 0x8) Shut Down Status Register -------- 
#define D940C_SHDWC_WAKEUP0   ((D940_REG) 0x1 <<  0) //!< (SHDWC) Wake Up 0 Status
#define D940C_SHDWC_RTTWK     ((D940_REG) 0x1 << 16) //!< (SHDWC) Real Time Timer wake Up

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR Real Time Timer Controller Interface
// *****************************************************************************
typedef struct _D940S_RTTC {
	D940_REG	 RTTC_RTMR; 	//!< Real-time Mode Register
	D940_REG	 RTTC_RTAR; 	//!< Real-time Alarm Register
	D940_REG	 RTTC_RTVR; 	//!< Real-time Value Register
	D940_REG	 RTTC_RTSR; 	//!< Real-time Status Register
} D940S_RTTC,   __MAGICV_EXTERNAL *D940PS_RTTC;

// -------- RTTC_RTMR : (RTTC Offset: 0x0) Real-time Mode Register -------- 
#define D940C_RTTC_RTPRES     ((D940_REG) 0xFFFF <<  0) //!< (RTTC) Real-time Timer Prescaler Value
#define D940C_RTTC_ALMIEN     ((D940_REG) 0x1 << 16) //!< (RTTC) Alarm Interrupt Enable
#define D940C_RTTC_RTTINCIEN  ((D940_REG) 0x1 << 17) //!< (RTTC) Real Time Timer Increment Interrupt Enable
#define D940C_RTTC_RTTRST     ((D940_REG) 0x1 << 18) //!< (RTTC) Real Time Timer Restart
// -------- RTTC_RTAR : (RTTC Offset: 0x4) Real-time Alarm Register -------- 
#define D940C_RTTC_ALMV       ((D940_REG) 0x0 <<  0) //!< (RTTC) Alarm Value
// -------- RTTC_RTVR : (RTTC Offset: 0x8) Current Real-time Value Register -------- 
#define D940C_RTTC_CRTV       ((D940_REG) 0x0 <<  0) //!< (RTTC) Current Real-time Value
// -------- RTTC_RTSR : (RTTC Offset: 0xc) Real-time Status Register -------- 
#define D940C_RTTC_ALMS       ((D940_REG) 0x1 <<  0) //!< (RTTC) Real-time Alarm Status
#define D940C_RTTC_RTTINC     ((D940_REG) 0x1 <<  1) //!< (RTTC) Real-time Timer Increment

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR Periodic Interval Timer Controller Interface
// *****************************************************************************
typedef struct _D940S_PITC {
	D940_REG	 PITC_PIMR; 	//!< Period Interval Mode Register
	D940_REG	 PITC_PISR; 	//!< Period Interval Status Register
	D940_REG	 PITC_PIVR; 	//!< Period Interval Value Register
	D940_REG	 PITC_PIIR; 	//!< Period Interval Image Register
} D940S_PITC,   __MAGICV_EXTERNAL *D940PS_PITC;

// -------- PITC_PIMR : (PITC Offset: 0x0) Periodic Interval Mode Register -------- 
#define D940C_PITC_PIV        ((D940_REG) 0xFFFFF <<  0) //!< (PITC) Periodic Interval Value
#define D940C_PITC_PITEN      ((D940_REG) 0x1 << 24) //!< (PITC) Periodic Interval Timer Enabled
#define D940C_PITC_PITIEN     ((D940_REG) 0x1 << 25) //!< (PITC) Periodic Interval Timer Interrupt Enable
// -------- PITC_PISR : (PITC Offset: 0x4) Periodic Interval Status Register -------- 
#define D940C_PITC_PITS       ((D940_REG) 0x1 <<  0) //!< (PITC) Periodic Interval Timer Status
// -------- PITC_PIVR : (PITC Offset: 0x8) Periodic Interval Value Register -------- 
#define D940C_PITC_CPIV       ((D940_REG) 0xFFFFF <<  0) //!< (PITC) Current Periodic Interval Value
#define D940C_PITC_PICNT      ((D940_REG) 0xFFF << 20) //!< (PITC) Periodic Interval Counter
// -------- PITC_PIIR : (PITC Offset: 0xc) Periodic Interval Image Register -------- 

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR Watchdog Timer Controller Interface
// *****************************************************************************
typedef struct _D940S_WDTC {
	D940_REG	 WDTC_WDCR; 	//!< Watchdog Control Register
	D940_REG	 WDTC_WDMR; 	//!< Watchdog Mode Register
	D940_REG	 WDTC_WDSR; 	//!< Watchdog Status Register
} D940S_WDTC,   __MAGICV_EXTERNAL *D940PS_WDTC;

// -------- WDTC_WDCR : (WDTC Offset: 0x0) Periodic Interval Image Register -------- 
#define D940C_WDTC_WDRSTT     ((D940_REG) 0x1 <<  0) //!< (WDTC) Watchdog Restart
#define D940C_WDTC_KEY        ((D940_REG) 0xFF << 24) //!< (WDTC) Watchdog KEY Password
// -------- WDTC_WDMR : (WDTC Offset: 0x4) Watchdog Mode Register -------- 
#define D940C_WDTC_WDV        ((D940_REG) 0xFFF <<  0) //!< (WDTC) Watchdog Timer Restart
#define D940C_WDTC_WDFIEN     ((D940_REG) 0x1 << 12) //!< (WDTC) Watchdog Fault Interrupt Enable
#define D940C_WDTC_WDRSTEN    ((D940_REG) 0x1 << 13) //!< (WDTC) Watchdog Reset Enable
#define D940C_WDTC_WDRPROC    ((D940_REG) 0x1 << 14) //!< (WDTC) Watchdog Timer Restart
#define D940C_WDTC_WDDIS      ((D940_REG) 0x1 << 15) //!< (WDTC) Watchdog Disable
#define D940C_WDTC_WDD        ((D940_REG) 0xFFF << 16) //!< (WDTC) Watchdog Delta Value
#define D940C_WDTC_WDDBGHLT   ((D940_REG) 0x1 << 28) //!< (WDTC) Watchdog Debug Halt
#define D940C_WDTC_WDIDLEHLT  ((D940_REG) 0x1 << 29) //!< (WDTC) Watchdog Idle Halt
// -------- WDTC_WDSR : (WDTC Offset: 0x8) Watchdog Status Register -------- 
#define D940C_WDTC_WDUNF      ((D940_REG) 0x1 <<  0) //!< (WDTC) Watchdog Underflow
#define D940C_WDTC_WDERR      ((D940_REG) 0x1 <<  1) //!< (WDTC) Watchdog Error

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR Timer Counter Channel Interface
// *****************************************************************************
typedef struct _D940S_TC {
	D940_REG	 TC_CCR; 	//!< Channel Control Register
	D940_REG	 TC_CMR; 	//!< Channel Mode Register (Capture Mode / Waveform Mode)
	D940_REG	 Reserved0[2]; 	//!< 
	D940_REG	 TC_CV; 	//!< Counter Value
	D940_REG	 TC_RA; 	//!< Register A
	D940_REG	 TC_RB; 	//!< Register B
	D940_REG	 TC_RC; 	//!< Register C
	D940_REG	 TC_SR; 	//!< Status Register
	D940_REG	 TC_IER; 	//!< Interrupt Enable Register
	D940_REG	 TC_IDR; 	//!< Interrupt Disable Register
	D940_REG	 TC_IMR; 	//!< Interrupt Mask Register
} D940S_TC,   __MAGICV_EXTERNAL *D940PS_TC;

// -------- TC_CCR : (TC Offset: 0x0) TC Channel Control Register -------- 
#define D940C_TC_CLKEN        ((D940_REG) 0x1 <<  0) //!< (TC) Counter Clock Enable Command
#define D940C_TC_CLKDIS       ((D940_REG) 0x1 <<  1) //!< (TC) Counter Clock Disable Command
#define D940C_TC_SWTRG        ((D940_REG) 0x1 <<  2) //!< (TC) Software Trigger Command
// -------- TC_CMR : (TC Offset: 0x4) TC Channel Mode Register: Capture Mode / Waveform Mode -------- 
#define D940C_TC_CLKS         ((D940_REG) 0x7 <<  0) //!< (TC) Clock Selection
#define 	D940C_TC_CLKS_TIMER_DIV1_CLOCK     ((D940_REG) 0x0) //!< (TC) Clock selected: TIMER_DIV1_CLOCK
#define 	D940C_TC_CLKS_TIMER_DIV2_CLOCK     ((D940_REG) 0x1) //!< (TC) Clock selected: TIMER_DIV2_CLOCK
#define 	D940C_TC_CLKS_TIMER_DIV3_CLOCK     ((D940_REG) 0x2) //!< (TC) Clock selected: TIMER_DIV3_CLOCK
#define 	D940C_TC_CLKS_TIMER_DIV4_CLOCK     ((D940_REG) 0x3) //!< (TC) Clock selected: TIMER_DIV4_CLOCK
#define 	D940C_TC_CLKS_TIMER_DIV5_CLOCK     ((D940_REG) 0x4) //!< (TC) Clock selected: TIMER_DIV5_CLOCK
#define 	D940C_TC_CLKS_XC0                  ((D940_REG) 0x5) //!< (TC) Clock selected: XC0
#define 	D940C_TC_CLKS_XC1                  ((D940_REG) 0x6) //!< (TC) Clock selected: XC1
#define 	D940C_TC_CLKS_XC2                  ((D940_REG) 0x7) //!< (TC) Clock selected: XC2
#define D940C_TC_CLKI         ((D940_REG) 0x1 <<  3) //!< (TC) Clock Invert
#define D940C_TC_BURST        ((D940_REG) 0x3 <<  4) //!< (TC) Burst Signal Selection
#define 	D940C_TC_BURST_NONE                 ((D940_REG) 0x0 <<  4) //!< (TC) The clock is not gated by an external signal
#define 	D940C_TC_BURST_XC0                  ((D940_REG) 0x1 <<  4) //!< (TC) XC0 is ANDed with the selected clock
#define 	D940C_TC_BURST_XC1                  ((D940_REG) 0x2 <<  4) //!< (TC) XC1 is ANDed with the selected clock
#define 	D940C_TC_BURST_XC2                  ((D940_REG) 0x3 <<  4) //!< (TC) XC2 is ANDed with the selected clock
#define D940C_TC_CPCSTOP      ((D940_REG) 0x1 <<  6) //!< (TC) Counter Clock Stopped with RC Compare
#define D940C_TC_LDBSTOP      ((D940_REG) 0x1 <<  6) //!< (TC) Counter Clock Stopped with RB Loading
#define D940C_TC_CPCDIS       ((D940_REG) 0x1 <<  7) //!< (TC) Counter Clock Disable with RC Compare
#define D940C_TC_LDBDIS       ((D940_REG) 0x1 <<  7) //!< (TC) Counter Clock Disabled with RB Loading
#define D940C_TC_ETRGEDG      ((D940_REG) 0x3 <<  8) //!< (TC) External Trigger Edge Selection
#define 	D940C_TC_ETRGEDG_NONE                 ((D940_REG) 0x0 <<  8) //!< (TC) Edge: None
#define 	D940C_TC_ETRGEDG_RISING               ((D940_REG) 0x1 <<  8) //!< (TC) Edge: rising edge
#define 	D940C_TC_ETRGEDG_FALLING              ((D940_REG) 0x2 <<  8) //!< (TC) Edge: falling edge
#define 	D940C_TC_ETRGEDG_BOTH                 ((D940_REG) 0x3 <<  8) //!< (TC) Edge: each edge
#define D940C_TC_EEVTEDG      ((D940_REG) 0x3 <<  8) //!< (TC) External Event Edge Selection
#define 	D940C_TC_EEVTEDG_NONE                 ((D940_REG) 0x0 <<  8) //!< (TC) Edge: None
#define 	D940C_TC_EEVTEDG_RISING               ((D940_REG) 0x1 <<  8) //!< (TC) Edge: rising edge
#define 	D940C_TC_EEVTEDG_FALLING              ((D940_REG) 0x2 <<  8) //!< (TC) Edge: falling edge
#define 	D940C_TC_EEVTEDG_BOTH                 ((D940_REG) 0x3 <<  8) //!< (TC) Edge: each edge
#define D940C_TC_EEVT         ((D940_REG) 0x3 << 10) //!< (TC) External Event  Selection
#define 	D940C_TC_EEVT_TIOB                 ((D940_REG) 0x0 << 10) //!< (TC) Signal selected as external event: TIOB TIOB direction: input
#define 	D940C_TC_EEVT_XC0                  ((D940_REG) 0x1 << 10) //!< (TC) Signal selected as external event: XC0 TIOB direction: output
#define 	D940C_TC_EEVT_XC1                  ((D940_REG) 0x2 << 10) //!< (TC) Signal selected as external event: XC1 TIOB direction: output
#define 	D940C_TC_EEVT_XC2                  ((D940_REG) 0x3 << 10) //!< (TC) Signal selected as external event: XC2 TIOB direction: output
#define D940C_TC_ABETRG       ((D940_REG) 0x1 << 10) //!< (TC) TIOA or TIOB External Trigger Selection
#define D940C_TC_ENETRG       ((D940_REG) 0x1 << 12) //!< (TC) External Event Trigger enable
#define D940C_TC_WAVESEL      ((D940_REG) 0x3 << 13) //!< (TC) Waveform  Selection
#define 	D940C_TC_WAVESEL_UP                   ((D940_REG) 0x0 << 13) //!< (TC) UP mode without atomatic trigger on RC Compare
#define 	D940C_TC_WAVESEL_UPDOWN               ((D940_REG) 0x1 << 13) //!< (TC) UPDOWN mode without automatic trigger on RC Compare
#define 	D940C_TC_WAVESEL_UP_AUTO              ((D940_REG) 0x2 << 13) //!< (TC) UP mode with automatic trigger on RC Compare
#define 	D940C_TC_WAVESEL_UPDOWN_AUTO          ((D940_REG) 0x3 << 13) //!< (TC) UPDOWN mode with automatic trigger on RC Compare
#define D940C_TC_CPCTRG       ((D940_REG) 0x1 << 14) //!< (TC) RC Compare Trigger Enable
#define D940C_TC_WAVE         ((D940_REG) 0x1 << 15) //!< (TC) 
#define D940C_TC_ACPA         ((D940_REG) 0x3 << 16) //!< (TC) RA Compare Effect on TIOA
#define 	D940C_TC_ACPA_NONE                 ((D940_REG) 0x0 << 16) //!< (TC) Effect: none
#define 	D940C_TC_ACPA_SET                  ((D940_REG) 0x1 << 16) //!< (TC) Effect: set
#define 	D940C_TC_ACPA_CLEAR                ((D940_REG) 0x2 << 16) //!< (TC) Effect: clear
#define 	D940C_TC_ACPA_TOGGLE               ((D940_REG) 0x3 << 16) //!< (TC) Effect: toggle
#define D940C_TC_LDRA         ((D940_REG) 0x3 << 16) //!< (TC) RA Loading Selection
#define 	D940C_TC_LDRA_NONE                 ((D940_REG) 0x0 << 16) //!< (TC) Edge: None
#define 	D940C_TC_LDRA_RISING               ((D940_REG) 0x1 << 16) //!< (TC) Edge: rising edge of TIOA
#define 	D940C_TC_LDRA_FALLING              ((D940_REG) 0x2 << 16) //!< (TC) Edge: falling edge of TIOA
#define 	D940C_TC_LDRA_BOTH                 ((D940_REG) 0x3 << 16) //!< (TC) Edge: each edge of TIOA
#define D940C_TC_ACPC         ((D940_REG) 0x3 << 18) //!< (TC) RC Compare Effect on TIOA
#define 	D940C_TC_ACPC_NONE                 ((D940_REG) 0x0 << 18) //!< (TC) Effect: none
#define 	D940C_TC_ACPC_SET                  ((D940_REG) 0x1 << 18) //!< (TC) Effect: set
#define 	D940C_TC_ACPC_CLEAR                ((D940_REG) 0x2 << 18) //!< (TC) Effect: clear
#define 	D940C_TC_ACPC_TOGGLE               ((D940_REG) 0x3 << 18) //!< (TC) Effect: toggle
#define D940C_TC_LDRB         ((D940_REG) 0x3 << 18) //!< (TC) RB Loading Selection
#define 	D940C_TC_LDRB_NONE                 ((D940_REG) 0x0 << 18) //!< (TC) Edge: None
#define 	D940C_TC_LDRB_RISING               ((D940_REG) 0x1 << 18) //!< (TC) Edge: rising edge of TIOA
#define 	D940C_TC_LDRB_FALLING              ((D940_REG) 0x2 << 18) //!< (TC) Edge: falling edge of TIOA
#define 	D940C_TC_LDRB_BOTH                 ((D940_REG) 0x3 << 18) //!< (TC) Edge: each edge of TIOA
#define D940C_TC_AEEVT        ((D940_REG) 0x3 << 20) //!< (TC) External Event Effect on TIOA
#define 	D940C_TC_AEEVT_NONE                 ((D940_REG) 0x0 << 20) //!< (TC) Effect: none
#define 	D940C_TC_AEEVT_SET                  ((D940_REG) 0x1 << 20) //!< (TC) Effect: set
#define 	D940C_TC_AEEVT_CLEAR                ((D940_REG) 0x2 << 20) //!< (TC) Effect: clear
#define 	D940C_TC_AEEVT_TOGGLE               ((D940_REG) 0x3 << 20) //!< (TC) Effect: toggle
#define D940C_TC_ASWTRG       ((D940_REG) 0x3 << 22) //!< (TC) Software Trigger Effect on TIOA
#define 	D940C_TC_ASWTRG_NONE                 ((D940_REG) 0x0 << 22) //!< (TC) Effect: none
#define 	D940C_TC_ASWTRG_SET                  ((D940_REG) 0x1 << 22) //!< (TC) Effect: set
#define 	D940C_TC_ASWTRG_CLEAR                ((D940_REG) 0x2 << 22) //!< (TC) Effect: clear
#define 	D940C_TC_ASWTRG_TOGGLE               ((D940_REG) 0x3 << 22) //!< (TC) Effect: toggle
#define D940C_TC_BCPB         ((D940_REG) 0x3 << 24) //!< (TC) RB Compare Effect on TIOB
#define 	D940C_TC_BCPB_NONE                 ((D940_REG) 0x0 << 24) //!< (TC) Effect: none
#define 	D940C_TC_BCPB_SET                  ((D940_REG) 0x1 << 24) //!< (TC) Effect: set
#define 	D940C_TC_BCPB_CLEAR                ((D940_REG) 0x2 << 24) //!< (TC) Effect: clear
#define 	D940C_TC_BCPB_TOGGLE               ((D940_REG) 0x3 << 24) //!< (TC) Effect: toggle
#define D940C_TC_BCPC         ((D940_REG) 0x3 << 26) //!< (TC) RC Compare Effect on TIOB
#define 	D940C_TC_BCPC_NONE                 ((D940_REG) 0x0 << 26) //!< (TC) Effect: none
#define 	D940C_TC_BCPC_SET                  ((D940_REG) 0x1 << 26) //!< (TC) Effect: set
#define 	D940C_TC_BCPC_CLEAR                ((D940_REG) 0x2 << 26) //!< (TC) Effect: clear
#define 	D940C_TC_BCPC_TOGGLE               ((D940_REG) 0x3 << 26) //!< (TC) Effect: toggle
#define D940C_TC_BEEVT        ((D940_REG) 0x3 << 28) //!< (TC) External Event Effect on TIOB
#define 	D940C_TC_BEEVT_NONE                 ((D940_REG) 0x0 << 28) //!< (TC) Effect: none
#define 	D940C_TC_BEEVT_SET                  ((D940_REG) 0x1 << 28) //!< (TC) Effect: set
#define 	D940C_TC_BEEVT_CLEAR                ((D940_REG) 0x2 << 28) //!< (TC) Effect: clear
#define 	D940C_TC_BEEVT_TOGGLE               ((D940_REG) 0x3 << 28) //!< (TC) Effect: toggle
#define D940C_TC_BSWTRG       ((D940_REG) 0x3 << 30) //!< (TC) Software Trigger Effect on TIOB
#define 	D940C_TC_BSWTRG_NONE                 ((D940_REG) 0x0 << 30) //!< (TC) Effect: none
#define 	D940C_TC_BSWTRG_SET                  ((D940_REG) 0x1 << 30) //!< (TC) Effect: set
#define 	D940C_TC_BSWTRG_CLEAR                ((D940_REG) 0x2 << 30) //!< (TC) Effect: clear
#define 	D940C_TC_BSWTRG_TOGGLE               ((D940_REG) 0x3 << 30) //!< (TC) Effect: toggle
// -------- TC_SR : (TC Offset: 0x20) TC Channel Status Register -------- 
#define D940C_TC_COVFS        ((D940_REG) 0x1 <<  0) //!< (TC) Counter Overflow
#define D940C_TC_LOVRS        ((D940_REG) 0x1 <<  1) //!< (TC) Load Overrun
#define D940C_TC_CPAS         ((D940_REG) 0x1 <<  2) //!< (TC) RA Compare
#define D940C_TC_CPBS         ((D940_REG) 0x1 <<  3) //!< (TC) RB Compare
#define D940C_TC_CPCS         ((D940_REG) 0x1 <<  4) //!< (TC) RC Compare
#define D940C_TC_LDRAS        ((D940_REG) 0x1 <<  5) //!< (TC) RA Loading
#define D940C_TC_LDRBS        ((D940_REG) 0x1 <<  6) //!< (TC) RB Loading
#define D940C_TC_ETRGS        ((D940_REG) 0x1 <<  7) //!< (TC) External Trigger
#define D940C_TC_CLKSTA       ((D940_REG) 0x1 << 16) //!< (TC) Clock Enabling
#define D940C_TC_MTIOA        ((D940_REG) 0x1 << 17) //!< (TC) TIOA Mirror
#define D940C_TC_MTIOB        ((D940_REG) 0x1 << 18) //!< (TC) TIOA Mirror
// -------- TC_IER : (TC Offset: 0x24) TC Channel Interrupt Enable Register -------- 
// -------- TC_IDR : (TC Offset: 0x28) TC Channel Interrupt Disable Register -------- 
// -------- TC_IMR : (TC Offset: 0x2c) TC Channel Interrupt Mask Register -------- 

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR Timer Counter Interface
// *****************************************************************************
typedef struct _D940S_TCB {
	D940S_TC	 TCB_TC0; 	//!< TC Channel 0
	D940_REG	 Reserved0[4]; 	//!< 
	D940S_TC	 TCB_TC1; 	//!< TC Channel 1
	D940_REG	 Reserved1[4]; 	//!< 
	D940S_TC	 TCB_TC2; 	//!< TC Channel 2
	D940_REG	 Reserved2[4]; 	//!< 
	D940_REG	 TCB_BCR; 	//!< TC Block Control Register
	D940_REG	 TCB_BMR; 	//!< TC Block Mode Register
} D940S_TCB,   __MAGICV_EXTERNAL *D940PS_TCB;

// -------- TCB_BCR : (TCB Offset: 0xc0) TC Block Control Register -------- 
#define D940C_TCB_SYNC        ((D940_REG) 0x1 <<  0) //!< (TCB) Synchro Command
// -------- TCB_BMR : (TCB Offset: 0xc4) TC Block Mode Register -------- 
#define D940C_TCB_TC0XC0S     ((D940_REG) 0x3 <<  0) //!< (TCB) External Clock Signal 0 Selection
#define 	D940C_TCB_TC0XC0S_TCLK0                ((D940_REG) 0x0) //!< (TCB) TCLK0 connected to XC0
#define 	D940C_TCB_TC0XC0S_NONE                 ((D940_REG) 0x1) //!< (TCB) None signal connected to XC0
#define 	D940C_TCB_TC0XC0S_TIOA1                ((D940_REG) 0x2) //!< (TCB) TIOA1 connected to XC0
#define 	D940C_TCB_TC0XC0S_TIOA2                ((D940_REG) 0x3) //!< (TCB) TIOA2 connected to XC0
#define D940C_TCB_TC1XC1S     ((D940_REG) 0x3 <<  2) //!< (TCB) External Clock Signal 1 Selection
#define 	D940C_TCB_TC1XC1S_TCLK1                ((D940_REG) 0x0 <<  2) //!< (TCB) TCLK1 connected to XC1
#define 	D940C_TCB_TC1XC1S_NONE                 ((D940_REG) 0x1 <<  2) //!< (TCB) None signal connected to XC1
#define 	D940C_TCB_TC1XC1S_TIOA0                ((D940_REG) 0x2 <<  2) //!< (TCB) TIOA0 connected to XC1
#define 	D940C_TCB_TC1XC1S_TIOA2                ((D940_REG) 0x3 <<  2) //!< (TCB) TIOA2 connected to XC1
#define D940C_TCB_TC2XC2S     ((D940_REG) 0x3 <<  4) //!< (TCB) External Clock Signal 2 Selection
#define 	D940C_TCB_TC2XC2S_TCLK2                ((D940_REG) 0x0 <<  4) //!< (TCB) TCLK2 connected to XC2
#define 	D940C_TCB_TC2XC2S_NONE                 ((D940_REG) 0x1 <<  4) //!< (TCB) None signal connected to XC2
#define 	D940C_TCB_TC2XC2S_TIOA0                ((D940_REG) 0x2 <<  4) //!< (TCB) TIOA0 connected to XC2
#define 	D940C_TCB_TC2XC2S_TIOA1                ((D940_REG) 0x3 <<  4) //!< (TCB) TIOA2 connected to XC2

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR USB Device Interface
// *****************************************************************************
typedef struct _D940S_UDP {
	D940_REG	 UDP_NUM; 	//!< Frame Number Register
	D940_REG	 UDP_GLBSTATE; 	//!< Global State Register
	D940_REG	 UDP_FADDR; 	//!< Function Address Register
	D940_REG	 Reserved0[1]; 	//!< 
	D940_REG	 UDP_IER; 	//!< Interrupt Enable Register
	D940_REG	 UDP_IDR; 	//!< Interrupt Disable Register
	D940_REG	 UDP_IMR; 	//!< Interrupt Mask Register
	D940_REG	 UDP_ISR; 	//!< Interrupt Status Register
	D940_REG	 UDP_ICR; 	//!< Interrupt Clear Register
	D940_REG	 Reserved1[1]; 	//!< 
	D940_REG	 UDP_RSTEP; 	//!< Reset Endpoint Register
	D940_REG	 Reserved2[1]; 	//!< 
	D940_REG	 UDP_CSR[8]; 	//!< Endpoint Control and Status Register
	D940_REG	 UDP_FDR[8]; 	//!< Endpoint FIFO Data Register
	D940_REG	 Reserved3[1]; 	//!< 
	D940_REG	 UDP_TXVC; 	//!< Transceiver Control Register
} D940S_UDP,   __MAGICV_EXTERNAL *D940PS_UDP;

// -------- UDP_FRM_NUM : (UDP Offset: 0x0) USB Frame Number Register -------- 
#define D940C_UDP_FRM_NUM     ((D940_REG) 0x7FF <<  0) //!< (UDP) Frame Number as Defined in the Packet Field Formats
#define D940C_UDP_FRM_ERR     ((D940_REG) 0x1 << 16) //!< (UDP) Frame Error
#define D940C_UDP_FRM_OK      ((D940_REG) 0x1 << 17) //!< (UDP) Frame OK
// -------- UDP_GLB_STATE : (UDP Offset: 0x4) USB Global State Register -------- 
#define D940C_UDP_FADDEN      ((D940_REG) 0x1 <<  0) //!< (UDP) Function Address Enable
#define D940C_UDP_CONFG       ((D940_REG) 0x1 <<  1) //!< (UDP) Configured
#define D940C_UDP_ESR         ((D940_REG) 0x1 <<  2) //!< (UDP) Enable Send Resume
#define D940C_UDP_RSMINPR     ((D940_REG) 0x1 <<  3) //!< (UDP) A Resume Has Been Sent to the Host
#define D940C_UDP_RMWUPE      ((D940_REG) 0x1 <<  4) //!< (UDP) Remote Wake Up Enable
// -------- UDP_FADDR : (UDP Offset: 0x8) USB Function Address Register -------- 
#define D940C_UDP_FADD        ((D940_REG) 0xFF <<  0) //!< (UDP) Function Address Value
#define D940C_UDP_FEN         ((D940_REG) 0x1 <<  8) //!< (UDP) Function Enable
// -------- UDP_IER : (UDP Offset: 0x10) USB Interrupt Enable Register -------- 
#define D940C_UDP_EPINT0      ((D940_REG) 0x1 <<  0) //!< (UDP) Endpoint 0 Interrupt
#define D940C_UDP_EPINT1      ((D940_REG) 0x1 <<  1) //!< (UDP) Endpoint 0 Interrupt
#define D940C_UDP_EPINT2      ((D940_REG) 0x1 <<  2) //!< (UDP) Endpoint 2 Interrupt
#define D940C_UDP_EPINT3      ((D940_REG) 0x1 <<  3) //!< (UDP) Endpoint 3 Interrupt
#define D940C_UDP_EPINT4      ((D940_REG) 0x1 <<  4) //!< (UDP) Endpoint 4 Interrupt
#define D940C_UDP_EPINT5      ((D940_REG) 0x1 <<  5) //!< (UDP) Endpoint 5 Interrupt
#define D940C_UDP_EPINT6      ((D940_REG) 0x1 <<  6) //!< (UDP) Endpoint 6 Interrupt
#define D940C_UDP_EPINT7      ((D940_REG) 0x1 <<  7) //!< (UDP) Endpoint 7 Interrupt
#define D940C_UDP_RXSUSP      ((D940_REG) 0x1 <<  8) //!< (UDP) USB Suspend Interrupt
#define D940C_UDP_RXRSM       ((D940_REG) 0x1 <<  9) //!< (UDP) USB Resume Interrupt
#define D940C_UDP_EXTRSM      ((D940_REG) 0x1 << 10) //!< (UDP) USB External Resume Interrupt
#define D940C_UDP_SOFINT      ((D940_REG) 0x1 << 11) //!< (UDP) USB Start Of frame Interrupt
#define D940C_UDP_WAKEUP      ((D940_REG) 0x1 << 13) //!< (UDP) USB Resume Interrupt
// -------- UDP_IDR : (UDP Offset: 0x14) USB Interrupt Disable Register -------- 
// -------- UDP_IMR : (UDP Offset: 0x18) USB Interrupt Mask Register -------- 
// -------- UDP_ISR : (UDP Offset: 0x1c) USB Interrupt Status Register -------- 
#define D940C_UDP_ENDBUSRES   ((D940_REG) 0x1 << 12) //!< (UDP) USB End Of Bus Reset Interrupt
// -------- UDP_ICR : (UDP Offset: 0x20) USB Interrupt Clear Register -------- 
// -------- UDP_RST_EP : (UDP Offset: 0x28) USB Reset Endpoint Register -------- 
#define D940C_UDP_EP0         ((D940_REG) 0x1 <<  0) //!< (UDP) Reset Endpoint 0
#define D940C_UDP_EP1         ((D940_REG) 0x1 <<  1) //!< (UDP) Reset Endpoint 1
#define D940C_UDP_EP2         ((D940_REG) 0x1 <<  2) //!< (UDP) Reset Endpoint 2
#define D940C_UDP_EP3         ((D940_REG) 0x1 <<  3) //!< (UDP) Reset Endpoint 3
#define D940C_UDP_EP4         ((D940_REG) 0x1 <<  4) //!< (UDP) Reset Endpoint 4
#define D940C_UDP_EP5         ((D940_REG) 0x1 <<  5) //!< (UDP) Reset Endpoint 5
#define D940C_UDP_EP6         ((D940_REG) 0x1 <<  6) //!< (UDP) Reset Endpoint 6
#define D940C_UDP_EP7         ((D940_REG) 0x1 <<  7) //!< (UDP) Reset Endpoint 7
// -------- UDP_CSR : (UDP Offset: 0x30) USB Endpoint Control and Status Register -------- 
#define D940C_UDP_TXCOMP      ((D940_REG) 0x1 <<  0) //!< (UDP) Generates an IN packet with data previously written in the DPR
#define D940C_UDP_RX_DATA_BK0 ((D940_REG) 0x1 <<  1) //!< (UDP) Receive Data Bank 0
#define D940C_UDP_RXSETUP     ((D940_REG) 0x1 <<  2) //!< (UDP) Sends STALL to the Host (Control endpoints)
#define D940C_UDP_ISOERROR    ((D940_REG) 0x1 <<  3) //!< (UDP) Isochronous error (Isochronous endpoints)
#define D940C_UDP_TXPKTRDY    ((D940_REG) 0x1 <<  4) //!< (UDP) Transmit Packet Ready
#define D940C_UDP_FORCESTALL  ((D940_REG) 0x1 <<  5) //!< (UDP) Force Stall (used by Control, Bulk and Isochronous endpoints).
#define D940C_UDP_RX_DATA_BK1 ((D940_REG) 0x1 <<  6) //!< (UDP) Receive Data Bank 1 (only used by endpoints with ping-pong attributes).
#define D940C_UDP_DIR         ((D940_REG) 0x1 <<  7) //!< (UDP) Transfer Direction
#define D940C_UDP_EPTYPE      ((D940_REG) 0x7 <<  8) //!< (UDP) Endpoint type
#define 	D940C_UDP_EPTYPE_CTRL                 ((D940_REG) 0x0 <<  8) //!< (UDP) Control
#define 	D940C_UDP_EPTYPE_ISO_OUT              ((D940_REG) 0x1 <<  8) //!< (UDP) Isochronous OUT
#define 	D940C_UDP_EPTYPE_BULK_OUT             ((D940_REG) 0x2 <<  8) //!< (UDP) Bulk OUT
#define 	D940C_UDP_EPTYPE_INT_OUT              ((D940_REG) 0x3 <<  8) //!< (UDP) Interrupt OUT
#define 	D940C_UDP_EPTYPE_ISO_IN               ((D940_REG) 0x5 <<  8) //!< (UDP) Isochronous IN
#define 	D940C_UDP_EPTYPE_BULK_IN              ((D940_REG) 0x6 <<  8) //!< (UDP) Bulk IN
#define 	D940C_UDP_EPTYPE_INT_IN               ((D940_REG) 0x7 <<  8) //!< (UDP) Interrupt IN
#define D940C_UDP_DTGLE       ((D940_REG) 0x1 << 11) //!< (UDP) Data Toggle
#define D940C_UDP_EPEDS       ((D940_REG) 0x1 << 15) //!< (UDP) Endpoint Enable Disable
#define D940C_UDP_RXBYTECNT   ((D940_REG) 0x7FF << 16) //!< (UDP) Number Of Bytes Available in the FIFO
// -------- UDP_TXVC : (UDP Offset: 0x74) Transceiver Control Register -------- 
#define D940C_UDP_TXVDIS      ((D940_REG) 0x1 <<  8) //!< (UDP) 
#define D940C_UDP_PUON        ((D940_REG) 0x1 <<  9) //!< (UDP) Pull-up ON

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR Two-wire Interface
// *****************************************************************************
typedef struct _D940S_TWI {
	D940_REG	 TWI_CR; 	//!< Control Register
	D940_REG	 TWI_MMR; 	//!< Master Mode Register
	D940_REG	 TWI_SMR; 	//!< Slave Mode Register
	D940_REG	 TWI_IADR; 	//!< Internal Address Register
	D940_REG	 TWI_CWGR; 	//!< Clock Waveform Generator Register
	D940_REG	 Reserved0[3]; 	//!< 
	D940_REG	 TWI_SR; 	//!< Status Register
	D940_REG	 TWI_IER; 	//!< Interrupt Enable Register
	D940_REG	 TWI_IDR; 	//!< Interrupt Disable Register
	D940_REG	 TWI_IMR; 	//!< Interrupt Mask Register
	D940_REG	 TWI_RHR; 	//!< Receive Holding Register
	D940_REG	 TWI_THR; 	//!< Transmit Holding Register
} D940S_TWI,   __MAGICV_EXTERNAL *D940PS_TWI;

// -------- TWI_CR : (TWI Offset: 0x0) TWI Control Register -------- 
#define D940C_TWI_START       ((D940_REG) 0x1 <<  0) //!< (TWI) Send a START Condition
#define D940C_TWI_STOP        ((D940_REG) 0x1 <<  1) //!< (TWI) Send a STOP Condition
#define D940C_TWI_MSEN        ((D940_REG) 0x1 <<  2) //!< (TWI) TWI Master Transfer Enabled
#define D940C_TWI_MSDIS       ((D940_REG) 0x1 <<  3) //!< (TWI) TWI Master Transfer Disabled
#define D940C_TWI_SVEN        ((D940_REG) 0x1 <<  4) //!< (TWI) TWI Slave Transfer Enabled
#define D940C_TWI_SVDIS       ((D940_REG) 0x1 <<  5) //!< (TWI) TWI Slave Transfer Disabled
#define D940C_TWI_SWRST       ((D940_REG) 0x1 <<  7) //!< (TWI) Software Reset
// -------- TWI_MMR : (TWI Offset: 0x4) TWI Master Mode Register -------- 
#define D940C_TWI_IADRSZ      ((D940_REG) 0x3 <<  8) //!< (TWI) Internal Device Address Size
#define 	D940C_TWI_IADRSZ_NO                   ((D940_REG) 0x0 <<  8) //!< (TWI) No internal device address
#define 	D940C_TWI_IADRSZ_1_BYTE               ((D940_REG) 0x1 <<  8) //!< (TWI) One-byte internal device address
#define 	D940C_TWI_IADRSZ_2_BYTE               ((D940_REG) 0x2 <<  8) //!< (TWI) Two-byte internal device address
#define 	D940C_TWI_IADRSZ_3_BYTE               ((D940_REG) 0x3 <<  8) //!< (TWI) Three-byte internal device address
#define D940C_TWI_MREAD       ((D940_REG) 0x1 << 12) //!< (TWI) Master Read Direction
#define D940C_TWI_DADR        ((D940_REG) 0x7F << 16) //!< (TWI) Device Address
// -------- TWI_SMR : (TWI Offset: 0x8) TWI Slave Mode Register -------- 
#define D940C_TWI_SADR        ((D940_REG) 0x7F << 16) //!< (TWI) Slave Device Address
// -------- TWI_CWGR : (TWI Offset: 0x10) TWI Clock Waveform Generator Register -------- 
#define D940C_TWI_CLDIV       ((D940_REG) 0xFF <<  0) //!< (TWI) Clock Low Divider
#define D940C_TWI_CHDIV       ((D940_REG) 0xFF <<  8) //!< (TWI) Clock High Divider
#define D940C_TWI_CKDIV       ((D940_REG) 0x7 << 16) //!< (TWI) Clock Divider
// -------- TWI_SR : (TWI Offset: 0x20) TWI Status Register -------- 
#define D940C_TWI_TXCOMP      ((D940_REG) 0x1 <<  0) //!< (TWI) Transmission Completed
#define D940C_TWI_RXRDY       ((D940_REG) 0x1 <<  1) //!< (TWI) Receive holding register ReaDY
#define D940C_TWI_TXRDY       ((D940_REG) 0x1 <<  2) //!< (TWI) Transmit holding register ReaDY
#define D940C_TWI_SVREAD      ((D940_REG) 0x1 <<  3) //!< (TWI) Slave Read
#define D940C_TWI_SVACC       ((D940_REG) 0x1 <<  4) //!< (TWI) Slave Access
#define D940C_TWI_GCACC       ((D940_REG) 0x1 <<  5) //!< (TWI) General Call Access
#define D940C_TWI_OVRE        ((D940_REG) 0x1 <<  6) //!< (TWI) Overrun Error
#define D940C_TWI_UNRE        ((D940_REG) 0x1 <<  7) //!< (TWI) Underrun Error
#define D940C_TWI_NACK        ((D940_REG) 0x1 <<  8) //!< (TWI) Not Acknowledged
#define D940C_TWI_ARBLST      ((D940_REG) 0x1 <<  9) //!< (TWI) Arbitration Lost
// -------- TWI_IER : (TWI Offset: 0x24) TWI Interrupt Enable Register -------- 
// -------- TWI_IDR : (TWI Offset: 0x28) TWI Interrupt Disable Register -------- 
// -------- TWI_IMR : (TWI Offset: 0x2c) TWI Interrupt Mask Register -------- 

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR Usart
// *****************************************************************************
typedef struct _D940S_USART {
	D940_REG	 US_CR; 	//!< Control Register
	D940_REG	 US_MR; 	//!< Mode Register
	D940_REG	 US_IER; 	//!< Interrupt Enable Register
	D940_REG	 US_IDR; 	//!< Interrupt Disable Register
	D940_REG	 US_IMR; 	//!< Interrupt Mask Register
	D940_REG	 US_CSR; 	//!< Channel Status Register
	D940_REG	 US_RHR; 	//!< Receiver Holding Register
	D940_REG	 US_THR; 	//!< Transmitter Holding Register
	D940_REG	 US_BRGR; 	//!< Baud Rate Generator Register
	D940_REG	 US_RTOR; 	//!< Receiver Time-out Register
	D940_REG	 US_TTGR; 	//!< Transmitter Time-guard Register
	D940_REG	 Reserved0[5]; 	//!< 
	D940_REG	 US_FIDI; 	//!< FI_DI_Ratio Register
	D940_REG	 US_NER; 	//!< Nb Errors Register
	D940_REG	 Reserved1[1]; 	//!< 
	D940_REG	 US_IF; 	//!< IRDA_FILTER Register
	D940_REG	 Reserved2[44]; 	//!< 
	D940_REG	 US_RPR; 	//!< Receive Pointer Register
	D940_REG	 US_RCR; 	//!< Receive Counter Register
	D940_REG	 US_TPR; 	//!< Transmit Pointer Register
	D940_REG	 US_TCR; 	//!< Transmit Counter Register
	D940_REG	 US_RNPR; 	//!< Receive Next Pointer Register
	D940_REG	 US_RNCR; 	//!< Receive Next Counter Register
	D940_REG	 US_TNPR; 	//!< Transmit Next Pointer Register
	D940_REG	 US_TNCR; 	//!< Transmit Next Counter Register
	D940_REG	 US_PTCR; 	//!< PDC Transfer Control Register
	D940_REG	 US_PTSR; 	//!< PDC Transfer Status Register
} D940S_USART,   __MAGICV_EXTERNAL *D940PS_USART;

//#ifdef __chess__
typedef D940S_USART D940S_US;
typedef D940PS_USART D940PS_US;
//#endif

// -------- US_CR : (USART Offset: 0x0) Debug Unit Control Register -------- 
#define D940C_US_STTBRK       ((D940_REG) 0x1 <<  9) //!< (USART) Start Break
#define D940C_US_STPBRK       ((D940_REG) 0x1 << 10) //!< (USART) Stop Break
#define D940C_US_STTTO        ((D940_REG) 0x1 << 11) //!< (USART) Start Time-out
#define D940C_US_SENDA        ((D940_REG) 0x1 << 12) //!< (USART) Send Address
#define D940C_US_RSTIT        ((D940_REG) 0x1 << 13) //!< (USART) Reset Iterations
#define D940C_US_RSTNACK      ((D940_REG) 0x1 << 14) //!< (USART) Reset Non Acknowledge
#define D940C_US_RETTO        ((D940_REG) 0x1 << 15) //!< (USART) Rearm Time-out
#define D940C_US_DTREN        ((D940_REG) 0x1 << 16) //!< (USART) Data Terminal ready Enable
#define D940C_US_DTRDIS       ((D940_REG) 0x1 << 17) //!< (USART) Data Terminal ready Disable
#define D940C_US_RTSEN        ((D940_REG) 0x1 << 18) //!< (USART) Request to Send enable
#define D940C_US_RTSDIS       ((D940_REG) 0x1 << 19) //!< (USART) Request to Send Disable
// -------- US_MR : (USART Offset: 0x4) Debug Unit Mode Register -------- 
#define D940C_US_USMODE       ((D940_REG) 0xF <<  0) //!< (USART) Usart mode
#define 	D940C_US_USMODE_NORMAL               ((D940_REG) 0x0) //!< (USART) Normal
#define 	D940C_US_USMODE_RS485                ((D940_REG) 0x1) //!< (USART) RS485
#define 	D940C_US_USMODE_HWHSH                ((D940_REG) 0x2) //!< (USART) Hardware Handshaking
#define 	D940C_US_USMODE_MODEM                ((D940_REG) 0x3) //!< (USART) Modem
#define 	D940C_US_USMODE_ISO7816_0            ((D940_REG) 0x4) //!< (USART) ISO7816 protocol: T = 0
#define 	D940C_US_USMODE_ISO7816_1            ((D940_REG) 0x6) //!< (USART) ISO7816 protocol: T = 1
#define 	D940C_US_USMODE_IRDA                 ((D940_REG) 0x8) //!< (USART) IrDA
#define 	D940C_US_USMODE_SWHSH                ((D940_REG) 0xC) //!< (USART) Software Handshaking
#define D940C_US_CLKS         ((D940_REG) 0x3 <<  4) //!< (USART) Clock Selection (Baud Rate generator Input Clock
#define 	D940C_US_CLKS_CLOCK                ((D940_REG) 0x0 <<  4) //!< (USART) Clock
#define 	D940C_US_CLKS_FDIV1                ((D940_REG) 0x1 <<  4) //!< (USART) fdiv1
#define 	D940C_US_CLKS_SLOW                 ((D940_REG) 0x2 <<  4) //!< (USART) slow_clock (ARM)
#define 	D940C_US_CLKS_EXT                  ((D940_REG) 0x3 <<  4) //!< (USART) External (SCK)
#define D940C_US_CHRL         ((D940_REG) 0x3 <<  6) //!< (USART) Clock Selection (Baud Rate generator Input Clock
#define 	D940C_US_CHRL_5_BITS               ((D940_REG) 0x0 <<  6) //!< (USART) Character Length: 5 bits
#define 	D940C_US_CHRL_6_BITS               ((D940_REG) 0x1 <<  6) //!< (USART) Character Length: 6 bits
#define 	D940C_US_CHRL_7_BITS               ((D940_REG) 0x2 <<  6) //!< (USART) Character Length: 7 bits
#define 	D940C_US_CHRL_8_BITS               ((D940_REG) 0x3 <<  6) //!< (USART) Character Length: 8 bits
#define D940C_US_SYNC         ((D940_REG) 0x1 <<  8) //!< (USART) Synchronous Mode Select
#define D940C_US_NBSTOP       ((D940_REG) 0x3 << 12) //!< (USART) Number of Stop bits
#define 	D940C_US_NBSTOP_1_BIT                ((D940_REG) 0x0 << 12) //!< (USART) 1 stop bit
#define 	D940C_US_NBSTOP_15_BIT               ((D940_REG) 0x1 << 12) //!< (USART) Asynchronous (SYNC=0) 2 stop bits Synchronous (SYNC=1) 2 stop bits
#define 	D940C_US_NBSTOP_2_BIT                ((D940_REG) 0x2 << 12) //!< (USART) 2 stop bits
#define D940C_US_MSBF         ((D940_REG) 0x1 << 16) //!< (USART) Bit Order
#define D940C_US_MODE9        ((D940_REG) 0x1 << 17) //!< (USART) 9-bit Character length
#define D940C_US_CKLO         ((D940_REG) 0x1 << 18) //!< (USART) Clock Output Select
#define D940C_US_OVER         ((D940_REG) 0x1 << 19) //!< (USART) Over Sampling Mode
#define D940C_US_INACK        ((D940_REG) 0x1 << 20) //!< (USART) Inhibit Non Acknowledge
#define D940C_US_DSNACK       ((D940_REG) 0x1 << 21) //!< (USART) Disable Successive NACK
#define D940C_US_MAX_ITER     ((D940_REG) 0x1 << 24) //!< (USART) Number of Repetitions
#define D940C_US_FILTER       ((D940_REG) 0x1 << 28) //!< (USART) Receive Line Filter
// -------- US_IER : (USART Offset: 0x8) Debug Unit Interrupt Enable Register -------- 
#define D940C_US_RXBRK        ((D940_REG) 0x1 <<  2) //!< (USART) Break Received/End of Break
#define D940C_US_TIMEOUT      ((D940_REG) 0x1 <<  8) //!< (USART) Receiver Time-out
#define D940C_US_ITERATION    ((D940_REG) 0x1 << 10) //!< (USART) Max number of Repetitions Reached
#define D940C_US_NACK         ((D940_REG) 0x1 << 13) //!< (USART) Non Acknowledge
#define D940C_US_RIIC         ((D940_REG) 0x1 << 16) //!< (USART) Ring INdicator Input Change Flag
#define D940C_US_DSRIC        ((D940_REG) 0x1 << 17) //!< (USART) Data Set Ready Input Change Flag
#define D940C_US_DCDIC        ((D940_REG) 0x1 << 18) //!< (USART) Data Carrier Flag
#define D940C_US_CTSIC        ((D940_REG) 0x1 << 19) //!< (USART) Clear To Send Input Change Flag
// -------- US_IDR : (USART Offset: 0xc) Debug Unit Interrupt Disable Register -------- 
// -------- US_IMR : (USART Offset: 0x10) Debug Unit Interrupt Mask Register -------- 
// -------- US_CSR : (USART Offset: 0x14) Debug Unit Channel Status Register -------- 
#define D940C_US_RI           ((D940_REG) 0x1 << 20) //!< (USART) Image of RI Input
#define D940C_US_DSR          ((D940_REG) 0x1 << 21) //!< (USART) Image of DSR Input
#define D940C_US_DCD          ((D940_REG) 0x1 << 22) //!< (USART) Image of DCD Input
#define D940C_US_CTS          ((D940_REG) 0x1 << 23) //!< (USART) Image of CTS Input

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR Synchronous Serial Controller Interface
// *****************************************************************************
typedef struct _D940S_SSC {
	D940_REG	 SSC_CR; 	//!< Control Register
	D940_REG	 SSC_CMR; 	//!< Clock Mode Register
	D940_REG	 Reserved0[2]; 	//!< 
	D940_REG	 SSC_RCMR; 	//!< Receive Clock ModeRegister
	D940_REG	 SSC_RFMR; 	//!< Receive Frame Mode Register
	D940_REG	 SSC_TCMR; 	//!< Transmit Clock Mode Register
	D940_REG	 SSC_TFMR; 	//!< Transmit Frame Mode Register
	D940_REG	 SSC_RHR; 	//!< Receive Holding Register
	D940_REG	 SSC_THR; 	//!< Transmit Holding Register
	D940_REG	 Reserved1[2]; 	//!< 
	D940_REG	 SSC_RSHR; 	//!< Receive Sync Holding Register
	D940_REG	 SSC_TSHR; 	//!< Transmit Sync Holding Register
	D940_REG	 Reserved2[2]; 	//!< 
	D940_REG	 SSC_SR; 	//!< Status Register
	D940_REG	 SSC_IER; 	//!< Interrupt Enable Register
	D940_REG	 SSC_IDR; 	//!< Interrupt Disable Register
	D940_REG	 SSC_IMR; 	//!< Interrupt Mask Register
	D940_REG	 Reserved3[44]; 	//!< 
	D940_REG	 SSC_RPR; 	//!< Receive Pointer Register
	D940_REG	 SSC_RCR; 	//!< Receive Counter Register
	D940_REG	 SSC_TPR; 	//!< Transmit Pointer Register
	D940_REG	 SSC_TCR; 	//!< Transmit Counter Register
	D940_REG	 SSC_RNPR; 	//!< Receive Next Pointer Register
	D940_REG	 SSC_RNCR; 	//!< Receive Next Counter Register
	D940_REG	 SSC_TNPR; 	//!< Transmit Next Pointer Register
	D940_REG	 SSC_TNCR; 	//!< Transmit Next Counter Register
	D940_REG	 SSC_PTCR; 	//!< PDC Transfer Control Register
	D940_REG	 SSC_PTSR; 	//!< PDC Transfer Status Register
} D940S_SSC,   __MAGICV_EXTERNAL *D940PS_SSC;

// -------- SSC_CR : (SSC Offset: 0x0) SSC Control Register -------- 
#define D940C_SSC_RXEN        ((D940_REG) 0x1 <<  0) //!< (SSC) Receive Enable
#define D940C_SSC_RXDIS       ((D940_REG) 0x1 <<  1) //!< (SSC) Receive Disable
#define D940C_SSC_TXEN        ((D940_REG) 0x1 <<  8) //!< (SSC) Transmit Enable
#define D940C_SSC_TXDIS       ((D940_REG) 0x1 <<  9) //!< (SSC) Transmit Disable
#define D940C_SSC_SWRST       ((D940_REG) 0x1 << 15) //!< (SSC) Software Reset
// -------- SSC_RCMR : (SSC Offset: 0x10) SSC Receive Clock Mode Register -------- 
#define D940C_SSC_CKS         ((D940_REG) 0x3 <<  0) //!< (SSC) Receive/Transmit Clock Selection
#define 	D940C_SSC_CKS_DIV                  ((D940_REG) 0x0) //!< (SSC) Divided Clock
#define 	D940C_SSC_CKS_TK                   ((D940_REG) 0x1) //!< (SSC) TK Clock signal
#define 	D940C_SSC_CKS_RK                   ((D940_REG) 0x2) //!< (SSC) RK pin
#define D940C_SSC_CKO         ((D940_REG) 0x7 <<  2) //!< (SSC) Receive/Transmit Clock Output Mode Selection
#define 	D940C_SSC_CKO_NONE                 ((D940_REG) 0x0 <<  2) //!< (SSC) Receive/Transmit Clock Output Mode: None RK pin: Input-only
#define 	D940C_SSC_CKO_CONTINOUS            ((D940_REG) 0x1 <<  2) //!< (SSC) Continuous Receive/Transmit Clock RK pin: Output
#define 	D940C_SSC_CKO_DATA_TX              ((D940_REG) 0x2 <<  2) //!< (SSC) Receive/Transmit Clock only during data transfers RK pin: Output
#define D940C_SSC_CKI         ((D940_REG) 0x1 <<  5) //!< (SSC) Receive/Transmit Clock Inversion
#define D940C_SSC_START       ((D940_REG) 0xF <<  8) //!< (SSC) Receive/Transmit Start Selection
#define 	D940C_SSC_START_CONTINOUS            ((D940_REG) 0x0 <<  8) //!< (SSC) Continuous, as soon as the receiver is enabled, and immediately after the end of transfer of the previous data.
#define 	D940C_SSC_START_TX                   ((D940_REG) 0x1 <<  8) //!< (SSC) Transmit/Receive start
#define 	D940C_SSC_START_LOW_RF               ((D940_REG) 0x2 <<  8) //!< (SSC) Detection of a low level on RF input
#define 	D940C_SSC_START_HIGH_RF              ((D940_REG) 0x3 <<  8) //!< (SSC) Detection of a high level on RF input
#define 	D940C_SSC_START_FALL_RF              ((D940_REG) 0x4 <<  8) //!< (SSC) Detection of a falling edge on RF input
#define 	D940C_SSC_START_RISE_RF              ((D940_REG) 0x5 <<  8) //!< (SSC) Detection of a rising edge on RF input
#define 	D940C_SSC_START_LEVEL_RF             ((D940_REG) 0x6 <<  8) //!< (SSC) Detection of any level change on RF input
#define 	D940C_SSC_START_EDGE_RF              ((D940_REG) 0x7 <<  8) //!< (SSC) Detection of any edge on RF input
#define 	D940C_SSC_START_0                    ((D940_REG) 0x8 <<  8) //!< (SSC) Compare 0
#define D940C_SSC_STTDLY      ((D940_REG) 0xFF << 16) //!< (SSC) Receive/Transmit Start Delay
#define D940C_SSC_PERIOD      ((D940_REG) 0xFF << 24) //!< (SSC) Receive/Transmit Period Divider Selection
// -------- SSC_RFMR : (SSC Offset: 0x14) SSC Receive Frame Mode Register -------- 
#define D940C_SSC_DATLEN      ((D940_REG) 0x1F <<  0) //!< (SSC) Data Length
#define D940C_SSC_LOOP        ((D940_REG) 0x1 <<  5) //!< (SSC) Loop Mode
#define D940C_SSC_MSBF        ((D940_REG) 0x1 <<  7) //!< (SSC) Most Significant Bit First
#define D940C_SSC_DATNB       ((D940_REG) 0xF <<  8) //!< (SSC) Data Number per Frame
#define D940C_SSC_FSLEN       ((D940_REG) 0xFF << 12) //!< (SSC) Receive/Transmit Frame Sync length
#define D940C_SSC_FSOS        ((D940_REG) 0x7 << 20) //!< (SSC) Receive/Transmit Frame Sync Output Selection
#define 	D940C_SSC_FSOS_NONE                 ((D940_REG) 0x0 << 20) //!< (SSC) Selected Receive/Transmit Frame Sync Signal: None RK pin Input-only
#define 	D940C_SSC_FSOS_NEGATIVE             ((D940_REG) 0x1 << 20) //!< (SSC) Selected Receive/Transmit Frame Sync Signal: Negative Pulse
#define 	D940C_SSC_FSOS_POSITIVE             ((D940_REG) 0x2 << 20) //!< (SSC) Selected Receive/Transmit Frame Sync Signal: Positive Pulse
#define 	D940C_SSC_FSOS_LOW                  ((D940_REG) 0x3 << 20) //!< (SSC) Selected Receive/Transmit Frame Sync Signal: Driver Low during data transfer
#define 	D940C_SSC_FSOS_HIGH                 ((D940_REG) 0x4 << 20) //!< (SSC) Selected Receive/Transmit Frame Sync Signal: Driver High during data transfer
#define 	D940C_SSC_FSOS_TOGGLE               ((D940_REG) 0x5 << 20) //!< (SSC) Selected Receive/Transmit Frame Sync Signal: Toggling at each start of data transfer
#define D940C_SSC_FSEDGE      ((D940_REG) 0x1 << 24) //!< (SSC) Frame Sync Edge Detection
// -------- SSC_TCMR : (SSC Offset: 0x18) SSC Transmit Clock Mode Register -------- 
// -------- SSC_TFMR : (SSC Offset: 0x1c) SSC Transmit Frame Mode Register -------- 
#define D940C_SSC_DATDEF      ((D940_REG) 0x1 <<  5) //!< (SSC) Data Default Value
#define D940C_SSC_FSDEN       ((D940_REG) 0x1 << 23) //!< (SSC) Frame Sync Data Enable
// -------- SSC_SR : (SSC Offset: 0x40) SSC Status Register -------- 
#define D940C_SSC_TXRDY       ((D940_REG) 0x1 <<  0) //!< (SSC) Transmit Ready
#define D940C_SSC_TXEMPTY     ((D940_REG) 0x1 <<  1) //!< (SSC) Transmit Empty
#define D940C_SSC_ENDTX       ((D940_REG) 0x1 <<  2) //!< (SSC) End Of Transmission
#define D940C_SSC_TXBUFE      ((D940_REG) 0x1 <<  3) //!< (SSC) Transmit Buffer Empty
#define D940C_SSC_RXRDY       ((D940_REG) 0x1 <<  4) //!< (SSC) Receive Ready
#define D940C_SSC_OVRUN       ((D940_REG) 0x1 <<  5) //!< (SSC) Receive Overrun
#define D940C_SSC_ENDRX       ((D940_REG) 0x1 <<  6) //!< (SSC) End of Reception
#define D940C_SSC_RXBUFF      ((D940_REG) 0x1 <<  7) //!< (SSC) Receive Buffer Full
#define D940C_SSC_TXSYN       ((D940_REG) 0x1 << 10) //!< (SSC) Transmit Sync
#define D940C_SSC_RXSYN       ((D940_REG) 0x1 << 11) //!< (SSC) Receive Sync
#define D940C_SSC_TXENA       ((D940_REG) 0x1 << 16) //!< (SSC) Transmit Enable
#define D940C_SSC_RXENA       ((D940_REG) 0x1 << 17) //!< (SSC) Receive Enable
// -------- SSC_IER : (SSC Offset: 0x44) SSC Interrupt Enable Register -------- 
// -------- SSC_IDR : (SSC Offset: 0x48) SSC Interrupt Disable Register -------- 
// -------- SSC_IMR : (SSC Offset: 0x4c) SSC Interrupt Mask Register -------- 

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR Serial Parallel Interface
// *****************************************************************************
typedef struct _D940S_SPI {
	D940_REG	 SPI_CR; 	//!< Control Register
	D940_REG	 SPI_MR; 	//!< Mode Register
	D940_REG	 SPI_RDR; 	//!< Receive Data Register
	D940_REG	 SPI_TDR; 	//!< Transmit Data Register
	D940_REG	 SPI_SR; 	//!< Status Register
	D940_REG	 SPI_IER; 	//!< Interrupt Enable Register
	D940_REG	 SPI_IDR; 	//!< Interrupt Disable Register
	D940_REG	 SPI_IMR; 	//!< Interrupt Mask Register
	D940_REG	 Reserved0[4]; 	//!< 
	D940_REG	 SPI_CSR[4]; 	//!< Chip Select Register
	D940_REG	 Reserved1[48]; 	//!< 
	D940_REG	 SPI_RPR; 	//!< Receive Pointer Register
	D940_REG	 SPI_RCR; 	//!< Receive Counter Register
	D940_REG	 SPI_TPR; 	//!< Transmit Pointer Register
	D940_REG	 SPI_TCR; 	//!< Transmit Counter Register
	D940_REG	 SPI_RNPR; 	//!< Receive Next Pointer Register
	D940_REG	 SPI_RNCR; 	//!< Receive Next Counter Register
	D940_REG	 SPI_TNPR; 	//!< Transmit Next Pointer Register
	D940_REG	 SPI_TNCR; 	//!< Transmit Next Counter Register
	D940_REG	 SPI_PTCR; 	//!< PDC Transfer Control Register
	D940_REG	 SPI_PTSR; 	//!< PDC Transfer Status Register
} D940S_SPI,   __MAGICV_EXTERNAL *D940PS_SPI;

// -------- SPI_CR : (SPI Offset: 0x0) SPI Control Register -------- 
#define D940C_SPI_SPIEN       ((D940_REG) 0x1 <<  0) //!< (SPI) SPI Enable
#define D940C_SPI_SPIDIS      ((D940_REG) 0x1 <<  1) //!< (SPI) SPI Disable
#define D940C_SPI_SWRST       ((D940_REG) 0x1 <<  7) //!< (SPI) SPI Software reset
#define D940C_SPI_LASTXFER    ((D940_REG) 0x1 << 24) //!< (SPI) SPI Last Transfer
// -------- SPI_MR : (SPI Offset: 0x4) SPI Mode Register -------- 
#define D940C_SPI_MSTR        ((D940_REG) 0x1 <<  0) //!< (SPI) Master/Slave Mode
#define D940C_SPI_PS          ((D940_REG) 0x1 <<  1) //!< (SPI) Peripheral Select
#define 	D940C_SPI_PS_FIXED                ((D940_REG) 0x0 <<  1) //!< (SPI) Fixed Peripheral Select
#define 	D940C_SPI_PS_VARIABLE             ((D940_REG) 0x1 <<  1) //!< (SPI) Variable Peripheral Select
#define D940C_SPI_PCSDEC      ((D940_REG) 0x1 <<  2) //!< (SPI) Chip Select Decode
#define D940C_SPI_FDIV        ((D940_REG) 0x1 <<  3) //!< (SPI) Clock Selection
#define D940C_SPI_MODFDIS     ((D940_REG) 0x1 <<  4) //!< (SPI) Mode Fault Detection
#define D940C_SPI_LLB         ((D940_REG) 0x1 <<  7) //!< (SPI) Clock Selection
#define D940C_SPI_PCS         ((D940_REG) 0xF << 16) //!< (SPI) Peripheral Chip Select
#define D940C_SPI_DLYBCS      ((D940_REG) 0xFF << 24) //!< (SPI) Delay Between Chip Selects
// -------- SPI_RDR : (SPI Offset: 0x8) Receive Data Register -------- 
#define D940C_SPI_RD          ((D940_REG) 0xFFFF <<  0) //!< (SPI) Receive Data
#define D940C_SPI_RPCS        ((D940_REG) 0xF << 16) //!< (SPI) Peripheral Chip Select Status
// -------- SPI_TDR : (SPI Offset: 0xc) Transmit Data Register -------- 
#define D940C_SPI_TD          ((D940_REG) 0xFFFF <<  0) //!< (SPI) Transmit Data
#define D940C_SPI_TPCS        ((D940_REG) 0xF << 16) //!< (SPI) Peripheral Chip Select Status
// -------- SPI_SR : (SPI Offset: 0x10) Status Register -------- 
#define D940C_SPI_RDRF        ((D940_REG) 0x1 <<  0) //!< (SPI) Receive Data Register Full
#define D940C_SPI_TDRE        ((D940_REG) 0x1 <<  1) //!< (SPI) Transmit Data Register Empty
#define D940C_SPI_MODF        ((D940_REG) 0x1 <<  2) //!< (SPI) Mode Fault Error
#define D940C_SPI_OVRES       ((D940_REG) 0x1 <<  3) //!< (SPI) Overrun Error Status
#define D940C_SPI_ENDRX       ((D940_REG) 0x1 <<  4) //!< (SPI) End of Receiver Transfer
#define D940C_SPI_ENDTX       ((D940_REG) 0x1 <<  5) //!< (SPI) End of Receiver Transfer
#define D940C_SPI_RXBUFF      ((D940_REG) 0x1 <<  6) //!< (SPI) RXBUFF Interrupt
#define D940C_SPI_TXBUFE      ((D940_REG) 0x1 <<  7) //!< (SPI) TXBUFE Interrupt
#define D940C_SPI_NSSR        ((D940_REG) 0x1 <<  8) //!< (SPI) NSSR Interrupt
#define D940C_SPI_TXEMPTY     ((D940_REG) 0x1 <<  9) //!< (SPI) TXEMPTY Interrupt
#define D940C_SPI_SPIENS      ((D940_REG) 0x1 << 16) //!< (SPI) Enable Status
// -------- SPI_IER : (SPI Offset: 0x14) Interrupt Enable Register -------- 
// -------- SPI_IDR : (SPI Offset: 0x18) Interrupt Disable Register -------- 
// -------- SPI_IMR : (SPI Offset: 0x1c) Interrupt Mask Register -------- 
// -------- SPI_CSR : (SPI Offset: 0x30) Chip Select Register -------- 
#define D940C_SPI_CPOL        ((D940_REG) 0x1 <<  0) //!< (SPI) Clock Polarity
#define D940C_SPI_NCPHA       ((D940_REG) 0x1 <<  1) //!< (SPI) Clock Phase
#define D940C_SPI_CSAAT       ((D940_REG) 0x1 <<  3) //!< (SPI) Chip Select Active After Transfer
#define D940C_SPI_BITS        ((D940_REG) 0xF <<  4) //!< (SPI) Bits Per Transfer
#define 	D940C_SPI_BITS_8                    ((D940_REG) 0x0 <<  4) //!< (SPI) 8 Bits Per transfer
#define 	D940C_SPI_BITS_9                    ((D940_REG) 0x1 <<  4) //!< (SPI) 9 Bits Per transfer
#define 	D940C_SPI_BITS_10                   ((D940_REG) 0x2 <<  4) //!< (SPI) 10 Bits Per transfer
#define 	D940C_SPI_BITS_11                   ((D940_REG) 0x3 <<  4) //!< (SPI) 11 Bits Per transfer
#define 	D940C_SPI_BITS_12                   ((D940_REG) 0x4 <<  4) //!< (SPI) 12 Bits Per transfer
#define 	D940C_SPI_BITS_13                   ((D940_REG) 0x5 <<  4) //!< (SPI) 13 Bits Per transfer
#define 	D940C_SPI_BITS_14                   ((D940_REG) 0x6 <<  4) //!< (SPI) 14 Bits Per transfer
#define 	D940C_SPI_BITS_15                   ((D940_REG) 0x7 <<  4) //!< (SPI) 15 Bits Per transfer
#define 	D940C_SPI_BITS_16                   ((D940_REG) 0x8 <<  4) //!< (SPI) 16 Bits Per transfer
#define D940C_SPI_SCBR        ((D940_REG) 0xFF <<  8) //!< (SPI) Serial Clock Baud Rate
#define D940C_SPI_DLYBS       ((D940_REG) 0xFF << 16) //!< (SPI) Serial Clock Baud Rate
#define D940C_SPI_DLYBCT      ((D940_REG) 0xFF << 24) //!< (SPI) Delay Between Consecutive Transfers

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR USB Host Interface
// *****************************************************************************
typedef struct _D940S_UHP {
	D940_REG	 UHP_HcRevision; 	//!< Revision
	D940_REG	 UHP_HcControl; 	//!< Operating modes for the Host Controller
	D940_REG	 UHP_HcCommandStatus; 	//!< Command & status Register
	D940_REG	 UHP_HcInterruptStatus; 	//!< Interrupt Status Register
	D940_REG	 UHP_HcInterruptEnable; 	//!< Interrupt Enable Register
	D940_REG	 UHP_HcInterruptDisable; 	//!< Interrupt Disable Register
	D940_REG	 UHP_HcHCCA; 	//!< Pointer to the Host Controller Communication Area
	D940_REG	 UHP_HcPeriodCurrentED; 	//!< Current Isochronous or Interrupt Endpoint Descriptor
	D940_REG	 UHP_HcControlHeadED; 	//!< First Endpoint Descriptor of the Control list
	D940_REG	 UHP_HcControlCurrentED; 	//!< Endpoint Control and Status Register
	D940_REG	 UHP_HcBulkHeadED; 	//!< First endpoint register of the Bulk list
	D940_REG	 UHP_HcBulkCurrentED; 	//!< Current endpoint of the Bulk list
	D940_REG	 UHP_HcBulkDoneHead; 	//!< Last completed transfer descriptor
	D940_REG	 UHP_HcFmInterval; 	//!< Bit time between 2 consecutive SOFs
	D940_REG	 UHP_HcFmRemaining; 	//!< Bit time remaining in the current Frame
	D940_REG	 UHP_HcFmNumber; 	//!< Frame number
	D940_REG	 UHP_HcPeriodicStart; 	//!< Periodic Start
	D940_REG	 UHP_HcLSThreshold; 	//!< LS Threshold
	D940_REG	 UHP_HcRhDescriptorA; 	//!< Root Hub characteristics A
	D940_REG	 UHP_HcRhDescriptorB; 	//!< Root Hub characteristics B
	D940_REG	 UHP_HcRhStatus; 	//!< Root Hub Status register
	D940_REG	 UHP_HcRhPortStatus[2]; 	//!< Root Hub Port Status Register
} D940S_UHP,   __MAGICV_EXTERNAL *D940PS_UHP;


// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR Ethernet MAC
// *****************************************************************************
typedef struct _D940S_EMAC {
	D940_REG	 EMAC_CTL; 	//!< Network Control Register
	D940_REG	 EMAC_CFG; 	//!< Network Configuration Register
	D940_REG	 EMAC_SR; 	//!< Network Status Register
	D940_REG	 EMAC_TAR; 	//!< Transmit Address Register
	D940_REG	 EMAC_TCR; 	//!< Transmit Control Register
	D940_REG	 EMAC_TSR; 	//!< Transmit Status Register
	D940_REG	 EMAC_RBQP; 	//!< Receive Buffer Queue Pointer
	D940_REG	 Reserved0[1]; 	//!< 
	D940_REG	 EMAC_RSR; 	//!< Receive Status Register
	D940_REG	 EMAC_ISR; 	//!< Interrupt Status Register
	D940_REG	 EMAC_IER; 	//!< Interrupt Enable Register
	D940_REG	 EMAC_IDR; 	//!< Interrupt Disable Register
	D940_REG	 EMAC_IMR; 	//!< Interrupt Mask Register
	D940_REG	 EMAC_MAN; 	//!< PHY Maintenance Register
	D940_REG	 Reserved1[2]; 	//!< 
	D940_REG	 EMAC_FRA; 	//!< Frames Transmitted OK Register
	D940_REG	 EMAC_SCOL; 	//!< Single Collision Frame Register
	D940_REG	 EMAC_MCOL; 	//!< Multiple Collision Frame Register
	D940_REG	 EMAC_OK; 	//!< Frames Received OK Register
	D940_REG	 EMAC_SEQE; 	//!< Frame Check Sequence Error Register
	D940_REG	 EMAC_ALE; 	//!< Alignment Error Register
	D940_REG	 EMAC_DTE; 	//!< Deferred Transmission Frame Register
	D940_REG	 EMAC_LCOL; 	//!< Late Collision Register
	D940_REG	 EMAC_ECOL; 	//!< Excessive Collision Register
	D940_REG	 EMAC_CSE; 	//!< Carrier Sense Error Register
	D940_REG	 EMAC_TUE; 	//!< Transmit Underrun Error Register
	D940_REG	 EMAC_CDE; 	//!< Code Error Register
	D940_REG	 EMAC_ELR; 	//!< Excessive Length Error Register
	D940_REG	 EMAC_RJB; 	//!< Receive Jabber Register
	D940_REG	 EMAC_USF; 	//!< Undersize Frame Register
	D940_REG	 EMAC_SQEE; 	//!< SQE Test Error Register
	D940_REG	 EMAC_DRFC; 	//!< Discarded RX Frame Register
	D940_REG	 Reserved2[3]; 	//!< 
	D940_REG	 EMAC_HSH; 	//!< Hash Address High[63:32]
	D940_REG	 EMAC_HSL; 	//!< Hash Address Low[31:0]
	D940_REG	 EMAC_SA1L; 	//!< Specific Address 1 Low, First 4 bytes
	D940_REG	 EMAC_SA1H; 	//!< Specific Address 1 High, Last 2 bytes
	D940_REG	 EMAC_SA2L; 	//!< Specific Address 2 Low, First 4 bytes
	D940_REG	 EMAC_SA2H; 	//!< Specific Address 2 High, Last 2 bytes
	D940_REG	 EMAC_SA3L; 	//!< Specific Address 3 Low, First 4 bytes
	D940_REG	 EMAC_SA3H; 	//!< Specific Address 3 High, Last 2 bytes
	D940_REG	 EMAC_SA4L; 	//!< Specific Address 4 Low, First 4 bytes
	D940_REG	 EMAC_SA4H; 	//!< Specific Address 4 High, Last 2 bytesr
} D940S_EMAC,   __MAGICV_EXTERNAL *D940PS_EMAC;

// -------- EMAC_CTL : (EMAC Offset: 0x0)  -------- 
#define D940C_EMAC_LB         ((D940_REG) 0x1 <<  0) //!< (EMAC) Loopback. Optional. When set, loopback signal is at high level.
#define D940C_EMAC_LBL        ((D940_REG) 0x1 <<  1) //!< (EMAC) Loopback local. 
#define D940C_EMAC_RE         ((D940_REG) 0x1 <<  2) //!< (EMAC) Receive enable. 
#define D940C_EMAC_TE         ((D940_REG) 0x1 <<  3) //!< (EMAC) Transmit enable. 
#define D940C_EMAC_MPE        ((D940_REG) 0x1 <<  4) //!< (EMAC) Management port enable. 
#define D940C_EMAC_CSR        ((D940_REG) 0x1 <<  5) //!< (EMAC) Clear statistics registers. 
#define D940C_EMAC_ISR        ((D940_REG) 0x1 <<  6) //!< (EMAC) Increment statistics registers. 
#define D940C_EMAC_WES        ((D940_REG) 0x1 <<  7) //!< (EMAC) Write enable for statistics registers. 
#define D940C_EMAC_BP         ((D940_REG) 0x1 <<  8) //!< (EMAC) Back pressure. 
// -------- EMAC_CFG : (EMAC Offset: 0x4) Network Configuration Register -------- 
#define D940C_EMAC_SPD        ((D940_REG) 0x1 <<  0) //!< (EMAC) Speed. 
#define D940C_EMAC_FD         ((D940_REG) 0x1 <<  1) //!< (EMAC) Full duplex. 
#define D940C_EMAC_BR         ((D940_REG) 0x1 <<  2) //!< (EMAC) Bit rate. 
#define D940C_EMAC_CAF        ((D940_REG) 0x1 <<  4) //!< (EMAC) Copy all frames. 
#define D940C_EMAC_NBC        ((D940_REG) 0x1 <<  5) //!< (EMAC) No broadcast. 
#define D940C_EMAC_MTI        ((D940_REG) 0x1 <<  6) //!< (EMAC) Multicast hash enable
#define D940C_EMAC_UNI        ((D940_REG) 0x1 <<  7) //!< (EMAC) Unicast hash enable. 
#define D940C_EMAC_BIG        ((D940_REG) 0x1 <<  8) //!< (EMAC) Receive 1522 bytes. 
#define D940C_EMAC_EAE        ((D940_REG) 0x1 <<  9) //!< (EMAC) External address match enable. 
#define D940C_EMAC_CLK        ((D940_REG) 0x3 << 10) //!< (EMAC) 
#define 	D940C_EMAC_CLK_HCLK_8               ((D940_REG) 0x0 << 10) //!< (EMAC) HCLK divided by 8
#define 	D940C_EMAC_CLK_HCLK_16              ((D940_REG) 0x1 << 10) //!< (EMAC) HCLK divided by 16
#define 	D940C_EMAC_CLK_HCLK_32              ((D940_REG) 0x2 << 10) //!< (EMAC) HCLK divided by 32
#define 	D940C_EMAC_CLK_HCLK_64              ((D940_REG) 0x3 << 10) //!< (EMAC) HCLK divided by 64
#define D940C_EMAC_RTY        ((D940_REG) 0x1 << 12) //!< (EMAC) 
#define D940C_EMAC_RMII       ((D940_REG) 0x1 << 13) //!< (EMAC) 
// -------- EMAC_SR : (EMAC Offset: 0x8) Network Status Register -------- 
#define D940C_EMAC_MDIO       ((D940_REG) 0x1 <<  1) //!< (EMAC) 
#define D940C_EMAC_IDLE       ((D940_REG) 0x1 <<  2) //!< (EMAC) 
// -------- EMAC_TCR : (EMAC Offset: 0x10) Transmit Control Register -------- 
#define D940C_EMAC_LEN        ((D940_REG) 0x7FF <<  0) //!< (EMAC) 
#define D940C_EMAC_NCRC       ((D940_REG) 0x1 << 15) //!< (EMAC) 
// -------- EMAC_TSR : (EMAC Offset: 0x14) Transmit Control Register -------- 
#define D940C_EMAC_OVR        ((D940_REG) 0x1 <<  0) //!< (EMAC) 
#define D940C_EMAC_COL        ((D940_REG) 0x1 <<  1) //!< (EMAC) 
#define D940C_EMAC_RLE        ((D940_REG) 0x1 <<  2) //!< (EMAC) 
#define D940C_EMAC_TXIDLE     ((D940_REG) 0x1 <<  3) //!< (EMAC) 
#define D940C_EMAC_BNQ        ((D940_REG) 0x1 <<  4) //!< (EMAC) 
#define D940C_EMAC_COMP       ((D940_REG) 0x1 <<  5) //!< (EMAC) 
#define D940C_EMAC_UND        ((D940_REG) 0x1 <<  6) //!< (EMAC) 
// -------- EMAC_RSR : (EMAC Offset: 0x20) Receive Status Register -------- 
#define D940C_EMAC_BNA        ((D940_REG) 0x1 <<  0) //!< (EMAC) 
#define D940C_EMAC_REC        ((D940_REG) 0x1 <<  1) //!< (EMAC) 
// -------- EMAC_ISR : (EMAC Offset: 0x24) Interrupt Status Register -------- 
#define D940C_EMAC_DONE       ((D940_REG) 0x1 <<  0) //!< (EMAC) 
#define D940C_EMAC_RCOM       ((D940_REG) 0x1 <<  1) //!< (EMAC) 
#define D940C_EMAC_RBNA       ((D940_REG) 0x1 <<  2) //!< (EMAC) 
#define D940C_EMAC_TOVR       ((D940_REG) 0x1 <<  3) //!< (EMAC) 
#define D940C_EMAC_TUND       ((D940_REG) 0x1 <<  4) //!< (EMAC) 
#define D940C_EMAC_RTRY       ((D940_REG) 0x1 <<  5) //!< (EMAC) 
#define D940C_EMAC_TBRE       ((D940_REG) 0x1 <<  6) //!< (EMAC) 
#define D940C_EMAC_TCOM       ((D940_REG) 0x1 <<  7) //!< (EMAC) 
#define D940C_EMAC_TIDLE      ((D940_REG) 0x1 <<  8) //!< (EMAC) 
#define D940C_EMAC_LINK       ((D940_REG) 0x1 <<  9) //!< (EMAC) 
#define D940C_EMAC_ROVR       ((D940_REG) 0x1 << 10) //!< (EMAC) 
#define D940C_EMAC_HRESP      ((D940_REG) 0x1 << 11) //!< (EMAC) 
// -------- EMAC_IER : (EMAC Offset: 0x28) Interrupt Enable Register -------- 
// -------- EMAC_IDR : (EMAC Offset: 0x2c) Interrupt Disable Register -------- 
// -------- EMAC_IMR : (EMAC Offset: 0x30) Interrupt Mask Register -------- 
// -------- EMAC_MAN : (EMAC Offset: 0x34) PHY Maintenance Register -------- 
#define D940C_EMAC_DATA       ((D940_REG) 0xFFFF <<  0) //!< (EMAC) 
#define D940C_EMAC_CODE       ((D940_REG) 0x3 << 16) //!< (EMAC) 
#define D940C_EMAC_REGA       ((D940_REG) 0x1F << 18) //!< (EMAC) 
#define D940C_EMAC_PHYA       ((D940_REG) 0x1F << 23) //!< (EMAC) 
#define D940C_EMAC_RW         ((D940_REG) 0x3 << 28) //!< (EMAC) 
#define D940C_EMAC_HIGH       ((D940_REG) 0x1 << 30) //!< (EMAC) 
#define D940C_EMAC_LOW        ((D940_REG) 0x1 << 31) //!< (EMAC) 

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR Control Area Network MailBox Interface
// *****************************************************************************
typedef struct _D940S_CAN_MB {
	D940_REG	 CAN_MB_MMR; 	//!< MailBox Mode Register
	D940_REG	 CAN_MB_MAM; 	//!< MailBox Acceptance Mask Register
	D940_REG	 CAN_MB_MID; 	//!< MailBox ID Register
	D940_REG	 CAN_MB_MFID; 	//!< MailBox Family ID Register
	D940_REG	 CAN_MB_MSR; 	//!< MailBox Status Register
	D940_REG	 CAN_MB_MDL; 	//!< MailBox Data Low Register
	D940_REG	 CAN_MB_MDH; 	//!< MailBox Data High Register
	D940_REG	 CAN_MB_MCR; 	//!< MailBox Control Register
} D940S_CAN_MB,   __MAGICV_EXTERNAL *D940PS_CAN_MB;

// -------- CAN_MMR : (CAN_MB Offset: 0x0) CAN Message Mode Register -------- 
#define D940C_CAN_MTIMEMARK   ((D940_REG) 0xFFFF <<  0) //!< (CAN_MB) Mailbox Timemark
#define D940C_CAN_PRIOR       ((D940_REG) 0xF << 16) //!< (CAN_MB) Mailbox Priority
#define D940C_CAN_MOT         ((D940_REG) 0x7 << 24) //!< (CAN_MB) Mailbox Object Type
#define 	D940C_CAN_MOT_DIS                  ((D940_REG) 0x0 << 24) //!< (CAN_MB) 
#define 	D940C_CAN_MOT_RX                   ((D940_REG) 0x1 << 24) //!< (CAN_MB) 
#define 	D940C_CAN_MOT_RXOVERWRITE          ((D940_REG) 0x2 << 24) //!< (CAN_MB) 
#define 	D940C_CAN_MOT_TX                   ((D940_REG) 0x3 << 24) //!< (CAN_MB) 
#define 	D940C_CAN_MOT_CONSUMER             ((D940_REG) 0x4 << 24) //!< (CAN_MB) 
#define 	D940C_CAN_MOT_PRODUCER             ((D940_REG) 0x5 << 24) //!< (CAN_MB) 
// -------- CAN_MAM : (CAN_MB Offset: 0x4) CAN Message Acceptance Mask Register -------- 
#define D940C_CAN_MIDvB       ((D940_REG) 0x3FFFF <<  0) //!< (CAN_MB) Complementary bits for identifier in extended mode
#define D940C_CAN_MIDvA       ((D940_REG) 0x7FF << 18) //!< (CAN_MB) Identifier for standard frame mode
#define D940C_CAN_MIDE        ((D940_REG) 0x1 << 29) //!< (CAN_MB) Identifier Version
// -------- CAN_MID : (CAN_MB Offset: 0x8) CAN Message ID Register -------- 
// -------- CAN_MFID : (CAN_MB Offset: 0xc) CAN Message Family ID Register -------- 
// -------- CAN_MSR : (CAN_MB Offset: 0x10) CAN Message Status Register -------- 
#define D940C_CAN_MTIMESTAMP  ((D940_REG) 0xFFFF <<  0) //!< (CAN_MB) Timer Value
#define D940C_CAN_MDLC        ((D940_REG) 0xF << 16) //!< (CAN_MB) Mailbox Data Length Code
#define D940C_CAN_MRTR        ((D940_REG) 0x1 << 20) //!< (CAN_MB) Mailbox Remote Transmission Request
#define D940C_CAN_MABT        ((D940_REG) 0x1 << 22) //!< (CAN_MB) Mailbox Message Abort
#define D940C_CAN_MRDY        ((D940_REG) 0x1 << 23) //!< (CAN_MB) Mailbox Ready
#define D940C_CAN_MMI         ((D940_REG) 0x1 << 24) //!< (CAN_MB) Mailbox Message Ignored
// -------- CAN_MDL : (CAN_MB Offset: 0x14) CAN Message Data Low Register -------- 
// -------- CAN_MDH : (CAN_MB Offset: 0x18) CAN Message Data High Register -------- 
// -------- CAN_MCR : (CAN_MB Offset: 0x1c) CAN Message Control Register -------- 
#define D940C_CAN_MACR        ((D940_REG) 0x1 << 22) //!< (CAN_MB) Abort Request for Mailbox
#define D940C_CAN_MTCR        ((D940_REG) 0x1 << 23) //!< (CAN_MB) Mailbox Transfer Command

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR Control Area Network Interface
// *****************************************************************************
typedef struct _D940S_CAN {
	D940_REG	 CAN_MR; 	//!< Mode Register
	D940_REG	 CAN_IER; 	//!< Interrupt Enable Register
	D940_REG	 CAN_IDR; 	//!< Interrupt Disable Register
	D940_REG	 CAN_IMR; 	//!< Interrupt Mask Register
	D940_REG	 CAN_SR; 	//!< Status Register
	D940_REG	 CAN_BR; 	//!< Baudrate Register
	D940_REG	 CAN_TIM; 	//!< Timer Register
	D940_REG	 CAN_TIMESTP; 	//!< Time Stamp Register
	D940_REG	 CAN_ECR; 	//!< Error Counter Register
	D940_REG	 CAN_TCR; 	//!< Transfer Command Register
	D940_REG	 CAN_ACR; 	//!< Abort Command Register
	D940_REG	 Reserved0[52]; 	//!< 
	D940_REG	 CAN_VR; 	//!< Version Register
	D940_REG	 Reserved1[64]; 	//!< 
	D940S_CAN_MB	 CAN_MB0; 	//!< CAN Mailbox 0
	D940S_CAN_MB	 CAN_MB1; 	//!< CAN Mailbox 1
	D940S_CAN_MB	 CAN_MB2; 	//!< CAN Mailbox 2
	D940S_CAN_MB	 CAN_MB3; 	//!< CAN Mailbox 3
	D940S_CAN_MB	 CAN_MB4; 	//!< CAN Mailbox 4
	D940S_CAN_MB	 CAN_MB5; 	//!< CAN Mailbox 5
	D940S_CAN_MB	 CAN_MB6; 	//!< CAN Mailbox 6
	D940S_CAN_MB	 CAN_MB7; 	//!< CAN Mailbox 7
	D940S_CAN_MB	 CAN_MB8; 	//!< CAN Mailbox 8
	D940S_CAN_MB	 CAN_MB9; 	//!< CAN Mailbox 9
	D940S_CAN_MB	 CAN_MB10; 	//!< CAN Mailbox 10
	D940S_CAN_MB	 CAN_MB11; 	//!< CAN Mailbox 11
	D940S_CAN_MB	 CAN_MB12; 	//!< CAN Mailbox 12
	D940S_CAN_MB	 CAN_MB13; 	//!< CAN Mailbox 13
	D940S_CAN_MB	 CAN_MB14; 	//!< CAN Mailbox 14
	D940S_CAN_MB	 CAN_MB15; 	//!< CAN Mailbox 15
} D940S_CAN,   __MAGICV_EXTERNAL *D940PS_CAN;

// -------- CAN_MR : (CAN Offset: 0x0) CAN Mode Register -------- 
#define D940C_CAN_CANEN       ((D940_REG) 0x1 <<  0) //!< (CAN) CAN Controller Enable
#define D940C_CAN_LPM         ((D940_REG) 0x1 <<  1) //!< (CAN) Disable/Enable Low Power Mode
#define D940C_CAN_ABM         ((D940_REG) 0x1 <<  2) //!< (CAN) Disable/Enable Autobaud/Listen Mode
#define D940C_CAN_OVL         ((D940_REG) 0x1 <<  3) //!< (CAN) Disable/Enable Overload Frame
#define D940C_CAN_TEOF        ((D940_REG) 0x1 <<  4) //!< (CAN) Time Stamp messages at each end of Frame
#define D940C_CAN_TTM         ((D940_REG) 0x1 <<  5) //!< (CAN) Disable/Enable Time Trigger Mode
#define D940C_CAN_TIMFRZ      ((D940_REG) 0x1 <<  6) //!< (CAN) Enable Timer Freeze
#define D940C_CAN_DRPT        ((D940_REG) 0x1 <<  7) //!< (CAN) Disable Repeat
// -------- CAN_IER : (CAN Offset: 0x4) CAN Interrupt Enable Register -------- 
#define D940C_CAN_MB0         ((D940_REG) 0x1 <<  0) //!< (CAN) Mailbox 0 Flag
#define D940C_CAN_MB1         ((D940_REG) 0x1 <<  1) //!< (CAN) Mailbox 1 Flag
#define D940C_CAN_MB2         ((D940_REG) 0x1 <<  2) //!< (CAN) Mailbox 2 Flag
#define D940C_CAN_MB3         ((D940_REG) 0x1 <<  3) //!< (CAN) Mailbox 3 Flag
#define D940C_CAN_MB4         ((D940_REG) 0x1 <<  4) //!< (CAN) Mailbox 4 Flag
#define D940C_CAN_MB5         ((D940_REG) 0x1 <<  5) //!< (CAN) Mailbox 5 Flag
#define D940C_CAN_MB6         ((D940_REG) 0x1 <<  6) //!< (CAN) Mailbox 6 Flag
#define D940C_CAN_MB7         ((D940_REG) 0x1 <<  7) //!< (CAN) Mailbox 7 Flag
#define D940C_CAN_MB8         ((D940_REG) 0x1 <<  8) //!< (CAN) Mailbox 8 Flag
#define D940C_CAN_MB9         ((D940_REG) 0x1 <<  9) //!< (CAN) Mailbox 9 Flag
#define D940C_CAN_MB10        ((D940_REG) 0x1 << 10) //!< (CAN) Mailbox 10 Flag
#define D940C_CAN_MB11        ((D940_REG) 0x1 << 11) //!< (CAN) Mailbox 11 Flag
#define D940C_CAN_MB12        ((D940_REG) 0x1 << 12) //!< (CAN) Mailbox 12 Flag
#define D940C_CAN_MB13        ((D940_REG) 0x1 << 13) //!< (CAN) Mailbox 13 Flag
#define D940C_CAN_MB14        ((D940_REG) 0x1 << 14) //!< (CAN) Mailbox 14 Flag
#define D940C_CAN_MB15        ((D940_REG) 0x1 << 15) //!< (CAN) Mailbox 15 Flag
#define D940C_CAN_ERRA        ((D940_REG) 0x1 << 16) //!< (CAN) Error Active Mode Flag
#define D940C_CAN_WARN        ((D940_REG) 0x1 << 17) //!< (CAN) Warning Limit Flag
#define D940C_CAN_ERRP        ((D940_REG) 0x1 << 18) //!< (CAN) Error Passive Mode Flag
#define D940C_CAN_BOFF        ((D940_REG) 0x1 << 19) //!< (CAN) Bus Off Mode Flag
#define D940C_CAN_SLEEP       ((D940_REG) 0x1 << 20) //!< (CAN) Sleep Flag
#define D940C_CAN_WAKEUP      ((D940_REG) 0x1 << 21) //!< (CAN) Wakeup Flag
#define D940C_CAN_TOVF        ((D940_REG) 0x1 << 22) //!< (CAN) Timer Overflow Flag
#define D940C_CAN_TSTP        ((D940_REG) 0x1 << 23) //!< (CAN) Timestamp Flag
#define D940C_CAN_CERR        ((D940_REG) 0x1 << 24) //!< (CAN) CRC Error
#define D940C_CAN_SERR        ((D940_REG) 0x1 << 25) //!< (CAN) Stuffing Error
#define D940C_CAN_AERR        ((D940_REG) 0x1 << 26) //!< (CAN) Acknowledgment Error
#define D940C_CAN_FERR        ((D940_REG) 0x1 << 27) //!< (CAN) Form Error
#define D940C_CAN_BERR        ((D940_REG) 0x1 << 28) //!< (CAN) Bit Error
// -------- CAN_IDR : (CAN Offset: 0x8) CAN Interrupt Disable Register -------- 
// -------- CAN_IMR : (CAN Offset: 0xc) CAN Interrupt Mask Register -------- 
// -------- CAN_SR : (CAN Offset: 0x10) CAN Status Register -------- 
#define D940C_CAN_RBSY        ((D940_REG) 0x1 << 29) //!< (CAN) Receiver Busy
#define D940C_CAN_TBSY        ((D940_REG) 0x1 << 30) //!< (CAN) Transmitter Busy
#define D940C_CAN_OVLY        ((D940_REG) 0x1 << 31) //!< (CAN) Overload Busy
// -------- CAN_BR : (CAN Offset: 0x14) CAN Baudrate Register -------- 
#define D940C_CAN_PHASE2      ((D940_REG) 0x7 <<  0) //!< (CAN) Phase 2 segment
#define D940C_CAN_PHASE1      ((D940_REG) 0x7 <<  4) //!< (CAN) Phase 1 segment
#define D940C_CAN_PROPAG      ((D940_REG) 0x7 <<  8) //!< (CAN) Programmation time segment
#define D940C_CAN_SYNC        ((D940_REG) 0x3 << 12) //!< (CAN) Re-synchronization jump width segment
#define D940C_CAN_BRP         ((D940_REG) 0x7F << 16) //!< (CAN) Baudrate Prescaler
#define D940C_CAN_SMP         ((D940_REG) 0x1 << 24) //!< (CAN) Sampling mode
// -------- CAN_TIM : (CAN Offset: 0x18) CAN Timer Register -------- 
#define D940C_CAN_TIMER       ((D940_REG) 0xFFFF <<  0) //!< (CAN) Timer field
// -------- CAN_TIMESTP : (CAN Offset: 0x1c) CAN Timestamp Register -------- 
// -------- CAN_ECR : (CAN Offset: 0x20) CAN Error Counter Register -------- 
#define D940C_CAN_REC         ((D940_REG) 0xFF <<  0) //!< (CAN) Receive Error Counter
#define D940C_CAN_TEC         ((D940_REG) 0xFF << 16) //!< (CAN) Transmit Error Counter
// -------- CAN_TCR : (CAN Offset: 0x24) CAN Transfer Command Register -------- 
#define D940C_CAN_TIMRST      ((D940_REG) 0x1 << 31) //!< (CAN) Timer Reset Field
// -------- CAN_ACR : (CAN Offset: 0x28) CAN Abort Command Register -------- 

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR Multimedia Card Interface
// *****************************************************************************
typedef struct _D940S_MCI {
	D940_REG	 MCI_CR; 	//!< MCI Control Register
	D940_REG	 MCI_MR; 	//!< MCI Mode Register
	D940_REG	 MCI_DTOR; 	//!< MCI Data Timeout Register
	D940_REG	 MCI_SDCR; 	//!< MCI SD Card Register
	D940_REG	 MCI_ARGR; 	//!< MCI Argument Register
	D940_REG	 MCI_CMDR; 	//!< MCI Command Register
	D940_REG	 Reserved0[2]; 	//!< 
	D940_REG	 MCI_RSPR[4]; 	//!< MCI Response Register
	D940_REG	 MCI_RDR; 	//!< MCI Receive Data Register
	D940_REG	 MCI_TDR; 	//!< MCI Transmit Data Register
	D940_REG	 Reserved1[2]; 	//!< 
	D940_REG	 MCI_SR; 	//!< MCI Status Register
	D940_REG	 MCI_IER; 	//!< MCI Interrupt Enable Register
	D940_REG	 MCI_IDR; 	//!< MCI Interrupt Disable Register
	D940_REG	 MCI_IMR; 	//!< MCI Interrupt Mask Register
	D940_REG	 Reserved2[44]; 	//!< 
	D940_REG	 MCI_RPR; 	//!< Receive Pointer Register
	D940_REG	 MCI_RCR; 	//!< Receive Counter Register
	D940_REG	 MCI_TPR; 	//!< Transmit Pointer Register
	D940_REG	 MCI_TCR; 	//!< Transmit Counter Register
	D940_REG	 MCI_RNPR; 	//!< Receive Next Pointer Register
	D940_REG	 MCI_RNCR; 	//!< Receive Next Counter Register
	D940_REG	 MCI_TNPR; 	//!< Transmit Next Pointer Register
	D940_REG	 MCI_TNCR; 	//!< Transmit Next Counter Register
	D940_REG	 MCI_PTCR; 	//!< PDC Transfer Control Register
	D940_REG	 MCI_PTSR; 	//!< PDC Transfer Status Register
} D940S_MCI,   __MAGICV_EXTERNAL *D940PS_MCI;

// -------- MCI_CR : (MCI Offset: 0x0) MCI Control Register -------- 
#define D940C_MCI_MCIEN       ((D940_REG) 0x1 <<  0) //!< (MCI) Multimedia Interface Enable
#define D940C_MCI_MCIDIS      ((D940_REG) 0x1 <<  1) //!< (MCI) Multimedia Interface Disable
#define D940C_MCI_PWSEN       ((D940_REG) 0x1 <<  2) //!< (MCI) Power Save Mode Enable
#define D940C_MCI_PWSDIS      ((D940_REG) 0x1 <<  3) //!< (MCI) Power Save Mode Disable
#define D940C_MCI_SWRST       ((D940_REG) 0x1 <<  7) //!< (MCI) MCI Software reset
// -------- MCI_MR : (MCI Offset: 0x4) MCI Mode Register -------- 
#define D940C_MCI_CLKDIV      ((D940_REG) 0xFF <<  0) //!< (MCI) Clock Divider
#define D940C_MCI_PWSDIV      ((D940_REG) 0x7 <<  8) //!< (MCI) Power Saving Divider
#define D940C_MCI_PDCPADV     ((D940_REG) 0x1 << 14) //!< (MCI) PDC Padding Value
#define D940C_MCI_PDCMODE     ((D940_REG) 0x1 << 15) //!< (MCI) PDC Oriented Mode
#define D940C_MCI_BLKLEN      ((D940_REG) 0xFFF << 18) //!< (MCI) Data Block Length
// -------- MCI_DTOR : (MCI Offset: 0x8) MCI Data Timeout Register -------- 
#define D940C_MCI_DTOCYC      ((D940_REG) 0xF <<  0) //!< (MCI) Data Timeout Cycle Number
#define D940C_MCI_DTOMUL      ((D940_REG) 0x7 <<  4) //!< (MCI) Data Timeout Multiplier
#define 	D940C_MCI_DTOMUL_1                    ((D940_REG) 0x0 <<  4) //!< (MCI) DTOCYC x 1
#define 	D940C_MCI_DTOMUL_16                   ((D940_REG) 0x1 <<  4) //!< (MCI) DTOCYC x 16
#define 	D940C_MCI_DTOMUL_128                  ((D940_REG) 0x2 <<  4) //!< (MCI) DTOCYC x 128
#define 	D940C_MCI_DTOMUL_256                  ((D940_REG) 0x3 <<  4) //!< (MCI) DTOCYC x 256
#define 	D940C_MCI_DTOMUL_1024                 ((D940_REG) 0x4 <<  4) //!< (MCI) DTOCYC x 1024
#define 	D940C_MCI_DTOMUL_4096                 ((D940_REG) 0x5 <<  4) //!< (MCI) DTOCYC x 4096
#define 	D940C_MCI_DTOMUL_65536                ((D940_REG) 0x6 <<  4) //!< (MCI) DTOCYC x 65536
#define 	D940C_MCI_DTOMUL_1048576              ((D940_REG) 0x7 <<  4) //!< (MCI) DTOCYC x 1048576
// -------- MCI_SDCR : (MCI Offset: 0xc) MCI SD Card Register -------- 
#define D940C_MCI_SCDSEL      ((D940_REG) 0xF <<  0) //!< (MCI) SD Card Selector
#define D940C_MCI_SCDBUS      ((D940_REG) 0x1 <<  7) //!< (MCI) SD Card Bus Width
// -------- MCI_CMDR : (MCI Offset: 0x14) MCI Command Register -------- 
#define D940C_MCI_CMDNB       ((D940_REG) 0x3F <<  0) //!< (MCI) Command Number
#define D940C_MCI_RSPTYP      ((D940_REG) 0x3 <<  6) //!< (MCI) Response Type
#define 	D940C_MCI_RSPTYP_NO                   ((D940_REG) 0x0 <<  6) //!< (MCI) No response
#define 	D940C_MCI_RSPTYP_48                   ((D940_REG) 0x1 <<  6) //!< (MCI) 48-bit response
#define 	D940C_MCI_RSPTYP_136                  ((D940_REG) 0x2 <<  6) //!< (MCI) 136-bit response
#define D940C_MCI_SPCMD       ((D940_REG) 0x7 <<  8) //!< (MCI) Special CMD
#define 	D940C_MCI_SPCMD_NONE                 ((D940_REG) 0x0 <<  8) //!< (MCI) Not a special CMD
#define 	D940C_MCI_SPCMD_INIT                 ((D940_REG) 0x1 <<  8) //!< (MCI) Initialization CMD
#define 	D940C_MCI_SPCMD_SYNC                 ((D940_REG) 0x2 <<  8) //!< (MCI) Synchronized CMD
#define 	D940C_MCI_SPCMD_IT_CMD               ((D940_REG) 0x4 <<  8) //!< (MCI) Interrupt command
#define 	D940C_MCI_SPCMD_IT_REP               ((D940_REG) 0x5 <<  8) //!< (MCI) Interrupt response
#define D940C_MCI_OPDCMD      ((D940_REG) 0x1 << 11) //!< (MCI) Open Drain Command
#define D940C_MCI_MAXLAT      ((D940_REG) 0x1 << 12) //!< (MCI) Maximum Latency for Command to respond
#define D940C_MCI_TRCMD       ((D940_REG) 0x3 << 16) //!< (MCI) Transfer CMD
#define 	D940C_MCI_TRCMD_NO                   ((D940_REG) 0x0 << 16) //!< (MCI) No transfer
#define 	D940C_MCI_TRCMD_START                ((D940_REG) 0x1 << 16) //!< (MCI) Start transfer
#define 	D940C_MCI_TRCMD_STOP                 ((D940_REG) 0x2 << 16) //!< (MCI) Stop transfer
#define D940C_MCI_TRDIR       ((D940_REG) 0x1 << 18) //!< (MCI) Transfer Direction
#define D940C_MCI_TRTYP       ((D940_REG) 0x3 << 19) //!< (MCI) Transfer Type
#define 	D940C_MCI_TRTYP_BLOCK                ((D940_REG) 0x0 << 19) //!< (MCI) Block Transfer type
#define 	D940C_MCI_TRTYP_MULTIPLE             ((D940_REG) 0x1 << 19) //!< (MCI) Multiple Block transfer type
#define 	D940C_MCI_TRTYP_STREAM               ((D940_REG) 0x2 << 19) //!< (MCI) Stream transfer type
// -------- MCI_SR : (MCI Offset: 0x40) MCI Status Register -------- 
#define D940C_MCI_CMDRDY      ((D940_REG) 0x1 <<  0) //!< (MCI) Command Ready flag
#define D940C_MCI_RXRDY       ((D940_REG) 0x1 <<  1) //!< (MCI) RX Ready flag
#define D940C_MCI_TXRDY       ((D940_REG) 0x1 <<  2) //!< (MCI) TX Ready flag
#define D940C_MCI_BLKE        ((D940_REG) 0x1 <<  3) //!< (MCI) Data Block Transfer Ended flag
#define D940C_MCI_DTIP        ((D940_REG) 0x1 <<  4) //!< (MCI) Data Transfer in Progress flag
#define D940C_MCI_NOTBUSY     ((D940_REG) 0x1 <<  5) //!< (MCI) Data Line Not Busy flag
#define D940C_MCI_ENDRX       ((D940_REG) 0x1 <<  6) //!< (MCI) End of RX Buffer flag
#define D940C_MCI_ENDTX       ((D940_REG) 0x1 <<  7) //!< (MCI) End of TX Buffer flag
#define D940C_MCI_RXBUFF      ((D940_REG) 0x1 << 14) //!< (MCI) RX Buffer Full flag
#define D940C_MCI_TXBUFE      ((D940_REG) 0x1 << 15) //!< (MCI) TX Buffer Empty flag
#define D940C_MCI_RINDE       ((D940_REG) 0x1 << 16) //!< (MCI) Response Index Error flag
#define D940C_MCI_RDIRE       ((D940_REG) 0x1 << 17) //!< (MCI) Response Direction Error flag
#define D940C_MCI_RCRCE       ((D940_REG) 0x1 << 18) //!< (MCI) Response CRC Error flag
#define D940C_MCI_RENDE       ((D940_REG) 0x1 << 19) //!< (MCI) Response End Bit Error flag
#define D940C_MCI_RTOE        ((D940_REG) 0x1 << 20) //!< (MCI) Response Time-out Error flag
#define D940C_MCI_DCRCE       ((D940_REG) 0x1 << 21) //!< (MCI) data CRC Error flag
#define D940C_MCI_DTOE        ((D940_REG) 0x1 << 22) //!< (MCI) Data timeout Error flag
#define D940C_MCI_OVRE        ((D940_REG) 0x1 << 30) //!< (MCI) Overrun flag
#define D940C_MCI_UNRE        ((D940_REG) 0x1 << 31) //!< (MCI) Underrun flag
// -------- MCI_IER : (MCI Offset: 0x44) MCI Interrupt Enable Register -------- 
// -------- MCI_IDR : (MCI Offset: 0x48) MCI Interrupt Disable Register -------- 
// -------- MCI_IMR : (MCI Offset: 0x4c) MCI Interrupt Mask Register -------- 

// *****************************************************************************
//               REGISTER ADDRESS DEFINITION FOR D940HF
// *****************************************************************************

// ========== Register definition for SYS peripheral ========== 
#define D940C_SYS_GPBR1 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFD54) //!< (SYS) General Purpose Register 1
#define D940C_SYS_GPBR3 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFD5C) //!< (SYS) General Purpose Register 3
#define D940C_SYS_GPBR2 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFD58) //!< (SYS) General Purpose Register 2
#define D940C_SYS_GPBR0 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFD50) //!< (SYS) General Purpose Register 0
// ========== Register definition for SDRAMC peripheral ========== 
#define D940C_SDRAMC_MR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEA00) //!< (SDRAMC) SDRAM Controller Mode Register
#define D940C_SDRAMC_IMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEA1C) //!< (SDRAMC) SDRAM Controller Interrupt Mask Register
#define D940C_SDRAMC_LPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEA10) //!< (SDRAMC) SDRAM Controller Low Power Register
#define D940C_SDRAMC_MDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEA24) //!< (SDRAMC) SDRAM Memory Device Register
#define D940C_SDRAMC_IER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEA14) //!< (SDRAMC) SDRAM Controller Interrupt Enable Register
#define D940C_SDRAMC_HSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEA0C) //!< (SDRAMC) SDRAM Controller High Speed Register
#define D940C_SDRAMC_ISR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEA20) //!< (SDRAMC) SDRAM Controller Interrupt Mask Register
#define D940C_SDRAMC_IDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEA18) //!< (SDRAMC) SDRAM Controller Interrupt Disable Register
#define D940C_SDRAMC_CR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEA08) //!< (SDRAMC) SDRAM Controller Configuration Register
#define D940C_SDRAMC_TR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEA04) //!< (SDRAMC) SDRAM Controller Refresh Timer Register
// ========== Register definition for SMC peripheral ========== 
#define D940C_SMC_CTRL1 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC1C) //!< (SMC)  Control Register for CS 1
#define D940C_SMC_PULSE7 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC74) //!< (SMC)  Pulse Register for CS 7
#define D940C_SMC_PULSE6 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC64) //!< (SMC)  Pulse Register for CS 6
#define D940C_SMC_PULSE3 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC34) //!< (SMC)  Pulse Register for CS 3
#define D940C_SMC_CYCLE5 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC58) //!< (SMC)  Cycle Register for CS 5
#define D940C_SMC_CTRL0 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC0C) //!< (SMC)  Control Register for CS 0
#define D940C_SMC_PULSE5 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC54) //!< (SMC)  Pulse Register for CS 5
#define D940C_SMC_PULSE0 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC04) //!< (SMC)  Pulse Register for CS 0
#define D940C_SMC_CTRL4 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC4C) //!< (SMC)  Control Register for CS 4
#define D940C_SMC_SETUP1 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC10) //!< (SMC)  Setup Register for CS 1
#define D940C_SMC_CYCLE0 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC08) //!< (SMC)  Cycle Register for CS 0
#define D940C_SMC_CTRL6 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC6C) //!< (SMC)  Control Register for CS 6
#define D940C_SMC_SETUP5 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC50) //!< (SMC)  Setup Register for CS 5
#define D940C_SMC_SETUP2 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC20) //!< (SMC)  Setup Register for CS 2
#define D940C_SMC_CYCLE6 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC68) //!< (SMC)  Cycle Register for CS 6
#define D940C_SMC_PULSE2 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC24) //!< (SMC)  Pulse Register for CS 2
#define D940C_SMC_SETUP4 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC40) //!< (SMC)  Setup Register for CS 4
#define D940C_SMC_CYCLE2 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC28) //!< (SMC)  Cycle Register for CS 2
#define D940C_SMC_CTRL2 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC2C) //!< (SMC)  Control Register for CS 2
#define D940C_SMC_PULSE1 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC14) //!< (SMC)  Pulse Register for CS 1
#define D940C_SMC_CYCLE7 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC78) //!< (SMC)  Cycle Register for CS 7
#define D940C_SMC_CTRL3 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC3C) //!< (SMC)  Control Register for CS 3
#define D940C_SMC_SETUP7 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC70) //!< (SMC)  Setup Register for CS 7
#define D940C_SMC_CTRL7 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC7C) //!< (SMC)  Control Register for CS 7
#define D940C_SMC_CTRL5 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC5C) //!< (SMC)  Control Register for CS 5
#define D940C_SMC_CYCLE1 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC18) //!< (SMC)  Cycle Register for CS 1
#define D940C_SMC_SETUP0 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC00) //!< (SMC)  Setup Register for CS 0
#define D940C_SMC_PULSE4 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC44) //!< (SMC)  Pulse Register for CS 4
#define D940C_SMC_CYCLE3 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC38) //!< (SMC)  Cycle Register for CS 3
#define D940C_SMC_CYCLE4 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC48) //!< (SMC)  Cycle Register for CS 4
#define D940C_SMC_SETUP6 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC60) //!< (SMC)  Setup Register for CS 6
#define D940C_SMC_SETUP3 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC30) //!< (SMC)  Setup Register for CS 3
// ========== Register definition for SMCII peripheral ========== 
#define D940C_SMCII_CTRL1 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC1C) //!< (SMCII)  Control Register for CS 1
#define D940C_SMCII_PULSE7 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC74) //!< (SMCII)  Pulse Register for CS 7
#define D940C_SMCII_PULSE6 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC64) //!< (SMCII)  Pulse Register for CS 6
#define D940C_SMCII_PULSE3 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC34) //!< (SMCII)  Pulse Register for CS 3
#define D940C_SMCII_CYCLE5 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC58) //!< (SMCII)  Cycle Register for CS 5
#define D940C_SMCII_CTRL0 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC0C) //!< (SMCII)  Control Register for CS 0
#define D940C_SMCII_PULSE5 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC54) //!< (SMCII)  Pulse Register for CS 5
#define D940C_SMCII_PULSE0 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC04) //!< (SMCII)  Pulse Register for CS 0
#define D940C_SMCII_CTRL4 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC4C) //!< (SMCII)  Control Register for CS 4
#define D940C_SMCII_SETUP1 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC10) //!< (SMCII)  Setup Register for CS 1
#define D940C_SMCII_CYCLE0 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC08) //!< (SMCII)  Cycle Register for CS 0
#define D940C_SMCII_CTRL6 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC6C) //!< (SMCII)  Control Register for CS 6
#define D940C_SMCII_SETUP5 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC50) //!< (SMCII)  Setup Register for CS 5
#define D940C_SMCII_SETUP2 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC20) //!< (SMCII)  Setup Register for CS 2
#define D940C_SMCII_CYCLE6 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC68) //!< (SMCII)  Cycle Register for CS 6
#define D940C_SMCII_PULSE2 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC24) //!< (SMCII)  Pulse Register for CS 2
#define D940C_SMCII_SETUP4 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC40) //!< (SMCII)  Setup Register for CS 4
#define D940C_SMCII_CYCLE2 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC28) //!< (SMCII)  Cycle Register for CS 2
#define D940C_SMCII_CTRL2 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC2C) //!< (SMCII)  Control Register for CS 2
#define D940C_SMCII_PULSE1 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC14) //!< (SMCII)  Pulse Register for CS 1
#define D940C_SMCII_CYCLE7 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC78) //!< (SMCII)  Cycle Register for CS 7
#define D940C_SMCII_CTRL3 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC3C) //!< (SMCII)  Control Register for CS 3
#define D940C_SMCII_SETUP7 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC70) //!< (SMCII)  Setup Register for CS 7
#define D940C_SMCII_CTRL7 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC7C) //!< (SMCII)  Control Register for CS 7
#define D940C_SMCII_CTRL5 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC5C) //!< (SMCII)  Control Register for CS 5
#define D940C_SMCII_CYCLE1 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC18) //!< (SMCII)  Cycle Register for CS 1
#define D940C_SMCII_SETUP0 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC00) //!< (SMCII)  Setup Register for CS 0
#define D940C_SMCII_PULSE4 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC44) //!< (SMCII)  Pulse Register for CS 4
#define D940C_SMCII_CYCLE3 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC38) //!< (SMCII)  Cycle Register for CS 3
#define D940C_SMCII_CYCLE4 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC48) //!< (SMCII)  Cycle Register for CS 4
#define D940C_SMCII_SETUP6 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC60) //!< (SMCII)  Setup Register for CS 6
#define D940C_SMCII_SETUP3 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC30) //!< (SMCII)  Setup Register for CS 3
// ========== Register definition for SMCIII peripheral ========== 
#define D940C_SMCIII_CTRL1 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC1C) //!< (SMCIII)  Control Register for CS 1
#define D940C_SMCIII_PULSE7 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC74) //!< (SMCIII)  Pulse Register for CS 7
#define D940C_SMCIII_PULSE6 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC64) //!< (SMCIII)  Pulse Register for CS 6
#define D940C_SMCIII_PULSE3 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC34) //!< (SMCIII)  Pulse Register for CS 3
#define D940C_SMCIII_CYCLE5 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC58) //!< (SMCIII)  Cycle Register for CS 5
#define D940C_SMCIII_CTRL0 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC0C) //!< (SMCIII)  Control Register for CS 0
#define D940C_SMCIII_PULSE5 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC54) //!< (SMCIII)  Pulse Register for CS 5
#define D940C_SMCIII_PULSE0 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC04) //!< (SMCIII)  Pulse Register for CS 0
#define D940C_SMCIII_CTRL4 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC4C) //!< (SMCIII)  Control Register for CS 4
#define D940C_SMCIII_SETUP1 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC10) //!< (SMCIII)  Setup Register for CS 1
#define D940C_SMCIII_CYCLE0 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC08) //!< (SMCIII)  Cycle Register for CS 0
#define D940C_SMCIII_CTRL6 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC6C) //!< (SMCIII)  Control Register for CS 6
#define D940C_SMCIII_SETUP5 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC50) //!< (SMCIII)  Setup Register for CS 5
#define D940C_SMCIII_SETUP2 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC20) //!< (SMCIII)  Setup Register for CS 2
#define D940C_SMCIII_CYCLE6 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC68) //!< (SMCIII)  Cycle Register for CS 6
#define D940C_SMCIII_PULSE2 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC24) //!< (SMCIII)  Pulse Register for CS 2
#define D940C_SMCIII_SETUP4 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC40) //!< (SMCIII)  Setup Register for CS 4
#define D940C_SMCIII_CYCLE2 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC28) //!< (SMCIII)  Cycle Register for CS 2
#define D940C_SMCIII_CTRL2 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC2C) //!< (SMCIII)  Control Register for CS 2
#define D940C_SMCIII_PULSE1 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC14) //!< (SMCIII)  Pulse Register for CS 1
#define D940C_SMCIII_CYCLE7 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC78) //!< (SMCIII)  Cycle Register for CS 7
#define D940C_SMCIII_CTRL3 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC3C) //!< (SMCIII)  Control Register for CS 3
#define D940C_SMCIII_SETUP7 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC70) //!< (SMCIII)  Setup Register for CS 7
#define D940C_SMCIII_CTRL7 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC7C) //!< (SMCIII)  Control Register for CS 7
#define D940C_SMCIII_CTRL5 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC5C) //!< (SMCIII)  Control Register for CS 5
#define D940C_SMCIII_CYCLE1 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC18) //!< (SMCIII)  Cycle Register for CS 1
#define D940C_SMCIII_SETUP0 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC00) //!< (SMCIII)  Setup Register for CS 0
#define D940C_SMCIII_PULSE4 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC44) //!< (SMCIII)  Pulse Register for CS 4
#define D940C_SMCIII_CYCLE3 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC38) //!< (SMCIII)  Cycle Register for CS 3
#define D940C_SMCIII_CYCLE4 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC48) //!< (SMCIII)  Cycle Register for CS 4
#define D940C_SMCIII_SETUP6 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC60) //!< (SMCIII)  Setup Register for CS 6
#define D940C_SMCIII_SETUP3 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEC30) //!< (SMCIII)  Setup Register for CS 3
// ========== Register definition for MATRIX peripheral ========== 
#define D940C_MATRIX_MCFG0 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE00) //!< (MATRIX)  Master Configuration Register 0 
#define D940C_MATRIX_SCFG1 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE44) //!< (MATRIX)  Slave Configuration Register 1 
#define D940C_MATRIX_MCFG4 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE10) //!< (MATRIX)  Master Configuration Register 4 
#define D940C_MATRIX_MCFG2 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE08) //!< (MATRIX)  Master Configuration Register 2 
#define D940C_MATRIX_SCFG3 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE4C) //!< (MATRIX)  Slave Configuration Register 3 
#define D940C_MATRIX_SCFG0 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE40) //!< (MATRIX)  Slave Configuration Register 0 
#define D940C_MATRIX_PRAS3 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE98) //!< (MATRIX)  PRAS3 
#define D940C_MATRIX_PRAS0 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE80) //!< (MATRIX)  PRAS0 
#define D940C_MATRIX_MCFG3 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE0C) //!< (MATRIX)  Master Configuration Register 3 
#define D940C_MATRIX_PRAS1 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE88) //!< (MATRIX)  PRAS1 
#define D940C_MATRIX_PRAS2 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE90) //!< (MATRIX)  PRAS2 
#define D940C_MATRIX_SCFG2 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE48) //!< (MATRIX)  Slave Configuration Register 2 
#define D940C_MATRIX_MCFG5 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE14) //!< (MATRIX)  Master Configuration Register 5 
#define D940C_MATRIX_MCFG1 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE04) //!< (MATRIX)  Master Configuration Register 1 
#define D940C_MATRIX_PRAS4 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEEA0) //!< (MATRIX)  PRAS4 
#define D940C_MATRIX_MRCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEF00) //!< (MATRIX)  Master Remp Control Register 
#define D940C_MATRIX_SCFG4 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE50) //!< (MATRIX)  Slave Configuration Register 4 
// ========== Register definition for MATRIXII peripheral ========== 
#define D940C_MATRIXII_MCFG0 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE00) //!< (MATRIXII)  Master Configuration Register 0 
#define D940C_MATRIXII_SCFG1 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE44) //!< (MATRIXII)  Slave Configuration Register 1 
#define D940C_MATRIXII_MCFG4 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE10) //!< (MATRIXII)  Master Configuration Register 4 
#define D940C_MATRIXII_MCFG2 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE08) //!< (MATRIXII)  Master Configuration Register 2 
#define D940C_MATRIXII_SCFG3 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE4C) //!< (MATRIXII)  Slave Configuration Register 3 
#define D940C_MATRIXII_SCFG0 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE40) //!< (MATRIXII)  Slave Configuration Register 0 
#define D940C_MATRIXII_PRAS3 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE98) //!< (MATRIXII)  PRAS3 
#define D940C_MATRIXII_PRAS0 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE80) //!< (MATRIXII)  PRAS0 
#define D940C_MATRIXII_MCFG3 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE0C) //!< (MATRIXII)  Master Configuration Register 3 
#define D940C_MATRIXII_PRAS1 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE88) //!< (MATRIXII)  PRAS1 
#define D940C_MATRIXII_PRAS2 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE90) //!< (MATRIXII)  PRAS2 
#define D940C_MATRIXII_SCFG2 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE48) //!< (MATRIXII)  Slave Configuration Register 2 
#define D940C_MATRIXII_MCFG5 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE14) //!< (MATRIXII)  Master Configuration Register 5 
#define D940C_MATRIXII_MCFG1 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE04) //!< (MATRIXII)  Master Configuration Register 1 
#define D940C_MATRIXII_PRAS4 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEEA0) //!< (MATRIXII)  PRAS4 
#define D940C_MATRIXII_MRCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEF00) //!< (MATRIXII)  Master Remp Control Register 
#define D940C_MATRIXII_SCFG4 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE50) //!< (MATRIXII)  Slave Configuration Register 4 
// ========== Register definition for MATRIXIII peripheral ========== 
#define D940C_MATRIXIII_MCFG0 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE00) //!< (MATRIXIII)  Master Configuration Register 0 
#define D940C_MATRIXIII_SCFG1 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE44) //!< (MATRIXIII)  Slave Configuration Register 1 
#define D940C_MATRIXIII_MCFG4 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE10) //!< (MATRIXIII)  Master Configuration Register 4 
#define D940C_MATRIXIII_MCFG2 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE08) //!< (MATRIXIII)  Master Configuration Register 2 
#define D940C_MATRIXIII_SCFG3 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE4C) //!< (MATRIXIII)  Slave Configuration Register 3 
#define D940C_MATRIXIII_SCFG0 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE40) //!< (MATRIXIII)  Slave Configuration Register 0 
#define D940C_MATRIXIII_PRAS3 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE98) //!< (MATRIXIII)  PRAS3 
#define D940C_MATRIXIII_PRAS0 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE80) //!< (MATRIXIII)  PRAS0 
#define D940C_MATRIXIII_MCFG3 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE0C) //!< (MATRIXIII)  Master Configuration Register 3 
#define D940C_MATRIXIII_PRAS1 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE88) //!< (MATRIXIII)  PRAS1 
#define D940C_MATRIXIII_PRAS2 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE90) //!< (MATRIXIII)  PRAS2 
#define D940C_MATRIXIII_SCFG2 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE48) //!< (MATRIXIII)  Slave Configuration Register 2 
#define D940C_MATRIXIII_MCFG5 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE14) //!< (MATRIXIII)  Master Configuration Register 5 
#define D940C_MATRIXIII_MCFG1 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE04) //!< (MATRIXIII)  Master Configuration Register 1 
#define D940C_MATRIXIII_PRAS4 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEEA0) //!< (MATRIXIII)  PRAS4 
#define D940C_MATRIXIII_MRCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEF00) //!< (MATRIXIII)  Master Remp Control Register 
#define D940C_MATRIXIII_SCFG4 ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFEE50) //!< (MATRIXIII)  Slave Configuration Register 4 
// ========== Register definition for AIC peripheral ========== 
#define D940C_AIC_IVR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF100) //!< (AIC) IRQ Vector Register
#define D940C_AIC_SMR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF000) //!< (AIC) Source Mode Register
#define D940C_AIC_IMR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF110) //!< (AIC) Interrupt Mask Register
#define D940C_AIC_FFER  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF140) //!< (AIC) Fast Forcing Enable Register
#define D940C_AIC_IPR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF10C) //!< (AIC) Interrupt Pending Register
#define D940C_AIC_FVR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF104) //!< (AIC) FIQ Vector Register
#define D940C_AIC_DCR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF138) //!< (AIC) Debug Control Register (Protect)
#define D940C_AIC_EOICR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF130) //!< (AIC) End of Interrupt Command Register
#define D940C_AIC_SVR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF080) //!< (AIC) Source Vector Register
#define D940C_AIC_IECR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF120) //!< (AIC) Interrupt Enable Command Register
#define D940C_AIC_FFSR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF148) //!< (AIC) Fast Forcing Status Register
#define D940C_AIC_ISCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF12C) //!< (AIC) Interrupt Set Command Register
#define D940C_AIC_FFDR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF144) //!< (AIC) Fast Forcing Disable Register
#define D940C_AIC_ICCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF128) //!< (AIC) Interrupt Clear Command Register
#define D940C_AIC_CISR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF114) //!< (AIC) Core Interrupt Status Register
#define D940C_AIC_IDCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF124) //!< (AIC) Interrupt Disable Command Register
#define D940C_AIC_ISR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF108) //!< (AIC) Interrupt Status Register
#define D940C_AIC_SPU   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF134) //!< (AIC) Spurious Vector Register
// ========== Register definition for PDC_DBGU peripheral ========== 
#define D940C_DBGU_TCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF30C) //!< (PDC_DBGU) Transmit Counter Register
#define D940C_DBGU_RNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF310) //!< (PDC_DBGU) Receive Next Pointer Register
#define D940C_DBGU_TNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF318) //!< (PDC_DBGU) Transmit Next Pointer Register
#define D940C_DBGU_TPR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF308) //!< (PDC_DBGU) Transmit Pointer Register
#define D940C_DBGU_PTCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF320) //!< (PDC_DBGU) PDC Transfer Control Register
#define D940C_DBGU_RPR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF300) //!< (PDC_DBGU) Receive Pointer Register
#define D940C_DBGU_RCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF304) //!< (PDC_DBGU) Receive Counter Register
#define D940C_DBGU_RNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF314) //!< (PDC_DBGU) Receive Next Counter Register
#define D940C_DBGU_PTSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF324) //!< (PDC_DBGU) PDC Transfer Status Register
#define D940C_DBGU_TNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF31C) //!< (PDC_DBGU) Transmit Next Counter Register
// ========== Register definition for DBGU peripheral ========== 
#define D940C_DBGU_EXID ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF244) //!< (DBGU) Chip ID Extension Register
#define D940C_DBGU_BRGR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF220) //!< (DBGU) Baud Rate Generator Register
#define D940C_DBGU_CIDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF240) //!< (DBGU) Chip ID Register
#define D940C_DBGU_IDR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF20C) //!< (DBGU) Interrupt Disable Register
#define D940C_DBGU_CSR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF214) //!< (DBGU) Channel Status Register
#define D940C_DBGU_MR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF204) //!< (DBGU) Mode Register
#define D940C_DBGU_IMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF210) //!< (DBGU) Interrupt Mask Register
#define D940C_DBGU_CR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF200) //!< (DBGU) Control Register
#define D940C_DBGU_FNTR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF248) //!< (DBGU) Force NTRST Register
#define D940C_DBGU_THR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF21C) //!< (DBGU) Transmitter Holding Register
#define D940C_DBGU_RHR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF218) //!< (DBGU) Receiver Holding Register
#define D940C_DBGU_IER  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF208) //!< (DBGU) Interrupt Enable Register
// ========== Register definition for PDC_DBGUII peripheral ========== 
#define D940C_DBGUII_TCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF30C) //!< (PDC_DBGUII) Transmit Counter Register
#define D940C_DBGUII_RNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF310) //!< (PDC_DBGUII) Receive Next Pointer Register
#define D940C_DBGUII_TNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF318) //!< (PDC_DBGUII) Transmit Next Pointer Register
#define D940C_DBGUII_TPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF308) //!< (PDC_DBGUII) Transmit Pointer Register
#define D940C_DBGUII_PTCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF320) //!< (PDC_DBGUII) PDC Transfer Control Register
#define D940C_DBGUII_RPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF300) //!< (PDC_DBGUII) Receive Pointer Register
#define D940C_DBGUII_RCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF304) //!< (PDC_DBGUII) Receive Counter Register
#define D940C_DBGUII_RNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF314) //!< (PDC_DBGUII) Receive Next Counter Register
#define D940C_DBGUII_PTSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF324) //!< (PDC_DBGUII) PDC Transfer Status Register
#define D940C_DBGUII_TNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF31C) //!< (PDC_DBGUII) Transmit Next Counter Register
// ========== Register definition for DBGUII peripheral ========== 
#define D940C_DBGUII_EXID ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF244) //!< (DBGUII) Chip ID Extension Register
#define D940C_DBGUII_BRGR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF220) //!< (DBGUII) Baud Rate Generator Register
#define D940C_DBGUII_CIDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF240) //!< (DBGUII) Chip ID Register
#define D940C_DBGUII_IDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF20C) //!< (DBGUII) Interrupt Disable Register
#define D940C_DBGUII_CSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF214) //!< (DBGUII) Channel Status Register
#define D940C_DBGUII_MR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF204) //!< (DBGUII) Mode Register
#define D940C_DBGUII_IMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF210) //!< (DBGUII) Interrupt Mask Register
#define D940C_DBGUII_CR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF200) //!< (DBGUII) Control Register
#define D940C_DBGUII_FNTR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF248) //!< (DBGUII) Force NTRST Register
#define D940C_DBGUII_THR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF21C) //!< (DBGUII) Transmitter Holding Register
#define D940C_DBGUII_RHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF218) //!< (DBGUII) Receiver Holding Register
#define D940C_DBGUII_IER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF208) //!< (DBGUII) Interrupt Enable Register
// ========== Register definition for PDC_DBGUIII peripheral ========== 
#define D940C_DBGUIII_TCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF30C) //!< (PDC_DBGUIII) Transmit Counter Register
#define D940C_DBGUIII_RNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF310) //!< (PDC_DBGUIII) Receive Next Pointer Register
#define D940C_DBGUIII_TNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF318) //!< (PDC_DBGUIII) Transmit Next Pointer Register
#define D940C_DBGUIII_TPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF308) //!< (PDC_DBGUIII) Transmit Pointer Register
#define D940C_DBGUIII_PTCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF320) //!< (PDC_DBGUIII) PDC Transfer Control Register
#define D940C_DBGUIII_RPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF300) //!< (PDC_DBGUIII) Receive Pointer Register
#define D940C_DBGUIII_RCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF304) //!< (PDC_DBGUIII) Receive Counter Register
#define D940C_DBGUIII_RNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF314) //!< (PDC_DBGUIII) Receive Next Counter Register
#define D940C_DBGUIII_PTSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF324) //!< (PDC_DBGUIII) PDC Transfer Status Register
#define D940C_DBGUIII_TNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF31C) //!< (PDC_DBGUIII) Transmit Next Counter Register
// ========== Register definition for DBGUIII peripheral ========== 
#define D940C_DBGUIII_EXID ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF244) //!< (DBGUIII) Chip ID Extension Register
#define D940C_DBGUIII_BRGR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF220) //!< (DBGUIII) Baud Rate Generator Register
#define D940C_DBGUIII_CIDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF240) //!< (DBGUIII) Chip ID Register
#define D940C_DBGUIII_IDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF20C) //!< (DBGUIII) Interrupt Disable Register
#define D940C_DBGUIII_CSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF214) //!< (DBGUIII) Channel Status Register
#define D940C_DBGUIII_MR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF204) //!< (DBGUIII) Mode Register
#define D940C_DBGUIII_IMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF210) //!< (DBGUIII) Interrupt Mask Register
#define D940C_DBGUIII_CR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF200) //!< (DBGUIII) Control Register
#define D940C_DBGUIII_FNTR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF248) //!< (DBGUIII) Force NTRST Register
#define D940C_DBGUIII_THR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF21C) //!< (DBGUIII) Transmitter Holding Register
#define D940C_DBGUIII_RHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF218) //!< (DBGUIII) Receiver Holding Register
#define D940C_DBGUIII_IER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF208) //!< (DBGUIII) Interrupt Enable Register
// ========== Register definition for PIOA peripheral ========== 
#define D940C_PIOA_ODR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF414) //!< (PIOA) Output Disable Registerr
#define D940C_PIOA_SODR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF430) //!< (PIOA) Set Output Data Register
#define D940C_PIOA_ISR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF44C) //!< (PIOA) Interrupt Status Register
#define D940C_PIOA_ABSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF478) //!< (PIOA) AB Select Status Register
#define D940C_PIOA_IER  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF440) //!< (PIOA) Interrupt Enable Register
#define D940C_PIOA_PPUDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF460) //!< (PIOA) Pull-up Disable Register
#define D940C_PIOA_IMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF448) //!< (PIOA) Interrupt Mask Register
#define D940C_PIOA_PER  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF400) //!< (PIOA) PIO Enable Register
#define D940C_PIOA_IFDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF424) //!< (PIOA) Input Filter Disable Register
#define D940C_PIOA_OWDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF4A4) //!< (PIOA) Output Write Disable Register
#define D940C_PIOA_MDSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF458) //!< (PIOA) Multi-driver Status Register
#define D940C_PIOA_IDR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF444) //!< (PIOA) Interrupt Disable Register
#define D940C_PIOA_ODSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF438) //!< (PIOA) Output Data Status Register
#define D940C_PIOA_PPUSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF468) //!< (PIOA) Pull-up Status Register
#define D940C_PIOA_OWSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF4A8) //!< (PIOA) Output Write Status Register
#define D940C_PIOA_BSR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF474) //!< (PIOA) Select B Register
#define D940C_PIOA_OWER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF4A0) //!< (PIOA) Output Write Enable Register
#define D940C_PIOA_IFER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF420) //!< (PIOA) Input Filter Enable Register
#define D940C_PIOA_PDSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF43C) //!< (PIOA) Pin Data Status Register
#define D940C_PIOA_PPUER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF464) //!< (PIOA) Pull-up Enable Register
#define D940C_PIOA_OSR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF418) //!< (PIOA) Output Status Register
#define D940C_PIOA_ASR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF470) //!< (PIOA) Select A Register
#define D940C_PIOA_MDDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF454) //!< (PIOA) Multi-driver Disable Register
#define D940C_PIOA_CODR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF434) //!< (PIOA) Clear Output Data Register
#define D940C_PIOA_MDER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF450) //!< (PIOA) Multi-driver Enable Register
#define D940C_PIOA_PDR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF404) //!< (PIOA) PIO Disable Register
#define D940C_PIOA_OER  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF410) //!< (PIOA) Output Enable Register
#define D940C_PIOA_PSR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF408) //!< (PIOA) PIO Status Register
#define D940C_PIOA_IFSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF428) //!< (PIOA) Input Filter Status Register
// ========== Register definition for PIOB peripheral ========== 
#define D940C_PIOB_OWDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF6A4) //!< (PIOB) Output Write Disable Register
#define D940C_PIOB_MDER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF650) //!< (PIOB) Multi-driver Enable Register
#define D940C_PIOB_PPUSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF668) //!< (PIOB) Pull-up Status Register
#define D940C_PIOB_IMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF648) //!< (PIOB) Interrupt Mask Register
#define D940C_PIOB_ASR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF670) //!< (PIOB) Select A Register
#define D940C_PIOB_PPUDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF660) //!< (PIOB) Pull-up Disable Register
#define D940C_PIOB_PSR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF608) //!< (PIOB) PIO Status Register
#define D940C_PIOB_IER  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF640) //!< (PIOB) Interrupt Enable Register
#define D940C_PIOB_CODR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF634) //!< (PIOB) Clear Output Data Register
#define D940C_PIOB_OWER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF6A0) //!< (PIOB) Output Write Enable Register
#define D940C_PIOB_ABSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF678) //!< (PIOB) AB Select Status Register
#define D940C_PIOB_IFDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF624) //!< (PIOB) Input Filter Disable Register
#define D940C_PIOB_PDSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF63C) //!< (PIOB) Pin Data Status Register
#define D940C_PIOB_IDR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF644) //!< (PIOB) Interrupt Disable Register
#define D940C_PIOB_OWSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF6A8) //!< (PIOB) Output Write Status Register
#define D940C_PIOB_PDR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF604) //!< (PIOB) PIO Disable Register
#define D940C_PIOB_ODR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF614) //!< (PIOB) Output Disable Registerr
#define D940C_PIOB_IFSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF628) //!< (PIOB) Input Filter Status Register
#define D940C_PIOB_SODR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF630) //!< (PIOB) Set Output Data Register
#define D940C_PIOB_PPUER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF664) //!< (PIOB) Pull-up Enable Register
#define D940C_PIOB_ODSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF638) //!< (PIOB) Output Data Status Register
#define D940C_PIOB_ISR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF64C) //!< (PIOB) Interrupt Status Register
#define D940C_PIOB_OSR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF618) //!< (PIOB) Output Status Register
#define D940C_PIOB_MDSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF658) //!< (PIOB) Multi-driver Status Register
#define D940C_PIOB_IFER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF620) //!< (PIOB) Input Filter Enable Register
#define D940C_PIOB_OER  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF610) //!< (PIOB) Output Enable Register
#define D940C_PIOB_MDDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF654) //!< (PIOB) Multi-driver Disable Register
#define D940C_PIOB_BSR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF674) //!< (PIOB) Select B Register
#define D940C_PIOB_PER  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF600) //!< (PIOB) PIO Enable Register
// ========== Register definition for PIOC peripheral ========== 
#define D940C_PIOC_OWDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF8A4) //!< (PIOC) Output Write Disable Register
#define D940C_PIOC_SODR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF830) //!< (PIOC) Set Output Data Register
#define D940C_PIOC_PPUER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF864) //!< (PIOC) Pull-up Enable Register
#define D940C_PIOC_CODR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF834) //!< (PIOC) Clear Output Data Register
#define D940C_PIOC_PSR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF808) //!< (PIOC) PIO Status Register
#define D940C_PIOC_PDR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF804) //!< (PIOC) PIO Disable Register
#define D940C_PIOC_ODR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF814) //!< (PIOC) Output Disable Registerr
#define D940C_PIOC_PPUSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF868) //!< (PIOC) Pull-up Status Register
#define D940C_PIOC_ABSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF878) //!< (PIOC) AB Select Status Register
#define D940C_PIOC_IFSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF828) //!< (PIOC) Input Filter Status Register
#define D940C_PIOC_OER  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF810) //!< (PIOC) Output Enable Register
#define D940C_PIOC_IMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF848) //!< (PIOC) Interrupt Mask Register
#define D940C_PIOC_ASR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF870) //!< (PIOC) Select A Register
#define D940C_PIOC_MDDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF854) //!< (PIOC) Multi-driver Disable Register
#define D940C_PIOC_OWSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF8A8) //!< (PIOC) Output Write Status Register
#define D940C_PIOC_PER  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF800) //!< (PIOC) PIO Enable Register
#define D940C_PIOC_IDR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF844) //!< (PIOC) Interrupt Disable Register
#define D940C_PIOC_MDER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF850) //!< (PIOC) Multi-driver Enable Register
#define D940C_PIOC_PDSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF83C) //!< (PIOC) Pin Data Status Register
#define D940C_PIOC_MDSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF858) //!< (PIOC) Multi-driver Status Register
#define D940C_PIOC_OWER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF8A0) //!< (PIOC) Output Write Enable Register
#define D940C_PIOC_BSR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF874) //!< (PIOC) Select B Register
#define D940C_PIOC_PPUDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF860) //!< (PIOC) Pull-up Disable Register
#define D940C_PIOC_IFDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF824) //!< (PIOC) Input Filter Disable Register
#define D940C_PIOC_IER  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF840) //!< (PIOC) Interrupt Enable Register
#define D940C_PIOC_ISR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF84C) //!< (PIOC) Interrupt Status Register
#define D940C_PIOC_OSR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF818) //!< (PIOC) Output Status Register
#define D940C_PIOC_ODSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF838) //!< (PIOC) Output Data Status Register
#define D940C_PIOC_IFER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFF820) //!< (PIOC) Input Filter Enable Register
// ========== Register definition for CKGR peripheral ========== 
#define D940C_CKGR_MOR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC20) //!< (CKGR) Main Oscillator Register
#define D940C_CKGR_PLLBR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC2C) //!< (CKGR) PLL B Register
#define D940C_CKGR_MCFR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC24) //!< (CKGR) Main Clock  Frequency Register
#define D940C_CKGR_PLLAR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC28) //!< (CKGR) PLL A Register
// ========== Register definition for PMC peripheral ========== 
#define D940C_PMC_IDR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC64) //!< (PMC) Interrupt Disable Register
#define D940C_PMC_MOR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC20) //!< (PMC) Main Oscillator Register
#define D940C_PMC_PLLBR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC2C) //!< (PMC) PLL B Register
#define D940C_PMC_PCER  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC10) //!< (PMC) Peripheral Clock Enable Register
#define D940C_PMC_PCKR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC40) //!< (PMC) Programmable Clock Register
#define D940C_PMC_MCKR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC30) //!< (PMC) Master Clock Register
#define D940C_PMC_PLLAR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC28) //!< (PMC) PLL A Register
#define D940C_PMC_SCDR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC04) //!< (PMC) System Clock Disable Register
#define D940C_PMC_PCDR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC14) //!< (PMC) Peripheral Clock Disable Register
#define D940C_PMC_SCSR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC08) //!< (PMC) System Clock Status Register
#define D940C_PMC_PCSR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC18) //!< (PMC) Peripheral Clock Status Register
#define D940C_PMC_MCFR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC24) //!< (PMC) Main Clock  Frequency Register
#define D940C_PMC_SCER  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC00) //!< (PMC) System Clock Enable Register
#define D940C_PMC_IMR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC6C) //!< (PMC) Interrupt Mask Register
#define D940C_PMC_IER   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC60) //!< (PMC) Interrupt Enable Register
#define D940C_PMC_SR    ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC68) //!< (PMC) Status Register
// ========== Register definition for PMCII peripheral ========== 
#define D940C_PMCII_IDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC64) //!< (PMCII) Interrupt Disable Register
#define D940C_PMCII_MOR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC20) //!< (PMCII) Main Oscillator Register
#define D940C_PMCII_PLLBR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC2C) //!< (PMCII) PLL B Register
#define D940C_PMCII_PCER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC10) //!< (PMCII) Peripheral Clock Enable Register
#define D940C_PMCII_PCKR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC40) //!< (PMCII) Programmable Clock Register
#define D940C_PMCII_MCKR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC30) //!< (PMCII) Master Clock Register
#define D940C_PMCII_PLLAR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC28) //!< (PMCII) PLL A Register
#define D940C_PMCII_SCDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC04) //!< (PMCII) System Clock Disable Register
#define D940C_PMCII_PCDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC14) //!< (PMCII) Peripheral Clock Disable Register
#define D940C_PMCII_SCSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC08) //!< (PMCII) System Clock Status Register
#define D940C_PMCII_PCSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC18) //!< (PMCII) Peripheral Clock Status Register
#define D940C_PMCII_MCFR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC24) //!< (PMCII) Main Clock  Frequency Register
#define D940C_PMCII_SCER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC00) //!< (PMCII) System Clock Enable Register
#define D940C_PMCII_IMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC6C) //!< (PMCII) Interrupt Mask Register
#define D940C_PMCII_IER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC60) //!< (PMCII) Interrupt Enable Register
#define D940C_PMCII_SR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFC68) //!< (PMCII) Status Register
// ========== Register definition for RSTC peripheral ========== 
#define D940C_RSTC_RCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFD00) //!< (RSTC) Reset Control Register
#define D940C_RSTC_RMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFD08) //!< (RSTC) Reset Mode Register
#define D940C_RSTC_RSR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFD04) //!< (RSTC) Reset Status Register
// ========== Register definition for SHDWC peripheral ========== 
#define D940C_SHDWC_SHSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFD18) //!< (SHDWC) Shut Down Status Register
#define D940C_SHDWC_SHMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFD14) //!< (SHDWC) Shut Down Mode Register
#define D940C_SHDWC_SHCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFD10) //!< (SHDWC) Shut Down Control Register
// ========== Register definition for RTTC peripheral ========== 
#define D940C_RTTC_RTSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFD2C) //!< (RTTC) Real-time Status Register
#define D940C_RTTC_RTMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFD20) //!< (RTTC) Real-time Mode Register
#define D940C_RTTC_RTVR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFD28) //!< (RTTC) Real-time Value Register
#define D940C_RTTC_RTAR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFD24) //!< (RTTC) Real-time Alarm Register
// ========== Register definition for PITC peripheral ========== 
#define D940C_PITC_PIVR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFD38) //!< (PITC) Period Interval Value Register
#define D940C_PITC_PISR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFD34) //!< (PITC) Period Interval Status Register
#define D940C_PITC_PIIR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFD3C) //!< (PITC) Period Interval Image Register
#define D940C_PITC_PIMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFD30) //!< (PITC) Period Interval Mode Register
// ========== Register definition for WDTC peripheral ========== 
#define D940C_WDTC_WDCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFD40) //!< (WDTC) Watchdog Control Register
#define D940C_WDTC_WDSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFD48) //!< (WDTC) Watchdog Status Register
#define D940C_WDTC_WDMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFFFD44) //!< (WDTC) Watchdog Mode Register
// ========== Register definition for TC0 peripheral ========== 
#define D940C_TC0_SR    ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA0020) //!< (TC0) Status Register
#define D940C_TC0_RC    ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA001C) //!< (TC0) Register C
#define D940C_TC0_RB    ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA0018) //!< (TC0) Register B
#define D940C_TC0_CCR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA0000) //!< (TC0) Channel Control Register
#define D940C_TC0_CMR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA0004) //!< (TC0) Channel Mode Register (Capture Mode / Waveform Mode)
#define D940C_TC0_IER   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA0024) //!< (TC0) Interrupt Enable Register
#define D940C_TC0_IDR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA0028) //!< (TC0) Interrupt Disable Register
#define D940C_TC0_RA    ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA0014) //!< (TC0) Register A
#define D940C_TC0_CV    ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA0010) //!< (TC0) Counter Value
#define D940C_TC0_IMR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA002C) //!< (TC0) Interrupt Mask Register
// ========== Register definition for TC1 peripheral ========== 
#define D940C_TC1_RB    ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA0058) //!< (TC1) Register B
#define D940C_TC1_CCR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA0040) //!< (TC1) Channel Control Register
#define D940C_TC1_IER   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA0064) //!< (TC1) Interrupt Enable Register
#define D940C_TC1_IDR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA0068) //!< (TC1) Interrupt Disable Register
#define D940C_TC1_SR    ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA0060) //!< (TC1) Status Register
#define D940C_TC1_CMR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA0044) //!< (TC1) Channel Mode Register (Capture Mode / Waveform Mode)
#define D940C_TC1_RA    ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA0054) //!< (TC1) Register A
#define D940C_TC1_RC    ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA005C) //!< (TC1) Register C
#define D940C_TC1_IMR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA006C) //!< (TC1) Interrupt Mask Register
#define D940C_TC1_CV    ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA0050) //!< (TC1) Counter Value
// ========== Register definition for TC2 peripheral ========== 
#define D940C_TC2_CMR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA0084) //!< (TC2) Channel Mode Register (Capture Mode / Waveform Mode)
#define D940C_TC2_CCR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA0080) //!< (TC2) Channel Control Register
#define D940C_TC2_CV    ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA0090) //!< (TC2) Counter Value
#define D940C_TC2_RA    ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA0094) //!< (TC2) Register A
#define D940C_TC2_RB    ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA0098) //!< (TC2) Register B
#define D940C_TC2_IDR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA00A8) //!< (TC2) Interrupt Disable Register
#define D940C_TC2_IMR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA00AC) //!< (TC2) Interrupt Mask Register
#define D940C_TC2_RC    ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA009C) //!< (TC2) Register C
#define D940C_TC2_IER   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA00A4) //!< (TC2) Interrupt Enable Register
#define D940C_TC2_SR    ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA00A0) //!< (TC2) Status Register
// ========== Register definition for TCB0 peripheral ========== 
#define D940C_TCB0_BMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA00C4) //!< (TCB0) TC Block Mode Register
#define D940C_TCB0_BCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA00C0) //!< (TCB0) TC Block Control Register
// ========== Register definition for UDP peripheral ========== 
#define D940C_UDP_GLBSTATE ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA4004) //!< (UDP) Global State Register
#define D940C_UDP_FDR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA4050) //!< (UDP) Endpoint FIFO Data Register
#define D940C_UDP_RSTEP ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA4028) //!< (UDP) Reset Endpoint Register
#define D940C_UDP_FADDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA4008) //!< (UDP) Function Address Register
#define D940C_UDP_NUM   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA4000) //!< (UDP) Frame Number Register
#define D940C_UDP_IDR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA4014) //!< (UDP) Interrupt Disable Register
#define D940C_UDP_IMR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA4018) //!< (UDP) Interrupt Mask Register
#define D940C_UDP_CSR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA4030) //!< (UDP) Endpoint Control and Status Register
#define D940C_UDP_IER   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA4010) //!< (UDP) Interrupt Enable Register
#define D940C_UDP_ICR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA4020) //!< (UDP) Interrupt Clear Register
#define D940C_UDP_TXVC  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA4074) //!< (UDP) Transceiver Control Register
#define D940C_UDP_ISR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA401C) //!< (UDP) Interrupt Status Register
// ========== Register definition for TWI0 peripheral ========== 
#define D940C_TWI0_THR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFAC034) //!< (TWI0) Transmit Holding Register
#define D940C_TWI0_IDR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFAC028) //!< (TWI0) Interrupt Disable Register
#define D940C_TWI0_SMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFAC008) //!< (TWI0) Slave Mode Register
#define D940C_TWI0_CWGR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFAC010) //!< (TWI0) Clock Waveform Generator Register
#define D940C_TWI0_IADR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFAC00C) //!< (TWI0) Internal Address Register
#define D940C_TWI0_RHR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFAC030) //!< (TWI0) Receive Holding Register
#define D940C_TWI0_IER  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFAC024) //!< (TWI0) Interrupt Enable Register
#define D940C_TWI0_MMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFAC004) //!< (TWI0) Master Mode Register
#define D940C_TWI0_SR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFAC020) //!< (TWI0) Status Register
#define D940C_TWI0_CR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFAC000) //!< (TWI0) Control Register
#define D940C_TWI0_IMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFAC02C) //!< (TWI0) Interrupt Mask Register
// ========== Register definition for TWI1 peripheral ========== 
#define D940C_TWI1_SR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD4020) //!< (TWI1) Status Register
#define D940C_TWI1_RHR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD4030) //!< (TWI1) Receive Holding Register
#define D940C_TWI1_IER  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD4024) //!< (TWI1) Interrupt Enable Register
#define D940C_TWI1_CWGR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD4010) //!< (TWI1) Clock Waveform Generator Register
#define D940C_TWI1_IDR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD4028) //!< (TWI1) Interrupt Disable Register
#define D940C_TWI1_THR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD4034) //!< (TWI1) Transmit Holding Register
#define D940C_TWI1_IMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD402C) //!< (TWI1) Interrupt Mask Register
#define D940C_TWI1_MMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD4004) //!< (TWI1) Master Mode Register
#define D940C_TWI1_SMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD4008) //!< (TWI1) Slave Mode Register
#define D940C_TWI1_CR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD4000) //!< (TWI1) Control Register
#define D940C_TWI1_IADR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD400C) //!< (TWI1) Internal Address Register
// ========== Register definition for PDC_US0 peripheral ========== 
#define D940C_US0_TCR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB010C) //!< (PDC_US0) Transmit Counter Register
#define D940C_US0_PTCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0120) //!< (PDC_US0) PDC Transfer Control Register
#define D940C_US0_RNCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0114) //!< (PDC_US0) Receive Next Counter Register
#define D940C_US0_PTSR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0124) //!< (PDC_US0) PDC Transfer Status Register
#define D940C_US0_TNCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB011C) //!< (PDC_US0) Transmit Next Counter Register
#define D940C_US0_RNPR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0110) //!< (PDC_US0) Receive Next Pointer Register
#define D940C_US0_RCR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0104) //!< (PDC_US0) Receive Counter Register
#define D940C_US0_TPR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0108) //!< (PDC_US0) Transmit Pointer Register
#define D940C_US0_TNPR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0118) //!< (PDC_US0) Transmit Next Pointer Register
#define D940C_US0_RPR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0100) //!< (PDC_US0) Receive Pointer Register
// ========== Register definition for US0 peripheral ========== 
#define D940C_US0_RHR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0018) //!< (US0) Receiver Holding Register
#define D940C_US0_NER   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0044) //!< (US0) Nb Errors Register
#define D940C_US0_IER   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0008) //!< (US0) Interrupt Enable Register
#define D940C_US0_CR    ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0000) //!< (US0) Control Register
#define D940C_US0_THR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB001C) //!< (US0) Transmitter Holding Register
#define D940C_US0_CSR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0014) //!< (US0) Channel Status Register
#define D940C_US0_BRGR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0020) //!< (US0) Baud Rate Generator Register
#define D940C_US0_RTOR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0024) //!< (US0) Receiver Time-out Register
#define D940C_US0_TTGR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0028) //!< (US0) Transmitter Time-guard Register
#define D940C_US0_IDR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB000C) //!< (US0) Interrupt Disable Register
#define D940C_US0_MR    ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0004) //!< (US0) Mode Register
#define D940C_US0_IF    ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB004C) //!< (US0) IRDA_FILTER Register
#define D940C_US0_FIDI  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0040) //!< (US0) FI_DI_Ratio Register
#define D940C_US0_IMR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0010) //!< (US0) Interrupt Mask Register
// ========== Register definition for PDC_US0II peripheral ========== 
#define D940C_US0II_TCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB010C) //!< (PDC_US0II) Transmit Counter Register
#define D940C_US0II_PTCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0120) //!< (PDC_US0II) PDC Transfer Control Register
#define D940C_US0II_RNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0114) //!< (PDC_US0II) Receive Next Counter Register
#define D940C_US0II_PTSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0124) //!< (PDC_US0II) PDC Transfer Status Register
#define D940C_US0II_TNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB011C) //!< (PDC_US0II) Transmit Next Counter Register
#define D940C_US0II_RNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0110) //!< (PDC_US0II) Receive Next Pointer Register
#define D940C_US0II_RCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0104) //!< (PDC_US0II) Receive Counter Register
#define D940C_US0II_TPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0108) //!< (PDC_US0II) Transmit Pointer Register
#define D940C_US0II_TNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0118) //!< (PDC_US0II) Transmit Next Pointer Register
#define D940C_US0II_RPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0100) //!< (PDC_US0II) Receive Pointer Register
// ========== Register definition for US0II peripheral ========== 
#define D940C_US0II_RHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0018) //!< (US0II) Receiver Holding Register
#define D940C_US0II_NER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0044) //!< (US0II) Nb Errors Register
#define D940C_US0II_IER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0008) //!< (US0II) Interrupt Enable Register
#define D940C_US0II_CR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0000) //!< (US0II) Control Register
#define D940C_US0II_THR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB001C) //!< (US0II) Transmitter Holding Register
#define D940C_US0II_CSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0014) //!< (US0II) Channel Status Register
#define D940C_US0II_BRGR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0020) //!< (US0II) Baud Rate Generator Register
#define D940C_US0II_RTOR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0024) //!< (US0II) Receiver Time-out Register
#define D940C_US0II_TTGR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0028) //!< (US0II) Transmitter Time-guard Register
#define D940C_US0II_IDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB000C) //!< (US0II) Interrupt Disable Register
#define D940C_US0II_MR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0004) //!< (US0II) Mode Register
#define D940C_US0II_IF  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB004C) //!< (US0II) IRDA_FILTER Register
#define D940C_US0II_FIDI ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0040) //!< (US0II) FI_DI_Ratio Register
#define D940C_US0II_IMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0010) //!< (US0II) Interrupt Mask Register
// ========== Register definition for PDC_US0III peripheral ========== 
#define D940C_US0III_TCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB010C) //!< (PDC_US0III) Transmit Counter Register
#define D940C_US0III_PTCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0120) //!< (PDC_US0III) PDC Transfer Control Register
#define D940C_US0III_RNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0114) //!< (PDC_US0III) Receive Next Counter Register
#define D940C_US0III_PTSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0124) //!< (PDC_US0III) PDC Transfer Status Register
#define D940C_US0III_TNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB011C) //!< (PDC_US0III) Transmit Next Counter Register
#define D940C_US0III_RNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0110) //!< (PDC_US0III) Receive Next Pointer Register
#define D940C_US0III_RCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0104) //!< (PDC_US0III) Receive Counter Register
#define D940C_US0III_TPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0108) //!< (PDC_US0III) Transmit Pointer Register
#define D940C_US0III_TNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0118) //!< (PDC_US0III) Transmit Next Pointer Register
#define D940C_US0III_RPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0100) //!< (PDC_US0III) Receive Pointer Register
// ========== Register definition for US0III peripheral ========== 
#define D940C_US0III_RHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0018) //!< (US0III) Receiver Holding Register
#define D940C_US0III_NER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0044) //!< (US0III) Nb Errors Register
#define D940C_US0III_IER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0008) //!< (US0III) Interrupt Enable Register
#define D940C_US0III_CR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0000) //!< (US0III) Control Register
#define D940C_US0III_THR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB001C) //!< (US0III) Transmitter Holding Register
#define D940C_US0III_CSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0014) //!< (US0III) Channel Status Register
#define D940C_US0III_BRGR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0020) //!< (US0III) Baud Rate Generator Register
#define D940C_US0III_RTOR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0024) //!< (US0III) Receiver Time-out Register
#define D940C_US0III_TTGR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0028) //!< (US0III) Transmitter Time-guard Register
#define D940C_US0III_IDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB000C) //!< (US0III) Interrupt Disable Register
#define D940C_US0III_MR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0004) //!< (US0III) Mode Register
#define D940C_US0III_IF ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB004C) //!< (US0III) IRDA_FILTER Register
#define D940C_US0III_FIDI ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0040) //!< (US0III) FI_DI_Ratio Register
#define D940C_US0III_IMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB0010) //!< (US0III) Interrupt Mask Register
// ========== Register definition for PDC_US1 peripheral ========== 
#define D940C_US1_PTCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4120) //!< (PDC_US1) PDC Transfer Control Register
#define D940C_US1_RCR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4104) //!< (PDC_US1) Receive Counter Register
#define D940C_US1_RPR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4100) //!< (PDC_US1) Receive Pointer Register
#define D940C_US1_PTSR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4124) //!< (PDC_US1) PDC Transfer Status Register
#define D940C_US1_TPR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4108) //!< (PDC_US1) Transmit Pointer Register
#define D940C_US1_TCR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB410C) //!< (PDC_US1) Transmit Counter Register
#define D940C_US1_RNPR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4110) //!< (PDC_US1) Receive Next Pointer Register
#define D940C_US1_TNCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB411C) //!< (PDC_US1) Transmit Next Counter Register
#define D940C_US1_RNCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4114) //!< (PDC_US1) Receive Next Counter Register
#define D940C_US1_TNPR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4118) //!< (PDC_US1) Transmit Next Pointer Register
// ========== Register definition for US1 peripheral ========== 
#define D940C_US1_THR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB401C) //!< (US1) Transmitter Holding Register
#define D940C_US1_TTGR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4028) //!< (US1) Transmitter Time-guard Register
#define D940C_US1_BRGR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4020) //!< (US1) Baud Rate Generator Register
#define D940C_US1_IDR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB400C) //!< (US1) Interrupt Disable Register
#define D940C_US1_MR    ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4004) //!< (US1) Mode Register
#define D940C_US1_RTOR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4024) //!< (US1) Receiver Time-out Register
#define D940C_US1_CR    ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4000) //!< (US1) Control Register
#define D940C_US1_IMR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4010) //!< (US1) Interrupt Mask Register
#define D940C_US1_FIDI  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4040) //!< (US1) FI_DI_Ratio Register
#define D940C_US1_RHR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4018) //!< (US1) Receiver Holding Register
#define D940C_US1_IER   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4008) //!< (US1) Interrupt Enable Register
#define D940C_US1_CSR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4014) //!< (US1) Channel Status Register
#define D940C_US1_IF    ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB404C) //!< (US1) IRDA_FILTER Register
#define D940C_US1_NER   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4044) //!< (US1) Nb Errors Register
// ========== Register definition for PDC_US1II peripheral ========== 
#define D940C_US1II_PTCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4120) //!< (PDC_US1II) PDC Transfer Control Register
#define D940C_US1II_RCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4104) //!< (PDC_US1II) Receive Counter Register
#define D940C_US1II_RPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4100) //!< (PDC_US1II) Receive Pointer Register
#define D940C_US1II_PTSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4124) //!< (PDC_US1II) PDC Transfer Status Register
#define D940C_US1II_TPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4108) //!< (PDC_US1II) Transmit Pointer Register
#define D940C_US1II_TCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB410C) //!< (PDC_US1II) Transmit Counter Register
#define D940C_US1II_RNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4110) //!< (PDC_US1II) Receive Next Pointer Register
#define D940C_US1II_TNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB411C) //!< (PDC_US1II) Transmit Next Counter Register
#define D940C_US1II_RNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4114) //!< (PDC_US1II) Receive Next Counter Register
#define D940C_US1II_TNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4118) //!< (PDC_US1II) Transmit Next Pointer Register
// ========== Register definition for US1II peripheral ========== 
#define D940C_US1II_THR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB401C) //!< (US1II) Transmitter Holding Register
#define D940C_US1II_TTGR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4028) //!< (US1II) Transmitter Time-guard Register
#define D940C_US1II_BRGR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4020) //!< (US1II) Baud Rate Generator Register
#define D940C_US1II_IDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB400C) //!< (US1II) Interrupt Disable Register
#define D940C_US1II_MR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4004) //!< (US1II) Mode Register
#define D940C_US1II_RTOR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4024) //!< (US1II) Receiver Time-out Register
#define D940C_US1II_CR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4000) //!< (US1II) Control Register
#define D940C_US1II_IMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4010) //!< (US1II) Interrupt Mask Register
#define D940C_US1II_FIDI ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4040) //!< (US1II) FI_DI_Ratio Register
#define D940C_US1II_RHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4018) //!< (US1II) Receiver Holding Register
#define D940C_US1II_IER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4008) //!< (US1II) Interrupt Enable Register
#define D940C_US1II_CSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4014) //!< (US1II) Channel Status Register
#define D940C_US1II_IF  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB404C) //!< (US1II) IRDA_FILTER Register
#define D940C_US1II_NER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4044) //!< (US1II) Nb Errors Register
// ========== Register definition for PDC_US1III peripheral ========== 
#define D940C_US1III_PTCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4120) //!< (PDC_US1III) PDC Transfer Control Register
#define D940C_US1III_RCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4104) //!< (PDC_US1III) Receive Counter Register
#define D940C_US1III_RPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4100) //!< (PDC_US1III) Receive Pointer Register
#define D940C_US1III_PTSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4124) //!< (PDC_US1III) PDC Transfer Status Register
#define D940C_US1III_TPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4108) //!< (PDC_US1III) Transmit Pointer Register
#define D940C_US1III_TCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB410C) //!< (PDC_US1III) Transmit Counter Register
#define D940C_US1III_RNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4110) //!< (PDC_US1III) Receive Next Pointer Register
#define D940C_US1III_TNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB411C) //!< (PDC_US1III) Transmit Next Counter Register
#define D940C_US1III_RNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4114) //!< (PDC_US1III) Receive Next Counter Register
#define D940C_US1III_TNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4118) //!< (PDC_US1III) Transmit Next Pointer Register
// ========== Register definition for US1III peripheral ========== 
#define D940C_US1III_THR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB401C) //!< (US1III) Transmitter Holding Register
#define D940C_US1III_TTGR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4028) //!< (US1III) Transmitter Time-guard Register
#define D940C_US1III_BRGR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4020) //!< (US1III) Baud Rate Generator Register
#define D940C_US1III_IDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB400C) //!< (US1III) Interrupt Disable Register
#define D940C_US1III_MR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4004) //!< (US1III) Mode Register
#define D940C_US1III_RTOR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4024) //!< (US1III) Receiver Time-out Register
#define D940C_US1III_CR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4000) //!< (US1III) Control Register
#define D940C_US1III_IMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4010) //!< (US1III) Interrupt Mask Register
#define D940C_US1III_FIDI ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4040) //!< (US1III) FI_DI_Ratio Register
#define D940C_US1III_RHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4018) //!< (US1III) Receiver Holding Register
#define D940C_US1III_IER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4008) //!< (US1III) Interrupt Enable Register
#define D940C_US1III_CSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4014) //!< (US1III) Channel Status Register
#define D940C_US1III_IF ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB404C) //!< (US1III) IRDA_FILTER Register
#define D940C_US1III_NER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB4044) //!< (US1III) Nb Errors Register
// ========== Register definition for PDC_US2 peripheral ========== 
#define D940C_US2_TNCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB811C) //!< (PDC_US2) Transmit Next Counter Register
#define D940C_US2_RNCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8114) //!< (PDC_US2) Receive Next Counter Register
#define D940C_US2_TPR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8108) //!< (PDC_US2) Transmit Pointer Register
#define D940C_US2_TNPR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8118) //!< (PDC_US2) Transmit Next Pointer Register
#define D940C_US2_PTCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8120) //!< (PDC_US2) PDC Transfer Control Register
#define D940C_US2_TCR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB810C) //!< (PDC_US2) Transmit Counter Register
#define D940C_US2_RCR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8104) //!< (PDC_US2) Receive Counter Register
#define D940C_US2_PTSR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8124) //!< (PDC_US2) PDC Transfer Status Register
#define D940C_US2_RNPR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8110) //!< (PDC_US2) Receive Next Pointer Register
#define D940C_US2_RPR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8100) //!< (PDC_US2) Receive Pointer Register
// ========== Register definition for US2 peripheral ========== 
#define D940C_US2_RTOR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8024) //!< (US2) Receiver Time-out Register
#define D940C_US2_CSR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8014) //!< (US2) Channel Status Register
#define D940C_US2_CR    ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8000) //!< (US2) Control Register
#define D940C_US2_BRGR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8020) //!< (US2) Baud Rate Generator Register
#define D940C_US2_NER   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8044) //!< (US2) Nb Errors Register
#define D940C_US2_FIDI  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8040) //!< (US2) FI_DI_Ratio Register
#define D940C_US2_TTGR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8028) //!< (US2) Transmitter Time-guard Register
#define D940C_US2_RHR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8018) //!< (US2) Receiver Holding Register
#define D940C_US2_IDR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB800C) //!< (US2) Interrupt Disable Register
#define D940C_US2_THR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB801C) //!< (US2) Transmitter Holding Register
#define D940C_US2_MR    ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8004) //!< (US2) Mode Register
#define D940C_US2_IMR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8010) //!< (US2) Interrupt Mask Register
#define D940C_US2_IF    ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB804C) //!< (US2) IRDA_FILTER Register
#define D940C_US2_IER   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8008) //!< (US2) Interrupt Enable Register
// ========== Register definition for PDC_US2II peripheral ========== 
#define D940C_US2II_TNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB811C) //!< (PDC_US2II) Transmit Next Counter Register
#define D940C_US2II_RNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8114) //!< (PDC_US2II) Receive Next Counter Register
#define D940C_US2II_TPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8108) //!< (PDC_US2II) Transmit Pointer Register
#define D940C_US2II_TNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8118) //!< (PDC_US2II) Transmit Next Pointer Register
#define D940C_US2II_PTCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8120) //!< (PDC_US2II) PDC Transfer Control Register
#define D940C_US2II_TCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB810C) //!< (PDC_US2II) Transmit Counter Register
#define D940C_US2II_RCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8104) //!< (PDC_US2II) Receive Counter Register
#define D940C_US2II_PTSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8124) //!< (PDC_US2II) PDC Transfer Status Register
#define D940C_US2II_RNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8110) //!< (PDC_US2II) Receive Next Pointer Register
#define D940C_US2II_RPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8100) //!< (PDC_US2II) Receive Pointer Register
// ========== Register definition for US2II peripheral ========== 
#define D940C_US2II_RTOR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8024) //!< (US2II) Receiver Time-out Register
#define D940C_US2II_CSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8014) //!< (US2II) Channel Status Register
#define D940C_US2II_CR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8000) //!< (US2II) Control Register
#define D940C_US2II_BRGR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8020) //!< (US2II) Baud Rate Generator Register
#define D940C_US2II_NER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8044) //!< (US2II) Nb Errors Register
#define D940C_US2II_FIDI ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8040) //!< (US2II) FI_DI_Ratio Register
#define D940C_US2II_TTGR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8028) //!< (US2II) Transmitter Time-guard Register
#define D940C_US2II_RHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8018) //!< (US2II) Receiver Holding Register
#define D940C_US2II_IDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB800C) //!< (US2II) Interrupt Disable Register
#define D940C_US2II_THR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB801C) //!< (US2II) Transmitter Holding Register
#define D940C_US2II_MR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8004) //!< (US2II) Mode Register
#define D940C_US2II_IMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8010) //!< (US2II) Interrupt Mask Register
#define D940C_US2II_IF  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB804C) //!< (US2II) IRDA_FILTER Register
#define D940C_US2II_IER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8008) //!< (US2II) Interrupt Enable Register
// ========== Register definition for PDC_US2III peripheral ========== 
#define D940C_US2III_TNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB811C) //!< (PDC_US2III) Transmit Next Counter Register
#define D940C_US2III_RNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8114) //!< (PDC_US2III) Receive Next Counter Register
#define D940C_US2III_TPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8108) //!< (PDC_US2III) Transmit Pointer Register
#define D940C_US2III_TNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8118) //!< (PDC_US2III) Transmit Next Pointer Register
#define D940C_US2III_PTCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8120) //!< (PDC_US2III) PDC Transfer Control Register
#define D940C_US2III_TCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB810C) //!< (PDC_US2III) Transmit Counter Register
#define D940C_US2III_RCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8104) //!< (PDC_US2III) Receive Counter Register
#define D940C_US2III_PTSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8124) //!< (PDC_US2III) PDC Transfer Status Register
#define D940C_US2III_RNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8110) //!< (PDC_US2III) Receive Next Pointer Register
#define D940C_US2III_RPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8100) //!< (PDC_US2III) Receive Pointer Register
// ========== Register definition for US2III peripheral ========== 
#define D940C_US2III_RTOR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8024) //!< (US2III) Receiver Time-out Register
#define D940C_US2III_CSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8014) //!< (US2III) Channel Status Register
#define D940C_US2III_CR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8000) //!< (US2III) Control Register
#define D940C_US2III_BRGR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8020) //!< (US2III) Baud Rate Generator Register
#define D940C_US2III_NER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8044) //!< (US2III) Nb Errors Register
#define D940C_US2III_FIDI ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8040) //!< (US2III) FI_DI_Ratio Register
#define D940C_US2III_TTGR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8028) //!< (US2III) Transmitter Time-guard Register
#define D940C_US2III_RHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8018) //!< (US2III) Receiver Holding Register
#define D940C_US2III_IDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB800C) //!< (US2III) Interrupt Disable Register
#define D940C_US2III_THR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB801C) //!< (US2III) Transmitter Holding Register
#define D940C_US2III_MR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8004) //!< (US2III) Mode Register
#define D940C_US2III_IMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8010) //!< (US2III) Interrupt Mask Register
#define D940C_US2III_IF ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB804C) //!< (US2III) IRDA_FILTER Register
#define D940C_US2III_IER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFB8008) //!< (US2III) Interrupt Enable Register
// ========== Register definition for PDC_SSC0 peripheral ========== 
#define D940C_SSC0_TNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC118) //!< (PDC_SSC0) Transmit Next Pointer Register
#define D940C_SSC0_TCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC10C) //!< (PDC_SSC0) Transmit Counter Register
#define D940C_SSC0_RNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC114) //!< (PDC_SSC0) Receive Next Counter Register
#define D940C_SSC0_RPR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC100) //!< (PDC_SSC0) Receive Pointer Register
#define D940C_SSC0_TPR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC108) //!< (PDC_SSC0) Transmit Pointer Register
#define D940C_SSC0_RCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC104) //!< (PDC_SSC0) Receive Counter Register
#define D940C_SSC0_RNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC110) //!< (PDC_SSC0) Receive Next Pointer Register
#define D940C_SSC0_PTCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC120) //!< (PDC_SSC0) PDC Transfer Control Register
#define D940C_SSC0_TNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC11C) //!< (PDC_SSC0) Transmit Next Counter Register
#define D940C_SSC0_PTSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC124) //!< (PDC_SSC0) PDC Transfer Status Register
// ========== Register definition for SSC0 peripheral ========== 
#define D940C_SSC0_IMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC04C) //!< (SSC0) Interrupt Mask Register
#define D940C_SSC0_RFMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC014) //!< (SSC0) Receive Frame Mode Register
#define D940C_SSC0_RCMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC010) //!< (SSC0) Receive Clock ModeRegister
#define D940C_SSC0_SR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC040) //!< (SSC0) Status Register
#define D940C_SSC0_RSHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC030) //!< (SSC0) Receive Sync Holding Register
#define D940C_SSC0_THR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC024) //!< (SSC0) Transmit Holding Register
#define D940C_SSC0_CR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC000) //!< (SSC0) Control Register
#define D940C_SSC0_TFMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC01C) //!< (SSC0) Transmit Frame Mode Register
#define D940C_SSC0_CMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC004) //!< (SSC0) Clock Mode Register
#define D940C_SSC0_IER  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC044) //!< (SSC0) Interrupt Enable Register
#define D940C_SSC0_TCMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC018) //!< (SSC0) Transmit Clock Mode Register
#define D940C_SSC0_IDR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC048) //!< (SSC0) Interrupt Disable Register
#define D940C_SSC0_TSHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC034) //!< (SSC0) Transmit Sync Holding Register
#define D940C_SSC0_RHR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC020) //!< (SSC0) Receive Holding Register
// ========== Register definition for PDC_SSC0II peripheral ========== 
#define D940C_SSC0II_TNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC118) //!< (PDC_SSC0II) Transmit Next Pointer Register
#define D940C_SSC0II_TCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC10C) //!< (PDC_SSC0II) Transmit Counter Register
#define D940C_SSC0II_RNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC114) //!< (PDC_SSC0II) Receive Next Counter Register
#define D940C_SSC0II_RPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC100) //!< (PDC_SSC0II) Receive Pointer Register
#define D940C_SSC0II_TPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC108) //!< (PDC_SSC0II) Transmit Pointer Register
#define D940C_SSC0II_RCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC104) //!< (PDC_SSC0II) Receive Counter Register
#define D940C_SSC0II_RNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC110) //!< (PDC_SSC0II) Receive Next Pointer Register
#define D940C_SSC0II_PTCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC120) //!< (PDC_SSC0II) PDC Transfer Control Register
#define D940C_SSC0II_TNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC11C) //!< (PDC_SSC0II) Transmit Next Counter Register
#define D940C_SSC0II_PTSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC124) //!< (PDC_SSC0II) PDC Transfer Status Register
// ========== Register definition for SSC0II peripheral ========== 
#define D940C_SSC0II_IMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC04C) //!< (SSC0II) Interrupt Mask Register
#define D940C_SSC0II_RFMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC014) //!< (SSC0II) Receive Frame Mode Register
#define D940C_SSC0II_RCMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC010) //!< (SSC0II) Receive Clock ModeRegister
#define D940C_SSC0II_SR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC040) //!< (SSC0II) Status Register
#define D940C_SSC0II_RSHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC030) //!< (SSC0II) Receive Sync Holding Register
#define D940C_SSC0II_THR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC024) //!< (SSC0II) Transmit Holding Register
#define D940C_SSC0II_CR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC000) //!< (SSC0II) Control Register
#define D940C_SSC0II_TFMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC01C) //!< (SSC0II) Transmit Frame Mode Register
#define D940C_SSC0II_CMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC004) //!< (SSC0II) Clock Mode Register
#define D940C_SSC0II_IER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC044) //!< (SSC0II) Interrupt Enable Register
#define D940C_SSC0II_TCMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC018) //!< (SSC0II) Transmit Clock Mode Register
#define D940C_SSC0II_IDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC048) //!< (SSC0II) Interrupt Disable Register
#define D940C_SSC0II_TSHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC034) //!< (SSC0II) Transmit Sync Holding Register
#define D940C_SSC0II_RHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC020) //!< (SSC0II) Receive Holding Register
// ========== Register definition for PDC_SSC0III peripheral ========== 
#define D940C_SSC0III_TNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC118) //!< (PDC_SSC0III) Transmit Next Pointer Register
#define D940C_SSC0III_TCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC10C) //!< (PDC_SSC0III) Transmit Counter Register
#define D940C_SSC0III_RNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC114) //!< (PDC_SSC0III) Receive Next Counter Register
#define D940C_SSC0III_RPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC100) //!< (PDC_SSC0III) Receive Pointer Register
#define D940C_SSC0III_TPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC108) //!< (PDC_SSC0III) Transmit Pointer Register
#define D940C_SSC0III_RCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC104) //!< (PDC_SSC0III) Receive Counter Register
#define D940C_SSC0III_RNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC110) //!< (PDC_SSC0III) Receive Next Pointer Register
#define D940C_SSC0III_PTCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC120) //!< (PDC_SSC0III) PDC Transfer Control Register
#define D940C_SSC0III_TNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC11C) //!< (PDC_SSC0III) Transmit Next Counter Register
#define D940C_SSC0III_PTSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC124) //!< (PDC_SSC0III) PDC Transfer Status Register
// ========== Register definition for SSC0III peripheral ========== 
#define D940C_SSC0III_IMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC04C) //!< (SSC0III) Interrupt Mask Register
#define D940C_SSC0III_RFMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC014) //!< (SSC0III) Receive Frame Mode Register
#define D940C_SSC0III_RCMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC010) //!< (SSC0III) Receive Clock ModeRegister
#define D940C_SSC0III_SR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC040) //!< (SSC0III) Status Register
#define D940C_SSC0III_RSHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC030) //!< (SSC0III) Receive Sync Holding Register
#define D940C_SSC0III_THR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC024) //!< (SSC0III) Transmit Holding Register
#define D940C_SSC0III_CR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC000) //!< (SSC0III) Control Register
#define D940C_SSC0III_TFMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC01C) //!< (SSC0III) Transmit Frame Mode Register
#define D940C_SSC0III_CMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC004) //!< (SSC0III) Clock Mode Register
#define D940C_SSC0III_IER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC044) //!< (SSC0III) Interrupt Enable Register
#define D940C_SSC0III_TCMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC018) //!< (SSC0III) Transmit Clock Mode Register
#define D940C_SSC0III_IDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC048) //!< (SSC0III) Interrupt Disable Register
#define D940C_SSC0III_TSHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC034) //!< (SSC0III) Transmit Sync Holding Register
#define D940C_SSC0III_RHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFBC020) //!< (SSC0III) Receive Holding Register
// ========== Register definition for PDC_SSC1 peripheral ========== 
#define D940C_SSC1_TNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0118) //!< (PDC_SSC1) Transmit Next Pointer Register
#define D940C_SSC1_RNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0110) //!< (PDC_SSC1) Receive Next Pointer Register
#define D940C_SSC1_TCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC010C) //!< (PDC_SSC1) Transmit Counter Register
#define D940C_SSC1_PTCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0120) //!< (PDC_SSC1) PDC Transfer Control Register
#define D940C_SSC1_PTSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0124) //!< (PDC_SSC1) PDC Transfer Status Register
#define D940C_SSC1_TNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC011C) //!< (PDC_SSC1) Transmit Next Counter Register
#define D940C_SSC1_TPR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0108) //!< (PDC_SSC1) Transmit Pointer Register
#define D940C_SSC1_RCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0104) //!< (PDC_SSC1) Receive Counter Register
#define D940C_SSC1_RPR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0100) //!< (PDC_SSC1) Receive Pointer Register
#define D940C_SSC1_RNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0114) //!< (PDC_SSC1) Receive Next Counter Register
// ========== Register definition for SSC1 peripheral ========== 
#define D940C_SSC1_IDR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0048) //!< (SSC1) Interrupt Disable Register
#define D940C_SSC1_RHR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0020) //!< (SSC1) Receive Holding Register
#define D940C_SSC1_IER  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0044) //!< (SSC1) Interrupt Enable Register
#define D940C_SSC1_CR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0000) //!< (SSC1) Control Register
#define D940C_SSC1_RCMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0010) //!< (SSC1) Receive Clock ModeRegister
#define D940C_SSC1_SR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0040) //!< (SSC1) Status Register
#define D940C_SSC1_TSHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0034) //!< (SSC1) Transmit Sync Holding Register
#define D940C_SSC1_CMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0004) //!< (SSC1) Clock Mode Register
#define D940C_SSC1_RSHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0030) //!< (SSC1) Receive Sync Holding Register
#define D940C_SSC1_THR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0024) //!< (SSC1) Transmit Holding Register
#define D940C_SSC1_RFMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0014) //!< (SSC1) Receive Frame Mode Register
#define D940C_SSC1_TCMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0018) //!< (SSC1) Transmit Clock Mode Register
#define D940C_SSC1_TFMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC001C) //!< (SSC1) Transmit Frame Mode Register
#define D940C_SSC1_IMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC004C) //!< (SSC1) Interrupt Mask Register
// ========== Register definition for PDC_SSC1II peripheral ========== 
#define D940C_SSC1II_TNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0118) //!< (PDC_SSC1II) Transmit Next Pointer Register
#define D940C_SSC1II_RNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0110) //!< (PDC_SSC1II) Receive Next Pointer Register
#define D940C_SSC1II_TCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC010C) //!< (PDC_SSC1II) Transmit Counter Register
#define D940C_SSC1II_PTCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0120) //!< (PDC_SSC1II) PDC Transfer Control Register
#define D940C_SSC1II_PTSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0124) //!< (PDC_SSC1II) PDC Transfer Status Register
#define D940C_SSC1II_TNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC011C) //!< (PDC_SSC1II) Transmit Next Counter Register
#define D940C_SSC1II_TPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0108) //!< (PDC_SSC1II) Transmit Pointer Register
#define D940C_SSC1II_RCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0104) //!< (PDC_SSC1II) Receive Counter Register
#define D940C_SSC1II_RPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0100) //!< (PDC_SSC1II) Receive Pointer Register
#define D940C_SSC1II_RNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0114) //!< (PDC_SSC1II) Receive Next Counter Register
// ========== Register definition for SSC1II peripheral ========== 
#define D940C_SSC1II_IDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0048) //!< (SSC1II) Interrupt Disable Register
#define D940C_SSC1II_RHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0020) //!< (SSC1II) Receive Holding Register
#define D940C_SSC1II_IER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0044) //!< (SSC1II) Interrupt Enable Register
#define D940C_SSC1II_CR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0000) //!< (SSC1II) Control Register
#define D940C_SSC1II_RCMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0010) //!< (SSC1II) Receive Clock ModeRegister
#define D940C_SSC1II_SR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0040) //!< (SSC1II) Status Register
#define D940C_SSC1II_TSHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0034) //!< (SSC1II) Transmit Sync Holding Register
#define D940C_SSC1II_CMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0004) //!< (SSC1II) Clock Mode Register
#define D940C_SSC1II_RSHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0030) //!< (SSC1II) Receive Sync Holding Register
#define D940C_SSC1II_THR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0024) //!< (SSC1II) Transmit Holding Register
#define D940C_SSC1II_RFMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0014) //!< (SSC1II) Receive Frame Mode Register
#define D940C_SSC1II_TCMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0018) //!< (SSC1II) Transmit Clock Mode Register
#define D940C_SSC1II_TFMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC001C) //!< (SSC1II) Transmit Frame Mode Register
#define D940C_SSC1II_IMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC004C) //!< (SSC1II) Interrupt Mask Register
// ========== Register definition for PDC_SSC1III peripheral ========== 
#define D940C_SSC1III_TNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0118) //!< (PDC_SSC1III) Transmit Next Pointer Register
#define D940C_SSC1III_RNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0110) //!< (PDC_SSC1III) Receive Next Pointer Register
#define D940C_SSC1III_TCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC010C) //!< (PDC_SSC1III) Transmit Counter Register
#define D940C_SSC1III_PTCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0120) //!< (PDC_SSC1III) PDC Transfer Control Register
#define D940C_SSC1III_PTSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0124) //!< (PDC_SSC1III) PDC Transfer Status Register
#define D940C_SSC1III_TNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC011C) //!< (PDC_SSC1III) Transmit Next Counter Register
#define D940C_SSC1III_TPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0108) //!< (PDC_SSC1III) Transmit Pointer Register
#define D940C_SSC1III_RCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0104) //!< (PDC_SSC1III) Receive Counter Register
#define D940C_SSC1III_RPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0100) //!< (PDC_SSC1III) Receive Pointer Register
#define D940C_SSC1III_RNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0114) //!< (PDC_SSC1III) Receive Next Counter Register
// ========== Register definition for SSC1III peripheral ========== 
#define D940C_SSC1III_IDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0048) //!< (SSC1III) Interrupt Disable Register
#define D940C_SSC1III_RHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0020) //!< (SSC1III) Receive Holding Register
#define D940C_SSC1III_IER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0044) //!< (SSC1III) Interrupt Enable Register
#define D940C_SSC1III_CR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0000) //!< (SSC1III) Control Register
#define D940C_SSC1III_RCMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0010) //!< (SSC1III) Receive Clock ModeRegister
#define D940C_SSC1III_SR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0040) //!< (SSC1III) Status Register
#define D940C_SSC1III_TSHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0034) //!< (SSC1III) Transmit Sync Holding Register
#define D940C_SSC1III_CMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0004) //!< (SSC1III) Clock Mode Register
#define D940C_SSC1III_RSHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0030) //!< (SSC1III) Receive Sync Holding Register
#define D940C_SSC1III_THR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0024) //!< (SSC1III) Transmit Holding Register
#define D940C_SSC1III_RFMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0014) //!< (SSC1III) Receive Frame Mode Register
#define D940C_SSC1III_TCMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC0018) //!< (SSC1III) Transmit Clock Mode Register
#define D940C_SSC1III_TFMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC001C) //!< (SSC1III) Transmit Frame Mode Register
#define D940C_SSC1III_IMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC004C) //!< (SSC1III) Interrupt Mask Register
// ========== Register definition for PDC_SSC2 peripheral ========== 
#define D940C_SSC2_RNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4114) //!< (PDC_SSC2) Receive Next Counter Register
#define D940C_SSC2_PTCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4120) //!< (PDC_SSC2) PDC Transfer Control Register
#define D940C_SSC2_TCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC410C) //!< (PDC_SSC2) Transmit Counter Register
#define D940C_SSC2_PTSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4124) //!< (PDC_SSC2) PDC Transfer Status Register
#define D940C_SSC2_TNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4118) //!< (PDC_SSC2) Transmit Next Pointer Register
#define D940C_SSC2_RCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4104) //!< (PDC_SSC2) Receive Counter Register
#define D940C_SSC2_RNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4110) //!< (PDC_SSC2) Receive Next Pointer Register
#define D940C_SSC2_RPR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4100) //!< (PDC_SSC2) Receive Pointer Register
#define D940C_SSC2_TNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC411C) //!< (PDC_SSC2) Transmit Next Counter Register
#define D940C_SSC2_TPR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4108) //!< (PDC_SSC2) Transmit Pointer Register
// ========== Register definition for SSC2 peripheral ========== 
#define D940C_SSC2_IMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC404C) //!< (SSC2) Interrupt Mask Register
#define D940C_SSC2_IER  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4044) //!< (SSC2) Interrupt Enable Register
#define D940C_SSC2_THR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4024) //!< (SSC2) Transmit Holding Register
#define D940C_SSC2_RFMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4014) //!< (SSC2) Receive Frame Mode Register
#define D940C_SSC2_TFMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC401C) //!< (SSC2) Transmit Frame Mode Register
#define D940C_SSC2_IDR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4048) //!< (SSC2) Interrupt Disable Register
#define D940C_SSC2_RSHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4030) //!< (SSC2) Receive Sync Holding Register
#define D940C_SSC2_TCMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4018) //!< (SSC2) Transmit Clock Mode Register
#define D940C_SSC2_RHR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4020) //!< (SSC2) Receive Holding Register
#define D940C_SSC2_RCMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4010) //!< (SSC2) Receive Clock ModeRegister
#define D940C_SSC2_SR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4040) //!< (SSC2) Status Register
#define D940C_SSC2_CR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4000) //!< (SSC2) Control Register
#define D940C_SSC2_CMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4004) //!< (SSC2) Clock Mode Register
#define D940C_SSC2_TSHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4034) //!< (SSC2) Transmit Sync Holding Register
// ========== Register definition for PDC_SSC2II peripheral ========== 
#define D940C_SSC2II_RNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4114) //!< (PDC_SSC2II) Receive Next Counter Register
#define D940C_SSC2II_PTCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4120) //!< (PDC_SSC2II) PDC Transfer Control Register
#define D940C_SSC2II_TCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC410C) //!< (PDC_SSC2II) Transmit Counter Register
#define D940C_SSC2II_PTSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4124) //!< (PDC_SSC2II) PDC Transfer Status Register
#define D940C_SSC2II_TNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4118) //!< (PDC_SSC2II) Transmit Next Pointer Register
#define D940C_SSC2II_RCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4104) //!< (PDC_SSC2II) Receive Counter Register
#define D940C_SSC2II_RNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4110) //!< (PDC_SSC2II) Receive Next Pointer Register
#define D940C_SSC2II_RPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4100) //!< (PDC_SSC2II) Receive Pointer Register
#define D940C_SSC2II_TNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC411C) //!< (PDC_SSC2II) Transmit Next Counter Register
#define D940C_SSC2II_TPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4108) //!< (PDC_SSC2II) Transmit Pointer Register
// ========== Register definition for SSC2II peripheral ========== 
#define D940C_SSC2II_IMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC404C) //!< (SSC2II) Interrupt Mask Register
#define D940C_SSC2II_IER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4044) //!< (SSC2II) Interrupt Enable Register
#define D940C_SSC2II_THR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4024) //!< (SSC2II) Transmit Holding Register
#define D940C_SSC2II_RFMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4014) //!< (SSC2II) Receive Frame Mode Register
#define D940C_SSC2II_TFMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC401C) //!< (SSC2II) Transmit Frame Mode Register
#define D940C_SSC2II_IDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4048) //!< (SSC2II) Interrupt Disable Register
#define D940C_SSC2II_RSHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4030) //!< (SSC2II) Receive Sync Holding Register
#define D940C_SSC2II_TCMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4018) //!< (SSC2II) Transmit Clock Mode Register
#define D940C_SSC2II_RHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4020) //!< (SSC2II) Receive Holding Register
#define D940C_SSC2II_RCMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4010) //!< (SSC2II) Receive Clock ModeRegister
#define D940C_SSC2II_SR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4040) //!< (SSC2II) Status Register
#define D940C_SSC2II_CR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4000) //!< (SSC2II) Control Register
#define D940C_SSC2II_CMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4004) //!< (SSC2II) Clock Mode Register
#define D940C_SSC2II_TSHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4034) //!< (SSC2II) Transmit Sync Holding Register
// ========== Register definition for PDC_SSC2III peripheral ========== 
#define D940C_SSC2III_RNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4114) //!< (PDC_SSC2III) Receive Next Counter Register
#define D940C_SSC2III_PTCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4120) //!< (PDC_SSC2III) PDC Transfer Control Register
#define D940C_SSC2III_TCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC410C) //!< (PDC_SSC2III) Transmit Counter Register
#define D940C_SSC2III_PTSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4124) //!< (PDC_SSC2III) PDC Transfer Status Register
#define D940C_SSC2III_TNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4118) //!< (PDC_SSC2III) Transmit Next Pointer Register
#define D940C_SSC2III_RCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4104) //!< (PDC_SSC2III) Receive Counter Register
#define D940C_SSC2III_RNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4110) //!< (PDC_SSC2III) Receive Next Pointer Register
#define D940C_SSC2III_RPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4100) //!< (PDC_SSC2III) Receive Pointer Register
#define D940C_SSC2III_TNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC411C) //!< (PDC_SSC2III) Transmit Next Counter Register
#define D940C_SSC2III_TPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4108) //!< (PDC_SSC2III) Transmit Pointer Register
// ========== Register definition for SSC2III peripheral ========== 
#define D940C_SSC2III_IMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC404C) //!< (SSC2III) Interrupt Mask Register
#define D940C_SSC2III_IER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4044) //!< (SSC2III) Interrupt Enable Register
#define D940C_SSC2III_THR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4024) //!< (SSC2III) Transmit Holding Register
#define D940C_SSC2III_RFMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4014) //!< (SSC2III) Receive Frame Mode Register
#define D940C_SSC2III_TFMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC401C) //!< (SSC2III) Transmit Frame Mode Register
#define D940C_SSC2III_IDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4048) //!< (SSC2III) Interrupt Disable Register
#define D940C_SSC2III_RSHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4030) //!< (SSC2III) Receive Sync Holding Register
#define D940C_SSC2III_TCMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4018) //!< (SSC2III) Transmit Clock Mode Register
#define D940C_SSC2III_RHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4020) //!< (SSC2III) Receive Holding Register
#define D940C_SSC2III_RCMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4010) //!< (SSC2III) Receive Clock ModeRegister
#define D940C_SSC2III_SR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4040) //!< (SSC2III) Status Register
#define D940C_SSC2III_CR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4000) //!< (SSC2III) Control Register
#define D940C_SSC2III_CMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4004) //!< (SSC2III) Clock Mode Register
#define D940C_SSC2III_TSHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC4034) //!< (SSC2III) Transmit Sync Holding Register
// ========== Register definition for PDC_SSC3 peripheral ========== 
#define D940C_SSC3_RNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0110) //!< (PDC_SSC3) Receive Next Pointer Register
#define D940C_SSC3_TPR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0108) //!< (PDC_SSC3) Transmit Pointer Register
#define D940C_SSC3_RCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0104) //!< (PDC_SSC3) Receive Counter Register
#define D940C_SSC3_TNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD011C) //!< (PDC_SSC3) Transmit Next Counter Register
#define D940C_SSC3_RNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0114) //!< (PDC_SSC3) Receive Next Counter Register
#define D940C_SSC3_RPR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0100) //!< (PDC_SSC3) Receive Pointer Register
#define D940C_SSC3_PTSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0124) //!< (PDC_SSC3) PDC Transfer Status Register
#define D940C_SSC3_PTCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0120) //!< (PDC_SSC3) PDC Transfer Control Register
#define D940C_SSC3_TCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD010C) //!< (PDC_SSC3) Transmit Counter Register
#define D940C_SSC3_TNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0118) //!< (PDC_SSC3) Transmit Next Pointer Register
// ========== Register definition for SSC3 peripheral ========== 
#define D940C_SSC3_IER  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0044) //!< (SSC3) Interrupt Enable Register
#define D940C_SSC3_THR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0024) //!< (SSC3) Transmit Holding Register
#define D940C_SSC3_IDR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0048) //!< (SSC3) Interrupt Disable Register
#define D940C_SSC3_CMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0004) //!< (SSC3) Clock Mode Register
#define D940C_SSC3_SR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0040) //!< (SSC3) Status Register
#define D940C_SSC3_RHR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0020) //!< (SSC3) Receive Holding Register
#define D940C_SSC3_TFMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD001C) //!< (SSC3) Transmit Frame Mode Register
#define D940C_SSC3_CR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0000) //!< (SSC3) Control Register
#define D940C_SSC3_IMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD004C) //!< (SSC3) Interrupt Mask Register
#define D940C_SSC3_TCMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0018) //!< (SSC3) Transmit Clock Mode Register
#define D940C_SSC3_RSHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0030) //!< (SSC3) Receive Sync Holding Register
#define D940C_SSC3_RCMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0010) //!< (SSC3) Receive Clock ModeRegister
#define D940C_SSC3_TSHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0034) //!< (SSC3) Transmit Sync Holding Register
#define D940C_SSC3_RFMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0014) //!< (SSC3) Receive Frame Mode Register
// ========== Register definition for PDC_SSC3II peripheral ========== 
#define D940C_SSC3II_RNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0110) //!< (PDC_SSC3II) Receive Next Pointer Register
#define D940C_SSC3II_TPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0108) //!< (PDC_SSC3II) Transmit Pointer Register
#define D940C_SSC3II_RCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0104) //!< (PDC_SSC3II) Receive Counter Register
#define D940C_SSC3II_TNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD011C) //!< (PDC_SSC3II) Transmit Next Counter Register
#define D940C_SSC3II_RNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0114) //!< (PDC_SSC3II) Receive Next Counter Register
#define D940C_SSC3II_RPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0100) //!< (PDC_SSC3II) Receive Pointer Register
#define D940C_SSC3II_PTSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0124) //!< (PDC_SSC3II) PDC Transfer Status Register
#define D940C_SSC3II_PTCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0120) //!< (PDC_SSC3II) PDC Transfer Control Register
#define D940C_SSC3II_TCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD010C) //!< (PDC_SSC3II) Transmit Counter Register
#define D940C_SSC3II_TNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0118) //!< (PDC_SSC3II) Transmit Next Pointer Register
// ========== Register definition for SSC3II peripheral ========== 
#define D940C_SSC3II_IER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0044) //!< (SSC3II) Interrupt Enable Register
#define D940C_SSC3II_THR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0024) //!< (SSC3II) Transmit Holding Register
#define D940C_SSC3II_IDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0048) //!< (SSC3II) Interrupt Disable Register
#define D940C_SSC3II_CMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0004) //!< (SSC3II) Clock Mode Register
#define D940C_SSC3II_SR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0040) //!< (SSC3II) Status Register
#define D940C_SSC3II_RHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0020) //!< (SSC3II) Receive Holding Register
#define D940C_SSC3II_TFMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD001C) //!< (SSC3II) Transmit Frame Mode Register
#define D940C_SSC3II_CR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0000) //!< (SSC3II) Control Register
#define D940C_SSC3II_IMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD004C) //!< (SSC3II) Interrupt Mask Register
#define D940C_SSC3II_TCMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0018) //!< (SSC3II) Transmit Clock Mode Register
#define D940C_SSC3II_RSHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0030) //!< (SSC3II) Receive Sync Holding Register
#define D940C_SSC3II_RCMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0010) //!< (SSC3II) Receive Clock ModeRegister
#define D940C_SSC3II_TSHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0034) //!< (SSC3II) Transmit Sync Holding Register
#define D940C_SSC3II_RFMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0014) //!< (SSC3II) Receive Frame Mode Register
// ========== Register definition for PDC_SSC3III peripheral ========== 
#define D940C_SSC3III_RNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0110) //!< (PDC_SSC3III) Receive Next Pointer Register
#define D940C_SSC3III_TPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0108) //!< (PDC_SSC3III) Transmit Pointer Register
#define D940C_SSC3III_RCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0104) //!< (PDC_SSC3III) Receive Counter Register
#define D940C_SSC3III_TNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD011C) //!< (PDC_SSC3III) Transmit Next Counter Register
#define D940C_SSC3III_RNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0114) //!< (PDC_SSC3III) Receive Next Counter Register
#define D940C_SSC3III_RPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0100) //!< (PDC_SSC3III) Receive Pointer Register
#define D940C_SSC3III_PTSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0124) //!< (PDC_SSC3III) PDC Transfer Status Register
#define D940C_SSC3III_PTCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0120) //!< (PDC_SSC3III) PDC Transfer Control Register
#define D940C_SSC3III_TCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD010C) //!< (PDC_SSC3III) Transmit Counter Register
#define D940C_SSC3III_TNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0118) //!< (PDC_SSC3III) Transmit Next Pointer Register
// ========== Register definition for SSC3III peripheral ========== 
#define D940C_SSC3III_IER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0044) //!< (SSC3III) Interrupt Enable Register
#define D940C_SSC3III_THR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0024) //!< (SSC3III) Transmit Holding Register
#define D940C_SSC3III_IDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0048) //!< (SSC3III) Interrupt Disable Register
#define D940C_SSC3III_CMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0004) //!< (SSC3III) Clock Mode Register
#define D940C_SSC3III_SR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0040) //!< (SSC3III) Status Register
#define D940C_SSC3III_RHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0020) //!< (SSC3III) Receive Holding Register
#define D940C_SSC3III_TFMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD001C) //!< (SSC3III) Transmit Frame Mode Register
#define D940C_SSC3III_CR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0000) //!< (SSC3III) Control Register
#define D940C_SSC3III_IMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD004C) //!< (SSC3III) Interrupt Mask Register
#define D940C_SSC3III_TCMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0018) //!< (SSC3III) Transmit Clock Mode Register
#define D940C_SSC3III_RSHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0030) //!< (SSC3III) Receive Sync Holding Register
#define D940C_SSC3III_RCMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0010) //!< (SSC3III) Receive Clock ModeRegister
#define D940C_SSC3III_TSHR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0034) //!< (SSC3III) Transmit Sync Holding Register
#define D940C_SSC3III_RFMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD0014) //!< (SSC3III) Receive Frame Mode Register
// ========== Register definition for PDC_SPI0 peripheral ========== 
#define D940C_SPI0_PTCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8120) //!< (PDC_SPI0) PDC Transfer Control Register
#define D940C_SPI0_TPR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8108) //!< (PDC_SPI0) Transmit Pointer Register
#define D940C_SPI0_PTSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8124) //!< (PDC_SPI0) PDC Transfer Status Register
#define D940C_SPI0_TCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC810C) //!< (PDC_SPI0) Transmit Counter Register
#define D940C_SPI0_TNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8118) //!< (PDC_SPI0) Transmit Next Pointer Register
#define D940C_SPI0_RNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8114) //!< (PDC_SPI0) Receive Next Counter Register
#define D940C_SPI0_RPR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8100) //!< (PDC_SPI0) Receive Pointer Register
#define D940C_SPI0_TNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC811C) //!< (PDC_SPI0) Transmit Next Counter Register
#define D940C_SPI0_RNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8110) //!< (PDC_SPI0) Receive Next Pointer Register
#define D940C_SPI0_RCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8104) //!< (PDC_SPI0) Receive Counter Register
// ========== Register definition for SPI0 peripheral ========== 
#define D940C_SPI0_CSR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8030) //!< (SPI0) Chip Select Register
#define D940C_SPI0_IDR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8018) //!< (SPI0) Interrupt Disable Register
#define D940C_SPI0_RDR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8008) //!< (SPI0) Receive Data Register
#define D940C_SPI0_IER  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8014) //!< (SPI0) Interrupt Enable Register
#define D940C_SPI0_MR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8004) //!< (SPI0) Mode Register
#define D940C_SPI0_TDR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC800C) //!< (SPI0) Transmit Data Register
#define D940C_SPI0_SR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8010) //!< (SPI0) Status Register
#define D940C_SPI0_IMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC801C) //!< (SPI0) Interrupt Mask Register
#define D940C_SPI0_CR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8000) //!< (SPI0) Control Register
// ========== Register definition for PDC_SPI0II peripheral ========== 
#define D940C_SPI0II_PTCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8120) //!< (PDC_SPI0II) PDC Transfer Control Register
#define D940C_SPI0II_TPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8108) //!< (PDC_SPI0II) Transmit Pointer Register
#define D940C_SPI0II_PTSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8124) //!< (PDC_SPI0II) PDC Transfer Status Register
#define D940C_SPI0II_TCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC810C) //!< (PDC_SPI0II) Transmit Counter Register
#define D940C_SPI0II_TNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8118) //!< (PDC_SPI0II) Transmit Next Pointer Register
#define D940C_SPI0II_RNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8114) //!< (PDC_SPI0II) Receive Next Counter Register
#define D940C_SPI0II_RPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8100) //!< (PDC_SPI0II) Receive Pointer Register
#define D940C_SPI0II_TNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC811C) //!< (PDC_SPI0II) Transmit Next Counter Register
#define D940C_SPI0II_RNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8110) //!< (PDC_SPI0II) Receive Next Pointer Register
#define D940C_SPI0II_RCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8104) //!< (PDC_SPI0II) Receive Counter Register
// ========== Register definition for SPI0II peripheral ========== 
#define D940C_SPI0II_CSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8030) //!< (SPI0II) Chip Select Register
#define D940C_SPI0II_IDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8018) //!< (SPI0II) Interrupt Disable Register
#define D940C_SPI0II_RDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8008) //!< (SPI0II) Receive Data Register
#define D940C_SPI0II_IER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8014) //!< (SPI0II) Interrupt Enable Register
#define D940C_SPI0II_MR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8004) //!< (SPI0II) Mode Register
#define D940C_SPI0II_TDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC800C) //!< (SPI0II) Transmit Data Register
#define D940C_SPI0II_SR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8010) //!< (SPI0II) Status Register
#define D940C_SPI0II_IMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC801C) //!< (SPI0II) Interrupt Mask Register
#define D940C_SPI0II_CR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8000) //!< (SPI0II) Control Register
// ========== Register definition for PDC_SPI0III peripheral ========== 
#define D940C_SPI0III_PTCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8120) //!< (PDC_SPI0III) PDC Transfer Control Register
#define D940C_SPI0III_TPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8108) //!< (PDC_SPI0III) Transmit Pointer Register
#define D940C_SPI0III_PTSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8124) //!< (PDC_SPI0III) PDC Transfer Status Register
#define D940C_SPI0III_TCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC810C) //!< (PDC_SPI0III) Transmit Counter Register
#define D940C_SPI0III_TNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8118) //!< (PDC_SPI0III) Transmit Next Pointer Register
#define D940C_SPI0III_RNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8114) //!< (PDC_SPI0III) Receive Next Counter Register
#define D940C_SPI0III_RPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8100) //!< (PDC_SPI0III) Receive Pointer Register
#define D940C_SPI0III_TNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC811C) //!< (PDC_SPI0III) Transmit Next Counter Register
#define D940C_SPI0III_RNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8110) //!< (PDC_SPI0III) Receive Next Pointer Register
#define D940C_SPI0III_RCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8104) //!< (PDC_SPI0III) Receive Counter Register
// ========== Register definition for SPI0III peripheral ========== 
#define D940C_SPI0III_CSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8030) //!< (SPI0III) Chip Select Register
#define D940C_SPI0III_IDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8018) //!< (SPI0III) Interrupt Disable Register
#define D940C_SPI0III_RDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8008) //!< (SPI0III) Receive Data Register
#define D940C_SPI0III_IER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8014) //!< (SPI0III) Interrupt Enable Register
#define D940C_SPI0III_MR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8004) //!< (SPI0III) Mode Register
#define D940C_SPI0III_TDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC800C) //!< (SPI0III) Transmit Data Register
#define D940C_SPI0III_SR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8010) //!< (SPI0III) Status Register
#define D940C_SPI0III_IMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC801C) //!< (SPI0III) Interrupt Mask Register
#define D940C_SPI0III_CR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFC8000) //!< (SPI0III) Control Register
// ========== Register definition for PDC_SPI1 peripheral ========== 
#define D940C_SPI1_PTCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC120) //!< (PDC_SPI1) PDC Transfer Control Register
#define D940C_SPI1_TPR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC108) //!< (PDC_SPI1) Transmit Pointer Register
#define D940C_SPI1_RNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC110) //!< (PDC_SPI1) Receive Next Pointer Register
#define D940C_SPI1_PTSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC124) //!< (PDC_SPI1) PDC Transfer Status Register
#define D940C_SPI1_TNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC11C) //!< (PDC_SPI1) Transmit Next Counter Register
#define D940C_SPI1_RCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC104) //!< (PDC_SPI1) Receive Counter Register
#define D940C_SPI1_RNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC114) //!< (PDC_SPI1) Receive Next Counter Register
#define D940C_SPI1_TCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC10C) //!< (PDC_SPI1) Transmit Counter Register
#define D940C_SPI1_RPR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC100) //!< (PDC_SPI1) Receive Pointer Register
#define D940C_SPI1_TNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC118) //!< (PDC_SPI1) Transmit Next Pointer Register
// ========== Register definition for SPI1 peripheral ========== 
#define D940C_SPI1_IER  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC014) //!< (SPI1) Interrupt Enable Register
#define D940C_SPI1_RDR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC008) //!< (SPI1) Receive Data Register
#define D940C_SPI1_SR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC010) //!< (SPI1) Status Register
#define D940C_SPI1_IMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC01C) //!< (SPI1) Interrupt Mask Register
#define D940C_SPI1_TDR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC00C) //!< (SPI1) Transmit Data Register
#define D940C_SPI1_IDR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC018) //!< (SPI1) Interrupt Disable Register
#define D940C_SPI1_CSR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC030) //!< (SPI1) Chip Select Register
#define D940C_SPI1_CR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC000) //!< (SPI1) Control Register
#define D940C_SPI1_MR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC004) //!< (SPI1) Mode Register
// ========== Register definition for PDC_SPI1II peripheral ========== 
#define D940C_SPI1II_PTCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC120) //!< (PDC_SPI1II) PDC Transfer Control Register
#define D940C_SPI1II_TPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC108) //!< (PDC_SPI1II) Transmit Pointer Register
#define D940C_SPI1II_RNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC110) //!< (PDC_SPI1II) Receive Next Pointer Register
#define D940C_SPI1II_PTSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC124) //!< (PDC_SPI1II) PDC Transfer Status Register
#define D940C_SPI1II_TNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC11C) //!< (PDC_SPI1II) Transmit Next Counter Register
#define D940C_SPI1II_RCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC104) //!< (PDC_SPI1II) Receive Counter Register
#define D940C_SPI1II_RNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC114) //!< (PDC_SPI1II) Receive Next Counter Register
#define D940C_SPI1II_TCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC10C) //!< (PDC_SPI1II) Transmit Counter Register
#define D940C_SPI1II_RPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC100) //!< (PDC_SPI1II) Receive Pointer Register
#define D940C_SPI1II_TNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC118) //!< (PDC_SPI1II) Transmit Next Pointer Register
// ========== Register definition for SPI1II peripheral ========== 
#define D940C_SPI1II_IER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC014) //!< (SPI1II) Interrupt Enable Register
#define D940C_SPI1II_RDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC008) //!< (SPI1II) Receive Data Register
#define D940C_SPI1II_SR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC010) //!< (SPI1II) Status Register
#define D940C_SPI1II_IMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC01C) //!< (SPI1II) Interrupt Mask Register
#define D940C_SPI1II_TDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC00C) //!< (SPI1II) Transmit Data Register
#define D940C_SPI1II_IDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC018) //!< (SPI1II) Interrupt Disable Register
#define D940C_SPI1II_CSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC030) //!< (SPI1II) Chip Select Register
#define D940C_SPI1II_CR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC000) //!< (SPI1II) Control Register
#define D940C_SPI1II_MR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC004) //!< (SPI1II) Mode Register
// ========== Register definition for PDC_SPI1III peripheral ========== 
#define D940C_SPI1III_PTCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC120) //!< (PDC_SPI1III) PDC Transfer Control Register
#define D940C_SPI1III_TPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC108) //!< (PDC_SPI1III) Transmit Pointer Register
#define D940C_SPI1III_RNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC110) //!< (PDC_SPI1III) Receive Next Pointer Register
#define D940C_SPI1III_PTSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC124) //!< (PDC_SPI1III) PDC Transfer Status Register
#define D940C_SPI1III_TNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC11C) //!< (PDC_SPI1III) Transmit Next Counter Register
#define D940C_SPI1III_RCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC104) //!< (PDC_SPI1III) Receive Counter Register
#define D940C_SPI1III_RNCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC114) //!< (PDC_SPI1III) Receive Next Counter Register
#define D940C_SPI1III_TCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC10C) //!< (PDC_SPI1III) Transmit Counter Register
#define D940C_SPI1III_RPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC100) //!< (PDC_SPI1III) Receive Pointer Register
#define D940C_SPI1III_TNPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC118) //!< (PDC_SPI1III) Transmit Next Pointer Register
// ========== Register definition for SPI1III peripheral ========== 
#define D940C_SPI1III_IER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC014) //!< (SPI1III) Interrupt Enable Register
#define D940C_SPI1III_RDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC008) //!< (SPI1III) Receive Data Register
#define D940C_SPI1III_SR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC010) //!< (SPI1III) Status Register
#define D940C_SPI1III_IMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC01C) //!< (SPI1III) Interrupt Mask Register
#define D940C_SPI1III_TDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC00C) //!< (SPI1III) Transmit Data Register
#define D940C_SPI1III_IDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC018) //!< (SPI1III) Interrupt Disable Register
#define D940C_SPI1III_CSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC030) //!< (SPI1III) Chip Select Register
#define D940C_SPI1III_CR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC000) //!< (SPI1III) Control Register
#define D940C_SPI1III_MR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFCC004) //!< (SPI1III) Mode Register
// ========== Register definition for UHP peripheral ========== 
#define D940C_UHP_HcInterruptStatus ((D940_REG __MAGICV_EXTERNAL *) 	0x0050000C) //!< (UHP) Interrupt Status Register
#define D940C_UHP_HcCommandStatus ((D940_REG __MAGICV_EXTERNAL *) 	0x00500008) //!< (UHP) Command & status Register
#define D940C_UHP_HcRhStatus ((D940_REG __MAGICV_EXTERNAL *) 	0x00500050) //!< (UHP) Root Hub Status register
#define D940C_UHP_HcInterruptDisable ((D940_REG __MAGICV_EXTERNAL *) 	0x00500014) //!< (UHP) Interrupt Disable Register
#define D940C_UHP_HcPeriodicStart ((D940_REG __MAGICV_EXTERNAL *) 	0x00500040) //!< (UHP) Periodic Start
#define D940C_UHP_HcControlCurrentED ((D940_REG __MAGICV_EXTERNAL *) 	0x00500024) //!< (UHP) Endpoint Control and Status Register
#define D940C_UHP_HcPeriodCurrentED ((D940_REG __MAGICV_EXTERNAL *) 	0x0050001C) //!< (UHP) Current Isochronous or Interrupt Endpoint Descriptor
#define D940C_UHP_HcBulkHeadED ((D940_REG __MAGICV_EXTERNAL *) 	0x00500028) //!< (UHP) First endpoint register of the Bulk list
#define D940C_UHP_HcRevision ((D940_REG __MAGICV_EXTERNAL *) 	0x00500000) //!< (UHP) Revision
#define D940C_UHP_HcBulkCurrentED ((D940_REG __MAGICV_EXTERNAL *) 	0x0050002C) //!< (UHP) Current endpoint of the Bulk list
#define D940C_UHP_HcRhDescriptorB ((D940_REG __MAGICV_EXTERNAL *) 	0x0050004C) //!< (UHP) Root Hub characteristics B
#define D940C_UHP_HcControlHeadED ((D940_REG __MAGICV_EXTERNAL *) 	0x00500020) //!< (UHP) First Endpoint Descriptor of the Control list
#define D940C_UHP_HcFmRemaining ((D940_REG __MAGICV_EXTERNAL *) 	0x00500038) //!< (UHP) Bit time remaining in the current Frame
#define D940C_UHP_HcHCCA ((D940_REG __MAGICV_EXTERNAL *) 	0x00500018) //!< (UHP) Pointer to the Host Controller Communication Area
#define D940C_UHP_HcLSThreshold ((D940_REG __MAGICV_EXTERNAL *) 	0x00500044) //!< (UHP) LS Threshold
#define D940C_UHP_HcRhPortStatus ((D940_REG __MAGICV_EXTERNAL *) 	0x00500054) //!< (UHP) Root Hub Port Status Register
#define D940C_UHP_HcInterruptEnable ((D940_REG __MAGICV_EXTERNAL *) 	0x00500010) //!< (UHP) Interrupt Enable Register
#define D940C_UHP_HcFmNumber ((D940_REG __MAGICV_EXTERNAL *) 	0x0050003C) //!< (UHP) Frame number
#define D940C_UHP_HcFmInterval ((D940_REG __MAGICV_EXTERNAL *) 	0x00500034) //!< (UHP) Bit time between 2 consecutive SOFs
#define D940C_UHP_HcControl ((D940_REG __MAGICV_EXTERNAL *) 	0x00500004) //!< (UHP) Operating modes for the Host Controller
#define D940C_UHP_HcBulkDoneHead ((D940_REG __MAGICV_EXTERNAL *) 	0x00500030) //!< (UHP) Last completed transfer descriptor
#define D940C_UHP_HcRhDescriptorA ((D940_REG __MAGICV_EXTERNAL *) 	0x00500048) //!< (UHP) Root Hub characteristics A
// ========== Register definition for EMAC peripheral ========== 
#define D940C_EMAC_SA1H ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD809C) //!< (EMAC) Specific Address 1 High, Last 2 bytes
#define D940C_EMAC_SA4L ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD80B0) //!< (EMAC) Specific Address 4 Low, First 4 bytes
#define D940C_EMAC_SA3H ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD80AC) //!< (EMAC) Specific Address 3 High, Last 2 bytes
#define D940C_EMAC_IMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8030) //!< (EMAC) Interrupt Mask Register
#define D940C_EMAC_TUE  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8068) //!< (EMAC) Transmit Underrun Error Register
#define D940C_EMAC_HSL  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8094) //!< (EMAC) Hash Address Low[31:0]
#define D940C_EMAC_MAN  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8034) //!< (EMAC) PHY Maintenance Register
#define D940C_EMAC_USF  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8078) //!< (EMAC) Undersize Frame Register
#define D940C_EMAC_ALE  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8054) //!< (EMAC) Alignment Error Register
#define D940C_EMAC_CTL  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8000) //!< (EMAC) Network Control Register
#define D940C_EMAC_SR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8008) //!< (EMAC) Network Status Register
#define D940C_EMAC_ECOL ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8060) //!< (EMAC) Excessive Collision Register
#define D940C_EMAC_DRFC ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8080) //!< (EMAC) Discarded RX Frame Register
#define D940C_EMAC_TCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8010) //!< (EMAC) Transmit Control Register
#define D940C_EMAC_CSE  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8064) //!< (EMAC) Carrier Sense Error Register
#define D940C_EMAC_RJB  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8074) //!< (EMAC) Receive Jabber Register
#define D940C_EMAC_CDE  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD806C) //!< (EMAC) Code Error Register
#define D940C_EMAC_SEQE ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8050) //!< (EMAC) Frame Check Sequence Error Register
#define D940C_EMAC_SA4H ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD80B4) //!< (EMAC) Specific Address 4 High, Last 2 bytesr
#define D940C_EMAC_SCOL ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8044) //!< (EMAC) Single Collision Frame Register
#define D940C_EMAC_SQEE ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD807C) //!< (EMAC) SQE Test Error Register
#define D940C_EMAC_TSR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8014) //!< (EMAC) Transmit Status Register
#define D940C_EMAC_ELR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8070) //!< (EMAC) Excessive Length Error Register
#define D940C_EMAC_LCOL ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD805C) //!< (EMAC) Late Collision Register
#define D940C_EMAC_HSH  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8090) //!< (EMAC) Hash Address High[63:32]
#define D940C_EMAC_FRA  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8040) //!< (EMAC) Frames Transmitted OK Register
#define D940C_EMAC_RSR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8020) //!< (EMAC) Receive Status Register
#define D940C_EMAC_IER  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8028) //!< (EMAC) Interrupt Enable Register
#define D940C_EMAC_SA2H ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD80A4) //!< (EMAC) Specific Address 2 High, Last 2 bytes
#define D940C_EMAC_MCOL ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8048) //!< (EMAC) Multiple Collision Frame Register
#define D940C_EMAC_OK   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD804C) //!< (EMAC) Frames Received OK Register
#define D940C_EMAC_SA3L ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD80A8) //!< (EMAC) Specific Address 3 Low, First 4 bytes
#define D940C_EMAC_SA2L ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD80A0) //!< (EMAC) Specific Address 2 Low, First 4 bytes
#define D940C_EMAC_RBQP ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8018) //!< (EMAC) Receive Buffer Queue Pointer
#define D940C_EMAC_DTE  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8058) //!< (EMAC) Deferred Transmission Frame Register
#define D940C_EMAC_CFG  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8004) //!< (EMAC) Network Configuration Register
#define D940C_EMAC_TAR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD800C) //!< (EMAC) Transmit Address Register
#define D940C_EMAC_IDR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD802C) //!< (EMAC) Interrupt Disable Register
#define D940C_EMAC_SA1L ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8098) //!< (EMAC) Specific Address 1 Low, First 4 bytes
// ========== Register definition for EMACII peripheral ========== 
#define D940C_EMACII_SA1H ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD809C) //!< (EMACII) Specific Address 1 High, Last 2 bytes
#define D940C_EMACII_SA4L ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD80B0) //!< (EMACII) Specific Address 4 Low, First 4 bytes
#define D940C_EMACII_SA3H ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD80AC) //!< (EMACII) Specific Address 3 High, Last 2 bytes
#define D940C_EMACII_IMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8030) //!< (EMACII) Interrupt Mask Register
#define D940C_EMACII_TUE ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8068) //!< (EMACII) Transmit Underrun Error Register
#define D940C_EMACII_HSL ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8094) //!< (EMACII) Hash Address Low[31:0]
#define D940C_EMACII_MAN ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8034) //!< (EMACII) PHY Maintenance Register
#define D940C_EMACII_USF ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8078) //!< (EMACII) Undersize Frame Register
#define D940C_EMACII_ALE ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8054) //!< (EMACII) Alignment Error Register
#define D940C_EMACII_CTL ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8000) //!< (EMACII) Network Control Register
#define D940C_EMACII_SR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8008) //!< (EMACII) Network Status Register
#define D940C_EMACII_ECOL ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8060) //!< (EMACII) Excessive Collision Register
#define D940C_EMACII_ISR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8024) //!< (EMACII) Interrupt Status Register
#define D940C_EMACII_DRFC ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8080) //!< (EMACII) Discarded RX Frame Register
#define D940C_EMACII_TCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8010) //!< (EMACII) Transmit Control Register
#define D940C_EMACII_CSE ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8064) //!< (EMACII) Carrier Sense Error Register
#define D940C_EMACII_RJB ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8074) //!< (EMACII) Receive Jabber Register
#define D940C_EMACII_CDE ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD806C) //!< (EMACII) Code Error Register
#define D940C_EMACII_SEQE ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8050) //!< (EMACII) Frame Check Sequence Error Register
#define D940C_EMACII_SA4H ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD80B4) //!< (EMACII) Specific Address 4 High, Last 2 bytesr
#define D940C_EMACII_SCOL ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8044) //!< (EMACII) Single Collision Frame Register
#define D940C_EMACII_SQEE ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD807C) //!< (EMACII) SQE Test Error Register
#define D940C_EMACII_TSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8014) //!< (EMACII) Transmit Status Register
#define D940C_EMACII_ELR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8070) //!< (EMACII) Excessive Length Error Register
#define D940C_EMACII_LCOL ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD805C) //!< (EMACII) Late Collision Register
#define D940C_EMACII_HSH ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8090) //!< (EMACII) Hash Address High[63:32]
#define D940C_EMACII_FRA ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8040) //!< (EMACII) Frames Transmitted OK Register
#define D940C_EMACII_RSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8020) //!< (EMACII) Receive Status Register
#define D940C_EMACII_IER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8028) //!< (EMACII) Interrupt Enable Register
#define D940C_EMACII_SA2H ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD80A4) //!< (EMACII) Specific Address 2 High, Last 2 bytes
#define D940C_EMACII_MCOL ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8048) //!< (EMACII) Multiple Collision Frame Register
#define D940C_EMACII_OK ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD804C) //!< (EMACII) Frames Received OK Register
#define D940C_EMACII_SA3L ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD80A8) //!< (EMACII) Specific Address 3 Low, First 4 bytes
#define D940C_EMACII_SA2L ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD80A0) //!< (EMACII) Specific Address 2 Low, First 4 bytes
#define D940C_EMACII_RBQP ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8018) //!< (EMACII) Receive Buffer Queue Pointer
#define D940C_EMACII_DTE ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8058) //!< (EMACII) Deferred Transmission Frame Register
#define D940C_EMACII_CFG ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8004) //!< (EMACII) Network Configuration Register
#define D940C_EMACII_TAR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD800C) //!< (EMACII) Transmit Address Register
#define D940C_EMACII_IDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD802C) //!< (EMACII) Interrupt Disable Register
#define D940C_EMACII_SA1L ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8098) //!< (EMACII) Specific Address 1 Low, First 4 bytes
// ========== Register definition for EMACIII peripheral ========== 
#define D940C_EMACIII_SA1H ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD809C) //!< (EMACIII) Specific Address 1 High, Last 2 bytes
#define D940C_EMACIII_SA4L ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD80B0) //!< (EMACIII) Specific Address 4 Low, First 4 bytes
#define D940C_EMACIII_SA3H ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD80AC) //!< (EMACIII) Specific Address 3 High, Last 2 bytes
#define D940C_EMACIII_IMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8030) //!< (EMACIII) Interrupt Mask Register
#define D940C_EMACIII_TUE ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8068) //!< (EMACIII) Transmit Underrun Error Register
#define D940C_EMACIII_HSL ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8094) //!< (EMACIII) Hash Address Low[31:0]
#define D940C_EMACIII_MAN ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8034) //!< (EMACIII) PHY Maintenance Register
#define D940C_EMACIII_USF ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8078) //!< (EMACIII) Undersize Frame Register
#define D940C_EMACIII_ALE ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8054) //!< (EMACIII) Alignment Error Register
#define D940C_EMACIII_CTL ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8000) //!< (EMACIII) Network Control Register
#define D940C_EMACIII_SR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8008) //!< (EMACIII) Network Status Register
#define D940C_EMACIII_ECOL ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8060) //!< (EMACIII) Excessive Collision Register
#define D940C_EMACIII_ISR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8024) //!< (EMACIII) Interrupt Status Register
#define D940C_EMACIII_DRFC ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8080) //!< (EMACIII) Discarded RX Frame Register
#define D940C_EMACIII_TCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8010) //!< (EMACIII) Transmit Control Register
#define D940C_EMACIII_CSE ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8064) //!< (EMACIII) Carrier Sense Error Register
#define D940C_EMACIII_RJB ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8074) //!< (EMACIII) Receive Jabber Register
#define D940C_EMACIII_CDE ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD806C) //!< (EMACIII) Code Error Register
#define D940C_EMACIII_SEQE ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8050) //!< (EMACIII) Frame Check Sequence Error Register
#define D940C_EMACIII_SA4H ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD80B4) //!< (EMACIII) Specific Address 4 High, Last 2 bytesr
#define D940C_EMACIII_SCOL ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8044) //!< (EMACIII) Single Collision Frame Register
#define D940C_EMACIII_SQEE ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD807C) //!< (EMACIII) SQE Test Error Register
#define D940C_EMACIII_TSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8014) //!< (EMACIII) Transmit Status Register
#define D940C_EMACIII_ELR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8070) //!< (EMACIII) Excessive Length Error Register
#define D940C_EMACIII_LCOL ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD805C) //!< (EMACIII) Late Collision Register
#define D940C_EMACIII_HSH ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8090) //!< (EMACIII) Hash Address High[63:32]
#define D940C_EMACIII_FRA ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8040) //!< (EMACIII) Frames Transmitted OK Register
#define D940C_EMACIII_RSR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8020) //!< (EMACIII) Receive Status Register
#define D940C_EMACIII_IER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8028) //!< (EMACIII) Interrupt Enable Register
#define D940C_EMACIII_SA2H ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD80A4) //!< (EMACIII) Specific Address 2 High, Last 2 bytes
#define D940C_EMACIII_MCOL ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8048) //!< (EMACIII) Multiple Collision Frame Register
#define D940C_EMACIII_OK ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD804C) //!< (EMACIII) Frames Received OK Register
#define D940C_EMACIII_SA3L ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD80A8) //!< (EMACIII) Specific Address 3 Low, First 4 bytes
#define D940C_EMACIII_SA2L ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD80A0) //!< (EMACIII) Specific Address 2 Low, First 4 bytes
#define D940C_EMACIII_RBQP ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8018) //!< (EMACIII) Receive Buffer Queue Pointer
#define D940C_EMACIII_DTE ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8058) //!< (EMACIII) Deferred Transmission Frame Register
#define D940C_EMACIII_CFG ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8004) //!< (EMACIII) Network Configuration Register
#define D940C_EMACIII_TAR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD800C) //!< (EMACIII) Transmit Address Register
#define D940C_EMACIII_IDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD802C) //!< (EMACIII) Interrupt Disable Register
#define D940C_EMACIII_SA1L ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFD8098) //!< (EMACIII) Specific Address 1 Low, First 4 bytes
// ========== Register definition for CAN0_MB0 peripheral ========== 
#define D940C_MB0_MID   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC208) //!< (CAN0_MB0) MailBox ID Register
#define D940C_MB0_MMR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC200) //!< (CAN0_MB0) MailBox Mode Register
#define D940C_MB0_MDH   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC218) //!< (CAN0_MB0) MailBox Data High Register
#define D940C_MB0_MFID  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC20C) //!< (CAN0_MB0) MailBox Family ID Register
#define D940C_MB0_MDL   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC214) //!< (CAN0_MB0) MailBox Data Low Register
#define D940C_MB0_MSR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC210) //!< (CAN0_MB0) MailBox Status Register
#define D940C_MB0_MCR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC21C) //!< (CAN0_MB0) MailBox Control Register
#define D940C_MB0_MAM   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC204) //!< (CAN0_MB0) MailBox Acceptance Mask Register
// ========== Register definition for CAN0_MB1 peripheral ========== 
#define D940C_MB1_MID   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC228) //!< (CAN0_MB1) MailBox ID Register
#define D940C_MB1_MMR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC220) //!< (CAN0_MB1) MailBox Mode Register
#define D940C_MB1_MCR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC23C) //!< (CAN0_MB1) MailBox Control Register
#define D940C_MB1_MDL   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC234) //!< (CAN0_MB1) MailBox Data Low Register
#define D940C_MB1_MFID  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC22C) //!< (CAN0_MB1) MailBox Family ID Register
#define D940C_MB1_MDH   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC238) //!< (CAN0_MB1) MailBox Data High Register
#define D940C_MB1_MAM   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC224) //!< (CAN0_MB1) MailBox Acceptance Mask Register
#define D940C_MB1_MSR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC230) //!< (CAN0_MB1) MailBox Status Register
// ========== Register definition for CAN0_MB2 peripheral ========== 
#define D940C_MB2_MCR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC25C) //!< (CAN0_MB2) MailBox Control Register
#define D940C_MB2_MDL   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC254) //!< (CAN0_MB2) MailBox Data Low Register
#define D940C_MB2_MDH   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC258) //!< (CAN0_MB2) MailBox Data High Register
#define D940C_MB2_MFID  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC24C) //!< (CAN0_MB2) MailBox Family ID Register
#define D940C_MB2_MSR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC250) //!< (CAN0_MB2) MailBox Status Register
#define D940C_MB2_MID   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC248) //!< (CAN0_MB2) MailBox ID Register
#define D940C_MB2_MMR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC240) //!< (CAN0_MB2) MailBox Mode Register
#define D940C_MB2_MAM   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC244) //!< (CAN0_MB2) MailBox Acceptance Mask Register
// ========== Register definition for CAN0_MB3 peripheral ========== 
#define D940C_MB3_MMR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC260) //!< (CAN0_MB3) MailBox Mode Register
#define D940C_MB3_MDH   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC278) //!< (CAN0_MB3) MailBox Data High Register
#define D940C_MB3_MDL   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC274) //!< (CAN0_MB3) MailBox Data Low Register
#define D940C_MB3_MID   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC268) //!< (CAN0_MB3) MailBox ID Register
#define D940C_MB3_MAM   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC264) //!< (CAN0_MB3) MailBox Acceptance Mask Register
#define D940C_MB3_MCR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC27C) //!< (CAN0_MB3) MailBox Control Register
#define D940C_MB3_MSR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC270) //!< (CAN0_MB3) MailBox Status Register
#define D940C_MB3_MFID  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC26C) //!< (CAN0_MB3) MailBox Family ID Register
// ========== Register definition for CAN0_MB4 peripheral ========== 
#define D940C_MB4_MDH   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC298) //!< (CAN0_MB4) MailBox Data High Register
#define D940C_MB4_MCR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC29C) //!< (CAN0_MB4) MailBox Control Register
#define D940C_MB4_MMR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC280) //!< (CAN0_MB4) MailBox Mode Register
#define D940C_MB4_MDL   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC294) //!< (CAN0_MB4) MailBox Data Low Register
#define D940C_MB4_MFID  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC28C) //!< (CAN0_MB4) MailBox Family ID Register
#define D940C_MB4_MAM   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC284) //!< (CAN0_MB4) MailBox Acceptance Mask Register
#define D940C_MB4_MID   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC288) //!< (CAN0_MB4) MailBox ID Register
#define D940C_MB4_MSR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC290) //!< (CAN0_MB4) MailBox Status Register
// ========== Register definition for CAN0_MB5 peripheral ========== 
#define D940C_MB5_MDH   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC2B8) //!< (CAN0_MB5) MailBox Data High Register
#define D940C_MB5_MDL   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC2B4) //!< (CAN0_MB5) MailBox Data Low Register
#define D940C_MB5_MFID  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC2AC) //!< (CAN0_MB5) MailBox Family ID Register
#define D940C_MB5_MAM   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC2A4) //!< (CAN0_MB5) MailBox Acceptance Mask Register
#define D940C_MB5_MID   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC2A8) //!< (CAN0_MB5) MailBox ID Register
#define D940C_MB5_MCR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC2BC) //!< (CAN0_MB5) MailBox Control Register
#define D940C_MB5_MMR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC2A0) //!< (CAN0_MB5) MailBox Mode Register
#define D940C_MB5_MSR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC2B0) //!< (CAN0_MB5) MailBox Status Register
// ========== Register definition for CAN0_MB6 peripheral ========== 
#define D940C_MB6_MMR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC2C0) //!< (CAN0_MB6) MailBox Mode Register
#define D940C_MB6_MDH   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC2D8) //!< (CAN0_MB6) MailBox Data High Register
#define D940C_MB6_MAM   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC2C4) //!< (CAN0_MB6) MailBox Acceptance Mask Register
#define D940C_MB6_MCR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC2DC) //!< (CAN0_MB6) MailBox Control Register
#define D940C_MB6_MID   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC2C8) //!< (CAN0_MB6) MailBox ID Register
#define D940C_MB6_MDL   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC2D4) //!< (CAN0_MB6) MailBox Data Low Register
#define D940C_MB6_MFID  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC2CC) //!< (CAN0_MB6) MailBox Family ID Register
#define D940C_MB6_MSR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC2D0) //!< (CAN0_MB6) MailBox Status Register
// ========== Register definition for CAN0_MB7 peripheral ========== 
#define D940C_MB7_MSR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC2F0) //!< (CAN0_MB7) MailBox Status Register
#define D940C_MB7_MID   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC2E8) //!< (CAN0_MB7) MailBox ID Register
#define D940C_MB7_MAM   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC2E4) //!< (CAN0_MB7) MailBox Acceptance Mask Register
#define D940C_MB7_MMR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC2E0) //!< (CAN0_MB7) MailBox Mode Register
#define D940C_MB7_MCR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC2FC) //!< (CAN0_MB7) MailBox Control Register
#define D940C_MB7_MDH   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC2F8) //!< (CAN0_MB7) MailBox Data High Register
#define D940C_MB7_MDL   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC2F4) //!< (CAN0_MB7) MailBox Data Low Register
#define D940C_MB7_MFID  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC2EC) //!< (CAN0_MB7) MailBox Family ID Register
// ========== Register definition for CAN0_MB8 peripheral ========== 
#define D940C_MB8_MCR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC31C) //!< (CAN0_MB8) MailBox Control Register
#define D940C_MB8_MDL   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC314) //!< (CAN0_MB8) MailBox Data Low Register
#define D940C_MB8_MDH   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC318) //!< (CAN0_MB8) MailBox Data High Register
#define D940C_MB8_MSR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC310) //!< (CAN0_MB8) MailBox Status Register
#define D940C_MB8_MFID  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC30C) //!< (CAN0_MB8) MailBox Family ID Register
#define D940C_MB8_MMR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC300) //!< (CAN0_MB8) MailBox Mode Register
#define D940C_MB8_MAM   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC304) //!< (CAN0_MB8) MailBox Acceptance Mask Register
#define D940C_MB8_MID   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC308) //!< (CAN0_MB8) MailBox ID Register
// ========== Register definition for CAN0_MB9 peripheral ========== 
#define D940C_MB9_MDH   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC338) //!< (CAN0_MB9) MailBox Data High Register
#define D940C_MB9_MDL   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC334) //!< (CAN0_MB9) MailBox Data Low Register
#define D940C_MB9_MID   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC328) //!< (CAN0_MB9) MailBox ID Register
#define D940C_MB9_MMR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC320) //!< (CAN0_MB9) MailBox Mode Register
#define D940C_MB9_MCR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC33C) //!< (CAN0_MB9) MailBox Control Register
#define D940C_MB9_MFID  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC32C) //!< (CAN0_MB9) MailBox Family ID Register
#define D940C_MB9_MSR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC330) //!< (CAN0_MB9) MailBox Status Register
#define D940C_MB9_MAM   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC324) //!< (CAN0_MB9) MailBox Acceptance Mask Register
// ========== Register definition for CAN0_MB10 peripheral ========== 
#define D940C_MB10_MDH  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC358) //!< (CAN0_MB10) MailBox Data High Register
#define D940C_MB10_MID  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC348) //!< (CAN0_MB10) MailBox ID Register
#define D940C_MB10_MSR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC350) //!< (CAN0_MB10) MailBox Status Register
#define D940C_MB10_MMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC340) //!< (CAN0_MB10) MailBox Mode Register
#define D940C_MB10_MCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC35C) //!< (CAN0_MB10) MailBox Control Register
#define D940C_MB10_MDL  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC354) //!< (CAN0_MB10) MailBox Data Low Register
#define D940C_MB10_MFID ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC34C) //!< (CAN0_MB10) MailBox Family ID Register
#define D940C_MB10_MAM  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC344) //!< (CAN0_MB10) MailBox Acceptance Mask Register
// ========== Register definition for CAN0_MB11 peripheral ========== 
#define D940C_MB11_MMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC360) //!< (CAN0_MB11) MailBox Mode Register
#define D940C_MB11_MDL  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC374) //!< (CAN0_MB11) MailBox Data Low Register
#define D940C_MB11_MDH  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC378) //!< (CAN0_MB11) MailBox Data High Register
#define D940C_MB11_MCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC37C) //!< (CAN0_MB11) MailBox Control Register
#define D940C_MB11_MAM  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC364) //!< (CAN0_MB11) MailBox Acceptance Mask Register
#define D940C_MB11_MFID ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC36C) //!< (CAN0_MB11) MailBox Family ID Register
#define D940C_MB11_MSR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC370) //!< (CAN0_MB11) MailBox Status Register
#define D940C_MB11_MID  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC368) //!< (CAN0_MB11) MailBox ID Register
// ========== Register definition for CAN0_MB12 peripheral ========== 
#define D940C_MB12_MCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC39C) //!< (CAN0_MB12) MailBox Control Register
#define D940C_MB12_MAM  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC384) //!< (CAN0_MB12) MailBox Acceptance Mask Register
#define D940C_MB12_MSR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC390) //!< (CAN0_MB12) MailBox Status Register
#define D940C_MB12_MMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC380) //!< (CAN0_MB12) MailBox Mode Register
#define D940C_MB12_MDL  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC394) //!< (CAN0_MB12) MailBox Data Low Register
#define D940C_MB12_MID  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC388) //!< (CAN0_MB12) MailBox ID Register
#define D940C_MB12_MDH  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC398) //!< (CAN0_MB12) MailBox Data High Register
#define D940C_MB12_MFID ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC38C) //!< (CAN0_MB12) MailBox Family ID Register
// ========== Register definition for CAN0_MB13 peripheral ========== 
#define D940C_MB13_MCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC3BC) //!< (CAN0_MB13) MailBox Control Register
#define D940C_MB13_MAM  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC3A4) //!< (CAN0_MB13) MailBox Acceptance Mask Register
#define D940C_MB13_MMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC3A0) //!< (CAN0_MB13) MailBox Mode Register
#define D940C_MB13_MID  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC3A8) //!< (CAN0_MB13) MailBox ID Register
#define D940C_MB13_MFID ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC3AC) //!< (CAN0_MB13) MailBox Family ID Register
#define D940C_MB13_MDH  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC3B8) //!< (CAN0_MB13) MailBox Data High Register
#define D940C_MB13_MSR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC3B0) //!< (CAN0_MB13) MailBox Status Register
#define D940C_MB13_MDL  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC3B4) //!< (CAN0_MB13) MailBox Data Low Register
// ========== Register definition for CAN0_MB14 peripheral ========== 
#define D940C_MB14_MFID ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC3CC) //!< (CAN0_MB14) MailBox Family ID Register
#define D940C_MB14_MSR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC3D0) //!< (CAN0_MB14) MailBox Status Register
#define D940C_MB14_MDH  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC3D8) //!< (CAN0_MB14) MailBox Data High Register
#define D940C_MB14_MCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC3DC) //!< (CAN0_MB14) MailBox Control Register
#define D940C_MB14_MAM  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC3C4) //!< (CAN0_MB14) MailBox Acceptance Mask Register
#define D940C_MB14_MDL  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC3D4) //!< (CAN0_MB14) MailBox Data Low Register
#define D940C_MB14_MID  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC3C8) //!< (CAN0_MB14) MailBox ID Register
#define D940C_MB14_MMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC3C0) //!< (CAN0_MB14) MailBox Mode Register
// ========== Register definition for CAN0_MB15 peripheral ========== 
#define D940C_MB15_MSR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC3F0) //!< (CAN0_MB15) MailBox Status Register
#define D940C_MB15_MMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC3E0) //!< (CAN0_MB15) MailBox Mode Register
#define D940C_MB15_MCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC3FC) //!< (CAN0_MB15) MailBox Control Register
#define D940C_MB15_MAM  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC3E4) //!< (CAN0_MB15) MailBox Acceptance Mask Register
#define D940C_MB15_MID  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC3E8) //!< (CAN0_MB15) MailBox ID Register
#define D940C_MB15_MDL  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC3F4) //!< (CAN0_MB15) MailBox Data Low Register
#define D940C_MB15_MDH  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC3F8) //!< (CAN0_MB15) MailBox Data High Register
#define D940C_MB15_MFID ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC3EC) //!< (CAN0_MB15) MailBox Family ID Register
// ========== Register definition for CAN0 peripheral ========== 
#define D940C_CAN0_VR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC0FC) //!< (CAN0) Version Register
#define D940C_CAN0_TCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC024) //!< (CAN0) Transfer Command Register
#define D940C_CAN0_TIM  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC018) //!< (CAN0) Timer Register
#define D940C_CAN0_IER  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC004) //!< (CAN0) Interrupt Enable Register
#define D940C_CAN0_TIMESTP ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC01C) //!< (CAN0) Time Stamp Register
#define D940C_CAN0_IDR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC008) //!< (CAN0) Interrupt Disable Register
#define D940C_CAN0_IMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC00C) //!< (CAN0) Interrupt Mask Register
#define D940C_CAN0_MR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC000) //!< (CAN0) Mode Register
#define D940C_CAN0_ECR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC020) //!< (CAN0) Error Counter Register
#define D940C_CAN0_ACR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC028) //!< (CAN0) Abort Command Register
#define D940C_CAN0_SR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC010) //!< (CAN0) Status Register
#define D940C_CAN0_BR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC014) //!< (CAN0) Baudrate Register
// ========== Register definition for CAN0II peripheral ========== 
#define D940C_CAN0II_VR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC0FC) //!< (CAN0II) Version Register
#define D940C_CAN0II_TCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC024) //!< (CAN0II) Transfer Command Register
#define D940C_CAN0II_TIM ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC018) //!< (CAN0II) Timer Register
#define D940C_CAN0II_IER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC004) //!< (CAN0II) Interrupt Enable Register
#define D940C_CAN0II_TIMESTP ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC01C) //!< (CAN0II) Time Stamp Register
#define D940C_CAN0II_IDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC008) //!< (CAN0II) Interrupt Disable Register
#define D940C_CAN0II_IMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC00C) //!< (CAN0II) Interrupt Mask Register
#define D940C_CAN0II_MR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC000) //!< (CAN0II) Mode Register
#define D940C_CAN0II_ECR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC020) //!< (CAN0II) Error Counter Register
#define D940C_CAN0II_ACR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC028) //!< (CAN0II) Abort Command Register
#define D940C_CAN0II_SR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC010) //!< (CAN0II) Status Register
#define D940C_CAN0II_BR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC014) //!< (CAN0II) Baudrate Register
// ========== Register definition for CAN0III peripheral ========== 
#define D940C_CAN0III_VR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC0FC) //!< (CAN0III) Version Register
#define D940C_CAN0III_TCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC024) //!< (CAN0III) Transfer Command Register
#define D940C_CAN0III_TIM ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC018) //!< (CAN0III) Timer Register
#define D940C_CAN0III_IER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC004) //!< (CAN0III) Interrupt Enable Register
#define D940C_CAN0III_TIMESTP ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC01C) //!< (CAN0III) Time Stamp Register
#define D940C_CAN0III_IDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC008) //!< (CAN0III) Interrupt Disable Register
#define D940C_CAN0III_IMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC00C) //!< (CAN0III) Interrupt Mask Register
#define D940C_CAN0III_MR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC000) //!< (CAN0III) Mode Register
#define D940C_CAN0III_ECR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC020) //!< (CAN0III) Error Counter Register
#define D940C_CAN0III_ACR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC028) //!< (CAN0III) Abort Command Register
#define D940C_CAN0III_SR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC010) //!< (CAN0III) Status Register
#define D940C_CAN0III_BR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFDC014) //!< (CAN0III) Baudrate Register
// ========== Register definition for CAN1_MB0 peripheral ========== 
// ========== Register definition for CAN1_MB1 peripheral ========== 
// ========== Register definition for CAN1_MB2 peripheral ========== 
// ========== Register definition for CAN1_MB3 peripheral ========== 
// ========== Register definition for CAN1_MB4 peripheral ========== 
// ========== Register definition for CAN1_MB5 peripheral ========== 
// ========== Register definition for CAN1_MB6 peripheral ========== 
// ========== Register definition for CAN1_MB7 peripheral ========== 
// ========== Register definition for CAN1_MB8 peripheral ========== 
// ========== Register definition for CAN1_MB9 peripheral ========== 
// ========== Register definition for CAN1_MB10 peripheral ========== 
// ========== Register definition for CAN1_MB11 peripheral ========== 
// ========== Register definition for CAN1_MB12 peripheral ========== 
// ========== Register definition for CAN1_MB13 peripheral ========== 
// ========== Register definition for CAN1_MB14 peripheral ========== 
// ========== Register definition for CAN1_MB15 peripheral ========== 
// ========== Register definition for CAN1 peripheral ========== 
#define D940C_CAN1_BR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE0014) //!< (CAN1) Baudrate Register
#define D940C_CAN1_SR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE0010) //!< (CAN1) Status Register
#define D940C_CAN1_TIM  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE0018) //!< (CAN1) Timer Register
#define D940C_CAN1_MR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE0000) //!< (CAN1) Mode Register
#define D940C_CAN1_IER  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE0004) //!< (CAN1) Interrupt Enable Register
#define D940C_CAN1_TIMESTP ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE001C) //!< (CAN1) Time Stamp Register
#define D940C_CAN1_IMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE000C) //!< (CAN1) Interrupt Mask Register
#define D940C_CAN1_IDR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE0008) //!< (CAN1) Interrupt Disable Register
#define D940C_CAN1_ECR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE0020) //!< (CAN1) Error Counter Register
#define D940C_CAN1_TCR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE0024) //!< (CAN1) Transfer Command Register
#define D940C_CAN1_ACR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE0028) //!< (CAN1) Abort Command Register
#define D940C_CAN1_VR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE00FC) //!< (CAN1) Version Register
// ========== Register definition for CAN1II peripheral ========== 
#define D940C_CAN1II_BR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE0014) //!< (CAN1II) Baudrate Register
#define D940C_CAN1II_SR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE0010) //!< (CAN1II) Status Register
#define D940C_CAN1II_TIM ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE0018) //!< (CAN1II) Timer Register
#define D940C_CAN1II_MR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE0000) //!< (CAN1II) Mode Register
#define D940C_CAN1II_IER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE0004) //!< (CAN1II) Interrupt Enable Register
#define D940C_CAN1II_TIMESTP ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE001C) //!< (CAN1II) Time Stamp Register
#define D940C_CAN1II_IMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE000C) //!< (CAN1II) Interrupt Mask Register
#define D940C_CAN1II_IDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE0008) //!< (CAN1II) Interrupt Disable Register
#define D940C_CAN1II_ECR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE0020) //!< (CAN1II) Error Counter Register
#define D940C_CAN1II_TCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE0024) //!< (CAN1II) Transfer Command Register
#define D940C_CAN1II_ACR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE0028) //!< (CAN1II) Abort Command Register
#define D940C_CAN1II_VR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE00FC) //!< (CAN1II) Version Register
// ========== Register definition for CAN1III peripheral ========== 
#define D940C_CAN1III_BR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE0014) //!< (CAN1III) Baudrate Register
#define D940C_CAN1III_SR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE0010) //!< (CAN1III) Status Register
#define D940C_CAN1III_TIM ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE0018) //!< (CAN1III) Timer Register
#define D940C_CAN1III_MR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE0000) //!< (CAN1III) Mode Register
#define D940C_CAN1III_IER ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE0004) //!< (CAN1III) Interrupt Enable Register
#define D940C_CAN1III_TIMESTP ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE001C) //!< (CAN1III) Time Stamp Register
#define D940C_CAN1III_IMR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE000C) //!< (CAN1III) Interrupt Mask Register
#define D940C_CAN1III_IDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE0008) //!< (CAN1III) Interrupt Disable Register
#define D940C_CAN1III_ECR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE0020) //!< (CAN1III) Error Counter Register
#define D940C_CAN1III_TCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE0024) //!< (CAN1III) Transfer Command Register
#define D940C_CAN1III_ACR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE0028) //!< (CAN1III) Abort Command Register
#define D940C_CAN1III_VR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFE00FC) //!< (CAN1III) Version Register
// ========== Register definition for MCI0 peripheral ========== 
#define D940C_MCI0_RDR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA8030) //!< (MCI0) MCI Receive Data Register
#define D940C_MCI0_CMDR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA8014) //!< (MCI0) MCI Command Register
#define D940C_MCI0_IDR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA8048) //!< (MCI0) MCI Interrupt Disable Register
#define D940C_MCI0_DTOR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA8008) //!< (MCI0) MCI Data Timeout Register
#define D940C_MCI0_TDR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA8034) //!< (MCI0) MCI Transmit Data Register
#define D940C_MCI0_IER  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA8044) //!< (MCI0) MCI Interrupt Enable Register
#define D940C_MCI0_MR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA8004) //!< (MCI0) MCI Mode Register
#define D940C_MCI0_IMR  ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA804C) //!< (MCI0) MCI Interrupt Mask Register
#define D940C_MCI0_CR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA8000) //!< (MCI0) MCI Control Register
#define D940C_MCI0_ARGR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA8010) //!< (MCI0) MCI Argument Register
#define D940C_MCI0_SDCR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA800C) //!< (MCI0) MCI SD Card Register
#define D940C_MCI0_SR   ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA8040) //!< (MCI0) MCI Status Register
#define D940C_MCI0_RSPR ((D940_REG __MAGICV_EXTERNAL *) 	0xFFFA8020) //!< (MCI0) MCI Response Register

// *****************************************************************************
//               PIO DEFINITIONS FOR D940HF
// *****************************************************************************
#define D940C_PIO_PA0        ((D940_REG) 1 <<  0) //!< Pin Controlled by PA0
#define D940C_PA0_MISO0    ((D940_REG) D940C_PIO_PA0) //!<  SPI0 Master In Slave (bi-directional)
#define D940C_PIO_PA1        ((D940_REG) 1 <<  1) //!< Pin Controlled by PA1
#define D940C_PA1_MOSI0    ((D940_REG) D940C_PIO_PA1) //!<  SPI0 Master Out Slave (bi-directional)
#define D940C_PA1_CFCE1    ((D940_REG) D940C_PIO_PA1) //!<  CompactFlash Chip Enable 1 (III)
#define D940C_PIO_PA10       ((D940_REG) 1 << 10) //!< Pin Controlled by PA10
#define D940C_PA10_RTS0     ((D940_REG) D940C_PIO_PA10) //!<  USART0 Ready To Send
#define D940C_PA10_TCLK1    ((D940_REG) D940C_PIO_PA10) //!<  Timer counter 1 external input
#define D940C_PIO_PA11       ((D940_REG) 1 << 11) //!< Pin Controlled by PA11
#define D940C_PA11_SCK0     ((D940_REG) D940C_PIO_PA11) //!<  USART0 Clock
#define D940C_PA11_NPCS02   ((D940_REG) D940C_PIO_PA11) //!<  SPI0 Peripheral Chip Select 2 (III)
#define D940C_PIO_PA12       ((D940_REG) 1 << 12) //!< Pin Controlled by PA12
#define D940C_PA12_IRQ1     ((D940_REG) D940C_PIO_PA12) //!<  AIC external interrupt 1, also to mAgicV M_INT1
#define D940C_PA12_RTS0     ((D940_REG) D940C_PIO_PA12) //!<  USART0 Ready To Send (III)
#define D940C_PIO_PA13       ((D940_REG) 1 << 13) //!< Pin Controlled by PA13
#define D940C_PA13_EMAC_MDIO ((D940_REG) D940C_PIO_PA13) //!<  ETH MDIO
#define D940C_PIO_PA14       ((D940_REG) 1 << 14) //!< Pin Controlled by PA14
#define D940C_PA14_EMAC_MDC ((D940_REG) D940C_PIO_PA14) //!<  ETH MDC
#define D940C_PA14_IRQ2     ((D940_REG) D940C_PIO_PA14) //!<  AIC external interrupt 2
#define D940C_PIO_PA15       ((D940_REG) 1 << 15) //!< Pin Controlled by PA15
#define D940C_PA15_EMAC_FCE100 ((D940_REG) D940C_PIO_PA15) //!<  ETH FCE100
#define D940C_PA15_TCLK2    ((D940_REG) D940C_PIO_PA15) //!<  Timer Counter 2 external input
#define D940C_PIO_PA16       ((D940_REG) 1 << 16) //!< Pin Controlled by PA16
#define D940C_PA16_EMAC_EREFCK ((D940_REG) D940C_PIO_PA16) //!<  EREFCK Pin
#define D940C_PA16_PCK0     ((D940_REG) D940C_PIO_PA16) //!<  PMC Programmable clock output 0
#define D940C_PIO_PA17       ((D940_REG) 1 << 17) //!< Pin Controlled by PA17
#define D940C_PA17_EMAC_ECRSDV ((D940_REG) D940C_PIO_PA17) //!<  EMAC ECRSDV
#define D940C_PA17_NCS4_CFCS0 ((D940_REG) D940C_PIO_PA17) //!<  Chip Select 4 / CompactFlash Chip Select 0 (III)
#define D940C_PIO_PA18       ((D940_REG) 1 << 18) //!< Pin Controlled by PA18
#define D940C_PA18_EMAC_ERX0 ((D940_REG) D940C_PIO_PA18) //!<  EMACB ERX0
#define D940C_PA18_NCS4_CFCS1 ((D940_REG) D940C_PIO_PA18) //!<  Chip Select 4 / CompactFlash Chip Select 1 (III)
#define D940C_PIO_PA19       ((D940_REG) 1 << 19) //!< Pin Controlled by PA19
#define D940C_PA19_EMAC_ERX1 ((D940_REG) D940C_PIO_PA19) //!<  EMACB ERX1
#define D940C_PA19_NCS6     ((D940_REG) D940C_PIO_PA19) //!<  Chip Select 6 (III)
#define D940C_PIO_PA2        ((D940_REG) 1 <<  2) //!< Pin Controlled by PA2
#define D940C_PA2_CLK      ((D940_REG) D940C_PIO_PA2) //!<  SPI0 Serial Clock
#define D940C_PA2_CFCE2    ((D940_REG) D940C_PIO_PA2) //!<  CompactFlash Chip Enable 2 (III)
#define D940C_PIO_PA20       ((D940_REG) 1 << 20) //!< Pin Controlled by PA20
#define D940C_PA20_EMAC_ERXER ((D940_REG) D940C_PIO_PA20) //!<  EMACB ERXER
#define D940C_PA20_NCS7     ((D940_REG) D940C_PIO_PA20) //!<  Chip Select 7 (III)
#define D940C_PIO_PA21       ((D940_REG) 1 << 21) //!< Pin Controlled by PA21
#define D940C_PA21_EMAC_ETX0 ((D940_REG) D940C_PIO_PA21) //!<  EMACB ETX0
#define D940C_PA21_m_ck     ((D940_REG) D940C_PIO_PA21) //!<  mAgicV clock
#define D940C_PIO_PA22       ((D940_REG) 1 << 22) //!< Pin Controlled by PA22
#define D940C_PA22_EMAC_ETX1 ((D940_REG) D940C_PIO_PA22) //!<  EMACB ETX1
#define D940C_PA22_a_ck     ((D940_REG) D940C_PIO_PA22) //!<  ARM clock
#define D940C_PIO_PA23       ((D940_REG) 1 << 23) //!< Pin Controlled by PA23
#define D940C_PA23_EMAC_ETXEN ((D940_REG) D940C_PIO_PA23) //!<  EMACB ETXEN
#define D940C_PIO_PA24       ((D940_REG) 1 << 24) //!< Pin Controlled by PA24
#define D940C_PA24_BMS      ((D940_REG) D940C_PIO_PA24) //!<  Boot memory select
#define D940C_PIO_PA25       ((D940_REG) 1 << 25) //!< Pin Controlled by PA25
#define D940C_PA25_EBI_NWAIT ((D940_REG) D940C_PIO_PA25) //!<  EBI NWAIT
#define D940C_PA25_RTS2     ((D940_REG) D940C_PIO_PA25) //!<  USART2 Ready to send
#define D940C_PIO_PA26       ((D940_REG) 1 << 26) //!< Pin Controlled by PA26
#define D940C_PA26_NCS4_CFCS0 ((D940_REG) D940C_PIO_PA26) //!<  Chip Select 4 / CompactFlash Chip Select 0
#define D940C_PA26_TIOA2    ((D940_REG) D940C_PIO_PA26) //!<  Timer Counter 2 Pin A
#define D940C_PIO_PA27       ((D940_REG) 1 << 27) //!< Pin Controlled by PA27
#define D940C_PA27_NCS5_CFCS1 ((D940_REG) D940C_PIO_PA27) //!<  Chip Select 5 / CompactFlash Chip Select 1
#define D940C_PA27_PCK2     ((D940_REG) D940C_PIO_PA27) //!<  PMC Programmable clock Output 2
#define D940C_PIO_PA28       ((D940_REG) 1 << 28) //!< Pin Controlled by PA28
#define D940C_PA28_NCS6     ((D940_REG) D940C_PIO_PA28) //!<  Chip Select 6
#define D940C_PA28_SMOE     ((D940_REG) D940C_PIO_PA28) //!<  SmartMedia Output Enable
#define D940C_PIO_PA29       ((D940_REG) 1 << 29) //!< Pin Controlled by PA29
#define D940C_PA29_NCS7     ((D940_REG) D940C_PIO_PA29) //!<  Chip Select 7
#define D940C_PA29_SMWE     ((D940_REG) D940C_PIO_PA29) //!<  SmartMedia Write Enable
#define D940C_PIO_PA3        ((D940_REG) 1 <<  3) //!< Pin Controlled by PA3
#define D940C_PA3_NPCS00   ((D940_REG) D940C_PIO_PA3) //!<  SPI0 Peripheral Chip Select 0
#define D940C_PA3_DOUT     ((D940_REG) D940C_PIO_PA3) //!<  CAN1 data OUT (III)
#define D940C_PIO_PA30       ((D940_REG) 1 << 30) //!< Pin Controlled by PA30
#define D940C_PA30_CFCE1    ((D940_REG) D940C_PIO_PA30) //!<  CompactFlash Chip Enable 1
#define D940C_PA30_PCK3     ((D940_REG) D940C_PIO_PA30) //!<  PMC Programmable clock Output 3
#define D940C_PIO_PA31       ((D940_REG) 1 << 31) //!< Pin Controlled by PA31
#define D940C_PA31_CFCE2    ((D940_REG) D940C_PIO_PA31) //!<  CompactFlash Chip Enable 2
#define D940C_PIO_PA4        ((D940_REG) 1 <<  4) //!< Pin Controlled by PA4
#define D940C_PA4_NPCS01   ((D940_REG) D940C_PIO_PA4) //!<  SPI0 Peripheral Chip Select 1
#define D940C_PIO_PA5        ((D940_REG) 1 <<  5) //!< Pin Controlled by PA5
#define D940C_PA5_NPCS02   ((D940_REG) D940C_PIO_PA5) //!<  SPI0 Peripheral Chip Select 2
#define D940C_PA5_TIOA0    ((D940_REG) D940C_PIO_PA5) //!<  Timer Counter 0 Pin A
#define D940C_PIO_PA6        ((D940_REG) 1 <<  6) //!< Pin Controlled by PA6
#define D940C_PA6_NPCS03   ((D940_REG) D940C_PIO_PA6) //!<  SPI0 Peripheral Chip Select 3
#define D940C_PA6_TIOB1    ((D940_REG) D940C_PIO_PA6) //!<  Timer Counter 1 Pin B
#define D940C_PIO_PA7        ((D940_REG) 1 <<  7) //!< Pin Controlled by PA7
#define D940C_PA7_RXD0     ((D940_REG) D940C_PIO_PA7) //!<  USART0 Receive Data
#define D940C_PA7_DRXD     ((D940_REG) D940C_PIO_PA7) //!<  DBGU data input
#define D940C_PIO_PA8        ((D940_REG) 1 <<  8) //!< Pin Controlled by PA8
#define D940C_PA8_TXD0     ((D940_REG) D940C_PIO_PA8) //!<  USART0 Transmit Data
#define D940C_PA8_PCK1     ((D940_REG) D940C_PIO_PA8) //!<  PMC Programmable clock Output 1
#define D940C_PIO_PA9        ((D940_REG) 1 <<  9) //!< Pin Controlled by PA9
#define D940C_PA9_CTS0     ((D940_REG) D940C_PIO_PA9) //!<  USART0 Clear To Send
#define D940C_PA9_NPCS01   ((D940_REG) D940C_PIO_PA9) //!<  SPI0 Peripheral Chip Select 1 (III)
#define D940C_PIO_PB0        ((D940_REG) 1 <<  0) //!< Pin Controlled by PB0
#define D940C_PB0_RD0      ((D940_REG) D940C_PIO_PB0) //!<  SSC receive data 0
#define D940C_PB0_NPCS03   ((D940_REG) D940C_PIO_PB0) //!<  SPI0 Peripheral Chip Select 3 (III)
#define D940C_PIO_PB1        ((D940_REG) 1 <<  1) //!< Pin Controlled by PB1
#define D940C_PB1_TD0      ((D940_REG) D940C_PIO_PB1) //!<  SSC transmit data 0
#define D940C_PB1_TIOB0    ((D940_REG) D940C_PIO_PB1) //!<  Timer Counter 0 Pin B
#define D940C_PIO_PB10       ((D940_REG) 1 << 10) //!< Pin Controlled by PB10
#define D940C_PB10_RF1      ((D940_REG) D940C_PIO_PB10) //!<  SSC Receive frame synchronism 1
#define D940C_PB10_RTS1     ((D940_REG) D940C_PIO_PB10) //!<  USART1 Ready To Send (III)
#define D940C_PIO_PB11       ((D940_REG) 1 << 11) //!< Pin Controlled by PB11
#define D940C_PB11_RK1      ((D940_REG) D940C_PIO_PB11) //!<  SSC Receive clock 1
#define D940C_PB11_EBI_A_22 ((D940_REG) D940C_PIO_PB11) //!<  EBI: Bus address pin 22 (III)
#define D940C_PIO_PB12       ((D940_REG) 1 << 12) //!< Pin Controlled by PB12
#define D940C_PB12_RD2      ((D940_REG) D940C_PIO_PB12) //!<  SSC receive data 2
#define D940C_PB12_EBI_A_23 ((D940_REG) D940C_PIO_PB12) //!<  EBI: Bus address pin 23 (III)
#define D940C_PIO_PB13       ((D940_REG) 1 << 13) //!< Pin Controlled by PB13
#define D940C_PB13_TD2      ((D940_REG) D940C_PIO_PB13) //!<  SSC transmit data 2
#define D940C_PIO_PB14       ((D940_REG) 1 << 14) //!< Pin Controlled by PB14
#define D940C_PB14_TF2      ((D940_REG) D940C_PIO_PB14) //!<  SSC Transmit frame synchronism 2
#define D940C_PB14_EBI_A_24 ((D940_REG) D940C_PIO_PB14) //!<  EBI: Bus address pin 24 (III)
#define D940C_PIO_PB15       ((D940_REG) 1 << 15) //!< Pin Controlled by PB15
#define D940C_PB15_TK2      ((D940_REG) D940C_PIO_PB15) //!<  SSC Transmit clock 2
#define D940C_PB15_NPCS03   ((D940_REG) D940C_PIO_PB15) //!<  SPI0 Peripheral Chip Select 3 (II)
#define D940C_PIO_PB16       ((D940_REG) 1 << 16) //!< Pin Controlled by PB16
#define D940C_PB16_RF2      ((D940_REG) D940C_PIO_PB16) //!<  SSC Receive frame synchronism 2
#define D940C_PB16_EMAC_MDC ((D940_REG) D940C_PIO_PB16) //!<  ETH MDC (II)
#define D940C_PIO_PB17       ((D940_REG) 1 << 17) //!< Pin Controlled by PB17
#define D940C_PB17_RK2      ((D940_REG) D940C_PIO_PB17) //!<  SSC Receive clock 2
#define D940C_PB17_EMAC_FCE100 ((D940_REG) D940C_PIO_PB17) //!<  ETH FCE100 (II)
#define D940C_PIO_PB18       ((D940_REG) 1 << 18) //!< Pin Controlled by PB18
#define D940C_PB18_RD3      ((D940_REG) D940C_PIO_PB18) //!<  SSC receive data 3
#define D940C_PB18_EBI_A_25 ((D940_REG) D940C_PIO_PB18) //!<  EBI: Bus address pin 25 (III)
#define D940C_PIO_PB19       ((D940_REG) 1 << 19) //!< Pin Controlled by PB19
#define D940C_PB19_TD3      ((D940_REG) D940C_PIO_PB19) //!<  SSC transmit data 3
#define D940C_PIO_PB2        ((D940_REG) 1 <<  2) //!< Pin Controlled by PB2
#define D940C_PB2_TF0      ((D940_REG) D940C_PIO_PB2) //!<  SSC Transmit frame synchronism 0
#define D940C_PB2_PCK0     ((D940_REG) D940C_PIO_PB2) //!<  PMC Programmable clock output 0 (II)
#define D940C_PIO_PB20       ((D940_REG) 1 << 20) //!< Pin Controlled by PB20
#define D940C_PB20_TF3      ((D940_REG) D940C_PIO_PB20) //!<  SSC Transmit frame synchronism 3
#define D940C_PB20_EMAC_MDC ((D940_REG) D940C_PIO_PB20) //!<  ETH MDC (III)
#define D940C_PIO_PB21       ((D940_REG) 1 << 21) //!< Pin Controlled by PB21
#define D940C_PB21_TK3      ((D940_REG) D940C_PIO_PB21) //!<  SSC Transmit clock 3
#define D940C_PB21_EMAC_FCE100 ((D940_REG) D940C_PIO_PB21) //!<  ETH FCE100 (III)
#define D940C_PIO_PB22       ((D940_REG) 1 << 22) //!< Pin Controlled by PB22
#define D940C_PB22_RF3      ((D940_REG) D940C_PIO_PB22) //!<  SSC Receive frame synchronism 3
#define D940C_PB22_RTS1     ((D940_REG) D940C_PIO_PB22) //!<  USART1 Ready To Send (II)
#define D940C_PIO_PB23       ((D940_REG) 1 << 23) //!< Pin Controlled by PB23
#define D940C_PB23_RK3      ((D940_REG) D940C_PIO_PB23) //!<  SSC Receive clock 3
#define D940C_PB23_DTXD     ((D940_REG) D940C_PIO_PB23) //!<  DBGU data output (II)
#define D940C_PIO_PB24       ((D940_REG) 1 << 24) //!< Pin Controlled by PB24
#define D940C_PB24_TCLK0    ((D940_REG) D940C_PIO_PB24) //!<  Timer counter 0 external input
#define D940C_PIO_PB25       ((D940_REG) 1 << 25) //!< Pin Controlled by PB25
#define D940C_PB25_IRQ0     ((D940_REG) D940C_PIO_PB25) //!<  AIC external interrupt 0, also to mAgicV M_INT0
#define D940C_PB25_RTS2     ((D940_REG) D940C_PIO_PB25) //!<  USART2 Ready To Send (II)
#define D940C_PIO_PB26       ((D940_REG) 1 << 26) //!< Pin Controlled by PB26
#define D940C_PB26_DIN      ((D940_REG) D940C_PIO_PB26) //!<  CAN0 data IN
#define D940C_PB26_NPCS02   ((D940_REG) D940C_PIO_PB26) //!<  SPI1 Peripheral Chip Select 2 (III)
#define D940C_PIO_PB27       ((D940_REG) 1 << 27) //!< Pin Controlled by PB27
#define D940C_PB27_DOUT     ((D940_REG) D940C_PIO_PB27) //!<  CAN0 data OUT
#define D940C_PIO_PB28       ((D940_REG) 1 << 28) //!< Pin Controlled by PB28
#define D940C_PB28_EBI_A_22 ((D940_REG) D940C_PIO_PB28) //!<  EBI: Bus address pin 22
#define D940C_PB28_NPCS01   ((D940_REG) D940C_PIO_PB28) //!<  SPI0 Peripheral Chip Select 1 (II)
#define D940C_PIO_PB29       ((D940_REG) 1 << 29) //!< Pin Controlled by PB29
#define D940C_PB29_EBI_A_23 ((D940_REG) D940C_PIO_PB29) //!<  EBI: Bus address pin 23
#define D940C_PB29_NPCS02   ((D940_REG) D940C_PIO_PB29) //!<  SPI0 Peripheral Chip Select 2 (II)
#define D940C_PIO_PB3        ((D940_REG) 1 <<  3) //!< Pin Controlled by PB3
#define D940C_PB3_TK0      ((D940_REG) D940C_PIO_PB3) //!<  SSC Transmit clock 0
#define D940C_PB3_DOUT     ((D940_REG) D940C_PIO_PB3) //!<  CAN0 data OUT (II)
#define D940C_PIO_PB30       ((D940_REG) 1 << 30) //!< Pin Controlled by PB30
#define D940C_PB30_EBI_A_24 ((D940_REG) D940C_PIO_PB30) //!<  EBI: Bus address pin 24
#define D940C_PB30_PCK2     ((D940_REG) D940C_PIO_PB30) //!<  PMC Programmable clock Output 2 (II)
#define D940C_PIO_PB31       ((D940_REG) 1 << 31) //!< Pin Controlled by PB31
#define D940C_PB31_EBI_A_25 ((D940_REG) D940C_PIO_PB31) //!<  EBI: Bus address pin 25
#define D940C_PB31_PCK3     ((D940_REG) D940C_PIO_PB31) //!<  PMC Programmable clock Output 3 (II)
#define D940C_PIO_PB4        ((D940_REG) 1 <<  4) //!< Pin Controlled by PB4
#define D940C_PB4_RF0      ((D940_REG) D940C_PIO_PB4) //!<  SSC Receive frame synchronism 0
#define D940C_PB4_RTS0     ((D940_REG) D940C_PIO_PB4) //!<  USART0 Receive to send (II)
#define D940C_PIO_PB5        ((D940_REG) 1 <<  5) //!< Pin Controlled by PB5
#define D940C_PB5_RK0      ((D940_REG) D940C_PIO_PB5) //!<  SSC Receive clock 0
#define D940C_PIO_PB6        ((D940_REG) 1 <<  6) //!< Pin Controlled by PB6
#define D940C_PB6_RD1      ((D940_REG) D940C_PIO_PB6) //!<  SSC receive data 1
#define D940C_PB6_DOUT     ((D940_REG) D940C_PIO_PB6) //!<  CAN0 data OUT (III)
#define D940C_PIO_PB7        ((D940_REG) 1 <<  7) //!< Pin Controlled by PB7
#define D940C_PB7_TD1      ((D940_REG) D940C_PIO_PB7) //!<  SSC transmit data 1
#define D940C_PB7_TIOA1    ((D940_REG) D940C_PIO_PB7) //!<  Timer Counter 1 Pin A
#define D940C_PIO_PB8        ((D940_REG) 1 <<  8) //!< Pin Controlled by PB8
#define D940C_PB8_TF1      ((D940_REG) D940C_PIO_PB8) //!<  SSC Transmit frame synchronism 1
#define D940C_PB8_PCK1     ((D940_REG) D940C_PIO_PB8) //!<  PMC Programmable clock Output 1 (II)
#define D940C_PIO_PB9        ((D940_REG) 1 <<  9) //!< Pin Controlled by PB9
#define D940C_PB9_TK1      ((D940_REG) D940C_PIO_PB9) //!<  SSC Transmit clock 1
#define D940C_PB9_NPCS01   ((D940_REG) D940C_PIO_PB9) //!<  SPI1 Peripheral Chip Select 1 (III)
#define D940C_PIO_PC0        ((D940_REG) 1 <<  0) //!< Pin Controlled by PC0
#define D940C_PC0_MISO1    ((D940_REG) D940C_PIO_PC0) //!<  SPI1 Master In Slave (bi-directional)
#define D940C_PC0_TD0      ((D940_REG) D940C_PIO_PC0) //!<  SSC transmit data 0 (II)
#define D940C_PIO_PC1        ((D940_REG) 1 <<  1) //!< Pin Controlled by PC1
#define D940C_PC1_MOSI1    ((D940_REG) D940C_PIO_PC1) //!<  SPI1 Master Out Slave (bi-directional)
#define D940C_PC1_TD1      ((D940_REG) D940C_PIO_PC1) //!<  SSC transmit data 1 (II)
#define D940C_PIO_PC10       ((D940_REG) 1 << 10) //!< Pin Controlled by PC10
#define D940C_PC10_TXD1     ((D940_REG) D940C_PIO_PC10) //!<  USART1 Transmit Data
#define D940C_PC10_EMAC_ETX0 ((D940_REG) D940C_PIO_PC10) //!<  EMACB ETX0 (III)
#define D940C_PIO_PC11       ((D940_REG) 1 << 11) //!< Pin Controlled by PC11
#define D940C_PC11_CTS1     ((D940_REG) D940C_PIO_PC11) //!<  USART1 Clear To Send
#define D940C_PC11_EMAC_ETX1 ((D940_REG) D940C_PIO_PC11) //!<  EMACB ETX1 (III)
#define D940C_PIO_PC12       ((D940_REG) 1 << 12) //!< Pin Controlled by PC12
#define D940C_PC12_RTS1     ((D940_REG) D940C_PIO_PC12) //!<  USART1 Ready to send
#define D940C_PC12_NPCS01   ((D940_REG) D940C_PIO_PC12) //!<  SPI1 Peripheral Chip Select 1 (II)
#define D940C_PIO_PC13       ((D940_REG) 1 << 13) //!< Pin Controlled by PC13
#define D940C_PC13_SCK1     ((D940_REG) D940C_PIO_PC13) //!<  USART1 Clock
#define D940C_PC13_TD3      ((D940_REG) D940C_PIO_PC13) //!<  SSC transmit data 3 (II)
#define D940C_PIO_PC14       ((D940_REG) 1 << 14) //!< Pin Controlled by PC14
#define D940C_PC14_RXD2     ((D940_REG) D940C_PIO_PC14) //!<  USART2 Receive Data
#define D940C_PC14_EBI_A_22 ((D940_REG) D940C_PIO_PC14) //!<  EBI: Bus address pin 22 (II)
#define D940C_PIO_PC15       ((D940_REG) 1 << 15) //!< Pin Controlled by PC15
#define D940C_PC15_TXD2     ((D940_REG) D940C_PIO_PC15) //!<  USART2 Transmit Data
#define D940C_PC15_EBI_A_23 ((D940_REG) D940C_PIO_PC15) //!<  EBI: Bus address pin 23 (II)
#define D940C_PIO_PC16       ((D940_REG) 1 << 16) //!< Pin Controlled by PC16
#define D940C_PC16_CTS2     ((D940_REG) D940C_PIO_PC16) //!<  USART2 Clear To Send
#define D940C_PC16_EBI_A_24 ((D940_REG) D940C_PIO_PC16) //!<  EBI: Bus address pin 24 (II)
#define D940C_PIO_PC17       ((D940_REG) 1 << 17) //!< Pin Controlled by PC17
#define D940C_PC17_RTS2     ((D940_REG) D940C_PIO_PC17) //!<  USART2 Ready to send
#define D940C_PC17_EBI_A_25 ((D940_REG) D940C_PIO_PC17) //!<  EBI: Bus address pin 25 (II)
#define D940C_PIO_PC18       ((D940_REG) 1 << 18) //!< Pin Controlled by PC18
#define D940C_PC18_SCK2     ((D940_REG) D940C_PIO_PC18) //!<  USART2 Receive Data
#define D940C_PC18_NPCS02   ((D940_REG) D940C_PIO_PC18) //!<  SPI1 Peripheral Chip Select 2 (II)
#define D940C_PIO_PC19       ((D940_REG) 1 << 19) //!< Pin Controlled by PC19
#define D940C_PC19_TIOB2    ((D940_REG) D940C_PIO_PC19) //!<  Timer Counter 2 Pin B
#define D940C_PC19_NPCS03   ((D940_REG) D940C_PIO_PC19) //!<  SPI1 Peripheral Chip Select 3 (II)
#define D940C_PIO_PC2        ((D940_REG) 1 <<  2) //!< Pin Controlled by PC2
#define D940C_PC2_CLK1     ((D940_REG) D940C_PIO_PC2) //!<  SPI1 Serial Clock
#define D940C_PC2_TD2      ((D940_REG) D940C_PIO_PC2) //!<  SSC transmit data 2 (II)
#define D940C_PIO_PC20       ((D940_REG) 1 << 20) //!< Pin Controlled by PC20
#define D940C_PC20_TWD      ((D940_REG) D940C_PIO_PC20) //!<  TWI1 Two-wire Serial Data
#define D940C_PC20_TD3      ((D940_REG) D940C_PIO_PC20) //!<  SSC transmit data 3 (III)
#define D940C_PIO_PC21       ((D940_REG) 1 << 21) //!< Pin Controlled by PC21
#define D940C_PC21_TWCK     ((D940_REG) D940C_PIO_PC21) //!<  TWI1 Two-wire Serial Clock
#define D940C_PC21_NPCS03   ((D940_REG) D940C_PIO_PC21) //!<  SPI1 Peripheral Chip Select 3 (III)
#define D940C_PIO_PC22       ((D940_REG) 1 << 22) //!< Pin Controlled by PC22
#define D940C_PC22_MCCDK    ((D940_REG) D940C_PIO_PC22) //!<  Multimedia Card MCCDK
#define D940C_PC22_DOUT     ((D940_REG) D940C_PIO_PC22) //!<  CAN1 data OUT (II)
#define D940C_PIO_PC23       ((D940_REG) 1 << 23) //!< Pin Controlled by PC23
#define D940C_PC23_MCCDA    ((D940_REG) D940C_PIO_PC23) //!<  Multimedia Card MCCDA
#define D940C_PIO_PC24       ((D940_REG) 1 << 24) //!< Pin Controlled by PC24
#define D940C_PC24_MCDA0    ((D940_REG) D940C_PIO_PC24) //!<  Multimedia Card MCDA0
#define D940C_PC24_SMOE     ((D940_REG) D940C_PIO_PC24) //!<  SmartMedia Output Enable (II)
#define D940C_PIO_PC25       ((D940_REG) 1 << 25) //!< Pin Controlled by PC25
#define D940C_PC25_MCDA1    ((D940_REG) D940C_PIO_PC25) //!<  Multimedia Card MCDA1
#define D940C_PC25_SMWE     ((D940_REG) D940C_PIO_PC25) //!<  SmartMedia Write Enable (II)
#define D940C_PIO_PC26       ((D940_REG) 1 << 26) //!< Pin Controlled by PC26
#define D940C_PC26_MCDA2    ((D940_REG) D940C_PIO_PC26) //!<  Multimedia Card MCDA2
#define D940C_PC26_NCS4_CFCS0 ((D940_REG) D940C_PIO_PC26) //!<  Chip Select 4 / CompactFlash Chip Select 0 (II)
#define D940C_PIO_PC27       ((D940_REG) 1 << 27) //!< Pin Controlled by PC27
#define D940C_PC27_MCDA3    ((D940_REG) D940C_PIO_PC27) //!<  Multimedia Card MCDA3
#define D940C_PC27_NCS5_CFCS1 ((D940_REG) D940C_PIO_PC27) //!<  Chip Select 5 / CompactFlash Chip Select 1 (II)
#define D940C_PIO_PC28       ((D940_REG) 1 << 28) //!< Pin Controlled by PC28
#define D940C_PC28_DIN      ((D940_REG) D940C_PIO_PC28) //!<  CAN1 data IN
#define D940C_PC28_NCS6     ((D940_REG) D940C_PIO_PC28) //!<  Chip Select 6 (II)
#define D940C_PIO_PC29       ((D940_REG) 1 << 29) //!< Pin Controlled by PC29
#define D940C_PC29_DOUT     ((D940_REG) D940C_PIO_PC29) //!<  CAN1 data OUT
#define D940C_PC29_NCS7     ((D940_REG) D940C_PIO_PC29) //!<  Chip Select 7 (II)
#define D940C_PIO_PC3        ((D940_REG) 1 <<  3) //!< Pin Controlled by PC3
#define D940C_PC3_NPCS00   ((D940_REG) D940C_PIO_PC3) //!<  SPI1 Peripheral Chip Select 0
#define D940C_PC3_EMAC_ETX0 ((D940_REG) D940C_PIO_PC3) //!<  EMACB ETX0 (II)
#define D940C_PIO_PC30       ((D940_REG) 1 << 30) //!< Pin Controlled by PC30
#define D940C_PC30_DRXD     ((D940_REG) D940C_PIO_PC30) //!<  DBGU data input
#define D940C_PC30_CFCE1    ((D940_REG) D940C_PIO_PC30) //!<  CompactFlash Chip Enable 1 (II)
#define D940C_PIO_PC31       ((D940_REG) 1 << 31) //!< Pin Controlled by PC31
#define D940C_PC31_DTXD     ((D940_REG) D940C_PIO_PC31) //!<  DBGU data output
#define D940C_PC31_CFCE2    ((D940_REG) D940C_PIO_PC31) //!<  CompactFlash Chip Enable 2 (II)
#define D940C_PIO_PC4        ((D940_REG) 1 <<  4) //!< Pin Controlled by PC4
#define D940C_PC4_NPCS01   ((D940_REG) D940C_PIO_PC4) //!<  SPI1 Peripheral Chip Select 1
#define D940C_PC4_EMAC_ETX1 ((D940_REG) D940C_PIO_PC4) //!<  EMACB ETX1 (II)
#define D940C_PIO_PC5        ((D940_REG) 1 <<  5) //!< Pin Controlled by PC5
#define D940C_PC5_NPCS02   ((D940_REG) D940C_PIO_PC5) //!<  SPI1 Peripheral Chip Select 2
#define D940C_PIO_PC6        ((D940_REG) 1 <<  6) //!< Pin Controlled by PC6
#define D940C_PC6_NPCS03   ((D940_REG) D940C_PIO_PC6) //!<  SPI1 Peripheral Chip Select 3
#define D940C_PC6_SMOE     ((D940_REG) D940C_PIO_PC6) //!<  SmartMedia Output Enable (III)
#define D940C_PIO_PC7        ((D940_REG) 1 <<  7) //!< Pin Controlled by PC7
#define D940C_PC7_TWD      ((D940_REG) D940C_PIO_PC7) //!<  TWI0 Two-wire Serial Data
#define D940C_PC7_TD0      ((D940_REG) D940C_PIO_PC7) //!<  SSC transmit data 0 (III)
#define D940C_PIO_PC8        ((D940_REG) 1 <<  8) //!< Pin Controlled by PC8
#define D940C_PC8_TWCK     ((D940_REG) D940C_PIO_PC8) //!<  TWI0 Two-wire Serial Clock
#define D940C_PC8_TD1      ((D940_REG) D940C_PIO_PC8) //!<  SSC transmit data 1 (III)
#define D940C_PIO_PC9        ((D940_REG) 1 <<  9) //!< Pin Controlled by PC9
#define D940C_PC9_RXD1     ((D940_REG) D940C_PIO_PC9) //!<  USART1 Receive Data
#define D940C_PC9_TD2      ((D940_REG) D940C_PIO_PC9) //!<  SSC transmit data 2 (III)

// *****************************************************************************
//               PERIPHERAL ID DEFINITIONS FOR D940HF
// *****************************************************************************
#define D940C_ID_SYS    ((D940_REG)  1) //!< Interrupt line shared by SDRAMC,DBGU, System controller and PMC
#define D940C_ID_PIOA   ((D940_REG)  2) //!< Parallel IO Controller A
#define D940C_ID_PIOB   ((D940_REG)  3) //!< Parallel IO Controller B
#define D940C_ID_PIOC   ((D940_REG)  4) //!< Parallel IO Controller C
#define D940C_ID_EMACB  ((D940_REG)  5) //!< EMACB
#define D940C_ID_US0    ((D940_REG)  6) //!< USART 0
#define D940C_ID_US1    ((D940_REG)  7) //!< USART 1
#define D940C_ID_US2    ((D940_REG)  8) //!< USART 2
#define D940C_ID_MCI0   ((D940_REG)  9) //!< MCI 0
#define D940C_ID_UDP    ((D940_REG) 10) //!< USB Device
#define D940C_ID_TWI0   ((D940_REG) 11) //!< Two-Wire Interface 0
#define D940C_ID_SPI0   ((D940_REG) 12) //!< Serial Peripheral Interface 0
#define D940C_ID_SPI1   ((D940_REG) 13) //!< Serial Peripheral Interface 1
#define D940C_ID_SSC0   ((D940_REG) 14) //!< Serial Synchronous Controller 0, also to M_INT2
#define D940C_ID_SSC1   ((D940_REG) 15) //!< Serial Synchronous Controller 1, also to M_INT2
#define D940C_ID_SSC2   ((D940_REG) 16) //!< Serial Synchronous Controller 2, also to M_INT2
#define D940C_ID_TC0    ((D940_REG) 17) //!< Timer Counter 0
#define D940C_ID_TC1    ((D940_REG) 18) //!< Timer Counter 1
#define D940C_ID_TC2    ((D940_REG) 19) //!< Timer Counter 2
#define D940C_ID_UHP    ((D940_REG) 20) //!< USB Host
#define D940C_ID_SSC3   ((D940_REG) 21) //!< Serial Synchronous Controller 3, also to M_INT2
#define D940C_ID_TWI1   ((D940_REG) 22) //!< Two-Wire Interface 1
#define D940C_ID_CAN0   ((D940_REG) 23) //!< Controller Area Network 0
#define D940C_ID_CAN1   ((D940_REG) 24) //!< Controller Area Network 1
#define D940C_ID_IRQ0   ((D940_REG) 29) //!< PIO B line 25, also to M_INT0
#define D940C_ID_IRQ1   ((D940_REG) 30) //!< PIO A line 5, also to M_INT1
#define D940C_ID_IRQ2   ((D940_REG) 31) //!< PIO A line 16

// *****************************************************************************
//               BASE ADDRESS DEFINITIONS FOR D940HF
// *****************************************************************************

#define D940C_BASE_SYS       ((D940S_SYS __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFFEA00)) //!< (SYS) Base Address
#define D940C_BASE_SDRAMC    ((D940S_SDRAMC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFFEA00)) //!< (SDRAMC) Base Address
#define D940C_BASE_SMC       ((D940S_SMC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFFEC00)) //!< (SMC) Base Address
#define D940C_BASE_SMCII     ((D940S_SMC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFFEC00)) //!< (SMCII) Base Address
#define D940C_BASE_SMCIII    ((D940S_SMC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFFEC00)) //!< (SMCIII) Base Address
#define D940C_BASE_MATRIX    ((D940S_MATRIX __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFFEE00)) //!< (MATRIX) Base Address
#define D940C_BASE_MATRIXII  ((D940S_MATRIX __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFFEE00)) //!< (MATRIXII) Base Address
#define D940C_BASE_MATRIXIII ((D940S_MATRIX __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFFEE00)) //!< (MATRIXIII) Base Address
#define D940C_BASE_AIC       ((D940S_AIC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFFF000)) //!< (AIC) Base Address
#define D940C_BASE_PDC_DBGU  ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFFF300)) //!< (PDC_DBGU) Base Address
#define D940C_BASE_DBGU      ((D940S_DBGU __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFFF200)) //!< (DBGU) Base Address
#define D940C_BASE_PDC_DBGUII ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFFF300)) //!< (PDC_DBGUII) Base Address
#define D940C_BASE_DBGUII    ((D940S_DBGU __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFFF200)) //!< (DBGUII) Base Address
#define D940C_BASE_PDC_DBGUIII ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFFF300)) //!< (PDC_DBGUIII) Base Address
#define D940C_BASE_DBGUIII   ((D940S_DBGU __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFFF200)) //!< (DBGUIII) Base Address
#define D940C_BASE_PIOA      ((D940S_PIO __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFFF400)) //!< (PIOA) Base Address
#define D940C_BASE_PIOB      ((D940S_PIO __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFFF600)) //!< (PIOB) Base Address
#define D940C_BASE_PIOC      ((D940S_PIO __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFFF800)) //!< (PIOC) Base Address
#define D940C_BASE_CKGR      ((D940S_CKGR __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFFFC20)) //!< (CKGR) Base Address
#define D940C_BASE_PMC       ((D940S_PMC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFFFC00)) //!< (PMC) Base Address
#define D940C_BASE_PMCII     ((D940S_PMC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFFFC00)) //!< (PMCII) Base Address
#define D940C_BASE_RSTC      ((D940S_RSTC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFFFD00)) //!< (RSTC) Base Address
#define D940C_BASE_SHDWC     ((D940S_SHDWC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFFFD10)) //!< (SHDWC) Base Address
#define D940C_BASE_RTTC      ((D940S_RTTC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFFFD20)) //!< (RTTC) Base Address
#define D940C_BASE_PITC      ((D940S_PITC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFFFD30)) //!< (PITC) Base Address
#define D940C_BASE_WDTC      ((D940S_WDTC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFFFD40)) //!< (WDTC) Base Address
#define D940C_BASE_TC0       ((D940S_TC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFA0000)) //!< (TC0) Base Address
#define D940C_BASE_TC1       ((D940S_TC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFA0040)) //!< (TC1) Base Address
#define D940C_BASE_TC2       ((D940S_TC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFA0080)) //!< (TC2) Base Address
#define D940C_BASE_TCB0      ((D940S_TCB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFA0000)) //!< (TCB0) Base Address
#define D940C_BASE_UDP       ((D940S_UDP __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFA4000)) //!< (UDP) Base Address
#define D940C_BASE_TWI0      ((D940S_TWI __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFAC000)) //!< (TWI0) Base Address
#define D940C_BASE_TWI1      ((D940S_TWI __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFD4000)) //!< (TWI1) Base Address
#define D940C_BASE_PDC_US0   ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFB0100)) //!< (PDC_US0) Base Address
#define D940C_BASE_US0       ((D940S_USART __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFB0000)) //!< (US0) Base Address
#define D940C_BASE_PDC_US0II ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFB0100)) //!< (PDC_US0II) Base Address
#define D940C_BASE_US0II     ((D940S_USART __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFB0000)) //!< (US0II) Base Address
#define D940C_BASE_PDC_US0III ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFB0100)) //!< (PDC_US0III) Base Address
#define D940C_BASE_US0III    ((D940S_USART __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFB0000)) //!< (US0III) Base Address
#define D940C_BASE_PDC_US1   ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFB4100)) //!< (PDC_US1) Base Address
#define D940C_BASE_US1       ((D940S_USART __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFB4000)) //!< (US1) Base Address
#define D940C_BASE_PDC_US1II ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFB4100)) //!< (PDC_US1II) Base Address
#define D940C_BASE_US1II     ((D940S_USART __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFB4000)) //!< (US1II) Base Address
#define D940C_BASE_PDC_US1III ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFB4100)) //!< (PDC_US1III) Base Address
#define D940C_BASE_US1III    ((D940S_USART __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFB4000)) //!< (US1III) Base Address
#define D940C_BASE_PDC_US2   ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFB8100)) //!< (PDC_US2) Base Address
#define D940C_BASE_US2       ((D940S_USART __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFB8000)) //!< (US2) Base Address
#define D940C_BASE_PDC_US2II ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFB8100)) //!< (PDC_US2II) Base Address
#define D940C_BASE_US2II     ((D940S_USART __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFB8000)) //!< (US2II) Base Address
#define D940C_BASE_PDC_US2III ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFB8100)) //!< (PDC_US2III) Base Address
#define D940C_BASE_US2III    ((D940S_USART __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFB8000)) //!< (US2III) Base Address
#define D940C_BASE_PDC_SSC0  ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFBC100)) //!< (PDC_SSC0) Base Address
#define D940C_BASE_SSC0      ((D940S_SSC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFBC000)) //!< (SSC0) Base Address
#define D940C_BASE_PDC_SSC0II ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFBC100)) //!< (PDC_SSC0II) Base Address
#define D940C_BASE_SSC0II    ((D940S_SSC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFBC000)) //!< (SSC0II) Base Address
#define D940C_BASE_PDC_SSC0III ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFBC100)) //!< (PDC_SSC0III) Base Address
#define D940C_BASE_SSC0III   ((D940S_SSC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFBC000)) //!< (SSC0III) Base Address
#define D940C_BASE_PDC_SSC1  ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFC0100)) //!< (PDC_SSC1) Base Address
#define D940C_BASE_SSC1      ((D940S_SSC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFC0000)) //!< (SSC1) Base Address
#define D940C_BASE_PDC_SSC1II ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFC0100)) //!< (PDC_SSC1II) Base Address
#define D940C_BASE_SSC1II    ((D940S_SSC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFC0000)) //!< (SSC1II) Base Address
#define D940C_BASE_PDC_SSC1III ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFC0100)) //!< (PDC_SSC1III) Base Address
#define D940C_BASE_SSC1III   ((D940S_SSC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFC0000)) //!< (SSC1III) Base Address
#define D940C_BASE_PDC_SSC2  ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFC4100)) //!< (PDC_SSC2) Base Address
#define D940C_BASE_SSC2      ((D940S_SSC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFC4000)) //!< (SSC2) Base Address
#define D940C_BASE_PDC_SSC2II ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFC4100)) //!< (PDC_SSC2II) Base Address
#define D940C_BASE_SSC2II    ((D940S_SSC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFC4000)) //!< (SSC2II) Base Address
#define D940C_BASE_PDC_SSC2III ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFC4100)) //!< (PDC_SSC2III) Base Address
#define D940C_BASE_SSC2III   ((D940S_SSC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFC4000)) //!< (SSC2III) Base Address
#define D940C_BASE_PDC_SSC3  ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFD0100)) //!< (PDC_SSC3) Base Address
#define D940C_BASE_SSC3      ((D940S_SSC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFD0000)) //!< (SSC3) Base Address
#define D940C_BASE_PDC_SSC3II ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFD0100)) //!< (PDC_SSC3II) Base Address
#define D940C_BASE_SSC3II    ((D940S_SSC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFD0000)) //!< (SSC3II) Base Address
#define D940C_BASE_PDC_SSC3III ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFD0100)) //!< (PDC_SSC3III) Base Address
#define D940C_BASE_SSC3III   ((D940S_SSC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFD0000)) //!< (SSC3III) Base Address
#define D940C_BASE_PDC_SPI0  ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFC8100)) //!< (PDC_SPI0) Base Address
#define D940C_BASE_SPI0      ((D940S_SPI __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFC8000)) //!< (SPI0) Base Address
#define D940C_BASE_PDC_SPI0II ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFC8100)) //!< (PDC_SPI0II) Base Address
#define D940C_BASE_SPI0II    ((D940S_SPI __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFC8000)) //!< (SPI0II) Base Address
#define D940C_BASE_PDC_SPI0III ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFC8100)) //!< (PDC_SPI0III) Base Address
#define D940C_BASE_SPI0III   ((D940S_SPI __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFC8000)) //!< (SPI0III) Base Address
#define D940C_BASE_PDC_SPI1  ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFCC100)) //!< (PDC_SPI1) Base Address
#define D940C_BASE_SPI1      ((D940S_SPI __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFCC000)) //!< (SPI1) Base Address
#define D940C_BASE_PDC_SPI1II ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFCC100)) //!< (PDC_SPI1II) Base Address
#define D940C_BASE_SPI1II    ((D940S_SPI __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFCC000)) //!< (SPI1II) Base Address
#define D940C_BASE_PDC_SPI1III ((D940S_PDC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFCC100)) //!< (PDC_SPI1III) Base Address
#define D940C_BASE_SPI1III   ((D940S_SPI __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFCC000)) //!< (SPI1III) Base Address
#define D940C_BASE_UHP       ((D940S_UHP __MAGICV_EXTERNAL*) 	((D940_REG)0x00500000)) //!< (UHP) Base Address
#define D940C_BASE_EMAC      ((D940S_EMAC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFD8000)) //!< (EMAC) Base Address
#define D940C_BASE_EMACII    ((D940S_EMAC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFD8000)) //!< (EMACII) Base Address
#define D940C_BASE_EMACIII   ((D940S_EMAC __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFD8000)) //!< (EMACIII) Base Address
#define D940C_BASE_CAN0_MB0  ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFDC200)) //!< (CAN0_MB0) Base Address
#define D940C_BASE_CAN0_MB1  ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFDC220)) //!< (CAN0_MB1) Base Address
#define D940C_BASE_CAN0_MB2  ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFDC240)) //!< (CAN0_MB2) Base Address
#define D940C_BASE_CAN0_MB3  ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFDC260)) //!< (CAN0_MB3) Base Address
#define D940C_BASE_CAN0_MB4  ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFDC280)) //!< (CAN0_MB4) Base Address
#define D940C_BASE_CAN0_MB5  ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFDC2A0)) //!< (CAN0_MB5) Base Address
#define D940C_BASE_CAN0_MB6  ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFDC2C0)) //!< (CAN0_MB6) Base Address
#define D940C_BASE_CAN0_MB7  ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFDC2E0)) //!< (CAN0_MB7) Base Address
#define D940C_BASE_CAN0_MB8  ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFDC300)) //!< (CAN0_MB8) Base Address
#define D940C_BASE_CAN0_MB9  ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFDC320)) //!< (CAN0_MB9) Base Address
#define D940C_BASE_CAN0_MB10 ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFDC340)) //!< (CAN0_MB10) Base Address
#define D940C_BASE_CAN0_MB11 ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFDC360)) //!< (CAN0_MB11) Base Address
#define D940C_BASE_CAN0_MB12 ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFDC380)) //!< (CAN0_MB12) Base Address
#define D940C_BASE_CAN0_MB13 ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFDC3A0)) //!< (CAN0_MB13) Base Address
#define D940C_BASE_CAN0_MB14 ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFDC3C0)) //!< (CAN0_MB14) Base Address
#define D940C_BASE_CAN0_MB15 ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFDC3E0)) //!< (CAN0_MB15) Base Address
#define D940C_BASE_CAN0      ((D940S_CAN __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFDC000)) //!< (CAN0) Base Address
#define D940C_BASE_CAN0II    ((D940S_CAN __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFDC000)) //!< (CAN0II) Base Address
#define D940C_BASE_CAN0III   ((D940S_CAN __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFDC000)) //!< (CAN0III) Base Address
#define D940C_BASE_CAN1_MB0  ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFE0200)) //!< (CAN1_MB0) Base Address
#define D940C_BASE_CAN1_MB1  ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFE0220)) //!< (CAN1_MB1) Base Address
#define D940C_BASE_CAN1_MB2  ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFE0240)) //!< (CAN1_MB2) Base Address
#define D940C_BASE_CAN1_MB3  ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFE0260)) //!< (CAN1_MB3) Base Address
#define D940C_BASE_CAN1_MB4  ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFE0280)) //!< (CAN1_MB4) Base Address
#define D940C_BASE_CAN1_MB5  ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFE02A0)) //!< (CAN1_MB5) Base Address
#define D940C_BASE_CAN1_MB6  ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFE02C0)) //!< (CAN1_MB6) Base Address
#define D940C_BASE_CAN1_MB7  ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFE02E0)) //!< (CAN1_MB7) Base Address
#define D940C_BASE_CAN1_MB8  ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFE0300)) //!< (CAN1_MB8) Base Address
#define D940C_BASE_CAN1_MB9  ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFE0320)) //!< (CAN1_MB9) Base Address
#define D940C_BASE_CAN1_MB10 ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFE0340)) //!< (CAN1_MB10) Base Address
#define D940C_BASE_CAN1_MB11 ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFE0360)) //!< (CAN1_MB11) Base Address
#define D940C_BASE_CAN1_MB12 ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFE0380)) //!< (CAN1_MB12) Base Address
#define D940C_BASE_CAN1_MB13 ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFE03A0)) //!< (CAN1_MB13) Base Address
#define D940C_BASE_CAN1_MB14 ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFE03C0)) //!< (CAN1_MB14) Base Address
#define D940C_BASE_CAN1_MB15 ((D940S_CAN_MB __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFE03E0)) //!< (CAN1_MB15) Base Address
#define D940C_BASE_CAN1      ((D940S_CAN __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFE0000)) //!< (CAN1) Base Address
#define D940C_BASE_CAN1II    ((D940S_CAN __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFE0000)) //!< (CAN1II) Base Address
#define D940C_BASE_CAN1III   ((D940S_CAN __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFE0000)) //!< (CAN1III) Base Address
#define D940C_BASE_MCI0      ((D940S_MCI __MAGICV_EXTERNAL*) 	((D940_REG)0xFFFA8000)) //!< (MCI0) Base Address

// *****************************************************************************
//               MEMORY MAPPING DEFINITIONS FOR D940HF
// *****************************************************************************

#define D940C_ITCM 	 ((char __MAGICV_EXTERNAL*) 	0x00100000) //!< Maximum ITCM Area base address
#define D940C_ITCM_SIZE	 ((D940_REG) 0x00008000) //!< Maximum ITCM Area size in byte (32 Kbyte)
#define D940C_DTCM 	 ((char __MAGICV_EXTERNAL*) 	0x00200000) //!< Maximum DTCM Area base address
#define D940C_DTCM_SIZE	 ((D940_REG) 0x00008000) //!< Maximum DTCM Area size in byte (32 Kbyte)
#define D940C_IRAM 	 ((char __MAGICV_EXTERNAL*) 	0x00300000) //!< Maximum Internal SRAM base address
#define D940C_IRAM_SIZE	 ((D940_REG) 0x0000C000) //!< Maximum Internal SRAM size in byte (48 Kbyte)
#define D940C_IRAM_MIN	 ((char __MAGICV_EXTERNAL*) 	0x00300000) //!< Minimum Internal RAM base address
#define D940C_IRAM_MIN_SIZE	 ((D940_REG) 0x00004000) //!< Minimum Internal RAM size in byte (16 Kbyte)
#define D940C_IROM 	 ((char __MAGICV_EXTERNAL*) 	0x00400000) //!< Internal ROM base address
#define D940C_IROM_SIZE	 ((D940_REG) 0x00008000) //!< Internal ROM size in byte (32 Kbyte)
#define D940C_EBI_CS0	 ((char __MAGICV_EXTERNAL*) 	0x10000000) //!< EBI Chip Select 0 base address
#define D940C_EBI_CS0_SIZE	 ((D940_REG) 0x10000000) //!< EBI Chip Select 0 size in byte (262144 Kbyte)
#define D940C_EBI_CS1	 ((char __MAGICV_EXTERNAL*) 	0x20000000) //!< EBI Chip Select 1 base address
#define D940C_EBI_CS1_SIZE	 ((D940_REG) 0x10000000) //!< EBI Chip Select 1 size in byte (262144 Kbyte)
#define D940C_EBI_SDRAM	 ((char __MAGICV_EXTERNAL*) 	0x20000000) //!< SDRAM on EBI Chip Select 1 base address
#define D940C_EBI_SDRAM_SIZE	 ((D940_REG) 0x10000000) //!< SDRAM on EBI Chip Select 1 size in byte (262144 Kbyte)
#define D940C_EBI_SDRAM_16BIT	 ((char __MAGICV_EXTERNAL*) 	0x20000000) //!< SDRAM on EBI Chip Select 1 base address
#define D940C_EBI_SDRAM_16BIT_SIZE	 ((D940_REG) 0x02000000) //!< SDRAM on EBI Chip Select 1 size in byte (32768 Kbyte)
#define D940C_EBI_SDRAM_32BIT	 ((char __MAGICV_EXTERNAL*) 	0x20000000) //!< SDRAM on EBI Chip Select 1 base address
#define D940C_EBI_SDRAM_32BIT_SIZE	 ((D940_REG) 0x04000000) //!< SDRAM on EBI Chip Select 1 size in byte (65536 Kbyte)
#define D940C_EBI_CS2	 ((char __MAGICV_EXTERNAL*) 	0x30000000) //!< EBI Chip Select 2 base address
#define D940C_EBI_CS2_SIZE	 ((D940_REG) 0x10000000) //!< EBI Chip Select 2 size in byte (262144 Kbyte)
#define D940C_EBI_CS3	 ((char __MAGICV_EXTERNAL*) 	0x40000000) //!< EBI Chip Select 3 base address
#define D940C_EBI_CS3_SIZE	 ((D940_REG) 0x10000000) //!< EBI Chip Select 3 size in byte (262144 Kbyte)
#define D940C_EBI_SM	 ((char __MAGICV_EXTERNAL*) 	0x40000000) //!< SmartMedia on Chip Select 3 base address
#define D940C_EBI_SM_SIZE	 ((D940_REG) 0x10000000) //!< SmartMedia on Chip Select 3 size in byte (262144 Kbyte)
#define D940C_EBI_CS4	 ((char __MAGICV_EXTERNAL*) 	0x50000000) //!< EBI Chip Select 4 base address
#define D940C_EBI_CS4_SIZE	 ((D940_REG) 0x10000000) //!< EBI Chip Select 4 size in byte (262144 Kbyte)
#define D940C_EBI_CF0	 ((char __MAGICV_EXTERNAL*) 	0x50000000) //!< CompactFlash 0 on Chip Select 4 base address
#define D940C_EBI_CF0_SIZE	 ((D940_REG) 0x10000000) //!< CompactFlash 0 on Chip Select 4 size in byte (262144 Kbyte)
#define D940C_EBI_CS5	 ((char __MAGICV_EXTERNAL*) 	0x60000000) //!< EBI Chip Select 5 base address
#define D940C_EBI_CS5_SIZE	 ((D940_REG) 0x10000000) //!< EBI Chip Select 5 size in byte (262144 Kbyte)
#define D940C_EBI_CF1	 ((char __MAGICV_EXTERNAL*) 	0x60000000) //!< CompactFlash 1 on Chip Select 5 base address
#define D940C_EBI_CF1_SIZE	 ((D940_REG) 0x10000000) //!< CompactFlash 1 on Chip Select 5 size in byte (262144 Kbyte)
#define D940C_EBI_CS6	 ((char __MAGICV_EXTERNAL*) 	0x70000000) //!< EBI Chip Select 6 base address
#define D940C_EBI_CS6_SIZE	 ((D940_REG) 0x10000000) //!< EBI Chip Select 6 size in byte (262144 Kbyte)
#define D940C_EBI_CS7	 ((char __MAGICV_EXTERNAL*) 	0x80000000) //!< EBI Chip Select 7 base address
#define D940C_EBI_CS7_SIZE	 ((D940_REG) 0x10000000) //!< EBI Chip Select 7 size in byte (262144 Kbyte)

#ifdef __chess__
#undef int
#undef char
#undef D940_REG
typedef volatile unsigned long D940_REG;
typedef volatile unsigned long __MAGICV_EXTERNAL * D940P_REG;
#endif

#endif
