#ifndef _DBIOS_MACRO_
#define _DBIOS_MACRO_

/*! \file DBIOS_macro.h
 * \brief Macros defined for accessing peripheral registers and for buffer exchanging plus utilities
 \verbatim
 The aim of this file is to create a layer of macros and functions implemented for both ARM and mAgicV compilers, in order to have the same source codes for the DBIOS functions. \n
 When __chess__ is defined, the macros are compiled for mAgicV (mAgicV C compiler defines __chess__ by default), if not they are compiled for ARM. \n
 The macros and functions marked in documentation as (MAGICV ONLY) are implemented only for mAgicV. \n
 In the documentation you can see the mAgicV macro implementation, plese refer to the DBIOS_macro.h file for the ARM implementation
 \endverbatim
*/



//*----------------------------------------------------------------------------
//* \enum   D940E_MAGICV_DMA_IntSegId
//* \briefiInternal DMA segments
//*----------------------------------------------------------------------------
enum D940E_MAGICV_DMA_IntSegId{
	DMA_PM = 0x1, //!< Program memory
#ifdef __magic_revB__	
	DMA_DM_INT8_UNSIGNED = 0x12, //!< Data memory (8bit integer with 0 extention)
	DMA_DM_INT16_UNSIGNED = 0x22, //!< Data memory (32bit integer with 0 extention)	
	DMA_DM_INT16_SIGNED = 0x32, //!< Data memory (16bit integer with sign extention)
	DMA_DM_INT32 = 0x2, //!< Data memory (32bit integer)
#endif
	DMA_DM_INT = 0x2, //!< Data memory (32bit integer)
	DMA_DM_FLOAT = 0x4, //!< Data memory (32bit floating point)
	DMA_DM_DOUBLE = 0x8 //!< Data memory (40bit floating point, represented as 64bit in ARM)
};


#ifdef __chess__

extern void D940F_MAGICV_DMA_Read(void chess_storage(EXT_DATA) * ext_buff,void *int_buff,unsigned int len)clobbers() property(loop_free);
extern void D940F_MAGICV_DMA_Write(void chess_storage(EXT_DATA) * ext_buff,void *int_buff,unsigned int len)clobbers() property(loop_free);

//! \brief Macro to write a long value to the console (only HW SIM)
//! \param y long: value to be written

#define outi(x,y) {volatile unsigned long tmp=y;_DBIOS_WriteExternalL((unsigned long __MAGICV_EXTERNAL *)((unsigned long)x),(unsigned long *)&tmp/*y*/,1);chess_separator(scheduler);}  
#define PRINT_BASE 0xffff0018

//! \brief Writes a mAgicV Flower register
//! \param reg int: address of the register to be written (see magicV_regs.h for reference)
//! \param val (int on ARM, int or long on mAgicV depending on register size: int for register sizes <= 16bit, long for registers sizes <= 32bit): value to be written

#define _DBIOS_WriteMagicReg(reg, val) \
			if (chess_manifest(reg==0)) {\
				{ _mreg_ctrl=(volatile unsigned long)(val);}\
			}else if (chess_manifest(reg==1)) {\
				{ _mreg_status=(volatile unsigned long)(val);}\
			}else if (chess_manifest(reg==2)) {\
				{ _mreg_mask=(volatile unsigned long)(val);}\
			}else if (chess_manifest(reg==12)) {\
				{ _mreg_step=(volatile unsigned long)(val);}\
			}else if (chess_manifest(reg==13)) {\
				{ _mreg_watch=(volatile unsigned long)(val);}\
			}else if (chess_manifest(reg==14)) {\
				{ _mreg_mutex=(volatile unsigned long)(val);}\
			}else if (chess_manifest(reg==101)) {\
				{ _mreg_dmaextcirc=(volatile unsigned long)(val);}\
			}else if (chess_manifest(reg==102)) {\
				{ _mreg_dmaextmod=(volatile unsigned long)(val);}\
			}else if (chess_manifest(reg==104)) {\
				{ _mreg_dmaintcirc=(volatile unsigned long)(val);}\
			}else if (chess_manifest(reg==105)) {\
				{  _mreg_dmaintmod=(volatile unsigned long)(val);}\
			}else if (chess_manifest(reg==107)) {\
				{ _mreg_dmaintseg=(volatile unsigned long)(val);}\
			}else if (chess_manifest(reg==111)) {\
				{ _mreg_dmactrl=(volatile unsigned long)(val);}\
			}else if (chess_manifest(reg==113)) {\
				{ _mreg_intsvr0=(volatile unsigned long)(val);}\
			}else if (chess_manifest(reg==114)) {\
				{ _mreg_intsvr1=(volatile unsigned long)(val);}\
			}else if (chess_manifest(reg==115)) {\
				{ _mreg_intsvr2=(volatile unsigned long)(val);}\
			}else if (chess_manifest(reg==116)) {\
				{ _mreg_intsvr3=(volatile unsigned long)(val);}\
			}else if (chess_manifest(reg==117)) {\
				{ _mreg_intsvr4=(volatile unsigned long)(val);}\
			}else if (chess_manifest(reg==118)) {\
				{ _mreg_intsvr5=(volatile unsigned long)(val);}\
			}else if (chess_manifest(reg==119)) {\
				{ _mreg_intsvr6=(volatile unsigned long)(val);}\
			}else if (chess_manifest(reg==120)) {\
				{ _mreg_intsvr7=(volatile unsigned long)(val);}\
			}else if (chess_manifest(reg==121)) {\
				{ _mreg_intmask=(volatile unsigned long)(val);}\
			}else if (chess_manifest(reg==124)) {\
				{ _mreg_intctrl=(volatile unsigned long)(val);}\
			}else if (chess_manifest(reg==125)) {\
				{ _mreg_wakectrl=(volatile unsigned long)(val);}\
			}else if (chess_manifest(reg==126)) {\
				{ _mreg_intsetreset=(volatile unsigned long)(val);}\
			}else if (chess_manifest(reg==127)) {\
				{ _mreg_intpriority=(volatile unsigned long)(val);}\
			}
			
//! \brief Reads a mAgicV Flower register
//! \param reg int: address of the register to be read (see magicV_regs.h for reference)
//! \param val (int on ARM, int or long on mAgicV depending on register size: int for register sizes <= 16bit, long for registers sizes <= 32bit): variable to fill with register value

#define _DBIOS_ReadMagicReg(reg, tmp) \
			if (chess_manifest(reg==0)) {\
				{ tmp=(unsigned long)_mreg_exception;}\
			}else if (chess_manifest(reg==1)) {\
				{ tmp=(unsigned long)_mreg_status;}\
			}else if (chess_manifest(reg==2)) {\
				{ tmp=(unsigned long)_mreg_mask;}\
			}else if (chess_manifest(reg==3)) {\
				{ tmp=(unsigned long)_mreg_exc;}\
			}else if (chess_manifest(reg==12)) {\
				{ tmp=(unsigned long)_mreg_step;}\
			}else if (chess_manifest(reg==13)) {\
				{ tmp=(unsigned int)_mreg_watch;}\
			}else if (chess_manifest(reg==14)) {\
				{ tmp=(unsigned long)_mreg_mutex;}\
			}else if (chess_manifest(reg==101)) {\
				{ tmp=(unsigned long)_mreg_dmaextcirc;}\
			}else if (chess_manifest(reg==102)) {\
				{ tmp=(unsigned long)_mreg_dmaextmod;}\
			}else if (chess_manifest(reg==104)) {\
				{ tmp=(unsigned long)_mreg_dmaintcirc;}\
			}else if (chess_manifest(reg==105)) {\
				{ tmp=(unsigned long)_mreg_dmaintmod;}\
			}else if (chess_manifest(reg==107)) {\
				{ tmp=(unsigned long)_mreg_dmaintseg;}\
			}else if (chess_manifest(reg==111)) {\
				{ tmp=(unsigned long)_mreg_dmastat;}\
			}else if (chess_manifest(reg==113)) {\
				{ tmp=(unsigned long)_mreg_intsvr0;}\
			}else if (chess_manifest(reg==114)) {\
				{ tmp=(unsigned long)_mreg_intsvr1;}\
			}else if (chess_manifest(reg==115)) {\
				{ tmp=(unsigned long)_mreg_intsvr2;}\
			}else if (chess_manifest(reg==116)) {\
				{ tmp=(unsigned long)_mreg_intsvr3;}\
			}else if (chess_manifest(reg==117)) {\
				{ tmp=(unsigned long)_mreg_intsvr4;}\
			}else if (chess_manifest(reg==118)) {\
				{ tmp=(unsigned long)_mreg_intsvr5;}\
			}else if (chess_manifest(reg==119)) {\
				{ tmp=(unsigned long)_mreg_intsvr6;}\
			}else if (chess_manifest(reg==120)) {\
				{ tmp=(unsigned long)_mreg_intsvr7;}\
			}else if (chess_manifest(reg==121)) {\
				{ tmp=(unsigned long)_mreg_intmask; }\
			}else if (chess_manifest(reg==122)) {\
				{ tmp=(unsigned long)_mreg_intstat;}\
			}else if (chess_manifest(reg==123)) {\
				{ tmp=(unsigned long)_mreg_intgstat;}\
			}else if (chess_manifest(reg==125)) {\
				{ tmp=(unsigned long)_mreg_wakestatus; }\
			}else if (chess_manifest(reg==126)) {\
				{ tmp=(unsigned long)_mreg_intreturn;}\
			}else if (chess_manifest(reg==127)) {\
				{ tmp=(unsigned long)_mreg_intpriority; }\
			}

//! \brief Writes a peripheral register mapped on AMBA
//! \param periph_buff_ptr (\<peripheral struct\> *) pointer to the peripheral
//! \param periph_name (unquoted string) peripheral name (the name of the peripheral structure) (see libV3.h for reference)
//! \param periph_reg (unquoted string) peripheral register name (the name of the field in the peripheral structure) (see libV3.h for reference)
//! \param val (long) value to be written

#define _DBIOS_WritePeripheralRegL(periph_buff_ptr,periph_name,periph_reg,val) \
		{\
			unsigned long reg=val;\
			D940S_##periph_name __MAGICV_EXTERNAL *pper=(D940S_##periph_name __MAGICV_EXTERNAL *)((unsigned long)periph_buff_ptr);\
			D940F_MAGICV_DMA_Write/*_WRITEEXT*/((unsigned long __MAGICV_EXTERNAL *)((unsigned long)(&(pper->periph_name##_##periph_reg))),(unsigned long *)&reg,1);\
			chess_separator(scheduler);\
		}

//! \brief Reads a peripheral register mapped on AMBA
//! \param periph_buff_ptr (\<peripheral struct\> *) pointer to the peripheral
//! \param periph_name (unquoted string) peripheral name (the name of the peripheral structure) (see libV3.h for reference)
//! \param periph_reg (unquoted string) peripheral register name (the name of the field in the peripheral structure) (see libV3.h for reference)
//! \param val_ptr (long *) pointer to the variables to be filled with the register value

#define _DBIOS_ReadPeripheralRegL(periph_buff_ptr,periph_name,periph_reg,val_ptr) \
		{\
			D940S_##periph_name __MAGICV_EXTERNAL *pper=(D940S_##periph_name __MAGICV_EXTERNAL *)((unsigned long)periph_buff_ptr);\
			D940F_MAGICV_DMA_Read/*_READEXT*/((unsigned long __MAGICV_EXTERNAL *)((unsigned long)(&(pper->periph_name##_##periph_reg))),(unsigned long *)(val_ptr),1);\
			chess_separator(scheduler);\
		}

//! \brief Writes a peripheral register array mapped on AMBA
//! \param periph_buff_ptr (\<peripheral struct\> *) pointer to the peripheral
//! \param periph_name (unquoted string) peripheral name (the name of the peripheral structure) (see libV3.h for reference)
//! \param periph_reg (unquoted string) peripheral register name (the name of the field in the peripheral structure: it must be an array) (see libV3.h for reference)
//! \param index (int) first array index to write
//! \param leng (int) number of fields to write
//! \param val_ptr (long *) pointer to the buffer to write

#define _DBIOS_WritePeripheralRegArrayL(periph_buff_ptr,periph_name,periph_reg,index,leng,val_ptr) \
		{\
			D940S_##periph_name __MAGICV_EXTERNAL *pper=(D940S_##periph_name __MAGICV_EXTERNAL *)((unsigned long)periph_buff_ptr);\
			D940F_MAGICV_DMA_Write/*_WRITEEXT*/((unsigned long __MAGICV_EXTERNAL *)((unsigned long)(&(pper->periph_name##_##periph_reg[index]))),(unsigned long *)(val_ptr),leng);\
			chess_separator(scheduler);\
		}

//! \brief Reads a peripheral register array mapped on AMBA
//! \param periph_buff_ptr (\<peripheral struct\> *) pointer to the peripheral
//! \param periph_name (unquoted string) peripheral name (the name of the peripheral structure) (see libV3.h for reference)
//! \param periph_reg (unquoted string) peripheral register name (the name of the field in the peripheral structure: it must be an array) (see libV3.h for reference)
//! \param index (int) first array index to read
//! \param leng (int) number of fields to read
//! \param val_ptr (long *) pointer to the buffer to be filled with registers values

#define _DBIOS_ReadPeripheralRegArrayL(periph_buff_ptr,periph_name,periph_reg,index,leng,val_ptr) \
		{\
			D940S_##periph_name __MAGICV_EXTERNAL *pper=(D940S_##periph_name __MAGICV_EXTERNAL *)((unsigned long)periph_buff_ptr);\
			D940F_MAGICV_DMA_Read/*_READEXT*/((unsigned long __MAGICV_EXTERNAL *)((unsigned long)(&(pper->periph_name##_##periph_reg[index]))),(unsigned long *)(val_ptr),leng);\
			chess_separator(scheduler);\
		}

//! \brief Inserts a break on the code (MAGICV ONLY)

inline assembly void _DBIOS_BreakCode() property(loop_free) property(functional) clobbers() {
	asm_begin
	- : - - : - - : - - - -
	asm_end
}

//! \brief Copies one buffer on another. On mAgicV copies an internal memory buffer into an AMBA bus mapped buffer using a locking DMA
//! \param ext_buff ARM: destination buffer; mAgicV: destination buffer address mapped on AMBA
//! \param int_buff ARM: source buffer; mAgicV: source buffer address in internal memory
//! \param len buffer length

#define _DBIOS_WriteExternalL(ext_buff,int_buff,len) \
		{\
			_WRITEEXT(ext_buff,int_buff,len);\
			chess_separator(scheduler);\
		}

//! \brief Copies one buffer on another. On mAgicV copies an AMBA bus mapped buffer into an internal memory buffer using a locking DMA
//! \param ext_buff ARM: source buffer; mAgicV: source buffer address mapped on AMBA
//! \param int_buff ARM: destination buffer; mAgicV: destination buffer address in internal memory
//! \param len buffer length

#define _DBIOS_ReadExternalL(ext_buff,int_buff,len) \
		{\
			_READEXT(ext_buff,int_buff,len);\
			chess_separator(scheduler);\
		}


//! \brief DBIOS initialization
//! ARM: init Magic register addresses using the mmapped base address; mAgicV: Init DBIOS setting DMA channel 2 (DBIOS always use the same DMA channel), external circular address 0xffffff, internal circular address 0xffff, external modifier 4 (byte addressing), internal modifier 1, and segment
//! \param seg ARM: mmapped address of the Magic registers; mAgicV: segment in internal memory

inline void _DBIOS_Init(D940E_MAGICV_DMA_IntSegId seg) //!< Internal DMA segment
{
	chess_separator(scheduler);
	{_mreg_dmactrl=(unsigned long)(0x550);}
	{_mreg_dmaextcirc=(unsigned long)(0xffffff);}
	{_mreg_dmaextmod=(unsigned int)(4);}
	{_mreg_dmaintcirc=(unsigned int)(0xffff);}
	{_mreg_dmaintmod=(unsigned int)(1);}
	{_mreg_dmaintseg=(unsigned int)(seg);}
	_DBIOS_BreakCode();
	chess_separator(scheduler);
}


#else //ARM or mAgicV

//! \brief Writes a mAgicV Flower register (ARM implementation)
//! \param reg integer: mmapped address of the register to be written (see magicV_regs.h for reference)
//! \param val (int on ARM, int or long on mAgicV depending on register size: int for register sizes <= 16bit, long for registers sizes <= 32bit) value to be written

extern unsigned long *_DBIOS_mreg_mmapped_base;
extern unsigned long *_DBIOS_DM_I_mmapped_base;
extern unsigned long *_DBIOS_DM_F_mmapped_base;
extern int _DBIOS_ctrl_base;

#define _DBIOS_WriteMagicReg(reg, val) \
	{*(_DBIOS_mreg_mmapped_base+reg)=(val);}

//! \brief Reads a mAgicV Flower register (ARM implementation)
//! \param reg integer: mmapped address of the register to be read (see magicV_regs.h for reference)
//! \param val (int on ARM, int or long on mAgicV depending on register size: int for register sizes <= 16bit, long for registers sizes <= 32bit) variable to fill with register value

#define _DBIOS_ReadMagicReg(reg, val) \
	{val=*(_DBIOS_mreg_mmapped_base+reg);/* printf("0x%x uu\n",(int)(((unsigned int *)D940_MAGIC_REG_BASE)+reg));*/}

//! \brief Copies one buffer on another. On mAgicV copies an internal memory buffer into an AMBA bus mapped buffer using a locking DMA (ARM implementation)
//! \param ext_buff destination buffer address
//! \param int_buff source buffer address
//! \param len buffer length

#define _DBIOS_WriteExternalL(ext_buff,int_buff,len) \
		{\
memcpy(ext_buff,int_buff,len);\
		}

//! \brief Copies one buffer on another. On mAgicV copies an AMBA bus mapped buffer into an internal memory buffer using a locking DMA (ARM implementation)
//! \param ext_buff source buffer address
//! \param int_buff destination buffer address
//! \param len buffer length

#define _DBIOS_ReadExternalL(ext_buff,int_buff,len) \
		{\
memcpy(int_buff,ext_buff,len);\
		}

//! \brief Writes a peripheral register mapped on AMBA (ARM implementation)
//! \param periph_buff_ptr (\<peripheral struct\> *) pointer to the peripheral
//! \param periph_name (unquoted string) peripheral name (the name of the peripheral structure) (see libV3.h for reference)
//! \param periph_reg (unquoted string) peripheral register name (the name of the field in the peripheral structure) (see libV3.h for reference)
//! \param val (long) value to be written

#define _DBIOS_WritePeripheralRegL(periph_buff_ptr,periph_name,periph_reg,val) \
		periph_buff_ptr->periph_name##_##periph_reg = val;

//! \brief Reads a peripheral register mapped on AMBA (ARM implementation)
//! \param periph_buff_ptr (\<peripheral struct\> *) pointer to the peripheral
//! \param periph_name (unquoted string) peripheral name (the name of the peripheral structure) (see libV3.h for reference)
//! \param periph_reg (unquoted string) peripheral register name (the name of the field in the peripheral structure) (see libV3.h for reference)
//! \param val_ptr (long *) pointer to the variables to be filled with the register value

#define _DBIOS_ReadPeripheralRegL(periph_buff_ptr,periph_name,periph_reg,val_ptr) \
		*(val_ptr)=periph_buff_ptr->periph_name##_##periph_reg;

//! \brief Writes a peripheral register array mapped on AMBA (ARM implementation)
//! \param periph_buff_ptr (\<peripheral struct\> *) pointer to the peripheral
//! \param periph_name (unquoted string) peripheral name (the name of the peripheral structure) (see libV3.h for reference)
//! \param periph_reg (unquoted string) peripheral register name (the name of the field in the peripheral structure: it must be an array) (see libV3.h for reference)
//! \param index (int) first array index to write
//! \param leng (int) number of fields to write
//! \param val_ptr (long *) pointer to the buffer to write

#define _DBIOS_WritePeripheralRegArrayL(periph_buff_ptr,periph_name,periph_reg,index,leng,val_ptr) \
		{	int i;\
			for(i=0;i<leng;i++)\
				periph_buff_ptr->periph_name##_##periph_reg[index+i] = *((val_ptr)+i);}

//! \brief Reads a peripheral register array mapped on AMBA (ARM implementation)
//! \param periph_buff_ptr (\<peripheral struct\> *) pointer to the peripheral
//! \param periph_name (unquoted string) peripheral name (the name of the peripheral structure) (see libV3.h for reference)
//! \param periph_reg (unquoted string) peripheral register name (the name of the field in the peripheral structure: it must be an array) (see libV3.h for reference)
//! \param index (int) first array index to read
//! \param leng (int) number of fields to read
//! \param val_ptr (long *) pointer to the buffer to be filled with registers values

#define _DBIOS_ReadPeripheralRegArrayL(periph_buff_ptr,periph_name,periph_reg,index,leng,val_ptr) \
		{	int i;for(i=0;i<leng;i++)\
				*((val_ptr)+i)=periph_buff_ptr->periph_name##_##periph_reg[index+i];}


//! \brief DBIOS initialization (ARM implementation) init Magic registers mmapped base
//! \param seg (D940E_MAGICV_DMA_IntSegId) Internal DMA segment

#define _DBIOS_Init(base)\
	_DBIOS_mreg_mmapped_base = base;

#endif //ARM or mAgicV

#endif //_DBIOS_MACRO_
