#include "DSPAbstract_ResourceManager.h"

#ifdef __PC__
DA_long *__magic_malloc_ptr = 0;
#endif

void * DSPAbstractRM_AllocMemory(DA_int size){
#ifdef __PC__
	DA_long *ptr = __magic_malloc_ptr;
	if ((DA_int)(ptr + size) >= 16000){
		return 0;
	}
	__magic_malloc_ptr+=size;
	return ptr;
#endif
	return NULL;
}

DA_RETCODE DSPAbstractRM_FreeMemory(void *ptr){
	return DA_SUCCESS;
}

DA_RETCODE DSPAbstractRM_Init(){
	int reg, dmi, dmf;
	
#ifndef __PC__
	_DBIOS_ctrl_base = open("/dev/mAgicV/mAgicV.ctrl", O_RDWR | O_SYNC);
	reg = open("/dev/mAgicV/mAgicV.regs", O_RDWR | O_SYNC);
	dmi = open("/dev/mAgicV/mAgicV.dm_i", O_RDWR | O_SYNC);
	dmf = open("/dev/mAgicV/mAgicV.dm_f", O_RDWR | O_SYNC);
	if((_DBIOS_ctrl_base == -1) || (reg == -1) || (dmi == -1) || (dmf == -1))
	{
		fprintf(stderr, "Unable to open mAgicV device\n");
		return DA_ENOMAP;
	};
	_DBIOS_DM_I_mmapped_base = (unsigned long *)mmap((void*) MAGICV_DMI_START, 0x10000, PROT_READ | PROT_WRITE, MAP_FIXED | MAP_SHARED, dmi, 0);
	_DBIOS_DM_F_mmapped_base = (unsigned long *)mmap((void*) MAGICV_DMF_START, 0x10000, PROT_READ | PROT_WRITE, MAP_FIXED | MAP_SHARED, dmf, 0);
	_DBIOS_mreg_mmapped_base = (unsigned long *)mmap((void*) MAGICV_REGS_START, 0x1000, PROT_READ | PROT_WRITE, MAP_FIXED | MAP_SHARED, reg, 0);
	if((_DBIOS_DM_I_mmapped_base == MAP_FAILED) || (_DBIOS_DM_F_mmapped_base == MAP_FAILED) || (_DBIOS_mreg_mmapped_base == MAP_FAILED))
	{
		fprintf(stderr, "mAgicV mmap failed: %d\n",errno);
		return DA_ENOMAP;
	}
	//reset mutexes
	_DBIOS_mreg_mmapped_base[D940_REG_MGCMUTEXCTL] = 0xffff0000;
#else
	_DBIOS_DM_I_mmapped_base = (unsigned long *)calloc(0x10000,sizeof(long));
	_DBIOS_DM_F_mmapped_base = (unsigned long *)calloc(0x10000,sizeof(long));
	_DBIOS_mreg_mmapped_base = (unsigned long *)calloc(0x1000,sizeof(long));
	_DBIOS_DM_I_mmapped_base[16003] = 66;
	DA_TLTasks *p = (DA_TLTasks *)_DBIOS_DM_I_mmapped_base[66];
	p->lAddress = 100;
	p->iIndex = 0;
	p->iMutex = 0;
	p->hQueueSend = xQueueCreate(4, 2);
	p->hQueueRecv = xQueueCreate(4, 2);

#endif
	return DA_SUCCESS;
}

