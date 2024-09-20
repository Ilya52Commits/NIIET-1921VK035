#include "fw_updater.h"

// write new FW to main flash
RAMFUNC void write_new_fw(void)
{
	uint32_t data[2];
	uint32_t address_from = UPDATER_FLASH_REGION_START, address_to = 0x0;
	int i;
	
	__disable_irq();
	while (address_to < UPDATER_FLASH_REGION_START)
	{
		// erase new page of main FW flash
		if (address_to % FLASH_PAGE_SIZE_BYTES == 0)
			MFLASH_ErasePage(address_to, MFLASH_Region_Main);
			
		// update 8 bytes of data
		MFLASH_ReadData(address_from, data, MFLASH_Region_Main);	
		MFLASH_WriteData(address_to, data, MFLASH_Region_Main);
		address_to += 8;
		address_from += 8;
	}
	
	// reset system
	for (i = 0; i < 1000; i++)	{}
	NVIC_SystemReset();
	while(1);
}
