#include "string.h"
#include "fw_updater.h"
#include "..\modbus\mb.h"

extern void systick_init(void);
extern uint8_t mb_address;
extern uint16_t system_status, command_status;

extern unsigned char mb_discretes[];
extern uint16_t mb_holding_regs[];
extern unsigned char mb_coils[];

extern void write_new_fw(void);

// main FW updater code
void perform_fw_update(void)
{
	// cycle infinitely
	while (command_status & FW_UPDATE_MODE)
	{
		// process modbus data exchange
		if (system_status & STATE_MB_POLL)
		{
			system_status &= ~STATE_MB_POLL;
			eMBPoll();
		}
		
		// erase page command
		if ((command_status & FW_UPDATE_ERASE_PAGE)&&(command_status & FW_UPDATE_MODE))
		{
			command_status &= ~FW_UPDATE_ERASE_PAGE;
			mb_coils[FW_UPDATE_IDX] &= ~(FW_UPDATE_PAGE_ERASED >> FW_UPDATE_POS);

			// flag mem operation start
			mb_discretes[FW_UPDATE_IDX] &= ~(FW_UPDATE_PAGE_ERASED >> FW_UPDATE_POS);

			erase_updater_page(mb_holding_regs[FW_UPDATE_PAGE_IN] & 0xFF);
		}
		
		// write buf to page command
		if ((command_status & FW_UPDATE_WRITE_BUF)&&(command_status & FW_UPDATE_MODE))
		{
			command_status &= ~FW_UPDATE_WRITE_BUF;
			mb_coils[FW_UPDATE_IDX] &= ~(FW_UPDATE_WRITE_BUF >> FW_UPDATE_POS);
			
			// flag mem operation start
			mb_discretes[FW_UPDATE_IDX] &= ~(FW_UPDATE_BUF_WRITTEN >> FW_UPDATE_POS);
			
			write_buffer_to_page(mb_holding_regs[FW_UPDATE_PAGE_IN] & 0xFF, mb_holding_regs[FW_UPDATE_PAGEBUF_IN] & 0xFF);
		}
		
		// read buf from page command
		if ((command_status & FW_UPDATE_READ_BUF)&&(command_status & FW_UPDATE_MODE))
		{
			command_status &= ~FW_UPDATE_READ_BUF;
			mb_coils[FW_UPDATE_IDX] &= ~(FW_UPDATE_READ_BUF >> FW_UPDATE_POS);
			
			// flag mem operation start
			mb_discretes[FW_UPDATE_IDX] &= ~(FW_UPDATE_BUF_READ >> FW_UPDATE_POS);
			
			read_buffer_from_page(mb_holding_regs[FW_UPDATE_PAGE_IN] & 0xFF, mb_holding_regs[FW_UPDATE_PAGEBUF_IN] & 0xFF);
		}
		
		// update FW flash
		if ((command_status & FW_UPDATE_WRITE_NEW)&&(command_status & FW_UPDATE_MODE))
		{
			command_status &= ~FW_UPDATE_WRITE_NEW;
			mb_coils[FW_UPDATE_IDX] &= ~(FW_UPDATE_WRITE_NEW >> FW_UPDATE_POS);
			write_new_fw();
		}
	}
}

//-----------------------------------------------------------------------------------------------------------------

// erase updater page (page_idx = 0...(FLASH_PAGE_TOTAL / 2 - 1))
void erase_updater_page(uint8_t page_idx)
{
	// page index error
	if (page_idx > (FLASH_PAGE_TOTAL >> 1) - 1)
		return;

	// erase current page
	__disable_irq();
	MFLASH_ErasePage(UPDATER_FLASH_REGION_START + (page_idx << FLASH_PAGE_SIZE_BYTES_LOG2), MFLASH_Region_Main);
	__enable_irq();
	
	// flag mem operation end
	mb_discretes[FW_UPDATE_IDX] |= (FW_UPDATE_PAGE_ERASED >> FW_UPDATE_POS);
}

//-----------------------------------------------------------------------------------------------------------------

// write new updater buffer to page (page_idx = 0...(FLASH_PAGE_TOTAL / 2 - 1), buf_idx = 0...BUFFERS_IN_PAGE-1)
void write_buffer_to_page(uint8_t page_idx, uint8_t buf_idx)
{
	uint8_t i;
	uint32_t data[2];
	uint32_t address;
	
	// size index errors
	if ((buf_idx > BUFFERS_IN_PAGE - 1) || (page_idx > (FLASH_PAGE_TOTAL >> 1) - 1))
		return;
	
	// current address to write
	address = UPDATER_FLASH_REGION_START + (page_idx << FLASH_PAGE_SIZE_BYTES_LOG2) + (buf_idx << FLASH_BUFFER_SIZE_LOG2);

	// write buffer to selected page (flash bus width 8 = 2 x 4 bytes per write operation)
	__disable_irq();
	for (i = 0; i < (FLASH_BUFFER_SIZE >> 3); i++) // 8 bytes per write -> (FLASH_BUFFER_SIZE / 8) write ops
	{
		memcpy((uint8_t*)&data[0], (uint8_t*)&mb_holding_regs[FW_UPDATE_BUFFER_START + (i<<2)], 8);
		MFLASH_WriteData(address, data, MFLASH_Region_Main);	
		address += 0x08;		
	}
	memset((uint8_t*)&mb_holding_regs[FW_UPDATE_BUFFER_START], 0, 128);
	__enable_irq (); 
	
	// flag mem operation end
	mb_discretes[FW_UPDATE_IDX] |= (FW_UPDATE_BUF_WRITTEN >> FW_UPDATE_POS);
}

//-----------------------------------------------------------------------------------------------------------------

// read updater buffer to registers (page_idx = 0...(FLASH_PAGE_TOTAL / 2 - 1), buf_idx = 0...BUFFERS_IN_PAGE-1)
void read_buffer_from_page(uint8_t page_idx, uint8_t buf_idx)
{
	uint8_t i;
	uint32_t data[2];
	uint32_t address;
	
	// size index errors
	if ((buf_idx > BUFFERS_IN_PAGE - 1) || (page_idx > (FLASH_PAGE_TOTAL >> 1) - 1))
		return;
	
	// current address to write
	address = UPDATER_FLASH_REGION_START + (page_idx << FLASH_PAGE_SIZE_BYTES_LOG2) + (buf_idx << FLASH_BUFFER_SIZE_LOG2);

	// write buffer to selected page (flash bus width 8 = 2 x 4 bytes per write operation)
	__disable_irq();
	for (i = 0; i < (FLASH_BUFFER_SIZE >> 3); i++) // 8 bytes per write -> (FLASH_BUFFER_SIZE / 8) read ops
	{
		MFLASH_ReadData(address, data, MFLASH_Region_Main);	
		memcpy((uint8_t*)&mb_holding_regs[FW_UPDATE_BUFFER_START + (i<<2)], (uint8_t*)&data[0], 8);
		address += 0x08;		
	}		
	__enable_irq (); 
	
	// flag mem operation end
	mb_discretes[FW_UPDATE_IDX] |= (FW_UPDATE_BUF_READ >> FW_UPDATE_POS);
}

//-----------------------------------------------------------------------------------------------------------------
