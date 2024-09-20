#ifndef _FW_UPDATER_H
#define _FW_UPDATER_H

//--- Includes -----------------------------------------------------------------
#include <stdio.h>
#include "K1921VK035.h"
#include "plib035.h"
#include "ComDef.h"

//--- Defines ------------------------------------------------------------------

// password to start FW update
#define FW_UPDATE_PASSWORD			(0xDEAD) // 57005 decimal
#define FLASH_BUFFER_SIZE			(128)
#define FLASH_PAGE_SIZE_BYTES		(1024UL)
#define FLASH_PAGE_SIZE_BYTES_LOG2 	(10UL)
#define FLASH_PAGE_TOTAL			(64UL)
#define FLASH_TOTAL_BYTES			(FLASH_PAGE_SIZE_BYTES * FLASH_PAGE_TOTAL)
#define FLASH_BUFFER_SIZE_LOG2		(7)
#define BUFFERS_IN_PAGE				(FLASH_PAGE_SIZE_BYTES / FLASH_BUFFER_SIZE)

// second half of flash memory
#define UPDATER_FLASH_REGION_START		(FLASH_TOTAL_BYTES >> 1)

#define RAMFUNC         __attribute__((section("RAMCODE")))

//--- Functions ----------------------------------------------------------------

// initialize FW updater
void perform_fw_update(void);

// erase updater page
void erase_updater_page(uint8_t);

// write new updater buffer to page
void write_buffer_to_page(uint8_t, uint8_t);

void read_buffer_from_page(uint8_t, uint8_t);

#endif //_FW_UPDATER_H
