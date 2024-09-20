#ifndef BOOT_CONF_H
#define BOOT_CONF_H

//--- Includes -----------------------------------------------------------------
#include <stdio.h>
#include "K1921VK035.h"

//--- Defines ------------------------------------------------------------------
//Global configuration
#define GLOBMEM(ADDR)   *(volatile uint32_t*)(ADDR)
#define RAMFUNC         __attribute__((section("RAMCODE")))

#endif //BOOT_CONF_H
