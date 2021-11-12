#ifndef __FLASH_H
#define __FLASH_H

/* includes */
#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"

#define Init_Add        0x08020000 //sector 5
#define Init_Add_2      0x08040000 //sector 6
#define Init_Add_3      0x08060000 //sector 7
/* functions */
void Flash_Write(unsigned int Address, unsigned long long Data);		// FLASH_Write DATA
int Flash_Read(uint32_t Address);				                // FLASH_read  DATA 
void Flash_Erase(uint32_t Address);
#endif  