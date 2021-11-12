/* includes */
#include "HALFlash.h"
//#include "hw_config.h"

//------------------------------------------------------------------------------
//     				===== FLASH_Write =====
//             		:사용자가 지정된 주소의 flash에 지정한 데이타를 저장한다.
//------------------------------------------------------------------------------

void Flash_Erase(uint32_t Address)
{
  
  uint32_t PageError;
  
  HAL_FLASH_Unlock();
  
  static FLASH_EraseInitTypeDef EraseinitStruct;

  EraseinitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  
  EraseinitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;

  EraseinitStruct.Sector = Address;//FLASH_USER_START_ADDR;
  
  EraseinitStruct.NbSectors = 1;//(FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;
    
  //HAL_FLASHEx_Erase(&EraseinitStruct, &PageError); 
  if(HAL_FLASHEx_Erase(&EraseinitStruct, &PageError) != HAL_OK)
  { 
    uint32_t errorcode = HAL_FLASH_GetError();
    //return HAL_ERROR;
  }
  
  /* Clear cache for flash */
  __HAL_FLASH_DATA_CACHE_DISABLE();
  __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

  __HAL_FLASH_DATA_CACHE_RESET();
  __HAL_FLASH_INSTRUCTION_CACHE_RESET();

  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  __HAL_FLASH_DATA_CACHE_ENABLE();
  
  /* Lock flash control register */
  HAL_FLASH_Lock();
}

void Flash_Write(unsigned int Address, unsigned long long Data)
{
  HAL_FLASH_Unlock();
  
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, Data); 
  
  HAL_FLASH_Lock();
}

//------------------------------------------------------------------------------
//     				===== FLASH_Read =====
//             		:사용자가 지정된 주소의 데이타를 읽어봐 반환한다.
//------------------------------------------------------------------------------
int Flash_Read(uint32_t Address)
{   
    return (*(__IO uint32_t*)Address);						// Return data from data register
}
