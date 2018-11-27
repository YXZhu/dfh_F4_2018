#include "flash.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#define FLASH_USER_START_ADDR   ADDR_FLASH_PAGE_60   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_PAGE_62 + FLASH_PAGE_SIZE   /* End @ of user Flash area */

uint32_t Address = 0, PAGEError = 0;
//__IO uint32_t data32 = 0 , MemoryProgramStatus = 0;

static FLASH_EraseInitTypeDef EraseInitStruct;

void flashinit(void)
{
	HAL_FLASH_Unlock();
	  /* Erase the user Flash area
    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
  EraseInitStruct.NbPages     = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
  {
	  
  }
}

void flashwrite1(int16_t *data,int16_t t)
{
	int16_t tt;
	//HAL_FLASH_Unlock();
	flashinit();
	 Address = FLASH_USER_START_ADDR;	
	for(tt=0;tt<t;tt++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address,data[tt]);
		Address += 2;
	}
	HAL_FLASH_Lock();
}

void flashread1(int16_t *data,int16_t t)
{
	int16_t t1;
   Address = FLASH_USER_START_ADDR;
	for(t1=0;t1<t;t1++)
	{
		data[t1] = *(__IO int16_t *)Address;
		Address += 2;
	}
}

