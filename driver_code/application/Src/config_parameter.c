
/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "main.h"

#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000)
#define FLASH_SECTOR_11    11U
	
void write_config_par(void *data, uint32_t lens)
{
	uint32_t SectorError = 0,tmp_data;;
	uint32_t offset_addr =0;
	static FLASH_EraseInitTypeDef EraseInitStruct;

	HAL_FLASH_Unlock();
	/* Fill EraseInit structure*/
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = FLASH_SECTOR_11;
  EraseInitStruct.NbSectors = 1;
  HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
	__HAL_FLASH_DATA_CACHE_DISABLE();
  __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
  __HAL_FLASH_DATA_CACHE_RESET();
  __HAL_FLASH_INSTRUCTION_CACHE_RESET();
  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  __HAL_FLASH_DATA_CACHE_ENABLE();

	while (offset_addr<lens)
  {
		tmp_data = *((uint32_t *)data+offset_addr/4);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADDR_FLASH_SECTOR_11+offset_addr, tmp_data);
    offset_addr = offset_addr + 4;
  }
	HAL_FLASH_Lock(); 
}
	
	
void read_config_par(void *data, uint32_t lens)
{
	memcpy(data,(uint32_t *)ADDR_FLASH_SECTOR_11,lens);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
