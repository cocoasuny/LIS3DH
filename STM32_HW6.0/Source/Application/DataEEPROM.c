#include "DataEEPROM.h"
#include "stm32l1xx.h"
#include <stdio.h>
#include "common.h"

/*******************************************************************************
* @brief   Internal EEPROM Write Init
* @param   None
* @param   None
* @retval  None
*******************************************************************************/
void DataEEPROM_IF_Init(void)
{
    /* Unlock the Internal EEPROM memory */
    DATA_EEPROM_Unlock();
    
    /* Clear all FLASH flags */  
    FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
              | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);  
}

/*******************************************************************************
* @brief   Write a data of temp to the data EEPROM area
* @param   Address:the destion
* @param   temp:the data to write
* @retval  FLASH_Status
*******************************************************************************/
FLASH_Status DataEEPROM_ByteWrite(uint32_t Address, uint8_t temp)
{
	FLASH_Status status = FLASH_COMPLETE;

	/* Check parameters */
	usr_para_check((Address >= DATA_EEPROM_START_ADDR) && (Address <= DATA_EEPROM_END_ADDR));
	
	DataEEPROM_IF_Init();
	
	status = DATA_EEPROM_EraseByte(Address);
	if(status != FLASH_COMPLETE)
	{
		return (status);
	}
	
	status = DATA_EEPROM_FastProgramByte(Address, temp);
	if(status != FLASH_COMPLETE)
	{
		return (status);
	}
	
	DATA_EEPROM_Lock();

	return (status);
}

/*******************************************************************************
* @brief   Read a data from the data EEPROM area
* @param   Address:the destion
* @param   temp:the data to write
* @retval  FLASH_Status
*******************************************************************************/
FLASH_Status DataEEPROM_ByteRead(uint32_t Address, uint8_t * p_Dst)
{
	FLASH_Status status = FLASH_COMPLETE;

	/* Check parameters */
	usr_para_check((Address >= DATA_EEPROM_START_ADDR) && (Address <= DATA_EEPROM_END_ADDR));

	*p_Dst = *(__IO uint32_t*)Address;

	return (status);
}

/*******************************************************************************
* @brief   Write size*1 bytes to the data EEPROM area
* @param   Address:the destion
* @param   temp:the data to write
* @retval  FLASH_Status
*******************************************************************************/
FLASH_Status DataEEPROM_MultiByteWrite(uint8_t * p_src, uint32_t Address, uint16_t size)
{
	FLASH_Status status = FLASH_ERROR_WRP;
	uint8_t i;

	/* Check parameters */
	usr_para_check((Address >= DATA_EEPROM_START_ADDR) && (Address <= DATA_EEPROM_END_ADDR));
	usr_para_check(p_src != NULL);
	
	DataEEPROM_IF_Init();

	for(i = 0;i < size;i ++)
	{
		status = DATA_EEPROM_EraseByte(Address + i);
		if(status != FLASH_COMPLETE)
		{
			return (status);
		}
		
		status = DATA_EEPROM_FastProgramByte(Address + i, *(p_src + i));
		if(status != FLASH_COMPLETE)
		{
			return (status);
		}
	}
	
	DATA_EEPROM_Lock();
   
	return (status);
}

/*******************************************************************************
* @brief   Read size*1 data from the data EEPROM area
* @param   Address:the destion
* @param   temp:the data to write
* @retval  FLASH_Status
*******************************************************************************/
FLASH_Status DataEEPROM_MultiByteRead(uint8_t * p_Dst, uint32_t Address, uint16_t size)
{
	FLASH_Status status = FLASH_COMPLETE;
	uint8_t i;

	/* Check parameters */
	usr_para_check((Address >= DATA_EEPROM_START_ADDR) && (Address <= DATA_EEPROM_END_ADDR));
	usr_para_check(p_Dst != NULL);
	for(i = 0;i < size;i ++)
	{
		*(p_Dst + i) = *(__IO uint32_t*)(Address + i);
	}

	return (status);
}


