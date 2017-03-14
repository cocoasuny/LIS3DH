#ifndef _DATAEEPROM_H_
#define _DATAEEPROM_H_

#include "stm32l1xx.h"

#define DATA_EEPROM_START_ADDR     0x08080000
#define DATA_EEPROM_END_ADDR       0x080803FF

void DataEEPROM_IF_Init(void);
FLASH_Status DataEEPROM_ByteWrite(uint32_t Address, uint8_t temp);
FLASH_Status DataEEPROM_ByteRead(uint32_t Address, uint8_t * p_Dst);
FLASH_Status DataEEPROM_MultiByteWrite(uint8_t * p_src,uint32_t Address, uint16_t size);
FLASH_Status DataEEPROM_MultiByteRead(uint8_t * p_Dst, uint32_t Address, uint16_t size);

#endif

