#ifndef _BOOTLOADER_H_
#define _BOOTLOADER_H_

#include <stdint.h>
#include "stm32l1xx_flash.h"

#define RECOPY_COUNT_MAX            (3)

#define BOOTLOADER_START_ADDRESS    ((uint32_t)0x08000000)
#define BOOTLOADER_END_ADDRESS      ((uint32_t)0x08003F00)
#define BUFFER_START_ADDRESS        ((uint32_t)0x08022000)
#define BUFFER_END_ADDRESS          ((uint32_t)0x08026000)

void UpdateBootloader(uint32_t crc);
FLASH_Status EnableBootloaderWriteProtection(void);

#endif // End of #define _COMMON_H_
