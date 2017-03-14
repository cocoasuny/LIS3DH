#ifndef _M95M01_H_
#define _M95M01_H_

#include <stdint.h>
#include <stdbool.h>
#include "platform.h"

#define EEPROM_TIMEOUT_CNT				    (2000)
/* M95M01 memory page size */
#define M95M01_PAGE_SPACE                   256             // Page space

/* M95M01 memory allocation */
#define CONTACT_TEMP_VALUE_ID               0               // Contact temperature data 
#define NON_CONTACT_TEMP_VALUE_ID           1               // Non-contact temperature(IR)
#define STEP_COUNT_VALUE_ID                 2               // Counting steps
#define CONFIG_FILE_ID                      3               // Configuration files
#define MONITOR_MODEL_VALUE_ID              4               // From monitor model

/* M95M01 memory Boundary*/
#define M95M01_CONTACT_TEMP_OFFSET          256*1
#define M95M01_NON_CONTACT_TEMP_OFFSET      256*2
#define M95M01_STEP_COUNT_OFFSET            256*3
#define M95M01_CONFIG_FILE_OFFSET           256*4
#define M95M01_MONITOR_MODEL_OFFSET         256*5
#define M95M01_ALL_SPACE                    256*512

/* Offset in the Config file */
/* Config Factory Info : 0:Flag of App active or not;
						 1:Flag of App Modify SN or not; 
						 2~5:App Modify SN Info   
						 6:  Moniter Template ID   
*/
#define M95M01_CONFIG_FACTORYINFO           M95M01_CONFIG_FILE_OFFSET + 20   //data length 20
#define M95M01_CONFIG_CPUINFO               M95M01_CONFIG_FILE_OFFSET + 50   //data length 57

#define M95M01_CONFIG_UPGRADE_FLAG          M95M01_CONFIG_FILE_OFFSET + 240
#define M95M01_CONFIG_FW_RX_FLAG            M95M01_CONFIG_FILE_OFFSET + 241
#define M95M01_CONFIG_FW_TYPE_FLAG          M95M01_CONFIG_FILE_OFFSET + 246

/* EEPROM SPI define */
#define RCC_AHBPeriph_EEPROM_CSGPIO         PT_RCC_AHBPeriph_EEPROM_CSGPIO
#define GPIO_PORT_EEPROM_CS                 PT_GPIO_PORT_EEPROM_CS
#define GPIO_Pin_EEPROM_CS                  PT_GPIO_Pin_EEPROM_CS

/* M95M01 EEPROM Commands */
#define M95M01_WRSR                         0x01            // Write Status Register
#define M95M01_WRITE                        0x02            // Write to Memory Array
#define M95M01_READ                         0x03            // Read from Memory Array
#define M95M01_WRDI                         0x04            // Write Disable
#define M95M01_RDSR                         0x05            // Read Status Register
#define M95M01_WREN                         0x06            // Write Enable
#define M95M01_READ_PAGE                    0x83            // Reads the page dedicated to identification
#define M95M01_WRITE_PAGE                   0x82            // Writes the page dedicated to identification

/* M95M01 EEPROM status register */
#define M95M01_WIP_STATUS                   0x01            // Indicates that the memory is busy

/**/
/* M95M01 EEPROM status register */
#define M95M01_CAPACITY_SPACE               0x00            // Indicates how many bytes available to be writed 
#define M95M01_IS_BUSY                      0x01            // Indicates that the memory is busy
#define M95M01_READ_AVAILABLE_SPACE         0x02            // Indicates how many bytes should be read out 

/* M95M01 context in configure file */
#define M95M01_FWDATA_OK                    0xaa            // Firmware is OK
#define M95M01_FWDATA_ERROR                 0xff            // Firmware is not OK
#define M95M01_NO_UPGRADE                   0xff            // no upgrade
#define M95M01_UPGRADE                      0x55            // upgrade
#define M95M01_BOOT_FIRMWARE                0x01            // Bootloader firmware
#define M95M01_APP_FIRMWARE                 0x02            // APP firmware
#define M95M01_NO_NEW_FIRMWARE              0xff            // NO new firmware

/* M95M01 context in configure file */
#define M95M01_APP_ACTIVE                   0x55            // APP is active
#define M95M01_APP_INACTIVE                 0xAA            // APP is inactive
#define M95M01_APP_MODIFYSN                 0x01            // APP is to modify the SN
#define M95M01_APP_MODIFIEDSN               0x02            // APP has modified the SN

typedef struct {
    uint32_t pWrite_m95m01;
    uint32_t pRead_m95m01;
} memory_info_t;

typedef struct {
    memory_info_t   memory_info_contact_temp;
    memory_info_t   memory_info_non_contact_temp;
    memory_info_t   memory_info_steps;
    memory_info_t   memory_info_config_file;
    memory_info_t   memory_info_monitor;
} m95m01_info_t;

extern m95m01_info_t m95m01_info;

void EEPROM_SPI_Configuration(void);
uint8_t EEPROM_SPI_ReadStatusRegister(void);
void EEPROM_SPI_WriteEnable(void);
void EEPROM_SPI_WriteDisable(void);
void EEPROM_SPI_ReadFromMemory(uint8_t *pDestStr,uint32_t pStartAddr,uint16_t len);
void EEPROM_SPI_WriteToMemory(uint8_t *pSrcStr,uint32_t pStartAddr,uint16_t len);

bool M95M01_Write(uint8_t data_id,uint8_t len,uint8_t * ptr);
bool M95M01_Read(uint8_t data_id,uint8_t len,uint8_t * ptr);
void M95M01_ReadOK(uint8_t data_id,uint16_t len,uint8_t state);
uint32_t GetM95M01State(uint8_t data_id,uint8_t getType);
void M95M01_CounterUpdate(void);
void M95M01_CounterCheckOut(void);
void ClearM95M01(uint16_t page_addr);
void EraseM95M01(uint16_t page_addr);
void M95M01HW_SelfTest(void);

#endif  // End of _M95M01_H_
