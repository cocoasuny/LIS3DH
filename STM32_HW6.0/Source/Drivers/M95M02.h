#ifndef _M95M02_H_
#define _M95M02_H_

#include <stdint.h>
#include <stdbool.h>
#include "platform.h"

#define EEPROM_TIMEOUT_CNT				(2000)
/* M95M02 memory page size */
#define M95M02_PAGE_SPACE                   256             // Page space

/* M95M02 memory allocation */
#define CONTACT_TEMP_VALUE_ID               0               // Contact temperature data 
#define NON_CONTACT_TEMP_VALUE_ID           1               // Non-contact temperature(IR)
#define STEP_COUNT_VALUE_ID                 2               // Counting steps
#define CONFIG_FILE_ID                      3               // Configuration files
#define MONITOR_MODEL_VALUE_ID              4               // From monitor model

/* M95M02 memory Boundary*/
#define M95M02_CONTACT_TEMP_OFFSET          256*1
#define M95M02_NON_CONTACT_TEMP_OFFSET      256*2
#define M95M02_STEP_COUNT_OFFSET            256*3
#define M95M02_CONFIG_FILE_OFFSET           256*4
#define M95M02_MONITOR_MODEL_OFFSET         256*5
#define M95M02_ALL_SPACE                    256*1024

/* EEPROM SPI define */
#define RCC_AHBPeriph_EEPROM_CSGPIO         PT_RCC_AHBPeriph_EEPROM_CSGPIO
#define GPIO_PORT_EEPROM_CS                 PT_GPIO_PORT_EEPROM_CS
#define GPIO_Pin_EEPROM_CS                  PT_GPIO_Pin_EEPROM_CS

/* M95M02 EEPROM Commands */
#define M95M02_WRSR                         0x01            // Write Status Register
#define M95M02_WRITE                        0x02            // Write to Memory Array
#define M95M02_READ                         0x03            // Read from Memory Array
#define M95M02_WRDI                         0x04            // Write Disable
#define M95M02_RDSR                         0x05            // Read Status Register
#define M95M02_WREN                         0x06            // Write Enable
#define M95M02_READ_PAGE                    0x83            // Reads the page dedicated to identification
#define M95M02_WRITE_PAGE                   0x82            // Writes the page dedicated to identification

/* M95M02 EEPROM status register */
#define M95M02_WIP_STATUS                   0x01            // Indicates that the memory is busy

/**/
/* M95M02 EEPROM status register */
#define M95M02_CAPACITY_SPACE               0x00            // Indicates how many bytes available to be writed 
#define M95M02_IS_BUSY                      0x01            // Indicates that the memory is busy
#define M95M02_READ_AVAILABLE_SPACE         0x02            // Indicates how many bytes should be read out 

typedef struct {
    uint32_t pWrite_m95m02;
    uint32_t pRead_m95m02;
} memory_info_t;

typedef struct {
    memory_info_t   memory_info_contact_temp;
    memory_info_t   memory_info_non_contact_temp;
    memory_info_t   memory_info_steps;
    memory_info_t   memory_info_config_file;
    memory_info_t   memory_info_monitor;
} m95m02_info_t;

extern m95m02_info_t m95m02_info;

void EEPROM_SPI_Configuration(void);
uint8_t EEPROM_SPI_ReadStatusRegister(void);
void EEPROM_SPI_WriteEnable(void);
void EEPROM_SPI_WriteDisable(void);
void EEPROM_SPI_ReadFromMemory(uint8_t *pDestStr,uint32_t pStartAddr,uint16_t len);
void EEPROM_SPI_WriteToMemory(uint8_t *pSrcStr,uint32_t pStartAddr,uint16_t len);

bool M95M02_Write(uint8_t data_id,uint8_t len,uint8_t * ptr);
bool M95M02_Read(uint8_t data_id,uint8_t len,uint8_t * ptr);
void M95M02_ReadOK(uint8_t data_id,uint8_t len,uint8_t state);
uint32_t GetM95M02State(uint8_t data_id,uint8_t getType);
void M95M02_CounterUpdate(void);
void M95M02_CounterCheckOut(void);
void ClearM95M02(uint8_t page_addr);

#endif  // End of _M95M02_H_
