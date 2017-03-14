#ifndef __SPI_H
#define __SPI_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include <stdio.h>
#include "TS_Interface.h"

#define SPI_OP_TIMEOUT			1000


/**@brief Category IDs for iOS notifications. */
typedef enum
{
    BLE_ANCS_CATEGORY_ID_OTHER,
    BLE_ANCS_CATEGORY_ID_INCOMING_CALL,
    BLE_ANCS_CATEGORY_ID_MISSED_CALL,
    BLE_ANCS_CATEGORY_ID_VOICE_MAIL,
    BLE_ANCS_CATEGORY_ID_SOCIAL,
    BLE_ANCS_CATEGORY_ID_SCHEDULE,
    BLE_ANCS_CATEGORY_ID_EMAIL,
    BLE_ANCS_CATEGORY_ID_NEWS,
    BLE_ANCS_CATEGORY_ID_HEALTH_AND_FITNESS,
    BLE_ANCS_CATEGORY_ID_BUSINESS_AND_FINANCE,
    BLE_ANCS_CATEGORY_ID_LOCATION,
    BLE_ANCS_CATEGORY_ID_ENTERTAINMENT
} ble_ancs_category_id_values_t;

/**@brief Event IDs for iOS notifications. */
typedef enum
{
    BLE_ANCS_EVENT_ID_NOTIFICATION_ADDED,
    BLE_ANCS_EVENT_ID_NOTIFICATION_MODIFIED,
    BLE_ANCS_EVENT_ID_NOTIFICATION_REMOVED
} ble_ancs_event_id_values_t;

#define RxDataSize           (270)
#define SPI_RX_Buf_MaxLen    (200)
#define ParamrterLimitMaxLen (100)
#define BasicInfoDataMaxLen  (50)

/************SPI Port define **************************************************/
#define RCC_APBPeriph_SysSPI           PT_RCC_APBPeriph_SysSPI
#define RCC_AHBPeriph_SysSPIGPIO       PT_RCC_AHBPeriph_SysSPIGPIO
#define RCC_AHBPeriph_SysSPINSSGPIO    PT_RCC_AHBPeriph_SysSPINSSGPIO
 
#define GPIO_SysSPI                    PT_GPIO_SysSPI
#define Periph_SysSPIPort              PT_Periph_SysSPIPort
#define GPIO_SysSPINSS                 PT_GPIO_SysSPINSS
#define Periph_SysSPINSSPort           PT_Periph_SysSPINSSPort

#define GPIO_PinSourceSysSPI_MOSI      PT_GPIO_PinSourceSysSPI_MOSI
#define GPIO_PinSourceSysSPI_MISO      PT_GPIO_PinSourceSysSPI_MISO
#define GPIO_PinSourceSysSPI_SCK       PT_GPIO_PinSourceSysSPI_SCK

#define GPIO_Pin_SysSCK                PT_GPIO_Pin_SysSCK
#define GPIO_Pin_SysMOSI               PT_GPIO_Pin_SysMOSI
#define GPIO_Pin_SysMISO               PT_GPIO_Pin_SysMISO
#define GPIO_Pin_SysNSS                PT_GPIO_Pin_SysNSS

#define GPIO_AF_SysSPI                 PT_GPIO_AF_SysSPI
#define SysSPI                         PT_SysSPI
#define Periph_SysSPI                  PT_Periph_SysSPI

extern uint8_t FirmwareUpdateType;
extern uint8_t  HR_SpO2_SendFlag;  // 0x00:none;0x01:HR;0x02:SpO2;0x03:HR&SpO2

void SPI_Configuration(void);
//void SPI_DeConfiguration(void);
//void SysCommunication_Init(void);
//void SysCommunication_DeInit(void);
//void SPI2_SendStrings(unsigned char *str);
//uint8_t SPI_ReadByte(void);
void SPITranslate_Task_Handler(event_t SPITranslate_Event);
//void DataUpdate(void);
//void SPISenddata(uint8_t ID);
void SPI_AlarmTransmit(uint8_t AlarmType);
void SPI_BATLevelTransmit(void);
void SPI_FWDataTransmitCtl(uint8_t Frame);
void SPI_DeviceInformationTransmit(void);
void SPI_FWDataTransmitBLEdata(uint8_t Frame);
void SPI_GPIO_Config(void);
bool SPI_Status_Wait(SPI_TypeDef * SPI, uint16_t bitMask, FlagStatus waitForStatus);
#endif /* __SPI_H */

