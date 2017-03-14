/**
  ******************************************************************************
  * @file    RedHare/STM32L152/DateMemoryManage.h 
  * @author  Firmware Development Group
  * @version V1.0.0
  * @date    2015\8
  * @brief   Header for DateMemoryManage.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 iCareTech </center></h2>
  *
  *
  ******************************************************************************
  */
  
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DATAMEMORYMANAGE_H
#define __DATAMEMORYMANAGE_H

#include "stm32l1xx.h"
#include "Global_Typedef.h"


/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void GetSysConfigInfo(SysConfigInfo_t *info);
void SetSysConfigInfo(SysConfigInfo_t info);
void GetFWUpDateConfigInfo(FWUpdateInfo_t *info);
void SetFWUpDateConfigInfo(FWUpdateInfo_t info);
uint8_t DataMemoryWrite(store_type_e type,uint8_t * pu8Src,uint8_t u8Length);
uint8_t DataMemoryRead(store_type_e type,MTDataBuffer_t * p_storageExtBuffer,uint8_t u8Length);
uint32_t GetDataMemoryTotleLength(void);
uint8_t DataMemoryReadACK(uint8_t u8ValidLength);
void DataMemoryRestore(void);
void DataMemoryEraseAll(void);
void SetOSAStartTime(void);
void GetOSAStartTime(uint8_t * time,uint8_t u8Length);
void SetCPUConfigInfo(SysConfigCPUInfo_t info);
void SetDeviceModeConfigInfo(void);
void GetCPUConfigInfo(void);
void SetFreeRunNightStartFrame(void);
void SetStorageConstVarConfigInfo(StorageReadConstVar_t info);
void GetStorageConstVarConfigInfo(StorageReadConstVar_t *p_Info);

#ifdef STORAGE_LOG

void SetStorageLog(StorageLog_t *pStorageLog);
void GetStorageLog(StorageLog_t *pStorageLog);
void SetStorageTimerErrorLog(StorageTimerErrorLog_t * pStorageTimerErrorLog);
void GetStorageTimerErrorLog(StorageTimerErrorLog_t * pStorageTimerErrorLog);
void SetStorageTotelLengthErrorLog(StorageTotelLengthErrorLog_t * pStorageTotelLengthErrorLog);
void GetStorageTotelLengthErrorLog(StorageTotelLengthErrorLog_t * pStorageTotelLengthErrorLog);

#endif


#endif /* __DATAMEMORYMANAGE_H */

/***************** (C) COPYRIGHT iCareTech Healthcare Co., Ltd.(END OF FILE)***********/




