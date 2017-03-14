/**
  ******************************************************************************
  * @file    Project/STM32L1xx_StdPeriph_Templates/main.h 
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    13-April-2012
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _GLOBAL_TYPEDEF_H
#define _GLOBAL_TYPEDEF_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include <stdio.h>
#include "Calendar.h"
#include "Platform.h"
#include "cc_alg.h"

typedef enum 
{
	OLEDDisON=0,
	OLEDDisEnterShutDown,
	OLEDDisShutDown,
	OLEDDisExitShutDown,
}OledDisStatus;

typedef struct _ParameterLimit
{
	uint8_t    VLowLimit;            //非常低门限
	uint8_t    LowLimit;             //低门限
	uint8_t    HigLimit;             //高门限
	uint8_t    VHigLimit;            //非常高门限
}Paramete_Limit_Typedef;

///* Define the temperature data parameters */
//typedef struct _Tem_Parameters
//{
//	date_str_typedef    date_s;               //采集年月日
//	RTC_TimeTypeDef     rtc_time;             //采集时分秒
//	uint8_t             NTC_Tem_TenDigit;     //十位
//	uint8_t             NTC_Tem_SingleDigit;  //个位
//	uint8_t             NTC_Tem_Tenth;        //十分位
//	float               NTCVal;               //NTC温度值
//	float               VLowLimit;            //非常低门限
//	float               LowLimit;             //低门限
//	float               HigLimit;             //高门限
//	float               VHigLimit;            //非常高门限
//}Tem_Parameters_Typedef;

/* Define the HR/SpO2  data parameters */
typedef struct _HR_SpO2_Parameters
{
	date_str_typedef    date_s;               //采集年月日
	RTC_TimeTypeDef     rtc_time;             //采集时分秒
	Paramete_Limit_Typedef  HR_Limit;         //心率门限值
	Paramete_Limit_Typedef  SpO2_Limit;       //血氧门限值
	HR_SPO2_DAT_T *   HrSpO2DataRep_pst;
}HR_SpO2_Parameters_Typedef;

/* Define the Temperture data parameters */
typedef struct _Temperture_Parameters
{
	date_str_typedef    date_s;               //采集年月日
	RTC_TimeTypeDef     rtc_time;             //采集时分秒
	float               Value;                //温度值
	Paramete_Limit_Typedef  Limit;            //门限值
}Temperture_Parameters_Typedef;

/* Define Pass Key data parameters */
typedef struct  _PassKey_Parameters
{
	uint8_t		ConfirmStatus;      //验证是否通过
	uint8_t     Status;             //是否收到验证密码
	uint8_t     len;
	uint8_t     p_contex[4];
	uint8_t     Connect_Status;
}PassKey_Typedef;

typedef struct
{
	uint8_t  DevSupportCap;
	const uint8_t  * pDevREV;
	const uint8_t  * pFWREV;
    const uint8_t  * pBootREV;
	const uint8_t	* pSeriNum;
	uint8_t  DeviceInfo_Status;
}DeviceInfomation_t;

typedef struct
{
    uint8_t     AppActiveStatus;
    uint8_t     AppModifySNStatus;
    uint8_t     AppModifySNInfo[4];
    uint8_t     MTID;
    uint8_t     DataLength;
    uint8_t     DataFormate;
}SysConfigInfo_t;

typedef struct
{
    uint8_t     Month;
    uint8_t     Day;
    uint8_t     Hour;
    uint8_t     Min;
    uint8_t     Sec;
	unsigned int lr;  
	unsigned int pc;    
}SysConfigCPUInfo_t;

typedef struct
{
    uint8_t    FWUpdateUPGRADEStatus;
    uint8_t    FWUpdateFWDATAStatus;
    uint8_t    FWUpdateCRC[4];
    uint8_t    FWUpdateFIRMWAREType;
}FWUpdateInfo_t;

typedef enum {NON_MONITOR_TYPE, MONITOR_TYPE}store_type_e;

typedef struct {
	uint32_t 	start_time;
	uint8_t 	availabe_length;	
	uint8_t		Buffer[EEPROMReadMaxData];
}MTDataBuffer_t;

typedef struct{
    uint32_t    u32StorageRdAddr;
}StorageConstVar_t;

typedef struct{
    uint8_t     Month;
    uint8_t     Day;
    uint8_t     Hour;
    uint8_t     Min;
    uint8_t     Sec;
    uint8_t     part_num;
    uint32_t    totel_availabe_length;
    uint32_t    rd_part_start_time;
    uint16_t    rd_part_start_page_addr;
    uint16_t    rd_part_end_page_addr;
    uint32_t    totel_len_InCurrReadPart;
    uint32_t    u32RdPointer;
    uint32_t    u32RdNum_CurrPartition;
    uint8_t     pre_rd_len;

    uint32_t    wr_part_start_time;
    uint16_t    wr_part_start_page_addr;
    uint16_t    wr_part_end_page_addr;
    uint32_t    u32WrPointer;
    uint32_t    u32WrNumCurrPart;
    uint8_t     pre_wr_len;

    uint32_t    u32CurrSectorStartAdrr;
    uint32_t    u32LastSectorStartAddr;
}StorageLog_t;

typedef struct{
    uint8_t     Month;
    uint8_t     Day;
    uint8_t     Hour;
    uint8_t     Min;
    uint8_t     Sec;
    uint8_t     cnt;
}StorageTimerErrorLog_t;

typedef struct{
    uint8_t     u8State;
    uint8_t     Month;
    uint8_t     Day;
    uint8_t     Hour;
    uint8_t     Min;
    uint8_t     Sec;
    uint32_t    u32CurrTotelLength;
    uint32_t    u32LastTotelLength;
    uint8_t     process;

    uint32_t    start_time_rd_part;
    uint16_t    start_page_addr_rd_part;
    uint16_t    end_page_addr_rd_part;
    uint32_t    u32TotelLenInCurrRdPart;
    uint32_t    u32RdPointer;
    uint32_t    u32RdNumInCurrPart;
    uint8_t     u8PreRdLen;

    uint32_t    start_time_wr_part;
    uint16_t    start_page_addr_wr_part;
    uint16_t    end_page_addr_wr_part;
    uint32_t    u32WrPointer;
    uint32_t    u32WrNumInCurrPart;
    uint8_t     u8PreWrLen;
}StorageTotelLengthErrorLog_t;

typedef struct{
	uint32_t 	start_time;
	uint16_t	start_page_addr;
	uint16_t  	end_page_addr;
}flash_partition_t;

typedef struct{
    flash_partition_t flash_read_partition;
    uint32_t    u32StorageReadAddr;
}StorageReadConstVar_t;


#endif /* _GLOBAL_TYPEDEF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
