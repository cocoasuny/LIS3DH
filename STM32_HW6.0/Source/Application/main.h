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
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
//#include "common.h"
#include "Global_Typedef.h"
#include "FWUpdate.h"
#include "BasicTimer.h"

/* Exported types ------------------------------------------------------------*/
extern uint16_t Device_Mode;
extern uint16_t Device_Mode_pre;

extern deviceStateTypedef Device_State;                     //设备工作模式：Work Alone or Work With APP
extern hrspo2MeasureModeTypedef    gFlagHR_SpO2Measure;     //HR SpO2 Measure Mode:HRSpO2_Measure_Normal;
                                                            //HRSpO2_Measure_Test: for factory Test

extern volatile Timer_ID_Typedef    gKeyTIMID;            //按键定时器，用于检测长、短按键
extern volatile Timer_ID_Typedef    gShutDownOledTIMID;
extern PassKey_Typedef	   g_PassKey;             //Pass Key info
extern DeviceInfomation_t  g_DeviceInfomation;

//extern uint8_t     SpO2_RealTimeSample_GetResultNum; //SpO2及时采集得到稳定结果数
//extern uint8_t     HR_RealTimeSample_GetResultNum;   //HR及时采集得到稳定结果数
//extern	bool 		flagIsHRSpO2FixNumberSampleKeyPress;



extern volatile uint8_t     gKeyPressTIMOut;             //按键长按超时标志
extern volatile uint8_t     gKeyPressStatus;             //按键可用状态
extern volatile uint8_t     gKeyLongPressStatus;         //长按按键状态

extern volatile uint8_t     gMCUStatus;                  //MCU状态
extern volatile uint8_t     gWakeUpNeedOLEDON;           //唤醒设备，需要点亮OLED
extern volatile uint8_t     gSyncACK;                    //数据同步应答信号
extern volatile uint8_t     gAlarmNotEnoughSpace;        //not enough space alarm flag 
extern volatile uint8_t     gFlagSyncDataInProgress;     //Sync Data In Progressing flag
#ifdef EMT_Test_App
extern volatile uint8_t   MakeDataCnt;
#endif

//extern uint16_t    MT_EEPROM_WriteAddr;           //写EEPROM地址
//extern uint16_t    MT_EEPROM_ReadAddr;            //读EEPROM地址

extern uint8_t 		SPO2_START_BY_KEY;
extern uint8_t      u8AppModifySeriNum[4];  //App Modify SN

//extern float V_AMB;
//extern float Obj_Tmp;
extern HR_SpO2_Parameters_Typedef 				gHRSpO2Val;     //保存最终心率血氧值
extern uint8_t     								gFWData[];
//extern Alarm_Type_Typedef 						gAlarmType;     //报警类型

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void PeriPower(uint8_t Power);
void MCO_OutPutLSE(uint8_t status);
uint8_t GetPeriphPowerStatus(void);
//void GetCPUInfo(void);
void AppModifySN(void);
uint32_t CovernDateto32(void);
//void Covern32toDate(date_str_typedef* date_s, uint32_t date);
//uint16_t  CovernTimeto16(void);
void Translateu32Tou8Array(uint32_t u32Temp,uint8_t *pu8Array);
uint32_t Translateu8ArrayTou32(uint8_t *pu8Array);

void gSubFunc_Stat_Set(uint16_t mask, uint8_t newState);
uint16_t gSubFunc_Stat_Get(uint16_t mask);

extern void SystemInit_AfterWakeup (void);

extern void OSASTest(void);

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
