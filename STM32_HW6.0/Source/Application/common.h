/**
  ******************************************************************************
  * @file    common.h
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    22-February-2013
  * @brief   This file contains all the functions prototypes for the RTC firmware 
  *          library.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
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
#ifndef __COMMON_H
#define __COMMON_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
/* Includes ------------------------------------------------------------------*/

#include "stm32l1xx.h"
#include <stdio.h>
#include "platform.h"
#include "AlarmCtrl.h"
#include "Global_Typedef.h"
#include "TS_Interface.h"
#include "Usart.h"
#include "Timer.h"
#include "PowerManage.h"
#include "Key.h"
#include "Font.h"	 
#include "calendar.h"
#include "FWUpdate.h"
#include "BasicTimer.h"
#include "LEDCtrl.h"
#include "AFE44x0.h"
//#include "SpO2HrCalCZT.h"
#include "PulseOximetryAFE.h"
#include "cc_alg_app_interface.h"
#include "ADS1118.h"
#include "Temperature.h"
#include "NTC.h"
#include "TPIS1S1253.h"
#include "SPI.h"
#include "Display.h"
#include "MotorCtl.h"
#include "step.h"
//#include "flash_if.h"
#include "bootUpdate.h"
#include "call.h"
#include "DeviceCtl.h"
#include "storeManage.h"
#include "MX25L1606E.h"
//#include "DataEEPROM.h"
#include "DataMemoryManage.h"
#include "led.h"


#include "lis3dh_driver.h"




#include "Monitoer_Template.h"
#include "SecondTick.h"
#include "m95m01.h"
#include "SecondTick.h"
#include "ParameterLimit.h"
#include "App_error.h"
#include "SyncData.h"
#include "PowerManage.h"
#include "charge.h"
#include "BatteryMonitor.h"
#include "bma2x2.h"
#include "bma250e.h"
#include "CRC.h"
#include "storeManage.h"
#include "bsp_accelero.h"


#include "main.h"

#ifdef BOARD_REDHARE_V3_0
	#include "OLED_MXS8475.h"
#endif	


//#define USR_ASSERT

/* Exported macro ------------------------------------------------------------*/
#ifdef  USR_ASSERT
  #define usr_para_check(expr) ((expr) ? (void)0 : usr_assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void usr_assert_failed(uint8_t* file, uint32_t line);
#else
  #define usr_para_check(expr) ((void)0)
#endif 						/* USR_ASSERT */
/* Global Various Declaration */


/* Global Various Declaration */



#ifdef __cplusplus
}
#endif

#endif /*__STM32L1xx_RTC_H */

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
