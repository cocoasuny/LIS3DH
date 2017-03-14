/**
  ******************************************************************************
  * @file    calendar.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    24-January-2012
  * @brief   This file contains all the functions prototypes for the calendar 
  *          firmware driver.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * FOR MORE INFORMATION PLEASE READ CAREFULLY THE LICENSE AGREEMENT FILE
  * LOCATED IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CALENDAR_H
#define __CALENDAR_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"

/** @addtogroup STM32L152D_EVAL_Demo
  * @{
  */

/** @addtogroup CALENDAR
  * @{
  */

/** @defgroup CALENDAR_Exported_Types
  * @{
  */

/* Time Structure definition */
typedef struct _time_t
{
  uint8_t seconds;
  uint8_t minutes;
  uint8_t hours;
}time_str_typedef;

/* Alarm Structure definition */
typedef struct _alarm_t
{
  uint8_t seconds;
  uint8_t minutes;
  uint8_t hours;
}alarm_str_typedef;

/* Date Structure definition */
typedef struct _date_t
{
	uint8_t month;
	uint8_t day;
	uint16_t year;
	uint8_t week;
}date_str_typedef;

/* Week Structure definition */
typedef struct _week_t
{
  uint8_t u8WkNum;
  uint8_t u8DayNum;
  uint8_t u8WkDayNum;
}week_str_typedef;



typedef void (*RTC_Wakeup_Callback) (void);


/** @defgroup CALENDAR_Exported_Constants
  * @{
  */
/**
  * @}
  */

/** @defgroup CALENDAR_Exported_Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup CALENDAR_Exported_Functions
  * @{
  */
void Calendar_Init(void);
//void Calendar_TimeDisplay(void);
void Calendar_TimeSet(time_str_typedef* time_dat);
void Calendar_DateSet(date_str_typedef* date_s);
void Calendar_WeekDayNum(const date_str_typedef* date_s, week_str_typedef *week_s);
void Calendar_DateGet(date_str_typedef* date_s);
void Calendar_Get(date_str_typedef *date_s,RTC_TimeTypeDef *rtc_time);
//void Calendar_WeekCal(date_str_typedef* date_s);


//void Calendar_DateDisplay(uint16_t nYear, uint8_t nMonth, uint8_t nDay);
//void Calendar_DateUpdate(void);
void Calendar_RTC_Configuration(void);
void Calendar_RTC_Period_Wakeup_Init(uint32_t u32SecCnt, RTC_Wakeup_Callback pCallBack);
void Calendar_RTC_Period_Wakeup_DeInit(void);
void RTC_Counter_Update(void);

void Calendar_RTC_Wakeup_Int_Handler(void);
void OLED_UpdateRTC(void);
void SetCalendar_Alarm_A(uint32_t Alarm_HH,uint32_t Alarm_MM,uint32_t Alarm_SS);
void DisCalendar_Alarm_A(void);
void AlarmB_Int_Handler(void);
void DisCalendar_Alarm_B(void);
void SetCalendar_Alarm_B(uint32_t Alarm_HH,uint32_t Alarm_MM,uint32_t Alarm_SS);
void Calendar_Alarm_Init(void);
#ifdef __cplusplus
}
#endif

#endif /* __CALENDAR_H */
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
