/**
  ******************************************************************************
  * @file    calendar.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    24-January-2012
  * @brief   This file includes the calendar driver for the STM32L152D_EVAL
  *          demonstration.
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

#include "calendar.h"
#include <stdlib.h>
#include "Usart.h"
#include "common.h"

/** @addtogroup STM32L152D_EVAL_Demo
  * @{
  */

/** @defgroup CALENDAR
  * @brief    This file includes the calendar driver for the STM32L152D_EVAL
  *           demonstration.
  * @{
  */

/* EXIT Interrupt for wakeup */
EXTI_InitTypeDef Exit_Init_Wakeup;
static RTC_Wakeup_Callback m_pRTC_Wakup_Callback;


/** @defgroup CALENDAR_Private_FunctionPrototypes
  * @{
  */
static void Calendar_WeekCal(date_str_typedef* date_s);
/**
  * @}
  */

/** @defgroup CALENDAR_Private_Functions
  * @{
  */

/**
  * @brief  Initializes calendar application.
  * @param  None
  * @retval None
  */
void Calendar_Init(void)
{
		time_str_typedef time_struct;
		date_str_typedef date_s;
		/* Initialize Date structure */
		date_s.month = 01;
		date_s.day = 01;
		date_s.year = 2014;

		time_struct.seconds = 0;
		time_struct.minutes = 0;
		time_struct.hours = 0;

		PWR_RTCAccessCmd(ENABLE);//enable external RTC,32.768KHz
		RCC_LSEConfig(RCC_LSE_ON);

		/*Config the RTC*/
		Calendar_RTC_Configuration();
		Calendar_TimeSet(&time_struct);
		Calendar_DateSet(&date_s);
		RTC_WriteBackupRegister(RTC_BKP_DR0,0x5AA5);

}

/**
  * @brief  Returns the time entered by user, using menu vavigation keys.
  * @param  None
  * @retval Current time RTC counter value
  */
void Calendar_TimeSet(time_str_typedef* time_dat)
{

		RTC_TimeTypeDef   RTC_TimeStructure;
		uint8_t  Tim=0;
		ErrorStatus  TimeSet=SUCCESS;

		/* Set the Time */
		RTC_TimeStructure.RTC_Hours   = time_dat->hours;
		RTC_TimeStructure.RTC_Minutes = time_dat->minutes;
		RTC_TimeStructure.RTC_Seconds = time_dat->seconds;

		Tim = 10;
		do
		{
			TimeSet = RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure);
			Tim--;
		}while((TimeSet == ERROR) && (Tim != 0));

		#ifdef nRF51822_SPIRX
			printf("TimeSetErr=%d\r\n",TimeSet);
		#endif
}


/**
  * @brief  Sets the date entered by user, using menu navigation keys.
  * @param  None
  * @retval None
  */
void Calendar_DateSet(date_str_typedef* date_s)
{
		RTC_DateTypeDef   RTC_DateStructure;
#ifdef nRF51822_SPIRX
		ErrorStatus DateSet = SUCCESS;
#endif

		//week_str_typedef 	rtc_week_s;
		/* Freeze DR */
		(void)RTC->DR;
		/*get the week number*/
		Calendar_WeekCal(date_s);

		RTC_DateStructure.RTC_Year  = (date_s->year) % 100;
		RTC_DateStructure.RTC_Month = date_s->month;
		RTC_DateStructure.RTC_Date  = date_s->day;
		RTC_DateStructure.RTC_WeekDay = date_s->week;
		RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);

		#ifdef nRF51822_SPIRX
			printf("DateSetErr=%d\r\n",DateSet);
		#endif
}

/**
  * @brief  Sets the date entered by user, using menu navigation keys.
  * @param  None
  * @retval None
  */
void Calendar_DateGet(date_str_typedef* date_s)
{
		RTC_DateTypeDef   RTC_DateStructure;

		RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);
		/*get the date from RTC*/

		date_s->year = RTC_DateStructure.RTC_Year + 2000;
		date_s->month = RTC_DateStructure.RTC_Month;
		date_s->day = RTC_DateStructure.RTC_Date;
		date_s->week = RTC_DateStructure.RTC_WeekDay;
}

void Calendar_Get(date_str_typedef *date_s,RTC_TimeTypeDef *rtc_time)
{
    date_str_typedef pre_date_s;
    RTC_TimeTypeDef  pre_rtc_time;

    RTC_GetTime(RTC_Format_BIN, &pre_rtc_time);
    Calendar_DateGet(&pre_date_s);               //获取当前时间

    date_s->year = pre_date_s.year;
    date_s->month = pre_date_s.month;
    date_s->day = pre_date_s.day;
    date_s->week = pre_date_s.week;

    rtc_time->RTC_H12 = pre_rtc_time.RTC_H12;
    rtc_time->RTC_Hours = pre_rtc_time.RTC_Hours;
    rtc_time->RTC_Minutes = pre_rtc_time.RTC_Minutes;
    rtc_time->RTC_Seconds = pre_rtc_time.RTC_Seconds;
}



/**
  * @brief  the week day number.
  * @param  None
  * @retval None
  */
static void Calendar_WeekCal(date_str_typedef* date_s)
{
  uint32_t a = 0, b = 0, c = 0, s = 0, e = 0, f = 0, g = 0, d = 0;
  if (date_s->month < 3)
  {
    a = date_s->year - 1;
  }
  else
  {
    a = date_s->year;
  }

  b = (a / 4) - (a / 100) + (a / 400);
  c = ((a - 1) / 4) - ((a - 1) / 100) + ((a - 1) / 400);
  s = b - c;
  if (date_s->month < 3)
  {
    e = 0;
    f =  date_s->day - 1 + 31 * (date_s->month - 1);
  }
  else
  {
    e = s + 1;
    f = date_s->day + (153 * (date_s->month - 3) + 2) / 5 + 58 + s;
  }
  g = (a + b) % 7;
  d = (f + g - e) % 7;
  date_s->week = d + 1;
}




/**
  * @brief  Determines the week number, the day number and the week day number.
  * @param  None
  * @retval None
  */
void Calendar_WeekDayNum(const date_str_typedef* date_s, week_str_typedef *week_s)
{
  uint32_t a = 0, b = 0, c = 0, s = 0, e = 0, f = 0, g = 0, d = 0;
  int32_t n = 0;
  if (date_s->month < 3)
  {
    a = date_s->year - 1;
  }
  else
  {
    a = date_s->year;
  }

  b = (a / 4) - (a / 100) + (a / 400);
  c = ((a - 1) / 4) - ((a - 1) / 100) + ((a - 1) / 400);
  s = b - c;
  if (date_s->month < 3)
  {
    e = 0;
    f =  date_s->day - 1 + 31 * (date_s->month - 1);
  }
  else
  {
    e = s + 1;
    f = date_s->day + (153 * (date_s->month - 3) + 2) / 5 + 58 + s;
  }
  g = (a + b) % 7;
  d = (f + g - e) % 7;
  n = f + 3 - d;
  if (n < 0)
  {
    week_s->u8WkNum = 53 - ((g - s) / 5);
  }
  else if (n > (364 + s))
  {
    week_s->u8WkNum = 1;
  }
  else
  {
    week_s->u8WkNum = (n / 7) + 1;
  }
  week_s->u8WkDayNum = d + 1;
  week_s->u8DayNum = f + 1;
}




/**
  * @brief  Configures the RTC.
  * @param  None
  * @retval None
  */
void Calendar_RTC_Configuration(void)
{
  RTC_InitTypeDef   RTC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  /* Allow access to RTC */
  PWR_RTCAccessCmd(ENABLE);

  RTC_WriteProtectionCmd(DISABLE);
  RTC_EnterInitMode();

  /* Reset Backup Domain */
  RCC_RTCResetCmd(ENABLE);
  RCC_RTCResetCmd(DISABLE);

  if (RTC_ReadBackupRegister(RTC_BKP_DR0) != 0x5AA5)
  {
    /*!< LSE Enable */
    RCC_LSEConfig(RCC_LSE_ON);

    /*!< Wait till LSE is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
    {}

    /*!< LCD Clock Source Selection */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

    /* Enable the RTC Clock */
    RCC_RTCCLKCmd(ENABLE);

    /* Wait for RTC APB registers synchronisation */
    RTC_WaitForSynchro();

    /*Calender Configuartion*/
    RTC_InitStructure.RTC_AsynchPrediv = 0x7F;
    RTC_InitStructure.RTC_SynchPrediv =  0xFF;
    RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
    RTC_Init(&RTC_InitStructure);
    RTC_WriteBackupRegister(RTC_BKP_DR0, 0x5AA5);
  }
  else
  {
    /* Wait for RTC APB registers synchronisation */
    RTC_WaitForSynchro();
  }
}
/**
  * Calendar_RTC_Period_Wakeup_Init()
  * @brief  init the RTC to period wakeup, resolution = 1s/(2^15)
  * @param  None
  * @retval None
  */
void Calendar_RTC_Period_Wakeup_Init(uint32_t u32SecCnt, RTC_Wakeup_Callback pCallBack)
{
  /*Disable RTC Wakeup counter*/
  RTC_WakeUpCmd(DISABLE);

  /*Init to 1s resolution */
  RTC_WakeUpClockConfig(RTC_WakeUpClock_RTCCLK_Div2);
  /*Set counter*/
  RTC_SetWakeUpCounter(u32SecCnt);


  /*Init the callback function*/
  m_pRTC_Wakup_Callback = pCallBack;

  /*Init the EXIT wakeup structure*/
  Exit_Init_Wakeup.EXTI_Line      = EXTI_Line20;          //EXTI Line
  Exit_Init_Wakeup.EXTI_Mode      = EXTI_Mode_Interrupt;  //EXTI Mode, Interrupt
  Exit_Init_Wakeup.EXTI_Trigger   = EXTI_Trigger_Rising; //EXTI trigger, Rasing edge
  Exit_Init_Wakeup.EXTI_LineCmd   = ENABLE;              //Default state is "enable"
  /*Enable RTC wakeup INT*/
  EXTI_Init(&Exit_Init_Wakeup);

  /*Enable RTC wakeup counter*/
  RTC_ClearITPendingBit(RTC_IT_WUT);
  RTC_ClearFlag(RTC_FLAG_WUTF);
  RTC_ITConfig(RTC_IT_WUT, ENABLE);
  RTC_WakeUpCmd(ENABLE);
}

/**
  * Calendar_RTC_Period_Wakeup_DeInit()
  * @brief
  * @param  None
  * @retval None
  */
void Calendar_RTC_Period_Wakeup_DeInit(void)
{
    /*Disable RTC Wakeup counter*/
    RTC_WakeUpCmd(DISABLE);

    /*Reset the callback function*/
    m_pRTC_Wakup_Callback = NULL;
    /*Disable EXIT*/
    Exit_Init_Wakeup.EXTI_LineCmd   = DISABLE;        // Disable interrupt
    EXTI_Init(&Exit_Init_Wakeup);

}


void Calendar_RTC_Wakeup_Int_Handler(void)
{

		//RTC_ClearFlag(RTC_FLAG_WUTF);
		if(m_pRTC_Wakup_Callback!=NULL)
		{
			m_pRTC_Wakup_Callback();
		}
}
/*******************************************************************************
* Function Name  : RTC_Counter_Update
* Description    : ?üD?RTC
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RTC_Counter_Update(void)
{
	#ifdef RTC_TEST_ENABLE
		date_str_typedef date_s;
		RTC_TimeTypeDef rtc_time;

		Calendar_Get(&date_s,&rtc_time);
		printf("Time: HH = %d, \t MM = %d, \t SS = %d \n",rtc_time.RTC_Hours, rtc_time.RTC_Minutes, rtc_time.RTC_Seconds);
		printf("Date: Year = %d, \t Month = %d, \t Day = %d, \t Wk = %d \n",date_s.year, date_s.month, date_s.day, date_s.week);
	#endif

	if(Device_Mode == Device_Mode_RTC)
	{
		if((Get_OLED_Dis_Status() == OLEDDisEnterShutDown) || (Get_OLED_Dis_Status() == OLEDDisON))
		{
			TS_SendEvent(gOledDisTaskID,gOledDisEventRTCUpDate_c); //·￠?íOLED?üD?RTC??ê?????
		}
	}
	else
	{
		Calendar_RTC_Period_Wakeup_DeInit();
	}
}
/*******************************************************************************
* Function Name  : OLED_UpdateRTC
* Description    : ?üD?OLED RTC??ê?
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_UpdateRTC(void)
{
		uint8_t Hours_H=0;
		uint8_t Hours_L=0;
		uint8_t Minutes_H=0;
		uint8_t Minutes_L=0;

		date_str_typedef    date_s;               //RTC è??ú
		RTC_TimeTypeDef     rtc_time;             //RTC ê±??

        Calendar_Get(&date_s,&rtc_time);

		Hours_H = (rtc_time.RTC_Hours / 10);
		Hours_L = (rtc_time.RTC_Hours % 10);

		Minutes_H = (rtc_time.RTC_Minutes / 10);
		Minutes_L = (rtc_time.RTC_Minutes % 10);


	#ifdef BOARD_REDHARE_V3_0
		/***************?üD?OLED RTC??ê?*************************/
		if((Get_OLED_Dis_Status() == OLEDDisEnterShutDown) || (Get_OLED_Dis_Status() == OLEDDisON))
		{
			OLED_DisplayRTC(Hours_H,Hours_L,Minutes_H,Minutes_L);
		}
	#endif
}
/*******************************************************************************
* Function Name  : SetCalendar_Alarm_A
* Description    : éè???¨ê±?÷???óB￡?ó?óúêμ???à2a?￡°??¨ê±
* Input          : ?¨ê±ê±?￠·??￠??
* Output         : None
* Return         : None
*******************************************************************************/
void SetCalendar_Alarm_A(uint32_t Alarm_HH,uint32_t Alarm_MM,uint32_t Alarm_SS)
{
		RTC_AlarmTypeDef  RTC_AlarmStructure;

        #ifdef Alarm_Set_Debug
            printf("\r\nSetCalendar_Alarm_A, %02d : %02d : %02d.\r\n",Alarm_HH,Alarm_MM,Alarm_SS);
        #endif
		/* Disable the alarm A */
		RTC_AlarmCmd(RTC_Alarm_A, DISABLE);
		RTC_ITConfig(RTC_IT_ALRA, DISABLE);

		/* Set the alarm X */
		RTC_AlarmStructure.RTC_AlarmTime.RTC_H12     = RTC_H12_AM;
		RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours   = Alarm_HH;
		RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes = Alarm_MM;
		RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds = Alarm_SS;
		RTC_AlarmStructure.RTC_AlarmDateWeekDay = 0x31;
		RTC_AlarmStructure.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;
		RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay;
		RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &RTC_AlarmStructure);

		RTC_ITConfig(RTC_IT_ALRA, ENABLE);
		/* Enable the alarm */
		RTC_AlarmCmd(RTC_Alarm_A, ENABLE);
		//RTC_GetTime(RTC_Format_BIN, &rtc_time);
}
/*******************************************************************************
* Function Name  : DisCalendar_Alarm_A
* Description    : 1?±??¨ê±?÷???ó
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DisCalendar_Alarm_A(void)
{
    #ifdef Alarm_Set_Debug
        printf("\r\nDisCalendar_Alarm_A\r\n");
    #endif
		/* Disable the alarm A */
		RTC_AlarmCmd(RTC_Alarm_A, DISABLE);
		RTC_ITConfig(RTC_IT_ALRA, DISABLE);
}

/*******************************************************************************
* Function Name  : SetCalendar_Alarm_B
* Description    : Set Alarm B while keeping "cross day" alarm work
* Input          :
* Output         : None
* Return         : None
*******************************************************************************/
void SetCalendar_Alarm_B(uint32_t Alarm_HH,uint32_t Alarm_MM,uint32_t Alarm_SS)
{
	RTC_AlarmTypeDef  RTC_AlarmStructure;

	/* Disable the alarm B */
	RTC_AlarmCmd(RTC_Alarm_B, DISABLE);
	RTC_ITConfig(RTC_IT_ALRB, DISABLE);
        #ifdef Alarm_Set_Debug
            printf("\r\nSetCalendar_Alarm_B, %02d : %02d : %02d.\r\n",Alarm_HH,Alarm_MM,Alarm_SS);
        #endif
	/* Set the alarm X */
	RTC_AlarmStructure.RTC_AlarmTime.RTC_H12     = RTC_H12_AM;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours   = Alarm_HH;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes = Alarm_MM;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds = Alarm_SS;
	RTC_AlarmStructure.RTC_AlarmDateWeekDay = 0x31;
	RTC_AlarmStructure.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;
	RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay;
	RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_B, &RTC_AlarmStructure);

	RTC_ITConfig(RTC_IT_ALRB, ENABLE);
	/* Enable the alarm */
	RTC_AlarmCmd(RTC_Alarm_B, ENABLE);
}
/*******************************************************************************
* Function Name  : DisCalendar_Alarm_B
* Description    : Disable RTC alarm B
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DisCalendar_Alarm_B(void)
{
    #ifdef Alarm_Set_Debug
        printf("\r\nDisCalendar_Alarm_B\r\n");
    #endif
		/* Disable the alarm B */
		RTC_AlarmCmd(RTC_Alarm_B, DISABLE);
		RTC_ITConfig(RTC_IT_ALRB, DISABLE);
}

/*******************************************************************************
* Function Name  : AlarmB_Int_Handler
* Description    : Init alarm interrupt structure
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void AlarmB_Int_Handler(void)
{

}
/*******************************************************************************
* Function Name  : Init Alarm
* Description    : Init alarm interrupt structure
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Calendar_Alarm_Init(void)
{
	EXTI_InitTypeDef  EXTI_InitStructure;

	/* EXTI configuration */
	EXTI_ClearITPendingBit(EXTI_Line17);
	EXTI_InitStructure.EXTI_Line = EXTI_Line17;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}



/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
