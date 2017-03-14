/**
  ******************************************************************************
  * @file    SecondTick.c
  * @author  Yun Zhu
  * @version V1.0.0
  * @date    6-Jun-2014
  ******************************************************************************
  */

/*
How to use this driver

@step before all: You must make sure RTC module has been initialized calling RTC_Config();

@Step0: Create task as 
		gTsSecTickTaskID_c = TS_CreateTask(gTsSecTickTaskPriority_c,SecTick_Task_Handler);

@Step1: Create event as you needed by calling:
		TS_SendEvent(gTsSecTickTaskID_c, gSecTikEventStart);
		

Note: 
		1. Event defined in ts_interface.h as
			#define gSecTikEventStart													((event_t)(1 << 0))
			#define gSecTikEventSuspend												((event_t)(1 << 1))
			#define gSecTikEventResume												((event_t)(1 << 2))
			#define gSecTikEventStop													((event_t)(1 << 3))
		2. Can get current counter value by calling Get_Ticker_Value(SecTick_Time_Typedef* ), the format is 
			typedef struct _SecTik_Time
			{
				uint8_t subseconds;		//0.1s
				uint8_t seconds;			//1s
				uint8_t minutes;			
				uint8_t hours;
			} SecTick_Time_Typedef;
		
Important:
		1. Add you OLED or display codes in function SecTick_Counter_Update();
		2. When enter other menu(especially the date/time menu), must reinit the RTC autowake up function as
				  Calendar_RTC_Period_Wakeup_Init(YourTimeResolution, YourCallBackFunction);
		3. Second Tick does not consume any internal resource if in background running
		4. The max value of ticker is 24hours, when overflow, will stop and enter 00:00:00:0

*/



#include "calendar.h"
#include "SecondTick.h"
#include <stdlib.h>
#include "Usart.h"
#include "platform.h"
#include "main.h"

/*
@Define the global varibles
*/

/* second ticker status */
static SecTick_Status_Typedef gSecTickerStatus = SECTIK_IDLE;
/* date store */
static SecTick_Date_Typedef         gStartDate = {0,0,0,0,0,0,0};
/* interval between two sec tick mode entry */
static uint32_t                gTimeSuspSec = 0;
/* current second tick counter value */
static uint32_t                gTimeTickSec = 0;

/* 	sectick run status define 		*/
static FlagStatus SecTick_Measure_BackGround = RESET;


/* test data */
#ifdef SECTICK_DEBUG_EN
	SecTick_Time_Typedef			gTestLastTick = {0,0,0,0};
#endif


/*
@Local function declaration
*/
static void Set_Current_SecTick_Status(SecTick_Status_Typedef newSecTickStatus);
static void SecTick_Display_Code(SecTick_Time_Typedef* CounterUpdate, uint32_t timeInvertal);
static void SecTick_Counter_Update(void);

/*
@function realization
*/

	
/**
  * @brief  get current second ticker's backgroud run status.
  * @param  None
  * @retval status of second ticker
  */
FlagStatus Get_SecTick_BackGroud_Status(void)
{
	return SecTick_Measure_BackGround;
}
	
/**
  * @brief  SET current second ticker's backgroud run status.
  * @param  None
  * @retval status of second ticker
  */
void Set_SecTick_BackGroud_Status(FlagStatus newStatus)
{
	SecTick_Measure_BackGround = newStatus;
}	
	
	
	
	
/**
  * @brief  get current second ticker's status.
  * @param  None
  * @retval status of second ticker
  */
SecTick_Status_Typedef Get_Current_SecTick_Status(void)
{
  return(gSecTickerStatus);
}

/**
  * @brief  set current second ticker's status.
  * @param  sec ticker's status
  * @retval None
  */
static void Set_Current_SecTick_Status(SecTick_Status_Typedef newSecTickStatus)
{
  gSecTickerStatus = newSecTickStatus;
}

/**
  * @brief  Sec Ticker Start.
  * @param  sec ticker's status
  * @retval None
  */
void SecTick_Start(void)
{
  date_str_typedef   RTC_DateStructure;
  RTC_TimeTypeDef   RTC_TimeStructure;
  /*init the RTC to 0.1s resolution*/
	//Calendar_RTC_Configuration();
  Calendar_RTC_Period_Wakeup_Init(SECTICK_WAKEUP_RESOLUTION, SecTick_Counter_Update);
	/* handle when in idle state */
	if(gSecTickerStatus == SECTIK_IDLE)
	{
		/*Get current date/time and save to static varibles */
        Calendar_Get(&RTC_DateStructure,&RTC_TimeStructure);
		gStartDate.year = RTC_DateStructure.year;
		gStartDate.day = RTC_DateStructure.day;
		gStartDate.month = RTC_DateStructure.month;
		gStartDate.seconds = RTC_TimeStructure.RTC_Seconds;
		gStartDate.minutes = RTC_TimeStructure.RTC_Minutes;
		gStartDate.hours = RTC_TimeStructure.RTC_Hours;
		gStartDate.subseconds = 0xff-(RTC_GetSubSecond() & 0xff);
		/*Init the suspend interval and second ticker init value*/
		gTimeSuspSec = 0;
		gTimeTickSec = 0;
		/*set to run state*/
		Set_Current_SecTick_Status(SECTIK_RUN);
	}
}

/**
  * @brief  Sec Ticker suspend.
  * @param  None
  * @retval None
  */
void SecTick_Suspend(void)
{
  /*save interval between start to suspend varibles */
	#ifdef SECTICK_DEBUG_EN
		printf("test, suspend \r\n");
  #endif
	
	gTimeSuspSec = gTimeTickSec;
  gTimeTickSec = 0;
	
	/*De-init the RTC auto wakeup function*/
	Calendar_RTC_Period_Wakeup_DeInit();
	
	/*set to suspend state*/
	Set_Current_SecTick_Status(SECTIK_SUSPEND);
}

/**
  * @brief  Sec Ticker suspend.
  * @param  None
  * @retval None
  */
void SecTick_Resume(void)
{
  date_str_typedef   RTC_DateStructure;
  RTC_TimeTypeDef   RTC_TimeStructure;
	#ifdef SECTICK_DEBUG_EN
		printf("resume \r\n");
  #endif
	/*init the RTC to 0.1s resolution*/
  Calendar_RTC_Period_Wakeup_Init(SECTICK_WAKEUP_RESOLUTION, SecTick_Counter_Update);

  /*Get current date/time and save to static varibles */
  Calendar_Get(&RTC_DateStructure,&RTC_TimeStructure);
  gStartDate.year = RTC_DateStructure.year;
  gStartDate.day = RTC_DateStructure.day;
  gStartDate.month = RTC_DateStructure.month;
  gStartDate.seconds = RTC_TimeStructure.RTC_Seconds;
  gStartDate.minutes = RTC_TimeStructure.RTC_Minutes;
  gStartDate.hours = RTC_TimeStructure.RTC_Hours;
  gStartDate.subseconds = 0xff-(RTC_GetSubSecond() & 0xff);
	/*set to run state*/
	Set_Current_SecTick_Status(SECTIK_RUN);
}

/**
  * @brief  Sec Ticker into Background.
  * @param  None
  * @retval None
  */
void SecTick_Back_Enter(void)
{
	#ifdef SECTICK_DEBUG_EN
		printf("test, enter back ground \r\n");
  #endif
	
	/*De-init the RTC auto wakeup function*/
	Calendar_RTC_Period_Wakeup_DeInit();

}

/**
  * @brief  Sec Ticker stop.
  * @param  None
  * @retval None
  */
void SecTick_Stop(void)
{
  #ifdef SECTICK_DEBUG_EN
    printf("test stop \r\n");
  #endif
	/*De-init the RTC auto wakeup function*/
	Calendar_RTC_Period_Wakeup_DeInit();

	/*set to idle state*/
	Set_Current_SecTick_Status(SECTIK_IDLE);
}

/**
  * @brief  Sec Ticker OLED update.
  * @param  None
  * @retval None
  */
static void SecTick_Counter_Update(void)
{
  SecTick_Date_Typedef  CurrDate;
  date_str_typedef   RTC_DateStructure;
  RTC_TimeTypeDef   RTC_TimeStructure;
  
  //SecTick_Time_Typedef  CounterUpdate;

  uint32_t          u32TimeCurrSec;
  uint32_t          u32TimeLastSec;

	
	/* 	If system enter measure or charge mode without key, push sectick into background mode 	*/
	if(Device_Mode == Device_Mode_Charge || 		/* In charge mode 		*/
		Device_Mode == Device_Mode_RTC ) 			/* switch to RTC due to BLE connection 	*/
	{
		#ifdef SECTICK_DEBUG_EN
			printf("ento back ground \r\n");
		#endif
		TS_SendEvent(gTsSecTickTaskID_c,gSecTikEventBackground);		/* 	Into back ground 		*/
		return;
	}
	
  /*Get current date and subsecond*/
  Calendar_Get(&RTC_DateStructure,&RTC_TimeStructure);

  CurrDate.year = RTC_DateStructure.year;
  CurrDate.day = RTC_DateStructure.day;
  CurrDate.month = RTC_DateStructure.month;
  CurrDate.seconds = RTC_TimeStructure.RTC_Seconds;
  CurrDate.minutes = RTC_TimeStructure.RTC_Minutes;
  CurrDate.hours = RTC_TimeStructure.RTC_Hours;

  CurrDate.subseconds = 0xff-(RTC_GetSubSecond() & 0xff);
  /*Calculate the time passby*/

  u32TimeLastSec = (((gStartDate.hours * 3600) + (gStartDate.minutes * 60) + gStartDate.seconds)<<8) + gStartDate.subseconds;

  if(CurrDate.year != gStartDate.year ||
    CurrDate.month != gStartDate.month ||
    CurrDate.day != gStartDate.day)
    {
      CurrDate.hours += 24;
    } 

  u32TimeCurrSec = (((CurrDate.hours * 3600) + (CurrDate.minutes * 60) + CurrDate.seconds)<<8) + CurrDate.subseconds;

  gTimeTickSec = u32TimeCurrSec - u32TimeLastSec + gTimeSuspSec;

	/*The max value equals to 24 hours*/
	if(gTimeTickSec >= ((24 * 3600)<<8))
	{
		TS_SendEvent(gTsSecTickTaskID_c, gSecTikEventStop);
	}

	/* Compensate the interrupt interval */

	/*Code for OLED display*/
	TS_SendEvent(gOledDisTaskID,gOledDisEventSecUpDate_c); //发送OLED显示Sec界面
	
  #ifdef SECTICK_DEBUG_EN
    printf("%d : %d : %d : %d \n", CurrDate.hours, CurrDate.minutes, CurrDate.seconds, CurrDate.subseconds);
  #endif

}
/*******************************************************************************
* Function Name  : OLED_UpdateSec
* Description    : 更新OLED Sec显示
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_UpdateSec(void)
{
		SecTick_Time_Typedef  CounterUpdate;
		
		/*loop to get the different value in format of hh.mm.ss.subs*/
		SecTick_Display_Code(&CounterUpdate, gTimeTickSec);

		/* 	Code for OLED display 	*/
		if(Device_Mode == Device_Mode_SecTick)
		{
			OLED_DisplaySecTick(CounterUpdate.hours,CounterUpdate.minutes,CounterUpdate.seconds,CounterUpdate.subseconds);
		}
}

/**
  * @brief  Get current tick value.
  * @param  None
  * @retval None
  */
void Get_Ticker_Value(SecTick_Time_Typedef* CounterUpdate)
{
  switch(gSecTickerStatus)
  {
    case SECTIK_IDLE:
      SecTick_Display_Code(CounterUpdate, 0);
      break;
    case SECTIK_RUN:
      SecTick_Display_Code(CounterUpdate, gTimeSuspSec);
      break;
    case SECTIK_SUSPEND:
      SecTick_Display_Code(CounterUpdate, gTimeTickSec);
      break;
    default:
      SecTick_Display_Code(CounterUpdate, 0);
      break;
  }
}


/**
  * @brief  Ticker Display Codeing.
  * @param  time interval
  * @retval None
  */
static void SecTick_Display_Code(SecTick_Time_Typedef* CounterUpdate, uint32_t timeInvertal)
{
  uint8_t           i;
  uint8_t           u8SubsecTemp;

  //get the hours different
  for(i = 0;(timeInvertal >= (3600<<8));i++,timeInvertal -= (3600<<8));
  CounterUpdate->hours = i%24;

  //get the minutes different
  for(i = 0;(timeInvertal >= (60<<8));i++,timeInvertal -= (60<<8));
  CounterUpdate->minutes = i;

  //get the second different
  for(i = 0;(timeInvertal >= (1<<8));i++,timeInvertal -= (1<<8));
  CounterUpdate->seconds = i;

  //get the subsecond different
  u8SubsecTemp = timeInvertal&0xff;

  if(u8SubsecTemp < 25)
  {
    CounterUpdate->subseconds = 0;
  }
  else if((u8SubsecTemp >= 25) && (u8SubsecTemp < 51))
  {
    CounterUpdate->subseconds = 1;
  }
  else if((u8SubsecTemp >= 51) && (u8SubsecTemp < 76))
  {
    CounterUpdate->subseconds = 2;
  }
  else if((u8SubsecTemp >= 76) && (u8SubsecTemp < 102))
  {
    CounterUpdate->subseconds = 3;
  }
  else if((u8SubsecTemp >= 102) && (u8SubsecTemp < 128))
  {
    CounterUpdate->subseconds = 4;
  }
  else if((u8SubsecTemp >= 128) && (u8SubsecTemp < 154))
  {
    CounterUpdate->subseconds = 5;
  }
  else if((u8SubsecTemp >= 154) && (u8SubsecTemp < 180))
  {
    CounterUpdate->subseconds = 6;
  }
  else if((u8SubsecTemp >= 180) && (u8SubsecTemp < 205))
  {
    CounterUpdate->subseconds = 7;
  }
  else if((u8SubsecTemp >= 205) && (u8SubsecTemp < 230))
  {
    CounterUpdate->subseconds = 8;
  }
  else if((u8SubsecTemp >= 230) && (u8SubsecTemp <= 255))
  {
    CounterUpdate->subseconds = 9;
  }
  else
  {
    CounterUpdate->subseconds = 0;
  }
}

/******************************************************************
*                        SecondTick_Task_Handler                  *
*******************************************************************/
void SecTick_Task_Handler(event_t SecTik_Event)
{
	#ifdef SECTICK_DEBUG_EN
	printf("second tick state transfer, new event = %x \n", SecTik_Event);
	#endif
	
	if(SecTik_Event&gSecTikEventStart)
	{
		/*init second tick and kick off program*/
		SecTick_Start();
	}
	else if(SecTik_Event&gSecTikEventSuspend)
	{
		/*stop the second tick*/
		SecTick_Suspend();
	}
	else if(SecTik_Event&gSecTikEventResume)
	{
		/*Resume the second tick*/
		SecTick_Resume();
	}
	else if(SecTik_Event&gSecTikEventStop)
	{
		/*stop tick*/
		SecTick_Stop();
	}
	else if(SecTik_Event&gSecTikEventBackground)
	{
		Set_SecTick_BackGroud_Status(SET);
		/*stop tick*/
		SecTick_Back_Enter();
	}
	else
	{
		/*stop tick*/
		SecTick_Stop();
	}
}




//end of file
