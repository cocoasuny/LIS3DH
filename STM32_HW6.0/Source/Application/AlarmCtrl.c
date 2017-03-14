#include "AlarmCtrl.h"
#include "Usart.h"
#include "common.h"

/* 	Static varibles define 		*/
static Alarm_Type_Typedef mAlarmType;
static uint8_t 	mAlarmTypeFlag;

/*******************************************************************************
* Function Name  : AlarmLevelSet
* Description    : Set Alarm Status, on or off
* Input          : uint8_t
* Output         : None
* Return         : None
*******************************************************************************/
void AlarmLevelSet(Alarm_Level_Typedef alarmLevel)
{
	/* 	Define the alarm type 		*/
	if(alarmLevel == ALARM_LEVEL_HIGH)
	{
		mAlarmTypeFlag = ALARM_TYPE_MOTOR_MASK;
	}
	else if(alarmLevel == ALARM_LEVEL_MID)
	{
		mAlarmTypeFlag = ALARM_TYPE_MOTOR_MASK;
	}
	else if(alarmLevel == ALARM_LEVEL_LOW)
	{
		mAlarmTypeFlag = ALARM_TYPE_MOTOR_MASK;
	}
	
	//mAlarmType.AlarmStatus = true;
	
	mAlarmType.AlarmLevel = alarmLevel;
}
/*******************************************************************************
* Function Name  : AlarmLevelSet
* Description    : Set Alarm Status, on or off
* Input          : uint8_t
* Output         : None
* Return         : None
*******************************************************************************/
void AlarmStatusSet(bool newStatus)
{
	mAlarmType.AlarmStatus = newStatus;
}

/*******************************************************************************
* Function Name  : AlarmStatusGet
* Description    : Get Alarm Status, true or false
* Input          : void
* Output         : None
* Return         : None
*******************************************************************************/
bool AlarmStatusGet(void)
{
	return(mAlarmType.AlarmStatus);
}

/*******************************************************************************
* Function Name  : AlarmLevelGet
* Description    : Get Alarm Status, true or false
* Input          : void
* Output         : None
* Return         : None
*******************************************************************************/
Alarm_Level_Typedef AlarmLevelGet(void)
{
	return(mAlarmType.AlarmLevel);
}

/*******************************************************************************
* Function Name  : AlarmStatusClear
* Description    : AlarmStatusClear
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void AlarmStatusClear(uint8_t alarmTypeClear)
{
	if(IS_ALARM_TYPE_MASK(alarmTypeClear) == 0)
	{
		return;
	}
	/* 	Clear accordinate alarm type if finished 	*/
	mAlarmTypeFlag = mAlarmTypeFlag & (~alarmTypeClear);
	
	/* 	change alarm status if all type of notification finished 	*/
	if(mAlarmTypeFlag == 0x00)
	{
		mAlarmType.AlarmStatus = false;
	}
}
