#ifndef __ALARMCTRL_H
#define __ALARMCTRL_H

#include "stm32l1xx.h"
#include <stdbool.h>


/* 		Data Define 				*/
#define ALARM_TYPE_LED_MASK 			(0x01)
#define ALARM_TYPE_MOTOR_MASK 			(0x02)


#define IS_ALARM_TYPE_MASK(alarmType) ((alarmType) & (ALARM_TYPE_LED_MASK | ALARM_TYPE_MOTOR_MASK)) 
	

/* 		Alarm Level define 			*/
typedef enum {ALARM_LEVEL_LOW = 0,	ALARM_LEVEL_MID,	ALARM_LEVEL_HIGH} Alarm_Level_Typedef;

typedef struct _AlarmType
{
	bool					AlarmStatus;
	Alarm_Level_Typedef     AlarmLevel;
}Alarm_Type_Typedef;


void AlarmLevelSet(Alarm_Level_Typedef alarmLevel);
void AlarmStatusSet(bool newStatus);
Alarm_Level_Typedef AlarmLevelGet(void);
bool AlarmStatusGet(void);
void AlarmStatusClear(uint8_t alarmTypeClear);


#endif /* #define __ALARMCTRL_H */


