#ifndef  __PARAMETER_H__
#define  __PARAMETER_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "common.h"

/* USER Basic Information ID */
#define USER_INFO_Height   0x01
#define USER_INFO_Weight   0x02
#define USER_INFO_Age      0x03
#define USER_INFO_Sex      0x04
#define PARAMETER_ALARM_TIME_SLOT 			(60)		/* 	NO Second alarm in 1 minutes 		*/

void UnPackParameterLimit(uint8_t *p_ParameterLimit,uint16_t Len);
//void UnPackBasicInfo(uint8_t *p_BasicInfo,uint16_t Len);
void Alarm_Task_Handler(event_t AlarmEvent);
void ResetSingleModeAlarmVirbreTim(void);

#endif /* __PARAMETER_H__ */

