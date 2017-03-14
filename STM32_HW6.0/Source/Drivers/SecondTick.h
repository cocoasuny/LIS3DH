/*****************************************************************************
* Kernel / task handling public API
*
* (c) Copyright 2006, Freescale, Inc. All rights reserved.
*
*
* No part of this document must be reproduced in any form - including copied,
* transcribed, printed or by any electronic means - without specific written
* permission from Freescale Semiconductor.
*
*****************************************************************************/
#ifndef _SECONDTICK_H_
#define _SECONDTICK_H_


/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "ts_interface.h"
#include "Display.h"

/*
	Typedef
*/
/* SecTick status define*/
typedef enum{SECTIK_IDLE = 0, SECTIK_RUN, SECTIK_SUSPEND} SecTick_Status_Typedef;
/* time counter value define */
typedef struct _SecTik_Time
{
	uint8_t subseconds;		//0.1s
	uint8_t seconds;			//1s
	uint8_t minutes;			
	uint8_t hours;
} SecTick_Time_Typedef;

/* whole date and time value define */
typedef struct _SecTik_Date
{
	uint8_t subseconds;			//0.1s
	uint8_t seconds;			//1s
	uint8_t minutes;			
	uint8_t hours;
	uint8_t month;
	uint8_t day;
	uint8_t year;
} SecTick_Date_Typedef;




/*
@Common Definition
*/

/* counter resolution */
#define SECTICK_WAKEUP_RESOLUTION 				(1638)

/*
@Global function declaration
*/
SecTick_Status_Typedef Get_Current_SecTick_Status(void); 
void Get_Ticker_Value(SecTick_Time_Typedef* CounterUpdate);
void SecTick_Task_Handler(event_t SecTik_Event);
void SecTick_Stop(void);
void SecTick_Start(void);
void SecTick_Suspend(void);
void SecTick_Resume(void);
void SecTick_Back_Enter(void);
void OLED_UpdateSec(void);

FlagStatus Get_SecTick_BackGroud_Status(void);
void Set_SecTick_BackGroud_Status(FlagStatus newStatus);

#endif
//end of files
