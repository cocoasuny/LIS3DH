//******************************************************************************
//  Claude ZHU
//  Hardware Team
//  (C) iCareTech Inc., 2013
//  All Rights Reserved.
//  Built with Keil MDK
//
//-------------------------------------------------------------------------------

#ifndef BasicTimer_H_
#define BasicTimer_H_

#include "stm32l1xx.h"
#include "platform.h"
#include <stdio.h>

/*Define the timer ID*/
typedef enum{TIMER_2 = 0,TIMER_3 = 1,TIMER_4 = 2,TIMER_5 = 3 ,TIMER_6 = 4,TIMER_7 = 5,TIMER_9 =6,TIMER_10 =7,TIMER_11 = 8,TIMER_ERROR = 9} Timer_ID_Typedef;
/*Define the timer type*/
typedef enum{TIM_TYPE_MS = 0, TIM_TYPE_US} Timer_Unit_Typedef;
/*define the timer status*/
typedef enum{Tim_Allocated = 1, Tim_Free = !Tim_Allocated} Timer_Status_Typedef;

/*define the channel number in PWM mode*/
typedef enum{TIM_PWM_CH1 = 0, TIM_PWM_CH2, TIM_PWM_CH3, TIM_PWM_CH4} Timer_PWM_Ch_Typedef;
/*define the timer mode*/
typedef enum{TIM_MODE_BASIC = 0, TIM_MODE_PWM} Timer_Mode_Typedef;

/*Define the timer call back function*/
typedef void (*BasicTimerCallBack) (Timer_ID_Typedef);

/*Define the timer structure for basic timer mode*/
typedef struct _TIM_Basic_Cfg
{
	Timer_Unit_Typedef enuTimerType;   //TIM_TYPE_MS--count in unit of ms, TIM_TYPE_US---count in unit of us

	uint16_t u16TimePeriod;

	uint8_t NVIC_IRQChannelPreemptionPriority;  /*!< Specifies the pre-emption priority for the IRQ channel
                                                   specified in NVIC_IRQChannel. This parameter can be a value
                                                   between 0 and 15 as described in the table @ref NVIC_Priority_Table */
	BasicTimerCallBack pIntCallBack;
}TIM_Basic_Cfg_Typedef;

/*Define the timer structure for PWM mode*/
typedef struct _TIM_PWM_Cfg
{
	Timer_Unit_Typedef 		enuTimerType;   //TIM_TYPE_MS--count in unit of ms, TIM_TYPE_US---count in unit of us
	Timer_ID_Typedef 			TIM_ID_PWM; 		//appointed timer for PWM mode, which fixed by hw design
	uint16_t 				PWM_Period; 	//0~0xffff, multiply the timer unit
	uint16_t 				PWM_Duty; 		//0~0xffff, PWM_DUTY/PWM_Period = cycle duty in %
	uint8_t 				PWM_Polarity; 	//TIM_OCPolarity_High, or TIM_OCPolarity_Low
	Timer_PWM_Ch_Typedef 	PWM_Ch;
	BasicTimerCallBack 		pIntCallBack;
}TIM_PWM_Cfg_Typedef;

/*Define the high level timer structure*/
typedef struct _TIM_Cfg
{
	Timer_Mode_Typedef 			TimerMode; 		//timer mode
	TIM_Basic_Cfg_Typedef 	*	TimerBasicCfg;	//config parameter for basic timer
	TIM_PWM_Cfg_Typedef 	*	TimerPWMCfg;	//config parameter for PWM timer
}TIM_Cfg_Typedef;


/*Define the Timer/Task table type*/
typedef struct _TIM_Task_Table
{
  TIM_TypeDef* TIMx;

  Timer_Status_Typedef enuTimStatus;    //current timer status

  BasicTimerCallBack pIntCallBack;      //timer interrupt call back function
}TIM_Task_Table_Typedef;

/*Define the max number of timer available*/
#define TIM_MAX_NUM 9

/*Define the prescaler for different unit*/
#define TIM_PRESCALER_1US   (GLOBAL_MCU_FREQ - 1)
#define TIM_PRESCALER_1MS   (GLOBAL_MCU_FREQ*1000 - 1)

/*Function Declaration*/
Timer_ID_Typedef Timer_Allocate(TIM_Cfg_Typedef *pTimerCfgDat);
void Timer_ChangeConfig(TIM_Cfg_Typedef *pTimerCfgDat,Timer_ID_Typedef TIM_ID);
void Timer_Free(Timer_ID_Typedef TIM_ID);
uint16_t Get_Timer_Cnt(Timer_ID_Typedef TIM_ID);
void Start_Timer_Cnt(Timer_ID_Typedef TIM_ID);
void Stop_Timer_Cnt(Timer_ID_Typedef TIM_ID);
void Clear_Timer_Cnt(Timer_ID_Typedef TIM_ID);
void Timer_Callback(Timer_ID_Typedef TIM_ID);

#endif

//end file

