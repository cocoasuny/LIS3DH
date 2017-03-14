//******************************************************************************
//  Claude ZHU
//  Hardware Team
//  (C) iCareTech Inc., 2013
//  All Rights Reserved.
//  Built with Keil MDK
//
//-------------------------------------------------------------------------------


/*
How to use this driver to config your timer interrupt

****Steps to allocate and init a timer ****


@Step0: rewrite timer interrupt function as below:

			void TIM3_IRQHandler(void)											//apply to every timer interrupt ISR
			{
				Timer_ID_Typedef Timer_ID = TIMER_3; 					//consistant with current timer
				TIM_ClearITPendingBit(TIM3, TIM_IT_Update);   //consistant with current timer
				Timer_Callback(TIMER_3);											//input parameter is the current timer ID
			}

@Step1: Prepare your call back function, which will be invoke when timer interrupt occurs
        Note, Must be (void), no paramters input
      For example:
				void LED_FLASH_T0(void)
				
@Step2: Define your timer configuration structure, "TIM_Cfg_Typedef" as TIM_Cfg_Test
 			For basic timer, init TIM_Basic_Cfg_Typedef
				@enuTimerType --- timer unit, ms or us
				@u16TimePeriod --- interrupt interval = u16TimePeriod*timer unit
				@NVIC_IRQChannelPreemptionPriority --- interrupt priority
				@pIntCallBack --- function pointer, point to your call back function
				For example:
					Tim_Basic_Cfg_Test.enuTimerType = TIM_TYPE_MS;
					Tim_Basic_Cfg_Test.u16TimePeriod = 1000; 
					Tim_Basic_Cfg_Test.NVIC_IRQChannelPreemptionPriority = 0x0f;
					Tim_Basic_Cfg_Test.pIntCallBack = LED_FLASH_T0;
					
					TIM_Cfg_Test.TimerMode = TIM_MODE_BASIC
					TIM_Cfg_Test.TimerBasicCfg = &Tim_Basic_Cfg_Test
					TIM_Cfg_Test.TimerPWMCfg = NULL
					
			For PWM timer, init TIM_PWM_Cfg_Typedef and link to TIM_Cfg_Typedef
				  The TimerMode should be "TIM_MODE_PWM
			
@Step3: Call Timer_Allocate() function to find a timer which can be used
				Note, if return value equals to TIMER_ERROR, means no timer resource is available.
			For example:
				Timer_ID = Timer_Allocate(&Tim_Cfg_Test);

@Step4: Call Start_Timer_Cnt() to kick off the timer

****Steps to Free a Timer ****
@Step1: Call void Timer_Free(Timer_ID_Typedef TIM_ID) to free a timer,
				This function will stop the timer, disable the timer, and free the timer resource;

****Interface Function ****
uint16_t Get_Timer_Cnt(Timer_ID_Typedef TIM_ID):
			----Get current timer value;
void Start_Timer_Cnt(Timer_ID_Typedef TIM_ID);
			----kick off a timer, note that the timer must be initialized before call
void Stop_Timer_Cnt(Timer_ID_Typedef TIM_ID);
			----stop a timer, DO NOT disable the interrupt
void Clear_Timer_Cnt(Timer_ID_Typedef TIM_ID);
			----Reset the timer value to all zeros


*/




#include "BasicTimer.h"
#include "stm32l1xx_tim.h"


/*Timer task table for all timer*/
static TIM_Task_Table_Typedef mTimerTaskEntryTable[TIM_MAX_NUM];

/*Declare local static function*/
static TIM_TypeDef* Locate_Tim_Entity(Timer_ID_Typedef enuTimID);
static IRQn_Type Locate_Tim_IRQn(Timer_ID_Typedef enuTimID);
static void Timer_Init(Timer_ID_Typedef TIM_ID, TIM_TypeDef* pTIM, TIM_Cfg_Typedef *pTimerCfgDat);
static void Timer_DeInit(Timer_ID_Typedef TIM_ID, TIM_TypeDef* pTIM);
static void Timer_IRQ_Disable(Timer_ID_Typedef TIM_ID);
static void Timer_IRQ_Enable(Timer_ID_Typedef TIM_ID);
/******************************************************************
*                        Locate_Tim_Entity                        *
* [Yun] return the timer base address of                          * 
******************************************************************/
static TIM_TypeDef* Locate_Tim_Entity(Timer_ID_Typedef enuTimID)
{
	switch(enuTimID)
	{
		case TIMER_2: return TIM2;
		case TIMER_3: return TIM3;
		case TIMER_4: return TIM4;
		case TIMER_5: return TIM5;
		case TIMER_6: return TIM6;
		case TIMER_7: return TIM7;
		case TIMER_9: return TIM9;
		case TIMER_10: return TIM10;
		case TIMER_11: return TIM11;
		default: return TIM2;
	}
}
/******************************************************************
*                        Locate_Tim_IRQn                          *
* [Yun] return the IRQ channel                                    * 
******************************************************************/
static IRQn_Type Locate_Tim_IRQn(Timer_ID_Typedef enuTimID)
{
	switch(enuTimID)
	{
		case TIMER_2: return TIM2_IRQn;
		case TIMER_3: return TIM3_IRQn;
		case TIMER_4: return TIM4_IRQn;
		case TIMER_5: return TIM5_IRQn;
		case TIMER_6: return TIM6_IRQn;
		case TIMER_7: return TIM7_IRQn;
		case TIMER_9: return TIM9_IRQn;
		case TIMER_10: return TIM10_IRQn;
		case TIMER_11: return TIM11_IRQn;
		default: return TIM2_IRQn; 		//as default, not sure
	}
}

/******************************************************************
*                        Timer_Allocate                           *
* [Yun] allocate the timer and init the timer                     * 
******************************************************************/
Timer_ID_Typedef Timer_Allocate(TIM_Cfg_Typedef *pTimerCfgDat)
{
	int8_t i8LoopIndex;
	TIM_TypeDef* pTIM;
	Timer_ID_Typedef TIM_ID;


    
	/*For PWM Mode timer*/
	if(pTimerCfgDat->TimerMode == TIM_MODE_PWM)
	{
		/*Translate to timer ID*/
		if(pTimerCfgDat->TimerPWMCfg->TIM_ID_PWM == TIMER_5
			|| pTimerCfgDat->TimerPWMCfg->TIM_ID_PWM == TIMER_6)
		{
			return TIMER_ERROR;
		}
		else
		{
			TIM_ID = pTimerCfgDat->TimerPWMCfg->TIM_ID_PWM;
		}
		
		/*Get the timer base address*/
		pTIM = Locate_Tim_Entity(TIM_ID);;
		mTimerTaskEntryTable[TIM_ID].TIMx = pTIM;
		/*Init the timer*/
		Timer_Init(TIM_ID, pTIM, pTimerCfgDat);
		/*Fulfill the entry table*/
		mTimerTaskEntryTable[TIM_ID].enuTimStatus = Tim_Allocated;
		mTimerTaskEntryTable[TIM_ID].pIntCallBack = pTimerCfgDat->TimerPWMCfg->pIntCallBack;
		/*Disable Timer IRQ and Clear Pending bit*/
		Timer_IRQ_Disable(TIM_ID);
		TIM_ClearITPendingBit(pTIM, TIM_IT_Update);
		return TIM_ID;
	}
	else if(pTimerCfgDat->TimerMode == TIM_MODE_BASIC)
	{
		/*Find a slot for new timer*/
		//For basic mode, timer 6/7 is first option
		if(mTimerTaskEntryTable[4].enuTimStatus == Tim_Free)
		{
			i8LoopIndex = 4;
		}
		else if(mTimerTaskEntryTable[5].enuTimStatus == Tim_Free)
		{
			i8LoopIndex = 5;
		}
		else
		{
			for(i8LoopIndex = (TIM_MAX_NUM - 1); i8LoopIndex >= 0 && mTimerTaskEntryTable[i8LoopIndex].enuTimStatus != Tim_Free; i8LoopIndex--)
			{
				/*Empty loop to fine the first not allocated slot*/
			};
		}

		/*Translate to timer ID*/
		switch(i8LoopIndex)
		{
			case 0: TIM_ID = TIMER_2; break;
			case 1: TIM_ID = TIMER_3; break;
			case 2: TIM_ID = TIMER_4; break;
			case 3: TIM_ID = TIMER_5; break;
			case 4: TIM_ID = TIMER_6; break;
			case 5: TIM_ID = TIMER_7; break;
			case 6: TIM_ID = TIMER_9; break;
			case 7: TIM_ID = TIMER_10; break;
			case 8: TIM_ID = TIMER_11; break;
			default: return TIMER_ERROR;
		}
		/*Get the timer base address*/
		pTIM = Locate_Tim_Entity(TIM_ID);
		mTimerTaskEntryTable[TIM_ID].TIMx = pTIM;
		/*Init the timer*/
		Timer_Init(TIM_ID, pTIM, pTimerCfgDat);
		/*Fulfill the entry table*/
		mTimerTaskEntryTable[TIM_ID].enuTimStatus = Tim_Allocated;
		mTimerTaskEntryTable[TIM_ID].pIntCallBack = pTimerCfgDat->TimerBasicCfg->pIntCallBack;
		/*Disable Timer IRQ and Clear Pending bit*/
		Timer_IRQ_Disable(TIM_ID);
		TIM_ClearITPendingBit(pTIM, TIM_IT_Update);
		
//		printf("\t time allocate %d \r\n", TIM_ID);
		
		return TIM_ID;
	}
	else
	{
		/*reserved for future*/
		return TIMER_ERROR;
	}


}
/******************************************************************
*                        Timer_ChangeConfig                       *
* [Keke] change the timer config and init the timer               * 
******************************************************************/
void Timer_ChangeConfig(TIM_Cfg_Typedef *pTimerCfgDat,Timer_ID_Typedef TIM_ID)
{

	TIM_TypeDef* pTIM;
	/* 	Exit when ERROR TIMER 		*/
	if(TIM_ID == TIMER_ERROR)
	{
		return;
	}
	
	Stop_Timer_Cnt(TIM_ID);

	/*For PWM Mode timer*/
	if(pTimerCfgDat->TimerMode == TIM_MODE_PWM)
	{
	
		/*Get the timer base address*/
		pTIM = Locate_Tim_Entity(TIM_ID);
		mTimerTaskEntryTable[TIM_ID].TIMx = pTIM;
		/*Init the timer*/
		Timer_Init(TIM_ID, pTIM, pTimerCfgDat);
		/*Fulfill the entry table*/
		mTimerTaskEntryTable[TIM_ID].enuTimStatus = Tim_Allocated;
		mTimerTaskEntryTable[TIM_ID].pIntCallBack = pTimerCfgDat->TimerPWMCfg->pIntCallBack;
		/*Disable Timer IRQ and Clear Pending bit*/
		Timer_IRQ_Disable(TIM_ID);
		TIM_ClearITPendingBit(pTIM, TIM_IT_Update);
		
		Stop_Timer_Cnt(TIM_ID);
	}
	else if(pTimerCfgDat->TimerMode == TIM_MODE_BASIC)
	{
		
		/*Get the timer base address*/
		pTIM = Locate_Tim_Entity(TIM_ID);
		mTimerTaskEntryTable[TIM_ID].TIMx = pTIM;
		/*Init the timer*/
		Timer_Init(TIM_ID, pTIM, pTimerCfgDat);
		/*Fulfill the entry table*/
		mTimerTaskEntryTable[TIM_ID].enuTimStatus = Tim_Allocated;
		mTimerTaskEntryTable[TIM_ID].pIntCallBack = pTimerCfgDat->TimerBasicCfg->pIntCallBack;
		/*Disable Timer IRQ and Clear Pending bit*/
		Timer_IRQ_Disable(TIM_ID);
		TIM_ClearITPendingBit(pTIM, TIM_IT_Update);
		
		Stop_Timer_Cnt(TIM_ID);
		
		//printf("\t time allocate %d \r\n", TIM_ID);
	}
	else
	{
		/*reserved for future*/
		
	}


}
/******************************************************************
*                        Timer_Free                               *
* [Yun] free the timer and init the timer                         * 
******************************************************************/
void Timer_Free(Timer_ID_Typedef TIM_ID)
{
	TIM_TypeDef* pTIM;
	
	if(TIM_ID == TIMER_ERROR)
	{
		return;
	}
	/*Get the timer base address*/
	pTIM = mTimerTaskEntryTable[TIM_ID].TIMx;
	/*Stop the Timer*/
	Stop_Timer_Cnt(TIM_ID);
	/*DeInit the timer*/
	Timer_DeInit(TIM_ID, pTIM);
	/*Fulfill the entry table*/
	mTimerTaskEntryTable[TIM_ID].enuTimStatus = Tim_Free;
	mTimerTaskEntryTable[TIM_ID].pIntCallBack = NULL;
	/*Disable Timer IRQ and Clear Pending bit*/
	Timer_IRQ_Disable(TIM_ID);
	TIM_ClearITPendingBit(pTIM, TIM_IT_Update);
//	printf("\t time free %d \r\n", TIM_ID);
}

/******************************************************************
*                        Timer_Init                               *
* [Yun] init the timer                                            * 
******************************************************************/
static void Timer_Init(Timer_ID_Typedef TIM_ID, TIM_TypeDef* pTIM, TIM_Cfg_Typedef *pTimerCfgDat)
{
	IRQn_Type TIM_IRQn_Ch;
	NVIC_InitTypeDef   TIM_NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef		TIM_BaseInitStructure;
	TIM_OCInitTypeDef 			TIM_OCInitStructure;
	/*Get the IRQn channel*/
	TIM_IRQn_Ch = Locate_Tim_IRQn(TIM_ID);

	/*Init the RCC */
	switch(TIM_ID)
	{
		case TIMER_2:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
			break;
		case TIMER_3:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
			break;
		case TIMER_4:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
			break;
		case TIMER_5:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
			break;
		case TIMER_6:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
			break;
		case TIMER_7:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
			break;
		case TIMER_9:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
			break;
		case TIMER_10:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
			break;
		case TIMER_11:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
			break;
		default:
			break;
	}


	if(pTimerCfgDat->TimerMode == TIM_MODE_PWM)
	{
		/*Init the PWM timer*/
		TIM_NVIC_InitStructure.NVIC_IRQChannel = TIM_IRQn_Ch;
		TIM_NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = pTimerCfgDat->TimerBasicCfg->NVIC_IRQChannelPreemptionPriority;
		TIM_NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
		TIM_NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	    NVIC_Init(&TIM_NVIC_InitStructure);
		/*Timer basic configure*/
		if(pTimerCfgDat->TimerPWMCfg->enuTimerType == TIM_TYPE_US)
		{
			TIM_BaseInitStructure.TIM_Prescaler = TIM_PRESCALER_1US;
		}
		else if(pTimerCfgDat->TimerPWMCfg->enuTimerType == TIM_TYPE_MS)
		{
			TIM_BaseInitStructure.TIM_Prescaler = TIM_PRESCALER_1MS;
		}
		TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_BaseInitStructure.TIM_Period = pTimerCfgDat->TimerPWMCfg->PWM_Period;
		//fixed as divide one
		TIM_BaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

		//init timer
		TIM_TimeBaseInit(pTIM, &TIM_BaseInitStructure);
		/*Init to PWM mode*/
		TIM_OCInitStructure.TIM_OCMode 			= TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputState 	= TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse 			= pTimerCfgDat->TimerPWMCfg->PWM_Duty;
		TIM_OCInitStructure.TIM_OCPolarity		= pTimerCfgDat->TimerPWMCfg->PWM_Polarity;
		
		switch(pTimerCfgDat->TimerPWMCfg->PWM_Ch)
		{
			case TIM_PWM_CH1: 
				TIM_OC1Init(pTIM,&TIM_OCInitStructure);
				TIM_OC1PreloadConfig(pTIM, TIM_OCPreload_Enable);
				break;
			case TIM_PWM_CH2:
				TIM_OC2Init(pTIM,&TIM_OCInitStructure);
				TIM_OC2PreloadConfig(pTIM, TIM_OCPreload_Enable);
				break;
			case TIM_PWM_CH3: 
				TIM_OC3Init(pTIM,&TIM_OCInitStructure);
				TIM_OC3PreloadConfig(pTIM, TIM_OCPreload_Enable);
				break;
			case TIM_PWM_CH4:
				TIM_OC4Init(pTIM,&TIM_OCInitStructure);
				TIM_OC4PreloadConfig(pTIM, TIM_OCPreload_Enable);
				break;
			default: break;
		}
	}
	else
		/*Init the Basic Timer*/
	{
		TIM_NVIC_InitStructure.NVIC_IRQChannel = TIM_IRQn_Ch;
		TIM_NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = pTimerCfgDat->TimerBasicCfg->NVIC_IRQChannelPreemptionPriority;
		TIM_NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
		TIM_NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	    NVIC_Init(&TIM_NVIC_InitStructure);

		/*Timer basic configure*/
		if(pTimerCfgDat->TimerBasicCfg->enuTimerType == TIM_TYPE_US)
		{
			TIM_BaseInitStructure.TIM_Prescaler = TIM_PRESCALER_1US;
		}
		else if(pTimerCfgDat->TimerBasicCfg->enuTimerType == TIM_TYPE_MS)
		{
			TIM_BaseInitStructure.TIM_Prescaler = TIM_PRESCALER_1MS;
		}
		TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_BaseInitStructure.TIM_Period = pTimerCfgDat->TimerBasicCfg->u16TimePeriod;
		//fixed as divide one
		TIM_BaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

		//init timer
		TIM_TimeBaseInit(pTIM, &TIM_BaseInitStructure);
	}
}



/******************************************************************
*                        Timer_DeInit                             *
* [Yun] destroy the timer                                         * 
******************************************************************/
static void Timer_DeInit(Timer_ID_Typedef TIM_ID, TIM_TypeDef* pTIM)
{
	IRQn_Type TIM_IRQn_Ch;
	NVIC_InitTypeDef   TIM_NVIC_InitStructure;

	/*Get the IRQn channel*/
	/*Get the IRQn channel*/
	TIM_IRQn_Ch = Locate_Tim_IRQn(TIM_ID);

	/*DeInit the RCC */
	switch(TIM_ID)
	{
		case TIMER_2:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, DISABLE);
			break;
		case TIMER_3:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE);
			break;
		case TIMER_4:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, DISABLE);
			break;
		case TIMER_5:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, DISABLE);
			break;
		case TIMER_6:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, DISABLE);
			break;
		case TIMER_7:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, DISABLE);
			break;
		case TIMER_9:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, DISABLE);
			break;
		case TIMER_10:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, DISABLE);
			break;
		case TIMER_11:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, DISABLE);
			break;
		default:
			break;
	}

	TIM_DeInit(pTIM);
	/*Timer IRQ disable*/
	TIM_ITConfig(pTIM, TIM_IT_Update, DISABLE);
	/*Disable NVIC*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	TIM_NVIC_InitStructure.NVIC_IRQChannel = TIM_IRQn_Ch;
	TIM_NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	TIM_NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	TIM_NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  	NVIC_Init(&TIM_NVIC_InitStructure);
}



/******************************************************************
*                        Timer_IRQ_Enable                              *
* [Yun] enable timer interrupt                                    * 
******************************************************************/
static void Timer_IRQ_Enable(Timer_ID_Typedef TIM_ID)
{
	TIM_TypeDef* pTIM;
	
	if(TIM_ID == TIMER_ERROR)
	{
		return;
	}
	
	/*Get the timer base address*/
	pTIM = mTimerTaskEntryTable[TIM_ID].TIMx;
	/*Enable IRQ*/
	TIM_ITConfig(pTIM, TIM_IT_Update, ENABLE);
}

/******************************************************************
*                        Timer_IRQ_Disable                        *
* [Yun] Disable timer interrupt                                   * 
******************************************************************/
static void Timer_IRQ_Disable(Timer_ID_Typedef TIM_ID)
{
	TIM_TypeDef* pTIM;
	if(TIM_ID == TIMER_ERROR)
	{
		return;
	}
	/*Get the timer base address*/
	pTIM = mTimerTaskEntryTable[TIM_ID].TIMx;
	/*Disable IRQ*/
	TIM_ITConfig(pTIM, TIM_IT_Update, DISABLE);
}

/******************************************************************
*                        Get_Timer_Cnt                            *
* [Yun] get current counter value                                 * 
******************************************************************/
uint16_t Get_Timer_Cnt(Timer_ID_Typedef TIM_ID)
{
	TIM_TypeDef* pTIM;
	
	if(TIM_ID == TIMER_ERROR)
	{
		return 0;
	}
	
	/*Get the timer base address*/
	pTIM = mTimerTaskEntryTable[TIM_ID].TIMx;

	return (TIM_GetCounter(pTIM));
}

/******************************************************************
*                        Start_Timer_Cnt                          *
* [Yun] kick off timer                                            * 
******************************************************************/
void Start_Timer_Cnt(Timer_ID_Typedef TIM_ID)
{
	TIM_TypeDef* pTIM;
	/* 	Exit when TIMER ERROR		*/
	if(TIM_ID == TIMER_ERROR)
	{
		return;
	}
	/*Get the timer base address*/
	pTIM = mTimerTaskEntryTable[TIM_ID].TIMx;
	//clear timer
	Clear_Timer_Cnt(TIM_ID);
	//enable timer interrupt
	Timer_IRQ_Enable(TIM_ID);
	//enable counter
	TIM_Cmd(pTIM, ENABLE);
}

/******************************************************************
*                        Stop_Timer_Cnt                           *
* [Yun] stop a timer                                              * 
******************************************************************/
void Stop_Timer_Cnt(Timer_ID_Typedef TIM_ID)
{
	TIM_TypeDef* pTIM;
	/* 	Exit when TIMER ERROR		*/
	if(TIM_ID == TIMER_ERROR)
	{
		return;
	}
	/*Get the timer base address*/
	pTIM = mTimerTaskEntryTable[TIM_ID].TIMx;
	TIM_Cmd(pTIM, DISABLE);
	Timer_IRQ_Disable(TIM_ID);
}

/******************************************************************
*                        Clear_Timer_Cnt                          *
* [Yun] stop a timer                                              * 
******************************************************************/
void Clear_Timer_Cnt(Timer_ID_Typedef TIM_ID)
{
	TIM_TypeDef* pTIM;
	/* 	Exit when TIMER ERROR		*/
	if(TIM_ID == TIMER_ERROR)
	{
		return;
	}
	/*Get the timer base address*/
	pTIM = mTimerTaskEntryTable[TIM_ID].TIMx;
	TIM_SetCounter(pTIM, 0x00000000);
}

/******************************************************************
*                        Timer_Callback                           *
* [Yun] Timer_Handler                                             * 
******************************************************************/
void Timer_Callback(Timer_ID_Typedef TIM_ID)
{
	/* 	Exit when TIMER ERROR		*/
	if(TIM_ID == TIMER_ERROR)
	{
		return;
	}
	if(mTimerTaskEntryTable[TIM_ID].enuTimStatus == Tim_Allocated && mTimerTaskEntryTable[TIM_ID].pIntCallBack!=NULL)
	{
		(mTimerTaskEntryTable[TIM_ID].pIntCallBack)(TIM_ID);
	}
}


//end file

