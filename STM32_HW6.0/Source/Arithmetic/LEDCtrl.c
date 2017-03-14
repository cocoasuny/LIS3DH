//******************************************************************************
//  Claude ZHU
//  Hardware Team
//  (C) iCareTech Inc., 2013
//  All Rights Reserved.
//  Built with Keil MDK
//
//-------------------------------------------------------------------------------

#include "LEDCtrl.h"
#include "SpO2_Top_Level.h"
#include "platform.h"

static Timer_ID_Typedef     gLed0TIMID = TIMER_ERROR;  //LED0
static Timer_ID_Typedef 	gLed1TIMID = TIMER_ERROR;  //LED1
static Timer_ID_Typedef 	gLedTIMID = TIMER_ERROR;  //LED
static LED_Attribute_Typedef   gLed_Attribute;

//LED pin definition, depend on your board configuration
		
#ifdef BOARD_REDHARE_V3_0
	static LED_PIN_Typedef LED0 = {GPIO_LED0, GPIO_Pin_LED0};
	static LED_PIN_Typedef LED1 = {GPIO_LED1, GPIO_Pin_LED1};
#endif	

#ifdef BOARD_REDHARE_V3_0	
static void LED0_Breath_INT_Handle(Timer_ID_Typedef timer_id);
static void LED1_Breath_INT_Handle(Timer_ID_Typedef timer_id);


	#ifndef LED_PWM_DMA_EN
		static uint16_t 			gPWMStepLED0;
		static uint16_t 			gPWMStepLED1;
			
		static LED_Slope_Typedef		gLed0Slope;
		static LED_Slope_Typedef 		gLed1Slope;
	
		static uint16_t 			gLed0LastTime;
		static uint16_t 			gLed1LastTime;
	
		static uint16_t 			gLed0IntCnt;
		static uint16_t 			gLed1IntCnt;
	#else
		static uint16_t 			gPwmDmaLedBuff[LED_DMA_BUF_LEN]=
					{
					0  ,100  ,200  ,300  ,400  ,500  ,600  ,700  ,800  ,900  ,
					1000  ,1100  ,1200  ,1300  ,1400  ,1500  ,1600  ,1700  ,1800  ,1900  ,
					2000  ,2100  ,2200  ,2300  ,2400  ,2500  ,2600  ,2700  ,2800  ,2900  ,
					3000  ,3100  ,3200  ,3300  ,3400  ,3500  ,3600  ,3700  ,3800  ,3900  ,
					4000  ,4100  ,4200  ,4300  ,4400  ,4500  ,4600  ,4700  ,4800  ,4900  ,
					5000  ,5100  ,5200  ,5300  ,5400  ,5500  ,5600  ,5700  ,5800  ,5900  ,
					6000  ,6100  ,6200  ,6300  ,6400  ,6500  ,6600  ,6700  ,6800  ,6900  ,
					7000  ,7100  ,7200  ,7300  ,7400  ,7500  ,7600  ,7700  ,7800  ,7900  ,
					8000  ,8100  ,8200  ,8300  ,8400  ,8500  ,8600  ,8700  ,8800  ,8900  ,
					9000  ,9100  ,9200  ,9300  ,9400  ,9500  ,9600  ,9700  ,9800  ,9900  ,
					10000  ,10100  ,10200  ,10300  ,10400  ,10500  ,10600  ,10700  ,10800  ,
					10900  ,11000  ,11100  ,11200  ,11300  ,11400  ,11500  ,11600  ,11700  ,
					11800  ,11900  ,12000  ,12100  ,12200  ,12300  ,12400  ,12500  ,12600  ,
					12700  ,12800  ,12900  ,13000  ,13100  ,13200  ,13300  ,13400  ,13500  ,
					13600  ,13700  ,13800  ,13900  ,14000  ,14100  ,14200  ,14300  ,14400  ,
					14500  ,14600  ,14700  ,14800  ,14900
					};
	#endif
#endif
	
/*Declare internal functions*/

void LED_GPIO_Init(void)
{
	/* GPIOLED Periph clock enable */
	GPIO_InitTypeDef        GPIO_InitStructure;

	#ifdef BOARD_REDHARE_V3_0

		/* GPIOLED Periph clock enable */
		RCC_AHBPeriphClockCmd(RCC_LEDPeriph_LED0, ENABLE);
		RCC_AHBPeriphClockCmd(RCC_LEDPeriph_LED1, ENABLE);
		/* Configure  LED0 pin in output pushpull mode */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_LED0;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIO_LED0, &GPIO_InitStructure);
	
		/* Configure  LED pins in output pushpull mode */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_LED1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIO_LED1, &GPIO_InitStructure);
	#endif
}

void LED_GPIO_DeInit(void)
{
	GPIO_InitTypeDef        GPIO_InitStructure;
	/* GPIOLED Periph clock enable */


	#ifdef BOARD_REDHARE_V3_0
	
		/* GPIOLED Periph clock enable */
		RCC_AHBPeriphClockCmd(RCC_LEDPeriph_LED0, ENABLE);
		RCC_AHBPeriphClockCmd(RCC_LEDPeriph_LED1, ENABLE);
		/* Configure  LED0 pin in output pushpull mode */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_LED0;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIO_LED0, &GPIO_InitStructure);
	
		/* Configure  LED pins in output pushpull mode */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_LED1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIO_LED1, &GPIO_InitStructure);
		/* GPIOLED Periph clock enable */
		RCC_AHBPeriphClockCmd(RCC_LEDPeriph_LED0, DISABLE);
		RCC_AHBPeriphClockCmd(RCC_LEDPeriph_LED1, DISABLE);
	#endif
}




/* Init the LED to PWM mode */
/* flashLastTime: 1.5ms(period) unit */

void LED_PWM_Start(LED_Flash_Speed_Typedef led_flash_speed, LED_TurnOn_Typedef led_sel, uint16_t flashLastTime)
{
	#ifdef BOARD_REDHARE_V3_0
		GPIO_InitTypeDef        GPIO_InitStructure;
		TIM_Cfg_Typedef Tim_Cfg_Test;
		TIM_PWM_Cfg_Typedef Tim_PWM_Cfg_Test;
		
		#ifdef LED_PWM_DMA_EN
			DMA_InitTypeDef DMA_InitStructure;
		#endif
	
		/* RED LED flash */
		if(led_sel == LED_RED || led_sel == LED_ORANGE)
		{
			/* init the pwm step and slope */
			#ifndef LED_PWM_DMA_EN
				gPWMStepLED0 = 0;
				gLed0Slope = LED_INC;
				gLed0LastTime = flashLastTime;
				gLed0IntCnt = 0;
			#endif
			
			/* GPIOLED Periph clock enable */
			RCC_AHBPeriphClockCmd(RCC_LEDPeriph_LED0, ENABLE);
			
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_LED0;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIO_LED0, &GPIO_InitStructure);
			GPIO_PinAFConfig(GPIO_LED0, GPIO_LED0_RESOURCE, PT_LED0_GPIO_AF_PWM_TIM);
			
			/*init timer 2 to PWM mode*/
			Tim_PWM_Cfg_Test.enuTimerType = TIM_TYPE_US;
			Tim_PWM_Cfg_Test.TIM_ID_PWM = TIMER_2;
			Tim_PWM_Cfg_Test.PWM_Period = LED_PWM_PERIOD;			//1ms
			Tim_PWM_Cfg_Test.PWM_Duty = 0;			//default value
			Tim_PWM_Cfg_Test.PWM_Polarity = TIM_OCPolarity_High;	
			Tim_PWM_Cfg_Test.PWM_Ch = PT_LED0_PWM_CH;
			
			#ifndef LED_PWM_DMA_EN
				Tim_PWM_Cfg_Test.pIntCallBack = LED0_Breath_INT_Handle;
			#else
				Tim_PWM_Cfg_Test.pIntCallBack = NULL;
			#endif

			Tim_Cfg_Test.TimerMode = TIM_MODE_PWM;
			Tim_Cfg_Test.TimerBasicCfg = NULL;
			Tim_Cfg_Test.TimerPWMCfg = &Tim_PWM_Cfg_Test;
			
			/* DMA Configuration */
			#ifdef LED_PWM_DMA_EN
				/* Enable DMA clock */
				RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
				
				/* DMA config in TIMx */
				TIM_DMAConfig(PT_LED0_PWM_TIM, PT_LED0_PWM_DMA_BASE, TIM_DMABurstLength_1Transfer);
				TIM_SelectCCDMA(PT_LED0_PWM_TIM, ENABLE);
				
				/* DMA Channel Config for LED0 */
				DMA_InitStructure.DMA_PeripheralBaseAddr = LED0_DMA_TIM2_DMAR_ADDR;
				DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)gPwmDmaLedBuff;
				DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;					//read from memory
				DMA_InitStructure.DMA_BufferSize = LED_DMA_BUF_LEN;					//150
				DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 	//Not increase
				DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;			//Not increase
				DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //16 bits
				DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //16 bits
				DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 					//circular mode
				DMA_InitStructure.DMA_Priority = DMA_Priority_Low; 					//low priority
				DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;						//not memory to memory
			
				DMA_Init(LED0_DMA_CHANNEL, &DMA_InitStructure);
				
				/* Disable DMA interrupt */
				DMA_ITConfig(LED0_DMA_CHANNEL, (DMA_IT_TC|DMA_IT_HT|DMA_IT_TE), DISABLE);
				
				/* DMA Enable */
				DMA_Cmd(LED0_DMA_CHANNEL, ENABLE);
				
			#endif


			/* pwm step setting */
			#ifndef LED_PWM_DMA_EN
				if(led_flash_speed == LED_FLASH_FAST)
				{
					gPWMStepLED0 = LED_PWM_STEP_FAST;
				}
				else if(led_flash_speed == LED_FLASH_SLOW)
				{
					gPWMStepLED0 = LED_PWM_STEP_SLOW;
				}
				else
				{
				}
			#endif

			gLed0TIMID = Timer_Allocate(&Tim_Cfg_Test);
			#ifndef LED_PWM_DMA_EN
				Start_Timer_Cnt(gLed0TIMID);
			#else
				TIM_Cmd(PT_LED0_PWM_TIM, ENABLE);
			#endif
			
			#ifdef LED_PWM_DMA_EN
				TIM_DMACmd(PT_LED0_PWM_TIM, PT_LED0_PWM_DMA_SOURCE, ENABLE);
			#endif
		}
		
		/* GREEN LED Flash */
		if(led_sel == LED_GREEN || led_sel == LED_ORANGE)
		{
			/* init the pwm step and slope */
			#ifndef LED_PWM_DMA_EN
				gPWMStepLED1 = 0;
				gLed1Slope = LED_INC;
				gLed1LastTime = flashLastTime;
				gLed1IntCnt = 0;
			#endif
			
			/* GPIOLED Periph clock enable */
			RCC_AHBPeriphClockCmd(RCC_LEDPeriph_LED1, ENABLE);
			
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_LED1;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIO_LED1, &GPIO_InitStructure);
			GPIO_PinAFConfig(GPIO_LED1, GPIO_LED1_RESOURCE, PT_LED1_GPIO_AF_PWM_TIM);
			
			/*init timer 2 to PWM mode*/
			Tim_PWM_Cfg_Test.enuTimerType = TIM_TYPE_US;
			Tim_PWM_Cfg_Test.TIM_ID_PWM = TIMER_3;
			Tim_PWM_Cfg_Test.PWM_Period = LED_PWM_PERIOD;			//1ms
			Tim_PWM_Cfg_Test.PWM_Duty = 0;			//default value
			Tim_PWM_Cfg_Test.PWM_Polarity = TIM_OCPolarity_High;	
			Tim_PWM_Cfg_Test.PWM_Ch = PT_LED1_PWM_CH;
			#ifndef LED_PWM_DMA_EN
				Tim_PWM_Cfg_Test.pIntCallBack = LED1_Breath_INT_Handle;
			#else
				Tim_PWM_Cfg_Test.pIntCallBack = NULL;
			#endif

			Tim_Cfg_Test.TimerMode = TIM_MODE_PWM;
			Tim_Cfg_Test.TimerBasicCfg = NULL;
			Tim_Cfg_Test.TimerPWMCfg = &Tim_PWM_Cfg_Test;
			
			/* DMA Configuration */
			#ifdef LED_PWM_DMA_EN
				/* Enable DMA clock */
				RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
				
				/* DMA config in TIMx */
				TIM_DMAConfig(PT_LED1_PWM_TIM, PT_LED1_PWM_DMA_BASE, TIM_DMABurstLength_1Transfer);
				TIM_SelectCCDMA(PT_LED1_PWM_TIM, ENABLE);
				
				/* DMA Channel Config for LED0 */
				DMA_InitStructure.DMA_PeripheralBaseAddr = LED1_DMA_TIM3_DMAR_ADDR;
				DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)gPwmDmaLedBuff;
				DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;					//read from memory
				DMA_InitStructure.DMA_BufferSize = LED_DMA_BUF_LEN;					//150
				DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 	//Not increase
				DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;			//Not increase
				DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //16 bits
				DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //16 bits
				DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 					//circular mode
				DMA_InitStructure.DMA_Priority = DMA_Priority_Low; 					//low priority
				DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;						//not memory to memory
			
				DMA_Init(LED1_DMA_CHANNEL, &DMA_InitStructure);
				
				/* Disable DMA interrupt */
				DMA_ITConfig(LED1_DMA_CHANNEL, (DMA_IT_TC|DMA_IT_HT|DMA_IT_TE), DISABLE);
				
				/* DMA Enable */
				DMA_Cmd(LED1_DMA_CHANNEL, ENABLE);
				
			#endif
			
			#ifndef LED_PWM_DMA_EN
				/* pwm step setting */
				if(led_flash_speed == LED_FLASH_FAST)
				{
					gPWMStepLED1 = LED_PWM_STEP_FAST;
				}
				else if(led_flash_speed == LED_FLASH_SLOW)
				{
					gPWMStepLED1 = LED_PWM_STEP_SLOW;
				}
				else
				{
				}
			#endif
			gLed1TIMID = Timer_Allocate(&Tim_Cfg_Test);
				
			#ifndef LED_PWM_DMA_EN
				Start_Timer_Cnt(gLed1TIMID);
			#else
				TIM_Cmd(PT_LED1_PWM_TIM, ENABLE);
			#endif
			
			#ifdef LED_PWM_DMA_EN
				TIM_DMACmd(PT_LED1_PWM_TIM, PT_LED1_PWM_DMA_SOURCE, ENABLE);
			#endif
		}
		
		
	#endif	
}

/* Deinit the LED to PWM mode */
/* Only for Redhare V2.0*/
void LED_PWM_Stop(LED_TurnOn_Typedef led_sel)
{
	#ifdef BOARD_REDHARE_V3_0
		GPIO_InitTypeDef        GPIO_InitStructure;
		/* RED or ALL */
		if(led_sel == LED_RED || led_sel == LED_ORANGE)
		{			
			/* clear glasttime */
			gLed0LastTime = 0;
			gLed0IntCnt = 0;
			/* GPIOLED Periph clock enable */
			RCC_AHBPeriphClockCmd(RCC_LEDPeriph_LED0, ENABLE);
			/* Configure  LED0 pin in output pushpull mode */
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_LED0;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIO_LED0, &GPIO_InitStructure);
			
			/* Free timer */
			Stop_Timer_Cnt(gLed0TIMID);
			Timer_Free(gLed0TIMID);
			gLed0TIMID = TIMER_ERROR;
			/* GPIOLED Periph clock enable */
			RCC_AHBPeriphClockCmd(RCC_LEDPeriph_LED0, DISABLE);
		}
		/* GREEN or ALL */
		if(led_sel == LED_GREEN || led_sel == LED_ORANGE)
		{			
			/* clear glasttime */
			gLed1LastTime = 0;
			gLed1IntCnt = 0;
			/* GPIOLED Periph clock enable */
			RCC_AHBPeriphClockCmd(RCC_LEDPeriph_LED1, ENABLE);
			/* Configure  LED0 pin in output pushpull mode */
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_LED1;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIO_LED1, &GPIO_InitStructure);
			
			/* Free timer */
			Stop_Timer_Cnt(gLed1TIMID);
			Timer_Free(gLed1TIMID);
			gLed1TIMID = TIMER_ERROR;
			
			/* GPIOLED Periph clock diable */
			RCC_AHBPeriphClockCmd(RCC_LEDPeriph_LED1, DISABLE);
		}
		

	#endif
}


/******************************************************************
*                        LED_ON                                   *
* [Yun] TURN ON A LED                                             * 
******************************************************************/
void LED_ON(LED_PIN_Typedef *LED)
{
    GPIO_SetBits(LED->GPIO_Bank, LED->LED_PIN);
}

/******************************************************************
*                        LED_Off                                  *
* [Yun] TURN Off A LED                                             * 
******************************************************************/
void LED_Off(LED_PIN_Typedef *LED)
{
    GPIO_ResetBits(LED->GPIO_Bank, LED->LED_PIN);
}

/******************************************************************
*                        LED_FLASH_Toggle                         *
* [Yun] invert the LED pin status                                 * 
******************************************************************/
void LED_FLASH_Toggle(LED_PIN_Typedef *LED)
{
	//GPIO_ToggleBits(LED->GPIO_Bank, LED->LED_PIN);     //屏蔽LED提示
}

void LED_FLASH_LED0(Timer_ID_Typedef TIM_ID)
{
	
	  LED_FLASH_Toggle(&LED0);
}

void LED_FLASH_LED1(Timer_ID_Typedef TIM_ID)
{
	  LED_FLASH_Toggle(&LED1);
}


/*******************************************************************************
* Function Name  : PWM Int handler
* Description    : To increase PWM duty value, for each period interrupt
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void LED0_Breath_INT_Handle(Timer_ID_Typedef timer_id)
{
	uint16_t pwm_duty;
	
	/*Change Channel if use other channels */
	#ifdef BOARD_REDHARE_V3_0
		#ifndef LED_PWM_DMA_EN
			/* Get LED0 PWM Timer's period and duty*/
			pwm_duty = PT_LED0_PWM_TIM->CCR2;
			
			/* Increase Timer cnt */
			gLed0IntCnt++;
			if(gLed0IntCnt == gLed0LastTime)
			{
				TS_SendEvent(gLedTaskID, gLedEventRedBreathStop);
				return;
			}

			/* Increase duty when less than period*/
			if(gLed0Slope == LED_INC)
			{
				if( pwm_duty + gPWMStepLED0 >= LED_PWM_PERIOD)
				{
					gLed0Slope = LED_DEC;
				}
				else
				{
					pwm_duty = pwm_duty + gPWMStepLED0;
				}
			}
			if(gLed0Slope == LED_DEC)
			{
				if ( pwm_duty < gPWMStepLED0)
				{
					gLed0Slope = LED_INC;
				}
				else
				{
					pwm_duty = pwm_duty - gPWMStepLED0;
				}
			}


			/* write new duty */
			PT_LED0_PWM_TIM->CCR2 = pwm_duty;
		#endif
	#endif
}
/*******************************************************************************
* Function Name  : PWM Int handler
* Description    : To increase PWM duty value, for each period interrupt
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void LED1_Breath_INT_Handle(Timer_ID_Typedef timer_id)
{
	uint16_t pwm_duty;
	/*Change Channel if use other channels */
	#ifdef BOARD_REDHARE_V3_0
		#ifndef LED_PWM_DMA_EN
			/* Get LED0 PWM Timer's period and duty*/
			pwm_duty = PT_LED1_PWM_TIM->CCR1;
	
			/* Increase Timer cnt */
			gLed1IntCnt++;
			if(gLed1IntCnt == gLed1LastTime)
			{
				TS_SendEvent(gLedTaskID, gLedEventGreenBreathStop);
				return;
			}
			
			/* Increase duty when less than period*/
			if(gLed1Slope == LED_INC)
			{
				if( pwm_duty + gPWMStepLED1 >= LED_PWM_PERIOD)
				{
					gLed1Slope = LED_DEC;
				}
				else
				{
					pwm_duty = pwm_duty + gPWMStepLED1;
				}
			}
			if(gLed1Slope == LED_DEC)
			{
				if ( pwm_duty < gPWMStepLED1)
				{
					gLed1Slope = LED_INC;
				}
				else
				{
					pwm_duty = pwm_duty - gPWMStepLED1;
				}
			}
		
			/* write new duty */
			PT_LED1_PWM_TIM->CCR1 = pwm_duty;
		#endif
	#endif
}

/*******************************************************************************
* Function Name  : LedTask
* Description    : Led任务处理函数，定时器中每隔500ms产生一次Led事件,用于指示程序
*                  运行
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LedTask(event_t events)
{
		
	#ifdef BOARD_REDHARE_V3_0
//		if(events & gLedEventRedBreathFastStart)
//		{
//			LED_PWM_Start(LED_FLASH_FAST, LED_RED, LED_FLASH_TIMEOUT);
//		}
//		else if(events & gLedEventRedBreathSlowStart)
//		{
//			LED_PWM_Start(LED_FLASH_SLOW, LED_RED, LED_FLASH_TIMEOUT);
//		}
//		else if(events & gLedEventGreenBreathFastStart)
//		{
//			LED_PWM_Start(LED_FLASH_FAST, LED_GREEN, LED_FLASH_TIMEOUT);
//		}
//		else if(events & gLedEventGreenBreathSlowStart)
//		{
//			LED_PWM_Start(LED_FLASH_SLOW, LED_GREEN, LED_FLASH_TIMEOUT);
//		}
//		else if(events & gLedEventAllBreathFastStart)
//		{
//			LED_PWM_Start(LED_FLASH_FAST, LED_ORANGE, LED_FLASH_TIMEOUT);
//		}
//		else if(events & gLedEventAllBreathSlowStart)
//		{
//			LED_PWM_Start(LED_FLASH_SLOW, LED_ORANGE, LED_FLASH_TIMEOUT);
//		}
//		else if(events & gLedEventRedBreathStop)
//		{
//			LED_PWM_Stop(LED_RED);
//		}
//		else if(events & gLedEventGreenBreathStop)
//		{
//			LED_PWM_Stop(LED_GREEN);
//		}
//		else if(events & gLedEventAllBreathStop)
//		{
//			LED_PWM_Stop(LED_ORANGE);
//		}
//		else if(events & gLedEventLED0Toggle_c)
//		{
//			LED_FLASH_Toggle(&LED0);
//		}
//		else if(events & gLedEventLED1Toggle_c)
//		{
//			LED_FLASH_Toggle(&LED1);
//		}
	#endif
}


/*******************************************************************************
* Function Name  : LED_FLASH_GREEN
* Description    : LED绿灯闪烁
* Input          : LED Colour, Freq
* Output         : None
* Return         : None
*******************************************************************************/
static void LED_FLASH(Timer_ID_Typedef TIMID)
{
		static uint16_t FlashCnt=0;
	
		TIMID = TIMID;
	
		if(AlarmStatusGet() == true)
		{
			if(FlashCnt <= gLed_Attribute.LEDFlashTime)
			{
				if(gLed_Attribute.LEDColour == LED_RED)
				{
					LED_FLASH_Toggle(&LED0); 
				}
				else if(gLed_Attribute.LEDColour == LED_ORANGE)
				{
					LED_FLASH_Toggle(&LED0);
					LED_FLASH_Toggle(&LED1); 
				}
				else if(gLed_Attribute.LEDColour == LED_GREEN)
				{
					LED_FLASH_Toggle(&LED1);  
				}
				FlashCnt++;
			}
			else
			{
				FlashCnt = 0;
				LED_DeInit();
				
				AlarmStatusClear(ALARM_TYPE_LED_MASK);
				//gAlarmType.AlarmStatus = OFF;
				//gAlarmType.AlarmLevel.High = OFF;
				//gAlarmType.AlarmLevel.Middle = OFF;
				//gAlarmType.AlarmLevel.Low = OFF;
			}
		}
		else
		{
			LED_DeInit();
		}
}
/*******************************************************************************
* Function Name  : LED_Init
* Description    : LED闪烁,用于控制LED闪烁
* Input          : LED Colour, Freq, FlashTime(次数)
* Output         : None
* Return         : None
*******************************************************************************/
void LED_Init(LED_TurnOn_Typedef LED_Colour,uint16_t Tim,uint16_t FlashTime)
{
		TIM_Cfg_Typedef         Tim_Cfg_LED_Index;       //LED timer配置，用于指示程序运行
		TIM_Basic_Cfg_Typedef 	Tim_Cfg_LED;       		 //LED timer配置，用于指示程序运行
		
		LED_GPIO_Init();
	
		gLed_Attribute.LEDColour = LED_Colour;
		gLed_Attribute.LEDFlashTime = FlashTime;
		
		/* 配置LED定时器，1s闪烁，用于指示程序运行 */
		Tim_Cfg_LED.enuTimerType = TIM_TYPE_MS;
		Tim_Cfg_LED.u16TimePeriod = Tim;
		Tim_Cfg_LED.NVIC_IRQChannelPreemptionPriority = LED_TIMPreemptionPriority;
		Tim_Cfg_LED.pIntCallBack = LED_FLASH;

		/* Init timer top define */
		Tim_Cfg_LED_Index.TimerMode 			= TIM_MODE_BASIC;
		Tim_Cfg_LED_Index.TimerBasicCfg 		= &Tim_Cfg_LED;
		Tim_Cfg_LED_Index.TimerPWMCfg 			= NULL;

		gLedTIMID = Timer_Allocate(&Tim_Cfg_LED_Index);
		Start_Timer_Cnt(gLedTIMID);
}
/*******************************************************************************
* Function Name  : LED_DeInit
* Description    : LED DeInit
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LED_DeInit(void)
{
		LED_GPIO_DeInit();
		Timer_Free(gLedTIMID);
		gLedTIMID = TIMER_ERROR;
		gLed_Attribute.LEDColour = LED_ERROR;
		gLed_Attribute.LEDFlashTime = 0;
}


//end files

