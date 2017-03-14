/**
  ******************************************************************************
  * @file    Project/STM32L1xx_StdPeriph_Templates/stm32l1xx_it.c 
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    13-April-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_it.h"
#include "common.h"
#include "cc_app_afe_drv_interface.h"
#include "gSensor_data_process.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}
// hard fault handler in C,
// with stack frame location as input parameter
void hard_fault_handler_c(unsigned int * hardfault_args)
{
	date_str_typedef    date_s;               //RTC 日期
	RTC_TimeTypeDef     rtc_time;             //RTC 时间
    SysConfigCPUInfo_t  info;

	unsigned int stacked_r0;
	unsigned int stacked_r1;
	unsigned int stacked_r2;
	unsigned int stacked_r3;
	unsigned int stacked_r12;
	unsigned int stacked_lr;
	unsigned int stacked_pc;
	unsigned int stacked_psr;

	Calendar_Get(&date_s,&rtc_time);

	stacked_r0 = ((unsigned long) hardfault_args[0]);
	stacked_r1 = ((unsigned long) hardfault_args[1]);
	stacked_r2 = ((unsigned long) hardfault_args[2]);
	stacked_r3 = ((unsigned long) hardfault_args[3]);

	stacked_r12 = ((unsigned long) hardfault_args[4]);
	stacked_lr = ((unsigned long) hardfault_args[5]);
	stacked_pc = ((unsigned long) hardfault_args[6]);
	stacked_psr = ((unsigned long) hardfault_args[7]);


	printf ("[Hard fault handler]\r\n");
	printf("Hard fault occured Time(MM/DD/HH/MM/SS): %d:%d:%d:%d:%d\r\n",
						date_s.month,date_s.day,rtc_time.RTC_Hours,rtc_time.RTC_Minutes,rtc_time.RTC_Seconds);

	printf ("R0 = %0.8x\r\n", stacked_r0);
	printf ("R1 = %0.8x\r\n", stacked_r1);
	printf ("R2 = %0.8x\r\n", stacked_r2);
	printf ("R3 = %0.8x\r\n", stacked_r3);
	printf ("R12 = %0.8x\r\n", stacked_r12);
	printf ("LR = %0.8x\r\n", stacked_lr);
	printf ("PC = %0.8x\r\n", stacked_pc);
	printf ("PSR = %0.8x\r\n", stacked_psr);
	printf ("BFAR = %0.8x\r\n", (*((volatile unsigned long *)(0xE000ED38))));
	printf ("CFSR = %0.8x\r\n", (*((volatile unsigned long *)(0xE000ED28))));
	printf ("HFSR = %0.8x\r\n", (*((volatile unsigned long *)(0xE000ED2C))));
	printf ("DFSR = %0.8x\r\n", (*((volatile unsigned long *)(0xE000ED30))));
	printf ("AFSR = %0.8x\r\n", (*((volatile unsigned long *)(0xE000ED3C))));

	/* Write CPU Information to EEPROM */
	info.Month = date_s.month;
	info.Day = date_s.day;
	info.Hour = rtc_time.RTC_Hours;
	info.Min = rtc_time.RTC_Minutes;
	info.Sec = rtc_time.RTC_Seconds;

	/* LR */
    info.lr = stacked_lr;

	/* PC */
    info.pc = stacked_pc;

    SetCPUConfigInfo(info);

	while(1)
	{
	}

	/* SoftReset */
//	SoftReset();
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{	
//    uint32_t r_sp,i;
//	uint8_t i=0;
//    r_sp = __get_PSP();

//	  /* Go to infinite loop when Hard Fault exception occurs */
//	  while (1)
//	  {
//		    printf("\r\nr_sp = 0x%x\r\n",*(uint32_t *)r_sp);
//			for(i=0;i<10;i++)
//			{
//				printf("\r\n 0x%0.8x\r\n",*(uint32_t *)(r_sp+i*4));
//			}
//			while(1);
//	  }
}


/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
//void SysTick_Handler(void)
//{
//}

/******************************************************************************/
/*                 STM32L1xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l1xx_xx.s).                                            */
/******************************************************************************/
void EXTI0_IRQHandler(void)
{
		static uint8_t TIM4Ctl_Flag=0;
		uint16_t  TIM4Cnt=0;
		TIM_Cfg_Typedef    Tim_Cfg_Key_Index;           //Key timer配置，用于检测长、短按键 
		TIM_Basic_Cfg_Typedef 	Tim_Cfg_Key; 
	
		if(EXTI_GetITStatus(EXTI_Line0) != RESET) 
		{
			/* Clear the EXTI line 0 pending bit */
			EXTI_ClearITPendingBit(EXTI_Line0);	
			gWakeUpNeedOLEDON = SET;

			if(gKeyPressTIMOut == true) //按键超时，判断为长按后松开按键
			{
					gKeyPressTIMOut = false;
					TIM4Ctl_Flag = 0;
			}
			else if(gKeyPressStatus == Valid)
			{
				if(TIM4Ctl_Flag == 0)
				{
					if(GPIO_ReadInputDataBit(GPIO_KEYWP,GPIO_Pin_KEYWP) == Bit_RESET)
					{
						TIM4Ctl_Flag = 1;
						TIM4Cnt=0;
						
						if(AlarmStatusGet() == true)//按键清除告警(关灯&停止振动)
						{
							AlarmStatusClear(ALARM_TYPE_LED_MASK | ALARM_TYPE_MOTOR_MASK);
							//gAlarmType.AlarmStatus = OFF;
							//gAlarmType.AlarmLevel.High = OFF;
							//gAlarmType.AlarmLevel.Middle = OFF;
							//gAlarmType.AlarmLevel.Low = OFF;
							KillMotor();
						}
						if(Get_OLED_Dis_Status() == OLEDDisEnterShutDown)  //关闭屏幕计时使能，按键退出
						{
							if(gShutDownOledTIMID != TIMER_ERROR)
							{
								Stop_Timer_Cnt(gShutDownOledTIMID);
								Start_Timer_Cnt(gShutDownOledTIMID);  //ReStart Shut Down Oled timer
							}
						}
					    if(Get_OLED_Dis_Status() == OLEDDisShutDown)
						{
							if(gAlarmNotEnoughSpace == true)  /* In case, not enough alarm */
							{
								OLED_DisplayNoEnoughSpaceFlash(ON);
							}
							TS_SendEvent(gTsPowerManageTaskID_c,gPowerManageRecoverOLEDDis);    //屏幕关闭情况下恢复屏幕显示
							gKeyPressTIMOut = true;  //响应按下唤醒OLED显示后，也不响应其他按键操作
						}
						else if(OccupiedDis == OLEDDisplay_Stat_Get())  //占用显示可被按键清除
						{
							OLEDDisplay_Stat_Set(NormalDis);   //按键清除后，设置为正常显示模式	
							if(Device_Mode == Device_Mode_RTC)
							{
								TS_SendEvent(gOledDisTaskID,gOledDisEventBlueIcon_c);   //显示RTC模式
							}
							else
							{
								Set_OLED_Dis_Status(OLEDDisON);
								TS_SendEvent(gOledDisTaskID,gOledDisEventDisplayAllRAM_c);  //更新RAM区域显示值屏幕
								TS_SendEvent(gOledDisTaskID,gOledDisEventClearModeDis_c);
							}
						}
						else
						{
							Stop_Timer_Cnt(gKeyTIMID);
							Clear_Timer_Cnt(gKeyTIMID);
							Start_Timer_Cnt(gKeyTIMID);
						}
					}
				}
				else
				{
					if(GPIO_ReadInputDataBit(GPIO_KEYWP,GPIO_Pin_KEYWP) == Bit_SET)
					{
						TIM4Ctl_Flag = 0;
						TIM4Cnt = Get_Timer_Cnt(gKeyTIMID);
						Stop_Timer_Cnt(gKeyTIMID);
						
						if(TIM4Cnt < LongPressTIM)//1s
						{
							//Key_State = Key_State_Short;
							TS_SendEvent(gKeyTaskID,gKeyEventShortPress_c);
							gKeyPressStatus = InValid;
							
							/* 配置KEY定时器，用于按键无效检测*/
							Tim_Cfg_Key.enuTimerType = TIM_TYPE_MS;
							Tim_Cfg_Key.u16TimePeriod = KeyInvalidTIM;
							Tim_Cfg_Key.NVIC_IRQChannelPreemptionPriority = KEY_TIMPreemptionPriority;
							Tim_Cfg_Key.pIntCallBack = KeyPressDetect;

							/* Init timer top define */
							Tim_Cfg_Key_Index.TimerMode 			= TIM_MODE_BASIC;
							Tim_Cfg_Key_Index.TimerBasicCfg 		= &Tim_Cfg_Key;
							Tim_Cfg_Key_Index.TimerPWMCfg 			= NULL;

							Timer_ChangeConfig(&Tim_Cfg_Key_Index,gKeyTIMID);
							Start_Timer_Cnt(gKeyTIMID);
						}						
					}
				}
//				printf("Key_int\r\n");
			}
		}
}

void EXTI2_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_LineBAT_PG) != RESET)
    {
        /* Clear the EXT2 line 2 pending bit */
        EXTI_ClearITPendingBit(EXTI_LineBAT_PG);  
        
		gWakeUpNeedOLEDON = SET;
        if(GPIO_ReadInputDataBit(GPIO_BAT_PG,EXTI_LineBAT_PG) == Bit_RESET)
        {
            /* Send the event to BatManage task */
            TS_SendEvent(gTsBatManageTaskID_c,gBATEnterChargeEvent);
        }
        else
        {
            /* Send the event to BatManage task */
            TS_SendEvent(gTsBatManageTaskID_c,gBATExitChargeEvent);
        }  
    } 
}

void DMA1_Channel1_IRQHandler(void)
{
#ifdef INTERRUPT_DEBUG
	printf("int DMA 1 \r\n");
#endif
    DMA_ClearFlag(DMA1_IT_TC1);
    setADCDMA_TransferComplete();  /* set flag_ADCDMA_TransferComplete global flag */
}

void EXTI15_10_IRQHandler(void)
{
#ifdef BOARD_REDHARE_V3_0
	
  if(EXTI_GetITStatus(AFE_ADC_DRDY_EXTI_LINE) != RESET)
  {
    /* Clear the EXTI line pending bit */
    EXTI_ClearITPendingBit(AFE_ADC_DRDY_EXTI_LINE);
    gWakeUpNeedOLEDON = SET;
      
 	PulseOxiAfe_DRY_Handler();
  }
#endif
	
	
#ifdef BOARD_REDHARE_V3_0 
		if(EXTI_GetITStatus(PT_EXTI_LineBTREQN) != RESET) //PA15 51822通知STM32读取数据
		{	
			 /* Clear the EXTI line10 pending bit */
			EXTI_ClearITPendingBit(PT_EXTI_LineBTREQN);
			gWakeUpNeedOLEDON = SET;
			
			if(Device_Mode == Device_Mode_Factory)  //工厂模式下不能使用事件调度
			{
				SPITranslate_Task_Handler(gSPITranslateEventRx);
			}
			else
			{
				TS_SendEvent(gTsSPITranslateTaskID_c,gSPITranslateEventRx);   //触发SPI读取数据事件	
			}
		}
#endif
	
	/* 	No use of BMA250E int2 	*/	
	if(EXTI_GetITStatus(PT_BMA250E_INT2_EXTI_LINE) != RESET) //LIS3DH INT2
	{
		EXTI_ClearITPendingBit(PT_BMA250E_INT2_EXTI_LINE);
		printf("Double Click\r\n");
	}
	
}
/******************************************************************
*                        TIM2_IRQHandler                           *
*                        Timer 2 IRQ Handler                       * 
******************************************************************/
void TIM2_IRQHandler(void)						  //apply to every timer interrupt ISR
{	
	Timer_ID_Typedef Timer_ID = TIMER_2; 		  //consistant with current timer
	
	gWakeUpNeedOLEDON = SET;
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);   //consistant with current timer
	Timer_Callback(Timer_ID); 	   				  //input parameter is the current timer ID
}
/******************************************************************
*                        TIM3_IRQHandler                           *
*                        Timer 3 IRQ Handler                       * 
******************************************************************/
void TIM3_IRQHandler(void)						  //apply to every timer interrupt ISR
{
	Timer_ID_Typedef Timer_ID = TIMER_3; 		  //consistant with current timer

	gWakeUpNeedOLEDON = SET;
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);   //consistant with current timer
	Timer_Callback(Timer_ID);					  //input parameter is the current timer ID
}
/******************************************************************
*                        TIM4_IRQHandler                           *
*                        Timer 4 IRQ Handler                       * 
******************************************************************/
void TIM4_IRQHandler(void)						  //apply to every timer interrupt ISR
{
	Timer_ID_Typedef Timer_ID = TIMER_4; 		  //consistant with current time
	
	gWakeUpNeedOLEDON = SET;
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);   //consistant with current timer
	Timer_Callback(Timer_ID);					  //input parameter is the current timer ID
}
/******************************************************************
*                        TIM5_IRQHandler                           *
*                        Timer 5 IRQ Handler                       * 
******************************************************************/
void TIM5_IRQHandler(void)						  //apply to every timer interrupt ISR
{
	Timer_ID_Typedef Timer_ID = TIMER_5; 		  //consistant with current timer
	
	gWakeUpNeedOLEDON = SET;
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);   //consistant with current timer
	Timer_Callback(Timer_ID);					  //input parameter is the current timer ID
}
/******************************************************************
*                        TIM6_IRQHandler                           *
*                        Timer 6 IRQ Handler                       * 
******************************************************************/
void TIM6_IRQHandler(void)						  //apply to every timer interrupt ISR
{
	Timer_ID_Typedef Timer_ID = TIMER_6; 		  //consistant with current timer
	
	gWakeUpNeedOLEDON = SET;
	TIM_ClearITPendingBit(TIM6, TIM_IT_Update);   //consistant with current timer
	Timer_Callback(Timer_ID);					  //input parameter is the current timer ID
}
/******************************************************************
*                        TIM7_IRQHandler                           *
*                        Timer 7 IRQ Handler                       * 
******************************************************************/
void TIM7_IRQHandler(void)						  //apply to every timer interrupt ISR
{
	Timer_ID_Typedef Timer_ID = TIMER_7; 		  //consistant with current timer
	
#ifdef INTERRUPT_DEBUG
	printf("int TIM 7 \r\n");
#endif

	
	gWakeUpNeedOLEDON = SET;
	TIM_ClearITPendingBit(TIM7, TIM_IT_Update);   //consistant with current timer
	Timer_Callback(Timer_ID);					  //input parameter is the current timer ID
}
/******************************************************************
*                        TIM9_IRQHandler                           *
*                        Timer 9 IRQ Handler                       * 
******************************************************************/
void TIM9_IRQHandler(void)						  //apply to every timer interrupt ISR
{
	Timer_ID_Typedef Timer_ID = TIMER_9; 		  //consistant with current timer
	
	gWakeUpNeedOLEDON = SET;
	TIM_ClearITPendingBit(TIM9, TIM_IT_Update);   //consistant with current timer
	Timer_Callback(Timer_ID);					  //input parameter is the current timer ID
}
/******************************************************************
*                        TIM10_IRQHandler                           *
*                        Timer 10 IRQ Handler                       * 
******************************************************************/
void TIM10_IRQHandler(void)						  //apply to every timer interrupt ISR
{
	Timer_ID_Typedef Timer_ID = TIMER_10; 		  //consistant with current timer

	gWakeUpNeedOLEDON = SET;
	TIM_ClearITPendingBit(TIM10, TIM_IT_Update);   //consistant with current timer
	Timer_Callback(Timer_ID);					  //input parameter is the current timer ID
}
/******************************************************************
*                        TIM11_IRQHandler                           *
*                        Timer 11 IRQ Handler                       * 
******************************************************************/
void TIM11_IRQHandler(void)						  //apply to every timer interrupt ISR
{
	Timer_ID_Typedef Timer_ID = TIMER_11; 		  //consistant with current timer

	gWakeUpNeedOLEDON = SET;
	TIM_ClearITPendingBit(TIM11, TIM_IT_Update);   //consistant with current timer
	Timer_Callback(Timer_ID);					  //input parameter is the current timer ID
}
void RTC_WKUP_IRQHandler(void)
{
#ifdef INTERRUPT_DEBUG
	printf("int RTC wakeup \r\n");
#endif
	RTC_ClearITPendingBit(RTC_IT_WUT);
    EXTI_ClearITPendingBit(EXTI_Line20);
	/* 	If in low power mode, not to response RTC interrupt 		*/
	if(Device_Mode != Device_Mode_LowBattery)
	{
		/* If not in Device_Mode_SecTick or Device_Mode_RTC, disable the RTC interrupt */
		if((Device_Mode != Device_Mode_SecTick) && (Device_Mode != Device_Mode_RTC) && (isFreeRunKickOff()!=true))
		{
			Calendar_RTC_Period_Wakeup_DeInit();
		}
		else
		{
			Calendar_RTC_Wakeup_Int_Handler();
		}
	}
	else
	{
		Calendar_RTC_Wakeup_Int_Handler();
	}
    

}
void EXTI9_5_IRQHandler(void)
{
	#ifdef BOARD_REDHARE_V3_0
		
		if(EXTI_GetITStatus(PT_EXTI_LineBTSYNC) != RESET) //PC5 51822通知STM32发送数据
		{
			 /* Clear the EXTI line10 pending bit */
			EXTI_ClearITPendingBit(PT_EXTI_LineBTSYNC);
			gWakeUpNeedOLEDON = SET;
			TS_SendEvent(gTsSyncDataTaskID_c,gSyncDataEventSPISent);   //触发SPI发送数据事件
		}
	#endif
}

void EXTI1_IRQHandler(void)
{
}
/**
  * @brief  This function handles RTC Alarms interrupt request.
  * @param  None
  * @retval None
  */
void RTC_Alarm_IRQHandler(void)
{
#ifdef INTERRUPT_DEBUG
	printf("int RTC alarm \r\n");
#endif
		if(RTC_GetITStatus(RTC_IT_ALRA) != RESET)
		{
			RTC_ClearITPendingBit(RTC_IT_ALRA);
            /* Clear the EXTI line */	
            EXTI_ClearITPendingBit(EXTI_Line17);
			#ifdef Monitor_Template_Debug
			//printf("MT Alarm\r\n");
			#endif
			if(GetMonitorTemplateType() == NormalMT)
			{
				TS_SendEvent(gTsMonitorTemplatID_c,gMonitorTemplateEventSampleData);  //发送正常监测模板采集数据
			}
            else if(GetMonitorTemplateType() == ExcetionMT)
			{
				TS_SendEvent(gTsMonitorTemplatID_c,gExcetionMonitorTemplateEventSampleData);  //发送异常监测模板采集数据
			}
			//TS_SendEvent(gTsMonitorTemplatID_c,gMonitorTemplateEventTest);
		} 
		if(RTC_GetITStatus(RTC_IT_ALRB) != RESET)
		{
			RTC_ClearITPendingBit(RTC_IT_ALRB);
            /* Clear the EXTI line */	
            EXTI_ClearITPendingBit(EXTI_Line17);
			gWakeUpNeedOLEDON = SET;
            TS_SendEvent(gTsMonitorTemplatID_c,gExcetionMonitorTemplateEventSetFreeRunGuard);
			//AlarmB_Int_Handler();
		}
        /* Clear the EXTI line */	
        EXTI_ClearITPendingBit(EXTI_Line17);        
}

void EXTI3_IRQHandler(void)
{
	if(EXTI_GetITStatus(PT_BMA250E_INT1_EXTI_LINE) != RESET)
	{
		/* Clear the EXT3 line 3 pending bit */
		EXTI_ClearITPendingBit(PT_BMA250E_INT1_EXTI_LINE);
		/* Add codes here for INT1 handler */
		Gsensor_Int1_Handler();
	}
}


/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
