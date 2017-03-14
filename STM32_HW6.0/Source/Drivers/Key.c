#include "stm32l1xx.h"
#include "Key.h"
#include "platform.h"
#include "main.h"
#include "common.h"

uint8_t SPO2_HR_Measure_State = Stop;

/*******************************************************************************
* Function Name  : KEY_Config
* Description    : 配置按键GPIO，按键采用中断的方式，中断服务函数中根据按键的时间
*                  判断长按或短按
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void KEY_Config(void)
{
    /* GPIOLED Periph clock enable */
//    GPIO_InitTypeDef   GPIO_InitStructure;
	TIM_Cfg_Typedef    Tim_Cfg_Key_Index;           //Key timer配置，用于检测长、短按键
	TIM_Basic_Cfg_Typedef 	Tim_Cfg_Key;
	EXTI_InitTypeDef   EXTI_InitStructure;    //按键采用中断输入的方式

    RCC_AHBPeriphClockCmd(RCC_KEYPeriph_KEYWP, ENABLE);

    /* Configure Key pins as input floating */
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_KEYWP;
//    GPIO_Init(GPIO_KEYWP, &GPIO_InitStructure);

    /* Connect EXTI Lines to KEY pins */
    SYSCFG_EXTILineConfig(EXTI_PortSourceKEYWP, EXTI_PinSourceKEYWP);

    /* Configure KEY EXTI lines */
    EXTI_InitStructure.EXTI_Line = EXTI_LineKEYWP;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

	/* 配置KEY定时器，用于长短按键检测 */
	Tim_Cfg_Key.enuTimerType = TIM_TYPE_MS;
	Tim_Cfg_Key.u16TimePeriod = PressTIMOut;
	Tim_Cfg_Key.NVIC_IRQChannelPreemptionPriority = KEY_TIMPreemptionPriority;
	Tim_Cfg_Key.pIntCallBack = KeyPressDetect;

	/* Init timer top define */
	Tim_Cfg_Key_Index.TimerMode 			= TIM_MODE_BASIC;
	Tim_Cfg_Key_Index.TimerBasicCfg 		= &Tim_Cfg_Key;
	Tim_Cfg_Key_Index.TimerPWMCfg 			= NULL;

    Tim_Cfg_Key_Index = Tim_Cfg_Key_Index;   //避免警告

//	if(gKeyTIMID != TIMER_ERROR)
//	{
//		Timer_Free(gKeyTIMID);
//	}
//	gKeyTIMID = Timer_Allocate(&Tim_Cfg_Key_Index);
//	Stop_Timer_Cnt(gKeyTIMID);
}
/*******************************************************************************
* Function Name  : KeyPressDetect
* Description    : 长按键处理
* Input          : events
* Output         : None
* Return         : None
*******************************************************************************/
void KeyPressDetect(Timer_ID_Typedef TIMID)
{
		TIM_Cfg_Typedef    Tim_Cfg_Key_Index;           //Key timer配置，用于检测长、短按键
		TIM_Basic_Cfg_Typedef 	Tim_Cfg_Key;

		if(gKeyPressStatus == Valid)
		{
			Stop_Timer_Cnt(gKeyTIMID);  //关闭Timer4
			//Key_State = Key_State_Long;
			gKeyLongPressStatus = ON;
			TS_SendEvent(gKeyTaskID,gKeyEventLongPress_c);
			gKeyPressTIMOut = true;
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
		else
		{
			gKeyPressStatus = Valid;

			/* 配置KEY定时器，用于长短按键检测 */
			Tim_Cfg_Key.enuTimerType = TIM_TYPE_MS;
			Tim_Cfg_Key.u16TimePeriod = PressTIMOut;
			Tim_Cfg_Key.NVIC_IRQChannelPreemptionPriority = KEY_TIMPreemptionPriority;
			Tim_Cfg_Key.pIntCallBack = KeyPressDetect;

			/* Init timer top define */
			Tim_Cfg_Key_Index.TimerMode 			= TIM_MODE_BASIC;
			Tim_Cfg_Key_Index.TimerBasicCfg 		= &Tim_Cfg_Key;
			Tim_Cfg_Key_Index.TimerPWMCfg 			= NULL;

			Timer_ChangeConfig(&Tim_Cfg_Key_Index,gKeyTIMID);
		}
}

/*******************************************************************************
* Function Name  : KeyTask
* Description    : 根据按键事件，处理按键任务
* Input          : events
* Output         : None
* Return         : None
*******************************************************************************/
void KeyTask(event_t events)
{
	static uint8_t SecTick_Measure_State = Stop;

	/* For incoming call ,not response */
	if(OccupiedDisCall == OLEDDisplay_Stat_Get())
	{
		return;
	}

	/* 	For any key pressed event, if in charge mode, not response 		*/
	if(Device_Mode == Device_Mode_Charge || Device_Mode == Device_Mode_LowBattery || Device_Mode == Device_Mode_CheckUp)
	{
		TS_SendEvent(gOledDisTaskID,gOledDisEventClearModeDis_c);
		return;
	}
	else
	{
		TS_SendEvent(gTsPowerManageTaskID_c,gPowerManageClearTIMCnt);  //Disable "display shutdown" downcounter
	}

	if(events & gKeyEventShortPress_c)    //短按键事件处理
	{
		#ifdef  Task_Key_Debug
		printf("Key Short Press\r\n");
		#endif

		switch(Device_Mode)
		{
			case Device_Mode_RTC :       //RTC模式下短按切换至SPO2/HR模式
                
                /* do not handle Long key press when in free run */
                if((MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID) && (true == isFreeRunKickOff()))
                {
                    return;
                }            
				if((MonitorTemplate.MTID != kICTMonitorTemplateFreeRunID) || (true != isFreeRunKickOff()))
				{
					Calendar_RTC_Period_Wakeup_DeInit();   //关闭RTC更新中断
				}
                
				if(SPO2_HR_Measure_State == Stop)  //在RTC模式下，SpO2/HR测量处于停止状态，短按按键开启SpO2/HR测量
				{
                    Device_Mode = Device_Mode_SPO2_HR;
					SPO2_START_BY_KEY = ON;
					TS_SendEvent(gOledDisTaskID,gOledDisEventMeaHR_c); //[ZHU] will show the "data display" when in measure

					TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStart);     //开启SpO2/HR测量

					gSubFunc_Stat_Set(SpO2_SingleWork_State | HR_SingleWork_State, ON);   //单设备采集SpO2/HR开启
					SPO2_HR_Measure_State = Start;
                    
					#ifdef  Task_Key_Debug
					printf("SPO2_HR Measure Start\r\n");
					#endif
				}
				else //if(SpO2_Stat_Get() == SUB_MOD_RUN)   //[ZHU] when run, long press to exist
				{
                    //there is no conditions to be here
                    #ifdef  Task_Key_Debug
                        printf("Mode RTC,SpO2 Measure RUN\r\n");
					#endif
				}		
				break;

			case Device_Mode_SPO2_HR:
//				if(SPO2_HR_Measure_State == Start)    //SPO2/HR正在测量过程中，短按停止SPO2/HR测量
				{
					/* 	To avoid, in case :
						1. single result test,
						2. long press to start
						3. short press to stop measure before result display
						Then, need press twice to enter next mode
					*/
//					if(
//						SpO2_RealTimeSample_GetResultNum != 0
//					&& 	HR_RealTimeSample_GetResultNum != 0
//					&& 	flagIsHRSpO2FixNumberSampleKeyPress == FALSE
//					)
//					{
//						flagIsHRSpO2FixNumberSampleKeyPress = TRUE;
//					}
					SPO2_START_BY_KEY = OFF;
					gSubFunc_Stat_Set(SpO2_SingleWork_State | HR_SingleWork_State, OFF);
					SPO2_HR_Measure_State = Stop;
					ResetSingleModeAlarmVirbreTim();  //停止时，清除振动告警间隔时间
					TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop);     //发送SpO2停止测量事件
                    
                    
                    if(OccupiedDisMove == OLEDDisplay_Stat_Get())  //占用显示可被按键清除
					{
						OLEDDisplay_Stat_Set(NormalDis);   //恢复正常显示模式
					}
                    Device_Mode = Device_Mode_RTC;
                    TS_SendEvent(gOledDisTaskID,gOledDisEventBlueIcon_c);   //显示RTC模式                   
					
					#ifdef  Task_Key_Debug
					printf("SPO2_HR Measure Stop\r\n");
					#endif
				}
				break;
			case Device_Mode_SecTick:
//				if((SecTick_Measure_State == Start) || (SecTick_Measure_State == Resume))  //SecTick处于运行过程中，短按进入Suspend状态
//				{
//					SecTick_Measure_State = Suspend;
//					TS_SendEvent(gTsSecTickTaskID_c,gSecTikEventSuspend);   //Suspend Second Tick
//
//					#ifdef Task_Key_Debug
//					printf("SecTick Measure Suspend\r\n");
//					#endif
//					break;
//				}
//				if(SecTick_Measure_State == Suspend)
//				{
//					SecTick_Measure_State = Resume;
//					TS_SendEvent(gTsSecTickTaskID_c,gSecTikEventResume);   //Resume Second Tick
//
//					#ifdef Task_Key_Debug
//					printf("SecTick Measure Resume\r\n");
//					#endif
//					break;
//				}
				if(SecTick_Measure_State == Stop) //SecTick 模式下,并且没有后台运行短按开始
				{
					SecTick_Measure_State = Start;
					TS_SendEvent(gTsSecTickTaskID_c,gSecTikEventStart);      //触发SecTick开启测量事件

					#ifdef  Task_Key_Debug
					printf("SecTick Measure Start\r\n");
					#endif
				}
				else if(SecTick_Measure_State == Start)
				{
					if(Get_SecTick_BackGroud_Status() == RESET)  //  没有后台运行
					{
						SecTick_Measure_State = Stop;                           //停止
						TS_SendEvent(gTsSecTickTaskID_c,gSecTikEventStop);      //触发SecTick关闭测量事件

						#ifdef  Task_Key_Debug
						printf("SecTick Measure Stop\r\n");
						#endif
					}
					else     //有后台运行
					{
						OLED_DisplayClear();    //先清屏
						Set_SecTick_BackGroud_Status(RESET);
						TS_SendEvent(gTsSecTickTaskID_c,gSecTikEventStart);   //Resume Second Tick

						#ifdef Task_Key_Debug
						printf("SecTick Measure Resume\r\n");
						#endif
					}
				}
				break;

			case Device_Mode_CheckUp: 		/* 	When in "app realtime sampling mode "		*/
				break;

			default: break;
		} //end of switch(Device_Mode)
	} //end of if(events & gKeyEventShortPress_c)    //短按键事件处理
	if(events & gKeyEventLongPress_c)    //长按键事件处理
	{
		#ifdef  Task_Key_Debug
		printf("Key Long Press\r\n");
		#endif

		/* do not handle Long key press when in free run */
		if((MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID) && (true == isFreeRunKickOff()))
		{
			return;
		}

		switch(Device_Mode)
		{
			case Device_Mode_RTC :     //RTC模式下，长按切换至SecTick模式

				Device_Mode = Device_Mode_SecTick;
                SetStorageInfo();
                ExtFLASH_SaveRdAddrToConst();
				if(SecTick_Measure_State == Start)
				{
					 if(Get_SecTick_BackGroud_Status() == SET)  //  有后台运行
					 {
						 OLED_DisplayClear();    //先清一次屏
						 Set_SecTick_BackGroud_Status(RESET);
						 TS_SendEvent(gTsSecTickTaskID_c,gSecTikEventStart);   //Resume Second Tick
					 }
				}
				else
				{
					OLED_DisplayClear();    //先清一次屏
					TS_SendEvent(gOledDisTaskID,gOledDisEventModeSecTick_c);  //显示SecTick模式
				}

				#ifdef  Task_Key_Debug
				printf("Device_Mode_SecTick\r\n");
				#endif

				break;
			case Device_Mode_SPO2_HR:
//				if(SPO2_HR_Measure_State == Stop)  //SPO2/HR模式下，SPO2/HR测量处于停止状态，长按开启SPO2/HR测量
//				{
//					SPO2_START_BY_KEY = ON;
//					TS_SendEvent(gOledDisTaskID,gOledDisEventMeaHR_c); //[ZHU] will show the "data display" when in measure
//					//TS_SendEvent(gOledDisTaskID, gOledDisEventModeKeepStable_c); //[ZHU] show "keep stable" when start spo2

//					/* 开启HR/SpO2测量前，将佩戴检测功能设置为测量模式 */
//					//Wear_Detect_Set(WEAR_DETECT_INC);
//					TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStart);     //发送SpO2开启测量事件，SpO2和HR测量属于同一事件，一起触发

//					gSubFunc_Stat_Set(SpO2_SingleWork_State | HR_SingleWork_State, ON);   //单设备采集SpO2/HR开启
//					SPO2_HR_Measure_State = Start;

//					#ifdef  Task_Key_Debug
//					printf("SPO2_HR Measure Start\r\n");
//					#endif
//				}
//				else
//				{
//					SPO2_START_BY_KEY = OFF;
//					gSubFunc_Stat_Set(SpO2_SingleWork_State | HR_SingleWork_State, OFF);
//					SPO2_HR_Measure_State = Stop;
//					ResetSingleModeAlarmVirbreTim();  //停止时，清除振动告警间隔时间
//					if(OccupiedDisMove == OLEDDisplay_Stat_Get())  //占用显示可被按键清除
//					{
//						OLEDDisplay_Stat_Set(NormalDis);   //恢复正常显示模式
//					}
//					TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop);     //发送SpO2停止测量事件
//					TS_SendEvent(gOledDisTaskID,gOledDisEventModeHR_c); //发送OLED显示SpO2/HR模式界面事件
//					#ifdef  Task_Key_Debug
//					printf("SPO2_HR Measure Stop\r\n");
//					#endif
//				}

				break;

			case Device_Mode_SecTick:
					if(SecTick_Measure_State == Start)     //进入后台测量
					{
						TS_SendEvent(gTsSecTickTaskID_c,gSecTikEventBackground);   //Second Tick back ground
					}
					else
					{
						Set_SecTick_BackGroud_Status(RESET);
					}
					Device_Mode = Device_Mode_RTC;
					TS_SendEvent(gOledDisTaskID,gOledDisEventBlueIcon_c);   //显示RTC模式

					#ifdef  Task_Key_Debug
					printf("Device_Mode_RTC\r\n");
					#endif
				break;
			default: break;
		} //end of switch(Device_Mode)
	}//end of if(events & gKeyEventLongPress_c)    //长按键事件处理
}


