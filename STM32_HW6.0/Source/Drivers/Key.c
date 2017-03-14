#include "stm32l1xx.h"
#include "Key.h"
#include "platform.h"
#include "main.h"
#include "common.h"

uint8_t SPO2_HR_Measure_State = Stop;

/*******************************************************************************
* Function Name  : KEY_Config
* Description    : ���ð���GPIO�����������жϵķ�ʽ���жϷ������и��ݰ�����ʱ��
*                  �жϳ�����̰�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void KEY_Config(void)
{
    /* GPIOLED Periph clock enable */
//    GPIO_InitTypeDef   GPIO_InitStructure;
	TIM_Cfg_Typedef    Tim_Cfg_Key_Index;           //Key timer���ã����ڼ�ⳤ���̰���
	TIM_Basic_Cfg_Typedef 	Tim_Cfg_Key;
	EXTI_InitTypeDef   EXTI_InitStructure;    //���������ж�����ķ�ʽ

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

	/* ����KEY��ʱ�������ڳ��̰������ */
	Tim_Cfg_Key.enuTimerType = TIM_TYPE_MS;
	Tim_Cfg_Key.u16TimePeriod = PressTIMOut;
	Tim_Cfg_Key.NVIC_IRQChannelPreemptionPriority = KEY_TIMPreemptionPriority;
	Tim_Cfg_Key.pIntCallBack = KeyPressDetect;

	/* Init timer top define */
	Tim_Cfg_Key_Index.TimerMode 			= TIM_MODE_BASIC;
	Tim_Cfg_Key_Index.TimerBasicCfg 		= &Tim_Cfg_Key;
	Tim_Cfg_Key_Index.TimerPWMCfg 			= NULL;

    Tim_Cfg_Key_Index = Tim_Cfg_Key_Index;   //���⾯��

//	if(gKeyTIMID != TIMER_ERROR)
//	{
//		Timer_Free(gKeyTIMID);
//	}
//	gKeyTIMID = Timer_Allocate(&Tim_Cfg_Key_Index);
//	Stop_Timer_Cnt(gKeyTIMID);
}
/*******************************************************************************
* Function Name  : KeyPressDetect
* Description    : ����������
* Input          : events
* Output         : None
* Return         : None
*******************************************************************************/
void KeyPressDetect(Timer_ID_Typedef TIMID)
{
		TIM_Cfg_Typedef    Tim_Cfg_Key_Index;           //Key timer���ã����ڼ�ⳤ���̰���
		TIM_Basic_Cfg_Typedef 	Tim_Cfg_Key;

		if(gKeyPressStatus == Valid)
		{
			Stop_Timer_Cnt(gKeyTIMID);  //�ر�Timer4
			//Key_State = Key_State_Long;
			gKeyLongPressStatus = ON;
			TS_SendEvent(gKeyTaskID,gKeyEventLongPress_c);
			gKeyPressTIMOut = true;
			gKeyPressStatus = InValid;

			/* ����KEY��ʱ�������ڰ�����Ч���*/
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

			/* ����KEY��ʱ�������ڳ��̰������ */
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
* Description    : ���ݰ����¼�������������
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

	if(events & gKeyEventShortPress_c)    //�̰����¼�����
	{
		#ifdef  Task_Key_Debug
		printf("Key Short Press\r\n");
		#endif

		switch(Device_Mode)
		{
			case Device_Mode_RTC :       //RTCģʽ�¶̰��л���SPO2/HRģʽ
                
                /* do not handle Long key press when in free run */
                if((MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID) && (true == isFreeRunKickOff()))
                {
                    return;
                }            
				if((MonitorTemplate.MTID != kICTMonitorTemplateFreeRunID) || (true != isFreeRunKickOff()))
				{
					Calendar_RTC_Period_Wakeup_DeInit();   //�ر�RTC�����ж�
				}
                
				if(SPO2_HR_Measure_State == Stop)  //��RTCģʽ�£�SpO2/HR��������ֹͣ״̬���̰���������SpO2/HR����
				{
                    Device_Mode = Device_Mode_SPO2_HR;
					SPO2_START_BY_KEY = ON;
					TS_SendEvent(gOledDisTaskID,gOledDisEventMeaHR_c); //[ZHU] will show the "data display" when in measure

					TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStart);     //����SpO2/HR����

					gSubFunc_Stat_Set(SpO2_SingleWork_State | HR_SingleWork_State, ON);   //���豸�ɼ�SpO2/HR����
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
//				if(SPO2_HR_Measure_State == Start)    //SPO2/HR���ڲ��������У��̰�ֹͣSPO2/HR����
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
					ResetSingleModeAlarmVirbreTim();  //ֹͣʱ������񶯸澯���ʱ��
					TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop);     //����SpO2ֹͣ�����¼�
                    
                    
                    if(OccupiedDisMove == OLEDDisplay_Stat_Get())  //ռ����ʾ�ɱ��������
					{
						OLEDDisplay_Stat_Set(NormalDis);   //�ָ�������ʾģʽ
					}
                    Device_Mode = Device_Mode_RTC;
                    TS_SendEvent(gOledDisTaskID,gOledDisEventBlueIcon_c);   //��ʾRTCģʽ                   
					
					#ifdef  Task_Key_Debug
					printf("SPO2_HR Measure Stop\r\n");
					#endif
				}
				break;
			case Device_Mode_SecTick:
//				if((SecTick_Measure_State == Start) || (SecTick_Measure_State == Resume))  //SecTick�������й����У��̰�����Suspend״̬
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
				if(SecTick_Measure_State == Stop) //SecTick ģʽ��,����û�к�̨���ж̰���ʼ
				{
					SecTick_Measure_State = Start;
					TS_SendEvent(gTsSecTickTaskID_c,gSecTikEventStart);      //����SecTick���������¼�

					#ifdef  Task_Key_Debug
					printf("SecTick Measure Start\r\n");
					#endif
				}
				else if(SecTick_Measure_State == Start)
				{
					if(Get_SecTick_BackGroud_Status() == RESET)  //  û�к�̨����
					{
						SecTick_Measure_State = Stop;                           //ֹͣ
						TS_SendEvent(gTsSecTickTaskID_c,gSecTikEventStop);      //����SecTick�رղ����¼�

						#ifdef  Task_Key_Debug
						printf("SecTick Measure Stop\r\n");
						#endif
					}
					else     //�к�̨����
					{
						OLED_DisplayClear();    //������
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
	} //end of if(events & gKeyEventShortPress_c)    //�̰����¼�����
	if(events & gKeyEventLongPress_c)    //�������¼�����
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
			case Device_Mode_RTC :     //RTCģʽ�£������л���SecTickģʽ

				Device_Mode = Device_Mode_SecTick;
                SetStorageInfo();
                ExtFLASH_SaveRdAddrToConst();
				if(SecTick_Measure_State == Start)
				{
					 if(Get_SecTick_BackGroud_Status() == SET)  //  �к�̨����
					 {
						 OLED_DisplayClear();    //����һ����
						 Set_SecTick_BackGroud_Status(RESET);
						 TS_SendEvent(gTsSecTickTaskID_c,gSecTikEventStart);   //Resume Second Tick
					 }
				}
				else
				{
					OLED_DisplayClear();    //����һ����
					TS_SendEvent(gOledDisTaskID,gOledDisEventModeSecTick_c);  //��ʾSecTickģʽ
				}

				#ifdef  Task_Key_Debug
				printf("Device_Mode_SecTick\r\n");
				#endif

				break;
			case Device_Mode_SPO2_HR:
//				if(SPO2_HR_Measure_State == Stop)  //SPO2/HRģʽ�£�SPO2/HR��������ֹͣ״̬����������SPO2/HR����
//				{
//					SPO2_START_BY_KEY = ON;
//					TS_SendEvent(gOledDisTaskID,gOledDisEventMeaHR_c); //[ZHU] will show the "data display" when in measure
//					//TS_SendEvent(gOledDisTaskID, gOledDisEventModeKeepStable_c); //[ZHU] show "keep stable" when start spo2

//					/* ����HR/SpO2����ǰ���������⹦������Ϊ����ģʽ */
//					//Wear_Detect_Set(WEAR_DETECT_INC);
//					TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStart);     //����SpO2���������¼���SpO2��HR��������ͬһ�¼���һ�𴥷�

//					gSubFunc_Stat_Set(SpO2_SingleWork_State | HR_SingleWork_State, ON);   //���豸�ɼ�SpO2/HR����
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
//					ResetSingleModeAlarmVirbreTim();  //ֹͣʱ������񶯸澯���ʱ��
//					if(OccupiedDisMove == OLEDDisplay_Stat_Get())  //ռ����ʾ�ɱ��������
//					{
//						OLEDDisplay_Stat_Set(NormalDis);   //�ָ�������ʾģʽ
//					}
//					TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop);     //����SpO2ֹͣ�����¼�
//					TS_SendEvent(gOledDisTaskID,gOledDisEventModeHR_c); //����OLED��ʾSpO2/HRģʽ�����¼�
//					#ifdef  Task_Key_Debug
//					printf("SPO2_HR Measure Stop\r\n");
//					#endif
//				}

				break;

			case Device_Mode_SecTick:
					if(SecTick_Measure_State == Start)     //�����̨����
					{
						TS_SendEvent(gTsSecTickTaskID_c,gSecTikEventBackground);   //Second Tick back ground
					}
					else
					{
						Set_SecTick_BackGroud_Status(RESET);
					}
					Device_Mode = Device_Mode_RTC;
					TS_SendEvent(gOledDisTaskID,gOledDisEventBlueIcon_c);   //��ʾRTCģʽ

					#ifdef  Task_Key_Debug
					printf("Device_Mode_RTC\r\n");
					#endif
				break;
			default: break;
		} //end of switch(Device_Mode)
	}//end of if(events & gKeyEventLongPress_c)    //�������¼�����
}


