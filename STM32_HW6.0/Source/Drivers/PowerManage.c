#include "common.h"
#include "led.h"

static uint32_t   TimeInLowPowerMode = 0;  // 处于低功耗模式时间(min)
static 	uint32_t  EnterLowPower_Time=0;
static  uint32_t  ExitLowPower_Time=0;
static  uint8_t   TimeInLowPowerModeCntCtl=1;   //系统处于低功耗模式计时
static  Timer_ID_Typedef     gBatterySampleTIMID=TIMER_ERROR;

static void BatterySample(Timer_ID_Typedef TIMID);

static void BatterySample(Timer_ID_Typedef TIMID)
{
		TIMID = TIMID;
		TS_SendEvent(gTsBatMonitorTaskID_c,gBATVoltageDetectEventStart);  //从Stop模式唤醒，采集一次电量
		#ifdef Battery_Debug
			printf("Battery Sample In Normal Work\r\n");
		#endif 
}
/*******************************************************************************
* Function Name  : PowerManage_Task_Handler
* Description    : 处理Power Manage Events
* Input          : PowerManage_Event
* Output         : None
* Return         : None
*******************************************************************************/
void PowerManage_Task_Handler(event_t PowerManage_Event)
{
		if(PowerManage_Event & gPowerManageClearTIMCnt)         //清除计时关闭屏幕
		{
			if(Get_OLED_Dis_Status() == OLEDDisEnterShutDown)  //关闭屏幕计时使能，按键退出
			{
				//Set_OLED_Dis_Status(OLEDDisON);
				if(gShutDownOledTIMID != TIMER_ERROR)
				{
					//Timer_Free(gShutDownOledTIMID);
					//gShutDownOledTIMID = TIMER_ERROR;
					Start_Timer_Cnt(gShutDownOledTIMID);  //ReStart Shut Down Oled timer
				}
			}
		}
		else if(PowerManage_Event & gPowerManageRecoverOLEDDis)  //Enable OLED display after power down
        {
			if(Get_OLED_Dis_Status() == OLEDDisShutDown)   //屏幕处于关闭状态
			{
				Set_OLED_Dis_Status(OLEDDisON);
				
				if(gMCUStatus == Stop)  					/* 	In case, system recover from LOWPOWER mode 	*/
				{ 
					gMCUStatus = Start;
					if(GetTimeInLowPowerMode() >= 60) 		/* 	In case, duration of LOWPOWER is greater than 60 second		*/
					{
						if((Device_Mode != Device_Mode_LowBattery) && (Device_Mode != Device_Mode_Charge)&& (Device_Mode != Device_Mode_CheckUp)
							&& (Device_Mode != Device_Mode_SecTick)
							)
						{
							if(SPO2_HR_Measure_State == Start)   //及时采集时，单设备在工作中，停止按键中状态判断
							{
								SPO2_HR_Measure_State = Stop;
							}
							Device_Mode=Device_Mode_RTC;
							TS_SendEvent(gOledDisTaskID,gOledDisEventBlueIcon_c);   /* 	Show RTC on OLED 		*/
						}
						else if(Device_Mode == Device_Mode_Charge)
						{
							TS_SendEvent(gOledDisTaskID,gOledDisEventModeCHARGE_c);  // Return the charge mode
						}
						else if(Device_Mode == Device_Mode_LowBattery) /*In case, in LowBattery mode */
						{
							OLED_DisplayClear();
							OLED_DisplayICON(ICON_ModeLowBattery);
							OLED_DisplayCtl(ON);
						}
                        else if(Device_Mode == Device_Mode_SecTick)
                        {
                            OLED_DisplayClear();
                            TS_SendEvent(gOledDisTaskID,gOledDisEventSecUpDate_c); //发送OLED显示Sec界面
                        }                            
					}
					else
					{
						if(Device_Mode == Device_Mode_RTC)
						{
							TS_SendEvent(gOledDisTaskID,gOledDisEventBlueIcon_c);   /* 	Show RTC on OLED 		*/
						}
						else if(Device_Mode == Device_Mode_Charge)
						{
							TS_SendEvent(gOledDisTaskID,gOledDisEventModeCHARGE_c);  // Return the charge mode
						}
						else if(Device_Mode == Device_Mode_LowBattery) /*In case, in LowBattery mode */
						{
							OLED_DisplayClear();
							OLED_DisplayICON(ICON_ModeLowBattery);
							OLED_DisplayCtl(ON);
						}
                        else if(Device_Mode == Device_Mode_SecTick)
                        {
                            OLED_DisplayClear();
                            TS_SendEvent(gOledDisTaskID,gOledDisEventSecUpDate_c); //发送OLED显示Sec界面
                        }  
						else
						{
							TS_SendEvent(gOledDisTaskID,gOledDisEventDisplayAllRAM_c);  //更新RAM区域显示值屏幕
						}
					}
				}
				else
				{
					if(Device_Mode == Device_Mode_RTC)
					{
						TS_SendEvent(gOledDisTaskID,gOledDisEventBlueIcon_c);   //显示RTC模式
					}
					else if(Device_Mode == Device_Mode_Charge)
					{
						TS_SendEvent(gOledDisTaskID,gOledDisEventModeCHARGE_c);  // Return the charge mode
					}
					else if(Device_Mode == Device_Mode_LowBattery) /*In case, in LowBattery mode */
					{
						OLED_DisplayClear();
						OLED_DisplayICON(ICON_ModeLowBattery);
						OLED_DisplayCtl(ON);
					}
                    else if(Device_Mode == Device_Mode_SecTick)
                    {
                        OLED_DisplayClear();
                        TS_SendEvent(gOledDisTaskID,gOledDisEventSecUpDate_c); //发送OLED显示Sec界面
                    }                    
					else
					{
						TS_SendEvent(gOledDisTaskID,gOledDisEventDisplayAllRAM_c);  //更新RAM区域显示值屏幕
					}
				}
				OLED_DisplayCtl(ON);
				TS_SendEvent(gOledDisTaskID,gOledDisEventClearModeDis_c);
			}
		}
}
/*******************************************************************************
* Function Name  : LowPowerManage
* Description    : Entry point of low power mode
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LowPowerManage(void)
{
    date_str_typedef        EnterLowPower_date_s;               //date of entering low power
    RTC_TimeTypeDef         EnterLowPower_rtc_time;             //time of entering low power

    //GPIO_InitTypeDef        GPIO_InitStructure;
    //static uint8_t       PrinCtl=0;
    uint8_t  Flag_WakeUpFromLowBatteryMode = false;
    
    if(
        (Device_Mode == Device_Mode_Charge)
    ||(Device_Mode == Device_Mode_FWUpdate)
    ||(Device_Mode == Device_Mode_IncomingCall)
    ||((Device_State == Device_WorkWithAPP) && (gFlagSyncDataInProgress == true))
    ||(AlarmStatusGet() == true)
    ||(Get_OLED_Dis_Status() != OLEDDisShutDown)
    ||(GetExtFlashAccessState() == FLASH_ACCESS_DISABLE)
    )  
    {
        /*  
            When cannot enter low power mode:   
            1. Charge mode
            2. FW update
            3. Incoming call
            4. Sync with APP
            5. Temperature test is undergoing
            6. Any alarm exist
            7. OLED isn't shutdown
        */
        
        if(Device_Mode == Device_Mode_Charge)
        {
            /*  In Charge mode, need to close battery sample    */
            if(gBatterySampleTIMID != TIMER_ERROR)
            {				
                /*  Close battery level sample timer if allocated       */
                Timer_Free(gBatterySampleTIMID);
                gBatterySampleTIMID = TIMER_ERROR;
            }
        }
    }
    else
    {
        /*  Prepare to enter low power mode     */
        if(
            cc_alg_SpO2_Stat_Get() == SPO2_HR_STATUS_STOP
        || cc_alg_SpO2_Stat_Get() == SPO2_HR_STATUS_ERROR
        || cc_alg_SpO2_Stat_Get() == SPO2_HR_STATUS_NULL
        )
        {
            PeriPower(OFF);
            /* Disable the Wakeup Interrupt */
            RTC_ITConfig(RTC_IT_WUT, DISABLE);
        }
        
        OLED_DriveSystemPower(OFF);
        OLED_DeConfiguration();
        LEDx_DeInit(LED_RED);
        LEDx_DeInit(LED_GREEN);
        Flash_PowerCtl(OFF);
        MX25_GPIO_DeInit();
        
        /*  Close battery sample    */
        if(gBatterySampleTIMID != TIMER_ERROR)
        {	
            /*  Close battery level sample timer if allocated       */
            Timer_Free(gBatterySampleTIMID);
            gBatterySampleTIMID = TIMER_ERROR;
        }
        
        /* Close OLEDShutdown time */
        if(gShutDownOledTIMID != TIMER_ERROR)
        {
            Timer_Free(gShutDownOledTIMID);
            gShutDownOledTIMID = TIMER_ERROR;
        }
        
        /* Clear Wake Up flag */
        PWR_ClearFlag(PWR_FLAG_WU);	
    
        /*save the time enter low power mode */
        if(TimeInLowPowerModeCntCtl == 1)
        {
            TimeInLowPowerModeCntCtl = 0;
            Calendar_Get(&EnterLowPower_date_s,&EnterLowPower_rtc_time);
            EnterLowPower_Time = (EnterLowPower_rtc_time.RTC_Hours * 3600 +EnterLowPower_rtc_time.RTC_Minutes * 60
                                + EnterLowPower_rtc_time.RTC_Seconds);
        }
        
        gMCUStatus = Stop;
        gWakeUpNeedOLEDON = RESET;
        
        /* Low Power in Low Battery Mode */
        if(Device_Mode == Device_Mode_LowBattery)
        {					
            /* Set BMA250E to Low Power */
            BMA250E_Int1_Port_Disable();
            BMA250E_Int2_Port_Disable();
            Set_BMA250E_LowPower();
            TS_ClearEvent(gTsStepTaskID_c);
            MCO_OutPutLSE(OFF);    //Disable MCO for NRF51822
            Flag_WakeUpFromLowBatteryMode = true;      

            MX25L_GPIO_LowPower_Config();
        }
        USART_DeConfiguration();
        
        
        if(true == EventListIsEmpty())
        {
            /*  Clear all pending bit before enter interrupt   */
            RTC_ClearITPendingBit(RTC_IT_WUT);
            RTC_ClearITPendingBit(RTC_IT_ALRA);
            RTC_ClearITPendingBit(RTC_IT_ALRB);
            
            PWR_UltraLowPowerCmd(ENABLE);
            PWR_EnterSTOPMode(PWR_Regulator_ON,PWR_STOPEntry_WFI);
            /*Stop here */
        }
    
        /* Code for system wakeup from system stop mode */
        RCC_MCOConfig(RCC_MCOSource_LSE,RCC_MCODiv_1);      /*  Enable 32.768khz clock for ble  */
        SystemInit_AfterWakeup();
        PWR_RTCAccessCmd(ENABLE);                           /*  Enable 32.768Khz                */
        
        /*  Enable the Wakeup Interrupt */
        RTC_ITConfig(RTC_IT_WUT, ENABLE);
        
        /*  Enable UART                 */
        USART_Configuration();
        /*  Power on the OLED if needed     */
        if(gWakeUpNeedOLEDON == SET)
        {
//			PrinCtl=1;
            WakeUpFromLowPower();
        }
        
        /*  Wakeup from Lower Battery Mode      */
        if(Flag_WakeUpFromLowBatteryMode == true)
        {
            StartStepCnt();
            MCO_OutPutLSE(ON);    //Enable MCO for NRF51822
            Flag_WakeUpFromLowBatteryMode = false;
        }
    }
}
/*******************************************************************************
* Function Name  : WakeUpFromLowPower
* Description    : 唤醒设备
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void WakeUpFromLowPower(void)
{
		date_str_typedef        ExitLowPower_date_s;                //退出低功耗RTC 日期
		RTC_TimeTypeDef         ExitLowPower_rtc_time;              //退出低功耗RTC 时间
		TIM_Cfg_Typedef         Tim_Cfg_ClaerOLED_Index;            //退出低功耗采集电量timer配置
		TIM_Basic_Cfg_Typedef 	Tim_Cfg_ClaerOLED;
	
		PeriPower(ON);
	
		/*get the time exit low power mode */		
		Calendar_Get(&ExitLowPower_date_s,&ExitLowPower_rtc_time);
        
		ExitLowPower_Time = (ExitLowPower_rtc_time.RTC_Hours * 3600 +ExitLowPower_rtc_time.RTC_Minutes * 60
							+ExitLowPower_rtc_time.RTC_Seconds);
		
		TimeInLowPowerMode = (ExitLowPower_Time - EnterLowPower_Time);
		TimeInLowPowerModeCntCtl = 1;
	
		if(gBatterySampleTIMID != TIMER_ERROR)   //已分配Timer
		{				
			Timer_Free(gBatterySampleTIMID);    //清除后重新配置时间
			gBatterySampleTIMID = TIMER_ERROR;
		}
		if(Device_Mode != Device_Mode_Charge)
		{
			/* 配置显示工作模式时，进入关闭屏幕时间定时器 */
			Tim_Cfg_ClaerOLED.enuTimerType = TIM_TYPE_MS;
			Tim_Cfg_ClaerOLED.u16TimePeriod = BATLevelSampleTIMWork;
			Tim_Cfg_ClaerOLED.NVIC_IRQChannelPreemptionPriority = OLEDShutDown_TIMPreemptionPriority;
			Tim_Cfg_ClaerOLED.pIntCallBack = BatterySample;

				/* Init timer top define */
				Tim_Cfg_ClaerOLED_Index.TimerMode 			= TIM_MODE_BASIC;
				Tim_Cfg_ClaerOLED_Index.TimerBasicCfg 		= &Tim_Cfg_ClaerOLED;
				Tim_Cfg_ClaerOLED_Index.TimerPWMCfg 		= NULL;

			gBatterySampleTIMID = Timer_Allocate(&Tim_Cfg_ClaerOLED_Index);
			Start_Timer_Cnt(gBatterySampleTIMID);
		}
		
		TS_SendEvent(gTsBatMonitorTaskID_c,gBATVoltageDetectEventStart);  //从Stop模式唤醒，采集一次电量
		#ifdef Battery_Debug
			printf("Battery Sample Wake Up\r\n");
		#endif 
}
/*******************************************************************************
* Function Name  : GetTimeInLowPowerMode
* Description    : 获取处于低功耗模式时间(n min)
* Input          : None
* Output         : None
* Return         : time (min)
*******************************************************************************/
uint32_t GetTimeInLowPowerMode(void)
{
	return TimeInLowPowerMode;
}
/**
  * @brief   GPIO Low Power configuration
  * @param  None
  * @retval None
  */
void GPIO_LowPower_Config(void)
{
		GPIO_InitTypeDef        GPIO_InitStructure;
	
		GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7
									 | GPIO_Pin_8 | GPIO_Pin_9  | GPIO_Pin_10| GPIO_Pin_11 | GPIO_Pin_12;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		/* the pin for STM32 Recevice data from nRF51822 */ 
//		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 ;
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
//        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
//        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//        GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2  
									| GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12
									| GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 ;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	    /* for EEPROM to Low Power */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(GPIOB, &GPIO_InitStructure);

	    /* for nRF51822 to Low Power nothing to do*/
//		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 ;
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
//        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
//        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//        GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2| GPIO_Pin_3 | GPIO_Pin_4 
										| GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_9  
										| GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 ;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOC, &GPIO_InitStructure);
		
		/* for nRF51822 to Low Power nothing to do */
//		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7 | GPIO_Pin_5 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 ;
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
//        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
//        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//        GPIO_Init(GPIOC, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOD, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOE, &GPIO_InitStructure);
		
//		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, DISABLE);
//		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, DISABLE);
//		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, DISABLE);
//		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, DISABLE);
//		
//		RCC_APB2PeriphClockCmd(RCC_APBxPeriph_SPI_BMA250E, DISABLE);
}

void MX25L_GPIO_LowPower_Config(void)
{
        GPIO_InitTypeDef        GPIO_InitStructure;
    
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
        GPIO_Init(GPIOB, &GPIO_InitStructure);  

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_0;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
        GPIO_Init(GPIOC, &GPIO_InitStructure);  

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOE, &GPIO_InitStructure);  




        /* handle the Peri_3V0 & OLED15V */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
        GPIO_Init(GPIOC, &GPIO_InitStructure);
        
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
        GPIO_Init(GPIOA, &GPIO_InitStructure);       
}



