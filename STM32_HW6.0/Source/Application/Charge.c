#include "charge.h"
#include "common.h"
#include "BatteryMonitor.h"
#include "Display.h"
#include <stdio.h>

static Timer_ID_Typedef gBATLevelSampleTIMID = TIMER_ERROR;

void BATChargeInit(void)
{
    /* GPIOLED Periph clock enable */
    GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	
    RCC_AHBPeriphClockCmd(RCC_BATPGPeriph, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_BATCHGPeriph, ENABLE);

    /* Configure PG pin as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = EXTI_LineBAT_PG;
    GPIO_Init(GPIO_BAT_PG, &GPIO_InitStructure);
    
    /* Configure CHG pin as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_BAT_CHG;
    GPIO_Init(GPIO_BAT_CHG, &GPIO_InitStructure);

    /* Connect EXTI Lines to KEY pins */
    SYSCFG_EXTILineConfig(EXTI_PortSourceBAT_PG, EXTI_PinSourceBAT_PG);

    /* Configure KEY EXTI lines */
    EXTI_InitStructure.EXTI_Line = EXTI_LineBAT_PG;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}

/*******************************************************************************
* @brief   Enter charge for battery
* @param   void
* @retval  void
*******************************************************************************/
void BATEnterCharge(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
#ifdef Battery_Debug
    printf("Now enter charging ...\r\n");
#endif
    
    /* Enable GPIOs clock */ 	
    RCC_AHBPeriphClockCmd(RCC_BATCEPeriph, ENABLE); 
    
    /* Configure CE pin as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_BAT_CE;
    GPIO_Init(GPIO_BAT_CE, &GPIO_InitStructure);
    
    GPIO_ResetBits(GPIO_BAT_CE,GPIO_Pin_BAT_CE);
    
    batteryVoltage.bExternPowerAvailable = true;

    if(Device_Mode != Device_Mode_FWUpdate)
    {
        Device_Mode = Device_Mode_Charge;
		TS_SendEvent(gTsBatMonitorTaskID_c,gBATVoltageDetectEventStart);  //采集一次电量
        TS_SendEvent(gOledDisTaskID,gOledDisEventModeCHARGE_c);
        TS_SendEvent(gTsLEDTaskID_c,gRedLEDFlashingStop);
        TS_SendEvent(gTsLEDTaskID_c,gGreenLEDFlashingStart);
    }
    else
    {
        Device_Mode_pre = Device_Mode_Charge;
        Device_Mode = Device_Mode_FWUpdate;
    }
}

/*******************************************************************************
* @brief   Battery has been full charged 
* @param   void
* @retval  void
*******************************************************************************/
static void BATFullCharge(void)
{
#ifdef Battery_Debug
    printf("Now full charge and stop change...\r\n");
#endif
    batteryVoltage.bFullScale = true;
    batteryVoltage.eBatteryAlarmState = BattStateNormal;
    
    if(Device_Mode != Device_Mode_FWUpdate)
    {
        TS_SendEvent(gOledDisTaskID,gOledDisEventModeCHARGE_c);
        TS_SendEvent(gTsLEDTaskID_c,gGreenLEDKeepingON);
    }
}

/*******************************************************************************
* @brief   Battery exits charge when the power is pulled 
* @param   void
* @retval  void
*******************************************************************************/
static void BATExitCharge(void)
{
    
//    GPIO_InitTypeDef GPIO_InitStructure;
//    
//#ifdef Battery_Debug
//    printf("Exit charging ...\r\n");
//#endif
//    
//    /* Enable GPIOs clock */ 	
//    RCC_AHBPeriphClockCmd(RCC_BATCEPeriph, ENABLE); 
//    
//    /* Configure CE pin as input floating */
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_BAT_CE;
//    GPIO_Init(GPIO_BAT_CE, &GPIO_InitStructure);
//    
//    GPIO_SetBits(GPIO_BAT_CE,GPIO_Pin_BAT_CE);

    if(batteryVoltage.bExternPowerAvailable)
    {
        batteryVoltage.bExternPowerAvailable = false;
    }
    
    batteryVoltage.bFullScale = false;
    if(Device_Mode != Device_Mode_FWUpdate)
    {
        TS_SendEvent(gOledDisTaskID,gOledDisEventModeCHARGE_c);
        TS_SendEvent(gTsLEDTaskID_c,gGreenLEDFlashingStop);
    }
    else
    {
        Device_Mode_pre = Device_Mode_RTC;
    }
}

/*******************************************************************************
* @brief   Alarm handler when the voltage is below the low voltage
* @param   void
* @retval  void
*******************************************************************************/
static void BATLowVoltAlarmHandler(void)
{
    batteryVoltage.eBatteryAlarmState = BattStateLow;
    TS_SendEvent(gOledDisTaskID,gOledDisEventModeCHARGE_c);
    TS_SendEvent(gTsLEDTaskID_c,gRedLEDFlashingStart);
}

/*******************************************************************************
* @brief   Alarm handler when the voltage is below the shutdown voltage 
* @param   void
* @retval  void
*******************************************************************************/
static void BATBadAlarmHandler(void)
{
    batteryVoltage.eBatteryAlarmState = BattStateOut;
    TS_SendEvent(gOledDisTaskID,gOledDisEventModeCHARGE_c);
}
/*******************************************************************************
* Function Name  : BATLevelSampleHandle
* Description    : 定时器中断产生采集电池电量事件
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void BATLevelSampleHandle(Timer_ID_Typedef TIMID)
{
		TIMID = TIMID;
		TS_SendEvent(gTsBatMonitorTaskID_c,gBATVoltageDetectEventStart);  //采集一次电池电量
	
		#ifdef Battery_Debug
			printf("Battery Charge State= %d\r\n",batteryVoltage.bExternPowerAvailable);
			printf("Battery Full Charge State = %d\r\n",batteryVoltage.bFullScale);
		#endif 
}
/*******************************************************************************
* Function Name  : BATStartChargeLevelSample
* Description    : 充电过程中，申请定时器控制电池电量采集
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void BATStartChargeLevelSample(void)
{
		TIM_Cfg_Typedef         Tim_Cfg_BATSample_Index;
		TIM_Basic_Cfg_Typedef 	Tim_Cfg_BATSample;
	
		/*  Configure the Timer for Battery Sample  */
		Tim_Cfg_BATSample.enuTimerType = TIM_TYPE_MS;
		Tim_Cfg_BATSample.u16TimePeriod = BATLevelSampleTIM;
		Tim_Cfg_BATSample.NVIC_IRQChannelPreemptionPriority = BATLevelSample_TIMPreemptionPriority;
		Tim_Cfg_BATSample.pIntCallBack = BATLevelSampleHandle;

		/* Init timer top define */
		Tim_Cfg_BATSample_Index.TimerMode 			= TIM_MODE_BASIC;
		Tim_Cfg_BATSample_Index.TimerBasicCfg 		= &Tim_Cfg_BATSample;
		Tim_Cfg_BATSample_Index.TimerPWMCfg 		= NULL;

        if(gBATLevelSampleTIMID != TIMER_ERROR)   //已分配Timer
        {				
            Timer_Free(gBATLevelSampleTIMID);    //清除后重新配置时间
            gBATLevelSampleTIMID = TIMER_ERROR;
        }     
		gBATLevelSampleTIMID = Timer_Allocate(&Tim_Cfg_BATSample_Index);
		Start_Timer_Cnt(gBATLevelSampleTIMID);
	
		#ifdef Battery_Debug
			printf("Start Bat Level Sample in Charging\r\n");
		#endif
}
/*******************************************************************************
* Function Name  : BATStopChargeLevelSample
* Description    : 充电结束，停止电池电量采集
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void BATStopChargeLevelSample(void)
{
		Timer_Free(gBATLevelSampleTIMID);
		gBATLevelSampleTIMID = TIMER_ERROR;
	
		#ifdef Battery_Debug
			printf("Stop Bat Level Sample in Charging\r\n");
		#endif
}
/*******************************************************************************
* @brief   Battery Manage handler
* @param   batteryManage_Event
* @retval  void
*******************************************************************************/
void BATManage_Task_Handler(event_t batteryManage_Event)
{
    switch(batteryManage_Event)
    {
        case gBATEnterChargeEvent:              /*  Enter Battery charge mode       */
            BATEnterCharge();
			BATStartChargeLevelSample();        /*  Start Battery Sample Timer      */
			if(gSubFunc_Stat_Get(SUBFUNC_ALL_STATE) != OFF
                || ((MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID) && (true == isFreeRunKickOff()))
              )
			{
				/* Stop all measure work		*/
                #ifdef FreeRunMode_Debug
                    printf("stop spo2 6 \r\n");
                #endif
				gSubFunc_Stat_Set(SUBFUNC_ALL_STATE,OFF); 		/* Stop all measure work 		*/
				TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop);
				OLEDDisplay_Stat_Set(NormalDis);                /*  Set OLED to Normal Mode     */
			}
            break;
        
        case gBATFullChargeEvent:               // Full charge
            BATFullCharge();
            break;
       
        case gBATExitChargeEvent:               // Exit charge
            BATExitCharge();
			BATStopChargeLevelSample();         /*  Stop Battery Sample     */
            /* When in Free Run, Start measure when exit charge */
            if((true == isFreeRunKickOff()) && (MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID))
            {
                if(cc_alg_SpO2_Stat_Get() != SPO2_HR_STATUS_RUNNING)
                {
                    if((MonitorTemplate.SampleID & SAMPLE_ID_HeartRate) 
                        || (MonitorTemplate.SampleID & SAMPLE_ID_SpO2))  //??HR/SPO2
                    {
                        TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStart);		/* 	Start HR/SPO2  sample 	*/
                    }
                }
            }
            else
            {
                TS_SendEvent(gTsStepTaskID_c, gStepResumeEvent);	/* 	Restart step conter 		*/
            }            
            break;
        
        case gBATLowEvent:                      // The battery is low
            BATLowVoltAlarmHandler();
			
			if(gSubFunc_Stat_Get(SUBFUNC_ALL_STATE) != OFF
               || ((MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID) && (true == isFreeRunKickOff()))
              )
			{
				/* Stop all measure work		*/
                #ifdef FreeRunMode_Debug
                    printf("stop spo2 7 \r\n");
                #endif
				gSubFunc_Stat_Set(SUBFUNC_ALL_STATE,OFF); 		/* Stop all measure work 		*/
				TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop);
				OLEDDisplay_Stat_Set(NormalDis);  //App停止及时显示Normal模式
			}
			Set_OLED_Dis_Status(OLEDDisShutDown);
            break;
        
        case gBATBadEvent:                      // The battery is out
            BATBadAlarmHandler();
			if(gSubFunc_Stat_Get(SUBFUNC_ALL_STATE) != OFF)
			{
                            #ifdef FreeRunMode_Debug
                                printf("stop spo2 8 \r\n");
                            #endif
				/* Stop all measure work		*/
				gSubFunc_Stat_Set(SUBFUNC_ALL_STATE,OFF); 		/* Stop all measure work 		*/
				TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop);
				OLEDDisplay_Stat_Set(NormalDis);  //App停止及时显示Normal模式
			}
            break;
   
        default:
            break;  
    }   
}
