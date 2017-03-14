/**
* @file				cc_alg_app_interface.c
* @brief 			application level interface
* @details			The functions and implementation of algorithm module to the application layer
* @date				2016/03/14
* @author			yun.zhu@ywkang.com  
* @version 			V1.0
* @copyright 		CloudCare Healthcare Co.,Ltd,All rights reserved
*/
#include "cc_alg_types.h"
#include "cc_alg.h"
#include "SpO2HrCalCZT.h"
#include "Display.h"
#include <stdlib.h>
#include "stdio.h"
#include "stdbool.h"
#include "SPO2_Coeff.h"
#include "IIR_Filter_Paramter.h"
#include "main.h"
#include "Font.h"
#include "storeManage.h"
#include "DataMemoryManage.h"
#include "AlarmCtrl.h"
#include "Timer.h"
#include "step.h"
#include "key.h"
#include "cc_alg_app_interface.h"
#include "Monitoer_Template.h"
#include "cc_alg_debug_ctrl_def.h"
#include "PulseOximetryAFE.h"
#include "cc_app_gsensor_drv_interface.h"

static uint8_t                          s_ui8SpO2ValidKeep;
static uint8_t                          s_ui8HrValidKeep;
static uint8_t                          s_ui8FlagSpO2DisClearScreem =0;
static uint8_t                          s_ui8FlagHRDisClearScreem =0;
//static uint8_t                          s_ui8HRGetStableResultCnt = 0;
//static uint8_t                          s_ui8SpO2GetStableResultCnt = 0;
/* 	static varibles for hr/spo2 monitor mode test 	*/

#if MOD_CTRL_IS_DATA_DOUBLE_CHECK_IN_MONITOR_MODE == true
	static uint8_t                          s_ui8HrSpO2MonitorStableCnt = 0;
	static st_Arr_ui8_Typedef            s_HrMonitorStore_st;
	static st_Arr_ui8_Typedef            s_SpO2MonitorStore_st;
	static uint8_t 							s_HrCheckBuf_arr[DATA_DOUBLE_CHECK_NUM];
	static uint8_t 							s_SpO2CheckBuf_arr[DATA_DOUBLE_CHECK_NUM];
#endif

/**
* @static 		SpotCheck counter
* @details 		
* 		-#	Clear this counter when: start, stop, measure
*		-#	Increae this counter when: in spot check and not in OSA mode
*/
static uint16_t                       s_uiSpotCheckCnt;

static SubModuleStat mSPI1OccupyStat = SUB_MOD_STOP;

static bool mIsMotionNoticeOccurs = false;
/*  static varilbes declaration         */
static uint8_t motion_detect_num;
/**
* @brief Define the instance of ALG module 
*/
static ccAlgBody_st s_ccAlgBody_st;

static HR_SPO2_INIT_PARA_Typedef s_SpO2HrInitParam;



static void spo2_event_distrute(uint8_t task_id, uint32_t event_id, SysEvtDatTypedef * SysEvtDat_pst);
static void spo2_Hr_measure_external_release(void);
static void spo2_Hr_measure_external_prepare(void);
static bool is_alg_module_response_system_event(void);
static void spo2_hr_post_handle(HR_SPO2_DAT_T * SpO2HrFinal_pst);
static void spo2_hr_interact_init(void);
static void data_pack_spo2hr(
							date_str_typedef    *pDate_s,               //RTC date
							RTC_TimeTypeDef     *pRtc_time,             //RTC time
							uint8_t HrSpO2Val,
							uint8_t paraId,
							uint8_t * pDatPack
);

static void u8toBCDConvert5Pos(uint8_t DataIn, uint8_t *pDatOut);
static void cc_spot_check_cnt_clr(void);
static void cc_spot_check_cnt_inc(void);



void isMotionNotice_Stat_Set(bool newState)
{
	mIsMotionNoticeOccurs = newState;
}

bool isMotionNotice_Stat_Get(void)
{
	return(mIsMotionNoticeOccurs);
}


/******************************************************************
*                        SPI1_Stat_Get                            *
* [Yun] Get SpO2 Sub module status                                * 
*******************************************************************/
SubModuleStat SPI1_Stat_Get(void)
{
	return(mSPI1OccupyStat);
}

/******************************************************************
*                        SPI1_Stat_Set                            *
* [Yun] Get SpO2 Sub module status                                * 
*******************************************************************/
void SPI1_Stat_Set(SubModuleStat newState)
{
	mSPI1OccupyStat = newState;
}

void spo2_hr_cal_mod_init(void)
{
    s_ccAlgBody_st.data_post_handle = spo2_hr_post_handle;
    s_ccAlgBody_st.cc_alg_sys_critical_resource_apply = spo2_Hr_measure_external_prepare;
    s_ccAlgBody_st.cc_alg_sys_critical_resource_release = spo2_Hr_measure_external_release;
    s_ccAlgBody_st.cc_alg_sys_delay_ms = Delay_ms;
    
    s_ccAlgBody_st.task_event_send = spo2_event_distrute;
    s_ccAlgBody_st.m_ccAlgParamIn_pst = NULL;
    
    /*  Init the algorithm module   */
    cc_alg_inst_init(&s_ccAlgBody_st);
    
    /*  Init the interaction module */
    spo2_hr_interact_init();
    
    
}


static void spo2_hr_interact_init(void)
{
    s_ui8SpO2ValidKeep = 0;
	s_ui8HrValidKeep = 0;
	s_ui8FlagSpO2DisClearScreem =1;
	s_ui8FlagHRDisClearScreem =1;
//	s_ui8HRGetStableResultCnt = 0;       /*  For getting stable HR value     */
//	s_ui8SpO2GetStableResultCnt = 0;     /*  For getting stable SpO2 value   */

#if MOD_CTRL_IS_DATA_DOUBLE_CHECK_IN_MONITOR_MODE	== true
	memset(&s_HrMonitorStore_st,0,sizeof(result_datBuf_Typedef));
	s_HrMonitorStore_st.m_ui16DataLen = DATA_DOUBLE_CHECK_NUM;
	s_HrMonitorStore_st.m_pui8Data = s_HrCheckBuf_arr;
	memset(&s_SpO2MonitorStore_st,0,sizeof(result_datBuf_Typedef));
	s_SpO2MonitorStore_st.m_ui16DataLen = DATA_DOUBLE_CHECK_NUM;
	s_SpO2MonitorStore_st.m_pui8Data = s_SpO2CheckBuf_arr;
#endif
}

/******************************************
* Name:             spo2_event_distrute
* Author:
* Date:
* Description:
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:
******************************************/
static void spo2_event_distrute(uint8_t task_id, uint32_t event_id, SysEvtDatTypedef * SysEvtDat_pst)
{
    TS_SendEvent(gTsSpO2TaskID_c, event_id);
}






/******************************************
* Name:             is_alg_module_response_system_event
* Author:
* Date:
* Description:
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:
******************************************/
static bool is_alg_module_response_system_event(void)
{
	if(
			(Device_Mode == Device_Mode_Charge
		|| 	Device_Mode == Device_Mode_LowBattery
		|| 	Device_Mode == Device_Mode_FWUpdate)
		)
	{
		return false;
	}
    else
    {
        return true;
    }
}

/******************************************
* Name:             spo2_Hr_measure_external_prepare
* Author:
* Date:
* Description:
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:
******************************************/
static void spo2_Hr_measure_external_prepare(void)
{
    mSPI1OccupyStat = SUB_MOD_RUN;
	mIsMotionNoticeOccurs = FALSE;
    
	/* 	Power on periph			*/
	if(OFF == GetPeriphPowerStatus())  //开启外部电源
	{
		PeriPower(ON);
	} 
}

/******************************************
* Name:             spo2_Hr_measure_external_prepare
* Author:
* Date:
* Description:
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:
******************************************/
static void spo2_Hr_measure_external_release(void)
{
	mSPI1OccupyStat = SUB_MOD_STOP;
	mIsMotionNoticeOccurs = FALSE;
}



/******************************************************************
*                        SpO2_Task_Handler                        *
* [Yun] init SpO2 resource and kick start SpO2                    * 
*******************************************************************/
void SpO2_Task_Handler(event_t SpO2_Event)
{
	
	SPO2_HR_CAL_STATUS_T 	SpO2HrCalStatus_st;
	SPO2_HR_CAL_MODE_T		SpO2HrCalMode_st;
	HR_SPO2_DAT_T *			pSpO2HrDatReport;
	
	/*	Init the SpO2 Cal Mode 		*/
//#if PRTF_DAT_DUMP_ONLY == true
//	SpO2HrCalMode_st.bIsRawDataDump 			= 	true;
//#else
//	SpO2HrCalMode_st.bIsRawDataDump 			= 	false;
//#endif
	
	SpO2HrCalMode_st.bIsMotionReportOnly		= 	false;
	
	/*	Init the status structure 		*/
	/** @note 	Clear this varibles in the process functions 	*/
	//SpO2HrCalStatus_st.bIsCalTimeOut			= 	false;
	//SpO2HrCalStatus_st.bIsMotionCheckPass		= 	false;
	//SpO2HrCalStatus_st.bIsSmallSignalDetected	= 	false;
	
	/*	Init the data report structure 	*/
	pSpO2HrDatReport = cc_alg_get_spo2_hr_result_str();

	
	#ifdef SPO2_EVENT_DEBUG_MODE
	printf("state transfer, new event = %x \n", SpO2_Event);
	#endif
    if( false == is_alg_module_response_system_event()
        && (SpO2_Event != gSpO2EventStop)
    )
    {
        return;
    }
	
	if(
        SpO2_Event == gSpO2EventStart
    ||  SpO2_Event == gSpO2EventStartInternal
        )
	{
		/* 	Clear the motion detect counter 			*/
		motion_detect_num = 0;
		/* 	Set Wear Detect only function 		*/
		Wear_Detect_Set(WEAR_DETECT_INC);

		/* 	Set the status and following actions 		*/
		if(cc_alg_SpO2_Stat_Get() != SPO2_HR_STATUS_RUNNING)
		{
			/* 	Start SpO2 measurement if any measurement open 		*/
			if((gSubFunc_Stat_Get(SPO2_NEEDED_STATE) != OFF) 
				|| ((MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID) && (true == isFreeRunKickOff())))
			{
                if(MonitorTemplate.MTID != kICTMonitorTemplateFreeRunID)
                {
                    s_SpO2HrInitParam.m_DataSmoothLevel = SPO2_SMOOTH_LEVEL3;
                }
                else
                {
                    s_SpO2HrInitParam.m_DataSmoothLevel = SPO2_SMOOTH_LEVEL1;
                }
                
				if((MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID) && (true == isFreeRunKickOff()))
				{
					s_SpO2HrInitParam.m_AlgSpO2HrRunMode = SPO2_HR_MODE_OSA;
				}
				else
				{
					s_SpO2HrInitParam.m_AlgSpO2HrRunMode = SPO2_HR_MODE_NULL;
				}
				
				
				
                if(gFlagHR_SpO2Measure == HRSpO2_Measure_Test)
                {
                    s_SpO2HrInitParam.m_bIsFactoryMode = true;
					cc_afe_set_factory_mode(true);
                }
                else
                {
                    s_SpO2HrInitParam.m_bIsFactoryMode = false;
					cc_afe_set_factory_mode(false);
                }
				
				/*	Get the DC Compensate 		*/
				cc_afe_get_dc_compensate(&s_SpO2HrInitParam.m_iAfeDcOffset);
				
				/*	Set the Motion Check threshold 	*/
				s_SpO2HrInitParam.m_uiMotionCheckThres 		= SENSOR_MOTION_THRESHOLD_SPO2;
				
				/*	Set the Clear-while_motion threshold 	*/
				
				s_SpO2HrInitParam.m_uiClearWhileMotionThres 	= SENSOR_DATA_ANTI_MOV_THRES;
				
                /*  Kick off the measurement        */
                cc_alg_measure_start(&s_SpO2HrInitParam);
				/* Set status 						*/
                cc_alg_SpO2_Stat_Set(SPO2_HR_STATUS_RUNNING);
				
				/*	Clear the SpOt check counter 		*/
				cc_spot_check_cnt_clr();
				
//				/* 	Set fix number flag 		*/
//				if(
//					SpO2_RealTimeSample_GetResultNum != 0
//					&& 	HR_RealTimeSample_GetResultNum != 0
//					&& 	(
//							gSubFunc_Stat_Get(SPO2_HR_SINGLEWORK_STATE) != OFF
//						||	gSubFunc_Stat_Get(SPO2_HR_REALTIME_STATE) != OFF
//						)
//					)
//				{
//					flagIsHRSpO2FixNumberSampleKeyPress = FALSE;
//				}
//				else
//				{
//					flagIsHRSpO2FixNumberSampleKeyPress = TRUE;
//				}
			}
		}	
	}
	else if(SpO2_Event&gSpO2EventStop)
	{
		/*Clear the motion detect counter */
		motion_detect_num = 0;
		
//		printf("m_SpO2_Submodule_stat:%d,sub state = %x \r\n",m_SpO2_Submodule_stat, gSubFunc_Stat_Get(SUBFUNC_ALL_STATE));
		
		/* 	If Wear Detect function still run 		*/

		Wear_Detect_Set(WEAR_DETECT_INC);		
		
        #ifdef SPO2_EVENT_DEBUG_MODE
            printf("spo2 stat = %x \r\n",cc_alg_SpO2_Stat_Get());
        #endif
        
		if(cc_alg_SpO2_Stat_Get() != SPO2_HR_STATUS_STOP)
		{
            #ifdef SPO2_EVENT_DEBUG_MODE
                printf("spo2 func need = %x \r\n",gSubFunc_Stat_Get(SPO2_NEEDED_STATE));
            #endif
			if((gSubFunc_Stat_Get(SPO2_NEEDED_STATE) == OFF)
                || ((MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID) && (true == isFreeRunKickOff())))
			{
				/*Deinit AFE and SPI program*/
				cc_alg_measure_stop();
				
#if SPO2_ERROR_TRACK_DEBUG == TRUE
				printf("measure stop in Stop Event proc \r\n");
#endif
				
				/*	Clear the Spot Check counter 	*/
				cc_spot_check_cnt_clr();
				
				/* Set gsensor to step count mode */
				
				gflagIsAccSampleRateConvert = FALSE;
				if((MonitorTemplate.MTID != kICTMonitorTemplateFreeRunID) || (true != isFreeRunKickOff()))
				{
					TS_SendEvent(gTsStepTaskID_c, gStepResumeEvent);
				}
				else
				{
					TS_SendEvent(gTsStepTaskID_c, gStepStopEvent);
				}
				/* set status */
                cc_alg_SpO2_Stat_Set(SPO2_HR_STATUS_STOP);
				if(OccupiedDisMove == OLEDDisplay_Stat_Get())  //占用显示可被按键清除
				{
					OLEDDisplay_Stat_Set(NormalDis);   //恢复正常显示模式
					Set_OLED_Dis_Status(OLEDDisON);
					TS_SendEvent(gOledDisTaskID,gOledDisEventDisplayAllRAM_c);  //更新RAM区域显示值屏幕
					TS_SendEvent(gOledDisTaskID,gOledDisEventClearModeDis_c);
				}
				if(SPO2_HR_Measure_State == Start)   /* Avoid to press twice to start */
				{
					SPO2_HR_Measure_State = Stop;
				}	
				
				/* 	Clear keep stable alarm timer 	*/
				ClearKeepStableTimer();
				
			}
		}
	}
	else if(SpO2_Event&gSpO2EventBaselineStart)
	{
		/*Clear the motion detect counter */
		motion_detect_num = 0;
		/*require to restart baseline adjust*/
		PulseOxiAfe_SignalAdj_Init();
		/*	Clr the report data structure 		*/
		cc_alg_clr_spo2_hr_result_st();
		
	}
	else if(SpO2_Event&gSpO2EventBaselineCal)
	{
		/*require to run baseline adjust*/
		PulseOxiAfe_SignalAdj_Check();
		
		/*	Clr system counter if needed 	*/
		if((MonitorTemplate.MTID != kICTMonitorTemplateFreeRunID) || (true != isFreeRunKickOff()))
		{		
#if MOD_CTRL_IS_FOUR_MINUTES_EXIT_ENABLE == false
			SpO2_RunTime_Cnt_Set(0);
#endif
		}
		else
		{
			SpO2_RunTime_Cnt_Set(0);
		}
		
	}
//    else if(SpO2_Event&gSpO2EventMeasureCalPro)
//    {
//        /*  Run spo2 cal routine to get the motion level and pre filter calculation   */
//        SpO2HrCal_Entry(true);
//    }
	else if(SpO2_Event&gSpO2EventMeasureInit)
	{
		/*	Init the SpO2 measure program		*/
		CZT_Process_Init();
        SpO2HrCalPreDataProcess();
        SpO2HrCalCZTFilterFlush();
		/*	Clear the Spot Check Counter 		*/
		cc_spot_check_cnt_clr();
		
		
		
	}
	else if(SpO2_Event&gSpO2EventMeasureCal)
	{
		/*	Run SpO2 measure program			*/
		SpO2HrCal_Entry(&SpO2HrCalMode_st, &SpO2HrCalStatus_st, pSpO2HrDatReport);
		
#if MOD_CTRL_IS_BLOCK_HUNDRED_SPO2
		if(pSpO2HrDatReport->m_ui8SpO2Val == 100)
		{
			pSpO2HrDatReport->m_ui8SpO2Val = 99;
		}
#endif
		/*	Clr system counter if needed 	*/
		if((MonitorTemplate.MTID != kICTMonitorTemplateFreeRunID) || (true != isFreeRunKickOff()))
		{		
#if MOD_CTRL_IS_FOUR_MINUTES_EXIT_ENABLE == false
		SpO2_RunTime_Cnt_Set(0);
#endif
		}
		else
		{
			SpO2_RunTime_Cnt_Set(0);
		}
		//ALG_PRINTF("Val = %d,%d,%d,%d\r\n",pSpO2HrDatReport->m_ui8HrVal,pSpO2HrDatReport->m_bIsHrValid,pSpO2HrDatReport->m_ui8SpO2Val,pSpO2HrDatReport->m_bIsSpO2Valid);
		
		
		/*	Check status and Status handler		*/
		if(SpO2HrCalStatus_st.bIsCalTimeOut	== true)
		{
			/*	Error occurs, need to restart the whole process 	*/
			TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventError); 
#if SPO2_ERROR_TRACK_DEBUG == true
            ALG_PRINTF("spo2 error internal calculate timeout \r\n");
#endif
			
		}
		else if(SpO2HrCalStatus_st.bIsSmallSignalDetected == true)
		{
			/*	Small PP Val, need to restart the baseline line 	*/
			TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventBaselineStart); 
		}
		else
		{
			/*	Nothing to do			*/
		}
		
		/*	Motion detected 									*/
		cc_sys_gsensor_spo2_motion_notice_handle(SpO2HrCalStatus_st.bIsMotionCheckPass);
		
		/*	Update the data				*/
		spo2_hr_post_handle(pSpO2HrDatReport);
	}				
	else if(SpO2_Event&gSpO2EventError)
	{
#if MOD_CTRL_IS_FORCE_RUNNING_ENABLE == true
		/* 	set status 								*/
		cc_alg_SpO2_Stat_Set(SPO2_HR_STATUS_ERROR);
		/*  restart whole process                   */
		TS_SendEvent(gTsSpO2TaskID_c, gSpO2EventStart);
#else
		if((MonitorTemplate.MTID != kICTMonitorTemplateFreeRunID) || (true != isFreeRunKickOff()))
		{				
            #ifdef FreeRunMode_Debug
                printf("error spo2 3 \r\n");
            #endif
        
            
            /*Code to handle spo2 error*/
            
            /* 	Stop the measurement				*/
            cc_alg_measure_stop();
#if SPO2_ERROR_TRACK_DEBUG == TRUE
			printf("measure stop in Error Event proc \r\n");
#endif
            /* 	Set gsensor to wakeup mode 				*/
            gflagIsAccSampleRateConvert = FALSE;
            
            if((MonitorTemplate.MTID != kICTMonitorTemplateFreeRunID) || (true != isFreeRunKickOff()))
            {
                TS_SendEvent(gTsStepTaskID_c, gStepResumeEvent);
            }
            else
            {
                TS_SendEvent(gTsStepTaskID_c, gStepStopEvent);
            }
            
            /* 	Set wear detect enable 		*/
            Wear_State_Set(WEAR_BAD,WEAR_DET_FINISH);
            
            /* 	Disable Wear Detect only function 		*/
            Wear_Detect_Set(WEAR_DETECT_INC);
            
            /* 	set status 								*/
            cc_alg_SpO2_Stat_Set(SPO2_HR_STATUS_ERROR);

            /* Display Error GUI */
            if(gSubFunc_Stat_Get(SPO2_HR_SINGLEWORK_STATE | SPO2_HR_REALTIME_STATE) != OFF)
            {
                TS_SendEvent(gOledDisTaskID,gOledDisEventSpO2Err_c);
                /* 	Send alarm event 				*/
                TS_SendEvent(gTsAlarmTaskID_c,gAlarmEventNoFinger); 
                
                if(OccupiedDisMove == OLEDDisplay_Stat_Get())  //占用显示可被按键清除
                {
                    OLEDDisplay_Stat_Set(NormalDis);   //恢复正常显示模式
                }
            }
            
            /* 	Clear keep stable alarm timer 	*/
            ClearKeepStableTimer();
            
            
            /* 	Disable all about SpO2 					*/
            gSubFunc_Stat_Set(SPO2_HR_SUBFUNC_ALL_STATE,OFF);
        }
        else
        {
            /* 	set status 								*/
            cc_alg_SpO2_Stat_Set(SPO2_HR_STATUS_ERROR);
            /*  restart whole process                   */
			TS_SendEvent(gTsSpO2TaskID_c, gSpO2EventStart);
        }
#endif
	}
	else if(SpO2_Event & gSpO2EventMotionDetected)
	{
		/* Motion Detected */
		motion_detect_num += 1;
		/* Added code here to notice user */
		if((gSubFunc_Stat_Get(SPO2_HR_MONITOR_STATE) == OFF) && (cc_alg_SpO2_Stat_Get() == SPO2_HR_STATUS_RUNNING) && mIsMotionNoticeOccurs == FALSE
			&& (OccupiedDisCall != OLEDDisplay_Stat_Get()))  
		{
			AlarmLevelSet(ALARM_LEVEL_LOW);
			OLEDDisplay_Stat_Set(OccupiedDisMove); 
			TS_SendEvent(gOledDisTaskID, gOledDisEventModeKeepStable_c);
			mIsMotionNoticeOccurs = TRUE;
		}
		
		/* [keke] enable MOTION_DETECT_LIMIT when SPO2/HR single work & realtime state is off */
		if((gSubFunc_Stat_Get(SPO2_HR_SINGLEWORK_STATE) == OFF) && (gSubFunc_Stat_Get(SPO2_HR_REALTIME_STATE) == OFF))
		{
			/* when in monitor template & detect in 1 minutes, then exist */
			if((gSubFunc_Stat_Get(SPO2_HR_MONITOR_STATE) != OFF) && motion_detect_num == MOTION_DETECT_LIMIT)
			{
				/* Deinit AFE and SPI program*/
				cc_alg_measure_stop();
#if SPO2_ERROR_TRACK_DEBUG == TRUE
				printf("measure stop in Motion Detected Event proc \r\n");
#endif
				/* Set gsensor to wakeup and step count mode */
				if((MonitorTemplate.MTID != kICTMonitorTemplateFreeRunID) || (true != isFreeRunKickOff()))
				{
					TS_SendEvent(gTsStepTaskID_c, gStepResumeEvent);
				}
				else
				{
					TS_SendEvent(gTsStepTaskID_c, gStepStopEvent);
				}
				/* Set status 						*/
				cc_alg_SpO2_Stat_Set(SPO2_HR_STATUS_ERROR);
				/* Display Error GUI */
				if(gSubFunc_Stat_Get(SPO2_HR_MONITOR_STATE) == OFF)
				{
					TS_SendEvent(gOledDisTaskID,gOledDisEventSpO2Err_c);
					
					if(OccupiedDisMove == OLEDDisplay_Stat_Get())  //占用显示可被按键清除
					{
						OLEDDisplay_Stat_Set(NormalDis);   //恢复正常显示模式
					}
				}
			}
		}
	}
    else if(SpO2_Event == gSpO2EventSignalCheck)
    {
        cc_alg_afe_signal_check();
    }
    else if(SpO2_Event == gSpO2EventSpotCheck)
    {
        /* @change Stop Measure IF NOT in OSA mode after pre-defined check circuit  */
        if((MonitorTemplate.MTID != kICTMonitorTemplateFreeRunID) || (true != isFreeRunKickOff()))
        {
            cc_spot_check_cnt_inc();
        }
        if(s_uiSpotCheckCnt == SPOT_CHECK_CYCLE_NUM_LIMIT)
        {
			cc_spot_check_cnt_clr();
            
			/* 	Stop Measurement*/
			//gSubFunc_Stat_Set(SPO2_NEEDED_STATE,OFF);
            TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventError);
			TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop);
             
        }
        else
        {
            PulseOxiAfe_Spot_Check_Kick();
        }
    }
}





/******************************************************************
*                        spo2_hr_post_handle                      *
******************************************************************/
static void spo2_hr_post_handle(HR_SPO2_DAT_T * SpO2HrFinal_pst)
{
//	uint8_t   Err_Code=0;
	uint8_t HR_Value[3] = {0};
	uint8_t SpO2_Value[3] = {0};
	uint8_t HR_SaveData[MT_STORAGE_DATA_LEN] = {0};
	uint8_t SpO2_SaveData[MT_STORAGE_DATA_LEN] = {0};
	#if MOD_CTRL_IS_DATA_DOUBLE_CHECK_IN_MONITOR_MODE == true
		uint8_t 	valTmp;
	#endif
	
	bool flagIsMonitorDatUpdate;
	
//	FlagStatus 		flagHRSampleDone = RESET;
//	FlagStatus 		flagSpO2SampleDone = RESET;
	
	date_str_typedef    date_s;               //RTC date
	RTC_TimeTypeDef     rtc_time;             //RTC time
    uint32_t            u32Date = 0;
	
	Calendar_Get(&date_s,&rtc_time);
	
	gHRSpO2Val.date_s.year = date_s.year;
	gHRSpO2Val.date_s.month = date_s.month;
	gHRSpO2Val.date_s.day = date_s.day;
	gHRSpO2Val.rtc_time.RTC_Hours = rtc_time.RTC_Hours;
	gHRSpO2Val.rtc_time.RTC_Minutes = rtc_time.RTC_Minutes;
	gHRSpO2Val.rtc_time.RTC_Seconds = rtc_time.RTC_Seconds;
	
	/* 	Process for HR data 			*/
	gHRSpO2Val.HrSpO2DataRep_pst->m_ui8HrVal = SpO2HrFinal_pst->m_ui8HrVal; 		//Save hr value
	gHRSpO2Val.HrSpO2DataRep_pst->m_ui8SpO2Val = SpO2HrFinal_pst->m_ui8SpO2Val;    //save spo2 value
	
    
	/* Handle HR value */
	if((Device_Mode == Device_Mode_SPO2_HR) && (gSubFunc_Stat_Get(HR_SingleWork_State) != OFF))
	{
		/* 	Single mode check 		*/
		if(ON == GetHR_SpO2DisplayMeasureStatus() && (true == SpO2HrFinal_pst->m_bIsHrValid || true == SpO2HrFinal_pst->m_bIsSpO2Valid))
		{
			OLED_DisplayHRSpO2MeasureFlash(OFF);  
			OLED_DisplayClear();
            #ifdef UI_CHINESE_ENABLE
                OLED_DisplayBPM(62,0,95,39,DIS_ICON_SpO2Measure);
            #else
                OLED_DisplayBPM(60,2,96,38,DIS_ICON_SpO2Measure);
            #endif
		}
		
//		if( true == SpO2HrFinal_pst->m_bIsHrValid && flagHRSampleDone == RESET)
        if( true == SpO2HrFinal_pst->m_bIsHrValid)
		{
			s_ui8HrValidKeep = 1;
			
			u8toBCDConvert5Pos(SpO2HrFinal_pst->m_ui8HrVal, HR_Value);

			if(HR_Value[2] == 0)
			{
				if(s_ui8FlagHRDisClearScreem == 1)
				{
					s_ui8FlagHRDisClearScreem = 0;
					OLED_Fill(0,0,60,20,0);
				}
				OLED_DisplayNum(1,0,HR_Value[1],Font_11x18);
				OLED_DisplayNum(14,0,HR_Value[0],Font_11x18);
				OLED_DisplayBPM(27,11,44,18,DIS_ICON_BPM_17x7);
			}
			else
			{
				OLED_DisplayNum(1,0,HR_Value[2],Font_11x18);
				OLED_DisplayNum(14,0,HR_Value[1],Font_11x18);
				OLED_DisplayNum(27,0,HR_Value[0],Font_11x18);
				OLED_DisplayBPM(40,11,57,18,DIS_ICON_BPM_17x7);
				s_ui8FlagHRDisClearScreem = 1;
			}
//			
//			/* 	Single display mode 	*/
//			if(HR_RealTimeSample_GetResultNum != 0)
//			{
//				flagHRSampleDone = SET;
//			}
		}
		else if(s_ui8HrValidKeep == 0 && OFF == GetHR_SpO2DisplayMeasureStatus())
		{
			if(s_ui8FlagHRDisClearScreem == 1)
			{
				s_ui8FlagHRDisClearScreem = 0;
				OLED_Fill(0,0,60,20,0);
			}
			OLED_DisplayNum(1,0,10,Font_11x18);
			OLED_DisplayNum(14,0,10,Font_11x18);
			OLED_DisplayBPM(27,11,44,18,DIS_ICON_BPM_17x7);
		}
	}

	/* Send data while in real time sample and data valid */
	if((Device_Mode == Device_Mode_CheckUp) && (gSubFunc_Stat_Get(HR_RealTimeSample_State) != OFF))   //SPI接收及时采集命令中控制该标志位
	{
		if (true == SpO2HrFinal_pst->m_bIsHrValid)
		{
//			if(HR_RealTimeSample_GetResultNum == 0)  //App连续采集，只要采集到有数据就传给App
//			{
//				s_ui8HRGetStableResultCnt = 0xff;  //for continously sample, keep this value, so the sample can be continued
				TS_SendEvent(gTsSPITranslateTaskID_c,gSPITranslateEventTxHR);   //产生发送HR事件,及时采集数据发送
//			}
//			else  //App只需要获取HR_RealTimeSample_GetResultNum个稳定数值
//			{
//				s_ui8HRGetStableResultCnt++;  //increase the HR counter, the max number is 256
//				if(s_ui8HRGetStableResultCnt <= HR_RealTimeSample_GetResultNum)
//				{
//					TS_SendEvent(gTsSPITranslateTaskID_c,gSPITranslateEventTxHR);   //产生发送HR事件,及时采集数据发送		
//				}
//			}
		}
//		else
//		{
//			if(HR_RealTimeSample_GetResultNum == 0)  //App连续采集，只要采集到有数据就传给App
//			{
//				s_ui8HRGetStableResultCnt = 0xff;  //for continously sample, keep this value, so the sample can be continued
//			}
//		}

	}

	/* Process for SPo2		*/

	/* 单设备测量SpO2时，OLED显示测量结果 */
	if((Device_Mode == Device_Mode_SPO2_HR) && (gSubFunc_Stat_Get(SpO2_SingleWork_State) != OFF))
	{
		/* 	Single mode measurement 	*/
//		if(true == SpO2HrFinal_pst->m_bIsSpO2Valid && flagSpO2SampleDone == RESET)
        if(true == SpO2HrFinal_pst->m_bIsSpO2Valid)
		{
			s_ui8SpO2ValidKeep = 1;
			
			u8toBCDConvert5Pos(SpO2HrFinal_pst->m_ui8SpO2Val, SpO2_Value);

			if(SpO2_Value[2] == 0)
			{
				if(s_ui8FlagSpO2DisClearScreem == 1)
				{
					s_ui8FlagSpO2DisClearScreem = 0;
					OLED_Fill(0,20,58,39,0);
				}
				OLED_DisplayNum(1,20,SpO2_Value[1],Font_11x18);
				OLED_DisplayNum(14,20,SpO2_Value[0],Font_11x18);
                #ifdef UI_CHINESE_ENABLE
                    OLED_DisplayBPM(26,30,61,39,DIS_ICON_Percent_11x9);
                #else
                    OLED_DisplayBPM(27,28,38,37,DIS_ICON_Percent_11x9);
                #endif
				
			}
			else
			{
				s_ui8FlagSpO2DisClearScreem = 1;
				OLED_DisplayNum(1,20,SpO2_Value[2],Font_11x18);
				OLED_DisplayNum(14,20,SpO2_Value[1],Font_11x18);
				OLED_DisplayNum(27,20,SpO2_Value[0],Font_11x18);
				OLED_DisplayBPM(40,28,51,37,DIS_ICON_Percent_11x9);
			}
			
//			/* 	Single display mode 	*/
//			if(SpO2_RealTimeSample_GetResultNum != 0)
//			{
//				flagSpO2SampleDone = SET;
//			}
		}
		else if (s_ui8SpO2ValidKeep == 0 && OFF == GetHR_SpO2DisplayMeasureStatus())
		{	
			if(s_ui8FlagSpO2DisClearScreem == 1)
			{
				s_ui8FlagSpO2DisClearScreem = 0;
				OLED_Fill(0,20,58,39,0);
			}
			OLED_DisplayNum(1,20,10,Font_11x18);
			OLED_DisplayNum(14,20,10,Font_11x18);
            #ifdef UI_CHINESE_ENABLE
                OLED_DisplayBPM(26,30,61,39,DIS_ICON_Percent_11x9);
            #else
                OLED_DisplayBPM(27,28,38,37,DIS_ICON_Percent_11x9);
            #endif
		}
	}

//	/* 	In single display mode, stop measurement while get one HR/SPO2 value 		*/
//	if((Device_Mode == Device_Mode_SPO2_HR) && (gSubFunc_Stat_Get(SPO2_HR_SINGLEWORK_STATE) == SPO2_HR_SINGLEWORK_STATE)
//		&& flagHRSampleDone == SET && flagSpO2SampleDone == SET)
//	{
//		gSubFunc_Stat_Set(SPO2_HR_SINGLEWORK_STATE, OFF); 
//		TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop);   /* 	Stop the measurement 		*/		
//	}
	
	
    /* 及时采集时，发送及时采集数据 */
	if((Device_Mode == Device_Mode_CheckUp) && (gSubFunc_Stat_Get(SpO2_RealTimeSample_State) != OFF))  //及时采集在运行
	{
		if(true == SpO2HrFinal_pst->m_bIsSpO2Valid)
		{
//			if(SpO2_RealTimeSample_GetResultNum == 0)  //App连续采集，只要采集到有数据就传给App
//			{
//				s_ui8SpO2GetStableResultCnt = 0xff;  //fix this value for continously sample
				TS_SendEvent(gTsSPITranslateTaskID_c,gSPITranslateEventTxSpO2);   //产生发送SpO2事件,及时采集数据发送
//			}
//			else  //App只需要获取SpO2_RealTimeSample_GetResultNum个稳定数值
//			{
//				s_ui8SpO2GetStableResultCnt++;  //increase the SpO2 counter, the max number is 256
//				if(s_ui8SpO2GetStableResultCnt <= SpO2_RealTimeSample_GetResultNum)
//				{
//					TS_SendEvent(gTsSPITranslateTaskID_c,gSPITranslateEventTxSpO2);   //产生发送SpO2事件,及时采集数据发送
//				}
//			}
		}
//		else
//		{
//			if(SpO2_RealTimeSample_GetResultNum == 0)  //App连续采集，只要采集到有数据就传给App
//			{
//				s_ui8SpO2GetStableResultCnt = 0xff;  //fix this value for continously sample
//			}
//		}

	}
	
//	/* 	Stop Sample when in realtime and reach the sample numbers 		*/
//	if(Device_Mode == Device_Mode_CheckUp)
//	{
//		if(gSubFunc_Stat_Get(SPO2_HR_REALTIME_STATE) == SPO2_HR_REALTIME_STATE && 
//			s_ui8SpO2GetStableResultCnt == SpO2_RealTimeSample_GetResultNum && 
//			s_ui8HRGetStableResultCnt == HR_RealTimeSample_GetResultNum)
//		{
//			//SpO2/HR执行完及时采集
//			gSubFunc_Stat_Set(SPO2_HR_REALTIME_STATE,OFF);
//			TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop);     //发送SpO2停止测量事件，HR/SpO2测量为同一事件控制	
//			ChangeDeviceMode_FromCheckUpToRTC();
//		}
//		/* 	in case only SpO2 measurement 	*/
//		else if(gSubFunc_Stat_Get(SPO2_HR_REALTIME_STATE) == SpO2_RealTimeSample_State && 
//			s_ui8SpO2GetStableResultCnt == SpO2_RealTimeSample_GetResultNum)
//		{
//			gSubFunc_Stat_Set(SpO2_RealTimeSample_State,OFF);
//			TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop);     //发送SpO2停止测量事件，HR/SpO2测量为同一事件控制	
//			ChangeDeviceMode_FromCheckUpToRTC();
//		}
//		else if(gSubFunc_Stat_Get(SPO2_HR_REALTIME_STATE) == HR_RealTimeSample_State && 
//			s_ui8HRGetStableResultCnt == HR_RealTimeSample_GetResultNum)
//		{
//			gSubFunc_Stat_Set(HR_RealTimeSample_State,OFF);
//			TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop);     //发送SpO2停止测量事件，HR/SpO2测量为同一事件控制	
//			ChangeDeviceMode_FromCheckUpToRTC();
//		}
//	}
//	
	
	/*      Write Data into EEPROM      */

	
//    printf("SpO2 Valid = %x, Hr Valid = %x \r\n",SpO2HrFinal_pst->m_bIsSpO2Valid,SpO2HrFinal_pst->m_bIsHrValid);
    
	flagIsMonitorDatUpdate = FALSE;
	if(
		(gSubFunc_Stat_Get(SPO2_HR_MONITOR_STATE) != OFF) 
		&& (true == SpO2HrFinal_pst->m_bIsSpO2Valid)
		&& (true == SpO2HrFinal_pst->m_bIsHrValid)
	)
	{
		
#if MOD_CTRL_IS_DATA_DOUBLE_CHECK_IN_MONITOR_MODE == true
		/* 	handle the case, which exceed up/low limit 	*/
		if(
			/* 	abnormal value 	*/
			(
				((gHRSpO2Val.HrSpO2DataRep_pst->m_ui8HrVal < gHRSpO2Val.HR_Limit.LowLimit) || (gHRSpO2Val.HrSpO2DataRep_pst->m_ui8HrVal > gHRSpO2Val.HR_Limit.HigLimit))
			||	((gHRSpO2Val.HrSpO2DataRep_pst->m_ui8SpO2Val <= SPO2_HI_CONF_LO))
			)
		)
		{
			if(s_ui8HrSpO2MonitorStableCnt < s_HrMonitorStore_st.m_ui16DataLen)
			{
				/* 	store current low reliable value and restart measure 		*/
                arr_push_data_func_ui8(&s_HrMonitorStore_st, &(gHRSpO2Val.HrSpO2DataRep_pst->m_ui8HrVal),1);
                arr_push_data_func_ui8(&s_SpO2MonitorStore_st, &(gHRSpO2Val.HrSpO2DataRep_pst->m_ui8SpO2Val),1);
				/* 	Increase the counter 		*/
				s_ui8HrSpO2MonitorStableCnt++;
				//ALG_PRINTF("low thres cnt = %d \r\n",s_ui8HrSpO2MonitorStableCnt);
				if(
					/* Only Monitor running 	*/
					(gSubFunc_Stat_Get(SPO2_HR_SINGLEWORK_STATE) == OFF )
					&&	(gSubFunc_Stat_Get(SPO2_HR_REALTIME_STATE) == OFF )
				)
				{
					/* 	Restart measurement 		*/
					TS_SendEvent(gTsSpO2TaskID_c, gSpO2EventBaselineStart);
				}
				/* 	No update 					*/
				flagIsMonitorDatUpdate = FALSE;
			}
			else if(s_ui8HrSpO2MonitorStableCnt == s_HrMonitorStore_st.m_ui16DataLen)
			{
				/* 	Push the average value to output global varibles 	*/
				
				/* 	Hr value 	*/
				arr_get_mean_func_ui8(&s_HrMonitorStore_st,&valTmp);

				gHRSpO2Val.HrSpO2DataRep_pst->m_ui8HrVal = valTmp;
				
				/* 	SpO2 value 	*/
				arr_get_mean_func_ui8(&s_SpO2MonitorStore_st,&valTmp);
				
				gHRSpO2Val.HrSpO2DataRep_pst->m_ui8SpO2Val = valTmp;
				
				/* 	reset to avoid deadlock	*/
				s_ui8HrSpO2MonitorStableCnt=0;
				/*  update monitor data		*/
				flagIsMonitorDatUpdate = TRUE;
			}
			else
			{
				/* 	Set to zero 	*/
				s_ui8HrSpO2MonitorStableCnt = 0;
				/*  update monitor data					*/
				flagIsMonitorDatUpdate = FALSE;
			}
		}
		else
		{
			s_ui8HrSpO2MonitorStableCnt = 0;
			/*  update monitor data					*/
			flagIsMonitorDatUpdate = TRUE;
		}
#else
		/*  update monitor data					*/
		flagIsMonitorDatUpdate = TRUE;
#endif // end of MOD_CTRL_IS_DATA_DOUBLE_CHECK_IN_MONITOR_MODE
		
		
		if(flagIsMonitorDatUpdate == TRUE)
		{
			/* 	Package the spo2/Hr value 	*/
			data_pack_spo2hr(
							&date_s,               //RTC date
							&rtc_time,             //RTC time
							gHRSpO2Val.HrSpO2DataRep_pst->m_ui8SpO2Val,
							SpO2ID,
							SpO2_SaveData
						);
			data_pack_spo2hr(
							&date_s,               //RTC date
							&rtc_time,             //RTC time
							gHRSpO2Val.HrSpO2DataRep_pst->m_ui8HrVal,
							HeartRateID,
							HR_SaveData
						);
			/* 	Write data into eeprom 		*/            
			//if(GetM95M01State(MONITOR_MODEL_VALUE_ID,M95M01_CAPACITY_SPACE) < (sizeof(SpO2_SaveData) + sizeof(HR_SaveData)))
			if(0)
            {
					//监测模板数据剩余存贮空间小于写入的数据大小，存贮空间报警
					#ifdef EEPROM_DEBUG
						ALG_PRINTF("Not Enough Space\r\n");
					#endif
					gAlarmNotEnoughSpace = true;
					//TS_SendEvent(gTsAlarmTaskID_c,gAlarmEventNotEnoughSpace); 
			}
			else
			{
					gAlarmNotEnoughSpace = false;
			}

            /* Create The Moniter Template Data Store Partition */
            u32Date = CovernDateto32();
            ExtFLASH_ExtCreatePartition(u32Date);  
            
            /* Save SpO2 Monitor Template data */
            DataMemoryWrite(MONITOR_TYPE,SpO2_SaveData,sizeof(SpO2_SaveData));
            //APP_ERROR_CHECK(Err_Code);
						
            /* Save HR Monitor Template data */
            DataMemoryWrite(MONITOR_TYPE,HR_SaveData,sizeof(HR_SaveData));
            //APP_ERROR_CHECK(Err_Code);
            /* write data into eeprom end */
                        
            if(
                (Device_State == Device_WorkWithAPP) 
                && (gSubFunc_Stat_Get(SPO2_HR_MONITOR_STATE) != OFF)
            )   //和App连接在一起并且所有监测模板运行结束，触发同步事件
            {
                TS_SendEvent(gTsSyncDataTaskID_c,gSyncDataFormat2EventStart);  //产生开始数据同步事件
                #ifdef SyncData_DEBUG
                    printf("Sync Data Start By MT SpO2...\r\n");
                #endif
            }
			
			/* 	Stop Measurement only when HR/SPO2 off 		*/
			gSubFunc_Stat_Set(SPO2_HR_MONITOR_STATE,OFF);    // 该标志位的置位在MonitorTemplate_Task_Handler中

			TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop);     //发送SpO2停止测量事件，HR/SpO2测量为同一事件控制,此时必定有HR值
			
			if(
				((gHRSpO2Val.HrSpO2DataRep_pst->m_ui8HrVal < gHRSpO2Val.HR_Limit.LowLimit) || (gHRSpO2Val.HrSpO2DataRep_pst->m_ui8HrVal > gHRSpO2Val.HR_Limit.HigLimit))
			||	(gHRSpO2Val.HrSpO2DataRep_pst->m_ui8SpO2Val < SPO2_HIGH_CONF_THRES)
			)
			{
				if(GetMonitorTemplateType() == NormalMT)  //正常监测模板下
				{
					//触发异常监测模板执行
					#ifdef Monitor_Template_Debug
						ALG_PRINTF("SpO2 Trigger Excetion Monitor Template\r\n");
					#endif
					TS_SendEvent(gTsMonitorTemplatID_c,gExcetionMonitorTemplateEventStart);  //产生异常监测模板开始事件
				}else if(GetMonitorTemplateType() == ExcetionMT)  //异常监测模板下
				{
					//设置监测模板采集数据位异常?
					#ifdef Monitor_Template_Debug
						ALG_PRINTF("SpO2 Set MT Measure Val to ExcetionVal\r\n");
					#endif
					SetMonitorTemplateMeaResultType(ExcetionVal);
				}
			}
		}
	}
	
	/* 产生参数门限报警 */
	if(
		(true == SpO2HrFinal_pst->m_bIsHrValid) 
	&&	(Device_State != Device_WorkWithAPP)
	) //及时采集模式设备不报警
	{
		if((gHRSpO2Val.HrSpO2DataRep_pst->m_ui8HrVal < gHRSpO2Val.HR_Limit.VLowLimit) || (gHRSpO2Val.HrSpO2DataRep_pst->m_ui8HrVal > gHRSpO2Val.HR_Limit.VHigLimit))
		{
			AlarmLevelSet(ALARM_LEVEL_HIGH);
			//gAlarmType.AlarmLevel.High = ON;  //高级告警
			//gAlarmType.AlarmStatus = ON;			
			TS_SendEvent(gTsAlarmTaskID_c,gAlarmEventParameterLimit); 
		}
		else if((gHRSpO2Val.HrSpO2DataRep_pst->m_ui8HrVal < gHRSpO2Val.HR_Limit.LowLimit) || (gHRSpO2Val.HrSpO2DataRep_pst->m_ui8HrVal > gHRSpO2Val.HR_Limit.HigLimit))
		{
			AlarmLevelSet(ALARM_LEVEL_LOW);	
			TS_SendEvent(gTsAlarmTaskID_c,gAlarmEventParameterLimit);
		}
	}
	
	
	if((true == SpO2HrFinal_pst->m_bIsSpO2Valid) && (Device_State != Device_WorkWithAPP)) //及时采集模式设备不报警
	{
		if((gHRSpO2Val.HrSpO2DataRep_pst->m_ui8SpO2Val < gHRSpO2Val.SpO2_Limit.VLowLimit) || (gHRSpO2Val.HrSpO2DataRep_pst->m_ui8SpO2Val > gHRSpO2Val.SpO2_Limit.VHigLimit))
		{
			AlarmLevelSet(ALARM_LEVEL_HIGH);	
			TS_SendEvent(gTsAlarmTaskID_c,gAlarmEventParameterLimit); 
		}
		else if((gHRSpO2Val.HrSpO2DataRep_pst->m_ui8SpO2Val < gHRSpO2Val.SpO2_Limit.LowLimit) || (gHRSpO2Val.HrSpO2DataRep_pst->m_ui8SpO2Val > gHRSpO2Val.SpO2_Limit.HigLimit))
		{
			AlarmLevelSet(ALARM_LEVEL_LOW);
			TS_SendEvent(gTsAlarmTaskID_c,gAlarmEventParameterLimit);
		}
	}
}


static void data_pack_spo2hr(
							date_str_typedef    *pDate_s,               //RTC date
							RTC_TimeTypeDef     *pRtc_time,             //RTC time
							uint8_t HrSpO2Val,
							uint8_t paraId,
							uint8_t * pDatPack
)
{
	#ifdef Monitor_Template_Debug
		ALG_PRINTF("MT Save SpO2 Valid\r\n");
	#endif
    
	pDatPack[0] = pRtc_time->RTC_Hours;
	pDatPack[1] = pRtc_time->RTC_Minutes;
	pDatPack[2] = pRtc_time->RTC_Seconds;
	
	pDatPack[3] = paraId;
	
	pDatPack[4] = GetMonitorTemplateType();

	pDatPack[5] = HrSpO2Val;   		/*	Value	*/
	pDatPack[6] = 0;		
	pDatPack[7] = 0;		
}

static void u8toBCDConvert5Pos(uint8_t DataIn, uint8_t *pDatOut)
{
	pDatOut[0] =         DataIn%10UL;			/* 	Unit Digi 	*/
	pDatOut[1] =        ((DataIn/10)%10);		/* 	Decade 		*/
	pDatOut[2] =       DataIn/100;				/*	Hundreds	*/
}

/**
* @brief 		Clear the SPOT Check counter 
*/
static void cc_spot_check_cnt_clr(void)
{
	s_uiSpotCheckCnt = 0;
}
/**
* @brief 		Increase the Spot Check counter
*/
static void cc_spot_check_cnt_inc(void)
{
	s_uiSpotCheckCnt++;
}




// end file
