/*
* Copyright(c) 2015, CloudCare Healthcare Co.,Ltd
* All rights reserved
* File Name:                cc_app_afe_drv_interface.c
* Descprition:              interface between AFE driver, MCU and Algorithm Moduel
*                           Need to be change when change AFE or MCU
* Created Date:             2016/03/15
* Author:                   yun.zhu@ywkang.com  
* Current Reversion:        V1.0
* Reversion History:
*   Rev *.*    
*     Author:     
*     Change Date:
*     Change List:
*/
#include "cc_gsensor_translate_layer.h"
#include "cc_app_gsensor_drv_interface.h"
#include "bma250e.h"
#include "bma250e.h"
#include "platform.h"
#include "stm32l1xx.h"
#include "Timer.h"
#include "Usart.h"
#include "common.h"
#include "PowerManage.h"
#include "IIC_GPIO.h"
#include "IIR_Filter_Paramter.h"
#include "cc_alg_app_interface.h"



static uint16_t mMoveCheckNoAlarmCnt;


static GsensorDrvInst_Typedef m_GsensorTransInst_st;

static void cc_sys_gsensor_data_read(GsensorDataType * GsenDat_pst);
static void cc_sys_gsensor_spo2_mov_check_init(uint16_t SampleFreq);
static bool is_motion_notice_needed(void);
/******************************************
* Name:             cc_app_afe_drv_interface_init()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      Init the function pointer of afe
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_app_gsensor_drv_interface_init(void)
{
    
    /*  Init the instance of AFE translate layer    */
    m_GsensorTransInst_st.cc_alg_gsensor_data_read = cc_sys_gsensor_data_read;
    m_GsensorTransInst_st.cc_alg_gsensor_init = cc_sys_gsensor_spo2_mov_check_init;
//    m_GsensorTransInst_st.cc_alg_gsensor_motion_notice = cc_sys_gsensor_spo2_motion_notice_handle;
    m_GsensorTransInst_st.m_pi16DatBufX = accDataBackup_x;
    m_GsensorTransInst_st.m_pi16DatBufY = accDataBackup_y;
    m_GsensorTransInst_st.m_pi16DatBufZ = accDataBackup_z;
    
    cc_gsensor_translate_layer_init(&m_GsensorTransInst_st);
}

/******************************************
* Name:             cc_sys_gsensor_data_read()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      Init the function pointer of afe
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
static void cc_sys_gsensor_data_read(GsensorDataType * GsenDat_pst)
{
	bma2x2acc_t sensorRawDat={0,0,0};
	/* 	Get the raw data 	*/
	bma2x2_read_accel_xyz(&sensorRawDat);

    GsenDat_pst->m_i16DatX = sensorRawDat.x;
    GsenDat_pst->m_i16DatY = sensorRawDat.y;
    GsenDat_pst->m_i16DatZ = sensorRawDat.z;
}

/*******************************************************************************
* Function Name  : cc_sys_gsensor_spo2_mov_check_init
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void cc_sys_gsensor_spo2_mov_check_init(uint16_t SampleFreq)
{
	/*	Init the SPI port to BAM250E	*/
	BMA250E_SPI_Oper_Init();
	
	/* 	Disable Init from BMA250E		*/
	BMA250E_Int1_Port_Disable();
	BMA250E_Int2_Port_Disable();
	    
	/* 	Init the BMA250E 		*/
	BMA250E_Init_SpO2_MoveCheck();

	/* 	Clear Static varibles 	*/
	mMoveCheckNoAlarmCnt = 0;
}


void cc_sys_gsensor_spo2_motion_notice_handle(bool isMotionCheckPass)
{
    
	/* 	Increase the counter 	*/
	if(is_motion_notice_needed() == true)
	{
		mMoveCheckNoAlarmCnt = (mMoveCheckNoAlarmCnt == 0xff) ? 0xff : mMoveCheckNoAlarmCnt + 1u;
	}

	/* 	Check the return status 	*/
	
	if(mMoveCheckNoAlarmCnt > GSEN_MOVECHECK_START_CNT_LIMIT && isMotionCheckPass == false)
	{
		if(
			((MonitorTemplate.MTID != kICTMonitorTemplateFreeRunID) || (true != isFreeRunKickOff()))
			&& (is_motion_notice_needed() == true)
			)
		{
			TS_SendEvent(gTsSpO2TaskID_c, gSpO2EventMotionDetected);
		}	
	}
	else
	{
		/*	Clear Motion Display for next motion notice 		*/
		isMotionNotice_Stat_Set(FALSE);
		/* Add code here to send event for moving check pass */
		if(OccupiedDisMove == OLEDDisplay_Stat_Get())  
		{
			if(gSubFunc_Stat_Get(SPO2_HR_SINGLEWORK_STATE | SPO2_HR_REALTIME_STATE) != OFF)
			{
				TS_SendEvent(gOledDisTaskID,gOledDisEventDisplayAllRAM_c); 
				Set_OLED_Dis_Status(OLEDDisON);
				/* 	When in realtime sampling, re-kick the OLED shutdown counter 	*/
				if(gSubFunc_Stat_Get(SPO2_HR_REALTIME_STATE ) != OFF)
				{
					TS_SendEvent(gOledDisTaskID,gOledDisEventClearModeDis_c);
				}
			}
			else
			{
				OLEDDisplay_Stat_Set(NormalDis); 
				Set_OLED_Dis_Status(OLEDDisShutDown); 
				OLED_DisplayCtl(OFF);
			}
        }
    }
}

static bool is_motion_notice_needed(void)
{
    /*  Need to be update later     */
    return(true);
}
