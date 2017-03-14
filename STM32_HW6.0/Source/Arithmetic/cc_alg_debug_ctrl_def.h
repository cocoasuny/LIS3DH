/*
* Copyright(c) 2015, CloudCare Healthcare Co.,Ltd
* All rights reserved
* File Name:                cc_alg_debug_ctrl_def.h
* Descprition:              the cc_alg_debug_ctrl_def.h files for translate layer
* Created Date:             2016/03/14
* Author:                   yun.zhu@ywkang.com
* Current Reversion:        V1.0
* Reversion History:
*   Rev *.*
*     Author:
*     Change Date:
*     Change List:
*/

#ifndef _CC_ALG_DEBUG_CTRL_DEF_H_
#define _CC_ALG_DEBUG_CTRL_DEF_H_


/**
* @brief Enable or Disable MSDST algorithm
*/
#define MOD_CTRL_IS_ENABLE_MSDST        false

/**
* @brief Enable or Disable MSDST algorithm Printf
*/
#define PRT_CTRL_IS_ENABLE_MSDST        false


/**
* @brief 		Enable or Disable LMS Filter
* @details		NLMS or LMS, Don't enable both at the same time
*/
#define MOD_CTRL_IS_ENABLE_NLMS			false
#define MOD_CTRL_IS_ENABLE_LMS			false

/**
* @brief Enable or Disable LMS Filter raw data printf
*/
#define PRT_CTRL_IS_ENABLE_NLMS_RAW_DATA			false


/**
* @brief Enable or Disable Wavelet Filter 
*/
#define MOD_CTRL_IS_ENABLE_WAVELET			true

/**
* @brief Enable or Disable Gsensor Frequency Point Cancel Function 
*/
#define MOD_CTRL_IS_ENABLE_GSEN_FREQ_CANCEL			false


/**
* @brief Enable or Disable Baseline HR calculation and used as reference in HR update
*/
#define MOD_CTRL_IS_ENABLE_BASELINE_HR_CAL			true




/**
* @def Force to DISABLE the function of clear-while motion
*/
#define 	MOD_CTRL_IS_BLOCK_CLEAR_WHILE_MOTION  		false

/**
* @def Enable or Disable the function of clear-while-motion
*/
#if MOD_CTRL_IS_BLOCK_CLEAR_WHILE_MOTION == true
#define MOD_CTRL_IS_CLEAR_WHILE_MOTION_EN 	false
#else
#define MOD_CTRL_IS_CLEAR_WHILE_MOTION_EN 	true
#endif


/**
* @def Force to DISABLE the motion check
*/
#define 	MOD_CTRL_IS_BLOCK_MOTION_CHECK  		false

/**
* @def Enable or Disable the motion check
*/
#if MOD_CTRL_IS_BLOCK_MOTION_CHECK == true ||\
	PRTF_RAW_DATA_DUMP_WAVELET == true ||\
	PRTF_DAT_DUMP_ONLY == true
	#define MOD_CTRL_IS_MOTION_CHECK_EN 	false
#else
	#define MOD_CTRL_IS_MOTION_CHECK_EN 	true
#endif







/*****************************************************************************/
/*                   Debug mode switch                                       */
/*****************************************************************************/
//#define 		SPO2_DIAG_DEBUG_MODE
//#define 		SPO2_ADJ_DEBUG_MODE
//#define 		SPO2_EVENT_DEBUG_MODE
//#define 		SPO2_RUNTIME_EST_MODE

/*    Printf Switch    */

//#define 		PRTF_AFE_REG_DUMP

//#define 		PRTF_SAMPLE_VALID_CHK


/**
* @def PRTF_SNR_SPO2_CAL 
*  Disable or Enable Printf for SpO2 calculation with SNR cal
*  -# false:Disable
*  -# true:Enable
*/
#define 		PRTF_SNR_SPO2_CAL       false

//#define       PRTF_SIGNAL_QUALITY

//#define       PRTF_MOV_R_MATCH

//#define 		PRTF_CZT_TOP

#define         PRTF_RAW_DATA_DUMP_WAVELET      false

/**
* @def 		Enable or Disable Raw data dump function
*/
#define 		PRTF_DAT_DUMP_ONLY      			false

//#define         PRTF_R_KALMAN_FILTER

//#define         PRTF_RST_SPEED_TEST

//#define             PRTF_RVAL_PROCESS

//#define     PRTF_RVAL_FOR_COMPARE

#define     PRTF_HR_CAL_TRACK       false

#define 	SPO2_ERROR_TRACK_DEBUG		false





#endif // _CC_ALG_DEBUG_CTRL_DEF_H_









// end file
