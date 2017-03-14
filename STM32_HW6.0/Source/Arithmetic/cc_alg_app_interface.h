/**
* @brief	Interface between APP and algorithm
* @date 	2016/03/12
* @author 	yun.zhu@ywkang.com  
* @rev      V1.0
*/

#ifndef _CC_ALG_INTERFACE
#define _CC_ALG_INTERFACE
#include "platform.h"
#include "TS_Interface.h"
/*    Motion Detect Counter Up Limit */
#define MOTION_DETECT_LIMIT 		(30)  

/**
* @def		The spot check cycle number for normal mode test
* @details 	Current, set to 5 cycle, 500 count per cycle
*/
#define     SPOT_CHECK_CYCLE_NUM_LIMIT      (5*100*5)

/**
* @def 		Enable or Disable IN_Hospital Mode
* @details 	IN_Hospital Mode, will not response ERROR status, measure keeps continues until STOP command
*/
#define 	MOD_CTRL_IS_FORCE_RUNNING_ENABLE 		false

/**
* @def 		Enable or Disable 4 minutes exist function
* @details 	In normal mode, require 4 minutes exist if not in Hospital mode or not in FreeRun mode
*/
#if MOD_CTRL_IS_FORCE_RUNNING_ENABLE == true
	#define 	MOD_CTRL_IS_FOUR_MINUTES_EXIT_ENABLE 		false
#else
	#define 	MOD_CTRL_IS_FOUR_MINUTES_EXIT_ENABLE 		true
#endif

/**
* @def 		Enable or Disablr Data Double Check Function
* @details 	Enable -- Will double check the data in Monitor Mode
*					  If the SpO2/Hr is abnormal, will restart the measure and get the average
*			Disable -- Not Check the data
*/
#define 	MOD_CTRL_IS_DATA_DOUBLE_CHECK_IN_MONITOR_MODE 	false

/**
* @def 		DATA_DOUBLE_CHECK_NUM
* @details	the limit of data double check
*/
#define 	DATA_DOUBLE_CHECK_NUM 				(2)


/**
* @brief Enable or Disable 100% SpO2 vaule 
*/
#define MOD_CTRL_IS_BLOCK_HUNDRED_SPO2		true

/**
* @def 		SET the motion Check threshold 
* @details 	the corresponding data format is uint32_t
*			To disable this function, can set this value to 0x7fffffff
*/
#define SENSOR_MOTION_THRESHOLD_SPO2		(1596)
/**
* @def 		SET the clear-while-motion threshold 
* @details 	the corresponding data format is uint32_t
*			To disable this function, can set this value to 0x7fffffff
*/
#define SENSOR_DATA_ANTI_MOV_THRES          (1024)



void spo2_hr_cal_mod_init(void);
void SpO2_Task_Handler(event_t SpO2_Event);
SubModuleStat SPI1_Stat_Get(void);
void SPI1_Stat_Set(SubModuleStat newState);
void isMotionNotice_Stat_Set(bool newState);


#endif
//end file
