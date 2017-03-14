
//******************************************************************************
//  Claude ZHU
//  Hardware Team
//  (C) iCareTech Inc., 2013
//
//
//
//-------------------------------------------------------------------------------

#ifndef GSENSOR_DATA_PROCESS_H_
#define GSENSOR_DATA_PROCESS_H_

#include "platform.h"
#include "stm32l1xx.h"
#include "PowerManage.h"
#include "bma250e.h"
#include "bma2x2.h"
#include "Process_Acc_Data.h"




typedef enum{GSEN_RANGE_2G = 3u, GSEN_RANGE_4G = 2u, GSEN_RANGE_8G = 1u, GSEN_RANGE_16G = 0u}gSensorRangeType;


/* 	Link to gsensor data out 	*/
#define gSensorDatBufX 		accDataBackup_x
#define gSensorDatBufY		accDataBackup_y
#define gSensorDatBufZ		accDataBackup_z


/* 	Get the Axes data from global varibles 		*/
#define DATA_Z(i) gSensorDatBufZ[i]
#define DATA_X(i) gSensorDatBufX[i]
#define DATA_Y(i) gSensorDatBufY[i]

#define SENSOR_DATA_CZT_LEN 			(96)

#define SENSOR_DAT_DOWN_SAMPLE_INTERVAL 	(4)
#define SENSOR_DAT_DOWN_SAMPLE_DAT_LEN 		(44)

#define SENSOR_DATA_BUF_LEN 			256 		//50hz odr, 2s interval

#define SENSOR_DATA_BUF_OFFSET 		50

#define SENSOR_DATA_BUF_OFFSET_ANTI_MOV     10

#define SENSOR_DATA_STD_VAL_POW_SHIFT       10


#define SPO2_MOVE_THRES_NIGHT						(10240)		/* 	For 4G range(1g = 128), about 0.01g offset		*/

#define SPO2_MOVE_THRES_DAY							(2560)		/* 	For 4G range(1g = 128), about 0.03g offset		*/

#define SPO2_MOVE_CHECK_TIME_SLOT 			(60)		/* 	NO Second alarm in 1 minutes 		*/

/*	define for slope detection 		*/
#define GSEN_SLOPE_CHEK_RANGE_LOW 		(SENSOR_DATA_BUF_LEN - 64)
#define GSEN_SLOPE_CHEK_RANGE_HIGH		(SENSOR_DATA_BUF_LEN - 16)

#define GSEN_WAKEUP_CHEK_NUM 		(64)
#define GSEN_WAKEUP_CHEK_Y_ZX_RATIO 	(16)
#define GSEN_WAKEUP_CHEK_Y_STABLE_NUM 	(32)


#define GSEN_SLOPE_CHEK_NUM 		(GSEN_SLOPE_CHEK_RANGE_HIGH - GSEN_SLOPE_CHEK_RANGE_LOW)	
#define GSEN_SLOPE_CHEK_SER 			(10)

#define GSEN_SLOPE_CHEK_THRE			(20)		/* 		Need refine this value	*/

#define GSEN_SLOPE_COMP_THRE 			(8) 		/* For 16G range 		*/

#define GSEN_STABLE_CHEK_OFFSET				(5)
#define GSEN_STABLE_CHEK_NUM 			(12)

#define GSEN_Z_TOP_THRES_LOW				(22)	/* 	For 16G range		*/
#define GSEN_Z_TOP_THRES_HIGH				(42)	/* 	For 16G range		*/

#define GSEN_X_TOP_THRES_LOW				(-10)	/* 	For 16G range		*/
#define GSEN_X_TOP_THRES_HIGH				(10)	/* 	For 16G range		*/

#define GSEN_Y_TOP_THRES_LOW				(-10)	/* 	For 16G range		*/
#define GSEN_Y_TOP_THRES_HIGH				(10)	/* 	For 16G range		*/

#define GSEN_TOP_STABLE_THRES 				(10)

#define GSEN_TOP_STABLE_STD_LEN				(16)

#define GSEN_TOP_STABLE_STD_THRES 			(1600)

#define IS_AXES_TOP_STABLE(axes_index,lowThre,highThre) 	(axes_index > lowThre && axes_index < highThre)
//#define IS_X_AXES_TOP_STABLE(axes_x) 	(axes_x > GSEN_X_TOP_THRES_LOW && axes_x < GSEN_X_TOP_THRES_HIGH)
//#define IS_Y_AXES_TOP_STABLE(axes_y) 	(axes_y > GSEN_Y_TOP_THRES_LOW && axes_y < GSEN_Y_TOP_THRES_HIGH)



#define GSEN_HAND_WAVE_TIME_LIMIT 			(10)

#define GSEN_STEPCNT_WAKEUP_THRES 			(256)

#define GSEN_MOVECHECK_START_CNT_LIMIT 		(3)

/* 	External varibles declaration 		*/


//void gSensor_Stat_Set(SubModuleStat newState);

//SubModuleStat gSensor_Stat_Get(void);

void Gsensor_Task_Handler(event_t gSensorEvent);

//void Gsensor_Int2_Handler(void);

void Gsensor_Int1_Handler(void);

CheckStat cc_gSensor_SpO2_move_check(uint16_t * pStdVal, int32_t *pStdValArr, uint8_t StdValArrOffset);

void Gsensor_SpO2_Move_Check_Init(void);

void gSensor_Raw_Dat_read_for_SpO2(uint8_t datIndex);

CheckStat gSensor_Wave_Hand_Check(gSensorRangeType gSensorRange);

void gSensor_StepCnt_Wakeup(int16_t * pAccX,int16_t *pAccY, int16_t *pAccZ, uint16_t lenData);

void gSensor_Raw_Dat_Copy_SpO2(void);

void gSensor_Raw_Dat_Copy_SpO2_for_StepCnt(uint8_t sampleInterval, gSensorRangeType gSenRange, uint16_t rawDatLen);



extern int16_t accDatBackTmpX[];
extern int16_t accDatBackTmpY[];
extern int16_t accDatBackTmpZ[];


#endif

//end files

