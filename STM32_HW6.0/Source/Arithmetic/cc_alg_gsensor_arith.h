/*
* Copyright(c) 2015, CloudCare Healthcare Co.,Ltd
* All rights reserved
* File Name:                cc_alg_gsensor_arith.h
* Descprition:              the functions to handle the gsensor data
* Created Date:             2016/03/17
* Author:                   yun.zhu@ywkang.com  
* Current Reversion:        V1.0
* Reversion History:
*   Rev *.*    
*     Author:     
*     Change Date:
*     Change List:
*/

#ifndef _CC_ALG_GSENSOR_ARITH_H
#define _CC_ALG_GSENSOR_ARITH_H

#include "cc_alg_types.h"
#include "IIR_Filter_Paramter.h"
#include "arm_math.h"
#include "cc_gsensor_translate_layer.h"

#define GSEN_BOSH250E_COMB_DAT_SRS 			(5) 	/* 	Right shift bits for combined data */



void cc_alg_gSensor_init(void);
void cc_gSensor_data_read_SpO2(uint8_t);
void cc_gsen_dat_std_cal(int16_t * pDatIn, uint16_t datLen, uint32_t * valStd);
bool cc_gSensor_move_check(int16_t * sensorDatBufC, uint16_t dataLen, uint32_t moveThreshold, uint16_t * pValStd);
bool cc_gSensor_PreSpO2_MoveCheck(uint16_t * pStdVal, int32_t *pStdValArr, uint8_t StdValArrOffset, uint32_t uiMotionCheckThres);
void cc_alg_gSensor_BP_Filter(int16_t * sensorDatBufC, uint16_t uDatLen);
void cc_alg_gSensor_BP_Filter_Init(void);
void cc_alg_gsensor_prt_data(uint16_t ui16Index);
void cc_gSensor_data_copy_to_buf(void);
FUNC_RETURN_TYPE cc_gSensor_dat_combine(int32_t * piSensorDatComb, uint16_t uiDatLen, uint8_t uiBitShift);

#endif //_CC_ALG_GSENSOR_ARITH_H

//end file
