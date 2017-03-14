/*
* Copyright(c) 2015, CloudCare Healthcare Co.,Ltd
* All rights reserved
* File Name:                cc_alg_basic_func.h
* Descprition:              the functions to support the calculation
* Created Date:             2016/03/12
* Author:                   yun.zhu@ywkang.com  
* Current Reversion:        V1.0
* Reversion History:
*   Rev *.*    
*     Author:     
*     Change Date:
*     Change List:
*/

#ifndef _CC_ALG_BASIC_CAL_FUNC
#define _CC_ALG_BASIC_CAL_FUNC

#include "cc_alg_types.h"

/**
*   @brief  calculation parameters 
*/


void czt_arm_abs_max_q31(
                          int32_t * pSrc,
                          uint16_t blockSize,
                          uint32_t * pResult);

uint8_t czt_msb_index_cal(uint32_t * pSrc);

void CZT_Dat_upscale(int32_t * pBufDatIn, uint16_t blockSize, int8_t * pScaleFactor);

bool is_peak_2nd_peak_match_fun(uint16_t ui16PosLow, uint16_t ui16PosHigh, uint16_t ui16MatchWindow, uint16_t ui16SecPeakOffset);
bool is_peak_match_fun(uint16_t ui16PosA, uint16_t ui16PosB, uint16_t ui16MatchWindow);

//void cc_alg_data_copy_int32(
//  int32_t * pSrc,
//  int32_t * pDst,
//  uint32_t blockSize);

void cc_alg_scale_int32(
  int32_t * pSrc,
  int32_t scaleFract,
  int8_t shift,
  int32_t * pDst,
  uint32_t blockSize);

void cc_alg_dat_std_cal_uint8(uint8_t * pDatIn, uint8_t 	datLen, uint16_t * valStd);
void cc_alg_data_smooth_func_uint8(uint8_t ui8DatNew, uint8_t ui8DatSmooth, uint8_t u8SmoothFactor, uint8_t * pui8DatOut);
void cc_alg_data_smooth_func_uint16(uint16_t ui16DatNew, uint16_t ui16DatSmooth, uint16_t ui16SmoothFactor, uint16_t * pui16DatOut);
void cc_alg_data_smooth_func_f32(float32_t f32DatNew, float32_t f32DatSmooth, uint16_t ui16SmoothFactor, float32_t * pf32DatOut);
#endif //_CC_ALG_BASIC_CAL_FUNC

//end file

