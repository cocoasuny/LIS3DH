/*
* Copyright(c) 2015, CloudCare Healthcare Co.,Ltd
* All rights reserved
* File Name:                cc_gsensor_translate_layer.h
* Descprition:              the cc_afe_translate_layer.h files for translate layer
* Created Date:             2016/03/14
* Author:                   yun.zhu@ywkang.com  
* Current Reversion:        V1.0
* Reversion History:
*   Rev *.*    
*     Author:     
*     Change Date:
*     Change List:
*/

#ifndef _CC_GSENSOR_TRANSLATE_LAYER_H
#define _CC_GSENSOR_TRANSLATE_LAYER_H

#include "stdio.h"
#include "cc_alg_types.h"

/**
* @brief Define the sensor basic parametes
*/

#define SENSOR_DATA_BUF_LEN 			    (256) 		//50hz odr, 2s interval
#define SENSOR_DATA_NUM_PER_PROC 		    (50)

/**
* @brief the typedef of gsensor data type
*/

typedef struct
{
    signed short m_i16DatX;
    signed short m_i16DatY;
    signed short m_i16DatZ;
} GsensorDataType;

/**
* @brief Definition of gsensor read function
*/
#define CC_ALG_GSEN_RD_PRT void (* cc_alg_gsensor_data_read)(GsensorDataType *)

/**
* @brief Definition of gsensor init function
*/
#define CC_ALG_GSEN_INIT_PRT void (* cc_alg_gsensor_init)(unsigned short)

/**
* @brief Definition of gsensor motion notice function
*/
#define CC_ALG_GSEN_MOTION_NOTIC_PRT void (* cc_alg_gsensor_motion_notice)(bool)



/**
* @brief the typedef of gensor process instance
*/
typedef struct
{
    signed short *   m_pi16DatBufX;
    signed short *   m_pi16DatBufY;
    signed short *   m_pi16DatBufZ;
    unsigned short    m_ui16DatBufLen;
    unsigned short    m_ui16NewDatLen;
    //unsigned short    m_ui16MotionThreshold;
    CC_ALG_GSEN_RD_PRT;
    CC_ALG_GSEN_INIT_PRT;
    //CC_ALG_GSEN_MOTION_NOTIC_PRT;
} GsensorDrvInst_Typedef;


/**
* @brief function definition
*/


void cc_gsensor_translate_layer_init(GsensorDrvInst_Typedef * GsensorDrvInstInit_pst);

//void cc_alg_gsensor_motion_notice(bool isMotionCheckPass);

void cc_alg_gsensor_spo2_motion_init(unsigned short ui16SampleFreq);

void cc_alg_gsensor_data_read(GsensorDataType * GsenData);

void cc_alg_get_axes_dat_buf(int16_t ** pi16X, int16_t ** pi16Y, int16_t ** pi16Z);


#endif //_CC_GSENSOR_TRANSLATE_LAYER_H

//end file
