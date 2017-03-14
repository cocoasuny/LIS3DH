/*
* Copyright(c) 2015, CloudCare Healthcare Co.,Ltd
* All rights reserved
* File Name:                cc_gsensor_translate_layer.c
* Descprition:              interface between Gsensor driver and Algorithm Moduel
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

static GsensorDrvInst_Typedef *     GsensorDrvInst_pst;

/******************************************
* Name:             cc_gsensor_translate_layer_init()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      Init the function pointer of afe
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_gsensor_translate_layer_init(GsensorDrvInst_Typedef * GsensorDrvInstInit_pst)
{
    /*  Init the Driver Layer instance  */
    GsensorDrvInst_pst = GsensorDrvInstInit_pst;
    
    /*  Fullfill the internal parameters    */
    GsensorDrvInst_pst->m_ui16DatBufLen = SENSOR_DATA_BUF_LEN;
    GsensorDrvInst_pst->m_ui16NewDatLen = SENSOR_DATA_NUM_PER_PROC;
    //GsensorDrvInst_pst->m_ui16MotionThreshold = SENSOR_MOTION_THRESHOLD_SPO2;
}



/******************************************
* Name:             cc_alg_gsensor_data_read()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      to get the data from gensor
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_alg_gsensor_data_read(GsensorDataType * GsenData)
{
    if(NULL != GsensorDrvInst_pst->cc_alg_gsensor_data_read)
    {
        GsensorDrvInst_pst->cc_alg_gsensor_data_read(GsenData);
    }
}

/******************************************
* Name:             cc_alg_gsensor_spo2_motion_init()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      init the gsensor to default value
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_alg_gsensor_spo2_motion_init(unsigned short ui16SampleFreq)
{
    if(NULL != GsensorDrvInst_pst->cc_alg_gsensor_init)
    {
        GsensorDrvInst_pst->cc_alg_gsensor_init(ui16SampleFreq);
    }
}

/******************************************
* Name:             cc_alg_gsensor_motion_notice()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      notice the external if motion exceed the threshold
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
//void cc_alg_gsensor_motion_notice(bool isMotionCheckPass)
//{
//    if(NULL != GsensorDrvInst_pst->cc_alg_gsensor_motion_notice)
//    {
//        GsensorDrvInst_pst->cc_alg_gsensor_motion_notice(isMotionCheckPass);
//    }
//}

/******************************************
* Name:             cc_alg_get_axes_dat_buf()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      set buffer to external buffer
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_alg_get_axes_dat_buf(int16_t ** pi16X, int16_t ** pi16Y, int16_t ** pi16Z)
{
    *pi16X = GsensorDrvInst_pst->m_pi16DatBufX;
    *pi16Y = GsensorDrvInst_pst->m_pi16DatBufY;
    *pi16Z = GsensorDrvInst_pst->m_pi16DatBufZ;
}






// end file
