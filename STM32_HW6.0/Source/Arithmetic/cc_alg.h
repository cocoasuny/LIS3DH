/*
* Copyright(c) 2015, CloudCare Healthcare Co.,Ltd
* All rights reserved
* File Name:                cc_alg.h
* Descprition:              the top .h files for algorithm process
* Created Date:             2016/03/11
* Author:                   yun.zhu@ywkang.com  
* Current Reversion:        V1.0
* Reversion History:
*   Rev *.*    
*     Author:     
*     Change Date:
*     Change List:
*/

#ifndef _CC_ALG
#define _CC_ALG

#include "cc_alg_types.h"



/**
* @brief Define the system debug information print function
*        should be the same format and usage as C standard "printf" function
* @note  Need to change this for difference implementation
*/
#define     ALG_PRINTF      printf

/**
* @brief Define the algorithm version number length
*/
#define     ALG_VER_NUM_LEN     (7)

/**
* @brief the structure of spo2_algorithm_body
*/
typedef struct{

    /*  function pointer that needs to be appointed at top application layer    */
    CC_ALG_SYS_EVENT_SEND_PRT;      //pointer to the system event distributuion function
    CC_ALG_SYS_DATA_HANDLE_PRT;     //pointer to the system data handler
    CC_ALG_SYS_CRITICAL_RESOURCE_APPLY_PRT;     //pointer to the system critical resource apply
    CC_ALG_SYS_CRITICAL_RESOURCE_RELEASE_PRT;
    CC_ALG_SYS_DELAY_MS_PRT;                    //system delay function
    /*  pointer to the input parameters     */
    ccAlgSpO2HrParam_st *       m_ccAlgParamIn_pst;
    
    /*  pointer to the output data          */
//    HR_SPO2_DAT_T *       m_HrSpO2DatOutput_pst;

} ccAlgBody_st;



FUNC_RETURN_TYPE cc_alg_inst_init(ccAlgBody_st * ccAlgBodyInit_pst);
void cc_alg_measure_start(HR_SPO2_INIT_PARA_Typedef * HrSpO2CalInitPara_pst);
void cc_alg_measure_stop(void);
void cc_alg_SpO2_Stat_Set(ccAlgSpO2HrStatus_en newState);
ccAlgSpO2HrStatus_en cc_alg_SpO2_Stat_Get(void);
HR_SPO2_DAT_T * cc_alg_get_spo2_hr_result_str(void);
void cc_alg_get_ver_num(const uint8_t ** pui8VerNumSer, uint8_t * pui8VerNumLen);
FUNC_RETURN_TYPE cc_alg_clr_spo2_hr_result_st(void);

#endif  //end of _CC_ALG
