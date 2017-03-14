/*
* Copyright(c) 2015, CloudCare Healthcare Co.,Ltd
* All rights reserved
* File Name:                cc_afe_translate_layer.h
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

#ifndef _CC_AFE_TRANSLATE_LAYER
#define _CC_AFE_TRANSLATE_LAYER

#include "stdio.h"


/**
* @brief The function pointer's typedef
*/


/**
* @brief The AFE Power on Routine
*/
#define CC_AFE_POWERON_INIT     void(* cc_afe_drv_power_init)(void)
    
/**
* @brief The AFE Power Down Routine
*/
#define CC_AFE_POWERDOWN     void(* cc_afe_drv_powerdown)(void)
    

/**
* @brief The AFE Data Interrup Enable routine
*/
#define CC_AFE_DATA_INT_ENABLE     void(* cc_afe_drv_enable_data_int)(void)
    
/**
* @brief The AFE Data Interrup disable routine
*/
#define CC_AFE_DATA_INT_DISABLE     void(* cc_afe_drv_disable_data_int)(void)

/**
* @brief The AFE internal timming enable routine
*/
#define CC_AFE_RUN_ENABLE     void(* cc_afe_drv_enable_running)(void)

/**
* @brief The AFE hw diag enable routine
*/
#define CC_AFE_DIAG_ENABLE     void(* cc_afe_drv_diag_enable)(void)

/**
* @brief The AFE power saving mode enable routine
*/
#define CC_AFE_POWER_SAVING_ENABLE     void(* cc_afe_drv_power_saving_mode_enable)(void)

/**
* @brief The AFE power saving mode disable routine
*/
#define CC_AFE_POWER_SAVING_DISABLE     void(* cc_afe_drv_power_saving_mode_disable)(void)

/**
* @brief The AFE SET AVERAGE number routine
*/
#define CC_AFE_SET_AVERAGE_NUM     void(* cc_afe_drv_set_average_num)(unsigned int)


/**
* @brief The AFE get diage status routine
*/
#define CC_AFE_GET_DIAG_STATUS     unsigned int (* cc_afe_drv_get_diag_status)(void)

/**
* @brief The AFE set LED current
*/
#define CC_AFE_SET_LED_CURR     void(* cc_afe_drv_led_curr_set)(unsigned char, unsigned char)

/**
* @brief The AFE set dc cancelation current
*/
#define CC_AFE_SET_DAC_CURR     void(* cc_afe_drv_Dac_set)(unsigned int)

/**
* @brief The AFE set dc cancelation current
*/
#define CC_AFE_SET_RF_VAL     void(* cc_afe_drv_rf_set)(unsigned int)

/**
* @brief The AFE set dc cancelation current
*/
#define CC_AFE_SET_CF_VAL     void(* cc_afe_drv_cf_set)(unsigned int)
    
/**
* @brief The AFE dac enable
*/
#define CC_AFE_DAC_ENABLE     void(* cc_afe_drv_dac_enable)(void)
    
/**
* @brief The AFE dac disable
*/
#define CC_AFE_DAC_DISABLE     void(* cc_afe_drv_dac_disable)(void)

/**
* @brief The AFE second gain set
*/
#define CC_AFE_SEC_GAIN_SET     void(* cc_afe_drv_second_gain_set)(unsigned int)

/**
* @brief The AFE reg dump
*/
#define CC_AFE_REG_DUMP     void(* cc_afe_drv_dump_reg)(void)
/**
* @brief The AFE reg sample reg read
*/
#define CC_AFE_SAMPLE_REG_READ     void (* cc_afe_drv_sample_reg_read)(signed int *)


/**
* @brief AFE instance define 
*/
typedef struct
{
    /*  function pointer list   */
    CC_AFE_POWERON_INIT;
    CC_AFE_POWERDOWN;
    CC_AFE_DATA_INT_ENABLE;
    CC_AFE_DATA_INT_DISABLE;
    CC_AFE_RUN_ENABLE;
    CC_AFE_DIAG_ENABLE;
    CC_AFE_GET_DIAG_STATUS;
    CC_AFE_POWER_SAVING_ENABLE;
    CC_AFE_POWER_SAVING_DISABLE;
    CC_AFE_SET_AVERAGE_NUM;
    CC_AFE_SET_LED_CURR;
    CC_AFE_SET_DAC_CURR;
    CC_AFE_SET_RF_VAL;
    CC_AFE_SET_CF_VAL;
    CC_AFE_DAC_ENABLE;
    CC_AFE_DAC_DISABLE;
    CC_AFE_SEC_GAIN_SET;
    CC_AFE_REG_DUMP;
    CC_AFE_SAMPLE_REG_READ;
} AfeDrvInst_Typedef;


void cc_afe_translate_layer_init(AfeDrvInst_Typedef * AfeDrvInstInit_pst);
void cc_afe_poweron_init(void);
void cc_afe_enable_data_int (void);
void cc_afe_disable_data_int (void);

/*  Functional Functions        */
void cc_afe_set_average_num(unsigned int ui32AveNum);
void cc_afe_diag_enable(void);
void cc_afe_enable_running(void);
void cc_afe_set_default_hw_mode(void);
void cc_afe_timing_init(void);
void cc_afe_powerdown(void);
void cc_afe_power_saving_mode_enable(void);
void cc_afe_power_saving_mode_disable(void);
unsigned int cc_afe_get_diag_status(void);
void cc_afe_led_curr_set(unsigned char ui8LED1Cur, unsigned char ui8LED2Cur);
void cc_afe_Dac_set(unsigned int ui32DacVal);
void cc_afe_rf_set(unsigned int ui32RfVal);
void cc_afe_cf_set(unsigned int ui32CfVal);
void cc_afe_second_gain_set(unsigned int ui32SecGain);
void cc_afe_dac_disable(void);
void cc_afe_dac_enable(void);
void cc_afe_dump_reg(void);
void cc_afe_sample_reg_read(signed int * pi32SampleData);
#endif //_CC_AFE_TRANSLATE_LAYER

//end file
