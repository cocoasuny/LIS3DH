/*
* Copyright(c) 2015, CloudCare Healthcare Co.,Ltd
* All rights reserved
* File Name:                cc_afe_translate_layer.c
* Descprition:              interface between AFE driver and Algorithm Moduel
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
#include "cc_afe_translate_layer.h"


static AfeDrvInst_Typedef *     AfeDrvInst_pst;

/******************************************
* Name:             cc_afe_translate_layer_init()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      Init the function pointer of afe
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe_translate_layer_init(AfeDrvInst_Typedef * AfeDrvInstInit_pst)
{
    /*  Init the Driver Layer instance  */
    AfeDrvInst_pst = AfeDrvInstInit_pst;
}

/******************************************
* Name:             cc_afe_poweron_init()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      enable data interrupt from AFE
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe_poweron_init(void)
{
    if(NULL != AfeDrvInst_pst->cc_afe_drv_power_init)
    {
        AfeDrvInst_pst->cc_afe_drv_power_init();
    }
}







/******************************************
* Name:             cc_afe_enable_data_int()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      enable data interrupt from AFE
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe_enable_data_int (void)
{
    if(NULL != AfeDrvInst_pst->cc_afe_drv_enable_data_int)
    {
        AfeDrvInst_pst->cc_afe_drv_enable_data_int();
    }
}


/******************************************
* Name:             cc_afe_disable_data_int()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      enable data interrupt from AFE
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe_disable_data_int (void)
{
    if(NULL != AfeDrvInst_pst->cc_afe_drv_disable_data_int)
    {
        AfeDrvInst_pst->cc_afe_drv_disable_data_int();
    }
}

/******************************************
* Name:             cc_afe_diag_enable()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      init afe to diag status
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe_diag_enable(void)
{
    if(NULL != AfeDrvInst_pst->cc_afe_drv_diag_enable)
    {
        AfeDrvInst_pst->cc_afe_drv_diag_enable();
    }
}
/******************************************
* Name:             cc_afe_set_average_num()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      set the average number of output
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe_set_average_num(unsigned int ui32AveNum)
{
    if(NULL != AfeDrvInst_pst->cc_afe_drv_set_average_num)
    {
        AfeDrvInst_pst->cc_afe_drv_set_average_num(ui32AveNum);
    }
}

/******************************************
* Name:             cc_afe_enable_running()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      enable the afe to run
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe_enable_running(void)
{
    if(NULL != AfeDrvInst_pst->cc_afe_drv_enable_running)
    {
        AfeDrvInst_pst->cc_afe_drv_enable_running();
    }
}

/******************************************
* Name:             cc_afe_powerdown()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      power down the afe
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe_powerdown(void)
{
    
    if(NULL != AfeDrvInst_pst->cc_afe_drv_powerdown)
    {
        AfeDrvInst_pst->cc_afe_drv_powerdown();
    }
}

/******************************************
* Name:             cc_afe_power_saving_mode_enable()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      enable power saving mode if possible
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe_power_saving_mode_enable(void)
{
    if(NULL != AfeDrvInst_pst->cc_afe_drv_power_saving_mode_enable)
    {
        AfeDrvInst_pst->cc_afe_drv_power_saving_mode_enable();
    }
}

/******************************************
* Name:             cc_afe_power_saving_mode_disable()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      enable power saving mode if possible
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe_power_saving_mode_disable(void)
{
    if(NULL != AfeDrvInst_pst->cc_afe_drv_power_saving_mode_disable)
    {
        AfeDrvInst_pst->cc_afe_drv_power_saving_mode_disable();
    }
}


/******************************************
* Name:             cc_afe_get_diag_status()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      enable power saving mode if possible
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
unsigned int cc_afe_get_diag_status(void)
{
    if(NULL != AfeDrvInst_pst->cc_afe_drv_get_diag_status)
    {
        return(AfeDrvInst_pst->cc_afe_drv_get_diag_status());
    }
    else
    {
        return(0xffffffff);
    }
}


/******************************************
* Name:             cc_afe_led_curr_set()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      set LED current
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe_led_curr_set(unsigned char ui8LED1Cur, unsigned char ui8LED2Cur)
{
    if(NULL != AfeDrvInst_pst->cc_afe_drv_led_curr_set)
    {
        AfeDrvInst_pst->cc_afe_drv_led_curr_set(ui8LED1Cur,ui8LED2Cur);
    }
}

/******************************************
* Name:             cc_afe_cf_set()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      set cf
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe_cf_set(unsigned int ui32CfVal)
{
    if(NULL != AfeDrvInst_pst->cc_afe_drv_cf_set)
    {
        AfeDrvInst_pst->cc_afe_drv_cf_set(ui32CfVal);
    }
}


/******************************************
* Name:             cc_afe_rf_set()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      set rf
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe_rf_set(unsigned int ui32RfVal)
{
    if(NULL != AfeDrvInst_pst->cc_afe_drv_rf_set)
    {
        AfeDrvInst_pst->cc_afe_drv_rf_set(ui32RfVal);
    }
}

/******************************************
* Name:             cc_afe_Dac_set()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      set dac
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe_Dac_set(unsigned int ui32DacVal)
{
    if(NULL != AfeDrvInst_pst->cc_afe_drv_Dac_set)
    {
        AfeDrvInst_pst->cc_afe_drv_Dac_set(ui32DacVal);
    }
}

/******************************************
* Name:             cc_afe_dac_enable()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      set dac
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe_dac_enable(void)
{
    if(NULL != AfeDrvInst_pst->cc_afe_drv_dac_enable)
    {
        AfeDrvInst_pst->cc_afe_drv_dac_enable();
    }
}

/******************************************
* Name:             cc_afe_dac_disable()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      set dac
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe_dac_disable(void)
{
    if(NULL != AfeDrvInst_pst->cc_afe_drv_dac_disable)
    {
        AfeDrvInst_pst->cc_afe_drv_dac_disable();
    }
}




/******************************************
* Name:             cc_afe_second_gain_set()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      set second gain control and gain
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe_second_gain_set(unsigned int ui32SecGain)
{
    if(NULL != AfeDrvInst_pst->cc_afe_drv_second_gain_set)
    {
        AfeDrvInst_pst->cc_afe_drv_second_gain_set(ui32SecGain);
    }
}
/******************************************
* Name:             cc_afe_dump_reg()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      dump all registers of afe
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe_dump_reg(void)
{
    if(NULL != AfeDrvInst_pst->cc_afe_drv_dump_reg)
    {
        AfeDrvInst_pst->cc_afe_drv_dump_reg();
    }
}
/******************************************
* Name:             cc_afe_sample_reg_read()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      dump all registers of afe
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe_sample_reg_read(signed int * pi32SampleData)
{
    if(NULL != AfeDrvInst_pst->cc_afe_drv_sample_reg_read)
    {
        AfeDrvInst_pst->cc_afe_drv_sample_reg_read(pi32SampleData);
    }
}


//end file
