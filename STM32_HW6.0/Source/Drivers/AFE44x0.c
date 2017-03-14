//******************************************************************************
//  Claude ZHU
//  Hardware Team
//  (C) iCareTech Inc., 2013
//  All Rights Reserved.
//  Built with Keil MDK
//
//-------------------------------------------------------------------------------
//--------------------------------------------------------------------------------

#include "AFE44x0.h"
#include "stdio.h"
/**
* AFE4403 instance pointer 
*/
static Afe4403InstTypedef *     Afe4403Inst_pst;

static void cc_afe4403_set_default_hw_mode(void);

/**
* @brief user define function here 
*/


/******************************************
* Name:                 Afe4403_Inst_Init
* Author:               yun.zhu@ywkang.com
* Date:                 2016/3/15
* Description:          init the afe 4403 instance
* Param[in]             
*    1. Afe4403InstTypedef
* Param[out]: NONE
*    1. 
* Return: AFE4403_FUNC_RETURN_TYPE
******************************************/
AFE4403_FUNC_RETURN_TYPE Afe4403_Inst_Init(Afe4403InstTypedef * Afe4403InstInit_pst)
{
    /*  Check the input parameters      */
    if(NULL == Afe4403InstInit_pst)
    {
        return(AFE4403_RET_VAL_PARAM_ERROR);
    }
    
    /*   init the pointer               */
    Afe4403Inst_pst = Afe4403InstInit_pst;
    
    Afe4403Inst_pst->afe4403_bus_rd         = Afe4403InstInit_pst->afe4403_bus_rd;
    Afe4403Inst_pst->afe4403_bus_wr         = Afe4403InstInit_pst->afe4403_bus_wr;
    Afe4403Inst_pst->sys_delay_ms           = Afe4403InstInit_pst->sys_delay_ms;
    Afe4403Inst_pst->sys_int_disable        = Afe4403InstInit_pst->sys_int_disable;
    Afe4403Inst_pst->sys_int_enable         = Afe4403InstInit_pst->sys_int_enable;
    
    return(AFE4403_RET_VAL_SUCCESS);
}


/******************************************
* Name:             afe4403_timing_init_func
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      init the afe4403's timing to desired status
* Param[in] 
*   1. Afe4403TimingTypedef
* Param[out]:
*    1. 
* Return:
******************************************/
AFE4403_FUNC_RETURN_TYPE afe4403_timing_init_func(Afe4403TimingTypedef * Afe4403Timing_pst)
{
    /*  Check the parameters    */
    if(
        NULL != Afe4403Timing_pst
    &&  (0 == Afe4403Timing_pst->m_Afe4403LedPulseWidth_ms || 0 == Afe4403Timing_pst->m_Afe4403SamplePeriod_cnt)
    )
    {
        return(AFE4403_RET_VAL_PARAM_ERROR);
    }
    
    /*  Init the timing     */
    afe4403_reg_write(AFE44x0_REG_PRPCOUNT, AFE44x0_PRPCOUNT_100HZ & AFE44x0_PRPCOUNT_PRPCOUNT);
    Afe4403Inst_pst->afe4403_bus_wr(AFE44x0_REG_LED2STC,        AFE44x0_LED2STC_LED2STC          &    AFE44x0_LED2STC_100HZ);           //Sample LED2 start count
    Afe4403Inst_pst->afe4403_bus_wr(AFE44x0_REG_LED2ENDC,       AFE44x0_LED2ENDC_LED2ENDC        &    AFE44x0_LED2ENDC_100HZ);           //Sample LED2 end count
    Afe4403Inst_pst->afe4403_bus_wr(AFE44x0_REG_LED2LEDSTC,     AFE44x0_LED2LEDSTC_LED2LEDSTC    &    AFE44x0_LED2LEDSTC_100HZ);           //LED2 start count
    Afe4403Inst_pst->afe4403_bus_wr(AFE44x0_REG_LED2LEDENDC,    AFE44x0_LED2LEDENDC_LED2LEDENDC  &    AFE44x0_LED2LEDENDC_100HZ);           //LED2 end count
    Afe4403Inst_pst->afe4403_bus_wr(AFE44x0_REG_ALED2STC,       AFE44x0_ALED2STC_ALED2STC        &    AFE44x0_ALED2STC_100HZ);           //Sample Ambient LED2 start count
    Afe4403Inst_pst->afe4403_bus_wr(AFE44x0_REG_ALED2ENDC,      AFE44x0_ALED2ENDC_ALED2ENDC      &    AFE44x0_ALED2ENDC_100HZ);           //Sample Ambient LED2 end count
    Afe4403Inst_pst->afe4403_bus_wr(AFE44x0_REG_LED1STC,        AFE44x0_LED1STC_LED1STC          &    AFE44x0_LED1STC_100HZ);           //Sample LED1 start count
    Afe4403Inst_pst->afe4403_bus_wr(AFE44x0_REG_LED1ENDC,       AFE44x0_LED1ENDC_LED1ENDC        &    AFE44x0_LED1ENDC_100HZ);           //Sample LED1 end count
    Afe4403Inst_pst->afe4403_bus_wr(AFE44x0_REG_LED1LEDSTC,     AFE44x0_LED1LEDSTC_LED1LEDSTC    &    AFE44x0_LED1LEDSTC_100HZ);           //LED1 start count
    Afe4403Inst_pst->afe4403_bus_wr(AFE44x0_REG_LED1LEDENDC,    AFE44x0_LED1LEDENDC_LED1LEDENDC  &    AFE44x0_LED1LEDENDC_100HZ);           //LED1 end count
    Afe4403Inst_pst->afe4403_bus_wr(AFE44x0_REG_ALED1STC,       AFE44x0_ALED1STC_ALED1STC        &    AFE44x0_ALED1STC_100HZ);           //Sample Ambient LED1 start count
    Afe4403Inst_pst->afe4403_bus_wr(AFE44x0_REG_ALED1ENDC,      AFE44x0_ALED1ENDC_ALED1ENDC      &    AFE44x0_ALED1ENDC_100HZ);           //Sample Ambient LED1 end count
    Afe4403Inst_pst->afe4403_bus_wr(AFE44x0_REG_LED2CONVST,     AFE44x0_LED2CONVST_LED2CONVST    &    AFE44x0_LED2CONVST_100HZ);           //LED2 convert start count
    Afe4403Inst_pst->afe4403_bus_wr(AFE44x0_REG_LED2CONVEND,    AFE44x0_LED2CONVEND_LED2CONVEND  &    AFE44x0_LED2CONVEND_100HZ);           //LED2 convert end count
    Afe4403Inst_pst->afe4403_bus_wr(AFE44x0_REG_ALED2CONVST,    AFE44x0_ALED2CONVST_ALED2CONVST  &    AFE44x0_ALED2CONVST_100HZ);           //LED2 ambient convert start count
    Afe4403Inst_pst->afe4403_bus_wr(AFE44x0_REG_ALED2CONVEND,   AFE44x0_ALED2CONVEND_ALED2CONVEND&    AFE44x0_ALED2CONVEND_100HZ);           //LED2 ambient convert end count
    Afe4403Inst_pst->afe4403_bus_wr(AFE44x0_REG_LED1CONVST,     AFE44x0_LED1CONVST_LED1CONVST    &    AFE44x0_LED1CONVST_100HZ);           //LED1 convert start count
    Afe4403Inst_pst->afe4403_bus_wr(AFE44x0_REG_LED1CONVEND,    AFE44x0_LED1CONVEND_LED1CONVEND  &    AFE44x0_LED1CONVEND_100HZ);           //LED1 convert end count
    Afe4403Inst_pst->afe4403_bus_wr(AFE44x0_REG_ALED1CONVST,    AFE44x0_ALED1CONVST_ALED1CONVST  &    AFE44x0_ALED1CONVST_100HZ);           //LED1 ambient convert start count
    Afe4403Inst_pst->afe4403_bus_wr(AFE44x0_REG_ALED1CONVEND,   AFE44x0_ALED1CONVEND_ALED1CONVEND&    AFE44x0_ALED1CONVEND_100HZ);           //LED1 ambient convert end count
    Afe4403Inst_pst->afe4403_bus_wr(AFE44x0_REG_ADCRSTSTCT0,    AFE44x0_ADCRSTSTCT0_ADCRSTSTCT0  &    AFE44x0_ADCRSTSTCT0_100HZ);           //ADC RESET 0 start count
    Afe4403Inst_pst->afe4403_bus_wr(AFE44x0_REG_ADCRSTENDCT0,   AFE44x0_ADCRSTENDCT0_ADCRSTENDCT0&    AFE44x0_ADCRSTENDCT0_100HZ);           //ADC RESET 0 end count
    Afe4403Inst_pst->afe4403_bus_wr(AFE44x0_REG_ADCRSTSTCT1,    AFE44x0_ADCRSTSTCT1_ADCRSTSTCT1  &    AFE44x0_ADCRSTSTCT1_100HZ);           //ADC RESET 1 start count
    Afe4403Inst_pst->afe4403_bus_wr(AFE44x0_REG_ADCRSTENDCT1,   AFE44x0_ADCRSTENDCT1_ADCRSTENDCT1&    AFE44x0_ADCRSTENDCT1_100HZ);           //ADC RESET 1 end count
    Afe4403Inst_pst->afe4403_bus_wr(AFE44x0_REG_ADCRSTSTCT2,    AFE44x0_ADCRSTSTCT2_ADCRSTSTCT2  &    AFE44x0_ADCRSTSTCT2_100HZ);           //ADC RESET 2 start count
    Afe4403Inst_pst->afe4403_bus_wr(AFE44x0_REG_ADCRSTENDCT2,   AFE44x0_ADCRSTENDCT2_ADCRSTENDCT2&    AFE44x0_ADCRSTENDCT2_100HZ);           //ADC RESET 2 end count
    Afe4403Inst_pst->afe4403_bus_wr(AFE44x0_REG_ADCRSTSTCT3,    AFE44x0_ADCRSTSTCT3_ADCRSTSTCT3  &    AFE44x0_ADCRSTSTCT3_100HZ);           //ADC RESET 3 start count
    Afe4403Inst_pst->afe4403_bus_wr(AFE44x0_REG_ADCRSTENDCT3,   AFE44x0_ADCRSTENDCT3_ADCRSTENDCT3&    AFE44x0_ADCRSTENDCT3_100HZ);           //ADC RESET 3 end count
    return(AFE4403_RET_VAL_SUCCESS);
}


/******************************************
* Name:             afe4403_reg_write
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      write afe4403 reg
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:
******************************************/
void afe4403_reg_write (unsigned char ui8RegAddr, unsigned int ui32DataIn)
{
    Afe4403Inst_pst->AFE4403_BUS_WR_FUNC(AFE44x0_REG_CONTROL0, 0x0);
	Afe4403Inst_pst->AFE4403_BUS_WR_FUNC(ui8RegAddr, ui32DataIn);
}


/******************************************
* Name:             afe4403_reg_read
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      write afe4403 reg
* Param[in]
*    1.     ui8RegAddr
* Param[out]:
*    1.     uint32
* Return:
******************************************/
unsigned int afe4403_reg_read(unsigned char ui8RegAddr)
{
    unsigned int retVal;
    
    Afe4403Inst_pst->AFE4403_BUS_WR_FUNC(AFE44x0_REG_CONTROL0, 0x1);
	Afe4403Inst_pst->AFE4403_BUS_RD_FUNC(ui8RegAddr,&retVal);
    
	return(retVal);
}


/******************************************
* Name:             afe4403_rd_sample_reg
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      read sample data register
* Param[in]
*    1.     ui8RegAddr
* Param[out]:
*    1.     uint32
* Return: AFE4403_FUNC_RETURN_TYPE
******************************************/

void afe4403_rd_sample_reg(signed int * pi32DatBuf)
{
    unsigned char Regs_i = 0;
	unsigned char i = 0;
	unsigned int 	tmpData = 0;
    
	afe4403_reg_write(AFE44x0_REG_CONTROL0,0x1);		// Enable SPI read
    
	for ( i=0, Regs_i = AFE44x0_REG_LED2VAL; Regs_i <= AFE44x0_REG_LED1_SUB_ALED1VAL; Regs_i++, i++)
	{
        Afe4403Inst_pst->AFE4403_BUS_RD_FUNC(Regs_i,&tmpData);
        //pi32DatBuf[i] = 0x00ffffff & tmpData;
		if((tmpData & 0x00200000)==0x00200000)
		{
			pi32DatBuf[i] = 0xFFC00000 | tmpData;
		}
		else
		{
			pi32DatBuf[i] = 0x1FFFFF & tmpData;
		}
	}
}

/******************************************
* Name:             afe4403_rd_sample_reg
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      read sample data register
* Param[in]
*    1.     ui8RegAddr
* Param[out]:
*    1.     uint32
* Return: AFE4403_FUNC_RETURN_TYPE
******************************************/

AFE4403_FUNC_RETURN_TYPE afe4403_wr_reg_mask(unsigned char ui8RegAddr, unsigned int ui32Mask, unsigned int ui32DatIn)
{
    /*  Check the input parameters  */
    
    unsigned int ui32RetVal;
    
    Afe4403Inst_pst->sys_int_disable();
    
    /*read out*/
    Afe4403Inst_pst->AFE4403_BUS_WR_FUNC(AFE44x0_REG_CONTROL0, 0x1);
	Afe4403Inst_pst->AFE4403_BUS_RD_FUNC(ui8RegAddr,&ui32RetVal);
	/*write in*/
	Afe4403Inst_pst->AFE4403_BUS_WR_FUNC(AFE44x0_REG_CONTROL0, 0x0);
	Afe4403Inst_pst->AFE4403_BUS_WR_FUNC(ui8RegAddr, ((ui32RetVal&(~ui32Mask))|(ui32DatIn&ui32Mask)));
    
    Afe4403Inst_pst->sys_int_enable();
    
    return(AFE4403_RET_VAL_SUCCESS);
}	

/******************************************
* Name:             afe4403_diag_enable()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      init afe to diag status
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void afe4403_diag_enable(void)
{
    afe4403_reg_write(AFE44x0_REG_CONTROL0, AFE44x0_CONTROL0_DIAG_EN);
}

/******************************************
* Name:             afe4403_set_average_num()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      init afe to diag status
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void afe4403_set_average_num(unsigned int ui32AveNum)
{
    afe4403_wr_reg_mask(AFE44x0_REG_CONTROL1, AFE44x0_CONTROL1_NUMAVG, ui32AveNum);
}

/******************************************
* Name:             cc_afe4403_enable_running()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      enable the afe to run
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe4403_enable_running(void)
{
    afe4403_wr_reg_mask(AFE44x0_REG_CONTROL1, AFE44x0_CONTROL1_TIMEREN, AFE44x0_TIMEREN_ENABLE);
}


/******************************************
* Name:             cc_afe4403_power_saving_mode_enable()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      enable power saving mode if possible
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe4403_power_saving_mode_enable(void)
{
    /*Set all 4 dynamic bits to 1*/
    unsigned int afeWrMask = 0;
    unsigned int afeWrData = 0;
    
    afeWrMask = AFE44x0_CONTROL2_DYNAMIC1 | AFE44x0_CONTROL2_DYNAMIC2 | AFE44x0_CONTROL2_DYNAMIC3 |\
                            AFE44x0_CONTROL2_DYNAMIC4;
    afeWrData = AFE44x0_DYNAMIC1_ENABLE | AFE44x0_DYNAMIC2_ENABLE | AFE44x0_DYNAMIC3_ENABLE |\
                            AFE44x0_DYNAMIC4_ENABLE;
                            
    afe4403_wr_reg_mask(AFE44x0_REG_CONTROL2, afeWrMask, afeWrData);
    /*Set timming counter for power down period*/
    //Sample Rate = 100Hz
    //T = 10ms = 10000us
    //Working Time = 5*0.25*1000 = 1.25ms
    //T1 = 200us
    //T2 = 300us
    //Powerdown counter start = 1.45ms/0.25us = 5800
    //Powerdown counter end = (10-0.3)ms/0.25 = 38800
    //afe4403_wr_reg_mask(AFE44x0_REG_PDNCYCLESTC, AFE44x0_PDNCYCLESTC_PDNCYCLESTC, 5800);
    //afe4403_wr_reg_mask(AFE44x0_REG_PDNCYCLEENDC, AFE44x0_PDNCYCLEENDC_PDNCYCLEENDC, 38800);
    afe4403_wr_reg_mask(AFE44x0_REG_PDNCYCLESTC, AFE44x0_PDNCYCLESTC_PDNCYCLESTC, AFE44x0_PDNCYCLESTC_NUM);
    afe4403_wr_reg_mask(AFE44x0_REG_PDNCYCLEENDC, AFE44x0_PDNCYCLEENDC_PDNCYCLEENDC, AFE44x0_PDNCYCLEENDC_NUM);
}

/******************************************
* Name:             cc_afe4403_power_saving_mode_disable()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      enable power saving mode if possible
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe4403_power_saving_mode_disable(void)
{
    /*Set all 4 dynamic bits to 1*/
    unsigned int afeWrMask = 0;
    unsigned int afeWrData = 0;
    afeWrMask = AFE44x0_CONTROL2_DYNAMIC1 | AFE44x0_CONTROL2_DYNAMIC2 | AFE44x0_CONTROL2_DYNAMIC3 |\
                            AFE44x0_CONTROL2_DYNAMIC4;
    afeWrData = AFE44x0_DYNAMIC1_DISABLE | AFE44x0_DYNAMIC2_DISABLE | AFE44x0_DYNAMIC3_DISABLE |\
                            AFE44x0_DYNAMIC4_DISABLE;
                            
    afe4403_wr_reg_mask(AFE44x0_REG_CONTROL2, afeWrMask, afeWrData);		

    /*clear counter register */
    afeWrMask = AFE44x0_PDNCYCLESTC_PDNCYCLESTC;
    afeWrData = 0;
    afe4403_wr_reg_mask(AFE44x0_REG_PDNCYCLESTC, afeWrMask, afeWrData);		

    afeWrMask = AFE44x0_PDNCYCLEENDC_PDNCYCLEENDC;
    afeWrData = 0;
    afe4403_wr_reg_mask(AFE44x0_REG_PDNCYCLEENDC, afeWrMask, afeWrData);	
}
/******************************************
* Name:             cc_afe4403_get_diag_status()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      enable power saving mode if possible
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
unsigned int cc_afe4403_get_diag_status(void)
{
    return(afe4403_reg_read(AFE44x0_REG_DIAG));
}
/******************************************
* Name:             cc_afe4403_led_curr_set()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      turn off all led
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe4403_led_curr_set(unsigned char ui8LED1Cur, unsigned char ui8LED2Cur)
{
    unsigned int    afeWrMask = 0;
    unsigned int    afeWrData = 0;
    
	afeWrMask = AFE44x0_LEDCNTRL_LEDRANGE | AFE44x0_LEDCNTRL_LED1 | AFE44x0_LEDCNTRL_LED2;
	afeWrData = 0 | ((ui8LED1Cur)<<8) | (ui8LED2Cur);

	afe4403_wr_reg_mask(AFE44x0_REG_LEDCNTRL, afeWrMask, afeWrData);
}

/******************************************
* Name:             cc_afe4403_cf_set()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      set LED current
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe4403_cf_set(unsigned int ui32CfVal)
{
	afe4403_wr_reg_mask(AFE44x0_REG_TIA_AMB_GAIN, AFE44x0_TIA_AMB_GAIN_CF_LED, (ui32CfVal));
}

/******************************************
* Name:             cc_afe4403_rf_set()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      set RF
* Param[in]
*    1. ui8RfVal
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe4403_rf_set(unsigned int ui32RfVal)
{
	afe4403_wr_reg_mask(AFE44x0_REG_TIA_AMB_GAIN, AFE44x0_TIA_AMB_GAIN_RF_LED, ui32RfVal);
}
/******************************************
* Name:             cc_afe4403_Dac_set()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      set dac
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe4403_Dac_set(unsigned int ui32DacVal)
{
	afe4403_wr_reg_mask(AFE44x0_REG_TIA_AMB_GAIN, AFE44x0_TIA_AMB_GAIN_AMBDAC, ui32DacVal);
}

/******************************************
* Name:             cc_afe4403_disable_cur_cancel()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      set dac
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe4403_disable_cur_cancel(void)
{
	afe4403_wr_reg_mask(AFE44x0_REG_TIA_AMB_GAIN, AFE44x0_TIA_AMB_GAIN_STAGE2EN, AFE44x0_STAGE2EN_BYPASS);
}

/******************************************
* Name:             cc_afe4403_enable_cur_cancel()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      set dac
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe4403_enable_cur_cancel(void)
{
	afe4403_wr_reg_mask(AFE44x0_REG_TIA_AMB_GAIN, AFE44x0_TIA_AMB_GAIN_STAGE2EN, AFE44x0_STAGE2EN_ENABLE);
}

/******************************************
* Name:             cc_afe4403_sec_gain_set()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      set second gain
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe4403_sec_gain_set(unsigned int ui32SecGainVal)
{
	afe4403_wr_reg_mask(AFE44x0_REG_TIA_AMB_GAIN, AFE44x0_TIA_AMB_GAIN_STG2GAIN, ui32SecGainVal);
}

/******************************************
* Name:             cc_afe4403_dump_reg()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      dump all registers of afe
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe4403_dump_reg(void)
{
    unsigned int afeWrMask;
    unsigned int afeWrData;
    for(afeWrMask = 0; afeWrMask < 0x32; afeWrMask++)
    {
        afeWrData = afe4403_reg_read(afeWrMask);
        printf("AFE4403, reg %x = %x \r\n",afeWrMask,afeWrData);
    }
}

/******************************************
* Name:             cc_afe4403_poweron_init()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      init afe from power down status
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_afe4403_poweron_init(void)
{

	afe4403_wr_reg_mask(AFE44x0_REG_LEDCNTRL, AFE44x0_LEDCNTRL_LED1|AFE44x0_LEDCNTRL_LED2, 0x00000000);
    
	/*  Set AFE timing      */
	afe4403_timing_init_func(NULL);	

    /*  Set to default hw mode  */
    cc_afe4403_set_default_hw_mode();
}

/******************************************
* Name:             cc_afe_set_default_hw_mode()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      the afe to default hw mode, depending on different chipset
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
static void cc_afe4403_set_default_hw_mode(void)
{
    unsigned int    afeWrMask = 0;
    unsigned int    afeWrData = 0;
    
    /*Set LED Range to 50mA*/
    afe4403_wr_reg_mask(AFE44x0_REG_LEDCNTRL, AFE44x0_LEDCNTRL_LEDRANGE, 0x00000000);

    /*Disable Seperate RX channel setting*/
    afe4403_wr_reg_mask(AFE44x0_REG_TIAGAIN, AFE44x0_TIAGAIN_ENSEPGAN, 0x00000000);

    /*Control register 3*/
    afeWrMask = AFE44x0_CONTROL3_TX3_MODE | AFE44x0_CONTROL3_CLKOUT_TRI | AFE44x0_CONTROL3_SOMI_TRI |\
                        AFE44x0_CONTROL3_CLKDIV;
    afeWrData = AFE44x0_TX3_MODE_DISABLE | AFE44x0_CLKOUT_TRI_ENABLE | AFE44x0_SOMI_TRI_DISABLE |\
                        AFE44x0_CLKDIV_4;
    /*Disable TX3*/
    afe4403_wr_reg_mask(AFE44x0_REG_CONTROL3, afeWrMask, afeWrData);
    
    afeWrMask = 0;
    afeWrData = 0;
    
    afeWrMask = AFE44x0_CONTROL2_TXBRGMOD | AFE44x0_CONTROL2_DIGOUT_TRISTATE | AFE44x0_CONTROL2_XTALDIS |\
                            AFE44x0_CONTROL2_PDN_TX | AFE44x0_CONTROL2_PDN_RX | AFE44x0_CONTROL2_PDN_AFE |\
                            AFE44x0_CONTROL2_TX_REF | AFE44x0_CONTROL2_EN_SLOW_DIAG;
    afeWrData = AFE44x0_TXBRGMOD_Push_Pull | AFE44x0_DIGOUT_TRISTATE_Normal | AFE44x0_XTALDIS_Crystal |\
                            AFE44x0_TX_POWER_ON | AFE44x0_RX_POWER_ON | AFE44x0_AFE_POWER_ON | \
                            AFE44x0_AFE_TX_REF_05V | AFE44x0_FAST_DIAG_MODE;	
         
    afe4403_wr_reg_mask(AFE44x0_REG_CONTROL2, afeWrMask, afeWrData);
}




// End of file
