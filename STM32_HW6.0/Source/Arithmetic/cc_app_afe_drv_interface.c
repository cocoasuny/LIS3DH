/*
* Copyright(c) 2015, CloudCare Healthcare Co.,Ltd
* All rights reserved
* File Name:                cc_app_afe_drv_interface.c
* Descprition:              interface between AFE driver, MCU and Algorithm Moduel
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
#include "Timer.h"
#include "cc_afe_translate_layer.h"
#include "cc_app_afe_drv_interface.h"
#include "AFE44x0.h"

static EXTI_InitTypeDef m_AfeDrdyEXTI_st;
static Afe4403InstTypedef m_Afe4403InstInit_st;
static AfeDrvInst_Typedef m_AfeDrvInst_st;

static AFE4403_FUNC_RETURN_TYPE AFE44xx_SPI_Read(unsigned char ui8RegAddr,unsigned int * pui32Dat);
static AFE4403_FUNC_RETURN_TYPE AFE44xx_SPI_Write (unsigned char reg_address, unsigned int data);
static void cc_sys_afe_gpio_init(void);
static void cc_sys_afe_gpio_deinit(void);
static void cc_sys_afe_bus_init(void);
static void cc_sys_afe_powerdown(void);
static void cc_sys_afe_init_data_int (void);
static void cc_sys_afe_enable_data_int (void);
static void cc_sys_afe_disable_data_int (void);
static void cc_sys_afe_ctrl_init(void);
static void cc_sys_afe_ctrl_deinit(void);
static void cc_sys_afe_poweron_int (void);

/******************************************
* Name:             cc_app_afe_drv_interface_init()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      Init the function pointer of afe
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
void cc_app_afe_drv_interface_init(void)
{
    /*  Init the interface functions    */
    m_Afe4403InstInit_st.afe4403_bus_rd = AFE44xx_SPI_Read;
    m_Afe4403InstInit_st.afe4403_bus_wr = AFE44xx_SPI_Write;
    m_Afe4403InstInit_st.sys_delay_ms = Delay_ms;
    m_Afe4403InstInit_st.sys_int_disable = __disable_irq;
    m_Afe4403InstInit_st.sys_int_enable = __enable_irq;
    /*  Init the AFE4403 module         */
    Afe4403_Inst_Init(&m_Afe4403InstInit_st);
    
    
    /*  Init the instance of AFE translate layer    */
    m_AfeDrvInst_st.cc_afe_drv_cf_set = cc_afe4403_cf_set;
    m_AfeDrvInst_st.cc_afe_drv_dac_disable = cc_afe4403_disable_cur_cancel;
    m_AfeDrvInst_st.cc_afe_drv_dac_enable = cc_afe4403_enable_cur_cancel;
    m_AfeDrvInst_st.cc_afe_drv_Dac_set = cc_afe4403_Dac_set;
    m_AfeDrvInst_st.cc_afe_drv_diag_enable = afe4403_diag_enable;
    m_AfeDrvInst_st.cc_afe_drv_disable_data_int = cc_sys_afe_disable_data_int;
    m_AfeDrvInst_st.cc_afe_drv_dump_reg = cc_afe4403_dump_reg;
    m_AfeDrvInst_st.cc_afe_drv_enable_data_int = cc_sys_afe_enable_data_int;
    m_AfeDrvInst_st.cc_afe_drv_enable_running = cc_afe4403_enable_running;
    m_AfeDrvInst_st.cc_afe_drv_get_diag_status = cc_afe4403_get_diag_status;
    m_AfeDrvInst_st.cc_afe_drv_led_curr_set = cc_afe4403_led_curr_set;
    m_AfeDrvInst_st.cc_afe_drv_powerdown = cc_sys_afe_powerdown;
    m_AfeDrvInst_st.cc_afe_drv_power_init = cc_sys_afe_poweron_int;
    m_AfeDrvInst_st.cc_afe_drv_power_saving_mode_disable = cc_afe4403_power_saving_mode_disable;
    m_AfeDrvInst_st.cc_afe_drv_power_saving_mode_enable = cc_afe4403_power_saving_mode_enable;
    m_AfeDrvInst_st.cc_afe_drv_rf_set = cc_afe4403_rf_set;
    m_AfeDrvInst_st.cc_afe_drv_second_gain_set = cc_afe4403_sec_gain_set;
    m_AfeDrvInst_st.cc_afe_drv_set_average_num = afe4403_set_average_num;
    m_AfeDrvInst_st.cc_afe_drv_sample_reg_read = afe4403_rd_sample_reg;
    cc_afe_translate_layer_init(&m_AfeDrvInst_st);
}

/******************************************
* Name:             cc_sys_afe_poweron_int()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      Init data interrupt from AFE
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
static void cc_sys_afe_poweron_int (void)
{
	//volatile unsigned short Init_i, j;
	cc_sys_afe_ctrl_init();
	/*reset and powerdown AFE chipset*/
	AFE44x0_GPIO_PDNZ_CLR;
	AFE44x0_GPIO_RESETZ_CLR;
	/*Delay*/
	Delay_ms(20);
	/*clear the reset & power down*/
	AFE44x0_GPIO_PDNZ_SET;					//Disable power down
	AFE44x0_GPIO_RESETZ_SET;		        //Disable reset
	/* Init DRDY Interrupt*/
	cc_sys_afe_init_data_int();

    cc_afe4403_poweron_init();
}


/******************************************
* Name:             cc_sys_afe_init_data_int()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      Init data interrupt from AFE
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
static void cc_sys_afe_init_data_int (void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    SYSCFG_EXTILineConfig(AFE_ADC_DRDY_EXTI_PORT, AFE_ADC_DRDY_EXTI_PIN);
                                                            //EXTI line mapping
    m_AfeDrdyEXTI_st.EXTI_Line      = AFE_ADC_DRDY_EXTI_LINE;         
                                                            //EXTI Line
    m_AfeDrdyEXTI_st.EXTI_Mode      = EXTI_Mode_Interrupt;
                                                            //EXTI Mode, Interrupt
    m_AfeDrdyEXTI_st.EXTI_Trigger   = EXTI_Trigger_Rising;
                                                            //EXTI trigger, Falling edge
    m_AfeDrdyEXTI_st.EXTI_LineCmd   = DISABLE;            //Default state is "Disable"

    EXTI_Init(&m_AfeDrdyEXTI_st);           //EXTI Init
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
static void cc_sys_afe_enable_data_int (void)
{
    m_AfeDrdyEXTI_st.EXTI_LineCmd   = ENABLE;         // Enable interrupt
    EXTI_Init(&m_AfeDrdyEXTI_st);
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
static void cc_sys_afe_disable_data_int (void)
{
	EXTI_ClearITPendingBit(AFE_ADC_DRDY_EXTI_LINE);
	EXTI_ClearFlag(AFE_ADC_DRDY_EXTI_LINE);
    m_AfeDrdyEXTI_st.EXTI_LineCmd   = DISABLE;        // Disable interrupt
    EXTI_Init(&m_AfeDrdyEXTI_st);
}



/******************************************
* Name:             cc_afe_gpio_init()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      init afe's gpio
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
static void cc_sys_afe_gpio_init(void)
{
  /*
  GPIO Set(MCU side)
  AFE_RESETZ     ----GPIOB---Pin1  ----OUTPUT
  AFE_PDNZ       ----GPIOE---Pin11 ----OUTPUT
  AFE_PD_ALM     ----GPIOE---Pin9  ----INPUT
  AFE_LED_ALM    ----GPIOE---Pin10 ----INPUT
  AFE_DIAG_END   ----GPIOE---Pin7  ----INPUT
  */
	GPIO_InitTypeDef AFE_GPIO_TYPE_CFG;
    //Enable AFE44x0 GPIO CLK


	RCC_AHBPeriphClockCmd(AFE_RESETZ_GPIO_CLK | AFE_PDNZ_GPIO_CLK | AFE_ADC_DRDY_GPIO_CLK ,ENABLE);
	
    //GPIO Configuration for AFE_RESETZ
    AFE_GPIO_TYPE_CFG.GPIO_Pin       = AFE_RESETZ;         //Pin definition
    AFE_GPIO_TYPE_CFG.GPIO_Mode      = GPIO_Mode_OUT;      //Set as Output mode
    AFE_GPIO_TYPE_CFG.GPIO_Speed     = GPIO_Speed_400KHz;   //Verylow speed
    AFE_GPIO_TYPE_CFG.GPIO_OType     = GPIO_OType_PP;      //Output type
    AFE_GPIO_TYPE_CFG.GPIO_PuPd      = GPIO_PuPd_DOWN;       //Enable Internal Pull-down

    GPIO_Init(AFE_RESETZ_PORT,&AFE_GPIO_TYPE_CFG);           //GPIO AFE_RESETZ init

    //GPIO Configuration for AFE_PDNZ
    AFE_GPIO_TYPE_CFG.GPIO_Pin       = AFE_PDNZ;           //Pin definition
    AFE_GPIO_TYPE_CFG.GPIO_Mode      = GPIO_Mode_OUT;      //Set as Output mode
    AFE_GPIO_TYPE_CFG.GPIO_Speed     = GPIO_Speed_400KHz;   //Middle speed
    AFE_GPIO_TYPE_CFG.GPIO_OType     = GPIO_OType_PP;      //Output type
    AFE_GPIO_TYPE_CFG.GPIO_PuPd      = GPIO_PuPd_DOWN;       //Enable Internal Pull-down

    GPIO_Init(AFE_PDNZ_PORT,&AFE_GPIO_TYPE_CFG);             //GPIO AFE_PDNZ init

    //GPIO Configuration for AFE_DRDY
	AFE_GPIO_TYPE_CFG.GPIO_Pin       = AFE_ADC_DRDY;  		
    AFE_GPIO_TYPE_CFG.GPIO_Mode      = GPIO_Mode_IN;       //Set as Input mode
    AFE_GPIO_TYPE_CFG.GPIO_Speed     = GPIO_Speed_10MHz;   //Middle speed
    AFE_GPIO_TYPE_CFG.GPIO_PuPd      = GPIO_PuPd_UP;       //Enable Internal Pull-up
	GPIO_Init(AFE_ADC_DRDY_PORT,&AFE_GPIO_TYPE_CFG);                     //GPIO init
	
}

/******************************************
* Name:             cc_afe_gpio_deinit()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      deinit afe's gpio
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
static void cc_sys_afe_gpio_deinit(void)
{
	GPIO_InitTypeDef AFE_GPIO_TYPE_CFG;

    //GPIO Configuration for AFE_RESETZ
    AFE_GPIO_TYPE_CFG.GPIO_Pin       = AFE_RESETZ;         //Pin definition
    AFE_GPIO_TYPE_CFG.GPIO_Mode      = GPIO_Mode_AN;      //Set as Output mode
    AFE_GPIO_TYPE_CFG.GPIO_Speed     = GPIO_Speed_400KHz;   //Verylow speed
    AFE_GPIO_TYPE_CFG.GPIO_OType     = GPIO_OType_PP;      //Output type
    AFE_GPIO_TYPE_CFG.GPIO_PuPd      = GPIO_PuPd_NOPULL;       //Enable Internal Pull-down

    GPIO_Init(AFE_RESETZ_PORT,&AFE_GPIO_TYPE_CFG);           //GPIO AFE_RESETZ init

    //GPIO Configuration for AFE_PDNZ
    AFE_GPIO_TYPE_CFG.GPIO_Pin       = AFE_PDNZ;           //Pin definition
    AFE_GPIO_TYPE_CFG.GPIO_Mode      = GPIO_Mode_IN;      //Set as Output mode
    AFE_GPIO_TYPE_CFG.GPIO_Speed     = GPIO_Speed_400KHz;   //Middle speed
    AFE_GPIO_TYPE_CFG.GPIO_OType     = GPIO_OType_PP;      //Output type
    AFE_GPIO_TYPE_CFG.GPIO_PuPd      = GPIO_PuPd_DOWN;       //Enable Internal Pull-down

    GPIO_Init(AFE_PDNZ_PORT,&AFE_GPIO_TYPE_CFG);             //GPIO AFE_PDNZ init

    //GPIO Configuration for AFE_DRDY
	AFE_GPIO_TYPE_CFG.GPIO_Pin       = AFE_ADC_DRDY;  		
    AFE_GPIO_TYPE_CFG.GPIO_Mode      = GPIO_Mode_AN;       //Set as Input mode
    AFE_GPIO_TYPE_CFG.GPIO_Speed     = GPIO_Speed_400KHz;   //Middle speed
    AFE_GPIO_TYPE_CFG.GPIO_PuPd      = GPIO_PuPd_NOPULL;       //Enable Internal Pull-up
	GPIO_Init(AFE_ADC_DRDY_PORT,&AFE_GPIO_TYPE_CFG);                     //GPIO init
}


/******************************************
* Name:             cc_afe_bus_init()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      init afe's data bus
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
static void cc_sys_afe_bus_init(void)
{
	GPIO_InitTypeDef AFE_GPIO_TYPE_CFG;
    SPI_InitTypeDef AFE_SPI_TYPE_CFG;           //Define the init type
    
    // Enable the SPI peripheral
    RCC_APB2PeriphClockCmd(RCC_APBxPeriph_SPI_AFE44x0, ENABLE);
    
    // Enable AFE4400 SCK, MOSI, MISO and NSS GPIO clocks */
    RCC_AHBPeriphClockCmd(AFE_SPI_STE_GPIO_CLK | AFE_SPI_SCLK_GPIO_CLK | AFE_SPI_SOMI_GPIO_CLK |
                        AFE_SPI_SIMO_GPIO_CLK , ENABLE);
	
	
    // SPI1 Alternate function mapping
    //GPIO_PinAFConfig(AFE_SPI_PORT, AFE_SPI_STE_PINSOURCE, GPIO_AF_SPI1);
    GPIO_PinAFConfig(AFE_SPI_PORT, AFE_SPI_SCLK_PINSOURCE, GPIO_AF_SPI1);
    GPIO_PinAFConfig(AFE_SPI_PORT, AFE_SPI_SOMI_PINSOURCE, GPIO_AF_SPI1);
    GPIO_PinAFConfig(AFE_SPI_PORT, AFE_SPI_SIMO_PINSOURCE, GPIO_AF_SPI1);

    // Enable GPIO as alternal function
    //AFE_GPIO_TYPE_CFG.GPIO_Pin       = AFE_SPI_STE_PIN | AFE_SPI_SCLK_PIN | AFE_SPI_SIMO_PIN | AFE_SPI_SOMI_PIN;    
	AFE_GPIO_TYPE_CFG.GPIO_Pin       = AFE_SPI_SCLK_PIN | AFE_SPI_SIMO_PIN | AFE_SPI_SOMI_PIN;    
                                                            //Pin definition
    AFE_GPIO_TYPE_CFG.GPIO_Mode      = GPIO_Mode_AF;       //Set as AF mode
    AFE_GPIO_TYPE_CFG.GPIO_Speed     = GPIO_Speed_10MHz;   //Middle speed
    AFE_GPIO_TYPE_CFG.GPIO_OType     = GPIO_OType_PP;      //Output type
    AFE_GPIO_TYPE_CFG.GPIO_PuPd      = GPIO_PuPd_UP;       //Enable Internal Pull-up

    GPIO_Init(AFE_SPI_PORT,&AFE_GPIO_TYPE_CFG);           //GPIO SCLK SIMO SOMI init
	
    // Enable STE as output mode
    //AFE_GPIO_TYPE_CFG.GPIO_Pin       = AFE_SPI_STE_PIN | AFE_SPI_SCLK_PIN | AFE_SPI_SIMO_PIN | AFE_SPI_SOMI_PIN;    
	AFE_GPIO_TYPE_CFG.GPIO_Pin       = AFE_SPI_STE_PIN;    
                                                            //Pin definition
    AFE_GPIO_TYPE_CFG.GPIO_Mode      = GPIO_Mode_OUT;       //Set as output mode
    AFE_GPIO_TYPE_CFG.GPIO_Speed     = GPIO_Speed_10MHz;   //Middle speed
    AFE_GPIO_TYPE_CFG.GPIO_OType     = GPIO_OType_PP;      //Output type
    AFE_GPIO_TYPE_CFG.GPIO_PuPd      = GPIO_PuPd_UP;       //Enable Internal Pull-up

    GPIO_Init(AFE_SPI_PORT,&AFE_GPIO_TYPE_CFG);           //GPIO STE init
    
    
    /*Configure SPI1 for AFE4400 as following setup:
      (+) SPI 1 for AFE4400
      (+) Master Mode
      (+) Full Duplex Mode
      (+) 8 bit mode
      (+) POL = Low
      (+) CPHA = Low
      (+) Hardware NSS control
      (+) Baud Rate <= 15MHz

    */


    /* Initialize the SPI_Direction member */
    AFE_SPI_TYPE_CFG.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    /* initialize the SPI_Mode member */
    AFE_SPI_TYPE_CFG.SPI_Mode = SPI_Mode_Master;
    /* initialize the SPI_DataSize member */
    AFE_SPI_TYPE_CFG.SPI_DataSize = SPI_DataSize_8b;
    /* Initialize the SPI_CPOL member */
    AFE_SPI_TYPE_CFG.SPI_CPOL = SPI_CPOL_Low;
    /* Initialize the SPI_CPHA member */
    AFE_SPI_TYPE_CFG.SPI_CPHA = SPI_CPHA_1Edge;
    /* Initialize the SPI_NSS member */
    //AFE_SPI_TYPE_CFG.SPI_NSS = SPI_NSS_Hard;
	AFE_SPI_TYPE_CFG.SPI_NSS = SPI_NSS_Soft;
    /* Initialize the SPI_BaudRatePrescaler member */
    AFE_SPI_TYPE_CFG.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    /* Initialize the SPI_FirstBit member */
    AFE_SPI_TYPE_CFG.SPI_FirstBit = SPI_FirstBit_MSB;
    /* Initialize the SPI_CRCPolynomial member */
    AFE_SPI_TYPE_CFG.SPI_CRCPolynomial = 7;

    SPI_Init(AFE44x0_SPI, &AFE_SPI_TYPE_CFG);
	/* Set NSS Pin(STE) as output mode */
	//SPI_SSOutputCmd(AFE44x0_SPI, ENABLE);
	SPI_SSOutputCmd(AFE44x0_SPI, DISABLE);
    
    
    
	
}


/******************************************
* Name:             cc_afe_ctrl_init()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      init afe on the side of MCU
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
static void cc_sys_afe_ctrl_init(void)
{
    cc_sys_afe_gpio_init();										// Initializes AFE44xx's input control lines
    cc_sys_afe_bus_init();										// Initialize SPI Alternative function 
}


/******************************************
* Name:             cc_afe_ctrl_deinit()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      deinit afe on the side of MCU
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
static void cc_sys_afe_ctrl_deinit(void)
{
	GPIO_InitTypeDef AFE_GPIO_TYPE_CFG;
		
	/* Disable SPI function */
	SPI_SSOutputCmd(AFE44x0_SPI, DISABLE);
    // set GPIO as input
	AFE_GPIO_TYPE_CFG.GPIO_Pin       = AFE_SPI_STE_PIN | AFE_SPI_SCLK_PIN | AFE_SPI_SIMO_PIN | AFE_SPI_SOMI_PIN;    
                                                            //Pin definition
    AFE_GPIO_TYPE_CFG.GPIO_Mode      = GPIO_Mode_AN;				//Set as analog in mode
    AFE_GPIO_TYPE_CFG.GPIO_Speed     = GPIO_Speed_400KHz;   //low speed
    AFE_GPIO_TYPE_CFG.GPIO_PuPd      = GPIO_PuPd_NOPULL;   //No Internal Pull-up

    GPIO_Init(AFE_SPI_PORT,&AFE_GPIO_TYPE_CFG);           //GPIO SCLK SIMO SOMI init
	
	/* Release GPIO port */
	cc_sys_afe_gpio_deinit();
}

/*********************************************************************************************************
* AFE44xx_SPI_Write																	                 *
**********************************************************************************************************/

static AFE4403_FUNC_RETURN_TYPE AFE44xx_SPI_Write (unsigned char reg_address, unsigned int data)
{
    //uint16_t u16TimeOut;
		//for debug
		//printf("\t\t reg wr, addr = %x, data = %x \n", reg_address, data );
    //GPIO_ResetBits(AFE_SPI_STE_PORT, AFE_SPI_STE);                      //Cleare STE to start a new communication 
    SPI_Cmd(AFE44x0_SPI, ENABLE);                                       // Enable SPI
	
	SPI_I2S_ReceiveData(AFE44x0_SPI);                        // Dummy Read Rx buf
	
	/* 	pull STE low 		*/
	GPIO_ResetBits(AFE_SPI_PORT, AFE_SPI_STE_PIN);                      //Cleare STE to start a new communication 
	
    SPI_I2S_SendData(AFE44x0_SPI, reg_address);      
	
	// Send the first byte to the TX Buffer: Address of register
	SPI_Status_Wait(AFE44x0_SPI,SPI_I2S_FLAG_TXE,SET);
		
    SPI_I2S_ReceiveData(AFE44x0_SPI);                        // Dummy Read Rx buf
    
    SPI_I2S_SendData(AFE44x0_SPI, (data>>16));                    // First Data, [23:16]
		
	SPI_Status_Wait(AFE44x0_SPI,SPI_I2S_FLAG_TXE,SET);
		
    SPI_I2S_ReceiveData(AFE44x0_SPI);                        // Dummy Read Rx buf

    SPI_I2S_SendData(AFE44x0_SPI, (data>>8));                     // Second Data, [15:8]

	SPI_Status_Wait(AFE44x0_SPI,SPI_I2S_FLAG_TXE,SET);
		
    SPI_I2S_ReceiveData(AFE44x0_SPI);                        // Dummy Read Rx buf

    SPI_I2S_SendData(AFE44x0_SPI, (data));                        // First Data, [7:0]

	SPI_Status_Wait(AFE44x0_SPI,SPI_I2S_FLAG_TXE,SET);
		
    SPI_I2S_ReceiveData(AFE44x0_SPI);                        // Dummy Read Rx buf
    
    SPI_Cmd(AFE44x0_SPI, DISABLE);                                      // Disable SPI
	/* 	pull STE low 		*/
	GPIO_SetBits(AFE_SPI_PORT, AFE_SPI_STE_PIN);                      //Cleare STE to start a new communication 

    return(AFE4403_RET_VAL_SUCCESS);
}



/*********************************************************************************************************
* afe4403_reg_read																	                 *
**********************************************************************************************************/
static AFE4403_FUNC_RETURN_TYPE AFE44xx_SPI_Read(unsigned char ui8RegAddr,unsigned int * pui32Dat)
{
    unsigned char SPI_Rx_buf[3] = {0x0,0x0,0x0};
	//uint16_t u16TimeOut = 0;

    SPI_Cmd(AFE44x0_SPI, ENABLE);                                       // Enable SPI

    SPI_I2S_ReceiveData(AFE44x0_SPI);

	/* 	pull STE low 		*/
	GPIO_ResetBits(AFE_SPI_PORT, AFE_SPI_STE_PIN);                      //Cleare STE to start a new communication 

    SPI_I2S_SendData(AFE44x0_SPI, ui8RegAddr);                  
                                                                        // Send the first byte to the TX Buffer: Address of registe
		
	SPI_Status_Wait(AFE44x0_SPI,SPI_I2S_FLAG_RXNE,SET);
		
    SPI_I2S_ReceiveData(AFE44x0_SPI);																		// read dummy data from address sending

	SPI_Status_Wait(AFE44x0_SPI,SPI_I2S_FLAG_TXE,SET);
		
    SPI_I2S_SendData(AFE44x0_SPI, 0x0000);                              // Write dummy data for first data

	SPI_Status_Wait(AFE44x0_SPI,SPI_I2S_FLAG_RXNE,SET);
		
    SPI_Rx_buf[0] = SPI_I2S_ReceiveData(AFE44x0_SPI);          // read first data

	SPI_Status_Wait(AFE44x0_SPI,SPI_I2S_FLAG_TXE,SET);
		
    SPI_I2S_SendData(AFE44x0_SPI, 0x0000);                              // Write dummy data for second data

	SPI_Status_Wait(AFE44x0_SPI,SPI_I2S_FLAG_RXNE,SET);
		
    SPI_Rx_buf[1] = SPI_I2S_ReceiveData(AFE44x0_SPI);          // read second data
		
	SPI_Status_Wait(AFE44x0_SPI,SPI_I2S_FLAG_TXE,SET);
		
    SPI_I2S_SendData(AFE44x0_SPI, 0x0000);                              // Write dummy data for third data

	SPI_Status_Wait(AFE44x0_SPI,SPI_I2S_FLAG_RXNE,SET);
		
    SPI_Rx_buf[2] = SPI_I2S_ReceiveData(AFE44x0_SPI);          // read third data
		

	SPI_Status_Wait(AFE44x0_SPI,SPI_I2S_FLAG_TXE,SET);
		
	SPI_Status_Wait(AFE44x0_SPI,SPI_I2S_FLAG_BSY,RESET);

    SPI_Cmd(AFE44x0_SPI, DISABLE);                                      // Disable SPI
	/* 	pull STE up 		*/
	GPIO_SetBits(AFE_SPI_PORT, AFE_SPI_STE_PIN);                      //Cleare STE to start a new communication 

    (*pui32Dat) = (0x00<<24)|(SPI_Rx_buf[0]<<16)|(SPI_Rx_buf[1]<<8)|(SPI_Rx_buf[2]);
    return(AFE4403_RET_VAL_SUCCESS);
}

/******************************************
* Name:             cc_sys_afe_powerdown()
* Author:           yun.zhu@ywkang.com
* Date:             2016/3/15
* Description:      the afe to power down mode
* Param[in]
*    1. 
* Param[out]:
*    1. 
* Return:           NONE
******************************************/
static void cc_sys_afe_powerdown(void)
{
    
	/*Disable GPIO interrupt*/
	cc_sys_afe_disable_data_int();

	/*GPIO and SPI disable, and close clock source*/
	cc_sys_afe_ctrl_deinit();
    
    /*  Need change due to different connections    */
	AFE44x0_GPIO_PDNZ_CLR;
	AFE44x0_GPIO_RESETZ_CLR;
}

//end file
