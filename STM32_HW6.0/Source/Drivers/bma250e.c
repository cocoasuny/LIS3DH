#include "bma250e.h"
#include <stdio.h>
#include <stdint.h>
#include "bma2x2.h"
#include "IIC_GPIO.h"
#include "step.h"
#include "SPI.h"
#include "platform.h"

/*******************************************************************************
* @brief  Configures the spi interface for BMA250E
* @param  void
* @retval void
*******************************************************************************/

/******************************************************************
*                        BMA250E_SPI_Port_Init                    *
* [Yun] Init BMA250e spi port                                     * 
*******************************************************************/
void BMA250E_SPI_Oper_Init(void)
{
	GPIO_InitTypeDef BMA250E_GPIO_TYPE_CFG;
    SPI_InitTypeDef BMA250E_SPI_TYPE_CFG;           //Define the init type
    
    // Enable the SPI peripheral
    RCC_APB2PeriphClockCmd(RCC_APBxPeriph_SPI_BMA250E, ENABLE);
    
    // Enable SCK, MOSI, MISO and NSS GPIO clocks */
    RCC_AHBPeriphClockCmd(BMA250E_SPI_STE_GPIO_CLK | BMA250E_SPI_SCLK_GPIO_CLK | BMA250E_SPI_SOMI_GPIO_CLK |
                        BMA250E_SPI_SIMO_GPIO_CLK , ENABLE);
	
    // SPI1 Alternate function mapping
    GPIO_PinAFConfig(BMA250E_SPI_PORT, BMA250E_SPI_SCLK_PINSOURCE, GPIO_AF_SPI1);
    GPIO_PinAFConfig(BMA250E_SPI_PORT, BMA250E_SPI_SOMI_PINSOURCE, GPIO_AF_SPI1);
    GPIO_PinAFConfig(BMA250E_SPI_PORT, BMA250E_SPI_SIMO_PINSOURCE, GPIO_AF_SPI1);

    // Enable GPIO as alternal function
	BMA250E_GPIO_TYPE_CFG.GPIO_Pin       = PT_BMA250E_SPI_SCLK_PIN | BMA250E_SPI_SIMO_PIN | BMA250E_SPI_SOMI_PIN; // Pin definition
    BMA250E_GPIO_TYPE_CFG.GPIO_Mode      = GPIO_Mode_AF;       //Set as AF mode
    BMA250E_GPIO_TYPE_CFG.GPIO_Speed     = GPIO_Speed_10MHz;   //Middle speed
    BMA250E_GPIO_TYPE_CFG.GPIO_OType     = GPIO_OType_PP;      //Output type
    BMA250E_GPIO_TYPE_CFG.GPIO_PuPd      = GPIO_PuPd_UP;       //Enable Internal Pull-up

    GPIO_Init(BMA250E_SPI_PORT,&BMA250E_GPIO_TYPE_CFG);           //GPIO SCLK SIMO SOMI init
	
    // Enable STE as output mode
	BMA250E_GPIO_TYPE_CFG.GPIO_Pin       = BMA250E_SPI_STE_PIN;     //Pin definition
    BMA250E_GPIO_TYPE_CFG.GPIO_Mode      = GPIO_Mode_OUT;           //Set as output mode
    BMA250E_GPIO_TYPE_CFG.GPIO_Speed     = GPIO_Speed_10MHz;        //Middle speed
    BMA250E_GPIO_TYPE_CFG.GPIO_OType     = GPIO_OType_PP;           //Output type
    BMA250E_GPIO_TYPE_CFG.GPIO_PuPd      = GPIO_PuPd_UP;            //Enable Internal Pull-up

    GPIO_Init(PT_BMA250E_SPI_STE_PORT,&BMA250E_GPIO_TYPE_CFG);             //GPIO STE init

    /* 	SPI init 				*/
    /*Configure SPI1 for BMA250E as following setup:
      (+) SPI 1 for AFE4400
      (+) Master Mode
      (+) Full Duplex Mode
      (+) 8 bit mode
      (+) POL = Low
      (+) CPHA = Low
      (+) Software NSS control
      (+) Baud Rate <= 15MHz                */
    /* Initialize the SPI_Direction member */
    BMA250E_SPI_TYPE_CFG.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    /* initialize the SPI_Mode member */
    BMA250E_SPI_TYPE_CFG.SPI_Mode = SPI_Mode_Master;
    /* initialize the SPI_DataSize member */
    BMA250E_SPI_TYPE_CFG.SPI_DataSize = SPI_DataSize_8b;
    /* Initialize the SPI_CPOL member */
    BMA250E_SPI_TYPE_CFG.SPI_CPOL = SPI_CPOL_Low;
    /* Initialize the SPI_CPHA member */
    BMA250E_SPI_TYPE_CFG.SPI_CPHA = SPI_CPHA_1Edge;
    /* Initialize the SPI_NSS member */
    //AFE_SPI_TYPE_CFG.SPI_NSS = SPI_NSS_Hard;
	BMA250E_SPI_TYPE_CFG.SPI_NSS = SPI_NSS_Soft;
    /* Initialize the SPI_BaudRatePrescaler member */
    BMA250E_SPI_TYPE_CFG.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    /* Initialize the SPI_FirstBit member */
    BMA250E_SPI_TYPE_CFG.SPI_FirstBit = SPI_FirstBit_MSB;
    /* Initialize the SPI_CRCPolynomial member */
    BMA250E_SPI_TYPE_CFG.SPI_CRCPolynomial = 7;

    SPI_Init(BMA250E_SPI, &BMA250E_SPI_TYPE_CFG);
	/* Set NSS Pin(STE) as output mode */
	//SPI_SSOutputCmd(AFE44x0_SPI, ENABLE);
	SPI_SSOutputCmd(BMA250E_SPI, DISABLE);
}

/******************************************************************
*                        BMA250E_SPI_Port_DeInit                  *
* 1. set SPI port to GPIO analog input                            * 
*******************************************************************/
void BMA250E_SPI_Oper_DeInit(void)
{
	GPIO_InitTypeDef BMA250E_GPIO_TYPE_CFG;
    
    // Enable the SPI peripheral
    RCC_APB2PeriphClockCmd(RCC_APBxPeriph_SPI_BMA250E, ENABLE);
    // Enable SCK, MOSI, MISO and NSS GPIO clocks */
    RCC_AHBPeriphClockCmd(BMA250E_SPI_STE_GPIO_CLK | BMA250E_SPI_SCLK_GPIO_CLK | BMA250E_SPI_SOMI_GPIO_CLK |
                        BMA250E_SPI_SIMO_GPIO_CLK , ENABLE);
	
    // Enable GPIO as alternal function
	BMA250E_GPIO_TYPE_CFG.GPIO_Pin       = PT_BMA250E_SPI_SCLK_PIN | BMA250E_SPI_SIMO_PIN | BMA250E_SPI_SOMI_PIN; //Pin definition
    BMA250E_GPIO_TYPE_CFG.GPIO_Mode      = GPIO_Mode_AN;            //Set as analog mode
    BMA250E_GPIO_TYPE_CFG.GPIO_Speed     = GPIO_Speed_400KHz;   
    BMA250E_GPIO_TYPE_CFG.GPIO_OType     = GPIO_OType_PP;           //Output type
    BMA250E_GPIO_TYPE_CFG.GPIO_PuPd      = GPIO_PuPd_NOPULL;        //Enable Internal Pull-up

    GPIO_Init(BMA250E_SPI_PORT,&BMA250E_GPIO_TYPE_CFG);             //GPIO SCLK SIMO SOMI init
	
    // Enable STE as output mode
	BMA250E_GPIO_TYPE_CFG.GPIO_Pin       = BMA250E_SPI_STE_PIN;     //Pin definition
    BMA250E_GPIO_TYPE_CFG.GPIO_Mode      = GPIO_Mode_AN;            //Set as analog mode
    BMA250E_GPIO_TYPE_CFG.GPIO_Speed     = GPIO_Speed_400KHz;   
    BMA250E_GPIO_TYPE_CFG.GPIO_OType     = GPIO_OType_PP;           //Output type
    BMA250E_GPIO_TYPE_CFG.GPIO_PuPd      = GPIO_PuPd_NOPULL;        //Enable Internal Pull-up

    GPIO_Init(PT_BMA250E_SPI_STE_PORT,&BMA250E_GPIO_TYPE_CFG);             //GPIO STE init
}

/******************************************************************
*                        BMA250E_Int1_Port_Init                    *
* [Yun] init STM32L port to interrupt input and enable             * 
*******************************************************************/
void BMA250E_Int1_Port_Enable(void)
{
	EXTI_InitTypeDef BMA250E_INT1_EXTI_TYPE_CFG;
	GPIO_InitTypeDef BMA250E_INT1_GPIO_TYPE_CFG;
    
    /* 	Enable GPIO port 		*/
    /* 	Enable clock 			*/
    RCC_AHBPeriphClockCmd(BMA250E_INT1_GPIO_CLK, ENABLE);
	
	/*	Enable gpio function 	*/
 	BMA250E_INT1_GPIO_TYPE_CFG.GPIO_Pin       = BMA250E_INT1_GPIO_PIN;
    BMA250E_INT1_GPIO_TYPE_CFG.GPIO_Mode      = GPIO_Mode_IN;
    BMA250E_INT1_GPIO_TYPE_CFG.GPIO_Speed     = GPIO_Speed_10MHz;  
    BMA250E_INT1_GPIO_TYPE_CFG.GPIO_OType     = GPIO_OType_PP;   
    BMA250E_INT1_GPIO_TYPE_CFG.GPIO_PuPd      = GPIO_PuPd_UP;  

    GPIO_Init(PT_BMA250E_INT1_GPIO_PORT,&BMA250E_INT1_GPIO_TYPE_CFG); 

    /* 	Enable EXTI interrupt line 		*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    SYSCFG_EXTILineConfig(BMA250E_INT1_EXTI_PORT, BMA250E_INT1_EXTI_PIN);

    BMA250E_INT1_EXTI_TYPE_CFG.EXTI_Line      = BMA250E_INT1_EXTI_LINE;         // EXTI Line
    BMA250E_INT1_EXTI_TYPE_CFG.EXTI_Mode      = EXTI_Mode_Interrupt;            // EXTI Mode, Interrupt
    BMA250E_INT1_EXTI_TYPE_CFG.EXTI_Trigger   = EXTI_Trigger_Falling;            // EXTI trigger, Falling edge
    BMA250E_INT1_EXTI_TYPE_CFG.EXTI_LineCmd   = ENABLE;

    EXTI_Init(&BMA250E_INT1_EXTI_TYPE_CFG);                                     // EXTI Init

    EXTI_ClearITPendingBit(BMA250E_INT1_EXTI_LINE);
	EXTI_ClearFlag(BMA250E_INT1_EXTI_LINE);
}

/******************************************************************
*                        BMA250E_Int1_Port_Disable                *
* [Yun] init STM32L port to interrupt input and enable             * 
*******************************************************************/
void BMA250E_Int1_Port_Disable(void)
{
	EXTI_InitTypeDef BMA250E_INT1_EXTI_TYPE_CFG;
	GPIO_InitTypeDef BMA250E_INT1_GPIO_TYPE_CFG;
    
    /* 	Clear the interrupt flag 		*/
    EXTI_ClearITPendingBit(BMA250E_INT1_EXTI_LINE);
	EXTI_ClearFlag(BMA250E_INT1_EXTI_LINE);
    /* 	Enable clock 			*/
    RCC_AHBPeriphClockCmd(BMA250E_INT1_GPIO_CLK, ENABLE);
	
	/*	Enable gpio function 	*/
 	BMA250E_INT1_GPIO_TYPE_CFG.GPIO_Pin       = BMA250E_INT1_GPIO_PIN;
    BMA250E_INT1_GPIO_TYPE_CFG.GPIO_Mode      = GPIO_Mode_AN;
    BMA250E_INT1_GPIO_TYPE_CFG.GPIO_Speed     = GPIO_Speed_10MHz;  
    BMA250E_INT1_GPIO_TYPE_CFG.GPIO_OType     = GPIO_OType_PP;   
    BMA250E_INT1_GPIO_TYPE_CFG.GPIO_PuPd      = GPIO_PuPd_UP;  

    GPIO_Init(PT_BMA250E_INT1_GPIO_PORT,&BMA250E_INT1_GPIO_TYPE_CFG); 

    /* 	Enable EXTI interrupt line 		*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    //SYSCFG_EXTILineConfig(BMA250E_INT1_EXTI_PORT, BMA250E_INT1_EXTI_PIN);

    BMA250E_INT1_EXTI_TYPE_CFG.EXTI_Line      = BMA250E_INT1_EXTI_LINE;                 // EXTI Line
    BMA250E_INT1_EXTI_TYPE_CFG.EXTI_Mode      = EXTI_Mode_Interrupt;                    // EXTI Mode, Interrupt
    BMA250E_INT1_EXTI_TYPE_CFG.EXTI_Trigger   = EXTI_Trigger_Rising;                    // EXTI trigger, Falling edge
    BMA250E_INT1_EXTI_TYPE_CFG.EXTI_LineCmd   = DISABLE;

    EXTI_Init(&BMA250E_INT1_EXTI_TYPE_CFG);                                             // EXTI Init
}

/******************************************************************
*                        BMA250E_Int2_Port_Enable                  *
* [Yun] set STM32L port to analog input                           * 
*******************************************************************/
void BMA250E_Int2_Port_Enable(void)
{
	EXTI_InitTypeDef BMA250E_INT2_EXTI_TYPE_CFG;
	GPIO_InitTypeDef BMA250E_INT2_GPIO_TYPE_CFG;
    
    /* 	Enable GPIO port 		*/

    /* 	Enable clock 			*/
    RCC_AHBPeriphClockCmd(BMA250E_INT2_GPIO_CLK, ENABLE);
	
	/*	Enable gpio function 	*/
 	BMA250E_INT2_GPIO_TYPE_CFG.GPIO_Pin       = BMA250E_INT2_GPIO_PIN;
    BMA250E_INT2_GPIO_TYPE_CFG.GPIO_Mode      = GPIO_Mode_IN;
    BMA250E_INT2_GPIO_TYPE_CFG.GPIO_Speed     = GPIO_Speed_10MHz;  
    BMA250E_INT2_GPIO_TYPE_CFG.GPIO_OType     = GPIO_OType_PP;   
    BMA250E_INT2_GPIO_TYPE_CFG.GPIO_PuPd      = GPIO_PuPd_UP;  

    GPIO_Init(PT_BMA250E_INT2_GPIO_PORT,&BMA250E_INT2_GPIO_TYPE_CFG); 

    /* 	Enable EXTI interrupt line 		*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    SYSCFG_EXTILineConfig(BMA250E_INT2_EXTI_PORT, BMA250E_INT2_EXTI_PIN);

    BMA250E_INT2_EXTI_TYPE_CFG.EXTI_Line      = BMA250E_INT2_EXTI_LINE;                 // EXTI Line
    BMA250E_INT2_EXTI_TYPE_CFG.EXTI_Mode      = EXTI_Mode_Interrupt;                    // EXTI Mode, Interrupt
    BMA250E_INT2_EXTI_TYPE_CFG.EXTI_Trigger   = EXTI_Trigger_Rising;                    // EXTI trigger, Falling edge
    BMA250E_INT2_EXTI_TYPE_CFG.EXTI_LineCmd   = ENABLE;

    EXTI_Init(&BMA250E_INT2_EXTI_TYPE_CFG);                                             // EXTI Init

    EXTI_ClearITPendingBit(BMA250E_INT2_EXTI_LINE);
	EXTI_ClearFlag(BMA250E_INT2_EXTI_LINE);
}

/******************************************************************
*                        BMA250E_Int2_Port_Disable                *
* [Yun] init STM32L port to interrupt input and enable             * 
*******************************************************************/
void BMA250E_Int2_Port_Disable(void)
{
	EXTI_InitTypeDef BMA250E_INT2_EXTI_TYPE_CFG;
	GPIO_InitTypeDef BMA250E_INT2_GPIO_TYPE_CFG;
    
    /* 	Clear the interrupt flag 		*/
    EXTI_ClearITPendingBit(BMA250E_INT2_EXTI_LINE);
	EXTI_ClearFlag(BMA250E_INT2_EXTI_LINE);
    /* 	Enable clock 			*/
    RCC_AHBPeriphClockCmd(BMA250E_INT2_GPIO_CLK, ENABLE);
	
	/*	Enable gpio function 	*/
 	BMA250E_INT2_GPIO_TYPE_CFG.GPIO_Pin       = BMA250E_INT2_GPIO_PIN;
    BMA250E_INT2_GPIO_TYPE_CFG.GPIO_Mode      = GPIO_Mode_AN;
    BMA250E_INT2_GPIO_TYPE_CFG.GPIO_Speed     = GPIO_Speed_10MHz;  
    BMA250E_INT2_GPIO_TYPE_CFG.GPIO_OType     = GPIO_OType_PP;   
    BMA250E_INT2_GPIO_TYPE_CFG.GPIO_PuPd      = GPIO_PuPd_UP;  

    GPIO_Init(PT_BMA250E_INT2_GPIO_PORT,&BMA250E_INT2_GPIO_TYPE_CFG); 

    /* 	Enable EXTI interrupt line 		*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    //SYSCFG_EXTILineConfig(BMA250E_INT2_EXTI_PORT, BMA250E_INT2_EXTI_PIN);

    BMA250E_INT2_EXTI_TYPE_CFG.EXTI_Line      = BMA250E_INT2_EXTI_LINE;                 // EXTI Line
    BMA250E_INT2_EXTI_TYPE_CFG.EXTI_Mode      = EXTI_Mode_Interrupt;                    // EXTI Mode, Interrupt
    BMA250E_INT2_EXTI_TYPE_CFG.EXTI_Trigger   = EXTI_Trigger_Rising;                    // EXTI trigger, Falling edge
    BMA250E_INT2_EXTI_TYPE_CFG.EXTI_LineCmd   = DISABLE;

    EXTI_Init(&BMA250E_INT2_EXTI_TYPE_CFG);                                             // EXTI Init
}

/*******************************************************************************
* @brief  Sends a byte through the SPI interface and return the byte
* @param  SPIx: To select the SPIx/I2Sx peripheral
* @param  wr_temp: Data to be transmitted
* @param  *rd_temp: The value of the received data
* @retval The state of read and write bma250e by SPI interface.
*******************************************************************************/
char BMA250E_SPI_ReadWriteByte(SPI_TypeDef* SPIx,uint8_t wr_temp,uint8_t *rd_temp)
{
	uint32_t wr_timeout = 1000,rd_timeout = 40000;
    
	 /* Send byte through the SPIx peripheral */
    SPI_I2S_SendData(SPIx,wr_temp);
	
	/* Loop while DR register in not emplty */
    while ((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (--wr_timeout))
	{}
    
    /* Check if tx is timeout? */
    if(!wr_timeout)
    {
        return BMA250E_ERROR;
    }
 	
    /* Wait to receive a byte */
    while ((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET) && (--rd_timeout))
	{}
	
	/* Check if rx is timeout? */    
    if(!rd_timeout)
    {
        return BMA250E_ERROR;
    }
    
    /* Return the byte read from the SPI bus */
    *rd_temp = SPI_I2S_ReceiveData(SPIx);

    return BAM250E_CORR;
}

char BMA250E_SPI_Read(uint8_t dev_addr,uint8_t register_addr,uint8_t *register_data,uint8_t rd_len)
{
    uint8_t  rd_temp    = 0,wr_temp = 0;
    uint16_t index      = 0;
    uint32_t status     = BAM250E_CORR;
    uint32_t wr_timeout = 1000,rd_timeout = 40000;
   
    wr_temp = register_addr | BMA250E_SPI_READ;
    
    SPI_Cmd(BMA250E_SPI, ENABLE);
	
	SPI_I2S_ReceiveData(BMA250E_SPI);
	
	GPIO_ResetBits(BMA250E_SPI_STE_PORT,BMA250E_SPI_STE_PIN);       // CS = 0

	if(BMA250E_SPI_ReadWriteByte(BMA250E_SPI, wr_temp, &rd_temp) == BAM250E_CORR)
	{
        SPI_I2S_SendData(BMA250E_SPI,wr_temp);
		for(index = 0;index < rd_len;index++)
		{   
            wr_timeout = 1000,rd_timeout = 40000;
            
            /* Loop while DR register in not emplty */
            while ((SPI_I2S_GetFlagStatus(BMA250E_SPI, SPI_I2S_FLAG_TXE) == RESET) && (--wr_timeout))
            {}
            
            /* Check if tx is timeout? */
            if(!wr_timeout)
            {
                status = BMA250E_ERROR;
                break;
            }
            
            SPI_I2S_SendData(BMA250E_SPI,0);
            
            /* Wait to receive a byte */
            while ((SPI_I2S_GetFlagStatus(BMA250E_SPI, SPI_I2S_FLAG_RXNE) == RESET) && (--rd_timeout))
            {}
            
            /* Check if rx is timeout? */    
            if(!rd_timeout)
            {
                status = BMA250E_ERROR;
                break;
            }
            
            /* Return the byte read from the SPI bus */
            *(register_data + index) = SPI_I2S_ReceiveData(BMA250E_SPI); 	
        }
    } 
    else
    {
        status = BMA250E_ERROR;
    }
   
    GPIO_SetBits(BMA250E_SPI_STE_PORT,BMA250E_SPI_STE_PIN);         // CS = 1
    SPI_Cmd(BMA250E_SPI, DISABLE);

    return(status);
}

//char BMA250E_SPI_Read(uint8_t dev_addr,uint8_t register_addr,uint8_t *register_data,uint8_t rd_len)
//{
//    uint8_t  rd_temp    = 0,wr_temp = 0;
//    uint16_t index      = 0;
//    uint32_t status     = BAM250E_CORR;
//   
//    wr_temp = register_addr | BMA250E_SPI_READ;
//    
//    SPI_Cmd(BMA250E_SPI, ENABLE);
//	GPIO_ResetBits(BMA250E_SPI_STE_PORT,BMA250E_SPI_STE_PIN);       // CS = 0

//	if(BMA250E_SPI_ReadWriteByte(BMA250E_SPI, wr_temp, &rd_temp) == BAM250E_CORR)
//	{
//		for(index = 0;index < rd_len;index++)
//		{   
//            if(BMA250E_SPI_ReadWriteByte(BMA250E_SPI, 0, (register_data + index)) != BAM250E_CORR)
//            {
//                status = BMA250E_ERROR;
//                break;
//            }
//            else
//            {
//                status = BAM250E_CORR;
//            }	
//        }
//    } 
//    else
//    {
//        status = BMA250E_ERROR;
//    }
//   
//    GPIO_SetBits(BMA250E_SPI_STE_PORT,BMA250E_SPI_STE_PIN);         // CS = 1
//    SPI_Cmd(BMA250E_SPI, DISABLE);
//    
//    return(status);
//}

char BMA250E_SPI_Write(uint8_t dev_addr,uint8_t register_addr,uint8_t *register_data,uint8_t wr_len)
{
    uint8_t  	rd_temp = 0;
	uint8_t 	wr_temp = 0;
    BMA250E_STAT status     = BMA250E_ERROR;
    uint16_t index      = 0;
    
    wr_temp = register_addr | BMA250E_SPI_WRITE;
    
    SPI_Cmd(BMA250E_SPI, ENABLE);
	
	SPI_I2S_ReceiveData(BMA250E_SPI);
	
	GPIO_ResetBits(BMA250E_SPI_STE_PORT,BMA250E_SPI_STE_PIN);       // CS = 0
    
    if(BMA250E_SPI_ReadWriteByte(BMA250E_SPI, wr_temp, &rd_temp) == BAM250E_CORR)
    {
        for(index = 0;index < wr_len;index ++)
        {
            if(BMA250E_SPI_ReadWriteByte(BMA250E_SPI, *(register_data + index), &rd_temp) != BAM250E_CORR)
            {
                status = BMA250E_ERROR;
                break;
            }
            else
            {
                status = BAM250E_CORR;
            }
        }
    } 
    else
    {
        status = BMA250E_ERROR;
    }
    
    GPIO_SetBits(BMA250E_SPI_STE_PORT,BMA250E_SPI_STE_PIN);         // CS = 1
    SPI_Cmd(SysSPI, DISABLE);
    
    return(status);
}

void BMA250E_Init_Step(void)
{
    bma2x2_soft_reset();
    Delay_ms(3); 
    //bma2x2_set_offset_target(BMA2x2_CUT_OFF,0);
    bma2x2_set_slow_comp(BMA2x2_SLOW_COMP_X,0);
    bma2x2_set_slow_comp(BMA2x2_SLOW_COMP_Y,0);
    bma2x2_set_slow_comp(BMA2x2_SLOW_COMP_Z,0);
    bma2x2_set_sleep_dur(BMA2x2_SLEEP_DUR_25MS);    // set sleep_dur time to 25ms
    bma2x2_set_sleeptmr_mode (1);                   // enable Eqidistant sampling mode(EST)
    bma2x2_set_range(BMA2x2_RANGE_16G);
    bma2x2_set_bandwidth(BMA2x2_BW_250HZ); 
    bma2x2_set_int_od(BMA2x2_INT1_OUTPUT,0);        // open drain
    bma2x2_set_int_lvl(BMA2x2_INT1_LEVEL,0);        // active low level for INT1 pin
    bma2x2_set_fifo_mode(1);                        // FIFO mode
    bma2x2_set_int_ffull(1);
    bma2x2_set_int1_ffull(1); 
    bma2x2_set_mode(BMA2x2_MODE_LOWPOWER1);
}

void BMA250E_Init_SingleTap(void)
{
    bma2x2_soft_reset();
    Delay_ms(3); 
    
    // Single-Tap Interrupt 
    bma2x2_set_range(BMA2x2_RANGE_16G);
    bma2x2_set_tap_sample (3);
    bma2x2_set_tap_thr(1);
    bma2x2_set_tap_shock (0);                       // 50ms
    bma2x2_set_tap_quiet (1);                       // 20ms
    bma2x2_set_tap_dur (0);                         // 50ms
    bma2x2_set_int_od(BMA2x2_INT1_OUTPUT,1);        // open drain
    bma2x2_set_int_lvl(BMA2x2_INT1_LEVEL,0);        // active low level for INT1 pin
    bma2x2_set_int_s_tap (BMA2x2_INT1_STAP,1);
    bma2x2_set_Int_Enable(BMA2x2_Single_Tap_Interrupt,1);
    bma2x2_set_mode(BMA2x2_MODE_NORMAL);  
}

void BMA250E_Init_Suspend(void)
{
    bma2x2_soft_reset();
    Delay_ms(3); 
    
    // Single-Tap Interrupt 
    bma2x2_set_range(BMA2x2_RANGE_16G);
    bma2x2_set_int_ffull(0);
    bma2x2_set_int1_ffull(0); 
    bma2x2_set_tap_sample (3);
    bma2x2_set_tap_thr(1);
    bma2x2_set_tap_shock (0);                       // 50ms
    bma2x2_set_tap_quiet (1);                       // 20ms
    bma2x2_set_tap_dur (0);                         // 50ms
    bma2x2_set_int_od(BMA2x2_INT1_OUTPUT,1);        // open drain
    bma2x2_set_int_lvl(BMA2x2_INT1_LEVEL,0);        // active low level for INT1 pin
    bma2x2_set_Int_Enable(BMA2x2_Single_Tap_Interrupt,1);
    bma2x2_set_int_s_tap (BMA2x2_INT1_STAP,1);
    bma2x2_set_mode(BMA2x2_MODE_NORMAL);     
}

/*******************************************************************************
* Function Name  : BMA250E_Init_SpO2_MoveCheck
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BMA250E_Init_SpO2_MoveCheck(void)
{
    bma2x2_soft_reset();
    Delay_ms(3); 
    
    // Single-Tap Interrupt 
    bma2x2_set_range(BMA2x2_RANGE_4G);			/* Change to 4G full scale 		*/
	bma2x2_set_bandwidth(BMA2x2_BW_250HZ); 		/* Set to 250 Hz 				*/
    bma2x2_set_offset_target(BMA2x2_CUT_OFF,0);
    bma2x2_set_slow_comp(BMA2x2_SLOW_COMP_X,1);
    bma2x2_set_slow_comp(BMA2x2_SLOW_COMP_Y,1);
    bma2x2_set_slow_comp(BMA2x2_SLOW_COMP_Z,1);
    bma2x2_set_mode(BMA2x2_MODE_NORMAL);    	/* Set to normal mode 			*/

}

/*******************************************************************************
* Function Name  : BMA250E_Init_Stop
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BMA250E_Init_Stop(void)
{
    bma2x2_soft_reset();
    Delay_ms(3); 
    bma2x2_set_mode(BMA2x2_MODE_STANDBY);
}
//end of files
