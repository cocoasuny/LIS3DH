/******************** (C) COPYRIGHT 2014 iCareTech ********************

BMA250e low level driver 
Provide SPI init and SPI Write/Read
*
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "BMA250e_Driver.h"
#include "platform.h"
#include "stm32l1xx.h"
#include "Usart.h"
#include "common.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/


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
	BMA250E_GPIO_TYPE_CFG.GPIO_Pin       = BMA250E_SPI_STE_PIN | BMA250E_SPI_SIMO_PIN | BMA250E_SPI_SOMI_PIN;    
                                                            //Pin definition
    BMA250E_GPIO_TYPE_CFG.GPIO_Mode      = GPIO_Mode_AF;       //Set as AF mode
    BMA250E_GPIO_TYPE_CFG.GPIO_Speed     = GPIO_Speed_10MHz;   //Middle speed
    BMA250E_GPIO_TYPE_CFG.GPIO_OType     = GPIO_OType_PP;      //Output type
    BMA250E_GPIO_TYPE_CFG.GPIO_PuPd      = GPIO_PuPd_UP;       //Enable Internal Pull-up

    GPIO_Init(BMA250E_SPI_PORT,&BMA250E_GPIO_TYPE_CFG);           //GPIO SCLK SIMO SOMI init
	
    // Enable STE as output mode
	BMA250E_GPIO_TYPE_CFG.GPIO_Pin       = BMA250E_SPI_STE_PIN;    
                                                            //Pin definition
    BMA250E_GPIO_TYPE_CFG.GPIO_Mode      = GPIO_Mode_OUT;       //Set as output mode
    BMA250E_GPIO_TYPE_CFG.GPIO_Speed     = GPIO_Speed_10MHz;   //Middle speed
    BMA250E_GPIO_TYPE_CFG.GPIO_OType     = GPIO_OType_PP;      //Output type
    BMA250E_GPIO_TYPE_CFG.GPIO_PuPd      = GPIO_PuPd_UP;       //Enable Internal Pull-up

    GPIO_Init(BMA250E_SPI_PORT,&BMA250E_GPIO_TYPE_CFG);           //GPIO STE init


    /* 	SPI init 				*/
    /*Configure SPI1 for BMA250E as following setup:
      (+) SPI 1 for AFE4400
      (+) Master Mode
      (+) Full Duplex Mode
      (+) 8 bit mode
      (+) POL = Low
      (+) CPHA = Low
      (+) Software NSS control
      (+) Baud Rate <= 15MHz

    */
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
	BMA250E_GPIO_TYPE_CFG.GPIO_Pin       = BMA250E_SPI_STE_PIN | BMA250E_SPI_SIMO_PIN | BMA250E_SPI_SOMI_PIN;    
                                                            //Pin definition
    BMA250E_GPIO_TYPE_CFG.GPIO_Mode      = GPIO_Mode_AN;       //Set as analog mode
    BMA250E_GPIO_TYPE_CFG.GPIO_Speed     = GPIO_Speed_400KHz;   
    BMA250E_GPIO_TYPE_CFG.GPIO_OType     = GPIO_OType_PP;      //Output type
    BMA250E_GPIO_TYPE_CFG.GPIO_PuPd      = GPIO_PuPd_NOPULL;       //Enable Internal Pull-up

    GPIO_Init(BMA250E_SPI_PORT,&BMA250E_GPIO_TYPE_CFG);           //GPIO SCLK SIMO SOMI init
	
    // Enable STE as output mode
	BMA250E_GPIO_TYPE_CFG.GPIO_Pin       = BMA250E_SPI_STE_PIN;    
                                                            //Pin definition
    BMA250E_GPIO_TYPE_CFG.GPIO_Mode      = GPIO_Mode_AN;       //Set as analog mode
    BMA250E_GPIO_TYPE_CFG.GPIO_Speed     = GPIO_Speed_400KHz;   
    BMA250E_GPIO_TYPE_CFG.GPIO_OType     = GPIO_OType_PP;      //Output type
    BMA250E_GPIO_TYPE_CFG.GPIO_PuPd      = GPIO_PuPd_NOPULL;       //Enable Internal Pull-up

    GPIO_Init(BMA250E_SPI_PORT,&BMA250E_GPIO_TYPE_CFG);           //GPIO STE init
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

    BMA250E_INT1_EXTI_TYPE_CFG.EXTI_Line      = BMA250E_INT1_EXTI_LINE;         
                                                            //EXTI Line
    BMA250E_INT1_EXTI_TYPE_CFG.EXTI_Mode      = EXTI_Mode_Interrupt;
                                                            //EXTI Mode, Interrupt
    BMA250E_INT1_EXTI_TYPE_CFG.EXTI_Trigger   = EXTI_Trigger_Rising;
                                                            //EXTI trigger, Falling edge
    BMA250E_INT1_EXTI_TYPE_CFG.EXTI_LineCmd   = ENABLE;

    EXTI_Init(&BMA250E_INT1_EXTI_TYPE_CFG);           //EXTI Init

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

    BMA250E_INT1_EXTI_TYPE_CFG.EXTI_Line      = BMA250E_INT1_EXTI_LINE;         
                                                            //EXTI Line
    BMA250E_INT1_EXTI_TYPE_CFG.EXTI_Mode      = EXTI_Mode_Interrupt;
                                                            //EXTI Mode, Interrupt
    BMA250E_INT1_EXTI_TYPE_CFG.EXTI_Trigger   = EXTI_Trigger_Rising;
                                                            //EXTI trigger, Falling edge
    BMA250E_INT1_EXTI_TYPE_CFG.EXTI_LineCmd   = DISABLE;

    EXTI_Init(&BMA250E_INT1_EXTI_TYPE_CFG);           //EXTI Init
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

    BMA250E_INT2_EXTI_TYPE_CFG.EXTI_Line      = BMA250E_INT2_EXTI_LINE;         
                                                            //EXTI Line
    BMA250E_INT2_EXTI_TYPE_CFG.EXTI_Mode      = EXTI_Mode_Interrupt;
                                                            //EXTI Mode, Interrupt
    BMA250E_INT2_EXTI_TYPE_CFG.EXTI_Trigger   = EXTI_Trigger_Rising;
                                                            //EXTI trigger, Falling edge
    BMA250E_INT2_EXTI_TYPE_CFG.EXTI_LineCmd   = ENABLE;

    EXTI_Init(&BMA250E_INT2_EXTI_TYPE_CFG);           //EXTI Init

    EXTI_ClearITPendingBit(BMA250E_INT2_EXTI_LINE);
	EXTI_ClearFlag(BMA250E_INT2_EXTI_LINE);
}


/******************************************************************
*                        BMA250E_Int1_Port_Disable                *
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

    BMA250E_INT2_EXTI_TYPE_CFG.EXTI_Line      = BMA250E_INT2_EXTI_LINE;         
                                                            //EXTI Line
    BMA250E_INT2_EXTI_TYPE_CFG.EXTI_Mode      = EXTI_Mode_Interrupt;
                                                            //EXTI Mode, Interrupt
    BMA250E_INT2_EXTI_TYPE_CFG.EXTI_Trigger   = EXTI_Trigger_Rising;
                                                            //EXTI trigger, Falling edge
    BMA250E_INT2_EXTI_TYPE_CFG.EXTI_LineCmd   = DISABLE;

    EXTI_Init(&BMA250E_INT2_EXTI_TYPE_CFG);           //EXTI Init
}



/******************************************************************
*                        BMA250E_SPI_Write_Byte                   *
* [Yun] write one byte to BMA250E thru SPI                        * 
*******************************************************************/
BMA250E_STAT BMA250E_SPI_Write_Byte(uint8_t regAddr, uint8_t regData)
{
    uint16_t u16TimeOut = 0;

    uint8_t 	addrWrite;

    /* 	set to write operation 		*/
    addrWrite = 0x7F & regAddr;

	SPI_Cmd(BMA250E_SPI, ENABLE);                                       // Enable SPI
	
	/* 	pull STE low 		*/
	GPIO_ResetBits(BMA250E_SPI_STE_PORT, BMA250E_SPI_STE_PIN);                      //Cleare STE to start a new communication 
	
    SPI_I2S_SendData(BMA250E_SPI, addrWrite);      
                                                                        // Send the first byte to the TX Buffer: Address of register
    while (SPI_I2S_GetFlagStatus(BMA250E_SPI, SPI_I2S_FLAG_TXE)!=SET)	//TX buffer empty?
		{
			if(++u16TimeOut == BMA250E_SPI_OP_TIMEOUT)
			{
				return(BMA250E_ERROR);
			}
		}
    SPI_I2S_ReceiveData(BMA250E_SPI); 
    

    SPI_I2S_SendData(BMA250E_SPI, regData);                  
    while (SPI_I2S_GetFlagStatus(BMA250E_SPI, SPI_I2S_FLAG_TXE)!=SET)	// TX buffer empty?
		{
			if(++u16TimeOut == BMA250E_SPI_OP_TIMEOUT)
			{
				return(BMA250E_ERROR);
			}
		}
    SPI_I2S_ReceiveData(BMA250E_SPI); 
    
    SPI_Cmd(BMA250E_SPI, DISABLE);                          
	/* 	pull STE low 		*/
	GPIO_SetBits(BMA250E_SPI_STE_PORT, BMA250E_SPI_STE_PIN);

	return(BAM250E_CORR);

}

/******************************************************************
*                        BMA250E_SPI_Read_Byte                    *
* [Yun] read one byte to BMA250E thru SPI                        * 
*******************************************************************/
BMA250E_STAT BMA250E_SPI_Read_Byte(uint8_t regAddr, uint8_t * regData)
{

    uint16_t u16TimeOut = 0;

    uint8_t 	addrRead;

    uint8_t 	inDat;

    /* 	set to read operation 		*/
    addrRead = 0x80 | regAddr;

	SPI_Cmd(BMA250E_SPI, ENABLE);                                       // Enable SPI
	
	/* 	pull STE low 		*/
	GPIO_ResetBits(BMA250E_SPI_STE_PORT, BMA250E_SPI_STE_PIN);                      //Cleare STE to start a new communication 
	
    SPI_I2S_SendData(BMA250E_SPI, addrRead);      
    
    u16TimeOut = 0;
    while (SPI_I2S_GetFlagStatus(BMA250E_SPI, SPI_I2S_FLAG_RXNE)!=SET)	// RX buffer Ready?
		{
			if(++u16TimeOut == BMA250E_SPI_OP_TIMEOUT)
			{
				return(BMA250E_ERROR);
			}
		}
    SPI_I2S_ReceiveData(BMA250E_SPI); 
    u16TimeOut = 0;
    SPI_I2S_SendData(BMA250E_SPI, addrRead);     /* 	Don't care the data 		*/ 
                                                                        // Send the first byte to the TX Buffer: Address of register
    while (SPI_I2S_GetFlagStatus(BMA250E_SPI, SPI_I2S_FLAG_RXNE)!=SET)	// RX buffer Ready?
		{
			if(++u16TimeOut == BMA250E_SPI_OP_TIMEOUT)
			{
				return(BMA250E_ERROR);
			}
		}
    inDat = SPI_I2S_ReceiveData(BMA250E_SPI); 

	u16TimeOut = 0;
    while (SPI_I2S_GetFlagStatus(BMA250E_SPI, SPI_I2S_FLAG_TXE)!=SET)	// TX buffer empty?
		{
			if(++u16TimeOut == BMA250E_SPI_OP_TIMEOUT)
			{
				return(BMA250E_ERROR);
			}
		}
	u16TimeOut = 0;
    while (SPI_I2S_GetFlagStatus(BMA250E_SPI, SPI_I2S_FLAG_BSY)!=RESET)	// Busy?
		{
			if(++u16TimeOut == BMA250E_SPI_OP_TIMEOUT)
			{
				return(BMA250E_ERROR);
			}
		}

    SPI_Cmd(BMA250E_SPI, DISABLE);                            
	/* 	pull STE up 		*/
	GPIO_SetBits(BMA250E_SPI_STE_PORT, BMA250E_SPI_STE_PIN);  

	*regData = inDat;

	return(BAM250E_CORR);

}

/******************************************************************
*                        BMA250E_SPI_Read_Byte                    *
* [Yun] read one byte to BMA250E thru SPI                        * 
*******************************************************************/
BMA250E_STAT BMA250E_Get_Raw_Dat(AxesRaw_t * pRawDat)
{
	
}




/******************* (C) COPYRIGHT 2014 iCareTech ****************************/

/*****END OF FILE****/
