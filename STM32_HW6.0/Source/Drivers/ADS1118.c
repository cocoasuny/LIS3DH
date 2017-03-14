#include "ADS1118.h"

/*******************************************************************************
* Function Name  : ADS1118_Init
* Description    : 配置ADS1118 SPI接口，SPI2
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADS1118_Init(void)
{
	    GPIO_InitTypeDef GPIO_InitStructure;
        SPI_InitTypeDef  SPI_InitStructure;
	
		/**************ADS1118 CS Pin Config ***************************************/
		RCC_AHBPeriphClockCmd(ADS1118_CS_GPIO_CLK, ENABLE);	
		
		/* Configure  LED0 pin in output pushpull mode */
		GPIO_InitStructure.GPIO_Pin = ADS1118_CS_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(ADS1118_CS_GPIO_PORT, &GPIO_InitStructure);
	
		GPIO_SetBits(ADS1118_CS_GPIO_PORT,ADS1118_CS_PIN);
	
        /**************ADS1118 SPI  config *****************************************/
        /* Enable the SPI peripheral */
        RCC_APB1PeriphClockCmd(RCC_APBPeriph_ADS118_SPI, ENABLE);

        /* Enable OLED SPI  SCK, MOSI, MISO and NSS GPIO  clocks */
        RCC_AHBPeriphClockCmd(ADS1118_SPI_SCK_GPIO_CLK | ADS1118_SPI_MOSI_GPIO_CLK  
                                                   | ADS1118_SPI_MISO_GPIO_CLK,ENABLE);

        /* SPI pin mappings */
        GPIO_PinAFConfig(ADS1118_SPI_MISO_GPIO_PORT,   ADS1118_SPI_MISO_SOURCE,  ADS1118_SPI_MISO_AF);
        GPIO_PinAFConfig(ADS1118_SPI_MOSI_GPIO_PORT,  ADS1118_SPI_MOSI_SOURCE, ADS1118_SPI_MOSI_AF);
        GPIO_PinAFConfig(ADS1118_SPI_SCK_GPIO_PORT,   ADS1118_SPI_SCK_SOURCE,  ADS1118_SPI_SCK_AF);

        /* OLED SPI pin config */
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;

        /* OLED SPI SCK pin configuration */
        GPIO_InitStructure.GPIO_Pin = ADS1118_SPI_SCK_PIN;
        GPIO_Init(ADS1118_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

        /* OLED SPI  MOSI pin configuration */
        GPIO_InitStructure.GPIO_Pin =  ADS1118_SPI_MOSI_PIN;
        GPIO_Init(ADS1118_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

        /* OLED SPI MISO pin configuration */ 
        GPIO_InitStructure.GPIO_Pin = ADS1118_SPI_MISO_PIN;
        GPIO_Init(ADS1118_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);
		

        /* SPI configuration -------------------------------------------------------*/
        SPI_I2S_DeInit(ADS1118_SPI);
        SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
        SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
        SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;   //（极性再确认下）
        SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32; // (8M/32 =250KHz)
        SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
        SPI_InitStructure.SPI_CRCPolynomial = 7;

        SPI_Init(ADS1118_SPI, &SPI_InitStructure);

        /* Disable the SPI peripheral */
        SPI_Cmd(ADS1118_SPI, DISABLE);
}
/*******************************************************************************
* Function Name  : ADS1118_DeInit
* Description    : 配置ADS1118 SPI接口为AN模式
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADS1118_DeInit(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;

		/* Disable the SPI peripheral */
        SPI_Cmd(ADS1118_SPI, DISABLE);
	
		/**************ADS1118 CS Pin Config ***************************************/
		RCC_AHBPeriphClockCmd(ADS1118_CS_GPIO_CLK, ENABLE);	
		
		/* Configure  ADS1118 CS pin in output pushpull mode */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	
		/* ADS1118 SPI CS pin configuration */
		GPIO_InitStructure.GPIO_Pin = ADS1118_CS_PIN;
		GPIO_Init(ADS1118_CS_GPIO_PORT, &GPIO_InitStructure);
	
		GPIO_SetBits(ADS1118_CS_GPIO_PORT,ADS1118_CS_PIN);
	
		/* Configure  ADS1118 CS pin in output pushpull mode */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		
        /**************ADS1118 SPI  config *****************************************/
        /* Enable OLED SPI  SCK, MOSI, MISO and NSS GPIO  clocks */
        RCC_AHBPeriphClockCmd(ADS1118_SPI_SCK_GPIO_CLK | ADS1118_SPI_MOSI_GPIO_CLK  
                                                   | ADS1118_SPI_MISO_GPIO_CLK,ENABLE);
		
        /* ADS1118 SPI SCK pin configuration */
        GPIO_InitStructure.GPIO_Pin = ADS1118_SPI_SCK_PIN;
        GPIO_Init(ADS1118_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

        /* ADS1118 SPI  MOSI pin configuration */
        GPIO_InitStructure.GPIO_Pin =  ADS1118_SPI_MOSI_PIN;
        GPIO_Init(ADS1118_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

        /* ADS1118 SPI MISO pin configuration */ 
        GPIO_InitStructure.GPIO_Pin = ADS1118_SPI_MISO_PIN;
        GPIO_Init(ADS1118_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);
		
		OLED_MXS8475SPIConfig();   /* ADS1118与OLED共用SPI接口，ADS1118使用后Init OLED SPI*/
}
/*******************************************************************************
* Function Name  : ADS1118_GetVal
* Description    : 获取ADS1118转换结果,并启动一次转换
* Input          : 控制命令
* Output         : None
* Return         : None
*******************************************************************************/
bool  ADS1118_GetVal(uint16_t CMD, int16_t * pDatRe)
{
		uint8_t   CMD_H=0;
		uint8_t   CMD_L=0;
		uint8_t   SPI_Rx_Buf[2]={0};
	
		CMD_H = (uint8_t)(0x00FF & (CMD >> 8));
		CMD_L = (uint8_t)(0x00FF & CMD);
	
		SPI_Cmd(ADS1118_SPI, ENABLE);
		SPI_I2S_ReceiveData(ADS1118_SPI);

		GPIO_ResetBits(ADS1118_CS_GPIO_PORT,ADS1118_CS_PIN);

		Delay_ms(2);

		SPI_I2S_SendData(ADS1118_SPI, CMD_H); //写ADS1115控制寄存器高8位

		if(SPI_Status_Wait(ADS1118_SPI,SPI_I2S_FLAG_RXNE,SET))
		{
			return(FALSE);
		}


		SPI_Rx_Buf[0] = SPI_I2S_ReceiveData(ADS1118_SPI); //读ADS1118转换寄存器高8位
		
		if(SPI_Status_Wait(ADS1118_SPI,SPI_I2S_FLAG_TXE,SET))
		{
			return(FALSE);
		}
		
		
		SPI_I2S_SendData(ADS1118_SPI, CMD_L); //写ADS1115控制寄存器低8位
		
		if(SPI_Status_Wait(ADS1118_SPI,SPI_I2S_FLAG_RXNE,SET))
		{
			return(FALSE);
		}

		SPI_Rx_Buf[1] = SPI_I2S_ReceiveData(ADS1118_SPI); //读ADS1118转换寄存器低8位
		if(SPI_Status_Wait(ADS1118_SPI,SPI_I2S_FLAG_TXE,SET))
		{
			return(FALSE);
		}
		
		SPI_Cmd(ADS1118_SPI, DISABLE); 
		GPIO_SetBits(ADS1118_CS_GPIO_PORT,ADS1118_CS_PIN);
		
		(*pDatRe) = (SPI_Rx_Buf[0] << 8) | SPI_Rx_Buf[1];
		
        return(TRUE); 
}













