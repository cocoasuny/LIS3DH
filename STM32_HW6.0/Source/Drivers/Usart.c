#include "stm32l1xx.h"
#include "Usart.h"
#include "platform.h"
/*******************************************************************************
* Function Name  : USART_Configuration
* Description    : Uasrt Config
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
	
	#ifdef BOARD_REDHARE_V3_0
		/* config USART3 clock */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 , ENABLE);
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD , ENABLE);
	  
		GPIO_PinAFConfig(GPIOD ,GPIO_PinSource8,GPIO_AF_USART3);

		/* USART1 GPIO config */
		/* Configure USART3 Tx (PD.08) as alternate function push-pull */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		
		GPIO_Init(GPIOD, &GPIO_InitStructure); 
		
		/* USART3 mode config */
		USART_InitStructure.USART_BaudRate = 115200;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Tx;
		USART_Init(USART3, &USART_InitStructure); 
		
		USART_Cmd(USART3, ENABLE);
	#endif
}
/*******************************************************************************
* Function Name  : USART_DeConfiguration
* Description    : Uasrt Config
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART_DeConfiguration(void)
{
	GPIO_InitTypeDef        GPIO_InitStructure;
	
	USART_Cmd(USART3, DISABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}
/*******************************************************************************
* Function Name  : USART_DeConfiguration
* Description    : Uasrt Config
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART_GPIOConfiguration(void)
{
	GPIO_InitTypeDef        GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}
/*******************************************************************************
* Function Name  : fputc
* Description    : 映射Printf
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int fputc(int ch, FILE *f)
{
	uint32_t  TimeCnt=0;
	
#ifdef BOARD_REDHARE_V3_0
	USART_SendData(USART3, (unsigned char) ch);
	TimeCnt = 16000;
	while ((!(USART3->SR & USART_FLAG_TXE)) && (TimeCnt != 0))
	{
		TimeCnt--;
	}
    return (ch);
#endif
}

void Usart_SendStrings(uint8_t *str)
{
		uint8_t ch;
		uint8_t i=0;
		ch=str[0];
		while(ch != '\0')
		{
			ch = str[i++];
			#ifdef BOARD_REDHARE_V3_0
				USART_SendData(USART3,ch);
				while (!(USART3->SR & USART_FLAG_TXE));
			#endif
		}
}
void Usart_SendChar(uint8_t ch)
{
	uint8_t retry=0;
	
	USART_SendData(USART3,ch);
	while (!(USART3->SR & USART_FLAG_TXE))
	{
		retry++;
		if(retry>250)break;
	}
}
/*******************************************************************************
* Function Name  : SelfCheckLCDCtl1
* Description    : 控制Test LCD显示结果
* Input          : ID，CMD
* Output         : None
* Return         : None
*******************************************************************************/
void SelfCheckLCDCtl(uint8_t ID ,uint8_t CMD)
{

	Usart_SendChar(0xEE);
	Usart_SendChar(0xB1);
	Usart_SendChar(0x23);
	Usart_SendChar(0x00);
	Usart_SendChar(0x00);
	Usart_SendChar(0x00);
	Usart_SendChar(ID);
	Usart_SendChar(CMD);
	Usart_SendChar(0xFF);
	Usart_SendChar(0xFC);
	Usart_SendChar(0xFF);
	Usart_SendChar(0xFF);	
}
/*******************************************************************************
* Function Name  : SelfCheckLCDCtl1
* Description    : 控制Test LCD显示结果
* Input          : ID，CMD
* Output         : None
* Return         : None
*******************************************************************************/
void SelfCheck_TextUpdate(uint8_t ID,uint8_t *ptext,uint8_t len)
{
	uint8_t i=0;
	
	Usart_SendChar(0xEE);
	Usart_SendChar(0xB1);
	Usart_SendChar(0x10);
	Usart_SendChar(0x00);
	Usart_SendChar(0x00);
	Usart_SendChar(0x00);
	
	Usart_SendChar(ID);
	
	for(i=0;i<len;i++)
	{
		Usart_SendChar(ptext[i]);
	}
	
	
	Usart_SendChar(0xFF);
	Usart_SendChar(0xFC);
	Usart_SendChar(0xFF);
	Usart_SendChar(0xFF);	
}
