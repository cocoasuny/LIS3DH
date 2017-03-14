/******************** (C) COPYRIGHT 2014-2024,iCareTech.Co.,Ltd. ********************
 * File Name��ADS1115.c
 * Author: Jason  
 * Version: 1.0   
 * Date: 2014/3/4
 * Desciption: ������ϸ˵���˳����ļ���ɵ���Ҫ���ܣ�������ģ������Ľӿڣ���ʹ�÷���
 *             ���磺ADS1115.c��Ҫʵ��ADC ADS1115������������ʵ���˶���ADS1115�Ĵ����Ķ�д��
 *             ADS1115����ģʽ�����á�ѡ��ADS1115ת��ͨ��������ADS1115ת���� ��ȡת������ȡ�
 *Function List:
 *             1��void ADS1115_GPIO_Config(void):����ADS1115 GPIO
 *             2��void ADS1115_IIC_Config(void): ����MCU��ADS1115 IIC�ӿ�
 *             3��Function 3: *************
 *             4��Function 4: *************
 *History:
 *       <Author>      <Time>       <Version>     <Desciption>
 *        Jason        2014/3/4         1.0           Creat
**********************************************************************************/

#include "stm32l1xx.h"
#include "OLED_MXS8475.h"
#include "font.h"
#include "Timer.h"
#include "Usart.h"
#include "platform.h"
#include "PowerManage.h"



/*******************************************************************************
* Function Name  : OLED_CtrlLinesConfig
* Description    : ��ʼ��OLED GPIO A0\Rest
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_MX8475CtrlLinesConfig(void)
{
//        GPIO_InitTypeDef        OLED_GPIOInitStructure;
//	
//		/* Config OLEDA0 in output mode */
//        RCC_AHBPeriphClockCmd(RCC_APBPeriph_OLED_MXS8475A0,ENABLE);

//        OLED_GPIOInitStructure.GPIO_Pin = GPIO_Pin_OLED_MXS8475A0;
//        OLED_GPIOInitStructure.GPIO_Mode = GPIO_Mode_OUT;
//        OLED_GPIOInitStructure.GPIO_OType = GPIO_OType_PP;
//        OLED_GPIOInitStructure.GPIO_Speed = GPIO_Speed_40MHz;
//        OLED_GPIOInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//        
//        GPIO_Init(GPIO_OLED_MXS8475A0, &OLED_GPIOInitStructure);
//		
//		/* Config OLEDRest GPIO in output mode */
//		RCC_AHBPeriphClockCmd(RCC_APBPeriph_OLED_MXS8475Rest,ENABLE);

//        OLED_GPIOInitStructure.GPIO_Pin = GPIO_Pin_OLED_MXS8475Rest;
//        OLED_GPIOInitStructure.GPIO_Mode = GPIO_Mode_OUT;
//        OLED_GPIOInitStructure.GPIO_OType = GPIO_OType_PP;
//        OLED_GPIOInitStructure.GPIO_Speed = GPIO_Speed_40MHz;
//        OLED_GPIOInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//        
//        GPIO_Init(GPIO_OLED_MXS8475Rest, &OLED_GPIOInitStructure);
//		
//		/* Config OLED power\A0\Rest default state */
//		
//		OLED_DriveSystemPower(OFF);   //Shut down OLED 15V 
//        GPIO_SetBits(GPIO_OLED_MXS8475A0,GPIO_Pin_OLED_MXS8475A0);
//        GPIO_ResetBits(GPIO_OLED_MXS8475Rest,GPIO_Pin_OLED_MXS8475Rest);
}
/*******************************************************************************
* Function Name  : OLED_MX8475CtrlLinesDeConfig
* Description    : DeInit OLED GPIO A0\Rest
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_MX8475CtrlLinesDeConfig(void)
{
//        GPIO_InitTypeDef        OLED_GPIOInitStructure;

//		GPIO_ResetBits(GPIO_OLED_MXS8475Power,GPIO_Pin_OLED_MXS8475Power); //Ĭ�Ϲر�15V��Դ	

//		OLED_GPIOInitStructure.GPIO_Pin = GPIO_Pin_OLED_MXS8475Power;
//        OLED_GPIOInitStructure.GPIO_Mode = GPIO_Mode_AN;
//        OLED_GPIOInitStructure.GPIO_OType = GPIO_OType_PP;
//		OLED_GPIOInitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL; 
//        OLED_GPIOInitStructure.GPIO_Speed = GPIO_Speed_400KHz;
//        
//        GPIO_Init(GPIO_OLED_MXS8475Power, &OLED_GPIOInitStructure);
//	
//		/* Config OLEDA0 in AN mode */
//        OLED_GPIOInitStructure.GPIO_Pin = GPIO_Pin_OLED_MXS8475A0; 
//        OLED_GPIOInitStructure.GPIO_Mode = GPIO_Mode_AN;
//        OLED_GPIOInitStructure.GPIO_OType = GPIO_OType_PP;
//		OLED_GPIOInitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL; 
//        OLED_GPIOInitStructure.GPIO_Speed = GPIO_Speed_400KHz;
//        
//        GPIO_Init(GPIO_OLED_MXS8475A0, &OLED_GPIOInitStructure);
//		
//		/* Config OLEDRest GPIO in ANput mode */
//        OLED_GPIOInitStructure.GPIO_Pin = GPIO_Pin_OLED_MXS8475Rest;
//        OLED_GPIOInitStructure.GPIO_Mode = GPIO_Mode_AN;
//        OLED_GPIOInitStructure.GPIO_OType = GPIO_OType_PP;
//		OLED_GPIOInitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL; 
//        OLED_GPIOInitStructure.GPIO_Speed = GPIO_Speed_400KHz;
//        
//        GPIO_Init(GPIO_OLED_MXS8475Rest, &OLED_GPIOInitStructure);
}
/*******************************************************************************
* Function Name  : OLED_MXS8475GPIOSPIConfig
* Description    : ����GPIO��ģ��SPI����OLED
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_MXS8475GPIOSPIConfig(void)
{
//        GPIO_InitTypeDef        OLED_GPIOInitStructure;

//        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD,ENABLE);
//	    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
//	    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC,ENABLE);

//        /* Configure OLEDA0\OLEDRest in  output mode,����д���������*/
//        OLED_GPIOInitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_7;
//        OLED_GPIOInitStructure.GPIO_Mode = GPIO_Mode_OUT;
//        OLED_GPIOInitStructure.GPIO_OType = GPIO_OType_PP;
//        OLED_GPIOInitStructure.GPIO_Speed = GPIO_Speed_40MHz;
//        OLED_GPIOInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//        
//        GPIO_Init(GPIOD, &OLED_GPIOInitStructure);


//        /* Configure OLEDA0\OLEDRest in  output mode,����д���������*/
//        OLED_GPIOInitStructure.GPIO_Pin = GPIO_Pin_4;
//        OLED_GPIOInitStructure.GPIO_Mode = GPIO_Mode_OUT;
//        OLED_GPIOInitStructure.GPIO_OType = GPIO_OType_PP;
//        OLED_GPIOInitStructure.GPIO_Speed = GPIO_Speed_40MHz;
//        OLED_GPIOInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//        
//        GPIO_Init(GPIOA, &OLED_GPIOInitStructure);
//		
//        /* Configure OLEDA0\OLEDRest in  output mode,����д���������*/
//        OLED_GPIOInitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 |GPIO_Pin_12;
//        OLED_GPIOInitStructure.GPIO_Mode = GPIO_Mode_OUT;
//        OLED_GPIOInitStructure.GPIO_OType = GPIO_OType_PP;
//        OLED_GPIOInitStructure.GPIO_Speed = GPIO_Speed_40MHz;
//        OLED_GPIOInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//        
//        GPIO_Init(GPIOC, &OLED_GPIOInitStructure);  
}
/*******************************************************************************
* Function Name  : OLED_SPIConfig
* Description    : ��ʼ��OLED_SPIConfigģ�飬����Ӳ��SPIģ������OLED
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_MXS8475SPIConfig(void)
{
//        GPIO_InitTypeDef GPIO_InitStructure;
//        SPI_InitTypeDef  SPI_InitStructure;

//        /**************OLED SPI  config *****************************************/
//        /* Enable the SPI peripheral */
//        RCC_APB1PeriphClockCmd(RCC_APBPeriph_OLED_MXS8475SPI, ENABLE);

//        /* Enable OLED SPI  SCK, MOSI, MISO and NSS GPIO  clocks */
//        RCC_AHBPeriphClockCmd(OLED_MXS8475SPI_SCK_GPIO_CLK | OLED_MXS8475SPI_MOSI_GPIO_CLK  
//                                                   | OLED_MXS8475SPI_NSS_GPIO_CLK,ENABLE);

//        /* SPI pin mappings */
//        GPIO_PinAFConfig(OLED_MXS8475_SPI_NSS_GPIO_PORT,   OLED_MXS8475_SPI_NSS_SOURCE,  OLED_MXS8475_SPI_NSS_AF);
//        GPIO_PinAFConfig(OLED_MXS8475_SPI_MOSI_GPIO_PORT,  OLED_MXS8475_SPI_MOSI_SOURCE, OLED_MXS8475_SPI_MOSI_AF);
//        GPIO_PinAFConfig(OLED_MXS8475_SPI_SCK_GPIO_PORT,   OLED_MXS8475_SPI_SCK_SOURCE,  OLED_MXS8475_SPI_SCK_AF);

//        /* OLED SPI pin config */
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
//        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;

//        /* OLED SPI SCK pin configuration */
//        GPIO_InitStructure.GPIO_Pin = OLED_MXS8475_SPI_SCK_PIN;
//        GPIO_Init(OLED_MXS8475_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

//        /* OLED SPI  MOSI pin configuration */
//        GPIO_InitStructure.GPIO_Pin =  OLED_MXS8475_SPI_MOSI_PIN;
//        GPIO_Init(OLED_MXS8475_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

//        /* OLED SPI NSS pin configuration */ 
//        GPIO_InitStructure.GPIO_Pin = OLED_MXS8475_SPI_NSS_PIN;
//        GPIO_Init(OLED_MXS8475_SPI_NSS_GPIO_PORT, &GPIO_InitStructure);

//        /* SPI configuration -------------------------------------------------------*/
//        SPI_I2S_DeInit(OLED_MXS8475SPI);
//        SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
//        SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
//        SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
//        SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
//        SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;   //��������ȷ���£�
//        SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
//        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; // (8M/32 =250KHz)
//        SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
//        SPI_InitStructure.SPI_CRCPolynomial = 7;

//        SPI_Init(OLED_MXS8475SPI, &SPI_InitStructure);

//        /* Enable the SPI peripheral */
//        SPI_Cmd(OLED_MXS8475SPI, DISABLE);
//        SPI_SSOutputCmd(OLED_MXS8475SPI, ENABLE);
}
/*******************************************************************************
* Function Name  : OLED_MXS8475SPIDeConfig
* Description    : Deinit OLED_SPIConfigģ�飬����Ӳ��SPIģ������OLED
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_MXS8475SPIDeConfig(void)
{
//        GPIO_InitTypeDef GPIO_InitStructure;

//        /**************OLED SPI  config *****************************************/
//		        /* Enable the SPI peripheral */
//        SPI_Cmd(OLED_MXS8475SPI, DISABLE);
//        SPI_SSOutputCmd(OLED_MXS8475SPI, DISABLE);
//	
//        /* OLED SPI pin config */
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
//        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL; 
//        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;

//        /* OLED SPI SCK pin configuration */
//        GPIO_InitStructure.GPIO_Pin = OLED_MXS8475_SPI_SCK_PIN;
//        GPIO_Init(OLED_MXS8475_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

//        /* OLED SPI  MOSI pin configuration */
//        GPIO_InitStructure.GPIO_Pin =  OLED_MXS8475_SPI_MOSI_PIN;
//        GPIO_Init(OLED_MXS8475_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

//        /* OLED SPI NSS pin configuration */ 
//        GPIO_InitStructure.GPIO_Pin = OLED_MXS8475_SPI_NSS_PIN;
//        GPIO_Init(OLED_MXS8475_SPI_NSS_GPIO_PORT, &GPIO_InitStructure);
}
/*******************************************************************************
* Function Name  : MXS8475_DefConfig
* Description    : MXS8475Ĭ������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MXS8475_DefConfig(void)
{
//		/* 15V Power On */

//		GPIO_SetBits(GPIO_OLED_MXS8475Power,GPIO_Pin_OLED_MXS8475Power); //����15V��Դ

//        GPIO_SetBits(GPIO_OLED_MXS8475A0,GPIO_Pin_OLED_MXS8475A0);
//        GPIO_SetBits(GPIO_OLED_MXS8475Rest,GPIO_Pin_OLED_MXS8475Rest);
//        GPIO_ResetBits(GPIO_OLED_MXS8475Rest,GPIO_Pin_OLED_MXS8475Rest);     //��λMXS8475
//        Delay_us(1);
//        GPIO_SetBits(GPIO_OLED_MXS8475Rest,GPIO_Pin_OLED_MXS8475Rest);

//        MXS8475_WriteReg(0x01,0);   //�����λ
//        MXS8475_WriteReg(0x02,0x00);
//        MXS8475_WriteReg(0x07,0x00);
//        MXS8475_WriteReg(0x09,0x00);
//        MXS8475_WriteReg(0x10,0x04);
//        MXS8475_WriteReg(0x12,0xFF);
//        MXS8475_WriteReg(0x13,0x00);
//        MXS8475_WriteReg(0x14,0x00);
//        MXS8475_WriteReg(0x16,0x00);
//        MXS8475_WriteReg(0x17,0x00);
//        MXS8475_WriteReg(0x18,0x04);
//        MXS8475_WriteReg(0x1A,0x01);
//        MXS8475_WriteReg(0x1C,0x00);
//        MXS8475_WriteReg(0x1D,0x00);
//        MXS8475_WriteRegEx(0x30,0x10,0x6F);
//        MXS8475_WriteRegEx(0x32,0x00,0x26);
//        MXS8475_WriteReg(0x34,0x00);
//        MXS8475_WriteReg(0x35,0x0B);
//        MXS8475_WriteReg(0x36,0x00);
//        MXS8475_WriteReg(0x37,0x26);
//        MXS8475_WriteReg(0x38,0x70);
//        MXS8475_WriteReg(0x39,0x00);
//        MXS8475_WriteReg(0x48,0x03);
//        MXS8475_WriteReg(0xC3,0x00);
//        MXS8475_WriteReg(0xC4,0x00);
//        MXS8475_WriteReg(0xCC,0x00);
//        MXS8475_WriteReg(0xCD,0x00);
//        MXS8475_WriteReg(0xD0,0x80);
//        MXS8475_WriteReg(0xD2,0x00);
//        MXS8475_WriteReg(0xD9,0x00);
//        MXS8475_WriteReg(0xDB,0x04);  //���ȵ���
//        MXS8475_WriteReg(0xDD,0x86); 
}
/*******************************************************************************
* Function Name  : OLED_SPISendChar
* Description    : ����OLED SPI ����һ���ַ���ʹ��ʱ����Ҫ�ȵ���
*                  SPI_Cmd(OLEDSPI, ENABLE);�������֮����Ҫ����
*                  SPI_Cmd(OLEDSPI, DISABLE);�Ա����NSS�ź�
* Input          : ���͵��ַ�
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_SPISendChar(unsigned char ch)
{
        SPI_I2S_SendData(OLED_MXS8475SPI,ch);
	
		SPI_Status_Wait(OLED_MXS8475SPI,SPI_I2S_FLAG_TXE,SET);
	
		SPI_I2S_ClearFlag(OLED_MXS8475SPI, SPI_I2S_FLAG_TXE);       
}
/*******************************************************************************
* Function Name  : MXS8475_WriteReg
* Description    : дMXS8475�Ĵ���
* Input          : д�Ĵ�����ַ��д�Ĵ�����
* Output         : None
* Return         : None
*******************************************************************************/
void MXS8475_WriteReg(unsigned char reg , unsigned char dat )
{
//#if  1  //1��ʹ��Ӳ��SPI��0��ʹ��GPIOģ��SPI
//        SPI_Cmd(OLED_MXS8475SPI, ENABLE);

//        GPIO_ResetBits(GPIO_OLED_MXS8475A0,GPIO_Pin_OLED_MXS8475A0);   //д����
//        OLED_SPISendChar(reg);
//		
////        Delay_us(20);     //��Ҫ����SPIд���ݵ��ٶ����޸�

//        GPIO_SetBits(GPIO_OLED_MXS8475A0,GPIO_Pin_OLED_MXS8475A0);   //д����
//        OLED_SPISendChar(dat);	
////        Delay_us(20);

//        SPI_Cmd(OLED_MXS8475SPI, DISABLE);
//#else
//        OLED_GPIOWriteReg(reg,1);
//        //Delay_us(5);
//        OLED_GPIOWriteReg(dat,0);
//        //Delay_us(5);
//#endif
        
}
/*******************************************************************************
* Function Name  : MXS8475_WriteRegEx
* Description    : MXS8475һ���Ĵ�����д�������
* Input          : д�Ĵ�����ַ��д�Ĵ�����
* Output         : None
* Return         : None
*******************************************************************************/
void MXS8475_WriteRegEx(unsigned char reg , unsigned char dat ,unsigned char Num)
{
//#if  1  //1��ʹ��Ӳ��SPI��0��ʹ��GPIOģ��SPI
//        SPI_Cmd(OLED_MXS8475SPI, ENABLE);

//        GPIO_ResetBits(GPIO_OLED_MXS8475A0,GPIO_Pin_OLED_MXS8475A0);   //д����
//        OLED_SPISendChar(reg);
//        Delay_us(20);      //��Ҫ����SPIд���ݵ��ٶ����޸�

//        GPIO_SetBits(GPIO_OLED_MXS8475A0,GPIO_Pin_OLED_MXS8475A0);   //д����
//        OLED_SPISendChar(dat);
//        Delay_us(20);      //��Ҫ����SPIд���ݵ��ٶ����޸�

//        OLED_SPISendChar(Num);
//        Delay_us(20);

//        SPI_Cmd(OLED_MXS8475SPI, DISABLE);
//#else
//        OLED_GPIOWriteReg(reg,1);
//        Delay_us(5);
//        OLED_GPIOWriteReg(dat,0);
//        Delay_us(5);
//        OLED_GPIOWriteReg(Num,0);
//        Delay_us(5);
//#endif
}
/*******************************************************************************
* Function Name  : MXS8475_WriteCMD
* Description    : дMXS8475�Ĵ���
* Input          : д�Ĵ�����
* Output         : None
* Return         : None
*******************************************************************************/
void MXS8475_WriteCMD(unsigned char cmd )
{
//#if  1  //1��ʹ��Ӳ��SPI;0:ʹ��GPIOģ��SPI
//        SPI_Cmd(OLED_MXS8475SPI, ENABLE);

//        GPIO_ResetBits(GPIO_OLED_MXS8475A0,GPIO_Pin_OLED_MXS8475A0);   //д����
//        OLED_SPISendChar(cmd);
//        Delay_us(20);     //��Ҫ����SPIд���ݵ��ٶ����޸�

//        SPI_Cmd(OLED_MXS8475SPI, DISABLE);
//#else
//        OLED_GPIOWriteReg(cmd,1);
//#endif
}
/*******************************************************************************
* Function Name  : MXS8475_WriteDat
* Description    : дMXS8475�Ĵ���
* Input          : д�Ĵ�����ַ��д�Ĵ�����
* Output         : None
* Return         : None
*******************************************************************************/
void MXS8475_WriteDat(uint8_t dat)
{
//#if  1  //1��ʹ��Ӳ��SPI;0:ʹ��GPIOģ��SPI
//        SPI_Cmd(OLED_MXS8475SPI, ENABLE);

//        GPIO_SetBits(GPIO_OLED_MXS8475A0,GPIO_Pin_OLED_MXS8475A0);   //д����
//        OLED_SPISendChar(dat);
//        //Delay_us(1);

//        SPI_Cmd(OLED_MXS8475SPI, DISABLE);
//#else
//        OLED_GPIOWriteReg(dat,0);
//#endif
}

/*******************************************************************************
* Function Name  : OLED_DisplayChar16*16
* Description    : OLED��ʾ8*16�ַ�
* Input          : XS:��ʼ�У�YS����ʼ��
* Output         : None
* Return         : None
*******************************************************************************/
//void OLED_MXS8475_DisplayChar(unsigned char XS,unsigned char YS,unsigned char ch)
//{
//        unsigned char i=0;
//		unsigned char XE=0;
//		unsigned char YE=0;
//        MXS8475_WriteReg(0x1D,4);

//		XE = XS + 8;
//		YE = YS + 16;
//		MXS8475_WriteReg(0x34,XS);
//        MXS8475_WriteReg(0x35,XE);
//        MXS8475_WriteReg(0x36,YS);
//        MXS8475_WriteReg(0x37,YE);
//		

//        MXS8475_WriteCMD(0x08);
//			
//        for(i =0;i<16;i++)
//		{
//			MXS8475_WriteDat(Ascii8X16[ch*16+i]);			
//		}
//        MXS8475_WriteReg(0x02,0x01);
//}

//display a pitrue
void MXS8475_displayPitrue(unsigned char xBoxStart,unsigned char xBoxEnd,unsigned char yBoxStart,unsigned char yBoxEnd,const unsigned char * pitrueData,unsigned int dataLeng)
{
	unsigned short i = 0;
	
	OLED_Clear(0,11,0,31,0x00);
	
	MXS8475_WriteReg(0x02,0x00);
	MXS8475_WriteReg(0x1D,0x08);	
	
	MXS8475_WriteReg(0x34,xBoxStart);			  			 	  
	MXS8475_WriteReg(0x35,xBoxEnd);	
	MXS8475_WriteReg(0x36,yBoxStart);					
	MXS8475_WriteReg(0x37,yBoxEnd);		

	MXS8475_WriteCMD(0x08);	
  

    for(i = 0;i < dataLeng;i++)
	{
		MXS8475_WriteDat(*pitrueData);
		pitrueData++;
	}
		
	MXS8475_WriteReg(0x02,0x01);	
}
void OLED_Clear(unsigned char x,unsigned char xx,unsigned char y,unsigned char yy,unsigned char liang)
{
	int ii = 0;
	unsigned int iii = 0;
	if(xx==15)
	{
		iii = 624;
	}
	else if(xx==11)
	{
		iii = 468;
	}
	if(OFF == GetPeriphPowerStatus())  //�����ⲿ��Դ
	{
		PeriPower(ON);
	}
	if(Get_OLED_Config_Status() == DISABLE)
	{
		OLED_Configuration();
	}
	
	MXS8475_WriteReg(0x02,0x00);
	MXS8475_WriteReg(0x1D,0x08);
	MXS8475_WriteReg(0x34,x);
	MXS8475_WriteReg(0x35,xx);
	MXS8475_WriteReg(0x36,y);
	MXS8475_WriteReg(0x37,yy);
	
	MXS8475_WriteCMD(0x08);	
	
	for(ii = 0;ii<iii;ii++)
	{
		MXS8475_WriteDat(liang);
	}
	MXS8475_WriteReg(0x02,0x01);
}





























