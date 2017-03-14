#include "stm32l1xx.h"
#include "OLED_UG2832.h"
#include "Timer.h"
#include "Font.h"

#define     XLevelL		    0x00
#define     XLevelH		    0x10
#define     XLevel		    ((XLevelH&0x0F)*16+XLevelL)
#define     Max_Column	    128
#define     Max_Row		    64
#define	    Brightness	    0xCF



#define     X_WIDTH         128
#define     Y_WIDTH         64
#define		Page			8


/*******************************************************************************
* Function Name  : OLED_CtrlLinesConfig
* Description    : 初始化OLED GPIO A0\Rest
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_UG2832CtrlLinesConfig(void)
{
        GPIO_InitTypeDef        OLED_GPIOInitStructure;

        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD,ENABLE);

        /* Configure OLEDA0\OLEDRest in  output mode,控制写命令或数据*/
        OLED_GPIOInitStructure.GPIO_Pin = GPIO_Pin_OLED_UG2832A0 | GPIO_Pin_OLED_UG2832Rest;
        OLED_GPIOInitStructure.GPIO_Mode = GPIO_Mode_OUT;
        OLED_GPIOInitStructure.GPIO_OType = GPIO_OType_PP;
        OLED_GPIOInitStructure.GPIO_Speed = GPIO_Speed_40MHz;
        OLED_GPIOInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

        GPIO_Init(GPIO_OLED_UG2832, &OLED_GPIOInitStructure);

        GPIO_SetBits(GPIO_OLED_UG2832,GPIO_Pin_OLED_UG2832A0);
        GPIO_SetBits(GPIO_OLED_UG2832,GPIO_Pin_OLED_UG2832Rest);
}
/*******************************************************************************
* Function Name  : OLED_SPIConfig
* Description    : 初始化OLED_SPIConfig模块，采用硬件SPI模块驱动OLED
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_UG2832SPIConfig(void)
{
        GPIO_InitTypeDef GPIO_InitStructure;
        SPI_InitTypeDef  SPI_InitStructure;

        /**************OLED SPI  config *****************************************/
        /* Enable the SPI peripheral */
        RCC_APB1PeriphClockCmd(RCC_APBPeriph_OLED_UG2832SPI, ENABLE);

        /* Enable OLED SPI  SCK, MOSI, MISO and NSS GPIO  clocks */
        RCC_AHBPeriphClockCmd(OLED_UG2832SPI_SCK_GPIO_CLK | OLED_UG2832SPI_MOSI_GPIO_CLK | OLED_UG2832SPI_MISO_GPIO_CLK
                                                | OLED_UG2832SPI_NSS_GPIO_CLK,ENABLE);

        /* SPI pin mappings */
        GPIO_PinAFConfig(OLED_UG2832_SPI_NSS_GPIO_PORT,   OLED_UG2832_SPI_NSS_SOURCE,  OLED_UG2832_SPI_NSS_AF);
        GPIO_PinAFConfig(OLED_UG2832_SPI_MOSI_GPIO_PORT, OLED_UG2832_SPI_MOSI_SOURCE, OLED_UG2832_SPI_MOSI_AF);
        GPIO_PinAFConfig(OLED_UG2832_SPI_MISO_GPIO_PORT, OLED_UG2832_SPI_MISO_SOURCE, OLED_UG2832_SPI_MISO_AF);
        GPIO_PinAFConfig(OLED_UG2832_SPI_SCK_GPIO_PORT,   OLED_UG2832_SPI_SCK_SOURCE,   OLED_UG2832_SPI_SCK_AF);

        /* OLED SPI pin config */
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;

        /* OLED SPI SCK pin configuration */
        GPIO_InitStructure.GPIO_Pin = OLED_UG2832_SPI_SCK_PIN;
        GPIO_Init(OLED_UG2832_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

        /* OLED SPI  MOSI pin configuration */
        GPIO_InitStructure.GPIO_Pin =  OLED_UG2832_SPI_MOSI_PIN;
        GPIO_Init(OLED_UG2832_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

        /* OLED SPI MISO pin configuration */
        GPIO_InitStructure.GPIO_Pin = OLED_UG2832_SPI_MISO_PIN;
        GPIO_Init(OLED_UG2832_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

        /* OLED SPI NSS pin configuration */
        GPIO_InitStructure.GPIO_Pin = OLED_UG2832_SPI_NSS_PIN;
        GPIO_Init(OLED_UG2832_SPI_NSS_GPIO_PORT, &GPIO_InitStructure);

        /* SPI configuration -------------------------------------------------------*/
        SPI_I2S_DeInit(OLED_MXS8475SPI);
        SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
        SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
        SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;   //（极性再确认下）
        SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; // (8M/32 =250KHz)
        SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
        SPI_InitStructure.SPI_CRCPolynomial = 7;

        SPI_Init(OLED_UG2832SPI, &SPI_InitStructure);

        /* Enable the SPI peripheral */
        SPI_Cmd(OLED_UG2832SPI, ENABLE);
        SPI_SSOutputCmd(OLED_UG2832SPI, ENABLE);
}
void OLED_UG2832_WB(uint8_t data)
{
    /* Loop while DR register in not emplty */
    //while (SPI_I2S_GetFlagStatus(OLEDSPI, SPI_I2S_FLAG_TXE) == RESET);
    /* Send byte through the SPI2 peripheral */
		SPI_I2S_SendData(OLED_UG2832SPI,data);
        while (SPI_I2S_GetFlagStatus(OLED_UG2832SPI, SPI_I2S_FLAG_TXE) == RESET);
        SPI_I2S_ClearFlag(OLED_UG2832SPI,SPI_I2S_FLAG_TXE);
}
/*******************????????***********************/
void LCD_WrDat(unsigned char dat)
{
#if  1      //1：硬件SPI；0：GPIO模拟SPI
    OLED_UG2832_DC_H;
	//SPI_Cmd(OLEDSPI, ENABLE);
	OLED_UG2832_WB(dat);
	Delay_us(1);
	//SPI_Cmd(OLEDSPI, DISABLE);
#else
	unsigned char i;
	//LCD_DC=1;
	OLED_UG2832_DC_H;
	for(i=0;i<8;i++) //发送一个八位数据
	{
		if((dat << i) & 0x80)
		{
			//LCD_SDA  = 1;
			SD_A_H;
		}
		else
		{
			//LCD_SDA  = 0;
			SD_A_L;
		}
		//LCD_SCL = 0;
		//LCD_SCL = 1;
		SC_L;
		SC_H;
	}
#endif
}

/********************??????**********************/
void LCD_WrCmd(unsigned char cmd)
{
#if  1      //1：硬件SPI；0：GPIO模拟SPI
    OLED_UG2832_DC_L;
	//SPI_Cmd(OLEDSPI, ENABLE);
    OLED_UG2832_WB(cmd);
	Delay_us(1);
	//SPI_Cmd(OLEDSPI, DISABLE);
#else
	unsigned char i;
	OLED_UG2832_DC_L;
	for(i=0;i<8;i++) //发送一个八位数据
	{
		if((cmd << i) & 0x80)
		{
			//LCD_SDA  = 1;
			SD_A_H;
		}
		else
		{
			//LCD_SDA  = 0;
			SD_A_L;
		}
		//LCD_SCL = 0;
		//LCD_SCL = 1;
		SC_L;
		SC_H;
	}
#endif
}

/*********************LCD 设置坐标************************************/
void LCD_Set_Pos(unsigned char x, unsigned char y)
{
x=x+4;
LCD_WrCmd(0xb0+y);
LCD_WrCmd(((x&0xf0)>>4)|0x10);
LCD_WrCmd((x&0x0f)|0x01);
}
/*********************LCD全屏************************************/
void LCD_Fill(unsigned char bmp_dat)
{
unsigned char y,x;
for(y=0;y<8;y++)
{
LCD_WrCmd(0xb0+y);
LCD_WrCmd(0x01);
LCD_WrCmd(0x10);
for(x=0;x<X_WIDTH;x++)
LCD_WrDat(bmp_dat);
}
}
/*********************LCD复位************************************/
void LCD_CLS(void)
{
unsigned char y,x;
for(y=0;y<8;y++)
{
//LCD_WrCmd(0xb0+y);
//LCD_WrCmd(0x01);
//LCD_WrCmd(0x10);
	LCD_Set_Pos(0,y);
	for(x=0;x<X_WIDTH;x++)
	LCD_WrDat(0);
}
}
/*********************OLED SSD1306初始化************************************/
void OLED_UG2832_Init(void)
{
	//LCD_SCL=1;
	SC_H;
	//LCD_RST=0;
	OLED_UG2832_RST_L;
	Delay_ms(50);
	//LCD_RST=1;       //从上电到下面开始初始化要有足够的时间，即等待RC复位完毕
	OLED_UG2832_RST_H;
	LCD_WrCmd(0xae);//--turn off oled panel
	LCD_WrCmd(0x00);//---set low column address
	LCD_WrCmd(0x10);//---set high column address
	LCD_WrCmd(0x40);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
	LCD_WrCmd(0x81);//--set contrast control register
	LCD_WrCmd(0x8f); // Set SEG Output Current Brightness
	LCD_WrCmd(0xa1);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
	LCD_WrCmd(0xc8);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
	LCD_WrCmd(0xa6);//--set normal display
	LCD_WrCmd(0xa8);//--set multiplex ratio(1 to 64)
	LCD_WrCmd(0x1f);//--1/32 duty
	LCD_WrCmd(0xd3);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	LCD_WrCmd(0x00);//-not offset
	LCD_WrCmd(0xd5);//--set display clock divide ratio/oscillator frequency
	LCD_WrCmd(0x80);//--set divide ratio, Set Clock as 100 Frames/Sec
	LCD_WrCmd(0xd9);//--set pre-charge period
	LCD_WrCmd(0xf1);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	LCD_WrCmd(0xda);//--set com pins hardware configuration
	LCD_WrCmd(0x02);
	LCD_WrCmd(0xdb);//--set vcomh
	LCD_WrCmd(0x40);//Set VCOM Deselect Level
	LCD_WrCmd(0x20);//-Set Page Addressing Mode (0x00/0x01/0x02)
	LCD_WrCmd(0x02);//
	LCD_WrCmd(0x8d);//--set Charge Pump enable/disable
	LCD_WrCmd(0x14);//--set(0x10) disable
	LCD_WrCmd(0xa4);// Disable Entire Display On (0xa4/0xa5)
	LCD_WrCmd(0xa6);// Disable Inverse Display On (0xa6/a7)
	LCD_WrCmd(0xaf);//--turn on oled panel
	LCD_Fill(0x00);  //初始清屏
	LCD_Set_Pos(0,0);
}

/***************功能描述：显示6*8一组标准ASCII字符串	显示的坐标（x,y），y为页范围0～7****************/
void LCD_P6x8Str(unsigned char x,unsigned char y,unsigned char ch[])
{
unsigned char c=0,i=0,j=0;
while (ch[j]!='\0')
{
c =ch[j]-32;
if(x>126){x=0;y++;}
LCD_Set_Pos(x,y);
for(i=0;i<6;i++)
LCD_WrDat(F6x8[c][i]);
x+=6;
j++;
}
}
/*******************功能描述：显示8*16一组标准ASCII字符串	 显示的坐标（x,y），y为页范围0～7****************/
void LCD_P8x16Char(unsigned char x,unsigned char y,unsigned char ch)
{
	unsigned char c=0,i=0;
	if (ch!='\0')
	{
		c =ch-32;
		if(x>120)
		{
			x=0;
			y++;
		}
		LCD_Set_Pos(x,y);
		for(i=0;i<8;i++)
		LCD_WrDat(F8X16[c*16+i]);
		LCD_Set_Pos(x,y+1);
		for(i=0;i<8;i++)
		LCD_WrDat(F8X16[c*16+i+8]);
	}
}
void LCD_P8x16ClearChar(unsigned char x,unsigned char y,unsigned char ch)
{
	unsigned char i=0;
	if (ch!='\0')
	{
		if(x>120)
		{
			x=0;
			y++;
		}
		LCD_Set_Pos(x,y);
		for(i=0;i<8;i++)
		LCD_WrDat(0);
		LCD_Set_Pos(x,y+1);
		for(i=0;i<8;i++)
		LCD_WrDat(0);
	}
}
/*******************功能描述：显示8*16一组标准ASCII字符串	 显示的坐标（x,y），y为页范围0～7****************/
void LCD_P8x16Str(unsigned char x,unsigned char y,unsigned char ch[])
{
unsigned char c=0,i=0,j=0;
while (ch[j]!='\0')
{
c =ch[j]-32;
if(x>120){x=0;y++;}
LCD_Set_Pos(x,y);
for(i=0;i<8;i++)
LCD_WrDat(F8X16[c*16+i]);
LCD_Set_Pos(x,y+1);
for(i=0;i<8;i++)
LCD_WrDat(F8X16[c*16+i+8]);
x+=8;
j++;
}
}
/*****************功能描述：显示16*16点阵  显示的坐标（x,y），y为页范围0～7****************************/
void LCD_P16x16Ch(unsigned char x,unsigned char y,unsigned char N)
{
unsigned char wm=0;
unsigned int adder=32*N;  //
LCD_Set_Pos(x , y);
for(wm = 0;wm < 16;wm++)  //
{
LCD_WrDat(F16x16[adder]);
adder += 1;
}
LCD_Set_Pos(x,y + 1);
for(wm = 0;wm < 16;wm++) //
{
LCD_WrDat(F16x16[adder]);
adder += 1;
}
}
/***********功能描述：显示显示BMP图片128×64起始点坐标(x,y),x的范围0～127，y为页的范围0～7*****************/
void Draw_BMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1, unsigned char const BMP[])
{
 unsigned int j=0;
 unsigned char x,y;

  if(y1%8==0) y=y1/8;
  else y=y1/8+1;
	for(y=y0;y<y1;y++)
	{
		LCD_Set_Pos(x0,y);
		for(x=x0;x<x1;x++)
	    {
	    	LCD_WrDat(BMP[j++]);
	    }
	}
}
/*******************功能描述：显示8*16一组标准ASCII字符串	 显示的坐标（x,y），y为页范围0～7****************/
void LCD_P24x24Char(unsigned char x,unsigned char y,unsigned char ch)
{
	unsigned char c=0,i=0;
	if (ch!='\0')
	{
		c =ch-48;
		c=0;
		if(x>120)
		{
			x=0;
			y++;
		}
		LCD_Set_Pos(x,y);
		for(i=0;i<24;i++)
		LCD_WrDat(nAsciiDot24x24[c*72+i]);

		LCD_Set_Pos(x,y+1);
		for(i=0;i<24;i++)
		LCD_WrDat(nAsciiDot24x24[c*72+i+24]);

		LCD_Set_Pos(x,y+2);
		for(i=0;i<24;i++)
		LCD_WrDat(nAsciiDot24x24[c*72+i+48]);
	}
}

















