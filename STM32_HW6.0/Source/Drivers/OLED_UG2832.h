#ifndef __OLED_UG2832_H
#define __OLED_UG2832_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "common.h"

/* Exported macro ------------------------------------------------------------*/
#define GPIO_Pin_OLED_UG2832A0                       GPIO_Pin_7
#define GPIO_Pin_OLED_UG2832Rest                     GPIO_Pin_1

#define GPIO_OLED_UG2832                             GPIOD

#define RCC_APBPeriph_OLED_UG2832SPI             RCC_APB1Periph_SPI3

#define OLED_UG2832SPI_SCK_GPIO_CLK             RCC_AHBPeriph_GPIOC
#define OLED_UG2832SPI_MOSI_GPIO_CLK            RCC_AHBPeriph_GPIOC
#define OLED_UG2832SPI_MISO_GPIO_CLK            RCC_AHBPeriph_GPIOC
#define OLED_UG2832SPI_NSS_GPIO_CLK             RCC_AHBPeriph_GPIOA


#define OLED_UG2832_SPI_NSS_GPIO_PORT        GPIOA
#define OLED_UG2832_SPI_MOSI_GPIO_PORT       GPIOC
#define OLED_UG2832_SPI_MISO_GPIO_PORT       GPIOC
#define OLED_UG2832_SPI_SCK_GPIO_PORT        GPIOC

#define OLED_UG2832_SPI_NSS_SOURCE              GPIO_PinSource4
#define OLED_UG2832_SPI_MOSI_SOURCE             GPIO_PinSource12
#define OLED_UG2832_SPI_MISO_SOURCE             GPIO_PinSource11
#define OLED_UG2832_SPI_SCK_SOURCE              GPIO_PinSource10

#define OLED_UG2832_SPI_NSS_AF            GPIO_AF_SPI3
#define OLED_UG2832_SPI_MOSI_AF           GPIO_AF_SPI3
#define OLED_UG2832_SPI_MISO_AF           GPIO_AF_SPI3
#define OLED_UG2832_SPI_SCK_AF            GPIO_AF_SPI3

#define OLED_UG2832_SPI_NSS_PIN           GPIO_Pin_4
#define OLED_UG2832_SPI_MOSI_PIN          GPIO_Pin_12
#define OLED_UG2832_SPI_MISO_PIN          GPIO_Pin_11
#define OLED_UG2832_SPI_SCK_PIN           GPIO_Pin_10

#define OLED_UG2832SPI                   SPI3

#define	OLED_UG2832_DC_L                       GPIO_ResetBits(GPIO_OLED_UG2832, GPIO_Pin_OLED_UG2832A0);
#define OLED_UG2832_DC_H                       GPIO_SetBits(GPIO_OLED_UG2832, GPIO_Pin_OLED_UG2832A0);

#define OLED_UG2832_RST_L                      GPIO_ResetBits(GPIO_OLED_UG2832, GPIO_Pin_OLED_UG2832Rest)
#define OLED_UG2832_RST_H                      GPIO_SetBits(GPIO_OLED_UG2832, GPIO_Pin_OLED_UG2832Rest)
/************Ê¹ÓÃGPIOÄ£ÄâSPIÊ±Ê¹ÓÃ***************************/
#define	SC_H		GPIO_SetBits(GPIOC,GPIO_Pin_10)
#define	SC_L		GPIO_ResetBits(GPIOC,GPIO_Pin_10)

#define	SD_A_H		GPIO_SetBits(GPIOC,GPIO_Pin_12)
#define	SD_A_L		GPIO_ResetBits(GPIOC,GPIO_Pin_12)


#define	RES_H		GPIO_SetBits(GPIOD,GPIO_Pin_1)
#define	RES_L		GPIO_ResetBits(GPIOD,GPIO_Pin_1)


#define	CS_H		GPIO_SetBits(GPIOA,GPIO_Pin_4)
#define	CS_L		GPIO_ResetBits(GPIOA,GPIO_Pin_4)

#define	A0_H		GPIO_SetBits(GPIOD,GPIO_Pin_7)
#define	A0_L		GPIO_ResetBits(GPIOD,GPIO_Pin_7)
/**************************************************************/

/* Exported functions ------------------------------------------------------- */

void OLED_UG2832CtrlLinesConfig(void);
void OLED_UG2832SPIConfig(void);
void OLED_UG2832_WB(uint8_t data);
void OLED_UG2832_Init(void);


void LCD_CLS(void);
void LCD_Set_Pos(unsigned char x, unsigned char y);
void LCD_Fill(unsigned char bmp_dat);
void LCD_P16x16Ch(unsigned char x,unsigned char y,unsigned char N);
void LCD_P6x8Str(unsigned char x,unsigned char y,unsigned char ch[]);
void LCD_P8x16Str(unsigned char x,unsigned char y,unsigned char ch[]);
void LCD_P8x16Char(unsigned char x,unsigned char y,unsigned char ch);
void LCD_P24x24Char(unsigned char x,unsigned char y,unsigned char ch);
void LCD_P8x16ClearChar(unsigned char x,unsigned char y,unsigned char ch);
void Draw_BMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char const BMP[]);
void OledDisTask(event_t events);

void WriteData(unsigned char dat);
void WriteCommand(unsigned char cmd);
void SetStartColumn(unsigned char d);
void SetAddressingMode(unsigned char d);
void SetColumnAddress(unsigned char a, unsigned char b);
void SetPageAddress(unsigned char a, unsigned char b);
void SetStartLine(unsigned char d);
void SetContrastControl(unsigned char d);
void Set_Charge_Pump(unsigned char d);
void Set_Segment_Remap(unsigned char d);
void Set_Entire_Display(unsigned char d);
void Set_Inverse_Display(unsigned char d);
void Set_Multiplex_Ratio(unsigned char d);
void Set_Display_On_Off(unsigned char d);
void SetStartPage(unsigned char d);
void Set_Common_Remap(unsigned char d);
void Set_Display_Offset(unsigned char d);
void Set_Display_Clock(unsigned char d);
void Set_Precharge_Period(unsigned char d);
void Set_Common_Config(unsigned char d);
void Set_VCOMH(unsigned char d);
void Set_NOP(void);
void FillArea(unsigned char spage, unsigned char epage,unsigned char scolumn, unsigned char ecolumn,unsigned char pt);
void CheckOutline(void);
void Display8x16Str(unsigned char page, unsigned char column, unsigned char *str); //8x16Ascii×Ö·û
void DisplayChinese(unsigned char page, unsigned char column, unsigned int location);//16x16ÖÐÎÄ×Ö·
void Displaypicture1(unsigned char page ,unsigned char column);
void Displaypictureclear1(unsigned char page ,unsigned char column);

#endif /* OLED_UG2832 */

