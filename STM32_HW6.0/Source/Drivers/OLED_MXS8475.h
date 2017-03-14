#ifndef __OLED_MXS8475_H
#define __OLED_MXS8475_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "platform.h"



#ifdef BOARD_REDHARE_V3_0
	/* Exported macro ------------------------------------------------------------*/
	/* Power 15V GPIO(PB0) define */
	#define RCC_APBPeriph_OLED_MXS8475Power              RCC_AHBPeriph_GPIOE
	#define GPIO_Pin_OLED_MXS8475Power                   GPIO_Pin_3
	#define GPIO_OLED_MXS8475Power                       GPIOE
	#define OLED_Power_Periph                            Periph_GPIOE

	/* OLED GPIO-A0 (PB1) define */
	#define RCC_APBPeriph_OLED_MXS8475A0                 RCC_AHBPeriph_GPIOB
	#define GPIO_Pin_OLED_MXS8475A0                      GPIO_Pin_1
	#define GPIO_OLED_MXS8475A0                          GPIOB
	#define OLED_A0_Periph                               Periph_GPIOB

	/* OLED GPIO-Rest (PB14) define */
	#define RCC_APBPeriph_OLED_MXS8475Rest               RCC_AHBPeriph_GPIOB
	#define GPIO_Pin_OLED_MXS8475Rest                    GPIO_Pin_9
	#define GPIO_OLED_MXS8475Rest                        GPIOB
	#define OLED_Rest_Periph                             Periph_GPIOB
	
	/* OLED SPI define */
	#define RCC_APBPeriph_OLED_MXS8475SPI                RCC_APB1Periph_SPI2

	#define OLED_MXS8475SPI_SCK_GPIO_CLK                 RCC_AHBPeriph_GPIOB
	#define OLED_MXS8475SPI_MOSI_GPIO_CLK                RCC_AHBPeriph_GPIOB
	#define OLED_MXS8475SPI_NSS_GPIO_CLK                 RCC_AHBPeriph_GPIOB

	#define OLED_MXS8475_SPI_NSS_GPIO_PORT               GPIOB
	#define OLED_MXS8475_SPI_MOSI_GPIO_PORT              GPIOB
	#define OLED_MXS8475_SPI_SCK_GPIO_PORT               GPIOB
	#define OLED_SPI_GPIO_Periph						 Periph_GPIOB

	#define OLED_MXS8475_SPI_NSS_SOURCE                  GPIO_PinSource12
	#define OLED_MXS8475_SPI_MOSI_SOURCE                 GPIO_PinSource15
	#define OLED_MXS8475_SPI_SCK_SOURCE                  GPIO_PinSource13

	#define OLED_MXS8475_SPI_NSS_AF                      GPIO_AF_SPI2
	#define OLED_MXS8475_SPI_MOSI_AF                     GPIO_AF_SPI2
	#define OLED_MXS8475_SPI_SCK_AF                      GPIO_AF_SPI2

	#define OLED_MXS8475_SPI_NSS_PIN                     GPIO_Pin_12
	#define OLED_MXS8475_SPI_MOSI_PIN                    GPIO_Pin_15
	#define OLED_MXS8475_SPI_SCK_PIN                     GPIO_Pin_13

	#define OLED_MXS8475SPI                              SPI2
	#define OLED_SPI_Periph                              Periph_SPI2
#endif


/* Exported functions ------------------------------------------------------- */
void OLED_MX8475CtrlLinesConfig(void);
void OLED_MX8475CtrlLinesDeConfig(void);
void OLED_MXS8475GPIOSPIConfig(void);
void OLED_MXS8475SPIDeConfig(void);
void OLED_MXS8475SPIConfig(void);
void MXS8475_DefConfig(void);
//void OLED_MXS8475_DisplayChar(unsigned char XS,unsigned char YS,unsigned char ch);
void MXS8475_displayPitrue(unsigned char xBoxStart,unsigned char xBoxEnd,unsigned char yBoxStart,unsigned char yBoxEnd,const unsigned char * pitrueData,unsigned int dataLeng);


void OLED_SPISendChar(unsigned char ch);
void MXS8475_WriteReg(unsigned char reg , unsigned char dat);
void MXS8475_WriteRegEx(unsigned char reg , unsigned char dat ,unsigned char Num);
void MXS8475_WriteCMD(unsigned char cmd );
void MXS8475_WriteDat(uint8_t dat );

void OLED_GPIOWriteReg(unsigned char  ch, unsigned cmd);

void OLED_Clear(unsigned char x,unsigned char xx,unsigned char y,unsigned char yy,unsigned char liang);

//extern unsigned char  aket_96x39_pitrue1[];
#endif /* __OLED_MXS8475_H */



























