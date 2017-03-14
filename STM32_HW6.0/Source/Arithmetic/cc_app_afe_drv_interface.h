/*
* Copyright(c) 2015, CloudCare Healthcare Co.,Ltd
* All rights reserved
* File Name:                _CC_APP_AFE_DRV_INTERFACE.h
* Descprition:              the _CC_APP_AFE_DRV_INTERFACE.h files for translate layer
* Created Date:             2016/03/14
* Author:                   yun.zhu@ywkang.com  
* Current Reversion:        V1.0
* Reversion History:
*   Rev *.*    
*     Author:     
*     Change Date:
*     Change List:
*/

#ifndef _CC_APP_AFE_DRV_INTERFACE
#define _CC_APP_AFE_DRV_INTERFACE

#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_syscfg.h"
#include "cc_alg_app_interface.h"
#include "platform.h"
#include "PowerManage.h"


/*
*     AFE44XX related MCU PIO Source allocation
*		Need to change in platform.h
* 		
*/
#define AFE_RESETZ_PORT      PT_AFE_RESETZ_PORT

#define AFE_PDNZ_PORT        PT_AFE_PDNZ_PORT
#define AFE_ADC_DRDY_PORT    PT_AFE_ADC_DRDY_PORT
#define AFE_DIAG_END_PORT    PT_AFE_DIAG_END_PORT

#define AFE_RESETZ        PT_AFE_RESETZ
#define AFE_PDNZ          PT_AFE_PDNZ
#define AFE_ADC_DRDY      PT_AFE_ADC_DRDY
#define AFE_DIAG_END      PT_AFE_DIAG_END

#define AFE_ADC_DRDY_EXTI_PORT       PT_AFE_ADC_DRDY_EXTI_PORT
#define AFE_ADC_DRDY_EXTI_PIN        PT_AFE_ADC_DRDY_EXTI_PIN
#define AFE_ADC_DRDY_EXTI_LINE       PT_AFE_ADC_DRDY_EXTI_LINE

#define AFE_ADC_DIAG_EXTI_PORT       PT_AFE_ADC_DIAG_EXTI_PORT
#define AFE_ADC_DIAG_EXTI_PIN        PT_AFE_ADC_DIAG_EXTI_PIN
#define AFE_ADC_DIAG_EXTI_LINE       PT_AFE_ADC_DIAG_EXTI_LINE

/* AFE44x0 SPI Port Definition*/
#define AFE_SPI_PORT		PT_AFE_SPI_PORT
#define AFE_SPI_STE_PINSOURCE	PT_AFE_SPI_STE_PINSOURCE
#define AFE_SPI_SCLK_PINSOURCE	PT_AFE_SPI_SCLK_PINSOURCE
#define AFE_SPI_SOMI_PINSOURCE	PT_AFE_SPI_SOMI_PINSOURCE
#define AFE_SPI_SIMO_PINSOURCE	PT_AFE_SPI_SIMO_PINSOURCE

#define AFE_SPI_STE_PIN		PT_AFE_SPI_STE_PIN
#define AFE_SPI_SCLK_PIN	PT_AFE_SPI_SCLK_PIN
#define AFE_SPI_SOMI_PIN	PT_AFE_SPI_SOMI_PIN
#define AFE_SPI_SIMO_PIN	PT_AFE_SPI_SIMO_PIN

#define AFE44x0_SPI PT_AFE44x0_SPI

#define RCC_APBxPeriph_SPI_AFE44x0		PT_RCC_APBxPeriph_SPI_AFE44x0

#define AFE_SPI_STE_GPIO_CLK				PT_AFE_SPI_STE_GPIO_CLK
#define AFE_SPI_SCLK_GPIO_CLK				PT_AFE_SPI_SCLK_GPIO_CLK
#define AFE_SPI_SOMI_GPIO_CLK				PT_AFE_SPI_SOMI_GPIO_CLK
#define AFE_SPI_SIMO_GPIO_CLK				PT_AFE_SPI_SIMO_GPIO_CLK

#define AFE_RESETZ_GPIO_CLK        PT_AFE_RESETZ_GPIO_CLK
#define AFE_PDNZ_GPIO_CLK          PT_AFE_PDNZ_GPIO_CLK
#define AFE_ADC_DRDY_GPIO_CLK      PT_AFE_ADC_DRDY_GPIO_CLK
#define AFE_DIAG_END_GPIO_CLK      PT_AFE_DIAG_END_GPIO_CLK

#define PMG_AFE44XX_GPIO						PT_PMG_AFE44XX_GPIO
#define PMG_AFE44XX_SPI							PT_PMG_AFE44XX_SPI


/****************************************************************/
/* GPIO Operation for AFE44x0*/
/****************************************************************/

#define AFE44x0_GPIO_PDNZ_SET GPIO_SetBits(AFE_PDNZ_PORT, AFE_PDNZ)
#define AFE44x0_GPIO_PDNZ_CLR	GPIO_ResetBits(AFE_PDNZ_PORT, AFE_PDNZ)

#define AFE44x0_GPIO_RESETZ_SET GPIO_SetBits(AFE_RESETZ_PORT, AFE_RESETZ)
#define AFE44x0_GPIO_RESETZ_CLR	GPIO_ResetBits(AFE_RESETZ_PORT, AFE_RESETZ)

void cc_app_afe_drv_interface_init(void);

#endif //_CC_APP_AFE_DRV_INTERFACE

//end file
