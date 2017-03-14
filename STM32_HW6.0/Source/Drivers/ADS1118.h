#ifndef __ADS1118_H_
#define __ADS1118_H_


/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "common.h"


/* ADS1118 SPI define */
#define RCC_APBPeriph_ADS118_SPI                PT_RCC_APBPeriph_ADS118_SPI

#define ADS1118_CS_GPIO_CLK                     PT_ADS1118_CS_GPIO_CLK
#define ADS1118_SPI_SCK_GPIO_CLK                PT_ADS1118_SPI_SCK_GPIO_CLK
#define ADS1118_SPI_MOSI_GPIO_CLK               PT_ADS1118_SPI_MOSI_GPIO_CLK
#define ADS1118_SPI_MISO_GPIO_CLK               PT_ADS1118_SPI_MISO_GPIO_CLK

#define ADS1118_SPI_MISO_GPIO_PORT              PT_ADS1118_SPI_MISO_GPIO_PORT
#define ADS1118_SPI_MOSI_GPIO_PORT              PT_ADS1118_SPI_MOSI_GPIO_PORT
#define ADS1118_SPI_SCK_GPIO_PORT               PT_ADS1118_SPI_SCK_GPIO_PORT
#define ADS1118_CS_GPIO_PORT                    PT_ADS1118_CS_GPIO_PORT

#define ADS1118_SPI_MISO_SOURCE                 PT_ADS1118_SPI_MISO_SOURCE
#define ADS1118_SPI_MOSI_SOURCE                 PT_ADS1118_SPI_MOSI_SOURCE
#define ADS1118_SPI_SCK_SOURCE                  PT_ADS1118_SPI_SCK_SOURCE

#define ADS1118_SPI_MISO_AF                     PT_ADS1118_SPI_MISO_AF
#define ADS1118_SPI_MOSI_AF                     PT_ADS1118_SPI_MOSI_AF
#define ADS1118_SPI_SCK_AF                      PT_ADS1118_SPI_SCK_AF

#define ADS1118_SPI_MISO_PIN                    PT_ADS1118_SPI_MISO_PIN
#define ADS1118_SPI_MOSI_PIN                    PT_ADS1118_SPI_MOSI_PIN
#define ADS1118_SPI_SCK_PIN                     PT_ADS1118_SPI_SCK_PIN
#define ADS1118_CS_PIN                          PT_ADS1118_CS_PIN

#define ADS1118_SPI                             PT_ADS1118_SPI

/*************ADS1118 Control Register∂®“Â**************************************/
////Single-end Channel define
#define SingleEndCH0    0x4000
#define SingleEndCH1    0x5000
#define SingleEndCH2    0x6000
#define SingleEndCH3    0x7000

////Differential Channel define
#define DifferentialCH0_1   0x0000
#define DifferentialCH0_3   0x1000
#define DifferentialCH1_3   0x2000
#define DifferentialCH2_3   0x3000

////Full scale define
#define FS_6    0x0000
#define FS_4    0x0200
#define FS_2    0x0400
#define FS_1    0x0600
#define FS_05   0x0800
#define FS_02   0x0A00

////Device operating mode
#define ContinuousMode    0x0000
#define SigleShotMode     0x0100

////Data rate define
#define DR8SPS        0x0000
#define DR16SPS       0x0020
#define DR32SPS       0x0040
#define DR64SPS       0x0060
#define DR128SPS      0x0080
#define DR250SPS      0x00A0
#define DR475SPS      0x00C0
#define DR860SPS      0x00E0

////Temperature sensor Mode define
#define MODEADC    0x0000
#define MODETS     0x0010

////PULL Up define
#define PULLUPDISABLE    0x0000
#define PULLUPENABLE     0x0008

////No Operature define
#define NOP              0x0000
#define UpdateCR         0x0002

#define ADS1118Start    0x8000|FS_1|SigleShotMode|DR128SPS|MODEADC|PULLUPDISABLE|UpdateCR

void ADS1118_Init(void);
void ADS1118_DeInit(void);
bool ADS1118_GetVal(uint16_t CMD, int16_t * pDatRe);


#endif /* __ADS1118_H_ */


