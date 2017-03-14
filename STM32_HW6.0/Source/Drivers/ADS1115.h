#ifndef __ADS1115_H
#define __ADS1115_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "cpal.h"
#include "IIC_GPIO.h"

//#define ADS1115_IIC_HW          //使用硬件IIC
#define ADS1115_IIC_SW          //使用软件IIC

extern uint8_t ADS1115_IIC_RXTC_Flag;

enum ADS1115_StateTypeDef
{
  ADS1115_State_ConfigControlRg=0,        //配置ADS1115控制寄存器
  ADS1115_State_ConfigLo,                      //配置ADS1115Lo寄存器
  ADS1115_State_ConfigHi,                      //配置ADS1115Hi寄存器
  ADS1115_State_ConversionRdy,                 //ADS1115准备转换
  ADS1115_State_PointConversionRg,             //指向ADS1115转换寄存器
  ADS1115_State_ReadRdy,                       //转换完成，准备读取
  ADS1115_State_Conversion,              //ADS1115转换中
};


/* Exported macro ------------------------------------------------------------*/
/*****************ADS1115 ---------STM32L152***********************************/
/*****************SCL     <--------SCL2 (PB10)*********************************/
/*****************SDA     <------->SDA2 (PB11)*********************************/
/*****************ALERT/RDY------->PD11       *********************************/
#define  ADS1115_ALERTRDY_GPIO_CLK      PT_ADS1115_ALERTRDY_GPIO_CLK
#define  ADS1115_ALERTRDY_GPIO_PORT     PT_ADS1115_ALERTRDY_GPIO_PORT
#define  ADS1115_ALERTRDY_PIN           PT_ADS1115_ALERTRDY_PIN
#define  Periph_ADS1115_RDY             PT_Periph_ADS1115_RDY 

#define  ADS1115_RCC_APBPeriph_SYSCFG   PT_ADS1115_RCC_APBPeriph_SYSCFG
#define  ADS1115_EXTI_PortSourceGPIO    PT_ADS1115_EXTI_PortSourceGPIO
#define  ADS1115_EXTI_PinSource         PT_ADS1115_EXTI_PinSource
#define  ADS1115_EXTI_Line              PT_ADS1115_EXTI_Line

/*************CPAL IIC初始化宏定义*********************************************/
#define  I2C_SPEED  100000  /* Speed in Hz */
#define  I2C_DevStructure        I2C2_DevStructure

#define W_ADS1115_ADDRESS            0x90       //写ADS1115 IIC地址
#define R_ADS1115_ADDRESS            0x91       //读ADS1115 IIC地址
#define W_ADS1115_CRDataSize_One     1          //ADS1115 Config Register写一个字节
#define W_ADS1115_CRDataSize_Thr     3          //ADS1115 Config Register写三个字节

/*************ADS1115 Control Register定义**************************************/
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

////Comparator mode define
#define Tr_COMP    0x0000
#define Wn_COMP    0x0010

////Comparator polarity define
#define COMP_POL_L  0x0000
#define COMP_POL_H  0x0008

////Compatator latch define
#define COMP_LAT_OFF    0x0000
#define COMP_LAT_ON     0x0004

////Comparator queue define
#define COMP_QUE_ONE    0x0000
#define COMP_QUE_TWO    0x0001
#define COMP_QUE_THR    0x0002
#define COMP_Disable    0x0003


#define ADS1115Start    0x8000|FS_02|SigleShotMode|DR128SPS|Tr_COMP|COMP_POL_H|COMP_LAT_OFF|COMP_QUE_ONE


/* Exported functions ------------------------------------------------------- */
void ADS1115_Init(void);
void ADS1115_DeInit(void);
void ADS1115_GPIO_Init(void);
void ADS1115_GPIO_DeInit(void);
void ADS1115_IIC_Init(void);
void ADS1115_IIC_DeInit(void);
void ADS1115_Default_Config(void);
void ADS1115_Config_Lo(void);
void ADS1115_Config_Hi(void);
void ADS1115_PointConversionRg(void);
void ADS1115_ReadVal(void);
void ADS1115_GetVal(unsigned short Setdata);

extern CPAL_TransferTypeDef  sRxStructure, sTxStructure;
extern enum ADS1115_StateTypeDef  ADS1115_State;

#endif/*__ADS1115_H*/
