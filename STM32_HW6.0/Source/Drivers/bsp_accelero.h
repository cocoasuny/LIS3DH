/**
  ****************************************************************************************
  * @file    bsp_accelero.h
  * @author  Jason
  * @version V1.0.0
  * @date    2017-3-9
  * @brief   header of bsp_accelero.c
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2017 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_ACCELERO_H
#define __BSP_ACCELERO_H

#include "platform.h"
#include "lis3dh_driver.h"

/* debug switch for lis3dh driver */
#define DEBUG_LIS3DH_DRV


#define LIS3DH_TAP_THRESHOLD		35  //0~127
#define LIS3DH_TAP_LIMIT_TIME		20  //0~127
#define LIS3DH_TAP_LATENCY_TIME		80  //0~255
#define LIS3DH_TAP_WINDOW_TIME		80  //0~255

/* type define for Accelerometer read mode */
typedef enum
{
    NORMAL_READ = 0,
    FIFO_READ
}ACC_READ_MODE_T;

/* type define for Accelerometer config */
typedef struct
{
    LIS3DH_ODR_t            ODR;
    LIS3DH_Fullscale_t      FS;
    uint8_t                 WaterMark;    //0~31
}ACC_CONFIG_T;

typedef struct
{
   short        x; /**< holds x-axis acceleration data sign extended. Range -512 to 511. */
   short        y; /**< holds y-axis acceleration data sign extended. Range -512 to 511. */
   short        z; /**< holds z-axis acceleration data sign extended. Range -512 to 511. */
} ACC_FIFO_DATA_T;

/* function declare */
uint8_t bsp_accelero_init_step(ACC_READ_MODE_T mode,ACC_CONFIG_T *config);
uint8_t bsp_accelero_normal_read(AxesRaw_t *data);
uint8_t bsp_accelero_fifo_read(ACC_FIFO_DATA_T *fifoData);
uint8_t bsp_accelero_double_click_enable(void);
uint8_t bsp_accelero_single_click_enable(void);

#endif /* __BSP_ACCELERO_H */

/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/

