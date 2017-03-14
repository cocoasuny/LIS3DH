/**
  ****************************************************************************************
  * @file    bsp_accelero.c
  * @author  Jason
  * @version V1.0.0
  * @date    2017-3-9
  * @brief   bsp driver for Accelero
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2017 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "bsp_accelero.h"

/* privare function declare */
static status_t acc_lis3dh_int1_init(LIS3DH_IntPinConf_t pinConf);
static status_t acc_lis3dh_int2_init(LIS3DH_IntPinConf_t pinConf);
static void convert_ODR_to_time_ms(float *time);

/**
  * @brief  Accelerometer initiate
  * @param  uint8_t mode: FIFO_READ/NORMAL_READ; ACC_CONFIG_T config
  * @retval Status
  */
uint8_t bsp_accelero_init_step(ACC_READ_MODE_T mode,ACC_CONFIG_T *config)
{
    status_t    ret;
    uint8_t     ID = 0;
    #ifdef DEBUG_LIS3DH_DRV
        uint8_t     response = 0;
    #endif
    
    /* SPI Init */
    LIS3DH_SpiInit();

    LIS3DH_GetWHO_AM_I(&ID);
    if(ID != LIS3DH)
    {
        #ifdef DEBUG_LIS3DH_DRV
            printf("LIS3DH Drv Err\r\n");
        #endif
        return MEMS_ERROR;
    }
    #ifdef DEBUG_LIS3DH_DRV
    else
    {
        printf("LIS3DH Drv OK\r\n");
    }
    #endif
        
    response = LIS3DH_SetODR(config->ODR);
    #ifdef DEBUG_LIS3DH_DRV
        if(response == 1)
        {
            printf("SET_ODR_OK\r\n");
        }
    #endif
    response = LIS3DH_SetMode(LIS3DH_NORMAL);
    #ifdef DEBUG_LIS3DH_DRV
        if(response == 1)
        {
            printf("SET_MODE_OK\r\n");
        }
    #endif
    response = LIS3DH_SetFullScale(config->FS); 
    #ifdef DEBUG_LIS3DH_DRV    
        if(response == 1)
        {
            printf("SET_FULLSCALE_OK\r\n");
        } 
    #endif
    response = LIS3DH_SetAxis(LIS3DH_X_ENABLE | LIS3DH_Y_ENABLE | LIS3DH_Z_ENABLE);
    #ifdef DEBUG_LIS3DH_DRV
        if(response == 1)
        {
            printf("SET_AXIS_OK\r\n");
        }
    #endif
    
    /* config for Fifo read mode */
    if(mode == FIFO_READ)
    {
        LIS3DH_FIFOModeEnable(LIS3DH_FIFO_MODE);
        LIS3DH_Int1LatchEnable(MEMS_ENABLE);
        LIS3DH_SetWaterMark(config->WaterMark);
        
        /*	Map Watermark interrupt to Int1 port 	*/
        ret = acc_lis3dh_int1_init(LIS3DH_CLICK_ON_PIN_INT1_DISABLE | LIS3DH_CLICK_ON_PIN_INT1_DISABLE |
                                   LIS3DH_I1_INT2_ON_PIN_INT1_DISABLE | LIS3DH_I1_DRDY1_ON_INT1_DISABLE |
                                   LIS3DH_I1_DRDY2_ON_INT1_DISABLE | LIS3DH_WTM_ON_INT1_ENABLE |
                                   LIS3DH_INT1_OVERRUN_DISABLE);
    }
	
	bsp_accelero_single_click_enable();
       
    return ret;
}

/**
  * @brief  normal read the data of accelerometer
  * @param[out]  AxesRaw_t *data
  * @retval Status
  */
uint8_t bsp_accelero_normal_read(AxesRaw_t *data)
{
    status_t ret;
    
    ret = LIS3DH_GetAccAxesRaw(data);
    
    return ret;
}

/**
  * @brief  fifo read the data of accelerometer
  * @param[out]  ACC_FIFO_DATA_T *fifoData
  * @retval Status
  */
uint8_t bsp_accelero_fifo_read(ACC_FIFO_DATA_T *fifoData)
{
    uint8_t         i=0;
    AxesRaw_t       data;
    status_t        ret;
    
    for(i=0;i<32;i++)
    {
        LIS3DH_GetAccAxesRaw(&data);
        fifoData[i].x = data.AXIS_X;
        fifoData[i].y = data.AXIS_Y;
        fifoData[i].z = data.AXIS_Z;
    }
    
    ret = LIS3DH_FIFOModeEnable(LIS3DH_FIFO_BYPASS_MODE);
    ret = LIS3DH_FIFOModeEnable(LIS3DH_FIFO_MODE);
    
    return ret;
}
/**
  * @brief  ebable the double click
  * @param  None
  * @retval Status
  */
uint8_t bsp_accelero_double_click_enable(void)
{
    status_t    ret; 
    
    /* set the threshold */
    LIS3DH_SetClickTHS(LIS3DH_TAP_THRESHOLD);   //与敲多重有关系
    
    /* set the elapse between the start of the click-detection procedure */
    LIS3DH_SetClickLIMIT(LIS3DH_TAP_LIMIT_TIME);   //两次敲击之间时间间隔
    
    /*define the time interval that starts after the first click detection where
        the click-detection procedure is disabled, in cases where the device is configured for
        double-click detection. */
    LIS3DH_SetClickLATENCY(LIS3DH_TAP_LATENCY_TIME);
    
    /* define the maximum interval of time that can elapse after the end of the
        latency interval in which the click-detection procedure can start, in cases where the device
        is configured for double-click detection. */
    LIS3DH_SetClickWINDOW(LIS3DH_TAP_WINDOW_TIME);
    
    LIS3DH_SetClickCFG( 
                        LIS3DH_ZD_ENABLE | LIS3DH_ZS_DISABLE  | LIS3DH_YD_ENABLE  | 
                        LIS3DH_YS_DISABLE | LIS3DH_XD_ENABLE  | LIS3DH_XS_DISABLE 
                       );
    
    ret = acc_lis3dh_int2_init(
                        LIS3DH_CLICK_ON_PIN_INT2_ENABLE | LIS3DH_I2_INT1_ON_PIN_INT2_DISABLE |               
                        LIS3DH_I2_INT2_ON_PIN_INT2_DISABLE | LIS3DH_I2_BOOT_ON_INT2_DISABLE |                   
                        LIS3DH_INT_ACTIVE_HIGH
                        );
                        
    return ret;                            
}

/**
  * @brief  ebable the single click
  * @param  None
  * @retval Status
  */
uint8_t bsp_accelero_single_click_enable(void)
{
    status_t    ret;
    float       time_ms = 0;
    uint8_t     limit_Time = 0;
    
    /* set the threshold */
    LIS3DH_SetClickTHS(LIS3DH_TAP_THRESHOLD);   //与敲多重有关系
    
    /* set the elapse between the start of the click-detection procedure */
    convert_ODR_to_time_ms(&time_ms);
    printf("ms:%f\r\n",time_ms);
    limit_Time = (uint8_t)(LIS3DH_TAP_LIMIT_TIME / time_ms);
    if(limit_Time != 0)
    {
        LIS3DH_SetClickLIMIT(LIS3DH_TAP_LIMIT_TIME);   //
    }
    #ifdef DEBUG_LIS3DH_DRV
    else
    {
        printf("LIS3DH_TAP_LIMIT_TIME Set Err\r\n");
    }
    #endif
        
    LIS3DH_SetClickCFG( 
                        LIS3DH_ZS_ENABLE | LIS3DH_ZD_DISABLE  | LIS3DH_YS_DISABLE  | 
                        LIS3DH_YD_DISABLE | LIS3DH_XS_DISABLE  | LIS3DH_XD_DISABLE 
                       );
    
    ret = acc_lis3dh_int2_init(
                        LIS3DH_CLICK_ON_PIN_INT2_ENABLE | LIS3DH_I2_INT1_ON_PIN_INT2_DISABLE |               
                        LIS3DH_I2_INT2_ON_PIN_INT2_DISABLE | LIS3DH_I2_BOOT_ON_INT2_DISABLE |                   
                        LIS3DH_INT_ACTIVE_HIGH
                        );
                        
    return ret;                            
}

/*******************************************************************************
* Function Name  : LIS3DH_SetInt1Pin
* Description    : Set Interrupt1 pin Function
* Input          :  LIS3DH_CLICK_ON_PIN_INT1_ENABLE/DISABLE    | LIS3DH_I1_INT1_ON_PIN_INT1_ENABLE/DISABLE |              
                    LIS3DH_I1_INT2_ON_PIN_INT1_ENABLE/DISABLE  | LIS3DH_I1_DRDY1_ON_INT1_ENABLE/DISABLE    |              
                    LIS3DH_I1_DRDY2_ON_INT1_ENABLE/DISABLE     | LIS3DH_WTM_ON_INT1_ENABLE/DISABLE         |           
                    LIS3DH_INT1_OVERRUN_ENABLE/DISABLE  
* example        : SetInt1Pin(LIS3DH_CLICK_ON_PIN_INT1_ENABLE | LIS3DH_I1_INT1_ON_PIN_INT1_ENABLE |              
                    LIS3DH_I1_INT2_ON_PIN_INT1_DISABLE | LIS3DH_I1_DRDY1_ON_INT1_ENABLE | LIS3DH_I1_DRDY2_ON_INT1_ENABLE |
                    LIS3DH_WTM_ON_INT1_DISABLE | LIS3DH_INT1_OVERRUN_DISABLE   ) 
* Note           : To enable Interrupt signals on INT1 Pad (You MUST use all input variable in the argument, as example)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
static status_t acc_lis3dh_int1_init(LIS3DH_IntPinConf_t pinConf)
{
    status_t ret;
    
    EXTI_InitTypeDef BMA250E_INT1_EXTI_TYPE_CFG;
	GPIO_InitTypeDef BMA250E_INT1_GPIO_TYPE_CFG;
    
    /* 	Enable GPIO port 		*/
    /* 	Enable clock 			*/
    RCC_AHBPeriphClockCmd(LIS_A_INT1_GPIO_CLK, ENABLE);
	
	/*	Enable gpio function 	*/
 	BMA250E_INT1_GPIO_TYPE_CFG.GPIO_Pin       = LIS_A_INT1_PIN;
    BMA250E_INT1_GPIO_TYPE_CFG.GPIO_Mode      = GPIO_Mode_IN;
    BMA250E_INT1_GPIO_TYPE_CFG.GPIO_Speed     = GPIO_Speed_10MHz;  
    BMA250E_INT1_GPIO_TYPE_CFG.GPIO_OType     = GPIO_OType_PP;   
    BMA250E_INT1_GPIO_TYPE_CFG.GPIO_PuPd      = GPIO_PuPd_UP;  
    GPIO_Init(LIS_A_INT1_GPIO_PORT,&BMA250E_INT1_GPIO_TYPE_CFG); 

    /* 	Enable EXTI interrupt line 		*/
    SYSCFG_EXTILineConfig(LIS_A_INT1_EXTI_PORT_SOURCE, LIS_A_INT1_EXTI_PIN_SOURCE);

    BMA250E_INT1_EXTI_TYPE_CFG.EXTI_Line      = LIS_A_INT1_EXTI_LINE;         // EXTI Line
    BMA250E_INT1_EXTI_TYPE_CFG.EXTI_Mode      = EXTI_Mode_Interrupt;            // EXTI Mode, Interrupt
    BMA250E_INT1_EXTI_TYPE_CFG.EXTI_Trigger   = EXTI_Trigger_Rising;            // EXTI trigger, Falling edge
    BMA250E_INT1_EXTI_TYPE_CFG.EXTI_LineCmd   = ENABLE;

    EXTI_Init(&BMA250E_INT1_EXTI_TYPE_CFG);                                     // EXTI Init

    EXTI_ClearITPendingBit(LIS_A_INT1_EXTI_LINE);
	EXTI_ClearFlag(LIS_A_INT1_EXTI_LINE);
    
    ret = LIS3DH_SetInt1Pin(pinConf);
    
    return ret;
}

/*******************************************************************************
* Function Name  : LIS3DH_SetInt2Pin
* Description    : Set Interrupt2 pin Function
* Input          : LIS3DH_CLICK_ON_PIN_INT2_ENABLE/DISABLE   | LIS3DH_I2_INT1_ON_PIN_INT2_ENABLE/DISABLE |               
                   LIS3DH_I2_INT2_ON_PIN_INT2_ENABLE/DISABLE | LIS3DH_I2_BOOT_ON_INT2_ENABLE/DISABLE |                   
                   LIS3DH_INT_ACTIVE_HIGH/LOW
* example        : LIS3DH_SetInt2Pin(LIS3DH_CLICK_ON_PIN_INT2_ENABLE/DISABLE | LIS3DH_I2_INT1_ON_PIN_INT2_ENABLE/DISABLE |               
                   LIS3DH_I2_INT2_ON_PIN_INT2_ENABLE/DISABLE | LIS3DH_I2_BOOT_ON_INT2_ENABLE/DISABLE |                   
                   LIS3DH_INT_ACTIVE_HIGH/LOW)
* Note           : To enable Interrupt signals on INT2 Pad (You MUST use all input variable in the argument, as example)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
static status_t acc_lis3dh_int2_init(LIS3DH_IntPinConf_t pinConf)
{
    status_t ret;
    
    EXTI_InitTypeDef BMA250E_INT2_EXTI_TYPE_CFG;
	GPIO_InitTypeDef BMA250E_INT2_GPIO_TYPE_CFG;
    
    /* 	Enable GPIO port 		*/
    /* 	Enable clock 			*/
    RCC_AHBPeriphClockCmd(LIS_A_INT2_GPIO_CLK, ENABLE);
	
	/*	Enable gpio function 	*/
 	BMA250E_INT2_GPIO_TYPE_CFG.GPIO_Pin       = LIS_A_INT2_PIN;
    BMA250E_INT2_GPIO_TYPE_CFG.GPIO_Mode      = GPIO_Mode_IN;
    BMA250E_INT2_GPIO_TYPE_CFG.GPIO_Speed     = GPIO_Speed_10MHz;  
    BMA250E_INT2_GPIO_TYPE_CFG.GPIO_OType     = GPIO_OType_PP;   
    BMA250E_INT2_GPIO_TYPE_CFG.GPIO_PuPd      = GPIO_PuPd_UP;  
    GPIO_Init(LIS_A_INT2_GPIO_PORT,&BMA250E_INT2_GPIO_TYPE_CFG); 

    /* 	Enable EXTI interrupt line 		*/
    SYSCFG_EXTILineConfig(LIS_A_INT2_EXTI_PORT_SOURCE, LIS_A_INT2_EXTI_PIN_SOURCE);

    BMA250E_INT2_EXTI_TYPE_CFG.EXTI_Line      = LIS_A_INT2_EXTI_LINE;         // EXTI Line
    BMA250E_INT2_EXTI_TYPE_CFG.EXTI_Mode      = EXTI_Mode_Interrupt;            // EXTI Mode, Interrupt
    BMA250E_INT2_EXTI_TYPE_CFG.EXTI_Trigger   = EXTI_Trigger_Rising;            // EXTI trigger, Falling edge
    BMA250E_INT2_EXTI_TYPE_CFG.EXTI_LineCmd   = ENABLE;

    EXTI_Init(&BMA250E_INT2_EXTI_TYPE_CFG);                                     // EXTI Init

    EXTI_ClearITPendingBit(LIS_A_INT2_EXTI_LINE);
	EXTI_ClearFlag(LIS_A_INT2_EXTI_LINE);
    
    ret = LIS3DH_SetInt2Pin(pinConf);
    
    return ret;
}
/**
  * @brief  convert odr to time in ms
  * @param[out]  float *time
  * @retval Status
  */
static void convert_ODR_to_time_ms(float *time)
{
    uint8_t value = 0;
    
    /* Get the current ODR */
    LIS3DH_ReadReg(LIS3DH_CTRL_REG1, &value);
    
    value &= 0xF0;
    value = value >> LIS3DH_ODR_BIT;
    
    switch(value)
    {
        case LIS3DH_ODR_1Hz:
        {
            *time = 1000;
        }
        break;
        case LIS3DH_ODR_10Hz:
        {
            *time = 100;
        }
        break;
        case LIS3DH_ODR_25Hz:
        {
            *time = 40;
        }
        break;
        case LIS3DH_ODR_50Hz:
        {
            *time = 20;
        }
        break;
        case LIS3DH_ODR_100Hz:
        {
            *time = 10;
        }
        break;
        case LIS3DH_ODR_200Hz:
        {
            *time = 5;
        }
        break;
        case LIS3DH_ODR_400Hz:
        {
            *time = 2.5;
        }
        break;
        default:
        {
            *time = 1000;
        }
        break;
    }
}





/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/


