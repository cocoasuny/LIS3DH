/**
  ******************************************************************************
  * @file    Project/STM32_CPAL_Template/cpal_usercallback.c
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    21-December-2012
  * @brief   This file provides all the CPAL UserCallback functions .
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Usart.h"
#include "ADS1115.h"
#include "platform.h"
#include "lis3dh_driver.h"

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/*------------------------------------------------------------------------------
                     CPAL User Callbacks implementations
------------------------------------------------------------------------------*/


/*=========== Timeout UserCallback ===========*/


/**
  * @brief  User callback that manages the Timeout error.
  * @param  pDevInitStruct .
  * @retval None.
  */
uint32_t CPAL_TIMEOUT_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
     /* Set default values for the state parameters */
  pDevInitStruct->CPAL_State = CPAL_STATE_READY;
  pDevInitStruct->wCPAL_DevError = CPAL_I2C_ERR_NONE ;
  pDevInitStruct->wCPAL_Timeout  = CPAL_I2C_TIMEOUT_DEFAULT;

	printf("ADS1115 IIC Time Out\r\n");
  /* Reset the CPAL device */
  CPAL_I2C_DeInit(pDevInitStruct);

  /* Initialize CPAL device with the selected parameters */
  CPAL_I2C_Init(pDevInitStruct);

  return CPAL_PASS;
}


/*=========== Transfer UserCallback ===========*/


/**
  * @brief  Manages the End of Tx transfer event.
  * @param  pDevInitStruct
  * @retval None
  */
void CPAL_I2C_TXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{	
	if(pDevInitStruct->CPAL_Dev == CPAL_I2C1) 
	{
		flag_LIS3DH_IIC_DMA_TX_Finished = 1;
	}
	/*DMA event from I2C2*/
	if(pDevInitStruct->CPAL_Dev == CPAL_I2C2)
	{
    //printf("DMA transmit complate\r\n");
    switch (ADS1115_State)
    {
        case ADS1115_State_ConfigControlRg:
            ADS1115_State = ADS1115_State_ConfigLo;
			//printf("ADS1115_State_ConfigLo\r\n");
            break;
        case ADS1115_State_ConfigLo:
			//printf("ADS1115_State_ConfigHiM\r\n");
            ADS1115_Config_Lo();
			ADS1115_State = ADS1115_State_ConfigHi;
			//printf("ADS1115_State_ConfigHi\r\n");
            break;
        case ADS1115_State_ConfigHi:
			//printf("ADS1115_State_ConversionRdyM\r\n");
            ADS1115_Config_Hi();
            ADS1115_State = ADS1115_State_ConversionRdy;
			//printf("ADS1115_State_ConversionRdy\r\n");
            break;
        case ADS1115_State_PointConversionRg:
            ADS1115_PointConversionRg();
            ADS1115_State = ADS1115_State_ReadRdy;
			//printf("ADS1115_State_ReadRdy\r\n");
            break;
        default:
			//printf("ADS1115_State_Error\r\n");
			break;
    }
	}
    /* Deinitialize CPAL device */
 // CPAL_I2C_DeInit(&I2C2_DevStructure);

  /* Initialize CPAL device with the selected parameters */
  //CPAL_I2C_Init(&I2C2_DevStructure);
}


/**
  * @brief  Manages the End of Rx transfer event.
  * @param  pDevInitStruct
  * @retval None
  */
void CPAL_I2C_RXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
	if(pDevInitStruct->CPAL_Dev == CPAL_I2C1) 
	{
		flag_LIS3DH_IIC_DMA_RX_Finished = 1;
	}
	if(pDevInitStruct->CPAL_Dev == CPAL_I2C2) 
	{
		ADS1115_IIC_RXTC_Flag = 1;
	}
	
}


/**
  * @brief  Manages Tx transfer event.
  * @param  pDevInitStruct
  * @retval None
  */
/*void CPAL_I2C_TX_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{

}*/


/**
  * @brief  Manages Rx transfer event.
  * @param  pDevInitStruct
  * @retval None
  */
/*void CPAL_I2C_RX_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{

}*/


/**
  * @brief  Manages the End of DMA Tx transfer event.
  * @param  pDevInitStruct
  * @retval None
  */
/*void CPAL_I2C_DMATXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{

}
*/

/**
  * @brief  Manages the Half of DMA Tx transfer event.
  * @param  pDevInitStruct
  * @retval None
  */
/*void CPAL_I2C_DMATXHT_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{

}*/


/**
  * @brief  Manages Error of DMA Tx transfer event.
  * @param  pDevInitStruct
  * @retval None
  */
/*void CPAL_I2C_DMATXTE_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{

}*/


/**
  * @brief  Manages the End of DMA Rx transfer event.
  * @param  pDevInitStruct
  * @retval None
  */
void CPAL_I2C_DMARXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
	/*DMA event from I2C2*/
	if(pDevInitStruct->CPAL_Dev == CPAL_I2C2) 
	{
		ADS1115_State = ADS1115_State_ConversionRdy;
	}
}


/**
  * @brief  Manages the Half of DMA Rx transfer event.
  * @param  pDevInitStruct
  * @retval None
  */
/*void CPAL_I2C_DMARXHT_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{

}*/


/**
  * @brief  Manages Error of DMA Rx transfer event.
  * @param  pDevInitStruct
  * @retval None
  */
/*void CPAL_I2C_DMARXTE_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{

}*/


/*=========== Error UserCallback ===========*/


/**
  * @brief  User callback that manages the I2C device errors.
  * @note   Make sure that the define USE_SINGLE_ERROR_CALLBACK is uncommented in
  *         the cpal_conf.h file, otherwise this callback will not be functional.
  * @param  pDevInitStruct.
  * @param  DeviceError.
  * @retval None
  */
void CPAL_I2C_ERR_UserCallback(CPAL_DevTypeDef pDevInstance, uint32_t DeviceError)
{

}

/**
  * @brief  User callback that manages BERR I2C device errors.
  * @note   Make sure that the define USE_MULTIPLE_ERROR_CALLBACK is uncommented in
  *         the cpal_conf.h file, otherwise this callback will not be functional.
  * @param  pDevInstance.
  * @retval None
  */
/*void CPAL_I2C_BERR_UserCallback(CPAL_DevTypeDef pDevInstance)
{

}*/


/**
  * @brief  User callback that manages ARLO I2C device errors.
  * @note   Make sure that the define USE_MULTIPLE_ERROR_CALLBACK is uncommented in
  *         the cpal_conf.h file, otherwise this callback will not be functional.
  * @param  pDevInstance.
  * @retval None
  */
/*void CPAL_I2C_ARLO_UserCallback(CPAL_DevTypeDef pDevInstance)
{

}*/


/**
  * @brief  User callback that manages OVR I2C device errors.
  * @note   Make sure that the define USE_MULTIPLE_ERROR_CALLBACK is uncommented in
  *         the cpal_conf.h file, otherwise this callback will not be functional.
  * @param  pDevInstance.
  * @retval None
  */
/*void CPAL_I2C_OVR_UserCallback(CPAL_DevTypeDef pDevInstance)
{

}*/


/**
  * @brief  User callback that manages AF I2C device errors.
  * @note   Make sure that the define USE_MULTIPLE_ERROR_CALLBACK is uncommented in
  *         the cpal_conf.h file, otherwise this callback will not be functional.
  * @param  pDevInstance.
  * @retval None
  */
/*void CPAL_I2C_AF_UserCallback(CPAL_DevTypeDef pDevInstance)
{

}*/


/*=========== Addressing Mode UserCallback ===========*/


/**
  * @brief  User callback that manage General Call Addressing mode.
  * @param  pDevInitStruct
  * @retval None
  */
/*void CPAL_I2C_GENCALL_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{

}*/


/**
  * @brief  User callback that manage Dual Address Addressing mode.
  * @param  pDevInitStruct
  * @retval None
  */
/*void CPAL_I2C_DUALF_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{

}*/


/*=========== Listen Mode UserCallback ===========*/


/**
  * @brief  User callback that manage slave read operation.
  * @param  pDevInitStruct
  * @retval None
  */
/*void CPAL_I2C_SLAVE_READ_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
}*/


/**
  * @brief  User callback that manage slave write operation.
  * @param  pDevInitStruct
  * @retval None
  */
/*void CPAL_I2C_SLAVE_WRITE_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
}*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
