/**
  ******************************************************************************
  * @file    STM32L1xx_IAP/src/flash_if.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    24-January-2012
  * @brief   This file provides all the memory related operation functions.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * FOR MORE INFORMATION PLEASE READ CAREFULLY THE LICENSE AGREEMENT FILE
  * LOCATED IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/** @addtogroup STM32L1xx_IAP
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include "flash_if.h"
#include "FWUpdate.h"
#include "common.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Unlocks Flash for write access
  * @param  None
  * @retval None
  */
void FLASH_If_Init(void)
{ 
  /* Unlock the Program memory */
  FLASH_Unlock();

  /* Clear all FLASH flags */  
  FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
                  | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);   
}

/**
  * @brief  This function does an erase of all user flash area
  * @param  StartSector: start of user flash area
  * @retval 0: user flash area successfully erased
  *         1: error occurred
  */
uint32_t FLASH_If_Erase(uint32_t StartSector)
{
	uint32_t flashaddress;
	FLASH_Status 	flagFlashErase;
	flashaddress = StartSector;
	
  while (flashaddress <= (uint32_t) FWUPDATE_BUF_FLASH_LAST_PAGE_ADDRESS)
  {
	__set_PRIMASK(1);		/* 	Disable all interrupt 		*/
	flagFlashErase = FLASH_ErasePage(flashaddress); 
	__set_PRIMASK(0);		/* 	Enable all interrupt 		*/
	  
    if (flagFlashErase == FLASH_COMPLETE)
    {
      flashaddress += FLASH_PAGE_SIZE;
    }
    else
    {
      /* Error occurred while page erase */
      return (1);
    }
  }
  return (0);
}

/**
  * @brief  This function writes a data buffer in flash (data are 32-bit aligned).
  * @note   After writing data buffer, the flash content is checked.
  * @param  FlashAddress: start address for writing data buffer
  * @param  Data: pointer on data buffer
  * @param  DataLength: length of data buffer (unit is 32-bit word)   
  * @retval 0: Data successfully written to Flash memory
  *         1: Error occurred while writing data in Flash memory
  *         2: Written Data in flash memory is different from expected one
  */
uint32_t FLASH_If_Write(__IO uint32_t* FlashAddress, uint32_t* Data ,uint16_t DataLength)
{
  FLASH_Status status = FLASH_BUSY;
  uint32_t* malPointer = (uint32_t *)Data;
  uint32_t memBuffer[32]; /* Temporary buffer holding data that will be written in a half-page space */
  uint32_t* mempBuffer = memBuffer;
  uint32_t i = 0;
    
  while (malPointer < (uint32_t*)(Data + DataLength))
  {
    /* Fill with the received buffer */
    while (mempBuffer < (memBuffer + 32))
    {
      /* If there are still data available in the received buffer */
      if (malPointer < ((uint32_t *)Data + DataLength))
      {
        *(uint32_t *)(mempBuffer++) = *(uint32_t *)(malPointer++);
      }
      else /* no more data available in the received buffer: fill remaining with dummy 0 */
      {
        *(uint32_t *)(mempBuffer++) = 0;
      }
    }
#ifdef FWUpdate_Debug
    printf("Write the buffer to the memory...\r\n");
#endif
    /* Write the buffer to the memory */
	
	__set_PRIMASK(1);		/* 	Disable all interrupt 		*/
	
    status = FLASH_ProgramHalfPage(*FlashAddress, (uint32_t *)(memBuffer));
	
	__set_PRIMASK(0);		/* 	Disable all interrupt 		*/
	
#ifdef FWUpdate_Debug
    printf("FLASH_ProgramHalfPage completely...\r\n");
#endif  
    if (status != FLASH_COMPLETE)
    {
      /* Error occurred while writing data in Flash memory */
      return (1);
    }

    /* Check if flash content matches memBuffer */
    for (i = 0; i < 32; i++)
    {
      if ((*(uint32_t*)(*(uint32_t*)FlashAddress + 4 * i)) != memBuffer[i])
      {
        /* flash content doesn't match memBuffer */
        return(2);
      }
    }

    /* Increment the memory pointer */
    *FlashAddress += 128;

    /* Reinitialize the intermediate buffer pointer */
    mempBuffer = memBuffer;
  }

  return (0);
}

/**
  * @brief  Disables the write protection of bootloader area.
  * @param  None
  * @retval 0: Write Protection successfully disabled
  *         1: Error: Flash write unprotection failed
  *         2: Flash memory is not write protected
  */
FLASH_Status DisableBootloaderWriteProtection(void)
{
    FLASH_Status state = FLASH_COMPLETE;

    /* Clear all FLASH flags */  
    FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
                  | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);  
  
    /* Unlocks Flash for write access.*/
    FLASH_If_Init();
    
    /* Unlock the Option Bytes */  
    FLASH_OB_Unlock();
    
    /* Disables the write protection of user application pages */ 
    state = FLASH_OB_WRPConfig((FLASH_OB_GetWRP() | 0x0000000F), DISABLE);

    return state;
}

/**
  * @brief  Disables the write protection of user flash area.
  * @param  None
  * @retval 0: Write Protection successfully disabled
  *         1: Error: Flash write unprotection failed
  *         2: Flash memory is not write protected
  */
uint32_t FLASH_If_DisableWriteProtection(void)
{
  FLASH_Status status = FLASH_BUSY;

  /* Clear all FLASH flags */  
  FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
                  | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);  
  
  /* Test if user memory is write protected */
  if (FLASH_If_GetWriteProtectionStatus() != 0x00)
  {
    /* Unlock the Option Bytes */  
    FLASH_OB_Unlock();

    /* Disable the write protection of user application pages */ 
    status = FLASH_If_WriteProtectionConfig();
    if (status == FLASH_COMPLETE)
    {
      /* Write Protection successfully disabled */
      return (0);
    }
    else
    {
      /* Error: Flash write unprotection failed */
      return (1);
    }
  }
  else
  {
     /* Flash memory is not write protected */
     return(2);
  }
}
/**
  * @brief  Disables the write protection of user flash area.
  * @param  None
  * @retval 0: Write Protection successfully disabled
  *         1: Error: Flash write unprotection failed
  *         2: Flash memory is not write protected
  */
FLASH_Status FLASH_If_DisableAppSpaceWriteProtection(void)
{
    FLASH_Status state = FLASH_COMPLETE;

    /* Clear all FLASH flags */  
    FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
                  | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);  
  
    /* Unlocks Flash for write access.*/
    FLASH_If_Init();
    
    /* Unlock the Option Bytes */  
    FLASH_OB_Unlock();
    
    /* Disable the write protection of user application pages */ 
    state = FLASH_OB_WRPConfig(0xFFFFFFF0, DISABLE);
    if(state == FLASH_COMPLETE)
    {
        state = FLASH_OB_WRP1Config(0x00000003, DISABLE);
    }
    
   return state;
}
/**
  * @brief  Returns the write protection status of user flash area.
  * @param  None
  * @retval If the sector is write-protected, the corresponding bit in returned
  *         value is set.
  *         If the sector isn't write-protected, the corresponding bit in returned
  *         value is reset.
  *         e.g. if only sector 3 is write-protected, returned value is 0x00000008
  */
uint32_t FLASH_If_GetWriteProtectionStatus(void)
{
  return(FLASH_OB_GetWRP() & FLASH_PROTECTED_SECTORS);   
}

/**
  * @brief  Disable the write protection status of user flash area.
  * @param  None
  * @retval If the sector is write-protected, the corresponding bit in returned
  *         value is set.
  *         If the sector isn't write-protected, the corresponding bit in returned
  *         value is reset.
  *         e.g. if only sector 3 is write-protected, returned value is 0x00000008
  */
FLASH_Status FLASH_If_WriteProtectionConfig(void)
{
    FLASH_Status WRPstatus = FLASH_COMPLETE, WRP1status = FLASH_COMPLETE, WRP2status = FLASH_COMPLETE ;
    
	WRPstatus = WRPstatus;
	WRP1status = WRP1status;
    FLASH_OB_WRPConfig(OB_WRP_AllPages, DISABLE);
    WRP1status = FLASH_OB_WRP1Config(OB_WRP1_AllPages, DISABLE);
    WRP2status = FLASH_OB_WRP2Config(OB_WRP2_AllPages, DISABLE);  
    return (WRP2status);
}

/**
  * @}
  */

/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
