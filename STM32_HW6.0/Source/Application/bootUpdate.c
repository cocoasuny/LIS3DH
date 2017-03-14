#include "bootUpdate.h"
#include <stdint.h>
#include "common.h"
#include "flash_if.h"



void CopyFirmware(void)
{
    uint32_t index = 0;
    uint32_t flashdestBuffer[64] = {0};
	static uint32_t flashdestination = 0;
	static uint32_t flashsource = 0;

    flashsource = BUFFER_START_ADDRESS;
    for(flashdestination = BOOTLOADER_START_ADDRESS;flashdestination  < BOOTLOADER_END_ADDRESS;flashdestination += 256,flashsource += 256)
    {
        for(index = 0;index < 64;index ++)
        {
            flashdestBuffer[index] = *((uint32_t*)(flashsource + index * 4));
        }

        FLASH_ErasePage(flashdestination);
        FLASH_ProgramHalfPage(flashdestination, flashdestBuffer);
        FLASH_ProgramHalfPage(flashdestination + 128, flashdestBuffer + 32);
    }
}

bool FlashCrcCheck(uint32_t u32StartAddress,uint32_t u32EndAddress,uint32_t crc)
{
    uint32_t u32Crc_tmp = 0;

    /* Calculates the crc value in specified area */
    u32Crc_tmp = ComputeCRC(u32StartAddress,u32EndAddress);

    #ifdef FWUpdate_Debug
    printf("u32Crc_tmp = 0x%x,crc = 0x%x...\r\n",u32Crc_tmp,crc);
    #endif
    if(crc == u32Crc_tmp)
    {
        return true;
    }
    else
    {
        return false;
    }
}

FLASH_Status EnableBootloaderWriteProtection(void)
{
    FLASH_Status state = FLASH_COMPLETE;

    /* Clear all FLASH flags */
    FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
                  | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);

    /* Unlocks Flash for write access.*/
    FLASH_If_Init();

    /* Unlock the Option Bytes */
    FLASH_OB_Unlock();

    /* Enables the write protection of user application pages */
    state = FLASH_OB_WRPConfig(FLASH_OB_GetWRP() & 0x00003FFF, ENABLE);
    if(state == FLASH_COMPLETE)
    {
        state = FLASH_OB_WRPConfig(0x00000003, ENABLE);
    }

    return state;
}

void UpdateBootloader(uint32_t crc)
{
    uint8_t index  = 0;
    FWUpdateInfo_t FWUpdateInfo;

    if(FlashCrcCheck(FWUPDATE_BUFFER_START_ADDRESS,(FWUPDATE_BUFFER_START_ADDRESS + 0x3F00),crc))
    {
        /* Unlock the Flash Program Erase controller. */
        FLASH_If_Init();

        for(index = 0;index < RECOPY_COUNT_MAX;index ++)
        {
            CopyFirmware();
            if(FlashCrcCheck(BOOTLOADER_START_ADDRESS,(BOOTLOADER_START_ADDRESS + 0x3F00),crc))
            {
                break;
            }
        }

        /* Update the flags in the EEPROM */
        FWUpdateInfo.FWUpdateUPGRADEStatus = M95M01_NO_UPGRADE;
        FWUpdateInfo.FWUpdateFWDATAStatus = M95M01_FWDATA_ERROR;
        FWUpdateInfo.FWUpdateCRC[0] = 0;
        FWUpdateInfo.FWUpdateCRC[1] = 0;
        FWUpdateInfo.FWUpdateCRC[2] = 0;
        FWUpdateInfo.FWUpdateCRC[3] = 0;
        FWUpdateInfo.FWUpdateFIRMWAREType = M95M01_NO_NEW_FIRMWARE;

        SetFWUpDateConfigInfo(FWUpdateInfo);

        /* If the copy times > RECOPY_COUNT_MAX,need to tell the user to send back to the manufactory.*/
        if(index >= RECOPY_COUNT_MAX)
        {
            #ifdef FWUpdate_Debug
            printf("FLASHcopy is ERROR...\r\n");
            #endif
            OLED_DisplayClear();
            OLED_DisplayFullScreenBMP(MXS8475_FirmwareError);
            while(1); /* 	Dead Lock for Error writing 		*/
        }

        /* Enables the Write Protection of Bootloader */
        EnableBootloaderWriteProtection();

        /* Launch the option byte loading */
        FLASH_OB_Launch();

        /* SoftReset */
        SoftReset();
    }
    else
    {
        #ifdef FWUpdate_Debug
        printf("Before copying,crccheck is error...\r\n");
        #endif
    }
}
