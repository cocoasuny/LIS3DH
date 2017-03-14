#include "FWUpdate.h"
#include "flash_if.h"
#include "common.h"


TIM_Cfg_Typedef         Tim_Cfg_FirmwareReceive_Index;    
TIM_Basic_Cfg_Typedef 	Tim_Cfg_FirmwareReceive;
Timer_ID_Typedef        gFirmwareRXTimerID = TIMER_ERROR;

TIM_Cfg_Typedef         Tim_Cfg_FlashWriteTimeout_Index;    
TIM_Basic_Cfg_Typedef 	Tim_Cfg_FlashWriteTimeout;
Timer_ID_Typedef        gFlashWriteTimerID = TIMER_ERROR;

uint32_t u32FWTotelCrc_App = 0;
uint32_t u32FWTotelCnt = 0;
bool bFlagFWUpdateRunning = FALSE;
extern void ResetnRF51822Chip(void);
/*******************************************************************************
* Function Name  : ComputeCRC
* Description    : Compute the CRC of the specified flash space.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint32_t ComputeCRC(uint32_t u32StartAddress,uint32_t u32EndAddress)
{
    uint32_t flashstart = 0,u32Crc_tmp = 0,index = 0;
    uint8_t  rd_flash[256] = {0};

    for(flashstart = u32StartAddress;flashstart < u32EndAddress;flashstart += 256)
    {
        for(index = 0;index < 64;index ++)
        {
            Translateu32Tou8Array(*((uint32_t*)(flashstart + index * 4)),&rd_flash[index * 4]);
        }
        u32Crc_tmp = crc32(u32Crc_tmp,rd_flash, 256);
    }

    return u32Crc_tmp;
}

/*******************************************************************************
* Function Name  : SoftReset
* Description    : Reset STM32 Chip
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SoftReset(void)
{
	__set_FAULTMASK(1);      // Set Fault Mask
	NVIC_SystemReset();      // System Reset
}

/*******************************************************************************
* Function Name  : FWReceiveTimeoutHandler
* Description    : The timeout handler of firmware receive
* Input          : TIMID
* Return         : None
*******************************************************************************/
void FWReceiveTimeoutHandler(Timer_ID_Typedef TIMID)
{
    TIMID = TIMID;
    
#ifdef FWUpdate_Debug
    printf("\r\nDevice_Mode = 0x%x.\r\n\r\n",Device_Mode);
#endif

    Timer_Free(gFirmwareRXTimerID);
    gFirmwareRXTimerID = TIMER_ERROR;
    if((FirmwareUpdateType == Frame_TYPE_STM32APP)||(FirmwareUpdateType == Frame_TYPE_STM32BOOT))
    {
        Timer_Free(gFlashWriteTimerID);
        gFlashWriteTimerID = TIMER_ERROR;
    }

    SPI_FWDataTransmitCtl(STM32ERROR);
    bFlagFWUpdateRunning = FALSE;
    if(Device_Mode == Device_Mode_FWUpdate)
    {
        if(Device_Mode_pre == Device_Mode_Charge)
        {
            Device_Mode = Device_Mode_Charge; 
            TS_SendEvent(gOledDisTaskID,gOledDisEventModeCHARGE_c);  // Return the charge mode
        }
        else
        {
            Device_Mode = Device_Mode_RTC; 
            TS_SendEvent(gOledDisTaskID,gOledDisEventBlueIcon_c);   // Return the RTC mode
            TS_SendEvent(gTsStepTaskID_c, gStepRestartEvent);       // Restart the step function
        }
    }
#ifdef FWUpdate_Debug
    printf("FWReceiveTimeoutHandler.\r\n");
#endif    
}

/*******************************************************************************
* Function Name  : FWFlashWriteTimeoutHandler
* Description    : The timeout handler of write flash
* Input          : TIMID
* Return         : None
*******************************************************************************/
void FWFlashWriteTimeoutHandler(Timer_ID_Typedef TIMID)
{
    TIMID = TIMID;
    
#ifdef FWUpdate_Debug
    printf("\r\nFWFlashWriteTimeoutHandler.\r\n\r\n");
#endif
    Timer_Free(gFirmwareRXTimerID);
    gFirmwareRXTimerID = TIMER_ERROR;
    Timer_Free(gFlashWriteTimerID);
    gFlashWriteTimerID = TIMER_ERROR;
    
    SPI_FWDataTransmitCtl(STM32ERROR);
    bFlagFWUpdateRunning = FALSE;
    if(Device_Mode_pre == Device_Mode_Charge)
    {
        Device_Mode = Device_Mode_Charge; 
        TS_SendEvent(gOledDisTaskID,gOledDisEventModeCHARGE_c);     // Return the charge mode
    }
    else
    {
        Device_Mode = Device_Mode_RTC; 
        TS_SendEvent(gOledDisTaskID,gOledDisEventBlueIcon_c);       // Return the RTC mode
        TS_SendEvent(gTsStepTaskID_c, gStepRestartEvent);           // Restart the step function
    }
}
/*******************************************************************************
* Function Name  : FW_Update_Handler
* Description    : 固件升级任务处理函数
* Input          : Event
* Output         : None
* Return         : None
*******************************************************************************/
void FW_Update_Handler(event_t event)
{
    static uint32_t u32RxFWpackageCnt = 0;
    static uint32_t u32RxFWdataFrameCnt = 0;
    static uint32_t SPI_TransmitCRC32 = 0;
    static uint32_t SPI_TransmitCRC32_pre = 0;
    static uint32_t flashdestination = 0;
    static uint32_t flag_crcCheckErrorinPackage = 0;

    uint16_t  SPI_TransmitCRC16=0;
    FWUpdateInfo_t  FwUpdateInfo;
    #ifdef FWUpdate_Debug
    uint16_t index          = 0;
    #endif
    uint32_t u32Crc = 0;
    uint16_t u16Crc = 0;

    if(Device_Mode == Device_Mode_FWUpdate)
    {
		if(event & gFWUpdatePrepareRxFWDateEvent) //ready to receive firmware
		{
            /* Init the variables about firmware upgrade */
			u32RxFWpackageCnt = 0;
            SPI_TransmitCRC32 = 0;
            u32RxFWdataFrameCnt = 0;
            u32FWTotelCnt = 0;
            flashdestination = FWUPDATE_BUFFER_START_ADDRESS;

            if((FirmwareUpdateType == Frame_TYPE_STM32APP)||(FirmwareUpdateType == Frame_TYPE_STM32BOOT))
            {
                /* Unlock the Flash Program Erase controller and erase the space. */
                FLASH_If_Init();
                FLASH_If_Erase(FWUPDATE_BUFFER_START_ADDRESS);
                
                /* Init timer top define */
                if(gFlashWriteTimerID != TIMER_ERROR)
                {
                    Timer_Free(gFlashWriteTimerID);
                    gFlashWriteTimerID = TIMER_ERROR;
                }
                Tim_Cfg_FlashWriteTimeout.enuTimerType              = TIM_TYPE_MS;
                Tim_Cfg_FlashWriteTimeout.u16TimePeriod             = FWFlashWriteTimeOutTIM;
                Tim_Cfg_FlashWriteTimeout.NVIC_IRQChannelPreemptionPriority = OLEDShutDown_TIMPreemptionPriority;
                Tim_Cfg_FlashWriteTimeout.pIntCallBack              = FWFlashWriteTimeoutHandler;
                Tim_Cfg_FlashWriteTimeout_Index.TimerMode 			= TIM_MODE_BASIC;
                Tim_Cfg_FlashWriteTimeout_Index.TimerBasicCfg 		= &Tim_Cfg_FlashWriteTimeout;
                Tim_Cfg_FlashWriteTimeout_Index.TimerPWMCfg 		= NULL;
                gFlashWriteTimerID                                  = Timer_Allocate(&Tim_Cfg_FlashWriteTimeout_Index);
                if(gFlashWriteTimerID == TIMER_ERROR)
                {
                    #ifdef FWUpdate_Debug
                    printf("\r\n***Allocate FlashWriteTimer error***\r\n");
                    #endif
                    SPI_FWDataTransmitCtl(STM32ERROR);
                    if(Device_Mode == Device_Mode_FWUpdate)
                    {
                        if(Device_Mode_pre == Device_Mode_Charge)
                        {
                            Device_Mode = Device_Mode_Charge; 
                            TS_SendEvent(gOledDisTaskID,gOledDisEventModeCHARGE_c);  // Return the charge mode
                        }
                        else
                        {
                            Device_Mode = Device_Mode_RTC; 
                            TS_SendEvent(gOledDisTaskID,gOledDisEventBlueIcon_c);   // Return the RTC mode
                            TS_SendEvent(gTsLEDTaskID_c,gGreenLEDFlashingStop);
                            TS_SendEvent(gTsStepTaskID_c, gStepRestartEvent);       // Restart the step function
                        }
                    }
                }
            }

            #ifdef FWUpdate_Debug
                printf("Allocate timer...\r\n");
            #endif
            /* Init timer top define */
            if(gFirmwareRXTimerID != TIMER_ERROR)
            {
                Timer_Free(gFirmwareRXTimerID);
                gFirmwareRXTimerID = TIMER_ERROR;
            }
            Tim_Cfg_FirmwareReceive.enuTimerType                = TIM_TYPE_MS;
            Tim_Cfg_FirmwareReceive.u16TimePeriod               = FwReceiveTimeOutTIM;
            Tim_Cfg_FirmwareReceive.NVIC_IRQChannelPreemptionPriority = OLEDShutDown_TIMPreemptionPriority;
            Tim_Cfg_FirmwareReceive.pIntCallBack                = FWReceiveTimeoutHandler;
            Tim_Cfg_FirmwareReceive_Index.TimerMode 			= TIM_MODE_BASIC;
            Tim_Cfg_FirmwareReceive_Index.TimerBasicCfg 		= &Tim_Cfg_FirmwareReceive;
            Tim_Cfg_FirmwareReceive_Index.TimerPWMCfg 		    = NULL;
            gFirmwareRXTimerID                                  = Timer_Allocate(&Tim_Cfg_FirmwareReceive_Index);
            if(gFirmwareRXTimerID == TIMER_ERROR)
            {
                #ifdef FWUpdate_Debug
                printf("\r\n***Allocate FirmwareRXTimer error***\r\n");
                #endif
                SPI_FWDataTransmitCtl(STM32ERROR);
                if(Device_Mode == Device_Mode_FWUpdate)
                {
                    if(Device_Mode_pre == Device_Mode_Charge)
                    {
                        Device_Mode = Device_Mode_Charge; 
                        TS_SendEvent(gOledDisTaskID,gOledDisEventModeCHARGE_c);  // Return the charge mode
                    }
                    else
                    {
                        Device_Mode = Device_Mode_RTC; 
                        TS_SendEvent(gOledDisTaskID,gOledDisEventBlueIcon_c);   // Return the RTC mode
                        TS_SendEvent(gTsLEDTaskID_c,gGreenLEDFlashingStop);
                        TS_SendEvent(gTsStepTaskID_c, gStepRestartEvent);       // Restart the step function
                    }
                }
            }
            else
            {
                Start_Timer_Cnt(gFirmwareRXTimerID);
            }

            OLED_DisplayClear();
            OLED_DisplayFullScreenBMP(MXS8475_FirmwareUpdating);
            
            TS_SendEvent(gTsLEDTaskID_c,gGreenLEDFlashingStart);
            
            TS_SendEvent(gOledDisTaskID,gOledDisEventClearModeDis_c);  //close the OLED display timer
            
            /* 通知51822，开始接收固件数据,51822通知App发送固件数据 */
            SPI_FWDataTransmitCtl(TransmitFWStart); 
		}
        else if(event & gFWUpdateRxFWDateEvent)  //处理接收到的固件数据128字节 + 最后2字节CRC
		{
            #ifdef FWUpdate_Debug
                printf("Stop_Timer_Cnt...\r\n");
            #endif
            
            Stop_Timer_Cnt(gFirmwareRXTimerID);
            Start_Timer_Cnt(gFirmwareRXTimerID);
            
            if((FirmwareUpdateType == Frame_TYPE_STM32APP)||(FirmwareUpdateType == Frame_TYPE_STM32BOOT))
            {
                #ifdef FWUpdate_Debug
                printf("gu32RxFWpackageCnt = %d,u32RxFWdataFrameCnt = %d,flag_crcCheckErrorinPackage = %d.\r\n",
                        u32RxFWpackageCnt,u32RxFWdataFrameCnt,flag_crcCheckErrorinPackage);
                #endif
                /* Calculate the 32-bit crc value and the 16-bit crc value */
                if(!u32RxFWdataFrameCnt)
                {
                    SPI_TransmitCRC32_pre = 0;
                }
                if(flag_crcCheckErrorinPackage)
                {   
                    flag_crcCheckErrorinPackage = 0;
                    SPI_TransmitCRC32 = SPI_TransmitCRC32_pre;
                }
                #ifdef FWUpdate_Debug
                printf("SPI_TransmitCRC32_pre = 0x%x,SPI_TransmitCRC32 = 0x%x.\r\n",SPI_TransmitCRC32_pre,SPI_TransmitCRC32);
                #endif
                SPI_TransmitCRC32 = crc32(SPI_TransmitCRC32,gFWData,128); 
                SPI_TransmitCRC16 = crc32_To_crc16(SPI_TransmitCRC32);  
                #ifdef FWUpdate_Debug
                    printf("SPI_TransmitCRC32 = 0x%x,SPI_TransmitCRC16 = [0x%x,0x%x]\r\n",SPI_TransmitCRC32,SPI_TransmitCRC16 >> 8,(uint8_t)(SPI_TransmitCRC16 & 0x00FF));
                    printf("rx crc in FWData = [0x%x,0x%x]\r\n",gFWData[128],gFWData[129]);
                #endif
                
                /* Check the crc */
                if((gFWData[128] == (uint8_t)(SPI_TransmitCRC16 >> 8)) && (gFWData[129] == (uint8_t)(SPI_TransmitCRC16 & 0x00FF))) //	CRC successful
                {
                    #ifdef FWUpdate_Debug
                        printf("CRC ok,flashdestination = 0x%x,u32RxFWdataFrameCnt = %d...\r\n",flashdestination,u32RxFWdataFrameCnt);
                    #endif
                    SPI_TransmitCRC32_pre = SPI_TransmitCRC32;
                    Start_Timer_Cnt(gFlashWriteTimerID);
                    /* Write received data in Flash */
                    if (!FLASH_If_Write(&flashdestination, (uint32_t*) gFWData, (uint16_t)32))
                    {
                        #ifdef FWUpdate_Debug
                        printf("Write OK...\r\n");
                        #endif
                        /* Ask nRF51822 to send remain firmware */
                        SPI_FWDataTransmitCtl(TransmitFWRemain);
                    }
                    else
                    {
                        #ifdef FWUpdate_Debug
                        printf("Write Error...\r\n");
                        #endif
                        /* Notify app the hardware is error */
                        SPI_FWDataTransmitCtl(STM32ERROR);      // STM32 error
                    } 
                    
                    Stop_Timer_Cnt(gFlashWriteTimerID);
                    u32RxFWdataFrameCnt ++; 
                    u32RxFWpackageCnt++;
                }
                else  // CRC Check error
                {
                    flag_crcCheckErrorinPackage = 1;
                    #ifdef FWUpdate_Debug
                    printf("CRC error,flashdestination = 0x%x,u32RxFWdataFrameCnt = %d...\r\n",flashdestination,u32RxFWdataFrameCnt);
                    for(index = 0;index < 128;index ++)
                    {
                        printf("0x%x,",gFWData[index]);
                    }
                    printf("\r\n");
                    #endif
                    
                    /* re-send current package */
                    SPI_FWDataTransmitCtl(RepeatTransmitCurFW);
                }
                    
                /* Clear the data buffers */
                memset(gFWData,0,FIRMWARE_UPDATE_BUF_LEN*sizeof(uint8_t));
                if(u32RxFWdataFrameCnt >= 4)
                {
                    u32RxFWdataFrameCnt = 0;
                    SPI_TransmitCRC32 = 0;
                    SPI_TransmitCRC32_pre = 0;
                    flag_crcCheckErrorinPackage = 0;
                }
            }
            else
            {
                #ifdef FWUpdate_Debug
                    printf("Received nrf firmware data...\r\n");
                #endif
                                 
                /* Ask nRF51822 to send remain firmware */
                SPI_FWDataTransmitCtl(TransmitFWRemain);                
            }
		}
        else if(event & gFWUpdateStartFWDateEvent)  // The Fw has been translated completely,and switches to the bootloader
		{
            Stop_Timer_Cnt(gFirmwareRXTimerID);
            Timer_Free(gFirmwareRXTimerID);
            gFirmwareRXTimerID = TIMER_ERROR;
            bFlagFWUpdateRunning = FALSE;

            if((FirmwareUpdateType == Frame_TYPE_STM32APP)||(FirmwareUpdateType == Frame_TYPE_STM32BOOT))
            {
                #ifdef FWUpdate_Debug
                printf("Ready for restart STM32...\r\n");
                #endif
                
                Timer_Free(gFlashWriteTimerID);
                gFlashWriteTimerID = TIMER_ERROR;
                
                u32Crc = 0;
                u32Crc = crc32(u32Crc,(uint8_t *)FWUPDATE_BUFFER_START_ADDRESS, u32FWTotelCnt);
                u16Crc = crc32_To_crc16(u32Crc);
                #ifdef FWUpdate_Debug
                printf("u16Crc = 0x%04x,u32FWTotelCrc_App = 0x%04x...\r\n",u16Crc,u32FWTotelCrc_App);
                #endif
                if(u16Crc != u32FWTotelCrc_App)
                {
                    SPI_FWDataTransmitCtl(STM32ERROR);
                    if(Device_Mode == Device_Mode_FWUpdate)
                    {
                        if(Device_Mode_pre == Device_Mode_Charge)
                        {
                            Device_Mode = Device_Mode_Charge;
                            TS_SendEvent(gOledDisTaskID,gOledDisEventModeCHARGE_c);  // Return the charge mode
                        }
                        else
                        {
                            Device_Mode = Device_Mode_RTC;
                            TS_SendEvent(gOledDisTaskID,gOledDisEventBlueIcon_c);   // Return the RTC mode
                            TS_SendEvent(gTsLEDTaskID_c,gGreenLEDFlashingStop);
                            TS_SendEvent(gTsStepTaskID_c, gStepRestartEvent);       // Restart the step function
                        }
                    }
                    return;
                }

                u32Crc = 0;
                if(FirmwareUpdateType == Frame_TYPE_STM32APP)
                {
                    u32Crc = ComputeCRC(FWUPDATE_BUFFER_START_ADDRESS,FWUPDATE_BUF_FLASH_END_ADDRESS);
                }
                else
                {
                    u32Crc = ComputeCRC(FWUPDATE_BUFFER_START_ADDRESS,FWUPDATE_BUFFER_START_ADDRESS + 0x3f00);
                }

                /* Update the flag in DataMemory */
                FwUpdateInfo.FWUpdateUPGRADEStatus = M95M01_UPGRADE;
                FwUpdateInfo.FWUpdateFWDATAStatus = M95M01_FWDATA_OK;
                FwUpdateInfo.FWUpdateCRC[0] = u32Crc;
                FwUpdateInfo.FWUpdateCRC[1] = u32Crc >> 8;
                FwUpdateInfo.FWUpdateCRC[2] = u32Crc >> 16;
                FwUpdateInfo.FWUpdateCRC[3] = u32Crc >> 24;
                
                if(FirmwareUpdateType == Frame_TYPE_STM32APP)
                {
                    FwUpdateInfo.FWUpdateFIRMWAREType = M95M01_APP_FIRMWARE;
                }
                else
                {
                    FwUpdateInfo.FWUpdateFIRMWAREType = M95M01_BOOT_FIRMWARE;
                }
                
                SetFWUpDateConfigInfo(FwUpdateInfo);
                
                if(FirmwareUpdateType == Frame_TYPE_STM32APP)
                {
                    FLASH_If_DisableAppSpaceWriteProtection();
                }
                else
                {
                    DisableBootloaderWriteProtection();
                }
                
                #ifdef FWUpdate_Debug
                    printf("STM32 firmware update complete,Reset...\r\n\r\n\r\n");
                #endif
                ExtFLASH_SaveRdAddrToConst();
                ResetnRF51822Chip();

                /* Launch the option byte loading */
                FLASH_OB_Launch();
                SoftReset();
            }
            else
            {
                ExtFLASH_SaveRdAddrToConst();
                #ifdef FWUpdate_Debug
                    printf("nRF5188 firmware update complete,Reset...\r\n\r\n\r\n");
                #endif
                RCC->APB1ENR &= 0x00;
                RCC->APB2ENR &= 0x00;
                RCC->AHBENR  &= 0x00;
                SoftReset();
            }
		}
        else if(event & gFWUpdateStartFWNRFErrorEvent)
        {
            Timer_Free(gFirmwareRXTimerID);
            gFirmwareRXTimerID = TIMER_ERROR;
            Timer_Free(gFlashWriteTimerID);
            gFlashWriteTimerID = TIMER_ERROR;

            SPI_FWDataTransmitCtl(STM32ERROR);
            bFlagFWUpdateRunning = FALSE;
            if(Device_Mode_pre == Device_Mode_Charge)
            {
                Device_Mode = Device_Mode_Charge;
                TS_SendEvent(gOledDisTaskID,gOledDisEventModeCHARGE_c);     // Return the charge mode
            }
            else
            {
                Device_Mode = Device_Mode_RTC;
                TS_SendEvent(gOledDisTaskID,gOledDisEventBlueIcon_c);       // Return the RTC mode
                TS_SendEvent(gTsLEDTaskID_c,gGreenLEDFlashingStop);
                TS_SendEvent(gTsStepTaskID_c, gStepRestartEvent);           // Restart the step function
            }
        }
    }
}
