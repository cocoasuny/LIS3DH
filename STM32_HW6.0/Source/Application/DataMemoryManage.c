/**
  ******************************************************************************
  * @file    RedHare/STM32L152/DateMemoryManage.c
  * @author  Firmware Development Group
  * @version V1.0.0
  * @date    2015\8
  * @brief   DateMemoryManage.c program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 iCareTech </center></h2>
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "DataMemoryManage.h"
#include "platform.h"
#include "DataEEPROM.h"
#include "StoreManage.h"
#include "App_error.h"
/*******************************************************************************
* Function Name  : GetCPUConfigInfo
* Description    : Get Cpu Config Information when hard fault
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GetCPUConfigInfo(void)
{
    /* Cpu Config Info : 0:Month;
                         1:Day;
                         2:Hour
                         3:Min
                         4:Sec
                         5: lr (4 byte)
                         7: pc (4 byte)
    */

    uint8_t ConfigFile[20] = {0};   //max 20byte of Sys Config File
    unsigned int stacked_lr = 0;
    unsigned int stacked_pc = 0;

    /* Read CPU Config File From STM32 Internal EEPROM */
    DataEEPROM_MultiByteRead(ConfigFile,EEPROM_CONFIG_CPUINFO,sizeof(ConfigFile));

	printf ("[Hard fault handler]\r\n");
	printf("Hard fault occured Time(MM/DD/HH/MM/SS): %d:%d:%d:%d:%d\r\n",
						ConfigFile[0],ConfigFile[1],ConfigFile[2],ConfigFile[3],ConfigFile[4]);

    /* lr */
    stacked_lr = ((ConfigFile[5] << 24) & 0xFF000000) | (uint8_t)((ConfigFile[6] << 16) & 0x00FF0000)
               | ((ConfigFile[7] << 8)  & 0x0000FF00) | ((ConfigFile[8] << 0)  & 0x000000FF);

    /* pc */
    stacked_pc = ((ConfigFile[9] << 24) & 0xFF000000) | (uint8_t)((ConfigFile[10] << 16) & 0x00FF0000)
               | ((ConfigFile[11] << 8)  & 0x0000FF00) | ((ConfigFile[12] << 0)  & 0x000000FF);

    printf ("LR = %0.8x\r\n", stacked_lr);
	printf ("PC = %0.8x\r\n", stacked_pc);
}
/*******************************************************************************
* Function Name  : SetCPUConfigInfo
* Description    : Set Cpu Config Information when hard fault
* Input          : SysConfigCPUInfo_t *info
* Output         : Nont
* Return         : None
*******************************************************************************/
void SetCPUConfigInfo(SysConfigCPUInfo_t info)
{
    /* Cpu Config Info : 0:Month;
                         1:Day;
                         2:Hour
                         3:Min
                         4:Sec
                         5: lr (4 byte)
                         7: pc (4 byte)
    */

    uint8_t ConfigFile[20] = {0};   //max 20byte of Sys Config File


    ConfigFile[0] = info.Month;
    ConfigFile[1] = info.Day;
    ConfigFile[2] = info.Hour;
    ConfigFile[3] = info.Min;
    ConfigFile[4] = info.Sec;

    /* lr */
    ConfigFile[5] = (uint8_t)((info.lr >> 24) & 0x000000FF);
	ConfigFile[6] = (uint8_t)((info.lr >> 16) & 0x000000FF);
	ConfigFile[7] = (uint8_t)((info.lr >> 8)  & 0x000000FF);
	ConfigFile[8] = (uint8_t)((info.lr >> 0)  & 0x000000FF);

    /* pc */
	ConfigFile[9] = (uint8_t)((info.pc >> 24) & 0x000000FF);
	ConfigFile[10] = (uint8_t)((info.pc >> 16) & 0x000000FF);
	ConfigFile[11] = (uint8_t)((info.pc >> 8)  & 0x000000FF);
	ConfigFile[12] = (uint8_t)((info.pc >> 0)  & 0x000000FF);


    /* Write Cpu Config File From STM32 Internal EEPROM */
    DataEEPROM_MultiByteWrite(ConfigFile,EEPROM_CONFIG_CPUINFO,sizeof(ConfigFile));
}

/*******************************************************************************
* Function Name  : GetSysConfigInfo
* Description    : Get System Config Information
* Input          : SysConfigInfo_t *info
* Output         : SysConfigInfo_t *info
* Return         : None
*******************************************************************************/
void GetSysConfigInfo(SysConfigInfo_t *info)
{
    /* Config Factory Info : 0:Flag of App active or not;
                             1:Flag of App Modify SN or not;
                             2~5:App Modify SN Info
                             6:  Moniter Template ID   //not Needed from HWV4.1
                             7:  FreeRunMonitor datalength
    */

    uint8_t ConfigFile[20] = {0};   //max 20byte of Sys Config File

    /* Read Sys Config File From STM32 Internal EEPROM */
    DataEEPROM_MultiByteRead(ConfigFile,EEPROM_CONFIG_FACTORYINFO,sizeof(ConfigFile));

    info->AppActiveStatus = ConfigFile[0];
    info->AppModifySNStatus = ConfigFile[1];
    info->AppModifySNInfo[0] = ConfigFile[2];
    info->AppModifySNInfo[1] = ConfigFile[3];
    info->AppModifySNInfo[2] = ConfigFile[4];
    info->AppModifySNInfo[3] = ConfigFile[5];
    info->MTID = ConfigFile[6];
    info->DataLength = ConfigFile[7];
    info->DataFormate = ConfigFile[8];
}
/*******************************************************************************
* Function Name  : SetSysConfigInfo
* Description    : Set System Config Information
* Input          : SysConfigInfo_t *info
* Output         : SysConfigInfo_t *info
* Return         : None
*******************************************************************************/
void SetSysConfigInfo(SysConfigInfo_t info)
{
    /* Config Factory Info : 0:Flag of App active or not;
                         1:Flag of App Modify SN or not;
                         2~5:App Modify SN Info
                         6:  Moniter Template ID
                         7:  FreeRunMonitor datalength
    */

    uint8_t ConfigFile[20] = {0};   //max 20byte of Sys Config File


    ConfigFile[0] = info.AppActiveStatus;
    ConfigFile[1] = info.AppModifySNStatus;
    ConfigFile[2] = info.AppModifySNInfo[0];
    ConfigFile[3] = info.AppModifySNInfo[1];
    ConfigFile[4] = info.AppModifySNInfo[2];
    ConfigFile[5] = info.AppModifySNInfo[3];
    ConfigFile[6] = info.MTID;
    ConfigFile[7] = info.DataLength;
    ConfigFile[8] = info.DataFormate;

    /* Write Sys Config File From STM32 Internal EEPROM */
    DataEEPROM_MultiByteWrite(ConfigFile,EEPROM_CONFIG_FACTORYINFO,sizeof(ConfigFile));
}

/*******************************************************************************
* Function Name  : GetFWUpDateConfigInfo
* Description    : Get FW Update Config Information
* Input          : FWUpdateInfo_t *info
* Output         : FWUpdateInfo_t *info
* Return         : None
*******************************************************************************/
void GetFWUpDateConfigInfo(FWUpdateInfo_t *info)
{
    /* Config FW Update Config Info : 0:Flag of FW Update UPGRADE Status;
                                      1:Flag of FW Update FWDATA  Status;
                                      2~5:FW Update CRC Info
                                      6:Flag of FW Update FIRMWARE Type
    */

    uint8_t ConfigFile[10] = {0};   //max 10byte of FW Update Config File

    /* Read Sys Config File From STM32 Internal EEPROM */
    DataEEPROM_MultiByteRead(ConfigFile,EEPROM_CONFIG_UPGRADE_FLAG,sizeof(ConfigFile));

    info->FWUpdateUPGRADEStatus = ConfigFile[0];
    info->FWUpdateFWDATAStatus = ConfigFile[1];
    info->FWUpdateCRC[0] = ConfigFile[2];
    info->FWUpdateCRC[1] = ConfigFile[3];
    info->FWUpdateCRC[2] = ConfigFile[4];
    info->FWUpdateCRC[3] = ConfigFile[5];
    info->FWUpdateFIRMWAREType = ConfigFile[6];
}
/*******************************************************************************
* Function Name  : SetFWUpDateConfigInfo
* Description    : Set FW Update Config Information
* Input          : FWUpdateInfo_t *info
* Output         : FWUpdateInfo_t *info
* Return         : None
*******************************************************************************/
void SetFWUpDateConfigInfo(FWUpdateInfo_t info)
{
    /* Config FW Update Config Info : 0:Flag of FW Update UPGRADE Status;
                                      1:Flag of FW Update FWDATA  Status;
                                      2~5:FW Update CRC Info
                                      6:Flag of FW Update FIRMWARE Type
    */

    uint8_t ConfigFile[10] = {0};   //max 10byte of Sys Config File


    ConfigFile[0] = info.FWUpdateUPGRADEStatus;
    ConfigFile[1] = info.FWUpdateFWDATAStatus;
    ConfigFile[2] = info.FWUpdateCRC[0];
    ConfigFile[3] = info.FWUpdateCRC[1];
    ConfigFile[4] = info.FWUpdateCRC[2];
    ConfigFile[5] = info.FWUpdateCRC[3];
    ConfigFile[6] = info.FWUpdateFIRMWAREType;

    /* Write Sys Config File From STM32 Internal EEPROM */
    DataEEPROM_MultiByteWrite(ConfigFile,EEPROM_CONFIG_UPGRADE_FLAG,sizeof(ConfigFile));
}

/*******************************************************************************
* Function Name  : SetStorageConstVarConfigInfo
* Description    :
* Input          : StorageReadConstVar_t info
* Output         : None
* Return         : None
*******************************************************************************/
void SetStorageConstVarConfigInfo(StorageReadConstVar_t info)
{
    uint8_t ConfigFile[EEPROM_STORAGE_STORAGE_CONST_LEN] = {0};   //max 10byte of Sys Config File
    StorageReadConstVar_t *p_StorageReadConstVar = (StorageReadConstVar_t *)ConfigFile;

    #ifdef FLASH_DEBUG
		printf("\r\n[FLASH] SetStorageConstVarConfigInfo\r\n");
    #endif

    p_StorageReadConstVar->flash_read_partition.start_time = info.flash_read_partition.start_time;
    p_StorageReadConstVar->flash_read_partition.start_page_addr = info.flash_read_partition.start_page_addr;
    p_StorageReadConstVar->flash_read_partition.end_page_addr = info.flash_read_partition.end_page_addr;
    p_StorageReadConstVar->u32StorageReadAddr = info.u32StorageReadAddr;

    /* Write storage const variables to STM32 Internal EEPROM */
    DataEEPROM_MultiByteWrite(ConfigFile,EEPROM_CONFIG_STORAGE_CONST_VAR,sizeof(ConfigFile));
}

/*******************************************************************************
* Function Name  : GetStorageConstVarConfigInfo
* Description    :
* Input          : StorageConstVar_t *p_Info
* Output         : StorageConstVar_t *p_Info
* Return         : None
*******************************************************************************/
void GetStorageConstVarConfigInfo(StorageReadConstVar_t *p_Info)
{
    uint8_t ConfigFile[EEPROM_STORAGE_STORAGE_CONST_LEN] = {0};
    StorageReadConstVar_t *p_StorageReadConstVar = (StorageReadConstVar_t *)ConfigFile;

    /* Check parameters */
    usr_para_check(p_Info != NULL);

    /* Read storage const variables from STM32 Internal EEPROM */
    DataEEPROM_MultiByteRead(ConfigFile, EEPROM_CONFIG_STORAGE_CONST_VAR, sizeof(ConfigFile));

    p_Info->flash_read_partition.start_time = p_StorageReadConstVar->flash_read_partition.start_time;
    p_Info->flash_read_partition.start_page_addr = p_StorageReadConstVar->flash_read_partition.start_page_addr;
    p_Info->flash_read_partition.end_page_addr = p_StorageReadConstVar->flash_read_partition.end_page_addr;
    p_Info->u32StorageReadAddr = p_StorageReadConstVar->u32StorageReadAddr;
}

/*******************************************************************************
* Function Name  : DataMemoryWrite
* Description    : Write Data to Memory witch Store the data
* Input          : store_type_e,pu8Src,u8Length
* Output         : Nonr
* Return         : Msg
*******************************************************************************/
uint8_t DataMemoryWrite(store_type_e type,uint8_t * pu8Src,uint8_t u8Length)
{
    flashOperationMsg  Msg;
    uint8_t          Stat;

    Msg = ExtFLASH_ExtWrite(type, pu8Src, u8Length);

    if(Msg == FLASH_OPERATION_SUCCESS)
    {
        Stat = APP_SUCCESS;
    }
    else
    {
        Stat = ERR_WRITE_EEPROM;
    }

    return Stat;
}
/*******************************************************************************
* Function Name  : GetDataMemoryTotleLength
* Description    : get the totle length of the data in the DataMemory
* Input          : None
* Output         : None
* Return         : length
*******************************************************************************/
uint32_t GetDataMemoryTotleLength(void)
{
    return  (ExtFLASH_ExtGetTotelLen());
}
/*******************************************************************************
* Function Name  : DataMemoryRead
* Description    : Read Data from Memory witch Store the data
* Input          : store_type_e,pu8Src,u8Length
* Output         : Nonr
* Return         : Msg
*******************************************************************************/
uint8_t DataMemoryRead(store_type_e type,MTDataBuffer_t * p_storageExtBuffer,uint8_t u8Length)
{
    flashOperationMsg  Msg;
    uint8_t          Stat;
    storageExtBuffer_t buf;

    MX25_SPI_Configuration();

    Msg = ExtFLASH_ExtRead(type, &buf, u8Length);

    p_storageExtBuffer->availabe_length = buf.availabe_length;
    p_storageExtBuffer->start_time = buf.start_time;
    memcpy(p_storageExtBuffer->Buffer,buf.Buffer,buf.availabe_length);


    if(Msg == FLASH_OPERATION_SUCCESS)
    {
        Stat = APP_SUCCESS;
    }
    else
    {
        Stat = ERR_WRITE_EEPROM;
    }

    return Stat;
}
/*******************************************************************************
* Function Name  : DataMemoryReadACK
* Description    : Ack of Read Data
* Input          : u8Length
* Output         : Nonr
* Return         : Msg
*******************************************************************************/
uint8_t DataMemoryReadACK(uint8_t u8ValidLength)
{
    flashOperationMsg  Msg;
    uint8_t          Stat;

    Msg = ExtFLASH_ExtReadACK(u8ValidLength);

    if(Msg == FLASH_OPERATION_SUCCESS)
    {
        Stat = APP_SUCCESS;
    }
    else
    {
        Stat = ERR_WRITE_EEPROM;
    }

    return Stat;
}

/*******************************************************************************
* Function Name  : DataMemoryRestore
* Description    : Recovery the Data of DataMemory
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DataMemoryRestore(void)
{
    ExtFLASH_RestorePartition();
    ExtFLASH_StorageInit();
}
/*******************************************************************************
* Function Name  : DataMemoryEraseAll
* Description    : Erase All Data of DataMemory
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DataMemoryEraseAll(void)
{
	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] DataMemoryEraseAll\r\n");
	#endif
    MX25_SPI_Configuration();
    CMD_CE();
    Delay_ms(200);
	ExtFLASH_ClearRAM();
}

/*******************************************************************************
* Function Name  : SetOSAStartTime
* Description    : Save The OSA Monitor Template Start time
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SetOSAStartTime(void)
{
    date_str_typedef    date_s;               //RTC date
	RTC_TimeTypeDef     rtc_time;             //RTC time
    uint8_t 	        bufForStorage[FREERUN_STORAGE_HEADER_LEN] = {0}; //the buffer to data storage

    /* Get the Current Time */
	Calendar_Get(&date_s,&rtc_time);

	/* 	Init the buffer for data header 	*/
	bufForStorage[0] = '#';
	bufForStorage[1]=(uint8_t)(date_s.year/100);
	bufForStorage[2]=(uint8_t)(date_s.year%100);
	bufForStorage[3]=date_s.month;
	bufForStorage[4]=date_s.day;
	bufForStorage[5]=rtc_time.RTC_Hours;
	bufForStorage[6]=rtc_time.RTC_Minutes;
	bufForStorage[7]=rtc_time.RTC_Seconds;
	bufForStorage[8]='#';

    /* 	Write initial header into STM32 Internal EEPROM   */
    DataEEPROM_MultiByteWrite(bufForStorage,EEPROM_CONFIG_OSASTARTTIME,sizeof(bufForStorage));
}
/*******************************************************************************
* Function Name  : GetOSAStartTime
* Description    : Get The OSA Monitor Template Start time
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GetOSAStartTime(uint8_t * time,uint8_t u8Length)
{
    uint8_t 	bufForStorage[FREERUN_STORAGE_HEADER_LEN] = {0};				//the buffer to data storage
    uint8_t     i=0;

    /* 	Read initial header From STM32 Internal EEPROM   */
    DataEEPROM_MultiByteRead(bufForStorage,EEPROM_CONFIG_OSASTARTTIME,sizeof(bufForStorage));

    if(u8Length == FREERUN_STORAGE_HEADER_LEN)
    {
        for(i=0;i<FREERUN_STORAGE_HEADER_LEN;i++)
        {
            time[i] = bufForStorage[i];
        }
    }
}

/*******************************************************************************
* Function Name  : SetFreeRunNightStartFram
* Description    : Save Free Run Monitor Start Frame
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SetFreeRunNightStartFrame(void)
{
    date_str_typedef    date_s;               //RTC date
	RTC_TimeTypeDef     rtc_time;             //RTC time
    uint8_t 	        bufForStorage[MT_STORAGE_DATA_LEN] = {0}; //the buffer to data storage
    uint8_t		        Err_Code=0;
    uint32_t            u32Date = 0;

    #ifdef Monitor_Template_Debug
        printf("\r\nSetFreeRunNightStartFrame\r\n");
    #endif

    /* Get the Current Time */
	Calendar_Get(&date_s,&rtc_time);

	/* 	Init the buffer for data header 	*/
	bufForStorage[0] = rtc_time.RTC_Hours;
	bufForStorage[1] = rtc_time.RTC_Minutes;
	bufForStorage[2] = rtc_time.RTC_Seconds;
	bufForStorage[3] = FreeRunNightstartFram;
    bufForStorage[4] = MonitorTemplate.SampleFreq;
    bufForStorage[5] = MonitorTemplate.SetMTID;
    bufForStorage[6] = 0;
    bufForStorage[7] = 0;

    /* 	Begin to store the data */
    /* Create The Moniter Template Data Store Partition */
    u32Date = CovernDateto32();
    ExtFLASH_ExtCreatePartition(u32Date);

    /*	Write EEPROM 		*/
    Err_Code = DataMemoryWrite(MONITOR_TYPE,bufForStorage,sizeof(bufForStorage));
    APP_ERROR_CHECK(Err_Code);
}
/*******************************************************************************
* Function Name  : SetDeviceModeConfigInfo
* Description    : ????2a芍??㏒那?谷豕??那㊣??米?㏒o?y3㏒?㏒那?or2a那??㏒那?
* Input          : SysConfigCPUInfo_t *info
* Output         : Nont
* Return         : None
*******************************************************************************/
void SetDeviceModeConfigInfo(void)
{
    /* Cpu Config Info : 0:Month;
                         1:Day;
                         2:Hour
                         3:Min
                         4:Sec
                         5: Mode
    */

    uint8_t ConfigFile[10] = {0};   //max 10byte of Sys Config File

	date_str_typedef    date_s;               //RTC 豕??迆
	RTC_TimeTypeDef     rtc_time;             //RTC 那㊣??
    
    Calendar_Get(&date_s,&rtc_time);
    
    ConfigFile[0] = date_s.month;
	ConfigFile[1] = date_s.day;
	ConfigFile[2] = rtc_time.RTC_Hours;
	ConfigFile[3]= rtc_time.RTC_Minutes;
	ConfigFile[4]= rtc_time.RTC_Seconds;
    
    ConfigFile[5]= gFlagHR_SpO2Measure;

    /* Write Work Mode Config File From STM32 Internal EEPROM */
    DataEEPROM_MultiByteWrite(ConfigFile,EEPROM_CONFIG_DeviceMode,sizeof(ConfigFile));
}

#ifdef STORAGE_LOG

void SetStorageLog(StorageLog_t *pStorageLog)
{
    /* Write storage log variables to STM32 Internal EEPROM */
    DataEEPROM_MultiByteWrite((uint8_t *)pStorageLog, EEPROM_STORAGE_LOG, sizeof(StorageLog_t));
}

void GetStorageLog(StorageLog_t *pStorageLog)
{
    /* Get storage log variables from STM32 Internal EEPROM */
    DataEEPROM_MultiByteRead((uint8_t *)pStorageLog, EEPROM_STORAGE_LOG, sizeof(StorageLog_t));

    #ifdef FLASH_DEBUG
    printf("\r\nSector erase error Time: %d-%d,%d:%d,%d\r\n",pStorageLog->Month,pStorageLog->Day,
                                                      pStorageLog->Hour,pStorageLog->Min,pStorageLog->Sec);
    printf("storageExtManage.available_partition = %d\r\n",pStorageLog->part_num);
    printf("pre_read_info.pre_rd_len = %d\r\n",pStorageLog->pre_rd_len);
    printf("pre_write_info.pre_wr_len = %d\r\n",pStorageLog->pre_wr_len);
    printf("current_rd_partition.start_time = %d\r\n",pStorageLog->rd_part_start_time);
    printf("current_rd_partition.start_page_addr = %d\r\n",pStorageLog->rd_part_start_page_addr);
    printf("current_rd_partition.end_page_addr = %d\r\n",pStorageLog->rd_part_end_page_addr);
    printf("totel_len_InCurrReadPartition = %d\r\n",pStorageLog->totel_len_InCurrReadPart);
    printf("storageIntManage.u32RdPointer = %d\r\n",pStorageLog->u32RdPointer);
    printf("storageIntManage.u32RdNum_CurrPartition = %d\r\n",pStorageLog->u32RdNum_CurrPartition);

    printf("\r\ncurrent_partition.start_time = %d\r\n",pStorageLog->wr_part_start_time);
    printf("current_partition.start_page_addr = %d\r\n",pStorageLog->wr_part_start_page_addr);
    printf("current_partition.end_page_addr = %d\r\n",pStorageLog->wr_part_end_page_addr);
    printf("storageIntManage.u32WrPointer = %d\r\n",pStorageLog->u32WrPointer);
    printf("storageIntManage.u32WrNum_CurrPartition = %d\r\n",pStorageLog->u32WrNumCurrPart);

    printf("\r\nstorageExtManage.totel_availabe_length = %d\r\n",pStorageLog->totel_availabe_length);
    printf("LastSectorAddr = 0x%08x\r\n",pStorageLog->u32LastSectorStartAddr);
    printf("CurrentSectorAddr = 0x%08x\r\n",pStorageLog->u32CurrSectorStartAdrr);
    #endif
}

void SetStorageTimerErrorLog(StorageTimerErrorLog_t * pStorageTimerErrorLog)
{
    /* Write storage log variables to STM32 Internal EEPROM */
    DataEEPROM_MultiByteWrite((uint8_t *)pStorageTimerErrorLog, EEPROM_STORAGETIMER_ERROR_LOG, sizeof(StorageTimerErrorLog_t));
}

void GetStorageTimerErrorLog(StorageTimerErrorLog_t * pStorageTimerErrorLog)
{
    /* Get storage log variables from STM32 Internal EEPROM */
    DataEEPROM_MultiByteRead((uint8_t *)pStorageTimerErrorLog, EEPROM_STORAGETIMER_ERROR_LOG, sizeof(StorageTimerErrorLog_t));

    #ifdef FLASH_DEBUG
    printf("\r\nTimer error Time: %d-%d,%d:%d,%d\r\n",pStorageTimerErrorLog->Month,pStorageTimerErrorLog->Day,pStorageTimerErrorLog->Hour,pStorageTimerErrorLog->Min,pStorageTimerErrorLog->Sec);
    printf("ErrorCnt = %d\r\n",pStorageTimerErrorLog->cnt);
    #endif
}

void SetStorageTotelLengthErrorLog(StorageTotelLengthErrorLog_t * pStorageTotelLengthErrorLog)
{
    /* Write storage log variables to STM32 Internal EEPROM */
    DataEEPROM_MultiByteWrite((uint8_t *)pStorageTotelLengthErrorLog, EEPROM_STORAGETIMER_ERROR_LOG, sizeof(StorageTotelLengthErrorLog_t));
}

void GetStorageTotelLengthErrorLog(StorageTotelLengthErrorLog_t * pStorageTotelLengthErrorLog)
{
    /* Get storage log variables from STM32 Internal EEPROM */
    DataEEPROM_MultiByteRead((uint8_t *)pStorageTotelLengthErrorLog, EEPROM_STORAGETIMER_ERROR_LOG, sizeof(StorageTotelLengthErrorLog_t));

    #ifdef STORAGE_LOG
    printf("\r\nTimer error Time: %d-%d,%d:%d,%d\r\n",pStorageTotelLengthErrorLog->Month,
                                                      pStorageTotelLengthErrorLog->Day,
                                                      pStorageTotelLengthErrorLog->Hour,
                                                      pStorageTotelLengthErrorLog->Min,
                                                      pStorageTotelLengthErrorLog->Sec);
    printf("LastTotelLen = 0x%08x\r\n",pStorageTotelLengthErrorLog->u32LastTotelLength);
    printf("CurrTotelLen = 0x%08x\r\n",pStorageTotelLengthErrorLog->u32CurrTotelLength);
    printf("process = %d\r\n",pStorageTotelLengthErrorLog->process);

    printf("Start Time in read partition:0x%08x\r\n",pStorageTotelLengthErrorLog->start_time_rd_part);
    printf("Start Addr in read partition:0x%04x\r\n",pStorageTotelLengthErrorLog->start_page_addr_rd_part);
    printf("End Addr in read partition:0x%04x\r\n",pStorageTotelLengthErrorLog->end_page_addr_rd_part);
    printf("Totel Length in read partition:0x%08x\r\n",pStorageTotelLengthErrorLog->u32TotelLenInCurrRdPart);
    printf("Read pointer:0x%08x\r\n",pStorageTotelLengthErrorLog->u32RdPointer);
    printf("Read number in read partition: %d\r\n",pStorageTotelLengthErrorLog->u32RdNumInCurrPart);
    printf("Pre Read number in read partition: %d\r\n",pStorageTotelLengthErrorLog->u8PreRdLen);

    printf("Start Time in write partition: 0x%08x\r\n",pStorageTotelLengthErrorLog->start_time_wr_part);
    printf("Start Addr in write partition: 0x%04x\r\n",pStorageTotelLengthErrorLog->start_page_addr_wr_part);
    printf("End Addr in write partition: 0x%04x\r\n",pStorageTotelLengthErrorLog->end_page_addr_wr_part);
    printf("Write pointer: 0x%08x\r\n",pStorageTotelLengthErrorLog->u32WrPointer);
    printf("Write number in write partition: %d\r\n",pStorageTotelLengthErrorLog->u32WrNumInCurrPart);
    printf("Pre write number in write partition: %d\r\n",pStorageTotelLengthErrorLog->u8PreWrLen);
    #endif
}


#endif

/**
  * @}
  */

/**
  * @}
*/

/***************** (C) COPYRIGHT iCareTech Healthcare Co., Ltd.(END OF FILE)***********/


