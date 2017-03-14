#include "storeManage.h"
#include "MX25L1606E.h"
#include "common.h"
#include <stdio.h>
#include <string.h>

static storageExtManage_t storageExtManage;
static storageIntManage_t storageIntManage;
static flash_partition_t current_partition;
static flash_partition_t current_rd_partition;
static flash_partition_t last_partition;
static sector_erase_info_t sector_erase_info;
static pre_read_info_t pre_read_info;
static pre_write_info_t pre_write_info;
static pre_write_info_t pre_write_info_backup;
static uint32_t totel_len_InCurrReadPartition = 0;
static uint8_t last_read_len = 0;
static flash_access_state_e flash_access_state = FLASH_ACCESS_ENABLE;
static uint32_t last_partition_is_empty = 0;
static uint32_t rd_index_in_ram = 0;
static uint8_t sector_erase_cnt = 0;
static uint8_t sector_erase_timer_running = 0;
static uint8_t need_delete_partition = 0;
static uint8_t u8Flag_DelPartInCreatePart = 0;
static uint32_t StorageInfoArr[16] = {0};
static uint8_t index_storage = 0;
static uint8_t u8Last_write_send = 0;


#ifdef STORAGE_LOG
static uint32_t last_erase_sector_addr = (FLASH_HEADER_SECTOR_BASE_ADDR - SECTOR_SIZE);
static uint32_t storage_error_cnt = 0;

#endif

// timer
TIM_Cfg_Typedef         Tim_Cfg_EraseTimer_Index;
TIM_Basic_Cfg_Typedef 	Tim_Cfg_EraseTimer;
Timer_ID_Typedef        gEraseTimerID = TIMER_ERROR;

TIM_Cfg_Typedef         Tim_Cfg_FlashBusyCheckTimer_Index;
TIM_Basic_Cfg_Typedef 	Tim_Cfg_FlashBusyCheckTimer;
Timer_ID_Typedef        gFlashBusyCheckTimerID = TIMER_ERROR;


static bool ExtFLASH_IsPartitionValid(flash_partition_t *pflash_partition);
static void ExtFLASH_AddPartition(void);
void ExtFLASH_UpdateHeaderAreaInCreatePartition(void);
static void ExtFLASH_BackupOneSector(uint32_t flash_addr);
flashOperationMsg ExtFLASH_RestoreOnePartition(flash_partition_t * p_flash_partition, uint8_t * p_u8Flag);
void ExtFLASH_WriteHeaderAreaPageProgram(uint32_t flash_addr,uint8_t * pu8Src,uint8_t u8len);
void ExtFLASH_StorageInit(void);
void ExtFLASH_DeletePartition(uint8_t num);
ReturnMsg ExtFLASH_FlashPageWrite(void);
static void ExtFLASH_PreReadInit(void);
ReturnMsg ExtFLASH_PreRead(uint8_t * pu8Dst, uint16_t u8Length,uint32_t flash_rd_start_addr);
ReturnMsg ExtFLASH_PreReadInWrCurrPage(void);
static void ExtFLASH_BuildCurrReadPartition(void);
static bool ExtFLASH_IsPageStarttimeValid(uint32_t p_start_time);
static uint32_t ExtFLASH_ComputeCurPageStartAddr(uint32_t flash_addr);
ReturnMsg ExtFLASH_BufferPreRead( uint16_t u8Length);
uint32_t ExtFLASH_GetLengthInDeletePartition(void);
ReturnMsg ExtFLASH_RealFlashPageWrite(uint32_t flash_addr, uint32_t start_time, uint32_t totel_len,
                                      uint8_t *pu8Src, uint8_t len_tmp);
flashOperationMsg ExtFLASH_GetValidLengthInCurrPage(uint32_t flash_addr, uint32_t * pu32Len);
int8_t CheckTotelLenErr(uint32_t u32TmpTotelLen,uint8_t u8Process);
#ifdef STORAGE_LOG
    int8_t CheckEraseSectorAddr(uint32_t addr);
    int8_t SaveStorageTimerErrorLog(void);
    int8_t SaveStorageTotelLengthErrorLog(uint32_t last_len,uint32_t curr_len,uint8_t process,uint8_t state);
    void ExtFLASH_DispalyError(void);
#endif

/*********************************************************************************
  * Analyzes the data from the specified page
  * @param  pu8Src: the source data
  * @param  formate: the formate of the source data
  * @retval none
**********************************************************************************/
void AnalyzePageContext(uint8_t *pu8Src, uint8_t formate)
{
	uint16_t index;
    uint32_or_uint8_u * p_wr_length = (uint32_or_uint8_u *)(pu8Src + 4);

    if(0 == formate)
    {
        printf("[FLASH] AnalyzePageContext: Start Time: %02d-%02d-%02d,Datalength: %d\r\n",
            ((pu8Src[3] << 8) + pu8Src[2]),pu8Src[1],pu8Src[0],(p_wr_length->l));
        for(index = 0;index < (PAGE_SIZE - 8)/8;index ++)
        {
            printf("	[FLASH] Time: %02d:%02d:%02d,paraID: %02d,HR: %02d,Spo2: %02d,MoveLevel: %02d,Confi: %02d\r\n",
                *(pu8Src + 0 + (index * 8 + 8)),*(pu8Src + 1 + (index * 8 + 8)),
                *(pu8Src + 2 + (index * 8 + 8)),*(pu8Src + 3 + (index * 8 + 8)),
                *(pu8Src + 4 + (index * 8 + 8)),*(pu8Src + 5 + (index * 8 + 8)),
                *(pu8Src + 6 + (index * 8 + 8)),*(pu8Src + 7 + (index * 8 + 8)));
        }
    }
    else if(1 == formate)
    {
        printf("[FLASH] AnalyzePageContext: Start Time: %02d-%02d-%02d,Datalength: %d\r\n",
            ((pu8Src[3] << 8) + pu8Src[2]),pu8Src[1],pu8Src[0],(p_wr_length->l));
        for(index = 0;index < (PAGE_SIZE - 8)/5;index ++)
        {
            printf("	[FLASH] count: %04d, HR: %02d,SPO2: %02d,Move: %02d\r\n",
                (*(pu8Src + 0 + (index * 5 + 8)) << 8) + (*(pu8Src + 1 + (index * 5 + 8))),
                *(pu8Src + 2 + (index * 5 + 8)),*(pu8Src + 3 + (index * 5 + 8)),
                *(pu8Src + 4 + (index * 5 + 8)));
        }
    }
    else if(2 == formate)
    {
        printf("[FLASH] AnalyzePageContext: \r\n");
        for(index = 0;index < (PAGE_SIZE - 8)/8;index ++)
        {
            printf("%02d,Start Time: %02d-%02d-%02d,start_addr: 0x%04x,end_addr: 0x%04x\r\n",
                    index,
                    (*(pu8Src + (index * 8) + 3) << 8) + *(pu8Src + (index * 8) + 2),
                    *(pu8Src + (index * 8) + 1),
                    *(pu8Src + (index * 8) + 0),
                    ((*(pu8Src + (index * 8) + 5) << 8) + *(pu8Src + (index * 8) + 4)),
                    ((*(pu8Src + (index * 8) + 7) << 8) + *(pu8Src + (index * 8) + 6)));
        }
    }
    else if(3 == formate)
    {
        for(index = 0;index < (PAGE_SIZE - 8)/8;index ++)
        {
            printf("%02d:%02d:%02d,%02d,%02d,",
                *(pu8Src + 0 + (index * 8 + 8)),*(pu8Src + 1 + (index * 8 + 8)),
                *(pu8Src + 2 + (index * 8 + 8)),
                *(pu8Src + 4 + (index * 8 + 8)),*(pu8Src + 5 + (index * 8 + 8)));
        }
    }
}

/*********************************************************************************
  * Read and print the specification page data
  * @param  flash_start_addr: the start addr
  * @param  len: length
  * @retval none
**********************************************************************************/
void CheckWrite(uint32_t flash_start_addr, uint16_t len)
{
	uint8_t rd_temp[256] = {0};
	uint16_t index;

    /* Check parameters */
	usr_para_check(len <= PAGE_SIZE);

    #ifdef FLASH_DEBUG
	    printf("[FLASH] CheckWrite:\r\n");
    #endif
	CMD_READ(flash_start_addr, rd_temp, len);

	for(index = 0;index < len;index ++)
	{
		printf("0x%02x, ",*(rd_temp + index));
		if(!((index + 1) % 16))
		{
			printf("\r\n");
		}
	}

    if(flash_start_addr >= FLASH_HEADER_SECTOR_BASE_ADDR)
    {
        AnalyzePageContext(rd_temp, 2);
    }
    else
    {
        AnalyzePageContext(rd_temp, 0);
    }

    //AnalyzePageContext(rd_temp, 1);
    //AnalyzePageContext(rd_temp, 3);
}

/*********************************************************************************
  * Print the source data in the specification area
  * @param  p_buffer: the source data
  * @param  len: the length of data
  * @retval none
**********************************************************************************/
void CheckRead(uint8_t *p_buffer, uint16_t len)
{
	uint16_t index;

	usr_para_check(len <= 256);
    usr_para_check(p_buffer != NULL);

    #ifdef FLASH_DEBUG
	    printf("[FLASH] CheckRead:\r\n");
    #endif

	for(index = 0;index < len;index ++)
	{
		printf("0x%02x, ",*(p_buffer + index));
		if(!((index + 1) % 16))
		{
			printf("\r\n");
		}
	}
    printf("\r\n");
}

/*********************************************************************************
  * Read and print the partition information
  * @param  none
  * @retval none
**********************************************************************************/
void CheckPartition(void)
{
	uint16_t search_num = 0;
	uint16_t search_end = 0;
	uint8_t partition_tmp[8];
	uint32_t partition_base_addr = FLASH_HEADER_SECTOR_BASE_ADDR;
	flash_partition_t * p_flash_partition = (flash_partition_t *)partition_tmp;

	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] CheckPartition\r\n");
	#endif

	do{
		#ifdef FLASH_DEBUG
		printf("\r\nThe %dst partition is: \r\n",search_num);
		#endif

		CMD_READ(partition_base_addr + (search_num * sizeof(flash_partition_t)),
                partition_tmp,
                sizeof(flash_partition_t));
		#ifdef FLASH_DEBUG
			printf("	[FLASH] Start Time: %02d:%02d:%02d\r\n",((partition_tmp[3] << 8) + partition_tmp[2]),partition_tmp[1],partition_tmp[0]);
			printf("	[FLASH] start_page_addr	= 0x%04x\r\n",p_flash_partition->start_page_addr);
			printf("	[FLASH] end_page_addr	= 0x%04x\r\n",p_flash_partition->end_page_addr);
		#endif

		if(ExtFLASH_IsPartitionValid(p_flash_partition) == true)
		{
			search_num ++;
		}
		else
		{
			search_end = 1;
		}
	}while(!search_end);
}

/*********************************************************************************
  * Clear and reset the internal buffers for external users
  * @param  None
  * @retval None
**********************************************************************************/
void ExtFLASH_ClearRAM(void)
{
	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_ClearRAM\r\n");
	#endif

	memset(&current_rd_partition, 0, sizeof(flash_partition_t));
	memset(&storageIntManage, 0, sizeof(storageIntManage_t));
	memset(&storageExtManage, 0, sizeof(storageExtManage_t));
	memset(&current_partition, 0, sizeof(flash_partition_t));
	memset(&pre_write_info, 0, sizeof(pre_write_info_t));
	memset(&pre_read_info, 0, sizeof(pre_read_info_t));
	totel_len_InCurrReadPartition = 0;
	if(gEraseTimerID != TIMER_ERROR)
    {
        //Timer_Free(gEraseTimerID);
        //gEraseTimerID = TIMER_ERROR;
        Stop_Timer_Cnt(gEraseTimerID);
    }
}

/*********************************************************************************
  * Save current read pointer to const area
  * @param  void
  * @retval None
**********************************************************************************/
void ExtFLASH_SaveRdAddrToConst(void)
{
    StorageReadConstVar_t StorageReadConstVar;
    StorageReadConstVar_t StorageReadConstVar_tmp;
    #ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_SaveRdAddrToConst\r\n");
    #endif

    StorageReadConstVar.flash_read_partition.start_time = current_rd_partition.start_time;
    StorageReadConstVar.flash_read_partition.start_page_addr = current_rd_partition.start_page_addr;
    StorageReadConstVar.flash_read_partition.end_page_addr = current_rd_partition.end_page_addr;
    StorageReadConstVar.u32StorageReadAddr = storageIntManage.u32RdPointer;

    #ifdef FLASH_DEBUG
        printf("    [FLASH] Start time = 0x%08x\r\n",StorageReadConstVar.flash_read_partition.start_time);
        printf("    [FLASH] Start page addr = 0x%04x\r\n",StorageReadConstVar.flash_read_partition.start_page_addr);
        printf("    [FLASH] End page addr = 0x%04x\r\n",StorageReadConstVar.flash_read_partition.end_page_addr);
        printf("    [FLASH] u32StorageReadAddr = 0x%08x\r\n",StorageReadConstVar.u32StorageReadAddr);
    #endif

    GetStorageConstVarConfigInfo(&StorageReadConstVar_tmp);
    if((StorageReadConstVar.flash_read_partition.start_time != StorageReadConstVar_tmp.flash_read_partition.start_time) ||
      (StorageReadConstVar.flash_read_partition.start_page_addr != StorageReadConstVar_tmp.flash_read_partition.start_page_addr) ||
      (StorageReadConstVar.flash_read_partition.end_page_addr != StorageReadConstVar_tmp.flash_read_partition.end_page_addr) ||
      (StorageReadConstVar.u32StorageReadAddr != StorageReadConstVar_tmp.u32StorageReadAddr))
    {
        SetStorageConstVarConfigInfo(StorageReadConstVar);
    }
}

/*********************************************************************************
  * get current read pointer to const area
  * @param  void
  * @retval None
**********************************************************************************/
int8_t ExtFLASH_GetRdAddrFromConst(StorageReadConstVar_t *p_StorageReadConstVar)
{
    StorageReadConstVar_t StorageReadConstVar;
    int8_t ret = 0;

    #ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_GetRdAddrFromConst\r\n");
    #endif

    if(p_StorageReadConstVar != NULL)
    {
        memset(&StorageReadConstVar, 0, sizeof(StorageReadConstVar_t));
        GetStorageConstVarConfigInfo(&StorageReadConstVar);

        if((true == (ExtFLASH_IsPartitionValid(&StorageReadConstVar.flash_read_partition))) &&
          (StorageReadConstVar.u32StorageReadAddr < FLASH_HEADER_SECTOR_BASE_ADDR))
        {
            memcpy(p_StorageReadConstVar,&StorageReadConstVar,sizeof(StorageReadConstVar_t));
            ret = 0;
        }
        else
        {
            ret = -1;
        }
    }
    else
    {
        ret = -1;
    }

    #ifdef FLASH_DEBUG
		printf("    [FLASH] ExtFLASH_GetRdAddrFromConst,End\r\n");
        printf("    [FLASH] Start time = 0x%08x\r\n",p_StorageReadConstVar->flash_read_partition.start_time);
        printf("    [FLASH] Start page addr = 0x%04x\r\n",p_StorageReadConstVar->flash_read_partition.start_page_addr);
        printf("    [FLASH] End page addr = 0x%04x\r\n",p_StorageReadConstVar->flash_read_partition.end_page_addr);
        printf("    [FLASH] u32StorageReadAddr = 0x%08x\r\n",p_StorageReadConstVar->u32StorageReadAddr);
    #endif

    return (ret);
}


/*********************************************************************************
  * Print the pre-read area data
  * @param  len: the length to print
  * @retval None
**********************************************************************************/
void CheckPreReadArea(uint32_t len)
{
	uint16_t index;

	printf("\r\n[FLASH] CheckPreReadArea,len = %d,The pre-read data:\r\n",len);
	for(index = 0;index < len;index ++)
	{
		printf("0x%02x, ",*(pre_read_info.pre_rd_buffer + index));
		if(!((index + 1) % 16))
		{
			printf("\r\n");
		}
	}
    printf("\r\n");
}

/*********************************************************************************
  * Get the authority of storage stage
  * @param  None
  * @retval Flash access state
**********************************************************************************/
flash_access_state_e GetExtFlashAccessState(void)
{
	return (flash_access_state);
}

/*********************************************************************************
  * Disables the authority of accessing storage
  * @param  None
  * @retval None
**********************************************************************************/
static void DisableExtFlashAccess(void)
{
    #ifdef FLASH_DEBUG
		printf("	[FLASH] DisableExtFlashAccess\r\n");
    #endif
	flash_access_state = FLASH_ACCESS_DISABLE;
}

/*********************************************************************************
  * Enables the authority of accessing storage
  * @param  None
  * @retval None
**********************************************************************************/
static void EnableExtFlashAccess(void)
{
    #ifdef FLASH_DEBUG
		printf("	[FLASH] EnableExtFlashAccess\r\n");
    #endif
	flash_access_state = FLASH_ACCESS_ENABLE;
}

/*********************************************************************************
  * Handle the timeout of timer in erasing sector
  * @param  TIMID:
  * @retval None
**********************************************************************************/
static void SectorEraseTimeoutHandler(Timer_ID_Typedef TIMID)
{
    TIMID = TIMID;

	sector_erase_cnt ++;
	#ifdef FLASH_DEBUG
		USART_Configuration();
		printf("\r\n[FLASH] SectorEraseTimeoutHandler, %d.\r\n",sector_erase_cnt);
	#endif

	TS_SendEvent(gTsStorageTaskID_c,gStorageCheckIsBusy);
}

/*********************************************************************************
  * Init and start the timer for erasing sector
  * @param  None
  * @retval None
**********************************************************************************/
static void EraseSectorTimerInit(void)
{
	#ifdef FLASH_DEBUG
		printf("[FLASH] EraseSectorTimerInit\r\n");
	#endif

	if(gEraseTimerID != TIMER_ERROR)
    {
        #ifdef FLASH_DEBUG
			printf("\r\n***[FLASH] gEraseTimerID != TIMER_ERROR\r\n");
		#endif
        Timer_Free(gEraseTimerID);
        gEraseTimerID = TIMER_ERROR;
    }
	Tim_Cfg_EraseTimer.enuTimerType       	= TIM_TYPE_MS;
    Tim_Cfg_EraseTimer.u16TimePeriod      	= EraseSectorTimeOutTIM;
    Tim_Cfg_EraseTimer.NVIC_IRQChannelPreemptionPriority = OLEDShutDown_TIMPreemptionPriority;
    Tim_Cfg_EraseTimer.pIntCallBack         = SectorEraseTimeoutHandler;
    Tim_Cfg_EraseTimer_Index.TimerMode		= TIM_MODE_BASIC;
    Tim_Cfg_EraseTimer_Index.TimerBasicCfg 	= &Tim_Cfg_EraseTimer;
    Tim_Cfg_EraseTimer_Index.TimerPWMCfg 	= NULL;
    gEraseTimerID                         	= Timer_Allocate(&Tim_Cfg_EraseTimer_Index);
	if(gEraseTimerID == TIMER_ERROR)
	{
	   	#ifdef FLASH_DEBUG
			printf("[FLASH] After allocated, gEraseTimerID is error\r\n");
		#endif
	}
}

/*********************************************************************************
  * Check if the flash is include the partition
  * @param  flash_addr: the specification flash addr
  * @retval True or false
**********************************************************************************/
static bool ExtFlash_CheckNeedDeletePartition(uint32_t flash_addr)
{
	uint8_t partition_tmp[20];
	flash_partition_t * p_flash_partition = (flash_partition_t *)partition_tmp;
	uint32_t partition_base_addr = FLASH_HEADER_SECTOR_BASE_ADDR;
	uint32_t tmp_start_addr = 0;
    bool state = true;

	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFlash_CheckNeedDeletePartition\r\n");
	#endif

	/* Check if need to delete the partition? */
	CMD_READ(partition_base_addr, partition_tmp, sizeof(flash_partition_t));
	#ifdef FLASH_DEBUG
		printf("	[FLASH] p_flash_partition.start_page_addr	= 0x%04x\r\n",p_flash_partition->start_page_addr);
		printf("	[FLASH] p_flash_partition.end_page_addr 	= 0x%04x\r\n",p_flash_partition->end_page_addr);
		printf("	[FLASH] p_flash_partition.start_time		= 0x%08x\r\n",p_flash_partition->start_time);
		printf("	[FLASH] flash_addr		= 0x%08x\r\n",flash_addr);
	#endif

	tmp_start_addr = p_flash_partition->start_page_addr * PAGE_SIZE;
	#ifdef FLASH_DEBUG
		printf("	[FLASH] tmp_start_addr		= 0x%08x\r\n",tmp_start_addr);
	#endif
	if(((tmp_start_addr >= flash_addr) && (tmp_start_addr < (flash_addr + SECTOR_SIZE))) &&
        (p_flash_partition->start_time != current_partition.start_time))
	{
		#ifdef FLASH_DEBUG
			printf("	[FLASH] state = TRUE\r\n");
		#endif
        state = TRUE;
	}
	else
	{
		#ifdef FLASH_DEBUG
			printf("	[FLASH] state = FALSE\r\n");
		#endif
        state = FALSE;
	}

    return (state);
}

/*********************************************************************************
  * Handle the callback for erased the next area when write
  * @param  None
  * @retval None
**********************************************************************************/
static void EraseNextAreaInWrite_CallBackHandler(void)
{
    #ifdef FLASH_DEBUG
		printf("\r\nEraseNextAreaInWrite_CallBackHandler\r\n");
	#endif

    ExtFLASH_FlashPageWrite();

    if(need_delete_partition == 1)
    {
	    ExtFLASH_BuildCurrReadPartition();
        ExtFLASH_PreReadInit();
        need_delete_partition = 0;
    }

    EnableExtFlashAccess();
}

/*********************************************************************************
  * Handle the callback for erased the header area when write
  * @param  None
  * @retval None
**********************************************************************************/
static void EraseHeaderAreaInWrite_CallbackHandler(void)
{
	#ifdef FLASH_DEBUG
		printf("EraseHeaderAreaInWrite_CallbackHandler\r\n");
	#endif

    ExtFLASH_DeletePartition(1);
	if(storageExtManage.available_partition)
	{
		storageExtManage.available_partition--;
	}

    /* Erase the header Area */
	sector_erase_info.sector_erase_process = ERASE_NEXT_AREA;
	sector_erase_info.erase_page_addr = storageIntManage.u32WrPointer;
	sector_erase_info.pEraseCallBack = EraseNextAreaInWrite_CallBackHandler;
	TS_SendEvent(gTsStorageTaskID_c,gStorageEraseSector);
}

/*********************************************************************************
  * Handle the callback for erased the backup area when write
  * @param  None
  * @retval None
**********************************************************************************/
static void EraseBackupAreaInWrite_CallBackHandler(void)
{
	#ifdef FLASH_DEBUG
		printf("EraseBackupAreaInWrite_CallBackHandler\r\n");
	#endif

	/* Backup the header area */
	ExtFLASH_BackupOneSector(FLASH_HEADER_SECTOR_BASE_ADDR);

	/* Erase the header Area */
	sector_erase_info.sector_erase_process = ERASE_HEADER_AREA;
	sector_erase_info.erase_page_addr = 0;
	sector_erase_info.pEraseCallBack = EraseHeaderAreaInWrite_CallbackHandler;
	TS_SendEvent(gTsStorageTaskID_c,gStorageEraseSector);
}

/*********************************************************************************
  * Handle the callback for erased the next area when creating new partition
  * @param  None
  * @retval None
**********************************************************************************/
/*static void EraseNextAreaInCreatePartition_CallBackHandler(void)
{
	#ifdef FLASH_DEBUG
		printf("EraseNextAreaInCreatePartition_CallBackHandler\r\n");
	#endif

	ExtFLASH_UpdateHeaderAreaInCreatePartition();
}*/

/*********************************************************************************
  * Handle the callback for erased the header area when creating new partition
  * @param  None
  * @retval None
**********************************************************************************/
static void EraseHeaderAreaInCreatePartition_Handler(void)
{
	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] EraseHeaderAreaInCreatePartition_Handler\r\n");
	#endif

	/* Update the header area */
	ExtFLASH_AddPartition();
	EnableExtFlashAccess();

    #ifdef FLASH_DEBUG
		printf("[FLASH] After CreatePartition:\r\n");
        printf("    [FLASH] current_rd_partition.start_time: %04d:%02d:%02d\r\n",
                (uint16_t)(current_rd_partition.start_time >> 16),
                (uint8_t)(current_rd_partition.start_time >> 8),
                (uint8_t)(current_rd_partition.start_time));
		printf("	[FLASH] current_rd_partition.start_page_addr = 0x%04x\r\n",
                current_rd_partition.start_page_addr);
		printf("	[FLASH] current_rd_partition.end_page_addr   = 0x%04x\r\n",
                current_rd_partition.end_page_addr);

        printf("	[FLASH] current_partition.start_time: %04d:%02d:%02d\r\n",
                (uint16_t)(current_partition.start_time >> 16),
                (uint8_t)(current_partition.start_time >> 8),
                (uint8_t)(current_partition.start_time));
		printf("	[FLASH] current_partition.start_page_addr 	= 0x%04x\r\n",
                current_partition.start_page_addr);
		printf("	[FLASH] current_partition.end_page_addr 	= 0x%04x\r\n",
                current_partition.end_page_addr);
	#endif
}

/*********************************************************************************
  * Handle the callback for erased the backup area when creating new partition
  * @param  None
  * @retval None
**********************************************************************************/
static void EraseBackupAreaInCreatePartition_Handler(void)
{
	#ifdef FLASH_DEBUG
		printf("EraseBackupAreaInCreatePartition_Handler\r\n");
	#endif
	/* Backup the header area */
	ExtFLASH_BackupOneSector(FLASH_HEADER_SECTOR_BASE_ADDR);

	/* Erase the header Area */
	sector_erase_info.sector_erase_process = ERASE_HEADER_AREA;
	sector_erase_info.erase_page_addr = 0;
	sector_erase_info.pEraseCallBack = EraseHeaderAreaInCreatePartition_Handler;
	TS_SendEvent(gTsStorageTaskID_c,gStorageEraseSector);
}

/*********************************************************************************
  * Handle the callback after erased the head area when received read-OK ack
  * @param  None
  * @retval None
**********************************************************************************/
static void EraseHeaderAreaInReadACK_CallBackHandler(void)
{
	#ifdef FLASH_DEBUG
		printf("EraseHeaderAreaInReadACK_CallBackHandler\r\n");
	#endif

    #ifdef FLASH_DEBUG
        CheckWrite(FLASH_HEADER_SECTOR_BASE_ADDR, PAGE_SIZE);
        CheckWrite(FLASH_BACKUP_SECTOR_BASE_ADDR, PAGE_SIZE);
    #endif

	ExtFLASH_DeletePartition(1);
	if(storageExtManage.available_partition)
	{
		storageExtManage.available_partition--;
	}
	ExtFLASH_BuildCurrReadPartition();
    ExtFLASH_PreReadInit();
	EnableExtFlashAccess();
}

/*********************************************************************************
  * Handle the callback after erased the head area when received read-OK ack
  * @param  None
  * @retval None
**********************************************************************************/
static void EraseBackupAreaInReadACK_CallBackHandler(void)
{
	#ifdef FLASH_DEBUG
		printf("EraseBackupAreaInReadACK_CallBackHandler\r\n");
	#endif

    #ifdef FLASH_DEBUG
    CheckWrite(FLASH_HEADER_SECTOR_BASE_ADDR, PAGE_SIZE);
    #endif

	/* Backup the header area */
	ExtFLASH_BackupOneSector(FLASH_HEADER_SECTOR_BASE_ADDR);

	/* Erase the header Area */
	sector_erase_info.sector_erase_process = ERASE_HEADER_AREA;
	sector_erase_info.erase_page_addr = 0;
	sector_erase_info.pEraseCallBack = EraseHeaderAreaInReadACK_CallBackHandler;
	TS_SendEvent(gTsStorageTaskID_c,gStorageEraseSector);
}

/*********************************************************************************
  *
  * @param  None
  * @retval None
**********************************************************************************/
int8_t ExtFLASH_CheckInCurPart(flash_partition_t flash_part, uint32_t u32TmpAddr)
{
    int8_t ret = 0;
    uint32_t u32TmpStartAddr = 0;
    uint32_t u32TmpEndAddr = 0;

	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_CheckInCurPart,u32TmpAddr = 0x%08x\r\n",u32TmpAddr);
	#endif

    if(u32TmpAddr < FLASH_HEADER_SECTOR_BASE_ADDR)
    {
        u32TmpStartAddr = (flash_part.start_page_addr << 8);
        u32TmpEndAddr = (flash_part.end_page_addr << 8) + PAGE_SIZE;
        if(flash_part.start_page_addr <= flash_part.end_page_addr)
        {
            if((u32TmpStartAddr <= u32TmpAddr) && (u32TmpAddr < u32TmpEndAddr))
            {
                ret = 0;
            }
            else
            {
                ret = -1;
            }
        }
        else
        {
            if((u32TmpEndAddr <= u32TmpAddr) && (u32TmpAddr < u32TmpStartAddr))
            {
                ret = -1;
            }
            else
            {
                ret = 0;
            }
        }
    }
    else
    {
        ret = -1;
    }

    #ifdef FLASH_DEBUG
		printf("    [FLASH] ExtFLASH_CheckInCurPart,End,ret = %d\r\n",ret);
	#endif
    return (ret);
}

/*********************************************************************************
  * Check if the tmp_part is current write partition
  * @param  tmp_part
  * @param  tmp_part
  * @retval  0---same;
  *          -1---Different
**********************************************************************************/
int8_t ExtFLASH_CheckIsCurWrPart(flash_partition_t tmp_part, flash_partition_t cur_wr_part)
{
    int8_t ret = 0;
    #ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_CheckIsCurPart\r\n");
	#endif

    if((cur_wr_part.start_time == tmp_part.start_time)
    && (cur_wr_part.start_page_addr == tmp_part.start_page_addr))
    {
        ret = 0;
    }
    else
    {
        ret = -1;
    }

    #ifdef FLASH_DEBUG
		printf("    [FLASH] ExtFLASH_CheckIsCurPart,End,ret = %d\r\n",ret);
	#endif
    return (ret);
}

/*********************************************************************************
  * Check if the tmp_part is current write partition
  * @param  tmp_part
  * @param  tmp_part
  * @retval  0---same;
  *          -1---Different
**********************************************************************************/
int8_t ExtFLASH_CheckIsSamePart(flash_partition_t *p_tmp_part1, flash_partition_t *p_tmp_part2)
{
    int8_t ret = 0;
    #ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_CheckIsSamePart\r\n");
	#endif

    if((p_tmp_part1->start_time == p_tmp_part2->start_time)
    && (p_tmp_part1->start_page_addr == p_tmp_part2->start_page_addr))
    {
        ret = 0;
    }
    else
    {
        ret = -1;
    }

    #ifdef FLASH_DEBUG
		printf("    [FLASH] ExtFLASH_CheckIsSamePart,End,ret = %d\r\n",ret);
	#endif
    return (ret);
}

/*********************************************************************************
  * Updates the header area when creating partition
  * @param  None
  * @retval None
**********************************************************************************/
void ExtFLASH_UpdateHeaderAreaInCreatePartition(void)
{
	#ifdef FLASH_DEBUG
		printf("ExtFLASH_UpdateHeaderAreaInCreatePartition\r\n");
	#endif

	/* Fisrt,erase the backup area */
    /* Second,backup the header area */
    /* Third,add the new partition */
    /* Note: it is the first step this function */
	sector_erase_info.sector_erase_process = ERASE_BACKUP_AREA;
	sector_erase_info.pEraseCallBack = EraseBackupAreaInCreatePartition_Handler;
	TS_SendEvent(gTsStorageTaskID_c,gStorageEraseSector);
}

/*********************************************************************************
  * Write the header area
  * @param  None
  * @retval None
**********************************************************************************/
void ExtFLASH_WriteHeaderAreaPageProgram(uint32_t flash_addr,uint8_t * pu8Src,uint8_t u8len)
{
	uint32_t temp0;
	uint32_t temp1;

	/* Check paprameters */
	usr_para_check(pu8Src != NULL);
	usr_para_check((FLASH_HEADER_SECTOR_BASE_ADDR <= flash_addr)
                && (flash_addr <= FLASH_BACKUP_SECTOR_BASE_ADDR));

	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_WriteHeaderAreaPageProgram\r\n");
	#endif

    temp0 = ExtFLASH_ComputeCurPageStartAddr(flash_addr);
    temp1 = ExtFLASH_ComputeCurPageStartAddr(flash_addr + u8len);
	if(temp0 != temp1)
	{
		CMD_PageProgram(flash_addr, (uint8_t *)pu8Src, (PAGE_SIZE - (flash_addr % PAGE_SIZE)));
		CMD_PageProgram(temp1, ((uint8_t *)pu8Src + (PAGE_SIZE - (flash_addr % PAGE_SIZE))), ((flash_addr + u8len) - temp1));
	}
	else
	{
		CMD_PageProgram(flash_addr, pu8Src, u8len);
	}
}

/*********************************************************************************
  * Write a partition to header area
  * @param  partition_index: the index to write
  * @param  start_time: start time of the parttion
  * @param  start_addr_page: start page addr of the parttion
  * @param  end_addr_page: end page addr of the parttion
  * @retval None
**********************************************************************************/
void ExtFLASH_WritePartitionInfo(uint32_t partition_index, uint32_t start_time, uint16_t start_addr_page, uint16_t end_addr_page)
{
	uint8_t tmp[8] = {0};
	flash_partition_t * p_partition = (flash_partition_t *)tmp;
	uint32_t flash_addr = 0;

	/* Check paprameters */
    usr_para_check(partition_index > 0);

	#ifdef FLASH_DEBUG
		printf("ExtFLASH_WritePartitionInfo\r\n");
	#endif

	p_partition->start_time = start_time;
	p_partition->start_page_addr = start_addr_page;
	p_partition->end_page_addr = end_addr_page;

    flash_addr = FLASH_HEADER_SECTOR_BASE_ADDR + (partition_index - 1) * sizeof(flash_partition_t);
	CMD_PageProgram(flash_addr, tmp, sizeof(flash_partition_t));

    #ifdef FLASH_DEBUG
		printf("	[FLASH] ExtFLASH_WritePartitionInfo,End\r\n");
		CheckWrite(FLASH_HEADER_SECTOR_BASE_ADDR, PAGE_SIZE);
	#endif
}

/*********************************************************************************
  * Delete the specification partition
  * @param  num: the index to delete
  * @retval None
**********************************************************************************/
void ExtFLASH_DeletePartition(uint8_t num)
{
	uint8_t partition_tmp[8];
	bool search_is_completed = false;
	flash_partition_t * p_flash_partition = (flash_partition_t *)partition_tmp;
	uint32_t read_addr = 0;
	uint32_t write_addr = 0;

	#ifdef FLASH_DEBUG
        uint8_t i;
		printf("\r\n[FLASH] ExtFLASH_DeletePartition\r\n");
	#endif

    read_addr = FLASH_BACKUP_SECTOR_BASE_ADDR + (num * sizeof(flash_partition_t));
    write_addr = FLASH_HEADER_SECTOR_BASE_ADDR;
	do{
		CMD_READ(read_addr, partition_tmp, sizeof(flash_partition_t));
        #ifdef FLASH_DEBUG
		    for(i = 0;i < sizeof(flash_partition_t);i ++)
            {
                printf("0x%02x,",partition_tmp[i]);
            }
            printf("\r\n");
	    #endif
		if(ExtFLASH_IsPartitionValid(p_flash_partition) == true)
		{
			ExtFLASH_WriteHeaderAreaPageProgram(write_addr, partition_tmp, sizeof(flash_partition_t));
			write_addr += sizeof(flash_partition_t);
            read_addr += sizeof(flash_partition_t);
		}
		else
		{
			search_is_completed = true;
		}
	}while(search_is_completed == false);

	#ifdef FLASH_DEBUG
		printf("	[FLASH] ExtFLASH_DeletePartition End,check the result: \r\n");
		CheckWrite(FLASH_HEADER_SECTOR_BASE_ADDR, PAGE_SIZE);
	#endif
}

/*********************************************************************************
  * Delete the specification partition when init
  * @param  num: the index to delete
  * @retval None
**********************************************************************************/
void ExtFLASH_DeleteRealPartitionInit(uint8_t num)
{
	uint8_t partition_tmp[8];
	bool search_is_completed = FALSE;
	flash_partition_t * p_flash_partition = (flash_partition_t *)partition_tmp;
	uint32_t read_start_addr = FLASH_BACKUP_SECTOR_BASE_ADDR;
	uint32_t write_addr = FLASH_HEADER_SECTOR_BASE_ADDR;
    uint8_t index = 0;

	#ifdef FLASH_DEBUG
		printf("\r\nExtFLASH_DeletePartitionInit\r\n");
	#endif

	for(index = 0;index < num;index ++)
	{
		CMD_READ(read_start_addr,partition_tmp,sizeof(flash_partition_t));
		ExtFLASH_WriteHeaderAreaPageProgram(write_addr, partition_tmp, sizeof(flash_partition_t));
        write_addr += sizeof(flash_partition_t);
        read_start_addr += sizeof(flash_partition_t);
	}

    /* Just update read addr */
    read_start_addr += sizeof(flash_partition_t);

    do{
		CMD_READ(read_start_addr,partition_tmp,sizeof(flash_partition_t));
		if(ExtFLASH_IsPartitionValid(p_flash_partition) == TRUE)
		{
			ExtFLASH_WriteHeaderAreaPageProgram(write_addr, partition_tmp, sizeof(flash_partition_t));
			write_addr += sizeof(flash_partition_t);
            read_start_addr += sizeof(flash_partition_t);
		}
		else
		{
			search_is_completed = TRUE;
		}
	}while(search_is_completed == FALSE);

	#ifdef FLASH_DEBUG
		printf("	[FLASH] nExtFLASH_DeletePartitionInit,End,check the result: \r\n");
		CheckWrite(FLASH_HEADER_SECTOR_BASE_ADDR + PAGE_SIZE, 256);
		CheckWrite(FLASH_HEADER_SECTOR_BASE_ADDR, 256);
	#endif
}

/*********************************************************************************
  * Add partition
  * @param  None
  * @retval None
**********************************************************************************/
static void ExtFLASH_AddPartition(void)
{
	uint8_t partition_tmp[8];
	uint16_t search_num = 0;
	flash_partition_t * p_flash_partition = (flash_partition_t *)partition_tmp;
	uint32_t read_start_addr = FLASH_BACKUP_SECTOR_BASE_ADDR;
	uint32_t write_addr = FLASH_HEADER_SECTOR_BASE_ADDR;
    uint8_t u8FlagDelPart = 0;

	#ifdef FLASH_DEBUG
		printf("[FLASH] ExtFLASH_AddPartition, last_partition_is_empty= %d,u8Flag_DelPartInCreatePart = %d\r\n",
		                                                last_partition_is_empty,u8Flag_DelPartInCreatePart);
	#endif

	if(storageExtManage.available_partition <= 1)
	{
		/* Add the current partition information to the header area */
		ExtFLASH_WriteHeaderAreaPageProgram(write_addr,
                                            (uint8_t *)(&current_partition),
                                            sizeof(flash_partition_t));
		#ifdef FLASH_DEBUG
		printf("[FLASH] write_addr = 0x%08x \r\n",write_addr);
		#endif
	}
	else
	{
        /* Direct copy the partition which is no change */
        for(search_num = 0;search_num < (storageExtManage.available_partition - 2);search_num ++)
        {
            #ifdef FLASH_DEBUG
                printf("[FLASH] search_num = %d\r\n",search_num);
            #endif
            CMD_READ(read_start_addr, partition_tmp, sizeof(flash_partition_t));
            if(ExtFLASH_IsPartitionValid(p_flash_partition) == true)
            {
                if(1 == u8Flag_DelPartInCreatePart)
                {
                    u8Flag_DelPartInCreatePart = 0;
                    u8FlagDelPart = 1;
                }
                else
                {
                    ExtFLASH_WriteHeaderAreaPageProgram(write_addr,
                                                        partition_tmp,
                                                        sizeof(flash_partition_t));
                    write_addr += sizeof(flash_partition_t);
                }

                read_start_addr += sizeof(flash_partition_t);
            }
            else
            {
                break;
            }
        }

        if(1 == u8FlagDelPart)
        {
            storageExtManage.available_partition--;
            u8FlagDelPart = 0;
        }

		if(last_partition_is_empty)
		{
			last_partition_is_empty = 0;

			/* Update the end_page_addr in pre-partition */
			CMD_READ(read_start_addr,partition_tmp, sizeof(flash_partition_t));
			if(ExtFLASH_IsPartitionValid(p_flash_partition) == true)
			{
				p_flash_partition->start_page_addr = current_partition.start_page_addr;
				p_flash_partition->end_page_addr = current_partition.end_page_addr;
				p_flash_partition->start_time = current_partition.start_time;
				ExtFLASH_WriteHeaderAreaPageProgram(write_addr,
                                                    partition_tmp,
                                                    sizeof(flash_partition_t));
				write_addr += sizeof(flash_partition_t);
			}

			storageExtManage.available_partition--;

            /* Builds current read partition */
            ExtFLASH_BuildCurrReadPartition();
            ExtFLASH_PreReadInit();
		}
		else
		{
			#ifdef FLASH_DEBUG
                printf("[FLASH] last_partition.start_time = 0x%08x\r\n",last_partition.start_time);
                printf("[FLASH] last_partition.start_page_addr = 0x%04x\r\n",last_partition.start_page_addr);
				printf("[FLASH] last_partition.end_page_addr = 0x%04x\r\n",last_partition.end_page_addr);
                printf("[FLASH] storageExtManage.available_partition = %d\r\n",storageExtManage.available_partition);
			#endif

            CMD_READ(read_start_addr, partition_tmp, sizeof(flash_partition_t));
			if(ExtFLASH_IsPartitionValid(p_flash_partition) == true)
			{
                p_flash_partition->end_page_addr = last_partition.end_page_addr;
				ExtFLASH_WriteHeaderAreaPageProgram(write_addr,
                                                    partition_tmp,
                                                    sizeof(flash_partition_t));
				write_addr += sizeof(flash_partition_t);
                read_start_addr += sizeof(flash_partition_t);
			}

			/* Add the current partition information to the header area */
			ExtFLASH_WriteHeaderAreaPageProgram(write_addr,
                                                (uint8_t *)(&current_partition),
                                                (sizeof(flash_partition_t)));
		}
	}

	#ifdef FLASH_DEBUG
		printf("[FLASH] ExtFLASH_AddPartition End\r\n");
		CheckWrite(FLASH_HEADER_SECTOR_BASE_ADDR,PAGE_SIZE);
	#endif
}

static void ExtFLASH_AddEachPartition(uint32_t index, flash_partition_t * p_flash_partition)
{
	uint32_t write_addr_base = FLASH_HEADER_SECTOR_BASE_ADDR;
	uint32_t write_addr = 0;

	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_AddEachPartition,index = 0x%08x\r\n",index);
	#endif

	write_addr = write_addr_base + (index * sizeof(flash_partition_t));
	CMD_PageProgram(write_addr, (uint8_t *)p_flash_partition, sizeof(flash_partition_t));

	#ifdef FLASH_DEBUG
		delay1ms(10);
		CheckWrite(FLASH_HEADER_SECTOR_BASE_ADDR, PAGE_SIZE);
	#endif
}

/*********************************************************************************
  * Add partition
  * @param  None
  * @retval None
**********************************************************************************/
flashOperationMsg ExtFLASH_ExtSearchPartition(flash_partition_t * p_flash_partition)
{
	flashOperationMsg msg = FLASH_OPERATION_SUCCESS;
	uint32_t search_addr = 0;
	uint8_t search_end = 0;
    uint16_t search_num = 0;
	uint8_t temp[4] = {0};
	uint32_or_uint8_u * p_start_time = (uint32_or_uint8_u *)temp;

	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_ExtSearchPartition\r\n");
	#endif

	search_addr = p_flash_partition->start_page_addr << 8;
	CMD_READ(search_addr, temp, sizeof(temp));
	if(ExtFLASH_IsPageStarttimeValid(p_start_time->l) == true)
	{
		p_flash_partition->start_time = p_start_time->l;
	}
    else
    {
        msg = FLASH_OPERATION_FAIL;
        return (msg);
    }
	#ifdef FLASH_DEBUG
		printf("	[FLASH] p_start_time->l = 0x%08x\r\n",p_start_time->l);
	#endif

	search_addr += PAGE_SIZE;
	do{
		#ifdef FLASH_DEBUG
			printf("	[FLASH] search_num = %d\r\n",search_num);
		#endif

		CMD_READ(search_addr, temp, sizeof(temp));
		#ifdef FLASH_DEBUG
			printf("	[FLASH] p_start_time->l = 0x%08x\r\n",p_start_time->l);
		#endif

		if((ExtFLASH_IsPageStarttimeValid(p_start_time->l) == true)
        && (p_start_time->l == p_flash_partition->start_time))
		{
			#ifdef FLASH_DEBUG
				printf("	[FLASH] ExtFLASH_IsPageStarttimeValid,True\r\n");
				printf("	[FLASH] p_start_time->l = 0x%08x\r\n",p_start_time->l);
				printf("	[FLASH] p_flash_partition->start_time = 0x%08x\r\n",
                        p_flash_partition->start_time);
			#endif

			search_num ++;
			p_flash_partition->start_time = p_start_time->l;
			search_addr += PAGE_SIZE;
			if(search_addr >= FLASH_HEADER_SECTOR_BASE_ADDR)
			{
                msg = FLASH_OPERATION_SUCCESS;
				search_end = 1;
			}
		}
		else
		{
            #ifdef FLASH_DEBUG
				printf("	[FLASH] ExtFLASH_IsPageStarttimeValid,False\r\n");
			#endif
			msg = FLASH_OPERATION_SUCCESS;
			search_end = 1;
		}
	}while(search_end == 0);

	p_flash_partition->end_page_addr = (search_addr >> 8) - 1;

	#ifdef FLASH_DEBUG
		printf("	[FLASH] p_flash_partition->start_time	= 0x%08x\r\n",
                    p_flash_partition->start_time);
        printf("	[FLASH] p_flash_partition->start_page_addr = 0x%04x\r\n",
                    p_flash_partition->start_page_addr);
		printf("	[FLASH] p_flash_partition->end_page_addr   = 0x%04x\r\n",
                    p_flash_partition->end_page_addr);
		printf("	[FLASH] ExtFLASH_ExtSearchPartition,end\r\n");
	#endif

	return (msg);
}

/*********************************************************************************
  * Backup the specification sector
  * @param  flash_addr: the start addr of the sector to backup
  * @retval None
**********************************************************************************/
static void ExtFLASH_BackupOneSector(uint32_t flash_addr)
{
	uint8_t temp[128] = {0},i;
    uint32_t write_addr = 0;
    uint32_t read_addr = 0;

	/* Check parameters */
	usr_para_check(!(flash_addr % SECTOR_SIZE));

	#ifdef FLASH_DEBUG
		printf("[FLASH] Now,backup the sector @ 0x%08x\r\n",flash_addr);
	#endif

    write_addr = FLASH_BACKUP_SECTOR_BASE_ADDR;
    read_addr = flash_addr;

	for(i = 0;i < 32;i ++)
	{
		CMD_READ(read_addr, temp, sizeof(temp));
		CMD_PageProgram(write_addr, temp, sizeof(temp));
        write_addr += 128;
        read_addr += 128;
	}

    #ifdef FLASH_DEBUG
        CheckWrite(FLASH_BACKUP_SECTOR_BASE_ADDR, PAGE_SIZE);
    #endif
}

void ExtFLASH_CopySector(uint32_t flash_addr_src,uint32_t flash_addr_dst)
{
	uint8_t temp[128] = {0},i;
    uint32_t write_addr = 0;
    uint32_t read_addr = 0;

	/* Check parameters */
	usr_para_check(!(flash_addr_src % SECTOR_SIZE));
    usr_para_check(!(flash_addr_dst % SECTOR_SIZE));

	#ifdef FLASH_DEBUG
		printf("[FLASH] Now,Copy the 0x%08x sector to the 0x%08x sector\r\n",flash_addr_src,flash_addr_dst);
	#endif

    CMD_SE(flash_addr_dst);
    delay1ms(200);

    write_addr = flash_addr_dst;
    read_addr = flash_addr_src;

	for(i = 0;i < 32;i ++)
	{
		CMD_READ(read_addr, temp, sizeof(temp));
		CMD_PageProgram(write_addr, temp, sizeof(temp));
        write_addr += 128;
        read_addr += 128;
	}
}
/*********************************************************************************
  * Check if the partition is valid
  * @param  pflash_partition: the addr of flash partition
  * @retval Valid or invalid
**********************************************************************************/
static bool ExtFLASH_IsPartitionValid(flash_partition_t *pflash_partition)
{
	/* Check parameters */
	usr_para_check(pflash_partition != NULL);

    #ifdef FLASH_DEBUG
        printf("\r\n[FLASH] ExtFLASH_IsPartitionValid,");
	#endif

	if((pflash_partition->start_page_addr <= PAGE_NUM)
   && (pflash_partition->end_page_addr <= PAGE_NUM)
   && (pflash_partition->start_time != 0xFFFFFFFF))
	{
		#ifdef FLASH_DEBUG
			printf("Valid\r\n");
		#endif
		return true;
	}
	else
	{
		#ifdef FLASH_DEBUG
			printf("Unvalid\r\n");
		#endif
		return false;
	}
}

/*********************************************************************************
  * Check if the start time is valid
  * @param  p_start_time: start time
  * @retval Valid or invalid
**********************************************************************************/
static bool ExtFLASH_IsPageStarttimeValid(uint32_t p_start_time)
{
	/* Check parameters */
	if(p_start_time == 0xFFFFFFFF)
	{
		return false;
	}
	else
	{
		return true;
	}
}

/*********************************************************************************
  * Check if flash addr is section boundary
  * @param  flash_addr: flash addr to check
  * @retval Valid or invalid
**********************************************************************************/
static bool ExtFLASH_IsSectionBoundary(uint32_t flash_addr)
{
	/* Check parameters */
	usr_para_check(flash_addr <= FlashSize);

	return ((!(flash_addr % SECTOR_SIZE)) ? true : false);
}

/*********************************************************************************
  * Check if flash addr is page boundary
  * @param  flash_addr: flash addr to check
  * @retval Valid or invalid
**********************************************************************************/
static bool ExtFLASH_IsPageBoundary(uint32_t flash_addr)
{
	/* Check parameters */
	usr_para_check(flash_addr <= FlashSize);

	return ((!(flash_addr % PAGE_SIZE)) ? true : false);
}

/*********************************************************************************
  * Compute page start addr for the specification flash addr
  * @param  flash_addr: flash addr
  * @retval Valid or invalid
**********************************************************************************/
static uint32_t ExtFLASH_ComputeCurPageStartAddr(uint32_t flash_addr)
{
	/* Check parameters */
	usr_para_check(flash_addr <= FlashSize);

	return ((flash_addr / PAGE_SIZE) * PAGE_SIZE);
}

/*********************************************************************************
  * Compute page start addr for the specification flash addr
  * @param  flash_addr: flash addr
  * @retval Valid or invalid
**********************************************************************************/
static uint32_t ExtFLASH_ComputeLastPageStartAddr(uint32_t flash_addr)
{
    uint32_t u32TmpAddr = 0;
    uint32_t u32TmpCurPageAddr = 0;

	/* Check parameters */
	usr_para_check(flash_addr <= FlashSize);

    u32TmpCurPageAddr = ExtFLASH_ComputeCurPageStartAddr(flash_addr);

    u32TmpAddr = ((u32TmpCurPageAddr >= PAGE_SIZE) ? (u32TmpCurPageAddr - PAGE_SIZE)
                : ((u32TmpCurPageAddr + FLASH_HEADER_SECTOR_BASE_ADDR - PAGE_SIZE)));

	return (u32TmpAddr);
}

/*********************************************************************************
  * Compute next page start addr for the specification flash addr
  * @param  flash_addr: flash addr
  * @retval Valid or invalid
**********************************************************************************/
static uint32_t ExtFLASH_ComputeNextPageStartAddr(uint32_t flash_addr)
{
    uint32_t flash_addr_temp = 0;

	/* Check parameters */
	usr_para_check(flash_addr <= FlashSize);

    flash_addr_temp = ExtFLASH_ComputeCurPageStartAddr(flash_addr);
    flash_addr_temp += PAGE_SIZE;
    flash_addr_temp %= FLASH_HEADER_SECTOR_BASE_ADDR;

	return (flash_addr_temp);
}

/*********************************************************************************
  * Compute the start addr of current sector for the specification flash addr
  * @param  flash_addr: flash addr
  * @retval Valid or invalid
**********************************************************************************/
uint32_t ExtFLASH_ComputeCurrSectorStartAddr(uint32_t flash_addr)
{
	/* Check parameters */
	usr_para_check(flash_addr <= FlashSize);

	return ((flash_addr / SECTOR_SIZE) * SECTOR_SIZE);
}

/*********************************************************************************
  * Compute the start addr of next sector for the specification flash addr
  * @param  flash_addr: flash addr
  * @retval Valid or invalid
**********************************************************************************/
static uint32_t ExtFLASH_ComputeNextSectorStartAddr(uint32_t flash_addr)
{
    uint32_t flash_addr_temp = 0;

	/* Check parameters */
	usr_para_check(flash_addr <= FlashSize);

    flash_addr_temp = ExtFLASH_ComputeCurrSectorStartAddr(flash_addr);
    flash_addr_temp += SECTOR_SIZE;
    flash_addr_temp %= FLASH_HEADER_SECTOR_BASE_ADDR;

	return (flash_addr_temp);
}

/*********************************************************************************
  * Restore the write information of partition
  * @param  p_flash_partition:
  * @retval None
**********************************************************************************/
static void ExtFLASH_RestoreWrInfo(flash_partition_t * p_flash_partition)
{
	uint8_t temp[8] = {0};
	uint32_or_uint8_u * p_start_time = NULL;
	uint32_or_uint8_u * p_wr_current = NULL;
	uint8_t search_is_end = 0;
	uint32_t search_start_addr = 0;
	uint32_t local_current_len_inCurrentPartition = 0;

	/* Check parameters */
	usr_para_check(p_flash_partition != NULL);

	#ifdef FLASH_DEBUG
		printf("[FLASH] ExtFLASH_RestoreInfo,searching the current partition area:\r\n");
	#endif

	p_start_time = (uint32_or_uint8_u *)temp;
	p_wr_current = (uint32_or_uint8_u *)(temp + TOTEL_LENGTH_AREA_OFFSET);
	search_start_addr = p_flash_partition->start_page_addr * PAGE_SIZE;
	search_start_addr %= FLASH_HEADER_SECTOR_BASE_ADDR;

	do{
		CMD_READ(search_start_addr, temp, sizeof(temp));
		if(p_start_time->l == p_flash_partition->start_time)
		{
			local_current_len_inCurrentPartition = p_wr_current->l;
            search_start_addr += PAGE_SIZE;
            search_start_addr %= FLASH_HEADER_SECTOR_BASE_ADDR;
		}
		else
		{
			search_is_end = 1;
		}
	}while(search_is_end == 0);

    p_flash_partition->end_page_addr
        = ExtFLASH_ComputeCurPageStartAddr(search_start_addr) / PAGE_SIZE;
	storageIntManage.u32WrPointer = search_start_addr;
	storageIntManage.u32WrNum_CurrPartition = local_current_len_inCurrentPartition;

	#ifdef FLASH_DEBUG
		printf("[FLASH] Search complete\r\n");
	#endif
}

/*********************************************************************************
  * Compute totel length of current partition
  * @param  p_flash_partition: the start addr of the partition
  * @retval The totel length of the partition
**********************************************************************************/
static uint32_t ExtFLASH_ComputeTotelLengthInCurrPartition(flash_partition_t * p_flash_partition)
{
	uint8_t temp[PAGE_HEADER_SIZE] = {0};
	uint32_or_uint8_u * p_start_time = (uint32_or_uint8_u *)(temp + START_TIME_AREA_OFFSET);
	uint32_or_uint8_u * p_wr_length = (uint32_or_uint8_u *)(temp + TOTEL_LENGTH_AREA_OFFSET);
	uint8_t search_is_end = 0;
	uint32_t len = 0,len_old = 0;
    uint32_t tmp_read_addr = 0;

	/* Check parameters */
	usr_para_check(p_flash_partition != NULL);

	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_ComputeTotelLengthInCurrPartition\r\n");
	#endif

	#ifdef FLASH_DEBUG
        printf("[FLASH] p_flash_partition.start_time: %04d:%02d:%02d\r\n",
                    (uint16_t)(p_flash_partition->start_time >> 16),
                    (uint8_t)(p_flash_partition->start_time >> 8),
                    (uint8_t)(p_flash_partition->start_time));
		printf("[FLASH] p_flash_partition.start_page_addr = 0x%04x\r\n",p_flash_partition->start_page_addr);
		printf("[FLASH] p_flash_partition.end_page_addr = 0x%04x\r\n",p_flash_partition->end_page_addr);
	#endif

	if (p_flash_partition->end_page_addr != p_flash_partition->start_page_addr)
	{
		tmp_read_addr = (p_flash_partition->end_page_addr << 8);
		tmp_read_addr %= FLASH_HEADER_SECTOR_BASE_ADDR;

		CMD_READ(tmp_read_addr, temp, PAGE_HEADER_SIZE);
        if((p_start_time->l == p_flash_partition->start_time) &&
            (p_wr_length->l <= FLASH_HEADER_SECTOR_BASE_ADDR))
        {
            len = p_wr_length->l;
        }
        else
        {
            len = 0;
        }
	}
	else
	{
        tmp_read_addr = (p_flash_partition->start_page_addr << 8);
        tmp_read_addr %= FLASH_HEADER_SECTOR_BASE_ADDR;

		do{
			CMD_READ(tmp_read_addr, temp, PAGE_HEADER_SIZE);
			if(p_start_time->l == p_flash_partition->start_time)
			{
                tmp_read_addr += PAGE_SIZE;
                tmp_read_addr %= FLASH_HEADER_SECTOR_BASE_ADDR;

				len = p_wr_length->l;
				if(len <= len_old)
				{
					search_is_end = 1;
					#ifdef FLASH_DEBUG
						printf("Search end\r\n");
					#endif
				}
				len_old = len;
			}
			else
			{
				search_is_end = 1;
				#ifdef FLASH_DEBUG
					printf("Search end\r\n");
				#endif
			}
		}while(search_is_end == 0);
	}

    if(0 == ExtFLASH_CheckIsCurWrPart(*p_flash_partition, current_partition))
	{
		len += pre_write_info.pre_wr_len;
	}

	#ifdef FLASH_DEBUG
		printf("[FLASH] ExtFLASH_ComputeTotelLengthInCurrPartition,End,len = %d\r\n",len);
	#endif
	return (len);
}

static void ExtFLASH_PreReadInit(void)
{
	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_PreReadInit\r\n");
	#endif

	if(totel_len_InCurrReadPartition >= PAGE_SIZE)
	{
        memset(pre_read_info.pre_rd_buffer, 0, PAGE_SIZE);
		ExtFLASH_PreRead(pre_read_info.pre_rd_buffer, PAGE_SIZE, storageIntManage.u32RdPointer);
		pre_read_info.pre_rd_len = PAGE_SIZE;
	}
    else if(totel_len_InCurrReadPartition >= (PAGE_SIZE - PAGE_HEADER_SIZE))
    {
        memset(pre_read_info.pre_rd_buffer, 0, PAGE_SIZE);
		ExtFLASH_PreRead(pre_read_info.pre_rd_buffer, totel_len_InCurrReadPartition,
                        storageIntManage.u32RdPointer);
		pre_read_info.pre_rd_len = totel_len_InCurrReadPartition;
    }
	else
	{
        memset(pre_read_info.pre_rd_buffer, 0, PAGE_SIZE);
        memcpy(pre_read_info.pre_rd_buffer, pre_write_info.pre_wr_buffer, totel_len_InCurrReadPartition);
		pre_read_info.pre_rd_len = totel_len_InCurrReadPartition;
	}

    #ifdef FLASH_DEBUG
		printf("[FLASH] ExtFLASH_PreReadInit,End\r\n");
        CheckPreReadArea(PAGE_SIZE);
	#endif
}

/*********************************************************************************
  * Build the partition information for read
  * @param  None
  * @retval None
**********************************************************************************/
static void ExtFLASH_BuildCurrReadPartition(void)
{
	uint8_t partition_tmp[20] = {0};
	flash_partition_t * p_flash_partition = (flash_partition_t *)partition_tmp;
	uint32_t partition_base_addr = FLASH_HEADER_SECTOR_BASE_ADDR;
	uint32_t totel_len_at_partition = 0;
    uint8_t u8State = 0;
    uint8_t index = 0;

	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_BuildCurrReadPartition\r\n");
	#endif

	if(storageExtManage.available_partition)
	{
        do{
        	CMD_READ(partition_base_addr, partition_tmp, sizeof(flash_partition_t));
        	#ifdef FLASH_DEBUG
                printf("[FLASH] p_flash_partition.start_time: %04d:%02d:%02d\r\n",
                        (uint16_t)(p_flash_partition->start_time >> 16),
                        (uint8_t)(p_flash_partition->start_time >> 8),
                        (uint8_t)(p_flash_partition->start_time));
                printf("[FLASH] p_flash_partition.start_page_addr = 0x%04x\r\n",
                        p_flash_partition->start_page_addr);
        		printf("[FLASH] p_flash_partition.end_page_addr = 0x%04x\r\n",
                        p_flash_partition->end_page_addr);
        	#endif
        	if(ExtFLASH_IsPartitionValid(p_flash_partition) == true)
        	{
                if(0 != ExtFLASH_CheckIsSamePart(p_flash_partition,&current_partition))
                {
                    current_rd_partition.start_time = p_flash_partition->start_time;
                    current_rd_partition.start_page_addr = p_flash_partition->start_page_addr;
                    current_rd_partition.end_page_addr = p_flash_partition->end_page_addr;
                    totel_len_at_partition
                        = ExtFLASH_ComputeTotelLengthInCurrPartition(p_flash_partition);
                    storageIntManage.u32RdPointer
                        = current_rd_partition.start_page_addr * PAGE_SIZE + PAGE_HEADER_OFFSET;
                    totel_len_InCurrReadPartition           = totel_len_at_partition;
                    //storageIntManage.u32RdNum_CurrPartition = 0;
                }
                else
                {
                    current_rd_partition.start_time         = current_partition.start_time;
                    current_rd_partition.start_page_addr    = current_partition.start_page_addr;
                    current_rd_partition.end_page_addr      = current_partition.end_page_addr;
                    totel_len_at_partition
                        = ExtFLASH_ComputeTotelLengthInCurrPartition(p_flash_partition);
                    storageIntManage.u32RdPointer
                        = current_rd_partition.start_page_addr * PAGE_SIZE + PAGE_HEADER_OFFSET;
                    totel_len_InCurrReadPartition           = totel_len_at_partition;
                }

                if(storageExtManage.available_partition >= 2)
                {
                    if(0 == ExtFLASH_ComputeTotelLengthInCurrPartition(p_flash_partition))
                    {
                        #ifdef FLASH_DEBUG
                    		printf("    [FLASH] Current Partition is no data ...\r\n");
                    	#endif

                        CMD_SE(FLASH_BACKUP_SECTOR_BASE_ADDR);
                        index = 0;
                        while((IsFlashBusy() == TRUE) && (++ index < 30))
                        {
                            delay1ms(10);
                        }

                        ExtFLASH_BackupOneSector(FLASH_HEADER_SECTOR_BASE_ADDR);

                        CMD_SE(FLASH_HEADER_SECTOR_BASE_ADDR);
                        index = 0;
                        while((IsFlashBusy() == TRUE) && (++ index < 30))
                        {
                            delay1ms(10);
                        }

                        ExtFLASH_DeletePartition(1);
                        u8State = 0;
                    }
                    else
                    {
                        u8State = 1;
                    }
                }
                else
                {
                    u8State = 1;
                }
        	}
            else
            {
                u8State = 1;
            }
        }while(0 == u8State);

        if(storageExtManage.available_partition <= 1)
        {
		    ExtFLASH_PreReadInit();
        }

		#ifdef FLASH_DEBUG
			printf("\r\n[FLASH] After ExtFLASH_BuildCurrReadPartition:\r\n");
            printf("[FLASH] storageExtManage.available_partition = %d\r\n",
                        storageExtManage.available_partition);
            printf("[FLASH] RD partition,Start Time: %02d:%02d:%02d\r\n",
                        (uint16_t)(current_rd_partition.start_time >> 16),
                        (uint8_t)(current_rd_partition.start_time >> 8),
                        (uint8_t)(current_rd_partition.start_time));
            printf("[FLASH] current_rd_partition.start_page_addr = 0x%04x\r\n",
                        current_rd_partition.start_page_addr);
			printf("[FLASH] current_rd_partition.end_page_addr	 = 0x%04x\r\n",
                        current_rd_partition.end_page_addr);
			printf("[FLASH] storageIntManage.u32RdPointer = 0x%08x\r\n",
                        storageIntManage.u32RdPointer);
            printf("[FLASH] storageIntManage.u32RdNum_CurrPartition = %d\r\n",
                        storageIntManage.u32RdNum_CurrPartition);
            printf("[FLASH] WR partition,Start Time: %02d:%02d:%02d\r\n",
                        (uint16_t)(current_partition.start_time >> 16),
                        (uint8_t)(current_partition.start_time >> 8),
                        (uint8_t)(current_partition.start_time));
			printf("[FLASH] totel_len_InCurrReadPartition 	= %d\r\n",totel_len_InCurrReadPartition);
			printf("[FLASH] pre_read_info.pre_rd_len = %d\r\n",pre_read_info.pre_rd_len);
            printf("[FLASH] storageIntManage.u32WrPointer = 0x%08x\r\n",
                        storageIntManage.u32WrPointer);
			printf("[FLASH] storageIntManage.u32WrNum_CurrPartition = %d\r\n",
                        storageIntManage.u32WrNum_CurrPartition);
            printf("[FLASH] storageExtManage.totel_availabe_length = %d\r\n",
                        storageExtManage.totel_availabe_length);
		#endif
	}
    #ifdef FLASH_DEBUG
	else
	{
	    printf("\r\n[FLASH] No partition in Flash....\r\n");
	}
    #endif

	#ifdef FLASH_DEBUG
		printf("[FLASH] ExtFLASH_BuildCurrReadPartition,End\r\n");
	#endif
}

uint32_t ExtFLASH_GetCurPageLenArea(uint32_t u32TmpAddr)
{
    uint8_t partition_tmp[PAGE_HEADER_SIZE] = {0};
	page_head_t * p_page_header = (page_head_t *)partition_tmp;
    uint32_t u32CurPageStartAddr = 0;
    uint32_t u32LenTmp = 0;

    u32CurPageStartAddr = ExtFLASH_ComputeCurPageStartAddr(u32TmpAddr);
    CMD_READ(u32CurPageStartAddr, partition_tmp, PAGE_HEADER_SIZE);
    u32LenTmp = p_page_header->len.l;

    return (u32LenTmp);
}

uint32_t ExtFLASH_ReadOutInCurPart(flash_partition_t flash_partition, uint32_t u32TmpAddr)
{
	uint32_t cur_page_start_addr = 0;
    uint32_t u32TmpLastAddr = 0;
    uint32_t u32TmpLen = 0;
    uint32_t u32TmpLen1 = 0;

	/* Check parameters */
	usr_para_check(u32TmpAddr < FLASH_HEADER_SECTOR_BASE_ADDR);

	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_ReadOutInCurPart,u32TmpAddr = 0x%08x\r\n",u32TmpAddr);
	#endif

    if(0 == ExtFLASH_CheckInCurPart(flash_partition, u32TmpAddr))
    {
        cur_page_start_addr = ExtFLASH_ComputeCurPageStartAddr(u32TmpAddr);
        u32TmpLastAddr = ExtFLASH_ComputeLastPageStartAddr(u32TmpAddr);
        if((0 == ExtFLASH_CheckInCurPart(flash_partition, u32TmpLastAddr)))
        {
            u32TmpLen = ExtFLASH_GetCurPageLenArea(u32TmpLastAddr);
        }
        else
        {
             u32TmpLen = 0;
        }

        u32TmpLen1 = ((u32TmpAddr >= (cur_page_start_addr + PAGE_HEADER_SIZE))
                                    ? (u32TmpAddr - (cur_page_start_addr + PAGE_HEADER_SIZE)) : 0);

        u32TmpLen += u32TmpLen1;
    }
    else
    {
        u32TmpLen = 0;
    }

    #ifdef FLASH_DEBUG
        printf("[FLASH] ExtFLASH_ReadOutInCurPart,End,u32TmpLen = 0x%08x\r\n",u32TmpLen);
	#endif

	return (u32TmpLen);
}

/*********************************************************************************
  * Init the storage buffer
  * @param  None
  * @retval None
**********************************************************************************/
void ExtFLASH_StorageInit(void)
{
	uint8_t partition_tmp[20] = {0};
	flash_partition_t * p_flash_partition = (flash_partition_t *)partition_tmp;
	uint32_t partition_base_addr = FLASH_HEADER_SECTOR_BASE_ADDR;
	bool search_is_completed = false;
	uint16_t search_num = 0;
	uint32_t totel_len_at_partition = 0;
    StorageReadConstVar_t StorageReadConstVar;

	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_StorageInit\r\n");
	#endif

    EraseSectorTimerInit();

	/* Restore the pointer for writing and reading */
    memset(&current_rd_partition, 0, sizeof(flash_partition_t));
    memset(&current_partition, 0, sizeof(flash_partition_t));
	memset(&storageIntManage, 0, sizeof(storageIntManage_t));
	memset(&storageExtManage, 0, sizeof(storageExtManage_t));
	memset(&pre_write_info, 0, sizeof(pre_write_info_t));
	memset(&pre_read_info, 0, sizeof(pre_read_info_t));
    memset(&StorageReadConstVar, 0, sizeof(StorageReadConstVar_t));
	totel_len_InCurrReadPartition = 0;

	DisableExtFlashAccess();
	do{
		#ifdef FLASH_DEBUG
			printf("[FLASH] Now Reading the addr: 0x%08x\r\n",partition_base_addr + (search_num * sizeof(flash_partition_t)));
		#endif
		CMD_READ(partition_base_addr + (search_num * sizeof(flash_partition_t)),
		          partition_tmp,
		          sizeof(flash_partition_t));
		#ifdef FLASH_DEBUG
            printf("	[FLASH] Start Time: %02d:%02d:%02d\r\n",((partition_tmp[3] << 8) + partition_tmp[2]),partition_tmp[1],partition_tmp[0]);
            printf("	[FLASH] p_flash_partition.start_page_addr 	= 0x%04x\r\n",p_flash_partition->start_page_addr);
			printf("	[FLASH] p_flash_partition.end_page_addr 	= 0x%04x\r\n",p_flash_partition->end_page_addr);
		#endif
		if(ExtFLASH_IsPartitionValid(p_flash_partition) == true)
		{
			totel_len_at_partition = ExtFLASH_ComputeTotelLengthInCurrPartition(p_flash_partition);
			storageExtManage.totel_availabe_length += totel_len_at_partition;
            CheckTotelLenErr(storageExtManage.totel_availabe_length,1);
			storageExtManage.available_partition++;
			memcpy(&current_partition, p_flash_partition,sizeof(flash_partition_t));
			if(!search_num)
			{
				current_rd_partition.end_page_addr 		= p_flash_partition->end_page_addr;
				current_rd_partition.start_page_addr 	= p_flash_partition->start_page_addr;
				current_rd_partition.start_time 		= p_flash_partition->start_time;
				storageIntManage.u32RdPointer
                    = p_flash_partition->start_page_addr * PAGE_SIZE + PAGE_HEADER_OFFSET;
				totel_len_InCurrReadPartition 			= totel_len_at_partition;
			}
			search_num++;
		}
		else
		{
			search_is_completed = true;
		}

		#ifdef FLASH_DEBUG
			printf("[FLASH] search_num = %d\r\n",search_num);
		#endif
	}while(search_is_completed == false);

	if(storageExtManage.available_partition)
	{
		ExtFLASH_RestoreWrInfo(&current_partition);
        if(1 == storageExtManage.available_partition)
        {
            current_rd_partition.start_time = current_partition.start_time;
            current_rd_partition.start_page_addr = current_partition.start_page_addr;
            current_rd_partition.end_page_addr = current_partition.end_page_addr;

            if(0 == ExtFLASH_GetRdAddrFromConst(&StorageReadConstVar))
            {
                StorageReadConstVar.u32StorageReadAddr =
                    ExtFLASH_ComputeCurPageStartAddr(StorageReadConstVar.u32StorageReadAddr);
                if((0 == ExtFLASH_CheckInCurPart(current_rd_partition, StorageReadConstVar.u32StorageReadAddr))
               && (0 == ExtFLASH_CheckIsSamePart(&current_rd_partition, &StorageReadConstVar.flash_read_partition)))
                {
                    storageIntManage.u32RdPointer
                        = (true == ExtFLASH_IsPageBoundary(StorageReadConstVar.u32StorageReadAddr))
                        ? (ExtFLASH_ComputeCurPageStartAddr(StorageReadConstVar.u32StorageReadAddr) + PAGE_HEADER_OFFSET)
                        : (StorageReadConstVar.u32StorageReadAddr);
                    totel_len_InCurrReadPartition = storageIntManage.u32WrNum_CurrPartition;
                    storageIntManage.u32RdNum_CurrPartition
                        = ExtFLASH_ReadOutInCurPart(current_rd_partition, StorageReadConstVar.u32StorageReadAddr);
                    storageExtManage.totel_availabe_length
                        = (storageExtManage.totel_availabe_length >= storageIntManage.u32RdNum_CurrPartition)
                        ? (storageExtManage.totel_availabe_length - storageIntManage.u32RdNum_CurrPartition)
                        : (0);
                    CheckTotelLenErr(storageExtManage.totel_availabe_length,2);
                }
                else
                {
        		    storageIntManage.u32RdPointer
                        = (current_rd_partition.start_page_addr << 8) + PAGE_HEADER_OFFSET;
                }
            }
            else
            {
    		    storageIntManage.u32RdPointer
                    = (current_rd_partition.start_page_addr << 8) + PAGE_HEADER_OFFSET;
            }
        }
        else
        {
            if(0 == ExtFLASH_GetRdAddrFromConst(&StorageReadConstVar))
            {
                if((0 == ExtFLASH_CheckInCurPart(current_rd_partition, StorageReadConstVar.u32StorageReadAddr))
               && (0 == ExtFLASH_CheckIsSamePart(&current_rd_partition, &StorageReadConstVar.flash_read_partition)))
                {
                    storageIntManage.u32RdPointer
                        = (true == ExtFLASH_IsPageBoundary(StorageReadConstVar.u32StorageReadAddr))
                          ? (ExtFLASH_ComputeCurPageStartAddr(StorageReadConstVar.u32StorageReadAddr) + PAGE_HEADER_OFFSET)
                          : (StorageReadConstVar.u32StorageReadAddr);
                    totel_len_InCurrReadPartition
                        = ExtFLASH_ComputeTotelLengthInCurrPartition(&current_rd_partition);
                    storageIntManage.u32RdNum_CurrPartition
                        = ExtFLASH_ReadOutInCurPart(current_rd_partition, StorageReadConstVar.u32StorageReadAddr);
                    storageExtManage.totel_availabe_length
                        = (storageExtManage.totel_availabe_length >= storageIntManage.u32RdNum_CurrPartition)
                         ? (storageExtManage.totel_availabe_length - storageIntManage.u32RdNum_CurrPartition)
                         : (0);
                    CheckTotelLenErr(storageExtManage.totel_availabe_length,3);
                }
                else
                {
                    storageIntManage.u32RdPointer
                        = (current_rd_partition.start_page_addr << 8) + PAGE_HEADER_OFFSET;
                }
            }
            else
            {
                storageIntManage.u32RdPointer
                    = (current_rd_partition.start_page_addr << 8) + PAGE_HEADER_OFFSET;
            }
        }

        ExtFLASH_PreReadInit();
	}

	EnableExtFlashAccess();

	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] After ExtFLASH_StorageInit:\r\n");

        printf("    [FLASH] RD partition,start_time: %02d:%02d:%02d\r\n",(uint16_t)(current_rd_partition.start_time >> 16),
                                                                         (uint8_t)(current_rd_partition.start_time >> 8),
                                                                         (uint8_t)(current_rd_partition.start_time));
		printf("	[FLASH] current_rd_partition.start_page_addr = 0x%04x\r\n",current_rd_partition.start_page_addr);
		printf("	[FLASH] current_rd_partition.end_page_addr   = 0x%04x\r\n",current_rd_partition.end_page_addr);
		printf("	[FLASH] storageIntManage.u32RdPointer	= 0x%08x\r\n",storageIntManage.u32RdPointer);
        printf("	[FLASH] storageIntManage.u32RdNum_CurrPartition = %d\r\n",storageIntManage.u32RdNum_CurrPartition);
        printf("	[FLASH] totel_len_InCurrReadPartition 	= %d\r\n",totel_len_InCurrReadPartition);
		printf("	[FLASH] pre_read_info.pre_rd_len 	= %d\r\n\r\n",pre_read_info.pre_rd_len);

        printf("    [FLASH] WR partition,start_time: %02d:%02d:%02d\r\n",(uint16_t)(current_partition.start_time >> 16),
                                                                         (uint8_t)(current_partition.start_time >> 8),
                                                                         (uint8_t)(current_partition.start_time));
		printf("	[FLASH] current_partition.start_page_addr 	= 0x%04x\r\n",current_partition.start_page_addr);
		printf("	[FLASH] current_partition.end_page_addr 	= 0x%04x\r\n",current_partition.end_page_addr);
		printf("	[FLASH] storageIntManage.u32WrPointer 	= 0x%08x\r\n",storageIntManage.u32WrPointer);
		printf("	[FLASH] storageIntManage.u32WrNum_CurrPartition = %d\r\n\r\n",storageIntManage.u32WrNum_CurrPartition);

		printf("	[FLASH] storageExtManage.available_partition 	= %d\r\n",storageExtManage.available_partition);
		printf("	[FLASH] storageExtManage.totel_availabe_length 	= %d\r\n",storageExtManage.totel_availabe_length);
	#endif
}

/*********************************************************************************
  * Search valid start page
  * @param  None
  * @retval None
**********************************************************************************/
flashOperationMsg ExtFLASH_SearchValidStartPage(uint32_t flash_start_addr, flash_partition_t * p_flash_partition)
{
	flashOperationMsg msg = FLASH_OPERATION_SUCCESS;
    uint8_t data_tmp[8] = {0};
	page_header_t * p_page_header = (page_header_t *)data_tmp;
    uint32_t page_base_addr = flash_start_addr;
	bool bSearchCompleted = false;

    /* Check paremeters */
    usr_para_check(p_flash_partition != NULL);

	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_SearchValidStartPage,flash_start_addr = 0x%08x\r\n",flash_start_addr);
	#endif

    // Mx25 Spi Config
    MX25_SPI_Configuration();

	do{
		CMD_READ(page_base_addr, data_tmp, sizeof(page_header_t));
		#ifdef FLASH_DEBUG
			printf("	[FLASH] p_page_header.start_time 		= 0x%08x\r\n",p_page_header->start_time);
            printf("	[FLASH] p_page_header.len 		= 0x%08x\r\n",p_page_header->len);
		#endif
		if(ExtFLASH_IsPageStarttimeValid(p_page_header->start_time) == true)
		{
            p_flash_partition->start_page_addr = (page_base_addr/PAGE_SIZE);
            bSearchCompleted = true;
            msg = FLASH_OPERATION_SUCCESS;

            #ifdef FLASH_DEBUG
    		    printf("    [FLASH] ExtFLASH_SearchValidStartPage,End\r\n");
                printf("	[FLASH] Start Time: %02d:%02d:%02d\r\n",(uint16_t)(p_flash_partition->start_time >> 16),
                                                                (uint8_t)(p_flash_partition->start_time >> 8),
                                                                (uint8_t)(p_flash_partition->start_time));
        		printf("	[FLASH] p_flash_partition->start_page_addr = 0x%04x\r\n",p_flash_partition->start_page_addr);
        		printf("	[FLASH] p_flash_partition->end_page_addr   = 0x%04x\r\n",p_flash_partition->end_page_addr);
    	    #endif
		}
		else
		{
            page_base_addr += PAGE_SIZE;
            if(page_base_addr >= FLASH_HEADER_SECTOR_BASE_ADDR)
            {
                bSearchCompleted = true;
                msg = FLASH_OPERATION_FAIL;
            }
		}
	}while(bSearchCompleted == false);

    #ifdef FLASH_DEBUG
         printf("   [FLASH] ExtFLASH_SearchValidStartPage,End\r\n");
	#endif

	return (msg);
}

/*********************************************************************************
  * Restore the specification partition
  * @param  p_flash_partition: the addr of the partition
  * @param  p_u8Flag: the end flag
  * @retval flashOperationMsg
**********************************************************************************/
flashOperationMsg ExtFLASH_RestoreOnePartition(flash_partition_t * p_flash_partition, uint8_t * p_u8Flag)
{
    flashOperationMsg msg = FLASH_OPERATION_SUCCESS;
	uint32_t read_addr = 0;
    uint8_t data_tmp[8] = {0};
    page_header_t * p_page_header = (page_header_t *)data_tmp;
    bool bSearchEnd = FALSE;
    uint32_t start_time_tmp = 0;

    /* Check parameters */
    usr_para_check(p_flash_partition != NULL);

	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_RestoreOnePartition,0x%08x\r\n",(p_flash_partition->start_page_addr << 8));
	#endif

    // Mx25 Spi Config
    MX25_SPI_Configuration();

    // Read the first start time
    read_addr = (p_flash_partition->start_page_addr << 8);
    CMD_READ(read_addr, data_tmp, sizeof(page_header_t));
    start_time_tmp = p_page_header->start_time;
    #ifdef FLASH_DEBUG
		printf("	[FLASH] start_time_tmp    = 0x%08x\r\n",start_time_tmp);
	#endif

    if(true == ExtFLASH_IsPageStarttimeValid(start_time_tmp))
    {
        // Search
    	do{
    		CMD_READ(read_addr, data_tmp, sizeof(page_header_t));
            #ifdef FLASH_DEBUG
                CheckRead(data_tmp, sizeof(page_header_t));
            #endif
            if(p_page_header->start_time == start_time_tmp)
            {
                read_addr += PAGE_SIZE;
                if(read_addr >= FLASH_HEADER_SECTOR_BASE_ADDR)
                {
                    p_flash_partition->end_page_addr = (read_addr - PAGE_SIZE)/PAGE_SIZE;
                    p_flash_partition->start_time = start_time_tmp;
                    if(p_u8Flag != NULL)
                    {
                        *p_u8Flag = 1;
                    }

                    bSearchEnd = TRUE;
                }
            }
            else
            {
                read_addr = ((read_addr >= PAGE_SIZE)
                            ? (read_addr - PAGE_SIZE)
                            : (read_addr + FLASH_HEADER_SECTOR_BASE_ADDR - PAGE_SIZE));
                p_flash_partition->end_page_addr = (read_addr/PAGE_SIZE);
                p_flash_partition->start_time = start_time_tmp;
                bSearchEnd = TRUE;
            }
    	}while(bSearchEnd == FALSE);

        msg = FLASH_OPERATION_SUCCESS;

        #ifdef FLASH_DEBUG
    		printf("    [FLASH] ExtFLASH_RestoreOnePartition,FLASH_OPERATION_SUCCESS,End\r\n");
            printf("	[FLASH] Start Time: %02d:%02d:%02d\r\n",(uint16_t)(p_flash_partition->start_time >> 16),
                                                                (uint8_t)(p_flash_partition->start_time >> 8),
                                                                (uint8_t)(p_flash_partition->start_time));
            printf("	[FLASH] p_flash_partition.start_page_addr    = 0x%04x\r\n",p_flash_partition->start_page_addr);
            printf("	[FLASH] p_flash_partition.end_page_addr    = 0x%04x\r\n",p_flash_partition->end_page_addr);
    	#endif
    }
    else
    {
        msg = FLASH_OPERATION_FAIL;
        #ifdef FLASH_DEBUG
    		printf("    [FLASH] ExtFLASH_RestoreOnePartition,FLASH_OPERATION_FAIL,End\r\n");
        #endif
    }

	return (msg);
}

void ExtFLASH_ConnectPartition(uint8_t totel_num, flash_partition_t * p_flash_partition)
{
	uint8_t partition_tmp[8];
    flash_partition_t * p_flash_partition_tmp = (flash_partition_t *)partition_tmp;
	uint32_t read_start_addr = FLASH_BACKUP_SECTOR_BASE_ADDR;
	uint32_t write_addr = FLASH_HEADER_SECTOR_BASE_ADDR;
    uint16_t index = 0;

	#ifdef FLASH_DEBUG
		printf("\r\nExtFLASH_ConnectPartition\r\n");
	#endif

    read_start_addr = FLASH_BACKUP_SECTOR_BASE_ADDR + sizeof(flash_partition_t);
	for(index = 0;index < (totel_num - 1);index ++)
	{
		CMD_READ(read_start_addr,partition_tmp,sizeof(flash_partition_t));
		ExtFLASH_WriteHeaderAreaPageProgram(write_addr, partition_tmp, sizeof(flash_partition_t));
        write_addr += sizeof(flash_partition_t);
        read_start_addr += sizeof(flash_partition_t);
	}

    p_flash_partition_tmp->start_time = p_flash_partition->start_time;
    p_flash_partition_tmp->start_page_addr = p_flash_partition->start_page_addr;
    p_flash_partition_tmp->end_page_addr = p_flash_partition->end_page_addr;
    ExtFLASH_WriteHeaderAreaPageProgram(write_addr, partition_tmp, sizeof(flash_partition_t));

	#ifdef FLASH_DEBUG
		printf("	[FLASH] ExtFLASH_ConnectPartition,End,check the result: \r\n");
		CheckWrite(FLASH_HEADER_SECTOR_BASE_ADDR, PAGE_SIZE);
	#endif
}

void ExtFLASH_CheckSamePartition(void)
{
    uint8_t partition_tmp[8];
	flash_partition_t * p_flash_partition = (flash_partition_t *)partition_tmp;
	flash_partition_t flash_partition_first;
    flash_partition_t flash_partition_last;
    uint32_t read_start_addr = 0;
    uint32_t search_index = 0;
    uint8_t index = 0;
    bool bSearchEnd = FALSE;

	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_CheckSamePartition\r\n");
	#endif

    read_start_addr = FLASH_HEADER_SECTOR_BASE_ADDR;
    CMD_READ(read_start_addr, partition_tmp, sizeof(flash_partition_t));
    flash_partition_first.start_time = p_flash_partition->start_time;
    flash_partition_first.start_page_addr = p_flash_partition->start_page_addr;
    flash_partition_first.end_page_addr = p_flash_partition->end_page_addr;

    read_start_addr += sizeof(flash_partition_t);
    do{
        CMD_READ(read_start_addr, partition_tmp, sizeof(flash_partition_t));
        if((p_flash_partition->start_time == 0xFFFFFFFF) ||
           (p_flash_partition->start_page_addr == 0xFFFF) ||
            (p_flash_partition->end_page_addr == 0xFFFF))
        {
            bSearchEnd = TRUE;
        }
        else
        {
            flash_partition_last.start_time = p_flash_partition->start_time;
            flash_partition_last.start_page_addr = p_flash_partition->start_page_addr;
            flash_partition_last.end_page_addr = p_flash_partition->end_page_addr;
            read_start_addr += sizeof(flash_partition_t);
            search_index++;
        }
    }while(bSearchEnd == FALSE);

    if(flash_partition_first.start_time == flash_partition_last.start_time)
    {
        flash_partition_first.start_page_addr = flash_partition_last.start_page_addr;

        CMD_SE(FLASH_BACKUP_SECTOR_BASE_ADDR);
        index = 0;
        while((IsFlashBusy() == TRUE) && (++ index < 30))
        {
            delay1ms(10);
        }

        ExtFLASH_BackupOneSector(FLASH_HEADER_SECTOR_BASE_ADDR);

        CMD_SE(FLASH_HEADER_SECTOR_BASE_ADDR);
        index = 0;
        while((IsFlashBusy() == TRUE) && (++ index < 30))
        {
            delay1ms(10);
        }

        ExtFLASH_ConnectPartition(search_index, &flash_partition_first);
    }

    #ifdef FLASH_DEBUG
		printf("    [FLASH] ExtFLASH_CheckSamePartition,End\r\n");
	#endif
}

void ExtFLASH_SequencePartition(void)
{
    uint8_t partition_tmp[8];
	flash_partition_t * p_flash_partition = (flash_partition_t *)partition_tmp;
	flash_partition_t flash_partition_last;
    uint32_t read_start_addr = 0;
    uint32_t write_start_addr = 0;
    uint32_t search_index = 0;
    uint8_t index = 0;
    bool bSearchEnd = FALSE;

	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_SequencePartition\r\n");
	#endif

    read_start_addr = FLASH_HEADER_SECTOR_BASE_ADDR;
    CMD_READ(read_start_addr, partition_tmp, sizeof(flash_partition_t));
    flash_partition_last.start_time = p_flash_partition->start_time;
    search_index = 1;

    do{
        read_start_addr += sizeof(flash_partition_t);
        CMD_READ(read_start_addr, partition_tmp, sizeof(flash_partition_t));
    	#ifdef FLASH_DEBUG
            printf("    [FLASH] Start Time: %04d-%02d-%02d\r\n",
                                (uint16_t)(p_flash_partition->start_time >> 16),
                                (uint8_t)(p_flash_partition->start_time >> 8),
                                (uint8_t)(p_flash_partition->start_time));
    	#endif

        if((p_flash_partition->start_time == 0xFFFFFFFF) ||
            (p_flash_partition->start_page_addr == 0xFFFF) ||
            (p_flash_partition->end_page_addr == 0xFFFF))
        {
            search_index = 0;
            bSearchEnd = TRUE;
        }
        else
        {
            if(flash_partition_last.start_time > p_flash_partition->start_time)
            {
                bSearchEnd = TRUE;
            }
            else
            {
                search_index++;
            }
        }
    }while(bSearchEnd == FALSE);

    #ifdef FLASH_DEBUG
        printf("    [FLASH] search_index = %d\r\n",search_index);
    #endif

    if(search_index > 0)
    {
        CMD_SE(FLASH_BACKUP_SECTOR_BASE_ADDR);
        delay1ms(200);
        ExtFLASH_BackupOneSector(FLASH_HEADER_SECTOR_BASE_ADDR);
        CMD_SE(FLASH_HEADER_SECTOR_BASE_ADDR);
        delay1ms(200);

        bSearchEnd = FALSE;
        read_start_addr = FLASH_BACKUP_SECTOR_BASE_ADDR + (search_index * sizeof(flash_partition_t));
        write_start_addr = FLASH_HEADER_SECTOR_BASE_ADDR;
        do{
            CMD_READ(read_start_addr, partition_tmp, sizeof(flash_partition_t));
        	#ifdef FLASH_DEBUG
                printf("    [FLASH] Start Time: %04d-%02d-%02d\r\n",(uint16_t)(p_flash_partition->start_time >> 16),(uint8_t)(p_flash_partition->start_time >> 8),(uint8_t)(p_flash_partition->start_time));
        	#endif

            if(p_flash_partition->start_time == 0xFFFFFFFF)
            {
                bSearchEnd = TRUE;
                #ifdef FLASH_DEBUG
            		printf("    [FLASH] End\r\n");
            	#endif

            }
            else
            {
                CMD_PageProgram(write_start_addr, partition_tmp, sizeof(flash_partition_t));
                write_start_addr += sizeof(flash_partition_t);
                read_start_addr += sizeof(flash_partition_t);
            }
        }while(bSearchEnd == FALSE);

        read_start_addr = FLASH_BACKUP_SECTOR_BASE_ADDR;
        for(index = 0;index < search_index;index ++)
        {
            CMD_READ(read_start_addr, partition_tmp, sizeof(flash_partition_t));
        	#ifdef FLASH_DEBUG
                printf("    [FLASH] Start Time: %04d-%02d-%02d\r\n",
                                    (uint16_t)(p_flash_partition->start_time >> 16),
                                    (uint8_t)(p_flash_partition->start_time >> 8),
                                    (uint8_t)(p_flash_partition->start_time));
        	#endif

            CMD_PageProgram(write_start_addr, partition_tmp, sizeof(flash_partition_t));
            write_start_addr += sizeof(flash_partition_t);
            read_start_addr += sizeof(flash_partition_t);
        }
    }

    #ifdef FLASH_DEBUG
		printf("    [FLASH] ExtFLASH_SequencePartition,End\r\n");
        CheckWrite(FLASH_HEADER_SECTOR_BASE_ADDR, PAGE_SIZE);
	#endif
}


void ExtFLASH_RestorePartition(void)
{
	flash_partition_t flash_partition_tmp;
    uint32_t search_index = 0;
    uint32_t u32FlashStart_tmp = 0;
    bool bSearchEnd = FALSE;
    uint8_t flag = 0;

	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_RestorePartition\r\n");
	#endif

    /* Mx25 Spi Config */
    MX25_SPI_Configuration();

    memset(pre_write_info.pre_wr_buffer, 0, sizeof(pre_write_info_t));
    memset(pre_write_info_backup.pre_wr_buffer, 0, sizeof(pre_write_info_t));
    pre_write_info.pre_wr_len = 0;
    pre_write_info_backup.pre_wr_len = 0;

	CMD_SE(FLASH_HEADER_SECTOR_BASE_ADDR);
	delay1ms(200);

    u32FlashStart_tmp = 0;
    do{
        if(FLASH_OPERATION_SUCCESS
            == ExtFLASH_SearchValidStartPage(u32FlashStart_tmp, &flash_partition_tmp))
        {
            if(FLASH_OPERATION_SUCCESS
                == ExtFLASH_RestoreOnePartition(&flash_partition_tmp, &flag))
            {
                ExtFLASH_AddEachPartition(search_index ++, &flash_partition_tmp);
                flash_partition_tmp.start_page_addr =
                    (ExtFLASH_ComputeNextSectorStartAddr(flash_partition_tmp.end_page_addr * PAGE_SIZE))
                    / PAGE_SIZE;
                if(1 == flag)
                {
                    bSearchEnd = TRUE;
                    #ifdef FLASH_DEBUG
                		printf("    [FLASH] ExtFLASH_RestoreOnePartition,No memery\r\n");
                    #endif
                    break;
                    //while(1);
                }
            }

            u32FlashStart_tmp = (flash_partition_tmp.end_page_addr << 8) + PAGE_SIZE;
        }
        else
        {
            bSearchEnd = TRUE;
            #ifdef FLASH_DEBUG
                printf("    [FLASH] There is no valid partition\r\n");
    	    #endif
        }
    }while(bSearchEnd == FALSE);

    ExtFLASH_CheckSamePartition();

    ExtFLASH_SequencePartition();

    #ifdef FLASH_DEBUG
        CheckPartition();
    #endif

    #ifdef FLASH_DEBUG
		printf("    [FLASH] ExtFLASH_ExtRestorePartition,Over,End\r\n");
	#endif
}

/*********************************************************************************
  * Create a partition
  * @param  start_time: start time
  * @retval flashOperationMsg
**********************************************************************************/
flashOperationMsg ExtFLASH_ExtCreatePartition(uint32_t start_time)
{
	flashOperationMsg state = FLASH_OPERATION_SUCCESS;
	uint32_t current_page_start_addr = 0;
    uint32_t del_len = 0;
    uint8_t u8Cnt = 0;

	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_ExtCreatePartition\r\n");
        printf("    [FLASH] Start Time: %02d-%02d-%02d\r\n",
                            (uint16_t)(start_time >> 16),(uint8_t)(start_time >> 8),(uint8_t)(start_time));
		printf("    [FLASH] storageIntManage.u32WrPointer = 0x%08x\r\n",storageIntManage.u32WrPointer);
        printf("    [FLASH] storageIntManage.u32RdPointer = 0x%08x\r\n",storageIntManage.u32RdPointer);
	#endif

	if(start_time != current_partition.start_time)
	{
        /* MX25 Spi Configuration */
        MX25_SPI_Configuration();

		if(0 == storageExtManage.available_partition)
		{
			#ifdef FLASH_DEBUG
				printf("[FLASH] Create the first Partition...\r\n");
			#endif
			/* Builds the new partion parameters */
			current_partition.start_page_addr 	    = 0;
			current_partition.end_page_addr		    = 0;
			current_partition.start_time		    = start_time;

			current_rd_partition.start_page_addr 	= 0;
			current_rd_partition.end_page_addr		= 0;
			current_rd_partition.start_time		    = start_time;

			/* Update the local parameters */
            storageIntManage.u32RdNum_CurrPartition = 0;
            storageIntManage.u32RdPointer = current_rd_partition.start_page_addr * PAGE_SIZE
                + PAGE_HEADER_SIZE;
			storageIntManage.u32WrPointer 		    = 0;
			storageIntManage.u32WrNum_CurrPartition = 0;

			/* Update the extern parameters */
			storageExtManage.available_partition++;

            pre_read_info.pre_rd_len = 0;
            pre_write_info.pre_wr_len = 0;
            storageExtManage.totel_availabe_length = 0;

            CMD_SE(FLASH_HEADER_SECTOR_BASE_ADDR);
	        delay1ms(200);
            ExtFLASH_WritePartitionInfo(1,
                                        current_partition.start_time,
                                        current_partition.start_page_addr,
                                        current_partition.end_page_addr);

			#ifdef FLASH_DEBUG
				printf("[FLASH] Create the first Partition,END...\r\n");
			#endif
		}
		else
		{
            #ifdef FLASH_DEBUG
                printf("    [FLASH] storageExtManage.totel_availabe_length = 0x%08x\r\n",
                                    storageExtManage.totel_availabe_length);
                printf("    [FLASH] totel_len_InCurrReadPartition = 0x%08x\r\n",
                                    totel_len_InCurrReadPartition);
	        #endif
            if((storageExtManage.totel_availabe_length) && (totel_len_InCurrReadPartition))
			{
                /* First,end the current partition  */
    			/* Backup current partition infomation to last_partition */
                //current_page_start_addr 		= ExtFLASH_ComputeCurPageStartAddr(storageIntManage.u32WrPointer);
                last_partition.start_time       = current_partition.start_time;
                last_partition.start_page_addr 	= current_partition.start_page_addr;

                if((storageIntManage.u32WrNum_CurrPartition > 0) ||(pre_write_info.pre_wr_len > 0))
                {
                    if(pre_write_info.pre_wr_len > 0)
                    {
        				/* Program the data length to the data length area in current page */
        				storageIntManage.u32WrNum_CurrPartition += pre_write_info.pre_wr_len;

                        /* Check if this sector should be erased? */
                        if(TRUE == ExtFLASH_IsSectionBoundary(storageIntManage.u32WrPointer))
                        {
                            CMD_SE(storageIntManage.u32WrPointer);
                            while(u8Cnt < SECTOR_ERASE_MAX_CNT)
                            {
                                delay1ms(50);
                                u8Cnt ++;
                            }
                        }

                        /* Page Write */
                        ExtFLASH_RealFlashPageWrite(storageIntManage.u32WrPointer,
                                                    last_partition.start_time,
                                                    storageIntManage.u32WrNum_CurrPartition,
                                                    pre_write_info.pre_wr_buffer,
                                                    pre_write_info.pre_wr_len);
                        storageIntManage.u32WrPointer += PAGE_SIZE;
                        storageIntManage.u32WrPointer %= FLASH_HEADER_SECTOR_BASE_ADDR;

                        /* Clear buffers */
        				memset(pre_write_info.pre_wr_buffer, 0, pre_write_info.pre_wr_len);
        				pre_write_info.pre_wr_len = 0;
                    }

                    /* Compute the end addr of current partition again */
                    last_partition.end_page_addr =
                        ExtFLASH_ComputeLastPageStartAddr(storageIntManage.u32WrPointer) / PAGE_SIZE;

                    /* Builds the new partion parameters */
                    if(TRUE == ExtFLASH_IsSectionBoundary(storageIntManage.u32WrPointer))
                    {
                        current_partition.start_page_addr = storageIntManage.u32WrPointer / PAGE_SIZE;
                    }
                    else
                    {
                        current_partition.start_page_addr =
                            ExtFLASH_ComputeNextSectorStartAddr(storageIntManage.u32WrPointer) / PAGE_SIZE;
                    }
                }
                else
                {
                    last_partition_is_empty = 1;

                    /* Builds the new partion parameters */
                    current_partition.start_page_addr 	= last_partition.start_page_addr;
                }

				#ifdef FLASH_DEBUG
					CheckWrite(storageIntManage.u32WrPointer, PAGE_SIZE);
				#endif

                /* Builds the new partion parameters */
                current_partition.start_time		= start_time;
                current_partition.end_page_addr		= current_partition.start_page_addr;

				/* Update the local parameters */
				storageIntManage.u32WrPointer 		= current_partition.start_page_addr * PAGE_SIZE;
				storageIntManage.u32WrNum_CurrPartition = 0;

				/* Update the extern parameters */
				storageExtManage.available_partition++;
			}
			else
			{
				#ifdef FLASH_DEBUG
					printf("[FLASH] Current partition is no data!!!!\r\n");
					printf("[FLASH] pre_write_info.pre_wr_len = %d\r\n",pre_write_info.pre_wr_len);
                    printf("[FLASH] storageIntManage.u32WrPointer = 0x%08x\r\n",
                                    storageIntManage.u32WrPointer);
                    printf("[FLASH] storageIntManage.u32WrNum_CurrPartition = 0x%08x\r\n",
                                    storageIntManage.u32WrNum_CurrPartition);
				#endif
                if(storageIntManage.u32WrNum_CurrPartition)
                {
    				if(pre_write_info.pre_wr_len)
    				{
                        current_page_start_addr =
                            ExtFLASH_ComputeCurPageStartAddr(storageIntManage.u32WrPointer);

    					/* Program the data length to the data length area in current page */
    					storageIntManage.u32WrNum_CurrPartition += pre_write_info.pre_wr_len;

                        /* Check if this sector should be erased? */
                        if(TRUE == ExtFLASH_IsSectionBoundary(current_page_start_addr))
                        {
                            CMD_SE(current_page_start_addr);
                            while(u8Cnt < SECTOR_ERASE_MAX_CNT)
                            {
                                delay1ms(50);
                                u8Cnt ++;
                            }
                        }

                        /* Page Write */
                        ExtFLASH_RealFlashPageWrite(current_page_start_addr,
                                                    current_partition.start_time,
                                                    storageIntManage.u32WrNum_CurrPartition,
                                                    pre_write_info.pre_wr_buffer,
                                                    pre_write_info.pre_wr_len);

                        /* Clear buffers */
        				memset(pre_write_info.pre_wr_buffer, 0, pre_write_info.pre_wr_len);
        				pre_write_info.pre_wr_len = 0;
    				}

                    /* Builds the new partition parameters */
                    current_partition.start_time = start_time;
    				current_partition.start_page_addr =
                        ExtFLASH_ComputeNextSectorStartAddr(storageIntManage.u32WrPointer) / PAGE_SIZE;
    				current_partition.end_page_addr	= current_partition.start_page_addr;
                }
                else
                {
                    /* There is really no data in current partition */
                    /* Builds the new partition parameters */
    				current_partition.start_time = start_time;
                    current_partition.start_page_addr =
                        ExtFLASH_ComputeCurrSectorStartAddr(storageIntManage.u32WrPointer) / PAGE_SIZE;
    				current_partition.end_page_addr = current_partition.start_page_addr;
                }

				/* Update the local parameters */
				storageIntManage.u32WrPointer = current_partition.start_page_addr * PAGE_SIZE;
				storageIntManage.u32WrNum_CurrPartition = 0;
                storageIntManage.u32RdNum_CurrPartition = 0;

				/* Update the extern parameters */
				storageExtManage.available_partition++;
				last_partition_is_empty = 1;
			}

			#ifdef FLASH_DEBUG
				printf("[FLASH] storageIntManage.u32WrPointer = 0x%08x\r\n",
                    storageIntManage.u32WrPointer);
			#endif
			/* Check if deleting partition is necessary?  */
            if(TRUE == ExtFlash_CheckNeedDeletePartition(storageIntManage.u32WrPointer))
            {
                del_len = ExtFLASH_GetLengthInDeletePartition();
                #ifdef FLASH_DEBUG
        			printf("	[FLASH] Need to delete a partition,del_len = %d\r\n",del_len);
        		#endif

                storageExtManage.totel_availabe_length
                = (storageExtManage.totel_availabe_length >= del_len)
                  ? (storageExtManage.totel_availabe_length - del_len)
                  : 0;
                CheckTotelLenErr(storageExtManage.totel_availabe_length,4);
                storageIntManage.u32RdNum_CurrPartition = 0;
                u8Flag_DelPartInCreatePart = 1;
            }
            else
            {
                u8Flag_DelPartInCreatePart = 0;
            }

			/* Now,update the header area */
			ExtFLASH_UpdateHeaderAreaInCreatePartition();
		}

        #ifdef FLASH_DEBUG
            printf("\r\n[FLASH] Current Information:\r\n");
            printf("    [FLASH] RD partition,start_time: %02d:%02d:%02d\r\n",
                                    (uint16_t)(current_rd_partition.start_time >> 16),
                                    (uint8_t)(current_rd_partition.start_time >> 8),
                                    (uint8_t)(current_rd_partition.start_time));
            printf("	[FLASH] current_rd_partition.start_page_addr = 0x%04x\r\n",
                                    current_rd_partition.start_page_addr);
            printf("	[FLASH] current_rd_partition.end_page_addr   = 0x%04x\r\n",
                                    current_rd_partition.end_page_addr);
            printf("	[FLASH] storageIntManage.u32RdPointer	= 0x%08x\r\n",
                                    storageIntManage.u32RdPointer);
            printf("	[FLASH] totel_len_InCurrReadPartition 	= %d\r\n",
                                    totel_len_InCurrReadPartition);
            printf("	[FLASH] pre_read_info.pre_rd_len 	= %d\r\n\r\n",
                                    pre_read_info.pre_rd_len);

            printf("	[FLASH] storageIntManage.u32WrPointer 	= 0x%08x\r\n",
                                    storageIntManage.u32WrPointer);
            printf("	[FLASH] storageIntManage.u32WrNum_CurrPartition = %d\r\n\r\n",
                                    storageIntManage.u32WrNum_CurrPartition);

            printf("	[FLASH] storageExtManage.available_partition 	= %d\r\n",
                                    storageExtManage.available_partition);
            printf("	[FLASH] storageExtManage.totel_availabe_length 	= %d\r\n\r\n",
                                    storageExtManage.totel_availabe_length);
            printf("    [FLASH] WR partition,start_time: %02d:%02d:%02d\r\n",
                                    (uint16_t)(current_partition.start_time >> 16),
                                    (uint8_t)(current_partition.start_time >> 8),
                                    (uint8_t)(current_partition.start_time));
            printf("	[FLASH] current_partition.start_page_addr 	= 0x%04x\r\n",
                                    current_partition.start_page_addr);
            printf("	[FLASH] current_partition.end_page_addr 	= 0x%04x\r\n",
                                    current_partition.end_page_addr);
        #endif
	}
	#ifdef FLASH_DEBUG
	else
	{
		printf("[FLASH] The specification partition has been created\r\n");
	}
	#endif

    EnableExtFlashAccess();

	return state;
}

/*********************************************************************************
  * Write data to storage for external user
  * @param  type: data type
  * @param  pu8Src: the source to write
  * @param  u8Length: the length to write
  * @retval flashOperationMsg
**********************************************************************************/
flashOperationMsg ExtFLASH_ExtWrite(store_type_e type,uint8_t * pu8Src,uint8_t u8Length)
{
	flashOperationMsg state = FLASH_OPERATION_SUCCESS;
    #ifdef FLASH_DEBUG
        uint8_t i;
    #endif

	/* Check parameters */
	usr_para_check((type == NON_MONITOR_TYPE)||(type == MONITOR_TYPE));
	usr_para_check(pu8Src != NULL);
	usr_para_check(u8Length > 0);

    /* Mx25 Spi Config */
    MX25_SPI_Configuration();

	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_ExtWrite\r\n");
		printf("	[FLASH] INCOMING DATA : ");
		for(i = 0;i < u8Length;i ++)
		{
			printf("0x%02x, ",*(pu8Src + i));
		}
		printf("\r\n");
        printf("	[FLASH] Analyzed,Time: %d:%d:%d,parameterID: %d,HR: %d,Spo2: %d,MoveLevel: %d,Confi: %d\r\n",
            *(pu8Src + 0),*(pu8Src + 1),*(pu8Src + 2),*(pu8Src + 3),*(pu8Src + 4),*(pu8Src + 5),*(pu8Src + 6),*(pu8Src + 7));
	#endif

	#ifdef FLASH_DEBUG
		printf("	[FLASH] storageExtManage.available_partition = %d\r\n",
                                storageExtManage.available_partition);
        printf("	[FLASH] storageIntManage.u32RdPointer = 0x%08x\r\n",
                                storageIntManage.u32RdPointer);
        printf("	[FLASH] storageIntManage.u32RdNum_CurrPartition = 0x%08x\r\n",
                                storageIntManage.u32RdNum_CurrPartition);
    #endif
    if(storageExtManage.available_partition)
    {
        storageExtManage.totel_availabe_length += u8Length;
        CheckTotelLenErr(storageExtManage.totel_availabe_length,5);
        if((pre_write_info.pre_wr_len + u8Length) < (PAGE_SIZE - PAGE_HEADER_SIZE))
        {
            memcpy(pre_write_info.pre_wr_buffer + pre_write_info.pre_wr_len, pu8Src, u8Length);
            pre_write_info.pre_wr_len += u8Length;

            if(0 == ExtFLASH_CheckIsCurWrPart(current_rd_partition, current_partition))
            {
                totel_len_InCurrReadPartition += u8Length;
                #ifdef FLASH_DEBUG
					printf("[FLASH] storageExtManage.totel_availabe_length = %d\r\n",
					        storageExtManage.totel_availabe_length);
                    printf("[FLASH] storageIntManage.u32RdNum_CurrPartition = %d\r\n",
                            storageIntManage.u32RdNum_CurrPartition);
                #endif

                ExtFLASH_PreReadInWrCurrPage();
            }
        }
        else
        {
            if(0 == ExtFLASH_CheckIsCurWrPart(current_rd_partition, current_partition))
			{
				totel_len_InCurrReadPartition += u8Length;
			}

            if((pre_write_info.pre_wr_len + u8Length) == (PAGE_SIZE - PAGE_HEADER_SIZE))
            {
                memcpy(pre_write_info_backup.pre_wr_buffer,
                        pre_write_info.pre_wr_buffer,
                        pre_write_info.pre_wr_len);
                memcpy(pre_write_info_backup.pre_wr_buffer + pre_write_info.pre_wr_len,
                        pu8Src,
                        u8Length);
                pre_write_info_backup.pre_wr_len = PAGE_SIZE - PAGE_HEADER_SIZE;
                pre_write_info.pre_wr_len = 0;
            }
            else
            {
                memcpy(pre_write_info_backup.pre_wr_buffer,
                        pre_write_info.pre_wr_buffer,
                        pre_write_info.pre_wr_len);
                pre_write_info_backup.pre_wr_len = pre_write_info.pre_wr_len;

                memcpy(pre_write_info.pre_wr_buffer, pu8Src, u8Length);
                pre_write_info.pre_wr_len = u8Length;
            }

            if(FLASH_ACCESS_ENABLE == GetExtFlashAccessState())
            {
                DisableExtFlashAccess();
                u8Last_write_send = 0;
                TS_SendEvent(gTsStorageTaskID_c,gStorageIntWrite);
            }
            else
            {
                u8Last_write_send = 1;
            }
        }
    }
	else
	{
		#ifdef FLASH_DEBUG
			printf("\r\n[FLASH] There is no valid partition.\r\n");
		#endif

		state = FLASH_OPERATION_FAIL;
	}

    if(1 == u8Last_write_send)
    {
        if(FLASH_ACCESS_ENABLE == GetExtFlashAccessState())
        {
            u8Last_write_send = 0;
            TS_SendEvent(gTsStorageTaskID_c,gStorageIntWrite);
        }
    }

	#ifdef FLASH_DEBUG
		printf("	[FLASH] totel_len_InCurrReadPartition = %d\r\n",
                        totel_len_InCurrReadPartition);
		printf("	[FLASH] storageExtManage.totel_availabe_length = 0x%08x\r\n",
                        storageExtManage.totel_availabe_length);
		printf("	[FLASH] pre_write_info.pre_wr_len = 0x%08x\r\n",
                        pre_write_info.pre_wr_len);
	#endif

	return (state);
}

ReturnMsg ExtFLASH_FlashPageWrite(void)
{
	ReturnMsg state = FlashOperationSuccess;
	uint32_t current_page_start_addr = 0;

	/* Check if the current data should be writed to next page? */
	if(ExtFLASH_IsPageBoundary(storageIntManage.u32WrPointer) == false)
	{
		current_page_start_addr = ExtFLASH_ComputeCurPageStartAddr(storageIntManage.u32WrPointer);
		storageIntManage.u32WrPointer = current_page_start_addr;
	}

    /* Page Write */
    ExtFLASH_RealFlashPageWrite(storageIntManage.u32WrPointer,
                                current_partition.start_time,
                                storageIntManage.u32WrNum_CurrPartition,
                                pre_write_info_backup.pre_wr_buffer,
                                pre_write_info_backup.pre_wr_len);

	storageIntManage.u32WrPointer += PAGE_SIZE;
	storageIntManage.u32WrPointer %= FLASH_HEADER_SECTOR_BASE_ADDR;

	#ifdef FLASH_DEBUG
		printf("[FLASH] End internal write...\r\n");
		printf("After CMD_PageProgram:\r\n");
		printf("	[FLASH] storageExtManage.totel_availabe_length = 0x%08x\r\n",storageExtManage.totel_availabe_length);
		printf("	[FLASH] New storageIntManage.u32WrPointer = 0x%08x\r\n",storageIntManage.u32WrPointer);
		CheckWrite((storageIntManage.u32WrPointer - PAGE_SIZE),PAGE_SIZE);
	#endif

	return state;
}

/*********************************************************************************
  * Real page write to flash
  * @param  flash_addr: the page addr to write
  * @param  start_time: start time
  * @param  totel_len: the totel length
  * @param  pu8Src: the real source to write
  * @param  totel_len: the totel length
  * @param  len_tmp: the length of real source to write
  * @retval flashOperationMsg
**********************************************************************************/
ReturnMsg ExtFLASH_RealFlashPageWrite(uint32_t flash_addr, uint32_t start_time, uint32_t totel_len,
                                      uint8_t * pu8Src, uint8_t len_tmp)
{
	ReturnMsg state = FlashOperationSuccess;
	uint8_t temp[4] = {0};

    /* Check parameters */
    usr_para_check(pu8Src != NULL);

    #ifdef FLASH_DEBUG
		printf("[FLASH] ExtFLASH_RealFlashPageWrite,0x%08x\r\n",flash_addr);
    #endif

    IntDisable();

	/* First, program the start time in current page */
	*(temp + 0) = (uint8_t)(start_time);
	*(temp + 1) = (uint8_t)(start_time >> 8);
	*(temp + 2) = (uint8_t)(start_time >> 16);
	*(temp + 3) = (uint8_t)(start_time >> 24);
	CMD_PageProgram(flash_addr, temp, sizeof(temp));

	/* Second, program the availabe data length  into  current page  */
	*(temp + 0) = (uint8_t)(totel_len);
	*(temp + 1) = (uint8_t)(totel_len >> 8);
	*(temp + 2) = (uint8_t)(totel_len >> 16);
	*(temp + 3) = (uint8_t)(totel_len >> 24);
	CMD_PageProgram((flash_addr + TOTEL_LENGTH_AREA_OFFSET), temp, sizeof(temp));

	/* Third, program the availabe data  into  current page*/
	CMD_PageProgram((flash_addr + PAGE_HEADER_OFFSET), pu8Src, len_tmp);

    IntEnable();

	return (state);
}

/*********************************************************************************
  * The entry to internal write
  * @param  None
  * @retval ReturnMsg
**********************************************************************************/
ReturnMsg ExtFLASH_IntWrite(void)
{
	ReturnMsg state = FlashOperationSuccess;
    uint32_t len_delete = 0;

	#ifdef FLASH_DEBUG
		printf("[FLASH] ExtFLASH_IntWrite\r\n");
	#endif

	storageIntManage.u32WrNum_CurrPartition += pre_write_info_backup.pre_wr_len;
	current_partition.end_page_addr = storageIntManage.u32WrPointer / PAGE_SIZE;

	#ifdef FLASH_DEBUG
		printf("	[FLASH] storageIntManage.u32WrNum_CurrPartition = %d\r\n",storageIntManage.u32WrNum_CurrPartition);
        printf("	[FLASH] current_partition.end_page_addr = 0x%04x\r\n",current_partition.end_page_addr);
	#endif
	/* Check if the current data should be writed to next sector? */
	if(ExtFLASH_IsSectionBoundary(storageIntManage.u32WrPointer) == false)
	{
        #ifdef FLASH_DEBUG
			printf("	[FLASH] No need to erase the next sector\r\n");
		#endif

		ExtFLASH_FlashPageWrite();
        if(0 == ExtFLASH_CheckIsCurWrPart(current_rd_partition, current_partition))
		{
			ExtFLASH_PreReadInit();
		}
        EnableExtFlashAccess();
	}
	else
	{
		#ifdef FLASH_DEBUG
			printf("	[FLASH] Need to erase the next sector\r\n");
		#endif

        if(ExtFlash_CheckNeedDeletePartition(storageIntManage.u32WrPointer) == TRUE)
        {
            #ifdef FLASH_DEBUG
    			printf("	[FLASH] Need to delete a partition\r\n");
    		#endif

            len_delete = ExtFLASH_GetLengthInDeletePartition();
            #ifdef FLASH_DEBUG
    			printf("	[FLASH] len_delete = %d\r\n",len_delete);
    		#endif
            storageExtManage.totel_availabe_length
                = (storageExtManage.totel_availabe_length >= len_delete)
                                        ? (storageExtManage.totel_availabe_length - len_delete) : 0;
            CheckTotelLenErr(storageExtManage.totel_availabe_length,6);
            storageIntManage.u32RdNum_CurrPartition = 0;

            need_delete_partition = 1;
            /* Second,erase the next sector */
    		sector_erase_info.sector_erase_process = ERASE_BACKUP_AREA;
    		sector_erase_info.erase_page_addr = 0;
    		sector_erase_info.pEraseCallBack = EraseBackupAreaInWrite_CallBackHandler;
    		TS_SendEvent(gTsStorageTaskID_c,gStorageEraseSector);
        }
        else
        {
            #ifdef FLASH_DEBUG
    			printf("	[FLASH] No need to delete a partition\r\n");
    		#endif

            need_delete_partition = 0;
    		/* Second,erase the next sector */
    		sector_erase_info.sector_erase_process = ERASE_NEXT_AREA;
    		sector_erase_info.erase_page_addr = storageIntManage.u32WrPointer;
    		sector_erase_info.pEraseCallBack = EraseNextAreaInWrite_CallBackHandler;
    		TS_SendEvent(gTsStorageTaskID_c,gStorageEraseSector);
        }
	}

	return state;
}

/*********************************************************************************
  * Compute the length of valid data in current page
  * @param  flash_addr: flash addr
  * @param  pu32Len: the length of valid data
  * @retval flashOperationMsg
**********************************************************************************/
flashOperationMsg ExtFLASH_GetValidLengthInCurrPage(uint32_t flash_addr, uint32_t * pu32Len)
{
    flashOperationMsg state = FLASH_OPERATION_SUCCESS;
	uint32_t current_page_start_addr = 0;
	uint32_t current_page_len = 0;
    uint32_t temp_addr = 0;
    uint8_t u8Buffer[8] = {0},u8Buffer_pre[8] = {0};
    uint32_or_uint8_u * p_start_time = (uint32_or_uint8_u *)(u8Buffer + START_TIME_AREA_OFFSET);
    uint32_or_uint8_u * p_length = (uint32_or_uint8_u *)(u8Buffer + TOTEL_LENGTH_AREA_OFFSET);

    uint32_or_uint8_u * p_start_time_pre
        = (uint32_or_uint8_u *)(u8Buffer_pre + START_TIME_AREA_OFFSET);
    uint32_or_uint8_u * p_length_pre
        = (uint32_or_uint8_u *)(u8Buffer_pre + TOTEL_LENGTH_AREA_OFFSET);

	// Check parameters
	usr_para_check(flash_addr < FLASH_HEADER_SECTOR_BASE_ADDR);

	#ifdef FLASH_DEBUG
		printf("[FLASH] ExtFLASH_GetValidLengthInCurrPage,flash_addr = 0x%08x\r\n",flash_addr);
	#endif

	current_page_start_addr = ExtFLASH_ComputeCurPageStartAddr(flash_addr);
    temp_addr = (current_page_start_addr >= PAGE_SIZE)
                ? (current_page_start_addr - PAGE_SIZE)
                : ((current_page_start_addr + FLASH_HEADER_SECTOR_BASE_ADDR) - PAGE_SIZE);

	CMD_READ(current_page_start_addr, u8Buffer, PAGE_HEADER_SIZE);
    CMD_READ(temp_addr, u8Buffer_pre, PAGE_HEADER_SIZE);

    #ifdef FLASH_DEBUG
		printf("	[FLASH] p_start_time->l = 0x%08x\r\n",p_start_time->l);
		printf("	[FLASH] p_start_time_pre->l = 0x%08x\r\n",p_start_time_pre->l);
        printf("	[FLASH] p_length->l = 0x%08x\r\n",p_length->l);
		printf("	[FLASH] p_length_pre->l = 0x%08x\r\n",p_length_pre->l);
	#endif

    if((p_start_time_pre->l != 0xFFFFFFFF) && (p_start_time->l != 0xFFFFFFFF))
    {
        if((p_start_time->l == p_start_time_pre->l) && (p_length->l >= p_length_pre->l))
        {
            #ifdef FLASH_DEBUG
        		printf("	[FLASH] p_length->l = %d\r\n",p_length->l);
        		printf("	[FLASH] p_length_pre->l = %d\r\n",p_length_pre->l);
        	#endif
            current_page_len = p_length->l - p_length_pre->l;
            state = FLASH_OPERATION_SUCCESS;
        }
        else
        {
            current_page_len = p_length->l;
            state = FLASH_OPERATION_SUCCESS;
        }
    }
    else
    {
        if(p_start_time_pre->l == 0xFFFFFFFF)
        {
            current_page_len = p_length->l;
            state = FLASH_OPERATION_SUCCESS;
        }

        if(p_start_time->l == 0xFFFFFFFF)
        {
            current_page_len = 0;
            state = FLASH_OPERATION_FAIL;
        }
    }

    if(pu32Len != NULL)
    {
        *pu32Len = current_page_len;
    }

    #ifdef FLASH_DEBUG
        printf("[FLASH] ExtFLASH_GetValidLengthInCurrPage,valid length = %d\r\n",current_page_len);
	#endif

	return (state);
}

/*********************************************************************************
  * Compute the length of invalid data in current page
  * @param  flash_addr: flash addr
  * @param  pu32Len: the length of valid data
  * @retval flashOperationMsg
**********************************************************************************/
uint32_t ExtFLASH_GetInvalidLengthInCurrPage(uint32_t flash_addr)
{
	uint32_t current_page_len = 0;
    uint32_t data_len = 0;

	/* Check parameters */
	usr_para_check(flash_addr < FLASH_HEADER_SECTOR_BASE_ADDR);

	#ifdef FLASH_DEBUG
		printf("[FLASH] ExtFLASH_GetInvalidLengthInCurrPage,flash_addr = 0x%08x\r\n",flash_addr);
	#endif

    if(FLASH_OPERATION_SUCCESS == ExtFLASH_GetValidLengthInCurrPage(flash_addr, &data_len))
    {
        current_page_len = data_len;
        return ((PAGE_SIZE - PAGE_HEADER_SIZE) - current_page_len);
    }
    else
    {
        return (0);
    }
}

uint32_t ExtFLASH_GetLengthInDeletePartition(void)
{
	uint8_t partition_tmp[8] = {0};
    uint8_t page_tmp[8] = {0};
	flash_partition_t * p_flash_partition = (flash_partition_t *)partition_tmp;
    uint32_or_uint8_u * p_start_time = (uint32_or_uint8_u *)(page_tmp + START_TIME_AREA_OFFSET);
    uint32_or_uint8_u * p_length = (uint32_or_uint8_u *)(page_tmp + TOTEL_LENGTH_AREA_OFFSET);
	uint32_t tmp_end_addr = 0;
    uint32_t tmp_len = 0;

    #ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_GetLengthInDeletePartition\r\n");
	#endif

	CMD_READ(FLASH_HEADER_SECTOR_BASE_ADDR, partition_tmp, sizeof(flash_partition_t));
    #ifdef FLASH_DEBUG
        printf("	[FLASH] p_flash_partition.start_time		= 0x%08x\r\n",p_flash_partition->start_time);
		printf("	[FLASH] p_flash_partition.start_page_addr	= 0x%04x\r\n",p_flash_partition->start_page_addr);
		printf("	[FLASH] p_flash_partition.end_page_addr 	= 0x%04x\r\n",p_flash_partition->end_page_addr);
    #endif

    tmp_end_addr = (p_flash_partition->end_page_addr << 8);
    CMD_READ(tmp_end_addr, page_tmp, sizeof(page_tmp));

    if((p_start_time->l == p_flash_partition->start_time) &&
        (p_length->l <= FLASH_HEADER_SECTOR_BASE_ADDR))
    {
        tmp_len = p_length->l;
        if(0 == ExtFLASH_CheckIsSamePart(&current_rd_partition, p_flash_partition))
        {
            tmp_len = (tmp_len > storageIntManage.u32RdNum_CurrPartition) ?
                        (tmp_len - storageIntManage.u32RdNum_CurrPartition) : 0;
        }
    }
    else
    {
        tmp_len = 0;
    }

	return (tmp_len);
}

uint32_t ExtFLASH_ReadOKNumInCurrPage(uint32_t flash_addr)
{
	uint32_t current_page_start_addr;
	uint32_t temp = 0;

	/* Check parameters */
	usr_para_check(flash_addr < FLASH_HEADER_SECTOR_BASE_ADDR);

	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_ReadOKNumInCurrPage,flash_addr = 0x%08x\r\n",flash_addr);
	#endif

	current_page_start_addr = ExtFLASH_ComputeCurPageStartAddr(flash_addr);
	temp = flash_addr - (current_page_start_addr + PAGE_HEADER_OFFSET);
    temp = (flash_addr > (current_page_start_addr + PAGE_HEADER_OFFSET))
                            ? (flash_addr - (current_page_start_addr + PAGE_HEADER_OFFSET)) : 0;

	#ifdef FLASH_DEBUG
		printf("	[FLASH] temp = %d\r\n",temp);
	#endif

	return (temp);
}

flashOperationMsg ExtFLASH_ExtRead(store_type_e type,storageExtBuffer_t * p_storageExtBuffer,uint8_t u8Length)
{
	flashOperationMsg state = FLASH_OPERATION_SUCCESS;

	/* Check parameters */
	usr_para_check((type == NON_MONITOR_TYPE)||(type == MONITOR_TYPE));
	usr_para_check(p_storageExtBuffer != NULL);

	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_ExtRead\r\n");
		printf("	[FLASH] storageExtManage.totel_availabe_length = %d\r\n",
                    storageExtManage.totel_availabe_length);
        printf("	[FLASH] u8Length = %d\r\n",u8Length);
    #endif
	if(FLASH_ACCESS_ENABLE == GetExtFlashAccessState())
	{
		p_storageExtBuffer->start_time = current_rd_partition.start_time;
		if(u8Length <= pre_read_info.pre_rd_len)
		{
			p_storageExtBuffer->availabe_length = u8Length;
			last_read_len = u8Length;
			memcpy(p_storageExtBuffer->Buffer, pre_read_info.pre_rd_buffer, u8Length);
		}
		else
		{
			p_storageExtBuffer->availabe_length = pre_read_info.pre_rd_len;
			last_read_len = pre_read_info.pre_rd_len;
			memcpy(p_storageExtBuffer->Buffer, &pre_read_info.pre_rd_buffer, pre_read_info.pre_rd_len);
		}

        #ifdef FLASH_DEBUG
			printf("	[FLASH] storageExtBuffer->availabe_length = %d\r\n",p_storageExtBuffer->availabe_length);
        #endif
		state = FLASH_OPERATION_SUCCESS;
	}
	else
	{
		state = FLASH_OPERATION_FAIL;
	}

	return state;
}

flashOperationMsg ExtFLASH_ExtReadACK(uint8_t u8ValidLength)
{
	flashOperationMsg state = FLASH_OPERATION_SUCCESS;

	/* Check parameters */
	usr_para_check(u8ValidLength > 0);

	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_ExtReadACK,u8ValidLength = %d\r\n",u8ValidLength);
	#endif

	if(u8ValidLength <= last_read_len)
	{
        DisableExtFlashAccess();
		last_read_len = u8ValidLength;
        memset(pre_read_info.pre_rd_buffer,0,PAGE_SIZE);
        pre_read_info.pre_rd_len = 0;
		state = FLASH_OPERATION_SUCCESS;

		/* Send the event to pre_read data */
		TS_SendEvent(gTsStorageTaskID_c,gStorageReadACK);
	}
	else
	{
		state = FLASH_OPERATION_FAIL;
	}

	return (state);
}

ReturnMsg ExtFLASH_PreRead(uint8_t * pu8Dst, uint16_t u8Length,uint32_t flash_rd_start_addr)
{
	ReturnMsg state = FlashOperationSuccess;
	uint32_t current_page_valid_length = 0;
	uint32_t next_page_valid_length = 0;
	uint32_t i = 0;
	uint32_t read_out_len = 0;
	uint32_t read_completed = 0;
	uint32_t page_read_start_addr = 0;
	uint32_t remain_bytes_inCurrPage = 0;
	uint32_t temp_flash_rd_start_addr = 0;
    uint32_t datalen = 0;

	/* Check parameters */
	usr_para_check(pu8Dst != NULL);

	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_PreRead,u8Length = %d\r\n",u8Length);
	#endif

	if(u8Length > 0)
	{
		/* First,get the totel length in last page */
        ExtFLASH_GetValidLengthInCurrPage(flash_rd_start_addr, &datalen);
		current_page_valid_length = datalen;
		remain_bytes_inCurrPage = current_page_valid_length
                                            - ExtFLASH_ReadOKNumInCurrPage(flash_rd_start_addr);

		MX25_SPI_Configuration();
		if(remain_bytes_inCurrPage >= u8Length)
		{
			CMD_READ(flash_rd_start_addr, pu8Dst, u8Length);
		}
		else
		{
			CMD_READ(flash_rd_start_addr, pu8Dst, remain_bytes_inCurrPage);
			read_out_len += remain_bytes_inCurrPage;

			do{
				i++;
				temp_flash_rd_start_addr = flash_rd_start_addr + (i * PAGE_SIZE);
				temp_flash_rd_start_addr %= FLASH_HEADER_SECTOR_BASE_ADDR;
                ExtFLASH_GetValidLengthInCurrPage(temp_flash_rd_start_addr, &datalen);
				next_page_valid_length = datalen;

                /* Check if the current read partiion is the current write partition? */
                if(0 == ExtFLASH_CheckIsSamePart(&current_rd_partition, &current_partition))
                {
                    if(0 != ExtFLASH_CheckInCurPart(current_partition, temp_flash_rd_start_addr))
                    {
                        next_page_valid_length = 0;
                    }
                }

                #ifdef FLASH_DEBUG
					printf("	[FLASH] next_page_valid_length = %d\r\n",next_page_valid_length);
				#endif
                if(next_page_valid_length == 0)
                {
                    /* Read from RAM */
                    if((read_out_len + pre_write_info.pre_wr_len) <= u8Length)
                    {
                        memcpy(pre_read_info.pre_rd_buffer + read_out_len,
                                            pre_write_info.pre_wr_buffer,pre_write_info.pre_wr_len);
                        read_out_len += pre_write_info.pre_wr_len;
                    }
                    else
                    {
                        memcpy(pre_read_info.pre_rd_buffer + read_out_len,
                                            pre_write_info.pre_wr_buffer,(u8Length - read_out_len));
                        read_out_len += (u8Length - read_out_len);
                    }
                    read_completed = 1;
                }
                else
                {
                    /* Check if the sum of data read out form current page and next page is enough */
                    if((read_out_len + next_page_valid_length) >= u8Length)
                    {
                        page_read_start_addr
                                    = ExtFLASH_ComputeCurPageStartAddr(temp_flash_rd_start_addr);
                        CMD_READ(page_read_start_addr + PAGE_HEADER_OFFSET, pu8Dst + read_out_len,
                                 u8Length - read_out_len);
                        read_completed = 1;
                    }
                    else
                    {
                        page_read_start_addr
                                    = ExtFLASH_ComputeCurPageStartAddr(temp_flash_rd_start_addr);
                        if(next_page_valid_length <= (PAGE_SIZE - PAGE_HEADER_OFFSET))
                        {
                            CMD_READ(page_read_start_addr + PAGE_HEADER_OFFSET,
                                    pu8Dst + read_out_len, next_page_valid_length);
                            read_out_len += next_page_valid_length;
                        }
                    }
                }
			}while(read_completed == 0);
		}
	}

    #ifdef FLASH_DEBUG
        printf("    [FLASH] ExtFLASH_PreRead,END\r\n");
	#endif
	return state;
}

ReturnMsg ExtFLASH_PreReadInWrCurrPage(void)
{
	ReturnMsg state = FlashOperationSuccess;
    uint32_t tmp_index = 0;

	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_PreReadInWrCurrPage\r\n");
	#endif

    if((storageExtManage.totel_availabe_length) < (PAGE_SIZE - PAGE_HEADER_SIZE))
    {
        #ifdef FLASH_DEBUG
    		printf("    [FLASH] storageIntManage.u32RdPointer = 0x%08x\r\n",storageIntManage.u32RdPointer);
            printf("    [FLASH] storageIntManage.u32WrPointer = 0x%08x\r\n",storageIntManage.u32WrPointer);
    	#endif

        if(storageIntManage.u32RdPointer < (storageIntManage.u32WrPointer + PAGE_HEADER_SIZE))
        {
            #ifdef FLASH_DEBUG
        		printf("[FLASH] storageIntManage.u32RdPointer < (storageIntManage.u32WrPointer + PAGE_HEADER_SIZE)\r\n");
        	#endif

            ExtFLASH_PreRead(pre_read_info.pre_rd_buffer,
                            storageExtManage.totel_availabe_length,
                            storageIntManage.u32RdPointer);
		    pre_read_info.pre_rd_len = storageExtManage.totel_availabe_length;
        }
        else
        {
            if(storageIntManage.u32RdPointer < (storageIntManage.u32WrPointer + PAGE_SIZE))
            {
                #ifdef FLASH_DEBUG
            		printf("[FLASH] storageIntManage.u32RdPointer < (storageIntManage.u32WrPointer + PAGE_SIZE)\r\n");
            	#endif
                tmp_index = storageIntManage.u32RdPointer
                            - (storageIntManage.u32WrPointer + PAGE_HEADER_SIZE);
                memcpy(pre_read_info.pre_rd_buffer, (pre_write_info.pre_wr_buffer + tmp_index),
                       storageExtManage.totel_availabe_length);
                pre_read_info.pre_rd_len = storageExtManage.totel_availabe_length;

                #ifdef FLASH_DEBUG
            		printf("[FLASH] Data has been pre-read complete,pre_read_info.pre_rd_len = %d\r\n",
            		                                                pre_read_info.pre_rd_len);
                    CheckPreReadArea(pre_read_info.pre_rd_len);
            	#endif
            }
            else
            {
                #ifdef FLASH_DEBUG
            		printf("[FLASH] storageIntManage.u32RdPointer >= (storageIntManage.u32WrPointer + PAGE_SIZE)\r\n");
            	#endif
                ExtFLASH_PreRead(pre_read_info.pre_rd_buffer, PAGE_SIZE,
                                storageIntManage.u32RdPointer);
                pre_read_info.pre_rd_len = PAGE_SIZE;
            }
        }
    }
    else
    {
        #ifdef FLASH_DEBUG
    		printf("[FLASH] (storageExtManage.totel_availabe_length) >= (PAGE_SIZE - PAGE_HEADER_SIZE)\r\n");
    	#endif
        if(storageExtManage.totel_availabe_length < PAGE_SIZE)
        {
            ExtFLASH_PreRead(pre_read_info.pre_rd_buffer,
                            storageExtManage.totel_availabe_length,
                            storageIntManage.u32RdPointer);
            pre_read_info.pre_rd_len = storageExtManage.totel_availabe_length;
        }
        else
        {
            ExtFLASH_PreRead(pre_read_info.pre_rd_buffer, PAGE_SIZE, storageIntManage.u32RdPointer);
            pre_read_info.pre_rd_len = PAGE_SIZE;
        }
    }

	#ifdef FLASH_DEBUG
		printf("[FLASH] ExtFLASH_PreReadInCurrPage,END\r\n");
	#endif
	return (state);
}

ReturnMsg ExtFLASH_PreReadCurrPageRemain(uint8_t * pu8Dst,
                                        uint32_t *pu8Length,
                                        uint32_t flash_rd_start_addr)
{
	ReturnMsg state = FlashOperationSuccess;
	uint32_t current_page_valid_length = 0;
	uint32_t remain_bytes_inCurrPage = 0;
    uint32_t datalen = 0;

	/* Check parameters */
	usr_para_check(pu8Dst != NULL);

	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_PreReadCurrPageRemain\r\n");
	#endif

	MX25_SPI_Configuration();

	/* First,get the totel length in last page */
    ExtFLASH_GetValidLengthInCurrPage(flash_rd_start_addr, &datalen);
	current_page_valid_length = datalen;
	#ifdef FLASH_DEBUG
		printf("	[FLASH] current_page_valid_length = 0x%08x\r\n",current_page_valid_length);
	#endif
	remain_bytes_inCurrPage = current_page_valid_length -
								ExtFLASH_ReadOKNumInCurrPage(flash_rd_start_addr);

	#ifdef FLASH_DEBUG
		printf("	[FLASH] remain_bytes_inCurrPage = 0x%08x\r\n",remain_bytes_inCurrPage);
	#endif
	CMD_READ(flash_rd_start_addr, pu8Dst, remain_bytes_inCurrPage);

	*pu8Length = remain_bytes_inCurrPage;

    #ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_PreReadCurrPageRemain,End,remain_bytes_inCurrPage = %d\r\n",remain_bytes_inCurrPage);
        CheckRead(pu8Dst, remain_bytes_inCurrPage);
	#endif
	return (state);
}

/*********************************************************************************
  * Adjust the read pointer
  * @param  flash_rd_start_addr_src: the current read pointer
  * @param  u8Length: the length
  * @param  p_flash_rd_start_addr_dst: the destion read pointer
  * @retval ReturnMsg
**********************************************************************************/
ReturnMsg ExtFLASH_ModifyRdPointer(uint32_t flash_rd_start_addr_src,
                                   uint16_t u8Length,
                                   uint32_t * p_flash_rd_start_addr_dst)
{
	ReturnMsg state = FlashOperationSuccess;
	uint32_t current_page_valid_length = 0;
	uint32_t read_completed = 0;
	uint32_t remain_bytes_inCurrPage = 0;
	uint32_t temp_flash_rd_start_addr = 0;
    uint32_t datalen = 0;
    uint32_t u32Len_tmp = 0;
    uint32_t u32StartAddr = 0;
    uint32_t flash_rd_start_addr_dst = 0;

	/* Check parameters */
	usr_para_check(p_flash_rd_start_addr_dst != NULL);

	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_ModifyRdPointer,u8Length = %d\r\n",u8Length);
	#endif

	if(u8Length > 0)
	{
        /* First,get the totel length in last page */
        ExtFLASH_GetValidLengthInCurrPage(flash_rd_start_addr_src, &datalen);
		current_page_valid_length = datalen;
		remain_bytes_inCurrPage = current_page_valid_length
                                - ExtFLASH_ReadOKNumInCurrPage(flash_rd_start_addr_src);

        #ifdef FLASH_DEBUG
            printf("    [FLASH] remain_bytes_inCurrPage = %d\r\n",remain_bytes_inCurrPage);
	    #endif

		if(remain_bytes_inCurrPage >= u8Length)
		{
            flash_rd_start_addr_dst = flash_rd_start_addr_src + u8Length;
            state = FlashOperationSuccess;
		}
        else
        {
            u32Len_tmp = u8Length - remain_bytes_inCurrPage;
            remain_bytes_inCurrPage = 0;
            temp_flash_rd_start_addr = ExtFLASH_ComputeNextPageStartAddr(flash_rd_start_addr_src);

            #ifdef FLASH_DEBUG
                printf("    [FLASH] u32Len_tmp = %d\r\n",u32Len_tmp);
    	    #endif

            do {
                if(FLASH_OPERATION_SUCCESS ==
                  ExtFLASH_GetValidLengthInCurrPage(temp_flash_rd_start_addr + PAGE_HEADER_OFFSET,&datalen))
                {
            		if(u32Len_tmp <= datalen)
            		{
                        flash_rd_start_addr_dst
                            = temp_flash_rd_start_addr + PAGE_HEADER_OFFSET + u32Len_tmp;

                        #ifdef FLASH_DEBUG
                            printf("    [FLASH] flash_rd_start_addr_dst = 0x%08x\r\n",flash_rd_start_addr_dst);
                	    #endif
                        read_completed = 1;
                        state = FlashOperationSuccess;
            		}
            		else
            		{
                        u32Len_tmp -= datalen;
                        temp_flash_rd_start_addr += PAGE_SIZE;
                        temp_flash_rd_start_addr %= FLASH_HEADER_SECTOR_BASE_ADDR;
                        temp_flash_rd_start_addr
                            = ExtFLASH_ComputeCurPageStartAddr(temp_flash_rd_start_addr + PAGE_HEADER_OFFSET);
            		}
                }
                else
                {
                    flash_rd_start_addr_dst = temp_flash_rd_start_addr;
                    read_completed = 1;
                    state = FlashOperationSuccess;
                }
            }while(read_completed == 0);
        }
	}

    u32StartAddr = ExtFLASH_ComputeCurPageStartAddr(flash_rd_start_addr_dst);
    if((u32StartAddr <= flash_rd_start_addr_dst)
   && (flash_rd_start_addr_dst < (u32StartAddr + PAGE_HEADER_OFFSET)))
    {
        flash_rd_start_addr_dst = u32StartAddr + PAGE_HEADER_OFFSET;
    }

    if(p_flash_rd_start_addr_dst != NULL)
    {
        *p_flash_rd_start_addr_dst = flash_rd_start_addr_dst;
    }

    #ifdef FLASH_DEBUG
        printf("    [FLASH] ExtFLASH_ModifyRdPointer,END\r\n");
	#endif

	return (state);
}

uint32_t ExtFLASH_ExtGetTotelLen(void)
{
    return (storageExtManage.totel_availabe_length);
}

int8_t ExtFLASH_StorageCheck(void)
{
    int8_t ret = 0;
    uint8_t u8TmpPartition[HEADER_PARTITION_AREA_SIZE] = {0};
    uint32_t read_addr = FLASH_HEADER_SECTOR_BASE_ADDR;
    uint32_t u32TmpTotelLen = 0;
    flash_partition_t * p_flash_partition = NULL;
    bool bSearchEnd = FALSE;

    #ifdef FLASH_DEBUG
        printf("\r\n[FLASH] ExtFLASH_StorageCheck,storageExtManage.totel_availabe_length = %d\r\n",storageExtManage.totel_availabe_length);
	#endif

    CheckTotelLenErr(storageExtManage.totel_availabe_length,9);

    DisableExtFlashAccess();
    if(storageExtManage.available_partition > 0)
    {
        if(0 == ExtFLASH_CheckIsCurWrPart(current_rd_partition, current_partition))
        {
            u32TmpTotelLen = storageIntManage.u32WrNum_CurrPartition + pre_write_info.pre_wr_len;
        }
        else
        {
            p_flash_partition = (flash_partition_t *)u8TmpPartition;
            u32TmpTotelLen = 0;
            MX25_SPI_Configuration();

	        do{
		        #ifdef FLASH_DEBUG
			        printf("[FLASH] Now Reading the addr: 0x%08x\r\n",read_addr);
		        #endif

		        CMD_READ(read_addr, u8TmpPartition, sizeof(flash_partition_t));
		        #ifdef FLASH_DEBUG
                    printf("	[FLASH] Start Time: %02d:%02d:%02d\r\n",((u8TmpPartition[3] << 8) + u8TmpPartition[2]),u8TmpPartition[1],u8TmpPartition[0]);
                    printf("	[FLASH] start_page_addr: 0x%04x\r\n",p_flash_partition->start_page_addr);
			        printf("	[FLASH] end_page_addr : 0x%04x\r\n",p_flash_partition->end_page_addr);
		        #endif
		        if(true == ExtFLASH_IsPartitionValid(p_flash_partition))
		        {
			        u32TmpTotelLen += ExtFLASH_ComputeTotelLengthInCurrPartition(p_flash_partition);
                    read_addr += sizeof(flash_partition_t);
		        }
		        else
		        {
			        bSearchEnd = TRUE;
		        }
	        }while(bSearchEnd == FALSE);
        }

        /* Substract storageIntManage.u32RdNum_CurrPartition */
        u32TmpTotelLen = (u32TmpTotelLen > storageIntManage.u32RdNum_CurrPartition) ?
                        (u32TmpTotelLen - storageIntManage.u32RdNum_CurrPartition) :
                        0;

        /* Substract storageIntManage.u32RdNum_CurrPartition */
        storageExtManage.totel_availabe_length = u32TmpTotelLen;

        CheckTotelLenErr(storageExtManage.totel_availabe_length,7);
    }
    else
    {
        storageExtManage.totel_availabe_length = 0;
    }

    EnableExtFlashAccess();

    #ifdef FLASH_DEBUG
        printf("    [FLASH] ExtFLASH_StorageCheck,End,storageExtManage.totel_availabe_length = 0x%08x\r\n",
                                        storageExtManage.totel_availabe_length);
	#endif

    return (ret);
}

ReturnMsg ExtFLASH_IntReadACK(void)
{
	ReturnMsg state = FlashOperationSuccess;
	uint32_t tmp_len = 0;
    uint32_t tmp_rd_pointer = 0;
	uint32_t current_page_start_addr = 0;
	uint32_t last_page_addr = 0;
	uint32_t next_page_addr = 0;
	uint32_t aboard_data_length_curr_page = 0;

	#ifdef FLASH_DEBUG
		printf("\r\n[FLASH] ExtFLASH_IntReadACK\r\n");
        printf("	[FLASH] storageIntManage.u32RdPointer = 0x%08x\r\n",storageIntManage.u32RdPointer);
        printf("	[FLASH] storageExtManage.available_partition = 0x%08x\r\n",storageExtManage.available_partition);
	#endif

	DisableExtFlashAccess();
    storageExtManage.totel_availabe_length = (storageExtManage.totel_availabe_length >= last_read_len)
                                       ? (storageExtManage.totel_availabe_length - last_read_len) : 0;
    CheckTotelLenErr(storageExtManage.totel_availabe_length,8);

	#ifdef FLASH_DEBUG
		printf("	[FLASH] last_read_len = 0x%08x\r\n",last_read_len);
		printf("	[FLASH] storageExtManage.totel_availabe_length = 0x%08x\r\n",storageExtManage.totel_availabe_length);
		printf("	[FLASH] storageIntManage.u32RdNum_CurrPartition = %d\r\n",storageIntManage.u32RdNum_CurrPartition);
		printf("	[FLASH] storageIntManage.u32RdPointer = 0x%08x\r\n\r\n",storageIntManage.u32RdPointer);
        printf("	[FLASH] totel_len_InCurrReadPartition = %d\r\n",totel_len_InCurrReadPartition);
	#endif
	/* Check if current read partition is current write partition? */
    if(-1 == ExtFLASH_CheckIsCurWrPart(current_rd_partition, current_partition))
	{
		#ifdef FLASH_DEBUG
			printf("	[FLASH] current_rd_partition.start_time != current_partition.start_time\r\n");
            printf("    [FLASH] RD partition,start_time: %04d:%02d:%02d\r\n",(uint16_t)(current_rd_partition.start_time >> 16),
                                                                         (uint8_t)(current_rd_partition.start_time >> 8),
                                                                         (uint8_t)(current_rd_partition.start_time));
			printf("	[FLASH] current_rd_partition.start_page_addr = 0x%04x\r\n",current_rd_partition.start_page_addr);
			printf("	[FLASH] current_rd_partition.end_page_addr = 0x%04x\r\n",current_rd_partition.end_page_addr);
		#endif

        ExtFLASH_ModifyRdPointer(storageIntManage.u32RdPointer, last_read_len, &tmp_rd_pointer);
        storageIntManage.u32RdPointer = tmp_rd_pointer;
        #ifdef FLASH_DEBUG
		    printf("	[FLASH] storageIntManage.u32RdPointer = 0x%08x\r\n\r\n",storageIntManage.u32RdPointer);
		#endif

        // Add to the storageIntManage.u32RdNum_CurrPartition
    	storageIntManage.u32RdNum_CurrPartition += last_read_len;
        last_read_len = 0;

		/* If the data has been read out empty,delete this partition */
		if(storageIntManage.u32RdNum_CurrPartition >= totel_len_InCurrReadPartition)
		{
			/* First,erase the backup area */
			/* Seond,backup the header area */
			/* Third,erase the header area */
			/* Forth,delete the partition in header area */
			storageIntManage.u32RdNum_CurrPartition = 0;
			sector_erase_info.sector_erase_process = ERASE_BACKUP_AREA;
			sector_erase_info.erase_page_addr = 0;
			sector_erase_info.pEraseCallBack = EraseBackupAreaInReadACK_CallBackHandler;
			TS_SendEvent(gTsStorageTaskID_c,gStorageEraseSector);
		}
		else
		{
			/* Pre-read data to pre-buffer */
			if((storageIntManage.u32RdNum_CurrPartition + PAGE_SIZE) <= totel_len_InCurrReadPartition)
			{
				pre_read_info.pre_rd_len = PAGE_SIZE;
				memset(pre_read_info.pre_rd_buffer, 0, sizeof(pre_read_info.pre_rd_buffer));
				ExtFLASH_PreRead(pre_read_info.pre_rd_buffer, PAGE_SIZE, storageIntManage.u32RdPointer);
			}
			else
			{
				pre_read_info.pre_rd_len
                    = totel_len_InCurrReadPartition - storageIntManage.u32RdNum_CurrPartition;
				memset(pre_read_info.pre_rd_buffer, 0, sizeof(pre_read_info.pre_rd_buffer));
				ExtFLASH_PreRead(pre_read_info.pre_rd_buffer, pre_read_info.pre_rd_len,
                                storageIntManage.u32RdPointer);
			}
			EnableExtFlashAccess();

			#ifdef FLASH_DEBUG
				//CheckPreReadArea(pre_read_info.pre_rd_len);
			#endif
		}
	}
	else
	{
		#ifdef FLASH_DEBUG
			printf("	[FLASH] current_rd_partition.start_time == current_partition.start_time\r\n");
			printf("	[FLASH] storageIntManage.u32RdPointer = 0x%08x\r\n",storageIntManage.u32RdPointer);
			printf("	[FLASH] storageIntManage.u32WrPointer = 0x%08x\r\n",storageIntManage.u32WrPointer);
			printf("	[FLASH] storageIntManage.u32RdNum_CurrPartition = 0x%08x\r\n",storageIntManage.u32RdNum_CurrPartition);
			printf("	[FLASH] totel_len_InCurrReadPartition = 0x%08x\r\n",totel_len_InCurrReadPartition);
			printf("	[FLASH] pre_write_info.pre_wr_len = 0x%08x\r\n",pre_write_info.pre_wr_len);
		#endif

        aboard_data_length_curr_page
            = ExtFLASH_GetInvalidLengthInCurrPage(storageIntManage.u32RdPointer);
        last_page_addr = storageIntManage.u32RdPointer/PAGE_SIZE;
        storageIntManage.u32RdPointer += last_read_len;
        next_page_addr = storageIntManage.u32RdPointer/PAGE_SIZE;
        if(next_page_addr > last_page_addr)
        {
            storageIntManage.u32RdPointer += (PAGE_HEADER_OFFSET + aboard_data_length_curr_page);
        }
        storageIntManage.u32RdPointer %= FLASH_HEADER_SECTOR_BASE_ADDR;
        current_page_start_addr = ExtFLASH_ComputeCurPageStartAddr(storageIntManage.u32RdPointer);
        if((storageIntManage.u32RdPointer >= current_page_start_addr)
        && (storageIntManage.u32RdPointer < (current_page_start_addr + PAGE_HEADER_OFFSET)))
        {
            storageIntManage.u32RdPointer = current_page_start_addr + PAGE_HEADER_OFFSET;
        }

        // Add to the storageIntManage.u32RdNum_CurrPartition
        storageIntManage.u32RdNum_CurrPartition += last_read_len;

        if((storageIntManage.u32RdPointer < storageIntManage.u32WrPointer)
       || ((storageIntManage.u32RdPointer > storageIntManage.u32WrPointer)
       && (storageIntManage.u32RdPointer < (storageIntManage.u32WrPointer + FLASH_HEADER_SECTOR_BASE_ADDR))))
		{
			#ifdef FLASH_DEBUG
				printf("    [FLASH] storageIntManage.u32RdPointer < storageIntManage.u32WrPointer\r\n");
			#endif

            ExtFLASH_PreReadInWrCurrPage();
		}
		else
		{
            #ifdef FLASH_DEBUG
				printf("[FLASH] storageIntManage.u32RdPointer >= storageIntManage.u32WrPointer\r\n");
			#endif
			// if there is some data to send
			if(storageExtManage.totel_availabe_length)
			{
				// Just direct pre-read form write buffer in RAM
				#ifdef FLASH_DEBUG
					printf("[FLASH] pre_write_info.pre_wr_len = 0x%08x\r\n",pre_write_info.pre_wr_len);
					printf("[FLASH] rd_index_in_ram = 0x%08x\r\n",rd_index_in_ram);
					printf("[FLASH] last_read_len = 0x%08x\r\n",last_read_len);
				#endif

				rd_index_in_ram = storageIntManage.u32RdPointer
                                            - (storageIntManage.u32WrPointer + PAGE_HEADER_OFFSET);
                #ifdef FLASH_DEBUG
					printf("[FLASH] rd_index_in_ram = 0x%02x\r\n",rd_index_in_ram);
				#endif

				tmp_len = ((pre_write_info.pre_wr_len > rd_index_in_ram)
                        ? (pre_write_info.pre_wr_len - rd_index_in_ram) : 0);
				memset(pre_read_info.pre_rd_buffer, 0, sizeof(pre_read_info.pre_rd_buffer));
				memcpy(pre_read_info.pre_rd_buffer, pre_write_info.pre_wr_buffer + rd_index_in_ram, tmp_len);

				#ifdef FLASH_DEBUG
					CheckPreReadArea(tmp_len);
				#endif
			}
			else
			{
				pre_read_info.pre_rd_len = 0;

				#ifdef FLASH_DEBUG
					printf("	[FLASH] *** No data can be read ***\r\n");
				#endif
			}
		}

        last_read_len = 0;
        EnableExtFlashAccess();
	}

	#ifdef FLASH_DEBUG
		printf("	[FLASH] totel_len_InCurrReadPartition = %d\r\n",totel_len_InCurrReadPartition);
		printf("	[FLASH] ExtFLASH_IntReadACK,End\r\n");
	#endif

	return state;
}

void Storage_Task_Handler(event_t storage_event)
{
	switch(storage_event)
	{
		case gStorageIntWrite:
			#ifdef FLASH_DEBUG
				printf("\r\n[FLASH] gStorageIntWrite\r\n");
			#endif

			MX25_SPI_Configuration();
			ExtFLASH_IntWrite();
			break;

		case gStorageReadACK:
            #ifdef FLASH_DEBUG
                printf("\r\n[FLASH] gStorageReadACK\r\n");
            #endif
			MX25_SPI_Configuration();
			ExtFLASH_IntReadACK();
            #ifdef FLASH_DEBUG
                printf("	[FLASH] gStorageReadACK,End\r\n");
            #endif
			break;

		case gStorageEraseSector:
			#ifdef FLASH_DEBUG
				printf("\r\n[FLASH] gStorageEraseSector\r\n");
			#endif
            DisableExtFlashAccess();
			MX25_SPI_Configuration();
			if(IsFlashBusy() == FALSE)
			{
				if((sector_erase_info.sector_erase_process & ERASE_BACKUP_AREA)
                                                                            == ERASE_BACKUP_AREA)
				{
					#ifdef FLASH_DEBUG
					printf("[FLASH] To erase backup area...\r\n");
					#endif
					if(sector_erase_info.pEraseCallBack != NULL)
					{
						CMD_SE(FLASH_BACKUP_SECTOR_BASE_ADDR);
                        Start_Timer_Cnt(gEraseTimerID);
						sector_erase_cnt = 0;
						sector_erase_timer_running = 1;
					}
				}
				else if((sector_erase_info.sector_erase_process & ERASE_HEADER_AREA)
                                                                            == ERASE_HEADER_AREA)
				{
					#ifdef FLASH_DEBUG
					printf("[FLASH] To erase header area...\r\n");
					#endif

					CMD_SE(FLASH_HEADER_SECTOR_BASE_ADDR);
                    Start_Timer_Cnt(gEraseTimerID);
					sector_erase_cnt = 0;
					sector_erase_timer_running = 1;
				}
				else if((sector_erase_info.sector_erase_process & ERASE_NEXT_AREA)
                                                                            == ERASE_NEXT_AREA)
				{
					#ifdef FLASH_DEBUG
					printf("[FLASH] To erase the next area...\r\n");
					#endif

					CMD_SE(sector_erase_info.erase_page_addr);
                    Start_Timer_Cnt(gEraseTimerID);
					sector_erase_cnt = 0;
					sector_erase_timer_running = 1;
				}
				else
				{
					#ifdef FLASH_DEBUG
					printf("[FLASH] No erase task\r\n");
					#endif
				}
			}
			#ifdef FLASH_DEBUG
			else
			{
				printf("[FLASH] ***FLASH is Busy***\r\n");
			}
			#endif
			break;

		case gStorageCheckIsBusy:
			#ifdef FLASH_DEBUG
				printf("\r\n[FLASH] gStorageCheckBusy, sector_erase_cnt = %d\r\n",sector_erase_cnt);
			#endif

			if(sector_erase_timer_running)
			{
				MX25_SPI_Configuration();
				if(IsFlashBusy() == FALSE)
				{
					#ifdef FLASH_DEBUG
						printf("\r\n[FLASH] Flash is IDLE\r\n");
					#endif

					sector_erase_cnt = 0;
					//Timer_Free(gEraseTimerID);
	    			//gEraseTimerID = TIMER_ERROR;
	    			Stop_Timer_Cnt(gEraseTimerID);
					#ifdef FLASH_DEBUG
						printf("\r\n[FLASH] Stop gEraseTimerID\r\n");
					#endif
					sector_erase_timer_running = 0;
					if(sector_erase_info.pEraseCallBack != NULL)
					{
						sector_erase_info.pEraseCallBack();
					}
				}
				else
				{
					#ifdef FLASH_DEBUG
						printf("\r\n[FLASH] ---- Flash is BUSY ----\r\n");
					#endif

					if(sector_erase_cnt >= SECTOR_ERASE_MAX_CNT)
					{
						//Timer_Free(gEraseTimerID);
		    			//gEraseTimerID = TIMER_ERROR;
                        Stop_Timer_Cnt(gEraseTimerID);
						sector_erase_timer_running = 0;
						sector_erase_cnt = 0;
					}
				}
			}
			break;

		default:
			break;
	}
}

uint32_t GetTotelValidLength(void)
{
	#ifdef FLASH_DEBUG
		printf("[FLASH] GetTotelValidLength, totel_availabe_length = %d\r\n",storageExtManage.totel_availabe_length);
	#endif

	return (storageExtManage.totel_availabe_length);
}

void buildTest(void)
{
	uint8_t i;
	uint8_t temp[4] = {0};
	uint32_t write_addr = 0;
	uint32_or_uint8_u *p_start_time = (uint32_or_uint8_u *)temp;

	CMD_CE();
	delay1ms(200);

	for(i = 0;i < 32;i ++)
	{
		#ifdef FLASH_DEBUG
			printf("[FLASH] temp = %d\r\n",p_start_time->l);
		#endif
		CMD_PageProgram(write_addr + i * PAGE_SIZE, temp, sizeof(uint32_or_uint8_u));
		delay1ms(10);
		#ifdef FLASH_DEBUG
			CheckWrite(write_addr + i * PAGE_SIZE,PAGE_SIZE);
		#endif

		if(!(i%2))
		{
			p_start_time->l += 1;
		}
	}
}

/*********************************************************************************
  * Get the storage information
  * @param  flash_addr: the start addr of the partition
  * @param  start_time: the start time of the partition
  * @param  page_num: the number of page
  * @retval None
**********************************************************************************/
void GetStorageInfo(void)
{
    BLEprintf("arr[%d]=%08x\r\n",index_storage,StorageInfoArr[index_storage]);
    index_storage ++;
    if(index_storage >= sizeof(StorageInfoArr))
    {
        index_storage = 0;
    }

    BLEprintf("arr[%d]=%08x\r\n",index_storage,StorageInfoArr[index_storage]);
    index_storage ++;
    if(index_storage >= 15)
    {
        index_storage = 0;
    }
}

/*********************************************************************************
  * Get the storage information
  * @param  flash_addr: the start addr of the partition
  * @param  start_time: the start time of the partition
  * @param  page_num: the number of page
  * @retval None
**********************************************************************************/
void SetStorageInfo(void)
{
    StorageInfoArr[0] = storageExtManage.available_partition;
    StorageInfoArr[1] = storageExtManage.totel_availabe_length;
    StorageInfoArr[2] = current_rd_partition.start_time;
    StorageInfoArr[3] = current_rd_partition.start_page_addr;
    StorageInfoArr[4] = current_rd_partition.end_page_addr;
    StorageInfoArr[5] = totel_len_InCurrReadPartition;
    StorageInfoArr[6] = storageIntManage.u32RdPointer;
    StorageInfoArr[7] = storageIntManage.u32RdNum_CurrPartition;
    StorageInfoArr[8] = pre_read_info.pre_rd_len;

    StorageInfoArr[9] = current_partition.start_time;
    StorageInfoArr[10] = current_partition.start_page_addr;
    StorageInfoArr[11] = current_partition.end_page_addr;
    StorageInfoArr[12] = storageIntManage.u32WrPointer;
    StorageInfoArr[13] = storageIntManage.u32WrNum_CurrPartition;
    StorageInfoArr[14] = pre_write_info.pre_wr_len;
    #ifdef STORAGE_LOG
    StorageInfoArr[15] = storage_error_cnt;
    #endif
}


void GetStorageTmpInfo(void)
{
    //#ifdef FLASH_DEBUG
		printf("\r\n\r\n***[FLASH] GetStorageTmpInfo***\r\n");
	//#endif
    printf("storageExtManage.available_partition = %d\r\n",storageExtManage.available_partition);
    printf("storageExtManage.totel_availabe_length = %d\r\n",storageExtManage.totel_availabe_length);
    printf("current_rd_partition.start_time = %d\r\n",current_rd_partition.start_time);
    printf("current_rd_partition.start_page_addr = %d\r\n",current_rd_partition.start_page_addr);
    printf("current_rd_partition.end_page_addr = %d\r\n",current_rd_partition.end_page_addr);
    printf("totel_len_InCurrReadPartition = %d\r\n",totel_len_InCurrReadPartition);
    printf("storageIntManage.u32RdPointer = %d\r\n",storageIntManage.u32RdPointer);
    printf("current_rd_partition.end_page_addr = %d\r\n",current_rd_partition.end_page_addr);
    printf("totel_len_InCurrReadPartition = %d\r\n",totel_len_InCurrReadPartition);
    printf("storageIntManage.u32RdNum_CurrPartition = %d\r\n",storageIntManage.u32RdNum_CurrPartition);
    printf("current_rd_partition.end_page_addr = %d\r\n",current_rd_partition.end_page_addr);
    printf("pre_read_info.pre_rd_len = %d\r\n\r\n",pre_read_info.pre_rd_len);

    printf("current_partition.start_time = %d\r\n",current_partition.start_time);
    printf("current_partition.start_page_addr = %d\r\n",current_partition.start_page_addr);
    printf("current_partition.end_page_addr = %d\r\n",current_partition.end_page_addr);
    printf("storageIntManage.u32WrPointer = %d\r\n",storageIntManage.u32WrPointer);
    printf("storageIntManage.u32WrNum_CurrPartition = %d\r\n",storageIntManage.u32WrNum_CurrPartition);
    printf("pre_write_info.pre_wr_len = %d\r\n",pre_write_info.pre_wr_len);
}

/*********************************************************************************
  * Create a virtual partition for test
  * @param  flash_addr: the start addr of the partition
  * @param  start_time: the start time of the partition
  * @param  page_num: the number of page
  * @retval None
**********************************************************************************/
void CreatePartitionForTest(uint32_t flash_addr, uint32_t start_time, uint32_t page_num)
{
    uint8_t rd_temp[PAGE_SIZE] = {0};
    uint32_t index = 0;
    uint32_t data_len = 0;
	uint32_t write_addr = 0;
    uint8_t rtc_time[3] = {0};
    uint32_t j = 0;

    #ifdef FLASH_DEBUG
		printf("\r\n[FLASH] CreatePartitionForTest\r\n");
	#endif

    write_addr = ExtFLASH_ComputeCurrSectorStartAddr(flash_addr);
	for(index = 0;index < page_num;index ++)
	{
        /* Erase the sector */
        if(true == ExtFLASH_IsSectionBoundary(write_addr))
        {
            CMD_SE(write_addr);
            delay1ms(200);
        }

        /* Manully write data to page */
        /* First,build data */
        data_len += (PAGE_SIZE - PAGE_HEADER_SIZE);
        for(j = 0;j < (PAGE_SIZE - PAGE_HEADER_SIZE)/8;j++)
        {
            if((rtc_time[2] == 0) && (rtc_time[1] == 0) && (rtc_time[0] == 0))
            {
                rd_temp[(j + 1) * 8] = rtc_time[2];
                rd_temp[(j + 1) * 8 + 1] = rtc_time[1];
                rd_temp[(j + 1) * 8 + 2] = rtc_time[0];
                rd_temp[(j + 1) * 8 + 3] = FreeRunNightstartFram;
                rd_temp[(j + 1) * 8 + 4] = 1;
                rd_temp[(j + 1) * 8 + 5] = 0x10;
                rd_temp[(j + 1) * 8 + 6] = 0;
                rd_temp[(j + 1) * 8 + 7] = 0;
            }
            else
            {
                rd_temp[(j + 1) * 8] = rtc_time[2];
                rd_temp[(j + 1) * 8 + 1] = rtc_time[1];
                rd_temp[(j + 1) * 8 + 2] = rtc_time[0];
                rd_temp[(j + 1) * 8 + 3] = OSASID;
                rd_temp[(j + 1) * 8 + 4] = 70;
                rd_temp[(j + 1) * 8 + 5] = 99;
                rd_temp[(j + 1) * 8 + 6] = 0;
                rd_temp[(j + 1) * 8 + 7] = 100;
            }

            rtc_time[0]++;
            if(rtc_time[0] > 59)
            {
                rtc_time[1] ++;
                rtc_time[0] = 0;
            }
            if(rtc_time[1] > 59)
            {
                rtc_time[2] ++;
                rtc_time[1] = 0;
            }
        }

        /* Second,page write data */
		ExtFLASH_RealFlashPageWrite(write_addr, start_time, data_len,
                                    (rd_temp + PAGE_HEADER_OFFSET),
                                    (PAGE_SIZE-PAGE_HEADER_SIZE));

        /* Third,update the write pointers */
        write_addr += PAGE_SIZE;
        write_addr %= FLASH_HEADER_SECTOR_BASE_ADDR;
	}

    #ifdef FLASH_DEBUG
		printf("\r\n[FLASH] CreatePartitionForTest,End\r\n");
	#endif
}

int8_t CheckTotelLenErr(uint32_t u32TmpTotelLen,uint8_t u8Process)
{
    int8_t ret = 0;

    #ifdef STORAGE_LOG

    static uint32_t u32LastTotelLen = 0;
    static bool bTotelLenError = FALSE;

    if(FALSE == bTotelLenError)
    {
        if(u32TmpTotelLen >= FLASH_HEADER_SECTOR_BASE_ADDR)
        {
            SaveStorageTotelLengthErrorLog(u32LastTotelLen,u32TmpTotelLen,u8Process,1);
            bTotelLenError = TRUE;
            ExtFLASH_DispalyError();
        }
        u32LastTotelLen = u32TmpTotelLen;
    }

    #endif

    return (ret);
}

#ifdef STORAGE_LOG

void ExtFLASH_DispalyError(void)
{
    StorageTotelLengthErrorLog_t StorageTotelLengthErrorLog;

    OLED_Configuration();
    USART_Configuration();
    OLED_DisplayClear();
    OLED_DisplayFullScreenBMP(MXS8475_FirmwareError);
    while(1) /* 	Dead Lock for Error writing 		*/
    {
        GetStorageTotelLengthErrorLog(&StorageTotelLengthErrorLog);
        delay1ms(200);
    }
}

int8_t CheckEraseSectorAddr(uint32_t addr)
{
    int8_t ret = 0;

    if(addr == (last_erase_sector_addr + SECTOR_SIZE))
    {
        ret = 0;
    }
    else
    {
        if((0 == addr) && ((FLASH_HEADER_SECTOR_BASE_ADDR - SECTOR_SIZE) == last_erase_sector_addr))
        {
            ret = 0;
        }
        else
        {
            ret = -1;
        }
    }

    return (ret);
}

int8_t CheckEraseSectorSequent(uint32_t addr)
{
    int8_t ret = 0;
    StorageLog_t storageLog;
    date_str_typedef date_s;               //RTC date
	RTC_TimeTypeDef rtc_time;             //RTC time

    if(addr < FLASH_HEADER_SECTOR_BASE_ADDR)
    {
        if(0 != CheckEraseSectorAddr(addr))
        {
            storage_error_cnt++;
            if(storage_error_cnt > 0)
            {
                /* Get the Current Time */
            	Calendar_Get(&date_s,&rtc_time);

                storageLog.Month = date_s.month;
                storageLog.Day = date_s.day;
                storageLog.Hour = rtc_time.RTC_Hours;
                storageLog.Min = rtc_time.RTC_Minutes;
                storageLog.Sec = rtc_time.RTC_Seconds;
                storageLog.part_num = storageExtManage.available_partition;
                storageLog.pre_rd_len = pre_read_info.pre_rd_len;
                storageLog.pre_wr_len = pre_write_info.pre_wr_len;
                storageLog.rd_part_end_page_addr = current_rd_partition.end_page_addr;
                storageLog.rd_part_start_page_addr = current_rd_partition.start_page_addr;
                storageLog.rd_part_start_time = current_rd_partition.start_time;
                storageLog.totel_availabe_length = storageExtManage.totel_availabe_length;
                storageLog.totel_len_InCurrReadPart = totel_len_InCurrReadPartition;
                storageLog.u32CurrSectorStartAdrr = addr;
                storageLog.u32LastSectorStartAddr = last_erase_sector_addr;
                storageLog.u32RdNum_CurrPartition = storageIntManage.u32RdNum_CurrPartition;
                storageLog.u32RdPointer = storageIntManage.u32RdPointer;
                storageLog.u32WrNumCurrPart = storageIntManage.u32WrNum_CurrPartition;
                storageLog.u32WrPointer = storageIntManage.u32WrPointer;
                storageLog.wr_part_end_page_addr = current_partition.end_page_addr;
                storageLog.wr_part_start_page_addr = current_partition.start_page_addr;
                storageLog.wr_part_start_time = current_partition.start_time;
                SetStorageLog(&storageLog);
            }

            last_erase_sector_addr = addr;
            ret = -1;

            #ifdef FLASH_DEBUG
        		printf("\r\n[FLASH] CheckEraseSectorSequent,u32LastSectorStartAddr = 0x%08x,u32CurrSectorStartAdrr = 0x%08x \r\n",last_erase_sector_addr,addr);
        	#endif
        }
        else
        {
            last_erase_sector_addr = addr;
            ret = 0;
        }
    }
    else
    {
        ret = 0;
    }

    return (ret);
}

int8_t SaveStorageTotelLengthErrorLog(uint32_t last_len,uint32_t curr_len,uint8_t process,uint8_t state)
{
    date_str_typedef date_s;               //RTC date
    RTC_TimeTypeDef rtc_time;             //RTC time
    StorageTotelLengthErrorLog_t StorageTotelLengthErrorLog;

    /* Get the Current Time */
	Calendar_Get(&date_s,&rtc_time);

    StorageTotelLengthErrorLog.u8State = state;
    StorageTotelLengthErrorLog.Month = date_s.month;
    StorageTotelLengthErrorLog.Day = date_s.day;
    StorageTotelLengthErrorLog.Hour = rtc_time.RTC_Hours;
    StorageTotelLengthErrorLog.Min = rtc_time.RTC_Minutes;
    StorageTotelLengthErrorLog.Sec = rtc_time.RTC_Seconds;
    StorageTotelLengthErrorLog.u32LastTotelLength = last_len;
    StorageTotelLengthErrorLog.u32CurrTotelLength = curr_len;
    StorageTotelLengthErrorLog.process = process;

    StorageTotelLengthErrorLog.start_time_rd_part = current_rd_partition.start_time;
    StorageTotelLengthErrorLog.start_page_addr_rd_part = current_rd_partition.start_page_addr;
    StorageTotelLengthErrorLog.end_page_addr_rd_part = current_rd_partition.end_page_addr;
    StorageTotelLengthErrorLog.u32TotelLenInCurrRdPart = totel_len_InCurrReadPartition;
    StorageTotelLengthErrorLog.u32RdPointer = storageIntManage.u32RdPointer;
    StorageTotelLengthErrorLog.u32RdNumInCurrPart = storageIntManage.u32RdNum_CurrPartition;
    StorageTotelLengthErrorLog.u8PreRdLen = pre_read_info.pre_rd_len;

    StorageTotelLengthErrorLog.start_time_wr_part = current_partition.start_time;
    StorageTotelLengthErrorLog.start_page_addr_wr_part = current_partition.start_page_addr;
    StorageTotelLengthErrorLog.end_page_addr_wr_part = current_partition.end_page_addr;
    StorageTotelLengthErrorLog.u32WrPointer = storageIntManage.u32WrPointer;
    StorageTotelLengthErrorLog.u32WrNumInCurrPart = storageIntManage.u32WrNum_CurrPartition;
    StorageTotelLengthErrorLog.u8PreWrLen = pre_write_info.pre_wr_len;

    SetStorageTotelLengthErrorLog(&StorageTotelLengthErrorLog);

    return (0);
}

int8_t SaveStorageTimerErrorLog(void)
{
    static uint8_t u8EraseTimerCnt_Test = 0;
    date_str_typedef date_s;               //RTC date
    RTC_TimeTypeDef rtc_time;             //RTC time
    StorageTimerErrorLog_t StorageTimerErrorLog;

    u8EraseTimerCnt_Test++;

    /* Get the Current Time */
	Calendar_Get(&date_s,&rtc_time);

    StorageTimerErrorLog.Month = date_s.month;
    StorageTimerErrorLog.Day = date_s.day;
    StorageTimerErrorLog.Hour = rtc_time.RTC_Hours;
    StorageTimerErrorLog.Min = rtc_time.RTC_Minutes;
    StorageTimerErrorLog.Sec = rtc_time.RTC_Seconds;
    StorageTimerErrorLog.cnt = u8EraseTimerCnt_Test;
    SetStorageTimerErrorLog(&StorageTimerErrorLog);

    return (0);
}

#endif


