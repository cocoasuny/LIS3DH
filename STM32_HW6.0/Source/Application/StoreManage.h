#ifndef _STOREMANAGE_H_
#define _STOREMANAGE_H_

#include "stm32l1xx.h"
#include "MX25L1606E.h"
#include "Platform.h"
#include "TS_Interface.h"
#include "Global_Typedef.h"

#define START_TIME_AREA_OFFSET					(0)
#define TOTEL_LENGTH_AREA_OFFSET				(4)
#define PAGE_HEADER_OFFSET						(8)

#define START_TIME_AREA_SIZE					(4)
#define TOTEL_LENGTH_AREA_SIZE					(4)
#define PAGE_HEADER_SIZE						(START_TIME_AREA_SIZE + TOTEL_LENGTH_AREA_SIZE)

#define HEADER_START_TIME_AREA_SIZE				(4)
#define HEADER_START_ADDR_AREA_SIZE				(4)
#define HEADER_PARTITION_AREA_SIZE				(8)

#define FLASH_RAW_DATA_BASE_ADDR				(0 * SECTOR_SIZE)
#define FLASH_BACKUP_SECTOR_BASE_ADDR			((SECTOR_NUM - 1) * SECTOR_SIZE)
#define FLASH_HEADER_SECTOR_BASE_ADDR			((SECTOR_NUM - 2) * SECTOR_SIZE)

#define NO_ERASE								(0x00)
#define ERASE_BACKUP_AREA						(0x01)
#define ERASE_HEADER_AREA						(0x02)
#define ERASE_NEXT_AREA							(0x04)

#define SECTOR_ERASE_MAX_CNT					(20)

typedef enum {FLASH_ACCESS_ENABLE, FLASH_ACCESS_DISABLE = ~FLASH_ACCESS_ENABLE} flash_access_state_e;

typedef enum {FLASH_OPERATION_SUCCESS, FLASH_OPERATION_FAIL = ~FLASH_OPERATION_SUCCESS} flashOperationMsg;

typedef union {
	uint8_t 	arr[4];
	uint32_t 	l;
}uint32_or_uint8_u;

typedef struct{
	uint32_t 	start_time;
	uint32_t	len;
}page_header_t;

typedef struct{
	uint32_or_uint8_u 	start_time;
	uint32_or_uint8_u	len;
}page_head_t;

typedef struct {
	uint8_t 	sector_erase_process;
	uint32_t 	erase_page_addr;
	uint32_t 	backup_page_addr;
	void 		(*pEraseCallBack)(void);
	uint8_t		action_before_erase;
	uint8_t		last_partition_unavailable;
}sector_erase_info_t;

typedef struct {
	uint32_t 	u32WrPointer;
	uint32_t	u32WrNum_CurrPartition;
	uint32_t 	u32RdPointer;
	uint32_t	u32RdNum_CurrPartition;
}storageIntManage_t;

typedef struct {
	uint32_t 	start_time;
	//uint32_t 	availabe_length;
	uint32_t 	totel_availabe_length;
	uint8_t		available_partition;
}storageExtManage_t;

typedef struct {
	uint32_t 	start_time;
	uint8_t 	availabe_length;
	uint8_t		Buffer[EEPROMReadMaxData];
}storageExtBuffer_t;

typedef struct {
	uint16_t 	pre_rd_len;
	uint8_t		pre_rd_buffer[PAGE_SIZE];
}pre_read_info_t;

typedef struct {
	uint16_t 	pre_wr_len;
	uint8_t		pre_wr_buffer[PAGE_SIZE];
}pre_write_info_t;

typedef struct {
	uint8_t		FW_update_flag;
	uint8_t 	FW_Update_Type;
	uint32_t 	FW_crc_eeprom;
	uint8_t 	configFile[40];
}non_monitor_data_t;

void ExtFLASH_CopySector(uint32_t flash_addr_src,uint32_t flash_addr_dst);
void ExtFLASH_StorageInit(void);
int8_t ExtFLASH_StorageCheck(void);
uint32_t ExtFLASH_ExtGetTotelLen(void);
void SectorEraseTestHandler(void);
void Storage_Task_Handler(event_t storage_event);
flash_access_state_e GetExtFlashAccessState(void);
void ExtFLASH_RestorePartition(void);
flashOperationMsg ExtFLASH_ExtCreatePartition(uint32_t start_time);
flashOperationMsg ExtFLASH_ExtWrite(store_type_e type,uint8_t * pu8Src,uint8_t u8Length);
void ExtFLASH_WritePartitionInfo(uint32_t partition_index, uint32_t start_time, uint16_t start_addr_page, uint16_t end_addr_page);
flashOperationMsg ExtFLASH_ExtRead(store_type_e type,storageExtBuffer_t * p_storageExtBuffer,uint8_t u8Length);
flashOperationMsg ExtFLASH_ExtReadACK(uint8_t u8ValidLength);
uint32_t GetTotelValidLength(void);
void CheckWrite(uint32_t flash_start_addr,uint16_t len);
void ExtFLASH_ClearRAM(void);
void buildTest(void);
void GetStorageInfo(void);
void CreatePartitionForTest(uint32_t flash_addr, uint32_t start_time, uint32_t page_num);
uint32_t ExtFLASH_ComputeCurrSectorStartAddr(uint32_t flash_addr);
void ExtFLASH_SaveRdAddrToConst(void);
void GetStorageTmpInfo(void);
void SetStorageInfo(void);
int8_t ExtFLASH_GetRdAddrFromConst(StorageReadConstVar_t *p_StorageReadConstVar);

#ifdef STORAGE_LOG
int8_t CheckEraseSectorSequent(uint32_t addr);
void ExtFLASH_DispalyError(void);
#endif

#endif			// End of _STOREMANAGE_H_
