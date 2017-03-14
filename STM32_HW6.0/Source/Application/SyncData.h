#ifndef		__SYNCDATA_H_
#define 	__SYNCDATA_H_

#include "common.h"

#define SyncDataFormat0       0  //数据同步第0种格式：非鼾症类方案
#define SyncDataFormat1       1  //数据同步第一种格式：针对kICTMonitorTemplateFreeRunID
#define SyncDataFormat2       2  //数据同步第二种格式

extern uint32_t	TotleDataLen;
extern uint8_t     CurDataLen;
extern uint32_t    RemainDataLen;
extern uint8_t     SPISendedDataLen;
extern uint8_t     SyncACKCnt;     //数据同步应答次数
extern uint16_t     SyncDataLen;
extern uint8_t     Flag_SyncDataDone;
extern uint8_t     Flag_RxedSyncDataLength;
extern uint16_t    RxedSyncDataLength;

void SyncData_Task_Handler(event_t Sync_Event);
int BLEprintf(const char *fmt,...);

#endif   /* __SYNCDATA_H_ */

