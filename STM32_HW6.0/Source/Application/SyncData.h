#ifndef		__SYNCDATA_H_
#define 	__SYNCDATA_H_

#include "common.h"

#define SyncDataFormat0       0  //����ͬ����0�ָ�ʽ������֢�෽��
#define SyncDataFormat1       1  //����ͬ����һ�ָ�ʽ�����kICTMonitorTemplateFreeRunID
#define SyncDataFormat2       2  //����ͬ���ڶ��ָ�ʽ

extern uint32_t	TotleDataLen;
extern uint8_t     CurDataLen;
extern uint32_t    RemainDataLen;
extern uint8_t     SPISendedDataLen;
extern uint8_t     SyncACKCnt;     //����ͬ��Ӧ�����
extern uint16_t     SyncDataLen;
extern uint8_t     Flag_SyncDataDone;
extern uint8_t     Flag_RxedSyncDataLength;
extern uint16_t    RxedSyncDataLength;

void SyncData_Task_Handler(event_t Sync_Event);
int BLEprintf(const char *fmt,...);

#endif   /* __SYNCDATA_H_ */

