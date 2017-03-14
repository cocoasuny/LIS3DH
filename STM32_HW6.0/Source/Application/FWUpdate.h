#ifndef __FWUPDATA_H
#define __FWUPDATA_H

/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
#include "TS_Interface.h"

#define FIRMWARE_UPDATE_BUF_LEN 				(130) 		/* 	Size of firmware update buffer , uint8_t 	*/

#define Frame_TYPE_STM32APP                     0x00   
#define Frame_TYPE_STM32BOOT                    0x10   
#define Frame_TYPE_NRFAPP                       0x20    
#define Frame_TYPE_NRFBOOT                      0x30    
#define Frame_TYPE_NRFSD                        0x40    

/* 固件数据传输控制命令定义 */
#define PrepareRxFWData                           0x01  //准备传输固件数据包
#define TransmitFWStart                           0x02  //开始传输固件数据包
#define RepeatTransmitCurFW                       0x03  //重发当前固件数据包
#define StartUpDateFW                             0x04  //开始更新固件数据
#define TransmitFWRemain                          0x05  //传输剩余固件数据包
#define STM32ERROR                                0x06  //STM32 error
#define Nrf51822ERROR                             0x07  //设备固件错误

#define  AppA        0
#define  AppB        1

extern uint32_t u32FWTotelCrc_App;
extern uint32_t u32FWTotelCnt;
extern bool bFlagFWUpdateRunning;

void FW_Update_Handler(event_t event);
void SoftReset(void);
uint32_t ComputeCRC(uint32_t u32StartAddress,uint32_t u32EndAddress);

#endif /* __FWUPDATA_H */


