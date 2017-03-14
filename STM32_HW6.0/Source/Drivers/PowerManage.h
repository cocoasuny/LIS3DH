#ifndef    __POWERMANAGE_H_
#define    __POWERMANAGE_H_
#include "TS_Interface.h"
#include "common.h"
void PowerManage_Task_Handler(event_t PowerManage_Event);
uint32_t GetTimeInLowPowerMode(void);
void LowPowerManage(void);
void GPIO_LowPower_Config(void);
void WakeUpFromLowPower(void);
void MX25L_GPIO_LowPower_Config(void);

extern void GPIO_Def_Config(void);

#endif /* __POWERMANAGE_H_ */

