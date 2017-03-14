#ifndef _CHARGE_H_
#define _CHARGE_H_

#include "common.h"

void BATChargeInit(void);
void BATManage_Task_Handler(event_t batteryManage_Event);

#endif
