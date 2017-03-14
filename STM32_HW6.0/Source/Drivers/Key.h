#ifndef __KEY_H
#define __KEY_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "BasicTimer.h"
#include "TS_Interface.h"

extern uint8_t SPO2_HR_Measure_State;

void KEY_Config(void);
void KeyTask(event_t events);
void KeyPressDetect(Timer_ID_Typedef TIMID);

#endif /* __KEY_H */
