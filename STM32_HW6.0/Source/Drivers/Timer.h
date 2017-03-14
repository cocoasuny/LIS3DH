#ifndef __TIMER_H
#define __TIMER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void Delay_Init(void);
void Delay_us(unsigned int nus);
void Delay_ms(unsigned short nms);
void delay1ms(unsigned int nms);
void Delay_ms_soft(uint16_t nms);

#endif /* __TIMER_H */
