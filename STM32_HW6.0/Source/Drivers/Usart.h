#ifndef __USART_H
#define __USART_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include <stdio.h>

void USART_Configuration(void);
void USART_DeConfiguration(void);
void USART_GPIOConfiguration(void);
int fputc(int ch, FILE *f);
void Usart_SendStrings(uint8_t *str);
void Usart_SendChar(uint8_t ch);
void SelfCheckLCDCtl(uint8_t ID ,uint8_t CMD);
void SelfCheck_TextUpdate(uint8_t ID,uint8_t *ptext,uint8_t len);

#endif /* __USART_H */

