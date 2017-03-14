#ifndef _LED_H_
#define _LED_H_

#include "stm32l1xx.h"
#include "common.h"

typedef enum{NORMAL_MODE, BREATHING_MODE}led_method_e;

typedef struct {
    uint8_t                 LEDx;
    uint16_t                period;
    led_method_e       led_method;
    uint32_t                flash_cnt;
}led_t;

#define LED_RED                 (0x01)
#define LED_GREEN               (0x02)

#define LED_NUM_TOTEL           (2)

void LED_Task_Handler(event_t led_Event);
void LEDx_Init(uint8_t led);
void LEDx_DeInit(uint8_t led);

#endif   // End of #ifndef _LED_H_
