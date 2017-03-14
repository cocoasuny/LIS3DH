#include "led.h"
#include "stm32l1xx.h"

TIM_Cfg_Typedef         Tim_Cfg_RedLED;
TIM_Basic_Cfg_Typedef 	Tim_Basic_Cfg_RedLED;
Timer_ID_Typedef        gRedLEDxTimerID = TIMER_ERROR;

TIM_Cfg_Typedef         Tim_Cfg_GreenLEDx;
TIM_Basic_Cfg_Typedef 	Tim_Basic_Cfg_GreenLEDx;
Timer_ID_Typedef        gGreenLEDxTimerID = TIMER_ERROR;


static uint8_t red_led_cnt = 0;
static uint8_t green_led_cnt = 0;


void LEDx_Init(uint8_t led)
{
    if((led & LED_RED) == LED_RED)
    {
        GPIO_InitTypeDef GPIO_InitStructure;

        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIO_LED_R, ENABLE);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_LED_R;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIO_PORT_LED_R, &GPIO_InitStructure);
    }
    else if((led & LED_GREEN) == LED_GREEN)
    {
        GPIO_InitTypeDef GPIO_InitStructure;

        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIO_LED_G, ENABLE);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_LED_G;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIO_PORT_LED_G, &GPIO_InitStructure);
    }
}

void LEDx_ON(uint8_t led)
{
    if((led & LED_RED) == LED_RED)
    {
        GPIO_SetBits(GPIO_PORT_LED_R, GPIO_Pin_LED_R);
    }
    else if((led & LED_GREEN) == LED_GREEN)
    {
        GPIO_SetBits(GPIO_PORT_LED_G, GPIO_Pin_LED_G);
    }
}

void LEDx_OFF(uint8_t led)
{
    if((led & LED_RED) == LED_RED)
    {
        GPIO_ResetBits(GPIO_PORT_LED_R, GPIO_Pin_LED_R);
    }
    else if((led & LED_GREEN) == LED_GREEN)
    {
        GPIO_ResetBits(GPIO_PORT_LED_G, GPIO_Pin_LED_G);
    }
}

void LEDx_DeInit(uint8_t led)
{
    if((led & LED_RED) == LED_RED)
    {
        GPIO_InitTypeDef GPIO_InitStructure;

        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIO_LED_R, ENABLE);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_LED_R;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
        GPIO_Init(GPIO_PORT_LED_R, &GPIO_InitStructure);
    }
    else if((led & LED_GREEN) == LED_GREEN)
    {
        GPIO_InitTypeDef GPIO_InitStructure;

        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIO_LED_G, ENABLE);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_LED_G;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
        GPIO_Init(GPIO_PORT_LED_G, &GPIO_InitStructure);
    }
}

void RedLEDTimeoutHandler(Timer_ID_Typedef TIMID)
{
    TIMID = TIMID;

    red_led_cnt ++;
    red_led_cnt %= 2;
    if(!red_led_cnt)
    {
        LEDx_OFF(LED_RED);
    }
    else
    {
        LEDx_ON(LED_RED);
    }
}

void GreenLEDTimeoutHandler(Timer_ID_Typedef TIMID)
{
    TIMID = TIMID;

    #ifdef LED_DEBUG
        printf("GreenLEDTimeoutHandler,green_led_cnt = %d...\r\n",green_led_cnt);
    #endif

    green_led_cnt ++;
    green_led_cnt %= 2;
    if(!green_led_cnt)
    {
        LEDx_OFF(LED_GREEN);
    }
    else
    {
        LEDx_ON(LED_GREEN);
    }
}

void GreenAndRedLEDTimeoutHandler(Timer_ID_Typedef TIMID)
{
    TIMID = TIMID;

    #ifdef LED_DEBUG
        printf("GreenAndRedLEDTimeoutHandler,green_led_cnt = %d...\r\n",green_led_cnt);
    #endif

    green_led_cnt ++;
    green_led_cnt %= 2;
    if(!green_led_cnt)
    {
        LEDx_OFF(LED_GREEN);
        LEDx_OFF(LED_RED);
    }
    else
    {
        LEDx_ON(LED_GREEN);
        LEDx_ON(LED_RED);
    }
}


void LED_Task_Handler(event_t led_Event)
{
    switch(led_Event)
    {
        case gRedLEDFlashingStart:
            #ifdef LED_DEBUG
                printf("\r\n[LED] RedLEDFlashingStart\r\n");
            #endif
            LEDx_Init(LED_RED);
            Tim_Basic_Cfg_RedLED.enuTimerType                = TIM_TYPE_MS;
            Tim_Basic_Cfg_RedLED.u16TimePeriod               = RedLEDTimeOutTIM;
            Tim_Basic_Cfg_RedLED.NVIC_IRQChannelPreemptionPriority = OLEDShutDown_TIMPreemptionPriority;
            Tim_Basic_Cfg_RedLED.pIntCallBack                = RedLEDTimeoutHandler;
            Tim_Cfg_RedLED.TimerMode 			= TIM_MODE_BASIC;
            Tim_Cfg_RedLED.TimerBasicCfg 		= &Tim_Basic_Cfg_RedLED;
            Tim_Cfg_RedLED.TimerPWMCfg 		    = NULL;
            gRedLEDxTimerID                     = Timer_Allocate(&Tim_Cfg_RedLED);
            if(gRedLEDxTimerID != TIMER_ERROR)
            {
                LEDx_ON(LED_RED);
                red_led_cnt = 0;
                Start_Timer_Cnt(gRedLEDxTimerID);
            }
            #ifdef LED_DEBUG
            else
            {
                printf("***gRedLEDxTimerID,Timer_Allocate Error***\r\n");
            }
            #endif
            break;

        case gRedLEDKeepingON:
            #ifdef LED_DEBUG
                printf("\r\n[LED] RedLEDKeepingON\r\n");
            #endif
            Timer_Free(gRedLEDxTimerID);
            LEDx_ON(LED_RED);
            break;

        case gRedLEDFlashingStop:
            #ifdef LED_DEBUG
                printf("\r\n[LED] RedLEDFlashingStop\r\n");
            #endif
            Timer_Free(gRedLEDxTimerID);
            LEDx_OFF(LED_RED);
            LEDx_DeInit(LED_RED);
            break;

        case gGreenLEDFlashingStart:
            #ifdef LED_DEBUG
                printf("\r\n[LED] GreenLEDFlashingStart\r\n");
            #endif
            LEDx_Init(LED_GREEN);
            Tim_Basic_Cfg_GreenLEDx.enuTimerType                = TIM_TYPE_MS;
            Tim_Basic_Cfg_GreenLEDx.u16TimePeriod               = RedLEDTimeOutTIM;
            Tim_Basic_Cfg_GreenLEDx.NVIC_IRQChannelPreemptionPriority = OLEDShutDown_TIMPreemptionPriority;
            Tim_Basic_Cfg_GreenLEDx.pIntCallBack                = GreenLEDTimeoutHandler;
            Tim_Cfg_GreenLEDx.TimerMode 			= TIM_MODE_BASIC;
            Tim_Cfg_GreenLEDx.TimerBasicCfg 		= &Tim_Basic_Cfg_GreenLEDx;
            Tim_Cfg_GreenLEDx.TimerPWMCfg 		    = NULL;
            gGreenLEDxTimerID                     = Timer_Allocate(&Tim_Cfg_GreenLEDx);
            if(gGreenLEDxTimerID != TIMER_ERROR)
            {
                LEDx_ON(LED_GREEN);
                red_led_cnt = 0;
                Start_Timer_Cnt(gGreenLEDxTimerID);
            }
            #ifdef LED_DEBUG
            else
            {
                printf("***gGreenLEDxTimerID,Timer_Allocate Error***\r\n");
            }
            #endif
            break;

        case gGreenLEDKeepingON:
            #ifdef LED_DEBUG
                printf("\r\n[LED] GreenLEDKeepingON\r\n");
            #endif
            Timer_Free(gGreenLEDxTimerID);
            LEDx_ON(LED_GREEN);
            break;

        case gGreenLEDFlashingStop:
            #ifdef LED_DEBUG
                printf("\r\n[LED] GreenLEDFlashingStop\r\n");
            #endif
            Timer_Free(gGreenLEDxTimerID);
            LEDx_OFF(LED_GREEN);
            LEDx_DeInit(LED_GREEN);
            break;

        case gRedLED_GreenLEDFlashingStart:
            #ifdef LED_DEBUG
                printf("\r\n[LED] gRedLED_GreenLEDFlashingStart\r\n");
            #endif
            LEDx_Init(LED_GREEN);
            LEDx_Init(LED_RED);
            Timer_Free(gGreenLEDxTimerID);
            Timer_Free(gRedLEDxTimerID);
            Tim_Basic_Cfg_GreenLEDx.enuTimerType                = TIM_TYPE_MS;
            Tim_Basic_Cfg_GreenLEDx.u16TimePeriod               = RedLEDTimeOutTIM;
            Tim_Basic_Cfg_GreenLEDx.NVIC_IRQChannelPreemptionPriority = OLEDShutDown_TIMPreemptionPriority;
            Tim_Basic_Cfg_GreenLEDx.pIntCallBack                = GreenAndRedLEDTimeoutHandler;
            Tim_Cfg_GreenLEDx.TimerMode 			= TIM_MODE_BASIC;
            Tim_Cfg_GreenLEDx.TimerBasicCfg 		= &Tim_Basic_Cfg_GreenLEDx;
            Tim_Cfg_GreenLEDx.TimerPWMCfg 		    = NULL;
            gGreenLEDxTimerID                     = Timer_Allocate(&Tim_Cfg_GreenLEDx);
            if(gGreenLEDxTimerID != TIMER_ERROR)
            {
                LEDx_ON(LED_GREEN);
                LEDx_ON(LED_RED);
                red_led_cnt = 0;
                Start_Timer_Cnt(gGreenLEDxTimerID);
            }
            #ifdef LED_DEBUG
            else
            {
                printf("***gRedLED_GreenLEDFlashingStart,Timer_Allocate Error***\r\n");
            }
            #endif
            break;

        default:
            break;
    }
}




