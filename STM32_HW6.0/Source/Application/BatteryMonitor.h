#ifndef _BATTERYMONITOR_H_
#define _BATTERYMONITOR_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32l1xx_adc.h"
#include "TS_Kernel.h"
#include "platform.h"


#define 	CHARGE_FULL_TIMEOUT_WITH_CHG 		10
#define 	CHARGE_FULL_TIMEOUT_WITHOUT_CHG 		20

typedef enum {BattStateNormal,BattStateOut,BattStateLow} emBatteryAlarmState;

typedef struct {
    float           fBat_shutdown_level;
    float           fBat_low_alarm_level;
    float           fBat_0_grid_level;        
    float           fBat_1_grid_level;      
    float           fBat_2_grid_level;        
    float           fBat_3_grid_level;   
    float           fBat_full_charge_level;     
} BatLevel_t;

typedef struct {
    float                   fVoltage;
    uint8_t                 u8LastLevel;
    uint8_t                 u8CurrentLevel;
    emBatteryAlarmState     eBatteryAlarmState;                  // The battery alarm state
    bool                    bFullScale;                     // if true,the battery is full
    bool                    bExternPowerAvailable;          // if true,the extern power is pluged in
} BatVoltage_t;

extern BatVoltage_t batteryVoltage;
//extern BatLevel_t batteryLevel;

void BATVolDetectInit(void);
void setADCDMA_TransferComplete(void);
void VoltageDetect_Task_Handler(event_t voltDetect_Event);
uint8_t GetBatScaleLevel(void);
emBatteryAlarmState GetBatteryAlarmState(void);
bool GetBatChargeState(void);
bool GetBatFullChargeState(void);
void BATVoltageDetect(void);
void configureGPIO_ADC(void);
void DeconfigureGPIO_ADC(void);
void configureDMA_ADC(void);
void configureADC_BAT(void);
float ADC_SampleAndConv(ADC_TypeDef* ADCx);

#endif  // End of _BATTERYMONITOR_H_
