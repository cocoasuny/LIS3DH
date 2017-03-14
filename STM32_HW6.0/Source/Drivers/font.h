#ifndef __FONT_H
#define __FONT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"

#define UI_CHINESE_ENABLE           //∫∫ªØ∞ÊUI

#define Font_14x28       0
#define Font_11x18       1
#define Font_6x7         2

//extern unsigned char const nAsciiDot8x16[];


extern uint8_t const MXS8475_ModeKeepStable[];
extern uint8_t const DIS_ICON_SpO2Err[];
extern uint8_t const AppCtlShow1[];
extern uint8_t const ChargeFull[];
extern uint8_t const LowBatterWarn[];
extern uint8_t const DIS_ICON_SpO2Mode[];
extern uint8_t const DIS_ICON_SpO2Measure[];
extern uint8_t const DIS_ICON_NTCMode[];
extern uint8_t const DIS_ICON_BPM_17x7[];
extern uint8_t const DIS_ICON_Percent_11x9[];
extern uint8_t const DIS_ICON_LINE_11x3[];
extern uint8_t const DIS_ICON_M_5x7[];
extern uint8_t const DIS_ICON_KM_10x7[];
extern uint8_t const DIS_ICON_CAL_17x7[];
extern uint8_t const StartProgress[5][56];
extern uint8_t const StartPic[468];
extern uint8_t const BoundRemindPicON[4];
extern uint8_t const BoundRemindPicOFF[4];
extern uint8_t const BoundRemindPic2[468];
extern uint8_t const MeasuringPic[7][48];
#ifndef UI_CHINESE_ENABLE
extern uint8_t const NTCMeasuringPic[468];
#endif
extern uint8_t const NTCMeasureResult[468];
extern uint8_t const HR_SpO2MeasuringPic[468];
#ifdef UI_CHINESE_ENABLE
extern uint8_t const Charging[];
#else
extern uint8_t const BATChargingFlashPic[5][468];
#endif
extern uint8_t const WearBad[468];
extern uint8_t const DIS_ICON_NotEnoughSpace[468];

extern uint8_t const   BoundSuc[];
extern uint8_t const   BoundTIMOut[];
extern uint8_t const   BoundRelease[468];

extern uint8_t  const  Num14x28[10][56];
extern uint8_t  const  Num11x18[12][56];
extern uint8_t  const  Num6x7[10][7];
extern uint8_t  const  Colon4x28[28];
extern uint8_t  const  Colon4x7[10];
extern uint8_t  const  Slash4x7[7];
extern uint8_t  const  Dot2x18[18];
extern uint8_t  const  MonitorTemplate11x7[14];
extern uint8_t  const  BlueTooth11x7[14];
extern uint8_t  const  BatteryLevel[6][14];
#ifdef UI_CHINESE_ENABLE
extern uint8_t const CHINAESE_BatteryLevel[5][36];
#endif
extern uint8_t  const  Progress57x3[7][24];

extern uint8_t const MXS8475_FirmwareUpdating[];
extern const uint8_t MXS8475_FirmwareError[];

extern uint8_t const Calling[];

extern uint8_t const Progress1[];
extern uint8_t const TestModePicON[];
extern uint8_t const TestModePicOFF[];
#endif /* __FONT_H */
