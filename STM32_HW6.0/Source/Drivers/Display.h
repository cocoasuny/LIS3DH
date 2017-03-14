#ifndef __DISPLAY_H
#define __DISPLAY_H

#include "SecondTick.h"
#include "TS_Interface.h"
#include "Global_Typedef.h"
#include "BasicTimer.h"

#define BlankBatteryLevel     5
#define DISPLAY_KEEPSTABLE_TIME_SLOT    60

void OLED_Configuration(void);
//void OLED_ReConfiguration(void);
void OLED_DeConfiguration(void);
void Set_OLED_Config_Status(uint8_t Status);
uint8_t Get_OLED_Config_Status(void);
void Set_OLED_Dis_Status(OledDisStatus status);
OledDisStatus Get_OLED_Dis_Status(void);
void OLED_DisplayChar(uint8_t Line, uint8_t Column, uint8_t Ascii);
void OLED_DisplayICON(uint8_t ICON);
void OLED_DisplayClear(void);
void OLED_DisplayClearWithOutRAM(void);
//void OLED_DisplayStr(uint8_t Line,uint8_t Colume,uint8_t ch[]);
void OledDisTask(event_t events);
void ShutDownOLED(Timer_ID_Typedef TIMID);
void BoundTimeOut(Timer_ID_Typedef TIMID);
void OLED_DriveSystemPower(uint8_t power);
uint8_t Get_OLED_DriveSystemPowerStatus(void);

void OLED_PutPixel(uint16_t _usX, uint16_t _usY, uint8_t _ucColor);
//uint8_t OLED_GetPixel(uint16_t _usX, uint16_t _usY);
//void OLED_DrawLine(uint16_t _usX1 , uint16_t _usY1 , uint16_t _usX2 , uint16_t _usY2 , uint8_t _ucColor);
//void OLED_DrawPoints(uint16_t *x, uint16_t *y, uint16_t _usSize, uint8_t _ucColor);
//void OLED_DrawRect(uint16_t _usX, uint16_t _usY, uint8_t _usHeight, uint16_t _usWidth, uint8_t _ucColor);
//void OLED_DrawCircle(uint16_t _usX, uint16_t _usY, uint16_t _usRadius, uint8_t _ucColor);
//void OLED_DrawBMP(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth, const uint8_t  *_ptr);
//void OLED_DisplayNum14x28(uint8_t x, uint8_t y, uint8_t Num);
//void OLED_DisplayDian14x28(void);

void OLED_Fill(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t dot)  ;
void OLED_DisplayNum(uint8_t x,uint8_t y,uint8_t chr,uint8_t Font);
void OLED_DisplayColon_Font_6x7(uint8_t x,uint8_t y);
void OLED_DisplaySlash_Font_6x7(uint8_t x,uint8_t y);
void OLED_DisplayDot_Font_11x18(uint8_t x,uint8_t y);

/* 最终有用的 */

void OLED_DisplayCtl(uint8_t status);
void OLEDDisplay_Stat_Set(OLEDDisplayStat newState);
OLEDDisplayStat OLEDDisplay_Stat_Get(void);
void OLED_DisplayBlueToothICON(uint8_t status);
void OLED_DisplayMonitorTemplateICON(uint8_t status);
void OLED_DisplayBatteryLevelICON(uint8_t Level);
void OLED_DisplayBatteryChargrLevelICON(uint8_t Level);
void OLED_DisplayProgress(uint8_t Progress);
void OLED_DisplayTestMode(uint8_t Status);
void OLED_DisplayRTC(uint8_t Hour_H,uint8_t Hour_L,uint8_t Min_H,uint8_t Min_L);
void OLED_DisplaySecTick(uint8_t Hour,uint8_t Min,uint8_t Sec,uint8_t SubSec);
void OLED_DisplayPassKey(uint8_t *	pKeyBuf);
void OLED_DisplayFullScreenBMP(const unsigned char * pitrueData);
void OLED_DisplayOccupyFullScreenBMP(const unsigned char * pitrueData);
void OLED_DisplayBPM(uint8_t x,uint8_t y,uint8_t _xE,uint8_t _yE,const unsigned char * pitrueData);
void OLED_DisplayStartFlash(void);
bool GetOLED_DisplayStartFlashStatus(void);
void OLED_DisplayBoundRemindFlash(uint8_t status);
void OLED_DisplayHRSpO2MeasureFlash(uint8_t status);
uint8_t GetHR_SpO2DisplayMeasureStatus(void);
void OLED_DisplayBATCharingFlash(uint8_t status);
void OLED_DisplayNoEnoughSpaceFlash(uint8_t status);
void OLED_DisplayIncomingCall(const uint8_t * pitrueData);
void OLED_DisplayInconmingCallFlash(uint8_t status);


void ClearKeepStableTimer(void);

#endif /* DISPLAY */
