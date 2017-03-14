#ifndef   __MONITOR_TEMPLATE_
#define   __MONITOR_TEMPLATE_

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include <string.h>
#include <stdlib.h>
#include "stdio.h"
#include "TS_Interface.h"

#define Monitor_Templat_TopLevel_Debug


#define FreeRunModeWithoutID

#define MaxMTLen       1200     //????????:??????:1200/4 = 300????
#define MaxEMTLen      80       //??????????:??????:80/4 = 20????
#define MaxMTDataLen   80       //??????????
#define MaxClockArrLen 80       //??????????????
#define FREERUN_STORAGE_HEADER_LEN 		(9)
#define FREERUN_STORAGE_TIME_STAMP_LEN 	(9)
#define kICTMonitorTemplateFreeRunMaxSampleFreq   250   //????????????(s)

#define MT_STORAGE_DATA_LEN 		    (8)


typedef enum
{
	NormalMT=0,
	ExcetionMT,
}MonitorTemplateType_Typedef;

typedef enum
{
	NormalVal=0,
	ExcetionVal,
}MonitorTemplateMeaResult_Typedef;

typedef struct
{
	uint8_t  Context[MaxMTLen];
	uint16_t DataLen;
}SampleTemplate_Typedef;

typedef struct
{
	uint8_t  Context[MaxEMTLen];
	uint16_t DataLen;
}SampleExcetionTemplate_Typedef;

typedef struct
{
    uint32_t     LightSampleFreq;         //??????????
    uint8_t      LightMTClockArr[MaxClockArrLen]; //?????????????
    uint8_t      DataLen;
}LightMTinfo_Typedef;

typedef struct
{
	uint32_t                   SampleFreq;              // Interval
	uint8_t 				   ClockArr[MaxClockArrLen];// Rang
	SampleTemplate_Typedef     SampleTimeClock;			//?????/?
	uint8_t                    SampleWorkDay;           //??Work Day
	uint16_t                   SampleLastTime;			//??????
	uint16_t              	   SampleID;				//????
	uint16_t                   SampleErrTime;           //???????????
	uint8_t                    MTSwitch;                //????????
	uint8_t                    VibrateSwitch;           //????,????
	uint8_t                    MTID;                    //??ID(??????)
    uint8_t                    SetMTID;                 //App?????ID
	uint16_t                   MotionInfoUpdateFreq;    //????????
	uint8_t                    ReadLength;              //????????,kICTMonitorTemplateFreeRunID??
    LightMTinfo_Typedef        LightMTinfo;             //??????????
}Monitor_Template_Typedef;

typedef struct
{
	uint32_t                   		 SampleFreq;              //????
	uint16_t                  		 SampleLastTime;          //??????
	uint16_t                  		 SampleID;                //????
	SampleExcetionTemplate_Typedef   SampleTimeClock;         //?????
}Excetion_Monitor_Template_Typedef;

extern Monitor_Template_Typedef MonitorTemplate;

char *strtok_r(char *s, const char *delim, char **save_ptr);
void SubString(void);
void UnPackMonitorTemplate(const uint8_t *p_MonitorTemplate,uint16_t Len);
void MonitorTemplate_Task_Handler(event_t MonitorTemplate_Event);
void ChangeDeviceMode_FromCheckUpToRTC(void);
MonitorTemplateType_Typedef GetMonitorTemplateType(void);
void  SetMonitorTemplateType(MonitorTemplateType_Typedef type);
void SetMonitorTemplateMeaResultType(MonitorTemplateMeaResult_Typedef type);
MonitorTemplateMeaResult_Typedef GetMonitorTemplateMeaResultType(void);
void Set_FreeRunStatusMonitorRecord_Time(void);
bool isFreeRunKickOff(void);
void OSASStartFrame(RTC_TimeTypeDef  rtc_time);
#endif /* __MONITOR_TEMPLATE_ */

