#ifndef _STEP_H_
#define _STEP_H_

#include "stdio.h"
#include "stdint.h"
#include "arm_math.h"
#include "bma2x2.h"
#include "TS_Interface.h"

#define STEP_FREQ_MIN               (0.45)

#define STEP_FREQ_MAX               (4.5)

#define STEP_CALORIE_COEFFICIENT    (2415919)               // 0.001125

#define STEP_STRIDE_COEFFICIENT     (229)                    // 0.01 * 0.7

#define STEP_STATIC_COUNT           (10)

#define STEP_ACTIVITY_BUFFER        (9) 

#define BMA250E_FIFO_LEVEL 	        (32)

#define STEP_MAX_PROCESS_NUM 		(50)

#define BMA250E_SPO2_FIFO_LEVEL     (256)

#define BMA250E_DATA_COPY_LEN 		(BMA250E_SPO2_FIFO_LEVEL - BMA250E_FIFO_LEVEL)

//typedef struct {
//    short         height;
//    short         weight;
//	uint8_t         age;
//	uint8_t         sex;    //0:ÄÐ£»1£ºÅ®
//} userInfo_t;

typedef enum {INIT,RUNNING,SUSPEND} stepStatus_en;
typedef enum {NO_DISPLAY,STEP_DISPLAY,DISTANCE_DISPLAY,CALORIE_DISPLAY} stepDisplay_en;

typedef struct {
    stepStatus_en   eStepStatus;
    uint8_t         stepActivity;
    uint8_t         stepActivity_count;
    stepDisplay_en  eStepDisplay;
    uint8_t         stepTempEnable;             // Enable the temporary step count event
    uint32_t        stepCount;
    uint32_t        stepCount_old;
    uint8_t         gStepCountArr[5];           // The step count in BCD
    uint32_t        stepCount_temp;             // The base step count for temporary
    uint8_t         gStepCountArr_temp[5];      // The temporary step count in BCD
    int32_t         distance;                   // There is 14 decimal bit in this variable
    uint32_t        distance_int;               // There is no decimal bit
    int32_t         distance_old;
    uint8_t         distanceArr[5];             // The distance in BCD
    q31_t           calorie;                    // There is 14 decimal bit in this variable
    uint32_t        calorie_int;
    q31_t           calorie_old;
    uint8_t         calorieArr[5];              // The calorie in BCD
	uint32_t        stepPreCount;                
	uint32_t        distancePreCount;
	uint32_t        caloriePreCount;
} step_t;

extern step_t  step;
extern int16_t accDataBackup_x[];          // For backup the acceleration data for SPO2 and HR
extern int16_t accDataBackup_y[];          // For backup the acceleration data for SPO2 and HR
extern int16_t accDataBackup_z[];          // For backup the acceleration data for SPO2 and HR
//extern userInfo_t  userInfo;
extern bool gflagIsAccSampleRateConvert;


void Step_Task_Handler(event_t step_Event);
void BMA250E_SelfTest(void);
void BMA250EHW_SelfTest(void);
void Set_BMA250E_LowPower(void);
void StartStepCnt(void);
//uint8_t  u32toBCDConvert(uint32_t DataIn, uint8_t *pDatOut);

void Step_Cnt_Handler(int16_t * accDataX,int16_t * accDataY,int16_t * accDataZ, uint16_t numSample, bool flagIsInSpO2MeasureMode);
#endif
