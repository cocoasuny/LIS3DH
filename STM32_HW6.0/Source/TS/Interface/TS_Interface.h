/*****************************************************************************
* Kernel / task handling public API
*
* (c) Copyright 2006, Freescale, Inc. All rights reserved.
*
*
* No part of this document must be reproduced in any form - including copied,
* transcribed, printed or by any electronic means - without specific written
* permission from Freescale Semiconductor.
*
*****************************************************************************/

#ifndef _TS_INTERFACE_H_
#define _TS_INTERFACE_H_

#include "stdint.h"
#include "TS_Kernel.h"
//#include "platform.h"
#include <stdbool.h>


/*****************************************************************************
******************************************************************************
* Public macro definitions
******************************************************************************
*****************************************************************************/

/* Maximum number of tasks in the task table. */
#define gTsMaxTasks_c   20
#define gTsMaxEvent_c   40

#ifndef gTsDebug_d
  #define gTsDebug_d  FALSE                   /* Enable  kernel debug code. */
//  #define gTsDebug_d  TRUE                   /* Enable  kernel debug code. */
#endif

/*****************************************************************************
******************************************************************************
* Public type definitions
******************************************************************************
*****************************************************************************/

/* Unique task identifier numbers. */
/* tsTaskID_t ==  0 is reserved for the idle task. */
/* tsTaskID_t == -1 is reserved as an invalid value. */
typedef index_t tsTaskID_t;
#define gTsIdleTaskID_c     (( tsTaskID_t )  0 )
#define gTsInvalidTaskID_c  (( tsTaskID_t ) -1 )


/* Task priority. */
/* tsTaskPriority_t 越大，优先级越高 */
/* tsTaskPriority_t ==  0 is reserved for the idle task. */
/* tsTaskPriority_t == -1 is reserved as an invalid value. */
typedef uint8_t tsTaskPriority_t;
#define gTsIdleTaskPriority_c       (( tsTaskPriority_t )  0 )
#define gTsInvalidTaskPriority_c    (( tsTaskPriority_t ) -1 )

#define gTsLedTaskPriority_c                       0x07
#define gTsKeyTaskPriority_c                       0x0B
#define gTsOledDisTaskPriority_c                   0x04
#define gTsSpO2TaskPriority_c 	                   0x02
#define gTsSPITranslateTaskPriority_c              0x06
#define gTsMonitorTemplateTaskPriority_c           0x08
#define gTsSecTickTaskPriority_c 				   0x09
#define gTsPowerManageTaskPriority_c               0x0A
#define gTsSyncDataTaskPriority_c                  0x01
#define gTs3AxesTaskPriority_c 					   0x0C
#define gTsAlarmTaskPriority_c 					   0x0D
#define gTsBatManageTaskPriority_c 			       0x0E     // huayun,2014-07-21
#define gTsVoltDetectTaskPriority_c 			   0x0F     // huayun,2014-07-21
#define gTsStepTaskPriority_c 			           0x10     // huayun,2014-10-13
#define gTsFWUpdateTaskPriority_c 			       0x11
#define gTsIncomingCallTaskPriority_c 			   0x12
#define gTsDeviceControlTaskPriority_c             0x13
#define gTsStorageManageTaskPriority_c             0x14


/* Task Event */
/* Key Event */
#define gKeyEventLongPress_c 					  ((event_t) 1 << 0)
#define gKeyEventShortPress_c                     ((event_t) 1 << 1)

/* OLED Display Event */
#define gOledDisEventBlueIcon_c                   ((event_t) 1 << 0)
#define gOledDisEventMeaHR_c                      ((event_t) 1 << 1)
#define gOledDisEventModeHR_c                     ((event_t) 1 << 2)
#define gOledDisEventModeSecTick_c                ((event_t) 1 << 7)
#define gOledDisEventClearModeDis_c               ((event_t) 1 << 9)
#define gOledDisEventPassKey_c				      ((event_t) 1 << 11)
#define gOledDisEventModeStep_c                   ((event_t) 1 << 12)
#define gOledDisEventModeKeepStable_c             ((event_t) 1 << 13)
#define gOledDisEventAppCheckUp_c                 ((event_t) 1 << 14)
#define gOledDisEventSpO2Err_c					  ((event_t) 1 << 15)
#define gOledDisEventModeCHARGE_c                 ((event_t) 1 << 16)       // huayun,2014-07-21
#define gOledDisEventModeDistance_c               ((event_t) 1 << 17)
#define gOledDisEventModeCal_c                    ((event_t) 1 << 18)
#define gOledDisEventRTCUpDate_c                  ((event_t) 1 << 20)
#define gOledDisEventSecUpDate_c                  ((event_t) 1 << 21)
#define gOledDisEventDisplayAllRAM_c              ((event_t) 1 << 24)
#define gOledDisEventDisplayWearBad_c             ((event_t) 1 << 25)
#define gOledDisEventStepCnt_c                    ((event_t) 1 << 26)       // huayun,2014-10-16
#define gOledDisEventNotEnoughSpace_c             ((event_t) 1 << 27)
#define gOledDisEventClearKeepMoveDis_c           ((event_t) 1 << 28)
#define gOledDisEventDistanceCnt_c                ((event_t) 1 << 29)       // huayun,2014-11-07
#define gOledDisEventCalorieCnt_c                 ((event_t) 1 << 30)       // huayun,2014-11-07
/*SpO2 Event */

#define gSpO2EventStart							  ((event_t)(1 << 5))
#define gSpO2EventStop							  ((event_t)(1 << 6))

#define gSpO2EventMotionDetected			((event_t)(1 << 8))

//#define gSpO2EventWearDetStart 					((event_t)(1 << 9))

//#define gSpO2EventWearDetStop 					((event_t)(1 << 10))

//#define gSpO2EventMeasureCalPro                 ((event_t)(1 << 11))

//Event for Debug

/* SPI translate Event */
#define gSPITranslateEventTx                       ((event_t)(1 << 0))   //SPI发送事件
#define gSPITranslateEventRx                       ((event_t)(1 << 1))   //SPI接收事件
#define gSPITranslateEventTxHRSpO2                 ((event_t)(1 << 3))   //SPI发送即时采集心率血氧事件
#define gSPITranslateEventTxHR                     ((event_t)(1 << 4))   //SPI发送即时采集心率事件
#define gSPITranslateEventTxSpO2                   ((event_t)(1 << 5))   //SPI发送即时采集血氧事件
#define gSPITranslateEventTxMTHR                   ((event_t)(1 << 7))   //SPI发送监测模板采集心率事件
#define gSPITranslateEventTxMTSpO2                 ((event_t)(1 << 8))   //SPI发送监测模板采集血氧事件
#define gSPITranslateEventTxBatteryLevel           ((event_t)(1 << 9))   //SPI发送电池电量
#define gSPITranslateEventTxMoveLevel				((event_t)(1 << 10)) 	//move level send event

/* Monitor Template */
#define gMonitorTemplateEventStartSet             ((event_t)(1 << 0))       // Starting monitor
#define gMonitorTemplateEventSampleData           ((event_t)(1 << 1))
#define gMonitorTemplateEventTest                 ((event_t)(1 << 2))
#define gExcetionMonitorTemplateEventSet          ((event_t)(1 << 3))
#define gExcetionMonitorTemplateEventSampleData   ((event_t)(1 << 4))       // Save monitor data periodically
#define gExcetionMonitorTemplateEventStart        ((event_t)(1 << 5))
#define gExcetionMonitorTemplateEventStop         ((event_t)(1 << 6))

#define gExcetionMonitorTemplateEventSetFreeRunGuard               ((event_t)(1 << 7))
#define gExcetionMonitorTemplateEventStoreFreeRunData               ((event_t)(1 << 8))


#define gSecTikEventStop						   ((event_t)(1 << 3))
/* Second Ticker Event */
#define gSecTikEventStart							((event_t)(1 << 0))
#define gSecTikEventSuspend							((event_t)(1 << 1))
#define gSecTikEventResume							((event_t)(1 << 2))
#define gSecTikEventStop							((event_t)(1 << 3))
#define gSecTikEventBackground						((event_t)(1 << 4))

/* Power Manage Event */
#define gPowerManageClearTIMCnt                     ((event_t)(1 << 0))
#define gPowerManageRecoverOLEDDis                  ((event_t)(1 << 1))

/* Sync Data Event */
#define gSyncDataEventStart							((event_t)(1 << 0))
#define gSyncDataEventSPISent						((event_t)(1 << 1))
#define gSyncDataFormat1EventStart					((event_t)(1 << 2))
#define gSyncDataFormat2EventStart					((event_t)(1 << 3))

/* 3Axes event */
#define g3AxesEventWakeupInit										((event_t)(1 << 0))
#define g3AxesEventMoveCheckInit								((event_t)(1 << 1))
#define g3AxesEventMoveCheck										((event_t)(1 << 2))

/* Alarm event */
#define gAlarmEventNoFinger                          ((event_t)(1 << 0))
#define gAlarmEventParameterLimit                    ((event_t)(1 << 1))
#define gAlarmEventNotEnoughSpace                    ((event_t)(1 << 2))
#define gAlarmEventFactoryHWTest                     ((event_t)(1 << 3))
#define gAlarmEventFreeRunMTRunning                  ((event_t)(1 << 4))

/* Battery voltage detect Event */
#define gBATVoltageDetectEventStart                 ((event_t)(1 << 0))

/* Battery Change Event */
#define gBATEnterChargeEvent                        ((event_t)(1 << 0))
#define gBATFullChargeEvent                         ((event_t)(1 << 1))
#define gBATExitChargeEvent                         ((event_t)(1 << 2))
#define gBATLowEvent                                ((event_t)(1 << 3))
#define gBATBadEvent                                ((event_t)(1 << 4))

/* Step Event */
#define gStepStartEvent                             ((event_t)(1 << 0))
#define gStepRestartEvent                           ((event_t)(1 << 1))
#define gStepCountEvent                             ((event_t)(1 << 2))
#define gStepEnableDispStepEvent                    ((event_t)(1 << 3))         // Display Step count
#define gStepDisableDispStepEvent                   ((event_t)(1 << 4))         // No display step count
#define gStepEnableDispDistanceEvent                ((event_t)(1 << 5))         // Display Step count
#define gStepDisableDispDistanceEvent               ((event_t)(1 << 6))         // No display step count
#define gStepEnableDispCalorieEvent                 ((event_t)(1 << 7))         // Display Step count
#define gStepDisableDispCalorieEvent                ((event_t)(1 << 8))         // No display step count
#define gStepEnableTempEvent                        ((event_t)(1 << 9))
#define gStepDisableTempEvent                       ((event_t)(1 << 10))
#define gStepClearEvent                             ((event_t)(1 << 11))
#define gStepGetEvent                               ((event_t)(1 << 12))
#define gStepStopEvent                              ((event_t)(1 << 13))
//#define gStepSpO2EnterEvent 						((event_t)(1 << 14))
#define gStepResumeEvent 							((event_t)(1 << 15))
#define gMonitonInfoRecordEvent 					((event_t)(1 << 16))

/* FW Update Event */
#define gFWUpdateRxFWDateEvent                      ((event_t)(1 << 0))
#define gFWUpdateStartRxFWDateEvent                 ((event_t)(1 << 1))
#define gFWUpdatePrepareRxFWDateEvent               ((event_t)(1 << 2))
#define gFWUpdateStartFWDateEvent                   ((event_t)(1 << 3))
#define gFWUpdateStartFWNRFErrorEvent               ((event_t)(1 << 4))

/* Call Event */
#define gCallIncomingCallAdded                      ((event_t)(1 << 0))
#define gCallIncomingCallRemoved                    ((event_t)(1 << 1))

/* Device Control Event */
#define gDevCtlAppModifySNEvent                     ((event_t)(1 << 0))
#define gDevCtlAppModifyWorkModeEvent               ((event_t)(1 << 1))


/* Storage Event */
#define gStorageInit                      			((event_t)(1 << 0))
#define gStorageIntWrite                    		((event_t)(1 << 1))
#define gStorageIntRead                    			((event_t)(1 << 2))
#define gStorageReadACK                    			((event_t)(1 << 3))
#define gStorageCreatePartition						((event_t)(1 << 4))
#define gStorageCheckIsBusy							((event_t)(1 << 5))
#define gStorageEraseSector							((event_t)(1 << 6))

/* LEDx Event */
#define gRedLEDFlashingStart                        ((event_t)(1 << 0))
#define gRedLEDFlashingStop                         ((event_t)(1 << 1))
#define gRedLEDKeepingON                            ((event_t)(1 << 2))
#define gGreenLEDFlashingStart                      ((event_t)(1 << 3))
#define gGreenLEDFlashingStop                       ((event_t)(1 << 4))
#define gGreenLEDKeepingON                          ((event_t)(1 << 5))
#define gRedLED_GreenLEDFlashingStart               ((event_t)(1 << 6))


/* Task event handler function. */
typedef void ( *pfTsTaskEventHandler_t )(event_t);

/* The idle task, and only the idle task, is added to the task table */
/* by TS_Init(). The IdleTask() function must be provided by code */
/* external to the kernel. It is normally used for power management, */
/* non-volatile storage management, and other, similar background */
/* capabilities. */
extern void IdleTask( event_t );
extern void KeyTask(event_t );
extern void OledDisTask(event_t events);

/*****************************************************************************
******************************************************************************
* Public memory declarations
******************************************************************************
*****************************************************************************/

/* Store the TaskID of the idle task in a global, so the MAC layer can */
/* find it. */
extern tsTaskID_t gIdleTaskID;
extern tsTaskID_t gKeyTaskID;
extern tsTaskID_t gOledDisTaskID;  //OLED Display task
extern tsTaskID_t gTsSpO2TaskID_c; //SpO2 task
extern tsTaskID_t gTsSPITranslateTaskID_c;   //SPI translate tast,communicate with nRF51822
extern tsTaskID_t gTsMonitorTemplatID_c;     // Monitor Template task
extern tsTaskID_t gTsSecTickTaskID_c;        //Second Tick Task ID
extern tsTaskID_t gTsPowerManageTaskID_c;    //Power Manage Task ID
extern tsTaskID_t gTsSyncDataTaskID_c;       //Sync data Task ID
extern tsTaskID_t gTsAlarmTaskID_c;              //Alarm task ID
extern tsTaskID_t gTs3AxesTaskID_c;
/* The MAC TaskID must also be known by the MAC. */
extern tsTaskID_t gMacTaskID_c;
extern tsTaskID_t   gTsBatManageTaskID_c;          // Change Task ID,huayun,2014-07-21
extern tsTaskID_t   gTsBatMonitorTaskID_c;         // Voltage Detect Task ID,huayun,2014-07-21
extern tsTaskID_t   gTsStepTaskID_c;               // Step Task ID,huayun,2014-10-13
extern tsTaskID_t   gTsFWUpdateTaskID_c;           // FW Update Task ID
extern tsTaskID_t   gTsIncomingCallTaskID_c;
extern tsTaskID_t   gTsDeviceControlTaskID_c;
extern tsTaskID_t	gTsStorageTaskID_c;
extern tsTaskID_t	gTsLEDTaskID_c;

/*****************************************************************************
******************************************************************************
* Public Prototypes
******************************************************************************
*****************************************************************************/

/*****************************************************************************
 * TS_ClearEvent
 *
 * Remove events from a task's event flags.
 *****************************************************************************/
void TS_ClearEvent
  (
  tsTaskID_t taskID                   /* IN: Which task. */
  );

/*****************************************************************************
 * TS_CreateTask
 *
 * Add a task to the kernel's task table. Returns a task ID that can be passed
 * to TS_SendEvent() to identify the task. If the task table is full, returns
 * gTsInvalidTaskID_c.
 *
 * taskPriority == 0 is reserved for the idle task, and must never be specified
 * for any other task. TS_CreateTask() does not check for this.
 *
 * Note that TS_CreateTask() does not prevent a given event handler function
 * pointer from being added more than once to the task table.
 *
 * Note that if TS_CreateTask() is called with a taskPriority that is the
 * same as the priority of a task that is already in the task table, the
 * two tasks will end up in adjacent slots in the table. Which one is
 * called first by the scheduler is not specified.
 *****************************************************************************/
tsTaskID_t TS_CreateTask
  (
  tsTaskPriority_t taskPriority,                /* IN: Priority of new task. */
  pfTsTaskEventHandler_t pfTaskEventHandler     /* IN: Self explanatory. */
  );

/*****************************************************************************
 * TS_DestroyTask
 *
 * Remove a task from the kernel's task table.
 *****************************************************************************/
void TS_DestroyTask
  (
  tsTaskID_t taskId                     /* IN: Which task to destroy. */
  );

/*****************************************************************************
 * TS_Init()
 *
 * Initialize the kernel.
 *****************************************************************************/
void TS_Init( void );

/*****************************************************************************
 * TS_PendingEvents
 *
 * Checks for pending events for all the existing tasks. Returns true if there
 * is at least one pending event for at least one task.
 *****************************************************************************/
bool_t TS_PendingEvents( void );

/*****************************************************************************
 * TS_Scheduler
 *
 * Kernel main loop. Never returns once it is started.
 *****************************************************************************/
void TS_Scheduler( void );

/*****************************************************************************
 * TS_SendEvent
 *
 * Add events to a task's event flags.
 *****************************************************************************/
void TS_SendEvent
  (
  tsTaskID_t taskID,                    /* IN: Which task to send the event to. */
  event_t events                        /* IN: Event flag(s) to send. */
  );


bool EventListIsEmpty(void);

#endif/* #ifndef _TS_INTERFACE_H_ */

