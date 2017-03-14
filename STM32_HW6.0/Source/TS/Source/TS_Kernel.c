/*****************************************************************************
* Kernel / task handling implementation.
*
* (c) Copyright 2006, Freescale, Inc. All rights reserved.
*
*
* No part of this document must be reproduced in any form - including copied,
* transcribed, printed or by any electronic means - without specific written
* permission from Freescale Semiconductor Danmark A/S.
*
*
*****************************************************************************/

#include "FunctionLib.h"
#include "TS_Interface.h"
#include "TS_Kernel.h"
#include "list.h"
#include <stdlib.h>


/*****************************************************************************
******************************************************************************
* Private macros
******************************************************************************
*****************************************************************************/

/* Number of elements in an array. */
#ifndef NumberOfElements
#define NumberOfElements(array)     ((sizeof(array) / (sizeof(array[0]))))
#endif


/*****************************************************************************
******************************************************************************
* Private type declarations
******************************************************************************
*****************************************************************************/

/* If priority == g_InvalidTaskPriority_c, this entry is not in use. */
typedef struct tsTaskTableEntry_tag {
  uint8_t EventsLen;
  List  Events;              //[sun]
  tsTaskPriority_t priority;
  pfTsTaskEventHandler_t pfTaskEventHandler;
} tsTaskTableEntry_t;


/*****************************************************************************
******************************************************************************
* Private memory definitions
******************************************************************************
*****************************************************************************/

/* List of task descriptors. */
tsTaskTableEntry_t maTsTaskTable[gTsMaxTasks_c];

/* List of task ids (== indexes into maTsTaskTable[]), sorted by task */
/* priority, with the highest priority tasks at lower index positions */
/* in this table. */
tsTaskID_t maTsTaskIDsByPriority[NumberOfElements(maTsTaskTable)];

#if (gTsDebug_d == TRUE)
static uint16_t TsNonIdleLoopCounter = 0;
static uint16_t TsNonIdleLoopCounterMax = 0;
static uint32_t TsMainLoopCounter = 0;
#endif  /* gTsDebug_d */

/*****************************************************************************
******************************************************************************
* Private prototypes
******************************************************************************
*****************************************************************************/

/*****************************************************************************
******************************************************************************
* Public memory definitions
******************************************************************************
*****************************************************************************/

tsTaskID_t gIdleTaskID;

/*****************************************************************************
******************************************************************************
* Public functions
******************************************************************************
*****************************************************************************/

/* Remove events from a task's event flags. */
void TS_ClearEvent
  (
  tsTaskID_t taskID
  )
{
  IntDisable();
  EmptyTheList(&maTsTaskTable[taskID].Events);      //[sun]清除该任务中所有事件
  maTsTaskTable[taskID].EventsLen = 0;
  IntEnable();
}

/****************************************************************************/

/* Add a task to the task table.
 * Return the task ID, or gTsInvalidTaskID_c if the task table is full.
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
 */
tsTaskID_t TS_CreateTask
  (
  tsTaskPriority_t taskPriority,
  pfTsTaskEventHandler_t pfTaskEventHandler
  )
{
  index_t i;
  index_t freeSlot;
  index_t sizeofTaskId = sizeof(maTsTaskIDsByPriority[0]);

  /* Try to find a free slot in the task table. */
  for (i = 0, freeSlot = gTsInvalidTaskID_c; (i < NumberOfElements(maTsTaskTable));++i)
  {
    if (maTsTaskTable[i].priority == gTsInvalidTaskPriority_c)
    {
      freeSlot = i;
      break;
    }
  }                                     /* for (i = 0, freeSlot = 0xFF; ... */

  if (freeSlot == gTsInvalidTaskID_c)
  {
    return gTsInvalidTaskID_c;
  }

  maTsTaskTable[freeSlot].EventsLen = 0;
  maTsTaskTable[freeSlot].pfTaskEventHandler = pfTaskEventHandler;
  maTsTaskTable[freeSlot].priority = taskPriority;
  
  InitializeList(&maTsTaskTable[freeSlot].Events);   //[sun]初始化该任务事件链表

  /* maTsTaskIDsByPriority is maintained in sorted order, so 1) find the */
  /* place where this new task should go; 2) move everything up; and 3) add */
  /* the new task. */
  for (i = 0; i < NumberOfElements(maTsTaskIDsByPriority); i++)
  {
    /* If the end of the table is reached, just add the new task. */
    if (maTsTaskIDsByPriority[i] == gTsInvalidTaskPriority_c)
    {
      break;
    }

    /* If all tasks from this point on have lower priorities than the task */
    /* being added, move the rest up and insert the new one. */
    if (maTsTaskTable[maTsTaskIDsByPriority[i]].priority < taskPriority)
    {
      FLib_MemInPlaceCpy(&maTsTaskIDsByPriority[i + 1],
                         &maTsTaskIDsByPriority[i],
                         (NumberOfElements(maTsTaskIDsByPriority) - i - 1) * sizeofTaskId);
      break;
    }
  }                                     /* for (i = 0; ... */
  maTsTaskIDsByPriority[i] = freeSlot;

  return freeSlot;
}                                       /* TS_CreateTask() */

/****************************************************************************/

/* Remove a task from the task table. */
void TS_DestroyTask
  (
  tsTaskID_t taskID
  )
{
  index_t i;
  index_t sizeofTaskId = sizeof(maTsTaskIDsByPriority[0]);

  if (maTsTaskTable[taskID].priority == gTsInvalidTaskPriority_c)
  {
    return;
  }

  /* Mark this slot in the task descriptor table as unused. */
  maTsTaskTable[taskID].priority = gTsInvalidTaskPriority_c;

  /* Remove this task's ID from the priority table. Find it's position */
  /* in the table, and shift everything else down. */
  for (i = 0; i < NumberOfElements(maTsTaskIDsByPriority); i++)
  {
    if (maTsTaskIDsByPriority[i] == taskID)
    {
      FLib_MemCpy(&maTsTaskIDsByPriority[i],
                  &maTsTaskIDsByPriority[i + 1],
                  (NumberOfElements(maTsTaskIDsByPriority) - i - 1) * sizeofTaskId);

      /* Note that exactly one entry was removed. */
      maTsTaskIDsByPriority[NumberOfElements(maTsTaskIDsByPriority) - 1] = gTsInvalidTaskID_c;
	  maTsTaskTable[taskID].EventsLen = 0;
	  EmptyTheList(&maTsTaskTable[taskID].Events);
      break;
    }
  }

  return;
}                                       /* TS_DestroyTask() */

/****************************************************************************/

/* Initialize the task scheduler. */
void TS_Init(void)
{
  uint8_t i=0;
	
  for(i=0;i<gTsMaxTasks_c;i++)
  {
	  //((uint8_t *)maTsTaskTable)[i] = gTsInvalidTaskID_c;
	  ListIsEmpty(&maTsTaskTable[i].Events);
	  maTsTaskTable[i].EventsLen = 0;
	  maTsTaskTable[i].priority = 0xFF;
  }
  //FLib_MemSet(maTsTaskTable, gTsInvalidTaskPriority_c, sizeof(maTsTaskTable));
  FLib_MemSet(maTsTaskIDsByPriority, gTsInvalidTaskID_c, sizeof(maTsTaskIDsByPriority));

  gIdleTaskID = TS_CreateTask(gTsIdleTaskPriority_c, IdleTask);
}                                       /* TS_Init() */

/****************************************************************************/

/* Returns true if there are any pending events for any task. */
/* This function must not disable/enable interrupts. If it did, an */
/* an interrupt could be delayed until just after the task table scan, */
/* which might invalidate the result of the scan. The caller should */
/* consider whether or not to dis/enable interrupts before calling. */
bool_t TS_PendingEvents(void) {
//  index_t i;

//  for (i = 0; i < NumberOfElements(maTsTaskTable); i++) {
//    if (( maTsTaskTable[i].priority != gTsInvalidTaskPriority_c)
//        && maTsTaskTable[i].events) {
//      return TRUE;
//    }
//  }

  return FALSE;
}

/****************************************************************************/

/* Send events to a task. */
void TS_SendEvent
  (
  tsTaskID_t taskID,
  event_t events
  )
{
	Item  temp;
	
	temp.events = events;
	
	IntDisable();;
	if(maTsTaskTable[taskID].EventsLen < gTsMaxEvent_c)
	{
		if(!ListIsFull(&maTsTaskTable[taskID].Events))
		{
			if(AddItem(temp, &maTsTaskTable[taskID].Events) == true)
			{
				maTsTaskTable[taskID].EventsLen  = maTsTaskTable[taskID].EventsLen  + 1;
			}
		}
	}
	#if (gTsDebug_d == TRUE)
	else
	{
		printf("Creat task %d event err\r\n",taskID);
	}
	#endif /* gTsDebug_d */
	IntEnable();;
}                                       /* TS_SendEvent() */


/****************************************************************************/

/* the event of task is empty */
bool EventListIsEmpty(void)
{
	index_t i;
	index_t taskID;
	bool status = true;
	
    for (i = 0; i < NumberOfElements(maTsTaskIDsByPriority); ++i)
    {
		taskID = maTsTaskIDsByPriority[i];
		if (taskID == gTsInvalidTaskID_c)
		{
			break;
		}

		//if (maTsTaskTable[taskID].events)
		if(maTsTaskTable[taskID].EventsLen > 0)
		{
			if(ListIsEmpty(&maTsTaskTable[taskID].Events) == false)//事件链表不为空
			{
				status = false;
				break;
			}
		}
    }
	
	return status;
}


/****************************************************************************/

/* BeeStack's main task loop. Never returns. This function is the center of
 * the task system.
 */
void TS_Scheduler(void)
{
  index_t activeTask;
  event_t events;
  index_t i;
  index_t taskID;
  Item  temp;

  /* maTsTaskIDsByPriority[] is maintained in task priority order. If there */
  /* are fewer than the maximum number of tasks, the first gInvalidTaskID_c */
  /* marks the end of the table. */
  for (;;)
  {
    /* Look for the highest priority task that has an event flag set. */
    activeTask = gTsIdleTaskID_c;
    for (i = 0; i < NumberOfElements(maTsTaskIDsByPriority); ++i)
    {
      taskID = maTsTaskIDsByPriority[i];
      if (taskID == gTsInvalidTaskID_c)
      {
        break;
      }

      //if (maTsTaskTable[taskID].events)
	  if(maTsTaskTable[taskID].EventsLen > 0)
	  {
		  if(ListIsEmpty(&maTsTaskTable[taskID].Events) == false)//事件链表不为空
		  {
			activeTask = taskID;
			break;
		  }
	  }
    }

#if (gTsDebug_d == TRUE)
    /* Record the maximum number of times this loop executes without */
    /* a call to the idle task. */
    ++TsMainLoopCounter;
    if (activeTask == gTsIdleTaskID_c) {
      if (TsNonIdleLoopCounterMax < TsNonIdleLoopCounter) {
        TsNonIdleLoopCounterMax = TsNonIdleLoopCounter;
      }
      TsNonIdleLoopCounter = 0;
    } else {
      ++TsNonIdleLoopCounter;
    }
#endif /* gTsDebug_d */

    /* If there are no outstanding events, call the idle task. */
    IntDisable();
    //events = maTsTaskTable[activeTask].events;
    //maTsTaskTable[activeTask].events = 0;
	if(activeTask != gTsIdleTaskID_c)
	{
		temp = GetItem(&maTsTaskTable[activeTask].Events);
		events = temp.events;
		maTsTaskTable[activeTask].EventsLen = maTsTaskTable[activeTask].EventsLen - 1;
	}
	else
	{
		events = 0;
	}
    IntEnable();
	
	(*maTsTaskTable[activeTask].pfTaskEventHandler)(events);

  }                                     /* for (;;) */
}                                       /* TS_Scheduler() */
