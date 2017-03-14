/******************** (C) COPYRIGHT 2014 iCareTech ********************

*
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
//#include "BMA250E_Driver.h"
#include "bma250e.h"
#include "platform.h"
#include "stm32l1xx.h"
#include "Timer.h"
#include "Usart.h"
//#include "common.h"
#include "PowerManage.h"
#include "IIC_GPIO.h"
#include "IIR_Filter_Paramter.h"
#include "gSensor_data_process.h"
#include "cc_alg_app_interface.h"

static int16_t mGsenDatX[SENSOR_DATA_BUF_OFFSET];
static int16_t mGsenDatY[SENSOR_DATA_BUF_OFFSET];
static int16_t mGsenDatZ[SENSOR_DATA_BUF_OFFSET];

int16_t accDatBackTmpX[STEP_MAX_PROCESS_NUM] = {0};          // For backup the acceleration data for SPO2 and HR
int16_t accDatBackTmpY[STEP_MAX_PROCESS_NUM] = {0};          // For backup the acceleration data for SPO2 and HR
int16_t accDatBackTmpZ[STEP_MAX_PROCESS_NUM] = {0};          // For backup the acceleration data for SPO2 and HR


/* Gsensor submodule's status */
//static SubModuleStat m_gSensor_Submodule_stat = SUB_MOD_STOP;

/* 	time record for 	*/
static uint32_t u32TimeLastSec = 0;

/* Private function prototypes -----------------------------------------------*/

//static void max_min_find(int16_t * pDatArrBegin, uint16_t datLen, int16_t * datMax, int16_t * datMin);
bool gSensor_move_check(int16_t * sensorDatBufC, uint16_t dataLen, uint32_t moveThreshold, uint16_t * pValStd);
static void gsen_dat_std_cal(int16_t * pDatIn, uint16_t datLen, uint32_t * valStd);
static void gSensor_downsample_proc(int16_t * pSrc, int16_t * pDes, uint8_t sampleInterval, gSensorRangeType gSenRange, uint16_t rawDatLen);


/*******************************************************************************
* Function Name  : gSensor_Wave_Hand_Check
* Description    : handle for move check interrupt
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
CheckStat gSensor_Wave_Hand_Check(gSensorRangeType gSensorRange)
{
	/* 	Algorithm : 
		1. slope:
			a. rising to above a threshold in a given time
		2. stable:
			a. following the edge, in 2s interval, at least 20 points is stable;
			b. following the edge, in 2s interval, at least 20 points is bigger than threshold
	*/

	/*	Implement:
		1. Z Axes:data [0..255], 8 blocks, 0~7
			a. check slop in block 5,6
				1). standard: p0<p1<p2<p3<p4, 130ms
				2). standard: p4 - V_thres > p0
			b. check stable in following points, last to block 6,7
				1). at least 20 points bigger than threshold
				2). at least 20 points is within a range
		2. X/Y Axes:
			a. from -g to near zero

	*/


	int16_t 	i, k;
	uint8_t 	flagZRisingEdge;
	uint8_t 	flagZTop;
	uint8_t 	flagXStable, flagYStable, flagZStable;
	uint8_t 	cntTopSide;
	uint32_t 	curTimeInt;
	int16_t 	gsenValTmp1, gsenValTmp2;
	
	uint32_t 	stdStableX,stdStableY,stdStableZ;
	RTC_TimeTypeDef 			curTime;

	int16_t 	gSenSlopeCompThre, gSenSlopeChekThre;
	
	
	

#ifdef THREE_AXES_DEBUG_ENABLE
	printf("check hand \n");
#endif
	
	
	/* 	LUT the threshold for judgement 	*/
	gSenSlopeCompThre = GSEN_SLOPE_COMP_THRE << gSensorRange;
	gSenSlopeChekThre = GSEN_SLOPE_CHEK_THRE << gSensorRange;
	
	/* 	check the rising edge of Z axes 		*/
	flagZRisingEdge = 0;
	k = 0;
	for(i = GSEN_SLOPE_CHEK_RANGE_LOW + GSEN_SLOPE_CHEK_SER; i < GSEN_SLOPE_CHEK_RANGE_HIGH; i++)
	{
		/* 	loop to find the stable points to compare 	*/
		gsenValTmp1 = (DATA_Z(i) > DATA_Z(i-1)) ? DATA_Z(i) - DATA_Z(i-1) : DATA_Z(i-1) - DATA_Z(i);
		gsenValTmp2 = (DATA_Z(i) > DATA_Z(i+1)) ? DATA_Z(i) - DATA_Z(i+1) : DATA_Z(i+1) - DATA_Z(i);
		if(gsenValTmp1 < gSenSlopeCompThre && gsenValTmp2 < gSenSlopeCompThre)
		{
			/* 	Here is stable points A 		*/
			
			k = i - GSEN_SLOPE_CHEK_SER;
			
			gsenValTmp1 = (DATA_Z(k) > DATA_Z(k-1)) ? DATA_Z(k) - DATA_Z(k-1) : DATA_Z(k-1) - DATA_Z(k);
			gsenValTmp2 = (DATA_Z(k) > DATA_Z(k+1)) ? DATA_Z(k) - DATA_Z(k+1) : DATA_Z(k+1) - DATA_Z(k);
			
			if(gsenValTmp1 < gSenSlopeCompThre && gsenValTmp2 < gSenSlopeCompThre)
			{
				/* 	Compare points B occordinate to A 	*/
				/* 	in case z(i) > z(i-ser)... 	*/
				if( DATA_Z(i) - DATA_Z(k) > gSenSlopeChekThre )
				{
					/* 	check X axes has rising or falling edge at the same time slot 		*/
					if( (
							DATA_X(i) > DATA_X(k) &&
							(DATA_X(i) - DATA_X(k) > gSenSlopeChekThre)
						) ||
						(
							DATA_X(i) < DATA_X(k) &&
							(DATA_X(k) - DATA_X(i) > gSenSlopeChekThre)
						)
					   )
					{
						/* 	k is the rising edge 		*/
						#ifdef THREE_AXES_DEBUG_ENABLE
							printf("rising \n");
						#endif
						flagZRisingEdge = 1u;
						
						k = i;		/* 	Hold this position 		*/
						break;
					}
				}
				
			}

		}
	}

	
	/* 	check for top side axes			*/
	cntTopSide = 0;
	flagZTop = 0;
	if(flagZRisingEdge == 1u)
	{
		for(i = k + GSEN_STABLE_CHEK_OFFSET;  	/* k+offset to avoid shock at rising edge 		*/
			i < SENSOR_DATA_BUF_LEN; i++)
		{
			if( 
				IS_AXES_TOP_STABLE(DATA_Z(i),GSEN_Z_TOP_THRES_LOW << gSensorRange, GSEN_Z_TOP_THRES_HIGH << gSensorRange) 
//			&&	IS_X_AXES_TOP_STABLE(DATA_X(i)) 
//			&&	IS_Y_AXES_TOP_STABLE(DATA_Y(i))
				)
			{
				cntTopSide++;
			}
		}
		#ifdef THREE_AXES_DEBUG_ENABLE
			printf("Z cnt = %d \n",cntTopSide);
		#endif
		/* 	Stable Check for X/Y axes 		*/
		gsen_dat_std_cal(gSensorDatBufX + k + GSEN_STABLE_CHEK_OFFSET, GSEN_TOP_STABLE_STD_LEN, &stdStableX);

		gsen_dat_std_cal(gSensorDatBufY + k + GSEN_STABLE_CHEK_OFFSET, GSEN_TOP_STABLE_STD_LEN, &stdStableY);
		#ifdef THREE_AXES_DEBUG_ENABLE
			printf("std x = %d,y = %d\n",stdStableX,stdStableY);
		#endif
		
		
		if(
			cntTopSide >= GSEN_STABLE_CHEK_NUM 
		&&	stdStableX < (GSEN_TOP_STABLE_STD_THRES << (gSensorRange<<1u))
		&& 	stdStableY < (GSEN_TOP_STABLE_STD_THRES << (gSensorRange<<1u))
		)
		{
			flagZTop = 1u;
		}
		
	}
	
	
	/* 	Check the X/Y/Z std value 		*/
	gsen_dat_std_cal(gSensorDatBufX + SENSOR_DATA_BUF_LEN -GSEN_WAKEUP_CHEK_NUM , GSEN_WAKEUP_CHEK_NUM, &stdStableX);

	gsen_dat_std_cal(gSensorDatBufY + SENSOR_DATA_BUF_LEN -GSEN_WAKEUP_CHEK_NUM , GSEN_WAKEUP_CHEK_NUM, &stdStableY);
	
	gsen_dat_std_cal(gSensorDatBufZ + SENSOR_DATA_BUF_LEN -GSEN_WAKEUP_CHEK_NUM , GSEN_WAKEUP_CHEK_NUM, &stdStableZ);
	#ifdef THREE_AXES_DEBUG_ENABLE
		printf("48 len std x = %d,y = %d, z = %d\n",stdStableX,stdStableY,stdStableZ);
	#endif
		
	for(i = SENSOR_DATA_BUF_LEN -GSEN_WAKEUP_CHEK_NUM;  	/* k+offset to avoid shock at rising edge 		*/
		i < SENSOR_DATA_BUF_LEN; i++)
	{
		if( 
			IS_AXES_TOP_STABLE(DATA_Y(i),GSEN_Y_TOP_THRES_LOW << gSensorRange, GSEN_Y_TOP_THRES_LOW << gSensorRange) 
			)
		{
			cntTopSide++;
		}
	}
    #ifdef THREE_AXES_DEBUG_ENABLE
		printf("top side cnt = %d, stable X/Y/Z = %d,%d,%d \n",cntTopSide,stdStableX,stdStableY,stdStableZ);
	#endif
	if(stdStableX > (stdStableY * GSEN_WAKEUP_CHEK_Y_ZX_RATIO)
		&& stdStableZ > (stdStableY * GSEN_WAKEUP_CHEK_Y_ZX_RATIO)
		&& stdStableY < ((GSEN_TOP_STABLE_STD_THRES << (gSensorRange<<1u))) /* 	1 BITS shift to compensate "no sqrt operation" in std val calcuate 	*/
		&& cntTopSide >= GSEN_WAKEUP_CHEK_Y_STABLE_NUM
	)
	{
	/* 	Check if X/Y/Z is stable 	*/
		flagXStable = 1;
		flagYStable = 1;
		flagZStable = 1;
	}
	else
	{
	/* 	Check if X/Y/Z is stable 	*/
		flagXStable = 0;
		flagYStable = 0;
		flagZStable = 0;
	}
	
	


	
    #ifdef THREE_AXES_DEBUG_ENABLE
		printf("flagZTop = %d, flagZRisingEdge = %d,X/Y/Z Stable = %d%d%d\n",flagZTop,flagZRisingEdge,flagXStable,flagYStable,flagZStable);
	#endif
	/* 	Confirm a slope and stable condition 		*/
	if(
		(flagZTop == 1u && flagZRisingEdge == 1u)
		|| (flagXStable == 1u && flagYStable == 1u && flagZStable == 1u)
	)
	{
		/* 	Get current time, in second unit 		*/
  		/*Get current date and subsecond*/
		RTC_GetTime(RTC_Format_BIN, &curTime);

		/* 	Add night mode, no wakeup from 23:00 ~ 07:00 	*/
		if(curTime.RTC_Hours >= 23 || curTime.RTC_Hours <= 6)
		{
			return (CHECK_FAIL);
		}

		curTimeInt = (curTime.RTC_Hours * 3600) + (curTime.RTC_Minutes * 60) + curTime.RTC_Seconds;
		/* 	In case interval is bigger than 2s 		*/
		if( curTimeInt - u32TimeLastSec > GSEN_HAND_WAVE_TIME_LIMIT)
		{
			/* Send event to display the RTC screen */
			if(Get_OLED_Dis_Status() == OLEDDisShutDown && Device_Mode != Device_Mode_Charge && Device_Mode != Device_Mode_LowBattery
				&& Device_Mode != Device_Mode_CheckUp)
			{
				/* 	To Open Power supply to Peri device 	*/
				WakeUpFromLowPower();
				
				if(SPO2_HR_Measure_State == Start)   /* Avoid to press twice to start */
				{
					SPO2_HR_Measure_State = Stop;
				}
				/* 	Display RTC screen 						*/
				Device_Mode = Device_Mode_RTC;
				Set_OLED_Dis_Status(OLEDDisON);
				TS_SendEvent(gOledDisTaskID, gOledDisEventBlueIcon_c);
			}
			/* For Montior mode 	*/
			else if(
				Get_OLED_Dis_Status() == OLEDDisShutDown
			&&	cc_alg_SpO2_Stat_Get() == SPO2_HR_STATUS_RUNNING
			&&	gSubFunc_Stat_Get(SPO2_HR_MONITOR_STATE ) != OFF
			)
			{
				/* Avoid to press twice to start */
				if(SPO2_HR_Measure_State == Start)
				{
					SPO2_HR_Measure_State = Stop;
				}
				#ifdef THREE_AXES_DEBUG_ENABLE
					printf("in monitor mode wakeup \n");
				#endif
				
				/* 	Display RTC screen 						*/
				Device_Mode = Device_Mode_RTC;
				Set_OLED_Dis_Status(OLEDDisON);
				TS_SendEvent(gOledDisTaskID, gOledDisEventBlueIcon_c);
			}
			/* 	store current timer point 	*/
			u32TimeLastSec = curTimeInt;
			
			return(CHECK_PASS);
		} // end of if ( currTi....

	}	// end if if(flagZ ....)
	
	return (CHECK_FAIL);
 } // enf of func

/*******************************************************************************
* Function Name  : Gsensor_Int2_Handler
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void Gsensor_Int2_Handler(void)
//{
//	
//}

/*******************************************************************************
* Function Name  : Gsensor_Int1_Handler
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Gsensor_Int1_Handler(void)
{  
	TS_SendEvent(gTsStepTaskID_c,gStepGetEvent);
}



/*******************************************************************************
* Function Name  : gSensor_Raw_Dat_Copy_SpO2
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void gSensor_Raw_Dat_Copy_SpO2(void)
{
    memmove(mGsenDatX,gSensorDatBufX + SENSOR_DATA_BUF_LEN - GSEN_IIR_FILT_BLK_SIZE,GSEN_IIR_FILT_BLK_SIZE*sizeof(int16_t));
    memmove(mGsenDatY,gSensorDatBufY + SENSOR_DATA_BUF_LEN - GSEN_IIR_FILT_BLK_SIZE,GSEN_IIR_FILT_BLK_SIZE*sizeof(int16_t));
    memmove(mGsenDatZ,gSensorDatBufZ + SENSOR_DATA_BUF_LEN - GSEN_IIR_FILT_BLK_SIZE,GSEN_IIR_FILT_BLK_SIZE*sizeof(int16_t));
}

/*******************************************************************************
* Function Name  : gSensor_Raw_Dat_Copy_SpO2
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void gSensor_Raw_Dat_Copy_SpO2_for_StepCnt(uint8_t sampleInterval, gSensorRangeType gSenRange, uint16_t rawDatLen)
{
	int16_t * pDatSrc;
	int16_t * pDatDes;
	
	/* 	Copy X axes data 	*/
	pDatSrc = gSensorDatBufX + SENSOR_DATA_BUF_LEN - rawDatLen;
	pDatDes = accDatBackTmpX;
	
	gSensor_downsample_proc(pDatSrc, pDatDes, sampleInterval, gSenRange, rawDatLen);

	
	/* 	Copy Y axes data 	*/
	pDatSrc = gSensorDatBufY + SENSOR_DATA_BUF_LEN - rawDatLen;
	pDatDes = accDatBackTmpY;
	gSensor_downsample_proc(pDatSrc, pDatDes, sampleInterval, gSenRange, rawDatLen);

	/* 	Copy Z axes data 	*/
	pDatSrc = gSensorDatBufZ + SENSOR_DATA_BUF_LEN - rawDatLen;
	pDatDes = accDatBackTmpZ;
	gSensor_downsample_proc(pDatSrc, pDatDes, sampleInterval, gSenRange, rawDatLen);

}

static void gSensor_downsample_proc(int16_t * pSrc, int16_t * pDes, uint8_t sampleInterval, gSensorRangeType gSenRange, uint16_t rawDatLen)
{
	int16_t * pDatSrc;
	int16_t * pDatDes;
	uint16_t i;
	uint16_t j;
	
	/* 	Copy X axes data 	*/
	pDatSrc = pSrc;
	pDatDes = pDes;
	
	for(i = 0; i*sampleInterval < rawDatLen; i++)
	{
		pDatSrc = pSrc + i*sampleInterval;
		for(j = 0; (j < sampleInterval - 1) && (i*sampleInterval + j < rawDatLen); j++)
		{
			pDatSrc += j;
			*pDatDes++ = (*pDatSrc)>>gSenRange;
		}
	}
}




/*******************************************************************************
* Function Name  : gSensor_StepCnt_Wakeup
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void gSensor_StepCnt_Wakeup(int16_t * pAccX,int16_t *pAccY, int16_t *pAccZ, uint16_t lenData)
{
	/* 	Algorithm : 
		1. the maxVal - minVal > threshold 
	*/
	bool 	statStepCntWakeup;
	
	int16_t combGsenDat[SENSOR_DATA_BUF_OFFSET];
	int16_t 	i;
	uint16_t 	tmpStdVal;
	uint16_t 	lenDatComp;
	
	int16_t * pDatX;
	int16_t * pDatY;
	int16_t * pDatZ;
	
	if(pAccX == NULL || pAccY == NULL || pAccZ == NULL)
	{
		pDatX = gSensorDatBufX+SENSOR_DATA_BUF_LEN - SENSOR_DATA_BUF_OFFSET;
		pDatY = gSensorDatBufY+SENSOR_DATA_BUF_LEN - SENSOR_DATA_BUF_OFFSET;
		pDatZ = gSensorDatBufZ+SENSOR_DATA_BUF_LEN - SENSOR_DATA_BUF_OFFSET;
		lenDatComp = SENSOR_DATA_BUF_OFFSET;
	}
	else
	{
		pDatX = pAccX;
		pDatY = pAccY;
		pDatZ = pAccZ;
		lenDatComp = lenData;
	}
	
	
	/* 	Get the combined gsensor data 	*/
	
	for(i = 0; i < lenData; i++)
	{
		combGsenDat[i] =  (((pDatZ[i] >= 0) ? pDatZ[i] : - pDatZ[i]) + 
   						((pDatX[i] >= 0) ? pDatX[i] : - pDatX[i]) + 
						((pDatY[i] >= 0) ? pDatY[i] : - pDatY[i]));
	}
	
	/* 	Check move in XYZ axes 		*/
	statStepCntWakeup = gSensor_move_check(combGsenDat, lenDatComp, GSEN_STEPCNT_WAKEUP_THRES, &tmpStdVal);

	/* 	Move in any one of XYZ 		*/
	if(statStepCntWakeup == false)
	{
		/* 	Restart Step Cnt 		*/
		//printf("wake up \r\n");
		//TS_SendEvent(gTsStepTaskID_c, gStepRestartEvent);	
		step.eStepStatus = RUNNING;
	}
}

/*******************************************************************************
* Function Name  : cc_gSensor_move_check
* Description    : handle for move check interrupt
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
bool gSensor_move_check(int16_t * sensorDatBufC, uint16_t dataLen, uint32_t moveThreshold, uint16_t * pValStd)
{
	uint32_t valStd = 0;
	/*Add calculate codes here */
	
	/* 	Cal the std value of input array	*/
	
	gsen_dat_std_cal(sensorDatBufC, dataLen, &valStd);
	
	*pValStd = (valStd > 0xffff)? 0xffff : (uint16_t)valStd;
	
	#ifdef THREE_AXES_DEBUG_ENABLE
		ALG_PRINTF("std val = %x \r\n",valStd);
	#endif
	if(valStd < moveThreshold)
	{
		return(true);
	}
	else
	{
		return(false);
	}
}


 /******************************************************************
*                        dat_std_cal                              *
* [Yun] datLen <= 20, DatIn < 255 for hr <100 for spo2     		  *
* 		NOTE: no use data length more than 512, or overflow 	*
******************************************************************/
static void gsen_dat_std_cal(int16_t * pDatIn, uint16_t datLen, uint32_t * valStd)
{
	int32_t 	valMean;
	uint32_t 	sqrDat;
	uint32_t 	sqrDatSum;
	uint32_t 	datTmp;
	uint16_t 	i;
	
	if(datLen == 0)
	{
		return;
	}
	
	/* calculate the mean value of input data 	*/
	
	for(i = 0, valMean = 0; i < datLen; i++)
	{
		valMean += pDatIn[i];
	}
	
	valMean = valMean / datLen;
	
	/* calculate the summary of (x-ex)^2 		*/
	for(i = 0, sqrDatSum = 0; i < datLen; i++)
	{
		datTmp = (pDatIn[i] >= valMean)? pDatIn[i] - valMean : valMean - pDatIn[i];

		sqrDat = datTmp * datTmp;
		
		sqrDatSum += sqrDat;
	}

	*valStd = (sqrDatSum*100) / datLen;
}


 /*******************************************************************************
* Function Name  : Gsensor_Task_Handler
* Description    : task handler for 3 axes sensor
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Gsensor_Task_Handler(event_t gSensorEvent)
{
//	uint16_t tmpVal;
	if(gSensorEvent&g3AxesEventWakeupInit)
	{
		/* Init the double click init */
		//LIS3DH_Click_Wakeup_Init();
	}
	else if(gSensorEvent&g3AxesEventMoveCheck)
	{
		/* Calculate the move status */
		//gSensor_PreSpO2_MoveCheck(&tmpVal,true);
	}
}





//end of files 
