#include "step.h"
#include "stdio.h"
#include "Process_Acc_Data.h"
#include "cc_alg_app_interface.h"

#ifdef GSEN_LIS3DH
	#include "lis3dh_driver.h"
#endif

#ifdef GSEN_BMA250E
	#include "gSensor_data_process.h"
#endif

/* 	Global varibles 		*/
int16_t accDataBackup_x[BMA250E_SPO2_FIFO_LEVEL] = {0};          // For backup the acceleration data for SPO2 and HR
int16_t accDataBackup_y[BMA250E_SPO2_FIFO_LEVEL] = {0};          // For backup the acceleration data for SPO2 and HR
int16_t accDataBackup_z[BMA250E_SPO2_FIFO_LEVEL] = {0};          // For backup the acceleration data for SPO2 and HR




bool gflagIsAccSampleRateConvert = FALSE;

/* 	timer id for time estimation 	*/
#ifdef STEP_DEBUG
	Timer_ID_Typedef            StepTimeEst;
	TIM_Cfg_Typedef    			StepTimeEstTypeCfg;
	TIM_Basic_Cfg_Typedef 		StepTimeEstBasicTypeCfg;
	uint16_t 					StepTimeEstCntRec[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
#endif


step_t  step;
//userInfo_t  userInfo;
static bma2x2_t bma250e;
static bma2x2acc_t * 	p_bma2x2acc;
static bma2x2acc_t bma250edataArr[BMA250E_FIFO_LEVEL];
//static uint32_t stepCount_diff = 0;
//static uint32_t stepCount_diff_old = 0;          // For the temporary step count 
//static int32_t u32StepCnt = 0;
//const short lenperStepCoifficient[7] = {25, 32, 42, 64, 106, 128, 183};  // {0.2, 0.25, 0.33, 0.5, 0.83, 1.0, 1.2}; << 7
//int32_t speedArr[5] = {0};


//static void u32toBCDConvert5Pos(uint32_t DataIn, uint8_t *pDatOut);
//static int32_t GetStride(float32_t stepsIn2second);
//static void distance_calorie(step_t *step_tmp);




/*******************************************************************************
* @brief   Get stride
* @param   stepsIn4second: steps in 4 seconds       
* @retval  The stride
*******************************************************************************/
//static int32_t GetStride(float32_t stepsIn2second)
//{  
//    int32_t stride_tmp = 0;   // Stride
//  
//    if(stepsIn2second == 0)
//    {
//        stride_tmp         = 0;
//    }
//    else if((stepsIn2second >= 0) && (stepsIn2second <= 2))
//    {
//        stride_tmp         = userInfo.height * lenperStepCoifficient[0];
//    }
//    else if((stepsIn2second > 2) && (stepsIn2second <= 3))
//    {
//        stride_tmp         = userInfo.height * lenperStepCoifficient[1];
//    }
//    else if((stepsIn2second > 3) && (stepsIn2second <= 4))
//    {
//        stride_tmp         = userInfo.height * lenperStepCoifficient[2];
//    }
//    else if((stepsIn2second > 4) && (stepsIn2second <= 5))
//    {
//        stride_tmp         = userInfo.height * lenperStepCoifficient[3];
//    }
//    else if((stepsIn2second > 5) && (stepsIn2second <= 6))
//    {
//        stride_tmp         = userInfo.height * lenperStepCoifficient[4];
//    }
//    else if((stepsIn2second >= 6) && (stepsIn2second < 8))
//    {
//        stride_tmp         = userInfo.height * lenperStepCoifficient[5];
//    }
//    else
//    {
//        stride_tmp         = userInfo.height * lenperStepCoifficient[6];
//    }

//    return (stride_tmp * STEP_STRIDE_COEFFICIENT);                  // 22-bit
//}

//static void distance_calorie(step_t *step_tmp)
//{
//    int32_t distance_tmp = 0;
//    q31_t calorie_tmp = 0;               
//    int32_t stepCount_diff = 0;
//    int32_t stride = 0;
//    float32_t stepFrequency = 0;
//    q31_t speed = 0;

//    u32StepCnt ++;                                      // 890ms
//    if(step_tmp->stepCount > step_tmp->stepCount_old)
//    {
//        stepCount_diff = step_tmp->stepCount - step_tmp->stepCount_old;
//        stepFrequency = stepCount_diff/u32StepCnt;
//        if((STEP_FREQ_MIN < stepFrequency) && (stepFrequency < STEP_FREQ_MAX))
//        {
//            stride = GetStride(stepFrequency * 2.2);
//            distance_tmp = (stride * stepCount_diff) >> 8;
//            speed = stride * stepCount_diff;
//            calorie_tmp = (q31_t)(((q63_t)(((q63_t)userInfo.weight * STEP_CALORIE_COEFFICIENT) >> 8) * speed) >> 31);
//        }
//        else
//        {
//            calorie_tmp = 0;
//            distance_tmp = 0;
//        }
//        step_tmp->calorie           += calorie_tmp;
//        step_tmp->distance          += distance_tmp;
//        u32StepCnt  = 0;
//    }
//    else
//    {
//        step_tmp->calorie           += 0;
//        step_tmp->distance          += 0;
//    }
//}

/*******************************************************************************
* @brief   Backup the acceleration data for SPO2 and heart rate
* @param   *buffer
* @retval  void
*******************************************************************************/
void BackupAccdata(bma2x2acc_t *buffer)
{
    uint8_t index;
    memmove(accDataBackup_x,accDataBackup_x + BMA250E_FIFO_LEVEL,BMA250E_DATA_COPY_LEN*sizeof(int16_t));
    memmove(accDataBackup_y,accDataBackup_y + BMA250E_FIFO_LEVEL,BMA250E_DATA_COPY_LEN*sizeof(int16_t));
    memmove(accDataBackup_z,accDataBackup_z + BMA250E_FIFO_LEVEL,BMA250E_DATA_COPY_LEN*sizeof(int16_t));

    for (index = 0; index < BMA250E_FIFO_LEVEL; index ++)
    {
        accDataBackup_x[index + BMA250E_DATA_COPY_LEN]  = buffer[index].x;
        accDataBackup_y[index + BMA250E_DATA_COPY_LEN]  = buffer[index].y;
        accDataBackup_z[index + BMA250E_DATA_COPY_LEN]  = buffer[index].z;
    }
}

/*******************************************************************************
* @brief   Step task handler
* @param   step_Event
* @retval  void
*******************************************************************************/
void Step_Task_Handler(event_t step_Event)
{
    uint8_t  bma2x2accData[32*2*3];
    uint16_t index          = 0; 

    switch(step_Event)
    {
        case gStepStartEvent:                       // Start step
#ifdef STEP_DEBUG
            printf("Now start step.\r\n");
#endif      
            /* Reset all parameters */
			memset(&step,0,sizeof(step_t));
			SPI1_Stat_Set(SUB_MOD_STOP);
			StartStepCnt();
            break;
            
        case gStepRestartEvent:                       // Restart step   
#ifdef STEP_DEBUG
            printf("Now restart step.\r\n");
#endif      
		if(SPI1_Stat_Get() != SUB_MOD_RUN)
		{
            step.eStepStatus    = RUNNING;
            //init_fir_instance();
            BMA250E_SPI_Oper_Init();
            BMA250E_Init_Step(); 
            BMA250E_Int1_Port_Enable();
			startDetection();
			gflagIsAccSampleRateConvert = FALSE;
		}
		break;
		
		
		case gStepResumeEvent:
		
		if(SPI1_Stat_Get() != SUB_MOD_RUN)
		{
            step.eStepStatus    = RUNNING;
            //init_fir_instance();
            BMA250E_SPI_Oper_Init();
            BMA250E_Int1_Port_Enable();
            BMA250E_Init_Step(); 
			//startDetection();
			gflagIsAccSampleRateConvert = FALSE;
		}
		break;
		
        
        case gStepGetEvent:                       // New data is coming
            printf("INT1\r\n");
//#ifdef STEP_DEBUG
//            StepTimeEstCntRec[0] = Get_Timer_Cnt(StepTimeEst);
//#endif			
//            if(
//				(Device_Mode != Device_Mode_Charge
//		&& 	Device_Mode != Device_Mode_LowBattery
//		&& 	Device_Mode != Device_Mode_FWUpdate)
//		&& 	SPI1_Stat_Get() != SUB_MOD_RUN
//			)
//            {   
//                /* 	Get new 32 samples and convert to raw data (957us)*/
//                BMA250E_SPI_Oper_Init();
//                bma250e.bus_read(bma250e.dev_addr,BMA2x2_FIFO_DATA_OUTPUT_REG,bma2x2accData,32*2*3);        // 858us
//                if(SPI1_Stat_Get() != SUB_MOD_RUN)
//                {
//                    BMA250E_SPI_Oper_DeInit();
//                }

//                /* Accupy 90us to get the raw data in short formate.*/
//                p_bma2x2acc = bma250edataArr;
//                for(index = 0;index < 32;index ++)
//                {
//                    p_bma2x2acc[index].x = BMA2x2_GET_BITSLICE(*(bma2x2accData + (index * 6)),BMA2x2_ACC_X10_LSB)
//                                        | (BMA2x2_GET_BITSLICE(*(bma2x2accData + (index * 6) + 1),BMA2x2_ACC_X_MSB)<<(BMA2x2_ACC_X10_LSB__LEN));
//                    p_bma2x2acc[index].x = p_bma2x2acc[index].x << (sizeof(short)*8-(BMA2x2_ACC_X10_LSB__LEN + BMA2x2_ACC_X_MSB__LEN));
//                    p_bma2x2acc[index].x = p_bma2x2acc[index].x >> (sizeof(short)*8-(BMA2x2_ACC_X10_LSB__LEN + BMA2x2_ACC_X_MSB__LEN));

//                    p_bma2x2acc[index].y = BMA2x2_GET_BITSLICE(*(bma2x2accData + (index * 6) + 2),BMA2x2_ACC_Y10_LSB)
//                                        | (BMA2x2_GET_BITSLICE(*(bma2x2accData + (index * 6) + 3),BMA2x2_ACC_Y_MSB)<<(BMA2x2_ACC_Y10_LSB__LEN ));
//                    p_bma2x2acc[index].y = p_bma2x2acc[index].y << (sizeof(short)*8-(BMA2x2_ACC_Y10_LSB__LEN + BMA2x2_ACC_Y_MSB__LEN));
//                    p_bma2x2acc[index].y = p_bma2x2acc[index].y >> (sizeof(short)*8-(BMA2x2_ACC_Y10_LSB__LEN + BMA2x2_ACC_Y_MSB__LEN));

//                    p_bma2x2acc[index].z = BMA2x2_GET_BITSLICE(*(bma2x2accData + (index * 6) + 4),BMA2x2_ACC_Z10_LSB)
//                                        | (BMA2x2_GET_BITSLICE(*(bma2x2accData + (index * 6) + 5),BMA2x2_ACC_Z_MSB)<<(BMA2x2_ACC_Z10_LSB__LEN));
//                    p_bma2x2acc[index].z = p_bma2x2acc[index].z << (sizeof(short)*8-(BMA2x2_ACC_Z10_LSB__LEN + BMA2x2_ACC_Z_MSB__LEN));
//                    p_bma2x2acc[index].z = p_bma2x2acc[index].z >> (sizeof(short)*8-(BMA2x2_ACC_Z10_LSB__LEN + BMA2x2_ACC_Z_MSB__LEN));
//                }
//            
//                /* move to new buffer for wakeup(76us) 	*/
//                BackupAccdata(bma250edataArr);
//                
//                /* 	check if there is "hand wave wakeup(185us) 		*/
//                gSensor_Wave_Hand_Check(GSEN_RANGE_16G);
//                
//                if(step.eStepStatus == RUNNING)
//                {
//                    TS_SendEvent(gTsStepTaskID_c,gStepCountEvent); 
//                } 
//                if(step.eStepStatus == SUSPEND)
//                { 
//                    /* 	Check if there is suspend to running transfer 	*/
//                    gSensor_StepCnt_Wakeup(NULL, NULL, NULL, 0);
//                }
//            }
            break;
        
        case gStepEnableDispStepEvent:
//            step.eStepDisplay = STEP_DISPLAY;
//#ifdef STEP_DEBUG
//            printf("eStepDisplay = %d.\r\n",step.eStepDisplay);
//#endif
            break;
        
        case gStepDisableDispStepEvent:
//            step.eStepDisplay = NO_DISPLAY;
//#ifdef STEP_DEBUG
//            printf("eStepDisplay = %d.\r\n",step.eStepDisplay);
//#endif
            break;
        
        case gStepEnableDispDistanceEvent:
//            step.eStepDisplay = DISTANCE_DISPLAY;
//#ifdef STEP_DEBUG
//            printf("eStepDisplay = %d.\r\n",step.eStepDisplay);
//#endif
            break;
        
        case gStepDisableDispDistanceEvent:
//            step.eStepDisplay = NO_DISPLAY;
//#ifdef STEP_DEBUG
//            printf("eStepDisplay = %d.\r\n",step.eStepDisplay);
//#endif
            break;
        
        case gStepEnableDispCalorieEvent:
//            step.eStepDisplay = CALORIE_DISPLAY;
//#ifdef STEP_DEBUG
//            printf("eStepDisplay = %d.\r\n",step.eStepDisplay);
//#endif
            break;
        
        case gStepDisableDispCalorieEvent:
//            step.eStepDisplay = NO_DISPLAY;
//#ifdef STEP_DEBUG
//            printf("eStepDisplay = %d.\r\n",step.eStepDisplay);
//#endif
            break;
        
        case gStepEnableTempEvent:
//            step.stepTempEnable = 1;                // Enable the temporary step
//            step.stepCount_temp = step.stepCount;
//            for(index = 0;index < 5;index ++)
//            {
//                step.gStepCountArr_temp[index] = 0;
//            }
//            stepCount_diff      = 0;
//            stepCount_diff_old  = 0;
//            InitAlgo(3);
            break;
        
        case gStepDisableTempEvent:
//            step.stepTempEnable         = 0;        // Disable the temporary step
//            step.stepCount_temp         = 0;
//            step.gStepCountArr_temp[0]  = 0;
//            step.gStepCountArr_temp[1]  = 0;
//            step.gStepCountArr_temp[2]  = 0;
//            step.gStepCountArr_temp[3]  = 0;
//            step.gStepCountArr_temp[4]  = 0;
            break;
        
        case gStepCountEvent:                       // Step  

			if(gflagIsAccSampleRateConvert == TRUE)
			{
				/* 	If in suspend mode, wake up step counter 	*/
				if(step.eStepStatus == SUSPEND)
				{ 
					/* 	Check if there is suspend to running transfer 	*/
					gSensor_StepCnt_Wakeup(accDatBackTmpX,accDatBackTmpY,accDatBackTmpZ,
											(SENSOR_DAT_DOWN_SAMPLE_DAT_LEN - (SENSOR_DAT_DOWN_SAMPLE_DAT_LEN/SENSOR_DAT_DOWN_SAMPLE_INTERVAL)));
				}
				Step_Cnt_Handler(accDatBackTmpX,accDatBackTmpY,accDatBackTmpZ,
							(SENSOR_DAT_DOWN_SAMPLE_DAT_LEN - (SENSOR_DAT_DOWN_SAMPLE_DAT_LEN/SENSOR_DAT_DOWN_SAMPLE_INTERVAL)),
							TRUE);
			}
			else
			{
				Step_Cnt_Handler(NULL,NULL,NULL,BMA250E_FIFO_LEVEL,FALSE);
			}
            break;

        case gStepClearEvent:                        // Clear step
//            #ifdef MotionInfo_Debug
//				printf("Clear Step\r\n");
//			#endif
//			InitAlgo(3);  
//            // Reset all parameters
//			memset(&step,0,sizeof(step_t));
//            step.eStepStatus        = RUNNING;


//#ifdef STEP_DEBUG
//            printf("Now step count is clear.\r\n");
//#endif
            break;
        
        case gStepStopEvent:                        // Stop step
            BMA250E_Init_Stop();  
		
		
           
#ifdef STEP_DEBUG
            printf("Now step count is stop.\r\n");
#endif
            break;
            
		case gMonitonInfoRecordEvent:    //Motion Record info enent
			
		
			break;
        default:
            break;  
    }   
}
void BMA250EHW_SelfTest(void)
{
	BMA250E_SPI_Oper_Init();
	BMA250E_Int1_Port_Enable();
	bma2x2_init(&bma250e);
	if((bma250e.chip_id) == 0xF9)
	{
		SPI_AlarmTransmit(Alarm_ID_Reuse_BMA250_CORRECT);  
	}
	else
	{
		SPI_AlarmTransmit(Alarm_ID_Reuse_BMA250_ERROR);  
	}
}
void BMA250E_SelfTest(void)
{
	BMA250E_SPI_Oper_Init();
	BMA250E_Int1_Port_Enable();
	bma2x2_init(&bma250e);
	if((bma250e.chip_id) == 0xF9)
	{
		SelfCheckLCDCtl(ID_BMA250EStatus,CMD_Pass);
	}
	else
	{
		SelfCheckLCDCtl(ID_BMA250EStatus,CMD_Fail);
	}
}
void Set_BMA250E_LowPower(void)
{
	BMA250E_SPI_Oper_Init();
	bma2x2_init(&bma250e);
	bma2x2_soft_reset();
    Delay_ms(3); 
    bma2x2_set_mode(BMA2x2_MODE_DEEP_SUSPEND);
	BMA250E_SPI_Oper_DeInit();
}
void StartStepCnt(void)
{
	BMA250E_SPI_Oper_Init();
	BMA250E_Int1_Port_Enable();
	bma2x2_init(&bma250e);
	BMA250E_Init_Step();    
	InitAlgo(3);
	step.eStepStatus = RUNNING;
	step.eStepDisplay = NO_DISPLAY;
	//init_fir_instance(); 
	gflagIsAccSampleRateConvert = FALSE;
}


void Step_Cnt_Handler(int16_t * accDataX,int16_t * accDataY,int16_t * accDataZ, uint16_t numSample, bool flagIsInSpO2MeasureMode)
{
//	uint16_t index = 0;
//	uint16_t blockSize = 0;
	
	/* 	50 is the max number 	*/
//	int16_t fir_instance_src_y[STEP_MAX_PROCESS_NUM] = {0};
//	int16_t fir_instance_src_z[STEP_MAX_PROCESS_NUM] = {0};

//	int16_t fir_filter_result_y[STEP_MAX_PROCESS_NUM] = {0};
//	int16_t fir_filter_result_z[STEP_MAX_PROCESS_NUM] = {0};

//	if(accDataX == NULL || accDataY == NULL || accDataZ == NULL)
//	{
//		/* 	Get data from acc interrupt resource 	*/
//		for(index  = 0;index < BMA250E_FIFO_LEVEL;index ++)
//		{
//			/* 	push into filter buffer 	*/
//			//printf("%d\r\n",p_bma2x2acc[index].y);
//			fir_instance_src_y[index] = p_bma2x2acc[index].y;
//			fir_instance_src_z[index] = (p_bma2x2acc[index].x + p_bma2x2acc[index].z)>>1u;
//		}
//		blockSize = BMA250E_FIFO_LEVEL;
//	}
//	else
//	{
//		/* 	Get data from other procedures 	*/
//		blockSize = (numSample > STEP_MAX_PROCESS_NUM)? STEP_MAX_PROCESS_NUM : numSample;
//		
//		for(index  = 0;index < blockSize ;index ++)
//		{
//			/* 	push into filter buffer 	*/
//			//printf("%d \r\n",accDataY[index]);
//			fir_instance_src_y[index] = accDataY[index];
//			fir_instance_src_z[index] = (accDataX[index] + accDataZ[index])>>1u;
//		}
//		
//		blockSize = numSample;
//	}
	
	/* 	Filter the data 	*/
	//filter_fir_instance(fir_instance_src_y,fir_filter_result_y,fir_instance_src_z,fir_filter_result_z, blockSize);
	
	//processAccelarationData_new(NULL,fir_filter_result_y,fir_filter_result_z,blockSize);
	
//	step.stepCount      = getStepCount();   /* ZHUYUN, 1 bit present 0.5 step 	*/
	
	/* 	Compensate the step lost in spo2 measure mode 		*/
	/* 	The rule is 2 step, add 0.5 step as compensate 		*/
//	if(flagIsInSpO2MeasureMode == TRUE)
//	{
//		if(step.stepCount < (step.stepCount_old << 1u))
//		{
//			step.stepCount = step.stepCount;
//		}
//		else
//		{
//			step.stepCount = step.stepCount + ((step.stepCount - (step.stepCount_old << 1u)) >> 2u);
//			setStepCountCompensate((step.stepCount - (step.stepCount_old << 1u)) >> 2u);
//		}
//	}
	
//	step.stepCount = step.stepCount >> 1u; /* ZHUYUN, 1 bit present 1 step 	*/
	
	//printf("step = %d \r\n",step.stepCount);
//	step.stepActivity   = getActivity();          
	          

    /* Update the distance and calorie (6us)*/
    //distance_calorie(&step);
    
//    if(step.stepCount > 99999)
//    {
//        step.stepCount = 0;
//    }
//    if(step.distance_int > 99999)
//    {
//        step.distance_int = 0;
//    }
//    if(step.calorie_int > 99999)
//    {
//        step.calorie_int = 0;
//    }
        
	
	/* Update the distance and calorie display buffer (10us)*/
	//u32toBCDConvert5Pos(step.stepCount, step.gStepCountArr);
	
	
//	step.distance_int       = (uint32_t)(step.distance >> 14);
//	u32toBCDConvert5Pos(step.distance_int, step.distanceArr);
//	
//	step.calorie_int        = (uint32_t)(step.calorie >> 14); 
//	
//	u32toBCDConvert5Pos(step.calorie_int, step.calorieArr);
	
	
//#ifdef STEP_DEBUG
//	printf("calorie = %f,distance = %f,(step.calorie >> 17) = %d,(step.distance >> 17) = %d.\r\n",
//			(float)step.calorie,(float)step.distance,step.calorie_int,step.distance_int);
//#endif
//	/* Send the event according to the system's state and update the buffer for next cycle (6us)*/
//	
//	if(!step.stepTempEnable)
//	{        
//		if(
//			((Get_OLED_Dis_Status() == OLEDDisEnterShutDown) 
//			|| (Get_OLED_Dis_Status() == OLEDDisON))
//		)
//		{
//			switch(Device_Mode)
//			{
//				case Device_Mode_Step:
//					if(step.stepCount>step.stepCount_old)
//					{ 
//						TS_SendEvent(gOledDisTaskID,gOledDisEventStepCnt_c);
//					}
//					break;
//				
//				case Device_Mode_Distance:
//					if(step.distance > step.distance_old)
//					{ 
//						TS_SendEvent(gOledDisTaskID,gOledDisEventDistanceCnt_c);
//					}
//					break;

//				case Device_Mode_Calorie:
//					if(step.calorie > step.calorie_old)
//					{ 
//						TS_SendEvent(gOledDisTaskID,gOledDisEventCalorieCnt_c);
//					}
//					break;
//				
//				default:
//					break;
//			}
//		}
//  
//		/* Set the BMA250E to suspend mode if the activity is keeping stationary in 10 cycles */
//		if(!step.stepActivity)
//		{
//			step.stepActivity_count ++;
//			if(step.stepActivity_count > STEP_STATIC_COUNT)
//			{
//				step.eStepStatus = SUSPEND;
//				step.stepActivity_count = 0;   
////				BMA250E_Init_Suspend();
//#ifdef STEP_DEBUG
//				printf("-----eStepStatus = SUSPEND-----\r\n");
//#endif                          
//			}
//		}
//		else
//		{
//			step.stepActivity_count = 0;
//		}
//	}
//	else    // Enables temporary count function
//	{
//		stepCount_diff = step.stepCount - step.stepCount_temp;
//		if((Get_OLED_Dis_Status() == OLEDDisEnterShutDown) || (Get_OLED_Dis_Status() == OLEDDisON))
//		{
//			if(stepCount_diff > stepCount_diff_old)
//			{   
//				stepCount_diff_old         =          stepCount_diff;
////				u32toBCDConvert5Pos((uint32_t)stepCount_diff, step.gStepCountArr_temp);
//				TS_SendEvent(gOledDisTaskID,gOledDisEventStepCnt_c);
//			}
//		}
//#ifdef STEP_DEBUG
//		printf("stepTempEnable = %d,step.stepCount_temp = %d.\r\n",step.stepTempEnable,step.stepCount_temp);
//#endif
//	}

//	/* Updates the last step count,distance and calorie value */
//	step.stepCount_old = step.stepCount;
//	step.distance_old = step.distance;
//	step.calorie_old = step.calorie;
//	
}

//static void u32toBCDConvert5Pos(uint32_t DataIn, uint8_t *pDatOut)
//{
//    if(pDatOut != NULL)
//    {
//        pDatOut[0] =         DataIn%10UL;
//        pDatOut[1] =        DataIn%100UL/10UL;
//        pDatOut[2] =       DataIn%1000UL/100UL;
//        pDatOut[3] =      DataIn%10000UL/1000UL;
//        pDatOut[4] =     DataIn%100000UL/10000UL;
//    }
//}
/*******************************************************************************
* Function Name  : u32toBCDConvert
* Description    : 取32位不为0的数，并返回不为0的个数
* Input          : DataIn
* Output         : pDatOut
* Return         : Num
*******************************************************************************/
//uint8_t  u32toBCDConvert(uint32_t DataIn, uint8_t *pDatOut)
//{
//	int8_t i = 0;
//	
//    if(pDatOut != NULL)
//    {
//        pDatOut[0] =         DataIn%10UL;
//        pDatOut[1] =        DataIn%100UL/10UL;
//        pDatOut[2] =       DataIn%1000UL/100UL;
//        pDatOut[3] =      DataIn%10000UL/1000UL;
//        pDatOut[4] =     DataIn%100000UL/10000UL;
//    }
//	
//	/* 	Loop to find the digit number 	*/
//	for(i=4;i>0 && pDatOut[i] != 0;i--);
//	
//	return i+1;
//	
//}


