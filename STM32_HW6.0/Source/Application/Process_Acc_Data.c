/* EasyCASE V6.5 27/05/2009 18:37:02 */
/* EasyCASE O
If=vertical
LevelNumbers=no
LineNumbers=no
Colors=16777215,0,12582912,12632256,0,0,0,16711680,8388736,0,33023,32768,0,0,0,0,0,32768,12632256,255,65280,255,255,16711935
ScreenFont=Courier New,Regular,80,4,-11,0,400,0,0,0,0,0,0,3,2,1,49,96,96
PrinterFont=Arial,,110,4,-88,0,400,0,0,0,0,0,0,3,2,1,34,600,600
LastLevelId=277 */
/* EasyCASE ( 1
   Step Count Algo */
/*  $Date: 2009/05/26 16:05:10 $
 *  $Revision: 1.1 $
 *
 */

/*
* Copyright (C) 2009 Bosch Sensortec GmbH
*
* Pedometer Algorithm For Watches API
*
* Usage:  Application Programming Interface for Pedometer data processing
          
          THE SENSOR SHOULD BE PLACED IN SUCH A WAY THAT Z AXIS IS PERPENDICULAR
          TO WATCH SURFACE AND Y AXIS in 3-9 TIME DIRECTION OF WATCH
*
* Author:       Aibin.Paul@in.bosch.com
*/
/* EasyCASE ( 60
   Disclaimer */
/* Disclaimer
*
* Common:
* Bosch Sensortec products are developed for the consumer goods industry. They may only be used
* within the parameters of the respective valid product data sheet.  Bosch Sensortec products are
* provided with the express understanding that there is no warranty of fitness for a particular purpose.
* They are not fit for use in life-sustaining, safety or security sensitive systems or any system or device
* that may lead to bodily harm or property damage if the system or device malfunctions. In addition,
* Bosch Sensortec products are not fit for use in products which interact with motor vehicle systems.
* The resale and/or use of products are at the purchaser’s own risk and his own responsibility. The
* examination of fitness for the intended use is the sole responsibility of the Purchaser.
*
* The purchaser shall indemnify Bosch Sensortec from all third party claims, including any claims for
* incidental, or consequential damages, arising from any product use not covered by the parameters of
* the respective valid product data sheet or not approved by Bosch Sensortec and reimburse Bosch
* Sensortec for all costs in connection with such claims.
*
* The purchaser must monitor the market for the purchased products, particularly with regard to
* product safety and inform Bosch Sensortec without delay of all security relevant incidents.
*
* Engineering Samples are marked with an asterisk (*) or (e). Samples may vary from the valid
* technical specifications of the product series. They are therefore not intended or fit for resale to third
* parties or for use in end products. Their sole purpose is internal client testing. The testing of an
* engineering sample may in no way replace the testing of a product series. Bosch Sensortec
* assumes no liability for the use of engineering samples. By accepting the engineering samples, the
* Purchaser agrees to indemnify Bosch Sensortec from all claims arising from the use of engineering
* samples.
*
* Special:
* This software module (hereinafter called "Software") and any information on application-sheets
* (hereinafter called "Information") is provided free of charge for the sole purpose to support your
* application work. The Software and Information is subject to the following terms and conditions:
*
* The Software is specifically designed for the exclusive use for Bosch Sensortec products by
* personnel who have special experience and training. Do not use this Software if you do not have the
* proper experience or training.
*
* This Software package is provided `` as is `` and without any expressed or implied warranties,
* including without limitation, the implied warranties of merchantability and fitness for a particular
* purpose.
*
* Bosch Sensortec and their representatives and agents deny any liability for the functional impairment
* of this Software in terms of fitness, performance and safety. Bosch Sensortec and their
* representatives and agents shall not be liable for any direct or indirect damages or injury, except as
* otherwise stipulated in mandatory applicable law.
*
* The Information provided is believed to be accurate and reliable. Bosch Sensortec assumes no
* responsibility for the consequences of use of such Information nor for any infringement of patents or
* other rights of third parties which may result from its use. No license is granted by implication or
* otherwise under any patent or patent rights of Bosch. Specifications mentioned in the Information are
* subject to change without notice.
*
* It is not allowed to deliver the source code of the Software to any third party without permission of
* Bosch Sensortec.
*/
/* EasyCASE ) */
/* EasyCASE ( 59
   File Name For Doxy */
/*! \file Process_Acc_Data.c
    \brief This file contains all function implementations for the pedometer algorithm
    Details.
*/
/* EasyCASE ) */
/* EasyCASE ( 41
   INCLUDES */
/*=============================================================================*/
/****************************Includes*******************************************/
/*=============================================================================*/
#include "Process_Acc_Data.h"
#include "bma2x2.h"
#include "platform.h"
#include <stdio.h>
#include "arm_math.h"
#include "step.h"

/* EasyCASE ) */
/* EasyCASE ( 43
   Declerations And Defenitions */
   
/* EasyCASE ( 57
   DEFINES */
   
/* EasyCASE ) */
/* EasyCASE ( 51
   Variable Decleration */
   
/*==============================================================================*/
/***************************Global Variable Decleration**************************/
/*==============================================================================*/
/**\brief Holds the pedometer activity \n 
*/
static unsigned char V_Activity_U8;

/**\brief holds the mode \n 
*/
//static short    gs_mode;

/**\brief Holds last reset value \n 
*/
static unsigned long gs_stepCount_i32;

static uint32_t gs_stepCountCompensate_i32;	/* 	ZHUYUN 	*/

/**\brief Variable holds the division factor for the threshold 
*/
static unsigned char V_DivideFactor_U8R;

/**\brief Variable holds the Status Flags used 
*/
static unsigned char V_StatusFlags_U8R;

/* \brief Holds the  input values to the filter \n 
*/
//static short gs_CompositeFilter[C_FilterTaps_U8X];

/* \brief Holds the lazy walk lower limit threshold \n 
*/
static unsigned char V_LazyWalkLowerLimit_U8R =0;

/* EasyCASE ( 277
   Y axis processing variables */
/* \brief Holds the Yaxis Peak to peak threshold value \n 
*/
static unsigned short V_YaxisPP_S16R;

/* \brief Holds the Yaxis 2nd previous Peak to peak count \n 
*/
static unsigned char V_YaxisCycleCountPrev1_U8R;

/* \brief Holds the Yaxis  previous Peak to peak count \n
*/
static unsigned char V_YaxisCycleCountPrev_U8R;

/* \brief Holds the Yaxis Peak to peak count \n
*/
static unsigned char V_YaxisCycleCount_U8R;

/* \brief Holds the  input values to the Y filter \n
*/
//static short gs_CompositeFilterY[C_FilterTaps_U8X];

/* EasyCASE ) */
/* EasyCASE ( 276
   Filter Coefficients */
/* EasyCASE < */
/* \brief Holds the Filter coefficients for 0.24 in Q15 format \n
*/
/*LPF FIR filter of 1 HZ*/
//static const short gc_CompositeFilterCoeff[C_FilterTaps_U8X] =
//{
//-767,
//-777,
//-679,
//-463,
//-134,
//298,
//809,
//1368,
//1936,
//2475,
//2943,
//3306,
//3536,
//3615,
//3536,
//3306,
//2943,
//2475,
//1936,
//1368,
//809,
//298,
//-134,
//-463,
//-679,
//-777,
//-767
//};

/**************** New FIR Filter variables ****************/
static arm_fir_instance_q15 arm_fir_instance_y;
static arm_fir_instance_q15 arm_fir_instance_z;

//static q15_t arm_fir_instance_y_State_buffer[C_FilterTaps_U8X + STEP_MAX_PROCESS_NUM];
//static q15_t arm_fir_instance_z_State_buffer[C_FilterTaps_U8X + STEP_MAX_PROCESS_NUM];

//static const short gc_CompositeFilterCoeff[C_FilterTaps_U8X] = {
//      314,    110,    116,    112,     95,     66,     23,    -31,    -96,
//     -168,   -241,   -312,   -372,   -416,   -438,   -430,   -388,   -308,
//     -187,    -26,    174,    408,    669,    951,   1241,   1530,   1806,
//     2058,   2274,   2445,   2563,   2624,   2624,   2563,   2445,   2274,
//     2058,   1806,   1530,   1241,    951,    669,    408,    174,    -26,
//     -187,   -308,   -388,   -430,   -438,   -416,   -372,   -312,   -241,
//     -168,    -96,    -31,     23,     66,     95,    112,    116,    110,
//      314
//};

int16_t gs_FilterResult_y[32];
int16_t gs_FilterResult_z[32];

//static q15_t m_step_cnt_iir_state_buf_lp[GSEN_STEP_CNT_IIR_LP_2_3_Stage_Num*4];
//static q15_t m_step_cnt_iir_state_buf_hp[GSEN_STEP_CNT_IIR_HP_0_2_Stage_Num*4];

/*    ZHU YUN added code here       */
/* IIR Low Pass, 2.2Hz Cut off, paramters, in format of b10,b11,b12,a11,a12,,,, */ 

//const short GSEN_StepCnt_IIR_LP_2_3_Para[6*GSEN_STEP_CNT_IIR_LP_2_3_Stage_Num] = { 
//526 ,0  ,1052 ,526  ,28908  ,-14628 ,
//477 ,0  ,955  ,477  ,26233  ,-11758 ,
//442 ,0  ,885  ,442  ,24319  ,-9705  ,
//420 ,0  ,840  ,420  ,23095  ,-8392  ,
//409 ,0  ,819  ,409  ,22499  ,-7753  

// }; 
/* IIR High Pass, 0.55Hz cut off, paramters, in format of b10,b11,b12,a11,a12,,,, */ 

//const short GSEN_StepCnt_IIR_HP_0_2_Para[6*GSEN_STEP_CNT_IIR_HP_0_2_Stage_Num] = { 
//16171 ,0  ,-32342 ,16171  ,32333  ,-15967 ,
//15885 ,0  ,-31770 ,15885  ,31761  ,-15395 

// }; 
//  
//const int16_t GSEN_StepCnt_IIR_LP_2_3_Para_2[6*GSEN_STEP_CNT_IIR_LP_2_3_Stage_Num] = { 
//113 ,0 ,226 ,113 ,31479 ,-15547 ,
//108 ,0 ,216 ,108 ,30020 ,-14067 ,
//104 ,0 ,208 ,104 ,28882 ,-12912 ,
//101 ,0 ,202 ,101 ,28107 ,-12127 ,
//99 ,0 ,198 ,99 ,27717 ,-11730 
//};

//static arm_biquad_casd_df1_inst_q15 S_StepCnt_lp;
//static arm_biquad_casd_df1_inst_q15 S_StepCnt_hp;
/*     End of ZHU YUN added code   */


/* EasyCASE > */
/* EasyCASE ) */
/* EasyCASE ) */
/* EasyCASE ( 50
   Local Function Decleration */
/*==============================================================================*/
/*********************************Local function Declaration*********************/
/*==============================================================================*/

static void trendFinder(short, short*);
static short getAbsoluteShort(short);
static void YaxisAnalysis(short);
/* EasyCASE ) */
/* EasyCASE ) */
/* EasyCASE ( 46
   APIS */
/* EasyCASE ( 3
   getStepCount */
/* EasyCASE F */
/**
  \brief This function will return the number of steps counted to the calling function.
  \param None
  \return Number of steps counted
*/

///*    ZHU YUN added code here     */
///*    Gsensor Band Pass filter init func  */
//void \(void)
//{
//  /*  Step1, HP filter */
//  arm_biquad_cascade_df1_init_q15(&S_StepCnt_hp, GSEN_STEP_CNT_IIR_HP_0_2_Stage_Num, (q15_t*)GSEN_StepCnt_IIR_HP_0_2_Para, m_step_cnt_iir_state_buf_hp, GSEN_STEP_CNT_IIR_LP_2_3_Shift_Num);
//  
//  /*  Step2, IIR LP filter    */
//  arm_biquad_cascade_df1_init_q15(&S_StepCnt_lp, GSEN_STEP_CNT_IIR_LP_2_3_Stage_Num, (q15_t*)GSEN_StepCnt_IIR_LP_2_3_Para, m_step_cnt_iir_state_buf_lp, GSEN_STEP_CNT_IIR_HP_0_2_Shift_Num);

//}

///*    Gsensor Band Pass filter func     */
//void gSensor_Step_Cnt_BP_Filter(short * pDatIn, short * pDatOut)
//{
//  short   dat_hp_filt[GSEN_STEP_CNT_IIR_FILT_BLK_SIZE];
//  
//  /*  Step1, HP filter */
//  arm_biquad_cascade_df1_q15(&S_StepCnt_hp, pDatIn, dat_hp_filt,GSEN_STEP_CNT_IIR_FILT_BLK_SIZE);
//  
//  /*  Step2, IIR LP filter    */
//  arm_biquad_cascade_df1_q15(&S_StepCnt_lp, dat_hp_filt, pDatOut ,GSEN_STEP_CNT_IIR_FILT_BLK_SIZE);
//}

/*  End of ZHU YUN added code     */


//void init_fir_instance(void)
//{
//    arm_fir_init_q15(&arm_fir_instance_y,C_FilterTaps_U8X,(q15_t*)gc_CompositeFilterCoeff,(q15_t*)arm_fir_instance_y_State_buffer,STEP_MAX_PROCESS_NUM);
//    arm_fir_init_q15(&arm_fir_instance_z,C_FilterTaps_U8X,(q15_t*)gc_CompositeFilterCoeff,(q15_t*)arm_fir_instance_z_State_buffer,STEP_MAX_PROCESS_NUM);
//}

void filter_fir_instance(int16_t * pSrcY,int16_t * pDstY, 
						int16_t * pSrcZ,int16_t * pDstZ,
					uint16_t blockSize)
{
    arm_fir_q15(&arm_fir_instance_y,(q15_t *)pSrcY, pDstY,blockSize);
    arm_fir_q15(&arm_fir_instance_z,(q15_t *)pSrcZ, pDstZ,blockSize);
}


void setStepCountCompensate(uint32_t newStepCnt)
{
    gs_stepCountCompensate_i32 += newStepCnt;
	//printf("comp = %d\r\n",gs_stepCountCompensate_i32);
}

unsigned long getStepCount()
{
    /*
    **
    *********************************************************************************
     Function Name : unsigned long getStepCount()
     Input         : None
     Output        : Number of step counted
     Scope         : Global

     Detailed Description:
       This function will return gs_stepCount_i32 the number of steps counted to the
       calling function
    *********************************************************************************
    **
    */
    /* Return the number of steps counted */
    return (( gs_stepCount_i32 << 2u) + gs_stepCountCompensate_i32);
}

/* EasyCASE ) */
/* EasyCASE ( 225
   stopDetection */
/* EasyCASE F */
/**
  \brief This function will put the Pedometer Algorithm to Sleep mode.
  \param None
  \return None
*/
void stopDetection()
{
    /*
    **
    *********************************************************************************
     Function Name   : void stopDetection()
     Input           : None
     Output          : None
     Scope           : Global
     
     Detailed Description:
       This function will set the Mode of the pedometer gs_mode as MODE_SLEEPING
    *********************************************************************************
    **
    */ 
    /*All flags except Robustness Flag resetted*/    
    V_StatusFlags_U8R = (V_StatusFlags_U8R & M_DisableRobustness_U8X);
}

/* EasyCASE ) */
/* EasyCASE ( 226
   startDetection */
/* EasyCASE F */
/**
  \brief This function will start the Pedometer Algorithm if pedometer is in 
  sleep mode. If Pedometer Algorithm is running already, It will be restarted by
  this function and step count will be reset.By default the normal human activity
  detection is enabled
  \param None
  \return None
*/
void startDetection()
{
    /*
    **
    *********************************************************************************
    Function Name : void startDetection()

    Input         : None
    Output        : None
    Scope         : Global

    Detailed Description:
    This function will set the mode as detection mode. If this function is called
    in between the program this function will reset all the step count values.  
    *********************************************************************************
    **
    */       
    /*All flags except robustness cleared*/
    V_StatusFlags_U8R = (V_StatusFlags_U8R & M_DisableRobustness_U8X);

    /* Set the Previous data count as 1 */
    V_StatusFlags_U8R |= M_AlgoReset_U8X;

    /* Set the mode as Detection Mode */
    V_StatusFlags_U8R |= M_ModeDetection_U8X;
}
   
/* EasyCASE ) */
/* EasyCASE ( 232
   resetStepCount */
/* EasyCASE F */
/**
  \brief Thsi API will just reset the step count to zero. Pedometer Algorithm mode
  is not affected by this function.
  \param  None
  \return None
*/
void resetStepCount()
{
    /*
    **
    *********************************************************************************
    Function Name   : void resetStepCount()
    Input           : None
    Output          : None
    Scope           : Global

    Detailed Description:
    Thsi API will reset the step count gs_stepCount_i32 to zero and set the 
    gs_resetPrevData previous data count as 1
    *********************************************************************************
    **
    */ 
    /* Reset the number of step counted to 0 */
    gs_stepCount_i32 = C_Clear_U8X;
	
	gs_stepCountCompensate_i32 = C_Clear_U8X;

    /*All flags except Robustness resetted*/
    V_StatusFlags_U8R = (V_StatusFlags_U8R & M_DisableRobustness_U8X);

    /* Set the previous data as 1 */
    V_StatusFlags_U8R|=(M_AlgoReset_U8X|M_ModeDetection_U8X);
}

/* EasyCASE ) */
/* EasyCASE ( 8
   processAccelarationData */
/* EasyCASE F */
/**
  \brief  This API calculates Composite value from  X,Y,Z Axis acceleration
  values and updates the step count. It also classifies the type of step
  whether it is a Jog/Walk/Slow Walk step. This function returns the calculated
  composite value.
  \param short f_x_i16 : X-Axis acceleration value from the sensor \n
         short f_y_i16 : Y-Axis acceleration value from the sensor \n
         short f_z_i16 : Z-Axis acceleration value from the sensor \n
  \return Composite value
*/
/*
**********************************************************************************************
  Function Name   : short processAccelarationData(short f_x_i16, short f_y_i16, short f_z_i16)
  Input           : short f_x_i16
                    short f_y_i16
                    short f_z_i16
  Output          : Composite value
  Scope           : Global

  Detailed description :
    Variables:
        short f_x_i16 : X-Axis acceleration value from the sensor
        short f_y_i16 : Y-Axis acceleration value from the sensor
        short f_z_i16 : Z-Axis acceleration value from the sensor

  Description:
        This API gets the X,Y,Z Axis values from the sensor and calculates the
        composite and the average composite value. This API also takes care whether
        the step is a walking step, jogging step , slow walking step  and
         stores the corresponding step count values.
         
         THE SENSOR SHOULD BE PLACED IN SUCH A WAY THAT Z AXIS IS PERPENDICULAR
         TO WATCH SURFACE
         
*********************************************************************************************
**
*/

void processAccelarationData_new(int16_t * pDataInputX,int16_t * pDataInputY,int16_t * pDataInputZ,uint16_t numSample)
{
    uint16_t index;
	
    for(index = 0;index < numSample;index++)
    {
		//printf("%d %d\r\n",pDataInputY[index],pDataInputZ[index]);
        processAccelarationData(0, pDataInputY[index], pDataInputZ[index]);
    }  
}

short processAccelarationData(short f_x_i16, short f_y_i16, short f_z_i16)
{
    /* EasyCASE ( 23
      Variable Declerations */
    /* Holds the steps counted while not in counting */
    static char     s_interStepCount_i8=0;
       
    /* 0 if there is no correction. correction value if there needs to be a correction */
    static char     s_correctionCount_i8=0;
       
    /* Holds the composite high for the step cycle */
    static short    s_resultHigh_i16=0x7FFF;
       
    /* Holds the sample counts between trend changes */
    static char    s_sampleCountForCycle_i16=0;
       
    /* Holds the time count between second last & last step */
    static char    s_countStepToStepPrev1_i16=0;
       
    /* Holds the time count between third last & second last step */
    static char     s_countStepToStepPrev2_i16=0;
       
    /* Holds the time count between last & this step */
    static char    gs_countStepToStep_i16=0;

    /*Holds error count between 2 steps*/
    static char v_ErrorCount_u8=0;

    /*Holds the count between two consecutive counts*/
    static char s_ModeTimer_i8=0;

    /*Holds prev Result Diff*/
    static short v_PreResultDiff_s16r=0;

    /*Holds the previous activity type*/
    static char v_PreviousActivity_u8=0;

    /* Initialise the composite value to 0 */
    short  composite_i16;

    /*Store the previous PP value from trend finder*/
    static short v_PreTrendPeakDiff_s16r=0;

    /* Initialise the filter result value to 0 */
//    long FilterResult_l32;
   
    /*brief Used to hold index for filtering \n*/
//    char gs_FilterIndex;
    
    /*The previous status are stored*/
    static char v_PreStatusFlags_u8r=0;
    
    /*Holds the Filtered Y axis value*/
    short v_FilteredY_s16r=0;
    
    /* EasyCASE ) */
    /* EasyCASE ( 24
      Mode is sleeping? */
    /* Check whether the mode is sleeping mode */
    if (V_StatusFlags_U8R & (M_ModeDetection_U8X|M_ModeCounting_U8X))
    {
    }
    else
    {
        /* Return 0 */
        return 0;
    }
      
    /* EasyCASE ) */
    /* EasyCASE ( 25
      Reset Previous data */
    if (V_StatusFlags_U8R & M_AlgoReset_U8X)
    {
        /* Set the reset the previous data as 0 */
        V_StatusFlags_U8R &= ~M_AlgoReset_U8X;
          
        /* Holds the steps counted while not in counting */
        s_interStepCount_i8=C_Clear_U8X;
          
        /* 0 if there is no correction. correction value if there needs to be a correction */
        s_correctionCount_i8=C_Clear_U8X;
          
        /* Holds the composite high for the step cycle */
        s_resultHigh_i16=0x7FFF;
          
        /* Holds the sample counts between trend changes */
        s_sampleCountForCycle_i16=C_Clear_U8X;
          
        /* Holds the time count between second last & last step */
        s_countStepToStepPrev1_i16=C_Clear_U8X;
          
        /* Holds the time count between third last & second last step */
        s_countStepToStepPrev2_i16=C_Clear_U8X;
          
        /* Holds the time count between last & this step */
        gs_countStepToStep_i16=C_Clear_U8X;

        /*Holds the previous activity type*/
        v_PreviousActivity_u8=C_Clear_U8X;

        /*Holds error count between 2 steps*/
        v_ErrorCount_u8=C_Clear_U8X;

        /*Holds the count between two consecutive counts*/
        s_ModeTimer_i8=C_Clear_U8X;

        /*Holds prev Result Diff*/
        v_PreResultDiff_s16r=C_Clear_U8X;
        v_PreTrendPeakDiff_s16r=C_Clear_U8X;

        /*The variable holding filter result are cleared*/
//        FilterResult_l32=C_Clear_U8X;

        /* EasyCASE - */
        /*  Holds the Yaxis Peak to peak threshold value cleared*/
        V_YaxisPP_S16R=0;

        /*  Holds the Yaxis 2nd previous Peak to peak count cleared*/
        V_YaxisCycleCountPrev1_U8R=0;

        /* Holds the Yaxis  previous Peak to peak count cleared*/
        V_YaxisCycleCountPrev_U8R=0;

        /*Holds the Yaxis Peak to peak count cleared*/
        V_YaxisCycleCount_U8R=0;
//        for (gs_FilterIndex=0;gs_FilterIndex<C_FilterTaps_U8X;gs_FilterIndex++)
//        {
//            gs_CompositeFilter[gs_FilterIndex] = C_Clear_U8X;
//            gs_CompositeFilterY[gs_FilterIndex] = C_Clear_U8X;
//        }
    }
      
    /* EasyCASE ) */
    /* EasyCASE ( 213
      Change in Robstness feature */
    /*Check whether robustness feature status changed*/
    if (((V_StatusFlags_U8R & M_DisableRobustness_U8X) != (v_PreStatusFlags_u8r & M_DisableRobustness_U8X)))
    {
        /*Robustness feature got changed So Clear Temporary counts */
        s_correctionCount_i8=0;
        s_interStepCount_i8=0;
        V_StatusFlags_U8R &=~(M_Qualified_U8X|M_UnQualified_U8X);
        s_ModeTimer_i8 = 0;
        V_StatusFlags_U8R |=M_ModeDetection_U8X;
        V_StatusFlags_U8R &=~M_ModeCounting_U8X;
    }
      
    /* EasyCASE ) */
    /* EasyCASE ( 118
        Counts And Composite Calculation */
    /* Increment the cycle and step to step counters */
    s_sampleCountForCycle_i16++;

    /* Increment the step to step count */
    gs_countStepToStep_i16++;

    /* Calculate the composite acceleration component value */
    composite_i16 = (f_z_i16);

    /* EasyCASE - */
    /*Increment Count Mode Elapse Timer*/
    s_ModeTimer_i8++;
      
   /* EasyCASE ( 272
      Buffer Value updation */
//   for (gs_FilterIndex=C_FilterTaps_U8X-C_CountOne_U8X;gs_FilterIndex>C_CountZero_U8X;gs_FilterIndex--)
//      {
//      gs_CompositeFilter[gs_FilterIndex] = gs_CompositeFilter[gs_FilterIndex-C_CountOne_U8X];
//      gs_CompositeFilterY[gs_FilterIndex] = gs_CompositeFilterY[gs_FilterIndex-C_CountOne_U8X];
//      }
//   gs_CompositeFilter[C_CountZero_U8X] = composite_i16;
//   gs_CompositeFilterY[C_CountZero_U8X] = f_y_i16;
    /* EasyCASE ) */
    /* EasyCASE ( 273
      Z axis filtering */
    /*The variable get cleared*/
//   FilterResult_l32=C_Clear_U8X;

//   for (gs_FilterIndex=C_CountZero_U8X;gs_FilterIndex<C_FilterTaps_U8X;gs_FilterIndex++)
//      {
//      FilterResult_l32 = FilterResult_l32 + (long)((long)gs_CompositeFilter[(C_FilterTaps_U8X-C_CountOne_U8X)-gs_FilterIndex]
//      *(long)gc_CompositeFilterCoeff[gs_FilterIndex]);
//      }
      
   /* Divide by 32768 to compensate Q15 format multiplication.*/
//   composite_i16 = (short)(FilterResult_l32>>C_Q15ConversionFactor_U8X);
    composite_i16 = f_z_i16;
   
    /* EasyCASE ) */
    /* EasyCASE ( 274
      Y axis Filtering */
   /*The variable get cleared*/
//   FilterResult_l32=C_Clear_U8X;
//    for (gs_FilterIndex=C_CountZero_U8X;gs_FilterIndex<C_FilterTaps_U8X;gs_FilterIndex++)
//    {
//        FilterResult_l32 = FilterResult_l32 + (long)((long)gs_CompositeFilterY[(C_FilterTaps_U8X-C_CountOne_U8X)-gs_FilterIndex]
//        *(long)gc_CompositeFilterCoeff[gs_FilterIndex]);
//    }

    /* Divide by 32768 to compensate Q15 format multiplication.*/
    v_FilteredY_s16r = f_y_i16;

    /* EasyCASE ) */
    /* Analyse the current Z axis wave for trend calculation */
    trendFinder(composite_i16, &composite_i16);

    /*Y axis analysis.This will find the peak to peak threshold and count of each step*/
    YaxisAnalysis(v_FilteredY_s16r);
    
    /* EasyCASE ) */
    /* EasyCASE ( 16
        Human Activity */
    /* Check whether the trend is positive or negative */
    if (V_StatusFlags_U8R & M_NegativeTrend_U8X)
    {
        /* Acceleration for step start */
        s_resultHigh_i16 = composite_i16;
    }
    else
    {
        /*Check whether negative peak reached*/
        if ((V_StatusFlags_U8R & M_PositiveTrend_U8X))
        {
            /* EasyCASE ( 248 Time out in Detection */
            if ((s_ModeTimer_i8>C_DetectionModeTimeOut_U8X)&&( V_StatusFlags_U8R&M_ModeDetection_U8X))/* No steps for 2.8 seconds in Detection Mode */
            {
                s_ModeTimer_i8=C_Clear_U8X;
                if (((V_StatusFlags_U8R & M_DisableRobustness_U8X)==0))
                {
                    /* No activity in detection mode; so clear the Temporary step count*/
                    s_correctionCount_i8=C_Clear_U8X;
                    s_interStepCount_i8=C_Clear_U8X;
                    V_StatusFlags_U8R &=~(M_Qualified_U8X|M_UnQualified_U8X);
                    v_ErrorCount_u8=C_Clear_U8X;
                }
            }
            
            /* EasyCASE ) */
            /* EasyCASE ( 249 
            			Time out in Counting */
            if ((s_ModeTimer_i8>C_CountingModeTimeOut_U8X)&&( V_StatusFlags_U8R&M_ModeCounting_U8X))/* No steps for 4 seconds in Count Mode*/
            {
                s_ModeTimer_i8=C_Clear_U8X;
                if (((V_StatusFlags_U8R & M_DisableRobustness_U8X)==0))
                {
                    /* No activity in counting mode; so clear the Temporary step count*/
                    s_correctionCount_i8=C_Clear_U8X;
                    V_StatusFlags_U8R &=~(M_Qualified_U8X|M_UnQualified_U8X);
                    v_ErrorCount_u8=C_Clear_U8X;
                    V_StatusFlags_U8R|=M_ModeDetection_U8X;
                    V_StatusFlags_U8R&=~M_ModeCounting_U8X;
                }
            }
			
            /* EasyCASE ) */
            /*Clearing the status byte for qualified and unqualified bits*/
            V_StatusFlags_U8R &=~(M_Qualified_U8X|M_UnQualified_U8X);
            
            /* EasyCASE - */
            /*The current peak obtained is checked with previous peak value and made sure that peak obtained is greater than half of previous peak.
            		    Also the Y axis peak is checked whether it is more than JOG limit */
			
            //if (((s_resultHigh_i16 - composite_i16)>(v_PreTrendPeakDiff_s16r>>1))||(V_YaxisPP_S16R>=JOG_LOWER_LIMIT))
            if (((s_resultHigh_i16 - composite_i16)>(v_PreTrendPeakDiff_s16r>>2))||(V_YaxisPP_S16R>=WALKING_LOWER_LIMIT))
            {
                /*Peak to peak value stored*/
                composite_i16 = s_resultHigh_i16 - composite_i16;
                #ifdef STEP_CNT_ALGORITHM_DEBUG
					printf("composite = %d \r\n", composite_i16);
				#endif
				
                /* Clear the Activity */
                V_Activity_U8 = C_Clear_U8X;
                
                /* Check whether the step is valid or not */
                /* EasyCASE ( 14 
                			Lazy Walk */
                /* Check whether the step is lazy walk step */
                if (IS_LAZY_WALK_STEP(composite_i16))
                {
                    if (((getAbsoluteShort(gs_countStepToStep_i16 -  s_countStepToStepPrev1_i16) <= MAX_COUNT_DIFFERENCE) ||
                        ( getAbsoluteShort(gs_countStepToStep_i16 -  s_countStepToStepPrev2_i16) <=MAX_COUNT_DIFFERENCE) )&&
                        ((gs_countStepToStep_i16 > C_LazyWalkStepToStepLowerLimit_U8X) && 
                        (gs_countStepToStep_i16 < C_LazyWalkStepToStepUpperLimit_U8X)))
                    {
						#ifdef STEP_CNT_ALGORITHM_DEBUG
							printf("lazy walk \r\n");
						#endif
                        V_StatusFlags_U8R |= M_Qualified_U8X;
                    }
                    else
                    {
                        V_StatusFlags_U8R |= M_UnQualified_U8X;
                    }

                    /* Slow activity */
                    V_Activity_U8 |= M_SlowWalk_U8X;
                }
                
                /* EasyCASE ) */
                /* EasyCASE ( 11
                			Walk */
                else if (IS_WALK_STEP(composite_i16))
                {
                    if (((getAbsoluteShort(gs_countStepToStep_i16 -  s_countStepToStepPrev1_i16) <= MAX_COUNT_DIFFERENCE) ||
                        ( getAbsoluteShort(gs_countStepToStep_i16 -  s_countStepToStepPrev2_i16) <=MAX_COUNT_DIFFERENCE) )&&
                        ((gs_countStepToStep_i16 > C_WalkStepToStepLowerLimit_U8X) && 
                        ( gs_countStepToStep_i16 < C_WalkStepToStepUpperLimit_U8X)))
                    {
						#ifdef STEP_CNT_ALGORITHM_DEBUG
							printf("walk \r\n");
						#endif
                        V_StatusFlags_U8R |= M_Qualified_U8X;
                    }
                    else
                    {
                        V_StatusFlags_U8R |= M_UnQualified_U8X;
                    }
                    
                    /* Medium activity */
                    V_Activity_U8 |= M_Walk_U8X;
                }
                
                /* EasyCASE ) */
                /* EasyCASE ( 12
                Jog */
                /* Check whether the step is Jog step */
                else if (IS_JOG_STEP(composite_i16))
                {
                    if (((getAbsoluteShort(gs_countStepToStep_i16 -  s_countStepToStepPrev1_i16) <= MAX_COUNT_DIFFERENCE) ||
                        ( getAbsoluteShort(gs_countStepToStep_i16 -  s_countStepToStepPrev2_i16) <=MAX_COUNT_DIFFERENCE) )&&
                        ((gs_countStepToStep_i16 > C_JogStepToStepLowerLimit_U8X) && 
                        ( gs_countStepToStep_i16 < C_JogStepToStepUpperLimit_U8X)))
                    {
						#ifdef STEP_CNT_ALGORITHM_DEBUG
							printf("jog walk \r\n");
						#endif
                        V_StatusFlags_U8R |= M_Qualified_U8X;
                    }
                    else
                    {
                        V_StatusFlags_U8R |= M_UnQualified_U8X;
                    }
                    
                    /* Brisk activity */
                    V_Activity_U8 |= M_Jog_U8X;
                }
                else
                {
                    /*When no move is not detected in Z axis,check whether JOG detected in Y axis*/
                    /* EasyCASE ( 275
                      Y axis analysis */
                    if (IS_LAZY_WALK_STEPY(V_YaxisPP_S16R))
                    {
                        if (((getAbsoluteShort(V_YaxisCycleCount_U8R -  V_YaxisCycleCountPrev_U8R) <= MAX_COUNT_DIFFERENCE) ||
                            ( getAbsoluteShort(V_YaxisCycleCount_U8R -  V_YaxisCycleCountPrev1_U8R) <=MAX_COUNT_DIFFERENCE) )&&
                            ((V_YaxisCycleCount_U8R > C_LazyWalkStepToStepLowerLimit_U8X) && 
                            (V_YaxisCycleCount_U8R < C_LazyWalkStepToStepUpperLimit_U8X)))
                        {
						#ifdef STEP_CNT_ALGORITHM_DEBUG
							printf("Y lazy walk \r\n");
						#endif
                            V_StatusFlags_U8R |= M_Qualified_U8X;
                        }
                        else
                        {
                            V_StatusFlags_U8R |= M_UnQualified_U8X;
                        }
                        
                        /* Brisk activity */
                        V_Activity_U8 |= M_SlowWalk_U8X;
                            
                        /* EasyCASE - */
                        /*Updating Peak to peak value with Y axis value for future use in activity monitor*/
                        composite_i16=V_YaxisPP_S16R;
						
                        /*Peak to peak threshold values of Y axis are cleared after using*/
                        V_YaxisPP_S16R=0;
                    }
					
					
                    else if (IS_WALK_STEPY(V_YaxisPP_S16R))
                    {
                        if (((getAbsoluteShort(V_YaxisCycleCount_U8R -  V_YaxisCycleCountPrev_U8R) <= MAX_COUNT_DIFFERENCE) ||
                            ( getAbsoluteShort(V_YaxisCycleCount_U8R -  V_YaxisCycleCountPrev1_U8R) <=MAX_COUNT_DIFFERENCE) )&&
                            ((V_YaxisCycleCount_U8R > C_WalkStepToStepLowerLimit_U8X) && 
                            (V_YaxisCycleCount_U8R < C_WalkStepToStepUpperLimit_U8X)))
                        {
						#ifdef STEP_CNT_ALGORITHM_DEBUG
							printf("Y walk \r\n");
						#endif
                            V_StatusFlags_U8R |= M_Qualified_U8X;
                        }
                        else
                        {
                            V_StatusFlags_U8R |= M_UnQualified_U8X;
                        }
                        
                        /* Brisk activity */
                        V_Activity_U8 |= M_Walk_U8X;
                            
                        /* EasyCASE - */
                        /*Updating Peak to peak value with Y axis value for future use in activity monitor*/
                        composite_i16=V_YaxisPP_S16R;
						
                        /*Peak to peak threshold values of Y axis are cleared after using*/
                        V_YaxisPP_S16R=0;
                    }
					
                    else if (IS_JOG_STEPY(V_YaxisPP_S16R))
                    {
                        if (((getAbsoluteShort(V_YaxisCycleCount_U8R -  V_YaxisCycleCountPrev_U8R) <= MAX_COUNT_DIFFERENCE) ||
                            ( getAbsoluteShort(V_YaxisCycleCount_U8R -  V_YaxisCycleCountPrev1_U8R) <=MAX_COUNT_DIFFERENCE) )&&
                            ((V_YaxisCycleCount_U8R > C_JogStepToStepLowerLimit_U8X) && 
                            (V_YaxisCycleCount_U8R < C_JogStepToStepUpperLimit_U8X)))
                        {
						#ifdef STEP_CNT_ALGORITHM_DEBUG
							printf("Y jog walk \r\n");
						#endif
                            V_StatusFlags_U8R |= M_Qualified_U8X;
                        }
                        else
                        {
                            V_StatusFlags_U8R |= M_UnQualified_U8X;
                        }
                        
                        /* Brisk activity */
                        V_Activity_U8 |= M_Jog_U8X;
                            
                        /* EasyCASE - */
                        /*Updating Peak to peak value with Y axis value for future use in activity monitor*/
                        composite_i16=V_YaxisPP_S16R;
						
                        /*Peak to peak threshold values of Y axis are cleared after using*/
                        V_YaxisPP_S16R=0;
                    }
					
                    /* EasyCASE ) */
                }
                
                /* EasyCASE ) */
                /* EasyCASE ( 15
                   Step counting */
                /*Check whether step is valid or not*/
                if (((V_StatusFlags_U8R & (M_Qualified_U8X|M_UnQualified_U8X))!=0))
                {
                    /*If there is change in activity and the current result diff are
                    greater than certain "Threshold" then  temporary counts are cleared .
                    Threshold=(Largest of Current Result Diff and previous Result Diff)/2 .
                    This is applicable in detection mode*/
                    /* EasyCASE ( 220
                      Activity Monitor */
                    if (((v_PreviousActivity_u8 & V_Activity_U8)==C_Clear_U8X) &&
                       (v_PreviousActivity_u8 !=C_Clear_U8X) &&(v_PreResultDiff_s16r!=C_Clear_U8X)&&
                       ( V_StatusFlags_U8R&M_ModeDetection_U8X)&&
                       (((getAbsoluteShort(composite_i16-v_PreResultDiff_s16r))<<C_CountOne_U8X)>
                       ((composite_i16>v_PreResultDiff_s16r)?composite_i16:v_PreResultDiff_s16r))
                       &&((V_StatusFlags_U8R & M_DisableRobustness_U8X)==0))
                    {
                        /* Activities differ in Detection state;  So clear
                        the temporary step count*/
                        s_interStepCount_i8 = C_Clear_U8X;
                        s_correctionCount_i8=C_Clear_U8X;
                        V_StatusFlags_U8R &=~(M_Qualified_U8X|M_UnQualified_U8X);
                    }
                    
                    /* EasyCASE ) */
                    /*Stores the current Activity Type*/
                    v_PreviousActivity_u8=V_Activity_U8;
                        
                    /*Stores the current result Diff*/
                    v_PreResultDiff_s16r=composite_i16;
                        
                    /*Error count cleared*/
                    v_ErrorCount_u8=C_Clear_U8X;
                        
                    /*Reset the Mode Timer*/
                    s_ModeTimer_i8=C_Clear_U8X;
                    
                    /* EasyCASE - */
                    /* Check whether the  step is Qualified */
                    if (V_StatusFlags_U8R & M_Qualified_U8X)
                    {
                        /* Check whether the mode is counting mode */
                        if (V_StatusFlags_U8R&M_ModeCounting_U8X)
                        {
                            /* EasyCASE ( 222 Counting Mode */
                            /*Check whether correction count >3 in CountingMode*/
                            if (s_correctionCount_i8>=C_CorrectionCountLimit_U8X)
                            {
                                /* Add the step count with Correction count */
                                gs_stepCount_i32+=(s_correctionCount_i8+C_CountOne_U8X);
                                /* Reset the correction counter */
                                s_correctionCount_i8 = C_Clear_U8X;
                            }
                            else
                            {
                                /* Increment the step count */
                                gs_stepCount_i32 ++; 
                            }
                            /* EasyCASE ) */
                        }
                        else
                        {
                            /*Check whether current mode is Detection Mode*/
                            /* EasyCASE ( 223 Detection Mode */
                            if (V_StatusFlags_U8R&M_ModeDetection_U8X)
                            {
                                /*Correction count is added to interstep count
                                when correction count >3 in detection mode*/
                                if (s_correctionCount_i8>2*C_CorrectionCountLimit_U8X)
                                {
                                    /* Increment the step count */
                                    s_interStepCount_i8+=(s_correctionCount_i8+1);
                                    
                                    /* Reset the correction counter */
                                    s_correctionCount_i8 = C_Clear_U8X;
                                }
                                else
                                {
                                    /* Increment the step count */
                                    s_interStepCount_i8++;
                                }
                                
                                /*When interstep count > 9 mode changed to counting in case if Robustness feature enabled
                                When interstep count > 3 mode changed to counting in case if Robustness feature disabled*/
                                if (((s_interStepCount_i8 > C_InterStepCountLimit_U8X)&&
                                    ((V_StatusFlags_U8R & M_DisableRobustness_U8X)==0))||
                                    ((s_interStepCount_i8 > C_InterStepCountLimitNoRobustness) && 
                                    ((V_StatusFlags_U8R & M_DisableRobustness_U8X) == M_DisableRobustness_U8X)))
                                {
                                    /* Set the mode to MODE_COUNTING */
                                    V_StatusFlags_U8R|=M_ModeCounting_U8X;
                                    V_StatusFlags_U8R&=~M_ModeDetection_U8X;
                                        
                                    /* Increment the step */
                                    gs_stepCount_i32+=(s_interStepCount_i8+s_correctionCount_i8);
                                        
                                    /* Reset the interstep counter */
                                    s_interStepCount_i8 = C_Clear_U8X;
                                        
                                    /* Reset the correction counter */
                                    s_correctionCount_i8 = C_Clear_U8X;
                                }
                            }
                            /* EasyCASE ) */
                        }
                    }
                    else
                    {
                        /* EasyCASE ( 221
                            Correction Count */
                        /*Check whether Step is unqualified*/
                        if (V_StatusFlags_U8R & M_UnQualified_U8X)
                        {
                            /* Increment the correction count */
                            s_correctionCount_i8++;
                        }
                        /* EasyCASE ) */
                    }
                }
                
                /* EasyCASE ( 224
                    Step to Step Count Updation */
                /*Count step to step is updated if the trend change is not due to noise*/
                if (composite_i16>C_SensorNoiseLevel_U8X)
                {
                    /* Update the last, secondlast and thridlast count variables */
                    s_countStepToStepPrev2_i16 = s_countStepToStepPrev1_i16;
                    s_countStepToStepPrev1_i16 = gs_countStepToStep_i16;
                    #ifdef TEST
                    V_CountSteptoStep_U16R=gs_countStepToStep_i16;
                    #endif
                    gs_countStepToStep_i16 = C_Clear_U8X;
                }
                
                /* EasyCASE ) */
                /* EasyCASE ) */
                /* Reset the sample count for cycle */
                s_sampleCountForCycle_i16 = C_Clear_U8X;
                
                /* EasyCASE - */
                v_PreTrendPeakDiff_s16r=composite_i16;
            }
            else
            {
                v_PreTrendPeakDiff_s16r=s_resultHigh_i16 - composite_i16;
            }
            
            /* EasyCASE ( 253
                Error Count */
            /*Error count is incremented if the step is notvalid and not due to noise*/
             /*Check whether step is valid or not*/
            if (((V_StatusFlags_U8R & (M_Qualified_U8X|M_UnQualified_U8X))==0)&&(composite_i16>C_SensorNoiseLevel_U8X))
            {
                /*Error count is incremented*/
                v_ErrorCount_u8++;
            }
            
            /*When the error count becomes greater than 3 the
             temporary counts are cleared*/
            if (v_ErrorCount_u8>C_ErrorCountLimit_U8X)
            {
                /*The mode changed to detection and counts are cleared*/
                V_StatusFlags_U8R|=M_ModeDetection_U8X;
                V_StatusFlags_U8R&=~M_ModeCounting_U8X;
                v_ErrorCount_u8=C_Clear_U8X;
                s_correctionCount_i8=C_Clear_U8X;
                s_interStepCount_i8=C_Clear_U8X;
                V_StatusFlags_U8R &=~(M_Qualified_U8X|M_UnQualified_U8X);
                s_ModeTimer_i8 = C_Clear_U8X;
            }
            /* EasyCASE ) */
        }
    }
    /* EasyCASE ) */
    /*Current status are stored*/
    v_PreStatusFlags_u8r=V_StatusFlags_U8R;
    
    /* return the composite value */
    return composite_i16;
}

/* EasyCASE ) */
/* EasyCASE ( 9
   getActivity */
/* EasyCASE F */
/**
  \brief This function will Return the nature of the step \n
   Whether the step is of Jogg(0x12)/Walk(0x11)/Slow Walk(0x10) nature\n
  \param None
  \return Step Nature
*/
unsigned char getActivity(void)
{
    /*
    **
    **********************************************************************************
     Function Name : unsigned char getActivity(void)
     Input         : None
     Output        : None

     Detailed Description:
       This function will Return the Activity of the Pedometer.
    **********************************************************************************
    **
    */
    char v_Activity_u8r;
    if (V_Activity_U8&M_Walk_U8X)
    {
        /*Current activity is walk*/
        v_Activity_u8r=0x11;
    }
    else
    {
        if (V_Activity_U8&M_SlowWalk_U8X)
        {
            /*Current activity is slow walk*/
            v_Activity_u8r=0x10;
        }
        else
        {
            if (V_Activity_U8&M_Jog_U8X)
            {
                /*Current activity is jog*/
                v_Activity_u8r=0x12;
            }
            else
            {
                v_Activity_u8r=C_Clear_U8X;
            }
        }
    }
    return (v_Activity_u8r);
}

/* EasyCASE ) */
/* EasyCASE ( 94
   InitAlgo */
/* EasyCASE F */
/**
  \brief This function will initialze the variables that are used in the
  Pedometer Algorithm. It should be called in Power On Init.0 is passed as parameter for 2G,
  1 for 4G and 2 for 8G\n
  <b>Calling instance:<b>\n Call the function after giving a delay (10msec) after power on
  \param unsigned char v_GRange_u8r : Parameter used to set the division factor for threshold.\n
  0-->2G\n
  1-->4G\n
  2-->8G\n
  \return None\n
*/
void InitAlgo(unsigned char v_GRange_u8r)
{
    /*
    **
    **********************************************************************************
     Function Name  : void InitAlgo(void)
     Input          : None
     Output         : None
     Scope          : Global

     Detailed Description:
           This function will initialze the variables that are used in the Algorithm.
    **********************************************************************************
    **
    */
    /* Reset the activity as 0 */
    V_Activity_U8 = C_Clear_U8X;

    /* Reset the step count */
    gs_stepCount_i32 = C_Clear_U8X;
	
	gs_stepCountCompensate_i32 = C_Clear_U8X;
    
    /* Set the Flag so that algo starts fresh */
    V_StatusFlags_U8R = C_Clear_U8X;
    V_StatusFlags_U8R |= (M_ModeDetection_U8X|M_AlgoReset_U8X);
    V_DivideFactor_U8R = v_GRange_u8r;
    
    /*Default lower limit set as robust*/
    V_LazyWalkLowerLimit_U8R = LAZY_WALKING_LOWER_LIMIT_Robust;
}

/* EasyCASE ) */
/* EasyCASE ( 208
   Enable Robustness */
/* EasyCASE F */
/**
  \brief This function will start the Pedometer Algorithm with robustness
  feature enabled.By default robustness feature will be there in the algorithm.
  If any time the robustness feature is disabled then user need to call this
  functionto enable the robustness feature. 
  
  \param None
  \return None
*/

void enableRobustness()
{
    /*
    **
    *********************************************************************************
     Function Name : void enableRobustness()

     Input         : None
     Output        : None
     Scope         : Global

     Detailed Description:
     This function will start the Pedometer Algorithm with robustness
     feature enabled.By default robustness feature will be there in the algorithm.
     If any time the robustness feature is disabled then user need to call this
     function to enable the robustness feature.
    **********************************************************************************  
    **
    */
    V_StatusFlags_U8R&=~(M_DisableRobustness_U8X);
    
    /*Lower threshold for robust mode*/
    V_LazyWalkLowerLimit_U8R=LAZY_WALKING_LOWER_LIMIT_Robust;
}

/* EasyCASE ) */
/* EasyCASE ( 209
   Disable Robustness */
/* EasyCASE F */
/**
  \brief This function will start the Pedometer Algorithm with robustness
  feature disabled.By default robustness feature will be there in the algorithm.
  If any time the robustness feature need to be disabled then user need to call this
  function 
  
  \param None
  \return None
*/
void disableRobustness()
{
    /*
    **
    *********************************************************************************
     Function Name : void disableRobustness()

     Input         : None
     Output        : None
     Scope         : Global

     Detailed Description:
     This function will start the Pedometer Algorithm with robustness
     feature disabled.By default robustness feature will be there in the algorithm.
     If any time the robustness feature need to be disabled then user need to call this
     function 
    **********************************************************************************  
    **
    */
    V_StatusFlags_U8R|=(M_DisableRobustness_U8X);
    
     /*Lower threshold for  non robust mode*/
    V_LazyWalkLowerLimit_U8R=LAZY_WALKING_LOWER_LIMIT_NoRobust;
}

/* EasyCASE ) */
/* EasyCASE ) */
/* EasyCASE ( 45
   Local Functions */
/* EasyCASE ( 2
   getAbsoluteShort */
/* EasyCASE F */
static short getAbsoluteShort(short f_val_i16)
{
    /*
    **
    *********************************************************************************
     Function Name : static short getAbsoluteShort(short f_val_i16)
     Input         : short f_val_i16
     Output        : positive f_val_i16
     Scope         : Local

     Detailed Description:

     Variables:
        short f_val_i16:
          This is the variable whose absolute short has to be returned

     Description:
          The function recieves f_val_i16 as input and the function always return the
         positive value of f_val_i16

    *********************************************************************************
    **
    */
    /* Return the positive value of the passed variable */
    return (f_val_i16 < 0)? -f_val_i16 : f_val_i16;
}

/* EasyCASE ) */
/* EasyCASE ( 186
   trendFinder */
/* EasyCASE F */
static void trendFinder(short f_composite_i16, short* f_optimalComposite_i16p)
{
    /*
    **
    ***************************************************************************************************
     Function Name   : static char trendFinder(short f_composite_i16, short* f_optimalComposite_i16p)
     Input           : short f_composite_i16
                       short* f_optimalComposite_i16p
     Output          : Returns 1 if the trend is positive
                       Returns -1 if the trend is negative
                       Returns 0 if there is no trend change
     Scope           : Local

     Detailed Description:
       Variables:
         short f_composite_i16:
                   This parameter is the composite value whose trend has to be
                   found out.
         short* f_optimalComposite_i16p:
                   The address at which the trend of the passed composite value is
                   stored
       Output :Returns 1 if the trend is positive
               Returns -1 if the trend is negative
               Returns 0 if there is no trend change


     Description:
         This function verifies if there is trend change from the given input and
         the previous ones and returns 1 or -1 if there is positive or negative
         trend change and returns 0 if there is no trend change.
    ****************************************************************************************************
    **
    */
   
    /* Holds the last composite value */
    static signed short     s_compositePrev1_i16 = -1;
    
    /* Holds the second last composite value */
    static signed short     s_compositePrev2_i16 = -1;
    
    /* Holds the current trend */
    static signed char      s_currTrend_i8 = -1;
	
	/* 	Static counter to count the interval between negative and positive 		*/
	static unsigned short 	s_np_counter = 0;
    
    /*Stores the  high peak value of wave*/
//    static short v_Com_ResultHigh_s16r=0;
    
    /*Stores the  low peak value of wave*/
//    static short v_Com_ResultLow_s16r=0;
	
    /*Stores the  high peak value of wave*/
//    static short v_Com_Confirm_ResultHigh_s16r=0;
    
    /*Stores the  low peak value of wave*/
//    static short v_Com_Confirm_ResultLow_s16r=0;
	
	
    /* Holds the Change in the trend */
    V_StatusFlags_U8R &=~(M_NegativeTrend_U8X|M_PositiveTrend_U8X);
    
    /* Check whether the current trend is positive */
    if (s_currTrend_i8 == 1)
    {
        /* check whether there is a trend change between the present and the previous composite values */
        if (((f_composite_i16 < s_compositePrev1_i16) && (f_composite_i16 < s_compositePrev2_i16)))
        {
			#ifdef STEP_CNT_ALGORITHM_DEBUG
				printf("p \r\n");
			#endif
			/* Confirm this positive peak 	*/
//			v_Com_ResultHigh_s16r = (s_compositePrev1_i16 > s_compositePrev2_i16) ? s_compositePrev1_i16 : s_compositePrev2_i16;
//			if( 
//				(((v_Com_ResultHigh_s16r - v_Com_Confirm_ResultLow_s16r) > C_SensorNoiseLevel_U8X)&& (s_np_counter > 6))
//			|| (s_compositePrev1_i16 == 0 && s_compositePrev2_i16 == 0)
//			)
			{
//				v_Com_Confirm_ResultHigh_s16r = v_Com_ResultHigh_s16r;
				
				//printf("confirm p \r\n");
				
				/* 	Reset counter 		*/
				s_np_counter = 0;
				
				/* Set the current trend as negative */
				s_currTrend_i8 = -1;
				
				/* Set the change in trend as negative */
				V_StatusFlags_U8R |= M_NegativeTrend_U8X;
				
				/* Return the optimal composite value */
				*f_optimalComposite_i16p = (s_compositePrev1_i16 > s_compositePrev2_i16) ? s_compositePrev1_i16 : s_compositePrev2_i16;
			}
        }
    }
    else
    {
        /* If the current trend is negative */
        /* check whether there is a trend change between the present and the previous composite values */
        if (f_composite_i16 > s_compositePrev1_i16 && f_composite_i16 > s_compositePrev2_i16)
        {
			#ifdef STEP_CNT_ALGORITHM_DEBUG
				printf("n \r\n");
			#endif
			/* Confirm this negative peak 	*/
//			v_Com_ResultLow_s16r = (s_compositePrev1_i16 < s_compositePrev2_i16) ? s_compositePrev1_i16 : s_compositePrev2_i16;
//			if( 
//				(((v_Com_Confirm_ResultHigh_s16r - v_Com_ResultLow_s16r) > C_SensorNoiseLevel_U8X) && (s_np_counter > 6))
//			|| (s_compositePrev1_i16 == 0 && s_compositePrev2_i16 == 0)
//			)
			{
				//printf("confirm n \r\n");
//				v_Com_Confirm_ResultLow_s16r = v_Com_ResultLow_s16r;
				/* 	Reset counter 		*/
				s_np_counter = 0;			

				/* Set the current trend as Positive */
				s_currTrend_i8 = 1;
				
				/* Set the change in trend as Positive */
				V_StatusFlags_U8R |= M_PositiveTrend_U8X;
				
				/* Return the optimal composite value */
				*f_optimalComposite_i16p = (s_compositePrev1_i16 < s_compositePrev2_i16) ? s_compositePrev1_i16 : s_compositePrev2_i16;
			}
        }
    }
    
    /* Update the second last composite value */
    s_compositePrev2_i16 = s_compositePrev1_i16;
    
    /* Update the last composite values */
    s_compositePrev1_i16 = f_composite_i16;
    
	/* 	Reset counter 		*/
	s_np_counter++;
	
    /* Return the Change in trend */
}

/* EasyCASE ) */
/* EasyCASE ( 263
   YaxisAnalysis */
/* EasyCASE F */
static void YaxisAnalysis(short f_composite_i16)
{
    /*
    **
    ***************************************************************************************************
     Function Name   : static void YaxisAnalysis(short f_composite_i16)
     Input           : short f_composite_i16
                       
     Output          : Output peak to peak value and count for Y axis
                       
     Scope           : Local

     Detailed Description:
       Variables:
         short f_composite_i16:
                   This parameter is the composite value whose trend has to be
                   found out and Peak tp peak threshold and count need to find.
       
       Output :None


     Description:
         This function verifies if there is trend change from the given input and
         the previous ones and returns peak to peak value and count of each cycle
    ****************************************************************************************************
    **
    */
   
    /* Holds the last composite value */
    static signed short     s_compositePrev1_i16 = -1;
    
    /* Holds the second last composite value */
    static signed short     s_compositePrev2_i16 = -1;
    
    /* Holds the current trend */
    static signed char      s_currTrend_i8 = -1;
    
    /*Counts the count between peaks*/
    static unsigned char v_PPCount_u8r=0;
    
    /*Stores the  high peak value of wave*/
    static short v_ResultHigh_s16r=0;
    
    /*Stores the  low peak value of wave*/
    static short v_ResultLow_s16r=0;
    
    /* Holds the Change in the trend */
    //V_StatusFlags_U8R &=~(M_NegativeTrend_U8X|M_PositiveTrend_U8X);
    
    /* Check whether the current trend is positive */
    if (s_currTrend_i8 == 1)
    {
        /* check whether there is a trend change between the present and the previous composite values */
        if ((f_composite_i16 < s_compositePrev1_i16) && (f_composite_i16 < s_compositePrev2_i16))
        {
            /* Set the current trend as negative */
            s_currTrend_i8 = -1;
            
            /* Set the change in trend as negative */
            //V_StatusFlags_U8R |= M_NegativeTrend_U8X;
            
            /* Return the optimal composite value */
            v_ResultHigh_s16r = (s_compositePrev1_i16 > s_compositePrev2_i16) ? s_compositePrev1_i16 : s_compositePrev2_i16;
        }
    }
    else
    {
        /* If the current trend is negative */
        /* check whether there is a trend change between the present and the previous composite values */
        if ((f_composite_i16 > s_compositePrev1_i16) && (f_composite_i16 > s_compositePrev2_i16))
        {
            /* Set the current trend as Positive */
            s_currTrend_i8 = 1;
            
            /* Set the change in trend as Positive */
            //V_StatusFlags_U8R |= M_PositiveTrend_U8X;
            
            /* Return the optimal composite value */
            v_ResultLow_s16r = (s_compositePrev1_i16 < s_compositePrev2_i16) ? s_compositePrev1_i16 : s_compositePrev2_i16;
            
            /*Check whether current peak is not due to sensor noise*/
            if ((v_ResultHigh_s16r-v_ResultLow_s16r) > C_SensorNoiseLevel_U8X)
            {
                /*Peak to peak value  stored to global variable*/
                V_YaxisPP_S16R = v_ResultHigh_s16r-v_ResultLow_s16r;
                
                /*Previous Cycle counts are updated */
                V_YaxisCycleCountPrev1_U8R = V_YaxisCycleCountPrev_U8R;
                V_YaxisCycleCountPrev_U8R = V_YaxisCycleCount_U8R;
                
                /*New Cycle count updated to global variable */
                V_YaxisCycleCount_U8R = v_PPCount_u8r;
                
                /*Current cycle count variable cleared*/
                v_PPCount_u8r = 0;
            }
        }
    }

	/*Incrementing the count between peaks*/
    v_PPCount_u8r++;
    
    /* EasyCASE - */
    /* Update the second last composite value */
    s_compositePrev2_i16 = s_compositePrev1_i16;
    
    /* Update the last composite values */
    s_compositePrev1_i16 = f_composite_i16;
}

/* EasyCASE ) */
/* EasyCASE ) */
/* EasyCASE ) */
