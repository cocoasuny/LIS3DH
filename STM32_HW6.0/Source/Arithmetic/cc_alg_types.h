/*
* Copyright(c) 2015, CloudCare Healthcare Co.,Ltd
* All rights reserved
* File Name:                cc_alg_types.h
* Descprition:              the typedef .h files for algorithm process
* Created Date:             2016/03/12
* Author:                   yun.zhu@ywkang.com  
* Current Reversion:        V1.0
* Reversion History:
*   Rev *.*    
*     Author:     
*     Change Date:
*     Change List:
*/

#ifndef _CC_ALG_TYPES
#define _CC_ALG_TYPES

#include "stdlib.h"
#include "string.h"
#include "stdbool.h"
#include "stdio.h"

/**
*   @brief generic data type define here, need to change for different platform
*/
    /* exact-width signed integer types */
typedef   signed          char      int8_t;
typedef   signed short     int      int16_t;
typedef   signed           int      int32_t;
typedef   signed long long          int64_t;

    /* exact-width unsigned integer types */
typedef unsigned          char      uint8_t;
typedef unsigned short     int      uint16_t;
typedef unsigned           int      uint32_t;
typedef unsigned long long          uint64_t;

    /* exact-width float types */
typedef float                       float32_t;
typedef double                      float64_t;

    /*  Null Definition         */
#ifndef NULL
    #define NULL            (0)
#endif

    /*  Bool type Definition         */

//#ifndef bool
//    #define     bool        uint8_t
//#endif

#ifndef true
    #define true    (1)        
#endif

#ifndef false
    #define false   (0)
#endif

#define FLAG_FFT 					(0)
#define FLAG_IFFT 					(1)
#define FLAG_REV_BIT_ORDER_ENABLE			(1)
#define FLAG_REV_BIT_ORDER_DISABLE	 		(0)
    
    
    
    
    
    
    
    
    
/**
* @brief Definition of system event for spo2 measurement
*/
#define gSpO2EventBaselineStart					  ((uint32_t)(1 << 1))
#define gSpO2EventBaselineCal					  ((uint32_t)(1 << 2))

#define gSpO2EventMeasureInit					  ((uint32_t)(1 << 3))
#define gSpO2EventMeasureCal 					  ((uint32_t)(1 << 4))

#define gSpO2EventError							  ((uint32_t)(1 << 7))

#define gSpO2EventStartInternal					((uint32_t)(1 << 9))

#define gSpO2EventSignalCheck                   ((uint32_t)(1 << 10))

#define gSpO2EventSpotCheck                     ((uint32_t)(1 << 11))

/**
* @brief Definition of system event function's data body
*/
typedef struct{
    uint16_t *   m_pui16SysEvtDat;
} SysEvtDatTypedef;

    
/**
* @brief Definition of system event function
*/
#define CC_ALG_SYS_EVENT_SEND_PRT void (* task_event_send)(uint8_t, uint32_t, SysEvtDatTypedef *)

#define CC_ALG_SYS_EVENT_SEND_FUNC(task_id, event_id, pSysEvtDat)\
           task_event_send(task_id, event_id, pSysEvtDat)

/**
* @brief Definition of system data handle function
*/
#define CC_ALG_SYS_DATA_HANDLE_PRT void (* data_post_handle)( HR_SPO2_DAT_T *)

#define CC_ALG_SYS_DATA_HANDLE_FUNC(DataReport_pst)\
           data_post_handle(DataReport_pst)
  
/**
* @brief Definition of System critical resource apply routine
*/
#define CC_ALG_SYS_CRITICAL_RESOURCE_APPLY_PRT void (* cc_alg_sys_critical_resource_apply)(void)
/**
* @brief Definition of System critical resource release routine
*/
#define CC_ALG_SYS_CRITICAL_RESOURCE_RELEASE_PRT void (* cc_alg_sys_critical_resource_release)(void)

/**
* @brief Definition of system delay in million second
*/
#define CC_ALG_SYS_DELAY_MS_PRT void (* cc_alg_sys_delay_ms)(uint16_t)


/**
*   @brief SPO2_SMOOTH_LEVEL
*       LEVEL 1-- fast response time, less average
*       LEVEL 2-- balanced repsonse time
*       LEVEL 3-- stable data output
*/
typedef enum{
    SPO2_SMOOTH_LEVEL1     = (1),
    SPO2_SMOOTH_LEVEL2     = (2),
    SPO2_SMOOTH_LEVEL3     = (3),
} SPO2_SMOOTH_LEVEL_TYPE;  

/**
*   @brief Function Return Code definition
*/
typedef enum{
    RET_VAL_NULL            = (0),
    RET_VAL_SUCCESS         = (1),
    RET_VAL_PARAM_ERROR     = (-1),
    RET_VAL_CAL_ERROR       = (-2),
    RET_VAL_CAL_FAIL        = (-3)      /*  Not finish the calculation as routine   */
} FUNC_RETURN_TYPE;  

/**
* @brief Definition of SpO2/Hr data value structure
*/
typedef struct{
	uint8_t 			m_ui8SpO2Val;
	uint8_t 			m_ui8HrVal;
    uint16_t            m_ui16MotionVal;
	bool 		        m_bIsSpO2Valid;
	bool 		        m_bIsHrValid;
    uint8_t             m_ui8ConfVal;
}HR_SPO2_DAT_T;

/**
* @brief Definition of SpO2 Hr module running mode
*/
typedef enum{
    SPO2_HR_MODE_NULL   = 0,            /*  Null for reserved   */
    SPO2_HR_MODE_OSA    = 1,            /*  OSA monitor mode    */
    SPO2_HR_MODE_DEMO   = 2,            /*  demo mode           */
    SPO2_HR_MODE_COPD   = 3,            /*  COPD mode           */
    SPO2_HR_MODE_DETECT = 4             /*  Detect mode, only detect the reflect light  */
} ccAlgSpO2HrRunMode_en;

/**
* @brief Definition of SpO2Hr_Algorithm_Input_Parameters
*/
typedef struct{
    ccAlgSpO2HrRunMode_en   m_AlgRunMode_en;        //the running mode of algorithm module
    uint32_t                m_AlgRunTimeOut_s;      //the timeout limit from start measuring, 
                                                    //unit is second, should not less than 20
                                                    //if set to 0, will never stop the measure
    int32_t                 m_AlgDetThres;          //the threshold when in detect mode

} ccAlgSpO2HrParam_st;

/**
* @brief Definition of algorithm module status 
*/
typedef enum{
    SPO2_HR_STATUS_NULL         = 0,            /*  Null for reserved                   */
    SPO2_HR_STATUS_BASELINE     = 1,            /*  in baseline adjustment              */
    SPO2_HR_STATUS_ERROR        = 2,            /*  algorithm mode error                */
    SPO2_HR_STATUS_RUNNING      = 3,            /*  algorithm is running                */
    SPO2_HR_STATUS_MOTION       = 4,             /*  motion detected                     */
    SPO2_HR_STATUS_CHECK        = 5,            /*  spot check is ongoing                   */
    SPO2_HR_STATUS_STOP         = 6
} ccAlgSpO2HrStatus_en;


/**
* @brief Definition of SpO2/Hr run paramters input structure
*/
typedef struct{
    SPO2_SMOOTH_LEVEL_TYPE  m_DataSmoothLevel;
    ccAlgSpO2HrRunMode_en   m_AlgSpO2HrRunMode;
    bool                    m_bIsFactoryMode;
	int32_t 				m_iAfeDcOffset;
	uint32_t				m_uiMotionCheckThres;
	uint32_t 				m_uiClearWhileMotionThres;
}HR_SPO2_INIT_PARA_Typedef;

/**
* @brief Definition of 24bits IR/RED sample data 
*/
typedef struct{
    uint8_t     m_ui8MsbIr;
    uint8_t     m_ui8MsbRed;
    uint16_t    m_ui16LsbIr;
    uint16_t    m_ui16LsbRed;
}sampleDataRaw_Typedef;


/**
* @brief Definition of basic 24 bits to 32bit signed data 
*/
#define SAMPLE_DATA_24BITS_TO_SIGN32BITS(Msb8Bits, Lsb16Bits) \
        (((Msb8Bits) & 0x80) == 0x80)?\
            (0xFFC00000 | ((Msb8Bits) << 16)) | ((Lsb16Bits) & 0x0000ffff):\
            (0x1FFFFF & ((Msb8Bits) << 16)) | ((Lsb16Bits) & 0x0000ffff)




#endif  //end of _CC_ALG_TYPES
