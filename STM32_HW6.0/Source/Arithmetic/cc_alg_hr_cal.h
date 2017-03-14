/*
* Copyright(c) 2015, CloudCare Healthcare Co.,Ltd
* All rights reserved
* File Name:                cc_alg_hr_cal.h
* Descprition:              the cc_alg_hr_cal.h files for translate layer
* Created Date:             2016/03/14
* Author:                   yun.zhu@ywkang.com  
* Current Reversion:        V1.0
* Reversion History:
*   Rev *.*    
*     Author:     
*     Change Date:
*     Change List:
*/

#ifndef _CC_ALG_HR_CAL_H
#define _CC_ALG_HR_CAL_H

#include "cc_alg_types.h"
#include "arrayOperaFunction.h"
#include "cc_alg_hr_spo2_cal_param_def.h"
#include "cc_alg_basic_func.h"


#define HR_BASELINE_TIMESPAN		(120)
#define HR_BASELINE_AVE_FACTOR_SLOW		(48)
#define HR_BASELINE_AVE_FACTOR_FAST		(24)
#define HR_BASELINE_PERIOD_DIFF_MAX 	(80)

#define HR_REALTIME_PERIOD_DIFF_MAX 	(140)
#define HR_REALTIME_AVE_FACTOR_MID      (12)
#define HR_REALTIME_AVE_FACTOR_SLOW		(24)
#define HR_REALTIME_AVE_FACTOR_FAST		(2)

#define HR_BASELINE_BUF_LEN			(10)
#define HR_REALTIME_BUF_LEN			(4)

#define HR_MAX_FREQ_POS             (511)



/**
 * @brief The declaration of HR long time reserve structure
 */
typedef struct{
	st_Arr_ui16_Typedef * m_pArrHrPos;
	uint16_t 		m_ui16HrPosBaseline;
	float32_t 		m_f32HrPosBaseline;
	uint16_t 		m_ui16HrBaselineTimespan;
	bool 			m_bIsHrBaselineStable;
	uint8_t 		m_ui8HrBaselineAveFactorFast;
	uint8_t 		m_ui8HrBaselineAveFactorSlow;
	uint16_t 		m_ui16HrBaselineCnt;
	uint8_t 		m_ui8LowConfCnt;
	uint8_t 		m_ui8LowConfCntMax;
	uint16_t 		m_ui16HrChangeSpeedMax;
} st_HrPosBaseline_Typedef;

/**
 * @brief The declaration of Hr position matched structure
 */
typedef struct{
	uint16_t * 		m_pArrMatchedPos;
	bool			m_bIsBothMainPeak;
    bool            m_bIsPerfectSignalQuality;
	bool * 			m_pArrIsBothHighReliable;
	int32_t * 		m_piSpecPeakValOfIr;
	uint16_t 		m_uRatioMainToSecondPeak;
} st_HrPosMatch_Typedef;

/**
 * @brief The declaration of HR realtime structure
 */
typedef struct{
	st_Arr_ui16_Typedef * m_pArrHrPos;
	uint16_t 		m_ui16HrRealTime;
	float32_t 		m_f32HrPosRealTime;
	uint16_t 		m_ui16HrRealTimeQuick;
	uint8_t 		m_ui8HrRealTimeFactorFast;
	uint8_t 		m_ui8HrRealTimeFactorSlow;
    uint8_t         m_ui8HrRealTimeFactorMid;
	uint16_t 		m_ui16HrChangeSpeedMax;
    bool            m_bIsRealTimeHrReady;
    bool 			m_bIsRealTimeHrQuickReady;
    bool 			m_bIsCurrentRealTimeHrReliable;
    bool            m_bIsCurrentRealTimeHrFullyTrust;
} st_HrPosRealTime_Typedef;

typedef struct{
	int32_t	 	i32PeakVal[FREQ_PEAK_BUF_LEN];
	uint16_t 	u16PeakPos[FREQ_PEAK_BUF_LEN];
	uint8_t 	u8DatLen;
	uint8_t 	u8ValidPeakNum;
}Freq_Peak_Buf_Typedef;

typedef struct{
    int32_t     m_i32PeakVal[FREQ_PEAK_MATCH_BUF_LEN];
    uint16_t    m_ui16PeakPos[FREQ_PEAK_MATCH_BUF_LEN];
    bool        m_bIsHighConfPeak[FREQ_PEAK_MATCH_BUF_LEN];
    uint8_t     m_ui8PeakLen;
    bool 		peakTrustFlag;
    bool        m_bIsMainSecondMatch;
}Freq_Peak_Pos_Typedef;

typedef struct{
	int32_t     m_i32P2pValueIr;
	int32_t     m_i32P2pValueRed;
	bool		m_bIsP2PCheckPass;
	uint16_t    m_ui16XcorrRatioIr;
	uint16_t    m_ui16XcorrRatioRed;
	bool 		m_bIsXocrrCheckPass;
	bool 		m_bIsMotionCheckPass;
    bool        m_bIsMotionClearOngoing;
	uint8_t		m_uiMotionLevel;
} st_SignalCheck_Typedef;
	
	

/**
 * @brief Function export
 */

void cc_alg_weighted_hr_cal(
					Freq_Peak_Buf_Typedef * 		pPeakBufIr,
					Freq_Peak_Buf_Typedef * 		pPeakBufRed,
					Freq_Peak_Buf_Typedef * 		pPeakBufDiff,
					HR_SPO2_DAT_T * 			pResultOut,
					st_SignalCheck_Typedef *		pSignalCheck_pst,
					st_HrPosBaseline_Typedef *		HrPosBaseline_pst,
					st_HrPosRealTime_Typedef * 		HrPosRealTime_pst,
                    uint8_t                         bIsEnableHrWeigth
		);
FUNC_RETURN_TYPE get_hr_from_freq_pos(uint16_t ui16FreqPos, uint8_t * pui8HrVal);
FUNC_RETURN_TYPE get_freq_pos_from_hr(uint8_t ui8HrVal, uint16_t * pui16FreqPos);

                    
#endif // _CC_ALG_HR_CAL_H

// end file
