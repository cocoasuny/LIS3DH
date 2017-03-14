#ifndef __SPO2_COEFF_H
#define __SPO2_COEFF_H

//#include "SpO2HrCalCZT.h"
#include "arm_math.h"
#include "cc_alg_types.h"
#include "cc_alg_hr_spo2_cal_param_def.h"

/*  
    Define the offset of SpO2 LUT index     
    Must be change if update the LUT's offset
*/
#define     SPO2_LUT_INDEX_OFFSET     (37)

/*  
    Define the 70% index cut line     
    Must be change if update the LUT
*/
#define     SPO2_LUT_SEVENTY_INDEX_RAW          (108)
#define     SPO2_LUT_SEVENTY_INDEX_COMP         (108)

/*   Define the length of LUT table     */
#define     SPO2_LUT_LENGTH                     (155)


/*  Define the length of Rval Suppres Table input length    */
#define     SPO2_SUPPRESS_MED_CONF_LEN          (3)
#define     SPO2_SUPPRESS_SIG_CONF_LEN          (4)

/*  Define the length of Medical Confidence table input length  */
#define     MED_SPO2_LEVEL_LEN                  (2)
#define     MED_SPO2_DIFF_LEN                   (3)
#define     MED_SPO2_LEVEL_CUT                  (90)
#define     MED_SPO2_DIFF_CUT_1                 (3)
#define     MED_SPO2_DIFF_CUT_2                 (8)


#define PEAK_WEIGHT_TABLE_OFFSET		FREQ_PEAK_POS_LP_LIMIT



/*  SpO2 LUT type define    */
typedef struct{
    uint16_t            m_ui16SpO2LutOffset;    // offset of SpO2 Table, the index for LUT equals to R*100 - m_ui16SpO2LutOffset
    uint16_t            m_ui16SpO2LutEnd;       // The LUT index for lowest SpO2 value
    float32_t const  *  m_pf32SpO2LutTable;     // Pointer to SpO2 Table
} SpO2Lut_str;

extern const q31_t B_LP_100HZ_4_8[];

/* AW parameters, in format of real, image, real, image */
extern const int16_t CZT_AW_Par[];
/* FFT_HK parameters, in format of real, image, real, image */
extern const int16_t CZT_FFT_Hk_Par[];

/* 	Table for Peak Position weighted calculation 			*/
/* 	size = [25 : 494], 470 number 							*/
extern const q15_t tablePeakWeighCoef[];

///* 	Table to compensate the SPO2 value due to breath 	*/
//extern const float32_t tableSPO2WeighCoef[];


void spo2_lut_init_func(bool bIsFullRangeSpO2Support, bool bIsCompensateEnable);
void get_SpO2_val(uint16_t ui16Rval, float32_t *pf32SpO2Val);

void get_SpO2_Rval_Filter_Coef(uint8_t ui8MedConfCoef, uint8_t ui8SigConfCoef, uint8_t * pui8RvalFilterCoef);
void get_medical_conf_level(int16_t i16SpO2Diff, uint8_t ui8SpO2BaseLine, uint8_t * pui8MedConf);
#endif /* __SPO2_COEFF_H */

