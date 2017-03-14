//******************************************************************************
//  Claude ZHU
//  Hardware Team
//  (C) iCareTech Inc., 2013
//  All Rights Reserved.
//  Built with Keil MDK
//
//-------------------------------------------------------------------------------

#ifndef PulseOximetryAFE_H_
#define PulseOximetryAFE_H_


#include "cc_alg_types.h"


/*****************************************************************************/
/*                   Type Struct Definition                                  */
/*****************************************************************************/

#define		AFE_DAT_AVE_NUM		(	31	)

#define     AFE_DAT_BUF_LEN    (100)

typedef enum{WEAR_GOOD = 1, WEAR_BAD = !WEAR_GOOD} WearLock_typedef;
typedef enum{WEAR_DET_FINISH = 1, WEAR_DET_NOT_FINISH = !WEAR_DET_FINISH} WearDetState_typedef;

typedef enum{WEAR_DETECT_ONLY = 0, WEAR_DETECT_INC = !WEAR_DETECT_ONLY} WearDetType_typedef;

typedef enum{ADJ_STAGE_ONE = 0, ADJ_STAGE_TWO = !ADJ_STAGE_ONE} AdjStage_Typedef;
typedef enum{DRDY_STATE_BASELINE_ADJ = 0, DRDY_STATE_MEASURE, DRDY_STATE_SPOT_CHECK} gDrdyIntState_typedef;


typedef struct
{
    void (* task_event_send)(uint8_t, uint32_t, SysEvtDatTypedef *);    //pointer to the sys event distribute 
    
    void (* data_copy_trig)(void);                                      // functional pointer to data copy trigger implementation
    
    /*  Pointer to the new data input buffer    */
    sampleDataRaw_Typedef *   sampleDataRaw_pst;
} ccAlgAfe_st;





typedef struct
{
	WearLock_typedef 		wearState;
	WearDetState_typedef 	wearDetState;
} WearDetResult_Typedef;


/*Typedef: data sampled from AFE*/
typedef struct{
	int32_t 	redData;
	int32_t 	irData;
	int32_t 	redDiffData;
	int32_t 	irDiffData;
}PulseOxi_Sample_Typedef;


/*Typedef: TX channel Configuration*/
typedef struct{
	uint8_t 	ledTurnOn;			//LED status
	uint8_t 	redLedCurr;			//red LED current value
	uint8_t 	irLedCurr;			//ir LED current value
}PulseOxi_Tx_Channel_Typedef;

/*Typedef: RX channel Configuration*/
typedef struct{
	unsigned int 	cfTIAVal;			//CF value
	unsigned int  	rfTIAVal;			//RF Value
	unsigned int 	ambDac;				//Ambient Cancel DAC
	unsigned int 	gainStage2;			//Stage 2 Gain
	bool 	enStage2;			//status of stage2 gain
}PulseOxi_Rx_Channel_Typedef;



/*****************************************************************************/
/*                   Parameters Definition                                   */
/*****************************************************************************/

#define RX_CH_RF_DEFAULT_VAL			AFE44x0_RF_LED_10K		//10K

#define RX_CH_CF_DEFAULT_VAL			AFE44x0_CF_LED_100pF_Plus_5pF

/*Define the target and threshold for different test location */

/*
 22bits width for AFE440x, valid bit width is 21 bits due to differential sample
 the full scale is 
 0 ~ 2^21 = 2097152
*/


    



//    /*  For 6dB,2uA         */
//	#define TARGET_SIGNAL_LEVEL_MAX				(1572864)				//0.9v
//	#define TARGET_SIGNAL_LEVEL_MIN 			(1048576)				//0.6v
//	#define TARGET_SIGNAL_LEVEL						(1398101)				//0.8v
//	
//	#define	TARGET_SIGNAL_LEVEL_MAX_RUN			(1747626)
//	#define TARGET_SIGNAL_LEVEL_MIN_RUN			(0)

//    /*  For 6dB, 7uA        */
//	#define TARGET_SIGNAL_LEVEL_MAX				(1048576)				//0.6v
//	#define TARGET_SIGNAL_LEVEL_MIN 			(-838860)				//0.2V
//	#define TARGET_SIGNAL_LEVEL						(699050)				//0.4v
//	
//	#define	TARGET_SIGNAL_LEVEL_MAX_RUN			(1398101)
//	#define TARGET_SIGNAL_LEVEL_MIN_RUN			(-1398101)

//    /*  For 6dB, 5uA        */
//	#define TARGET_SIGNAL_LEVEL_MAX				(699050)				//0.4v
//	#define TARGET_SIGNAL_LEVEL_MIN 			(-699050)				//0.4V
//	#define TARGET_SIGNAL_LEVEL						(0)				//0v
//	
//	#define	TARGET_SIGNAL_LEVEL_MAX_RUN			(1048576)
//	#define TARGET_SIGNAL_LEVEL_MIN_RUN			(-1048576)

    /*  For 9.5dB, 7uA      */
//	#define TARGET_SIGNAL_LEVEL_MAX				(1677721)				//80%(max)
//	#define TARGET_SIGNAL_LEVEL_MIN 			(524288)				//50%(min)
//	#define TARGET_SIGNAL_LEVEL						(1048576)				//25%
//	
//	#define	TARGET_SIGNAL_LEVEL_MAX_RUN			(1677721)
//	#define TARGET_SIGNAL_LEVEL_MIN_RUN			(524288)



/*****************************************************************************/
/*                   AFE Parameter Configuration Function Declaration        */
/*****************************************************************************/
void PulseOxiAfe_Init(void);
void PulseOxiAfe_DeInit(void);
void PulseOxiAfe_Diag_En(void);
void PulseOxiAfe_Sample_Data(PulseOxi_Sample_Typedef *pulseOxiSampleData);
bool PulseOxiAfe_Diag_Check(void);
void PulseOxiAfe_DRY_Handler(void);

void PulseOxiAfe_SignalAdj_Init(void);
void PulseOxiAfe_SignalAdj_Check(void);

void PulseOxiAfe_RxCh_Cfg(PulseOxi_Rx_Channel_Typedef* Rx_Chan_Val);
void PulseOxiAfe_TxCh_Cfg(PulseOxi_Tx_Channel_Typedef* Tx_Chan_Val);

void Wear_State_Clear(void);

void Wear_State_Set(WearLock_typedef WearLock,WearDetState_typedef WearDetState);

void Wear_State_Get(WearDetResult_Typedef * wearDetRes);

void Wear_Detect_Set(WearDetType_typedef flagState);

WearDetType_typedef Wear_Detect_Get(void);

/* 	Runtime counter kick func 		*/
void SpO2_RunTime_Cnt_Kick(void);

/* 	Runtime counter get func 		*/
uint32_t SpO2_RunTime_Cnt_Get(void);

/* 	Runtime counter set func 		*/
void SpO2_RunTime_Cnt_Set(uint32_t newValue);

FUNC_RETURN_TYPE cc_alg_afe_init(ccAlgAfe_st * ccAlgAfeInit_pst);

void cc_alg_afe_signal_check( void );

void PulseOxiAfe_Spot_Check_Kick(void);

void cc_afe_get_dc_compensate(int32_t * piDcCompVal);

void cc_afe_set_factory_mode(bool newMode);


#endif 

/*PulseOximetryAFE_H_*/

