//******************************************************************************
//  Claude ZHU
//  Hardware Team
//  (C) iCareTech Inc., 2013
//
//
//
//-------------------------------------------------------------------------------

#ifndef AFE44x0_H_
#define AFE44x0_H_

#include "stdlib.h"
#include "string.h"

/**
* @brief Define the system debug information print function
*        should be the same format and usage as C standard "printf" function
* @note  Need to change this for difference implementation
*/
#define     ALG_PRINTF      printf


/*    Enable Low Power Mode        */
#define 		AFE4403_TIMMING_LOWPOWER

/**
*   @brief Function Return Code definition
*/
typedef enum{
    AFE4403_RET_VAL_NULL            = (0),
    AFE4403_RET_VAL_SUCCESS         = (1),
    AFE4403_RET_VAL_PARAM_ERROR     = (-1),
    AFE4403_RET_VAL_CAL_ERROR       = (-2),
} AFE4403_FUNC_RETURN_TYPE;  


/**
* @brief Struture of afe4403 timming mode 
*/
typedef struct
{
    unsigned int    m_Afe4403SamplePeriod_cnt;
    unsigned int    m_Afe4403LedPulseWidth_ms;
} Afe4403TimingTypedef;



/**
* @brief Definition of SPI write function
*/
#define AFE4403_BUS_WR_PRT AFE4403_FUNC_RETURN_TYPE (* afe4403_bus_wr)(unsigned char, unsigned int)

#define AFE4403_BUS_WR_FUNC(ui8RegAddr, ui32DataIn)\
           afe4403_bus_wr(ui8RegAddr, ui32DataIn)

/**
* @brief Definition of SPI read function
*/
#define AFE4403_BUS_RD_PRT AFE4403_FUNC_RETURN_TYPE (* afe4403_bus_rd)(unsigned char,unsigned int *)

#define AFE4403_BUS_RD_FUNC(ui8RegAddr, pui32DataRet)\
           afe4403_bus_rd(ui8RegAddr, pui32DataRet)

/**
* @brief Definition of system interrupt enable function
*/
#define SYS_INTERRUPT_ENABLE_PRT void (* sys_int_enable)(void)

#define SYS_INTERRUPT_ENABLE_FUNC()\
           sys_int_enable()

/**
* @brief Definition of system interrupt enable function
*/
#define SYS_INTERRUPT_DISABLE_PRT int (* sys_int_disable)(void)

#define SYS_INTERRUPT_DISABLE_FUNC()\
           sys_int_disable()

/**
* @brief Definition of system interrupt enable function
*/
#define SYS_DELAY_MS_PRT void (* sys_delay_ms)(unsigned short)

#define SYS_DELAY_MS_FUNC()\
           sys_delay_ms()




/**
* @brief Definition of AFE4403 instance
*/
typedef struct
{
    /*  Function pointers need init     */
    AFE4403_BUS_WR_PRT;
    AFE4403_BUS_RD_PRT;
    SYS_INTERRUPT_ENABLE_PRT;
    SYS_INTERRUPT_DISABLE_PRT;
    SYS_DELAY_MS_PRT;
    /*  Others needs to be init          */
    
    
    
} Afe4403InstTypedef;





/****************************************************************/
/*   AFE44x0 Parameters Definition                              */
/****************************************************************/
/*Pulse Repetition Period count max value, equal to 62.5Hz*/
#define AFE44x0_PRPCNT_MAX 64000
/*Pulse Repetition Period count min value, equal to 5KHz*/
#define AFE44x0_PRPCNT_MIN 800
/*4MHz main clock duration, 0.25us = 4(factor)*/
#define AFE44x0_MAIN_CLK_FACTOR			4
/*LED pulse width, 4MHZ clock, = 0.25us*/
#define AFE44x0_LED_PULSE_WIDTH			100*AFE44x0_MAIN_CLK_FACTOR
//#define AFE44x0_LED_PULSE_WIDTH_70US		70*AFE44x0_MAIN_CLK_FACTOR
//#define AFE44x0_LED_PULSE_WIDTH_100US		100*AFE44x0_MAIN_CLK_FACTOR
//#define AFE44x0_LED_PULSE_WIDTH_150US		150*AFE44x0_MAIN_CLK_FACTOR
//#define AFE44x0_LED_PULSE_WIDTH_200US		200*AFE44x0_MAIN_CLK_FACTOR
/*Sample Pulse Width, Based on 4MHz clock*/
#define AFE44x0_SAMPLE_PULSE_WIDTH		70*AFE44x0_MAIN_CLK_FACTOR
//#define AFE44x0_SAMPLE_PULSE_WIDTH_50US		50*AFE44x0_MAIN_CLK_FACTOR
//#define AFE44x0_SAMPLE_PULSE_WIDTH_80US		80*AFE44x0_MAIN_CLK_FACTOR
//#define AFE44x0_SAMPLE_PULSE_WIDTH_130US		130*AFE44x0_MAIN_CLK_FACTOR
//#define AFE44x0_SAMPLE_PULSE_WIDTH_180US		180*AFE44x0_MAIN_CLK_FACTOR
/*Reset Pulse Width, Based on 4MHz clock*/
#define AFE44x0_RST_PULSE_WIDTH		8
//#define AFE44x0_RST_PULSE_WIDTH_4CLK		4
//#define AFE44x0_RST_PULSE_WIDTH_6CLK		6
//#define AFE44x0_RST_PULSE_WIDTH_8CLK		8
/*Convert pulse width*/
#define AFE44x0_CONV_PULSE_WIDTH		(200*AFE44x0_MAIN_CLK_FACTOR - AFE44x0_RST_PULSE_WIDTH)

#define AFE44x0_SAMPLE_FREQ         (100)


	/*Timing Parameters Definition for 100Hz sample rate */
	#ifdef AFE4403_TIMMING_LOWPOWER

        /* 	100Hz, 2%duty cycle,32ave	*/
        #define		AFE44x0_PDNCYCLESTC_NUM		(	27470	)
        #define		AFE44x0_PDNCYCLEENDC_NUM		(	39199	)
        #define		AFE44x0_DAT_AVE_NUM		(	31	)
        #define	 	AFE44x0_PRPCOUNT_100HZ		(	39999	)
        #define	 	AFE44x0_LED1LEDSTC_100HZ		(	1600	)
        #define	 	AFE44x0_LED1LEDENDC_100HZ		(	2399	)
        #define	 	AFE44x0_LED2LEDSTC_100HZ		(	0	)
        #define	 	AFE44x0_LED2LEDENDC_100HZ		(	799	)
        #define	 	AFE44x0_LED1STC_100HZ		(	1680	)
        #define	 	AFE44x0_LED1ENDC_100HZ		(	2399	)
        #define	 	AFE44x0_LED2STC_100HZ		(	80	)
        #define	 	AFE44x0_LED2ENDC_100HZ		(	799	)
        #define	 	AFE44x0_ALED1STC_100HZ		(	2480	)
        #define	 	AFE44x0_ALED1ENDC_100HZ		(	3199	)
        #define	 	AFE44x0_ALED2STC_100HZ		(	880	)
        #define	 	AFE44x0_ALED2ENDC_100HZ		(	1599	)
        #define	 	AFE44x0_LED1CONVST_100HZ		(	13744	)
        #define	 	AFE44x0_LED1CONVEND_100HZ		(	20203	)
        #define	 	AFE44x0_LED2CONVST_100HZ		(	808	)
        #define	 	AFE44x0_LED2CONVEND_100HZ		(	7267	)
        #define	 	AFE44x0_ALED1CONVST_100HZ		(	20212	)
        #define	 	AFE44x0_ALED1CONVEND_100HZ		(	26671	)
        #define	 	AFE44x0_ALED2CONVST_100HZ		(	7276	)
        #define	 	AFE44x0_ALED2CONVEND_100HZ		(	13735	)
        #define	 	AFE44x0_ADCRSTSTCT0_100HZ		(	801	)
        #define	 	AFE44x0_ADCRSTENDCT0_100HZ		(	806	)
        #define	 	AFE44x0_ADCRSTSTCT1_100HZ		(	7269	)
        #define	 	AFE44x0_ADCRSTENDCT1_100HZ		(	7274	)
        #define	 	AFE44x0_ADCRSTSTCT2_100HZ		(	13737	)
        #define	 	AFE44x0_ADCRSTENDCT2_100HZ		(	13742	)
        #define	 	AFE44x0_ADCRSTSTCT3_100HZ		(	20205	)
        #define	 	AFE44x0_ADCRSTENDCT3_100HZ		(	20210	)
	
	#else
		/* 	100Hz, 4% duty cycle 	*/
		#define AFE44x0_PRPCOUNT_100HZ				(39999)
		//LED Pulse counter
		#define AFE44x0_LED1LEDSTC_100HZ				(0x2710)
		#define AFE44x0_LED1LEDENDC_100HZ				(0x36af)
		#define AFE44x0_LED2LEDSTC_100HZ				(0x7530)
		#define AFE44x0_LED2LEDENDC_100HZ				(0x84cf)
		//sample LED start & end counter
		#define AFE44x0_LED1STC_100HZ 				(0x2760)
		#define AFE44x0_LED1ENDC_100HZ 				(0x36ae)
		#define AFE44x0_LED2STC_100HZ 				(0x7580)
		#define AFE44x0_LED2ENDC_100HZ 				(0x84ce)
		//Sample Ambient LED start & end counter
		#define AFE44x0_ALED1STC_100HZ 				(0x4e70)
		#define AFE44x0_ALED1ENDC_100HZ 			(0x5dbe)
		#define AFE44x0_ALED2STC_100HZ 				(0x50)
		#define AFE44x0_ALED2ENDC_100HZ 			(0xf9e)
		//LED convert start & end counter
		#define AFE44x0_LED1CONVST_100HZ 			(0x4e26)
		#define AFE44x0_LED1CONVEND_100HZ 		(0x752f)
		#define AFE44x0_LED2CONVST_100HZ 			(0x6)
		#define AFE44x0_LED2CONVEND_100HZ 		(0x270f)
		//LED Ambient convert start & end counter
		#define AFE44x0_ALED1CONVST_100HZ 			(0x7536)
		#define AFE44x0_ALED1CONVEND_100HZ 			(0x9c3f)
		#define AFE44x0_ALED2CONVST_100HZ 			(0x2716)
		#define AFE44x0_ALED2CONVEND_100HZ 			(0x4e1f)
		//ADC reset 0 start & end
		#define AFE44x0_ADCRSTSTCT0_100HZ 			(0x0)
		#define AFE44x0_ADCRSTENDCT0_100HZ 			(0x05)
		//ADC reset 1 start & end
		#define AFE44x0_ADCRSTSTCT1_100HZ 			(0x2710)
		#define AFE44x0_ADCRSTENDCT1_100HZ 			(0x2715)
		//ADC Reset 2 start & end
		#define AFE44x0_ADCRSTSTCT2_100HZ 			(0x4e20)
		#define AFE44x0_ADCRSTENDCT2_100HZ 			(0x4e25)
		//ADC Reset 3 start & end
		#define AFE44x0_ADCRSTSTCT3_100HZ 			(0x7530)
		#define AFE44x0_ADCRSTENDCT3_100HZ 			(0x7535)
	#endif




/*******************  Bit definition for AFE44x0_REG_CONTROL0 register  ********************/
#define  AFE44x0_CONTROL0_SW_RST            ((unsigned int)0x00000008)           //soft reset
#define  AFE44x0_CONTROL0_DIAG_EN           ((unsigned int)0x00000004)           //diagnose enable
#define  AFE44x0_CONTROL0_TIM_COUNT_RST     ((unsigned int)0x00000002)           //timer counter reset
#define  AFE44x0_CONTROL0_SPI_READ          ((unsigned int)0x00000001)           //SPI read enable

/*******************  Bit definition for AFE44x0_REG_CONTROL1 register  ********************/

#define  AFE44x0_CONTROL1_TIMEREN           ((unsigned int)0x00000100)           //Timer Enable
#define  AFE44x0_CONTROL1_NUMAVG			((unsigned int)0x000000ff)           //Average number


/*******************  Bit definition for AFE44x0_REG_TIA_AMB_GAIN register  ********************/
#define  AFE44x0_TIA_AMB_GAIN_AMBDAC        ((unsigned int)0x000F0000)           //Ambient DAC value
#define  AFE44x0_TIA_AMB_GAIN_STAGE2EN      ((unsigned int)0x00004000)           //Stage 2 enable
#define  AFE44x0_TIA_AMB_GAIN_STG2GAIN      ((unsigned int)0x00000700)           //Stage 2 gain setting
#define  AFE44x0_TIA_AMB_GAIN_CF_LED        ((unsigned int)0x000000F8)           //Program CF for LEDs
#define  AFE44x0_TIA_AMB_GAIN_RF_LED        ((unsigned int)0x00000007)           //Program RF for LEDs

#define AFE44x0_TIA_AMB_GAIN_FLTRCNRSEL	((unsinged int)0x00008000) 			//must be 0


/*******************  Bit definition for AFE44x0_REG_TIAGAIN register  ********************/

#define  AFE44x0_TIAGAIN_ENSEPGAN        ((unsigned int)0x00008000)           //1--Enable seperate RF/CF for LED1/2
#define  AFE44x0_TIAGAIN_STAGE2EN1       ((unsigned int)0x00004000)           //Stage 2 enable
#define  AFE44x0_TIAGAIN_STG2GAIN1       ((unsigned int)0x00000700)           //Stage 2 gain setting
#define  AFE44x0_TIAGAIN_CF_LED1         ((unsigned int)0x000000F8)           //Program CF for LEDs
#define  AFE44x0_TIAGAIN_RF_LED1         ((unsigned int)0x00000007)           //Program RF for LEDs


/*******************  Bit definition for AFE44x0_REG_LEDCNTRL register  ********************/
#define  AFE44x0_LEDCNTRL_LEDRANGE         ((unsigned int)0x00030000)           //config the fullscale of LED output current


#define  AFE44x0_LEDCNTRL_LED1              ((unsigned int)0x0000FF00)           //Program LED current for LED1 signal
#define  AFE44x0_LEDCNTRL_LED2              ((unsigned int)0x000000FF)           //Program LED current for LED2 signal

/*******************  Bit definition for AFE44x0_REG_CONTROL2 register  ********************/
//AFE4403 DEBUG CHANGE
#define  AFE44x0_CONTROL2_DIGOUT_TRISTATE  ((unsigned int)0x00000400)           //Digital output 3-state mode


#define  AFE44x0_CONTROL2_DYNAMIC1		((unsigned int)0x00100000)
#define  AFE44x0_CONTROL2_DYNAMIC2		((unsigned int)0x00004000)
#define  AFE44x0_CONTROL2_DYNAMIC3		((unsigned int)0x00000010)
#define  AFE44x0_CONTROL2_DYNAMIC4		((unsigned int)0x00000008)
#define  AFE44x0_CONTROL2_TX_REF		((unsigned int)0x00060000)
#define  AFE44x0_CONTROL2_EN_SLOW_DIAG	((unsigned int)0x00000100)


#define  AFE44x0_CONTROL2_TXBRGMOD         ((unsigned int)0x00000800)           //Tx bridge mode
#define  AFE44x0_CONTROL2_XTALDIS          ((unsigned int)0x00000200)           //Crystal disable mode
#define  AFE44x0_CONTROL2_PDN_TX           ((unsigned int)0x00000004)           //Tx power-down
#define  AFE44x0_CONTROL2_PDN_RX           ((unsigned int)0x00000002)           //Rx power-down
#define  AFE44x0_CONTROL2_PDN_AFE          ((unsigned int)0x00000001)           //AFE power-down


/*******************  Bit definition for AFE44x0_REG_ALARM register  ********************/

/*******************  Bit definition for Reserved register  ********************/
#define  AFE44x0_SPARE1_SPARE1              ((unsigned int)0x00000000)           //RESERVED 
#define  AFE44x0_TIAGAIN_TIAGAIN            ((unsigned int)0x00000000)           //RESERVED FOR Factory
#define  AFE44x0_SPARE2_SPARE2              ((unsigned int)0x00000000)           //RESERVED 
#define  AFE44x0_SPARE3_SPARE3              ((unsigned int)0x00000000)           //RESERVED 
#define  AFE44x0_SPARE4_SPARE4              ((unsigned int)0x00000000)           //RESERVED 
#define  AFE44x0_RESERVED1_RESERVED1        ((unsigned int)0x00000000)           //RESERVED 
#define  AFE44x0_RESERVED2_RESERVED2        ((unsigned int)0x00000000)           //RESERVED 

/*******************  Bit definition for AFE44x0_REG_DIAG register  ********************/
#define  AFE44x0_DIAG_PD_ALM               ((unsigned int)0x00001000)           //Power-down alarm status diagnostic flag
#define  AFE44x0_DIAG_LED_ALM              ((unsigned int)0x00000800)           //LED alarm status diagnostic flag
#define  AFE44x0_DIAG_LED1OPEN             ((unsigned int)0x00000400)           //LED1 open diagnostic flag
#define  AFE44x0_DIAG_LED2OPEN             ((unsigned int)0x00000200)           //LED2 open diagnostic flag
#define  AFE44x0_DIAG_LEDSC                ((unsigned int)0x00000100)           //LED short diagnostic flag
#define  AFE44x0_DIAG_OUTPSHGND            ((unsigned int)0x00000080)           //OUTP to GND diagnostic flag
#define  AFE44x0_DIAG_OUTNSHGND            ((unsigned int)0x00000040)           //OUTN to GND diagnostic flag
#define  AFE44x0_DIAG_PDOC                 ((unsigned int)0x00000020)           //PD open diagnostic flag
#define  AFE44x0_DIAG_PDSC                 ((unsigned int)0x00000010)           //PD short diagnostic flag
#define  AFE44x0_DIAG_INNSCGND             ((unsigned int)0x00000008)           //INN to GND diagnostic flag
#define  AFE44x0_DIAG_INPSCGND             ((unsigned int)0x00000004)           //INP to GND diagnostic flag
#define  AFE44x0_DIAG_INNSCLED             ((unsigned int)0x00000002)           //INN to LED diagnostic flag
#define  AFE44x0_DIAG_INPSCLED             ((unsigned int)0x00000001)           //INP to LED diagnostic flag

/*******************  Bit definition for AFE44x0_REG_CONTROL3 register  ********************/

#define  AFE44x0_CONTROL3_TX3_MODE		((unsigned int)0x00008000)
#define  AFE44x0_CONTROL3_SOMI_TRI		((unsigned int)0x00000010)
#define  AFE44x0_CONTROL3_CLKOUT_TRI	((unsigned int)0x00000008)
#define  AFE44x0_CONTROL3_CLKDIV		((unsigned int)0x00000007)


/*******************  Bit definition for AFE44x0_REG_PDNCYCLESTC register  ********************/
#define  AFE44x0_PDNCYCLESTC_PDNCYCLESTC		((unsigned int)0x0000FFFF)


/*******************  Bit definition for AFE44x0_REG_PDNCYCLEENDC register  ********************/
#define  AFE44x0_PDNCYCLEENDC_PDNCYCLEENDC		((unsigned int)0x0000FFFF)

/******************   Bit definition for one data 16 bits register      *******************/
#define  AFE44x0_LED2STC_LED2STC                 ((unsigned int)0x0000FFFF)           //Sample LED2 start count
#define  AFE44x0_LED2ENDC_LED2ENDC               ((unsigned int)0x0000FFFF)           //Sample LED2 end count
#define  AFE44x0_LED2LEDSTC_LED2LEDSTC           ((unsigned int)0x0000FFFF)           //LED2 start count
#define  AFE44x0_LED2LEDENDC_LED2LEDENDC         ((unsigned int)0x0000FFFF)           //LED2 end count
#define  AFE44x0_ALED2STC_ALED2STC               ((unsigned int)0x0000FFFF)           //Sample Ambient LED2 start count
#define  AFE44x0_ALED2ENDC_ALED2ENDC             ((unsigned int)0x0000FFFF)           //Sample Ambient LED2 end count
#define  AFE44x0_LED1STC_LED1STC                 ((unsigned int)0x0000FFFF)           //Sample LED1 start count
#define  AFE44x0_LED1ENDC_LED1ENDC               ((unsigned int)0x0000FFFF)           //Sample LED1 end count
#define  AFE44x0_LED1LEDSTC_LED1LEDSTC           ((unsigned int)0x0000FFFF)           //LED1 start count
#define  AFE44x0_LED1LEDENDC_LED1LEDENDC         ((unsigned int)0x0000FFFF)           //LED1 end count
#define  AFE44x0_ALED1STC_ALED1STC               ((unsigned int)0x0000FFFF)           //Sample Ambient LED1 start count
#define  AFE44x0_ALED1ENDC_ALED1ENDC             ((unsigned int)0x0000FFFF)           //Sample Ambient LED1 end count
#define  AFE44x0_LED2CONVST_LED2CONVST           ((unsigned int)0x0000FFFF)           //LED2 convert start count
#define  AFE44x0_LED2CONVEND_LED2CONVEND         ((unsigned int)0x0000FFFF)           //LED2 convert end count
#define  AFE44x0_ALED2CONVST_ALED2CONVST         ((unsigned int)0x0000FFFF)           //LED2 ambient convert start count
#define  AFE44x0_ALED2CONVEND_ALED2CONVEND       ((unsigned int)0x0000FFFF)           //LED2 ambient convert end count
#define  AFE44x0_LED1CONVST_LED1CONVST           ((unsigned int)0x0000FFFF)           //LED1 convert start count
#define  AFE44x0_LED1CONVEND_LED1CONVEND         ((unsigned int)0x0000FFFF)           //LED1 convert end count
#define  AFE44x0_ALED1CONVST_ALED1CONVST         ((unsigned int)0x0000FFFF)           //LED1 ambient convert start count
#define  AFE44x0_ALED1CONVEND_ALED1CONVEND       ((unsigned int)0x0000FFFF)           //LED1 ambient convert end count
#define  AFE44x0_ADCRSTSTCT0_ADCRSTSTCT0         ((unsigned int)0x0000FFFF)           //ADC RESET 0 start count
#define  AFE44x0_ADCRSTENDCT0_ADCRSTENDCT0       ((unsigned int)0x0000FFFF)           //ADC RESET 0 end count
#define  AFE44x0_ADCRSTSTCT1_ADCRSTSTCT1         ((unsigned int)0x0000FFFF)           //ADC RESET 1 start count
#define  AFE44x0_ADCRSTENDCT1_ADCRSTENDCT1       ((unsigned int)0x0000FFFF)           //ADC RESET 1 end count
#define  AFE44x0_ADCRSTSTCT2_ADCRSTSTCT2         ((unsigned int)0x0000FFFF)           //ADC RESET 2 start count
#define  AFE44x0_ADCRSTENDCT2_ADCRSTENDCT2       ((unsigned int)0x0000FFFF)           //ADC RESET 2 end count
#define  AFE44x0_ADCRSTSTCT3_ADCRSTSTCT3         ((unsigned int)0x0000FFFF)           //ADC RESET 3 start count
#define  AFE44x0_ADCRSTENDCT3_ADCRSTENDCT3       ((unsigned int)0x0000FFFF)           //ADC RESET 3 end count
#define  AFE44x0_PRPCOUNT_PRPCOUNT               ((unsigned int)0x0000FFFF)           //Pulse repetition period count

/******************   Bit definition for one data 24 bits register      *******************/
#define AFE44x0_LED2VAL_LED2VAL                       ((unsigned int)0x00FFFFFF)            // 42 LED2VAL[23:0]
#define AFE44x0_ALED2VAL_ALED2VAL                     ((unsigned int)0x00FFFFFF)            // 43 ALED2VAL[23:0]
#define AFE44x0_LED1VAL_LED1VAL                       ((unsigned int)0x00FFFFFF)            // 44 LED1VAL[23:0]
#define AFE44x0_ALED1VAL_ALED1VAL                     ((unsigned int)0x00FFFFFF)            // 45 ALED1VAL[23:0]
#define AFE44x0_LED2_SUB_ALED2VAL_LED2_SUB_ALED2VAL   ((unsigned int)0x00FFFFFF)            // 46 LED2-ALED2VAL[23:0]
#define AFE44x0_LED1_SUB_ALED1VAL_LED1_SUB_ALED1VAL   ((unsigned int)0x00FFFFFF)            // 47 LED1-ALED1VAL[23:0]



/****************************************************************/
/* AFE44x0 Register Address Definition*/
/****************************************************************/
#define AFE44x0_REG_CONTROL0		0x00		//0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 SW_RST DIAG_EN SPI_READ TIM_COUNT_RST 
#define AFE44x0_REG_LED2STC		0x01 			// 1 0 0 0 0 0 0 0 0 LED2STC[15:0]
#define AFE44x0_REG_LED2ENDC		0x02		// 2 0 0 0 0 0 0 0 0 LED2ENDC[15:0]
#define AFE44x0_REG_LED2LEDSTC		0x03		// 3 0 0 0 0 0 0 0 0 LED2LEDSTC[15:0]
#define AFE44x0_REG_LED2LEDENDC		0x04		// 4 0 0 0 0 0 0 0 0 LED2LEDENDC[15:0]
#define AFE44x0_REG_ALED2STC		0x05		// 5 0 0 0 0 0 0 0 0 ALED2STC[15:0]
#define AFE44x0_REG_ALED2ENDC		0x06		// 6 0 0 0 0 0 0 0 0 ALED2ENDC[15:0]
#define AFE44x0_REG_LED1STC		0x07		// 7 0 0 0 0 0 0 0 0 LED1STC[15:0]
#define AFE44x0_REG_LED1ENDC		0x08		// 8 0 0 0 0 0 0 0 0 LED1ENDC[15:0]
#define AFE44x0_REG_LED1LEDSTC		0x09		// 9 0 0 0 0 0 0 0 0 LED1LEDSTC[15:0]
#define AFE44x0_REG_LED1LEDENDC		0x0A		// 10 0 0 0 0 0 0 0 0 LED1LEDENDC[15:0]
#define AFE44x0_REG_ALED1STC		0x0B		// 11 0 0 0 0 0 0 0 0 ALED1STC[15:0]
#define AFE44x0_REG_ALED1ENDC		0x0C		// 12 0 0 0 0 0 0 0 0 ALED1ENDC[15:0]
#define AFE44x0_REG_LED2CONVST		0x0D		// 13 0 0 0 0 0 0 0 0 LED2CONVST[15:0]
#define AFE44x0_REG_LED2CONVEND		0x0E		// 14 0 0 0 0 0 0 0 0 LED2CONVEND[15:0]
#define AFE44x0_REG_ALED2CONVST		0x0F		// 15 0 0 0 0 0 0 0 0 ALED2CONVST[15:0]
#define AFE44x0_REG_ALED2CONVEND	0x10		// 16 0 0 0 0 0 0 0 0 ALED2CONVEND[15:0]
#define AFE44x0_REG_LED1CONVST		0x11		// 17 0 0 0 0 0 0 0 0 LED1CONVST[15:0]
#define AFE44x0_REG_LED1CONVEND		0x12		// 18 0 0 0 0 0 0 0 0 LED1CONVEND[15:0]
#define AFE44x0_REG_ALED1CONVST		0x13		// 19 0 0 0 0 0 0 0 0 ALED1CONVST[15:0]
#define AFE44x0_REG_ALED1CONVEND	0x14		// 20 0 0 0 0 0 0 0 0 ALED1CONVEND[15:0]
#define AFE44x0_REG_ADCRSTSTCT0		0x15		// 21 0 0 0 0 0 0 0 0 ADCRSTCT0[15:0]
#define AFE44x0_REG_ADCRSTENDCT0	0x16		// 22 0 0 0 0 0 0 0 0 ADCRENDCT0[15:0]
#define AFE44x0_REG_ADCRSTSTCT1		0x17		// 23 0 0 0 0 0 0 0 0 ADCRSTCT1[15:0]
#define AFE44x0_REG_ADCRSTENDCT1	0x18		// 24 0 0 0 0 0 0 0 0 ADCRENDCT1[15:0]
#define AFE44x0_REG_ADCRSTSTCT2		0x19		// 25 0 0 0 0 0 0 0 0 ADCRSTCT2[15:0]
#define AFE44x0_REG_ADCRSTENDCT2	0x1A		// 26 0 0 0 0 0 0 0 0 ADCRENDCT2[15:0]
#define AFE44x0_REG_ADCRSTSTCT3		0x1B		// 27 0 0 0 0 0 0 0 0 ADCRSTCT3[15:0]
#define AFE44x0_REG_ADCRSTENDCT3	0x1C		// 28 0 0 0 0 0 0 0 0 ADCRENDCT3[15:0]
#define AFE44x0_REG_PRPCOUNT		0x1D		// 29 0 0 0 0 0 0 0 0 PRPCT[15:0]
#define AFE44x0_REG_CONTROL1		0x1E		//
#define AFE44x0_REG_SPARE1		0x1F		//31 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
#define AFE44x0_REG_TIAGAIN		0x20 		//32 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
#define AFE44x0_REG_TIA_AMB_GAIN	0x21		//33 0 0 0 0 AMBDAC[3:0] 0 0 0 0 STG2GAIN[2:0] CF_LED[4:0] RF_LED[2:0] STAGE2EN
#define AFE44x0_REG_LEDCNTRL		0x22		//34 0 0 0 0 0 0 1 LED1[7:0] LED2[7:0] LEDCUROFF
#define AFE44x0_REG_CONTROL2		0x23		// 35 0 0 0 0 0 0 1 0 0 0 0 0 1 0 0 0 0 0 PDNTX PDNRX PDNAFE XTALDIS TXBRGMOD DIGOUT_TRISTATE
#define AFE44x0_REG_SPARE2		0x24		// 36 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
#define AFE44x0_REG_SPARE3		0x25		// 37 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
#define AFE44x0_REG_SPARE4		0x26		// 38 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
#define AFE44x0_REG_RESERVED1		0x27		// 39 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
#define AFE44x0_REG_RESERVED2		0x28		// 40 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
#define AFE44x0_REG_ALARM		0x29		// 41 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ALMPINCLKEN
#define AFE44x0_REG_LED2VAL		0x2A		// 42 LED2VAL[23:0]
#define AFE44x0_REG_ALED2VAL		0x2B		// 43 ALED2VAL[23:0]
#define AFE44x0_REG_LED1VAL		0x2C		// 44 LED1VAL[23:0]
#define AFE44x0_REG_ALED1VAL		0x2D		// 45 ALED1VAL[23:0]
#define AFE44x0_REG_LED2_SUB_ALED2VAL	0x2E		// 46 LED2-ALED2VAL[23:0]
#define AFE44x0_REG_LED1_SUB_ALED1VAL	0x2F		// 47 LED1-ALED1VAL[23:0]
#define AFE44x0_REG_DIAG		0x30 		// 48 0 0 0 0 0 0 0 0 0 0 0 PDOC PDSC LEDSC PD_ALM LED_ALM INPSCLED INNSCLED INPSCGND INNSCGND LED1OPEN LED2OPEN OUTPSHGND OUT 
/*Register for AFE4403*/

#define AFE44x0_REG_CONTROL3 		0x31
#define AFE44x0_REG_PDNCYCLESTC 	0x32
#define AFE44x0_REG_PDNCYCLEENDC	0x33



/****************************************************************/
/* AFE44x0 Register Function Definition*/
/****************************************************************/

/** @defgroup AFE44x0_CLKALMPIN 
  * @{
  */
#define AFE44x0_CLKALMPIN_SLED2_SLED1			((unsigned int)0x00000000)			//PD_ALM(Sample LED2 Pulse) LED_ALM(Sample LED1 Pulse)
#define AFE44x0_CLKALMPIN_LED2_LED1				((unsigned int)0x00000200)			//PD_ALM(LED2 LED Pulse) LED_ALM(LED1 LED Pulse)
#define AFE44x0_CLKALMPIN_SLED2A_SLED1A			((unsigned int)0x00000400)			//PD_ALM(Sample LED2 ambient Pulse) LED_ALM(Sample LED1 ambient Pulse)
#define AFE44x0_CLKALMPIN_LED2C_LED1C			((unsigned int)0x00000600)			//PD_ALM(LED2 Convert) LED_ALM(LED1 Convert)
#define AFE44x0_CLKALMPIN_LED2AC_LED1AC			((unsigned int)0x00000800)			//PD_ALM(LED2 Ambient convert ) LED_ALM(LED1 ambient)
#define AFE44x0_CLKALMPIN_LEDNO_LEDNO			((unsigned int)0x00000a00)| \
												((unsigned int)0x00000c00)| \
												((unsigned int)0x00000e00)			//PD_ALM(no output) LED_ALM(no output)
#define IS_AFE44x0_CLKALMPIN_MODE(MODE)			(((MODE) == AFE44x0_CLKALMPIN_SLED2_SLED1) || \
                                    			((MODE) == AFE44x0_CLKALMPIN_LED2_LED1) || \
                                    			((MODE) == AFE44x0_CLKALMPIN_SLED2A_SLED1A) || \
                                    			((MODE) == AFE44x0_CLKALMPIN_LED2C_LED1C) || \
                                    			((MODE) == AFE44x0_CLKALMPIN_LED2AC_LED1AC) || \
                                    			((MODE) == AFE44x0_CLKALMPIN_LEDNO_LEDNO))



/** @defgroup AFE44x0_AMBDAC 
  * @{
  */
#define AFE44x0_AMBDAC_Current_0				((unsigned int)0x00000000)			//0uA
#define AFE44x0_AMBDAC_Current_1				((unsigned int)0x00010000)			//1uA
#define AFE44x0_AMBDAC_Current_2				((unsigned int)0x00020000)			//2uA
#define AFE44x0_AMBDAC_Current_3				((unsigned int)0x00030000)			//3uA
#define AFE44x0_AMBDAC_Current_4				((unsigned int)0x00040000)			//4uA
#define AFE44x0_AMBDAC_Current_5				((unsigned int)0x00050000)			//5uA
#define AFE44x0_AMBDAC_Current_6				((unsigned int)0x00060000)			//6uA
#define AFE44x0_AMBDAC_Current_7				((unsigned int)0x00070000)			//7uA
#define AFE44x0_AMBDAC_Current_8				((unsigned int)0x00080000)			//8uA
#define AFE44x0_AMBDAC_Current_9				((unsigned int)0x00090000)			//9uA
#define AFE44x0_AMBDAC_Current_10				((unsigned int)0x000a0000)			//10uA

#define IS_AFE44x0_AMBDAC_Current(Current)		(((Current) == AFE44x0_AMBDAC_Current_0) || \
                                    			((Current) == AFE44x0_AMBDAC_Current_1) || \
                                    			((Current) == AFE44x0_AMBDAC_Current_2) || \
                                    			((Current) == AFE44x0_AMBDAC_Current_3) || \
                                    			((Current) == AFE44x0_AMBDAC_Current_4) || \
                                    			((Current) == AFE44x0_AMBDAC_Current_5) || \
                                    			((Current) == AFE44x0_AMBDAC_Current_6) || \
                                    			((Current) == AFE44x0_AMBDAC_Current_7) || \
                                    			((Current) == AFE44x0_AMBDAC_Current_8) || \
                                    			((Current) == AFE44x0_AMBDAC_Current_9) || \
                                    			((Current) == AFE44x0_AMBDAC_Current_10))


/** @defgroup AFE44x0_STG2GAIN 
  * @{
  */
#define AFE44x0_STG2GAIN_0dB					(0x0<<8)			//0dB, linear 1
#define AFE44x0_STG2GAIN_3_5dB				(0x1<<8)			//3.5dB, linear 1.5
#define AFE44x0_STG2GAIN_6dB					(0x2<<8)			//6dB, linear 2
#define AFE44x0_STG2GAIN_9_5dB				(0x3<<8)			//9.5dB, linear 3
#define AFE44x0_STG2GAIN_12dB					(0x4<<8)			//12dB, linear 4

#define IS_AFE44x0_STG2GAIN(GAIN)		(((GAIN) == AFE44x0_STG2GAIN_0dB) || \
                                    			((GAIN) == AFE44x0_STG2GAIN_3_5dB) || \
                                    			((GAIN) == AFE44x0_STG2GAIN_6dB) || \
                                    			((GAIN) == AFE44x0_STG2GAIN_9_5dB) || \
                                    			((GAIN) == AFE44x0_STG2GAIN_12dB))


/** @defgroup AFE44x0_CF_LED 
  * @{
  */
#define AFE44x0_CF_LED_5pF						0x00000000			//5pF
#define AFE44x0_CF_LED_5pF_Plus_5pF				0x00000008			//5pF+5pF
#define AFE44x0_CF_LED_15pF_Plus_5pF			0x00000010			//15pF+5pF
#define AFE44x0_CF_LED_25pF_Plus_5pF			0x00000020			//25pF+5pF
#define AFE44x0_CF_LED_50pF_Plus_5pF			0x00000040			//50pF+5pF
#define AFE44x0_CF_LED_150pF_Plus_5pF			0x00000080			//150pF+5pF
#define AFE44x0_CF_LED_100pF_Plus_5pF			0x00000078 			//100pF+5pF
#define AFE44x0_CF_LED_200pF_Plus_5pF			0x000000C0			//210pF
#define IS_AFE44x0_CF_LED(Cap)		(((Cap) == AFE44x0_CF_LED_5pF) || \
                                    			((Cap) == AFE44x0_CF_LED_5pF_Plus_5pF) || \
                                    			((Cap) == AFE44x0_CF_LED_15pF_Plus_5pF) || \
                                    			((Cap) == AFE44x0_CF_LED_25pF_Plus_5pF) || \
                                    			((Cap) == AFE44x0_CF_LED_50pF_Plus_5pF) || \
                                    			((Cap) == AFE44x0_CF_LED_150pF_Plus_5pF))


/** @defgroup AFE44x0_RF_LED 
  * @{
  */
#define AFE44x0_RF_LED_500K						0x00000000			//500K OHM
#define AFE44x0_RF_LED_250K						0x00000001			//250K OHM
#define AFE44x0_RF_LED_100K						0x00000002			//100K OHM
#define AFE44x0_RF_LED_50K						0x00000003			//50K OHM
#define AFE44x0_RF_LED_25K						0x00000004			//25K OHM
#define AFE44x0_RF_LED_10K						0x00000005			//10K OHM
#define AFE44x0_RF_LED_1M						  0x00000006			//1M OHM

#define IS_AFE44x0_RF_LED(Res)		(((Res) == AFE44x0_RF_LED_500K) || \
                                    			((Res) == AFE44x0_RF_LED_250K) || \
                                    			((Res) == AFE44x0_RF_LED_100K) || \
                                    			((Res) == AFE44x0_RF_LED_50K) || \
                                    			((Res) == AFE44x0_RF_LED_25K) || \
                                    			((Res) == AFE44x0_RF_LED_10K) || \
                                    			((Res) == AFE44x0_RF_LED_1M))


/** @defgroup AFE44x0_TXBRGMOD 
  * @{
  */
#define AFE44x0_TXBRGMOD_H_Bridge						0x00000000			//LED driver is configured as H-Bridge
#define AFE44x0_TXBRGMOD_Push_Pull					0x00000800			//LED driver is configured as push pull
#define IS_AFE44x0_TXBRGMOD(MODE)		(((MODE) == AFE44x0_TXBRGMOD_H_Bridge) || \
                                    			((MODE) == AFE44x0_TXBRGMOD_Push_Pull))

/** @defgroup AFE44x0_DIGOUT_TRISTATE 
  * @{
  */
#define AFE44x0_DIGOUT_TRISTATE_Normal						0x00000000			//digital out pin in normal mode when no selected
#define AFE44x0_DIGOUT_TRISTATE_Tristate					0x00000400			//digital out pin in tristate mode when no selected
#define IS_AFE44x0_DIGOUT_TRISTATE(MODE)		(((MODE) == AFE44x0_DIGOUT_TRISTATE_Normal) || \
                                    			((MODE) == AFE44x0_DIGOUT_TRISTATE_Tristate))


/** @defgroup AFE44x0_XTALDIS 
  * @{
  */
#define AFE44x0_XTALDIS_Crystal						0x00000000			//crystal module enable, 8MHz crystal
#define AFE44x0_XTALDIS_External					0x00000200			//crystal module disable, 8MHz external clock source
#define IS_AFE44x0_XTALDIS(MODE)		(((MODE) == AFE44x0_XTALDIS_Crystal) || \
                                    			((MODE) == AFE44x0_XTALDIS_External))

/** @defgroup AFE44x0_TX_POWER 
  * @{
  */
#define AFE44x0_TX_POWER_ON						0x00000000			//TX power on
#define AFE44x0_TX_POWER_OFF					0x00000004			//TX power off
#define IS_AFE44x0_TX_POWER(MODE)		(((MODE) == AFE44x0_TX_POWER_ON) || \
                                    			((MODE) == AFE44x0_TX_POWER_OFF))

/** @defgroup AFE44x0_RX_POWER 
  * @{
  */
#define AFE44x0_RX_POWER_ON						0x00000000			//RX power on
#define AFE44x0_RX_POWER_OFF					0x00000002			//RX power off
#define IS_AFE44x0_RX_POWER(MODE)		(((MODE) == AFE44x0_RX_POWER_ON) || \
                                    			((MODE) == AFE44x0_RX_POWER_OFF))

/** @defgroup AFE44x0_AFE_POWER 
  * @{
  */
#define AFE44x0_AFE_POWER_ON					0x00000000			//RX power on
#define AFE44x0_AFE_POWER_OFF					0x00000001			//RX power off
#define IS_AFE44x0_AFE_POWER(MODE)		(((MODE) == AFE44x0_AFE_POWER_ON) || \
                                    			((MODE) == AFE44x0_AFE_POWER_OFF))

/** @defgroup AFE44x0_ALMPINCLKEN 
  * @{
  */
#define AFE44x0_ALMPINCLKEN_DISABLE			0x00000000			//Alarm pin clock disable
#define AFE44x0_ALMPINCLKEN_ENABLE			0x00000080			//Alarm pin clock enable
#define IS_AFE44x0_ALMPINCLKEN(MODE)		(((MODE) == AFE44x0_ALMPINCLKEN_DISABLE) || \
                                    			((MODE) == AFE44x0_ALMPINCLKEN_ENABLE))


/** @defgroup AFE44x0_LEDCUROFF 
  * @{
  */
#define AFE44x0_LEDCUROFF_ON					0x00000000			//LED Current on
#define AFE44x0_LEDCUROFF_OFF					0x00020000			//LED current off
#define IS_AFE44x0_LEDCUROFF(MODE)		(((MODE) == AFE44x0_LEDCUROFF_ON) || \
                                    			((MODE) == AFE44x0_LEDCUROFF_OFF))


/** @defgroup AFE44x0_STAGE2EN 
  * @{
  */
#define AFE44x0_STAGE2EN_BYPASS				0x00000000			//Stage 2 amplifier bypass
#define AFE44x0_STAGE2EN_ENABLE				0x00004000			//Stage 2 amplifier enable
#define IS_AFE44x0_STAGE2EN(MODE)		(((MODE) == AFE44x0_STAGE2EN_BYPASS) || \
                                    			((MODE) == AFE44x0_STAGE2EN_ENABLE))

/** @defgroup AFE44x0_TIMEREN 
  * @{
  */
#define AFE44x0_TIMEREN_DISABLE				0x00000000			//timer module disable
#define AFE44x0_TIMEREN_ENABLE				0x00000100			//timer module enable
#define IS_AFE44x0_TIMEREN(MODE)		(((MODE) == AFE44x0_TIMEREN_ENABLE) || \
                                    			((MODE) == AFE44x0_TIMEREN_DISABLE))


/** @defgroup flag when diagnose 
  * @{
  */
#define AFE44x0_DIAG_FLAG_INPSCLED		0x00000001             //
#define AFE44x0_DIAG_FLAG_INNSCLED		0x00000002             //
#define AFE44x0_DIAG_FLAG_INPSCGND		0x00000004             //
#define AFE44x0_DIAG_FLAG_INNSCGND		0x00000008             //
#define AFE44x0_DIAG_FLAG_PDSC			0x00000010             //
#define AFE44x0_DIAG_FLAG_PDOC			0x00000020             //
#define AFE44x0_DIAG_FLAG_OUTNSHGND		0x00000040             //
#define AFE44x0_DIAG_FLAG_OUTPSHGND		0x00000080             //
#define AFE44x0_DIAG_FLAG_LEDSC			0x00000100             //
#define AFE44x0_DIAG_FLAG_LED2OPEN		0x00000200             //
#define AFE44x0_DIAG_FLAG_LED1OPEN		0x00000400             //
#define AFE44x0_DIAG_FLAG_LED_ALM		0x00000800             //
#define AFE44x0_DIAG_FLAG_PD_ALM		0x00001000             //
#define IS_AFE44x0_DIAG_FLAG(FLAG)		(((FLAG) == AFE44x0_DIAG_FLAG_INPSCLED) || \
                                    			((FLAG) == AFE44x0_DIAG_FLAG_INNSCLED) || \
                                    			((FLAG) == AFE44x0_DIAG_FLAG_INPSCGND) || \
                                    			((FLAG) == AFE44x0_DIAG_FLAG_INNSCGND) || \
                                    			((FLAG) == AFE44x0_DIAG_FLAG_PDSC) || \
                                    			((FLAG) == AFE44x0_DIAG_FLAG_PDOC) || \
                                    			((FLAG) == AFE44x0_DIAG_FLAG_OUTNSHGND) || \
                                    			((FLAG) == AFE44x0_DIAG_FLAG_OUTPSHGND) || \
                                    			((FLAG) == AFE44x0_DIAG_FLAG_LEDSC) || \
                                    			((FLAG) == AFE44x0_DIAG_FLAG_LED2OPEN) || \
                                    			((FLAG) == AFE44x0_DIAG_FLAG_LED1OPEN) || \
                                    			((FLAG) == AFE44x0_DIAG_FLAG_LED_ALM) || \
                                    			((FLAG) == AFE44x0_DIAG_FLAG_PD_ALM))

/*  Added register bit definition for AFE4403*/
	/** @defgroup AFE4403_REG_TX3_MODE
	  * @{
	  */
	#define AFE44x0_TX3_MODE_ENABLE 		0x00008000  		//Enable LED3
	#define AFE44x0_TX3_MODE_DISABLE 		0x00000000  		//Disable LED3

	/** @defgroup AFE4403_REG_SOMI_TRI
	  * @{
	  */
	#define AFE44x0_SOMI_TRI_ENABLE 		0x00000010  		//Place SOMI in tri state
	#define AFE44x0_SOMI_TRI_DISABLE 		0x00000000  		//Place SOMI in active mode

	/** @defgroup AFE4403_REG_CLKOUT_TRI
	  * @{
	  */
	#define AFE44x0_CLKOUT_TRI_ENABLE 		0x00000008  		//Place CLKOUT in tri state
	#define AFE44x0_CLKOUT_TRI_DISABLE 		0x00000000  		//Place CLKOUT in active mode


	/** @defgroup AFE44x0_AFE_TX_REF 
	  * @{
	  */
	#define AFE44x0_AFE_TX_REF_025V					0x00000000			//0.25V
	#define AFE44x0_AFE_TX_REF_05V					0x00020000			//0.5V
	#define AFE44x0_AFE_TX_REF_10V					0x00040000			//1.0V
	#define AFE44x0_AFE_TX_REF_075V					0x00060000			//0.75V


	/** @defgroup AFE4403_REG_CLKDIV
	  * @{
	  */
	#define AFE44x0_CLKDIV_2 		0x00000000  		//DIV 2, CLOCK RANGE 8~12MHz
	#define AFE44x0_CLKDIV_4 		0x00000002  		//DIV 4, CLOCK RANGE 16~24MHz
	#define AFE44x0_CLKDIV_6 		0x00000003  		//DIV 6, CLOCK RANGE 24~36MHz
	#define AFE44x0_CLKDIV_8 		0x00000004  		//DIV 8, CLOCK RANGE 32~48MHz
	#define AFE44x0_CLKDIV_12 		0x00000005  		//DIV 12, CLOCK RANGE 48~60MHz
	#define AFE44x0_CLKDIV_1 		0x00000007  		//DIV 1, CLOCK RANGE 4~6MHz
	#define IS_AFE44x0_CLKDIV(MODE)		(((MODE) == AFE44x0_CLKDIV_2) || \
	                                    ((MODE) == AFE44x0_CLKDIV_4) || \
	                                    ((MODE) == AFE44x0_CLKDIV_6) || \
	                                    ((MODE) == AFE44x0_CLKDIV_8) || \
	                                    ((MODE) == AFE44x0_CLKDIV_12) || \
	                                    ((MODE) == AFE44x0_CLKDIV_1)) 
	/** @defgroup AFE44x0_CONTROL2_EN_SLOW_DIAG
	  * @{
	  */
	#define AFE44x0_SLOW_DIAG_MODE 		0x00000000			//enable fast mode, 8ms
	#define AFE44x0_FAST_DIAG_MODE		0x00000100 			//enable slow mode, 16ms

	/** @defgroup AFE44x0_CONTROL2_DYNAMIC1/2/3/4
	  * @{
	  */
	#define AFE44x0_DYNAMIC1_ENABLE 		0x00100000			//enable 
	#define AFE44x0_DYNAMIC1_DISABLE		0x00000000 			//disable
	
	#define AFE44x0_DYNAMIC2_ENABLE 		0x00004000			//enable 
	#define AFE44x0_DYNAMIC2_DISABLE		0x00000000 			//disable
	
	#define AFE44x0_DYNAMIC3_ENABLE 		0x00000010			//enable 
	#define AFE44x0_DYNAMIC3_DISABLE		0x00000000 			//disable
	
	#define AFE44x0_DYNAMIC4_ENABLE 		0x00000008			//enable 
	#define AFE44x0_DYNAMIC4_DISABLE		0x00000000 			//disable




/****************************************************************/
/*              Functions Added for Control                     */
/****************************************************************/



AFE4403_FUNC_RETURN_TYPE afe4403_timing_init_func(Afe4403TimingTypedef * Afe4403Timing_pst);
void afe4403_reg_write (unsigned char ui8RegAddr, unsigned int ui32DataIn);
unsigned int afe4403_reg_read(unsigned char ui8RegAddr);
void afe4403_rd_sample_reg(signed int * pi32DatBuf);
AFE4403_FUNC_RETURN_TYPE afe4403_wr_reg_mask(unsigned char ui8RegAddr, unsigned int ui32Mask, unsigned int ui32DatIn);

/*  Functional functions    */
void afe4403_set_average_num(unsigned int ui32AveNum);
void afe4403_diag_enable(void);
void cc_afe4403_enable_running(void);

void cc_afe4403_power_saving_mode_disable(void);
void cc_afe4403_power_saving_mode_enable(void);
unsigned int cc_afe4403_get_diag_status(void);

void cc_afe4403_led_curr_set(unsigned char ui8LED1Cur, unsigned char ui8LED2Cur);
void cc_afe4403_cf_set(unsigned int ui32CfVal);
void cc_afe4403_rf_set(unsigned int ui32RfVal);
void cc_afe4403_Dac_set(unsigned int ui32DacVal);

void cc_afe4403_disable_cur_cancel(void);
void cc_afe4403_enable_cur_cancel(void);
void cc_afe4403_sec_gain_set(unsigned int ui32SecGainVal);

AFE4403_FUNC_RETURN_TYPE Afe4403_Inst_Init(Afe4403InstTypedef * Afe4403InstInit_pst);
void cc_afe4403_dump_reg(void);
void cc_afe4403_poweron_init(void);

#endif /*AFE44x0_H_*/
