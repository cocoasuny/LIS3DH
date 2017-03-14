#ifndef __PLATFORM_H
#define __PLATFORM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include <stdio.h>

/************** 	116k Flash Space Definition 		************/
/*

	0x08003000 			-----------------------------
						-	Boot version(4Byte)     -
	0x08003004          -----------------------------
	0x08003005          -----------------------------
	                    -   Bootloader code         -
	0x08003f00          -----------------------------
	0x08003F80			-----------------------------
	                    -	SN Num(256Bytes)        -
	0x08003FFF          -----------------------------
	0x08004000          -----------------------------
	                    -   Application Code        -
	0x08020CFF          -----------------------------
	0x08020D00          -----------------------------
	                    -   Blank                   -
	0x08020EFF          -----------------------------
	0x08020F00          -----------------------------
	                    -   NTC TEMP coef(2000Byte) -
	0x080216EF          -----------------------------
	0x080216D0          -----------------------------
	                    -   NTC RT coef(2000Byte)   -
	0x08021ECF          -----------------------------
	0x08021ED0          -----------------------------
	                    -       Blank(300Bytes)  	-
	0x08021FFF          -----------------------------

	0x08022000          -----------------------------
                        -       FW Update Buf 		-
	0x0803ECFF          -----------------------------
	0x0803ED00          -----------------------------
	                    -   Blank                   -
	0x0803EEFF          -----------------------------
	0x0803EF00          -----------------------------
                        -       Reserve 			-
	0x0803FFFF          -----------------------------


*/

/* define the address from where user application will be loaded,
   the application address should be a start sector address */
#define APP_START_ADDRESS           (uint32_t)0x8004000
#define FWUPDATE_BUFFER_START_ADDRESS    (uint32_t)0x8022000

/* define the address from where IAP will be loaded, 0x08000000:BANK1 or
   0x08030000:BANK2 */
#define FLASH_START_ADDRESS         (uint32_t)0x08000000

#define FWUPDATE_BUF_FLASH_LAST_PAGE_ADDRESS             (uint32_t)0x0803EC00
/* define the address from where user application will be loaded,
   the application address should be a start sector address */
#define FWUPDATE_BUF_FLASH_END_ADDRESS      (uint32_t)0x0803ECFF



/***********STM32 Internal EEPROM Space(1Kbyte) Definition****************************/
#define EEPROM_CONTACT_TEMP_OFFSET          DATA_EEPROM_START_ADDR + 256*0
#define EEPROM_NON_CONTACT_TEMP_OFFSET      DATA_EEPROM_START_ADDR + 256*1
#define EEPROM_STEP_COUNT_OFFSET            DATA_EEPROM_START_ADDR + 256*2
#define EEPROM_CONFIG_FILE_OFFSET           DATA_EEPROM_START_ADDR + 256*3


/* Offset in the Config file */
/* Config Factory Info : 0:Flag of App active or not;
						 1:Flag of App Modify SN or not;
						 2~5:App Modify SN Info
						 6:  Moniter Template ID
*/
#define EEPROM_CONFIG_FACTORYINFO           EEPROM_CONFIG_FILE_OFFSET + 20   //data length 20
#define EEPROM_CONFIG_CPUINFO               EEPROM_CONFIG_FILE_OFFSET + 50   //data length 57
#define EEPROM_CONFIG_OSASTARTTIME          EEPROM_CONFIG_FILE_OFFSET + 110  //data length 10
#define EEPROM_CONFIG_STORAGE_CONST_VAR     (EEPROM_CONFIG_FILE_OFFSET + 120)//data length 20
#define EEPROM_CONFIG_DeviceMode            (EEPROM_CONFIG_FILE_OFFSET + 140)//data length 10

#define EEPROM_CONFIG_UPGRADE_FLAG          EEPROM_CONFIG_FILE_OFFSET + 240
#define EEPROM_CONFIG_FW_RX_FLAG            EEPROM_CONFIG_FILE_OFFSET + 241
#define EEPROM_CONFIG_FW_TYPE_FLAG          EEPROM_CONFIG_FILE_OFFSET + 246

#define EEPROM_STORAGE_LOG                  (EEPROM_NON_CONTACT_TEMP_OFFSET + 0)//data length 100
#define EEPROM_STORAGETIMER_ERROR_LOG       (EEPROM_NON_CONTACT_TEMP_OFFSET + 100)//data length 20


#define EEPROM_STORAGE_LOG_LEN 		                    (50)
#define EEPROM_STORAGE_STORAGE_CONST_LEN 		        (12)


/* Monitor Template ID define */
#define 	kICTMonitorTemplateNone                          0        //未选择方案
//#define		kICTMonitorTemplateAtrialFibrillationID          2        //房颤
//#define     kICTMonitorTemplateCHDID                         3        //冠心病
//#define     kICTMonitorTemplateHeartRateID                   4        //循环系统-其它心脏病
//#define     kICTMonitorTemplateAsthmaticBronchitis           5        //支气管哮喘
//#define     kICTMonitorTemplateSleepApneaID                  6        //睡眠综合症
//#define     kIctmonitorTemplateCustomizeID                   7        //自定义
//#define     kIctmonitorTemplateSubHealthID                   8        //亚健康
#define     kICTMonitorTemplateAltitudeStressID              9        //高原反应
#define     kICTMonitorTemplateCOPD                          10       //慢阻肺
#define     kICTMonitorTemplateHypoxemia                     11       //夜间低血氧症
#define     kICTMonitorTemplateSleepApneaManagerID           12       // 睡眠呼吸暂停综合症管理 v4.2.2
#define     kIctmonitorTemplateNightLowSpo2ManagerID         13       // 夜间低血氧症管理  4.2.2
#define     kICTMonitorTemplateOSA                           0x10     // OSAS
#define 	kICTMonitorTemplateFreeRunID					 0xFF	  // Free run, will start as "real time sample", 1s update rate



/* Exported types ------------------------------------------------------------*/
typedef enum {SUB_MOD_RUN = 0, SUB_MOD_ERROR, SUB_MOD_CHECK, SUB_MOD_STOP} SubModuleStat;

typedef enum {CHECK_FAIL = 0, CHECK_PASS = !CHECK_FAIL} CheckStat;

typedef enum {Device_Alone = 0,Device_WorkWithAPP} deviceStateTypedef;

typedef enum {HRSpO2_Measure_Normal = 0, HRSpO2_Measure_Test} hrspo2MeasureModeTypedef;


typedef enum
{
	NormalDis = 0,
	OccupiedDis,          //OccupiedDis:可以被按键清除
	OccupiedDisMove,      //被keep stable 占用，被运动检测清除
	OccupiedDisCall       //被来电占用，被来电挂断清除
} OLEDDisplayStat;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Board Level Define */

	#define 	SPO2_AFE_AFE4403
	#define 	BOARD_REDHARE_V3_0

	#define 	GSEN_BMA250E

	#define 	App2_0


/* 	FFT Algorithm LUT size definition 		*/
#define 	FFT_TABLE_1024_512_POINTS


/*******************SeriNum Address define ************************************/
#define SeriNum_ADDRESS           (uint32_t)0x8003F80
#define NTC_Tem_ADDRESS           (uint32_t)0x8020F00     //NTC Tempurature Array Address
#define NTC_Rt_ref_ADDRESS        (uint32_t)0x80216D0     //NTC Rt_ref Array Address
#define STM32_BOOT_VERSION_ADDR   (uint32_t)0x8003000		/* Address for Bootloader version number 	*/
/**********************Clock Define********************************************/
#define GLOBAL_MCU_FREQ 16 						//In MHz unit

/**********************Data Syn size define **********************************/
#define SYNC_DATA_SIZE_RD      230
#define EEPROMReadMaxData  		230

/* TPIS Measure Enable */
//#define TPISMeasureEN         //Enable measure TPIS

/*****************************************************************************/
/*                   Debug mode switch                                       */
/*****************************************************************************/
//#define Task_Key_Debug         //Printf Task Key Debug Information
//#define NTC_EVENT_DEBUG_MODE
//#define nRF51822_SPIRX           //nRF51822SPI接收调试开关
//#define nRF51822_SPIRXResource   //nRF51822SPI原始数据接收开关
//#define Monitor_Template_Debug     //监测模板调试开关
//#define SECTICK_DEBUG_EN           //Second tick debug switch
//#define OLED_CTL_DEBUG             // OLED control debug switch
//#define Parameter_Limit_Debug      //Set parameter limit debug
//#define LowPower_DEBUG               //Low Power debug
//#define RTC_TEST_ENABLE		    	//RTC Debug En
//#define SPO2_DEBUG_ENABLE				//SPO2 only mode running
//#define SyncData_DEBUG               //Sync data debug
//#define PassKey_DEBUG                //Pass key debug
//#define ALARM_DEBUG                  //Alarm Debug
//#define THREE_AXES_DEBUG_ENABLE		//enable 3Axes acc debug mode
//#define BREATH_LED_DEBUG_ENABLE			//led breath function debug enable
//#define UI_TEST_ENABLE               //UI test enable
//#define Memory_Manage_DEBUG           //Memory Manage DEBUG
//#define M95M01_Debug					//EEPROM LOW LEVEL DEBUG INFORMATION
//#define EEPROM_DEBUG                  //EEPROM Debug Switch
//#define Battery_Debug
//#define FWUpdate_Debug                 //FW Update debug
//#define STEP_DEBUG
//#define INTERRUPT_DEBUG
//#define MotionInfo_Debug
//#define ANCS_DEBUG
//#define EMT_Test_App
//#define AppModifySN_Debug
//#define FreeRunMode_Debug
//#define BLEPRINTGetSleepApneaInfo
//#define FLASH_DEBUG
//#define Alarm_Set_Debug
//#define STORAGE_LOG

#define ACTIVE_PROCESS_ENABLE			// Enable the active process

#define DEVICE_SUPPORT_ABILITY 	(0x06)

//#define STEP_CNT_ALGORITHM_DEBUG
/* platform self check switch */
//#define BasicInfo_Debug            //Basic user information debug
//#define RDALLEEPROM               //Read All EEPROM data for test
//#define SPI_nRF_Debug             //SPI transmit debug
//#define RDCPUINFO                 //Read CPU information to loacte the program status

/*self check define */
#define ID_SelfCheckProgramStatus   0x01
#define ID_RedLEDStatus             0x02
#define ID_GreenLEDStatus           0x03
#define ID_OrangeLEDStatus          0x04
#define ID_VirbreStatus             0x05
#define ID_OLEDStatus               0x06
#define ID_KeyStatus                0x07
#define ID_nRF51822Status           0x08
//#define ID_M95M01Status             0x09
#define ID_MX25L1606EStatus         0x09
#define ID_BMA250EStatus            0x0A
#define ID_AFE44xxStatus            0x0B
#define ID_ADS1118Status            0x0C
#define ID_SN                       0x0D
#define ID_BatterySmpStatus         0x0E

#define CMD_STOP                    0x00
#define CMD_Start                   0x01

#define CMD_Checking                0x00
#define CMD_Fail                    0x01
#define CMD_Pass                    0x02
#define CMD_PressKey                0x03


/* 		Define sub-function status 		*/
#define 	SpO2_MonitorTemplate_State 		(1 << 0)
#define 	HR_MonitorTemplate_State 		(1 << 1)

#define 	SpO2_SingleWork_State 			(1 << 2)
#define 	HR_SingleWork_State 			(1 << 3)

#define 	SpO2_RealTimeSample_State 		(1 << 4)
#define 	HR_RealTimeSample_State 		(1 << 5)


#define 	SPO2_HR_MONITOR_STATE			(SpO2_MonitorTemplate_State | HR_MonitorTemplate_State)
#define 	SPO2_HR_SINGLEWORK_STATE 		(SpO2_SingleWork_State | HR_SingleWork_State)
#define 	SPO2_HR_REALTIME_STATE 			(SpO2_RealTimeSample_State | HR_RealTimeSample_State)

#define 	SPO2_HR_SUBFUNC_ALL_STATE 		(SPO2_HR_MONITOR_STATE | SPO2_HR_SINGLEWORK_STATE | SPO2_HR_REALTIME_STATE)

#define 	SUBFUNC_ALL_STATE 				(SPO2_HR_SUBFUNC_ALL_STATE)

#define 	SPO2_NEEDED_STATE 			    (SPO2_HR_SUBFUNC_ALL_STATE)


#define Device_Mode_RTC       (1 << 0)
#define Device_Mode_SPO2_HR   (1 << 1)
#define Device_Mode_SecTick   (1 << 4)
#define Device_Mode_Step      (1 << 5)
#define Device_Mode_PassKey   (1 << 6)
#define Device_Mode_CheckUp   (1 << 7)
#define Device_Mode_Charge    (1 << 8)
#define Device_Mode_Distance  (1 << 9)
#define Device_Mode_Calorie   (1 << 10)
#define Device_Mode_LowBattery   (1 << 11)
#define Device_Mode_Factory      (1 << 12)
#define Device_Mode_FWUpdate      (1 << 13)
#define Device_Mode_FactoryTest   (1 << 14)
#define Device_Mode_IncomingCall  (1 << 15)



/****************************************************************************/
/*             Ble Usart Debug Switch                                       */
/****************************************************************************/


/**********************IQR Priority Define***********************************/
#define OLED_TIMPreemptionPriority           0x0F         //OLED频率控制定时器,用于显示动画
#define KEY_TIMPreemptionPriority            0x05         //KEY长短按键定时器
#define IIC_TIMPreemptionPriority            0x00         //IIC TimeOut Timer
#define AFE44xxDIAG_TIMPreemptionPriority    0x0F         //AFE44xx DIAG TimeOut Timer
#define SYNCPollPreemptionPriority           0x0F         //Sync Flash Read Busy Poll Timer

#define RTC_PeriodWakeUpPreemptionPriority   0x0F         //RTC
#define RTC_PeriodWakeUpSubPriority          0x0F         //RTC

#define TIM_DefaultPreemptionPriority        0x0F         //TIM默认优先级
#define TIM_DefaultSubPriority               0x0F         //TIM默认优先级

#define AFE44xxDRDYPreemptionPriority        0x0E         //AFE44xx DRDY
#define AFE44xxDRDYSubPriority               0x0E         //AFE44xx DRDY

#define AFE44xxDIAGPreemptionPriority        0x02         //AFE44xx DIAG
#define AFE44xxDIAGSubPriority               0x02         //AFE44xx DIAG

#define KEY_PressPreemptionPriority          0x01         //Key Press Int
#define KEY_PressSubPriority                 0x01         //Key Press Int

#define ADS1115_RDYINTPreemptionPriority     0x0F         //ADS1115 Ready INT
#define ADS1115_RDYINTSubPriority            0x0F         //ADS1115 Ready INT

#define BT_REQNPreemptionPriority            0x02         //BT REQN中断，redHare中与AFE4400DIAG中断公用
#define BT_REQNSubPriority                   0x02         //BT REQN中断，redHare中与AFE4400DIAG中断公用

#define Motor_TIMPreemptionPriority          0x0E

#define LIS3DHINT1PreemptionPriority 				0x0F					//LIS3DH Data
#define LIS3DHINT1SubPriority				 				0x0F					//LIS3DH Data
#define LIS3DHINT2PreemptionPriority				0x06					//LIS3DH Wakeup
#define LIS3DHINT2SubPriority								0x06					//LIS3DH Wakeup

#define MonitorTemplatePreemptionPriority    0x03
#define MonitorTemplateSubPriority           0x0F

#define OLEDShutDown_TIMPreemptionPriority   0x0F

#define BATLevelSample_TIMPreemptionPriority   0x0F

#define STEP_TIMPreemptionPriority            0x0F

#define BatterySamplePreemptionPriority      0x0F         //ADC电量采集中断
#define BatterySampleSubPriority             0x0F         //

/* IRQ Channel Define*/


#ifdef BOARD_REDHARE_V3_0
	#define AFE44xxDRDY_IRQ_CH 							EXTI15_10_IRQn;
#endif


/*****************Motor virbrate time define **********************************/
#define Motor_Vibrate_Time    200     //定义Motor振动200ms

/*****************SYNC Read Flash Busy poll time define ***********************/
#define SYNCRDFlashPollTime   100

/**************OLED display flash time define ********************************/
#define StartFlashTimPeriod   400            //开机动画刷新显示频率(eg: 200 ms刷新一副界面)
#define BoundRemindFlashTimPeriod   400      //绑定动画刷新显示频率(eg: 200 ms刷新一副界面)
#define MeasuringFlashTIMPeriod     200      //测量中界面动画刷新显示频率
#define BATChargingFlashTIMPeriod     1000  //充电显示界面动画刷新显示频率
#define NotEnoughSpaceFlashTIMPeriod  1000  //空间不足显示界面刷新显示频率
#define IncomingCallFlashTIMPeriod    1000  //来电提示显示界面刷新显示频率

// /*                   Multi-used GPIO define                   */
// #ifdef BOARD_REDHARE_V3_0
//
// 	#define PT_GPIO_UNUSED_RCC_A				RCC_AHBPeriph_GPIOA
// 	#define PT_GPIO_UNUSED_RCC_B				RCC_AHBPeriph_GPIOB
// 	#define PT_GPIO_UNUSED_RCC_C				RCC_AHBPeriph_GPIOC
// 	#define PT_GPIO_UNUSED_RCC_D				RCC_AHBPeriph_GPIOD
// 	#define PT_GPIO_UNUSED_RCC_E				RCC_AHBPeriph_GPIOE
// 	#define PT_GPIO_UNUSED_RCC_H				RCC_AHBPeriph_GPIOH
//
// 	#define PT_GPIO_UNUSED_PORT_A 				GPIOA
// 	#define PT_GPIO_UNUSED_PORT_B 				GPIOB
// 	#define PT_GPIO_UNUSED_PORT_C 				GPIOC
// 	#define PT_GPIO_UNUSED_PORT_D 				GPIOD
// 	#define PT_GPIO_UNUSED_PORT_E 				GPIOE
// 	#define PT_GPIO_UNUSED_PORT_H 				GPIOH

// 	#define PT_GPIO_UNUSED_PIN_PORTA			(GPIO_Pin_1 | GPIO_Pin_10 | GPIO_Pin_12)
// 	#define PT_GPIO_UNUSED_PIN_PORTB			(GPIO_Pin_2)
// 	#define PT_GPIO_UNUSED_PIN_PORTC			(GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4)
// 	#define PT_GPIO_UNUSED_PIN_PORTD			(GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3
// 												|GPIO_Pin_4 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_9
// 												|GPIO_Pin_11| GPIO_Pin_12| GPIO_Pin_13| GPIO_Pin_14
// 												|GPIO_Pin_15)
// 	#define PT_GPIO_UNUSED_PIN_PORTE 			(GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_4
// 												|GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_10
// 												|GPIO_Pin_11| GPIO_Pin_12| GPIO_Pin_15)
// 	#define PT_GPIO_UNUSED_PIN_PORTH 			(GPIO_Pin_2)
//
// #endif
/****************OLED Define  *************************************************/


/*****************LED Define***************************************************/


#ifdef BOARD_REDHARE_V3_0

	/*****************Peri Power Define********************************************/
	#define RCC_PowerPeriph       RCC_AHBPeriph_GPIOC
	#define GPIO_Pin_Power        GPIO_Pin_0
	#define GPIO_Power            GPIOC
#endif

/*   MX25L1606E Power define  */
#define RCC_FlashPowerPeriph       RCC_AHBPeriph_GPIOE
#define GPIO_Pin_PowerFlash        GPIO_Pin_15
#define GPIO_PowerFlash            GPIOE

/*    AFE4403 3.7v POWER SUPPLY Define    */
#ifdef BOARD_REDHARE_V3_0
	#define RCC_AFE44xx_PowerPeriph       RCC_AHBPeriph_GPIOA
	#define GPIO_Pin_AFE44xx_Power        GPIO_Pin_9
	#define GPIO_AFE44xx_Power_Port				GPIOA
	#define PT_PMG_AFE44xx_POWER_GPIO					Periph_GPIOA
	#define PT_PMG_Periph_POWER_GPIO 					Periph_GPIOA
#endif




/*****************Key Define**************************************************/
#define LongPressTIM          1000    // ms,按键大于该时间为长按键事件
#define PressTIMOut           1000   // ms,按键超时时间，按键超过该时间判定为长按键
#define KeyInvalidTIM         200    // ms,按键不可用时间

#define SpO2_HR_OLEDShutDownTIM  10000   //ms,SpO2_HR模式下，稳定读数后进入屏幕关闭状态时间
#define Charge_OLEDShutDownTIM      30000   //ms,Charge模式下，进入屏幕关闭状态时间
#define Normal_OLEDShutDownTIM   5000   //ms,普通模式（显示工作模式）模式下，稳定读数后进入屏幕关闭状态时间
#define BoundTimeOutTIM          60000   //ms,绑定超时定时
#define BATLevelSampleTIM        60000   //ms,电池电量采集定时,充电模式下
#define BATLevelSampleTIMWork    8000   //ms,电池电量采集定时,正常工作模式下
#define FwReceiveTimeOutTIM      8000   //ms,Firmware update receive timeout
#define FWFlashWriteTimeOutTIM      4000   //ms,Firmware update Write Flash timeout
#define EraseSectorTimeOutTIM      50   //ms,Erase External Flash Sector timeout
#define RedLEDTimeOutTIM         500   //ms


#ifdef BOARD_REDHARE_V3_0

	#define RCC_KEYPeriph_KEYWP   RCC_AHBPeriph_GPIOA
	#define GPIO_Pin_KEYWP        GPIO_Pin_0
	#define GPIO_KEYWP            GPIOA
	#define PT_Periph_Key         Periph_GPIOA
	#define EXTI_PortSourceKEYWP  EXTI_PortSourceGPIOA
	#define EXTI_PinSourceKEYWP   EXTI_PinSource0
	#define EXTI_LineKEYWP        EXTI_Line0
#endif



/*SpO2 board/chipset level port Define*/

/* AFE44x0 GPIO Port Definition*/

#ifdef BOARD_REDHARE_V3_0
	#define PT_AFE_RESETZ_PORT      GPIOC
	#define PT_AFE_PDNZ_PORT        GPIOD
	#define PT_AFE_ADC_DRDY_PORT    GPIOA

	#define PT_AFE_RESETZ        GPIO_Pin_6
	#define PT_AFE_PDNZ          GPIO_Pin_5
	#define PT_AFE_ADC_DRDY      GPIO_Pin_11

	#define PT_AFE_ADC_DRDY_EXTI_PORT       EXTI_PortSourceGPIOA
	#define PT_AFE_ADC_DRDY_EXTI_PIN        EXTI_PinSource11
	#define PT_AFE_ADC_DRDY_EXTI_LINE       EXTI_Line11

	/****************************************************************/
	/* AFE44x0 SPI Port Definition*/
	/****************************************************************/
	#define PT_AFE_SPI_PORT		GPIOA
	#define PT_AFE_SPI_STE_PINSOURCE	GPIO_PinSource4
	#define PT_AFE_SPI_SCLK_PINSOURCE	GPIO_PinSource5
	#define PT_AFE_SPI_SOMI_PINSOURCE	GPIO_PinSource6
	#define PT_AFE_SPI_SIMO_PINSOURCE	GPIO_PinSource7

	#define PT_AFE_SPI_STE_PIN				GPIO_Pin_4
	#define PT_AFE_SPI_SCLK_PIN				GPIO_Pin_5
	#define PT_AFE_SPI_SOMI_PIN				GPIO_Pin_6
	#define PT_AFE_SPI_SIMO_PIN				GPIO_Pin_7

	#define PT_AFE44x0_SPI SPI1

	#define PT_RCC_APBxPeriph_SPI_AFE44x0		RCC_APB2Periph_SPI1

	#define PT_AFE_SPI_STE_GPIO_CLK					RCC_AHBPeriph_GPIOA
	#define PT_AFE_SPI_SCLK_GPIO_CLK				RCC_AHBPeriph_GPIOA
	#define PT_AFE_SPI_SOMI_GPIO_CLK				RCC_AHBPeriph_GPIOA
	#define PT_AFE_SPI_SIMO_GPIO_CLK				RCC_AHBPeriph_GPIOA

	#define PT_AFE_RESETZ_GPIO_CLK        RCC_AHBPeriph_GPIOC
	#define PT_AFE_PDNZ_GPIO_CLK          RCC_AHBPeriph_GPIOD
	#define PT_AFE_ADC_DRDY_GPIO_CLK      RCC_AHBPeriph_GPIOA

	//define for power management
	#define PT_PMG_AFE44XX_GPIO 			(Periph_GPIOA | Periph_GPIOC | Periph_GPIOD)
	#define PT_PMG_AFE44XX_SPI				(Periph_SPI1)
#endif

/* 	SPI definition for V4.0 with BMA250e 	*/
#ifdef GSEN_BMA250E
	#define PT_BMA250E_SPI_PORT		        GPIOA
	#define PT_BMA250E_SPI_STE_PORT 	    GPIOB
	#define PT_BMA250E_SPI_STE_PINSOURCE	GPIO_PinSource8
	#define PT_BMA250E_SPI_SCLK_PINSOURCE	GPIO_PinSource5
	#define PT_BMA250E_SPI_SOMI_PINSOURCE	GPIO_PinSource6
	#define PT_BMA250E_SPI_SIMO_PINSOURCE	GPIO_PinSource7

	#define PT_BMA250E_SPI_STE_PIN		    GPIO_Pin_8
	#define PT_BMA250E_SPI_SCLK_PIN	        GPIO_Pin_5
	#define PT_BMA250E_SPI_SOMI_PIN	        GPIO_Pin_6
	#define PT_BMA250E_SPI_SIMO_PIN	        GPIO_Pin_7

	#define PT_BMA250E_SPI                  SPI1
    #define PT_BMA250E_SPI_AF               GPIO_AF_SPI1
	#define PT_RCC_APBxPeriph_SPI_BMA250E	RCC_APB2Periph_SPI1

	#define PT_BMA250E_SPI_STE_GPIO_CLK				    RCC_AHBPeriph_GPIOB
	#define PT_BMA250E_SPI_SCLK_GPIO_CLK				RCC_AHBPeriph_GPIOA
	#define PT_BMA250E_SPI_SOMI_GPIO_CLK				RCC_AHBPeriph_GPIOA
	#define PT_BMA250E_SPI_SIMO_GPIO_CLK				RCC_AHBPeriph_GPIOA

	#define PT_BMA250E_INT1_GPIO_PORT    		GPIOA
	#define PT_BMA250E_INT1_GPIO_PINSOURCE		GPIO_PinSource3
	#define PT_BMA250E_INT1_GPIO_PIN    		GPIO_Pin_3
	#define PT_BMA250E_INT1_GPIO_CLK			RCC_AHBPeriph_GPIOA

	#define PT_BMA250E_INT2_GPIO_PORT    		GPIOC
	#define PT_BMA250E_INT2_GPIO_PINSOURCE		GPIO_PinSource13
	#define PT_BMA250E_INT2_GPIO_PIN    		GPIO_Pin_13
	#define PT_BMA250E_INT2_GPIO_CLK			RCC_AHBPeriph_GPIOC


	#define PT_BMA250E_INT1_EXTI_PORT       EXTI_PortSourceGPIOA
	#define PT_BMA250E_INT1_EXTI_PIN        EXTI_PinSource3
	#define PT_BMA250E_INT1_EXTI_LINE       EXTI_Line3

	#define PT_BMA250E_INT2_EXTI_PORT       EXTI_PortSourceGPIOC
	#define PT_BMA250E_INT2_EXTI_PIN        EXTI_PinSource13
	#define PT_BMA250E_INT2_EXTI_LINE       EXTI_Line13

#endif




/*********************ADS1115 Port define ************************************/
         /* ALT/RDY pin define */
#ifdef BOARD_REDHARE_V3_0

	#define PT_ADS1115_ALERTRDY_GPIO_CLK       RCC_AHBPeriph_GPIOB
	#define PT_ADS1115_ALERTRDY_GPIO_PORT      GPIOB
	#define PT_ADS1115_ALERTRDY_PIN            GPIO_Pin_7
	#define PT_Periph_ADS1115_RDY              Periph_GPIOB

	#define PT_ADS1115_RCC_APBPeriph_SYSCFG    RCC_APB2Periph_SYSCFG
	#define PT_ADS1115_EXTI_PortSourceGPIO     EXTI_PortSourceGPIOB
	#define PT_ADS1115_EXTI_PinSource          EXTI_PinSource7
	#define PT_ADS1115_EXTI_Line               EXTI_Line7

#endif

/********************ADS1115 Port define ***********************************/
/* ADS1118 SPI define */
#define PT_RCC_APBPeriph_ADS118_SPI                RCC_APB1Periph_SPI2

#define PT_ADS1118_SPI_SCK_GPIO_CLK                RCC_AHBPeriph_GPIOB
#define PT_ADS1118_SPI_MOSI_GPIO_CLK               RCC_AHBPeriph_GPIOB
#define PT_ADS1118_SPI_MISO_GPIO_CLK               RCC_AHBPeriph_GPIOB
#define PT_ADS1118_CS_GPIO_CLK                     RCC_AHBPeriph_GPIOC

#define PT_ADS1118_SPI_MISO_GPIO_PORT              GPIOB
#define PT_ADS1118_SPI_MOSI_GPIO_PORT              GPIOB
#define PT_ADS1118_SPI_SCK_GPIO_PORT               GPIOB
#define PT_ADS1118_CS_GPIO_PORT                    GPIOB

#define PT_ADS1118_SPI_MISO_SOURCE                 GPIO_PinSource14
#define PT_ADS1118_SPI_MOSI_SOURCE                 GPIO_PinSource15
#define PT_ADS1118_SPI_SCK_SOURCE                  GPIO_PinSource13

#define PT_ADS1118_SPI_MISO_AF                     GPIO_AF_SPI2
#define PT_ADS1118_SPI_MOSI_AF                     GPIO_AF_SPI2
#define PT_ADS1118_SPI_SCK_AF                      GPIO_AF_SPI2

#define PT_ADS1118_SPI_MISO_PIN                    GPIO_Pin_14
#define PT_ADS1118_SPI_MOSI_PIN                    GPIO_Pin_15
#define PT_ADS1118_SPI_SCK_PIN                     GPIO_Pin_13
#define PT_ADS1118_CS_PIN                          GPIO_Pin_7

#define PT_ADS1118_SPI                             SPI2

/*********************LIS3DH Port define ************************************/

#ifdef BOARD_REDHARE_V3_0
	/* INT0/INT1 pin define */
	#define PT_LIS3DH_INT1_GPIO_CLK       RCC_AHBPeriph_GPIOA
	#define PT_LIS3DH_INT1_GPIO_PORT      GPIOA
	#define PT_LIS3DH_INT1_PIN            GPIO_Pin_3
	#define PT_Periph_LIS3DH_INT1         Periph_GPIOA

	#define PT_LIS3DH_INT1_RCC_APBPeriph_SYSCFG    RCC_APB2Periph_SYSCFG
	#define PT_LIS3DH_INT1_EXTI_PortSourceGPIO     EXTI_PortSourceGPIOA
	#define PT_LIS3DH_INT1_EXTI_PinSource          EXTI_PinSource3
	#define PT_LIS3DH_INT1_EXTI_Line               EXTI_Line3

	#define PT_LIS3DH_INT2_GPIO_CLK       RCC_AHBPeriph_GPIOC
	#define PT_LIS3DH_INT2_GPIO_PORT      GPIOC
	#define PT_LIS3DH_INT2_PIN            GPIO_Pin_13
	#define PT_Periph_LIS3DH_INT2         Periph_GPIOC

	#define PT_LIS3DH_INT2_RCC_APBPeriph_SYSCFG    RCC_APB2Periph_SYSCFG
	#define PT_LIS3DH_INT2_EXTI_PortSourceGPIO     EXTI_PortSourceGPIOC
	#define PT_LIS3DH_INT2_EXTI_PinSource          EXTI_PinSource13
	#define PT_LIS3DH_INT2_EXTI_Line               EXTI_Line13

	#define PT_LIS3DH_SCL_GPIO_PORT      GPIOB
	#define PT_LIS3DH_SCL_PIN            GPIO_Pin_8

	#define PT_LIS3DH_SDA_GPIO_PORT      GPIOB
	#define PT_LIS3DH_SDA_PIN            GPIO_Pin_9

	#define LIS3DH_INT1_CH								EXTI3_IRQn
	#define LIS3DH_INT2_CH								EXTI15_10_IRQn

	#define PT_PMG_3AXES_IIC 							Periph_IIC1
	#define PT_PMG_3AXES_GPIO							(Periph_GPIOA | Periph_GPIOC)
#endif


/*********************2.5V Vref Out Define************************************/
#ifdef BOARD_REDHARE_V3_0
	#define PT_VrfOut_RCC_Periph               RCC_AHBPeriph_GPIOB
	#define PT_GPIO_Pin_VrfOut				   GPIO_Pin_6
	#define PT_GPIO_VrfOut                     GPIOB
	#define PT_Periph_VrfOut                   Periph_GPIOB
#endif


/********************Sys SPI define*******************************************/

#ifdef BOARD_REDHARE_V3_0
	#define PT_RCC_APBPeriph_SysSPI           RCC_APB1Periph_SPI3
	#define PT_RCC_AHBPeriph_SysSPIGPIO       RCC_AHBPeriph_GPIOC
	#define PT_RCC_AHBPeriph_SysSPINSSGPIO    RCC_AHBPeriph_GPIOB

	#define PT_GPIO_SysSPI                    GPIOC
	#define PT_Periph_SysSPIPort              Periph_GPIOC

	#define PT_GPIO_SysSPINSS                 GPIOB

	#define PT_GPIO_PinSourceSysSPI_MOSI      GPIO_PinSource12
	#define PT_GPIO_PinSourceSysSPI_MISO      GPIO_PinSource11
	#define PT_GPIO_PinSourceSysSPI_SCK       GPIO_PinSource10
	#define PT_GPIO_AF_SysSPI                 GPIO_AF_SPI3

	#define PT_GPIO_Pin_SysSCK                GPIO_Pin_10
	#define PT_GPIO_Pin_SysMOSI               GPIO_Pin_12
	#define PT_GPIO_Pin_SysMISO               GPIO_Pin_11

	#define PT_GPIO_Pin_SysNSS                GPIO_Pin_3

	#define PT_SysSPI                         SPI3
	#define PT_Periph_SysSPI                  Periph_SPI3

	#define PT_SysSPIBusyPort                 GPIOD
	#define PT_SysSPIBusyPin                  GPIO_Pin_9
	#define PT_RCC_AHBPeriph_SysSPIBusy       RCC_AHBPeriph_GPIOD

	#define SysSPIBusy_Set                    GPIO_SetBits(PT_SysSPIBusyPort,PT_SysSPIBusyPin)
	#define SysSPIBusy_Reset                  GPIO_ResetBits(PT_SysSPIBusyPort,PT_SysSPIBusyPin)
#endif


/*******************Bluetooth connected gpio define **************************/
#ifdef BOARD_REDHARE_V3_0
		#define PT_RCC_AHBPeriph_GPIO_BTREQN      RCC_AHBPeriph_GPIOA
		#define PT_GPIO_Pin_BTREQN                GPIO_Pin_15
		#define PT_GPIOBTREQN                     GPIOA

		#define PT_RCC_AHBPeriph_GPIO_BTSYNC      RCC_AHBPeriph_GPIOC
		#define PT_GPIO_Pin_BTSYNC                GPIO_Pin_5
		#define PT_GPIOBTSYNC                     GPIOC

		#define PT_RCC_AHBPeriph_GPIO_BTRESET     RCC_AHBPeriph_GPIOB
		#define PT_GPIO_Pin_BTRESET               GPIO_Pin_4
		#define PT_GPIOBTRESET                    GPIOB

		#define PT_RCC_APBPeriph_SYSCFG_BTREQN    RCC_APB2Periph_SYSCFG      //外部中断用
		#define PT_RCC_APBPeriph_SYSCFG_BTSYNC    RCC_APB2Periph_SYSCFG      //外部中断用

		#define PT_EXTI_PortSourceGPIO_BTREQN     EXTI_PortSourceGPIOA
		#define PT_EXTI_PinSourceBTREQN           EXTI_PinSource15
		#define PT_EXTI_LineBTREQN                EXTI_Line15

		#define PT_EXTI_PortSourceGPIO_BTSYNC     EXTI_PortSourceGPIOC
		#define PT_EXTI_PinSourceBTSYNC           EXTI_PinSource5
		#define PT_EXTI_LineBTSYNC                EXTI_Line5

		#define PT_PMG_Periph_BTREQN_GPIO 		Periph_GPIOA
		#define PT_PMG_Periph_BTSYNC_GPIO 		Periph_GPIOC
#endif


/******************EEPROM SPI define ****************************************/

#ifdef BOARD_REDHARE_V3_0
	#define PT_RCC_AHBPeriph_EEPROM_CSGPIO    RCC_AHBPeriph_GPIOB
	#define PT_GPIO_PORT_EEPROM_CS            GPIOB
	#define PT_GPIO_Pin_EEPROM_CS             GPIO_Pin_5
#endif

#define PT_RCC_AHBPeriph_FLASH_CS    		RCC_AHBPeriph_GPIOB
#define PT_GPIO_PORT_FLASH_CS            	GPIOB
#define PT_GPIO_Pin_FLASH_CS             	GPIO_Pin_5


/*********************Motor define ******************************************/

	#ifdef BOARD_REDHARE_V3_0
		#define PT_RCC_MotorPeriph                RCC_AHBPeriph_GPIOC
		#define PT_GPIO_Pin_Motor                 GPIO_Pin_9
		#define PT_GPIO_Motor                     GPIOC
		#define PT_Periph_Motor                   Periph_GPIO9
	#endif

/*********************LED define ******************************************/
#define RCC_AHBPeriph_GPIO_LED_R                RCC_AHBPeriph_GPIOC
#define GPIO_PORT_LED_R                         GPIOC
#define GPIO_Pin_LED_R                          GPIO_Pin_9

#define RCC_AHBPeriph_GPIO_LED_G                RCC_AHBPeriph_GPIOB
#define GPIO_PORT_LED_G                         GPIOB
#define GPIO_Pin_LED_G                          GPIO_Pin_6
/*********************BAT define**********************************************/

#ifdef BOARD_REDHARE_V3_0
	#define RCC_BATPGPeriph                 RCC_AHBPeriph_GPIOA
	#define RCC_BATCEPeriph                 RCC_AHBPeriph_GPIOE
	#define RCC_BATCHGPeriph                RCC_AHBPeriph_GPIOE
	#define EXTI_LineBAT_PG                 EXTI_Line2
	#define EXTI_LineBAT_CE                 EXTI_Line13
	#define EXTI_LineBAT_CHG                EXTI_Line9
	#define EXTI_PinSourceBAT_PG            EXTI_PinSource2
	#define EXTI_PinSourceBAT_CE            EXTI_PinSource13
	#define EXTI_PinSourceBAT_CHG           EXTI_PinSource9
	#define EXTI_PortSourceBAT_PG           EXTI_PortSourceGPIOA
	#define EXTI_PortSourceBAT_CE           EXTI_PortSourceGPIOE
	#define EXTI_PortSourceBAT_CHG          EXTI_PortSourceGPIOE
	#define GPIO_Pin_BAT_PG                 GPIO_Pin_2
	#define GPIO_Pin_BAT_CE                 GPIO_Pin_13
	#define GPIO_Pin_BAT_CHG                GPIO_Pin_9
	#define GPIO_BAT_PG                     GPIOA
	#define PT_Periph_BAT_PG                Periph_GPIOA
	#define GPIO_BAT_CE                     GPIOE
	#define GPIO_BAT_CHG                    GPIOE
	#define PT_Periph_BAT_CHG               Periph_GPIOE
#endif                  // End of BOARD_REDHARE_V3_0

/*********************Battery Detect pin define ************************************/
#define ADC_Channel_BATMONITOR          ADC_Channel_12
#define RCC_ADC_PORT_BATMONITOR         RCC_AHBPeriph_GPIOC
#define ADC_PORT_BATMONITOR             GPIOC
#define ADC_PORT_PIN_BATMONITOR         GPIO_Pin_2

#define Key_State_Short       0
#define Key_State_Long        1
#define Key_State_None        2

#define Key_Pressed           0
#define Key_UnPressed         1

#define Stop                  0
#define Start                 1
#define Suspend               2
#define Resume                3
#define Run                   4
#define LowPowerRun           5

#define ON                    1
#define OFF                   0

#define Valid                 1
#define InValid               0


/*********************App Command define*****************************************/
#define APP_CMD_ModifySN                             51
#define APP_CMD_RecoverTheData                       53
#define APP_CMD_SetSpO2NormalMeasure                 54
#define APP_CMD_SetSpO2TestMeasure                   55
#define APP_CMD_PRINTF                               (56)


/*********************Stm32 communicate with nRF51822 protocol define*************/
#define Frame_ConnectState                 0x01   //指示APP连接状态帧
#define Frame_DeviceInform                 0x02   //设备信息帧
#define Frame_CheckUpParam                 0x03   //数据采集帧，采集哪些参数
#define Frame_CheckUpDate                  0x04   //数据实时更新
#define Frame_SyncDateSwitch               0x05   //触发数据同步帧
#define Frame_SyncDate                     0x0D   //同步数据帧
#define Frame_FWUpdateResponse             0x07   //固件升级结果返回帧
#define Frame_MoniterTemplateSet           0x08   //监测模板设置帧
#define Frame_MoniterTemplateResult        0x09   //监测模板设置结果帧
#define Frame_Alarm                        0x0A   //报警指示帧
#define Frame_Limit                        0x0B   //参数门限设置帧
#define Frame_TimeSyn                      0x0C   //时间同步帧
#define Frame_PassKey                      0x0E   //密码验证帧
#define Frame_BATLevel                     0x0F   //电池电量等级
#define Frame_DeviceLog                    0x10   //日志打印帧
#define Frame_FWUpdate                     0x11   //固件升级帧
#define Frame_FWUpdateCtlA                 0x12   //固件升级数据传输控制帧,STM32To51822
#define Frame_FWUpdateCtlB                 0x13   //固件升级数据传输控制帧,51822ToSTM32
#define Frame_BasicInfo                    0x14   //用户基本信息数据帧
#define Frame_UpdateSportInfo              0x15   //通知STM32上传运动相关信息
#define Frame_DeviceInfo_Req 			   0x16 	/* 	Require Device Information 		*/
#define Frame_UpdateBleRate                0x17
#define Frame_FactoryHWTest                0x18   //通知STM32硬件测试
#define Frame_BackFactoryMode              0x19   //通知STM32返回工厂模式
#define Frame_ANCS_INCOMINGCALL            0x20   //ANCS


#define Start_DateUpdate                   0x01   //??????
#define Stop_DateUpdate                    0x00   //??????
#define Complate_DataUpdate                0x02   //??????
#define LongPress                          0x04
#define ShortPress                         0x03

#define LongPress                          0x04   //长按
#define ShortPress                         0x03   //短按

/*****************显示图标ID define *********************************************/
#define ICON_ModeHR                         1
#define ICON_MeasureHR                      2
#define ICON_ModeSpO2                       3
#define ICON_MeasureSpO2                    4
#define ICON_ModeRTC                        9
#define ICON_ModeStep                       13
#define ICON_ModeKeepStable					10
#define ICON_SpO2Err						11
#define ICON_AppCtl                         12
#define ICON_ModeFullCharge                 14
#define ICON_ModeCharging                   15
#define ICON_ModeLowBattery                 16
#define ICON_ModeDistance                   17
#define ICON_ModeCal                        18
#define ICON_WearBad                        19
#define ICON_NotEnoughSpace                 20

/*************参数ID定义**********************************************************/
#define CheckUpStop                        			 0x0000
#define SAMPLE_ID_Temperature                        0x0001
#define SAMPLE_ID_HeartRate                          0x0002
#define SAMPLE_ID_SpO2                               0x0004

#define TemperatureID                      0x01   //测量参数ID（不同于设备支持能力）
#define HeartRateID                        0x02
#define SpO2ID                             0x03
#define moveLevelID 					   0x0A
#define MotionInfoID                       0x33
#define StopExcetionFram                   0xFF   //异常监测模板结束帧

#define OSASID                             61
#define HypoxemiaID                        62
#define COPDID                             63
#define SleepApneaManagerID                (64)
#define NightLowSpo2ManagerID              (65)
#define FreeRunNightstartFram              0xFE

#define NormalMTVal                        0      //正常情况采集的数据
#define ExcetionMTVal                      1      //异常情况采集的数据

/*****************报警信息定义****************************************************/
#define Alarm_ID_LowBattery                   0x01
#define Alarm_ID_SpO2NoFinger                 0x02
#define Alarm_ID_Reuse_Active_History		  0x00
#define Alarm_ID_Reuse_EEPROM_CORRECT		  0x03
#define Alarm_ID_Reuse_EEPROM_ERROR			  0x04
#define Alarm_ID_Reuse_BMA250_CORRECT		  0x05
#define Alarm_ID_Reuse_BMA250_ERROR			  0x06
#define Alarm_ID_Reuse_MTSetSuccess           0x07
#define Alarm_ID_Reuse_DisplayPINCodeSuccess  0x08
#define Alarm_ID_Reuse_FreeRunMTRunning       0x09

/*********************SPI3 ADS8343 LCD switch pin PD0 define**********************/
#define RCC_SPI3SWPeriph      RCC_AHBPeriph_GPIOD
#define GPIO_Pin_SPI3SW       GPIO_Pin_0
#define GPIO_SPI3SW           GPIOD

#define ADS8343_Slect         GPIO_SetBits(GPIO_SPI3SW,GPIO_Pin_SPI3SW)
#define LCD_Slect             GPIO_ResetBits(GPIO_SPI3SW,GPIO_Pin_SPI3SW)


/*******************Timer Source Allocation*********************************/
#define SpO2_TIM_SOURCE		TIM6
#define RCC_SpO2_TIM 		RCC_APB1Periph_TIM6
#define SpO2_TIM_IRQn 		TIM6_IRQn

#define LED_TIM_SOURCE		TIM7
#define RCC_LED_TIM			RCC_APB1Periph_TIM7
#define LED_TIM_IRQn 		TIM7_IRQn

#define SpO2Evt_TIM_SOURCE  TIM5
#define RCC_SpO2Evt_TIM 	RCC_APB1Periph_TIM5
#define SpO2Evt_TIM_IRQn 	TIM5_IRQn
/* Exported functions ------------------------------------------------------- */
#endif



/* __PLATFORM_H */


