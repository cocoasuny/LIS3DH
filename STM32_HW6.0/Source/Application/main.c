/**
  ******************************************************************************
  * @file    Project/STM32L1xx_StdPeriph_Templates/main.c
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    13-April-2012
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "common.h"
#include "stdlib.h"
#include <time.h>
#include "cc_alg_app_interface.h"
#include "cc_app_afe_drv_interface.h"
#include "cc_app_gsensor_drv_interface.h"
#include "gSensor_data_process.h"
#include "flash_if.h"

/* Private variables ---------------------------------------------------------*/
uint16_t Device_Mode = Device_Mode_RTC;    //Device Work Mode: Device_Mode_SPO2_HR \ Device_Mode_RTC
uint16_t Device_Mode_pre = Device_Mode_RTC;

static uint8_t  gPeriphPowerStatus = OFF;   //status of power to peripheral

HR_SpO2_Parameters_Typedef      gHRSpO2Val;     //spo2/hr value
uint8_t     gFWData[FIRMWARE_UPDATE_BUF_LEN]={0};                    //FW update data, 128 bytes payload + 2 CRC bytes
//Alarm_Type_Typedef              gAlarmType;     //Alarm type

deviceStateTypedef     Device_State=Device_Alone;          //Device State:Work Alone or Work With APP
hrspo2MeasureModeTypedef    gFlagHR_SpO2Measure = HRSpO2_Measure_Normal;  //HR SpO2 Measure Mode:HRSpO2_Measure_Normal;
                                                                          //HRSpO2_Measure_Test: for factory Test

/* 	SubFunctionRun status define 	*/

static uint16_t 	gSubFuncState = OFF;

//uint8_t     SpO2_RealTimeSample_GetResultNum=0; //SpO2 sample numbers for a stable output, from APP; '0' means continuously sample
//uint8_t     HR_RealTimeSample_GetResultNum=0;   //HR sample numbers for a stable output, from APP; '0' means continuously sample
//bool 		flagIsHRSpO2FixNumberSampleKeyPress = TRUE;

  

//uint16_t    MT_EEPROM_WriteAddr=0x0000;           //Address of EEPROM Write
//uint16_t    MT_EEPROM_ReadAddr =0x0000;           //Address of EEPROM Read

uint8_t 	SPO2_START_BY_KEY = OFF;

PassKey_Typedef		g_PassKey;                    //Pass Key info
DeviceInfomation_t  g_DeviceInfomation;

const uint8_t DevREV[3]="6.1";  //HW board version
const uint8_t FWDEV[3]="611";   //STM32L FW version
uint8_t  u8AppModifySeriNum[4]={0};  //App Modify SN

/* Task definitions */
tsTaskID_t   gLedTaskID;                    //LED task
tsTaskID_t   gKeyTaskID;                    //Key task
tsTaskID_t   gOledDisTaskID;                //OLED Display task
tsTaskID_t   gTsSpO2TaskID_c;               //SpO2 task
tsTaskID_t   gTsMonitorTemplatID_c;         // Monitor Template task
tsTaskID_t   gTsSPITranslateTaskID_c;       //SPI translate tast,communicate with nRF51822
tsTaskID_t   gTsSecTickTaskID_c;            //Second Tick Task ID
tsTaskID_t   gTsPowerManageTaskID_c;        //Power Manage Task ID
tsTaskID_t 	 gTs3AxesTaskID_c; 				// 3 axes task ID
tsTaskID_t   gTsSyncDataTaskID_c;           //Sync data Task ID
tsTaskID_t   gTsAlarmTaskID_c;              //Alarm task ID
tsTaskID_t   gTsBatManageTaskID_c;          // Change Task ID,huayun,2014-07-21
tsTaskID_t   gTsBatMonitorTaskID_c;         // Voltage Monitor Task ID,huayun,2014-07-21
tsTaskID_t   gTsStepTaskID_c;               // Step Task ID,huayun,2014-10-13
tsTaskID_t   gTsFWUpdateTaskID_c;           // FW Update Task ID
tsTaskID_t   gTsIncomingCallTaskID_c;
tsTaskID_t   gTsDeviceControlTaskID_c;
tsTaskID_t	 gTsStorageTaskID_c;
tsTaskID_t	 gTsLEDTaskID_c;


volatile Timer_ID_Typedef            gKeyTIMID=TIMER_ERROR;  //Key Timer, to detect 'long press' and 'short press'
volatile Timer_ID_Typedef            gShutDownOledTIMID=TIMER_ERROR;


volatile uint8_t   gKeyPressTIMOut  =  false;     		//flag, True--'long press' timeout
volatile uint8_t   gKeyPressStatus  =  Valid;     //flag, Valid--Key is available
volatile uint8_t   gKeyLongPressStatus = OFF;     //flag, ON -- Long press
volatile uint8_t   gMCUStatus       =  Stop;      //flag, Stop--MCU is in stop
volatile uint8_t   gWakeUpNeedOLEDON = RESET;     //flag, RESET--No need to wakeup OLED
volatile uint8_t   gSyncACK =TRUE;                //flag, TRUE--SPI ACK
volatile uint8_t   gAlarmNotEnoughSpace = false;  //not enough space alarm flag
volatile uint8_t   gFlagSyncDataInProgress = false; //Sync Data In Progressing flag
#ifdef EMT_Test_App
volatile uint8_t   MakeDataCnt = 0;
#endif
uint64_t Tick = 0;

/* Private functions ---------------------------------------------------------*/
void NVIC_Congiguration(void);
void ActiveProgressNVIC_Congiguration(void);
void RTC_Config(void);
void nRF51822GPIO_Config(void);
void GPIO_Def_Config(void);
void ActiveProgress(void);
void FactoryLowPower(void);
void Self_Check_Progress(void);
void MX25L1606E_SelfTest(void);
void AFE44xx_SelfTest(void);
void BatterySample_SelfTest(void);
void nRF51822_SelfTest(void);
void TestGPIO_Config(void);
void ResetnRF51822Chip(void);
void AppModifySN(void);
static void u32toBCDConvert10Pos(uint32_t DataIn, uint8_t *pDatOut);
//long GetTick(int YY,int MM,int DD,int HH,int MMin,int SS);
//void OutputGMTIME(long tim);
void OSASTest(void);
void TestCreatFlashPartition(void);

/******************************************************************
*                        gSubFunc_Stat_Set                        *
* [Yun] set global Sub module status                              *
*******************************************************************/
void gSubFunc_Stat_Set(uint16_t mask, uint8_t newState)
{
	gSubFuncState = (newState == OFF)? gSubFuncState & (~mask) : gSubFuncState | mask;
}
/******************************************************************
*                        gSubFunc_Stat_Get                        *
* [Yun] Get global Sub module status                              *
*******************************************************************/
uint16_t gSubFunc_Stat_Get(uint16_t mask)
{
	return(gSubFuncState & mask);
}


void AccRead(Timer_ID_Typedef TIMID)
{
    uint8_t   response = 0;
	AxesRaw_t data;
    
    response = LIS3DH_GetAccAxesRaw(&data);
    LIS3DH_ConvAccValue(&data);
    if(response == 1)
    {
        printf("%6d,%6d,%6d\r\n", data.AXIS_X, data.AXIS_Y, data.AXIS_Z);
    }
    else
    {
        printf("Acc Read Err\r\n");
    }
//    uint8_t Value = 0;
//	static uint8_t cnt=0;
//	cnt++;
//	
//	LIS3DH_GetClickResponse(&Value);
//	if(Value == 0x00)
//	{
//		printf("%d,0x%x\r\n",cnt,Value);
//	}
//	else
//	{
//		printf("------------>  %d,0x%x\r\n",cnt,Value);
//	}
//    uint8_t i=0;
//    ACC_FIFO_DATA_T *pdata;
//    ACC_FIFO_DATA_T data[32];
//    pdata = data;
//    LIS3DH_ReadReg(LIS3DH_FIFO_SRC_REG,&Value);
//    printf("WTM Bit:0x%x\r\n",Value & 0x80);
//    if((Value & 0x80) == 0x80)
//    {
//        bsp_accelero_fifo_read(pdata);
//        for(i=0;i<32;i++)
//        {
//            printf("X=%6d Y=%6d Z=%6d \r\n", pdata[i].x, pdata[i].y, pdata[i].z);
//        }
//    }
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void IdleTask(event_t events)
{
//	LowPowerManage();
}

int main(void)
{
	/*!< At this stage the microcontroller clock setting is already configured,
	   this is done through SystemInit() function which is called from startup
	   file (startup_stm32l1xx_xx.s) before to branch to application main.
	   To reconfigure the default setting of SystemInit() function, refer to
	   system_stm32l1xx.c file
	 */
	/* Add your application code here*/

    SysConfigInfo_t DeviceCFInfo;    //Device configure information
    FWUpdateInfo_t  FWUpdateInfo;    //FW UpDate Config Information
    uint32_t u32Crc_eeprom = 0;
	bool  Flag_ActiveProgress=false;
    ReturnMsg msg = FlashIsBusy;
//	uint32_t i;
    //uint8_t EEPROMData[256] = {0};
    //uint8_t EEPROMData_readout[256] = {0};


#ifdef RDALLEEPROM
	uint16_t index=0;
	uint16_t j=0;
	uint8_t EEPROMData[256];
    uint8_t EEPROMData_readout[256] = {0};
    uint32_t pStartAddr;
    uint8_t state = 0;
#endif

	#ifdef SECTICK_DEBUG_EN
		date_str_typedef rtc_date_init;
		time_str_typedef rtc_time_init;
	#endif

	#ifdef RTC_TEST_ENABLE
		date_str_typedef rtc_date_init;
		time_str_typedef rtc_time_init;
		uint32_t rtc_test_u32;
	#endif

	/* 	Interrupt Vector Table 				*/
	NVIC_SetVectorTable(NVIC_VectTab_FLASH,APP_START_ADDRESS);


	/* 	Disable BOR Function if NOT			*/
	if(
		FLASH_OB_GetBOR() != OB_BOR_OFF
	)
	{
		FLASH_If_Init();
		/* 	Unlock Flash 			*/
		FLASH_OB_Unlock();
		/* 	Config the BOR to OFF 	*/
		FLASH_OB_BORConfig(OB_BOR_OFF);
		/* 	Launch Flash Programming process	*/
		FLASH_OB_Launch();
		/* 	Reset 					*/
		SoftReset();
	}

    /*  Init the driver layer       */
    cc_app_afe_drv_interface_init();
    /*  Init the gsensor interface  */
    cc_app_gsensor_drv_interface_init();
    /*  Init the algorithm moduel   */
    spo2_hr_cal_mod_init();
    /*  Init the Global data        */
    gHRSpO2Val.HrSpO2DataRep_pst = cc_alg_get_spo2_hr_result_str();
    
	/* 	Initial value of basic device information 		*/
    gHRSpO2Val.HR_Limit.VLowLimit = 40;		/* 	Hr ultra low threshold	 	*/
	gHRSpO2Val.HR_Limit.LowLimit = 50;		/*	Hr low threshold 			*/
	gHRSpO2Val.HR_Limit.HigLimit = 100;		/* 	Hr high threshold 			*/
	gHRSpO2Val.HR_Limit.VHigLimit = 120;	/* 	Hr ultra high threshold		*/

	gHRSpO2Val.SpO2_Limit.VLowLimit = 90;   /* 	spo2 ultra low threshold 	*/
	gHRSpO2Val.SpO2_Limit.LowLimit = 95;	/* 	spo2 low threshold 			*/
	gHRSpO2Val.SpO2_Limit.HigLimit = 100;	/* 	spo2 high threshold, no affect 	*/
	gHRSpO2Val.SpO2_Limit.VHigLimit = 100;	/* 	spo2 ultra high threshold, no affect 	*/


//	userInfo.height = 178;     				/* 	user height, unit cm						*/
//    userInfo.weight = 70;					/* 	user weight, unit kg						*/
//	userInfo.age = 35;						/* 	user age	 								*/
//	userInfo.sex = 0;   					/* 	user mate	 								*/

	MonitorTemplate.VibrateSwitch = DISABLE;    /* 	Disable Vibrate by default 				*/

	g_DeviceInfomation.DevSupportCap = DEVICE_SUPPORT_ABILITY + 0x30;	/* 	Device Capability Define 			*/
	g_DeviceInfomation.pDevREV = DevREV;
	g_DeviceInfomation.pFWREV = FWDEV;
	g_DeviceInfomation.pSeriNum = (const uint8_t *)SeriNum_ADDRESS;
	g_DeviceInfomation.pBootREV = (const uint8_t *)STM32_BOOT_VERSION_ADDR;

    /* Set the Default HR SpO2 Measure to Normal Mode */
    gFlagHR_SpO2Measure = HRSpO2_Measure_Normal;
    /* 	Get Data Length 	*/
    MonitorTemplate.ReadLength = MT_STORAGE_DATA_LEN;

	#ifdef EMT_Test_App
	  MakeDataCnt = 1;
	#endif

		 /* platform self-checking progress begin */
		#ifdef SelfCheck_EN
			Self_Check_Progress();
		#endif
		/* end of platform self-checking progress */

		/* Module Init */
		GPIO_Def_Config();
		Delay_Init();

		TestGPIO_Config();
		if(Device_Mode == Device_Mode_FactoryTest)
		{
            //Self_Check_Progress();
		}
		/* end of platform self-checking progress */

		/* Start Bootloader update procedure 		*/
        #ifdef FWUpdate_Debug
        	USART_Configuration();
		#endif
        #ifdef FWUpdate_Debug
            printf("\r\nSTM32_REDHARE ReStart...\r\n");
            printf("\r\nVersion: %d.%d.%d\r\n",FWDEV[0]-0x30,FWDEV[1]-0x30,FWDEV[2]-0x30);
        #endif
        GetFWUpDateConfigInfo(&FWUpdateInfo);

        if((FWUpdateInfo.FWUpdateUPGRADEStatus == M95M01_UPGRADE) &&
            (FWUpdateInfo.FWUpdateFIRMWAREType == M95M01_BOOT_FIRMWARE))
        {
            #ifdef FWUpdate_Debug
            printf("\r\nUpdate the bootloader firmware...\r\n");
            #endif
            u32Crc_eeprom = ((uint32_t)FWUpdateInfo.FWUpdateCRC[0]) | ((uint32_t)FWUpdateInfo.FWUpdateCRC[1] << 8) |
                            ((uint32_t)FWUpdateInfo.FWUpdateCRC[2] << 16) | ((uint32_t)FWUpdateInfo.FWUpdateCRC[3] << 24);
            UpdateBootloader(u32Crc_eeprom);
        }
		/* End of Bootloader update procedure 		*/

        /*  Start to clear the history data */
        USART_Configuration();
        #ifdef STORAGE_LOG
        GetStorageTotelLengthErrorLog(&StorageTotelLengthErrorLog);
        if(1 == StorageTotelLengthErrorLog.u8State)
        {
            ExtFLASH_DispalyError();
        }
        #endif
        MX25_SPI_Configuration();
        GetSysConfigInfo(&DeviceCFInfo);
        if(DeviceCFInfo.DataFormate != 5)
        {
EraseMX25Chip:
            msg = CMD_CE();
            switch(msg)
            {
                case FlashIsBusy:
                    #ifndef FLASH_DEBUG
                        printf("FlashIsBusy\r\n");
                    #endif
                    Flash_PowerCtl(ON);
                    delay1ms(2);
                    MX25_SPI_Configuration();
                    delay1ms(2);
                    goto EraseMX25Chip;
//                    break;

                case FlashTimeOut:
                    #ifndef FLASH_DEBUG
                        printf("FlashTimeOut\r\n");
                    #endif
                    delay1ms(2000);
                    break;

                default:
                    #ifdef FLASH_DEBUG
                        printf("FlashOperationSuccess\r\n");
                    #endif
                    break;
            }

            DeviceCFInfo.DataFormate = 5;
            SetSysConfigInfo(DeviceCFInfo);
        }

        /*****************************发布前屏蔽**************************************************/
        //CMD_CE();
        //CMD_SE(FLASH_RAW_DATA_BASE_ADDR);
        //CreatePartitionForTest(FLASH_HEADER_SECTOR_BASE_ADDR - (6 * SECTOR_SIZE),0x07e00318,30);
        //CreatePartitionForTest(FLASH_HEADER_SECTOR_BASE_ADDR - (4 * SECTOR_SIZE),0x07e00319,30);
        //DataMemoryRestore();
        /*for(i = (FLASH_HEADER_SECTOR_BASE_ADDR/PAGE_SIZE - 20);i <= FLASH_HEADER_SECTOR_BASE_ADDR/PAGE_SIZE;i ++)
        {
            CheckWrite(i * PAGE_SIZE, PAGE_SIZE);
        }*/
        /*for(i = 0x0000;i <= 0x0040;i ++)
        {
            CheckWrite(i * PAGE_SIZE, PAGE_SIZE);
        }*/
        //CheckWrite(FLASH_HEADER_SECTOR_BASE_ADDR - (3 * SECTOR_SIZE), PAGE_SIZE);
        //CheckWrite(0, PAGE_SIZE);
        //CheckWrite(FLASH_HEADER_SECTOR_BASE_ADDR, PAGE_SIZE);
        //CheckWrite(FLASH_BACKUP_SECTOR_BASE_ADDR, PAGE_SIZE);
        //CheckWrite(0x00027200, PAGE_SIZE);

        /*for(i = 0;i < 256;i ++)
        {
            EEPROMData[i] = i;
        }*/
        /*for(i = 0;i < 10000;i ++)
        {
            printf("i = %d\r\n",i);
            if(FlashOperationSuccess == CMD_SE(FLASH_BACKUP_SECTOR_BASE_ADDR))
            {
                if(FlashOperationSuccess == CMD_PageProgram(FLASH_BACKUP_SECTOR_BASE_ADDR, EEPROMData, sizeof(EEPROMData)))
                {
                    CMD_READ(FLASH_BACKUP_SECTOR_BASE_ADDR, EEPROMData_readout, sizeof(EEPROMData));
                    if(0 != memcmp(EEPROMData, EEPROMData_readout, sizeof(EEPROMData)))
                    {
                        printf("***memcmp***\r\n");
                        while(1);
                    }
                }
                else
                {
                    printf("***CMD_PageProgram,FlashTimeOut***\r\n");
                    while(1);
                }
            }
            else
            {
                 printf("***CMD_SE,FlashTimeOut***\r\n");
                 while(1);
            }
        }*/
        //while(1);
        /*****************************发布前屏蔽**************************************************/

        /* Storage Area Init */
        ExtFLASH_StorageInit();

        /* Get Save MTID */
        /* 	Read device config file from DataMemory	*/
        GetSysConfigInfo(&DeviceCFInfo);
		MonitorTemplate.MTID = DeviceCFInfo.MTID;

		/* Start Factory Mode */
		/* 	Read device config file from DataMemory	*/
        GetSysConfigInfo(&DeviceCFInfo);
	#ifdef ACTIVE_PROCESS_ENABLE
        DeviceCFInfo.AppActiveStatus = M95M01_APP_ACTIVE;    //硬件6.0不需要激活
		if(DeviceCFInfo.AppActiveStatus != M95M01_APP_ACTIVE)  /* 	in case , need active process 	*/
	#else
		if(0)
	#endif
		{
				ActiveProgress();			/* 	Active device process 		*/
				Flag_ActiveProgress = true;
		}
		/* End of Factory Mode */

		/* Start App Modify SN */
        /* 	Read device config file from DataMemory	*/
        GetSysConfigInfo(&DeviceCFInfo);
		if(DeviceCFInfo.AppModifySNStatus == M95M01_APP_MODIFYSN) /* in case , App modify the SN */
		{
			AppModifySN();
		}
		/* End of App Modify SN */

		/* Module Init */
		if(Flag_ActiveProgress == false)
		{
			GPIO_Def_Config();
			RTC_Config();
		}
		USART_Configuration();

		PeriPower(ON);
		if(Flag_ActiveProgress == false)
		{
			MCO_OutPutLSE(ON);         //MCO输出32.768KHz，供nRF52822使用
		}
// 		KEY_Config();
 		OLED_Configuration();

 		SPI_Configuration();
        MX25_SPI_Configuration();

#ifdef RDCPUINFO
		GetCPUInfo();
#endif
        BATVolDetectInit();
        BATChargeInit();

		if(Flag_ActiveProgress == false)
		{
			nRF51822GPIO_Config();
			ResetnRF51822Chip();
		}
		NVIC_Congiguration();

        Delay_ms(2000); 				/* 	Delay for battery level sample 		*/
        BATVoltageDetect();  			/* 	Sample the battery level 			*/

        printf("Start...\r\n");
        
        /* test for read acc */
        ACC_CONFIG_T cfg;
        cfg.FS = LIS3DH_FULLSCALE_4;
        cfg.ODR = LIS3DH_ODR_50Hz;
        cfg.WaterMark = 20;
        bsp_accelero_init_step(NORMAL_READ,&cfg);
        
        TIM_Cfg_Typedef         Tim_Cfg_Key_Index;           //Key timer配置，用于检测长、短按键
        TIM_Basic_Cfg_Typedef 	Tim_Cfg_Key;
        Timer_ID_Typedef        gAccReadTIMID = TIMER_ERROR;  
        
        /* 配置KEY定时器，用于长短按键检测 */
        Tim_Cfg_Key.enuTimerType = TIM_TYPE_MS;
        Tim_Cfg_Key.u16TimePeriod = 50;
        Tim_Cfg_Key.NVIC_IRQChannelPreemptionPriority = KEY_TIMPreemptionPriority;
        Tim_Cfg_Key.pIntCallBack = AccRead;

        /* Init timer top define */
        Tim_Cfg_Key_Index.TimerMode 			= TIM_MODE_BASIC;
        Tim_Cfg_Key_Index.TimerBasicCfg 		= &Tim_Cfg_Key;
        Tim_Cfg_Key_Index.TimerPWMCfg 			= NULL;

        Tim_Cfg_Key_Index = Tim_Cfg_Key_Index;   //避免警告

    	gAccReadTIMID = Timer_Allocate(&Tim_Cfg_Key_Index);
    	Start_Timer_Cnt(gAccReadTIMID);

        
//		/*****Trigger Hard fault handler for test  *****/
//		*(uint8_t *)(0xFFFFFFFF) = 0;
//		/************************/
		/* Task Scheduler Init */
		TS_Init();

		/* Creat Tasks */
//		gKeyTaskID = TS_CreateTask(gTsKeyTaskPriority_c,KeyTask);
//		gOledDisTaskID = TS_CreateTask(gTsOledDisTaskPriority_c,OledDisTask);
//		gTsSpO2TaskID_c = TS_CreateTask(gTsSpO2TaskPriority_c,SpO2_Task_Handler);
//		gTsSPITranslateTaskID_c = TS_CreateTask(gTsSPITranslateTaskPriority_c,SPITranslate_Task_Handler);
//		gTsSecTickTaskID_c = TS_CreateTask(gTsSecTickTaskPriority_c,SecTick_Task_Handler);
//		gTsMonitorTemplatID_c = TS_CreateTask(gTsMonitorTemplateTaskPriority_c,MonitorTemplate_Task_Handler);
//		gTsPowerManageTaskID_c = TS_CreateTask(gTsPowerManageTaskPriority_c,PowerManage_Task_Handler);
//		gTsSyncDataTaskID_c	= TS_CreateTask(gTsSyncDataTaskPriority_c,SyncData_Task_Handler);
//		gTs3AxesTaskID_c = TS_CreateTask(gTs3AxesTaskPriority_c, Gsensor_Task_Handler);
//		gTsAlarmTaskID_c = TS_CreateTask(gTsAlarmTaskPriority_c, Alarm_Task_Handler);
//		gTsBatManageTaskID_c = TS_CreateTask(gTsBatManageTaskPriority_c,BATManage_Task_Handler);
//		gTsBatMonitorTaskID_c = TS_CreateTask(gTsVoltDetectTaskPriority_c,VoltageDetect_Task_Handler);
//		gTsStepTaskID_c = TS_CreateTask(gTsStepTaskPriority_c,Step_Task_Handler);
//		gTsFWUpdateTaskID_c = TS_CreateTask(gTsFWUpdateTaskPriority_c,FW_Update_Handler);
//        gTsIncomingCallTaskID_c = TS_CreateTask(gTsIncomingCallTaskPriority_c,Call_Handler);
//		gTsDeviceControlTaskID_c = TS_CreateTask(gTsDeviceControlTaskPriority_c,DeviceControl_Handler);
//		gTsStorageTaskID_c = TS_CreateTask(gTsStorageManageTaskPriority_c,Storage_Task_Handler);
//        gTsLEDTaskID_c = TS_CreateTask(gTsLedTaskPriority_c, LED_Task_Handler);

		Device_Mode = Device_Mode_RTC;
		/* 上电时显示RTC、Blue ICon*/
		OLED_DisplayClear();      //清除图标显示

		/* 	Detect the charge status while power up 		*/
        if(GPIO_ReadInputDataBit(GPIO_BAT_PG,EXTI_LineBAT_PG) == Bit_RESET)
        {
            /* Send the event to BatManage task */
            TS_SendEvent(gTsBatManageTaskID_c,gBATEnterChargeEvent);
        }
		else
		{
			OLED_DisplayClear();			/* Clear OLED 				*/
			OLED_DisplayProgress(1);    	/* show the dot line 		*/
			OLED_DisplayBatteryLevelICON(GetBatScaleLevel()); /* 	Display battery level 		*/

			/* 	Display Monitor template icon		*/
			if(MonitorTemplate.MTSwitch == 0)
			{
				OLED_DisplayMonitorTemplateICON(OFF); /* Turn off Monitor template icon 		*/
			}
			else
			{
				OLED_DisplayMonitorTemplateICON(ON); /* Turn on Monitor template icon 		*/
			}

			/* 	Display Bluetooth icon 				*/
			if(Device_State == Device_WorkWithAPP)
			{
				OLED_DisplayBlueToothICON(ON);		/* Turn on Monitor template icon 		*/
			}
			else if(Device_State == Device_Alone)
			{
				OLED_DisplayBlueToothICON(OFF);		/* Turn off Monitor template icon 		*/
			}
            if(gFlagHR_SpO2Measure == HRSpO2_Measure_Normal)
            {
                OLED_DisplayTestMode(OFF);
            }
            else
            {
                OLED_DisplayTestMode(ON);
            }
			/* 	Display RTC time 					*/
			OLED_UpdateRTC();
		}
        
		/* 	Begin to count down to shut down OLED 		*/
		TS_SendEvent(gOledDisTaskID,gOledDisEventClearModeDis_c);

		/* 	Send event to init step counter and auto wakeup func	*/
		TS_SendEvent(gTsStepTaskID_c, gStepStartEvent);

		/* Start Scheduler */
		TS_Scheduler();
}
/**
  * @brief   GPIO default configuration
  * @param  None
  * @retval None
  */
void GPIO_Def_Config(void)
{
		GPIO_InitTypeDef        GPIO_InitStructure;

		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOH, ENABLE);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7
									| GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_15 | GPIO_Pin_8;


        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOB, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOC, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOD, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOE, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOH, &GPIO_InitStructure);

		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, DISABLE);
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, DISABLE);
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, DISABLE);
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, DISABLE);
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOH, DISABLE);

}
/*******************************************************************************
* Function Name  : MCO_OutPutLSE
* Description    : MCO输出LSE 32.768KHz
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MCO_OutPutLSE(uint8_t status)
{
		GPIO_InitTypeDef        GPIO_InitStructure;

		if(status == ON)
		{
			PWR_RTCAccessCmd(ENABLE);        //使能RTC,输出32.768KHz
			RCC_LSEConfig(RCC_LSE_ON);

			/* GPIOA Periph clock enable */

			RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

			/* Configure PA8 in MCO output mode */
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIOA, &GPIO_InitStructure);

			GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_MCO);
			RCC_MCOConfig(RCC_MCOSource_LSE,RCC_MCODiv_1);   //输出LSE，32.768KHz，供nRF51822使用
		}
		else
		{
			PWR_RTCAccessCmd(DISABLE);        //使能RTC,输出32.768KHz
			RCC_LSEConfig(RCC_LSE_OFF);

			RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

			/* Configure PA8 in MCO output mode */
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
		}
}
/*******************************************************************************
* Function Name  : RTC_Config
*******************************************************************************/
void RTC_Config(void)
{
	/**********RTC*********************/
	Calendar_Init();
	Calendar_Alarm_Init();
	SetCalendar_Alarm_B(0,0,10);
	Calendar_RTC_Period_Wakeup_Init(16384,RTC_Counter_Update);
}

/*******************************************************************************
* Function Name  : FactoryLowPower()
* Description    : 工厂低功耗模式
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FactoryLowPower(void)
{
	gKeyLongPressStatus = OFF;
	g_PassKey.Status = 0;
	g_PassKey.Connect_Status = 0;

	PeriPower(OFF);
	MCO_OutPutLSE(OFF);    //Disable MCO for NRF51822
	OLED_DriveSystemPower(OFF);
	OLED_DeConfiguration();

	GPIO_LowPower_Config();

    MX25L_GPIO_LowPower_Config();
    Flash_PowerCtl(OFF);

    /* RTC Low Power Config */
    PWR_RTCAccessCmd(ENABLE);

    RTC_WriteProtectionCmd(DISABLE);
    RTC_EnterInitMode();

    /* Reset Backup Domain */
    RCC_RTCResetCmd(ENABLE);
    RCC_RTCResetCmd(DISABLE);
    RCC_RTCCLKCmd(DISABLE);
    RCC_LSEConfig(RCC_LSE_OFF);


	/* Disable the Wakeup Interrupt */
	RTC_ITConfig(RTC_IT_WUT, DISABLE);
	/* Clear Wake Up flag */
	PWR_ClearFlag(PWR_FLAG_WU);

	USART_DeConfiguration();

	PWR_UltraLowPowerCmd(ENABLE);
	PWR_EnterSTOPMode(PWR_Regulator_LowPower,PWR_STOPEntry_WFI);

	/* Stop here */

	/* Code for system wakeup from system stop mode */
	RCC_MCOConfig(RCC_MCOSource_LSE,RCC_MCODiv_1);   //输出LSE，32.768KHz，供nRF51822使用
	SystemInit_AfterWakeup();   //
	PWR_RTCAccessCmd(ENABLE);        //使能RTC,输出32.768KHz
	PeriPower(ON);

	/* Enable the Wakeup Interrupt */
	RTC_ITConfig(RTC_IT_WUT, ENABLE);
	//OLED_Configuration();
	USART_Configuration();
}
/*******************************************************************************
* Function Name  : ActiveProgress()
* Description    : 开机激活程序
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ActiveProgress(void)
{
	uint8_t TimeCnt=0;
	uint8_t Time1SCnt=0;
    SysConfigInfo_t DeviceCFInfo;  //Device configure information

	/* Module Init */
	GPIO_Def_Config();
	USART_Configuration();
	PeriPower(ON);
	MCO_OutPutLSE(ON);         //MCO输出32.768KHz，供nRF52822使用
	KEY_Config();
	OLED_Configuration();
	SPI_Configuration();
	nRF51822GPIO_Config();
	ResetnRF51822Chip();
	ActiveProgressNVIC_Congiguration();

	/* Set BMA250E to Low Power */
	Set_BMA250E_LowPower();

	g_PassKey.Status = 0;   //初始为没有收到密码验证
	g_PassKey.Connect_Status = 0;
	Device_Mode = Device_Mode_Factory;
	while(1)
	{
		if(gKeyLongPressStatus == ON) // 长按事件
		{
			gKeyLongPressStatus = OFF;
			Time1SCnt=0;
			OLED_DisplayClear();
			OLED_DisplayStartFlash();    //显示开机画面
			while(GetOLED_DisplayStartFlashStatus() == false);  //显示开机画面完成

			RTC_Config();
			MCO_OutPutLSE(ON);  //开启蓝牙通讯

            //激活成功
            {
                /* Read Sys Config Info */
                GetSysConfigInfo(&DeviceCFInfo);

                /* Device Actived */
                DeviceCFInfo.AppActiveStatus = M95M01_APP_ACTIVE;

                /* Write Sys Config Info */
                SetSysConfigInfo(DeviceCFInfo);
                /* SoftReset */
                //SoftReset();
                return;
            }
        }
		else  //不是长按事件
		{
			if(TimeCnt < 15)
			{
				Delay_ms_soft(500);
				TimeCnt++;
				Time1SCnt++;
				if(GPIO_ReadInputDataBit(GPIO_KEYWP,GPIO_Pin_KEYWP) != Bit_SET)
				{
					if(Time1SCnt >= 5)
					{
						gKeyLongPressStatus = ON;
					}
				}
				else
				{
					Time1SCnt=0;
				}
			}
			else
			{
				TimeCnt = 0;
				Time1SCnt = 0;
				/* Enter Factory Lower Power */
				FactoryLowPower();
			}
		}
	}
}
/*******************************************************************************
* Function Name  : GetPeriphPowerStatus()
* Description    : 获取外设电源状态
* Input          : None
* Output         : None
* Return         : Power: ON/OFF
*******************************************************************************/
uint8_t GetPeriphPowerStatus(void)
{
	return gPeriphPowerStatus;
}
/*******************************************************************************
* Function Name  : PeriPower()
* Description    : 控制模块电源
* Input          : Power: ON/OFF
* Output         : None
* Return         : None
*******************************************************************************/
void PeriPower(uint8_t Power)
{
	GPIO_InitTypeDef        GPIO_InitStructure;

	if(Power == ON)
	{
		gPeriphPowerStatus = ON;

		/* Power On */
		RCC_AHBPeriphClockCmd(RCC_PowerPeriph, ENABLE);

		#ifdef BOARD_REDHARE_V3_0

		/* Power on AFE power supply */
		RCC_AHBPeriphClockCmd(RCC_AFE44xx_PowerPeriph, ENABLE);

		#endif
		/* Power ON Periph Pin config */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_Power;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIO_Power, &GPIO_InitStructure);

		/* 高电平开启外设电源 */
		GPIO_SetBits(GPIO_Power,GPIO_Pin_Power);

		#ifdef BOARD_REDHARE_V3_0
		/* Power on AFE power supply */

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_AFE44xx_Power;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIO_AFE44xx_Power_Port, &GPIO_InitStructure);

		GPIO_SetBits(GPIO_AFE44xx_Power_Port,GPIO_Pin_AFE44xx_Power);
		#endif
	}
	else
	{
		gPeriphPowerStatus = OFF;

        RCC_AHBPeriphClockCmd(RCC_PowerPeriph, ENABLE);

		/* Power OFF Periph Pin config */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_Power;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
		GPIO_Init(GPIO_Power, &GPIO_InitStructure);

		GPIO_ResetBits(GPIO_Power,GPIO_Pin_Power);

		#ifdef BOARD_REDHARE_V3_0
        
            /* Power on AFE power supply */
            RCC_AHBPeriphClockCmd(RCC_AFE44xx_PowerPeriph, ENABLE); 
        
			/* Power on AFE power supply */
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_AFE44xx_Power;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
            GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
			GPIO_Init(GPIO_AFE44xx_Power_Port, &GPIO_InitStructure);
            
            GPIO_ResetBits(GPIO_AFE44xx_Power_Port,GPIO_Pin_AFE44xx_Power);
		#endif
	}
}
/**
  * @brief  NVIC Configuration Function.
  * @param  None
  * @retval None
  */
void NVIC_Congiguration(void)
{
	NVIC_InitTypeDef   NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* Enable and set EXTI0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = KEY_PressPreemptionPriority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = KEY_PressSubPriority;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

    /*Init NVIC to PG pin in charging(huayun,2014-07-21)*/
    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = ADS1115_RDYINTPreemptionPriority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = ADS1115_RDYINTSubPriority;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = BT_REQNPreemptionPriority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = BT_REQNSubPriority;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/*Init RTC Auto wakeup NVIC*/
	NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = RTC_PeriodWakeUpPreemptionPriority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = RTC_PeriodWakeUpSubPriority;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the RTC Alarm Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = RTC_Alarm_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = MonitorTemplatePreemptionPriority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = MonitorTemplateSubPriority;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/*Init AFE DRDY NVIC*/
	NVIC_InitStructure.NVIC_IRQChannel = AFE44xxDRDY_IRQ_CH;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = AFE44xxDRDYPreemptionPriority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = AFE44xxDRDYSubPriority;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	#ifndef BOARD_REDHARE_V3_0
	/*Init AFE Diag NVIC*/
	NVIC_InitStructure.NVIC_IRQChannel = AFE44xxDIAG_IRQ_CH;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = AFE44xxDIAGPreemptionPriority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = AFE44xxDIAGSubPriority;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif

	/*Init 3AXES INT2*/
	NVIC_InitStructure.NVIC_IRQChannel = LIS3DH_INT2_CH;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = LIS3DHINT2PreemptionPriority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = LIS3DHINT2SubPriority;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/*Init 3AXES INT1*/
	NVIC_InitStructure.NVIC_IRQChannel = LIS3DH_INT1_CH;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = LIS3DHINT1PreemptionPriority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = LIS3DHINT1SubPriority;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}
/**
  * @brief  NVIC Configuration Function.
  * @param  None
  * @retval None
  */
void ActiveProgressNVIC_Congiguration(void)
{
	NVIC_InitTypeDef   NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* Enable and set EXTI0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = KEY_PressPreemptionPriority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = KEY_PressSubPriority;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = BT_REQNPreemptionPriority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = BT_REQNSubPriority;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/*Init AFE DRDY NVIC*/
	NVIC_InitStructure.NVIC_IRQChannel = AFE44xxDRDY_IRQ_CH;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = AFE44xxDRDYPreemptionPriority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = AFE44xxDRDYSubPriority;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
/*******************************************************************************
* Function Name  : nRF51822GPIO_Config
* Description    : nRF51822 GPIO Config,
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void nRF51822GPIO_Config(void)
{
	/* GPIOLED Periph clock enable */
	GPIO_InitTypeDef   GPIO_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;

	RCC_AHBPeriphClockCmd(PT_RCC_AHBPeriph_GPIO_BTREQN, ENABLE);
	RCC_AHBPeriphClockCmd(PT_RCC_AHBPeriph_GPIO_BTSYNC, ENABLE);
	RCC_AHBPeriphClockCmd(PT_RCC_AHBPeriph_GPIO_BTRESET, ENABLE);

	GPIO_InitStructure.GPIO_Pin = PT_GPIO_Pin_BTREQN;  //51822 REQN
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(PT_GPIOBTREQN, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PT_GPIO_Pin_BTSYNC;  //51822 SYNC
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(PT_GPIOBTSYNC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PT_GPIO_Pin_BTRESET;  //51822 Reset
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(PT_GPIOBTRESET, &GPIO_InitStructure);

	GPIO_SetBits(PT_GPIOBTRESET, PT_GPIO_Pin_BTRESET);

	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(PT_RCC_APBPeriph_SYSCFG_BTREQN, ENABLE);
	RCC_APB2PeriphClockCmd(PT_RCC_APBPeriph_SYSCFG_BTSYNC, ENABLE);

	SYSCFG_EXTILineConfig(PT_EXTI_PortSourceGPIO_BTREQN, PT_EXTI_PinSourceBTREQN);
	SYSCFG_EXTILineConfig(PT_EXTI_PortSourceGPIO_BTSYNC, PT_EXTI_PinSourceBTSYNC);

	EXTI_InitStructure.EXTI_Line = PT_EXTI_LineBTREQN | PT_EXTI_LineBTSYNC;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}

/*******************************************************************************
* Function Name  : Self_Check_Progress
* Description    : Self_Check_Progress
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Self_Check_Progress(void)
{
		uint8_t KeyPressCnt=0;
		uint8_t SN[10]={0};
		uint32_t SeriNum = 0;

		/* GPIOLED Periph clock enable */
		GPIO_InitTypeDef   GPIO_InitStructure;

		RCC_AHBPeriphClockCmd(RCC_KEYPeriph_KEYWP, ENABLE);

		/* Configure Key pins as input floating */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_KEYWP;
		GPIO_Init(GPIO_KEYWP, &GPIO_InitStructure);

		MCO_OutPutLSE(ON);         //MCO 32.768khz to nrf51822
		USART_Configuration();
		Delay_Init();

		SeriNum = (uint32_t)g_DeviceInfomation.pSeriNum[0]<<24 | (uint32_t)g_DeviceInfomation.pSeriNum[1]<<16 |
				  (uint32_t)g_DeviceInfomation.pSeriNum[2]<<8 | (uint32_t)g_DeviceInfomation.pSeriNum[3];

		u32toBCDConvert10Pos((uint32_t)(SeriNum),SN);

		SelfCheck_TextUpdate(ID_SN,SN,10);

		SelfCheckLCDCtl(ID_SelfCheckProgramStatus,CMD_Start);
		SelfCheckLCDCtl(ID_RedLEDStatus,CMD_STOP);
		SelfCheckLCDCtl(ID_GreenLEDStatus,CMD_STOP);
		SelfCheckLCDCtl(ID_OrangeLEDStatus,CMD_STOP);
		SelfCheckLCDCtl(ID_VirbreStatus,CMD_STOP);
		SelfCheckLCDCtl(ID_OLEDStatus,CMD_STOP);
		SelfCheckLCDCtl(ID_KeyStatus,CMD_Checking);
		SelfCheckLCDCtl(ID_nRF51822Status,CMD_Checking);
		//SelfCheckLCDCtl(ID_M95M01Status,CMD_Checking);
		SelfCheckLCDCtl(ID_MX25L1606EStatus,CMD_Checking);
		SelfCheckLCDCtl(ID_BMA250EStatus,CMD_Checking);
		SelfCheckLCDCtl(ID_AFE44xxStatus,CMD_Checking);
		SelfCheckLCDCtl(ID_ADS1118Status,CMD_Pass);
        SelfCheckLCDCtl(ID_BatterySmpStatus,CMD_Checking);

		/* Periph Power Check */
		PeriPower(ON);

	    /* LED Check */
//		LED_GPIO_Init();
//		SelfCheckLCDCtl(ID_RedLEDStatus,CMD_Start);
//		GPIO_SetBits(GPIO_LED0,GPIO_Pin_LED0);
//		Delay_ms(3000);
//		SelfCheckLCDCtl(ID_RedLEDStatus,CMD_STOP);
//		GPIO_ResetBits(GPIO_LED0,GPIO_Pin_LED0);
//
//		SelfCheckLCDCtl(ID_GreenLEDStatus,CMD_Start);
//		GPIO_SetBits(GPIO_LED1,GPIO_Pin_LED1);
//		Delay_ms(3000);
//		SelfCheckLCDCtl(ID_GreenLEDStatus,CMD_STOP);
//		GPIO_ResetBits(GPIO_LED1,GPIO_Pin_LED1);

//		SelfCheckLCDCtl(ID_OrangeLEDStatus,CMD_Start);
//		GPIO_SetBits(GPIO_LED0,GPIO_Pin_LED0);
//		GPIO_SetBits(GPIO_LED1,GPIO_Pin_LED1);
//		Delay_ms(3000);
//		SelfCheckLCDCtl(ID_OrangeLEDStatus,CMD_STOP);
//		GPIO_ResetBits(GPIO_LED0,GPIO_Pin_LED0);
//		GPIO_ResetBits(GPIO_LED1,GPIO_Pin_LED1);
//		LED_GPIO_DeInit();

		/* Motor Check */
        SelfCheckLCDCtl(ID_VirbreStatus,CMD_Start);
        Motor_Init();
		GPIO_ResetBits(GPIO_Motor,GPIO_Pin_Motor);
        Delay_ms(1000);
        SelfCheckLCDCtl(ID_VirbreStatus,CMD_STOP);
        Motor_DeInit();

		/* OLED Display Check */
		OLED_Configuration();
		SelfCheckLCDCtl(ID_OLEDStatus,CMD_Start);
		OLED_DisplayFullScreenBMP(StartPic);
		OLED_DisplayCtl(ON);
		Delay_ms(3000);
		SelfCheckLCDCtl(ID_OLEDStatus,CMD_STOP);
		OLED_DisplayClear();
		OLED_DeConfiguration();

		/* Key Press Check */
		while((GPIO_ReadInputDataBit(GPIO_KEYWP,GPIO_Pin_KEYWP) != Bit_RESET) && (KeyPressCnt < 20))
		{
			SelfCheckLCDCtl(ID_KeyStatus,CMD_PressKey);
			Delay_ms(500);
			KeyPressCnt++;
		}
		if(KeyPressCnt >= 20)
		{
			SelfCheckLCDCtl(ID_KeyStatus,CMD_Fail);
		}
		else
		{
			SelfCheckLCDCtl(ID_KeyStatus,CMD_Pass);
		}

		/* nRF51822 SPI Check */
		nRF51822_SelfTest();

		/* EEPROM Check */
		MX25L1606E_SelfTest();

        /* BMA250E Check*/
		BMA250E_SelfTest();

		/*AFE44xx Check*/
		AFE44xx_SelfTest();

        /* Battery Sample Check */
        BatterySample_SelfTest();

		SelfCheckLCDCtl(ID_SelfCheckProgramStatus,CMD_STOP);
}

void BatterySample_SelfTest(void)
{
        GPIO_InitTypeDef GPIO_InitStructure;
        float fVoltage  = 0;

        /* Enable GPIOs clock */
        RCC_AHBPeriphClockCmd(RCC_BATCEPeriph, ENABLE);

        /* Configure CE pin as input floating */
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_BAT_CE;
        GPIO_Init(GPIO_BAT_CE, &GPIO_InitStructure);

        /* Disables charge */
        GPIO_SetBits(GPIO_BAT_CE,GPIO_Pin_BAT_CE);

        configureGPIO_ADC();
        configureDMA_ADC();
        configureADC_BAT();

        fVoltage = ADC_SampleAndConv(ADC1);

        /* Enables charge */
        GPIO_ResetBits(GPIO_BAT_CE,GPIO_Pin_BAT_CE);

        if((fVoltage > 3.00) && (fVoltage < 4.20))
        {
            SelfCheckLCDCtl(ID_BatterySmpStatus,CMD_Pass);
        }
        else
        {
            SelfCheckLCDCtl(ID_BatterySmpStatus,CMD_Fail);
        }
}

void AFE44xx_SelfTest(void)
{
	/* 	Power on periph			*/
	if(OFF == GetPeriphPowerStatus())  //get peripheral power status
	{
		PeriPower(ON);
	}

	/* 	power on AFE and init timing		*/
	PulseOxiAfe_Init();

 	/* 	AFE diag							*/
 	PulseOxiAfe_Diag_En();
	Delay_ms(20);
	if(PulseOxiAfe_Diag_Check() == false)
	{
		SelfCheckLCDCtl(ID_AFE44xxStatus,CMD_Fail);
	}
	else
	{
		SelfCheckLCDCtl(ID_AFE44xxStatus,CMD_Pass);
	}

	/*	Disable AFE4400 */
	PulseOxiAfe_DeInit();
}
void TestGPIO_Config(void)
{
	    GPIO_InitTypeDef GPIO_InitStructure;

		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD , ENABLE);

		/* Configure USART3 Tx (PD.08) as input */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

		GPIO_Init(GPIOD, &GPIO_InitStructure);

		Delay_ms(20);

		if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_8) == Bit_SET)
		{
			Device_Mode = Device_Mode_FactoryTest; //set to factory mode
		}
}
void nRF51822_SelfTest(void)
{
	uint8_t TestCnt = 0;
	SPI_Configuration();
	nRF51822GPIO_Config();
	NVIC_Congiguration();

	Device_Mode = Device_Mode_Factory;
	g_DeviceInfomation.DeviceInfo_Status = 0;

	ResetnRF51822Chip();
	while((g_DeviceInfomation.DeviceInfo_Status == 0) && (TestCnt < 10))
	{
		Delay_ms(1000);
		TestCnt++;
	}
	if(TestCnt >= 10)
	{
		SelfCheckLCDCtl(ID_nRF51822Status,CMD_Fail);
	}
	else
	{
		SelfCheckLCDCtl(ID_nRF51822Status,CMD_Pass);
	}

}

void MX25L1606E_SelfTest(void)
{
    uint8_t wr_temp[20] = {0};
    uint8_t rd_temp[20] = {0};
    uint8_t index;
	uint32_t id;

	MX25_SPI_Configuration();
	CMD_ReadID(&id);

	if(id != FlashID)
	{
		SelfCheckLCDCtl(ID_MX25L1606EStatus,CMD_Fail);
	}
	else
	{
		for(index = 0;index < sizeof(wr_temp);index ++)
		{
			wr_temp[index] = index + 1;
		}

		CMD_SE(FLASH_BACKUP_SECTOR_BASE_ADDR);
		delay1ms(200);

		CMD_PageProgram(FLASH_BACKUP_SECTOR_BASE_ADDR, wr_temp, sizeof(wr_temp));
		CMD_READ(FLASH_BACKUP_SECTOR_BASE_ADDR, rd_temp, sizeof(wr_temp));

		for(index = 0;index < sizeof(wr_temp);index ++)
		{
			if(*(rd_temp + index) != *(wr_temp + index))
            {
				break;
			}
		}
	}

	if(index >= sizeof(wr_temp))
	{
		SelfCheckLCDCtl(ID_MX25L1606EStatus,CMD_Pass);
	}
	else
	{
		SelfCheckLCDCtl(ID_MX25L1606EStatus,CMD_Fail);
	}

	CMD_SE(FLASH_BACKUP_SECTOR_BASE_ADDR);
	delay1ms(200);
}

void ResetnRF51822Chip(void)
{
    #ifdef FWUpdate_Debug
        printf("Reset nRF51822 Chip...\r\n");
    #endif
    GPIO_ResetBits(PT_GPIOBTRESET,PT_GPIO_Pin_BTRESET);
    Delay_us(100);
    GPIO_SetBits(PT_GPIOBTRESET,PT_GPIO_Pin_BTRESET);
}
static void u32toBCDConvert10Pos(uint32_t DataIn, uint8_t *pDatOut)
{
	pDatOut[9] =         DataIn%10UL + 0x30;
	pDatOut[8] =        DataIn%100UL/10UL + 0x30;
	pDatOut[7] =       DataIn%1000UL/100UL + 0x30;
	pDatOut[6] =      DataIn%10000UL/1000UL + 0x30;
	pDatOut[5] =     DataIn%100000UL/10000UL + 0x30;
	pDatOut[4] =     DataIn%1000000UL/100000UL + 0x30;
	pDatOut[3] =     DataIn%10000000UL/1000000UL + 0x30;
	pDatOut[2] =     DataIn%100000000UL/10000000UL + 0x30;
	pDatOut[1] =     DataIn%1000000000UL/100000000UL + 0x30;
	pDatOut[0] =     DataIn%10000000000UL/1000000000UL + 0x30;
}
/*******************************************************************************
* Function Name  : GetCPUInfo
* Description    : GetCPUInfo
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GetCPUInfo(void)
{
	uint8_t temp[57]= {0};

	unsigned int stacked_r0;
	unsigned int stacked_r1;
	unsigned int stacked_r2;
	unsigned int stacked_r3;
	unsigned int stacked_r12;
	unsigned int stacked_lr;
	unsigned int stacked_pc;
	unsigned int stacked_psr;
	unsigned int stacked_bfar;
	unsigned int stacked_cfsr;
	unsigned int stacked_hfsr;
	unsigned int stacked_dfsr;
	unsigned int stacked_afsr;

	uint8_t	month = 0;
	uint8_t day   = 0;
	uint8_t Hours = 0;
	uint8_t RTC_Minutes = 0;
	uint8_t RTC_Seconds = 0;

//	EEPROM_SPI_ReadFromMemory(temp,M95M01_CONFIG_CPUINFO,sizeof(temp));

	/* Write CPU Information to EEPROM */
	/* Occuring time */
	month = temp[0];
	day   = temp[1];
	Hours = temp[2];
	RTC_Minutes = temp[3];
	RTC_Seconds = temp[4];

	/* R0 */
	stacked_r0 = (uint32_t)((temp[5] << 24) | (temp[6] << 16) | (temp[7] << 8) | (temp[8]));

	/* R1 */
	stacked_r1 = (uint32_t)((temp[9] << 24) | (temp[10] << 16) | (temp[11] << 8) | (temp[12]));

	/* R2 */
	stacked_r2 = (uint32_t)((temp[13] << 24) | (temp[14] << 16) | (temp[15] << 8) | (temp[16]));

	/* R3 */
	stacked_r3 = (uint32_t)((temp[17] << 24) | (temp[14] << 16) | (temp[19] << 8) | (temp[20]));

	/* R12 */
	stacked_r12 = (uint32_t)((temp[21] << 24) | (temp[22] << 16) | (temp[23] << 8) | (temp[24]));

	/* LR */
	stacked_lr = (uint32_t)((temp[25] << 24) | (temp[26] << 16) | (temp[27] << 8) | (temp[28]));

	/* PC */
	stacked_pc = (uint32_t)((temp[29] << 24) | (temp[30] << 16) | (temp[31] << 8) | (temp[32]));

	/* PSR */
	stacked_psr = (uint32_t)((temp[33] << 24) | (temp[34] << 16) | (temp[35] << 8) | (temp[36]));

	/* BFAR */
	stacked_bfar = (uint32_t)((temp[37] << 24) | (temp[38] << 16) | (temp[39] << 8) | (temp[40]));

	/* CFSR */
	stacked_cfsr = (uint32_t)((temp[41] << 24) | (temp[42] << 16) | (temp[43] << 8) | (temp[44]));

	/* HFSR */
	stacked_hfsr = (uint32_t)((temp[45] << 24) | (temp[46] << 16) | (temp[47] << 8) | (temp[48]));

	/* DFSR */
	stacked_dfsr = (uint32_t)((temp[49] << 24) | (temp[50] << 16) | (temp[51] << 8) | (temp[52]));

	/* AFSR */
	stacked_afsr = (uint32_t)((temp[53] << 24) | (temp[54] << 16) | (temp[55] << 8) | (temp[56]));

	printf ("[Hard fault handler]\r\n");
	printf("Hard fault occured Time(MM/DD/HH/MM/SS): %d:%d:%d:%d:%d\r\n",month,day,Hours,RTC_Minutes,RTC_Seconds);

	printf ("R0 = %0.8x\r\n", stacked_r0);
	printf ("R1 = %0.8x\r\n", stacked_r1);
	printf ("R2 = %0.8x\r\n", stacked_r2);
	printf ("R3 = %0.8x\r\n", stacked_r3);
	printf ("R12 = %0.8x\r\n", stacked_r12);
	printf ("LR = %0.8x\r\n", stacked_lr);
	printf ("PC = %0.8x\r\n", stacked_pc);
	printf ("PSR = %0.8x\r\n", stacked_psr);
	printf ("BFAR = %0.8x\r\n", stacked_bfar);
	printf ("CFSR = %0.8x\r\n", stacked_cfsr);
	printf ("HFSR = %0.8x\r\n", stacked_hfsr);
	printf ("DFSR = %0.8x\r\n", stacked_dfsr);
	printf ("AFSR = %0.8x\r\n", stacked_afsr);
}

void AppModifySN(void)
{
	uint32_t u32SeriNum=0;
	SysConfigInfo_t  DeviceCFInfo;       //Device configure information

	/* 写SN */
	FLASH_If_Init();
	if((FLASH_OB_GetWRP() & 0x0000000F))
	{
		DisableBootloaderWriteProtection();

		FLASH_OB_Launch();

		SoftReset();
	}
	else
	{
		/* 	Read device config file from DataMemory	*/
        GetSysConfigInfo(&DeviceCFInfo);

		u8AppModifySeriNum[0] = DeviceCFInfo.AppModifySNInfo[0];
	    u8AppModifySeriNum[1] = DeviceCFInfo.AppModifySNInfo[1];
	    u8AppModifySeriNum[2] = DeviceCFInfo.AppModifySNInfo[2];
	    u8AppModifySeriNum[3] = DeviceCFInfo.AppModifySNInfo[3];

		/* Unlock the Option Bytes */
		FLASH_OB_Unlock();
		FLASH_ErasePage(SeriNum_ADDRESS);
		u32SeriNum = Translateu8ArrayTou32(u8AppModifySeriNum);
		FLASH_ProgramHalfPage(SeriNum_ADDRESS, &u32SeriNum);

		DeviceCFInfo.AppModifySNStatus = M95M01_APP_MODIFIEDSN;

		/* 	Write Modify SN status into DataMemeoy */
        SetSysConfigInfo(DeviceCFInfo);

		/* Enables the Write Protection of Bootloader */
		EnableBootloaderWriteProtection();

		/* Launch the option byte loading */
		FLASH_OB_Launch();

		/* SoftReset */
		SoftReset();
	}
	/*写SN结束 */
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

///*******************************************************************************
//* Function Name  : GetTick
//* Description    : Get system tick
//* Input          : int YY,int MM,int DD,int HH,int MMin,int SS
//*                  eg: 2015-6-30 16:32:30
//*              		   GetTick(2015,6,30,16,32,30);
//* Output         : tick
//* Return         : None
//*******************************************************************************/
//long GetTick(int YY,int MM,int DD,int HH,int MMin,int SS)
//{
//    struct tm stm;

//    memset(&stm,0,sizeof(stm));

//    stm.tm_year=YY-1900;
//    stm.tm_mon=MM-1;
//    stm.tm_mday=DD;
//    stm.tm_hour=HH;
//    stm.tm_min=MMin;
//    stm.tm_sec=SS;

//    return mktime(&stm);
//}

//void OutputGMTIME(long tim)
//{
//	time_t t=0;
//	struct tm *p;
//	char s[100]={0};

//	t = tim;
//	p = localtime(&t);

//	//YYYY/MM/DD,HH:MM:SS
//	strftime(s, sizeof(s), "%Y/%m/%d %H:%M:%S,", p);
//	#ifdef BLEPRINTGetSleepApneaInfo
//		BLEprintf("%s",s);
//	#else
//		printf("%s",s);
//	#endif
//}

uint32_t CovernDateto32(void)
{
	uint32_t date=0;

	date_str_typedef    date_s;               //RTC 日期
	RTC_TimeTypeDef     rtc_time;             //RTC 时间

    Calendar_Get(&date_s,&rtc_time); //获取当前时间

	date = (uint32_t)((date_s.year << 16) | (date_s.month << 8) | (date_s.day));

	return date;
}

//void Covern32toDate(date_str_typedef* date_s, uint32_t date)
//{
//	date_s->year = (uint16_t)(date >> 16);
//	date_s->month = (uint8_t)(date >> 8);
//	date_s->day = (uint8_t)(date);
//}

//uint16_t  CovernTimeto16(void)
//{
//	uint16_t  TimeCnt=0;

//	date_str_typedef    date_s;               //RTC 日期
//	RTC_TimeTypeDef     rtc_time;             //RTC 时间

//	Calendar_Get(&date_s,&rtc_time);   //获取当前时间

//	TimeCnt = (uint16_t)((rtc_time.RTC_Hours*3600) + (rtc_time.RTC_Minutes*60) + (rtc_time.RTC_Seconds));

//	return TimeCnt;
//}
/*******************************************************************************
* @brief   Translates 32-bit to 8-bit array.
* @param   u32Temp:the src data to be translated
* @param   pu8Array:the dest data addr
* @retval  void
*******************************************************************************/
void Translateu32Tou8Array(uint32_t u32Temp,uint8_t *pu8Array)
{
    *pu8Array = u32Temp & 0x000000ff;
    *(pu8Array + 1) = (u32Temp >> 8) & 0x000000ff;
    *(pu8Array + 2) = (u32Temp >> 16) & 0x000000ff;
    *(pu8Array + 3) = (u32Temp >> 24) & 0x000000ff;
}

/*******************************************************************************
* @brief   Translates 32-bit to 8-bit array.
* @param   u32Temp:the src data to be translated
* @param   pu8Array:the dest data addr
* @retval  void
*******************************************************************************/
uint32_t Translateu8ArrayTou32(uint8_t *pu8Array)
{
    return (((uint32_t)(*pu8Array)) + (((uint32_t)(*(pu8Array + 1))) << 8) +
            ((uint32_t)(*(pu8Array + 2)) << 16) + ((uint32_t)(*(pu8Array + 3)) << 24));
}

#ifdef  USR_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void usr_assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    printf("Wrong parameters value: file %s on line %d\r\n", file, line);
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
