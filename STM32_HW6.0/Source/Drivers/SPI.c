#include "stm32l1xx.h"
#include "SPI.h"
#include <stdlib.h>
#include "Monitoer_Template.h"
#include "SyncData.h"
#include "common.h"

static void spi_data_send(uint8_t *pDatSend, uint8_t lenDat);
static void UnpPackAppModifySNInfo(uint8_t *p_BasicInfo,uint16_t Len);
static uint8_t SPI_ReadByte(void);
//static void DataUpdate(void);
static void SPISenddata(uint8_t ID);


uint8_t  RxData[RxDataSize]={0};
uint8_t  SPIRxDataInTx_Flag = 0;
uint8_t  FirmwareUpdateType = 0;
uint8_t  HR_SpO2_SendFlag=0;  // 0x00:none;0x01:HR;0x02:SpO2;0x03:HR&SpO2

void SPI_GPIO_Config(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;

    /* Enable SPI SCK, MOSI, MISO and NSS GPIO PB12 PB13 PB14 PB15 clocks */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_SysSPIGPIO,ENABLE);

    /* SPI pin mappings */
    GPIO_PinAFConfig(GPIO_SysSPI, GPIO_PinSourceSysSPI_MOSI, GPIO_AF_SysSPI);
    GPIO_PinAFConfig(GPIO_SysSPI, GPIO_PinSourceSysSPI_MISO, GPIO_AF_SysSPI);
    GPIO_PinAFConfig(GPIO_SysSPI, GPIO_PinSourceSysSPI_SCK, GPIO_AF_SysSPI);

    /*  SPI pin config */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;

    /* SPI SCK pin configuration */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_SysSCK;
    GPIO_Init(GPIO_SysSPI, &GPIO_InitStructure);

    /* SPI  MOSI pin configuration */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_SysMOSI;
    GPIO_Init(GPIO_SysSPI, &GPIO_InitStructure);

    /* SPI MISO pin configuration */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_SysMISO;
    GPIO_Init(GPIO_SysSPI, &GPIO_InitStructure);
}

/*******************************************************************************
* Function Name  : SPI_Configuration
* Description    : 系统SPI初始，用于与51822之间通讯及读写EEPROM
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_Configuration(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;
    SPI_InitTypeDef     SPI_InitStructure;

    /**************SPI config *****************************************/
    /* Enable the SPI peripheral */
    RCC_APB1PeriphClockCmd(RCC_APBPeriph_SysSPI, ENABLE);

    /* Enable SPI SCK, MOSI, MISO and NSS GPIO PB12 PB13 PB14 PB15 clocks */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_SysSPIGPIO,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_SysSPINSSGPIO,ENABLE);


    /* SPI pin mappings */
    GPIO_PinAFConfig(GPIO_SysSPI, GPIO_PinSourceSysSPI_MOSI, GPIO_AF_SysSPI);
    GPIO_PinAFConfig(GPIO_SysSPI, GPIO_PinSourceSysSPI_MISO, GPIO_AF_SysSPI);
    GPIO_PinAFConfig(GPIO_SysSPI, GPIO_PinSourceSysSPI_SCK, GPIO_AF_SysSPI);


    /*  SPI pin config */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;

    /* SPI SCK pin configuration */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_SysSCK;
    GPIO_Init(GPIO_SysSPI, &GPIO_InitStructure);

    /* SPI  MOSI pin configuration */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_SysMOSI;
    GPIO_Init(GPIO_SysSPI, &GPIO_InitStructure);

    /* SPI MISO pin configuration */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_SysMISO;
    GPIO_Init(GPIO_SysSPI, &GPIO_InitStructure);

	/* SPI NSS pin configuration */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;

    /* SPI SCK pin configuration */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_SysNSS;
    GPIO_Init(GPIO_SysSPINSS, &GPIO_InitStructure);
	GPIO_SetBits(GPIO_SysSPINSS,GPIO_Pin_SysNSS);


     /* SPI configuration -------------------------------------------------------*/
    SPI_I2S_DeInit(SysSPI);
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;   //（极性再确认下）
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; // (8M/8 =1MHz)
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;

    SPI_Init(SysSPI, &SPI_InitStructure);

    /* Enable the SPI peripheral */
    SPI_Cmd(SysSPI, DISABLE);
}
/*******************************************************************************
* Function Name  : SPI_DeConfiguration
* Description    : 系统SPI初始，用于与51822之间通讯及读写EEPROM
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//static void SPI_DeConfiguration(void)
//{
//	GPIO_InitTypeDef    GPIO_InitStructure;

//   /* SPI configuration -------------------------------------------------------*/
//    SPI_I2S_DeInit(SysSPI);
//
//    /*  SPI pin config */
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;

//    /* SPI SCK pin configuration */
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_SysSCK;
//    GPIO_Init(GPIO_SysSPI, &GPIO_InitStructure);

//    /* SPI  MOSI pin configuration */
//    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_SysMOSI;
//    GPIO_Init(GPIO_SysSPI, &GPIO_InitStructure);

//    /* SPI MISO pin configuration */
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_SysMISO;
//    GPIO_Init(GPIO_SysSPI, &GPIO_InitStructure);

////	/* SPI NSS pin configuration */
////    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
////    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
////    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
////    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_SysNSS;
////
////    GPIO_Init(GPIO_SysSPINSS, &GPIO_InitStructure);
//}
/*******************************************************************************
* Function Name  : SysCommunication_Init
* Description    : 系统SPI初始，用于与51822之间通讯及读写EEPROM
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void SysCommunication_Init(void)
//{
//    GPIO_InitTypeDef    GPIO_InitStructure;
//    SPI_InitTypeDef     SPI_InitStructure;

//    /**************SPI config *****************************************/
//    /* Enable the SPI peripheral */
//    RCC_APB1PeriphClockCmd(RCC_APBPeriph_SysSPI, ENABLE);

//    /* Enable SPI SCK, MOSI, MISO and NSS GPIO PB12 PB13 PB14 PB15 clocks */
//    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_SysSPIGPIO,ENABLE);
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_SysSPINSSGPIO,ENABLE);

//    /* SPI pin mappings */
//    GPIO_PinAFConfig(GPIO_SysSPI, GPIO_PinSourceSysSPI_MOSI, GPIO_AF_SysSPI);
//    GPIO_PinAFConfig(GPIO_SysSPI, GPIO_PinSourceSysSPI_MISO, GPIO_AF_SysSPI);
//    GPIO_PinAFConfig(GPIO_SysSPI, GPIO_PinSourceSysSPI_SCK, GPIO_AF_SysSPI);


//    /*  SPI pin config */
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;

//    /* SPI SCK pin configuration */
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_SysSCK;
//    GPIO_Init(GPIO_SysSPI, &GPIO_InitStructure);

//    /* SPI  MOSI pin configuration */
//    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_SysMOSI;
//    GPIO_Init(GPIO_SysSPI, &GPIO_InitStructure);

//    /* SPI MISO pin configuration */
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_SysMISO;
//    GPIO_Init(GPIO_SysSPI, &GPIO_InitStructure);

//	/* SPI NSS pin configuration */
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_SysNSS;
//
//    GPIO_Init(GPIO_SysSPINSS, &GPIO_InitStructure);
//
//	GPIO_SetBits(GPIO_SysSPINSS,GPIO_Pin_SysNSS);


//     /* SPI configuration -------------------------------------------------------*/
//    SPI_I2S_DeInit(SysSPI);
//    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
//    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
//    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
//    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
//    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;   //（极性再确认下）
//    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
//    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; // (8M/32 =250KHz)
//    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
//    SPI_InitStructure.SPI_CRCPolynomial = 7;

//    SPI_Init(SysSPI, &SPI_InitStructure);

//    /* Enable the SPI peripheral */
//    SPI_Cmd(SysSPI, DISABLE);
//}
/*******************************************************************************
* Function Name  : SysCommunication_DeInit
* Description    : 系统SPI初始，用于与51822之间通讯及读写EEPROM
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void SysCommunication_DeInit(void)
//{
//    GPIO_InitTypeDef    GPIO_InitStructure;

//   /* SPI configuration -------------------------------------------------------*/
//    SPI_I2S_DeInit(SysSPI);

//    /*  SPI pin config */
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;

//    /* SPI SCK pin configuration */
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_SysSCK;
//    GPIO_Init(GPIO_SysSPI, &GPIO_InitStructure);

//    /* SPI  MOSI pin configuration */
//    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_SysMOSI;
//    GPIO_Init(GPIO_SysSPI, &GPIO_InitStructure);

//    /* SPI MISO pin configuration */
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_SysMISO;
//    GPIO_Init(GPIO_SysSPI, &GPIO_InitStructure);


//	/* SPI NSS pin configuration */
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_SysNSS;
//
//    GPIO_Init(GPIO_SysSPINSS, &GPIO_InitStructure);
//}
//void SPI2_SendStrings(unsigned char  *str)
//{
//	unsigned char ch;
//	unsigned char i=0;
//
//	ch = str[0];
//    SPI_Cmd(SysSPI, ENABLE);
//	GPIO_ResetBits(GPIO_SysSPINSS,GPIO_Pin_SysNSS);
//	//SysSPIBusy_Set;

//	while(ch != '\0')
//    {
//		ch = str[i++];
//		SPI_I2S_SendData(SysSPI,ch);
//		while (SPI_I2S_GetFlagStatus(SysSPI, SPI_I2S_FLAG_TXE) == RESET);
//    }
//	SPI_Cmd(SysSPI, DISABLE);
//    GPIO_SetBits(GPIO_SysSPINSS,GPIO_Pin_SysNSS);
//	//SysSPIBusy_Reset;

//}
static uint8_t SPI_ReadByte(void)
{
		uint8_t data=0;

		SPI_I2S_SendData(SysSPI, 0xAC);

		SPI_Status_Wait(SysSPI, SPI_I2S_FLAG_RXNE, SET);

		data = SPI_I2S_ReceiveData(SysSPI);

		SPI_Status_Wait(SysSPI, SPI_I2S_FLAG_TXE, SET);

        return data;
}

/*******************************************************************************
* Function Name  : SPITranslate_Task_Handler
* Description    : SPITranslate_Task_Handler,处理SPITranslate任务
* Input          : SPITranslate_Event
* Output         : None
* Return         : None
*******************************************************************************/
void SPITranslate_Task_Handler(event_t SPITranslate_Event)
{
	uint8_t SPI_RX_Buf[SPI_RX_Buf_MaxLen]={0};
	uint8_t SPI_RX_Buf_Len=0;
	uint8_t i=0;
	uint8_t FrameHeader1=0x00;  //帧头第一个字节
	uint8_t FrameHeader2=0x00;  //帧头第二个字节
	uint8_t SPI_RX_Buf_LenH=0; //帧数据长度高8位
	uint8_t SPI_RX_Buf_LenL=0; //帧数据长度低8位
	uint8_t FrameTyp=0;    //帧类型
	uint16_t CheckUpParamID=0; //采集哪些参数控制字
//	uint8_t CheckUpParemResultNum=0;  //采集参数获取结果数
	uint8_t SyncDateSwitch = 0;//触发数据同步
	uint16_t MonitorTemplateLen=0; //接收监测模板数据长度+数据帧头(3)+数据长度(2)
	//uint8_t p_MonitorTemplate[MaxMTDataLen]={0};    //保存接收的监测模板数据
	uint8_t * p_MonitorTemplate = NULL;    //保存接收的监测模板数据
	uint16_t ParamrterLimitLen=0;   //接收参数门限设置数据长度+数据帧头(3)+数据长度(2)
	//uint8_t p_ParameterLimit[ParamrterLimitMaxLen]={0};     //保存参数门限设置数据
	uint8_t * p_ParameterLimit = NULL;
	uint16_t EEPROMDataLen=0;      //读取EEPROM长度
	uint16_t  BasicInfoDataLen=0;     //基本用户信息数据长度
	//uint8_t  p_BasicInfo[BasicInfoDataMaxLen] ={0}; //保存基本用户信息数据
	uint8_t * p_BasicInfo = NULL;
	uint8_t  SPIBusyTry_Cnt=0;    //SPI数据传输Busy时，try计数
    SysConfigInfo_t DeviceCFInfo;  //Device configure information

	time_str_typedef     m_rtc_new_time;
	date_str_typedef     m_rtc_date;
	week_str_typedef     m_rtc_week;


	if(SPITranslate_Event & gSPITranslateEventRx)  //读取SPI数据事件
	{
			if(SPIRxDataInTx_Flag == 0)
			{
				SPI_GPIO_Config();

				/* Sys Communication Init */
				SPI_Cmd(SysSPI, DISABLE);
				SPIBusyTry_Cnt = 0;
				do
				{
					SPI_Cmd(SysSPI, ENABLE);
					SPI_I2S_ReceiveData(SysSPI);
					GPIO_ResetBits(GPIO_SysSPINSS,GPIO_Pin_SysNSS);
					Delay_us(10);   //必须加，51822从低功耗模式SPI唤醒时需要时间恢复时钟

					FrameHeader1 = SPI_ReadByte();       //读取数据帧头1
					if(FrameHeader1 == 0xAB)
					{
						SPI_Cmd(SysSPI, DISABLE);
						Delay_us(10);
						GPIO_SetBits(GPIO_SysSPINSS,GPIO_Pin_SysNSS);
						Delay_us(10);
						Delay_ms(2);

						#ifdef SPI_nRF_Debug
						printf("Read Try Again\r\n");
						#endif
					}
					#ifdef SPI_nRF_Debug
					else
					{
						printf("Read Normal\r\n");
					}
					#endif
					SPIBusyTry_Cnt++;
				}while((FrameHeader1 == 0xAB) && (SPIBusyTry_Cnt < 20));

				FrameHeader2 = SPI_ReadByte();       //读取数据帧头2

				if((FrameHeader1 == 0xAA) && (FrameHeader2 == 0x55))
				{
					FrameHeader1 = 0x00;
					FrameHeader2 = 0x00;
					FrameTyp     = SPI_ReadByte();       //读取数据帧类型
					SPI_RX_Buf_LenH = SPI_ReadByte();    //读取数据帧长度高8位
					SPI_RX_Buf_LenL = SPI_ReadByte();    //读取数据帧长度低8位

					SPI_RX_Buf_Len = (SPI_RX_Buf_LenH<<8) + SPI_RX_Buf_LenL;   //获取数据帧长度
					if(SPI_RX_Buf_Len >= SPI_RX_Buf_MaxLen)
					{
						#ifdef Memory_Manage_DEBUG
							printf("Memory Allocate Err,file=%s,line=%d\r\n",__FILE__,__LINE__);
						#endif
						return;
					}

					for(i=0;i<SPI_RX_Buf_Len;i++)
					{
						SPI_RX_Buf[i]=SPI_ReadByte();    //读取nRF51822 SPI发送的数据
					}
					SPI_Cmd(SysSPI, DISABLE);
					Delay_us(10);
					GPIO_SetBits(GPIO_SysSPINSS,GPIO_Pin_SysNSS);
					Delay_us(10);

					/* Sys Communication DeInit */
					//SysCommunication_DeInit();

					#ifdef   nRF51822_SPIRXResource
					printf("Receice nRF51822 data,%d...\r\n",SPI_RX_Buf_Len);
					printf("Type=0x%x\r\n",FrameTyp);
					for(i=0;i<SPI_RX_Buf_Len;i++)
					{
						printf("0x%x, ",SPI_RX_Buf[i]);
					}
					printf("\r\n");
					#endif
				}
				else  //帧头不对
				{
					#ifdef SPI_nRF_Debug
					printf("\r\nH1=0x%x",FrameHeader1);
					printf("\r\nH2=0x%x",FrameHeader2);
					#endif
					SPI_Cmd(SysSPI, DISABLE);
					Delay_us(10);
					GPIO_SetBits(GPIO_SysSPINSS,GPIO_Pin_SysNSS);
					Delay_us(10);
				}
			}
			else
			{
				#ifdef SPI_nRF_Debug
				printf("Read Tx Data\r\n");
				#endif
				/*
				Frame format:
							[0]	-- N/A
							[1]	-- N/A
							[2]	-- Frame ID
							[3]	-- data length MSB
							[4]	-- data length LSB
							[..]-- pay load

				*/
				FrameTyp = RxData[2];
				SPI_RX_Buf_LenH = RxData[3];
				SPI_RX_Buf_LenL = RxData[4];

				SPI_RX_Buf_Len = (SPI_RX_Buf_LenH<<8) + SPI_RX_Buf_LenL;   //获取数据帧长度
				if(SPI_RX_Buf_Len >= SPI_RX_Buf_MaxLen)
				{
					#ifdef Memory_Manage_DEBUG
						printf("Memory Allocate Err,file=%s,line=%d\r\n",__FILE__,__LINE__);
					#endif
					return;
				}

				for(i=0;i<SPI_RX_Buf_Len;i++)
				{
					SPI_RX_Buf[i]=RxData[5+i];    //读取nRF51822 SPI发送的数据
				}

				memset(RxData,0,RxDataSize*sizeof(uint8_t));
				SPIRxDataInTx_Flag = 0;
			}

			/**		Process Body Based on Each Frame ID 		**/
			switch(FrameTyp)
			{
				/* 	Frame: Connection Indicator		*/
				case Frame_ConnectState:
					if(SPI_RX_Buf[0] == 0x00)				/* 		Connection not established 		*/
					{
						Device_State = Device_Alone;   		/* 	No connection to APP	*/
						g_PassKey.Connect_Status = 0;
						#ifdef   nRF51822_SPIRX
							printf("FRAME DISCONNECTED\r\n");
						#endif

						/*		Stop All Measurement		*/
						if(gSubFunc_Stat_Get(SpO2_RealTimeSample_State | HR_RealTimeSample_State) != OFF)
						{
						#ifdef   nRF51822_SPIRX
							printf("Stop Check up HRSpO2\r\n");
						#endif
							/* 	Stop SPO2/HR sample 	*/
							gSubFunc_Stat_Set(SpO2_RealTimeSample_State | HR_RealTimeSample_State,  OFF);
                            if((MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID) && (true == isFreeRunKickOff()))
                            {
                                //to be defined
                            }
                            else
                            {
                                TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop);
                            }
							OLEDDisplay_Stat_Set(NormalDis);		/* 	Force display mode back Normal Mode 	*/
						}

						HR_SpO2_SendFlag = 0x00;					/* 	Don't send HR/SPO2 value to APP			*/

						/* 	Back to RTC GUI if needed 	*/
						if((Device_Mode != Device_Mode_Factory) && (Device_Mode != Device_Mode_Charge) && (Device_Mode != Device_Mode_LowBattery))
						{
							if((Device_Mode == Device_Mode_PassKey)|| (Device_Mode == Device_Mode_CheckUp)
								||((Device_Mode == Device_Mode_RTC) && (Get_OLED_Dis_Status() != OLEDDisShutDown)))
							{
								#ifdef   nRF51822_SPIRX
									printf("show RTC when disconnected\r\n");
								#endif
                                /* 	Detect the charge status  */
                                if(GPIO_ReadInputDataBit(GPIO_BAT_PG,EXTI_LineBAT_PG) == Bit_RESET)
                                {
                                    /* Send the event to BatManage task */
                                    TS_SendEvent(gTsBatManageTaskID_c,gBATEnterChargeEvent);
                                }
                                else
                                {
                                    Device_Mode = Device_Mode_RTC;
                                    TS_SendEvent(gOledDisTaskID,gOledDisEventBlueIcon_c);
                                }
							}
						}
					}
					else										/*	BLE Connection established		*/
					{
						#ifdef   nRF51822_SPIRX
							printf("FRAME CONNECTED\r\n");
						#endif
						/* 	DISABLE "Show RTC if current mode is RTC mode" 	*/
//						if(Device_Mode == Device_Mode_RTC)
//						{
//							#ifdef   nRF51822_SPIRX
//								printf("show RTC when connected\r\n");
//							#endif
//							TS_SendEvent(gOledDisTaskID,gOledDisEventBlueIcon_c);
//						}

						#ifdef   nRF51822_SPIRX
							printf("\r\nApp Connected\r\n");
						#endif
					}
					TotleDataLen=0;
		            CurDataLen=0;
					RemainDataLen=0;
					SPISendedDataLen=0;
					SyncACKCnt=0;     //数据同步应答次数
					gSyncACK = TRUE;
					break;

				/* 	Frame: Real Time Sample Control Frame 	*/
				case Frame_CheckUpParam:
				/*
				Frame format:
							[0]	-- ID MSB, reserved for future
							[1]	-- ID LSB
							[2]	-- Number of sample, 0--continous sample, else -- sample required number of data

				*/
					if((MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID) && (true == isFreeRunKickOff()))
					{
                        /* Notice App, FreeRun Monitor Template is running */
                        /* Reuse the alarm channel */
						TS_SendEvent(gTsAlarmTaskID_c,gAlarmEventFreeRunMTRunning);
						return;
					}
					CheckUpParamID = (((uint16_t)SPI_RX_Buf[1]) << 8u) | (0x00ff & SPI_RX_Buf[1]);
					#ifdef App1_0
//					CheckUpParemResultNum = 0;
					#endif
					#ifdef App2_0
//					CheckUpParemResultNum = SPI_RX_Buf[2];	/* 	Get the sample number 	*/
					#endif
					if(Device_State == Device_WorkWithAPP)
					{
						/* 	HR sample 	*/
						if(CheckUpParamID & SAMPLE_ID_HeartRate)
						{
							#ifdef   nRF51822_SPIRX
								printf("Check up Heart Rate\r\n");
							#endif

							/* 	Not response while in low battery mode and charge mode 		*/
							if((Device_Mode == Device_Mode_LowBattery) || (Device_Mode == Device_Mode_Charge))
							{
								return;
							}
//							HR_RealTimeSample_GetResultNum = CheckUpParemResultNum;
							gSubFunc_Stat_Set(HR_RealTimeSample_State,ON);
							/* 开启HR/SpO2测量前，将佩戴检测功能设置为测量模式 */
							//Wear_Detect_Set(WEAR_DETECT_INC);
							TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStart);     //发送HR开启测量事件
							HR_SpO2_SendFlag |= SAMPLE_ID_HeartRate;
						}
						/* 	Sample SPO2		*/
						if(CheckUpParamID & SAMPLE_ID_SpO2)
						{
							#ifdef   nRF51822_SPIRX
								printf("Check up SpO2\r\n");
							#endif

							/* 	Not response while in low battery mode and charge mode 		*/
							if((Device_Mode == Device_Mode_LowBattery) || (Device_Mode == Device_Mode_Charge))
							{
								return;
							}
//							SpO2_RealTimeSample_GetResultNum = CheckUpParemResultNum;
							gSubFunc_Stat_Set(SpO2_RealTimeSample_State, ON);
							/* 开启HR/SpO2测量前，将佩戴检测功能设置为测量模式 */
							//Wear_Detect_Set(WEAR_DETECT_INC);
							TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStart);
							HR_SpO2_SendFlag |= SAMPLE_ID_SpO2;
						}
						/* 	Stop SpO2/hr/ntc sample 	*/
						if(CheckUpParamID == CheckUpStop)
						{
							//Stop All Measurement
							#ifdef   nRF51822_SPIRX
								printf("Stop Check up ALL sample\r\n");
							#endif

							/* 	Use these parameters to determine how single mode works 		*/

							gSubFunc_Stat_Set(HR_RealTimeSample_State | SpO2_RealTimeSample_State,OFF);
							TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop);
							HR_SpO2_SendFlag = 0x00;
							OLEDDisplay_Stat_Set(NormalDis);  /* 	Force display mode back Normal Mode 	*/
							if((Device_Mode != Device_Mode_LowBattery) && (Device_Mode != Device_Mode_Charge) && (Device_Mode == Device_Mode_CheckUp))
							{
								/* 	Back to RTC GUI 	*/
								Device_Mode = Device_Mode_RTC;
//								flagIsHRSpO2FixNumberSampleKeyPress = TRUE; 	/* 	To avoid key press twice for SPO2 single result test 	*/
								if(Get_OLED_Dis_Status() != OLEDDisShutDown)
								{
									TS_SendEvent(gOledDisTaskID,gOledDisEventBlueIcon_c);
								}
							}
						}
						else 		/* 	Handle the process between key-init sample and APP-init sample 	*/
						{
							Device_Mode = Device_Mode_CheckUp;  				//及时采集模式
							gSubFunc_Stat_Set(SpO2_SingleWork_State,OFF);        //及时采集时，停止单设备采集SpO2数据
							gSubFunc_Stat_Set(HR_SingleWork_State,OFF);          //及时采集时，停止单设备采集HR数据

							if(SPO2_HR_Measure_State == Start)
							{
								SPO2_HR_Measure_State = Stop;
							}
							/* 	if in second tick mode  	*/
							if(Device_Mode == Device_Mode_SecTick)
							{
								TS_SendEvent(gTsSecTickTaskID_c,gSecTikEventBackground);		/* 	Into back ground 		*/
							}

							/*显示App控制及时采集中界面*/
							OLEDDisplay_Stat_Set(NormalDis);  //App开始及时采集时显示Normal模式
							TS_SendEvent(gOledDisTaskID,gOledDisEventAppCheckUp_c);   //显示App及时采集中界面

						}
					}//end of if(Device_State == Device_WorkWithAPP)
					else  //Device Alone
					{
							//Stop All Measurement
							#ifdef   nRF51822_SPIRX
							printf("Stop Check up\r\n");
							#endif

							gSubFunc_Stat_Set(SpO2_RealTimeSample_State,OFF);
							gSubFunc_Stat_Set(HR_RealTimeSample_State,OFF);

							HR_SpO2_SendFlag = 0x00;

							if((MonitorTemplate.MTID != kICTMonitorTemplateFreeRunID) || (true != isFreeRunKickOff()))
							{
								TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop);
							}

					}
					break;

				/* 	Frame: Trig data sync 	*/
				case Frame_SyncDateSwitch:
				/*
				Frame format:
							[0]	-- Sync Command
				*/
					/* 	Not response while in low battery and charge 		*/
					if((Device_Mode == Device_Mode_LowBattery) || (Device_Mode == Device_Mode_Factory))
					{
						return;
					}

					/*  Not response while in free run */
					if((MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID) && (true == isFreeRunKickOff()))
					{
						return;
					}
					#ifdef   nRF51822_SPIRX
						printf("FRAME SYNC DATA\r\n");
					#else
						Delay_ms(2);
					#endif
					SyncDateSwitch = SPI_RX_Buf[0];
					/* 	start sync data 	*/
					if(SyncDateSwitch == Start_DateUpdate)
					{
						/* Sys Communication Init */
						//SysCommunication_Init();
						//DataUpdate();    //同步数据
						#ifdef SyncData_DEBUG
							printf("Start DataUpdate\r\n");
						#endif
                        gSyncACK = TRUE;
						TS_SendEvent(gTsSyncDataTaskID_c,gSyncDataFormat2EventStart);
					}
					/* 	data sync completed 	*/
					else if(SyncDateSwitch == Complate_DataUpdate)
					{
						//完成数据同步
						/* Sys Communication DeInit */
						//SysCommunication_DeInit();
						EEPROMDataLen = (((uint16_t)SPI_RX_Buf[1]) << 8u) | (0x00ff & SPI_RX_Buf[2]); /* 	Data sync length, Max = 256 	*/
						#ifdef SyncData_DEBUG
						printf("Synced data len:%d\r\n",EEPROMDataLen);
						#endif

						if(EEPROMDataLen == SyncDataLen)
						{
                            EEPROMDataLen = EEPROMDataLen - 2;  //2字节 CRC
                            Flag_RxedSyncDataLength = true;
                            RxedSyncDataLength = EEPROMDataLen;
                            if((Flag_RxedSyncDataLength == true) && (Flag_SyncDataDone == true))
                            {

                                DataMemoryReadACK(RxedSyncDataLength);

                                gSyncACK = TRUE;   //数据已同步
                                SyncDataLen = 0;
                                Flag_SyncDataDone = false;
                                Flag_RxedSyncDataLength = false;
                                RxedSyncDataLength = 0;
                            }
						}

						TS_SendEvent(gTsSyncDataTaskID_c,gSyncDataFormat2EventStart);  //开启下一段数据同步
					}
					/* 	data sync stop 	*/
					else if(SyncDateSwitch == Stop_DateUpdate)
					{
						#ifdef SyncData_DEBUG
							printf("Stop DataUpdate\r\n");
						#endif
						/* Sys Communication DeInit */
						//SysCommunication_DeInit();
					}
					break;

				/* 	Frame: Time sync 	*/
				case Frame_TimeSyn:
				/*
				Frame format:
							[0]	-- Year, MSB
							[1]	-- Year, LSB
							[2]	-- Month
							[3] -- Day
							[4] -- Hour
							[5] -- Minutes
							[6] -- Seconds

				*/

					#ifdef   nRF51822_SPIRX
						printf("FRAME SYNC TIME\r\n");
					#endif
					m_rtc_date.month=SPI_RX_Buf[2];
					m_rtc_date.day = SPI_RX_Buf[3];
					m_rtc_date.year = (SPI_RX_Buf[0]*100 + SPI_RX_Buf[1]);
					Calendar_WeekDayNum(&m_rtc_date, &m_rtc_week);

					m_rtc_new_time.hours = SPI_RX_Buf[4];
					m_rtc_new_time.minutes = SPI_RX_Buf[5];
					m_rtc_new_time.seconds = SPI_RX_Buf[6];
					Calendar_TimeSet(&m_rtc_new_time);
					Calendar_DateSet(&m_rtc_date);

					Device_State = Device_WorkWithAPP; 		/* set to work with APP 	*/

					/* 	Not response while in low battery and charge 		*/
					if(Device_Mode == Device_Mode_LowBattery)
					{
						return;
					}

					if(Device_Mode == Device_Mode_PassKey)
					{
                        /* 	Detect the charge status  */
                        if(GPIO_ReadInputDataBit(GPIO_BAT_PG,EXTI_LineBAT_PG) == Bit_RESET)
                        {
                            /* Send the event to BatManage task */
                            TS_SendEvent(gTsBatManageTaskID_c,gBATEnterChargeEvent);
                        }
                        else
                        {
                            Device_Mode = Device_Mode_RTC;          // set to RTC Mode and display RTC when time synced
                        }
					}
					if(Device_Mode == Device_Mode_RTC)
					{
						#ifdef   nRF51822_SPIRX
							printf("show RTC when time sync\r\n");
						#endif
						Set_OLED_Dis_Status(OLEDDisON);
						TS_SendEvent(gOledDisTaskID,gOledDisEventBlueIcon_c);
					}
					break;

				/* 	Frame: monitor template frame 	*/
				case Frame_MoniterTemplateSet:
				/*
				Frame format:
							[0]	-- data length, MSB
							[1]	-- data length, LSB
							[..]-- data payload

				*/
					#ifdef   nRF51822_SPIRX
						printf("FRAME MONI TEMP\r\n");
					#endif

						/* 	Not response while in low battery and charge 		*/
						if((Device_Mode == Device_Mode_LowBattery) || (Device_Mode == Device_Mode_FWUpdate))
						{
							return;
						}
						/* 	Get data length 		*/
						MonitorTemplateLen = (((uint16_t)SPI_RX_Buf[0]) << 8u) | (0x00ff & SPI_RX_Buf[1]);

						/* 	Process only when length is valid		*/
						if(MonitorTemplateLen < MaxMTDataLen)
						{
							p_MonitorTemplate = SPI_RX_Buf + 2;
							if((p_MonitorTemplate[0] == '#') && (p_MonitorTemplate[MonitorTemplateLen-1] == '#'))
							{
								UnPackMonitorTemplate(p_MonitorTemplate,MonitorTemplateLen);
							}
						}
					break;

				/* 	Frame: Set Parameter Threshold 		*/
				case  Frame_Limit:

					/*
					Frame format:
								[0]	-- Data Length MSB
								[1]	-- Data Length LSB
								[2]	-- Data ID
								[..]-- Data Payload

					*/

					/* 	Not response while in low battery and charge 		*/
					if((Device_Mode == Device_Mode_LowBattery) || (Device_Mode == Device_Mode_Charge))
					{
						return;
					}

					#ifdef   nRF51822_SPIRX
						printf("\r\nSet Parameter Limit...\r\n");
					#endif
					/* 	Get the data length 	*/
					ParamrterLimitLen = (((uint16_t)SPI_RX_Buf[0]) << 8u) | (0x00ff & SPI_RX_Buf[1]);

					#ifdef  Parameter_Limit_Debug
						if(ParamrterLimitLen > 19)
						{
							printf("Error: parameter limit length....\r\n");
						}
					#endif
					if(ParamrterLimitLen <= ParamrterLimitMaxLen)
					{
						/* 		Store the data		 	*/
						//for(i=0;i<ParamrterLimitLen;i++)
						//{
						//	p_ParameterLimit[i]=SPI_RX_Buf[i+2];
						//}
						p_ParameterLimit = SPI_RX_Buf + 2;
						UnPackParameterLimit(p_ParameterLimit,ParamrterLimitLen);
					}

					break;
				/* 	Frame: Require device information 		*/
				case Frame_DeviceInfo_Req:
					#ifdef   nRF51822_SPIRX
						printf("FRAME Device information require \r\n");
					#else
						Delay_ms(2);		/* 	Keep this delay to send data to NRF51822 		*/
					#endif

					SPI_DeviceInformationTransmit();

					/* 		Connection not established 		*/
					Device_State = Device_Alone;   			/* 	Set to device alone mode 	*/

					#ifdef   nRF51822_SPIRX
						printf("FRAME DISCONNECTED\r\n");
					#endif

					/* 	Stop All Measurement	*/
					if(gSubFunc_Stat_Get(SpO2_RealTimeSample_State | HR_RealTimeSample_State) != OFF)
					{
					#ifdef   nRF51822_SPIRX
						printf("Stop Check up HRSpO2\r\n");
					#endif
						gSubFunc_Stat_Set(SpO2_RealTimeSample_State | HR_RealTimeSample_State,  OFF);
																//App断线后停止及时采集SpO2数据
																//App断线后停止及时采集HR数据
						TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop);
						OLEDDisplay_Stat_Set(NormalDis);  //App断开连接时显示Normal模式
					}

					HR_SpO2_SendFlag = 0x00;             //App断线后停止发送及时采集SpO2/HR数据

					if((Device_Mode != Device_Mode_Factory) && (Device_Mode != Device_Mode_Charge) && (Device_Mode != Device_Mode_LowBattery))
					{
						if((Device_Mode == Device_Mode_PassKey)|| (Device_Mode == Device_Mode_CheckUp)
							||((Device_Mode == Device_Mode_RTC) && (Get_OLED_Dis_Status() != OLEDDisShutDown)))
						{
							#ifdef   nRF51822_SPIRX
								printf("show RTC when disconnected\r\n");
							#endif
							Device_Mode = Device_Mode_RTC;
							TS_SendEvent(gOledDisTaskID,gOledDisEventBlueIcon_c);   //显示RTC模式
						}
					}
					TotleDataLen=0;
		            CurDataLen=0;
					RemainDataLen=0;
					SPISendedDataLen=0;
					SyncACKCnt=0;     //数据同步应答次数
					gSyncACK = TRUE;
					break;

				/* 	Frame: passwork verify code 	*/
				case Frame_PassKey:
					#ifdef   nRF51822_SPIRX
						printf("FRAME PASKEY VERF \r\n");
					#endif

					g_PassKey.Status = 1;   //是否收到验证密码：1：收到，0：没有收到
					g_PassKey.Connect_Status  = 1; //收到密码时蓝牙连接状态：1：连接，0：断开
					g_PassKey.ConfirmStatus = SPI_RX_Buf[0];  //密码验证状态:1:通过，0：失败
					g_PassKey.len = 4;     //密码长度

					#ifdef PassKey_DEBUG
						printf("Pass Key Confirm\r\n");
						printf("Confirm Status=0x%x,\r\n",g_PassKey.ConfirmStatus);
						printf("Pass Key Len=%d\r\n",g_PassKey.len);
					#endif

					for(i=0;i<g_PassKey.len;i++)
					{
						g_PassKey.p_contex[i]=SPI_RX_Buf[i+2]-0x30; //保存密码
						#ifdef PassKey_DEBUG
							printf("0x%x, ",g_PassKey.p_contex[i]);
						#endif
					}

					/* 	Not to do following actions while in low battery and charge 		*/
					if(Device_Mode == Device_Mode_LowBattery)
					{
						return;
					}

					if(Device_Mode != Device_Mode_Factory)
					{
						if(g_PassKey.ConfirmStatus == 0x00)  //密码验证失败，显示随机生成的密码
						{
							#ifdef PassKey_DEBUG
								printf("Display Passkey\r\n");
							#endif

							/* 	if in second tick mode  	*/
							if(Device_Mode == Device_Mode_SecTick)
							{
								TS_SendEvent(gTsSecTickTaskID_c,gSecTikEventBackground);		/* 	Into back ground 		*/
							}
							Device_Mode = Device_Mode_PassKey;

							/* 	Stop all measure operation if there is 	*/
                            gSubFunc_Stat_Set(SUBFUNC_ALL_STATE, OFF);
                            if((MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID) && (true == isFreeRunKickOff()))
                            {
                                //to be defined
                            }
                            else
                            {
                                TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop);  	/*	stop SPO2 */

                            }

							TS_SendEvent(gOledDisTaskID,gOledDisEventPassKey_c);  //产生显示密码事件
						}
						else    //密码验证通过，显示RTC界面
						{
							#ifdef PassKey_DEBUG
								printf("Display RTC\r\n");
							#endif

							Device_State = Device_WorkWithAPP; 		/* set to work with APP 	*/

                            if((MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID) && (true == isFreeRunKickOff()))
                            {
                                /* 	if in second tick mode  	*/
                                if(Device_Mode == Device_Mode_SecTick)
                                {
                                    TS_SendEvent(gTsSecTickTaskID_c,gSecTikEventBackground);		/* 	Into back ground 		*/
                                }

                                if(Device_Mode == Device_Mode_PassKey)
                                {
                                    /* 	Detect the charge status  */
                                    if(GPIO_ReadInputDataBit(GPIO_BAT_PG,EXTI_LineBAT_PG) == Bit_RESET)
                                    {
                                        /* Send the event to BatManage task */
                                        TS_SendEvent(gTsBatManageTaskID_c,gBATEnterChargeEvent);
                                    }
                                    else
                                    {
                                        Device_Mode = Device_Mode_RTC;          // set to RTC Mode and display RTC when time synced
                                        #ifdef   nRF51822_SPIRX
                                            printf("show RTC when time Passkey\r\n");
                                        #endif
                                        Set_OLED_Dis_Status(OLEDDisON);
                                        TS_SendEvent(gOledDisTaskID,gOledDisEventBlueIcon_c);
                                    }
                                }
                            }
                            else
                            {
                                if(
                                    cc_alg_SpO2_Stat_Get() != SPO2_HR_STATUS_RUNNING
                                )
                                {
                                    /* 	if in second tick mode  	*/
                                    if(Device_Mode == Device_Mode_SecTick)
                                    {
                                        TS_SendEvent(gTsSecTickTaskID_c,gSecTikEventBackground);		/* 	Into back ground 		*/
                                    }

                                    if(Device_Mode == Device_Mode_PassKey)
                                    {
                                        /* 	Detect the charge status  */
                                        if(GPIO_ReadInputDataBit(GPIO_BAT_PG,EXTI_LineBAT_PG) == Bit_RESET)
                                        {
                                            /* Send the event to BatManage task */
                                            TS_SendEvent(gTsBatManageTaskID_c,gBATEnterChargeEvent);
                                        }
                                        else
                                        {
                                            Device_Mode = Device_Mode_RTC;          // set to RTC Mode and display RTC when time synced
                                        }
                                    }
                                }
                            }
						}
					}
					break;

					/* 	Frame: Firmware update data frame 	*/
					case Frame_FWUpdate:
						/* 	Not response while in low battery 		*/
                        if(Device_Mode != Device_Mode_FWUpdate)
						{
							return;
						}

						for(i=0;i<FIRMWARE_UPDATE_BUF_LEN;i++)  //保存固件数据128字节，+2字节（最后2字节)CRC
						{
							gFWData[i] = SPI_RX_Buf[i];
						}
                        TS_SendEvent(gTsFWUpdateTaskID_c,gFWUpdateRxFWDateEvent);  //产生接收固件数据事件
                        break;


					/* 	Frame: Firmware translate control Frame		*/
					case Frame_FWUpdateCtlB:
						if(PrepareRxFWData == (SPI_RX_Buf[0] & 0x0f))  // Prepare for receiving firmware
						{
                            #ifdef   FWUpdate_Debug
							printf("\r\nGet ready for data for FW Update...\r\n");
                            #endif
                            if(Device_Mode == Device_Mode_FWUpdate)
                            {
                                return ;
                            }
                            if(gSubFunc_Stat_Get(SUBFUNC_ALL_STATE) != OFF)
                            {
                                /* Stop all measure work		*/
                                gSubFunc_Stat_Set(SUBFUNC_ALL_STATE,OFF); 		/* Stop all measure work 		*/
                                TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop);
                            }
                            /* Stop second tick		*/
                            if(Get_Current_SecTick_Status() != SECTIK_IDLE)
                            {
                                TS_SendEvent(gTsSecTickTaskID_c, gSecTikEventStop);
                            }
                            /* Stop step		*/
                            TS_SendEvent(gTsStepTaskID_c,gStepStopEvent);
                            Device_Mode_pre = Device_Mode;
                            Device_Mode = Device_Mode_FWUpdate;
                            if((SPI_RX_Buf[0] & 0xf0) == Frame_TYPE_STM32APP)
                            {
                                FirmwareUpdateType = Frame_TYPE_STM32APP;
                            }
                            else if((SPI_RX_Buf[0] & 0xf0) == Frame_TYPE_STM32BOOT)
                            {
                                FirmwareUpdateType = Frame_TYPE_STM32BOOT;
                            }
                            else if((SPI_RX_Buf[0] & 0xf0) == Frame_TYPE_NRFBOOT)
                            {
                                FirmwareUpdateType = Frame_TYPE_NRFBOOT;
                            }
                            else if((SPI_RX_Buf[0] & 0xf0) == Frame_TYPE_NRFAPP)
                            {
                                FirmwareUpdateType = Frame_TYPE_NRFAPP;
                            }
                            else if((SPI_RX_Buf[0] & 0xf0) == Frame_TYPE_NRFSD)
                            {
                                FirmwareUpdateType = Frame_TYPE_NRFSD;
                            }
                            else
                            {}

                            if(FALSE == bFlagFWUpdateRunning)
                            {
                                bFlagFWUpdateRunning = TRUE;

                                u32FWTotelCrc_App = 0;
                                #ifdef   FWUpdate_Debug
                                printf("FirmwareUpdateType = 0x%x...\r\n",FirmwareUpdateType);
                                #endif
                                /* Ready for firmware update */
                                TS_SendEvent(gTsFWUpdateTaskID_c,gFWUpdatePrepareRxFWDateEvent);  //产生准备接收固件数据事件
                            }
						}
						else if(StartUpDateFW == (SPI_RX_Buf[0] & 0x0f)) // Switch to bootloader and run new firmware
						{
                            if(Device_Mode != Device_Mode_FWUpdate)
                            {
                                return;
                            }

                            u32FWTotelCrc_App = ((SPI_RX_Buf[1] << 8) |SPI_RX_Buf[2]);
                            u32FWTotelCnt = ((SPI_RX_Buf[3] << 24) | (SPI_RX_Buf[4] << 16) | (SPI_RX_Buf[5] << 8) | SPI_RX_Buf[6]);
                            TS_SendEvent(gTsFWUpdateTaskID_c,gFWUpdateStartFWDateEvent);  //开始升级固件软件事件
						}
                        else if(STM32ERROR == (SPI_RX_Buf[0] & 0x0f)) // Switch to bootloader and run new firmware
						{
                            OLED_DisplayClear();
                            OLED_DisplayFullScreenBMP(MXS8475_FirmwareError);
                            while(1); /* 	Dead Lock for Error writing 		*/
						}
                        else if(Nrf51822ERROR == (SPI_RX_Buf[0] & 0x0f)) // Switch to bootloader and run new firmware
						{
                            TS_SendEvent(gTsFWUpdateTaskID_c,gFWUpdateStartFWNRFErrorEvent);  //开始升级固件软件事件
						}
                        break;

					/* 	Frame: user basic information 	*/
					case  Frame_BasicInfo:
						#ifdef   nRF51822_SPIRX
							printf("\r\nBasic Info Frame...\r\n");
						#endif
							/*
							Frame format:
								[0] -- W/R command
								[1]	-- Data Length
								[2]	-- Data Length
								[..]-- Data Payload
							*/
						/* 	Basic info from APP 	*/
						BasicInfoDataLen = (((uint16_t)SPI_RX_Buf[1])<<8) | (0x00ff & SPI_RX_Buf[2]);   /* Get the frame length 	*/

						if(BasicInfoDataLen < BasicInfoDataMaxLen)
						{
							//for(i=0;i<BasicInfoDataLen;i++) 					/* 	Store basic info 		*/
							//{
							//	p_BasicInfo[i]=SPI_RX_Buf[i+3];
							//}
							if(SPI_RX_Buf[0] == 0)  //basic info :height/weight/age
							{
								p_BasicInfo = SPI_RX_Buf + 3;
//								UnPackBasicInfo(p_BasicInfo,BasicInfoDataLen);
								memset(p_BasicInfo,0,BasicInfoDataMaxLen*sizeof(uint8_t));
							}
							else if(SPI_RX_Buf[0] == APP_CMD_ModifySN)  //App modifies the device SN
							{
								p_BasicInfo = SPI_RX_Buf + 3;
								UnpPackAppModifySNInfo(p_BasicInfo,BasicInfoDataLen);
								memset(p_BasicInfo,0,BasicInfoDataMaxLen*sizeof(uint8_t));
							}
							else if(SPI_RX_Buf[0] == APP_CMD_RecoverTheData)  //Reset the pRead
							{
                                DataMemoryRestore();
                                TotleDataLen=0;
                                CurDataLen=0;
                                RemainDataLen=0;
                                SPISendedDataLen=0;
                                SyncACKCnt=0;     //数据同步应答次数
                                gSyncACK = TRUE;
							}
                            else if(SPI_RX_Buf[0] == APP_CMD_SetSpO2NormalMeasure)  //Set SpO2 Measure to Normal Mode
                            {
                                gFlagHR_SpO2Measure = HRSpO2_Measure_Normal;
                                if((Device_Mode == Device_Mode_RTC) && ((Get_OLED_Dis_Status() == OLEDDisEnterShutDown) || (Get_OLED_Dis_Status() == OLEDDisON)))
                                {
                                    //RTC显示模式下，收到正常模式后更新RTC界面中的模式控制图标
                                    if(gFlagHR_SpO2Measure == HRSpO2_Measure_Normal)
                                    {
                                        OLED_DisplayTestMode(OFF);
                                    }
                                    else
                                    {
                                        OLED_DisplayTestMode(ON);
                                    }
                                }
                                TS_SendEvent(gTsDeviceControlTaskID_c,gDevCtlAppModifyWorkModeEvent);
                            }
                            else if(SPI_RX_Buf[0] == APP_CMD_SetSpO2TestMeasure) //set SpO2 Measure to Test Mode
                            {
                                gFlagHR_SpO2Measure = HRSpO2_Measure_Test;
                                if((Device_Mode == Device_Mode_RTC) && ((Get_OLED_Dis_Status() == OLEDDisEnterShutDown) || (Get_OLED_Dis_Status() == OLEDDisON)))
                                {
                                    //RTC显示模式下，收到正常模式后更新RTC界面中的模式控制图标
                                    if(gFlagHR_SpO2Measure == HRSpO2_Measure_Normal)
                                    {
                                        OLED_DisplayTestMode(OFF);
                                    }
                                    else
                                    {
                                        OLED_DisplayTestMode(ON);
                                    }
                                }
                                TS_SendEvent(gTsDeviceControlTaskID_c,gDevCtlAppModifyWorkModeEvent);                                
                            }
                            else if(SPI_RX_Buf[0] == APP_CMD_PRINTF)
                            {
                                GetStorageInfo();
                            }
						}
					break;

					/* 	Frame: trig STM32 to update activity information 	*/
					case Frame_UpdateSportInfo:
						/* 	Not response while in low battery 		*/
						/* 	Not to do following actions while in low battery and charge 		*/
						if((Device_Mode == Device_Mode_LowBattery) || (Device_Mode == Device_Mode_Charge))
						{
							return;
						}
						/*  Not response while in free run */
						if((MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID) && (true == isFreeRunKickOff()))
						{
							return;
						}
						#ifdef   nRF51822_SPIRX
							printf("\r\nUpdate User Basic Info...\r\n");
						#else
							Delay_ms(2);
						#endif
						/* 	reuse alarm channel 	*/
						SPI_AlarmTransmit(Alarm_ID_Reuse_Active_History);
					break;

					 /* Frame: Notice STM32, Enter Factory Test Mode 	*/
				    case Frame_FactoryHWTest:

						/* 硬件测试，使用告警通道 */
						TS_SendEvent(gTsAlarmTaskID_c,gAlarmEventFactoryHWTest);

					break;

					/* 	Frame: Force STM32 entering factory mode 	*/
					case Frame_BackFactoryMode:

						/* 	Read device config file from DataMemory 	*/
                        GetSysConfigInfo(&DeviceCFInfo);

						DeviceCFInfo.AppActiveStatus = M95M01_APP_INACTIVE;

						/* 	Write Device active status into DataMemory */
                        SetSysConfigInfo(DeviceCFInfo);
						Delay_ms(100);

                        /* Erase Data Memory */
                        DataMemoryEraseAll();

						/* SoftReset */
						SoftReset();
					break;

					/* 	Frame: come-in call notic 		*/
                    case  Frame_ANCS_INCOMINGCALL:                               // ANCS
						/*  Not response while in free run */
						if((MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID) && (true == isFreeRunKickOff()))
						{
							return;
						}

                        if(BLE_ANCS_CATEGORY_ID_INCOMING_CALL == SPI_RX_Buf[0])
                        {
                            if(BLE_ANCS_EVENT_ID_NOTIFICATION_ADDED == SPI_RX_Buf[1])
                            {
                                #ifdef ANCS_DEBUG
                                printf("BLE_ANCS_CATEGORY_ID_INCOMING_CALL,Added...\r\n");
                                #endif

								TS_SendEvent(gTsIncomingCallTaskID_c,gCallIncomingCallAdded);
                            }
                            else if (BLE_ANCS_EVENT_ID_NOTIFICATION_REMOVED == SPI_RX_Buf[1])
                            {
                                #ifdef ANCS_DEBUG
                                printf("BLE_ANCS_CATEGORY_ID_INCOMING_CALL,Removed...\r\n");
                                #endif

								TS_SendEvent(gTsIncomingCallTaskID_c,gCallIncomingCallRemoved);
                            }
                            else
                            {}
                        }
                        break;

                    default :
						#ifdef   nRF51822_SPIRX
							printf("\r\nSPI Frame error data...\r\n");
							for(i=0;i<SPI_RX_Buf_Len;i++)
							{
								printf("0x%x, ",SPI_RX_Buf[i]);
							}
							printf("\r\n");
						#endif
                        break;
			}
			FrameTyp = 0x00;
			memset(SPI_RX_Buf,0,SPI_RX_Buf_MaxLen*sizeof(uint8_t));
	}
	/********************SPI RealTime data Transmit event**************************/
//	if(SPITranslate_Event & gSPITranslateEventTxHRSpO2) // HR/SpO2 data transmit event
//	{
//			if(HR_SpO2_SendFlag & SAMPLE_ID_HeartRate)    //transmit HR
//			{
//				SPISenddata(HeartRateID);
//				#ifdef   nRF51822_SPIRX
//				printf("Sample HR\r\n");
//				#endif
//			}
//			if(HR_SpO2_SendFlag & SAMPLE_ID_SpO2)        //transmit SpO2
//			{
//				SPISenddata(SpO2ID);
//				#ifdef   nRF51822_SPIRX
//				printf("Sample SpO2\r\n");
//				#endif
//			}
//	}
	if(SPITranslate_Event & gSPITranslateEventTxHR) 	// HR data, real-time sample, transmit
	{
			if(HR_SpO2_SendFlag & SAMPLE_ID_HeartRate)    //transmit HR data
			{
				SPISenddata(HeartRateID);
				Delay_ms(4);
				#ifdef   nRF51822_SPIRX
				printf("Sample HR\r\n");
				#endif
			}
	}
	if(SPITranslate_Event & gSPITranslateEventTxSpO2) // SpO2 data, real-time sample, transmit
	{
			if(HR_SpO2_SendFlag & SAMPLE_ID_SpO2)        //transmit SpO2 data
			{
				SPISenddata(SpO2ID);
				Delay_ms(4);
				#ifdef   nRF51822_SPIRX
				printf("Sample SpO2\r\n");
				#endif
			}
	}
	#ifdef  MoveLevelSample
	if(SPITranslate_Event & gSPITranslateEventTxMoveLevel) // Move Level data, real-time sample, transmit
	{
			SPISenddata(moveLevelID);
			Delay_ms(4);
			#ifdef   nRF51822_SPIRX
			printf("Sample Move Level\r\n");
			#endif

	}
	#endif

	if(SPITranslate_Event & gSPITranslateEventTxBatteryLevel)  //发送电池电量
	{
        if(Device_Mode != Device_Mode_FWUpdate)
        {
			SPI_BATLevelTransmit();
        }
	}
}
/*******************************************************************************
* Function Name  : DataUpdate
* Description    : DataUpdate,数据同步
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//static void DataUpdate(void)
//{
//#if   1
//		uint32_t	TotleDataLen=0;
//		uint8_t     CurDataLen=0;
//		uint32_t    RemainDataLen=0;
//		uint8_t     *SendData;       //SPI发送数据
//		uint8_t     *EEPROMData;     //EEPROM读取的数据
//		uint8_t     Err_Code=0;
//		uint8_t     j=0;
//		uint8_t     Len;
//		uint8_t 	i;
//
//		TotleDataLen = GetM95M01State(MONITOR_MODEL_VALUE_ID,M95M01_READ_AVAILABLE_SPACE);  //获取需要同步数据总长度
//		if(TotleDataLen > 0)       //有数据需要更新
//		{
//			if(TotleDataLen > (SYNC_DATA_SIZE_RD-9))     //获取当前数据包长度
//			{
//				CurDataLen = SYNC_DATA_SIZE_RD - 9;
//			}
//			else
//			{
//				CurDataLen = TotleDataLen;
//			}
//			RemainDataLen = TotleDataLen - CurDataLen;  //获取剩余数据包长度
//
//			SendData = (uint8_t *)malloc(sizeof(uint8_t) * (CurDataLen + 9));
//			EEPROMData = (uint8_t *)malloc(sizeof(uint8_t) * CurDataLen);
//
//			Err_Code = M95M01_Read(MONITOR_MODEL_VALUE_ID,CurDataLen,EEPROMData);
//			APP_ERROR_CHECK(Err_Code);
//
//			SendData[0] = 0xAA;                   //数据构造
//			SendData[1] = 0x54;
//			SendData[2] = Frame_SyncDate;

//			SendData[3] = (uint8_t)(CurDataLen / 256);
//			SendData[4] = (uint8_t)(CurDataLen % 256);

//			SendData[5] = (uint8_t)(RemainDataLen & 0xFF000000);
//			SendData[6] = (uint8_t)(RemainDataLen & 0x00FF0000);
//			SendData[7] = (uint8_t)(RemainDataLen & 0x0000FF00);
//			SendData[8] = (uint8_t)(RemainDataLen & 0x000000FF);
//
//			for(i=0;i<CurDataLen;i++)
//			{
//				SendData[i+9]= EEPROMData[i];
//			}
//			free(EEPROMData);
//
//			Len=(CurDataLen + 9)/24;
//			for(j=0;j<Len;j++)
//			{
//				SPI_Cmd(SysSPI, ENABLE);
//				GPIO_ResetBits(GPIO_SysSPINSS,GPIO_Pin_SysNSS);
//				//SysSPIBusy_Set;
//				Delay_us(10);   //必须加，51822从低功耗模式SPI唤醒时需要时间恢复时钟
//				for(i=(24*j);i<(24+j*24);i++)
//				{
//					SPI_I2S_SendData(SysSPI, SendData[i]); //????SPIx??????
//					SPI_Status_Wait(SysSPI,SPI_I2S_FLAG_TXE,SET);
//				}
//				SPI_Cmd(SysSPI, DISABLE);
//				Delay_us(10);
//				GPIO_SetBits(GPIO_SysSPINSS,GPIO_Pin_SysNSS);
//				//SysSPIBusy_Reset;
//				Delay_us(10);
//			}
//			Len = ((CurDataLen + 9)%24);
//			if(Len != 0)  //除不尽
//			{
//				SPI_Cmd(SysSPI, ENABLE);
//				GPIO_ResetBits(GPIO_SysSPINSS,GPIO_Pin_SysNSS);
//				//SysSPIBusy_Set;
//				Delay_us(10);   //必须加，51822从低功耗模式SPI唤醒时需要时间恢复时钟
//				for(i=(CurDataLen-Len);i<CurDataLen;i++)
//				{
//					SPI_I2S_SendData(SysSPI, SendData[i]); //????SPIx??????
//					SPI_Status_Wait(SysSPI,SPI_I2S_FLAG_TXE,SET);
//				}
//				for(i=0;i<(24-Len);i++)
//				{
//					SPI_I2S_SendData(SysSPI, SendData[CurDataLen-1]); //????SPIx??????
//					SPI_Status_Wait(SysSPI,SPI_I2S_FLAG_TXE,SET);
//				}
//				SPI_Cmd(SysSPI, DISABLE);
//				Delay_us(10);
//				GPIO_SetBits(GPIO_SysSPINSS,GPIO_Pin_SysNSS);
//				//SysSPIBusy_Reset;
//				Delay_us(10);
//			}
//			free(SendData);
//		}//end of if(TotleDataLen > 0)   //有数据更新
//
//#else  //测试数据同步使用
//		uint8_t senddata[120] = {0};

//		uint8_t RTCData[7]	= {0};;
//		uint8_t TemData[6]	= {0};;
//		uint8_t HRData[5]	= {0};;
//		uint8_t SpO2Data[5]	= {0};;
//		uint8_t retry=0;
//		uint8_t i=0,j=0;
//		date_str_typedef date_s;
//		RTC_TimeTypeDef rtc_time;
//
//      Calendar_Get(&date_s,&rtc_time);

//		printf("Start DataUpdate\r\n");

//		senddata[0] = 0xAA;
//		senddata[1] = 0x54;
//		senddata[2] = Frame_SyncDate;

//		senddata[3] = 0;
//		senddata[4] = 111;

//		senddata[5] = 0;
//		senddata[6] = 0;
//		senddata[7] = 0;
//		senddata[8] = 111;

//		for(j=0;j<3;j++)
//		{
//			senddata[9+37*j] = 0x14;
//			senddata[10+37*j] = 0x0E;
//			senddata[11+37*j] = date_s.month;
//			senddata[12+37*j] = date_s.day;
//			senddata[13+37*j] = rtc_time.RTC_Hours;
//			senddata[14+37*j] = rtc_time.RTC_Minutes+j;
//			senddata[15+37*j] = rtc_time.RTC_Seconds;

//			senddata[16+37*j] = TemperatureID;
//			senddata[17+37*j] = '3';
//			senddata[18+37*j] = '6';
//			senddata[19+37*j] = '.';
//			senddata[20+37*j] = '5'+j;
//			senddata[21+37*j] = '#';

//			senddata[22+37*j] = 0x14;
//			senddata[23+37*j] = 0x0E;
//			senddata[24+37*j] = date_s.month;
//			senddata[25+37*j] = date_s.day;
//			senddata[26+37*j] = rtc_time.RTC_Hours;
//			senddata[27+37*j] = rtc_time.RTC_Minutes+j;
//			senddata[28+37*j] = rtc_time.RTC_Seconds;

//			senddata[29+37*j] = HeartRateID;
//			senddata[30+37*j] = '0';
//			senddata[31+37*j] = '6'+j;
//			senddata[32+37*j] = '6';
//			senddata[33+37*j] = '#';

//			senddata[34+37*j] = 0x14;
//			senddata[35+37*j] = 0x0E;
//			senddata[36+37*j] = date_s.month;
//			senddata[37+37*j] = date_s.day;
//			senddata[38+37*j] = rtc_time.RTC_Hours;
//			senddata[39+37*j] = rtc_time.RTC_Minutes+j;
//			senddata[40+37*j] = rtc_time.RTC_Seconds;

//			senddata[41+37*j] = SpO2ID;
//			senddata[42+37*j] = '0';
//			senddata[43+37*j] = '9';
//			senddata[44+37*j] = '6'+j;
//			senddata[45+37*j] = '#';
//		}

//		for(j=0;j<6;j++)
//		{
//			SPI_Cmd(SysSPI, ENABLE);
//			GPIO_ResetBits(GPIO_SysSPINSS,GPIO_Pin_SysNSS);
//			//SysSPIBusy_Set;
//			Delay_us(10);   //必须加，51822从低功耗模式SPI唤醒时需要时间恢复时钟
//			for(i=(24*j);i<(24+j*24);i++)
//			{
//				SPI_I2S_SendData(SysSPI, senddata[i]); //????SPIx??????
//				SPI_Status_Wait(SysSPI,SPI_I2S_FLAG_TXE,SET);
//			}
//			SPI_Cmd(SysSPI, DISABLE);
//			Delay_us(10);
//			GPIO_SetBits(GPIO_SysSPINSS,GPIO_Pin_SysNSS);
//			//SysSPIBusy_Reset;
//			Delay_us(10);
//		}
//#endif
//}
/*******************************************************************************
* Function Name  : SPISenddata
* Description    : SPISenddata,SPI发送即时采集数据
* Input          : 即时采集参数ID
* Output         : None
* Return         : None
*******************************************************************************/
static void SPISenddata(uint8_t ID)
{
		uint8_t Senddata[24]={0};

		switch(ID)
		{
			case HeartRateID:            //发送心率
				Senddata[0] = 0xAA;
				Senddata[1] = 0x54;
				Senddata[2] = Frame_CheckUpDate;

				Senddata[3] = 0x00;  //数据长度
				Senddata[4] = 0x0E;  //数据长度

				Senddata[5] = 0x14;
				Senddata[6] = (uint8_t)(gHRSpO2Val.date_s.year-2000);
				Senddata[7] = gHRSpO2Val.date_s.month;
				Senddata[8] = gHRSpO2Val.date_s.day;
				Senddata[9] = gHRSpO2Val.rtc_time.RTC_Hours;
				Senddata[10] = gHRSpO2Val.rtc_time.RTC_Minutes;
				Senddata[11] = gHRSpO2Val.rtc_time.RTC_Seconds;

				Senddata[12] = HeartRateID;  //参数ID
				Senddata[13] = 0xFF;  //数据标志
				Senddata[14] = ((gHRSpO2Val.HrSpO2DataRep_pst)->m_ui8HrVal / 100) + 0x30;
				Senddata[15] = (((gHRSpO2Val.HrSpO2DataRep_pst)->m_ui8HrVal % 100)/10) + 0x30;
				Senddata[16] = ((gHRSpO2Val.HrSpO2DataRep_pst)->m_ui8HrVal % 10) + 0x30;
				Senddata[17] = 0x23;

				Senddata[23]=0xDD;

                spi_data_send(Senddata, 24);
				#ifdef   nRF51822_SPIRX
				printf("Send HR\r\n");
				#endif
				break;
			case SpO2ID:
				Senddata[0] = 0xAA;
				Senddata[1] = 0x54;
				Senddata[2] = Frame_CheckUpDate;

				Senddata[3] = 0x00;  //数据长度
				Senddata[4] = 0x0E;  //数据长度

				Senddata[5] = 0x14;
				Senddata[6] = (uint8_t)(gHRSpO2Val.date_s.year-2000);
				Senddata[7] = gHRSpO2Val.date_s.month;
				Senddata[8] = gHRSpO2Val.date_s.day;
				Senddata[9] = gHRSpO2Val.rtc_time.RTC_Hours;
				Senddata[10] = gHRSpO2Val.rtc_time.RTC_Minutes;
				Senddata[11] = gHRSpO2Val.rtc_time.RTC_Seconds;

				Senddata[12] = SpO2ID;  //参数ID
				Senddata[13] = 0xFF;  //数据标志

				Senddata[14] = ((gHRSpO2Val.HrSpO2DataRep_pst)->m_ui8SpO2Val / 100) + 0x30;
				Senddata[15] = ((gHRSpO2Val.HrSpO2DataRep_pst)->m_ui8SpO2Val % 100)/10 + 0x30;
				Senddata[16] = ((gHRSpO2Val.HrSpO2DataRep_pst)->m_ui8SpO2Val % 10) + 0x30;
				Senddata[17] = 0x23;

				Senddata[23]=0xDD;

                spi_data_send(Senddata, 24);

				#ifdef   nRF51822_SPIRX
				printf("Send SpO2\r\n");
				#endif
				break;

			#ifdef  MoveLevelSample
			case moveLevelID:
				Senddata[0] = 0xAA;
				Senddata[1] = 0x54;
				Senddata[2] = Frame_CheckUpDate;

				Senddata[3] = 0x00;  //数据长度
				Senddata[4] = 0x10;  //数据长度

				Senddata[5] = 0x14;
				Senddata[6] = (uint8_t)(gHRSpO2Val.date_s.year-2000);
				Senddata[7] = gHRSpO2Val.date_s.month;
				Senddata[8] = gHRSpO2Val.date_s.day;
				Senddata[9] = gHRSpO2Val.rtc_time.RTC_Hours;
				Senddata[10] = gHRSpO2Val.rtc_time.RTC_Minutes;
				Senddata[11] = gHRSpO2Val.rtc_time.RTC_Seconds;

				Senddata[12] = moveLevelID;  //参数ID
				Senddata[13] = 0xFF;  //数据标志

				Senddata[14] = (gHRSpO2Val.moveLevelVal / 10000) + 0x30;
				Senddata[15] = ((gHRSpO2Val.moveLevelVal / 1000)%10) + 0x30;
				Senddata[16] = ((gHRSpO2Val.moveLevelVal / 100)%10) + 0x30;
				Senddata[17] = ((gHRSpO2Val.moveLevelVal / 10)%10) + 0x30;
				Senddata[18] = (gHRSpO2Val.moveLevelVal %10) + 0x30;
				Senddata[19] = 0x23;

				Senddata[23]=0xDD;

                spi_data_send(Senddata, 24);
				#ifdef   nRF51822_SPIRX
				printf("Send Move Level\r\n");
				#endif
				break;
			#endif
			default : break;
		}
}
/*******************************************************************************
* Function Name  : SPI_AlarmTransmit
* Description    : SPI_AlarmTransmit,SPI发送告警信息
* Input          : 告警ID
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_AlarmTransmit(uint8_t AlarmType)
{
		uint8_t 	Senddata[24] = {0};

		//AlarmType = AlarmType;

		Senddata[0] = 0xAA;
		Senddata[1] = 0x54;
		Senddata[2] = Frame_Alarm;

		Senddata[3]=AlarmType;   //当前数据包长度
		Senddata[4]=0x11;

        spi_data_send(Senddata, 24);
}
/*******************************************************************************
* Function Name  : SPI_BATLevelTransmit
* Description    : SPI_BATLevelTransmit,SPI发送电池电量
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_BATLevelTransmit(void)
{
		uint8_t 	Senddata[24]={0};

		Senddata[0] = 0xAA;
		Senddata[1] = 0x54;
		Senddata[2] = Frame_BATLevel;

		Senddata[3] = GetBatScaleLevel();   /* 	Get the battery level 	*/

		#ifdef Battery_Debug
		printf("BAT Level Transmit:%d\r\n",Senddata[3]);
		#endif

        spi_data_send(Senddata, 24);
}
/*******************************************************************************
* Function Name  : SPI_FWDataTransmitCtl
* Description    : SPI_FWDataTransmitCtl,固件数据传输控制
* Input          : 固件数据传输控制帧
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FWDataTransmitCtl(uint8_t Frame)
{
		uint8_t 	Senddata[24]={0};

		/* 	Init the data 		*/
		Senddata[0] = 0xAA;
		Senddata[1] = 0x54;
		Senddata[2] = Frame_FWUpdateCtlA;
		Senddata[3] = Frame;   /* The frame control 	*/

        spi_data_send(Senddata, 24);
}
/*******************************************************************************
* Function Name  : SPI_DeviceInformationTransmit
* Description    : 设备硬件信息发送
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_DeviceInformationTransmit(void)
{

		uint8_t 	Senddata[24] = {0};

		Senddata[0] = 0xAA;
		Senddata[1] = 0x54;
		Senddata[2] = Frame_DeviceInform;

		Senddata[3] = g_DeviceInfomation.DevSupportCap;
		Senddata[4] = g_DeviceInfomation.pDevREV[0];
		Senddata[5] = g_DeviceInfomation.pDevREV[1];
		Senddata[6] = g_DeviceInfomation.pDevREV[2];
		Senddata[7] = g_DeviceInfomation.pFWREV[0];
		Senddata[8] = g_DeviceInfomation.pFWREV[1];
		Senddata[9] = g_DeviceInfomation.pFWREV[2];
		Senddata[10] = g_DeviceInfomation.pSeriNum[0];
		Senddata[11] = g_DeviceInfomation.pSeriNum[1];
		Senddata[12] = g_DeviceInfomation.pSeriNum[2];
		Senddata[13] = g_DeviceInfomation.pSeriNum[3];
//        Senddata[10] = 0x99;
//		Senddata[11] = 0x99;
//		Senddata[12] = 0x99;
//		Senddata[13] = 0x07;
		Senddata[14] = g_DeviceInfomation.pBootREV[0];
		Senddata[15] = g_DeviceInfomation.pBootREV[1];
		Senddata[16] = g_DeviceInfomation.pBootREV[2];
        Senddata[17] = 0x02;                               // 协议版本号

		spi_data_send(Senddata, 24);
        #ifdef FWUpdate_Debug
		printf("SPI_DeviceInformationTransmit...\r\n");
		#endif

		g_DeviceInfomation.DeviceInfo_Status = 1;
}

/*******************************************************************************
* Function Name  : SPI_FWDataTransmitCtl
* Description    : SPI_FWDataTransmitCtl,固件数据传输控制
* Input          : 固件数据传输控制帧
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FWDataTransmitBLEdata(uint8_t Frame)
{
		uint8_t 	Senddata[24] = {0};

		Senddata[0] = 0xAA;
		Senddata[1] = 0x54;
		Senddata[2] = Frame_UpdateBleRate;
		Senddata[3] = Frame;   //固件数据传输控制

		spi_data_send(Senddata, 24);
}

static void spi_data_send(uint8_t *pDatSend, uint8_t lenDat)
{
	uint8_t SPIBusyTry_Cnt=0;
	uint8_t Ack=0;
	uint8_t i;

    SPI_GPIO_Config();
	do
	{
		SPI_Cmd(SysSPI, ENABLE);
		GPIO_ResetBits(GPIO_SysSPINSS,GPIO_Pin_SysNSS);
		Delay_us(10);   /* Must have, delay for 51822 to wakeup 	*/
		for(i=0;i<lenDat;i++)
		{
			SPI_I2S_SendData(SysSPI, pDatSend[i]); /* 	Send Data thru SPI driver		*/
			Ack = SPI_I2S_ReceiveData(SysSPI);
			if(Ack == 0xAB)
			{
				break;
			}
			SPI_Status_Wait(SysSPI,SPI_I2S_FLAG_TXE,SET);
		}
		GPIO_SetBits(GPIO_SysSPINSS,GPIO_Pin_SysNSS);
		Delay_us(10);
		SPI_Cmd(SysSPI, DISABLE);
		Delay_ms(2);
		#ifdef FWUpdate_Debug
		if(Ack == 0xAB)
		{
			printf("Try again\r\n");
		}
		#endif
		SPIBusyTry_Cnt++;
	}while((Ack == 0xAB) && (SPIBusyTry_Cnt < 20));
}

/*******************************************************************************
* Function Name  : UnpPackAppModifySNInfo
* Description    : 解包App修改设备序列号信息
* Input          : p_ParameterLimit,len
* Output         : None
* Return         : None
*******************************************************************************/
static void UnpPackAppModifySNInfo(uint8_t *p_BasicInfo,uint16_t Len)
{
	uint8_t i=0;

	#ifdef AppModifySN_Debug
	printf("Unpack Modify SN Info...\r\n");
	printf("SNInfo Length:%d\r\n",Len);
	for(i=0;i<Len;i++)
	{
		printf("0x%x, ",p_BasicInfo[i]);
	}
	printf("\r\n");
	#endif

	for(i=0;i<Len;i++)
	{
		u8AppModifySeriNum[i] = p_BasicInfo[i];
	}

	TS_SendEvent(gTsDeviceControlTaskID_c,gDevCtlAppModifySNEvent);
}

bool SPI_Status_Wait(SPI_TypeDef * SPI, uint16_t bitMask, FlagStatus waitForStatus)
{
	uint16_t u16TimeOut = 0;
    while (SPI_I2S_GetFlagStatus(SPI, bitMask) != waitForStatus)
		{
			if(++u16TimeOut == SPI_OP_TIMEOUT)
			{
				return(TRUE);
			}
		}
	return(FALSE);
}


