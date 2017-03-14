#include "SyncData.h"
#include "stdlib.h"
#include <stdarg.h>

uint32_t	TotleDataLen=0;
uint8_t     CurDataLen=0;
uint32_t    RemainDataLen=0;
uint8_t     SPISendedDataLen=0;
uint8_t     SyncACKCnt=0;     //êy?Yí?2?ó|′e′?êy
uint16_t    SyncDataLen=0;
uint8_t     Flag_SyncDataDone=false;
uint8_t     Flag_RxedSyncDataLength = false;
uint16_t    RxedSyncDataLength = 0;   //App发送数据同步应答长度

static uint8_t     EEPROMData[EEPROMReadMaxData]={0};     //EEPROM读取的数据
static MTDataBuffer_t  FlashData;

/* 	SYNC timer 		*/
static Timer_ID_Typedef     gSYNCTIMID = TIMER_ERROR;
static Timer_ID_Typedef     gSYNCLenTIMID = TIMER_ERROR;

//static uint8_t send_flag = false;
extern uint8_t RxData[];
extern uint8_t  SPIRxDataInTx_Flag;

static void BLE_LOG(char *str);

/*******************************************************************************
* Function Name  : FlashReadPoll
* Description    : Flash read busy poll
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void FlashReadPoll(Timer_ID_Typedef TIMID)
{
    if(GetExtFlashAccessState() == FLASH_ACCESS_DISABLE)
    {
        #ifdef SyncData_DEBUG
            printf("Read Flash Busy, not send Sync Event\r\n");
        #endif
    }
    else
    {
        TS_SendEvent(gTsSyncDataTaskID_c,gSyncDataFormat2EventStart);
    }
}

void SyncData_Task_Handler(event_t Sync_Event)
{
		uint8_t     SendData[24] = {0};       //SPI发送数据
		uint8_t     Err_Code=0;
		uint8_t     i=0;
		uint8_t     Ack = 0;
		uint16_t    RxDataLen=0;
		uint8_t     SPIBusyTry_Cnt=0;    //SPI数据传输Busy时，try计数
		uint32_t    SyncDataCRC32 = 0;
		uint16_t    SyncDataCRC16 = 0;
		TIM_Cfg_Typedef         Tim_Cfg_SYNC_Index;		//SYNC timer配置，用于控制查询同步事件中Flash读状态
		TIM_Basic_Cfg_Typedef 	Tim_Cfg_SYNC;       	//SYNC timer配置

		SendData[0] = 0xAA;                   //数据构造
		SendData[1] = 0x54;
		SendData[2] = Frame_SyncDate;

		if(Sync_Event & gSyncDataFormat2EventStart)   //开始数据格式2同步事件
		{
			if(Device_State == Device_WorkWithAPP)
			{
				if(gSyncACK == TRUE) //上次同步已应答
				{
					SyncACKCnt = 0;
					TotleDataLen = GetDataMemoryTotleLength();  //获取需要同步数据总长度

                    #ifdef SyncData_DEBUG
                        printf("\r\n TotleDataLen:%d\r\n",TotleDataLen);
                    #endif

					if(TotleDataLen > 0)       //有数据需要更新
					{
						if(TotleDataLen > (SYNC_DATA_SIZE_RD-9))     //获取当前数据包长度
						{
							CurDataLen = SYNC_DATA_SIZE_RD - 9;
						}
						else
						{
							CurDataLen = TotleDataLen;
						}
						RemainDataLen = TotleDataLen ;  //获取剩余数据包长度(包含当前数据包长度)

						if(MonitorTemplate.ReadLength == 0)
						{
							#ifdef SyncData_DEBUG
								printf("Err: MonitorTemplate.ReadLength\r\n");
							#endif
							return;
						}
						CurDataLen = (CurDataLen/MonitorTemplate.ReadLength) * MonitorTemplate.ReadLength;

						memset(EEPROMData,0,EEPROMReadMaxData*sizeof(uint8_t));
                        memset(FlashData.Buffer,0,EEPROMReadMaxData*sizeof(uint8_t));

						//Err_Code = M95M01_Read(MONITOR_MODEL_VALUE_ID,CurDataLen,EEPROMData);
						if(GetExtFlashAccessState() == FLASH_ACCESS_DISABLE)
						{
							#ifdef SyncData_DEBUG
								printf("Read Flash Busy\r\n");
							#endif

                            /* for test */
                            MX25_SPI_Configuration();
                            if(IsFlashBusy() == FALSE)
                            {
                               //printf("Read Flash HW Not Busy\r\n");
                            }

							/* Flash Read Busy */
							/* Set the timer to Send gSyncDataFormat2EventStart */
							if(gSYNCTIMID != TIMER_ERROR)
							{
								Timer_Free(gSYNCTIMID);
								gSYNCTIMID = TIMER_ERROR;
							}

							Tim_Cfg_SYNC.enuTimerType = TIM_TYPE_MS;
							Tim_Cfg_SYNC.u16TimePeriod = SYNCRDFlashPollTime;
							Tim_Cfg_SYNC.NVIC_IRQChannelPreemptionPriority = SYNCPollPreemptionPriority;
							Tim_Cfg_SYNC.pIntCallBack = FlashReadPoll;

							/* Init timer top define */
							Tim_Cfg_SYNC_Index.TimerMode 			= TIM_MODE_BASIC;
							Tim_Cfg_SYNC_Index.TimerBasicCfg 		= &Tim_Cfg_SYNC;
							Tim_Cfg_SYNC_Index.TimerPWMCfg 			= NULL;

							gSYNCTIMID = Timer_Allocate(&Tim_Cfg_SYNC_Index);
							Start_Timer_Cnt(gSYNCTIMID);

							return;
						}
						/* Disable the timer of Send gSyncDataFormat2EventStart*/
						if(gSYNCTIMID != TIMER_ERROR)
						{
							Timer_Free(gSYNCTIMID);
							gSYNCTIMID = TIMER_ERROR;
						}

                        gSyncACK = FALSE;  //等待同步应答
						Flag_SyncDataDone = false;
                        gFlagSyncDataInProgress = true;

						Err_Code = (uint8_t)DataMemoryRead(MONITOR_TYPE,&FlashData,CurDataLen);
						CurDataLen = FlashData.availabe_length;
						if(CurDataLen == 0)
						{
							#ifdef SyncData_DEBUG
								printf("Read Flash availabe_length 0\r\n");
							#endif

							/* Set the timer to Send gSyncDataFormat2EventStart */
							if(gSYNCLenTIMID != TIMER_ERROR)
							{
								Timer_Free(gSYNCLenTIMID);
								gSYNCLenTIMID = TIMER_ERROR;
							}

							Tim_Cfg_SYNC.enuTimerType = TIM_TYPE_MS;
							Tim_Cfg_SYNC.u16TimePeriod = SYNCRDFlashPollTime;
							Tim_Cfg_SYNC.NVIC_IRQChannelPreemptionPriority = SYNCPollPreemptionPriority;
							Tim_Cfg_SYNC.pIntCallBack = FlashReadPoll;

							/* Init timer top define */
							Tim_Cfg_SYNC_Index.TimerMode 			= TIM_MODE_BASIC;
							Tim_Cfg_SYNC_Index.TimerBasicCfg 		= &Tim_Cfg_SYNC;
							Tim_Cfg_SYNC_Index.TimerPWMCfg 			= NULL;

							gSYNCLenTIMID = Timer_Allocate(&Tim_Cfg_SYNC_Index);
							Start_Timer_Cnt(gSYNCLenTIMID);

							return;
						}
						/* Disable the timer of Send gSyncDataFormat2EventStart*/
						if(gSYNCLenTIMID != TIMER_ERROR)
						{
							Timer_Free(gSYNCLenTIMID);
							gSYNCLenTIMID = TIMER_ERROR;
						}                        
						memmove(EEPROMData,FlashData.Buffer,CurDataLen);

						#ifdef SyncData_DEBUG
						printf("\r\n Read from EEPROM Start\r\n");
						for(i=0;i<CurDataLen;i++)
						{
							printf("0x%02x, ",EEPROMData[i]);
                    		if(!((i + 1) % 16))
                    		{
                    			printf("\r\n");
                    		}
						}
						printf("\r\n Read from EEPROM Stop\r\n");
						#endif
						APP_ERROR_CHECK(Err_Code);

						SyncDataCRC32 = crc32(SyncDataCRC32,EEPROMData,CurDataLen);
						SyncDataCRC16 = crc32_To_crc16(SyncDataCRC32);

						EEPROMData[CurDataLen] = (uint8_t)(SyncDataCRC16 >> 8);
						EEPROMData[CurDataLen+1] = (uint8_t)(SyncDataCRC16 & 0x00FF);

						CurDataLen = CurDataLen + 2;  //2为每一包数据同步数据中CRC
						SyncDataLen = CurDataLen;

						#ifdef SyncData_DEBUG
						printf("\r\n Start Sync data...\r\n");
						printf("\r\n TotleDataLen:%d\r\n",TotleDataLen);
						printf("\r\n RemainDataLen:%d\r\n",RemainDataLen);
						printf("\r\n CurDataLen:%d\r\n",CurDataLen);
						printf("\r\n SPISendedDataLen:%d\r\n",SPISendedDataLen);
						#else
						Delay_ms(2);
						#endif

//						SendData[0] = 0xAA;                   //数据构造
//						SendData[1] = 0x54;
//						SendData[2] = Frame_SyncDate;
						if(CurDataLen > 2)  //当前有数据更新才发送
						{
							SendData[3] = (uint8_t)(CurDataLen / 256);
							SendData[4] = (uint8_t)(CurDataLen % 256);

							SendData[5] = (uint8_t)((RemainDataLen & 0xFF000000) >> 24);
							SendData[6] = (uint8_t)((RemainDataLen & 0x00FF0000) >> 16);
							SendData[7] = (uint8_t)((RemainDataLen & 0x0000FF00) >> 8);
							SendData[8] = (uint8_t)(RemainDataLen & 0x000000FF);

							SendData[9] = SyncDataFormat2;
							SendData[10] = (uint8_t)((FlashData.start_time >> 16) / 100);  //start time
							SendData[11] = (uint8_t)((FlashData.start_time >> 16) % 100);  //start time
							SendData[12] = (uint8_t)((FlashData.start_time >> 8));  //start time
							SendData[13] = (uint8_t)(FlashData.start_time);  //start time
							SendData[14] = 0;  //start time
							SendData[15] = 0;  //start time
							SendData[16] = 0;  //start time
							SendData[17] = MonitorTemplate.ReadLength;

							if(CurDataLen < 5)
							{
								for(i=0;i<CurDataLen;i++)
								{
									SendData[i+18]= EEPROMData[i];
								}
								for(i=(18+CurDataLen);i<23;i++)  //curdatalen之后的数据设置为0
								{
									SendData[i]=0;
								}
								SPISendedDataLen = CurDataLen;
							}
							else
							{
								for(i=18;i<23;i++)
								{
									SendData[i]= EEPROMData[i-18];
								}
								SPISendedDataLen = 5;
							}

							SPIBusyTry_Cnt = 0;
                            SPI_GPIO_Config();
							do
							{
								/* SPI 发送数据 */
								SPI_Cmd(SysSPI, ENABLE);
								GPIO_ResetBits(GPIO_SysSPINSS,GPIO_Pin_SysNSS);
								//SysSPIBusy_Set;
								Delay_us(10);   //必须加，51822从低功耗模式SPI唤醒时需要时间恢复时钟
								for(i=0;i<23;i++)
								{
									SPI_I2S_SendData(SysSPI, SendData[i]); //????SPIx??????

									SPI_Status_Wait(SysSPI,SPI_I2S_FLAG_TXE,SET);

									SPI_Status_Wait(SysSPI,SPI_I2S_FLAG_RXNE,SET);

									Ack = SPI_I2S_ReceiveData(SysSPI);
									RxData[i] = Ack;

									#ifdef SPI_nRF_Debug
									printf("0x%x, ",Ack);
									#endif
									if((Ack == 0xAB)&&(i == 0))
									{
										break;
									}

									#ifdef SyncData_DEBUG
									//printf("0x%x, ",SendData[i]);
									#endif

								}
								if((RxData[0] == 0xAA) && (RxData[1] == 0x55))
								{
									RxDataLen = (RxData[3]<<8) + RxData[4];
									if(RxDataLen > 18)
									{
										RxDataLen = RxDataLen - 18;
										for(i=0;i<RxDataLen && i<(RxDataSize-23);i++)
										{
											SPI_I2S_SendData(SysSPI, 0xAC); //????SPIx??????

											SPI_Status_Wait(SysSPI,SPI_I2S_FLAG_TXE,SET);

											SPI_Status_Wait(SysSPI,SPI_I2S_FLAG_RXNE,SET);

											RxData[23+i] = SPI_I2S_ReceiveData(SysSPI);
										}
									}
									RxDataLen = 0;
									SPIRxDataInTx_Flag = 1;
								}
								else
								{
									RxDataLen = 0;
									memset(RxData,0,RxDataSize*sizeof(uint8_t));
								}

								SPI_Cmd(SysSPI, DISABLE);
								Delay_us(10);
								GPIO_SetBits(GPIO_SysSPINSS,GPIO_Pin_SysNSS);
								Delay_ms(2);
								#ifdef SPI_nRF_Debug
								if(Ack == 0xAB)
								{
									printf("\r\nTry Again1\r\n");
								}
								else
								{
									printf("\r\nNormal1\r\n");
								}
								#endif
								SPIBusyTry_Cnt++;
							}while((Ack == 0xAB) && (SPIBusyTry_Cnt < 20));
						} //end of if(CurDataLen > 2)  //当前有数据更新才发送
						else
						{
							SPI_FWDataTransmitBLEdata(0);
						}
					}//end of if(TotleDataLen > 0)       //有数据需要更新
					else
					{
						SPI_FWDataTransmitBLEdata(0);
                        gFlagSyncDataInProgress = false;
                        /* Flash Read Busy */
                        /* Set the timer to Send gSyncDataFormat2EventStart */
                        if(gSYNCTIMID != TIMER_ERROR)
                        {
                            Timer_Free(gSYNCTIMID);
                            gSYNCTIMID = TIMER_ERROR;
                        }
					}
				}//end of if(gSyncACK == TRUE) //上次同步已应答
				else
				{
					if(SyncACKCnt >= 1)
					{
						gSyncACK = TRUE;  //3次数据同步未应答
						TS_SendEvent(gTsSyncDataTaskID_c,gSyncDataFormat2EventStart);  //产生开始数据同步事件
					}
					SyncACKCnt++;  //数据同步未应答次数计数
                    gFlagSyncDataInProgress = false;
					#ifdef SyncData_DEBUG
						printf("\r\n Wate Sync Ack Cnt=%d\r\n",SyncACKCnt);
					#endif
				}
			}//end of if(Device_State == Device_WorkWithAPP)
		}
		else if(Sync_Event & gSyncDataEventSPISent) //SPI发送数据事件
		{
			#ifdef SyncData_DEBUG
				//printf("\r\n  Event Start SPI Send Sync Data\r\n");
				//printf("\r\n CurDataLen:%d\r\n",CurDataLen);
                //printf("\r\n SPISendedDataLen:%d\r\n",SPISendedDataLen);
			#endif
			if(SPISendedDataLen < CurDataLen)  //还有数据没有发送完
			{
				#ifdef SyncData_DEBUG
					//printf("\r\n  Start Spi Send Sync data...\r\n");
				#else
					Delay_ms(2);
				#endif

				if((CurDataLen-SPISendedDataLen)>20)
				{
					for(i=0;i<20;i++)
					{
						SendData[i+3] = EEPROMData[i+SPISendedDataLen];
					}
					SPISendedDataLen = SPISendedDataLen + 20;
				} // if((CurDataLen-SPISendedDataLen)>20)
				else  //if((CurDataLen-SPISendedDataLen)<=20)
				{
					for(i=0;i<(CurDataLen-SPISendedDataLen);i++)
					{
						SendData[i+3] = EEPROMData[i+SPISendedDataLen];
					}
					for(i=(CurDataLen-SPISendedDataLen);i<20;i++)  //curdatalen之后的数据设置为0
					{
						SendData[i+3] = 0;
					}
					SPISendedDataLen = CurDataLen;
				}  //end of if((CurDataLen-SPISendedDataLen)<=20)

				/* SPI 发送数据 */
				SPIBusyTry_Cnt = 0;
                SPI_GPIO_Config();
				do
				{
					SPI_Cmd(SysSPI, ENABLE);
					GPIO_ResetBits(GPIO_SysSPINSS,GPIO_Pin_SysNSS);
					//SysSPIBusy_Set;
					Delay_us(10);   //必须加，51822从低功耗模式SPI唤醒时需要时间恢复时钟
					for(i=0;i<23;i++)
					{
						SPI_I2S_SendData(SysSPI, SendData[i]); //????SPIx??????

						SPI_Status_Wait(SysSPI,SPI_I2S_FLAG_TXE,SET);

						SPI_Status_Wait(SysSPI,SPI_I2S_FLAG_RXNE,SET);

						Ack = SPI_I2S_ReceiveData(SysSPI);
						RxData[i] = Ack;
						#ifdef SPI_nRF_Debug
						printf("0x%x, ",Ack);
						#endif
						if((Ack == 0xAB)&&(i==0))
						{
							break;
						}

						#ifdef SyncData_DEBUG
						//printf("0x%x, ",SendData[i]);
						#endif
					}
					if((RxData[0] == 0xAA) && (RxData[1] == 0x55))
					{
						RxDataLen = (RxData[3]<<8) + RxData[4];
						if(RxDataLen > 18)
						{
							RxDataLen = RxDataLen - 18;
							for(i=0;i<RxDataLen && i<(RxDataSize-23);i++)
							{
								SPI_I2S_SendData(SysSPI, SendData[i]); //????SPIx??????

								SPI_Status_Wait(SysSPI,SPI_I2S_FLAG_TXE,SET);

								SPI_Status_Wait(SysSPI,SPI_I2S_FLAG_RXNE,SET);

								RxData[23+i] = SPI_I2S_ReceiveData(SysSPI);
							}
						}
						RxDataLen = 0;
						SPIRxDataInTx_Flag = 1;
					}
					else
					{
						RxDataLen = 0;
						memset(RxData,0,RxDataSize*sizeof(uint8_t));
					}
					SPI_Cmd(SysSPI, DISABLE);
					Delay_us(10);
					GPIO_SetBits(GPIO_SysSPINSS,GPIO_Pin_SysNSS);
					Delay_ms(2);
					#ifdef SPI_nRF_Debug
					if(Ack == 0xAB)
					{
						printf("\r\nTry Again2\r\n");
					}
					else
					{
						printf("\r\nNormal2\r\n");
					}
					#endif
				}while((Ack == 0xAB) && (SPIBusyTry_Cnt < 20));
			}
			else
			{
				//if(GetM95M01State(MONITOR_MODEL_VALUE_ID,M95M01_CAPACITY_SPACE) > 50)
                if(1)
				{
					gAlarmNotEnoughSpace = false;
				}
				TotleDataLen=0;
				CurDataLen=0;
				RemainDataLen=0;
				SPISendedDataLen=0;
				SyncACKCnt=0;     //数据同步应答次数
				Flag_SyncDataDone = true;

                if((Flag_RxedSyncDataLength == true) && (Flag_SyncDataDone == true))
                {

                    DataMemoryReadACK(RxedSyncDataLength);

                    gSyncACK = TRUE;   //数据已同步
                    SyncDataLen = 0;
                    Flag_SyncDataDone = false;
                    Flag_RxedSyncDataLength = false;
                    RxedSyncDataLength = 0;
                }

				#ifdef SyncData_DEBUG
					printf("Sync Data Done\r\n");
				#endif
			}
		}
}
/*******************************************************************************
* Function Name  : SPI_BATLevelTransmit
* Description    : SPI_BATLevelTransmit,SPI发送电池电量
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void BLE_LOG(char *str)
{
		uint8_t 	i=0;
	    uint8_t     BleLogDataTotleLen =0;
		uint8_t     BleLogDataCurLen =0;
		uint8_t     BleLogDataSPISendedLen =0;
		uint8_t     BleLogSendData[256]={0};
		uint8_t     SendData[24] = {0};       //SPI发送数据
		uint8_t 	retry=0;

		SendData[0] = 0xAA;             //数据构造
		SendData[1] = 0x54;
		SendData[2] = Frame_DeviceLog;

		BleLogDataTotleLen = strlen(str);  //打印数据总长度

		for(i=0;i<BleLogDataTotleLen;i++)
		{
			BleLogSendData[i]=str[i];
		}

		while((BleLogDataTotleLen - BleLogDataSPISendedLen) > 0)
		{
			if((BleLogDataTotleLen - BleLogDataSPISendedLen)>19)
			{
				BleLogDataCurLen = 19;
				SendData[3] = BleLogDataCurLen;
				for(i=0;i<BleLogDataCurLen;i++)
				{
					SendData[i+4] = BleLogSendData[BleLogDataSPISendedLen + i];
				}
				BleLogDataSPISendedLen = BleLogDataSPISendedLen + BleLogDataCurLen;
			}
			else
			{
				BleLogDataCurLen = (BleLogDataTotleLen - BleLogDataSPISendedLen);
				SendData[3] = BleLogDataCurLen;
				for(i=0;i<BleLogDataCurLen;i++)
				{
					SendData[i+4] = BleLogSendData[BleLogDataSPISendedLen + i];
				}
				for(i=BleLogDataCurLen;i<19;i++)
				{
					SendData[i+4] = 0;
				}
				BleLogDataSPISendedLen = BleLogDataSPISendedLen + BleLogDataCurLen;
			}

			/* SPI 发送数据 */
            SPI_GPIO_Config();
			SPI_Cmd(SysSPI, ENABLE);
			GPIO_ResetBits(GPIO_SysSPINSS,GPIO_Pin_SysNSS);
			//SysSPIBusy_Set;
			Delay_us(10);   //必须加，51822从低功耗模式SPI唤醒时需要时间恢复时钟
			for(i=0;i<23;i++)
			{
				SPI_I2S_SendData(SysSPI, SendData[i]); //????SPIx??????
				#ifdef SyncData_DEBUG
				printf("0x%x, ",SendData[i]);
				#endif
				while (SPI_I2S_GetFlagStatus(SysSPI, SPI_I2S_FLAG_TXE) == RESET) //?????SPI???????:????????
				{
					retry++;
					if(retry>200)break;
				}
			}
			SPI_Cmd(SysSPI, DISABLE);
			Delay_us(10);
			GPIO_SetBits(GPIO_SysSPINSS,GPIO_Pin_SysNSS);
			//SysSPIBusy_Reset;
			Delay_us(10);
		}

}
/*******************************************************************************
* Function Name  : BLEprintf
* Description    : BLEprintf格式化输出
* Input          : fmt
* Output         : None
* Return         : None
* eg：   BLEprintf("\r\n Test=%d",10);
*******************************************************************************/
int BLEprintf(const char *fmt,...)
{
        char printf_buf[256];
        va_list args;
        int printed;

        va_start(args, fmt);
        printed = vsprintf(printf_buf, fmt, args);
        va_end(args);

        BLE_LOG(printf_buf);

        return printed;
}




