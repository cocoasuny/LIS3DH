#include "stm32l1xx.h"
#include "NTC.h"
#include "platform.h"

/*****************************************************************************************************/
/*******   NTC_RES_BRIDGE_R1*NTC_RES_BRIDGE_R3*NTC_VOLTAGE_REF - R1*V*(NTC_RES_BRIDGE_R2+NTC_RES_BRIDGE_R3)   ****************/
/*******RT=----------------------------****************/
/*******    (NTC_RES_BRIDGE_R2+NTC_RES_BRIDGE_R3)*V + NTC_RES_BRIDGE_R2*NTC_VOLTAGE_REF        ****************/

#define NTC_VOLTAGE_REF  (1225)    //1225mV
#define NTC_RES_BRIDGE_R1    (1.5)     //K
#define NTC_RES_BRIDGE_R2    (1.5)     //K
#define NTC_RES_BRIDGE_R3    (2.2)     //K

#define NTC_R_REF      NTC_RES_BRIDGE_R1 * NTC_RES_BRIDGE_R3 * NTC_VOLTAGE_REF  
#define NTC_R         (NTC_RES_BRIDGE_R2 + NTC_RES_BRIDGE_R3)
#define NTC_VREF_R2    (NTC_VOLTAGE_REF * NTC_RES_BRIDGE_R2)

#define NTC_DAT_BUF_LEN 		(15)


/* 	NTC LUT address definition 	*/
static float32_t * Tempurature=(float32_t *)NTC_Tem_ADDRESS;
static float32_t * Rt_ref=(float32_t *)NTC_Rt_ref_ADDRESS;

/* 	STATIC Varibles for calculate 	*/
static float32_t 	NTC_Tem[NTC_DAT_BUF_LEN] = {0};
static uint8_t 		NTCTemCnt=0;     //NTC�¶Ȳ���������15����ȥ��ǰ5����ȡ���ֵ
static uint8_t 		NTCValue_Valid=InValid;  //NTC�����Ч״̬
		
static bool NTCSampleComplate_Flag=false;//NTC �ɼ���ɱ�־λ���ڼ�ʱ�ɼ��в���Ҫ
static uint8_t NTCGetStableResultCnt=0;    //NTC�ɼ��ȶ���ֵ����

/* 	NTC timer 		*/
static Timer_ID_Typedef     gNTCTIMID = TIMER_ERROR;

/* NTC submodule's status */
static SubModuleStat m_NTC_Submodule_stat = SUB_MOD_STOP;

/* 	NTC related global varibles 	*/
uint8_t   NTCDisplayCtl=0;         //NTC�������������ʾ��������


float32_t Temp_Resistance_calc(float32_t Vrx)
{
	float32_t RT=0;
	Vrx = NTC_R * Vrx;
	
	RT= (NTC_R_REF - NTC_RES_BRIDGE_R1*Vrx)/(Vrx + NTC_VREF_R2);
	return RT;	
}
float32_t Temp_Lookup(float32_t Rt) 
{
	uint16_t num=0;
	float32_t Tamb=0;
	float32_t Mid=0;
	
	for(num=0;num<NTC_LUT_LEN;num++)
	{
		if(Rt >= Rt_ref[num])
		{
			break;
		}
	}
	
	if(num >= NTC_LUT_LEN - 1)
	{
		Tamb = Tempurature[NTC_LUT_LEN - 1];
	}
	else
	{
		Mid = (Rt_ref[num] - Rt_ref[num+1])/2;
		if((Rt-Rt_ref[num]) <= Mid)
		{
			Tamb = Tempurature[num];
		}
		else
		{
			Tamb = Tempurature[num+1];
		}
	}
	return Tamb;
}
/*******************************************************************************
* Function Name  : NTC_Stat_Set
* Description    : ����NTC״̬
* Input          : newState
* Output         : None
* Return         : None
*******************************************************************************/
static void NTC_Stat_Set(SubModuleStat newState)
{
	m_NTC_Submodule_stat = newState;
}
/*******************************************************************************
* Function Name  : NTC_Stat_Get
* Description    : ��ȡNTC״̬
* Input          : None
* Output         : NTC State 
* Return         : None
*******************************************************************************/
SubModuleStat NTC_Stat_Get(void)
{
	return(m_NTC_Submodule_stat);
}
/*******************************************************************************
* Function Name  : NTCSample
* Description    : ��ʼ��ȡ����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NTCSample(Timer_ID_Typedef TIMID)
{
		TS_SendEvent(gTsNTCTaskID_c,gNTCEventReadValue); 
}
/*******************************************************************************
* Function Name  : NTC_Start
* Description    : NTC_Start,���붨ʱ�����ڶ�ʱ���ж��в���NTC��ʼ�����¼�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NTC_Start(void)
{
		TIM_Cfg_Typedef         Tim_Cfg_NTC_Index;		//NTC timer���ã����ڿ���NTC����Ƶ��
		TIM_Basic_Cfg_Typedef 	Tim_Cfg_NTC;       		//NTC timer���ã����ڿ���NTC����Ƶ��
	
		/* Set NTC Module state to RUN */
		NTC_Stat_Set(SUB_MOD_RUN);
		NTCValue_Valid = InValid;
		NTCDisplayCtl = 1;
		NTCTemCnt = 0;  
		/* Init NTC_VOLTAGE_REF Out GPIO */
		VrefOut_Init();
		VrefOut(ENABLE);    //���Vref 1.225V
	
		/* ����NTC timer,����NTCSampleTime���в��� */
		Tim_Cfg_NTC.enuTimerType = TIM_TYPE_MS;
		Tim_Cfg_NTC.u16TimePeriod = NTCSampleTime;
		Tim_Cfg_NTC.NVIC_IRQChannelPreemptionPriority = NTC_TIMPreemptionPriority;
		Tim_Cfg_NTC.pIntCallBack = NTCSample;
	
		/* Init timer top define */
		Tim_Cfg_NTC_Index.TimerMode 			= TIM_MODE_BASIC;
		Tim_Cfg_NTC_Index.TimerBasicCfg 		= &Tim_Cfg_NTC;
		Tim_Cfg_NTC_Index.TimerPWMCfg 			= NULL;

        if(gNTCTIMID != TIMER_ERROR)   //�ѷ���Timer
        {				
            Timer_Free(gNTCTIMID);    //�������������ʱ��
            gNTCTIMID = TIMER_ERROR;
        } 
		gNTCTIMID = Timer_Allocate(&Tim_Cfg_NTC_Index);
		//Clear_Timer_Cnt(gNTCTIMID);
		Start_Timer_Cnt(gNTCTIMID);
		
		/* �����¶Ȳ���ʱ������������ */
		TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStart);  //start wear detect event		

		#ifdef NTC_PRTF_TOP_LEVEL
			printf("NTC Allocate Timer For Sample=%d\r\n",gNTCTIMID);
		#endif
}
/*******************************************************************************
* Function Name  : NTCReadValue
* Description    : NTCReadValue,��ȡADS1115ת�����,��ͨ��OLED��ʾ���
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NTCReadValue(void)
{
		uint16_t NTC_D=0;       //���� TPIS����
		uint8_t i=0;
		int16_t  ADCVal=0;
		float32_t   Tem=0;  
		float32_t   Rt=0;
		float32_t   V_SENS=0;
		uint8_t NTC_SaveData[14] = {0};
//		uint8_t TestData[13];
		uint32_t  WriteEEPROMTimeOutCnt=80000;    //дEEPROM��ʱ
		uint8_t   Err_Code=0;	

		float32_t 	MaxTem=0;

		bool flagAdcSampleValid = FALSE;

		#ifdef BOARD_EVM_V1_0
		uint8_t NTC_Tem_TenDigit=0; //���� TPIS ʮλ
		uint8_t NTC_Tem_SingleDigit=0;//���� TPIS ��λ
		uint8_t NTC_Tem_Tenth=0; //����TPIS ʮ��λ
		#endif

		date_str_typedef    date_s;               //RTC ����
		RTC_TimeTypeDef     rtc_time;             //RTC ʱ��

		if(OFF == GetPeriphPowerStatus())  //�����ⲿ��Դ
		{
			PeriPower(ON);
		}
		
		ADS1118_Init();
		flagAdcSampleValid = ADS1118_GetVal(ADS1118Start | DifferentialCH2_3, &ADCVal);
		Delay_ms(30);
		ADS1118_DeInit();
		
		if(flagAdcSampleValid==FALSE)
		{
			//printf("spi error \r\n");
			return;
		}
		
		V_SENS= (float32_t)(ADCVal * 0.03125);//FS1 = FS/32768
		//V_SENS= (float)(ADCVal * 0.015625);//FS05 = FS/32768
		//V_SENS= (float)(ADCVal * 0.0078125);//FS02= FS/32768
		//V_SENS= (float)(ADCVal * 0.0625);//FS2 = FS/32768
		//printf("ADCVal = %x \r\n",ADCVal);
		//printf("V_SENS=%0.4f\r\n",V_SENS);
		Rt= Temp_Resistance_calc(V_SENS)-1;  //RedHare 2.0Ӳ���ϼ�1K�ĵ���ֵ
		//printf("RT = %0.3f\r\n",Rt);
		
		/* ��ȡNTC�¶�ֵ */
		Tem=Temp_Lookup(Rt);
		#ifdef NTC_PRTF_TOP_LEVEL
			printf("Get Temperature = %0.1f\r\n",Tem);
		#endif
		NTC_Tem[NTCTemCnt]=Tem;
		
		NTC_D= (uint8_t)Tem;
			
		
		gTemVal.NTC_Tem_Tenth = (uint8_t)(NTC_D / 10);   //�����¶�ʮλ
		gTemVal.NTC_Tem_SingleDigit = (uint8_t)(NTC_D % 10);   //�����¶ȸ�λ
		gTemVal.NTC_Tem_TenDigit = (uint8_t)((Tem - NTC_D)*10);      //�����¶�ʮ��λ
			
		Calendar_Get(&date_s,&rtc_time);
		
		gTemVal.date_s.year = date_s.year;
		gTemVal.date_s.month = date_s.month;
		gTemVal.date_s.day = date_s.day;
		gTemVal.rtc_time.RTC_Hours = rtc_time.RTC_Hours;
		gTemVal.rtc_time.RTC_Minutes = rtc_time.RTC_Minutes;
		gTemVal.rtc_time.RTC_Seconds = rtc_time.RTC_Seconds;
		
		
		//printf("NTCTemP[%d]=%f\r\n",NTCTemCnt,NTC_Tem[NTCTemCnt]);
		
		if((gSubFunc_Stat_Get(NTC_SingleWork_State) != OFF)  || (gSubFunc_Stat_Get(NTC_MonitorTemplate_State) != OFF) || 
			((gSubFunc_Stat_Get(NTC_RealTimeSample_State) != OFF)&&(NTC_RealTimeSample_GetResultNum != 0)) ||
			((MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID) && (true == isFreeRunKickOff()))
		   )  //�����ɼ�15������ȥ��ǰ5����ȡ���ֵ
		{
			NTCTemCnt++;
			if(NTCTemCnt >= NTC_DAT_BUF_LEN) 
			{
				NTCTemCnt=0;
				MaxTem=0;
				for(i=5;i < NTC_DAT_BUF_LEN;i++)
				{
					if(NTC_Tem[i]>MaxTem)
					{
						MaxTem = NTC_Tem[i];
						NTC_Tem[i]=0;
					}
				}
				gTemVal.NTCVal = MaxTem;     //���������¶�ֵ
				NTC_D= (uint16_t)(MaxTem*10);
				gTemVal.NTC_Tem_Tenth = (uint8_t)(NTC_D / 100);   //�����¶�ʮλ
				gTemVal.NTC_Tem_SingleDigit = (uint8_t)((NTC_D % 100)/ 10);   //�����¶ȸ�λ
				gTemVal.NTC_Tem_TenDigit = (uint8_t)(NTC_D % 10);      //�����¶�ʮ��λ
				NTCValue_Valid =Valid;
				
				if((Device_Mode == Device_Mode_NTC) && (gSubFunc_Stat_Get(NTC_SingleWork_State) != OFF))  //���豸����ģʽ�²�����������ʾ�������������ʾ
				{
					if(NTC_RealTimeSample_GetResultNum != 0u)
					{
						//StartMotor(1);
						gSubFunc_Stat_Set(NTC_SingleWork_State, OFF);
						TS_SendEvent(gTsNTCTaskID_c,gNTCEventStop);  //���豸��������������ֹͣ�����¼�
						TS_SendEvent(gOledDisTaskID,gOledDisEventNTCResult_c);   //NTC����������������ʾ�������
						
						if((gTemVal.NTCVal < gTemVal.VLowLimit) || (gTemVal.NTCVal > gTemVal.VHigLimit))
						{
							AlarmLevelSet(ALARM_LEVEL_HIGH);		
							TS_SendEvent(gTsAlarmTaskID_c,gAlarmEventParameterLimit); 
						}
						else if((gTemVal.NTCVal < gTemVal.LowLimit) || (gTemVal.NTCVal > gTemVal.HigLimit))
						{
							AlarmLevelSet(ALARM_LEVEL_LOW);	
							TS_SendEvent(gTsAlarmTaskID_c,gAlarmEventParameterLimit); 
						}
					}
				}
				
				if(gSubFunc_Stat_Get(NTC_MonitorTemplate_State) != OFF)   //���ģ������������£�������ɺ����
				{
					NTCSampleComplate_Flag = true;
				}
				if((gSubFunc_Stat_Get(NTC_RealTimeSample_State) != OFF) && (NTC_RealTimeSample_GetResultNum != 0))  //��ʱ�ɼ�����£�Appֻ��õ��ȶ����
				{					
					if((Device_State == Device_WorkWithAPP) && ((Device_Mode == Device_Mode_CheckUp) 
						||(Device_Mode == Device_Mode_IncomingCall)))
					{
						NTCGetStableResultCnt++;   //��ʱ�ɼ�����£�Appֻ��õ��ȶ��������
						
						if(NTCGetStableResultCnt <= NTC_RealTimeSample_GetResultNum)  //���ڵ���AppҪ��õ����ȶ����ݸ���֮����
						{
							TS_SendEvent(gTsSPITranslateTaskID_c,gSPITranslateEventTxNTC);
							if(NTCGetStableResultCnt == NTC_RealTimeSample_GetResultNum)
							{
								//NTC��ʱ�ɼ����
								NTCGetStableResultCnt = 0;
								gSubFunc_Stat_Set(NTC_RealTimeSample_State ,OFF);
								TS_SendEvent(gTsNTCTaskID_c,gNTCEventStop);   //���ģ�����н���������ֹͣ���ݲɼ��¼�
								ChangeDeviceMode_FromCheckUpToRTC();
							}
						}
					}
					else
					{
						NTCGetStableResultCnt = 0;
					}
				}
//				else if((gSubFunc_Stat_Get(NTC_RealTimeSample_State) != OFF) && (NTC_RealTimeSample_GetResultNum == 0))
//				{
//					if((Device_State == Device_WorkWithAPP) && (Device_Mode == Device_Mode_CheckUp))
//					{
//						TS_SendEvent(gTsSPITranslateTaskID_c,gSPITranslateEventTxNTC);
//					}
//				}
			}
		}

		if((gSubFunc_Stat_Get(NTC_SingleWork_State) != OFF) && (NTCValue_Valid == Valid) &&(NTC_RealTimeSample_GetResultNum == 0u))
		{
			//��������ģʽ�£����豸����ʱ��NTCֵ��Ч����ʵʱ��ʾNTC���
			if(NTCDisplayCtl == 1)
			{
				NTCDisplayCtl = 0;
				OLED_DisplayNTCMeasureFlash(OFF);   //�ر�NTC���������ж���
				OLED_DisplayClear();
				OLED_DisplayFullScreenBMP(NTCMeasureResult);
			}
			TS_SendEvent(gOledDisTaskID,gOledDisEventNTCResultContinuous_c);   //NTC��������������������ʾ�������
		}
//		if(Device_State == Device_Alone)  //���豸����ʱ����ʾ�������
//		{
//			if(gSubFunc_Stat_Get(NTC_SingleWork_State) != OFF)   //�������豸����
//			{

//				#ifdef BOARD_REDHARE_V3_0
//				
//					OLED_DisplayChar(7,10,gTemVal.NTC_Tem_Tenth);
//					OLED_DisplayChar(8,10,gTemVal.NTC_Tem_SingleDigit);
//					OLED_DisplayChar(9,10,23);
//					OLED_DisplayChar(10,10,gTemVal.NTC_Tem_TenDigit);
//					
//				#endif
//			}
//		}
		else //work with APP(��ʱ�ɼ�),���Ͳ������
		{
			if((gSubFunc_Stat_Get(NTC_RealTimeSample_State) != OFF) && (Device_Mode == Device_Mode_CheckUp)) //��ʱ�ɼ�ģʽ��&&��ʱ�ɼ�״̬����&&��App������һ��
			{    
				if(NTC_RealTimeSample_GetResultNum == 0)  //�κ����ݶ�����
				{
					TS_SendEvent(gTsSPITranslateTaskID_c,gSPITranslateEventTxNTC);
				}			
			}
		}
		/* NTC ���ģ�������� */
		if(gSubFunc_Stat_Get(NTC_MonitorTemplate_State) != OFF)  //NTC���ģ��������
		{
			/* ������ģ��������EEPROM */
			if(NTCSampleComplate_Flag == true)
			{
				NTC_SaveData[0]=(uint8_t)(date_s.year/100);
				NTC_SaveData[1]=(uint8_t)(date_s.year%100);
				NTC_SaveData[2]=date_s.month;
				NTC_SaveData[3]=date_s.day;
				NTC_SaveData[4]=rtc_time.RTC_Hours;
				NTC_SaveData[5]=rtc_time.RTC_Minutes;
				NTC_SaveData[6]=(rtc_time.RTC_Seconds == 35)? 36 : rtc_time.RTC_Seconds;

				NTC_SaveData[7]=TemperatureID;
				if(GetMonitorTemplateType() == NormalMT)  //��������ɼ�������
				{
					NTC_SaveData[8] = NormalMTVal;
				}else if(GetMonitorTemplateType() == ExcetionMT)  //�쳣����ɼ�������
				{
					NTC_SaveData[8] = ExcetionMTVal;
				}
				NTC_SaveData[9]=gTemVal.NTC_Tem_Tenth+0x30;
				NTC_SaveData[10]=gTemVal.NTC_Tem_SingleDigit+0x30;
				NTC_SaveData[11]='.';
				NTC_SaveData[12]=gTemVal.NTC_Tem_TenDigit+0x30;
				NTC_SaveData[13]='#';
				
				/* ���ģ������ʱ�����ɼ������ݺ������App���ӣ����ȱ���������EEPROM�󴥷�һ������ͬ��*/
				/* ���δ��App���ӣ��򽫼��ģ��ɼ����ݴ���EEPROM�У��豸����ʱͬ�� */
				
				/* ����Ч��NTCֵд��EEPROM�� */
				if(GetM95M01State(MONITOR_MODEL_VALUE_ID,M95M01_CAPACITY_SPACE) < sizeof(NTC_SaveData))
				{
						//���ģ������ʣ������ռ�С��д������ݴ�С�������ռ䱨��
						#ifdef EEPROM_DEBUG
							printf("Not Enough Space\r\n");
						#endif
						gAlarmNotEnoughSpace = true;
						//TS_SendEvent(gTsAlarmTaskID_c,gAlarmEventNotEnoughSpace); 
				}
				else
				{
						gAlarmNotEnoughSpace = false;
				}
				WriteEEPROMTimeOutCnt=80000;    //дEEPROM��ʱ
				EEPROM_SPI_Configuration();
				while((WriteEEPROMTimeOutCnt !=0) && (GetM95M01State(MONITOR_MODEL_VALUE_ID,M95M01_IS_BUSY) == false))  //��æ
				{
						WriteEEPROMTimeOutCnt --;
						Delay_ms(10);
						#ifdef Monitor_Template_Debug
						if(WriteEEPROMTimeOutCnt < 2)
						{
							printf("MT Write EEPROM Time out\r\n");
						}
						#endif
				}			
				/* дEEPROM */
				Err_Code = M95M01_Write(MONITOR_MODEL_VALUE_ID,sizeof(NTC_SaveData),NTC_SaveData);
				APP_ERROR_CHECK(Err_Code);
				
				/* ������ģ��ɼ�֮������ݣ����ģ�����н��� */
				gSubFunc_Stat_Set(NTC_MonitorTemplate_State,OFF);
				
				TS_SendEvent(gTsNTCTaskID_c,gNTCEventStop);   //���ģ�����н���������ֹͣ���ݲɼ��¼�
				NTCSampleComplate_Flag = false;
				
				#ifdef Monitor_Template_Debug
				printf("NTC Monitor Template Measure complete\r\n");
				#endif

				
				if((Device_State == Device_WorkWithAPP) && (gSubFunc_Stat_Get(NTC_MonitorTemplate_State) == OFF))   //��App������һ�������м��ģ�����н���������ͬ���¼�
				{
					TS_SendEvent(gTsSyncDataTaskID_c,gSyncDataEventStart);  //������ʼ����ͬ���¼�
					#ifdef SyncData_DEBUG
						printf("Sync Data Start By MT NTC...\r\n");
					#endif
				}
				/* ���ģ�����е�����£������쳣ֵʱ������������ģ�壬�򴥷��쳣���ģ�����У�*/
				/* ������쳣���ģ�壬�����ò������Ϊ�쳣 */
				#ifdef Monitor_Template_Debug
					printf("NTC VLowLimit = %0.2f\r\n",gTemVal.VLowLimit);
					printf("NTC LowLimit = %0.2f\r\n",gTemVal.LowLimit);
					printf("NTC HigLimit = %0.2f\r\n",gTemVal.HigLimit);
					printf("NTC VHigLimit = %0.2f\r\n",gTemVal.VHigLimit);	
					printf("NTC Val  = %0.2f\r\n",gTemVal.NTCVal);
				#endif
				if((gTemVal.NTCVal<gTemVal.LowLimit) || (gTemVal.NTCVal > gTemVal.HigLimit))  //�������õĲ�������ֵ
				{
					#ifdef Monitor_Template_Debug
						printf("NTC Val Beyond Limit = %0.2f\r\n",gTemVal.NTCVal);
					#endif
					if(GetMonitorTemplateType() == NormalMT)  //�������ģ����
					{
							//�����쳣���ģ��ִ��
							#ifdef Monitor_Template_Debug
								printf("NTC Trigger Excetion Monitor Template\r\n");
							#endif
							TS_SendEvent(gTsMonitorTemplatID_c,gExcetionMonitorTemplateEventStart);  //�����쳣���ģ�忪ʼ�¼�
					}else if(GetMonitorTemplateType() == ExcetionMT)  //�쳣���ģ����
					{
							//���ü��ģ��ɼ�����λ�쳣ֵ
							#ifdef Monitor_Template_Debug
								printf("NTC Set MT Measure Val to ExcetionVal\r\n");
							#endif
							SetMonitorTemplateMeaResultType(ExcetionVal);
					}
				}
				#ifdef Monitor_Template_Debug
				else   //�¶Ȳ����������
				{
						printf("NTC  Measure Val Normal\r\n");
				}
				#endif
			}
		}
		
		/* 	Generate temperature low/high alarm 	*/
//		if(
//			(NTCValue_Valid == Valid) 
//		&& (Device_State != Device_WorkWithAPP)
//		&& (gSubFunc_Stat_Get(NTC_SingleWork_State) != OFF)
//		)
//		{			
//			if((gTemVal.NTCVal < gTemVal.VLowLimit) || (gTemVal.NTCVal > gTemVal.VHigLimit))
//			{
//				gAlarmType.AlarmLevel.High = ON;  //�߼��澯
//				gAlarmType.AlarmStatus = ON;			
//				TS_SendEvent(gTsAlarmTaskID_c,gAlarmEventParameterLimit); 
//			}
//			else if((gTemVal.NTCVal < gTemVal.LowLimit) || (gTemVal.NTCVal > gTemVal.HigLimit))
//			{
//				gAlarmType.AlarmLevel.Low = ON;  //�ͼ��澯
//				gAlarmType.AlarmStatus = ON;			
//				TS_SendEvent(gTsAlarmTaskID_c,gAlarmEventParameterLimit); 
//			}
//		}
}
/*******************************************************************************
* Function Name  : NTC_Stop
* Description    : NTC_Stop,�ͷ�NTC Timer
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NTC_Stop(void)
{
		/* Set NTC Module state to Stop */
		NTC_Stat_Set(SUB_MOD_STOP);
		
		NTCValue_Valid = InValid;
		NTCDisplayCtl = 1;
	
		VrefOut(DISABLE);   //�ر�Vref 1.225V���
		Timer_Free(gNTCTIMID);
		gNTCTIMID  = TIMER_ERROR;
	
		/* DeInit Vref Out GPIO */
		VrefOut_DeInit();

		/* Stop wear detect */
                            #ifdef FreeRunMode_Debug
                                printf("stop spo2 9 \r\n");
                            #endif
		TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop);  //stop wear detect event	

		#ifdef NTC_PRTF_TOP_LEVEL
			printf("NTC Stop Measure\r\n");
		#endif
}
/*******************************************************************************
* Function Name  : NTC_Task_Handler
* Description    : NTC_Task_Handler,����NCT����
* Input          : NTC_Event
* Output         : None
* Return         : None
*******************************************************************************/
void NTC_Task_Handler(event_t NTC_Event)
{
		WearDetResult_Typedef  WearDetResult;
		int16_t tmpDat = 0;
	
		#ifdef NTC_EVENT_DEBUG_MODE
		printf("state transfer, new ntc event = %x \r\n", NTC_Event);
		#endif
	
		if(NTC_Event & gNTCEventStart)  
		{		
			if((Device_Mode == Device_Mode_LowBattery) || (Device_Mode == Device_Mode_Charge)
				|| (Device_Mode == Device_Mode_FWUpdate))
			{
				return;
			}
			if(NTC_Stat_Get() == SUB_MOD_STOP)  
			{
				if((gSubFunc_Stat_Get(NTC_SUBFUNC_ALL_STATE) != OFF) 
					|| ((MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID) && (true == isFreeRunKickOff())))
				{
					if(Device_Mode != Device_Mode_Charge && Device_Mode != Device_Mode_LowBattery )
						/* not work when in low battery and charge and already run */
					{
						NTC_Start();   //����NTC���ö�ʱ��
						/* 	Set fix number flag 		*/
						if(
							NTC_RealTimeSample_GetResultNum != 0
							&& (
								gSubFunc_Stat_Get(NTC_SingleWork_State) != OFF
								|| gSubFunc_Stat_Get(NTC_RealTimeSample_State) != OFF
								)
							)
						{
							flagIsNTCFixNumberSampleKeyPress = FALSE;
						}
						else
						{
							flagIsNTCFixNumberSampleKeyPress = TRUE;
						}
					}
				}
			}
		}
		else if(NTC_Event & gNTCEventReadValue)
		{
				if((Device_Mode == Device_Mode_LowBattery) || (Device_Mode == Device_Mode_Charge)
				|| (Device_Mode == Device_Mode_FWUpdate))
				{
					return;
				}
				Wear_State_Get(&WearDetResult); //��ȡ��������
				
				/* when in free run time, cancel the wear det */
				if(MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID)  // in free run 
				{
					if(true == isFreeRunKickOff())
					{
						WearDetResult.wearState = WEAR_GOOD;
						WearDetResult.wearDetState = WEAR_DET_FINISH;
					}
				}
				
				if(WearDetResult.wearDetState == WEAR_DET_FINISH) //���������
				{	
					if(WearDetResult.wearState == WEAR_GOOD) //�����
					{
						#ifdef NTC_PRTF_TOP_LEVEL
							printf("NTC Mea Wear Good\r\n");
						#endif
						NTCReadValue();   //NTC��ȡ�������ݣ����¼���ADS1115 INT�д���
					}
					else if(WearDetResult.wearState == WEAR_BAD) //û�������
					{
						#ifdef NTC_PRTF_TOP_LEVEL
							printf("NTC Mea Wear Bad\r\n");
						#endif
						
						//���������Ѿ������������ȷ�澯��App
						//��ʾ�������ȷ����
						if((Device_Mode == Device_Mode_NTC) && (gSubFunc_Stat_Get(NTC_SingleWork_State) != OFF))  //���豸����ģʽ���������ȷ��ʾ��ʾ����
						{
							TS_SendEvent(gOledDisTaskID,gOledDisEventDisplayWearBad_c);   //�����������ȷ��ʾ����
						}
						//ֹͣNTC���вɼ�
						gSubFunc_Stat_Set(NTC_SUBFUNC_ALL_STATE,OFF);

						TS_SendEvent(gTsNTCTaskID_c,gNTCEventStop); //ֹͣNTC����
					}						
				}
				else  //��������
				{
						//����ADS1118ת����������ת�����
						if(OFF == GetPeriphPowerStatus())  //�����ⲿ��Դ
						{
							PeriPower(ON);
						}
						ADS1118_Init();
						ADS1118_GetVal(ADS1118Start | DifferentialCH2_3, &tmpDat);
						ADS1118_DeInit();
					
						#ifdef NTC_PRTF_TOP_LEVEL
							printf("NTC Mea Wear Det Not Finish\r\n");
						#endif
				}	
		}
		else if(NTC_Event & gNTCEventStop)
		{
			if(NTC_Stat_Get() == SUB_MOD_RUN)  //stop NTC module unless the NTC state is Run
			{
				if(gSubFunc_Stat_Get(NTC_SUBFUNC_ALL_STATE) == OFF)
				{
					NTC_Stop();       //ֹͣNTC�������ͷ�����Ķ�ʱ��
					NTCTemCnt = 0;    //ֹͣNTC������NTC������������
					if(NTC_Measure_State == Start)
					{
						NTC_Measure_State = Stop;
					}
				}
			}
		}
}
/*******************************************************************************
* Function Name  : VrefOutConfig
* Description    : VrefOut_Init,�������Vref Out���ṩNTC�ο���ѹ
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void VrefOut_Init(void)
{
		GPIO_InitTypeDef        GPIO_InitStructure;
		
		/* Power On */
		RCC_AHBPeriphClockCmd(VrfOut_RCC_Periph, ENABLE);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_VrfOut;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(GPIO_VrfOut, &GPIO_InitStructure);
	
		GPIO_ResetBits(GPIO_VrfOut,GPIO_Pin_VrfOut);
}
/*******************************************************************************
* Function Name  : VrefOut_DeInit
* Description    : VrefOut_DeInit,�������Vref Out���ṩNTC�ο���ѹ
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void VrefOut_DeInit(void)
{
		GPIO_InitTypeDef        GPIO_InitStructure;

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_VrfOut;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
        GPIO_Init(GPIO_VrfOut, &GPIO_InitStructure);
}
/*******************************************************************************
* Function Name  : VrefOut
* Description    : VrefOut,ʹ�ܻ��ֹVrefOut
* Input          : Status
* Output         : None
* Return         : None
*******************************************************************************/
void VrefOut(uint8_t Status)
{
		if(Status == ENABLE)
		{
			GPIO_SetBits(GPIO_VrfOut,GPIO_Pin_VrfOut);
		}
		else if(Status == DISABLE)
		{
			GPIO_ResetBits(GPIO_VrfOut,GPIO_Pin_VrfOut);
		}
}


void ADS1118_SelfTest(void)
{
	int16_t  ADCVal=0;
	float32_t   V_SENS=0;
	float32_t   Rt=0;
	
	if(OFF == GetPeriphPowerStatus())  //�����ⲿ��Դ
	{
		PeriPower(ON);
	}
	
	VrefOut_Init();
	VrefOut(ENABLE);    //���Vref 1.225V
	
	ADS1118_Init();
	ADS1118_GetVal(ADS1118Start | DifferentialCH2_3, &ADCVal);
	Delay_ms(1000);
	ADS1118_GetVal(ADS1118Start | DifferentialCH2_3, &ADCVal);
	ADS1118_DeInit();
	
	V_SENS= (float32_t)(ADCVal * 0.03125);//FS1 = FS/32768
	Rt= Temp_Resistance_calc(V_SENS)-1;  //RedHare 2.0Ӳ���ϼ�1K�ĵ���ֵ
		
	if((Rt < Rt_ref[0]) && (Rt > Rt_ref[NTC_LUT_LEN-1]))
	{
		SelfCheckLCDCtl(ID_ADS1118Status,CMD_Pass);
	}
	else
	{
		SelfCheckLCDCtl(ID_ADS1118Status,CMD_Fail);
	}
	
	VrefOut(DISABLE);   //disable 1.225V voltage output
}


