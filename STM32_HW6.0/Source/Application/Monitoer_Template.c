#include "Monitoer_Template.h"
#include <string.h>
#include <stdlib.h>
#include "common.h"
Monitor_Template_Typedef       MonitorTemplate;
MonitorTemplateType_Typedef    gMT_Type=NormalMT;
static  MonitorTemplateMeaResult_Typedef gMTMeaSult = NormalVal;
static Excetion_Monitor_Template_Typedef  ExcetionMonitorTemplate;
static  uint8_t Flag_Excetion_MTRecovery = true;
static bool flagIsFreeRunKickOff = false;
static bool flagLightMonitorTemplateStatus = OFF;  //ON�����죬OFF��ҹ��
static bool bFlagMonitorTemplateIsRunning = false;
static bool bFlagNewDayEvent = FALSE;               // 00:00Event flag
static bool bFlagNewLightStart = FALSE;
static char OldTemplate[MaxMTDataLen + 1] = {0};
int32_t FindNextAlarmTimePoint(uint32_t u32CurrTime,uint32_t * pu32AlarmHH,uint32_t * pu32AlarmMM,uint8_t * pu8Flag);
static bool GetLightMonitorTemplateStatus(void);
static void SetFreeRunMTCtlTime(uint16_t CurTime);
static void FreeRunMTChangeToNinghtMT(uint16_t CurTime);
static uint8_t AdjustMT(uint16_t CurTime, uint16_t NextTime);
static void FreeRunMTSetLightCtlTime(uint16_t CurTime);


static void FreeRun_SampleData_Storage_Callback(void);


static void StartExcetionMonitorTemplate(void);
static void StopExcetionMonitorTemplate(void);
static void SendExcetionMonitorTemplateStopFram(void);
static void FreeRun_SampleData_StorageFunc(void);
static void FreeRunModeInit(void);
static void FreeRunModeDeInit(void);


bool isFreeRunKickOff(void)
{
	return flagIsFreeRunKickOff;
}

/* Parse S into tokens separated by characters in DELIM.
   If S is NULL, the saved pointer in SAVE_PTR is used as
   the next starting point.  For example:
        char s[] = "-abc-=-def";
        char *sp;
        x = strtok_r(s, "-", &sp);      // x = "abc", sp = "=-def"
        x = strtok_r(NULL, "-=", &sp);  // x = "def", sp = NULL
        x = strtok_r(NULL, "=", &sp);   // x = NULL
                // s = "abc\0-def\0"
*/
char *strtok_r(char *s, const char *delim, char **save_ptr)
{
    char *token;

    if (s == NULL)
	{
		s = *save_ptr;
	}

    /* Scan leading delimiters.  */
    s += strspn(s, delim);
    if (*s == '\0')
	{
		return NULL;
	}

    /* Find the end of the token.  */
    token = s;
    s = strpbrk(token, delim);
    if (s == NULL)
	{
        /* This token finishes the string.  */
        *save_ptr = strchr(token, '\0');
	}
    else
	{
        /* Terminate the token and make *SAVE_PTR point past it.  */
        *s = '\0';
        *save_ptr = s + 1;
    }

    return token;
}

/*******************************************************************************
* Function Name  : UnPackMonitorTemplate
* Description    : ������ģ������
* Input          : p_MonitorTemplate,len
* Output         : None
* Return         : None
*******************************************************************************/
void UnPackMonitorTemplate(const uint8_t *p_MonitorTemplate,uint16_t Len)
{
		uint16_t i=0;
		uint8_t 	str_len_t=0;

		uint8_t         TempH1=0;
		uint8_t         TempL1=0;
		uint8_t         TempH2=0;
		uint8_t         TempL2=0;

		//char buffer[50]="Fred male 25,John male 62,Anna female 16";
		char *buf=NULL;
		char Template[MaxMTDataLen+1]={0};
		char *p[20]={0};
		char *inner_ptr=NULL;
		char *outer_ptr=NULL;
		uint16_t        in=0;
        SysConfigInfo_t DeviceCFInfo;    //Device configure information
        uint8_t  MTID = kICTMonitorTemplateNone;

		#ifdef Monitor_Template_Debug
            printf("Unpack MonitorTemplate\r\n");
            for(i=0;i<Len;i++)
            {
                printf("%c",p_MonitorTemplate[i]);
            }
            printf("\r\n");
		#endif

		for(i=0;i<(Len);i++)
		{
			Template[i]=p_MonitorTemplate[i];
		}

        if(0 == memcmp(OldTemplate, Template, sizeof(Template)))
        {
            #ifdef Monitor_Template_Debug
				printf("***The same template!!!\r\n");
			#endif
            return ;
        }
        memcpy(OldTemplate, Template, sizeof(Template));

		Template[i] = '\0';		/* 	Add the end delimt 		*/

		buf=Template;

		while((p[in] = strtok_r(buf, "#", &outer_ptr))!=NULL)   //��ȡ���ģ��
		{
			buf=p[in];
			while((p[in]=strtok_r(buf, "|", &inner_ptr))!=NULL)  //
			{
				in++;
				buf=NULL;
			}
			in++;
			buf=NULL;
		}

		#ifdef Monitor_Template_Debug
		for(i=0;i<in-1;i++)   //��ӡ�ָ��ļ��ģ��
		{
			printf("p[%d]= %s,len=%d\r\n",i,p[i],strlen(p[i]));
		}
		#endif

		MonitorTemplate.SampleFreq = atoi(p[0]);  //p[0]������Ƶ��
		MonitorTemplate.SampleWorkDay = 0x00;     //p[2]����Workday
		for(i=0;i<7;i++)
		{
			if(p[2][i] == 0x31)
			{
				MonitorTemplate.SampleWorkDay |= (0x80 >> (i+1));
			}
		}
		MonitorTemplate.SampleLastTime = atoi(p[3]); //p[3]����ÿ�γ���ʱ��
		MonitorTemplate.SampleID = 0x0000;

		str_len_t = strlen(p[4]);
		for(i = 0;i < str_len_t;i++)                  //p[4]����ɼ�����ID
		{
			if(p[4][i] == 0x31)
			{
				MonitorTemplate.SampleID |= (0x0001 << (str_len_t-i-1));
			}
		}
		ExcetionMonitorTemplate.SampleID = 0x0000;
		str_len_t = strlen(p[5]);
		for(i = 0;i < str_len_t;i++)                  //p[5]�����쳣�ɼ�����ID
		{
			if(p[5][i] == 0x31)
			{
				ExcetionMonitorTemplate.SampleID |= (0x0001 << (str_len_t-i-1));
			}
		}

		ExcetionMonitorTemplate.SampleLastTime = atoi(p[6]); //p[6]�����쳣�����ɼ�ʱ��
		ExcetionMonitorTemplate.SampleFreq = atoi(p[7]);   //p[7]�����쳣�ɼ�Ƶ��
		MonitorTemplate.MTSwitch = atoi(p[8]);             //p[8]������ģ�忪?
		MonitorTemplate.VibrateSwitch = atoi(p[9]);        //p[9]�񶯿���

		if((in-1) > 10) //���ݼ��ģ����δ���˶���ʷ�ɼ�Ƶ�ʼ����ģ��ID��ʾ
		{
			MonitorTemplate.SetMTID = atoi(p[10]);  //p[10]������ģ��ID
			MonitorTemplate.MotionInfoUpdateFreq = atoi(p[11]); //p[11]�����˶����ݸ���Ƶ��
            //p[12]:����ɼ�ʱ���
            MonitorTemplate.LightMTinfo.LightSampleFreq = atoi(p[13]);   //p[13]�������ɼ�Ƶ��
		}
		else   //��ǰ�汾App��δ�ӣ���Ĭ��
		{
			MonitorTemplate.SetMTID = kICTMonitorTemplateAltitudeStressID;  //Ĭ�ϼ��ģ��ID
			MonitorTemplate.SampleID = 7;
			ExcetionMonitorTemplate.SampleID = 7;
			MonitorTemplate.MotionInfoUpdateFreq = 900; //Ĭ���˶����ݸ���Ƶ��
            MonitorTemplate.LightMTinfo.LightSampleFreq = 0;
		}

        /* Change kICTMonitorTemplateCOPD/kICTMonitorTemplateHypoxemia/kICTMonitorTemplateOSA to kICTMonitorTemplateFreeRunID */
        if((MonitorTemplate.SetMTID == kICTMonitorTemplateCOPD)
        || (MonitorTemplate.SetMTID == kICTMonitorTemplateHypoxemia)
        || (MonitorTemplate.SetMTID == kICTMonitorTemplateOSA)
        || (MonitorTemplate.SetMTID == kICTMonitorTemplateSleepApneaManagerID)
        || (MonitorTemplate.SetMTID == kIctmonitorTemplateNightLowSpo2ManagerID))
        {
            MonitorTemplate.MTID = kICTMonitorTemplateFreeRunID;
        }
        else
        {
            MonitorTemplate.MTID = MonitorTemplate.SetMTID;
        }

        if((MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID) &&
            MonitorTemplate.SampleFreq > kICTMonitorTemplateFreeRunMaxSampleFreq
            )
        {
             MonitorTemplate.SampleFreq = 1;  //set the OSAS SampleFreq to default
        }

		//MonitorTemplate.SampleFreq=360;   //for test
		//p[1]="10:00-10:45,17:00-18:04";  //for test
		MonitorTemplate.SampleTimeClock.DataLen = (strlen(p[1]) / 3)+1;

		if(MonitorTemplate.SampleTimeClock.DataLen > MaxMTLen)  //���յļ��ģ�峤�ȴ��������ģ�峤��
		{
			#ifdef Monitor_Template_Debug
				printf("Err1:  Receive MT Len Beyond MaxMTLen\r\n");
			#endif
			return ;
		}
		memset(MonitorTemplate.SampleTimeClock.Context,0,MaxMTLen*sizeof(uint8_t));
		if(MonitorTemplate.SampleFreq == 0)//��ʱ���ɼ�
		{
			for(i=0;i<(MonitorTemplate.SampleTimeClock.DataLen);i++)        //p[1]������ÿ��ʱ���
			{
				MonitorTemplate.SampleTimeClock.Context[i]=(uint8_t)(((p[1][i*3])-0x30)*10 + (p[1][i*3+1]-0x30));
			}
		}
		else  //��ʱ��βɼ�,���仮��Ϊʱ���
		{
			memset(MonitorTemplate.ClockArr,0,MaxClockArrLen*sizeof(uint8_t));
			if(MonitorTemplate.SampleTimeClock.DataLen > MaxClockArrLen)  //���յļ��ģ�峤�ȴ��������ģ�峤��
			{
				return ;
			}

			for(i=0;i<(MonitorTemplate.SampleTimeClock.DataLen);i++)        //p[1]������ÿ��ʱ���
			{
				MonitorTemplate.ClockArr[i]=(uint8_t)(((p[1][i*3])-0x30)*10 + (p[1][i*3+1]-0x30));
			}

			if((MonitorTemplate.SampleTimeClock.DataLen/4) == 2)  //�п���
			{
				TempH1 = MonitorTemplate.ClockArr[4];
				TempL1 = MonitorTemplate.ClockArr[5];
				TempH2 = MonitorTemplate.ClockArr[6];
				TempL2 = MonitorTemplate.ClockArr[7];

				MonitorTemplate.ClockArr[4] = MonitorTemplate.ClockArr[0];
				MonitorTemplate.ClockArr[5] = MonitorTemplate.ClockArr[1];
				MonitorTemplate.ClockArr[6] = MonitorTemplate.ClockArr[2];
				MonitorTemplate.ClockArr[7] = MonitorTemplate.ClockArr[3];

				MonitorTemplate.ClockArr[0] = TempH1;
				MonitorTemplate.ClockArr[1] = TempL1;
				MonitorTemplate.ClockArr[2] = TempH2;
				MonitorTemplate.ClockArr[3] = TempL2;
			}

            /*  stop the current FreeRun Monitor, when Monitor Changed  */
            /* 	Read device config file from DataEEPROM 	*/
            GetSysConfigInfo(&DeviceCFInfo);
            MTID = DeviceCFInfo.MTID;

            if(MonitorTemplate.SetMTID != MTID)   //�����໤����
            {
                MTID = MonitorTemplate.SetMTID;

                DeviceCFInfo.MTID = MTID;

                /* 	Write Modify MT ID into DataEEPROM */
                SetSysConfigInfo(DeviceCFInfo);

                /*  Stop while in free run */
                if((MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID) && (true == isFreeRunKickOff()))
                {
                    TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop);			/* 	Stop HR/SPO2 realtime sample 	*/

                    /* 	De-init RTC autowake up interrupt 		*/
                    Calendar_RTC_Period_Wakeup_DeInit();

                    /* 	Set Flag to indicate that FreeRun Mode has stoped 		*/
                    flagIsFreeRunKickOff = false;
                }
            }
		}//end of ��ʱ��βɼ�


        /* Light Monitor Templte info Analysis */
        MonitorTemplate.LightMTinfo.DataLen = (strlen(p[12]) / 3) + 1;
        if(MonitorTemplate.LightMTinfo.DataLen > MaxClockArrLen)  //���յİ���໤�������ȴ�����󳤶�
        {
            #ifdef Monitor_Template_Debug
				printf("Err1:  Receive Light MT Len Beyond MaxMTLen\r\n");
			#endif
			return ;
        }
        memset(MonitorTemplate.LightMTinfo.LightMTClockArr,0,MaxClockArrLen*sizeof(uint8_t));
        if(MonitorTemplate.LightMTinfo.LightSampleFreq == 0)
        {
            //����໤�������Ƶ��Ϊ0����ִ�а��첿��

        }
        else  //�а����ⷽ��
        {
            for(i=0;i<(MonitorTemplate.LightMTinfo.DataLen);i++)   //p[12]�������໤����ʱ���
            {
                MonitorTemplate.LightMTinfo.LightMTClockArr[i] = (uint8_t)(((p[12][i*3])-0x30)*10 + (p[12][i*3+1]-0x30));
            }

            if((MonitorTemplate.LightMTinfo.DataLen/4) == 2)  //�п���
			{
				TempH1 = MonitorTemplate.LightMTinfo.LightMTClockArr[4];
				TempL1 = MonitorTemplate.LightMTinfo.LightMTClockArr[5];
				TempH2 = MonitorTemplate.LightMTinfo.LightMTClockArr[6];
				TempL2 = MonitorTemplate.LightMTinfo.LightMTClockArr[7];

				MonitorTemplate.LightMTinfo.LightMTClockArr[4] = MonitorTemplate.LightMTinfo.LightMTClockArr[0];
				MonitorTemplate.LightMTinfo.LightMTClockArr[5] = MonitorTemplate.LightMTinfo.LightMTClockArr[1];
				MonitorTemplate.LightMTinfo.LightMTClockArr[6] = MonitorTemplate.LightMTinfo.LightMTClockArr[2];
				MonitorTemplate.LightMTinfo.LightMTClockArr[7] = MonitorTemplate.LightMTinfo.LightMTClockArr[3];

				MonitorTemplate.LightMTinfo.LightMTClockArr[0] = TempH1;
				MonitorTemplate.LightMTinfo.LightMTClockArr[1] = TempL1;
				MonitorTemplate.LightMTinfo.LightMTClockArr[2] = TempH2;
				MonitorTemplate.LightMTinfo.LightMTClockArr[3] = TempL2;
			}
        }


		#ifdef Monitor_Template_Debug
			printf("Monitor Template info...\r\n");
			printf("MT_Freq:%d\r\n",MonitorTemplate.SampleFreq);
//			for(i=0;i<MonitorTemplate.SampleTimeClock.DataLen;i++)
//			{
//				printf("MT_STC[%d]=%d\r\n",i,MonitorTemplate.SampleTimeClock.Context[i]);
//			}
//			printf("\r\n");
			printf("MT_WorkDay=%x\r\n",MonitorTemplate.SampleWorkDay);
			printf("MT_LastTime=%d\r\n",MonitorTemplate.SampleLastTime);
			printf("MT_SampleID=%x\r\n",MonitorTemplate.SampleID);
			printf("MT_ErrTime=%d\r\n",MonitorTemplate.SampleErrTime);
			printf("MT_VibrateSwitch=%d\r\n",MonitorTemplate.VibrateSwitch);
			printf("MT_MTID=%d\r\n",MonitorTemplate.MTID);
			printf("MT_MotionInfoUpdateFreq=%d\r\n",MonitorTemplate.MotionInfoUpdateFreq);
		#endif

        bFlagNewDayEvent = FALSE;
		TS_SendEvent(gTsMonitorTemplatID_c,gMonitorTemplateEventStartSet);    //�������ģ�忪ʼ�����¼�

		if(Device_Mode == Device_Mode_Factory)  //����ģʽ�£��¼����Ȼ�������
		{
			MonitorTemplate_Task_Handler(gMonitorTemplateEventStartSet);
		}
		if((Device_Mode == Device_Mode_RTC) && ((Get_OLED_Dis_Status() == OLEDDisEnterShutDown) || (Get_OLED_Dis_Status() == OLEDDisON)))
		{
			//RTC��ʾģʽ�£��յ����ģ������RTC�����еļ��ģ��ͼ��
			if(MonitorTemplate.MTSwitch == 0) //��ͣ���ģ��
			{
				OLED_DisplayMonitorTemplateICON(OFF); 							//clear the Monitor template icon
			}
			else
			{
				OLED_DisplayMonitorTemplateICON(ON); //��ʾ���ģ��
			}
		}
}
/*******************************************************************************
* Function Name  : MonitorTemplate_Task_Handler
* Description    : ������ģ���¼�
* Input          : MonitorTemplate_Event
* Output         : None
* Return         : None
*******************************************************************************/
void MonitorTemplate_Task_Handler(event_t MonitorTemplate_Event)
{
		uint8_t 		Date=0x01;
		uint16_t 		i=0;
		static int16_t 	SampleClock=0;  //�ɼ����ݵ�
		static uint16_t ExcetionSampleClock=0; //�쳣���ģ��ɼ����ݵ�
		date_str_typedef    date_s;               //RTC ����
		RTC_TimeTypeDef     rtc_time;             //RTC ʱ��
		uint16_t        MTTime=0;   //���ģ��ʱ��ת��Ϊ����
		uint16_t        CurTime=0;  //��ǰʱ��ת��Ϊ����
		uint32_t        AlarmHH=0;  //����Сʱ
		uint32_t        AlarmMM=0;  //���ӷ���
//        uint32_t        u32Date = 0;
        uint8_t         u8Flag = 0;

		Calendar_Get(&date_s,&rtc_time);
		CurTime = rtc_time.RTC_Hours*60 + rtc_time.RTC_Minutes;

		if(MonitorTemplate_Event == gMonitorTemplateEventStartSet)   //App����ʱ��ÿ��00��00ʱ�鿴���޵�ǰ���ģ��
		{
			Calendar_Get(&date_s,&rtc_time);

			#ifdef Monitor_Template_Debug
                printf("\r\n***gMonitorTemplateEventStartSet\r\n");
                printf("Current Time:");
                printf("RTC Hour=%d,",rtc_time.RTC_Hours);
                printf("RTC Min=%d\r\n",rtc_time.RTC_Minutes);
                printf("RTC Week=%d\r\n",date_s.week);
			#endif

			if((MonitorTemplate.SampleWorkDay & (Date<<(date_s.week-1))) == (Date<<(date_s.week-1)))  //Work Day���ڵ���
			{
                /* Create The Moniter Template Data Store Partition */
//                u32Date = CovernDateto32();
//                ExtFLASH_ExtCreatePartition(u32Date);

                if(!((TRUE == bFlagNewDayEvent) && (TRUE == bFlagMonitorTemplateIsRunning)))
                {
                    if((MonitorTemplate.MTID != kICTMonitorTemplateFreeRunID) || (true != isFreeRunKickOff()))
    				{
    					DisCalendar_Alarm_A();
    				}
                }

				SetMonitorTemplateMeaResultType(NormalVal);  //���ü��ģ��ɼ����Ϊ����
				SetMonitorTemplateType(NormalMT); //���ü��ģ������Ϊ�������ģ��
				if(MonitorTemplate.SampleFreq == 0) //����ʱ���ɼ�
				{
					/* �ҳ���ǰ���ʱ��� ,ʱ���Ҳ�ڼ��ģ�����ݽ�����ת��Ϊʱ���*/
					for(i=0;i<((MonitorTemplate.SampleTimeClock.DataLen)/2);)
					{
						if(MonitorTemplate.SampleTimeClock.Context[i*2]<rtc_time.RTC_Hours)
						{
							i++;
						}
						else
						{
							break;
						}
					}
					if(MonitorTemplate.SampleTimeClock.Context[i*2] == rtc_time.RTC_Hours)
					{
						while(i<((MonitorTemplate.SampleTimeClock.DataLen)/2))
						{
							//���ģ��ʱ���뵱ǰʱ��ת��Ϊ���ӽ��бȽ�
							MTTime = MonitorTemplate.SampleTimeClock.Context[i*2]*60 + MonitorTemplate.SampleTimeClock.Context[i*2+1];
							CurTime = rtc_time.RTC_Hours*60 + rtc_time.RTC_Minutes;
							if(MTTime < CurTime)
							{
								i++;
							}
							else
							{
								break;
							}
						}
					}
					/* �ҵ���ǰ���ʱ��� */
					SampleClock = i;
					if((SampleClock < (MonitorTemplate.SampleTimeClock.DataLen)/2))  //�ҵ����ʱ���
					{
						/* check if current time equals to first alarm time */
						if(rtc_time.RTC_Minutes == MonitorTemplate.SampleTimeClock.Context[SampleClock*2+1])
						{
							SampleClock++;  //point to next sample point
						}
						if(MonitorTemplate.MTSwitch == 0) //��ͣ���ģ��
						{
							DisCalendar_Alarm_A();
							#ifdef Monitor_Template_Debug
								printf("Suspend MT\r\n");
							#endif
						}
						else  //��ʼ���ģ��
						{
							SetCalendar_Alarm_A(MonitorTemplate.SampleTimeClock.Context[SampleClock*2],MonitorTemplate.SampleTimeClock.Context[SampleClock*2+1],1);
							#ifdef Monitor_Template_Debug
                                printf("First Alarm = %d\r\n",SampleClock);
                                printf("1,First Alarm Time: Hour=%d,Minute=%d\r\n",MonitorTemplate.SampleTimeClock.Context[SampleClock*2],
																			MonitorTemplate.SampleTimeClock.Context[SampleClock*2+1]);
							#endif
						}
						//SetCalendar_Alarm_A(rtc_time.RTC_Hours,rtc_time.RTC_Minutes,rtc_time.RTC_Seconds+1);  //for test
						SampleClock++;  //ָ����һ�����ģ��ʱ��ɼ���
					}
					#ifdef Monitor_Template_Debug
					else
					{
						//������ģ���ѹ�
						#ifdef Monitor_Template_Debug
                            printf("This day monitor template have passed\r\n");
						#endif
					}
					#endif
				}
				else	//����ʱ��βɼ�
				{
					//����ǰϵͳʱ������ģ��ʱ��ν��бȽ�
					CurTime = rtc_time.RTC_Hours*60 + rtc_time.RTC_Minutes;

					if(MonitorTemplate.MTSwitch == 0) //��ͣ���ģ��
					{
						DisCalendar_Alarm_A();
						#ifdef Monitor_Template_Debug
							printf("Suspend MT\r\n");
						#endif
                        if((MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID) && (true == isFreeRunKickOff()))
                        {
                            TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop);	/* 	Stop HR/SPO2 realtime sample 	*/
                            FreeRunModeDeInit();
                        }
					}
					else
					{
						if(MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID)  // Free run
						{
                            flagLightMonitorTemplateStatus = GetLightMonitorTemplateStatus();
                            #ifdef Monitor_Template_Debug
                                printf("Light MT Status:%d\r\n",flagLightMonitorTemplateStatus);
                            #endif

                            if(flagLightMonitorTemplateStatus == ON)//�ڰ��췽������ʱ�����
                            {
                                FreeRunMTSetLightCtlTime(CurTime);
                            }
                            else    //���ڰ��췽������ʱ�����,ҹ�䷽������
                            {
                                SetFreeRunMTCtlTime(CurTime);
                            }
//							/*Set AlarmB to Monitor the status of FreeRunning */
//							SetCalendar_Alarm_B(AlarmHH,AlarmMM,30);
						}
						else    //MonitorTemplate.MTID != kICTMonitorTemplateFreeRunID(���Ի�����)
						{
                            if(true == isFreeRunKickOff())  //Free run is kick off
                			{
                				/* Stop Free Run */
                                #ifdef FreeRunMode_Debug
                                            printf("stop spo2 3 \r\n");
                                #endif
                				TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop);			/* 	Stop HR/SPO2 realtime sample 	*/

                                #ifdef FreeRunMode_Debug
                					printf("deinit free run when change MT \r\n");
                				#endif
                				FreeRunModeDeInit();
                				DisCalendar_Alarm_A();
                			}

                            /* Set AlarmB 00:00,to set MonitorTemplate */
                            SetCalendar_Alarm_B(0,0,10);

							//�����´βɼ�ʱ���
                            if(0 == FindNextAlarmTimePoint(CurTime, &AlarmHH, &AlarmMM, NULL))
                            {
                                if(TRUE == bFlagNewDayEvent)
                                {
                                    bFlagNewDayEvent = FALSE;
                                    if(FALSE == bFlagMonitorTemplateIsRunning)
                                    {
                                        SetCalendar_Alarm_A(AlarmHH,AlarmMM,1);
                                    }
                                    else
                                    {
                                        bFlagMonitorTemplateIsRunning = FALSE;
                                    }
                                }
                                else    // app connect
                                {
        							SetCalendar_Alarm_A(AlarmHH,AlarmMM,1);
                                }
                            }
                            else
                            {
                            }
						}
					}
				}
			}//end of Work Day���ڵ���
			#ifdef Monitor_Template_Debug
			else  //����û�м��ģ�����У����������Ϣ
			{
				#ifdef Monitor_Template_Debug
				printf("There is no monitor template in the day%d",date_s.day);
				#endif
			}
			#endif

            SPI_AlarmTransmit(Alarm_ID_Reuse_MTSetSuccess);  //���ͼ໤�������ý������ʱʹ�ø澯ͨ��

		}//end of if(MonitorTemplate_Event == gMonitorTemplateEventStartSet)   //App����ʱ��ÿ��00��00ʱ�鿴���޵�ǰ���ģ��
		else if(MonitorTemplate_Event == gMonitorTemplateEventSampleData) //���ռ��ģ��ɼ�����
		{
			#ifdef Monitor_Template_Debug
            printf("\r\ngMonitorTemplateEventSampleData\r\n");
			Calendar_Get(&date_s,&rtc_time);
			printf("\r\nThe current time is :  %0.2d:%0.2d:%0.2d\r\n", rtc_time.RTC_Hours, rtc_time.RTC_Minutes, rtc_time.RTC_Seconds);
			#endif

			if(MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID)  //free run
			{
                flagLightMonitorTemplateStatus = GetLightMonitorTemplateStatus();
                if((flagLightMonitorTemplateStatus == OFF) ||
                    ((flagLightMonitorTemplateStatus == ON) && (flagIsFreeRunKickOff == true))) //���ڰ��췽��������,ҹ��������
                {
                    if((MonitorTemplate.SampleTimeClock.DataLen/4) == 2)  //�п���
                    {
                        if(
                            CurTime >= (MonitorTemplate.ClockArr[2]*60 + MonitorTemplate.ClockArr[3])
                            && CurTime < (MonitorTemplate.ClockArr[4]*60 + MonitorTemplate.ClockArr[5])
                        )  //stop free run
                        {
                            #if defined (FreeRunMode_Debug) || (defined Monitor_Template_Debug)
                                printf("cross stop sample in sample, curr time = %d \r\n",CurTime);
                            #endif


                            /* 	De-init the EEPROM for FreeRun Mode 		*/
    //						if(flagIsFreeRunKickOff == true)
    //						{
                                /* 	Overflow the stop time point, then stop the sample 	*/
                                #ifdef FreeRunMode_Debug
                                    printf("stop spo2 4 \r\n");
                                #endif
                                TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop);			/* 	Stop HR/SPO2 realtime sample 	*/
                                #ifdef FreeRunMode_Debug
                                    printf("deinit free run \r\n");
                                #endif
                                FreeRunModeDeInit();
    //						}

                            if(MonitorTemplate.LightMTinfo.LightSampleFreq == 0)
                            {
                                /* reset the start time */
                                AlarmHH = MonitorTemplate.ClockArr[4];
                                AlarmMM = MonitorTemplate.ClockArr[5];
                                #if defined (FreeRunMode_Debug) || (defined Monitor_Template_Debug)
                                    printf("reset alarm = %d, %d\r\n",AlarmHH,AlarmMM);
                                #endif
                                SetCalendar_Alarm_A(AlarmHH,AlarmMM,1);
                            }
                        }
                        else if ((CurTime >= (MonitorTemplate.ClockArr[4]*60 + MonitorTemplate.ClockArr[5]))
                            || (CurTime < (MonitorTemplate.ClockArr[2]*60 + MonitorTemplate.ClockArr[3]))
                                )   // start free run
                        {
                            #if defined (FreeRunMode_Debug) || (defined Monitor_Template_Debug)
                                printf("cross start sample \r\n");
                            #endif
                            /* 	In FreeRun mode: will start realtime sample on each ID 		*/
                            if((Device_Mode != Device_Mode_LowBattery) && (Device_Mode != Device_Mode_Charge)
                                && (Device_Mode != Device_Mode_FWUpdate))
                            {
                                TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStart);		/* 	Start HR/SPO2 realtime sample 	*/
                            }
                            /* 	Init the EEPROM for FreeRun Mode 		*/
                            if(flagIsFreeRunKickOff == false)
                            {
                                #ifdef FreeRunMode_Debug
                                    printf("init free run \r\n");
                                #endif

                                FreeRunModeInit();
                            }
                            /* set the stop time */
                            AlarmHH = MonitorTemplate.ClockArr[2];
                            AlarmMM = MonitorTemplate.ClockArr[3];
                            #if defined (FreeRunMode_Debug) || (defined Monitor_Template_Debug)
                                printf("set FreeRun MT Stop alarm = %d, %d\r\n",AlarmHH,AlarmMM);
                            #endif
                            SetCalendar_Alarm_A(AlarmHH,AlarmMM,1);
                        }
                    }
                    else
                    {
                        if(
                            CurTime >= (MonitorTemplate.ClockArr[2]*60 + MonitorTemplate.ClockArr[3])
                        ||  CurTime < (MonitorTemplate.ClockArr[0]*60 + MonitorTemplate.ClockArr[1])
                        )  //stop free run
                        {
                            #if defined (FreeRunMode_Debug) || (defined Monitor_Template_Debug)
                                printf("stop sample in sample, curr time = %d \r\n",CurTime);
                            #endif

                            /* 	De-init the EEPROM for FreeRun Mode 		*/
                            if(flagIsFreeRunKickOff == true)
                            {
                                #ifdef FreeRunMode_Debug
                                    printf("deinit free run \r\n");
                                #endif
                                /* 	Overflow the stop time point, then stop the sample 	*/
                                #ifdef FreeRunMode_Debug
                                    printf("stop spo2 5 \r\n");
                                #endif

                                TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop);			/* 	Stop HR/SPO2 realtime sample 	*/
                                FreeRunModeDeInit();
                            }

                            if(MonitorTemplate.LightMTinfo.LightSampleFreq == 0)
                            {
                                /* reset the start time */
                                AlarmHH = MonitorTemplate.ClockArr[0];
                                AlarmMM = MonitorTemplate.ClockArr[1];
                                #if defined (FreeRunMode_Debug) || (defined Monitor_Template_Debug)
                                    printf("reset alarm = %d, %d\r\n",AlarmHH,AlarmMM);
                                #endif
                                SetCalendar_Alarm_A(AlarmHH,AlarmMM,1);
                            }
                        }
                        else if ((CurTime >= (MonitorTemplate.ClockArr[0]*60 + MonitorTemplate.ClockArr[1]))
                            && (CurTime < (MonitorTemplate.ClockArr[2]*60 + MonitorTemplate.ClockArr[3]))
                                )   // start free run
                        {
                            #ifdef FreeRunMode_Debug
                                printf("start sample \r\n");
                            #endif

                            /* 	In FreeRun mode: will start realtime sample on each ID 		*/
                            if((Device_Mode != Device_Mode_LowBattery) && (Device_Mode != Device_Mode_Charge)
                                && (Device_Mode != Device_Mode_FWUpdate))
                            {
                                TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStart);		/* 	Start HR/SPO2 realtime sample 	*/
                            }
                            /* 	Init the EEPROM for FreeRun Mode 		*/
                            if(flagIsFreeRunKickOff == false)
                            {
                                #ifdef FreeRunMode_Debug
                                    printf("init free run \r\n");
                                #endif

                                FreeRunModeInit();
                            }
                            /* set the stop time */
                            AlarmHH = MonitorTemplate.ClockArr[2];
                            AlarmMM = MonitorTemplate.ClockArr[3];
                            #if defined (FreeRunMode_Debug) || (defined Monitor_Template_Debug)
                                printf("set FreeRun MT Stop alarm = %d, %d\r\n",AlarmHH,AlarmMM);
                            #endif
                            SetCalendar_Alarm_A(AlarmHH,AlarmMM,1);
                        }
                    }
                }
			}

            /* ��ȡ��ǰ�໤����״̬ */
            flagLightMonitorTemplateStatus = GetLightMonitorTemplateStatus();

            if(
                (MonitorTemplate.MTID != kICTMonitorTemplateFreeRunID)
            ||  ((MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID) && (flagLightMonitorTemplateStatus == ON))
            )    //���Ż��������� ����  ��֢�෽�����첿�ַ�������
			{
				if((Device_Mode != Device_Mode_LowBattery) && (Device_Mode != Device_Mode_Charge)
					&& (Device_Mode != Device_Mode_FWUpdate))
				{
                    if(FALSE == bFlagNewLightStart)
                    {
    					/* ������ǰ���ģ��ɼ������¼� */
    					if(MonitorTemplate.SampleID & SAMPLE_ID_SpO2)  //�ɼ�Ѫ��
    					{
    						#ifdef Monitor_Template_Debug
    							printf("Monitor Template SpO2\r\n");
    						#endif
    						gSubFunc_Stat_Set(SpO2_MonitorTemplate_State,ON);      //SpO2���ģ��������
    						/* ����HR/SpO2����ǰ���������⹦������Ϊ����ģʽ */
    						Wear_Detect_Set(WEAR_DETECT_INC);
    						TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStart);     //����SpO2���������¼�,SpO2��HR��������ͬһ�¼�
                        }

    					if(MonitorTemplate.SampleID & SAMPLE_ID_HeartRate)  //�ɼ�����
    					{
    						#ifdef Monitor_Template_Debug
    							printf("Monitor Template Sample HeartRate\r\n");
    						#endif
    						gSubFunc_Stat_Set(HR_MonitorTemplate_State, ON);      //HR���ģ��������

    						/* ����HR/SpO2����ǰ���������⹦������Ϊ����ģʽ */
    						Wear_Detect_Set(WEAR_DETECT_INC);
						    TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStart);     //����SpO2���������¼�,SpO2��HR��������ͬһ�¼�
    					}
                    }
                    else
                    {
                        bFlagNewLightStart = FALSE;
                    }
				}
			}

            //������Ի������´β���ʱ��
			if(MonitorTemplate.MTID != kICTMonitorTemplateFreeRunID)  //free run
			{
				if(MonitorTemplate.SampleFreq == 0) //����ʱ���ɼ�
				{
					if(SampleClock <((MonitorTemplate.SampleTimeClock.DataLen)/2))  //δ�������ģ��ʱ��㷶Χ
					{
						#ifdef Monitor_Template_Debug
						//printf("Set Next Alarm = %d",SampleClock);
						printf("Next Alarm Time:Hour=%d,Minute=%d\r\n",MonitorTemplate.SampleTimeClock.Context[SampleClock*2],
											MonitorTemplate.SampleTimeClock.Context[SampleClock*2+1]);
						#endif
						SetCalendar_Alarm_A(MonitorTemplate.SampleTimeClock.Context[SampleClock*2],
											MonitorTemplate.SampleTimeClock.Context[SampleClock*2+1],1); //������һ�����ģ��ʱ���
						SampleClock ++;
					}
					else  //���ģ��ִ����ɣ��ر�RTC�����ж�
					{
						#ifdef Monitor_Template_Debug
							printf("Monitor Template Conplated,Disable Alarm\r\n");
						#endif
						DisCalendar_Alarm_A();
						SampleClock=0;
					}
				}
				else  //����ʱ��βɼ�
				{
					Calendar_Get(&date_s,&rtc_time);
					//����ǰϵͳʱ������ģ��ʱ��ν��бȽ�
					CurTime = rtc_time.RTC_Hours*60 + rtc_time.RTC_Minutes;

					if(MonitorTemplate.MTSwitch == 0) //��ͣ���ģ��
					{
						DisCalendar_Alarm_A();
						#ifdef Monitor_Template_Debug
							printf("Suspend MT\r\n");
						#endif
					}
					else
					{
						//�����´βɼ�ʱ���
						if(0 == FindNextAlarmTimePoint(CurTime,&AlarmHH,&AlarmMM,&u8Flag))
						{
                            if(1 == u8Flag)
                            {
                                bFlagMonitorTemplateIsRunning = TRUE;
                            }
                            SetCalendar_Alarm_A(AlarmHH,AlarmMM,1);
						}
                        else
                        {
                            DisCalendar_Alarm_A();
                        }
					}
				}
			}

            //������֢�෽�����첿�ַ����´β���ʱ��
            if((MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID) && (MonitorTemplate.LightMTinfo.LightSampleFreq != 0))
            {
                Calendar_Get(&date_s,&rtc_time);
                //����ǰϵͳʱ������ģ��ʱ��ν��бȽ�
                CurTime = rtc_time.RTC_Hours*60 + rtc_time.RTC_Minutes;

                if(MonitorTemplate.MTSwitch == 0) //��ͣ���ģ��
                {
                    DisCalendar_Alarm_A();
                    #ifdef Monitor_Template_Debug
                        printf("Light Suspend MT\r\n");
                    #endif
                }
                else //����δ��ͣ��������֢�෽�����첿�ַ����´β���ʱ��
                {
                    flagLightMonitorTemplateStatus = GetLightMonitorTemplateStatus();
                    if(flagLightMonitorTemplateStatus == ON)  //����໤����������
                    {
                        FreeRunMTSetLightCtlTime(CurTime);
                    }
                    else    //���ڰ��췽������ʱ�����,ҹ�䷽������
                    {
                        SetFreeRunMTCtlTime(CurTime);
                    }
                }
            }
        }// end of if(MonitorTemplate_Event == gMonitorTemplateEventSampleData) //���ռ��ģ��ɼ�����
		else if(MonitorTemplate_Event == gMonitorTemplateEventTest)
		{
			Calendar_Get(&date_s,&rtc_time);
			SetCalendar_Alarm_A(rtc_time.RTC_Hours,rtc_time.RTC_Minutes+1,rtc_time.RTC_Seconds);
		}
		if(MonitorTemplate_Event == gExcetionMonitorTemplateEventStart)  //�����쳣���ģ�忪ʼ�¼�
		{
			if((Device_Mode == Device_Mode_LowBattery) || (Device_Mode == Device_Mode_Charge)
				|| (Device_Mode == Device_Mode_FWUpdate))
			{
				return;
			}
			if(GetMonitorTemplateType() == NormalMT)  //�������ģ�������²ſ�ʼ
			{
				if((ExcetionMonitorTemplate.SampleID == CheckUpStop) || (ExcetionMonitorTemplate.SampleFreq == 0u))
				{
					//�쳣���ģ���в���IDΪ0���������쳣���ģ�����
					#ifdef Monitor_Template_Debug
						printf("ExcetionMonitor ID : 0\r\n");
					#endif
				}
				else
				{
					StartExcetionMonitorTemplate();    //��ʼ�쳣���ģ��
				}
			}
		}
		if(MonitorTemplate_Event == gExcetionMonitorTemplateEventStop)  //�����쳣���ģ��ֹͣ�¼�
		{
			if(GetMonitorTemplateType() == ExcetionMT)  //�쳣���ģ�������²ſ�ʼ
			{
				StopExcetionMonitorTemplate();    //ֹͣ�쳣���ģ��
			}
		}
		if(MonitorTemplate_Event == gExcetionMonitorTemplateEventSet)  //�����쳣���ģ�������¼�
		{
			if((Device_Mode == Device_Mode_LowBattery) || (Device_Mode == Device_Mode_Charge)
				|| (Device_Mode == Device_Mode_FWUpdate))
			{
				return;
			}
			Calendar_Get(&date_s,&rtc_time);  //��ȡ��ǰʱ��
			for(i=0;i<(ExcetionMonitorTemplate.SampleTimeClock.DataLen);)
			{
				if(ExcetionMonitorTemplate.SampleTimeClock.Context[i*2]<rtc_time.RTC_Hours)
				{
					i++;
				}
				else
				{
					break;
				}
			}
			if(ExcetionMonitorTemplate.SampleTimeClock.Context[i*2] == rtc_time.RTC_Hours)
			{
				while(i<(ExcetionMonitorTemplate.SampleTimeClock.DataLen))
				{
					if(ExcetionMonitorTemplate.SampleTimeClock.Context[i*2+1]<rtc_time.RTC_Minutes)
					{
						i++;
					}
					else
					{
						break;
					}
				}
			}
			/* �ҵ���ǰ���ʱ��� */
			ExcetionSampleClock = i;
			if(ExcetionSampleClock < ExcetionMonitorTemplate.SampleTimeClock.DataLen)  //�ҵ����ʱ���
			{
				/* check if current time equals to first alarm time */
				if(rtc_time.RTC_Minutes == ExcetionMonitorTemplate.SampleTimeClock.Context[ExcetionSampleClock*2+1])
				{
					ExcetionSampleClock++;  //point to next sample point
				}
				SetCalendar_Alarm_A(ExcetionMonitorTemplate.SampleTimeClock.Context[ExcetionSampleClock*2],ExcetionMonitorTemplate.SampleTimeClock.Context[ExcetionSampleClock*2+1],1);
				#ifdef Monitor_Template_Debug
				printf("EMT First Alarm = %d\r\n",ExcetionSampleClock);
				printf("EMT First Alarm Time:  %d:%d\r\n",ExcetionMonitorTemplate.SampleTimeClock.Context[ExcetionSampleClock*2],
																ExcetionMonitorTemplate.SampleTimeClock.Context[ExcetionSampleClock*2+1]);
				#endif
				ExcetionSampleClock++;  //ָ����һ�����ģ��ʱ��ɼ���
			}
		}
		if(MonitorTemplate_Event == gExcetionMonitorTemplateEventSampleData)  //�����쳣���ģ��ɼ������¼�
		{
			#ifdef Monitor_Template_Debug
			Calendar_Get(&date_s,&rtc_time);
			printf("\r\nThe current time is :  %0.2d:%0.2d:%0.2d\r\n", rtc_time.RTC_Hours, rtc_time.RTC_Minutes, rtc_time.RTC_Seconds);
			#endif

			if((Device_Mode == Device_Mode_LowBattery) || (Device_Mode == Device_Mode_Charge)
				|| (Device_Mode == Device_Mode_FWUpdate))
			{
				return;
			}

			/* ������ǰ���ģ��ɼ������¼� */
			if(ExcetionMonitorTemplate.SampleID & SAMPLE_ID_SpO2)  //�ɼ�Ѫ��
			{
				#ifdef Monitor_Template_Debug
					printf("Excetion Monitor Template SpO2\r\n");
				#endif
				gSubFunc_Stat_Set(SpO2_MonitorTemplate_State, ON);      //SpO2���ģ��������

				/* ����HR/SpO2����ǰ���������⹦������Ϊ����ģʽ */
				Wear_Detect_Set(WEAR_DETECT_INC);
				TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStart);     //����SpO2���������¼�,SpO2��HR��������ͬһ�¼�
			}
			if(ExcetionMonitorTemplate.SampleID & SAMPLE_ID_HeartRate)  //�ɼ�����
			{
				#ifdef Monitor_Template_Debug
					printf("Excetion Monitor Template Sample HeartRate\r\n");
				#endif
				gSubFunc_Stat_Set(HR_MonitorTemplate_State,ON);      //HR���ģ��������

				/* ����HR/SpO2����ǰ���������⹦������Ϊ����ģʽ */
				Wear_Detect_Set(WEAR_DETECT_INC);
				TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStart);     //����SpO2���������¼�,SpO2��HR��������ͬһ�¼�
			}
			if(Flag_Excetion_MTRecovery == true)
			{
				if(ExcetionSampleClock < (ExcetionMonitorTemplate.SampleTimeClock.DataLen+1))  //δ�������ģ��ʱ��㷶Χ
				{
					#ifdef Monitor_Template_Debug
					printf("Next Alarm Time:Hour=%d,Minute=%d\r\n",ExcetionMonitorTemplate.SampleTimeClock.Context[ExcetionSampleClock*2],
										ExcetionMonitorTemplate.SampleTimeClock.Context[ExcetionSampleClock*2+1]);
					#endif
					SetCalendar_Alarm_A(ExcetionMonitorTemplate.SampleTimeClock.Context[ExcetionSampleClock*2],
										ExcetionMonitorTemplate.SampleTimeClock.Context[ExcetionSampleClock*2+1],1); //������һ�����ģ��ʱ���
					ExcetionSampleClock ++;
				}
				else  //һ���쳣���ģ��ִ�����
				{
					#ifdef Monitor_Template_Debug
						printf("A Trip Excetion Monitor Template Conplated\r\n");
					#endif
					ExcetionSampleClock=0;
					TS_SendEvent(gTsMonitorTemplatID_c,gExcetionMonitorTemplateEventStop);  //�����쳣���ģ��ֹͣ�¼�
				}
			}
			else
			{
				if(ExcetionSampleClock < (ExcetionMonitorTemplate.SampleTimeClock.DataLen))  //δ�������ģ��ʱ��㷶Χ
				{
					#ifdef Monitor_Template_Debug
					printf("Next Alarm Time:Hour=%d,Minute=%d\r\n",ExcetionMonitorTemplate.SampleTimeClock.Context[ExcetionSampleClock*2],
										ExcetionMonitorTemplate.SampleTimeClock.Context[ExcetionSampleClock*2+1]);
					#endif
					SetCalendar_Alarm_A(ExcetionMonitorTemplate.SampleTimeClock.Context[ExcetionSampleClock*2],
										ExcetionMonitorTemplate.SampleTimeClock.Context[ExcetionSampleClock*2+1],1); //������һ�����ģ��ʱ���
					ExcetionSampleClock ++;
				}
				else  //һ���쳣���ģ��ִ�����
				{
					#ifdef Monitor_Template_Debug
						printf("A Trip Excetion Monitor Template Conplated\r\n");
					#endif
					ExcetionSampleClock=0;
					TS_SendEvent(gTsMonitorTemplatID_c,gExcetionMonitorTemplateEventStop);  //�����쳣���ģ��ֹͣ�¼�
				}
			}
		}
        else if ( MonitorTemplate_Event == gExcetionMonitorTemplateEventSetFreeRunGuard)
        {
            Calendar_Get(&date_s,&rtc_time);

            if((rtc_time.RTC_Hours == 0) && (rtc_time.RTC_Minutes == 0))
            {
                if(MonitorTemplate.MTID != kICTMonitorTemplateFreeRunID)
                {
                    bFlagNewDayEvent = TRUE;
                    TS_SendEvent(gTsMonitorTemplatID_c,gMonitorTemplateEventStartSet);    //ÿ������12��������ģ�忪ʼ�����¼�
                }
            }

            if(MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID)
            {
                /* When in Free Run, AlarmB Monitor the status of FreeRunning */
                if(true == isFreeRunKickOff())
                {
                    if(cc_alg_SpO2_Stat_Get() != SPO2_HR_STATUS_RUNNING)
                    {
                        TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStart);		/* 	Start HR/SPO2  sample 	*/
                    }

                    /* Set Next AlarmB */
                    Set_FreeRunStatusMonitorRecord_Time();
                }
                else
                {
                    /* Disable AlarmB */
                    DisCalendar_Alarm_B();
                }
            }
        }
        else if( MonitorTemplate_Event == gExcetionMonitorTemplateEventStoreFreeRunData)
        {
            if(MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID)
            {
                FreeRun_SampleData_StorageFunc();
            }
        }
}

/***************�쳣���ģ��ִ��******************************************************************/
/*******************************************************************************
* Function Name  : StartExcetionMonitorTemplate
* Description    : ��ʼִ���쳣���ģ��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void StartExcetionMonitorTemplate(void)
{
		date_str_typedef    date_s;               //RTC ����
		RTC_TimeTypeDef     rtc_time;             //RTC ʱ��
		uint8_t    SampleEveryTime = 0;           //ÿ�βɼ�ʱ�䣬ͨ���ɼ�Ƶ��ת������
		uint8_t    i=0;
		uint8_t    index = 0;

		/* Init Excetion Monitor Template Status */
		SetMonitorTemplateType(ExcetionMT);  // ���ģ������Ϊ�쳣���ģ��
		SetMonitorTemplateMeaResultType(NormalVal);
		DisCalendar_Alarm_A();

		Calendar_Get(&date_s,&rtc_time);

		#ifdef Monitor_Template_Debug
			printf("\r\nCalc the Excetion MT Run Clcok...\r\n");
			printf("\r\nThe current time is :  %0.2d:%0.2d:%0.2d\r\n", rtc_time.RTC_Hours, rtc_time.RTC_Minutes, rtc_time.RTC_Seconds);
		#endif

		ExcetionMonitorTemplate.SampleTimeClock.DataLen = ExcetionMonitorTemplate.SampleLastTime / ExcetionMonitorTemplate.SampleFreq; // 300 / 60=5
		SampleEveryTime = ExcetionMonitorTemplate.SampleFreq / 60;  //���ɼ�Ƶ��ת��Ϊ����
		memset(ExcetionMonitorTemplate.SampleTimeClock.Context,0,MaxEMTLen*sizeof(uint8_t));

	    ExcetionMonitorTemplate.SampleTimeClock.Context[1] = rtc_time.RTC_Minutes + SampleEveryTime;  //��һ���쳣���ģ���ڵ�ǰʱ��
																									  //SampleEveryTime֮�����
		if(ExcetionMonitorTemplate.SampleTimeClock.Context[1] >= 60) //�������ʱ��1
		{
			ExcetionMonitorTemplate.SampleTimeClock.Context[1] = ExcetionMonitorTemplate.SampleTimeClock.Context[1] % 60;
			ExcetionMonitorTemplate.SampleTimeClock.Context[0] = rtc_time.RTC_Hours + 1;
			if(ExcetionMonitorTemplate.SampleTimeClock.Context[0] >= 24)  //ʱ���
			{
				ExcetionMonitorTemplate.SampleTimeClock.Context[0] =0;
			}
		}
		else  //��δ���
		{
			ExcetionMonitorTemplate.SampleTimeClock.Context[0] = rtc_time.RTC_Hours;
		}

		index = 2;
		for(i=0;i<(ExcetionMonitorTemplate.SampleTimeClock.DataLen) && ExcetionMonitorTemplate.SampleTimeClock.DataLen <= MaxEMTLen;i++) //�Ѿ�������һ���쳣���ģ��ִ��ʱ��
		{
			 ExcetionMonitorTemplate.SampleTimeClock.Context[index+1] = ExcetionMonitorTemplate.SampleTimeClock.Context[index-1] + SampleEveryTime;  //SampleEveryTime֮�����
			if(ExcetionMonitorTemplate.SampleTimeClock.Context[index+1] >= 60) //�������ʱ��1
			{
				ExcetionMonitorTemplate.SampleTimeClock.Context[index+1] = ExcetionMonitorTemplate.SampleTimeClock.Context[index+1] % 60;
				ExcetionMonitorTemplate.SampleTimeClock.Context[index] =  ExcetionMonitorTemplate.SampleTimeClock.Context[index-2] + 1;
				if(ExcetionMonitorTemplate.SampleTimeClock.Context[index] >= 24)  //ʱ���
				{
					ExcetionMonitorTemplate.SampleTimeClock.Context[index] =0;
				}
			}
			else  //��δ���
			{
				ExcetionMonitorTemplate.SampleTimeClock.Context[index] = ExcetionMonitorTemplate.SampleTimeClock.Context[index-2];
			}
			index = index + 2;
		}
		#ifdef Monitor_Template_Debug
		printf("Excetion MT Info...\r\n");
		printf("EMT_SampleID=%x\r\n",ExcetionMonitorTemplate.SampleID);
		printf("EMT_LastTime=%d\r\n",ExcetionMonitorTemplate.SampleLastTime);
		printf("EMT_Freq=%d\r\n",ExcetionMonitorTemplate.SampleFreq);
		for(i=0;i<(ExcetionMonitorTemplate.SampleTimeClock.DataLen + 1);i++)
		{
			printf("EMT_STC:  %0.2d:%0.2d\r\n",ExcetionMonitorTemplate.SampleTimeClock.Context[i*2],
											ExcetionMonitorTemplate.SampleTimeClock.Context[i*2+1]);
		}
		#endif
		TS_SendEvent(gTsMonitorTemplatID_c,gExcetionMonitorTemplateEventSet);  //�����쳣���ģ�������¼�

		if(Flag_Excetion_MTRecovery == false)  //�Ѵ��쳣���ģ���лָ�����һ���ӿ�ʼ���������򣬴ӵ�ǰ���ӿ�ʼ����
		{
			/* ������һ���쳣�ɼ� */
			/* ������ǰ���ģ��ɼ������¼� */
			if(ExcetionMonitorTemplate.SampleID & SAMPLE_ID_SpO2)  //�ɼ�Ѫ��
			{
				#ifdef Monitor_Template_Debug
					printf("First Excetion Monitor Template SpO2\r\n");
				#endif
				gSubFunc_Stat_Set(SpO2_MonitorTemplate_State ,ON);      //SpO2���ģ��������

				/* ����HR/SpO2����ǰ���������⹦������Ϊ����ģʽ */
				Wear_Detect_Set(WEAR_DETECT_INC);
				TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStart);     //����SpO2���������¼�,SpO2��HR��������ͬһ�¼�
			}
			if(ExcetionMonitorTemplate.SampleID & SAMPLE_ID_HeartRate)  //�ɼ�����
			{
				#ifdef Monitor_Template_Debug
					printf("First Excetion Monitor Template Sample HeartRate\r\n");
				#endif
				gSubFunc_Stat_Set(HR_MonitorTemplate_State,ON);      //HR���ģ��������

				/* ����HR/SpO2����ǰ���������⹦������Ϊ����ģʽ */
				Wear_Detect_Set(WEAR_DETECT_INC);
				TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStart);     //����SpO2���������¼�,SpO2��HR��������ͬһ�¼�
			}
		}
}
/*******************************************************************************
* Function Name  : StopExcetionMonitorTemplate
* Description    : ����ִ���쳣���ģ��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void StopExcetionMonitorTemplate(void)
{
		static uint8_t ExcetionMTValCnt = 0;  //�쳣���ģ��ɼ��õ��������ݴ���

		/* ֹͣ���ģ������ */
                            #ifdef FreeRunMode_Debug
                                printf("stop spo2 6 \r\n");
                            #endif

		gSubFunc_Stat_Set(SPO2_HR_MONITOR_STATE, OFF);
		TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop); //����SpO2ֹͣ�����¼���HR/SpO2����Ϊͬһ�¼�����,��ʱ�ض���HRֵ

		/* ����һ���쳣���ģ��ִ�н���֡ */
		SendExcetionMonitorTemplateStopFram();
		SetMonitorTemplateType(NormalMT);  //Stop�쳣���ģ��󣬽�������Ϊ�������ģ��

		/* �ж��Ƿ�ִ����һ���쳣���ģ�� */
		if(GetMonitorTemplateMeaResultType() == ExcetionVal)  //�쳣���ģ���쳣���
		{
			//�쳣���ģ���в�ý���쳣,������һ���쳣���ģ��
			ExcetionMTValCnt = 0;
			Flag_Excetion_MTRecovery = false;
			TS_SendEvent(gTsMonitorTemplatID_c,gExcetionMonitorTemplateEventStart);  //�����쳣���ģ�忪ʼ�¼�
			#ifdef Monitor_Template_Debug
				printf("Start Next Excetion Monitor Template A\r\n");
			#endif
		}else if(GetMonitorTemplateMeaResultType() == NormalVal)  //�쳣���ģ���н������
		{
			ExcetionMTValCnt = ExcetionMTValCnt+1;
			if(ExcetionMTValCnt >= 2)  //�������βɼ�����������
			{
				#ifdef Monitor_Template_Debug
					printf("Stop Excetion Monitor Template,Back to Normal MT\r\n");
				#endif
				ExcetionMTValCnt = 0;
				Flag_Excetion_MTRecovery = true;
                bFlagNewDayEvent = FALSE;
				TS_SendEvent(gTsMonitorTemplatID_c,gMonitorTemplateEventStartSet);   //�����������ģ��
			}
			else   //û���������βɼ�����������
			{
				#ifdef Monitor_Template_Debug
					printf("Start Next Excetion Monitor Template B\r\n");
				#endif
				Flag_Excetion_MTRecovery = false;
				TS_SendEvent(gTsMonitorTemplatID_c,gExcetionMonitorTemplateEventStart);  //�����쳣���ģ�忪ʼ�¼�
			}
		}
}
static void SendExcetionMonitorTemplateStopFram(void)
{
		date_str_typedef    date_s;               //RTC ����
		RTC_TimeTypeDef     rtc_time;             //RTC ʱ��
		uint8_t 			StopFram[MT_STORAGE_DATA_LEN] = {0};
		uint8_t  		    Err_Code=0;
        uint32_t            u32Date = 0;

		Calendar_Get(&date_s,&rtc_time);

		/* ����һ���쳣���ģ��ִ�н���֡ */
		StopFram[0]=rtc_time.RTC_Hours;
		StopFram[1]=rtc_time.RTC_Minutes;
		StopFram[2]=rtc_time.RTC_Seconds;

		StopFram[3]=StopExcetionFram;
		StopFram[4] = 0;
		StopFram[5] = 0;
		StopFram[6] = 0;
		StopFram[7] = 0;

		/* ����Ч��Stopֵд��EEPROM�� */
		//if(GetM95M01State(MONITOR_MODEL_VALUE_ID,M95M01_CAPACITY_SPACE) < sizeof(StopFram))
        if(0)
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

        /* Create The Moniter Template Data Store Partition */
        u32Date = CovernDateto32();
        ExtFLASH_ExtCreatePartition(u32Date);

        /* Save Stop Frame Monitor Template data */
        Err_Code = DataMemoryWrite(MONITOR_TYPE,StopFram,sizeof(StopFram));
        APP_ERROR_CHECK(Err_Code);

        #ifdef Monitor_Template_Debug
                printf("Send Excetion MT Stop Fram\r\n");
        #endif

        if(Device_State == Device_WorkWithAPP)   //��App������һ�������м��ģ�����н���������ͬ���¼�
        {
            TS_SendEvent(gTsSyncDataTaskID_c,gSyncDataFormat2EventStart);  //������ʼ����ͬ���¼�
        }
}
/*******************************************************************************
* Function Name  : GetMonitorTemplateType
* Description    : ��ȡ���ģ�����ͣ�NromalMT Or ExcetionMT
* Input          : None
* Output         : None
* Return         : MonitorTemplateType_Typedef
*******************************************************************************/
MonitorTemplateType_Typedef GetMonitorTemplateType(void)
{
		return gMT_Type;
}
/*******************************************************************************
* Function Name  : SetMonitorTemplateType
* Description    : ���ü��ģ������
* Input          : MonitorTemplateType_Typedef  :NromalMT Or ExcetionMT
* Output         : None
* Return         : None
*******************************************************************************/
void  SetMonitorTemplateType(MonitorTemplateType_Typedef type)
{
		gMT_Type = type;
}
/*******************************************************************************
* Function Name  : SetMonitorTemplateMeaResultType
* Description    : ���ü��ģ������������
* Input          : MonitorTemplateMeaResult_Typedef  :NromalVal Or ExcetionVal
* Output         : None
* Return         : None
*******************************************************************************/
void SetMonitorTemplateMeaResultType(MonitorTemplateMeaResult_Typedef type)
{
		gMTMeaSult = type;
}
/*******************************************************************************
* Function Name  : SetMonitorTemplateMeaResultType
* Description    : ���ü��ģ������������
* Input          : MonitorTemplateMeaResult_Typedef  :NromalVal Or ExcetionVal
* Output         : None
* Return         : None
*******************************************************************************/
MonitorTemplateMeaResult_Typedef GetMonitorTemplateMeaResultType(void)
{
		return gMTMeaSult;
}

void ChangeDeviceMode_FromCheckUpToRTC(void)
{
	if(Device_Mode == Device_Mode_CheckUp)  //��ʱ�ɼ�ģʽ��
	{
		if(gSubFunc_Stat_Get(SpO2_RealTimeSample_State | HR_RealTimeSample_State) == OFF)
		{
			OLEDDisplay_Stat_Set(NormalDis);  //App��ʱ�ɼ������ʾNormalģʽ
			Device_Mode = Device_Mode_RTC;    //ֹͣ�ɼ��󣬻ص�RTCģʽ
			Set_OLED_Dis_Status(OLEDDisON);
			TS_SendEvent(gOledDisTaskID,gOledDisEventBlueIcon_c);   //��ʾRTCģʽ
		}
	}
}

/* 	Init the EEPROM for FreeRun Mode 		*/
static void FreeRunModeInit(void)
{
    uint32_t        u32Date = 0;

    #ifdef Monitor_Template_Debug
        printf("\r\nFreeRunModeInit...\r\n");
    #endif

    /* 	Use these parameters to determine how single mode works 		*/
    gSubFunc_Stat_Set(HR_RealTimeSample_State | SpO2_RealTimeSample_State,OFF);
    HR_SpO2_SendFlag = 0x00;
    OLEDDisplay_Stat_Set(NormalDis);  /* 	Force display mode back Normal Mode 	*/
    if((Device_Mode != Device_Mode_LowBattery) && (Device_Mode != Device_Mode_Charge) && (Device_Mode == Device_Mode_CheckUp))
    {
        /* 	Back to RTC GUI 	*/
        Device_Mode = Device_Mode_RTC;
//        flagIsHRSpO2FixNumberSampleKeyPress = TRUE; 	/* 	To avoid key press twice for NTC/SPO2 single result test 	*/
        if(Get_OLED_Dis_Status() != OLEDDisShutDown)
        {
            TS_SendEvent(gOledDisTaskID,gOledDisEventBlueIcon_c);
        }
    }

    /* Notice App, FreeRun Monitor Template is running, if in Device_Mode_CheckUp */
    TS_SendEvent(gTsAlarmTaskID_c,gAlarmEventFreeRunMTRunning);

    /* Stop SpO2 Measure when in Device_Mode_SPO2_HR */
    if(SPO2_HR_Measure_State == Start)    //SPO2/HR���ڲ��������У��̰�ֹͣSPO2/HR����
    {
        /* 	To avoid, in case :
            1. single result test,
            2. long press to start
            3. short press to stop measure before result display
            Then, need press twice to enter next mode
        */
//        if(
//            SpO2_RealTimeSample_GetResultNum != 0
//        && 	HR_RealTimeSample_GetResultNum != 0
//        && 	flagIsHRSpO2FixNumberSampleKeyPress == FALSE
//        )
//        {
//            flagIsHRSpO2FixNumberSampleKeyPress = TRUE;
//        }
        SPO2_START_BY_KEY = OFF;
        gSubFunc_Stat_Set(SpO2_SingleWork_State | HR_SingleWork_State, OFF);
        SPO2_HR_Measure_State = Stop;
        ResetSingleModeAlarmVirbreTim();  //ֹͣʱ������񶯸澯���ʱ��
        TS_SendEvent(gOledDisTaskID,gOledDisEventModeHR_c); //����OLED��ʾSpO2/HRģʽ�����¼�
        #ifdef  FreeRunMode_Debug
        printf("SPO2_HR Measure Stop In FreeRun\r\n");
        #endif
    }

    /* Create The Moniter Template Data Store Partition */
    u32Date = CovernDateto32();
    ExtFLASH_ExtCreatePartition(u32Date);

    /* Set The Free Run Start Frame */
    SetFreeRunNightStartFrame();

    /* Init gHRSpO2Val.HR_Value and gHRSpO2Val.SpO2_Value to 0 */
    gHRSpO2Val.HrSpO2DataRep_pst->m_ui8HrVal = 0;
    gHRSpO2Val.HrSpO2DataRep_pst->m_ui8SpO2Val = 0;

	/* 	Set Flag to indicate that FreeRun Mode has kicked off 		*/
	flagIsFreeRunKickOff = true;

	/* 	Kick off RTC autowake up interrupt 		*/

	Calendar_RTC_Period_Wakeup_Init(16384,FreeRun_SampleData_Storage_Callback);

    /* Set AlarmB to Monitor The status of FreeRun */
    Set_FreeRunStatusMonitorRecord_Time();
}

static void FreeRun_SampleData_Storage_Callback(void)
{
    //printf("sm \r\n");
    static uint8_t Cnt = 0;

    Cnt++;

    if(Cnt >= MonitorTemplate.SampleFreq)
    {
        Cnt = 0;
        TS_SendEvent(gTsMonitorTemplatID_c,gExcetionMonitorTemplateEventStoreFreeRunData);
    }
}


/* 	Write data into EEPROM for FreeRun Mode 		*/
static void FreeRun_SampleData_StorageFunc(void)
{
	uint8_t		Err_Code=0;
    uint8_t     parameterID = 0;
	uint8_t 	bufForStorage[MT_STORAGE_DATA_LEN];				//the buffer to data storage

	#ifdef FreeRunMode_Debug
		uint8_t  i=0;
	#endif

    date_str_typedef    date_s;               //RTC ����
    RTC_TimeTypeDef     rtc_time;             //RTC ʱ��
    uint16_t        CurTime=0;  //��ǰʱ��ת��Ϊ����
    uint32_t        AlarmHH=0;  //����Сʱ
    uint32_t        AlarmMM=0;  //���ӷ���
    uint32_t        u32Date=0;

#ifdef FreeRunModeWithoutID
	if(isFreeRunKickOff() == false)
    {
        /* The OSAS has Stopped, storage the data when OSAS is running*/
        return;
    }

    /* Get The Current Time */
    Calendar_Get(&date_s,&rtc_time);
    CurTime = rtc_time.RTC_Hours*60 + rtc_time.RTC_Minutes;

	/* 	Init the buffer for data header 	*/
	bufForStorage[0]	= (uint8_t)(rtc_time.RTC_Hours);		/* 	Hour */
	bufForStorage[1]	= (uint8_t)(rtc_time.RTC_Minutes);		/* 	Minutes	*/
    bufForStorage[2]	= (uint8_t)(rtc_time.RTC_Seconds);		/* 	Seconds	*/

    /* Parameter ID */
    switch(MonitorTemplate.SetMTID)
    {
        case kICTMonitorTemplateOSA:
            parameterID = OSASID;
            break;

        case kICTMonitorTemplateHypoxemia:
            parameterID = HypoxemiaID;
            break;

        case kICTMonitorTemplateCOPD:
            parameterID = COPDID;
            break;

        case kICTMonitorTemplateSleepApneaManagerID:
            parameterID = SleepApneaManagerID;
            break;

        case kIctmonitorTemplateNightLowSpo2ManagerID:
            parameterID = NightLowSpo2ManagerID;
            break;

        default:
            break;
    }
    bufForStorage[3] = parameterID;

	/* 	HR value 		*/
	if(MonitorTemplate.SampleID & SAMPLE_ID_HeartRate)
	{
        if((gHRSpO2Val.HrSpO2DataRep_pst->m_ui8HrVal != 0) && (gHRSpO2Val.HrSpO2DataRep_pst->m_ui8HrVal < 40))
        {
            gHRSpO2Val.HrSpO2DataRep_pst->m_ui8HrVal = 40;
        }
		bufForStorage[4] = gHRSpO2Val.HrSpO2DataRep_pst->m_ui8HrVal;
	}

	/* 	SPO2 Value 		*/
	if(MonitorTemplate.SampleID & SAMPLE_ID_SpO2)
	{
		bufForStorage[5] = gHRSpO2Val.HrSpO2DataRep_pst->m_ui8SpO2Val;
	}
    
	/* MoveLevel Value */
	if(MonitorTemplate.SampleID & SAMPLE_ID_Temperature)
	{
		//bufForStorage[SaveDataCnt] = (uint8_t)(tmpDat%10);	 /* 	TEMP value digi 		*/
		bufForStorage[6] = (uint8_t)((gHRSpO2Val.HrSpO2DataRep_pst->m_ui16MotionVal & 0xFF00) >> 8);   /*	Only for test, represent the motion level	*/
	}
    bufForStorage[7] = gHRSpO2Val.HrSpO2DataRep_pst->m_ui8ConfVal;      //���Ŷ�

#endif

    /* 	Begin to store the data 			*/
    /* Create The Moniter Template Data Store Partition */
    u32Date = CovernDateto32();
    ExtFLASH_ExtCreatePartition(u32Date);

    /*	Write EEPROM 		*/
    Err_Code = DataMemoryWrite(MONITOR_TYPE,bufForStorage,sizeof(bufForStorage));
    APP_ERROR_CHECK(Err_Code);

    /* if the CurTime is not included by the FreeRun Time , stop the FreeRun*/
    if((MonitorTemplate.SampleTimeClock.DataLen/4) == 2)  //�п���
    {
        if(
            CurTime > (MonitorTemplate.ClockArr[2]*60 + MonitorTemplate.ClockArr[3])
            && CurTime < (MonitorTemplate.ClockArr[4]*60 + MonitorTemplate.ClockArr[5])
        )  //stop free run
        {
            #if defined (FreeRunMode_Debug) || (defined Monitor_Template_Debug)
                printf("cross stop sample in store, curr time = %d \r\n",CurTime);
            #endif


            /* 	De-init the EEPROM for FreeRun Mode 		*/
//			if(flagIsFreeRunKickOff == true)
//			{
                /* 	Overflow the stop time point, then stop the sample 	*/
                #ifdef FreeRunMode_Debug
                    printf("stop spo2 4 \r\n");
                #endif
                TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop);			/* 	Stop HR/SPO2 realtime sample 	*/
                #ifdef FreeRunMode_Debug
                    printf("deinit free run \r\n");
                #endif
                FreeRunModeDeInit();
//			}

            if(MonitorTemplate.LightMTinfo.LightSampleFreq == 0)
            {
                /* reset the start time */
                AlarmHH = MonitorTemplate.ClockArr[4];
                AlarmMM = MonitorTemplate.ClockArr[5];
                #if defined (FreeRunMode_Debug) || (defined Monitor_Template_Debug)
                    printf("reset alarm = %d, %d\r\n",AlarmHH,AlarmMM);
                #endif
                SetCalendar_Alarm_A(AlarmHH,AlarmMM,1);
            }
        }
    }
    else  //δ����
    {
        if(
            CurTime > (MonitorTemplate.ClockArr[2]*60 + MonitorTemplate.ClockArr[3])
        ||  CurTime < (MonitorTemplate.ClockArr[0]*60 + MonitorTemplate.ClockArr[1])
        )  //stop free run
        {
            #if defined (FreeRunMode_Debug) || (defined Monitor_Template_Debug)
                printf("stop sample in store, curr time = %d \r\n",CurTime);
            #endif

            /* 	De-init the EEPROM for FreeRun Mode 		*/
//          if(flagIsFreeRunKickOff == true)
            {
                #ifdef FreeRunMode_Debug
                    printf("deinit free run \r\n");
                #endif
                /* 	Overflow the stop time point, then stop the sample 	*/
                #ifdef FreeRunMode_Debug
                    printf("stop spo2 5 \r\n");
                #endif

                TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop);			/* 	Stop HR/SPO2 realtime sample 	*/
                FreeRunModeDeInit();
            }

            if(MonitorTemplate.LightMTinfo.LightSampleFreq == 0)
            {
                /* reset the start time */
                AlarmHH = MonitorTemplate.ClockArr[0];
                AlarmMM = MonitorTemplate.ClockArr[1];
                #if defined (FreeRunMode_Debug) || (defined Monitor_Template_Debug)
                    printf("reset alarm = %d, %d\r\n",AlarmHH,AlarmMM);
                #endif
                SetCalendar_Alarm_A(AlarmHH,AlarmMM,1);
            }
        }
    }
}

/* 	Init the EEPROM for FreeRun Mode 		*/
static void FreeRunModeDeInit(void)
{
    date_str_typedef    date_s;        //RTC ����
    RTC_TimeTypeDef     rtc_time;      //RTC ʱ��
    uint16_t            CurTime=0;     //��ǰʱ��ת��Ϊ����
    uint32_t            AlarmHH=0;     //����Сʱ
    uint32_t            AlarmMM=0;     //���ӷ���
    uint16_t            i=0;
    uint16_t            MTTime=0;

    #ifdef Monitor_Template_Debug
        printf("\r\nFreeRunModeDeInit\r\n");
    #endif

    Calendar_Get(&date_s,&rtc_time);
	CurTime = rtc_time.RTC_Hours*60 + rtc_time.RTC_Minutes;

	/* 	De-init RTC autowake up interrupt 		*/
	Calendar_RTC_Period_Wakeup_DeInit();

	/* 	Set Flag to indicate that FreeRun Mode has stoped 		*/
	flagIsFreeRunKickOff = false;

    /* Start Sync Date when OSAS Stopped,if Connected with App */
    if((Device_State == Device_WorkWithAPP) &&
       (MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID)
    )
    {
        TS_SendEvent(gTsSyncDataTaskID_c,gSyncDataFormat2EventStart);  //Start Sync Data in format1
        #ifdef SyncData_DEBUG
        printf("Sync Data Start By OSAS...\r\n");
        #endif
    }
    if((MonitorTemplate.LightMTinfo.LightSampleFreq != 0) && (MonitorTemplate.MTSwitch != 0))
    {
        flagLightMonitorTemplateStatus = GetLightMonitorTemplateStatus();
        if(flagLightMonitorTemplateStatus == ON)
        {
            //���ð���໤������ʼʱ��
            for(i=0;i<(MonitorTemplate.LightMTinfo.DataLen/4);i++)
            {
                if(CurTime < (MonitorTemplate.LightMTinfo.LightMTClockArr[i*4]*60 + MonitorTemplate.LightMTinfo.LightMTClockArr[i*4+1]))
                {
                    AlarmHH = MonitorTemplate.LightMTinfo.LightMTClockArr[i*4];
                    AlarmMM = MonitorTemplate.LightMTinfo.LightMTClockArr[i*4+1];
                    break;
                }
                else if((CurTime + (MonitorTemplate.LightMTinfo.LightSampleFreq/60)) <
                    (MonitorTemplate.LightMTinfo.LightMTClockArr[i*4+2]*60 + MonitorTemplate.LightMTinfo.LightMTClockArr[i*4+3]))
                {
                    //AlarmHH = (CurTime + (MonitorTemplate.LightMTinfo.LightSampleFreq/60)) / 60;
                    //AlarmMM = (CurTime + (MonitorTemplate.LightMTinfo.LightSampleFreq/60)) % 60;

                    /********************************************************************************
                    //                               Cur_Time - Orgin_Time
                    //   Next_Time = Orgin_Time + (------------------------- + 1)*freg  (min)
                    //                                        freq
                    **********************************************************************************/
                    #if 0
                    MTTime = (MonitorTemplate.LightMTinfo.LightMTClockArr[i*4]*60 + MonitorTemplate.LightMTinfo.LightMTClockArr[i*4+1])
                           + ((CurTime - (MonitorTemplate.LightMTinfo.LightMTClockArr[i*4]*60
                           + MonitorTemplate.LightMTinfo.LightMTClockArr[i*4+1]))/(MonitorTemplate.LightMTinfo.LightSampleFreq/60)+1)*(MonitorTemplate.LightMTinfo.LightSampleFreq/60);
                    #else
                    MTTime = CurTime + (MonitorTemplate.LightMTinfo.LightSampleFreq/60);
                    #endif
                    AlarmHH = (MTTime / 60);
                    if(AlarmHH >= 24)
                    {
                        AlarmHH = 0;
                    }
                    AlarmMM = (MTTime % 60);
                    break;
                }
                else if((CurTime + (MonitorTemplate.LightMTinfo.LightSampleFreq/60)) >= 1440) //����24��
                {
                    #if 0
                    MTTime = (MonitorTemplate.LightMTinfo.LightMTClockArr[i*4]*60 + MonitorTemplate.LightMTinfo.LightMTClockArr[i*4+1])
                       + ((CurTime - (MonitorTemplate.LightMTinfo.LightMTClockArr[i*4]*60 +
                            MonitorTemplate.LightMTinfo.LightMTClockArr[i*4+1]))/(MonitorTemplate.LightMTinfo.LightSampleFreq/60)+1)*(MonitorTemplate.LightMTinfo.LightSampleFreq/60);
                    #else
                    MTTime = CurTime + (MonitorTemplate.LightMTinfo.LightSampleFreq/60);
                    #endif
                    MTTime = MTTime - 1440;

                    AlarmHH = (MTTime / 60);
                    if(AlarmHH >= 24)
                    {
                        AlarmHH = 0;
                    }
                    AlarmMM = (MTTime % 60);
                    break;
                }
                else if(CurTime <= (MonitorTemplate.ClockArr[i*4]*60 + MonitorTemplate.ClockArr[i*4+1]))
                {
                    i=2;  //��ҹ�䷽������ʱ����ڡ��л���ҹ�䷽��
                    break;
                }
            }


            if(i<(MonitorTemplate.LightMTinfo.DataLen/4))
            {
                if(FALSE == bFlagNewLightStart)
                {
                    bFlagNewLightStart = TRUE;
                }
                SetCalendar_Alarm_A(AlarmHH,AlarmMM,1);
            }
            else if(MonitorTemplate.LightMTinfo.LightSampleFreq != 0) //�л���ҹ�䷽��
            {
                FreeRunMTChangeToNinghtMT(CurTime);
            }
            #ifdef Monitor_Template_Debug
            if(i<(MonitorTemplate.LightMTinfo.DataLen/4))
            {
                printf("FreeRun Stop, Set LightMT start alarm: Hour=%d,Minute=%d\r\n",AlarmHH,AlarmMM);
            }
            #endif
        }
        else    //���ڰ��췽������ʱ����ڣ�ҹ�䷽������
        {
            SetFreeRunMTCtlTime(CurTime);
        }
    }
    ExtFLASH_StorageCheck();
}
/*******************************************************************************
* Function Name  : Set_FreeRunStatusMonitorRecord_Time
* Description    : ���ü��FreeRun����״̬ʱ��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Set_FreeRunStatusMonitorRecord_Time(void)
{
	uint32_t            AlarmBHH=0;  //B����Сʱ
	uint32_t            AlarmBMM=0;  //B���ӷ���
	RTC_TimeTypeDef     rtc_time;             //RTC ʱ��
	uint16_t            MotionInfoUpdateFreq = 120; //�˶����ݸ���Ƶ�ʣ�Ĭ��15min

	DisCalendar_Alarm_B();
	RTC_GetTime(RTC_Format_BIN, &rtc_time);

	MotionInfoUpdateFreq = MotionInfoUpdateFreq / 60;
	AlarmBMM = ((rtc_time.RTC_Minutes / MotionInfoUpdateFreq) + 1) * MotionInfoUpdateFreq;
	if(AlarmBMM >= 60)
	{
		AlarmBMM = 0;
		AlarmBHH = rtc_time.RTC_Hours + 1;
	}
	else
	{
		AlarmBHH = rtc_time.RTC_Hours;
	}

	AlarmBHH %= 24;

	#if defined (FreeRunMode_Debug)
		printf("Next Monitor FreeRun Status Time: %0.2d:%0.2d\r\n", AlarmBHH, AlarmBMM);
	#endif
    
    if(Device_Mode != Device_Mode_LowBattery)
    {
        SetCalendar_Alarm_B(AlarmBHH,AlarmBMM,10);
    }
}

/*******************************************************************************
* Function Name  : GetLightMonitorTemplateStatus
* Description    : ��ȡ����໤����״̬
* Input          : None
* Output         : None
* Return         : Status
*******************************************************************************/
static bool GetLightMonitorTemplateStatus(void)
{
    bool LightMonitorTemplateStatus = OFF;    //ON�����죬OFF��ҹ��
    date_str_typedef    date_s;               //RTC����
	RTC_TimeTypeDef     rtc_time;             //RTCʱ��
    uint16_t            CurTime=0;            //��ǰʱ��ת��Ϊ����

    Calendar_Get(&date_s,&rtc_time);
	CurTime = rtc_time.RTC_Hours*60 + rtc_time.RTC_Minutes;

    if(MonitorTemplate.LightMTinfo.LightSampleFreq == 0)  //����໤����Ƶ��Ϊ0
    {
        LightMonitorTemplateStatus = OFF;  //����ҹ��໤����
    }
    else
    {
        if((MonitorTemplate.SampleTimeClock.DataLen/4) == 2)  //ҹ�䷽���п���
        {
            if(
                (CurTime >= MonitorTemplate.ClockArr[2]*60 + MonitorTemplate.ClockArr[3])
            &&  (CurTime < MonitorTemplate.LightMTinfo.LightMTClockArr[2]*60 + MonitorTemplate.LightMTinfo.LightMTClockArr[3])
            )
            {
                LightMonitorTemplateStatus = ON; //���а���໤����
            }
            else
            {
                LightMonitorTemplateStatus = OFF; //����ҹ��໤����
            }
        }
        else if((MonitorTemplate.LightMTinfo.DataLen / 4) == 2) //���췽���п���
        {
            if(
                (CurTime >= MonitorTemplate.LightMTinfo.LightMTClockArr[2]*60 + MonitorTemplate.LightMTinfo.LightMTClockArr[3])
            &&  (CurTime < MonitorTemplate.ClockArr[2]*60 + MonitorTemplate.ClockArr[3])
            )
            {
                LightMonitorTemplateStatus = OFF;   //����ҹ��໤����
            }
            else
            {
                LightMonitorTemplateStatus = ON;    //���а���໤����
            }
        }
        else   //���졢ҹ���δ����
        {
            if((MonitorTemplate.ClockArr[0]*60 + MonitorTemplate.ClockArr[1])
                >= (MonitorTemplate.LightMTinfo.LightMTClockArr[0]*60 + MonitorTemplate.LightMTinfo.LightMTClockArr[1])
            )
            {
                if(
                    (CurTime < MonitorTemplate.LightMTinfo.LightMTClockArr[2]*60 + MonitorTemplate.LightMTinfo.LightMTClockArr[3])
                )
                {
                    LightMonitorTemplateStatus = ON;   //���а���໤����
                }
                else
                {
                    LightMonitorTemplateStatus = OFF;  //����ҹ��໤����
                }
            }
            else
            {
                if(
                    (CurTime >= MonitorTemplate.ClockArr[2]*60 + MonitorTemplate.ClockArr[3] )
                &&  (CurTime < MonitorTemplate.LightMTinfo.LightMTClockArr[2]*60 + MonitorTemplate.LightMTinfo.LightMTClockArr[3])
                )
                {
                    LightMonitorTemplateStatus = ON;   //���а���໤����
                }
                else
                {
                    LightMonitorTemplateStatus = OFF;  //����ҹ��໤����
                }
            }
        }
    }

    #ifdef Monitor_Template_Debug
    printf("LightMonitorTemplateStatus = %d\r\n",LightMonitorTemplateStatus);
    #endif
    return LightMonitorTemplateStatus;
}
/*******************************************************************************
* Function Name  : SetFreeRunMTCtlTime
* Description    : ������֢�෽������ʱ�䣨����/ֹͣʱ�䣩
* Input          : None
* Output         : None
* Return         : Status
*******************************************************************************/
static void SetFreeRunMTCtlTime(uint16_t CurTime)
{
    uint32_t        AlarmHH=0;  //����Сʱ
	uint32_t        AlarmMM=0;  //���ӷ���

    if((MonitorTemplate.SampleTimeClock.DataLen/4) == 2)  //�п���
    {
        if(false == isFreeRunKickOff())
        {
            if(
                (CurTime >=  MonitorTemplate.ClockArr[2]*60 + MonitorTemplate.ClockArr[3])
            && 	(CurTime <  MonitorTemplate.ClockArr[4]*60 + MonitorTemplate.ClockArr[5])
            )
            /*  out of the montior range, set to beginning  */
            {
                AlarmHH = MonitorTemplate.ClockArr[4];
                AlarmMM = MonitorTemplate.ClockArr[5];
                #if defined (FreeRunMode_Debug) || (defined Monitor_Template_Debug)
                    printf("cross set start alarm = %d, %d\r\n",AlarmHH,AlarmMM);
                #endif
                SetCalendar_Alarm_A(AlarmHH,AlarmMM,1);
            }
            else if(
                (CurTime >=  MonitorTemplate.ClockArr[4]*60 + MonitorTemplate.ClockArr[5])
                || 	(CurTime <  MonitorTemplate.ClockArr[2]*60 + MonitorTemplate.ClockArr[3])
            )
            {
                /* In the Free Run Time, but the Free Run is not kuck off */
                CurTime = (CurTime + 1) % (24*60);

                if(CurTime == MonitorTemplate.ClockArr[2]*60 + MonitorTemplate.ClockArr[3])
                {
                    /*  already the end time point, then set to next beginning point */
                    if(MonitorTemplate.LightMTinfo.LightSampleFreq == 0)
                    {
                        AlarmHH = MonitorTemplate.ClockArr[4];
                        AlarmMM = MonitorTemplate.ClockArr[5];

                        #if defined (FreeRunMode_Debug) || (defined Monitor_Template_Debug)
                            printf("cross set start alarm = %d, %d\r\n",AlarmHH,AlarmMM);
                        #endif
                        SetCalendar_Alarm_A(AlarmHH,AlarmMM,1);
                    }
                    else
                    {
                        FreeRunMTSetLightCtlTime(CurTime);
                    }
                }
                else
                {
                    /*  Set to next minutes     */
                    AlarmHH = CurTime / 60;
                    AlarmMM = CurTime % 60;

                    #if defined (FreeRunMode_Debug) || (defined Monitor_Template_Debug)
                        printf("cross set start alarm = %d, %d\r\n",AlarmHH,AlarmMM);
                    #endif
                    SetCalendar_Alarm_A(AlarmHH,AlarmMM,1);
                }
            }

        }
        else
        {
            if(
                (CurTime >=  MonitorTemplate.ClockArr[2]*60 + MonitorTemplate.ClockArr[3])
            && 	(CurTime <  MonitorTemplate.ClockArr[4]*60 + MonitorTemplate.ClockArr[5])
            )
            /*  out of the montior range, set to beginning  */
            {
                /* Stop Free Run */
                #ifdef FreeRunMode_Debug
                    printf("stop spo2 1 \r\n");
                #endif
                TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop);			/* 	Stop HR/SPO2 realtime sample 	*/

                #ifdef FreeRunMode_Debug
                    printf("deinit free run when change MT \r\n");
                #endif
                FreeRunModeDeInit();

                if(MonitorTemplate.LightMTinfo.LightSampleFreq == 0)
                {
                    DisCalendar_Alarm_A();

                    AlarmHH = MonitorTemplate.ClockArr[4];
                    AlarmMM = MonitorTemplate.ClockArr[5];
                    #if defined (FreeRunMode_Debug) || (defined Monitor_Template_Debug)
                        printf("cross set start alarm = %d, %d\r\n",AlarmHH,AlarmMM);
                    #endif
                    SetCalendar_Alarm_A(AlarmHH,AlarmMM,1);
                }
            }
            else if(
                (CurTime >=  MonitorTemplate.ClockArr[4]*60 + MonitorTemplate.ClockArr[5])
                || 	(CurTime <  MonitorTemplate.ClockArr[2]*60 + MonitorTemplate.ClockArr[3])
            )
            {
                /*  running and in the montior range  */
                AlarmHH = MonitorTemplate.ClockArr[2];
                AlarmMM = MonitorTemplate.ClockArr[3];

                #if defined (FreeRunMode_Debug) || (defined Monitor_Template_Debug)
                    printf("cross set stop alarm = %d, %d\r\n",AlarmHH,AlarmMM);
                #endif
                SetCalendar_Alarm_A(AlarmHH,AlarmMM,1);
            }
        }
    }
    else  //û�п���
    {
        if(false == isFreeRunKickOff())
        {
            if(
                (CurTime <  MonitorTemplate.ClockArr[0]*60 + MonitorTemplate.ClockArr[1])
            || 	(CurTime >=  MonitorTemplate.ClockArr[2]*60 + MonitorTemplate.ClockArr[3])
            )
            {
                AlarmHH = MonitorTemplate.ClockArr[0];
                AlarmMM = MonitorTemplate.ClockArr[1];
                #if defined (FreeRunMode_Debug) || (defined Monitor_Template_Debug)
                    printf("set FreeRun MT start alarm = %d, %d\r\n",AlarmHH,AlarmMM);
                #endif

                SetCalendar_Alarm_A(AlarmHH,AlarmMM,1);
            }
            else if(
                (CurTime >=  MonitorTemplate.ClockArr[0]*60 + MonitorTemplate.ClockArr[1])
                && 	(CurTime <  MonitorTemplate.ClockArr[2]*60 + MonitorTemplate.ClockArr[3])
            )
            {
                /* In the Free Run Time, but the Free Run is not kuck off */
                CurTime = (CurTime + 1) % (24*60);

                if(CurTime == MonitorTemplate.ClockArr[2]*60 + MonitorTemplate.ClockArr[3])
                {
                    /*  already the end time point, then set to next beginning point */
                    if(MonitorTemplate.LightMTinfo.LightSampleFreq == 0)
                    {
                        AlarmHH = MonitorTemplate.ClockArr[0];
                        AlarmMM = MonitorTemplate.ClockArr[1];

                        #if defined (FreeRunMode_Debug) || (defined Monitor_Template_Debug)
                            printf("set FreeRun MT start alarm = %d, %d\r\n",AlarmHH,AlarmMM);
                        #endif

                        SetCalendar_Alarm_A(AlarmHH,AlarmMM,1);
                    }
                    else
                    {
                        FreeRunMTSetLightCtlTime(CurTime);
                    }
                }
                else
                {
                    /*  Set to next minutes     */
                    AlarmHH = CurTime / 60;
                    AlarmMM = CurTime % 60;

                    #if defined (FreeRunMode_Debug) || (defined Monitor_Template_Debug)
                        printf("set FreeRun MT start alarm = %d, %d\r\n",AlarmHH,AlarmMM);
                    #endif

                    SetCalendar_Alarm_A(AlarmHH,AlarmMM,1);
                }
            }
        }
        else
        {
            if(
                (CurTime <  MonitorTemplate.ClockArr[0]*60 + MonitorTemplate.ClockArr[1])
            || 	(CurTime >=  MonitorTemplate.ClockArr[2]*60 + MonitorTemplate.ClockArr[3])
            )
            {
                /* Stop Free Run */
                #ifdef FreeRunMode_Debug
                    printf("stop spo2 2 \r\n");
                #endif
                TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop);			/* 	Stop HR/SPO2 realtime sample 	*/

                #ifdef FreeRunMode_Debug
                    printf("deinit freerun when reach the end \r\n");
                #endif
                FreeRunModeDeInit();

                if(MonitorTemplate.LightMTinfo.LightSampleFreq == 0)
                {
                    DisCalendar_Alarm_A();

                    AlarmHH = MonitorTemplate.ClockArr[0];
                    AlarmMM = MonitorTemplate.ClockArr[1];
                    #if defined (FreeRunMode_Debug) || (defined Monitor_Template_Debug)
                        printf("set start alarm = %d, %d\r\n",AlarmHH,AlarmMM);
                    #endif
                    SetCalendar_Alarm_A(AlarmHH,AlarmMM,1);
                }
            }
            else if(
                (CurTime >=  MonitorTemplate.ClockArr[0]*60 + MonitorTemplate.ClockArr[1])
                && 	(CurTime <  MonitorTemplate.ClockArr[2]*60 + MonitorTemplate.ClockArr[3])
            )
            {
                /* Free run is kick off, set the stop time */
                AlarmHH = MonitorTemplate.ClockArr[2];
                AlarmMM = MonitorTemplate.ClockArr[3];
                #if defined (FreeRunMode_Debug) || (defined Monitor_Template_Debug)
                    printf("set stop alarm = %d, %d\r\n",AlarmHH,AlarmMM);
                #endif
                SetCalendar_Alarm_A(AlarmHH,AlarmMM,1);
            }
        }
    }
}
/*******************************************************************************
* Function Name  : FreeRunMTChangeToNinghtMT
* Description    : �л���ҹ�����з����������ÿ�ʼ����ʱ��
* Input          : None
* Output         : None
* Return         : Status
*******************************************************************************/
static void FreeRunMTChangeToNinghtMT(uint16_t CurTime)
{
    //�л���ҹ��໤��������
    #ifdef Monitor_Template_Debug
        printf("Change to Ninght MT\r\n");
    #endif
    SetFreeRunMTCtlTime(CurTime);
}

/*******************************************************************************
* Function Name  : AdjustMT
* Description    : ���ݵ�ǰʱ����´η�������ʱ��ȷ����������ʱ��
* Input          : None
* Output         : None
* Return         : Status
*******************************************************************************/
static uint8_t AdjustMT(uint16_t CurTime, uint16_t NextTime)
{
    uint8_t i=0;

    //��ǰʱ�����´�ʱ����Ƿ���ҹ�䷽��ʱ��
    if(CurTime <= NextTime)  //δ����
    {
        if((MonitorTemplate.SampleTimeClock.DataLen/4) == 2) //ҹ�䷽���п���
        {
            if(
                ((CurTime < (MonitorTemplate.ClockArr[2]*60 + MonitorTemplate.ClockArr[3]))
                    && (NextTime > (MonitorTemplate.ClockArr[0]*60 + MonitorTemplate.ClockArr[1])))
             || ((CurTime < (MonitorTemplate.ClockArr[6]*60 + MonitorTemplate.ClockArr[7]))
                    && (NextTime > (MonitorTemplate.ClockArr[4]*60 + MonitorTemplate.ClockArr[5])))
            )
            {
                i = 2;   //����ҹ�䷽��
            }
        }
        else    //ҹ�䷽��û�п���
        {
            if(
                (CurTime < (MonitorTemplate.ClockArr[2]*60 + MonitorTemplate.ClockArr[3]))
             && (NextTime > (MonitorTemplate.ClockArr[0]*60 + MonitorTemplate.ClockArr[1]))
            )
            {
                i = 2;   //����ҹ�䷽��
            }
        }
    }
    else  //���0��
    {
        if((MonitorTemplate.SampleTimeClock.DataLen/4) == 2) //ҹ�䷽���п���
        {
            if(CurTime <= (MonitorTemplate.ClockArr[4]*60 + MonitorTemplate.ClockArr[5]))
            {
                if(((MonitorTemplate.ClockArr[4]*60 + MonitorTemplate.ClockArr[5]) - CurTime)
                    < (MonitorTemplate.LightMTinfo.LightSampleFreq/60))
                {
                    i = 2;     //����ҹ�䷽��
                }
            }
            else
            {
                i = 2;     //����ҹ�䷽��
            }
        }
        else    //ҹ�䷽��û�п���
        {
            if(NextTime > (MonitorTemplate.ClockArr[0]*60 + MonitorTemplate.ClockArr[1]))
            {
                i = 2;     //����ҹ�䷽��
            }
        }
    }

    return i;
}
/*******************************************************************************
* Function Name  : FreeRunMTSetLightCtlTime
* Description    : ���ݵ�ǰʱ���ж��Ƿ��л�Ϊ���췽��
* Input          : None
* Output         : None
* Return         : Status
*******************************************************************************/
static void FreeRunMTSetLightCtlTime(uint16_t CurTime)
{
    uint16_t        i=0;
    uint16_t        MTTime=0;   //���ģ��ʱ��ת��Ϊ����
    uint32_t        AlarmHH=0;  //����Сʱ
    uint32_t        AlarmMM=0;  //���ӷ���

    for(i=0;i<(MonitorTemplate.LightMTinfo.DataLen/4);i++)
    {
        if(CurTime < (MonitorTemplate.LightMTinfo.LightMTClockArr[i*4]*60 + MonitorTemplate.LightMTinfo.LightMTClockArr[i*4+1]))
        {
            AlarmHH = MonitorTemplate.LightMTinfo.LightMTClockArr[i*4];
            AlarmMM = MonitorTemplate.LightMTinfo.LightMTClockArr[i*4+1];
            MTTime = CurTime + (MonitorTemplate.LightMTinfo.LightSampleFreq/60);
            break;
        }
        else if((CurTime + (MonitorTemplate.LightMTinfo.LightSampleFreq/60)) <
            (MonitorTemplate.LightMTinfo.LightMTClockArr[i*4+2]*60 + MonitorTemplate.LightMTinfo.LightMTClockArr[i*4+3]))
        {
            //AlarmHH = (CurTime + (MonitorTemplate.LightMTinfo.LightSampleFreq/60)) / 60;
            //AlarmMM = (CurTime + (MonitorTemplate.LightMTinfo.LightSampleFreq/60)) % 60;

            /********************************************************************************
            //                               Cur_Time - Orgin_Time
            //   Next_Time = Orgin_Time + (------------------------- + 1)*freg  (min)
            //                                        freq
            **********************************************************************************/
            #if 0
            MTTime = (MonitorTemplate.LightMTinfo.LightMTClockArr[i*4]*60 + MonitorTemplate.LightMTinfo.LightMTClockArr[i*4+1])
               + ((CurTime - (MonitorTemplate.LightMTinfo.LightMTClockArr[i*4]*60 +
                    MonitorTemplate.LightMTinfo.LightMTClockArr[i*4+1]))/(MonitorTemplate.LightMTinfo.LightSampleFreq/60)+1)*(MonitorTemplate.LightMTinfo.LightSampleFreq/60);
            #else
            MTTime = CurTime + (MonitorTemplate.LightMTinfo.LightSampleFreq/60);
            #endif
            AlarmHH = (MTTime / 60);
            if(AlarmHH >= 24)
            {
                AlarmHH = 0;
            }
            AlarmMM = (MTTime % 60);
            break;
        }
        else if((CurTime + (MonitorTemplate.LightMTinfo.LightSampleFreq/60)) >= 1440) //����24��
        {
            #if 0
            MTTime = (MonitorTemplate.LightMTinfo.LightMTClockArr[i*4]*60 + MonitorTemplate.LightMTinfo.LightMTClockArr[i*4+1])
               + ((CurTime - (MonitorTemplate.LightMTinfo.LightMTClockArr[i*4]*60 +
                    MonitorTemplate.LightMTinfo.LightMTClockArr[i*4+1]))/(MonitorTemplate.LightMTinfo.LightSampleFreq/60)+1)*(MonitorTemplate.LightMTinfo.LightSampleFreq/60);
            #else
            MTTime = CurTime + (MonitorTemplate.LightMTinfo.LightSampleFreq/60);
            #endif
            MTTime = MTTime - 1440;

            AlarmHH = (MTTime / 60);
            if(AlarmHH >= 24)
            {
                AlarmHH = 0;
            }
            AlarmMM = (MTTime % 60);
            break;
        }
        else if(CurTime <= (MonitorTemplate.ClockArr[i*4]*60 + MonitorTemplate.ClockArr[i*4+1]))
        {
            i=2;  //��ҹ�䷽������ʱ����ڣ��л���ҹ�䷽��
            break;
        }
    }
    if(MTTime >= 1440)
    {
        MTTime = MTTime - 1440;
    }
    if(AdjustMT(CurTime,MTTime) == 2)
    {
        i = 2;
    }

    if(i<(MonitorTemplate.LightMTinfo.DataLen/4))
    {
        SetCalendar_Alarm_A(AlarmHH,AlarmMM,1);
    }
    else if(MonitorTemplate.LightMTinfo.LightSampleFreq != 0) //�л���ҹ�䷽��
    {
        FreeRunMTChangeToNinghtMT(CurTime);
    }

    #ifdef Monitor_Template_Debug
    if(i<(MonitorTemplate.LightMTinfo.DataLen/4))
    {
        printf("Next Light Alarm Time: Hour=%d,Minute=%d\r\n",AlarmHH,AlarmMM);
    }
    else if(MonitorTemplate.LightMTinfo.LightSampleFreq == 0)
    {
        printf("This day Light monitor template have passed\r\n");
    }
    #endif
}

/*******************************************************************************
* Function Name  : FindNextAlarmTimePoint
* Description    : ������Ի�������һ������ʱ���
* Input          : u32CurrTime: The current of system
* Input          : pu32AlarmHH: The hours of next run-time
* Input          : pu32AlarmMM: The minutes of next run-time
* Input          : pu8Flag  : The flag which next run-time is over 23:59
* Return         : Status,0 - success,-1 - fail
*******************************************************************************/
int32_t FindNextAlarmTimePoint(uint32_t u32CurrTime,uint32_t * pu32AlarmHH,uint32_t * pu32AlarmMM,uint8_t * pu8Flag)
{
    uint32_t i = 0;
    uint32_t temp_time = 0;
    int32_t ret = 0;           // 0 - success,-1 - fail

    #ifdef Monitor_Template_Debug
        printf("\r\nFindNextAlarmTimePoint,u32CurrTime = 0x%08x\r\n",u32CurrTime);
    #endif

    if((pu32AlarmHH != NULL) && (pu32AlarmMM != NULL))
    {
        temp_time = u32CurrTime + (MonitorTemplate.SampleFreq/60);
        temp_time %= 1440;                              // 24*60 = 1440
        #ifdef Monitor_Template_Debug
            printf("temp_time = %d\r\n",temp_time);
        #endif

        for(i = 0;i < (MonitorTemplate.SampleTimeClock.DataLen/4);i ++)
        {
            #ifdef Monitor_Template_Debug
                printf("i = %d\r\n",i);
                printf("MonitorTemplate.ClockArr[i*4] = %d\r\n",MonitorTemplate.ClockArr[i*4]);
                printf("MonitorTemplate.ClockArr[i*4+1] = %d\r\n",MonitorTemplate.ClockArr[i*4+1]);
                printf("MonitorTemplate.ClockArr[i*4+2] = %d\r\n",MonitorTemplate.ClockArr[i*4+2]);
                printf("MonitorTemplate.ClockArr[i*4+3] = %d\r\n",MonitorTemplate.ClockArr[i*4+3]);
            #endif
            if((temp_time >= (MonitorTemplate.ClockArr[i*4]*60 + MonitorTemplate.ClockArr[i*4+1]))
            && (temp_time <= (MonitorTemplate.ClockArr[i*4+2]*60 + MonitorTemplate.ClockArr[i*4+3])))
            {
                *pu32AlarmHH = temp_time / 60;
                *pu32AlarmMM = temp_time % 60;

                if(pu8Flag != NULL)
                {
                    if((u32CurrTime < 1440) && ((u32CurrTime + (MonitorTemplate.SampleFreq/60)) >= 1440))  // 24*60 = 1440
                    {
                        *pu8Flag = 1;
                    }
                    else
                    {
                        *pu8Flag = 0;
                    }
                }
                break;
            }
        }

        if(i >= (MonitorTemplate.SampleTimeClock.DataLen/4))
        {
            ret = 0;
            #ifdef Monitor_Template_Debug
                printf("ret = 0\r\n");
            #endif
            if(i == 1)
            {
                *pu32AlarmHH = MonitorTemplate.ClockArr[0];
                *pu32AlarmMM = MonitorTemplate.ClockArr[1];   
            }  
            else if(i == 2)
            {
                *pu32AlarmHH = MonitorTemplate.ClockArr[4];
                *pu32AlarmMM = MonitorTemplate.ClockArr[5];                   
            }
            return (ret);
        }
    }
    else
    {
        ret = -1;
        #ifdef Monitor_Template_Debug
            printf("ret = -1\r\n");
        #endif
        return (ret);
    }

    *pu32AlarmHH %= 24;
    ret = 0;
    #ifdef Monitor_Template_Debug
        printf("ret = 0\r\n");
    #endif
    return (ret);
}


