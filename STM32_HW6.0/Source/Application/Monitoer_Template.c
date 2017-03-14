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
static bool flagLightMonitorTemplateStatus = OFF;  //ON：白天，OFF：夜间
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
* Description    : 解包监测模板数据
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

		while((p[in] = strtok_r(buf, "#", &outer_ptr))!=NULL)   //提取监测模板
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
		for(i=0;i<in-1;i++)   //打印分割后的监测模板
		{
			printf("p[%d]= %s,len=%d\r\n",i,p[i],strlen(p[i]));
		}
		#endif

		MonitorTemplate.SampleFreq = atoi(p[0]);  //p[0]保存监测频率
		MonitorTemplate.SampleWorkDay = 0x00;     //p[2]保存Workday
		for(i=0;i<7;i++)
		{
			if(p[2][i] == 0x31)
			{
				MonitorTemplate.SampleWorkDay |= (0x80 >> (i+1));
			}
		}
		MonitorTemplate.SampleLastTime = atoi(p[3]); //p[3]保存每次持续时间
		MonitorTemplate.SampleID = 0x0000;

		str_len_t = strlen(p[4]);
		for(i = 0;i < str_len_t;i++)                  //p[4]保存采集参数ID
		{
			if(p[4][i] == 0x31)
			{
				MonitorTemplate.SampleID |= (0x0001 << (str_len_t-i-1));
			}
		}
		ExcetionMonitorTemplate.SampleID = 0x0000;
		str_len_t = strlen(p[5]);
		for(i = 0;i < str_len_t;i++)                  //p[5]保存异常采集参数ID
		{
			if(p[5][i] == 0x31)
			{
				ExcetionMonitorTemplate.SampleID |= (0x0001 << (str_len_t-i-1));
			}
		}

		ExcetionMonitorTemplate.SampleLastTime = atoi(p[6]); //p[6]保存异常连续采集时间
		ExcetionMonitorTemplate.SampleFreq = atoi(p[7]);   //p[7]保存异常采集频率
		MonitorTemplate.MTSwitch = atoi(p[8]);             //p[8]保存监测模板开?
		MonitorTemplate.VibrateSwitch = atoi(p[9]);        //p[9]振动开关

		if((in-1) > 10) //兼容监测模板中未发运动历史采集频率及监测模板ID标示
		{
			MonitorTemplate.SetMTID = atoi(p[10]);  //p[10]保存监测模板ID
			MonitorTemplate.MotionInfoUpdateFreq = atoi(p[11]); //p[11]保存运动数据更新频率
            //p[12]:白天采集时间段
            MonitorTemplate.LightMTinfo.LightSampleFreq = atoi(p[13]);   //p[13]保存白天采集频率
		}
		else   //以前版本App，未加，用默认
		{
			MonitorTemplate.SetMTID = kICTMonitorTemplateAltitudeStressID;  //默认监测模板ID
			MonitorTemplate.SampleID = 7;
			ExcetionMonitorTemplate.SampleID = 7;
			MonitorTemplate.MotionInfoUpdateFreq = 900; //默认运动数据更新频率
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

		if(MonitorTemplate.SampleTimeClock.DataLen > MaxMTLen)  //接收的监测模板长度大于最大监测模板长度
		{
			#ifdef Monitor_Template_Debug
				printf("Err1:  Receive MT Len Beyond MaxMTLen\r\n");
			#endif
			return ;
		}
		memset(MonitorTemplate.SampleTimeClock.Context,0,MaxMTLen*sizeof(uint8_t));
		if(MonitorTemplate.SampleFreq == 0)//按时间点采集
		{
			for(i=0;i<(MonitorTemplate.SampleTimeClock.DataLen);i++)        //p[1]保存监测每天时间点
			{
				MonitorTemplate.SampleTimeClock.Context[i]=(uint8_t)(((p[1][i*3])-0x30)*10 + (p[1][i*3+1]-0x30));
			}
		}
		else  //按时间段采集,将其划分为时间点
		{
			memset(MonitorTemplate.ClockArr,0,MaxClockArrLen*sizeof(uint8_t));
			if(MonitorTemplate.SampleTimeClock.DataLen > MaxClockArrLen)  //接收的监测模板长度大于最大监测模板长度
			{
				return ;
			}

			for(i=0;i<(MonitorTemplate.SampleTimeClock.DataLen);i++)        //p[1]保存监测每天时间段
			{
				MonitorTemplate.ClockArr[i]=(uint8_t)(((p[1][i*3])-0x30)*10 + (p[1][i*3+1]-0x30));
			}

			if((MonitorTemplate.SampleTimeClock.DataLen/4) == 2)  //有跨天
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

            if(MonitorTemplate.SetMTID != MTID)   //更换监护方案
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
		}//end of 按时间段采集


        /* Light Monitor Templte info Analysis */
        MonitorTemplate.LightMTinfo.DataLen = (strlen(p[12]) / 3) + 1;
        if(MonitorTemplate.LightMTinfo.DataLen > MaxClockArrLen)  //接收的白天监护方案长度大于最大长度
        {
            #ifdef Monitor_Template_Debug
				printf("Err1:  Receive Light MT Len Beyond MaxMTLen\r\n");
			#endif
			return ;
        }
        memset(MonitorTemplate.LightMTinfo.LightMTClockArr,0,MaxClockArrLen*sizeof(uint8_t));
        if(MonitorTemplate.LightMTinfo.LightSampleFreq == 0)
        {
            //白天监护方案监测频率为0，不执行白天部分

        }
        else  //有白天监测方案
        {
            for(i=0;i<(MonitorTemplate.LightMTinfo.DataLen);i++)   //p[12]保存白天监护方案时间段
            {
                MonitorTemplate.LightMTinfo.LightMTClockArr[i] = (uint8_t)(((p[12][i*3])-0x30)*10 + (p[12][i*3+1]-0x30));
            }

            if((MonitorTemplate.LightMTinfo.DataLen/4) == 2)  //有跨天
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
		TS_SendEvent(gTsMonitorTemplatID_c,gMonitorTemplateEventStartSet);    //产生监测模板开始设置事件

		if(Device_Mode == Device_Mode_Factory)  //工厂模式下，事件调度还不能用
		{
			MonitorTemplate_Task_Handler(gMonitorTemplateEventStartSet);
		}
		if((Device_Mode == Device_Mode_RTC) && ((Get_OLED_Dis_Status() == OLEDDisEnterShutDown) || (Get_OLED_Dis_Status() == OLEDDisON)))
		{
			//RTC显示模式下，收到监测模板后更新RTC界面中的监测模板图标
			if(MonitorTemplate.MTSwitch == 0) //暂停监测模板
			{
				OLED_DisplayMonitorTemplateICON(OFF); 							//clear the Monitor template icon
			}
			else
			{
				OLED_DisplayMonitorTemplateICON(ON); //显示监测模板
			}
		}
}
/*******************************************************************************
* Function Name  : MonitorTemplate_Task_Handler
* Description    : 处理监测模板事件
* Input          : MonitorTemplate_Event
* Output         : None
* Return         : None
*******************************************************************************/
void MonitorTemplate_Task_Handler(event_t MonitorTemplate_Event)
{
		uint8_t 		Date=0x01;
		uint16_t 		i=0;
		static int16_t 	SampleClock=0;  //采集数据点
		static uint16_t ExcetionSampleClock=0; //异常监测模板采集数据点
		date_str_typedef    date_s;               //RTC 日期
		RTC_TimeTypeDef     rtc_time;             //RTC 时间
		uint16_t        MTTime=0;   //监测模板时间转换为分钟
		uint16_t        CurTime=0;  //当前时间转换为分钟
		uint32_t        AlarmHH=0;  //闹钟小时
		uint32_t        AlarmMM=0;  //闹钟分钟
//        uint32_t        u32Date = 0;
        uint8_t         u8Flag = 0;

		Calendar_Get(&date_s,&rtc_time);
		CurTime = rtc_time.RTC_Hours*60 + rtc_time.RTC_Minutes;

		if(MonitorTemplate_Event == gMonitorTemplateEventStartSet)   //App连接时或每天00：00时查看有无当前监测模板
		{
			Calendar_Get(&date_s,&rtc_time);

			#ifdef Monitor_Template_Debug
                printf("\r\n***gMonitorTemplateEventStartSet\r\n");
                printf("Current Time:");
                printf("RTC Hour=%d,",rtc_time.RTC_Hours);
                printf("RTC Min=%d\r\n",rtc_time.RTC_Minutes);
                printf("RTC Week=%d\r\n",date_s.week);
			#endif

			if((MonitorTemplate.SampleWorkDay & (Date<<(date_s.week-1))) == (Date<<(date_s.week-1)))  //Work Day等于当天
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

				SetMonitorTemplateMeaResultType(NormalVal);  //设置监测模板采集结果为正常
				SetMonitorTemplateType(NormalMT); //设置监测模板类型为正常监测模板
				if(MonitorTemplate.SampleFreq == 0) //按照时间点采集
				{
					/* 找出当前最近时间点 ,时间段也在监测模板数据解析中转化为时间点*/
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
							//监测模板时间与当前时间转换为分钟进行比较
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
					/* 找到当前最近时间点 */
					SampleClock = i;
					if((SampleClock < (MonitorTemplate.SampleTimeClock.DataLen)/2))  //找到最近时间点
					{
						/* check if current time equals to first alarm time */
						if(rtc_time.RTC_Minutes == MonitorTemplate.SampleTimeClock.Context[SampleClock*2+1])
						{
							SampleClock++;  //point to next sample point
						}
						if(MonitorTemplate.MTSwitch == 0) //暂停监测模板
						{
							DisCalendar_Alarm_A();
							#ifdef Monitor_Template_Debug
								printf("Suspend MT\r\n");
							#endif
						}
						else  //开始监测模板
						{
							SetCalendar_Alarm_A(MonitorTemplate.SampleTimeClock.Context[SampleClock*2],MonitorTemplate.SampleTimeClock.Context[SampleClock*2+1],1);
							#ifdef Monitor_Template_Debug
                                printf("First Alarm = %d\r\n",SampleClock);
                                printf("1,First Alarm Time: Hour=%d,Minute=%d\r\n",MonitorTemplate.SampleTimeClock.Context[SampleClock*2],
																			MonitorTemplate.SampleTimeClock.Context[SampleClock*2+1]);
							#endif
						}
						//SetCalendar_Alarm_A(rtc_time.RTC_Hours,rtc_time.RTC_Minutes,rtc_time.RTC_Seconds+1);  //for test
						SampleClock++;  //指向下一个监测模板时间采集点
					}
					#ifdef Monitor_Template_Debug
					else
					{
						//当天监测模板已过
						#ifdef Monitor_Template_Debug
                            printf("This day monitor template have passed\r\n");
						#endif
					}
					#endif
				}
				else	//按照时间段采集
				{
					//将当前系统时间与检测模板时间段进行比较
					CurTime = rtc_time.RTC_Hours*60 + rtc_time.RTC_Minutes;

					if(MonitorTemplate.MTSwitch == 0) //暂停监测模板
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

                            if(flagLightMonitorTemplateStatus == ON)//在白天方案运行时间段内
                            {
                                FreeRunMTSetLightCtlTime(CurTime);
                            }
                            else    //不在白天方案运行时间段内,夜间方案运行
                            {
                                SetFreeRunMTCtlTime(CurTime);
                            }
//							/*Set AlarmB to Monitor the status of FreeRunning */
//							SetCalendar_Alarm_B(AlarmHH,AlarmMM,30);
						}
						else    //MonitorTemplate.MTID != kICTMonitorTemplateFreeRunID(个性化方案)
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

							//计算下次采集时间点
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
			}//end of Work Day等于当天
			#ifdef Monitor_Template_Debug
			else  //当天没有监测模板运行，输出调试信息
			{
				#ifdef Monitor_Template_Debug
				printf("There is no monitor template in the day%d",date_s.day);
				#endif
			}
			#endif

            SPI_AlarmTransmit(Alarm_ID_Reuse_MTSetSuccess);  //发送监护方案设置结果，暂时使用告警通道

		}//end of if(MonitorTemplate_Event == gMonitorTemplateEventStartSet)   //App连接时或每天00：00时查看有无当前监测模板
		else if(MonitorTemplate_Event == gMonitorTemplateEventSampleData) //按照监测模板采集数据
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
                    ((flagLightMonitorTemplateStatus == ON) && (flagIsFreeRunKickOff == true))) //不在白天方案运行中,夜晚方案运行
                {
                    if((MonitorTemplate.SampleTimeClock.DataLen/4) == 2)  //有跨天
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

            /* 获取当前监护方案状态 */
            flagLightMonitorTemplateStatus = GetLightMonitorTemplateStatus();

            if(
                (MonitorTemplate.MTID != kICTMonitorTemplateFreeRunID)
            ||  ((MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID) && (flagLightMonitorTemplateStatus == ON))
            )    //个信化方案运行 或者  鼾症类方案白天部分方案运行
			{
				if((Device_Mode != Device_Mode_LowBattery) && (Device_Mode != Device_Mode_Charge)
					&& (Device_Mode != Device_Mode_FWUpdate))
				{
                    if(FALSE == bFlagNewLightStart)
                    {
    					/* 产生当前监测模板采集数据事件 */
    					if(MonitorTemplate.SampleID & SAMPLE_ID_SpO2)  //采集血氧
    					{
    						#ifdef Monitor_Template_Debug
    							printf("Monitor Template SpO2\r\n");
    						#endif
    						gSubFunc_Stat_Set(SpO2_MonitorTemplate_State,ON);      //SpO2监测模板在运行
    						/* 开启HR/SpO2测量前，将佩戴检测功能设置为测量模式 */
    						Wear_Detect_Set(WEAR_DETECT_INC);
    						TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStart);     //发送SpO2开启测量事件,SpO2和HR测量属于同一事件
                        }

    					if(MonitorTemplate.SampleID & SAMPLE_ID_HeartRate)  //采集心率
    					{
    						#ifdef Monitor_Template_Debug
    							printf("Monitor Template Sample HeartRate\r\n");
    						#endif
    						gSubFunc_Stat_Set(HR_MonitorTemplate_State, ON);      //HR监测模板在运行

    						/* 开启HR/SpO2测量前，将佩戴检测功能设置为测量模式 */
    						Wear_Detect_Set(WEAR_DETECT_INC);
						    TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStart);     //发送SpO2开启测量事件,SpO2和HR测量属于同一事件
    					}
                    }
                    else
                    {
                        bFlagNewLightStart = FALSE;
                    }
				}
			}

            //计算个性化方案下次测量时间
			if(MonitorTemplate.MTID != kICTMonitorTemplateFreeRunID)  //free run
			{
				if(MonitorTemplate.SampleFreq == 0) //按照时间点采集
				{
					if(SampleClock <((MonitorTemplate.SampleTimeClock.DataLen)/2))  //未超出监测模板时间点范围
					{
						#ifdef Monitor_Template_Debug
						//printf("Set Next Alarm = %d",SampleClock);
						printf("Next Alarm Time:Hour=%d,Minute=%d\r\n",MonitorTemplate.SampleTimeClock.Context[SampleClock*2],
											MonitorTemplate.SampleTimeClock.Context[SampleClock*2+1]);
						#endif
						SetCalendar_Alarm_A(MonitorTemplate.SampleTimeClock.Context[SampleClock*2],
											MonitorTemplate.SampleTimeClock.Context[SampleClock*2+1],1); //设置下一个监测模板时间点
						SampleClock ++;
					}
					else  //监测模板执行完成，关闭RTC闹钟中断
					{
						#ifdef Monitor_Template_Debug
							printf("Monitor Template Conplated,Disable Alarm\r\n");
						#endif
						DisCalendar_Alarm_A();
						SampleClock=0;
					}
				}
				else  //按照时间段采集
				{
					Calendar_Get(&date_s,&rtc_time);
					//将当前系统时间与检测模板时间段进行比较
					CurTime = rtc_time.RTC_Hours*60 + rtc_time.RTC_Minutes;

					if(MonitorTemplate.MTSwitch == 0) //暂停监测模板
					{
						DisCalendar_Alarm_A();
						#ifdef Monitor_Template_Debug
							printf("Suspend MT\r\n");
						#endif
					}
					else
					{
						//计算下次采集时间点
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

            //计算鼾症类方案白天部分方案下次测量时间
            if((MonitorTemplate.MTID == kICTMonitorTemplateFreeRunID) && (MonitorTemplate.LightMTinfo.LightSampleFreq != 0))
            {
                Calendar_Get(&date_s,&rtc_time);
                //将当前系统时间与检测模板时间段进行比较
                CurTime = rtc_time.RTC_Hours*60 + rtc_time.RTC_Minutes;

                if(MonitorTemplate.MTSwitch == 0) //暂停监测模板
                {
                    DisCalendar_Alarm_A();
                    #ifdef Monitor_Template_Debug
                        printf("Light Suspend MT\r\n");
                    #endif
                }
                else //方案未暂停，计算鼾症类方案白天部分方案下次测量时间
                {
                    flagLightMonitorTemplateStatus = GetLightMonitorTemplateStatus();
                    if(flagLightMonitorTemplateStatus == ON)  //白天监护方案在运行
                    {
                        FreeRunMTSetLightCtlTime(CurTime);
                    }
                    else    //不在白天方案运行时间段内,夜间方案运行
                    {
                        SetFreeRunMTCtlTime(CurTime);
                    }
                }
            }
        }// end of if(MonitorTemplate_Event == gMonitorTemplateEventSampleData) //按照监测模板采集数据
		else if(MonitorTemplate_Event == gMonitorTemplateEventTest)
		{
			Calendar_Get(&date_s,&rtc_time);
			SetCalendar_Alarm_A(rtc_time.RTC_Hours,rtc_time.RTC_Minutes+1,rtc_time.RTC_Seconds);
		}
		if(MonitorTemplate_Event == gExcetionMonitorTemplateEventStart)  //产生异常监测模板开始事件
		{
			if((Device_Mode == Device_Mode_LowBattery) || (Device_Mode == Device_Mode_Charge)
				|| (Device_Mode == Device_Mode_FWUpdate))
			{
				return;
			}
			if(GetMonitorTemplateType() == NormalMT)  //正常监测模板的情况下才开始
			{
				if((ExcetionMonitorTemplate.SampleID == CheckUpStop) || (ExcetionMonitorTemplate.SampleFreq == 0u))
				{
					//异常监测模板中参数ID为0，不进行异常监测模板操作
					#ifdef Monitor_Template_Debug
						printf("ExcetionMonitor ID : 0\r\n");
					#endif
				}
				else
				{
					StartExcetionMonitorTemplate();    //开始异常监测模板
				}
			}
		}
		if(MonitorTemplate_Event == gExcetionMonitorTemplateEventStop)  //产生异常监测模板停止事件
		{
			if(GetMonitorTemplateType() == ExcetionMT)  //异常监测模板的情况下才开始
			{
				StopExcetionMonitorTemplate();    //停止异常监测模板
			}
		}
		if(MonitorTemplate_Event == gExcetionMonitorTemplateEventSet)  //产生异常监测模板设置事件
		{
			if((Device_Mode == Device_Mode_LowBattery) || (Device_Mode == Device_Mode_Charge)
				|| (Device_Mode == Device_Mode_FWUpdate))
			{
				return;
			}
			Calendar_Get(&date_s,&rtc_time);  //获取当前时间
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
			/* 找到当前最近时间点 */
			ExcetionSampleClock = i;
			if(ExcetionSampleClock < ExcetionMonitorTemplate.SampleTimeClock.DataLen)  //找到最近时间点
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
				ExcetionSampleClock++;  //指向下一个监测模板时间采集点
			}
		}
		if(MonitorTemplate_Event == gExcetionMonitorTemplateEventSampleData)  //产生异常监测模板采集数据事件
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

			/* 产生当前监测模板采集数据事件 */
			if(ExcetionMonitorTemplate.SampleID & SAMPLE_ID_SpO2)  //采集血氧
			{
				#ifdef Monitor_Template_Debug
					printf("Excetion Monitor Template SpO2\r\n");
				#endif
				gSubFunc_Stat_Set(SpO2_MonitorTemplate_State, ON);      //SpO2监测模板在运行

				/* 开启HR/SpO2测量前，将佩戴检测功能设置为测量模式 */
				Wear_Detect_Set(WEAR_DETECT_INC);
				TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStart);     //发送SpO2开启测量事件,SpO2和HR测量属于同一事件
			}
			if(ExcetionMonitorTemplate.SampleID & SAMPLE_ID_HeartRate)  //采集心率
			{
				#ifdef Monitor_Template_Debug
					printf("Excetion Monitor Template Sample HeartRate\r\n");
				#endif
				gSubFunc_Stat_Set(HR_MonitorTemplate_State,ON);      //HR监测模板在运行

				/* 开启HR/SpO2测量前，将佩戴检测功能设置为测量模式 */
				Wear_Detect_Set(WEAR_DETECT_INC);
				TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStart);     //发送SpO2开启测量事件,SpO2和HR测量属于同一事件
			}
			if(Flag_Excetion_MTRecovery == true)
			{
				if(ExcetionSampleClock < (ExcetionMonitorTemplate.SampleTimeClock.DataLen+1))  //未超出监测模板时间点范围
				{
					#ifdef Monitor_Template_Debug
					printf("Next Alarm Time:Hour=%d,Minute=%d\r\n",ExcetionMonitorTemplate.SampleTimeClock.Context[ExcetionSampleClock*2],
										ExcetionMonitorTemplate.SampleTimeClock.Context[ExcetionSampleClock*2+1]);
					#endif
					SetCalendar_Alarm_A(ExcetionMonitorTemplate.SampleTimeClock.Context[ExcetionSampleClock*2],
										ExcetionMonitorTemplate.SampleTimeClock.Context[ExcetionSampleClock*2+1],1); //设置下一个监测模板时间点
					ExcetionSampleClock ++;
				}
				else  //一趟异常监测模板执行完成
				{
					#ifdef Monitor_Template_Debug
						printf("A Trip Excetion Monitor Template Conplated\r\n");
					#endif
					ExcetionSampleClock=0;
					TS_SendEvent(gTsMonitorTemplatID_c,gExcetionMonitorTemplateEventStop);  //产生异常监测模板停止事件
				}
			}
			else
			{
				if(ExcetionSampleClock < (ExcetionMonitorTemplate.SampleTimeClock.DataLen))  //未超出监测模板时间点范围
				{
					#ifdef Monitor_Template_Debug
					printf("Next Alarm Time:Hour=%d,Minute=%d\r\n",ExcetionMonitorTemplate.SampleTimeClock.Context[ExcetionSampleClock*2],
										ExcetionMonitorTemplate.SampleTimeClock.Context[ExcetionSampleClock*2+1]);
					#endif
					SetCalendar_Alarm_A(ExcetionMonitorTemplate.SampleTimeClock.Context[ExcetionSampleClock*2],
										ExcetionMonitorTemplate.SampleTimeClock.Context[ExcetionSampleClock*2+1],1); //设置下一个监测模板时间点
					ExcetionSampleClock ++;
				}
				else  //一趟异常监测模板执行完成
				{
					#ifdef Monitor_Template_Debug
						printf("A Trip Excetion Monitor Template Conplated\r\n");
					#endif
					ExcetionSampleClock=0;
					TS_SendEvent(gTsMonitorTemplatID_c,gExcetionMonitorTemplateEventStop);  //产生异常监测模板停止事件
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
                    TS_SendEvent(gTsMonitorTemplatID_c,gMonitorTemplateEventStartSet);    //每天晚上12点产生监测模板开始设置事件
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

/***************异常监测模板执行******************************************************************/
/*******************************************************************************
* Function Name  : StartExcetionMonitorTemplate
* Description    : 开始执行异常检测模板
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void StartExcetionMonitorTemplate(void)
{
		date_str_typedef    date_s;               //RTC 日期
		RTC_TimeTypeDef     rtc_time;             //RTC 时间
		uint8_t    SampleEveryTime = 0;           //每次采集时间，通过采集频率转换而来
		uint8_t    i=0;
		uint8_t    index = 0;

		/* Init Excetion Monitor Template Status */
		SetMonitorTemplateType(ExcetionMT);  // 监测模板类型为异常监测模板
		SetMonitorTemplateMeaResultType(NormalVal);
		DisCalendar_Alarm_A();

		Calendar_Get(&date_s,&rtc_time);

		#ifdef Monitor_Template_Debug
			printf("\r\nCalc the Excetion MT Run Clcok...\r\n");
			printf("\r\nThe current time is :  %0.2d:%0.2d:%0.2d\r\n", rtc_time.RTC_Hours, rtc_time.RTC_Minutes, rtc_time.RTC_Seconds);
		#endif

		ExcetionMonitorTemplate.SampleTimeClock.DataLen = ExcetionMonitorTemplate.SampleLastTime / ExcetionMonitorTemplate.SampleFreq; // 300 / 60=5
		SampleEveryTime = ExcetionMonitorTemplate.SampleFreq / 60;  //将采集频率转化为分钟
		memset(ExcetionMonitorTemplate.SampleTimeClock.Context,0,MaxEMTLen*sizeof(uint8_t));

	    ExcetionMonitorTemplate.SampleTimeClock.Context[1] = rtc_time.RTC_Minutes + SampleEveryTime;  //第一次异常监测模板在当前时间
																									  //SampleEveryTime之后进行
		if(ExcetionMonitorTemplate.SampleTimeClock.Context[1] >= 60) //分溢出，时加1
		{
			ExcetionMonitorTemplate.SampleTimeClock.Context[1] = ExcetionMonitorTemplate.SampleTimeClock.Context[1] % 60;
			ExcetionMonitorTemplate.SampleTimeClock.Context[0] = rtc_time.RTC_Hours + 1;
			if(ExcetionMonitorTemplate.SampleTimeClock.Context[0] >= 24)  //时溢出
			{
				ExcetionMonitorTemplate.SampleTimeClock.Context[0] =0;
			}
		}
		else  //分未溢出
		{
			ExcetionMonitorTemplate.SampleTimeClock.Context[0] = rtc_time.RTC_Hours;
		}

		index = 2;
		for(i=0;i<(ExcetionMonitorTemplate.SampleTimeClock.DataLen) && ExcetionMonitorTemplate.SampleTimeClock.DataLen <= MaxEMTLen;i++) //已经计算了一次异常监测模板执行时间
		{
			 ExcetionMonitorTemplate.SampleTimeClock.Context[index+1] = ExcetionMonitorTemplate.SampleTimeClock.Context[index-1] + SampleEveryTime;  //SampleEveryTime之后进行
			if(ExcetionMonitorTemplate.SampleTimeClock.Context[index+1] >= 60) //分溢出，时加1
			{
				ExcetionMonitorTemplate.SampleTimeClock.Context[index+1] = ExcetionMonitorTemplate.SampleTimeClock.Context[index+1] % 60;
				ExcetionMonitorTemplate.SampleTimeClock.Context[index] =  ExcetionMonitorTemplate.SampleTimeClock.Context[index-2] + 1;
				if(ExcetionMonitorTemplate.SampleTimeClock.Context[index] >= 24)  //时溢出
				{
					ExcetionMonitorTemplate.SampleTimeClock.Context[index] =0;
				}
			}
			else  //分未溢出
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
		TS_SendEvent(gTsMonitorTemplatID_c,gExcetionMonitorTemplateEventSet);  //产生异常监测模板设置事件

		if(Flag_Excetion_MTRecovery == false)  //已从异常监测模板中恢复，下一分钟开始测量；否则，从当前分钟开始测量
		{
			/* 开启第一次异常采集 */
			/* 产生当前监测模板采集数据事件 */
			if(ExcetionMonitorTemplate.SampleID & SAMPLE_ID_SpO2)  //采集血氧
			{
				#ifdef Monitor_Template_Debug
					printf("First Excetion Monitor Template SpO2\r\n");
				#endif
				gSubFunc_Stat_Set(SpO2_MonitorTemplate_State ,ON);      //SpO2监测模板在运行

				/* 开启HR/SpO2测量前，将佩戴检测功能设置为测量模式 */
				Wear_Detect_Set(WEAR_DETECT_INC);
				TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStart);     //发送SpO2开启测量事件,SpO2和HR测量属于同一事件
			}
			if(ExcetionMonitorTemplate.SampleID & SAMPLE_ID_HeartRate)  //采集心率
			{
				#ifdef Monitor_Template_Debug
					printf("First Excetion Monitor Template Sample HeartRate\r\n");
				#endif
				gSubFunc_Stat_Set(HR_MonitorTemplate_State,ON);      //HR监测模板在运行

				/* 开启HR/SpO2测量前，将佩戴检测功能设置为测量模式 */
				Wear_Detect_Set(WEAR_DETECT_INC);
				TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStart);     //发送SpO2开启测量事件,SpO2和HR测量属于同一事件
			}
		}
}
/*******************************************************************************
* Function Name  : StopExcetionMonitorTemplate
* Description    : 结束执行异常检测模板
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void StopExcetionMonitorTemplate(void)
{
		static uint8_t ExcetionMTValCnt = 0;  //异常监测模板采集得到正常数据次数

		/* 停止监测模板运行 */
                            #ifdef FreeRunMode_Debug
                                printf("stop spo2 6 \r\n");
                            #endif

		gSubFunc_Stat_Set(SPO2_HR_MONITOR_STATE, OFF);
		TS_SendEvent(gTsSpO2TaskID_c,gSpO2EventStop); //发送SpO2停止测量事件，HR/SpO2测量为同一事件控制,此时必定有HR值

		/* 发送一趟异常监测模板执行结束帧 */
		SendExcetionMonitorTemplateStopFram();
		SetMonitorTemplateType(NormalMT);  //Stop异常监测模板后，将其设置为正常监测模板

		/* 判断是否执行下一趟异常监测模板 */
		if(GetMonitorTemplateMeaResultType() == ExcetionVal)  //异常监测模板异常结果
		{
			//异常监测模板中测得结果异常,开启下一趟异常监测模板
			ExcetionMTValCnt = 0;
			Flag_Excetion_MTRecovery = false;
			TS_SendEvent(gTsMonitorTemplatID_c,gExcetionMonitorTemplateEventStart);  //产生异常监测模板开始事件
			#ifdef Monitor_Template_Debug
				printf("Start Next Excetion Monitor Template A\r\n");
			#endif
		}else if(GetMonitorTemplateMeaResultType() == NormalVal)  //异常监测模板中结果正常
		{
			ExcetionMTValCnt = ExcetionMTValCnt+1;
			if(ExcetionMTValCnt >= 2)  //连续两次采集到正常数据
			{
				#ifdef Monitor_Template_Debug
					printf("Stop Excetion Monitor Template,Back to Normal MT\r\n");
				#endif
				ExcetionMTValCnt = 0;
				Flag_Excetion_MTRecovery = true;
                bFlagNewDayEvent = FALSE;
				TS_SendEvent(gTsMonitorTemplatID_c,gMonitorTemplateEventStartSet);   //设置正常监测模板
			}
			else   //没有连续两次采集到正常数据
			{
				#ifdef Monitor_Template_Debug
					printf("Start Next Excetion Monitor Template B\r\n");
				#endif
				Flag_Excetion_MTRecovery = false;
				TS_SendEvent(gTsMonitorTemplatID_c,gExcetionMonitorTemplateEventStart);  //产生异常监测模板开始事件
			}
		}
}
static void SendExcetionMonitorTemplateStopFram(void)
{
		date_str_typedef    date_s;               //RTC 日期
		RTC_TimeTypeDef     rtc_time;             //RTC 时间
		uint8_t 			StopFram[MT_STORAGE_DATA_LEN] = {0};
		uint8_t  		    Err_Code=0;
        uint32_t            u32Date = 0;

		Calendar_Get(&date_s,&rtc_time);

		/* 发送一趟异常监测模板执行结束帧 */
		StopFram[0]=rtc_time.RTC_Hours;
		StopFram[1]=rtc_time.RTC_Minutes;
		StopFram[2]=rtc_time.RTC_Seconds;

		StopFram[3]=StopExcetionFram;
		StopFram[4] = 0;
		StopFram[5] = 0;
		StopFram[6] = 0;
		StopFram[7] = 0;

		/* 将有效的Stop值写入EEPROM中 */
		//if(GetM95M01State(MONITOR_MODEL_VALUE_ID,M95M01_CAPACITY_SPACE) < sizeof(StopFram))
        if(0)
		{
				//监测模板数据剩余存贮空间小于写入的数据大小，存贮空间报警
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

        if(Device_State == Device_WorkWithAPP)   //和App连接在一起并且所有监测模板运行结束，触发同步事件
        {
            TS_SendEvent(gTsSyncDataTaskID_c,gSyncDataFormat2EventStart);  //产生开始数据同步事件
        }
}
/*******************************************************************************
* Function Name  : GetMonitorTemplateType
* Description    : 获取监测模板类型：NromalMT Or ExcetionMT
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
* Description    : 设置监测模板类型
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
* Description    : 设置监测模板测量结果类型
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
* Description    : 设置监测模板测量结果类型
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
	if(Device_Mode == Device_Mode_CheckUp)  //及时采集模式下
	{
		if(gSubFunc_Stat_Get(SpO2_RealTimeSample_State | HR_RealTimeSample_State) == OFF)
		{
			OLEDDisplay_Stat_Set(NormalDis);  //App及时采集完成显示Normal模式
			Device_Mode = Device_Mode_RTC;    //停止采集后，回到RTC模式
			Set_OLED_Dis_Status(OLEDDisON);
			TS_SendEvent(gOledDisTaskID,gOledDisEventBlueIcon_c);   //显示RTC模式
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
    if(SPO2_HR_Measure_State == Start)    //SPO2/HR正在测量过程中，短按停止SPO2/HR测量
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
        ResetSingleModeAlarmVirbreTim();  //停止时，清除振动告警间隔时间
        TS_SendEvent(gOledDisTaskID,gOledDisEventModeHR_c); //发送OLED显示SpO2/HR模式界面事件
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

    date_str_typedef    date_s;               //RTC 日期
    RTC_TimeTypeDef     rtc_time;             //RTC 时间
    uint16_t        CurTime=0;  //当前时间转换为分钟
    uint32_t        AlarmHH=0;  //闹钟小时
    uint32_t        AlarmMM=0;  //闹钟分钟
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
    bufForStorage[7] = gHRSpO2Val.HrSpO2DataRep_pst->m_ui8ConfVal;      //置信度

#endif

    /* 	Begin to store the data 			*/
    /* Create The Moniter Template Data Store Partition */
    u32Date = CovernDateto32();
    ExtFLASH_ExtCreatePartition(u32Date);

    /*	Write EEPROM 		*/
    Err_Code = DataMemoryWrite(MONITOR_TYPE,bufForStorage,sizeof(bufForStorage));
    APP_ERROR_CHECK(Err_Code);

    /* if the CurTime is not included by the FreeRun Time , stop the FreeRun*/
    if((MonitorTemplate.SampleTimeClock.DataLen/4) == 2)  //有跨天
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
    else  //未跨天
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
    date_str_typedef    date_s;        //RTC 日期
    RTC_TimeTypeDef     rtc_time;      //RTC 时间
    uint16_t            CurTime=0;     //当前时间转换为分钟
    uint32_t            AlarmHH=0;     //闹钟小时
    uint32_t            AlarmMM=0;     //闹钟分钟
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
            //设置白天监护方案开始时间
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
                else if((CurTime + (MonitorTemplate.LightMTinfo.LightSampleFreq/60)) >= 1440) //大于24点
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
                    i=2;  //在夜间方案运行时间段内。切换至夜间方案
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
            else if(MonitorTemplate.LightMTinfo.LightSampleFreq != 0) //切换至夜间方案
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
        else    //不在白天方案运行时间段内，夜间方案运行
        {
            SetFreeRunMTCtlTime(CurTime);
        }
    }
    ExtFLASH_StorageCheck();
}
/*******************************************************************************
* Function Name  : Set_FreeRunStatusMonitorRecord_Time
* Description    : 设置监测FreeRun运行状态时间
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Set_FreeRunStatusMonitorRecord_Time(void)
{
	uint32_t            AlarmBHH=0;  //B闹钟小时
	uint32_t            AlarmBMM=0;  //B闹钟分钟
	RTC_TimeTypeDef     rtc_time;             //RTC 时间
	uint16_t            MotionInfoUpdateFreq = 120; //运动数据更新频率，默认15min

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
* Description    : 获取白天监护方案状态
* Input          : None
* Output         : None
* Return         : Status
*******************************************************************************/
static bool GetLightMonitorTemplateStatus(void)
{
    bool LightMonitorTemplateStatus = OFF;    //ON：白天，OFF：夜间
    date_str_typedef    date_s;               //RTC日期
	RTC_TimeTypeDef     rtc_time;             //RTC时间
    uint16_t            CurTime=0;            //当前时间转换为分钟

    Calendar_Get(&date_s,&rtc_time);
	CurTime = rtc_time.RTC_Hours*60 + rtc_time.RTC_Minutes;

    if(MonitorTemplate.LightMTinfo.LightSampleFreq == 0)  //白天监护方案频率为0
    {
        LightMonitorTemplateStatus = OFF;  //运行夜间监护方案
    }
    else
    {
        if((MonitorTemplate.SampleTimeClock.DataLen/4) == 2)  //夜间方案有跨天
        {
            if(
                (CurTime >= MonitorTemplate.ClockArr[2]*60 + MonitorTemplate.ClockArr[3])
            &&  (CurTime < MonitorTemplate.LightMTinfo.LightMTClockArr[2]*60 + MonitorTemplate.LightMTinfo.LightMTClockArr[3])
            )
            {
                LightMonitorTemplateStatus = ON; //运行白天监护方案
            }
            else
            {
                LightMonitorTemplateStatus = OFF; //运行夜间监护方案
            }
        }
        else if((MonitorTemplate.LightMTinfo.DataLen / 4) == 2) //白天方案有跨天
        {
            if(
                (CurTime >= MonitorTemplate.LightMTinfo.LightMTClockArr[2]*60 + MonitorTemplate.LightMTinfo.LightMTClockArr[3])
            &&  (CurTime < MonitorTemplate.ClockArr[2]*60 + MonitorTemplate.ClockArr[3])
            )
            {
                LightMonitorTemplateStatus = OFF;   //运行夜间监护方案
            }
            else
            {
                LightMonitorTemplateStatus = ON;    //运行白天监护方案
            }
        }
        else   //白天、夜晚均未跨天
        {
            if((MonitorTemplate.ClockArr[0]*60 + MonitorTemplate.ClockArr[1])
                >= (MonitorTemplate.LightMTinfo.LightMTClockArr[0]*60 + MonitorTemplate.LightMTinfo.LightMTClockArr[1])
            )
            {
                if(
                    (CurTime < MonitorTemplate.LightMTinfo.LightMTClockArr[2]*60 + MonitorTemplate.LightMTinfo.LightMTClockArr[3])
                )
                {
                    LightMonitorTemplateStatus = ON;   //运行白天监护方案
                }
                else
                {
                    LightMonitorTemplateStatus = OFF;  //运行夜间监护方案
                }
            }
            else
            {
                if(
                    (CurTime >= MonitorTemplate.ClockArr[2]*60 + MonitorTemplate.ClockArr[3] )
                &&  (CurTime < MonitorTemplate.LightMTinfo.LightMTClockArr[2]*60 + MonitorTemplate.LightMTinfo.LightMTClockArr[3])
                )
                {
                    LightMonitorTemplateStatus = ON;   //运行白天监护方案
                }
                else
                {
                    LightMonitorTemplateStatus = OFF;  //运行夜间监护方案
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
* Description    : 设置鼾症类方案运行时间（启动/停止时间）
* Input          : None
* Output         : None
* Return         : Status
*******************************************************************************/
static void SetFreeRunMTCtlTime(uint16_t CurTime)
{
    uint32_t        AlarmHH=0;  //闹钟小时
	uint32_t        AlarmMM=0;  //闹钟分钟

    if((MonitorTemplate.SampleTimeClock.DataLen/4) == 2)  //有跨天
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
    else  //没有跨天
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
* Description    : 切换至夜间运行方案，并设置开始运行时间
* Input          : None
* Output         : None
* Return         : Status
*******************************************************************************/
static void FreeRunMTChangeToNinghtMT(uint16_t CurTime)
{
    //切换至夜间监护方案部分
    #ifdef Monitor_Template_Debug
        printf("Change to Ninght MT\r\n");
    #endif
    SetFreeRunMTCtlTime(CurTime);
}

/*******************************************************************************
* Function Name  : AdjustMT
* Description    : 根据当前时间和下次方案运行时间确定方案运行时间
* Input          : None
* Output         : None
* Return         : Status
*******************************************************************************/
static uint8_t AdjustMT(uint16_t CurTime, uint16_t NextTime)
{
    uint8_t i=0;

    //当前时间与下次时间间是否有夜间方案时间
    if(CurTime <= NextTime)  //未跨天
    {
        if((MonitorTemplate.SampleTimeClock.DataLen/4) == 2) //夜间方案有跨天
        {
            if(
                ((CurTime < (MonitorTemplate.ClockArr[2]*60 + MonitorTemplate.ClockArr[3]))
                    && (NextTime > (MonitorTemplate.ClockArr[0]*60 + MonitorTemplate.ClockArr[1])))
             || ((CurTime < (MonitorTemplate.ClockArr[6]*60 + MonitorTemplate.ClockArr[7]))
                    && (NextTime > (MonitorTemplate.ClockArr[4]*60 + MonitorTemplate.ClockArr[5])))
            )
            {
                i = 2;   //运行夜间方案
            }
        }
        else    //夜间方案没有跨天
        {
            if(
                (CurTime < (MonitorTemplate.ClockArr[2]*60 + MonitorTemplate.ClockArr[3]))
             && (NextTime > (MonitorTemplate.ClockArr[0]*60 + MonitorTemplate.ClockArr[1]))
            )
            {
                i = 2;   //运行夜间方案
            }
        }
    }
    else  //跨过0点
    {
        if((MonitorTemplate.SampleTimeClock.DataLen/4) == 2) //夜间方案有跨天
        {
            if(CurTime <= (MonitorTemplate.ClockArr[4]*60 + MonitorTemplate.ClockArr[5]))
            {
                if(((MonitorTemplate.ClockArr[4]*60 + MonitorTemplate.ClockArr[5]) - CurTime)
                    < (MonitorTemplate.LightMTinfo.LightSampleFreq/60))
                {
                    i = 2;     //运行夜间方案
                }
            }
            else
            {
                i = 2;     //运行夜间方案
            }
        }
        else    //夜间方案没有跨天
        {
            if(NextTime > (MonitorTemplate.ClockArr[0]*60 + MonitorTemplate.ClockArr[1]))
            {
                i = 2;     //运行夜间方案
            }
        }
    }

    return i;
}
/*******************************************************************************
* Function Name  : FreeRunMTSetLightCtlTime
* Description    : 根据当前时间判断是否切换为白天方案
* Input          : None
* Output         : None
* Return         : Status
*******************************************************************************/
static void FreeRunMTSetLightCtlTime(uint16_t CurTime)
{
    uint16_t        i=0;
    uint16_t        MTTime=0;   //监测模板时间转换为分钟
    uint32_t        AlarmHH=0;  //闹钟小时
    uint32_t        AlarmMM=0;  //闹钟分钟

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
        else if((CurTime + (MonitorTemplate.LightMTinfo.LightSampleFreq/60)) >= 1440) //大于24点
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
            i=2;  //在夜间方案运行时间段内，切换至夜间方案
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
    else if(MonitorTemplate.LightMTinfo.LightSampleFreq != 0) //切换至夜间方案
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
* Description    : 计算个性化方案下一个运行时间点
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


