#include "ParameterLimit.h"
#include <string.h>
#include <stdlib.h>

static int32_t i32TimeLastSec = -PARAMETER_ALARM_TIME_SLOT;

/*******************************************************************************
* Function Name  : UnPackParameterLimit
* Description    : �������������������
* Input          : p_ParameterLimit,len
* Output         : None
* Return         : None
*******************************************************************************/
void UnPackParameterLimit(uint8_t *p_ParameterLimit,uint16_t Len)
{
		uint8_t i=0;
		uint8_t ID=0;
		char *buf=NULL;
		char *p[20]={0};
		uint8_t in=0;
		char *inner_ptr=NULL;
	
		char ParameterLimit[ParamrterLimitMaxLen]={0};
		
	
		#ifdef Parameter_Limit_Debug
		printf("Unpack Parameter Limit...\r\n");
		printf("Parameter Length:%d\r\n",Len);
		for(i=0;i<Len;i++)
		{
			printf("0x%x, ",p_ParameterLimit[i]);
		}
		printf("\r\n");
		#endif
								 				 
		ID = p_ParameterLimit[0];  //����ID��
		for(i=1;i<Len;i++)   //�������������������,//ȥ������������������֡ǰ1������,������ParameterLimit��//��1������Ϊ����ID
		{
			ParameterLimit[i-1]=p_ParameterLimit[i];
		}
		
		buf=ParameterLimit;
		
		while((p[in]=strtok_r(buf, "#", &inner_ptr))!=NULL)  //
		{
			in++;
			buf=NULL;
		}
		
		#ifdef Parameter_Limit_Debug
		printf("parameter limit info...\r\n");
		for(i=0;i<in;i++)   //��ӡ�ָ��Ĳ�������
		{
			printf("p[%d]= %s,len=%d\r\n",i,p[i],strlen(p[i]));
		}
		printf("\r\n");
		#endif

		/* ���������ID����ֵ */
		switch(ID)
		{
			case HeartRateID:
				#ifdef Parameter_Limit_Debug
				printf("Set HR Limit...\r\n");
				#endif
				gHRSpO2Val.HR_Limit.VLowLimit = atoi(p[0]);
				gHRSpO2Val.HR_Limit.LowLimit  = atoi(p[1]);
				gHRSpO2Val.HR_Limit.HigLimit  = atoi(p[2]);
				gHRSpO2Val.HR_Limit.VHigLimit  = atoi(p[3]);				
				break;
			case SpO2ID:
				#ifdef Parameter_Limit_Debug
				printf("Set SpO2 Limit...\r\n");
				#endif
				gHRSpO2Val.SpO2_Limit.VLowLimit = atoi(p[0]);
				gHRSpO2Val.SpO2_Limit.LowLimit  = atoi(p[1]);
				gHRSpO2Val.SpO2_Limit.HigLimit  = atoi(p[2]);
				gHRSpO2Val.SpO2_Limit.VHigLimit  = atoi(p[3]);
				break;
			default:
				#ifdef  Parameter_Limit_Debug
					printf("parameter limit ID error...\r\n");
				#endif
				break;
		}
}
/*******************************************************************************
* Function Name  : Alarm_Task_Handler
* Description    : task handler for alarm
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Alarm_Task_Handler(event_t AlarmEvent)
{
	
	date_str_typedef        CurDisNotEnoughSpace_date_s;               //cur��ʾ�ռ䲻�����RTC ����
	RTC_TimeTypeDef         CurDisNotEnoughSpace_rtc_time;             //cur��ʾ�ռ䲻�����RTC ʱ��
	static uint32_t         PreDisNotEnoughSpace_Time=0;
	uint32_t                CurDisNotEnoughSpace_Time=0;
	
	RTC_TimeTypeDef 			curTime;
	int32_t 	curTimeInt = 0;
	bool 	flagTimeSlotOut = false;
	
	if(AlarmEvent & gAlarmEventNoFinger)    //No finger alarm 
	{
		SPI_AlarmTransmit(Alarm_ID_SpO2NoFinger);
		if(Device_Mode == Device_Mode_CheckUp)  //��ʱ�ɼ�ģʽ��
		{
			gSubFunc_Stat_Set(SpO2_RealTimeSample_State | HR_RealTimeSample_State, OFF);
			ChangeDeviceMode_FromCheckUpToRTC();
		}
	}
	if(AlarmEvent & gAlarmEventParameterLimit)   //�������޸߱���
	{
		
		/*Get current time 	*/
		flagTimeSlotOut = false;

		RTC_GetTime(RTC_Format_BIN, &curTime);
		curTimeInt = (curTime.RTC_Hours * 3600) + (curTime.RTC_Minutes * 60) + curTime.RTC_Seconds;
		if(curTimeInt - i32TimeLastSec > PARAMETER_ALARM_TIME_SLOT)
		{
			flagTimeSlotOut = true;
			/* 	store current timer point 	*/
			i32TimeLastSec = curTimeInt;
		}
		if(AlarmStatusGet() == false)		/* 	If alarm is false 		*/
		{
			if( flagTimeSlotOut == true )		/* 	In case timeout flag is true	*/
			{
				/* 	Set alarm status 		*/
				AlarmStatusSet(true);
				
				/* 	Configure alarm types 	*/
				if(AlarmLevelGet() == ALARM_LEVEL_HIGH) //�߸澯
				{
						/* 	Flash Red LED 	*/
						//LED_DeInit();
						//LED_Init(LED_RED,500,120);
				}
				else if(AlarmLevelGet() == ALARM_LEVEL_MID) // �еȱ���
				{
						/* 	Add Code here for mid level alarm 	*/
				}
				else if(AlarmLevelGet() == ALARM_LEVEL_LOW) //�ͱ���
				{
						/* 	Flash Orange LED 	*/
						//LED_DeInit();
						//LED_Init(LED_ORANGE,500,120);
				}
				
				/* 	Vibrate 3 times 		*/
				StartMotor(3);
			}
		}
	}
	if(AlarmEvent & gAlarmEventNotEnoughSpace)  //�����ռ䲻�㱨��
	{
        Calendar_Get(&CurDisNotEnoughSpace_date_s,&CurDisNotEnoughSpace_rtc_time);
        
		CurDisNotEnoughSpace_Time = (CurDisNotEnoughSpace_date_s.day * 1440
									  +	CurDisNotEnoughSpace_rtc_time.RTC_Hours * 60 +CurDisNotEnoughSpace_rtc_time.RTC_Minutes);
		
		if((CurDisNotEnoughSpace_Time - PreDisNotEnoughSpace_Time) > 60)//1Сʱ
		{
			TS_SendEvent(gOledDisTaskID,gOledDisEventNotEnoughSpace_c);  //��ʾ�����ռ䲻�㱨������
			PreDisNotEnoughSpace_Time = CurDisNotEnoughSpace_Time;
		}
	}
	if(AlarmEvent & gAlarmEventFactoryHWTest) //����Ӳ������
	{
		//��������LED��EEPORM���
		Motor_Init();
		GPIO_SetBits(GPIO_Motor,GPIO_Pin_Motor);
		Delay_ms(200);
		GPIO_ResetBits(GPIO_Motor,GPIO_Pin_Motor);
		Delay_ms(200);
		
		GPIO_SetBits(GPIO_Motor,GPIO_Pin_Motor);
		Delay_ms(200);
		GPIO_ResetBits(GPIO_Motor,GPIO_Pin_Motor);
		Delay_ms(200);
		Motor_DeInit();		
				
		MX25L1606EHW_SelfTest();
		
		BMA250EHW_SelfTest();
	}
    if(AlarmEvent & gAlarmEventFreeRunMTRunning) //FreeRun MT is Running
    {
        SPI_AlarmTransmit(Alarm_ID_Reuse_FreeRunMTRunning);
    }
}
/*******************************************************************************
* Function Name  : UnPackBasicInfo
* Description    : ����û�������Ϣ����
* Input          : p_ParameterLimit,len
* Output         : None
* Return         : None
*******************************************************************************/
//void UnPackBasicInfo(uint8_t *p_BasicInfo,uint16_t Len)
//{
//		uint8_t i=0;
//		uint8_t ID = 0;
//	
//		#ifdef BasicInfo_Debug
//		printf("Unpack Basic Info...\r\n");
//		printf("BasicInfo Length:%d\r\n",Len);
//		for(i=0;i<Len;i++)
//		{
//			printf("0x%x, ",p_BasicInfo[i]);
//		}
//		printf("\r\n");
//		#endif
//
//		/* �������������Ϣ */
//		for(i=0;i<(Len/3);i++)
//		{
//			ID = p_BasicInfo[i*3];
//			switch(ID)
//			{
//				case USER_INFO_Height:
//                    if(((p_BasicInfo[i*3+1]*256 + p_BasicInfo[i*3+2]) >= 10) && ((p_BasicInfo[i*3+1]*256 + p_BasicInfo[i*3+2]) <= 250))
//                    {
//                        userInfo.height = p_BasicInfo[i*3+1]*256 + p_BasicInfo[i*3+2];
//                    }
//                    break;
//                    
//				case USER_INFO_Weight:
//                    if(((p_BasicInfo[i*3+1]*256 + p_BasicInfo[i*3+2]) >= 1) && ((p_BasicInfo[i*3+1]*256 + p_BasicInfo[i*3+2]) <= 300))
//                    {
//                        userInfo.weight = p_BasicInfo[i*3+1]*256 + p_BasicInfo[i*3+2];
//                    }
//                    break;
//                
//				case USER_INFO_Age:
//					userInfo.age =(uint8_t)(p_BasicInfo[i*3+1]*0 + p_BasicInfo[i*3+2]);
//                    break;
//                
//				case USER_INFO_Sex:
//					userInfo.sex = (uint8_t)(p_BasicInfo[i*3+1]*0+ p_BasicInfo[i*3+2]);
//                    break;
//                
//				default:
//                    break;				
//			}
//		}
//		
//		#ifdef BasicInfo_Debug
//			printf("Basic User Info\r\n");
//			printf("H:%d\r\n",userInfo.height);
//			printf("W:%d\r\n",userInfo.weight);
//			printf("Age:%d\r\n",userInfo.age);
//			printf("Sex:%d\r\n",userInfo.sex);
//		#endif
//}
/*******************************************************************************
* Function Name  : ResetSingleModeAlarmVirbreTim
* Description    : ��λ���豸ģʽ���񶯸澯ʱ��
* Input          : p_ParameterLimit,len
* Output         : None
* Return         : None
*******************************************************************************/
void ResetSingleModeAlarmVirbreTim(void)
{
	i32TimeLastSec = -PARAMETER_ALARM_TIME_SLOT;
}


