#include "MotorCtl.h"
#include "common.h"

static Timer_ID_Typedef MotorTIM_ID = TIMER_ERROR;
static uint8_t   Motor_Vibrate_Times=0;    //马达振动次数
static uint8_t   Motor_Vibrate_Cnt=0;
static uint8_t   EnableVibrateCnt=0;
static uint8_t 		flagMotorStatus = 0;

/*******************************************************************************
* Function Name  : Motor_Init
* Description    : 电机控制IO口初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Motor_Init(void)
{
		GPIO_InitTypeDef        GPIO_InitStructure;
	
		RCC_AHBPeriphClockCmd(RCC_MotorPeriph, ENABLE);
	
		/* Configure  LED0 pin in output pushpull mode */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_Motor;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIO_Motor, &GPIO_InitStructure);
}
/*******************************************************************************
* Function Name  : Motor_DeInit
* Description    : 电机控制IO口初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Motor_DeInit(void)
{
		GPIO_InitTypeDef        GPIO_InitStructure;
	
		/* Configure  LED0 pin in output pushpull mode */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_Motor;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
		//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
		GPIO_Init(GPIO_Motor, &GPIO_InitStructure);
}
/*******************************************************************************
* Function Name  : MotorCtl
* Description    : 控制电机的开启
* Input          : ENABLE\DISABLE
* Output         : None
* Return         : None
*******************************************************************************/
static void MotorCtl(uint8_t status)
{
		if((status == ENABLE) && (MonitorTemplate.VibrateSwitch == ENABLE))
		{
			if(OFF == GetPeriphPowerStatus())  //开启外部电源
			{
				PeriPower(ON);
			}
            Motor_Init();
			GPIO_ResetBits(GPIO_Motor,GPIO_Pin_Motor);
		}
		else if(status == DISABLE)
		{
			Motor_DeInit();
		}
}

static void Motor_Vibrate(Timer_ID_Typedef TIMID)
{	
	TIMID = TIMID;

	EnableVibrateCnt++;

	/* 	stop the timer 		*/
	Stop_Timer_Cnt(MotorTIM_ID);

	//printf("Times=%d\r\n",Motor_Vibrate_Cnt);
	if(EnableVibrateCnt == 1)
	{
		/* Init Motor Periph */
		MotorCtl(ENABLE);
		Motor_Vibrate_Cnt++;
		/* 	restart the timer 		*/
		Start_Timer_Cnt(MotorTIM_ID);
	}
	else
	{
		StopMotor();
		if(EnableVibrateCnt>=5)  //5*200m后重新振动
		{
			EnableVibrateCnt=0;
		}
		
		if(Motor_Vibrate_Cnt >= Motor_Vibrate_Times)  //超过振动次数
		{
			KillMotor();
		}
		else
		{
			/* 	restart the timer 		*/
			Start_Timer_Cnt(MotorTIM_ID);
		}
	}		
}
void StartMotor(uint8_t MotorVibrateTime)
{
	TIM_Basic_Cfg_Typedef   Tim_Cfg_Motor;       //Motor timer配置，用于指示程序运行
	TIM_Cfg_Typedef         Tim_Cfg_Motor_Index;	
	
	if(flagMotorStatus == 0u)
	{
	
		Motor_Vibrate_Times = MotorVibrateTime;  //振动次数
		
		/* Configure Motor control timer 	*/
		Tim_Cfg_Motor.enuTimerType = TIM_TYPE_MS;
		Tim_Cfg_Motor.u16TimePeriod = Motor_Vibrate_Time;
		Tim_Cfg_Motor.NVIC_IRQChannelPreemptionPriority = Motor_TIMPreemptionPriority;
		Tim_Cfg_Motor.pIntCallBack = Motor_Vibrate;

		/* Init timer top define */
		Tim_Cfg_Motor_Index.TimerMode        = TIM_MODE_BASIC;
		Tim_Cfg_Motor_Index.TimerBasicCfg    = &Tim_Cfg_Motor;
		Tim_Cfg_Motor_Index.TimerPWMCfg 	   = NULL;

		Timer_Free(MotorTIM_ID);
		MotorTIM_ID = Timer_Allocate(&Tim_Cfg_Motor_Index);
		//printf("Allocate Time=%d\r\n",MotorTIM_ID);
		flagMotorStatus = 1u;
		Start_Timer_Cnt(MotorTIM_ID);
	}
}



void StopMotor(void)
{
		MotorCtl(DISABLE);
		Motor_DeInit();
}
void KillMotor(void)
{
	StopMotor();
	Motor_Vibrate_Cnt = 0;
	EnableVibrateCnt=0;
	Timer_Free(MotorTIM_ID);
	//printf("free timer = %d \n", MotorTIM_ID);
	MotorTIM_ID = TIMER_ERROR;
	flagMotorStatus = 0u;
	/* 	Clear alarm type, only motor 	*/
	if(AlarmStatusGet() == true)
	{
		//printf("clear in moto \r\n");
		AlarmStatusClear(ALARM_TYPE_MOTOR_MASK);
	}
}




