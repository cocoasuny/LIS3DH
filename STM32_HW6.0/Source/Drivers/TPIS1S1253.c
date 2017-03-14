#include "stm32l1xx.h"
#include "TPIS1S1253.h"
#include "Timer.h"
#include "Usart.h"
#include "common.h"


#ifdef TPISMeasureEN

unsigned int    Tobj=0;
unsigned int    Tamb=0;
unsigned int    TPIS=0;


static Timer_ID_Typedef 	gTPISTIMID;     //TPIS定时器ID
static SubModuleStat        m_TPIS_Submodule_stat = SUB_MOD_STOP;   // TPIS module state
static uint8_t TPISTemCnt=0;    //TPIS温度测量数据计数，存5个去掉前2个取最大值

#define TObj_Offs    64473    //需要更具具体的传感器进行调整，25°时读出的目标温度计数(或者叫做DC offset值)
#define TAmb_offs    7801     //需要根据具体的传感器进行调整，25°时读出的环境温度计数
//#define Sens_TP      370    //TPIS 1S 1253
#define Sens_TP      320      //TPIS 1S 1252
#define Sen_Tamb     90


/*******************************************************************************
* Function Name  : TPIS_Init
* Description    : Configures the TPIS1S1253 pin
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TPIS_Init(void)
{
    /* GPIO Periph clock enable */
    GPIO_InitTypeDef        GPIO_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_TPIS1S1253Periph, ENABLE);  //TPIS1S1253 Dlink port
  
    /* Configure  Dlinnk pin in output pushpull mode */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_Dlink;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIO_TPIS1S1253, &GPIO_InitStructure);
	
    GPIO_SetBits(GPIO_TPIS1S1253,GPIO_Pin_Dlink);
}
/*******************************************************************************
* Function Name  : TPIS_DeInit
* Description    : Default Configures the TPIS1S1253 pin
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TPIS_DeInit(void)
{
    /* GPIO Periph clock enable */
    GPIO_InitTypeDef        GPIO_InitStructure;
  
    /* Configure  Dlinnk pin in output pushpull mode */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_Dlink;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
	
    GPIO_Init(GPIO_TPIS1S1253, &GPIO_InitStructure);
}
/*******************************************************************************
* Function Name  : TPIS_Stat_Set
* Description    : 设置TPIS状态
* Input          : newState
* Output         : None
* Return         : None
*******************************************************************************/
void TPIS_Stat_Set(SubModuleStat newState)
{
	m_TPIS_Submodule_stat = newState;
}
/*******************************************************************************
* Function Name  : TPIS_Stat_Get
* Description    : 获取TPIS状态
* Input          : None
* Output         : TPIS State 
* Return         : None
*******************************************************************************/
SubModuleStat TPIS_Stat_Get(void)
{
	return(m_TPIS_Submodule_stat);
}
/*******************************************************************************
* Function Name  : Read_TPIS
* Description    : 读取TPIS1S1253，高17位为Tobj，低14位为Tamb，均保存在全局变量中
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
float Read_TPIS(void)
{
    unsigned char i=0;
	float Temobj=0;
	//float Temamb=0;
    
    Tobj = 0;        //Clear Tobj
    Tamb = 0;        //Clear Tamb
	TPIS=0;
    
	//Set_PD2;
    PD2_OUT();       //Config PD2 Output mode 
    Set_PD2;         //PD2 High, pull up
    Delay_us(90);
    
    for(i=0;i<17;i++) //Read Tobj, 17Bits
    {
        PD2_OUT();
        Reset_PD2;
        //Delay_us(1);
        Set_PD2;
        //Delay_us(1);
        PD2_IN();
		Reset_PD2;
        Tobj <<= 1;
		//Delay_us(10);
        if(1 == GPIO_ReadInputDataBit(GPIO_TPIS1S1253,GPIO_Pin_Dlink))
        {
            Tobj++;
			//Tobj = (Tobj | 0x01);
        }	
    }
    for(i=0;i<14;i++) //Read Tamb,14Bits
    {
        PD2_OUT();
        Reset_PD2;
        //Delay_us(1);
        Set_PD2;
        //Delay_us(1);
        PD2_IN();
		Reset_PD2;
        Tamb <<= 1;
		//Delay_us(10);
        if(1 == GPIO_ReadInputDataBit(GPIO_TPIS1S1253,GPIO_Pin_Dlink))
        {
            Tamb++;
						//Tamb = (Tamb | 0x01);
        }
    }
    PD2_OUT();
    Reset_PD2;
    //Delay_us(5);
    PD2_IN();
	Set_PD2;
				

	//printf("Tobj= %d\r\n",Tobj);
	//printf("Tamb= %d\r\n",Tamb);
	//printf("TPIS= %d\r\n",TPIS);
	Temobj = CalTPISToTem(Tobj,Tamb);
	//Temamb = ((Tamb-TAmb_offs)/Sen_Tamb)+25;
	//Temobj = ((Tobj-TObj_Offs)/Sens_TP)+Temamb;
	
	//printf("Tobj= %0.1f\r\n",Temobj);
	//printf("Tamb= %0.1f\r\n",Temamb);
	
	return Temobj;		
}
float CalTPISToTem(float Cobj,float Camb)
{
		float Temobj=0;
		float Temamb=0;
	
		Temamb = ((Camb-TAmb_offs)/Sen_Tamb)+25;
		Temobj = ((Cobj-TObj_Offs)/Sens_TP)+Temamb;
	
		//printf("TPISTemamb = %0.1f",Temamb);
		//printf("TPISTemObj = %0.1f",Temobj);
	
		return Temobj;
}
/*******************************************************************************
* Function Name  : TPISSample
* Description    : TPISSample,TPIS定时器中断服务函数(回调函数),触发TPIS读取数据事件
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TPISSample(Timer_ID_Typedef TIMID)
{
		TS_SendEvent(gTsTPISTaskID_c,gTPISEventSample);  //产生TPIS读取数据事件
}

/*******************************************************************************
* Function Name  : TPIS_Start
* Description    : TPIS_Start,申请TPIS定时器，控制TPIS采样频率
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TPIS_Start(void)
{
		TIM_Cfg_Typedef         Tim_Cfg_TPIS_Index;       //TPIS timer配置，用于控制TPIS采样频率
		TIM_Basic_Cfg_Typedef 	Tim_Cfg_TPIS;
	
		/* TPIS GPIO Init */
		TPIS_Init();
	
		/* 配置TPIS timer,按照TPISSampleTime进行采样 */
		Tim_Cfg_TPIS.enuTimerType = TIM_TYPE_MS;
		Tim_Cfg_TPIS.u16TimePeriod = TPISSampleTime;
		Tim_Cfg_TPIS.NVIC_IRQChannelPreemptionPriority = TPIS_TIMPreemptionPriority;
		Tim_Cfg_TPIS.pIntCallBack = TPISSample;

		/* Init timer top define */
		Tim_Cfg_TPIS_Index.TimerMode 			= TIM_MODE_BASIC;
		Tim_Cfg_TPIS_Index.TimerBasicCfg 		= &Tim_Cfg_TPIS;
		Tim_Cfg_TPIS_Index.TimerPWMCfg 			= NULL;
	
		gTPISTIMID = Timer_Allocate(&Tim_Cfg_TPIS_Index);
		Start_Timer_Cnt(gTPISTIMID);
}
/*******************************************************************************
* Function Name  : TPIS_Stop
* Description    : TPIS_Stop,释放TPIS定时器
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TPIS_Stop(void)
{
		Timer_Free(gTPISTIMID);
		gTPISTIMID = TIMER_ERROR;
	
		/* TPIS GPIO DeInit */
		TPIS_DeInit();

}
/*******************************************************************************
* Function Name  : TPIS_Task_Handler
* Description    : TPIS_Task_Handler,处理NCT任务
* Input          : TPIS_Event
* Output         : None
* Return         : None
*******************************************************************************/
void TPIS_Task_Handler(event_t TPIS_Event)
{
		static float  TPIS_Tem[5];
		float  MaxTem=0;
		float  TPIS_Val=0;
		uint8_t i=0;
		
	
		if(TPIS_Event & gTPISEventStart)
		{
			if(TPIS_Stat_Get() == SUB_MOD_STOP) //start TPIS module unless the state is Stop 
			{
				TPIS_Stat_Set(SUB_MOD_RUN); //Set the TPIS module state to RUN
				TPIS_Start();       //申请定时器，在定时器中断中采样数据
			}
		}
		else if(TPIS_Event & gTPISEventSample)
		{
				IntDisable();    //关全部中断
				TPIS_Val = Read_TPIS();
				IntEnable();     //开全部中断
			
				if(TPIS_Val >40)
				{
					TPIS_Val = 40;
				}
				
				TPIS_Tem[TPISTemCnt] = TPIS_Val;   //保存TPIS测量结果，获取最大值
				TPISTemCnt++;
			
				if(TPISTemCnt >= 5)  //测量5次取最大值作为TPIS最终结果
				{
					TPISTemCnt = 0;
					for(i=2;i<5;i++)   //舍弃前2个数值，找到最大值
					{
						if(TPIS_Tem[i] > MaxTem)
						{
							MaxTem = TPIS_Tem[i];
							TPIS_Tem[i]=0;
						}
						//printf("MaxTem = %f\r\n",MaxTem);
						//printf("TPIS_Tem[%d] = %f\r\n",i,TPIS_Tem[i]);
					}
					
					/* 保存最终TPIS测量结果，产生停止测量事件 */
					gTPISVal.Value = MaxTem;
					MaxTem=0;
					//StartMotor(1);
					TS_SendEvent(gOledDisTaskID,gOledDisEventTPISResult_c);//产生屏幕显示TPIS测量结果事件
					TS_SendEvent(gTsTPISTaskID_c,gTPISEventStop);     //触发TPIS停止采集事件	
				}
		}
		else if(TPIS_Event & gTPISEventStop)
		{
			if(TPIS_Stat_Get() == SUB_MOD_RUN)  //stop TPIS module unless the state is RUN
			{
				TPIS_Stop();
				TPIS_Stat_Set(SUB_MOD_STOP);
				TPISTemCnt = 0;
			}
		}
}

#endif



