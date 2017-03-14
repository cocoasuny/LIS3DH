#include "stm32l1xx.h"
#include "TPIS1S1253.h"
#include "Timer.h"
#include "Usart.h"
#include "common.h"


#ifdef TPISMeasureEN

unsigned int    Tobj=0;
unsigned int    Tamb=0;
unsigned int    TPIS=0;


static Timer_ID_Typedef 	gTPISTIMID;     //TPIS��ʱ��ID
static SubModuleStat        m_TPIS_Submodule_stat = SUB_MOD_STOP;   // TPIS module state
static uint8_t TPISTemCnt=0;    //TPIS�¶Ȳ������ݼ�������5��ȥ��ǰ2��ȡ���ֵ

#define TObj_Offs    64473    //��Ҫ���߾���Ĵ��������е�����25��ʱ������Ŀ���¶ȼ���(���߽���DC offsetֵ)
#define TAmb_offs    7801     //��Ҫ���ݾ���Ĵ��������е�����25��ʱ�����Ļ����¶ȼ���
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
* Description    : ����TPIS״̬
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
* Description    : ��ȡTPIS״̬
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
* Description    : ��ȡTPIS1S1253����17λΪTobj����14λΪTamb����������ȫ�ֱ�����
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
* Description    : TPISSample,TPIS��ʱ���жϷ�����(�ص�����),����TPIS��ȡ�����¼�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TPISSample(Timer_ID_Typedef TIMID)
{
		TS_SendEvent(gTsTPISTaskID_c,gTPISEventSample);  //����TPIS��ȡ�����¼�
}

/*******************************************************************************
* Function Name  : TPIS_Start
* Description    : TPIS_Start,����TPIS��ʱ��������TPIS����Ƶ��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TPIS_Start(void)
{
		TIM_Cfg_Typedef         Tim_Cfg_TPIS_Index;       //TPIS timer���ã����ڿ���TPIS����Ƶ��
		TIM_Basic_Cfg_Typedef 	Tim_Cfg_TPIS;
	
		/* TPIS GPIO Init */
		TPIS_Init();
	
		/* ����TPIS timer,����TPISSampleTime���в��� */
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
* Description    : TPIS_Stop,�ͷ�TPIS��ʱ��
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
* Description    : TPIS_Task_Handler,����NCT����
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
				TPIS_Start();       //���붨ʱ�����ڶ�ʱ���ж��в�������
			}
		}
		else if(TPIS_Event & gTPISEventSample)
		{
				IntDisable();    //��ȫ���ж�
				TPIS_Val = Read_TPIS();
				IntEnable();     //��ȫ���ж�
			
				if(TPIS_Val >40)
				{
					TPIS_Val = 40;
				}
				
				TPIS_Tem[TPISTemCnt] = TPIS_Val;   //����TPIS�����������ȡ���ֵ
				TPISTemCnt++;
			
				if(TPISTemCnt >= 5)  //����5��ȡ���ֵ��ΪTPIS���ս��
				{
					TPISTemCnt = 0;
					for(i=2;i<5;i++)   //����ǰ2����ֵ���ҵ����ֵ
					{
						if(TPIS_Tem[i] > MaxTem)
						{
							MaxTem = TPIS_Tem[i];
							TPIS_Tem[i]=0;
						}
						//printf("MaxTem = %f\r\n",MaxTem);
						//printf("TPIS_Tem[%d] = %f\r\n",i,TPIS_Tem[i]);
					}
					
					/* ��������TPIS�������������ֹͣ�����¼� */
					gTPISVal.Value = MaxTem;
					MaxTem=0;
					//StartMotor(1);
					TS_SendEvent(gOledDisTaskID,gOledDisEventTPISResult_c);//������Ļ��ʾTPIS��������¼�
					TS_SendEvent(gTsTPISTaskID_c,gTPISEventStop);     //����TPISֹͣ�ɼ��¼�	
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



