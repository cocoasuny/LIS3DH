#include "stm32l1xx.h"
#include "Timer.h"
#include "common.h"

static unsigned char  fac_us=0;//us��ʱ������
static unsigned short fac_ms=0;//ms��ʱ������

/*******************************************************************************
* Function Name  : Delay_Init
* Description    : Configures Delay������ϵͳ�δ�ʱ�Ӷ�ʱ
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Delay_Init(void)	 
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);//ѡ���ⲿʱ��  HCLK/8
	fac_us=SystemCoreClock/8000000;	//Ϊϵͳʱ�ӵ�1/8  
	fac_ms=(unsigned short)fac_us*1000;//��ucos��,����ÿ��ms��Ҫ��systickʱ����   
}
/*******************************************************************************
* Function Name  : Delay_us
* Description    : us��ʱ����
* Input          : ��ʱʱ�䣨n us��
* Output         : None
* Return         : None
*******************************************************************************/
void Delay_us(unsigned int nus)
{		
	unsigned int volatile temp;	    	 
	SysTick->LOAD=nus*fac_us; //ʱ�����	  		 
	SysTick->VAL=0x00;        //��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;//��ʼ����	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;//�رռ�����
	SysTick->VAL =0X00; //��ռ�����
}
/*******************************************************************************
* Function Name  : Delay_ms
* Description    : ms��ʱ����
* Input          : ��ʱʱ��(n ms)ע����ʱnms��Χ��SysTick->LOADΪ24λ�Ĵ���,����,
*                  �����ʱΪ: nms<=0xffffff*8*1000/SYSCLK��SYSCLK��λΪHz,nms��λΪms
*                  ��8M������,nms<=16777
* Output         : None
* Return         : None
*******************************************************************************/
void Delay_ms(unsigned short nms)
{	 		  	  
	unsigned int volatile temp;		   
	SysTick->LOAD=(unsigned int)nms*fac_ms;//ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00; //��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk; //��ʼ���� 
	temp=SysTick->CTRL;
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;//�رռ�����
	SysTick->VAL =0X00;//��ռ�����	 
} 

void delay1ms(unsigned int nms)
{	 		  	  
	unsigned int  temp;		   
	SysTick->LOAD=(unsigned int)nms*fac_ms;//ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00; //��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk; //��ʼ����  
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;//�رռ�����
	SysTick->VAL =0X00;//��ռ�����	 
} 
void Delay_ms_soft(uint16_t nms)
{
	uint16_t volatile u16TimeCnt = 0;
	uint16_t volatile i=0;
	
	for(i=0;i<nms;i++)
	{
		for(u16TimeCnt=0;u16TimeCnt<3500;u16TimeCnt++);
	}
}






