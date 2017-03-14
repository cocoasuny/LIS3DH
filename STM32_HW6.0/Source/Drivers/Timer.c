#include "stm32l1xx.h"
#include "Timer.h"
#include "common.h"

static unsigned char  fac_us=0;//us延时倍乘数
static unsigned short fac_ms=0;//ms延时倍乘数

/*******************************************************************************
* Function Name  : Delay_Init
* Description    : Configures Delay，采用系统滴答时钟定时
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Delay_Init(void)	 
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);//选择外部时钟  HCLK/8
	fac_us=SystemCoreClock/8000000;	//为系统时钟的1/8  
	fac_ms=(unsigned short)fac_us*1000;//非ucos下,代表每个ms需要的systick时钟数   
}
/*******************************************************************************
* Function Name  : Delay_us
* Description    : us延时函数
* Input          : 延时时间（n us）
* Output         : None
* Return         : None
*******************************************************************************/
void Delay_us(unsigned int nus)
{		
	unsigned int volatile temp;	    	 
	SysTick->LOAD=nus*fac_us; //时间加载	  		 
	SysTick->VAL=0x00;        //清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;//开始倒数	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;//关闭计数器
	SysTick->VAL =0X00; //清空计数器
}
/*******************************************************************************
* Function Name  : Delay_ms
* Description    : ms延时函数
* Input          : 延时时间(n ms)注意延时nms范围，SysTick->LOAD为24位寄存器,所以,
*                  最大延时为: nms<=0xffffff*8*1000/SYSCLK，SYSCLK单位为Hz,nms单位为ms
*                  对8M条件下,nms<=16777
* Output         : None
* Return         : None
*******************************************************************************/
void Delay_ms(unsigned short nms)
{	 		  	  
	unsigned int volatile temp;		   
	SysTick->LOAD=(unsigned int)nms*fac_ms;//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00; //清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk; //开始倒数 
	temp=SysTick->CTRL;
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;//关闭计数器
	SysTick->VAL =0X00;//清空计数器	 
} 

void delay1ms(unsigned int nms)
{	 		  	  
	unsigned int  temp;		   
	SysTick->LOAD=(unsigned int)nms*fac_ms;//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00; //清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk; //开始倒数  
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;//关闭计数器
	SysTick->VAL =0X00;//清空计数器	 
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






