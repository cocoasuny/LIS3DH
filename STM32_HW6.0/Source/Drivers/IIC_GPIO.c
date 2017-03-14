#include "IIC_GPIO.h"

/*  IIC1 GPIO 模拟 */
//初始化IIC
void IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE );	
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT ;   //推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_8|GPIO_Pin_9); 	//PB10,PB11 输出高
}
//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA_H;	  	  
	IIC_SCL_H;
	//Delay_us(4);
 	IIC_SDA_L;//START:when CLK is high,DATA change form high to low 
	//Delay_us(4);
	IIC_SCL_L;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL_L;
	IIC_SDA_L;//STOP:when CLK is high DATA change form low to high
 	//Delay_us(4);
	IIC_SCL_H; 
	IIC_SDA_H;//发送I2C总线结束信号
	//Delay_us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	uint8_t READ_SDA=0;
	IIC_SDA_H;//Delay_us(1);	   
	IIC_SCL_H;//Delay_us(1);	
	SDA_IN();      //SDA设置为输入
	READ_SDA = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9);
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_L;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL_L;
	SDA_OUT();
	IIC_SDA_L;
	//Delay_us(2);
	IIC_SCL_H;
	//Delay_us(2);
	IIC_SCL_L;
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_SCL_L;
	SDA_OUT();
	IIC_SDA_H;
	//Delay_us(2);
	IIC_SCL_H;
	//Delay_us(2);
	IIC_SCL_L;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	SDA_OUT(); 	    
    IIC_SCL_L;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
		if((txd&0x80)>>7)
		{
			IIC_SDA_H;
		}
		else
		{
			IIC_SDA_L;
		}
		txd<<=1; 	  
		//Delay_us(2);   
		IIC_SCL_H;
		//Delay_us(2); 
		IIC_SCL_L;	
		//Delay_us(2);
    }	
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	uint8_t READ_SDA;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL_L; 
        //Delay_us(2);
		IIC_SCL_H;
		IIC_SDA_L;
        receive<<=1;
		READ_SDA = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9);
        if(READ_SDA)receive++;   
		//Delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}


/*  IIC2 GPIO 模拟 */
//初始化IIC
void IIC2_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE );	
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT ;   //推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_10|GPIO_Pin_11); 	//PB10,PB11 输出高
}
//初始化IIC
void IIC2_DeInit(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE );	
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN ;   //推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
//产生IIC起始信号
void IIC2_Start(void)
{
	SDA2_OUT();     //sda线输出
	IIC2_SDA_H;	  	  
	IIC2_SCL_H;
	//Delay_us(4);
 	IIC2_SDA_L;//START:when CLK is high,DATA change form high to low 
	//Delay_us(4);
	IIC2_SCL_L;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC2_Stop(void)
{
	SDA2_OUT();//sda线输出
	IIC2_SCL_L;
	IIC2_SDA_L;//STOP:when CLK is high DATA change form low to high
 	//Delay_us(4);
	IIC2_SCL_H; 
	IIC2_SDA_H;//发送I2C总线结束信号
	//Delay_us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t IIC2_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	uint8_t READ_SDA=0;
	
	IIC2_SDA_H;//Delay_us(1);	   
	IIC2_SCL_H;//Delay_us(1);	
	SDA2_IN();      //SDA设置为输入
	IIC2_SDA_L;
	READ_SDA = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11);
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC2_Stop();
			return 1;
		}
	}
	IIC2_SCL_L;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void IIC2_Ack(void)
{
	IIC2_SCL_L;
	SDA2_OUT();
	IIC2_SDA_L;
	//Delay_us(2);
	IIC2_SCL_H;
	//Delay_us(2);
	IIC2_SCL_L;
}
//不产生ACK应答		    
void IIC2_NAck(void)
{
	IIC2_SCL_L;
	SDA2_OUT();
	IIC2_SDA_H;
	//Delay_us(2);
	IIC2_SCL_H;
	//Delay_us(2);
	IIC2_SCL_L;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC2_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	SDA2_OUT(); 	    
    IIC2_SCL_L;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {     
		if((txd&0x80)>>7)
		{
			IIC2_SDA_H;
		}
		else
		{
			IIC2_SDA_L;
		}
		txd<<=1; 
		IIC2_SCL_H;
		//Delay_us(2);	  	
		IIC2_SCL_L;	
		//Delay_us(2);
    }	
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t IIC2_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	uint8_t READ_SDA;
	SDA2_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC2_SCL_L; 
        //Delay_us(2);
		IIC2_SCL_H;
		IIC2_SDA_L;
        receive<<=1;
		READ_SDA = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11);
        if(READ_SDA)receive++;   
		//Delay_us(1); 
    }					 
    if (!ack)
        IIC2_NAck();//发送nACK
    else
        IIC2_Ack(); //发送ACK   
    return receive;
}



