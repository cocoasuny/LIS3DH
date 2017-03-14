#include "IIC_GPIO.h"

/*  IIC1 GPIO ģ�� */
//��ʼ��IIC
void IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE );	
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT ;   //�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_8|GPIO_Pin_9); 	//PB10,PB11 �����
}
//����IIC��ʼ�ź�
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA_H;	  	  
	IIC_SCL_H;
	//Delay_us(4);
 	IIC_SDA_L;//START:when CLK is high,DATA change form high to low 
	//Delay_us(4);
	IIC_SCL_L;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL_L;
	IIC_SDA_L;//STOP:when CLK is high DATA change form low to high
 	//Delay_us(4);
	IIC_SCL_H; 
	IIC_SDA_H;//����I2C���߽����ź�
	//Delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	uint8_t READ_SDA=0;
	IIC_SDA_H;//Delay_us(1);	   
	IIC_SCL_H;//Delay_us(1);	
	SDA_IN();      //SDA����Ϊ����
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
	IIC_SCL_L;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
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
//������ACKӦ��		    
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
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	SDA_OUT(); 	    
    IIC_SCL_L;//����ʱ�ӿ�ʼ���ݴ���
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
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	uint8_t READ_SDA;
	SDA_IN();//SDA����Ϊ����
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
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}


/*  IIC2 GPIO ģ�� */
//��ʼ��IIC
void IIC2_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE );	
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT ;   //�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_10|GPIO_Pin_11); 	//PB10,PB11 �����
}
//��ʼ��IIC
void IIC2_DeInit(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE );	
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN ;   //�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
//����IIC��ʼ�ź�
void IIC2_Start(void)
{
	SDA2_OUT();     //sda�����
	IIC2_SDA_H;	  	  
	IIC2_SCL_H;
	//Delay_us(4);
 	IIC2_SDA_L;//START:when CLK is high,DATA change form high to low 
	//Delay_us(4);
	IIC2_SCL_L;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC2_Stop(void)
{
	SDA2_OUT();//sda�����
	IIC2_SCL_L;
	IIC2_SDA_L;//STOP:when CLK is high DATA change form low to high
 	//Delay_us(4);
	IIC2_SCL_H; 
	IIC2_SDA_H;//����I2C���߽����ź�
	//Delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
uint8_t IIC2_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	uint8_t READ_SDA=0;
	
	IIC2_SDA_H;//Delay_us(1);	   
	IIC2_SCL_H;//Delay_us(1);	
	SDA2_IN();      //SDA����Ϊ����
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
	IIC2_SCL_L;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
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
//������ACKӦ��		    
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
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC2_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	SDA2_OUT(); 	    
    IIC2_SCL_L;//����ʱ�ӿ�ʼ���ݴ���
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
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
uint8_t IIC2_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	uint8_t READ_SDA;
	SDA2_IN();//SDA����Ϊ����
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
        IIC2_NAck();//����nACK
    else
        IIC2_Ack(); //����ACK   
    return receive;
}



