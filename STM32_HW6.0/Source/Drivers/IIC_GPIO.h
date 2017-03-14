#ifndef   __IIC_H
#define   __IIC_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "Timer.h"

//IO方向设置
#define SDA_IN()  {GPIOB->MODER &= 0xFFF3FFFF;}
#define SDA_OUT() {GPIOB->MODER &= 0xFFF3FFFF;GPIOB->MODER |= 0x00040000;}


//IO操作函数	 
#define IIC_SDA_H  GPIO_SetBits(GPIOB,GPIO_Pin_9); 
#define IIC_SDA_L  GPIO_ResetBits(GPIOB,GPIO_Pin_9); 

#define IIC_SCL_H  GPIO_SetBits(GPIOB,GPIO_Pin_8); 
#define IIC_SCL_L  GPIO_ResetBits(GPIOB,GPIO_Pin_8); 


//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(uint8_t txd);			//IIC发送一个字节
uint8_t IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
uint8_t IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	

/**********************************************************************/

//IO方向设置
#define SDA2_IN()  {GPIOB->MODER &= 0xFF3FFFFF;}
#define SDA2_OUT() {GPIOB->MODER &= 0xFF3FFFFF;GPIOB->MODER |= 0x00400000;}


//IO操作函数	 
#define IIC2_SDA_H  GPIO_SetBits(GPIOB,GPIO_Pin_11); 
#define IIC2_SDA_L  GPIO_ResetBits(GPIOB,GPIO_Pin_11); 

#define IIC2_SCL_H  GPIO_SetBits(GPIOB,GPIO_Pin_10); 
#define IIC2_SCL_L  GPIO_ResetBits(GPIOB,GPIO_Pin_10); 


//IIC所有操作函数
void IIC2_Init(void);                //初始化IIC的IO口	
void IIC2_DeInit(void);              //恢复IIC IO口默认状态
void IIC2_Start(void);				//发送IIC开始信号
void IIC2_Stop(void);	  			//发送IIC停止信号
void IIC2_Send_Byte(uint8_t txd);			//IIC发送一个字节
uint8_t IIC2_Read_Byte(unsigned char ack);//IIC读取一个字节
uint8_t IIC2_Wait_Ack(void); 				//IIC等待ACK信号
void IIC2_Ack(void);					//IIC发送ACK信号
void IIC2_NAck(void);				//IIC不发送ACK信号

void IIC2_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC2_Read_One_Byte(uint8_t daddr,uint8_t addr);	

#endif    /* __IIC_H */
