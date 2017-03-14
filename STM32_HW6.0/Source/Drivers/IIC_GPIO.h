#ifndef   __IIC_H
#define   __IIC_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "Timer.h"

//IO��������
#define SDA_IN()  {GPIOB->MODER &= 0xFFF3FFFF;}
#define SDA_OUT() {GPIOB->MODER &= 0xFFF3FFFF;GPIOB->MODER |= 0x00040000;}


//IO��������	 
#define IIC_SDA_H  GPIO_SetBits(GPIOB,GPIO_Pin_9); 
#define IIC_SDA_L  GPIO_ResetBits(GPIOB,GPIO_Pin_9); 

#define IIC_SCL_H  GPIO_SetBits(GPIOB,GPIO_Pin_8); 
#define IIC_SCL_L  GPIO_ResetBits(GPIOB,GPIO_Pin_8); 


//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(uint8_t txd);			//IIC����һ���ֽ�
uint8_t IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
uint8_t IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	

/**********************************************************************/

//IO��������
#define SDA2_IN()  {GPIOB->MODER &= 0xFF3FFFFF;}
#define SDA2_OUT() {GPIOB->MODER &= 0xFF3FFFFF;GPIOB->MODER |= 0x00400000;}


//IO��������	 
#define IIC2_SDA_H  GPIO_SetBits(GPIOB,GPIO_Pin_11); 
#define IIC2_SDA_L  GPIO_ResetBits(GPIOB,GPIO_Pin_11); 

#define IIC2_SCL_H  GPIO_SetBits(GPIOB,GPIO_Pin_10); 
#define IIC2_SCL_L  GPIO_ResetBits(GPIOB,GPIO_Pin_10); 


//IIC���в�������
void IIC2_Init(void);                //��ʼ��IIC��IO��	
void IIC2_DeInit(void);              //�ָ�IIC IO��Ĭ��״̬
void IIC2_Start(void);				//����IIC��ʼ�ź�
void IIC2_Stop(void);	  			//����IICֹͣ�ź�
void IIC2_Send_Byte(uint8_t txd);			//IIC����һ���ֽ�
uint8_t IIC2_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
uint8_t IIC2_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC2_Ack(void);					//IIC����ACK�ź�
void IIC2_NAck(void);				//IIC������ACK�ź�

void IIC2_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC2_Read_One_Byte(uint8_t daddr,uint8_t addr);	

#endif    /* __IIC_H */
