/******************** (C) COPYRIGHT 2014-2024,iCareTech.Co.,Ltd. ********************
 * File Name��ADS1115.c
 * Author: Jason
 * Version: 1.0
 * Date: 2014/3/4
 * Desciption: ������ϸ˵���˳����ļ���ɵ���Ҫ���ܣ�������ģ������Ľӿڣ���ʹ�÷���
 *             ���磺ADS1115.c��Ҫʵ��ADC ADS1115������������ʵ���˶���ADS1115�Ĵ����Ķ�д��
 *             ADS1115����ģʽ�����á�ѡ��ADS1115ת��ͨ��������ADS1115ת���� ��ȡת������ȡ�
 *Function List:
 *             1��void ADS1115_GPIO_Config(void):����ADS1115 GPIO
 *             2��void ADS1115_IIC_Config(void): ����MCU��ADS1115 IIC�ӿ�
 *             3��Function 3: *************
 *             4��Function 4: *************
 *History:
 *       <Author>      <Time>       <Version>     <Desciption>
 *        Jason        2014/3/4         1.0           Creat
 *        David        2014/3/5         1.1         �޸���XXX��������Ҫ����XXXX��ԭ���޸ĺ�XXXX
**********************************************************************************/
#include "stm32l1xx.h"
#include "ADS1115.h"
#include "Timer.h"
#include "Usart.h"
#include "platform.h"
#include "common.h"

/* CPAL local transfer structures */
CPAL_TransferTypeDef  sRxStructure, sTxStructure;
enum ADS1115_StateTypeDef  ADS1115_State;


unsigned char BufferSize=1;               //CPAL��ʼ��IIC DMA�������ݴ�С����ʼΪ1
unsigned char tRxBuffer[2];         //CPAL��ʼ��IIC DMA�������ݴ����ռ䣬��ʼ�ռ��СΪ2
unsigned char tStateSignal[]={0x01};//CPAL��ʼ��IIC DMA�������ݴ����ռ䣬��ʼ�ռ��СΪ1������ֵ0x01
unsigned char OWN_ADDRESS=0x90;           //CPAL��ʼ��IIC Slave�豸IIC��ַ����ʼΪ0x90
unsigned char W_ADS1115_CRDataSize_One_Buf[W_ADS1115_CRDataSize_One]={0x00};//ADS1115Config Registerдһ���ֽ�Buffer
unsigned char W_ADS1115_CRDataSize_Thr_Buf[W_ADS1115_CRDataSize_Thr]={0x00,0x00,0x00};//ADS1115Config Registerд�����ֽ�Buffer
extern unsigned char ADS1115RxBuffer[2];   //ADS1115��ȡת�����ݴ����ռ�

uint8_t ADS1115_IIC_RXTC_Flag =0;

/*******************************************************************************
* Function Name  : ADS1115_GPIO_Init
* Description    : Configures ADS1115,ALERT/RDY pin,����ģʽ
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADS1115_GPIO_Init(void)
{
    /* GPIOLED Periph clock enable */
    GPIO_InitTypeDef        GPIO_InitStructure;
	EXTI_InitTypeDef        EXTI_InitStructure;
    RCC_AHBPeriphClockCmd(ADS1115_ALERTRDY_GPIO_CLK, ENABLE);

    /* Configure ALERT/RDY pin as input  */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = ADS1115_ALERTRDY_PIN;
    GPIO_Init(ADS1115_ALERTRDY_GPIO_PORT, &GPIO_InitStructure);
	
	Periperal_Allocate(Periph_ADS1115_RDY);  //����ADS1115 RDY Periph
	#ifdef LowPower_DEBUG
	printf("ADS1115 RDY SPI Periperal_Allocate = %x\r\n",Periph_ADS1115_RDY);
	#endif
	
	/* Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(ADS1115_RCC_APBPeriph_SYSCFG, ENABLE);
	
	SYSCFG_EXTILineConfig(ADS1115_EXTI_PortSourceGPIO, ADS1115_EXTI_PinSource); //ADS1115 Int
	
	EXTI_InitStructure.EXTI_Line = ADS1115_EXTI_Line;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);	
}
/*******************************************************************************
* Function Name  : ADS1115_GPIO_DeInit
* Description    : Configures ADS1115,ALERT/RDY pin,����ģʽ
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADS1115_GPIO_DeInit(void)
{
    /* GPIO ADS1115 RDY Periph clock enable */
//    GPIO_InitTypeDef        GPIO_InitStructure;
	EXTI_InitTypeDef        EXTI_InitStructure;

    /* Configure ALERT/RDY pin as input  */
	//Bug ��ʱ����
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
//    GPIO_InitStructure.GPIO_Pin = ADS1115_ALERTRDY_PIN;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
//    GPIO_Init(ADS1115_ALERTRDY_GPIO_PORT, &GPIO_InitStructure);

	EXTI_InitStructure.EXTI_Line = ADS1115_EXTI_Line;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);	
	
	Free_Periperal(Periph_ADS1115_RDY);  //�ͷ�ADS1115 RDY Periph
	
	#ifdef LowPower_DEBUG
	printf("ADS1115 RDY Periperal_Free = %x\r\n",Periph_ADS1115_RDY);
	#endif
}
/*******************************************************************************
* Function Name  : ADS1115_IIC_Init
* Description    : Configures ADS1115_IIC Ӳ��IIC�ӿڣ�����CPAL,����CPAL�����ã�
*                  �μ�cpal_conf.h�ļ�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADS1115_IIC_Init(void)
{
#ifdef ADS1115_IIC_HW
	TIM_Cfg_Typedef         Tim_Cfg_IIC_Index;       //IIC timer����,����1ms�жϣ��ṩTimeout
	TIM_Basic_Cfg_Typedef 	Tim_Cfg_IIC;
	
	/* ����IIC Timeout ��ʱ�� */
	Tim_Cfg_IIC.enuTimerType = TIM_TYPE_MS;
	Tim_Cfg_IIC.u16TimePeriod = 1;
	Tim_Cfg_IIC.NVIC_IRQChannelPreemptionPriority = IIC_TIMPreemptionPriority;
	Tim_Cfg_IIC.pIntCallBack = CPAL_I2C_TIMEOUT_Manager;

		/* Init timer top define */
		Tim_Cfg_IIC_Index.TimerMode 				= TIM_MODE_BASIC;
		Tim_Cfg_IIC_Index.TimerBasicCfg 		= &Tim_Cfg_IIC;
		Tim_Cfg_IIC_Index.TimerPWMCfg 			= NULL;
	
	
	gIICTIMID = Timer_Allocate(&Tim_Cfg_IIC_Index);
	
	
	
    /***********Ӳ��IIC2����*****************************************************/
    /* Start CPAL communication configuration ***********************************/
    /* Initialize local Reception structures */
    sRxStructure.wNumData = BufferSize;       /* Maximum Number of data to be received */
    sRxStructure.pbBuffer = tRxBuffer;        /* Common Rx buffer for all received data */
    sRxStructure.wAddr1 = 0;                  /* Not needed */
    sRxStructure.wAddr2 = 0;                  /* Not needed */

    /* Initialize local Transmission structures */
    sTxStructure.wNumData = BufferSize;       /* Maximum Number of data to be received */
    sTxStructure.pbBuffer = (uint8_t*)tStateSignal;     /* Common Rx buffer for all received data */
    sTxStructure.wAddr1 = OWN_ADDRESS;        /* The own board address */
    sTxStructure.wAddr2 = 0;                  /* Not needed */

    /* Configure the device structure */
    CPAL_I2C_StructInit(&I2C_DevStructure);      /* Set all fields to default values */
    I2C_DevStructure.CPAL_Mode = CPAL_MODE_MASTER;
    I2C_DevStructure.wCPAL_Options =  CPAL_OPT_NO_MEM_ADDR;
    I2C_DevStructure.CPAL_ProgModel = CPAL_PROGMODEL_DMA;
    I2C_DevStructure.CPAL_Direction =CPAL_DIRECTION_TXRX;

    I2C_DevStructure.pCPAL_I2C_Struct->I2C_ClockSpeed = I2C_SPEED;
    I2C_DevStructure.pCPAL_I2C_Struct->I2C_OwnAddress1 = OWN_ADDRESS;
    I2C_DevStructure.pCPAL_TransferRx = &sRxStructure;
    I2C_DevStructure.pCPAL_TransferTx = &sTxStructure;

    /* Configure the device mode to master */
    I2C_DevStructure.CPAL_Mode = CPAL_MODE_MASTER;
    /* Force the CPAL state to ready (in case a read operation has been initiated) */
    I2C_DevStructure.CPAL_State = CPAL_STATE_READY;

    /* Initialize CPAL device with the selected parameters */
    CPAL_I2C_Init(&I2C_DevStructure);
	
	Periperal_Allocate(Periph_IIC2);      //����IIC Periph
	
	#ifdef LowPower_DEBUG
	printf("ADS1115 IIC SPI Periperal_Allocate = %x\r\n",Periph_IIC2);
	#endif
#endif

#ifdef  ADS1115_IIC_SW
	IIC2_Init();
	
	/* �͹��Ĵ��� */
//	Periperal_Allocate(Periph_IIC2);      //����IIC Periph
//	
//	#ifdef LowPower_DEBUG
//	printf("ADS1115 IIC SPI Periperal_Allocate = %x\r\n",Periph_IIC2);
//	#endif
	
#endif
}
/*******************************************************************************
* Function Name  : ADS1115_DeIIC_Init
* Description    : Configures ADS1115_IIC Ӳ��IIC�ӿڣ�����CPAL,����CPAL�����ã�
*                  �μ�cpal_conf.h�ļ�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADS1115_IIC_DeInit(void)
{
#ifdef ADS1115_IIC_HW
	/* DeInitialize CPAL device with the selected parameters */
    CPAL_I2C_DeInit(&I2C_DevStructure);
	Timer_Free(gIICTIMID);
	gIICTIMID = TIMER_ERROR;
	Free_Periperal(Periph_IIC2);      //����IIC Periph
	
	#ifdef LowPower_DEBUG
	printf("ADS1115 IIC Periperal_Free = %x\r\n",Periph_IIC2);
	#endif
#endif 
	
#ifdef ADS1115_IIC_SW
	IIC2_DeInit();
	
//	Free_Periperal(Periph_IIC2);      //����IIC Periph
//	
//	#ifdef LowPower_DEBUG
//	printf("ADS1115 IIC Periperal_Free = %x\r\n",Periph_IIC2);
//	#endif
	
#endif 
}
/*******************************************************************************
* Function Name  : ADS1115_Defalut_Config
* Description    : ADS1115_Defalut_Config,ADS1115��ʼ�������ã���ADS1115.h�ļ���
*                  ͨ���궨������ADS1115Config Register��������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADS1115_Default_Config(void)
{
    unsigned char CR_MSB = 0xC4;               //Config Register��8λ
    unsigned char CR_LSB = 0x83;               //Config Register��8λ
    unsigned short CR_CMD = 0x0000;            //Config Register������
	
	#ifdef  ADS1115_IIC_SW
	uint8_t State=0;
	#endif

    CR_CMD = 0x8000|DifferentialCH2_3|FS_05|SigleShotMode|DR64SPS|Tr_COMP|COMP_POL_L|COMP_LAT_OFF|COMP_Disable;
    CR_LSB = (unsigned char)(0x00FF & CR_CMD);
    CR_MSB = (unsigned char)(0x00FF & (CR_CMD >> 8));

    ADS1115_State = ADS1115_State_ConfigControlRg;
	
#ifdef ADS1115_IIC_HW

    /*******дADS1115 Control Register ********************************************/
    W_ADS1115_CRDataSize_Thr_Buf[0] = 0x01;    //0x01:ָ��Config Register
    W_ADS1115_CRDataSize_Thr_Buf[1] = CR_MSB;  //CR_MSB:���ƼĴ�����8λ
    W_ADS1115_CRDataSize_Thr_Buf[2] = CR_LSB;  //CR_LSB:���ƼĴ�����8λ

    /* Initialize local Transmission structures */
    sTxStructure.wNumData = W_ADS1115_CRDataSize_Thr;  /* Maximum Number of data to be received */
    sTxStructure.wAddr1 = W_ADS1115_ADDRESS;           /* The ADS1115 write address */
    sTxStructure.wAddr2 = 0;                           /* Not needed */
    sTxStructure.pbBuffer = (uint8_t*)W_ADS1115_CRDataSize_Thr_Buf;

    /* Force the CPAL state to ready (in case a read operation has been initiated) */
    I2C_DevStructure.CPAL_State = CPAL_STATE_READY;

    ADS1115_State = ADS1115_State_ConfigLo;
    /* Start writing data in master mode */
    if (CPAL_I2C_Write(&I2C_DevStructure) == CPAL_PASS)
    {
    }
    //Delay_ms(500);
    //Delay_ms(500);
#endif
	
#ifdef  ADS1115_IIC_SW

	IIC2_Start();
	IIC2_Send_Byte(W_ADS1115_ADDRESS);  //����дIIC������ַ
	State = IIC2_Wait_Ack();
	if(State == 1)   //ʧ��
	{
		//
	}
	IIC2_Send_Byte(0x01);   //0x01:ָ��Config Register
	State = IIC2_Wait_Ack();
	if(State == 1)   //ʧ��
	{
		//
	}
	IIC2_Send_Byte(CR_MSB);  //CR_MSB:���ƼĴ�����8λ
	State = IIC2_Wait_Ack();
	if(State == 1)   //ʧ��
	{
		//
	}
	IIC2_Send_Byte(CR_LSB);  //CR_LSB:���ƼĴ�����8λ
	State = IIC2_Wait_Ack();
	if(State == 1)   //ʧ��
	{
		//
	}
	IIC2_Stop();//����һ��ֹͣ���� 
	
	ADS1115_State = ADS1115_State_ConfigLo;
	ADS1115_Config_Lo();    //дLo�Ĵ���
	ADS1115_State = ADS1115_State_ConfigHi;
	ADS1115_Config_Hi();    //дHi�Ĵ���
    ADS1115_State = ADS1115_State_ConversionRdy;	
#endif
}
/*******************************************************************************
* Function Name   : ADS1115_Config_Lo
* Description    : ADS1115_Config_Lo, ADS1115Ĭ�����ú�����ADS1115 Lo�Ĵ���
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADS1115_Config_Lo(void)
{
    unsigned char LO_THRESH_MSB = 0x80;        //Lo_Thresh Regiser��8λ
    unsigned char LO_THRESH_LSB = 0x00;        //Lo_Thresh Regiser��8λ
	
	#ifdef  ADS1115_IIC_SW
	uint8_t State=0;
	#endif
	
    LO_THRESH_MSB = 0x00; //ADS1115 ALERT/RDYָʾת�����
    LO_THRESH_LSB = 0x00;

#ifdef  ADS1115_IIC_HW
    /*******дADS1115 Lo_thresh Register ********************************************/
    W_ADS1115_CRDataSize_Thr_Buf[0] = 0x02;    //0x02:ָ��Lo_thresh Register
    W_ADS1115_CRDataSize_Thr_Buf[1] = LO_THRESH_MSB;  //LO_THRESH_MSB:LO_THRESH�Ĵ�����8λ
    W_ADS1115_CRDataSize_Thr_Buf[2] = LO_THRESH_LSB;  //LO_THRESH_LSB:LO_THRESH�Ĵ�����8λ

    /* Initialize local Transmission structures */
    sTxStructure.wNumData = W_ADS1115_CRDataSize_Thr;  /* Maximum Number of data to be received */
    sTxStructure.wAddr1 = W_ADS1115_ADDRESS;           /* The ADS1115 write address */
    sTxStructure.wAddr2 = 0;                           /* Not needed */
    sTxStructure.pbBuffer = (uint8_t*)W_ADS1115_CRDataSize_Thr_Buf;

    /* Force the CPAL state to ready (in case a read operation has been initiated) */
    I2C_DevStructure.CPAL_State = CPAL_STATE_READY;

	if(ADS1115_State == ADS1115_State_ConfigLo)
	{
		//printf("ADS1115_Config_Lo\r\n");
		/* Start writing data in master mode */
		if (CPAL_I2C_Write(&I2C_DevStructure) == CPAL_PASS)
		{
		}
	}
#endif
	
#ifdef	ADS1115_IIC_SW
	if(ADS1115_State == ADS1115_State_ConfigLo)
	{
		IIC2_Start();
		IIC2_Send_Byte(W_ADS1115_ADDRESS);  //����дIIC������ַ
		State = IIC2_Wait_Ack();
		if(State == 1)   //ʧ��
		{
			//
		}
		IIC2_Send_Byte(0x02);  //0x02:ָ��Lo_thresh Register
		State = IIC2_Wait_Ack();
		if(State == 1)   //ʧ��
		{
			//
		}
		IIC2_Send_Byte(LO_THRESH_MSB);  //LO_THRESH_MSB:LO_THRESH�Ĵ�����8λ
		State = IIC2_Wait_Ack();
		if(State == 1)   //ʧ��
		{
			//
		}
		IIC2_Send_Byte(LO_THRESH_LSB);  //LO_THRESH_LSB:LO_THRESH�Ĵ�����8λ
		State = IIC2_Wait_Ack();
		if(State == 1)   //ʧ��
		{
			//
		}
		IIC2_Stop();//����һ��ֹͣ���� 
	}
	
#endif
}
/*******************************************************************************
* Function Name   : ADS1115_Config_Hi
* Description    : ADS1115_Config_Hi, ADS1115����Lo�Ĵ���������Hi�Ĵ���
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADS1115_Config_Hi(void)
{
    unsigned char HI_THRESH_MSB = 0x00;        //Hi_Thresh Regiser��8λ
    unsigned char HI_THRESH_LSB = 0x00;        //Hi_Thresh Regiser��8λ
	
	#ifdef  ADS1115_IIC_SW
	uint8_t State=0;
	#endif
	
    HI_THRESH_MSB = 0xFF;
    HI_THRESH_LSB = 0xFF;
	
	
#ifdef  ADS1115_IIC_HW
    /*******дADS1115 Hi_thresh Register ********************************************/
    W_ADS1115_CRDataSize_Thr_Buf[0] = 0x03;    //0x03:ָ��Hi_thresh Register
    W_ADS1115_CRDataSize_Thr_Buf[1] = HI_THRESH_MSB;  //HI_THRESH_MSB:HI_THRESH�Ĵ�����8λ
    W_ADS1115_CRDataSize_Thr_Buf[2] = HI_THRESH_LSB;  //HI_THRESH_LSB:HI_THRESH�Ĵ�����8λ

    /* Initialize local Transmission structures */
    sTxStructure.wNumData = W_ADS1115_CRDataSize_Thr;  /* Maximum Number of data to be received */
    sTxStructure.wAddr1 = W_ADS1115_ADDRESS;           /* The ADS1115 write address */
    sTxStructure.wAddr2 = 0;                           /* Not needed */
    sTxStructure.pbBuffer = (uint8_t*)W_ADS1115_CRDataSize_Thr_Buf;

    /* Force the CPAL state to ready (in case a read operation has been initiated) */
    I2C_DevStructure.CPAL_State = CPAL_STATE_READY;
	
	if(ADS1115_State == ADS1115_State_ConfigHi)
	{
		/* Start writing data in master mode */
		if (CPAL_I2C_Write(&I2C_DevStructure) == CPAL_PASS)
		{
		}
	}

    //Delay_ms(500);
    //Delay_ms(500);
#endif
	
#ifdef  ADS1115_IIC_SW
	
	if(ADS1115_State == ADS1115_State_ConfigHi)
	{
		IIC2_Start();
		IIC2_Send_Byte(W_ADS1115_ADDRESS);  //����дIIC������ַ
		State = IIC2_Wait_Ack();
		if(State == 1)   //ʧ��
		{
			//
		}
		IIC2_Send_Byte(0x03);  //0x03:ָ��Hi_thresh Register
		State = IIC2_Wait_Ack();
		if(State == 1)   //ʧ��
		{
			//
		}
		IIC2_Send_Byte(HI_THRESH_MSB);  //HI_THRESH_MSB:HI_THRESH�Ĵ�����8λ
		State = IIC2_Wait_Ack();
		if(State == 1)   //ʧ��
		{
			//
		}
		IIC2_Send_Byte(HI_THRESH_LSB);  //HI_THRESH_LSB:HI_THRESH�Ĵ�����8λ
		State = IIC2_Wait_Ack();
		if(State == 1)   //ʧ��
		{
			//
		}
		IIC2_Stop();//����һ��ֹͣ���� 
	}
#endif
}

/*******************************************************************************
* Function Name  : ADS1115_Init
* Description    : Configures ADS1115,ALERT/RDY pin�Լ�Ӳ��IIC�ӿ�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADS1115_Init(void)
{
    ADS1115_GPIO_Init();     //����ADS1115ALERT/RDY pin
    ADS1115_IIC_Init();      //����ADS1115Ӳ��IIC�ӿ�
    ADS1115_Default_Config();  //ADS1115��ʼ������
}
/*******************************************************************************
* Function Name  : ADS1115_DeInit
* Description    : Configures ADS1115,ALERT/RDY pin�Լ�Ӳ��IIC�ӿ�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADS1115_DeInit(void)
{
    ADS1115_GPIO_DeInit();     //����ADS1115ALERT/RDY pin
    ADS1115_IIC_DeInit();      //����ADS1115Ӳ��IIC�ӿ�
	
   // ADS1115_Default_Config();  //ADS1115��ʼ������
}
/*******************************************************************************
* Function Name  : ADS1115_GetVal
* Description    : ADS1115_GetVal������һ��ת�����Ҷ�ȡ���ݣ���ȡ������ͨ��DMA�ķ�ʽ
*                  ������ȫ�ֱ���ADS1115_CHX_Val�У�����ֵ�Ļ�ȡ�ڻص�����CPAL_I2C_RXTC_UserCallback
*                  �л�ȡ��
* Input          : ADS1115Start | SingleEndCHX,ADS1115Start��ADS1115.h�����ã�
*                  eg��ADS1115_GetVal(ADS1115Start | SingleEndCH0);
* Output         : None
* Return         : None
*******************************************************************************/
void ADS1115_GetVal(unsigned short Setdata)
{
    unsigned char CR_MSB = 0xC4;               //Config Register��8λ
    unsigned char CR_LSB = 0x83;               //Config Register��8λ
	
	#ifdef  ADS1115_IIC_SW
	uint8_t State=0;
	#endif
	
    CR_LSB = (unsigned char)(0x00FF & Setdata);
    CR_MSB = (unsigned char)(0x00FF & (Setdata >> 8));

#ifdef ADS1115_IIC_HW
    /*******дADS1115 Control Register ********************************************/
    W_ADS1115_CRDataSize_Thr_Buf[0] = 0x01;    //0x01:ָ��Config Register
    W_ADS1115_CRDataSize_Thr_Buf[1] = CR_MSB;  //CR_MSB:���ƼĴ�����8λ
    W_ADS1115_CRDataSize_Thr_Buf[2] = CR_LSB;  //CR_LSB:���ƼĴ�����8λ

    /* Initialize local Transmission structures */
    sTxStructure.wNumData = W_ADS1115_CRDataSize_Thr;  /* Maximum Number of data to be received */
    sTxStructure.wAddr1 = W_ADS1115_ADDRESS;           /* The ADS1115 write address */
    sTxStructure.wAddr2 = 0;                           /* Not needed */
    sTxStructure.pbBuffer = (uint8_t*)W_ADS1115_CRDataSize_Thr_Buf;

    /* Force the CPAL state to ready (in case a read operation has been initiated) */
    I2C_DevStructure.CPAL_State = CPAL_STATE_READY;

    if(ADS1115_State == ADS1115_State_ConversionRdy)
    {
        ADS1115_State = ADS1115_State_PointConversionRg;
        /* Start writing data in master mode */
        if (CPAL_I2C_Write(&I2C_DevStructure) == CPAL_PASS)
        {
        }
    }
    //Delay_ms(500);
    //Delay_ms(500);
#endif
	
#ifdef  ADS1115_IIC_SW
	if(ADS1115_State == ADS1115_State_ConversionRdy)
	{
		IIC2_Start();
		IIC2_Send_Byte(W_ADS1115_ADDRESS);  //����дIIC������ַ
		State = IIC2_Wait_Ack();
		if(State == 1)   //ʧ��
		{
			//
		}
		IIC2_Send_Byte(0x01);  //0x01:ָ��Config Register
		State = IIC2_Wait_Ack();
		if(State == 1)   //ʧ��
		{
			//
		}
		IIC2_Send_Byte(CR_MSB);  //CR_MSB:���ƼĴ�����8λ
		State = IIC2_Wait_Ack();
		if(State == 1)   //ʧ��
		{
			//
		}
		IIC2_Send_Byte(CR_LSB);  //CR_LSB:���ƼĴ�����8λ
		State = IIC2_Wait_Ack();
		if(State == 1)   //ʧ��
		{
			//
		}
		IIC2_Stop();//����һ��ֹͣ���� 
	}
	ADS1115_State = ADS1115_State_PointConversionRg;
	ADS1115_PointConversionRg();    //����ת����ָ���ȡת������Ĵ���
	ADS1115_State = ADS1115_State_ReadRdy;
#endif
}
/*******************************************************************************
* Function Name  : ADS1115_PointConversionRg
* Description    : ָ��ADS1115Conversion Register��������ADS1115ת��֮�����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADS1115_PointConversionRg(void)
{
	#ifdef  ADS1115_IIC_SW
	uint8_t State=0;
	#endif
     /********ָ��Conversion Register*****************************************/
    W_ADS1115_CRDataSize_One_Buf[0] = 0x00;    //0x00:ָ��Conversion Register

#ifdef ADS1115_IIC_HW
    /* Initialize local Transmission structures */
    sTxStructure.wNumData = W_ADS1115_CRDataSize_One;  /* Maximum Number of data to be received */
    sTxStructure.wAddr1 = W_ADS1115_ADDRESS;           /* The own board address */
    sTxStructure.wAddr2 = 0;                           /* Not needed */
    sTxStructure.pbBuffer = (uint8_t*)W_ADS1115_CRDataSize_One_Buf;
   /* Configure the device mode to master */
    //I2C_DevStructure.CPAL_Mode = CPAL_MODE_MASTER;
    /* Force the CPAL state to ready (in case a read operation has been initiated) */
    I2C_DevStructure.CPAL_State = CPAL_STATE_READY;

      /* Start writing data in master mode */
    if (CPAL_I2C_Write(&I2C_DevStructure) == CPAL_PASS)
    {
    }
    //Delay_ms(500);
    //Delay_ms(500);
#endif
	
#ifdef  ADS1115_IIC_SW
	if(ADS1115_State == ADS1115_State_PointConversionRg)
	{
		IIC2_Start();
		IIC2_Send_Byte(W_ADS1115_ADDRESS);  //����дIIC������ַ
		State = IIC2_Wait_Ack();
		if(State == 1)   //ʧ��
		{
			//
		}
		IIC2_Send_Byte(0x00);  //0x00:ָ��Conversion Register
		State = IIC2_Wait_Ack();
		if(State == 1)   //ʧ��
		{
			//
		}
		IIC2_Stop();//����һ��ֹͣ���� 
	}
#endif
}
/*******************************************************************************
* Function Name  : ADS1115_ReadVal
* Description    : ��ȡADS1115ת����������������ADS1115RxBuffer��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADS1115_ReadVal(void)
{
	#ifdef  ADS1115_IIC_SW
	uint8_t State=0;
	#endif
	
#ifdef  ADS1115_IIC_HW
	uint16_t  ReadTimeOutCnt=1000;
	
     /*******************Read Conversion Register*********************************/
    /* Initialize local Reception structures */
    sRxStructure.wNumData = 2;       /* Maximum Number of data to be received */
    sRxStructure.wAddr1 = R_ADS1115_ADDRESS;/* The own board address */
    sRxStructure.wAddr2 = 0;                /* Not needed */
    sRxStructure.pbBuffer = ADS1115RxBuffer;/* Common Rx buffer for all received data */

    /* Force the CPAL state to ready (in case a read operation has been initiated) */
    I2C_DevStructure.CPAL_State = CPAL_STATE_READY;

    if(ADS1115_State == ADS1115_State_ReadRdy)
    {
		ADS1115_IIC_RXTC_Flag=0;
        /* Start waiting for data to be received in slave mode */
        if (CPAL_I2C_Read(&I2C_DevStructure) == CPAL_PASS)
        {
			while((ReadTimeOutCnt != 0) && (ADS1115_IIC_RXTC_Flag !=1))
			{
					ReadTimeOutCnt -= 1;
			}
        }
    }
    //Delay_ms(500);
    //Delay_ms(500);
#endif
	
#ifdef ADS1115_IIC_SW
	if(ADS1115_State == ADS1115_State_ReadRdy)
    {
		IIC2_Start();
		IIC2_Send_Byte(R_ADS1115_ADDRESS);  //���Ͷ�IIC������ַ
		State = IIC2_Wait_Ack();
		if(State == 1)   //ʧ��
		{
			//
		}
		ADS1115RxBuffer[0] = IIC2_Read_Byte(1);  //�����ֽ�
		State = IIC2_Wait_Ack();
		if(State == 1)   //ʧ��
		{
			//
		}
		ADS1115RxBuffer[1] = IIC2_Read_Byte(1);  //�����ֽ�
		State = IIC2_Wait_Ack();
		if(State == 1)   //ʧ��
		{
			//
		}
		IIC2_Stop();//����һ��ֹͣ���� 	
		ADS1115_State = ADS1115_State_ConversionRdy;
	}
#endif
}



