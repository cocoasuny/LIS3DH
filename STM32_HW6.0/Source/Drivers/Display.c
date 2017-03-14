#include "Display.h"
#include "platform.h"
#include "SecondTick.h"
#include "common.h"

/* 96*39 OLED���Դ澵��ռ��1K�ֽ�. ��8�У�ÿ��128���� */
static uint8_t s_ucGRAM[39][12];
static void OLED_Refresh_AllGram(void);
static uint8_t gFlag_StartFlashComplete = false;    //����������ʾ��ɱ�־
static OLEDDisplayStat  m_LedDisplayState=NormalDis; 
static Timer_ID_Typedef BATChargingTIMID = TIMER_ERROR;//�����涯����ʱ��
static uint8_t gHR_SpO2DisplayMeasureStatus =OFF;    //HR/SpO2��ʾ�����ж���״̬
static Timer_ID_Typedef OLEDDisNotEnoughSpaceTIMID = TIMER_ERROR;//�ռ䲻�㶨ʱ��
static Timer_ID_Typedef OLEDDisIncommingCallTIMID = TIMER_ERROR;//������ʾ���涨ʱ��
static uint8_t  gOLEDConfigStatus=DISABLE;     //OLED����״̬
static OledDisStatus        gShutDownOledTIMStatus=OLEDDisON;
static uint8_t gOLED_DriveSystemPowerStatus=OFF;
static Timer_ID_Typedef HR_SpO2PicTIMID = TIMER_ERROR;//HR/SpO2�������涯����ʱ��
static Timer_ID_Typedef        StartPicTIMID = TIMER_ERROR;  //����������ʱ��

/* 	time record for "keep stable alarm 		*/
static   int32_t       i32TimeLastSecKeepStable = (-DISPLAY_KEEPSTABLE_TIME_SLOT);

void ClearKeepStableTimer(void)
{
	i32TimeLastSecKeepStable = (-DISPLAY_KEEPSTABLE_TIME_SLOT);
}

/*******************************************************************************
* Function Name  : OLED_Configuration
* Description    : ��ʼ��OLED
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_Configuration(void)
{
	#ifdef BOARD_REDHARE_V3_0
	
		OLED_MX8475CtrlLinesConfig();
		if(OFF == Get_OLED_DriveSystemPowerStatus())
		{
			OLED_DriveSystemPower(ON);
            Delay_ms(100);
//			GPIO_SetBits(GPIO_OLED_MXS8475Rest,GPIO_Pin_OLED_MXS8475Rest);
		}
		OLED_MXS8475SPIConfig();
		MXS8475_DefConfig();
		OLEDDisplay_Stat_Set(NormalDis);
		Set_OLED_Config_Status(ENABLE);
	
	#endif

}
uint8_t Get_OLED_DriveSystemPowerStatus(void)
{
	return gOLED_DriveSystemPowerStatus;
}
/*******************************************************************************
* Function Name  : OLED_DriveSystemPower()
* Description    : ����OLED�����Դ
* Input          : Power: ON/OFF
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_DriveSystemPower(uint8_t power)
{
	GPIO_InitTypeDef        GPIO_InitStructure;

	if(power == ON)
	{
		gOLED_DriveSystemPowerStatus = ON;
		
        /* Power On */
//        RCC_AHBPeriphClockCmd(RCC_APBPeriph_OLED_MXS8475Power, ENABLE);

//        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_OLED_MXS8475Power;
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
//        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
//        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//        GPIO_Init(GPIO_OLED_MXS8475Power, &GPIO_InitStructure);
        
        /* �͵�ƽ����OLED�����Դ */
        //GPIO_SetBits(GPIO_OLED_MXS8475Power,GPIO_Pin_OLED_MXS8475Power); //����15V��Դ
	}
	else
	{
		gOLED_DriveSystemPowerStatus = OFF;
			
        RCC_AHBPeriphClockCmd(RCC_APBPeriph_OLED_MXS8475Power, ENABLE);
        
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_OLED_MXS8475Power;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIO_OLED_MXS8475Power, &GPIO_InitStructure);
        
        /* ����ΪANģʽ���ر�OLED�����Դ */

        //GPIO_ResetBits(GPIO_OLED_MXS8475Power,GPIO_Pin_OLED_MXS8475Power); //�ر�15V��Դ
	}
}
/*******************************************************************************
* Function Name  : OLED_ReConfiguration
* Description    : �ӵ͹���ģʽ�л��Ѻ��ʼ��OLED
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void OLED_ReConfiguration(void)
//{

//	#ifdef BOARD_REDHARE_V3_0
//		//OLED_MX8475CtrlLinesConfig();
//		GPIO_InitTypeDef        OLED_GPIOInitStructure;
//	
//		/* Config OLEDRest GPIO in output mode */
//		RCC_AHBPeriphClockCmd(RCC_APBPeriph_OLED_MXS8475Rest,ENABLE);

//        OLED_GPIOInitStructure.GPIO_Pin = GPIO_Pin_OLED_MXS8475Rest;
//        OLED_GPIOInitStructure.GPIO_Mode = GPIO_Mode_OUT;
//        OLED_GPIOInitStructure.GPIO_OType = GPIO_OType_PP;
//        OLED_GPIOInitStructure.GPIO_Speed = GPIO_Speed_40MHz;
//        OLED_GPIOInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//        
//        GPIO_Init(GPIO_OLED_MXS8475Rest, &OLED_GPIOInitStructure);
//		GPIO_SetBits(GPIO_OLED_MXS8475Rest,GPIO_Pin_OLED_MXS8475Rest);
//	
//		/* Config OLEDA0 in output mode */
//        RCC_AHBPeriphClockCmd(RCC_APBPeriph_OLED_MXS8475A0,ENABLE);

//        OLED_GPIOInitStructure.GPIO_Pin = GPIO_Pin_OLED_MXS8475A0;
//        OLED_GPIOInitStructure.GPIO_Mode = GPIO_Mode_OUT;
//        OLED_GPIOInitStructure.GPIO_OType = GPIO_OType_PP;
//        OLED_GPIOInitStructure.GPIO_Speed = GPIO_Speed_40MHz;
//        OLED_GPIOInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//        
//        GPIO_Init(GPIO_OLED_MXS8475A0, &OLED_GPIOInitStructure);
//	
//		OLED_MXS8475SPIConfig();
//		OLEDDisplay_Stat_Set(NormalDis);
//	#endif

//}
/*******************************************************************************
* Function Name  : OLED_DeConfiguration
* Description    : �ָ�OLEDGPIO
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_DeConfiguration(void)
{

	#ifdef BOARD_REDHARE_V3_0
		OLED_MX8475CtrlLinesDeConfig();
		OLED_MXS8475SPIDeConfig();
		Set_OLED_Config_Status(DISABLE);
		if(ON == Get_OLED_DriveSystemPowerStatus())
		{
			OLED_DriveSystemPower(OFF);
		}
	#endif

}
/*******************************************************************************
* Function Name  : Set_OLED_Dis_Status
* Description    : ����OLED��ʾ״̬
* Input          : 	OLEDDisON=0,
*					OLEDDisEnterShutDown,
*					OLEDDisShutDown,
*					OLEDDisExitShutDown,
* Output         : None
* Return         : None
*******************************************************************************/
void Set_OLED_Dis_Status(OledDisStatus status)
{
	gShutDownOledTIMStatus = status;
}
/*******************************************************************************
* Function Name  : Get_OLED_Dis_Status
* Description    : ��ȡOLED��ʾ״̬
* Input          : None
* Output         : None
* Return         : 	OLEDDisON=0,
*					OLEDDisEnterShutDown,
*					OLEDDisShutDown,
*					OLEDDisExitShutDown,
*******************************************************************************/
OledDisStatus Get_OLED_Dis_Status(void)
{
	return gShutDownOledTIMStatus;
}
/*******************************************************************************
* Function Name  : Set_OLED_Config_Status
* Description    : ����OLED��ʼ��״̬
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : None
*******************************************************************************/
void Set_OLED_Config_Status(uint8_t Status)
{
	gOLEDConfigStatus = Status;
}
/*******************************************************************************
* Function Name  : Get_OLED_Config_Status
* Description    : ��ȡOLED��ʼ��״̬
* Input          : None
* Output         : None
* Return         : ENABLE/DISABLE
*******************************************************************************/
uint8_t Get_OLED_Config_Status(void)
{
	return gOLEDConfigStatus;
}
/*******************************************************************************
* Function Name  : OLED_DisplayChar
* Description    : OLED��ʾ�ַ�
* Input          : Line,Column,Ascii
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_DisplayChar(uint8_t Line, uint8_t Column, uint8_t Ascii)
{
	#ifdef BOARD_REDHARE_V3_0
		//OLED_MXS8475_DisplayChar(Line,Column,Ascii);
	#endif
}
/*******************************************************************************
* Function Name  : OLED_DisplayICOM
* Description    : OLED��ʾͼ��
* Input          : ͼ��ID
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_DisplayICON(uint8_t ICON)
{
		switch(ICON)
		{
			case ICON_ModeHR:
//				OLED_DisplayFullScreenBMP(DIS_ICON_SpO2Mode);
				break;
			case ICON_MeasureHR:
				OLED_DisplayHRSpO2MeasureFlash(OFF); 
				if(OFF == GetHR_SpO2DisplayMeasureStatus())
				{
					OLED_DisplayHRSpO2MeasureFlash(ON);
				}
				break;
			case ICON_ModeKeepStable:
				OLED_DisplayOccupyFullScreenBMP(MXS8475_ModeKeepStable);
				break;
			case ICON_SpO2Err:
				//OLED_DisplayOccupyFullScreenBMP(DIS_ICON_SpO2Err);
				OLED_DisplayFullScreenBMP(DIS_ICON_SpO2Err);
				break;
			case ICON_AppCtl:
                #ifdef UI_CHINESE_ENABLE
                    OLED_DisplayFullScreenBMP(HR_SpO2MeasuringPic);  //���Ľ�����������/Ѫ����������һ��
                #else
                    OLED_DisplayFullScreenBMP(AppCtlShow1);
                #endif
				break;
			case ICON_ModeFullCharge:
				OLED_DisplayFullScreenBMP(ChargeFull);
				break;
			case ICON_ModeCharging:
				OLED_DisplayBATCharingFlash(ON);
				break;
			case ICON_ModeLowBattery:
				OLED_DisplayFullScreenBMP(LowBatterWarn);
				break;
			case ICON_WearBad:
				OLED_DisplayFullScreenBMP(WearBad);
				break;
			case ICON_NotEnoughSpace:
				OLED_DisplayOccupyFullScreenBMP(DIS_ICON_NotEnoughSpace);
				break;           
			default :break;
		}
}
/*******************************************************************************
* Function Name  : OLED_DisplayStr
* Description    : ��ʾ�ַ���
* Input          : Str
* Output         : None
* Return         : None
*******************************************************************************/
//void OLED_DisplayStr(uint8_t Line,uint8_t Colume,uint8_t ch[])
//{
//		uint8_t i=0;
//		uint8_t j=0;
//		uint8_t LineE=0;
//		uint8_t ColumeE=0;
//		uint8_t Temp=0;

//		//MXS8475_WriteReg(0x02,0x00);
//		MXS8475_WriteReg(0x1D,0x04);

//		while(ch[j] !='\0')
//		{
//			Temp = ch[j]-'A';
//			LineE = Line + 8;
//			ColumeE = Colume + 16;

//			MXS8475_WriteReg(0x34,Line);
//			MXS8475_WriteReg(0x35,LineE);
//			MXS8475_WriteReg(0x36,Colume);
//			MXS8475_WriteReg(0x37,ColumeE);

//			MXS8475_WriteCMD(0x08);

//			for(i =0;i<16;i++)
//			{
//				MXS8475_WriteDat(nAsciiDot8x16[Temp*16+i]);
//			}
//			Line = Line+1;
//			j++;
//		}
//		MXS8475_WriteReg(0x02,0x01);
//}
/*****************GUI ��ֲ *****************************************************/
/*
*********************************************************************************************************
*	�� �� ��: OLED_PutPixel
*	����˵��: ��1������
*	��    ��:
*			_usX,_usY : ��������
*			_ucColor  ��������ɫ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void OLED_PutPixel(uint16_t _usX, uint16_t _usY, uint8_t _ucColor)
{
	uint8_t ucValue;
	uint8_t ucPageAddr;
	uint8_t ucColAddr;
	uint8_t XE=0;
	uint8_t YE=0;
	
//	const uint8_t aOrTab[8]  = {0x01, 0x02, 0x04, 0x08,0x10,0x20,0x40,0x80};
	const uint8_t aAndTab[8] = {0xFE, 0xFD, 0xFB, 0xF7,0xEF,0xDF,0xBF,0x7F};
	const uint8_t aOrTab[8]  = {0x80, 0x40, 0x20, 0x10,0x08,0x04,0x02,0x01};

	ucPageAddr = _usX / 8;
	ucColAddr = _usY;

	ucValue = s_ucGRAM[ucPageAddr][ucColAddr];
	if (_ucColor == 0)
	{
		ucValue &= aAndTab[_usX % 8];
	}
	else
	{
		ucValue |= aOrTab[_usX % 8];
	}
	s_ucGRAM[ucPageAddr][ucColAddr] = ucValue;

	MXS8475_WriteReg(0x1D,4);

	XE = ucPageAddr + 8;
	YE = ucColAddr + 1;
	MXS8475_WriteReg(0x34,ucPageAddr);
	MXS8475_WriteReg(0x35,XE);
	MXS8475_WriteReg(0x36,ucColAddr);
	MXS8475_WriteReg(0x37,YE);


	MXS8475_WriteCMD(0x08);

	MXS8475_WriteDat(ucValue);			
	
	MXS8475_WriteReg(0x02,0x01);
}
/*
*********************************************************************************************************
*	�� �� ��: OLED_GetPixel
*	����˵��: ��ȡ1������
*	��    ��:
*			_usX,_usY : ��������
*	�� �� ֵ: ��ɫֵ (0, 1)
*********************************************************************************************************
*/
//uint8_t OLED_GetPixel(uint16_t _usX, uint16_t _usY)
//{
//	uint8_t ucValue;
//	uint8_t ucPageAddr;
//	uint8_t ucColAddr;

//	ucPageAddr = _usX / 8;
//	ucColAddr = _usY;

//	ucValue = s_ucGRAM[ucPageAddr][ucColAddr];
//	if (ucValue & (_usY % 8))
//	{
//		return 1;
//	}
//	else
//	{
//		return 0;
//	}
//}

/*
*********************************************************************************************************
*	�� �� ��: OLED_DrawLine
*	����˵��: ���� Bresenham �㷨����2��仭һ��ֱ�ߡ�
*	��    ��:
*			_usX1, _usY1 ����ʼ������
*			_usX2, _usY2 ����ֹ��Y����
*			_ucColor     ����ɫ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
//void OLED_DrawLine(uint16_t _usX1 , uint16_t _usY1 , uint16_t _usX2 , uint16_t _usY2 , uint8_t _ucColor)
//{
//	int32_t dx , dy ;
//	int32_t tx , ty ;
//	int32_t inc1 , inc2 ;
//	int32_t d , iTag ;
//	int32_t x , y ;

//	/* ���� Bresenham �㷨����2��仭һ��ֱ�� */

//	OLED_PutPixel(_usX1 , _usY1 , _ucColor);

//	/* ��������غϣ���������Ķ�����*/
//	if ( _usX1 == _usX2 && _usY1 == _usY2 )
//	{
//		return;
//	}

//	iTag = 0 ;
//	/* dx = abs ( _usX2 - _usX1 ); */
//	if (_usX2 >= _usX1)
//	{
//		dx = _usX2 - _usX1;
//	}
//	else
//	{
//		dx = _usX1 - _usX2;
//	}

//	/* dy = abs ( _usY2 - _usY1 ); */
//	if (_usY2 >= _usY1)
//	{
//		dy = _usY2 - _usY1;
//	}
//	else
//	{
//		dy = _usY1 - _usY2;
//	}

//	if ( dx < dy )   /*���dyΪ�Ƴ������򽻻��ݺ����ꡣ*/
//	{
//		uint16_t temp;

//		iTag = 1 ;
//		temp = _usX1; _usX1 = _usY1; _usY1 = temp;
//		temp = _usX2; _usX2 = _usY2; _usY2 = temp;
//		temp = dx; dx = dy; dy = temp;
//	}
//	tx = _usX2 > _usX1 ? 1 : -1 ;    /* ȷ������1���Ǽ�1 */
//	ty = _usY2 > _usY1 ? 1 : -1 ;
//	x = _usX1 ;
//	y = _usY1 ;
//	inc1 = 2 * dy ;
//	inc2 = 2 * ( dy - dx );
//	d = inc1 - dx ;
//	while ( x != _usX2 )     /* ѭ������ */
//	{
//		if ( d < 0 )
//		{
//			d += inc1 ;
//		}
//		else
//		{
//			y += ty ;
//			d += inc2 ;
//		}
//		if ( iTag )
//		{
//			OLED_PutPixel ( y , x , _ucColor) ;
//		}
//		else
//		{
//			OLED_PutPixel ( x , y , _ucColor) ;
//		}
//		x += tx ;
//	}
//}

/*
*********************************************************************************************************
*	�� �� ��: OLED_DrawPoints
*	����˵��: ���� Bresenham �㷨������һ��㣬������Щ�����������������ڲ�����ʾ��
*	��    ��:
*			x, y     ����������
*			_ucColor ����ɫ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
//void OLED_DrawPoints(uint16_t *x, uint16_t *y, uint16_t _usSize, uint8_t _ucColor)
//{
//	uint16_t i;

//	for (i = 0 ; i < _usSize - 1; i++)
//	{
//		OLED_DrawLine(x[i], y[i], x[i + 1], y[i + 1], _ucColor);
//	}
//}

/*
*********************************************************************************************************
*	�� �� ��: OLED_DrawRect
*	����˵��: ���ƾ��Ρ�
*	��    ��:
*			_usX,_usY���������Ͻǵ�����
*			_usHeight �����εĸ߶�
*			_usWidth  �����εĿ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
//void OLED_DrawRect(uint16_t _usX, uint16_t _usY, uint8_t _usHeight, uint16_t _usWidth, uint8_t _ucColor)
//{
//	/*
//	 ---------------->---
//	|(_usX��_usY)        |
//	V                    V  _usHeight
//	|                    |
//	 ---------------->---
//		  _usWidth
//	*/

//	OLED_DrawLine(_usX, _usY, _usX + _usWidth - 1, _usY, _ucColor);	/* �� */
//	OLED_DrawLine(_usX, _usY + _usHeight - 1, _usX + _usWidth - 1, _usY + _usHeight - 1, _ucColor);	/* �� */

//	OLED_DrawLine(_usX, _usY, _usX, _usY + _usHeight - 1, _ucColor);	/* �� */
//	OLED_DrawLine(_usX + _usWidth - 1, _usY, _usX + _usWidth - 1, _usY + _usHeight, _ucColor);	/* �� */
//}

/*
*********************************************************************************************************
*	�� �� ��: OLED_DrawCircle
*	����˵��: ����һ��Բ���ʿ�Ϊ1������
*	��    ��:
*			_usX,_usY  ��Բ�ĵ�����
*			_usRadius  ��Բ�İ뾶
*	�� �� ֵ: ��
*********************************************************************************************************
*/
//void OLED_DrawCircle(uint16_t _usX, uint16_t _usY, uint16_t _usRadius, uint8_t _ucColor)
//{
//	int32_t  D;			/* Decision Variable */
//	uint32_t  CurX;		/* ��ǰ X ֵ */
//	uint32_t  CurY;		/* ��ǰ Y ֵ */

//	D = 3 - (_usRadius << 1);
//	CurX = 0;
//	CurY = _usRadius;

//	while (CurX <= CurY)
//	{
//		OLED_PutPixel(_usX + CurX, _usY + CurY, _ucColor);
//		OLED_PutPixel(_usX + CurX, _usY - CurY, _ucColor);
//		OLED_PutPixel(_usX - CurX, _usY + CurY, _ucColor);
//		OLED_PutPixel(_usX - CurX, _usY - CurY, _ucColor);
//		OLED_PutPixel(_usX + CurY, _usY + CurX, _ucColor);
//		OLED_PutPixel(_usX + CurY, _usY - CurX, _ucColor);
//		OLED_PutPixel(_usX - CurY, _usY + CurX, _ucColor);
//		OLED_PutPixel(_usX - CurY, _usY - CurX, _ucColor);

//		if (D < 0)
//		{
//			D += (CurX << 2) + 6;
//		}
//		else
//		{
//			D += ((CurX - CurY) << 2) + 10;
//			CurY--;
//		}
//		CurX++;
//	}
//}

/*
*********************************************************************************************************
*	�� �� ��: OLED_DrawBMP
*	����˵��: ��LCD����ʾһ��BMPλͼ��λͼ����ɨ����򣺴����ң����ϵ���
*	��    ��:
*			_usX, _usY : ͼƬ������
*			_usHeight  ��ͼƬ�߶�
*			_usWidth   ��ͼƬ���
*			_ptr       ����ɫͼƬ����ָ�룬ÿ������ռ��1���ֽ�
*	�� �� ֵ: ��
*********************************************************************************************************
*/
//void OLED_DrawBMP(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth, const uint8_t *_ptr)
//{
//	uint16_t x, y;
////	uint16_t cnt=0;

//	for(y=0;y<39;y++)
//	{
//		for(x=0;x<96;x++)
//		{
//			OLED_PutPixel(_usX + x, _usY + y, 1);
//		}
//	}
//}

/****************OLED Ӧ�� ****************************************************/
/*******************************************************************************
* Function Name  : OledDisTask
* Description    : ������ʾ���񣬽�����ģʽ�ϵ���ʾ�����������б仯�������ڶ�Ӧ�Ĳ�
*                  �������д���
* Input          : events
* Output         : None
* Return         : None
*******************************************************************************/
void OledDisTask(event_t events)
{
		TIM_Cfg_Typedef         Tim_Cfg_ClaerOLED_Index;    //OLED��ʱ�ر� timer����
		TIM_Basic_Cfg_Typedef 	Tim_Cfg_ClaerOLED;
		static uint8_t BatteryLevel=4;
		RTC_TimeTypeDef 		curTime;
		int32_t 	            curTimeInt;
	
		if(events & gOledDisEventPassKey_c)    //��ʾPassKey
		{
			OLED_DisplayClear();
			OLED_DisplayPassKey(g_PassKey.p_contex);
            SPI_AlarmTransmit(Alarm_ID_Reuse_DisplayPINCodeSuccess);  //����PIN��ʾ�ɹ��������ʱʹ�ø澯ͨ��
			TS_SendEvent(gOledDisTaskID,gOledDisEventClearModeDis_c);   //��ʾPasskeyģʽ����ʱ����
		} 
		if(events & gOledDisEventBlueIcon_c)    //RTCģʽ����ʾ����ͼ�꼰��ص����¼�
		{
			if(Device_Mode == Device_Mode_RTC)
			{
				OLED_DisplayClear();  //�����Ļ��ʾ
				OLED_UpdateRTC();  //����RTC��ʾ

				if(Device_State == Device_WorkWithAPP)
				{
					OLED_DisplayBlueToothICON(ON);	/* 	Show BLE Icon 		*/
				}
				else if(Device_State == Device_Alone)
				{
					OLED_DisplayBlueToothICON(OFF);	/*	Disable BLE Icon 	*/
				}
				OLED_DisplayProgress(1);  /* Show Process Bar 	*/
				if(MonitorTemplate.MTSwitch == 0) //��ͣ���ģ��
				{
					OLED_DisplayMonitorTemplateICON(OFF); //��ʾ���ģ��
				}
				else
				{
					OLED_DisplayMonitorTemplateICON(ON); //��ʾ���ģ��
				}
                if(gFlagHR_SpO2Measure == HRSpO2_Measure_Normal)
                {
                    OLED_DisplayTestMode(OFF);
                }
                else
                {
                    OLED_DisplayTestMode(ON);
                }
				BatteryLevel = GetBatScaleLevel();   //��ȡ��ص���
				OLED_DisplayBatteryLevelICON(BatteryLevel); //��ʾ��ص���
				if((MonitorTemplate.MTID != kICTMonitorTemplateFreeRunID) || (true != isFreeRunKickOff()))
				{
					Calendar_RTC_Period_Wakeup_Init(16384,RTC_Counter_Update);  //����RTC�����ж�
				}
				TS_SendEvent(gOledDisTaskID,gOledDisEventClearModeDis_c);   //��ʾRTC���棬��ʱ����
			}
		}
		if(events & gOledDisEventRTCUpDate_c)  //����RTC��ʾʱ��
		{
			OLED_UpdateRTC();  //����RTC��ʾ
		}
		if(events & gOledDisEventSecUpDate_c) //����Sec��ʾ
		{
			OLED_UpdateSec();  //����Sec��ʾ
		}
		if(events & gOledDisEventMeaHR_c)       //��ʾSpO2\HR����������ͼ���¼�
		{
			//if(gSubFunc_Stat_Get(SPO2_HR_MONITOR_STATE) == OFF && gSubFunc_Stat_Get(SPO2_HR_REALTIME_STATE) == OFF)
			{
				if(gShutDownOledTIMID != TIMER_ERROR)   //�ѷ���Timer
				{				
					Timer_Free(gShutDownOledTIMID);    //�������������ʱ��
					gShutDownOledTIMID = TIMER_ERROR;
				}
				OLED_DisplayClear();
				OLED_DisplayICON(ICON_MeasureHR);
			}
		}
		
		if(events & gOledDisEventModeKeepStable_c)     //Show keep stable notice when in SpO2 measurement
		{
			if((cc_alg_SpO2_Stat_Get() == SPO2_HR_STATUS_RUNNING) 
				&& ((Device_Mode == Device_Mode_CheckUp) || (Device_Mode == Device_Mode_SPO2_HR)))  		/* 	Notice "keep stable" Only when in SpO2 running mode 	*/
			{
				OLEDDisplay_Stat_Set(OccupiedDisMove);   /* 	Show "keep stable" on OLED 		*/
				Set_OLED_Dis_Status(OLEDDisON);
				OLED_DisplayICON(ICON_ModeKeepStable);
			}
			RTC_GetTime(RTC_Format_BIN, &curTime);
			curTimeInt = (curTime.RTC_Hours * 3600) + (curTime.RTC_Minutes * 60) + curTime.RTC_Seconds;
			
			/* 	Vibrate to notify user if configured	*/
			if(
				curTimeInt - i32TimeLastSecKeepStable > DISPLAY_KEEPSTABLE_TIME_SLOT
				&& (AlarmStatusGet() == false)
			)
			{
				/* 	Set alarm status 		*/
				AlarmStatusSet(true);
				
				/* 	kick off motor for notify 		*/
				StartMotor(1);		
				
				/* 	store current timer point 	*/
				i32TimeLastSecKeepStable = curTimeInt;
				
				/*	Clear Motion Display for next motion notice 		*/
				isMotionNotice_Stat_Set(FALSE);
				
			}


			if(gSubFunc_Stat_Get(SPO2_HR_SINGLEWORK_STATE) != OFF)  //���豸����ģʽ�²���ʱ�ر���Ļ��ʾ
			{
				if(gShutDownOledTIMID != TIMER_ERROR)   		/* In case, timer has already allocated 	*/
				{				
					Timer_Free(gShutDownOledTIMID);   			/* Clear this timer and allocate a new one 	*/
					gShutDownOledTIMID = TIMER_ERROR;
				}
			}
			else
			{
				if(cc_alg_SpO2_Stat_Get() == SPO2_HR_STATUS_RUNNING)
				{
					TS_SendEvent(gOledDisTaskID,gOledDisEventClearKeepMoveDis_c);
				}
			}
		}
		
		if(events & gOledDisEventNotEnoughSpace_c)    //Not enough space for data storage
		{
			OLEDDisplay_Stat_Set(OccupiedDis);
			OLED_DisplayNoEnoughSpaceFlash(ON);
			TS_SendEvent(gOledDisTaskID,gOledDisEventClearModeDis_c);   //��ʾ�ռ��������澯���棬��ʱ����
		}
		
		if(events & gOledDisEventSpO2Err_c)     //error result
		{
			OLED_DisplayICON(ICON_SpO2Err);
			OLED_DisplayCtl(ON);
			if(ON == GetHR_SpO2DisplayMeasureStatus())
			{
				OLED_DisplayHRSpO2MeasureFlash(OFF);
			}
			TS_SendEvent(gOledDisTaskID,gOledDisEventClearModeDis_c);  //��ʾ���־�Ϣ���棬��ʱ����
		}	
		
		
		if(events & gOledDisEventAppCheckUp_c)     //��ʾApp���Ƽ�ʱ�ɼ��н���
		{
			OLED_DisplayClear();
			Set_OLED_Dis_Status(OLEDDisON);
			OLED_DisplayICON(ICON_AppCtl);
			TS_SendEvent(gOledDisTaskID,gOledDisEventClearModeDis_c);  //��ʾApp���Ƽ�ʱ�ɼ����棬��ʱ����
		}
		
		if(events & gOledDisEventModeHR_c)     //��ʾSpO2\HR����ģʽͼ���¼�
		{
			OLED_DisplayClear();
			OLED_DisplayICON(ICON_ModeHR);
			TS_SendEvent(gOledDisTaskID,gOledDisEventClearModeDis_c);  //��ʾSpO2\HR����ģʽͼ����棬��ʱ����
		}
		if(events & gOledDisEventDisplayWearBad_c)  //��ʾ�������ȷ����
		{
			OLED_DisplayClear();
			OLED_DisplayICON(ICON_WearBad);
			TS_SendEvent(gOledDisTaskID,gOledDisEventClearModeDis_c);  //��ʾ�������ȷ���棬��ʱ����
		}
		
		if(events & gOledDisEventModeSecTick_c)  //��ʾ���ģʽͼ���¼�
		{
			OLED_DisplayClear();
			OLED_DisplaySecTick(0,0,0,0);
			TS_SendEvent(gOledDisTaskID,gOledDisEventClearModeDis_c);   //��ʾ���ģʽͼ����棬��ʱ����
		}
		if(events & gOledDisEventDisplayAllRAM_c)   //����RAM����ֵOLED��ʾ��
		{
			OLED_Refresh_AllGram();
			OLEDDisplay_Stat_Set(NormalDis);   //�ָ�������ʾģʽ
		}
		if(events & gOledDisEventClearKeepMoveDis_c)  //��ʱ��������־�Ϣ������
		{
			/***************�����־�Ϣ������ͣ��10s��ر���Ļ*********************/
				if(gShutDownOledTIMID != TIMER_ERROR)   //�ѷ���Timer
				{				
					Timer_Free(gShutDownOledTIMID);    //�������������ʱ��
					gShutDownOledTIMID = TIMER_ERROR;
				}
				
				/* ������ʾ����ģʽʱ������ر���Ļʱ�䶨ʱ�� */
				Tim_Cfg_ClaerOLED.enuTimerType = TIM_TYPE_MS;
				Tim_Cfg_ClaerOLED.u16TimePeriod = Normal_OLEDShutDownTIM;
				Tim_Cfg_ClaerOLED.NVIC_IRQChannelPreemptionPriority = OLEDShutDown_TIMPreemptionPriority;
				Tim_Cfg_ClaerOLED.pIntCallBack = ShutDownOLED;

					/* Init timer top define */
					Tim_Cfg_ClaerOLED_Index.TimerMode 			= TIM_MODE_BASIC;
					Tim_Cfg_ClaerOLED_Index.TimerBasicCfg 		= &Tim_Cfg_ClaerOLED;
					Tim_Cfg_ClaerOLED_Index.TimerPWMCfg 		= NULL;

				gShutDownOledTIMID = Timer_Allocate(&Tim_Cfg_ClaerOLED_Index);
				Start_Timer_Cnt(gShutDownOledTIMID);
				if(Get_OLED_Dis_Status() == OLEDDisON)
				{
					Set_OLED_Dis_Status(OLEDDisEnterShutDown);     //��ʱ����Ļ�ر�
				}
		}
		if(events & gOledDisEventClearModeDis_c)
		{
			if(Device_Mode == Device_Mode_PassKey)
			{
                //��ʾPasskeyģʽʱ����ʱ����
                if(gShutDownOledTIMID != TIMER_ERROR)   //�ѷ���Timer
                {				
                    Timer_Free(gShutDownOledTIMID);    //�������������ʱ��
                    gShutDownOledTIMID = TIMER_ERROR;
                }
                /* ������ʾ����ģʽʱ���󶨳�ʱ��ʱ�� */
                Tim_Cfg_ClaerOLED.enuTimerType = TIM_TYPE_MS;
                Tim_Cfg_ClaerOLED.u16TimePeriod = BoundTimeOutTIM;
                Tim_Cfg_ClaerOLED.NVIC_IRQChannelPreemptionPriority = OLEDShutDown_TIMPreemptionPriority;
                Tim_Cfg_ClaerOLED.pIntCallBack = BoundTimeOut;

                /* Init timer top define */
                Tim_Cfg_ClaerOLED_Index.TimerMode 			= TIM_MODE_BASIC;
                Tim_Cfg_ClaerOLED_Index.TimerBasicCfg 		= &Tim_Cfg_ClaerOLED;
                Tim_Cfg_ClaerOLED_Index.TimerPWMCfg 		= NULL;

				gShutDownOledTIMID = Timer_Allocate(&Tim_Cfg_ClaerOLED_Index);
				Start_Timer_Cnt(gShutDownOledTIMID);
				if(Get_OLED_Dis_Status() == OLEDDisON)
				{
					Set_OLED_Dis_Status(OLEDDisEnterShutDown);     //�����ر�OLED��ʾ��ʱ
				}
			}	
            else if(Device_Mode == Device_Mode_FWUpdate)
            {
                Set_OLED_Dis_Status(OLEDDisON);
                if(gShutDownOledTIMID != TIMER_ERROR)   //�ѷ���Timer
				{				
					Timer_Free(gShutDownOledTIMID);    //�������������ʱ��
					gShutDownOledTIMID = TIMER_ERROR;
				}
            }
			else if(Device_Mode == Device_Mode_Charge)
			{
				/***************������ͣ��10s��ر���Ļ*********************/
				if(gShutDownOledTIMID != TIMER_ERROR)   //�ѷ���Timer
				{				
					Timer_Free(gShutDownOledTIMID);    //�������������ʱ��
					gShutDownOledTIMID = TIMER_ERROR;
				}
				
				/* ������ʾ����ģʽʱ������ر���Ļʱ�䶨ʱ�� */
				Tim_Cfg_ClaerOLED.enuTimerType = TIM_TYPE_MS;
				Tim_Cfg_ClaerOLED.u16TimePeriod = Charge_OLEDShutDownTIM;
				Tim_Cfg_ClaerOLED.NVIC_IRQChannelPreemptionPriority = OLEDShutDown_TIMPreemptionPriority;
				Tim_Cfg_ClaerOLED.pIntCallBack = ShutDownOLED;

					/* Init timer top define */
					Tim_Cfg_ClaerOLED_Index.TimerMode 			= TIM_MODE_BASIC;
					Tim_Cfg_ClaerOLED_Index.TimerBasicCfg 		= &Tim_Cfg_ClaerOLED;
					Tim_Cfg_ClaerOLED_Index.TimerPWMCfg 		= NULL;

				gShutDownOledTIMID = Timer_Allocate(&Tim_Cfg_ClaerOLED_Index);
				Start_Timer_Cnt(gShutDownOledTIMID);
				if(Get_OLED_Dis_Status() == OLEDDisON)
				{
					Set_OLED_Dis_Status(OLEDDisEnterShutDown);     //�����ر�OLED��ʾ��ʱ
				}
			}
			else   //������ʾPasskeyģʽʱ������ͣ��10s��ر���Ļ
			{
				/***************��ͨ����ͣ��10s��ر���Ļ*********************/
				if(gShutDownOledTIMID != TIMER_ERROR)   //�ѷ���Timer
				{				
					Timer_Free(gShutDownOledTIMID);    //�������������ʱ��
					gShutDownOledTIMID = TIMER_ERROR;
				}
				
				/* ������ʾ����ģʽʱ������ر���Ļʱ�䶨ʱ�� */
				Tim_Cfg_ClaerOLED.enuTimerType = TIM_TYPE_MS;
				Tim_Cfg_ClaerOLED.u16TimePeriod = Normal_OLEDShutDownTIM;
				Tim_Cfg_ClaerOLED.NVIC_IRQChannelPreemptionPriority = OLEDShutDown_TIMPreemptionPriority;
				Tim_Cfg_ClaerOLED.pIntCallBack = ShutDownOLED;

					/* Init timer top define */
					Tim_Cfg_ClaerOLED_Index.TimerMode 			= TIM_MODE_BASIC;
					Tim_Cfg_ClaerOLED_Index.TimerBasicCfg 		= &Tim_Cfg_ClaerOLED;
					Tim_Cfg_ClaerOLED_Index.TimerPWMCfg 		= NULL;

				gShutDownOledTIMID = Timer_Allocate(&Tim_Cfg_ClaerOLED_Index);
				Start_Timer_Cnt(gShutDownOledTIMID);
				if(Get_OLED_Dis_Status() == OLEDDisON)
				{
					Set_OLED_Dis_Status(OLEDDisEnterShutDown);     //�����ر�OLED��ʾ��ʱ
				}
			}
		}        
        if(events & gOledDisEventModeCHARGE_c)   // Battery charge Event
		{
            if(GetBatChargeState())                         // In charging
            {
                if(GetBatFullChargeState())
                {
					#ifdef Battery_Debug
                    printf("Now the battery is full charge...\r\n");
					#endif
					OLED_DisplayBATCharingFlash(OFF);
					OLED_DisplayClear();
					OLED_DisplayICON(ICON_ModeFullCharge);
                }
				else  //charging
				{
					#ifdef Battery_Debug
					printf("Now the battery is in charging...\r\n"); 
					#endif
					OLED_DisplayClear();
					OLED_DisplayICON(ICON_ModeCharging);
				}
            }
            else                                            // Out of charge
            {
				#ifdef Battery_Debug
					printf("Now the battery is out of charge...%f.\r\n",batteryVoltage.fVoltage); 
				#endif
                
                switch(GetBatteryAlarmState())              // Alarm...
                {
                    case BattStateNormal:
						#ifdef Battery_Debug
							printf("Now the battery is no alarm...\r\n");
						#endif
					
						Device_Mode = Device_Mode_RTC;    /* 	Set Device Mode to RTC 	*/
						Set_OLED_Dis_Status(OLEDDisON);
						TS_SendEvent(gOledDisTaskID,gOledDisEventBlueIcon_c);   /* 	Show the RTC GUI 	*/
                        break;
                    
                    case BattStateLow:                               // Low alarm
                        #ifdef Battery_Debug
							printf("Now the battery is low...\r\n");
						#endif
						
						Device_Mode = Device_Mode_LowBattery; 		/* 	Set Device Mode to LowBattery Mode 		*/
						SPI_AlarmTransmit(Alarm_ID_LowBattery);
						OLED_DisplayClear();
						OLED_DisplayICON(ICON_ModeLowBattery);
						OLED_DisplayCtl(ON);
                        break;
                    
                    case BattStateOut:                               // Out alarm  shut off
                        #ifdef Battery_Debug
							printf("Now the battery is out...\r\n");
						#endif

						/* 	[ ZHUYUN ]Shutdown OLED While still in Out of Power		*/
						Device_Mode = Device_Mode_LowBattery;
						//OLED_DisplayCtl(OFF);	
						//Set_OLED_Dis_Status(OLEDDisShutDown);
                        
                        /* Added by huayun,2015-04-20 */
                        OLED_DisplayClear();
						OLED_DisplayICON(ICON_ModeLowBattery);
						OLED_DisplayCtl(ON);
                        break;

                    default :
                        break;
                }
            }
			TS_SendEvent(gOledDisTaskID,gOledDisEventClearModeDis_c);  //�ر���Ļ��ʾ��ʱ
		}
}
void ShutDownOLED(Timer_ID_Typedef TIMID)
{
		TIMID = TIMID;
		Timer_Free(gShutDownOledTIMID);
		gShutDownOledTIMID = TIMER_ERROR;
		OLED_DisplayCtl(OFF);	
		Set_OLED_Dis_Status(OLEDDisShutDown);
		OLEDDisplay_Stat_Set(NormalDis);
}
void BoundTimeOut(Timer_ID_Typedef TIMID)
{
		TIMID = TIMID;
		Timer_Free(gShutDownOledTIMID);
		gShutDownOledTIMID = TIMER_ERROR;
		/*�󶨳�ʱ*/
        /* 	Detect the charge status  */
        if(GPIO_ReadInputDataBit(GPIO_BAT_PG,EXTI_LineBAT_PG) == Bit_RESET)
        {
            /* Send the event to BatManage task */
            TS_SendEvent(gTsBatManageTaskID_c,gBATEnterChargeEvent);
        }
        else if(Device_Mode != Device_Mode_Charge && Device_Mode != Device_Mode_LowBattery)
		{
			Device_Mode = Device_Mode_RTC; //�󶨳�ʱ�ص�RTCģʽ
			TS_SendEvent(gOledDisTaskID,gOledDisEventBlueIcon_c);  //������ʾRTC����
		}
}
 
/********************** BOARD_REDHARE_V3_0   GUI�ӿں��� ��ʼ********************************************************/

//void OLED_DisplayNum14x28(uint8_t x, uint8_t y, uint8_t Num)
//{
//		uint8_t       i=0;
//	
//        MXS8475_WriteReg(0x02,0x00);
//		MXS8475_WriteReg(0x1D,0x08);	
//		
//		MXS8475_WriteReg(0x34,x);
//        MXS8475_WriteReg(0x35,x+1);
//        MXS8475_WriteReg(0x36,y);
//        MXS8475_WriteReg(0x37,y+27);
//		
//        MXS8475_WriteCMD(0x08);
//			
//        for(i=0;i<56;i++)
//		{
//			MXS8475_WriteDat(Num14x28[Num][i]);
//		}
//        MXS8475_WriteReg(0x02,0x01);
//}

//void OLED_DisplayDian14x28(void)
//{
//		uint8_t       i=0;
//	
//        MXS8475_WriteReg(0x02,0x00);
//		MXS8475_WriteReg(0x1D,0x08);	
//		
//		MXS8475_WriteReg(0x34,4);
//        MXS8475_WriteReg(0x35,4);
//        MXS8475_WriteReg(0x36,1);
//        MXS8475_WriteReg(0x37,27);
//		
//        MXS8475_WriteCMD(0x08);
//			
//        for(i=0;i<28;i++)
//		{
//			MXS8475_WriteDat(dian[i]);
//		}
//        MXS8475_WriteReg(0x02,0x01);
//}
/*******************************************************************************
* Function Name  : OLED_Clear_AllGram
* Description    : ���ȫ������ʾRAM
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/	 
//static void OLED_Clear_AllGram(void)
//{
//	uint8_t i,n;	
//	
//	for(i=0;i<39;i++)  
//	{  
//		for(n=0;n<12;n++)
//		{
//			s_ucGRAM[i][n]=0x00;	
//		}
//	}	
//}
/*******************************************************************************
* Function Name  : OLED_Refresh_AllGram
* Description    : ����ȫ��OLED��ʾGram
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/	 
static void OLED_Refresh_AllGram(void)
{
	uint8_t i,n;		    
		
	if(Get_OLED_Dis_Status() != OLEDDisShutDown)
	{
		if(OFF == GetPeriphPowerStatus())  //�����ⲿ��Դ
		{
			PeriPower(ON);
		}
		if(Get_OLED_Config_Status() == DISABLE)
		{
			OLED_Configuration();
		}
		
		MXS8475_WriteReg(0x02,0x00);
		MXS8475_WriteReg(0x1D,0x80);

		MXS8475_WriteReg(0x34,0);
		MXS8475_WriteReg(0x35,11);
		MXS8475_WriteReg(0x36,0);
		MXS8475_WriteReg(0x37,38);

		MXS8475_WriteCMD(0x08);
			
		for(i=0;i<39;i++)  
		{  
			for(n=0;n<12;n++)
			{
				MXS8475_WriteDat(s_ucGRAM[i][n]);	
			}
		}		
		
		MXS8475_WriteReg(0x02,0x01);	
	}
			
}
/*******************************************************************************
* Function Name  : OLED_Refresh_Gram
* Description    : ����ָ������OLED��ʾGram
* Input          : _XS��x start
*				   _XE: x end
*                  _YS: y start
*                  _YE: y end
* Output         : None
* Return         : None
*******************************************************************************/	 
static void OLED_Refresh_Gram(uint8_t _XS, uint8_t _XE, uint8_t _YS, uint8_t _YE)
{
	uint8_t i,n;
	uint8_t XS;
	uint8_t XE;
	uint8_t YS;
	uint8_t YE;
	
	XS = _XS / 8;
	XE = _XE / 8;
	YS = _YS;
	YE = _YE;
	
	if(Get_OLED_Dis_Status() != OLEDDisShutDown)
	{
		if(OFF == GetPeriphPowerStatus())  //�����ⲿ��Դ
		{
			PeriPower(ON);
		}
		if(Get_OLED_Config_Status() == DISABLE)
		{
			OLED_Configuration();
		}
		
		if(NormalDis == OLEDDisplay_Stat_Get())   //����ģʽ����ʾ
		{
			MXS8475_WriteReg(0x02,0x00);
			MXS8475_WriteReg(0x1D,0x80);

			MXS8475_WriteReg(0x34,XS);
			MXS8475_WriteReg(0x35,XE);
			MXS8475_WriteReg(0x36,YS);
			MXS8475_WriteReg(0x37,YE);

			MXS8475_WriteCMD(0x08);
				
			for(i=YS;i<=YE;i++)  
			{  
				for(n=XS;n<=XE;n++)
				{
					MXS8475_WriteDat(s_ucGRAM[i][n]);	
				}
			}		
			
			MXS8475_WriteReg(0x02,0x01);	
		}	
	}	
}	
/*******************************************************************************
* Function Name  : OLED_DrawPoint
* Description    : ��ָ����������1��0��GRAM�У�
* Input          : _usX,_usY,_ucColor:1(���) or 0(���)
* Output         : None
* Return         : None
*******************************************************************************/
static void OLED_DrawPoint(uint8_t _usX,uint8_t _usY,uint8_t _ucColor)
{
	uint8_t ucValue;
	uint8_t ucPageAddr;
	uint8_t ucColAddr;
	
	//const uint8_t aOrTab[8]  = {0x01, 0x02, 0x04, 0x08,0x10,0x20,0x40,0x80};
	//const uint8_t aAndTab[8] = {0xFE, 0xFD, 0xFB, 0xF7,0xEF,0xDF,0xBF,0x7F};

	const uint8_t aOrTab[8]  = {0x80, 0x40, 0x20, 0x10,0x08,0x04,0x02,0x01};
	const uint8_t aAndTab[8] = {0x7F, 0xBF, 0xDF, 0xEF,0xF7,0xFB,0xFD,0xFE};

	if((_usX>96) || (_usY>39))
	{
		return;//������Χ��.
	}
	
	ucPageAddr = _usX / 8;
	ucColAddr = _usY;

	ucValue = s_ucGRAM[ucColAddr][ucPageAddr];
	if (_ucColor == 0)
	{
		ucValue &= aAndTab[_usX % 8];
	}
	else
	{
		ucValue |= aOrTab[_usX % 8];
	}
	s_ucGRAM[ucColAddr][ucPageAddr] = ucValue;    
} 
/*******************************************************************************
* Function Name  : OLED_Fill
* Description    : ���ָ������
* Input          : x1��x start
*				   x2: x end
*                  y1: y start
*                  y2: y end
*				   dot:1(���) or 0(���)
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_Fill(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t dot)  
{  
	uint8_t x,y;  
	for(x=x1;x<=x2;x++)
	{
		for(y=y1;y<=y2;y++)
		{
			OLED_DrawPoint(x,y,dot);
		}
	}													    
	OLED_Refresh_Gram(x1,x2,y1,y2);//������ʾ
}
/*******************************************************************************
* Function Name  : OLED_DisplayNum
* Description    : ��ָ���������ʾ����
* Input          : x,y,chr:����,Font:���壺Font_14x28/Font_11x18/Font_6x7
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_DisplayNum(uint8_t x,uint8_t y,uint8_t chr,uint8_t Font)
{      			    
	uint8_t temp,t,t1,temp1;
	uint8_t mode=1;
	uint8_t x0=x;
	uint8_t y0=y;
	
	switch(Font)
	{
		case Font_14x28:
			for(t=0;t<28;t++)
			{
				temp = Num14x28[chr][t*2];
				temp1 = Num14x28[chr][t*2+1];
				
				for(t1=0;t1<8;t1++)
				{
					if(temp&0x80)
					{
						OLED_DrawPoint(x,y,mode);
					}
					else 
					{
						OLED_DrawPoint(x,y,!mode);
					}
					temp<<=1;
					x++;
				}	
				
				for(t1=0;t1<6;t1++)
				{
					if(temp1&0x80)
					{
						OLED_DrawPoint(x,y,mode);
					}
					else 
					{
						OLED_DrawPoint(x,y,!mode);
					}
					temp1<<=1;
					x++;
				}	
				x=x0;
				y++;
			}   
			OLED_Refresh_Gram(x0,(x0+14),y0,(y0+28));
		break;
		case Font_11x18:
			for(t=0;t<18;t++)
			{
				temp = Num11x18[chr][t*2];
				temp1 = Num11x18[chr][t*2+1];
				
				for(t1=0;t1<8;t1++)
				{
					if(temp&0x80)
					{
						OLED_DrawPoint(x,y,mode);
					}
					else 
					{
						OLED_DrawPoint(x,y,!mode);
					}
					temp<<=1;
					x++;
				}	
				
				for(t1=0;t1<3;t1++)
				{
					if(temp1&0x80)
					{
						OLED_DrawPoint(x,y,mode);
					}
					else 
					{
						OLED_DrawPoint(x,y,!mode);
					}
					temp1<<=1;
					x++;
				}	
				x=x0;
				y++;
			}   
			OLED_Refresh_Gram(x0,(x0+11),y0,(y0+18));
		break;
		case Font_6x7:
			for(t=0;t<7;t++)
			{
				temp = Num6x7[chr][t];
				
				for(t1=0;t1<6;t1++)
				{
					if(temp&0x80)
					{
						OLED_DrawPoint(x,y,mode);
					}
					else 
					{
						OLED_DrawPoint(x,y,!mode);
					}
					temp<<=1;
					x++;
				}	
				x=x0;
				y++;
			}   
			OLED_Refresh_Gram(x0,(x0+6),y0,(y0+7));
		break;
		default:break;
	}
}
/*******************************************************************************
* Function Name  : OLED_DisplayColon_Font_14x28
* Description    : ��ָ���������ʾ":"
* Input          : x,y
* Output         : None
* Return         : None
*******************************************************************************/
static void OLED_DisplayColon_Font_14x28(uint8_t x,uint8_t y)
{
	uint8_t temp,t,t1;
	uint8_t mode=1;
	uint8_t x0=x;
	uint8_t y0=y;
	
	for(t=0;t<28;t++)
	{
		temp = Colon4x28[t];
		
		for(t1=0;t1<4;t1++)
		{
			if(temp&0x80)
			{
				OLED_DrawPoint(x,y,mode);
			}
			else 
			{
				OLED_DrawPoint(x,y,!mode);
			}
			temp<<=1;
			x++;
		}	
		x=x0;
		y++;
	}   
	OLED_Refresh_Gram(x0,(x0+4),y0,(y0+28));
}
/*******************************************************************************
* Function Name  : OLED_DisplayDot_Font_11x18
* Description    : ��ָ���������ʾ"."
* Input          : x,y
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_DisplayDot_Font_11x18(uint8_t x,uint8_t y)
{
	uint8_t temp,t,t1;
	uint8_t mode=1;
	uint8_t x0=x;
	uint8_t y0=y;
	
	for(t=0;t<18;t++)
	{
		temp = Dot2x18[t];
		
		for(t1=0;t1<2;t1++)
		{
			if(temp&0x80)
			{
				OLED_DrawPoint(x,y,mode);
			}
			else 
			{
				OLED_DrawPoint(x,y,!mode);
			}
			temp<<=1;
			x++;
		}	
		x=x0;
		y++;
	}   
	OLED_Refresh_Gram(x0,(x0+2),y0,(y0+18));
}
/*******************************************************************************
* Function Name  : OLED_DisplayMonitorTemplateICON_Font_11x7
* Description    : ��ָ���������ʾ���ģ��ͼ��
* Input          : x,y
* Output         : None
* Return         : None
*******************************************************************************/
static void OLED_DisplayMonitorTemplateICON_Font_11x7(uint8_t x,uint8_t y)
{
	uint8_t temp,t,t1,temp1;
	uint8_t mode=1;
	uint8_t x0=x;
	uint8_t y0=y;
	
	for(t=0;t<7;t++)
	{
		temp = MonitorTemplate11x7[t*2];
		temp1 = MonitorTemplate11x7[t*2+1];
		
		for(t1=0;t1<8;t1++)
		{
			if(temp&0x80)
			{
				OLED_DrawPoint(x,y,mode);
			}
			else 
			{
				OLED_DrawPoint(x,y,!mode);
			}
			temp<<=1;
			x++;
		}	
		
		for(t1=0;t1<3;t1++)
		{
			if(temp1&0x80)
			{
				OLED_DrawPoint(x,y,mode);
			}
			else 
			{
				OLED_DrawPoint(x,y,!mode);
			}
			temp1<<=1;
			x++;
		}	
		x=x0;
		y++;
	}   
	OLED_Refresh_Gram(x0,(x0+11),y0,(y0+7));
}
/*******************************************************************************
* Function Name  : OLED_ClearMonitorTemplateICON_Font_11x7
* Description    : ��ָ�������������ģ��ͼ����ʾ
* Input          : x,y
* Output         : None
* Return         : None
*******************************************************************************/
static void OLED_ClearMonitorTemplateICON_Font_11x7(uint8_t x,uint8_t y)
{
	uint8_t t,t1;
	uint8_t mode=1;
	uint8_t x0=x;
	uint8_t y0=y;
	
	for(t=0;t<7;t++)
	{		
		for(t1=0;t1<8;t1++)
		{
			OLED_DrawPoint(x,y,!mode);
			x++;
		}	
		
		for(t1=0;t1<3;t1++)
		{
			OLED_DrawPoint(x,y,!mode);
			x++;
		}	
		x=x0;
		y++;
	}   
	OLED_Refresh_Gram(x0,(x0+11),y0,(y0+7));
}
/*******************************************************************************
* Function Name  : OLED_DisplayBlueToothICON_Font_11x7
* Description    : ��ָ���������ʾ����ͼ��
* Input          : x,y
* Output         : None
* Return         : None
*******************************************************************************/
static void OLED_DisplayBlueToothICON_Font_11x7(uint8_t x,uint8_t y)
{
	uint8_t temp,t,t1,temp1;
	uint8_t mode=1;
	uint8_t x0=x;
	uint8_t y0=y;
	
	for(t=0;t<7;t++)
	{
		temp = BlueTooth11x7[t*2];
		temp1 = BlueTooth11x7[t*2+1];
		
		for(t1=0;t1<8;t1++)
		{
			if(temp&0x80)
			{
				OLED_DrawPoint(x,y,mode);
			}
			else 
			{
				OLED_DrawPoint(x,y,!mode);
			}
			temp<<=1;
			x++;
		}	
		
		for(t1=0;t1<3;t1++)
		{
			if(temp1&0x80)
			{
				OLED_DrawPoint(x,y,mode);
			}
			else 
			{
				OLED_DrawPoint(x,y,!mode);
			}
			temp1<<=1;
			x++;
		}	
		x=x0;
		y++;
	}   
	OLED_Refresh_Gram(x0,(x0+11),y0,(y0+7));
}
/*******************************************************************************
* Function Name  : OLED_ClearBlueToothICON_Font_11x7
* Description    : ��ָ��������������ͼ��
* Input          : x,y
* Output         : None
* Return         : None
*******************************************************************************/
static void OLED_ClearBlueToothICON_Font_11x7(uint8_t x,uint8_t y)
{
	uint8_t t,t1;
	uint8_t mode=1;
	uint8_t x0=x;
	uint8_t y0=y;
	
	for(t=0;t<7;t++)
	{
		for(t1=0;t1<8;t1++)
		{
			OLED_DrawPoint(x,y,!mode);
			x++;
		}	
		for(t1=0;t1<3;t1++)
		{
			OLED_DrawPoint(x,y,!mode);
			x++;
		}	
		x=x0;
		y++;
	}   
	OLED_Refresh_Gram(x0,(x0+11),y0,(y0+7));
}
/*******************************************************************************
* Function Name  : OLED_DisplayColon_Font_6x7
* Description    : ��ָ���������ʾ":"
* Input          : x,y
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_DisplayColon_Font_6x7(uint8_t x,uint8_t y)
{
	uint8_t temp,t,t1;
	uint8_t mode=1;
	uint8_t x0=x;
	uint8_t y0=y;
	
	for(t=0;t<10;t++)
	{
		temp = Colon4x7[t];
		
		for(t1=0;t1<2;t1++)
		{
			if(temp&0x80)
			{
				OLED_DrawPoint(x,y,mode);
			}
			else 
			{
				OLED_DrawPoint(x,y,!mode);
			}
			temp<<=1;
			x++;
		}	
		x=x0;
		y++;
	}   
	OLED_Refresh_Gram(x0,(x0+2),y0,(y0+10));
}
/*******************************************************************************
* Function Name  : OLED_DisplaySlash_Font_6x7
* Description    : ��ָ���������ʾ"/"
* Input          : x,y
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_DisplaySlash_Font_6x7(uint8_t x,uint8_t y)
{
	uint8_t temp,t,t1;
	uint8_t mode=1;
	uint8_t x0=x;
	uint8_t y0=y;
	
	for(t=0;t<7;t++)
	{
		temp = Slash4x7[t];
		
		for(t1=0;t1<4;t1++)
		{
			if(temp&0x80)
			{
				OLED_DrawPoint(x,y,mode);
			}
			else 
			{
				OLED_DrawPoint(x,y,!mode);
			}
			temp<<=1;
			x++;
		}	
		x=x0;
		y++;
	}   
	OLED_Refresh_Gram(x0,(x0+4),y0,(y0+7));
}
/*******************************************************************************
* Function Name  : OLED_DisplayBatteryLevel_Font_16x7
* Description    : ��ָ���������ʾ��ص�������
* Input          : x,y,Level:0/1/2/3/4;5:�հ�
* Output         : None
* Return         : None
*******************************************************************************/
static void OLED_DisplayBatteryLevel_Font_16x7(uint8_t x,uint8_t y,uint8_t Level)
{
	uint8_t temp,t,t1,temp1;
	uint8_t mode=1;
	uint8_t x0=x;
	uint8_t y0=y;
	
	for(t=0;t<7;t++)
	{
		temp = BatteryLevel[Level][t*2];
		temp1 = BatteryLevel[Level][t*2+1];
		
		for(t1=0;t1<8;t1++)
		{
			if(temp&0x80)
			{
				OLED_DrawPoint(x,y,mode);
			}
			else 
			{
				OLED_DrawPoint(x,y,!mode);
			}
			temp<<=1;
			x++;
		}	
		
		for(t1=0;t1<8;t1++)
		{
			if(temp1&0x80)
			{
				OLED_DrawPoint(x,y,mode);
			}
			else 
			{
				OLED_DrawPoint(x,y,!mode);
			}
			temp1<<=1;
			x++;
		}	
		x=x0;
		y++;
	}   
	OLED_Refresh_Gram(x0,(x0+16),y0,(y0+7));
}
/*******************************************************************************
* Function Name  : OLED_DisplayProgress_Font_57x3
* Description    : ��ָ���������ʾ������
* Input          : x,y,Progress:1/2/3/4/5/6/7
* Output         : None
* Return         : None
*******************************************************************************/
static void OLED_DisplayProgress_Font_57x3(uint8_t x,uint8_t y,uint8_t Progress)
{
	uint8_t temp,t,t1,t2;
	uint8_t mode=1;
	uint8_t x0=x;
	uint8_t y0=y;
	
	Progress = Progress-1;
	for(t=0;t<3;t++)
	{
		for(t2=0;t2<8;t2++)
		{
			temp = Progress57x3[Progress][t*8+t2];
			if(t2<7)
			{
				for(t1=0;t1<8;t1++)
				{
					if(temp&0x80)
					{
						OLED_DrawPoint(x,y,mode);
					}
					else 
					{
						OLED_DrawPoint(x,y,!mode);
					}
					temp<<=1;
					x++;
				}	
			}
			else
			{
				for(t1=0;t1<1;t1++)
				{
					if(temp&0x80)
					{
						OLED_DrawPoint(x,y,mode);
					}
					else 
					{
						OLED_DrawPoint(x,y,!mode);
					}
					temp<<=1;
					x++;
				}
			}
		}			
		x=x0;
		y++;
	}   
	OLED_Refresh_Gram(x0,(x0+57),y0,(y0+3));
}
/*******************************************************************************
* Function Name  : OLED_DisplayStartProgress
* Description    : ��ָ���������ʾ����������
* Input          : Progress:1/2/3/4/5
* Output         : None
* Return         : None
*******************************************************************************/
static void OLED_DisplayStartProgress(uint8_t progress)
{
		uint8_t  index=0;
	
		index = progress - 1;
		OLED_DisplayBPM(16,15,79,22,StartProgress[index]);
}
/*******************************************************************************
* Function Name  : OLED_DisplayStartProgress
* Description    : ��ָ���������ʾ����������
* Input          : Progress:1/2/3/4/5
* Output         : None
* Return         : None
*******************************************************************************/
static void OLED_DisplayStartPic(const unsigned char * pitrueData)
{
	OLED_DisplayFullScreenBMP(pitrueData);
}
/*******************Ӧ�ò���ʾ�ӿں���********************************************************************************/
/*******************************************************************************
* Function Name  : OLEDDisplay_Stat_Set
* Description    : ����OLED��ʾ״̬,���øú���֮ǰ����ʹ��OLED�����������RAM�е�����
* Input          : newState
* Output         : None
* Return         : None
*******************************************************************************/
void OLEDDisplay_Stat_Set(OLEDDisplayStat newState)
{
		m_LedDisplayState = newState;
}
/*******************************************************************************
* Function Name  : OLEDDisplay_Stat_Get
* Description    : ��ȡOLED��ʾ״̬
* Input          : None
* Output         : None
* Return         : OLEDDisplay State
*******************************************************************************/
OLEDDisplayStat OLEDDisplay_Stat_Get(void)
{
		return (m_LedDisplayState);
}

/*******************************************************************************
* Function Name  : OLED_DisplayBlueToothICON
* Description    : ��ʾ����ͼ��
* Input          : staus: ON / OFF
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_DisplayBlueToothICON(uint8_t status)
{
	if(status == ON)
	{
		OLED_DisplayBlueToothICON_Font_11x7(81,10);
	}
	else
	{
		OLED_ClearBlueToothICON_Font_11x7(81,10);
	}
}
/*******************************************************************************
* Function Name  : OLED_DisplayMonitorTemplateICON
* Description    : ��ʾ���ģ������ͼ��
* Input          : staus: ON / OFF
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_DisplayMonitorTemplateICON(uint8_t status)
{
	if(status == ON)
	{
		OLED_DisplayMonitorTemplateICON_Font_11x7(81,1);
	}
	else
	{
		OLED_ClearMonitorTemplateICON_Font_11x7(81,1);
	}
}
/*******************************************************************************
* Function Name  : OLED_DisplayBatteryLevelICON
* Description    : ��ʾ��ص���
* Input          : Level: 0/1/2/3/4;5:�հ�
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_DisplayBatteryLevelICON(uint8_t Level)
{
	OLED_DisplayBatteryLevel_Font_16x7(78,21,Level);
}
/*******************************************************************************
* Function Name  : OLED_DisplayBatteryChargrLevelICON
* Description    : ��ʾ�������µ�ص���
* Input          : Level: 0/1/2/3/4;5:�հ�
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_DisplayBatteryChargrLevelICON(uint8_t Level)
{
    #ifdef UI_CHINESE_ENABLE
        OLED_DisplayBPM(71,11,82,29,CHINAESE_BatteryLevel[Level]);
    #endif
}
/*******************************************************************************
* Function Name  : OLED_DisplayProgress
* Description    : ��ʾ������
* Input          : x,y,Progress:1/2/3/4/5/6/7
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_DisplayProgress(uint8_t Progress)
{
    if(Progress == 1)
    {
        OLED_DisplayBPM(39,32,55,35,Progress1);
    }
    else
    {
        OLED_DisplayProgress_Font_57x3(24,32,Progress);
    }
}
/*******************************************************************************
* Function Name  : OLED_DisplayTestMode
* Description    : ��ʾ����ģʽ
* Input          : ON/OFF
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_DisplayTestMode(uint8_t Status)
{
    if(Status == ON)
    {
        OLED_DisplayBPM(1,32,17,35,TestModePicON);
    }
    else
    {
        OLED_DisplayBPM(1,32,17,35,TestModePicOFF);
    }
}
/*******************************************************************************
* Function Name  : OLED_DisplayRTC
* Description    : ��ʾRTCʵʱʱ��
* Input          : Hour_H,Hour_L,Min_H,Min_L
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_DisplayRTC(uint8_t Hour_H,uint8_t Hour_L,uint8_t Min_H,uint8_t Min_L)
{
	uint8_t BatteryLevel=0;
	static  uint8_t Ctl=0;
	
	RTC_TimeTypeDef     rtc_time;             //RTC ʱ��
	RTC_GetTime(RTC_Format_BIN, &rtc_time);
	
	if(Device_Mode == Device_Mode_RTC)
	{
		OLED_DisplayNum(1,1,Hour_H,Font_14x28);
		OLED_DisplayNum(16,1,Hour_L,Font_14x28);
		OLED_DisplayColon_Font_14x28(33,1);
		OLED_DisplayNum(40,1,Min_H,Font_14x28);
		OLED_DisplayNum(56,1,Min_L,Font_14x28);
		
		/* ��ص���Ϊ0��ʱ��1s��˸һ��RTC�����µĵ��ͼ�� */
		BatteryLevel = GetBatScaleLevel();   //��ȡ��ص���
		
		if(BatteryLevel == 0)
		{
			if(Ctl == 0)
			{
				Ctl=1;
				OLED_DisplayBatteryLevelICON(BatteryLevel); //��ʾ��ص���
			}
			else
			{
				Ctl=0;
				OLED_DisplayBatteryLevelICON(BlankBatteryLevel); //��ʾ�հ׵�ص���
			}
		}
	}
}
/*******************************************************************************
* Function Name  : OLED_DisplaySecTick
* Description    : ��ʾ���
* Input          : Counter
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_DisplaySecTick(uint8_t Hour,uint8_t Min,uint8_t Sec,uint8_t SubSec)
{
		uint8_t Hours_H=0;
		uint8_t Hours_L=0;
		uint8_t Minutes_H=0;
		uint8_t Minutes_L=0;
		uint8_t Sec_H=0;
		uint8_t Sec_L=0;
		//uint8_t SubSec_H=0;
		uint8_t SubSec_L=0;
	
		Hours_H = (Hour / 10);
		Hours_L = (Hour % 10);
	
		Minutes_H = (Min / 10);
		Minutes_L = (Min % 10);
	
		Sec_H = (Sec / 10);
		Sec_L = (Sec % 10);
	 
	    //SubSec_H = (SubSec / 10);
		SubSec_L = (SubSec % 10);

		OLED_DisplayNum(3,10,Hours_H,Font_11x18);
		OLED_DisplayNum(16,10,Hours_L,Font_11x18);
		
		OLED_DisplayColon_Font_6x7(28,14);
		
		OLED_DisplayNum(31,10,Minutes_H,Font_11x18);
		OLED_DisplayNum(44,10,Minutes_L,Font_11x18);
		
		OLED_DisplayColon_Font_6x7(56,14);
		
		OLED_DisplayNum(59,10,Sec_H,Font_11x18);
		OLED_DisplayNum(72,10,Sec_L,Font_11x18);
		
		//OLED_DisplayNum(3,10,SubSec_H,Font_11x18);
		OLED_DisplayNum(85,21,SubSec_L,Font_6x7);
		
}
/*******************************************************************************
* Function Name  : OLED_DisplayPassKey
* Description    : ��ʾPassKey
* Input          : passkey
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_DisplayPassKey(uint8_t *	pKeyBuf)
{
	Set_OLED_Dis_Status(OLEDDisON);
	OLED_DisplayNum(13,11,pKeyBuf[0],Font_11x18);
	OLED_DisplayNum(32,11,pKeyBuf[1],Font_11x18);
	OLED_DisplayNum(51,11,pKeyBuf[2],Font_11x18);
	OLED_DisplayNum(70,11,pKeyBuf[3],Font_11x18);
}
/*******************************************************************************
* Function Name  : OLED_DisplayCtl
* Description    : �����ر�OLED��ʾ
* Input          : ON/OFF
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_DisplayCtl(uint8_t status)
{
		if(OFF == GetPeriphPowerStatus())  //�����ⲿ��Դ
		{
			PeriPower(ON);
		}
		if(Get_OLED_Config_Status() == DISABLE)
		{
			OLED_Configuration();
		}
		
		if(status == ON)
		{
			//MXS8475_WriteReg(0x02,0x01);  //OLED display on
		}
		else
		{
			if(Device_Mode == Device_Mode_Charge)
			{
				/* Shut Down BATCharing Flash in Device_Mode_Charge */
				OLED_DisplayBATCharingFlash(OFF);
			}
			MXS8475_WriteReg(0x02,0x00);  // OLED display off
			OLED_DeConfiguration();
		}
}
/*******************************************************************************
* Function Name  : OLED_DisplayClear
* Description    : �����ʾ
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_DisplayClear(void)
{
		uint8_t i,n;
	
		if(OFF == GetPeriphPowerStatus())  //�����ⲿ��Դ
		{
			PeriPower(ON);
		}
		if(Get_OLED_Config_Status() == DISABLE)
		{
			OLED_Configuration();
		}

		MXS8475_WriteReg(0x02,0x00);
		MXS8475_WriteReg(0x1D,0x80);

		MXS8475_WriteReg(0x34,0);
		MXS8475_WriteReg(0x35,11);
		MXS8475_WriteReg(0x36,0);
		MXS8475_WriteReg(0x37,38);

		MXS8475_WriteCMD(0x08);

		for(i=0;i<39;i++)  
		{  
			for(n=0;n<12;n++)
			{
				s_ucGRAM[i][n]=0x00;
				MXS8475_WriteDat(s_ucGRAM[i][n]);	
			}
		}	

		MXS8475_WriteReg(0x02,0x01);
}
/*******************************************************************************
* Function Name  : OLED_DisplayClear
* Description    : �����ʾ
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_DisplayClearWithOutRAM(void)
{
		uint8_t i,n;
	
		if(OFF == GetPeriphPowerStatus())  //�����ⲿ��Դ
		{
			PeriPower(ON);
		}
		if(Get_OLED_Config_Status() == DISABLE)
		{
			OLED_Configuration();
		}

		MXS8475_WriteReg(0x02,0x00);
		MXS8475_WriteReg(0x1D,0x80);

		MXS8475_WriteReg(0x34,0);
		MXS8475_WriteReg(0x35,11);
		MXS8475_WriteReg(0x36,0);
		MXS8475_WriteReg(0x37,38);

		MXS8475_WriteCMD(0x08);

		for(i=0;i<39;i++)  
		{  
			for(n=0;n<12;n++)
			{
				MXS8475_WriteDat(0x00);	
			}
		}	

		MXS8475_WriteReg(0x02,0x01);
}
/*******************************************************************************
* Function Name  : Start_FLASH_Display
* Description    : ��ʾ��������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static  void Start_FLASH_Display(Timer_ID_Typedef TIMID)
{
	static uint8_t index=0;
	
	switch(index)
	{
		case 0 :
			OLED_DisplayStartProgress(1);
			index = index + 1;
			break;
		case 1 :
			OLED_DisplayStartProgress(2);
			index = index + 1;
			break;
		case 2 :
			OLED_DisplayStartProgress(3);
			index = index + 1;
			break;
		case 3 :
			OLED_DisplayStartProgress(4);
			index = index + 1;
			break;
		case 4 :
			OLED_DisplayStartProgress(5);
			index = index + 1;
			break;
		case 5 :
			OLED_DisplayStartProgress(4);
			index = index + 1;
			break;
		case 6 :
			OLED_DisplayStartProgress(3);
			index = index + 1;
			break;
		case 7 :
			OLED_DisplayStartProgress(2);
			index = index + 1;
			break;
		case 8 :
			OLED_DisplayStartProgress(1);
			index = index + 1;
			break;
		case 9 :
			OLED_DisplayClear();
			index = index + 1;
			break;
		case 10 :
			OLED_DisplayStartPic(StartPic);
			index = index + 1;
			break;
		case 11 :
			index = index + 1;
			break;
		case 12 :
			index = index + 1;
			break;
		case 13:
			Stop_Timer_Cnt(StartPicTIMID);
			Timer_Free(StartPicTIMID);
			index = 0;
			gFlag_StartFlashComplete = true;  
			break;
		default:index = 0; break;
	}
}
/*******************************************************************************
* Function Name  : OLED_DisplayStartFlash
* Description    : ��ʾ��������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_DisplayStartFlash(void)
{
		TIM_Basic_Cfg_Typedef   Tim_Cfg_OLED;       // timer���ã����ڿ��ƽ����л�ʱ��
		TIM_Cfg_Typedef         Tim_Cfg_OLED_Index;

		gFlag_StartFlashComplete = false;      
		/* ���ö�ʱ��*/
		Tim_Cfg_OLED.enuTimerType = TIM_TYPE_MS;
		Tim_Cfg_OLED.u16TimePeriod = StartFlashTimPeriod;
		Tim_Cfg_OLED.NVIC_IRQChannelPreemptionPriority = OLED_TIMPreemptionPriority;
		Tim_Cfg_OLED.pIntCallBack = Start_FLASH_Display;

		/* Init timer top define */
		Tim_Cfg_OLED_Index.TimerMode        = TIM_MODE_BASIC;
		Tim_Cfg_OLED_Index.TimerBasicCfg    = &Tim_Cfg_OLED;
		Tim_Cfg_OLED_Index.TimerPWMCfg 	   = NULL;

		if(StartPicTIMID != TIMER_ERROR)   //�ѷ���Timer
		{				
			Timer_Free(StartPicTIMID);    //�������������ʱ��
			StartPicTIMID = TIMER_ERROR;
		}    
		StartPicTIMID = Timer_Allocate(&Tim_Cfg_OLED_Index);
		Start_Timer_Cnt(StartPicTIMID);
}
/*******************************************************************************
* Function Name  : GetOLED_DisplayStartFlashStatus
* Description    : ��ȡ��ʾ�������״̬�
* Input          : None
* Output         : None
* Return         : true:����������ʾ��� false:��������δ��ʾ���
*******************************************************************************/
bool GetOLED_DisplayStartFlashStatus(void)
{
	return gFlag_StartFlashComplete;
}
/*******************************************************************************
* Function Name  : OLED_DisplayBoundRemindFlash
* Description    : �����󶨶���
* Input          : status:ON/OFF
* Output         : None
* Return         : None
*******************************************************************************/
static void BoundRemind_Flash_Display(Timer_ID_Typedef TIM)
{
		static  uint8_t  index=0;
		TIM = TIM;
		
		if(index == 0)
		{
			OLED_DisplayBPM(35,19,65,20,BoundRemindPicON);
			index = 1;
		}
		else
		{
			OLED_DisplayBPM(35,19,65,20,BoundRemindPicOFF);
			index = 0;
		}
		
		
}
/*******************************************************************************
* Function Name  : OLED_DisplayBoundRemindFlash
* Description    : �����󶨶���
* Input          : status:ON/OFF
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_DisplayBoundRemindFlash(uint8_t status)
{
		static Timer_ID_Typedef        BoundPicTIMID;      //�󶨶�����ʱ��
		TIM_Basic_Cfg_Typedef   Tim_Cfg_OLED;       // timer���ã����ڿ��ƽ����л�ʱ��
		TIM_Cfg_Typedef         Tim_Cfg_OLED_Index;
    
		/* ���ö�ʱ��*/
		Tim_Cfg_OLED.enuTimerType = TIM_TYPE_MS;
		Tim_Cfg_OLED.u16TimePeriod = BoundRemindFlashTimPeriod;
		Tim_Cfg_OLED.NVIC_IRQChannelPreemptionPriority = OLED_TIMPreemptionPriority;
		Tim_Cfg_OLED.pIntCallBack = BoundRemind_Flash_Display;

		/* Init timer top define */
		Tim_Cfg_OLED_Index.TimerMode        = TIM_MODE_BASIC;
		Tim_Cfg_OLED_Index.TimerBasicCfg    = &Tim_Cfg_OLED;
		Tim_Cfg_OLED_Index.TimerPWMCfg 	   = NULL;
	
		OLED_DisplayFullScreenBMP(BoundRemindPic2);
	
		if(status == ON)
		{
            if(BoundPicTIMID != TIMER_ERROR)   //�ѷ���Timer
            {				
                Timer_Free(BoundPicTIMID);    //�������������ʱ��
                BoundPicTIMID = TIMER_ERROR;
            } 
			BoundPicTIMID = Timer_Allocate(&Tim_Cfg_OLED_Index);
			Start_Timer_Cnt(BoundPicTIMID);		
		}
		else
		{
			Stop_Timer_Cnt(BoundPicTIMID);
			Timer_Free(BoundPicTIMID);
			BoundPicTIMID = TIMER_ERROR;
			OLED_DisplayClear();
		}
}
/*******************************************************************************
* Function Name  : HR_SpO2Measuring_Flash_Display
* Description    : HR_SpO2�����н��涯����ʾ
* Input          : TIM
* Output         : None
* Return         : None
*******************************************************************************/
static void HR_SpO2Measuring_Flash_Display(Timer_ID_Typedef TIM)
{
		static  uint8_t  index=0;
		TIM = TIM;
	
		if((Device_Mode == Device_Mode_SPO2_HR) && (gSubFunc_Stat_Get(SPO2_HR_SINGLEWORK_STATE) != OFF)) //ֻ����HR_SpO2ģʽ��,���ҵ��豸����ʱ�Ż���ʾ
		{
			switch(index)
			{
				case 0 :
					OLED_DisplayBPM(0,33,95,37,MeasuringPic[0]);
					index = index + 1;
					break;
				case 1 :
					OLED_DisplayBPM(0,33,95,37,MeasuringPic[1]);
					index = index + 1;
					break;
				case 2 :
					OLED_DisplayBPM(0,33,95,37,MeasuringPic[2]);
					index = index + 1;
					break;
				case 3 :
					OLED_DisplayBPM(0,33,95,37,MeasuringPic[3]);
					index = index + 1;
					break;
				case 4 :
					OLED_DisplayBPM(0,33,95,37,MeasuringPic[4]);
					index = index + 1;
					break;
				case 5 :
					OLED_DisplayBPM(0,33,95,37,MeasuringPic[5]);
					index = index + 1;
					break;
				case 6 :
					OLED_DisplayBPM(0,33,95,37,MeasuringPic[6]);
					index = index + 1;
					break;
				case 7 :
					OLED_DisplayBPM(0,33,95,37,MeasuringPic[6]);
					index = index + 1;
					break;
				case 8 :
					OLED_DisplayBPM(0,33,95,37,MeasuringPic[5]);
					index = index + 1;
					break;
				case 9 :
					OLED_DisplayBPM(0,33,95,37,MeasuringPic[4]);
					index = index + 1;
					break;
				case 10 :
					OLED_DisplayBPM(0,33,95,37,MeasuringPic[3]);
					index = index + 1;
					break;
				case 11 :
					OLED_DisplayBPM(0,33,95,37,MeasuringPic[2]);
					index = index + 1;
					break;
				case 12 :
					OLED_DisplayBPM(0,33,95,37,MeasuringPic[1]);
					index = 0;
					break;
				default: index = 0;break;
			}
		}
		else   //����SpO2ģʽ��
		{
			Stop_Timer_Cnt(TIM);
			Timer_Free(TIM);
			HR_SpO2PicTIMID = TIMER_ERROR;
			index = 0;
		}
}
/*******************************************************************************
* Function Name  : OLED_DisplayHRSpO2MeasureFlash
* Description    : ����HR/SpO2�����н���
* Input          : status:ON/OFF
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_DisplayHRSpO2MeasureFlash(uint8_t status)
{
		TIM_Basic_Cfg_Typedef   Tim_Cfg_OLED;       // timer���ã����ڿ��ƽ����л�ʱ��
		TIM_Cfg_Typedef         Tim_Cfg_OLED_Index;
    
		/* ���ö�ʱ��*/
		Tim_Cfg_OLED.enuTimerType = TIM_TYPE_MS;
		Tim_Cfg_OLED.u16TimePeriod = MeasuringFlashTIMPeriod;
		Tim_Cfg_OLED.NVIC_IRQChannelPreemptionPriority = OLED_TIMPreemptionPriority;
		Tim_Cfg_OLED.pIntCallBack = HR_SpO2Measuring_Flash_Display;

		/* Init timer top define */
		Tim_Cfg_OLED_Index.TimerMode        = TIM_MODE_BASIC;
		Tim_Cfg_OLED_Index.TimerBasicCfg    = &Tim_Cfg_OLED;
		Tim_Cfg_OLED_Index.TimerPWMCfg 	   = NULL;

		if((status == ON) && (Device_Mode == Device_Mode_SPO2_HR) && (gSubFunc_Stat_Get(SPO2_HR_SINGLEWORK_STATE) != OFF))  //SpO2���豸ģʽ�²Ż���ʾ
		{
			OLED_DisplayFullScreenBMP(HR_SpO2MeasuringPic);
            if(HR_SpO2PicTIMID != TIMER_ERROR)   //�ѷ���Timer
            {				
                Timer_Free(HR_SpO2PicTIMID);    //�������������ʱ��
                HR_SpO2PicTIMID = TIMER_ERROR;
            }             
			HR_SpO2PicTIMID = Timer_Allocate(&Tim_Cfg_OLED_Index);
			Start_Timer_Cnt(HR_SpO2PicTIMID);
			gHR_SpO2DisplayMeasureStatus = ON;
		}
		else
		{
			Timer_Free(HR_SpO2PicTIMID);
			HR_SpO2PicTIMID = TIMER_ERROR;
			gHR_SpO2DisplayMeasureStatus = OFF;
		}
}
/*******************************************************************************
* Function Name  : GetHR_SpO2DisplayMeasureStatus
* Description    : ��ȡHR_SpO2��ʾ�����ж���״̬
* Input          : None
* Output         : None
* Return         : ON / OFF
*******************************************************************************/
uint8_t GetHR_SpO2DisplayMeasureStatus(void)
{
		return gHR_SpO2DisplayMeasureStatus;
}
/*******************************************************************************
* Function Name  : BATCharging_Flash_Display
* Description    : ����н��涯����ʾ
* Input          : status:ON/OFF
* Output         : None
* Return         : None
*******************************************************************************/
static void BATCharging_Flash_Display(Timer_ID_Typedef TIM)
{
		static  uint8_t  index=0;
		TIM = TIM;

		if((Device_Mode == Device_Mode_Charge) && (GetBatChargeState() == true) && (GetBatFullChargeState() == false))
		{
			switch(index)
			{
				case 0 :
                    #ifdef UI_CHINESE_ENABLE           //������UI 
                        OLED_DisplayBatteryChargrLevelICON(0); //��ʾ��ص���
                    #else
                        OLED_DisplayFullScreenBMP(BATChargingFlashPic[0]);
                    #endif
					index = index + 1;
					break;
				case 1 :
                    #ifdef UI_CHINESE_ENABLE           //������UI 
                        OLED_DisplayBatteryChargrLevelICON(1); //��ʾ��ص���
                    #else
                        OLED_DisplayFullScreenBMP(BATChargingFlashPic[1]);
                    #endif
					index = index + 1;
					break;
				case 2 :
                    #ifdef UI_CHINESE_ENABLE           //������UI 
                        OLED_DisplayBatteryChargrLevelICON(2); //��ʾ��ص���
                    #else
                        OLED_DisplayFullScreenBMP(BATChargingFlashPic[2]);
                    #endif
					index = index + 1;
					break;
				case 3 :
                    #ifdef UI_CHINESE_ENABLE           //������UI 
                        OLED_DisplayBatteryChargrLevelICON(3); //��ʾ��ص���
                    #else
                        OLED_DisplayFullScreenBMP(BATChargingFlashPic[3]);
                    #endif
					index = index + 1;
					break;
				case 4 :
                    #ifdef UI_CHINESE_ENABLE           //������UI 
                        OLED_DisplayBatteryChargrLevelICON(4); //��ʾ��ص���
                    #else
                        OLED_DisplayFullScreenBMP(BATChargingFlashPic[4]);
                    #endif
					index = GetBatScaleLevel();
					if(index >= 4)
					{
						index = index-1;
					}
					break;
				default: index = 0;break;
			}
		}			
		else
		{
			Timer_Free(BATChargingTIMID);
			BATChargingTIMID = TIMER_ERROR;
			index = 0;
		}
}
/*******************************************************************************
* Function Name  : OLED_DisplayBATCharingFlash
* Description    : ��綯̬��ʾ����
* Input          : status:ON/OFF
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_DisplayBATCharingFlash(uint8_t status)
{
		TIM_Basic_Cfg_Typedef   Tim_Cfg_OLED;       // timer���ã����ڿ��ƽ����л�ʱ��
		TIM_Cfg_Typedef         Tim_Cfg_OLED_Index;
    
		/* ���ö�ʱ��*/
		Tim_Cfg_OLED.enuTimerType = TIM_TYPE_MS;
		Tim_Cfg_OLED.u16TimePeriod = BATChargingFlashTIMPeriod;
		Tim_Cfg_OLED.NVIC_IRQChannelPreemptionPriority = OLED_TIMPreemptionPriority;
		Tim_Cfg_OLED.pIntCallBack = BATCharging_Flash_Display;

		/* Init timer top define */
		Tim_Cfg_OLED_Index.TimerMode        = TIM_MODE_BASIC;
		Tim_Cfg_OLED_Index.TimerBasicCfg    = &Tim_Cfg_OLED;
		Tim_Cfg_OLED_Index.TimerPWMCfg 	   = NULL;

		if((status == ON) && (Device_Mode == Device_Mode_Charge))  //���ģʽ�²Ż���ʾ
		{
            #ifdef UI_CHINESE_ENABLE           //������UI
                Set_OLED_Dis_Status(OLEDDisON);
                OLED_DisplayFullScreenBMP(Charging);   //UI������ʹ��
            #endif
            if(BATChargingTIMID != TIMER_ERROR)   //�ѷ���Timer
            {				
                Timer_Free(BATChargingTIMID);    //�������������ʱ��
                BATChargingTIMID = TIMER_ERROR;
            }               
			if(BATChargingTIMID == TIMER_ERROR)
			{
				BATChargingTIMID = Timer_Allocate(&Tim_Cfg_OLED_Index);
				Start_Timer_Cnt(BATChargingTIMID);	
			}				
		}
		else
		{
			Timer_Free(BATChargingTIMID);
			BATChargingTIMID = TIMER_ERROR;
			OLED_DisplayClear();
		}
}
/*******************************************************************************
* Function Name  : NotEnoughSpace_Flash_Display
* Description    : �ռ䲻��澯��ʾ����
* Input          : TIM
* Output         : None
* Return         : None
*******************************************************************************/
static void NotEnoughSpace_Flash_Display(Timer_ID_Typedef TIM)
{
		static uint8_t Ctl=0;
	
		if(Ctl == 0)
		{
			Ctl = 1;
			OLED_DisplayICON(ICON_NotEnoughSpace);
			Delay_ms(200);
		}
		else
		{
			Ctl = 0;
			Timer_Free(OLEDDisNotEnoughSpaceTIMID);
			OLEDDisNotEnoughSpaceTIMID = TIMER_ERROR;
			Set_OLED_Dis_Status(OLEDDisON);
			TS_SendEvent(gOledDisTaskID,gOledDisEventDisplayAllRAM_c);  //����RAM������ʾֵ��Ļ
		}
}
/*******************************************************************************
* Function Name  : OLED_DisplayNoEnoughSpaceFlash
* Description    : �ռ䲻��澯��ʾ����
* Input          : status:ON/OFF
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_DisplayNoEnoughSpaceFlash(uint8_t status)
{
		TIM_Basic_Cfg_Typedef   Tim_Cfg_OLED;       // timer���ã����ڿ��ƽ����л�ʱ��
		TIM_Cfg_Typedef         Tim_Cfg_OLED_Index;
    
		/* ���ö�ʱ��*/
		Tim_Cfg_OLED.enuTimerType = TIM_TYPE_MS;
		Tim_Cfg_OLED.u16TimePeriod = NotEnoughSpaceFlashTIMPeriod;
		Tim_Cfg_OLED.NVIC_IRQChannelPreemptionPriority = OLED_TIMPreemptionPriority;
		Tim_Cfg_OLED.pIntCallBack = NotEnoughSpace_Flash_Display;

		/* Init timer top define */
		Tim_Cfg_OLED_Index.TimerMode        = TIM_MODE_BASIC;
		Tim_Cfg_OLED_Index.TimerBasicCfg    = &Tim_Cfg_OLED;
		Tim_Cfg_OLED_Index.TimerPWMCfg 	   = NULL;

		if(status == ON)
		{
            if(OLEDDisNotEnoughSpaceTIMID != TIMER_ERROR)   //�ѷ���Timer
            {				
                Timer_Free(OLEDDisNotEnoughSpaceTIMID);    //�������������ʱ��
                OLEDDisNotEnoughSpaceTIMID = TIMER_ERROR;
            }             
			if(OLEDDisNotEnoughSpaceTIMID == TIMER_ERROR)
			{
				OLEDDisNotEnoughSpaceTIMID = Timer_Allocate(&Tim_Cfg_OLED_Index);
				Start_Timer_Cnt(OLEDDisNotEnoughSpaceTIMID);	
			}				
		}
		else
		{
			Timer_Free(OLEDDisNotEnoughSpaceTIMID);
			OLEDDisNotEnoughSpaceTIMID = TIMER_ERROR;
			Set_OLED_Dis_Status(OLEDDisON);
			TS_SendEvent(gOledDisTaskID,gOledDisEventDisplayAllRAM_c);  //����RAM������ʾֵ��Ļ
		}
}

/*******************************************************************************
* Function Name  : IncomingCall_Flash_Display
* Description    : ����������ʾ����
* Input          : TIM
* Output         : None
* Return         : None
*******************************************************************************/
static void IncomingCall_Flash_Display(Timer_ID_Typedef TIM)
{
		static uint8_t Ctl=0;
	
		if(Device_State == Device_WorkWithAPP)  //Connect to App
		{
			if(Ctl == 0)
			{
				Ctl = 1;
				Set_OLED_Dis_Status(OLEDDisON);
				OLED_DisplayIncomingCall(Calling);
			}
			else
			{
				Ctl = 0;
				
				OLED_DisplayClearWithOutRAM();
			}
		}
		else   //Disconnect from App
		{
			TS_SendEvent(gTsIncomingCallTaskID_c,gCallIncomingCallRemoved);
		}
}
/*******************************************************************************
* Function Name  : OLED_DisplayInconmingCallFlash
* Description    : ������ʾ��ʾ����
* Input          : status:ON/OFF
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_DisplayInconmingCallFlash(uint8_t status)
{
		TIM_Basic_Cfg_Typedef   Tim_Cfg_OLED;       // timer���ã����ڿ��ƽ����л�ʱ��
		TIM_Cfg_Typedef         Tim_Cfg_OLED_Index;
    
		/* ���ö�ʱ��*/
		Tim_Cfg_OLED.enuTimerType = TIM_TYPE_MS;
		Tim_Cfg_OLED.u16TimePeriod = IncomingCallFlashTIMPeriod;
		Tim_Cfg_OLED.NVIC_IRQChannelPreemptionPriority = OLED_TIMPreemptionPriority;
		Tim_Cfg_OLED.pIntCallBack = IncomingCall_Flash_Display;

		/* Init timer top define */
		Tim_Cfg_OLED_Index.TimerMode        = TIM_MODE_BASIC;
		Tim_Cfg_OLED_Index.TimerBasicCfg    = &Tim_Cfg_OLED;
		Tim_Cfg_OLED_Index.TimerPWMCfg 	   = NULL;

		if(status == ON)
		{
            if(OLEDDisIncommingCallTIMID != TIMER_ERROR)   //�ѷ���Timer
            {				
                Timer_Free(OLEDDisIncommingCallTIMID);    //�������������ʱ��
                OLEDDisIncommingCallTIMID = TIMER_ERROR;
            }             
			if(OLEDDisIncommingCallTIMID == TIMER_ERROR)
			{
				OLEDDisIncommingCallTIMID = Timer_Allocate(&Tim_Cfg_OLED_Index);
				Start_Timer_Cnt(OLEDDisIncommingCallTIMID);	
				Set_OLED_Dis_Status(OLEDDisON);
				OLED_DisplayIncomingCall(Calling);
			}				
		}
		else
		{
			Timer_Free(OLEDDisIncommingCallTIMID);
			OLEDDisIncommingCallTIMID = TIMER_ERROR;
			Set_OLED_Dis_Status(OLEDDisON);
			TS_SendEvent(gOledDisTaskID,gOledDisEventDisplayAllRAM_c);  //����RAM������ʾֵ��Ļ
		}
}

/*******************************************************************************
* Function Name  : OLED_DisplayFullScreenBMP
* Description    : ȫ����ʾBMPͼƬ
* Input          : BMPͼƬ����������Ϊȫ��ͼƬ����
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_DisplayFullScreenBMP(const unsigned char * pitrueData)
{
		uint8_t  i = 0;
		uint8_t  n = 0;
	
		if(OFF == GetPeriphPowerStatus())  //�����ⲿ��Դ
		{
			PeriPower(ON);
		}
		if(Get_OLED_Config_Status() == DISABLE)
		{
			OLED_Configuration();
		}
		
		
		MXS8475_WriteReg(0x02,0x00);
		MXS8475_WriteReg(0x1D,0x80);	

		MXS8475_WriteReg(0x34,0);			  			 	  
		MXS8475_WriteReg(0x35,11);	
		MXS8475_WriteReg(0x36,0);					
		MXS8475_WriteReg(0x37,38);		

		MXS8475_WriteCMD(0x08);	

		for(i=0;i<39;i++)  
		{  
			for(n=0;n<12;n++)
			{
				s_ucGRAM[i][n] = *pitrueData;
				MXS8475_WriteDat(s_ucGRAM[i][n]);
				pitrueData++;				
			}
		}
			
		MXS8475_WriteReg(0x02,0x01);	
}
/*******************************************************************************
* Function Name  : OLED_DisplayOccupyFullScreenBMP
* Description    : ȫ����ռ��ʾBMPͼƬ
* Input          : BMPͼƬ����������Ϊȫ��ͼƬ����
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_DisplayOccupyFullScreenBMP(const unsigned char * pitrueData)
{
		uint16_t  i = 0;
	
		if(OFF == GetPeriphPowerStatus())  //�����ⲿ��Դ
		{
			PeriPower(ON);
		}
		if(Get_OLED_Config_Status() == DISABLE)
		{
			OLED_Configuration();
		}
	
		OLEDDisplay_Stat_Set(OccupiedDisMove);   //��ռ��ʾ�����־�ֹ��
		if(NormalDis != OLEDDisplay_Stat_Get())   //��ռģʽ����ʾ
		{
			MXS8475_WriteReg(0x02,0x00);
			MXS8475_WriteReg(0x1D,0x80);	

			MXS8475_WriteReg(0x34,0);			  			 	  
			MXS8475_WriteReg(0x35,11);	
			MXS8475_WriteReg(0x36,0);					
			MXS8475_WriteReg(0x37,38);		

			MXS8475_WriteCMD(0x08);	
		
			for(i = 0;i < 468;i++)
			{
				MXS8475_WriteDat(*pitrueData);
				pitrueData++;
			}
			MXS8475_WriteReg(0x02,0x01);	
		}
}
/*******************************************************************************
* Function Name  : OLED_DisplayBPM
* Description    : ��ָ��������������ʾBMPͼƬ
* Input          : xS,xE,yS,yE,pitrueData
* Output         : None
* Return         : None
*******************************************************************************/
void OLED_DisplayBPM(uint8_t _xS,uint8_t _yS,uint8_t _xE,uint8_t _yE,const unsigned char * pitrueData)
{
	uint8_t temp,t,t1,t2;
	uint8_t mode=1;
	uint8_t x=_xS;
	uint8_t y=_yS;
	

	uint8_t Width;
	uint8_t Len;
	uint8_t WidthInt;
	uint8_t WidthInt1;
	uint8_t WidthRem;
	
	Len = _yE - _yS;
	Width = _xE - _xS;
	
	WidthInt = Width / 8;
	WidthRem = Width % 8;
	
	if(WidthRem == 0)
	{
		for(t=0;t<Len;t++)
		{
			for(t2=0;t2<WidthInt;t2++)
			{
				temp = pitrueData[t*WidthInt+t2];
			
				for(t1=0;t1<8;t1++)
				{
					if(temp&0x80)
					{
						OLED_DrawPoint(x,y,mode);
					}
					else 
					{
						OLED_DrawPoint(x,y,!mode);
					}
					temp<<=1;
					x++;
				}
			}	
			x=_xS;
			y++;
		}   
	}
	else
	{
		WidthInt1 = WidthInt+1;  //���������һ���ֽ�
		for(t=0;t<Len;t++)
		{
			for(t2=0;t2<WidthInt1;t2++)
			{
				temp = pitrueData[t*WidthInt1+t2];
				if(t2<WidthInt)
				{
					for(t1=0;t1<8;t1++)
					{
						if(temp&0x80)
						{
							OLED_DrawPoint(x,y,mode);
						}
						else 
						{
							OLED_DrawPoint(x,y,!mode);
						}
						temp<<=1;
						x++;
					}
				}
				else
				{
					for(t1=0;t1<WidthRem;t1++)
					{
						if(temp&0x80)
						{
							OLED_DrawPoint(x,y,mode);
						}
						else 
						{
							OLED_DrawPoint(x,y,!mode);
						}
						temp<<=1;
						x++;
					}	
				}
			}	
			x=_xS;
			y++;
		}   
	}
	OLED_Refresh_Gram(_xS,_xE,_yS,_yE);
}

void OLED_DisplayIncomingCall(const uint8_t * pitrueData)
{
	uint16_t  i = 0;

	if(OFF == GetPeriphPowerStatus())  //�����ⲿ��Դ
	{
		PeriPower(ON);
	}
	if(Get_OLED_Config_Status() == DISABLE)
	{
		OLED_Configuration();
	}

	OLEDDisplay_Stat_Set(OccupiedDisCall);   //��ռ��ʾ�����硱
	if(NormalDis != OLEDDisplay_Stat_Get())   //��ռģʽ����ʾ
	{
		MXS8475_WriteReg(0x02,0x00);
		MXS8475_WriteReg(0x1D,0x80);	

		MXS8475_WriteReg(0x34,0);			  			 	  
		MXS8475_WriteReg(0x35,11);	
		MXS8475_WriteReg(0x36,0);					
		MXS8475_WriteReg(0x37,38);		

		MXS8475_WriteCMD(0x08);	
	
		for(i = 0;i < 468;i++)
		{
			MXS8475_WriteDat(*pitrueData);
			pitrueData++;
		}
		MXS8475_WriteReg(0x02,0x01);	
	}
}  

/********************** BOARD_REDHARE_V3_0   GUI�ӿں���  ����********************************************************/




