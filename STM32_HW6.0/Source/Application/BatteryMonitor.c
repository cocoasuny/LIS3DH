#include "BatteryMonitor.h"
#include "common.h"
#include "platform.h"

#define ADC_CONV_BUFF_SIZE              6

volatile FlagStatus flag_ADCDMA_TransferComplete;
//__IO uint16_t 	ADC_ConvertedValue, T_StartupTimeDelay;
static uint8_t batteryFullCharge_cnt = 0;
static uint8_t batterySample_cnt = 0;
static uint16_t ADC_ConvertedValueBuff[ADC_CONV_BUFF_SIZE] = {0};
static BatLevel_t batteryLevel;
static DMA_InitTypeDef DMA_InitStructure;
BatVoltage_t batteryVoltage;


void acquireVoltageValue(void);
void clearADCDMA_TransferComplete(void);

static float32_t CalRealVoltage(void);

/*******************************************************************************
* @brief   Configures the GPIO for ADC1
* @param   void
* @retval  void
*******************************************************************************/
void BATVolDetectInit(void)
{
    batteryLevel.fBat_shutdown_level    = 3.2;
    batteryLevel.fBat_low_alarm_level   = 3.4;
    batteryLevel.fBat_0_grid_level      = 3.50;
    batteryLevel.fBat_1_grid_level      = 3.60;
    batteryLevel.fBat_2_grid_level      = 3.70;
	batteryLevel.fBat_3_grid_level      = 3.85;
	batteryLevel.fBat_full_charge_level = 4.05;
}

/*******************************************************************************
* @brief   Configures the GPIO for ADC1
* @param   void
* @retval  void
*******************************************************************************/
void configureGPIO_ADC(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
   
    /* Enable GPIOs clock */ 	
    RCC_AHBPeriphClockCmd(RCC_ADC_PORT_BATMONITOR, ENABLE);
    
    /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
    GPIO_InitStructure.GPIO_Pin = ADC_PORT_PIN_BATMONITOR;   
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(ADC_PORT_BATMONITOR, &GPIO_InitStructure);
}
/*******************************************************************************
* @brief   Configures the GPIO for ADC1
* @param   void
* @retval  void
*******************************************************************************/
void DeconfigureGPIO_ADC(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
   
    /* Enable GPIOs clock */ 	
    RCC_AHBPeriphClockCmd(RCC_ADC_PORT_BATMONITOR, ENABLE);
    
    /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
    GPIO_InitStructure.GPIO_Pin = ADC_PORT_PIN_BATMONITOR;   
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_Init(ADC_PORT_BATMONITOR, &GPIO_InitStructure);
}
/*******************************************************************************
* @brief   Configures the DMA for ADC1
* @param   void
* @retval  void
*******************************************************************************/
void configureDMA_ADC(void)
{
    /* Declare NVIC init Structure */
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable DMA1 clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* De-initialise  DMA */
    DMA_DeInit(DMA1_Channel1);

    /* DMA1 channel1 configuration */
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);	        // Set DMA channel Peripheral base address to ADC Data register
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_ConvertedValueBuff;   // Set DMA channel Memeory base addr to ADC_ConvertedValueBuff address
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                          // Set DMA channel direction to peripheral to memory
    DMA_InitStructure.DMA_BufferSize = ADC_CONV_BUFF_SIZE;                      // Set DMA channel buffersize to peripheral to ADC_CONV_BUFF_SIZE
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	        // Disable DMA channel Peripheral address auto increment
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     // Enable Memeory increment (To be verified ....)
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // set Peripheral data size to 8bit 
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;	        // set Memeory data size to 8bit 
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                               // Set DMA in normal mode
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;	                        // Set DMA channel priority to High
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                // Disable memory to memory option 
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);                                // Use Init structure to initialise channel1 (channel linked to ADC)

    /* Enable Transmit Complete Interrup for DMA channel 1 */ 
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

    /* Setup NVIC for DMA channel 1 interrupt request */
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = BatterySamplePreemptionPriority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = BatterySampleSubPriority;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* @brief   Configures the ADC module
* @param   void
* @retval  void
*******************************************************************************/
void configureADC_BAT(void)
{
    uint32_t ch_index = 0;
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	
    /* Enable ADC clock & SYSCFG */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    /* Enable the internal connection of Temperature sensor and with the ADC channels*/
    ADC_TempSensorVrefintCmd(ENABLE); 

    /* Wait until ADC + Temp sensor start */
	Delay_us(5);
    
    /* Setup ADC common init struct */
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInit(&ADC_CommonInitStructure);

    /* Initialise the ADC1 by using its init structure */
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;	                // Set conversion resolution to 12bit
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;                            // Enable Scan mode (single conversion for each channel of the group)
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;			            // Disable Continuous conversion
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None; // Disable external conversion trigger
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                  // Set conversion data alignement to right
    ADC_InitStructure.ADC_NbrOfConversion = ADC_CONV_BUFF_SIZE;             // Set conversion data alignement to ADC_CONV_BUFF_SIZE
    ADC_Init(ADC1, &ADC_InitStructure);

    /* ADC1 regular from channel1 to channel5 and internal reference channel17 configuration */ 
    for (ch_index = 1; ch_index < ADC_CONV_BUFF_SIZE; ch_index++)
    {
        ADC_RegularChannelConfig(ADC1, ADC_Channel_BATMONITOR, ch_index, ADC_SampleTime_384Cycles);
    }
    ADC_RegularChannelConfig(ADC1, ADC_Channel_17, ADC_CONV_BUFF_SIZE, ADC_SampleTime_384Cycles);   // Internal reference voltage
}

/*******************************************************************************
* @brief   Disables the DMA module and clock for ADC to power down
* @param   void
* @retval  void
*******************************************************************************/
void powerDownADC(void)
{
    /* Disable DMA channel1 */
    DMA_Cmd(DMA1_Channel1, DISABLE);  
    /* Disable ADC1 */
    ADC_Cmd(ADC1, DISABLE);

    /* Disable ADC1 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);  
    /* Disable DMA1 clock */
    //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, DISABLE);
	#ifdef Battery_Debug
		printf("batt DMA disable \r\n");
	#endif 
}

/*******************************************************************************
* @brief   Calculates the real voltage
* @param   void
* @retval  void
*******************************************************************************/
static float32_t CalRealVoltage(void)
{
    uint32_t temp  = 0,i;
    float32_t volADCvalue = 0;
    
    for(i = 0;i < (ADC_CONV_BUFF_SIZE - 1);i ++)
    {
        temp += *(ADC_ConvertedValueBuff + i);
    }
    
    volADCvalue = ((temp * 1.2 * 2) /((*(ADC_ConvertedValueBuff + 5))*5));
    
    return volADCvalue;
}

/*******************************************************************************
* @brief   Configures the GPIO for ADC1
* @param   void
* @retval  void
*******************************************************************************/
float ADC_SampleAndConv(ADC_TypeDef* ADCx)
{
	uint32_t TimeCnt=0;
	
	#ifdef Battery_Debug
		printf("batt require start \r\n");
	#endif 
    /* Re-enable DMA and ADC conf and start Temperature Data acquisition */ 
    acquireVoltageValue();
	
	TimeCnt = 160000;
    /* for DEBUG purpose uncomment the following line and comment the __WFI call to do not enter STOP mode */
    while ((!flag_ADCDMA_TransferComplete) && (TimeCnt !=0))
	{
			TimeCnt--;
	}

    /* Clear global flag for DMA transfert complete */
    clearADCDMA_TransferComplete(); 
	
	#ifdef Battery_Debug
		printf("batt require stop \r\n");
	#endif 
	
    powerDownADC();
    
    DeconfigureGPIO_ADC();

    return (CalRealVoltage());
}

void setADCDMA_TransferComplete(void)
{
    flag_ADCDMA_TransferComplete = SET;
}

void clearADCDMA_TransferComplete(void)
{
    flag_ADCDMA_TransferComplete = RESET;
}

void acquireVoltageValue(void)
{
	uint32_t TimeCnt=0;
    /* Enable ADC clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    /* Enable DMA1 clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);

	TimeCnt = 6000;
    /* Wait until the ADC1 is ready */
    while((ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET) && (TimeCnt != 0))
	{
		TimeCnt--;
	}		

    /* re-initialize DMA -- is it needed ?*/
    DMA_DeInit(DMA1_Channel1);
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel1, ENABLE);
  
    /* Enable DMA channel 1 Transmit complete interrupt*/
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

    /* Disable DMA mode for ADC1 */ 
    ADC_DMACmd(ADC1, DISABLE);

    /* Enable DMA mode for ADC1 */  
    ADC_DMACmd(ADC1, ENABLE);

    /* Clear global flag for DMA transfert complete */
    clearADCDMA_TransferComplete(); 

    /* Start ADC conversion */
    ADC_SoftwareStartConv(ADC1);
}
void BATVoltageDetect(void)
{
	/* Disable charge for improving the ADC value */
        if(batteryVoltage.bExternPowerAvailable) 
        {
            GPIO_SetBits(GPIO_BAT_CE,GPIO_Pin_BAT_CE);
        }
		#ifdef Battery_Debug
			printf("batt vol detect start \r\n");
		#endif 
        configureGPIO_ADC();
        configureDMA_ADC();
        configureADC_BAT();
        
        batteryVoltage.fVoltage = ADC_SampleAndConv(ADC1);

        /* Enables charge */
        if(batteryVoltage.bExternPowerAvailable) 
        {
            GPIO_ResetBits(GPIO_BAT_CE,GPIO_Pin_BAT_CE);
        }
        if(batteryVoltage.fVoltage >= batteryLevel.fBat_full_charge_level)
		{
			batteryVoltage.u8CurrentLevel = 4;
		}
		else if(batteryVoltage.fVoltage >= batteryLevel.fBat_3_grid_level)
        {
            batteryVoltage.u8CurrentLevel = 3;
        }
        else if(batteryVoltage.fVoltage >= batteryLevel.fBat_2_grid_level) 
        {
            batteryVoltage.u8CurrentLevel = 2;
        }
        else if(batteryVoltage.fVoltage >= batteryLevel.fBat_1_grid_level) 
        {
            batteryVoltage.u8CurrentLevel = 1;
        }
        else
        {
            batteryVoltage.u8CurrentLevel = 0;
        }
		#ifdef Battery_Debug
			printf("The battery voltage is %f v,the level is %d.\r\n",batteryVoltage.fVoltage,batteryVoltage.u8CurrentLevel);
		#endif 
}
void VoltageDetect_Task_Handler(event_t voltDetect_Event)
{
    static uint8_t pre_BatteryLevel = 5;
    
    if(Device_Mode != Device_Mode_FWUpdate)
    {
        if((voltDetect_Event & gBATVoltageDetectEventStart))
        {
            #ifdef Battery_Debug
                printf("batt event start \r\n");
            #endif 
            /* Disable charge for improving the ADC value */
            if(batteryVoltage.bExternPowerAvailable) 
            {
                GPIO_SetBits(GPIO_BAT_CE,GPIO_Pin_BAT_CE);
            }
            
            configureGPIO_ADC();
            configureDMA_ADC();
            configureADC_BAT();
            
            batteryVoltage.fVoltage = ADC_SampleAndConv(ADC1);

            /* Enables charge */
            if(batteryVoltage.bExternPowerAvailable) 
            {
                GPIO_ResetBits(GPIO_BAT_CE,GPIO_Pin_BAT_CE);
            }
            if(batteryVoltage.fVoltage >= batteryLevel.fBat_full_charge_level)
            {
                batteryVoltage.u8CurrentLevel = 4;
            }
            else if(batteryVoltage.fVoltage >= batteryLevel.fBat_3_grid_level)
            {
                batteryVoltage.u8CurrentLevel = 3;
            }
            else if(batteryVoltage.fVoltage >= batteryLevel.fBat_2_grid_level) 
            {
                batteryVoltage.u8CurrentLevel = 2;
            }
            else if(batteryVoltage.fVoltage >= batteryLevel.fBat_1_grid_level) 
            {
                batteryVoltage.u8CurrentLevel = 1;
            }
            else
            {
                batteryVoltage.u8CurrentLevel = 0;
            }
            
            if(!batteryVoltage.bExternPowerAvailable) 
            {
                /* Abandons the level data the first time.*/
                if(batterySample_cnt)
                {  
                    if(batteryVoltage.u8CurrentLevel > batteryVoltage.u8LastLevel)
                    {
                        batteryVoltage.u8CurrentLevel = batteryVoltage.u8LastLevel;
                    }
                }
            }
            batteryVoltage.u8LastLevel = batteryVoltage.u8CurrentLevel;
            if(batteryVoltage.fVoltage < batteryLevel.fBat_shutdown_level)
            {
                TS_SendEvent(gTsBatManageTaskID_c,gBATBadEvent);
            }
            else if(batteryVoltage.fVoltage < batteryLevel.fBat_low_alarm_level)
            {
                ExtFLASH_SaveRdAddrToConst();
                TS_SendEvent(gTsBatManageTaskID_c,gBATLowEvent);
            }
            
            if(batteryVoltage.fVoltage >= batteryLevel.fBat_full_charge_level)
            {   
                if(batteryVoltage.bExternPowerAvailable)
                {  
                    batteryFullCharge_cnt ++;
    //                if(batteryFullCharge_cnt > 3)
    //                {
    //                    batteryFullCharge_cnt = 0;
    //					if(!batteryVoltage.bFullScale)
    //					{
    //						batteryVoltage.bFullScale = true;
    //                        if(GPIO_ReadInputDataBit(GPIO_BAT_CHG,EXTI_LineBAT_CHG))
    //                        {
    //                            TS_SendEvent(gTsBatManageTaskID_c,gBATFullChargeEvent);
    //                        }
    //					}
    //                }
                    /* 	Change to get a stable full change flag 		*/
                    if((batteryFullCharge_cnt > CHARGE_FULL_TIMEOUT_WITH_CHG && GPIO_ReadInputDataBit(GPIO_BAT_CHG,EXTI_LineBAT_CHG) == 1u)
                        || (batteryFullCharge_cnt > CHARGE_FULL_TIMEOUT_WITHOUT_CHG))
                    {
                        batteryFullCharge_cnt = 0;
                        if(!batteryVoltage.bFullScale)
                        {
                            batteryVoltage.bFullScale = true;
                            TS_SendEvent(gTsBatManageTaskID_c,gBATFullChargeEvent);
                        }
                    }
                }
            }
            else
            {
                batteryVoltage.bFullScale = false;
                batteryVoltage.eBatteryAlarmState = BattStateNormal;
            }            
            
            /* only increments the first time.*/
            if(!batterySample_cnt)
            {
                batterySample_cnt ++;
            }
            
            /* SPI 更新电池电量等级 */
            /* Update Battery Level when changed */
            if(pre_BatteryLevel != GetBatScaleLevel())
            {
                TS_SendEvent(gTsSPITranslateTaskID_c,gSPITranslateEventTxBatteryLevel);
                pre_BatteryLevel = GetBatScaleLevel();
            }
    #ifdef Battery_Debug
            printf("The battery voltage is %f v,the level is %d.\r\n",batteryVoltage.fVoltage,batteryVoltage.u8CurrentLevel);
    #endif 
        }
    }
}

/*******************************************************************************
* @brief   Get the voltage-level of the battery
* @param   void
* @retval  voltage-level
*******************************************************************************/
uint8_t GetBatScaleLevel(void)
{
    return (batteryVoltage.u8CurrentLevel);
}

/*******************************************************************************
* @brief   Check if the battery is full charge
* @param   void
* @retval  true---Full charge;false---No full
*******************************************************************************/
bool GetBatFullChargeState(void)
{
    return (batteryVoltage.bFullScale);
}

/*******************************************************************************
* @brief   Get the alarm state 
* @param   void
* @retval  As follows:
*                BattStateNormal---No alarm;
*                BattStateLow   ---Low voltage alarm,need to tell users to charge
*                BattStateOut   ---The battery is bad and need to tell the user
*******************************************************************************/
emBatteryAlarmState GetBatteryAlarmState(void)
{
    return (batteryVoltage.eBatteryAlarmState);
}

/*******************************************************************************
* @brief   Check if the external power is plugged into the charge connector
* @param   void
* @retval  bool,true or false
*******************************************************************************/
bool GetBatChargeState(void)
{
    return (batteryVoltage.bExternPowerAvailable);
}
