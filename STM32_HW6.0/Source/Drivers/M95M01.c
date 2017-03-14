#include "m95m01.h"
#include "stm32l1xx.h"
#include "SPI.h"
#include "common.h"

m95m01_info_t m95m01_info;

void M95m01_CounterCheckOut(void);
void M95m01_CounterUpdate(void);
void m95m01ReadOK(uint8_t data_id,uint8_t len,uint8_t state);
/*******************************************************************************
* Function Name  : SPI_Configuration
* Description    : 系统SPI初始，用于与51822之间通讯及读写EEPROM
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EEPROM_SPI_Configuration(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;
    SPI_InitTypeDef     SPI_InitStructure;

    /* Enable the SPI peripheral */
    RCC_APB1PeriphClockCmd(RCC_APBPeriph_SysSPI, ENABLE);

    /* Enable SPI SCK, MOSI, MISO and NSS GPIO PB12 PB13 PB14 PB15 clocks */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_SysSPIGPIO,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_SysSPINSSGPIO,ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);          // Adds for EEPROM

    /* SPI pin mappings */
    GPIO_PinAFConfig(GPIO_SysSPI, GPIO_PinSourceSysSPI_MOSI, GPIO_AF_SysSPI);
    GPIO_PinAFConfig(GPIO_SysSPI, GPIO_PinSourceSysSPI_MISO, GPIO_AF_SysSPI);
    GPIO_PinAFConfig(GPIO_SysSPI, GPIO_PinSourceSysSPI_SCK, GPIO_AF_SysSPI);

    /*  SPI pin config */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;

    /* SPI SCK pin configuration */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_SysSCK;
    GPIO_Init(GPIO_SysSPI, &GPIO_InitStructure);

    /* SPI  MOSI pin configuration */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_SysMOSI;
    GPIO_Init(GPIO_SysSPI, &GPIO_InitStructure);

    /* SPI MISO pin configuration */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_SysMISO;
    GPIO_Init(GPIO_SysSPI, &GPIO_InitStructure);
    
    /* SPI NSS_EEPROM pin configuration */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_EEPROM_CS;
    GPIO_Init(GPIO_PORT_EEPROM_CS, &GPIO_InitStructure);
	GPIO_SetBits(GPIO_PORT_EEPROM_CS,GPIO_Pin_EEPROM_CS);

     /* SPI configuration -------------------------------------------------------*/
    SPI_I2S_DeInit(SysSPI);
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;   
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; // (8M/32 =250KHz)
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;

    SPI_Init(SysSPI, &SPI_InitStructure);

	GPIO_SetBits(GPIO_PORT_EEPROM_CS,GPIO_Pin_EEPROM_CS);         // CS = 1
    /* Enable the SPI peripheral */
    SPI_Cmd(SysSPI, DISABLE);
}
/*******************************************************************************
* @brief  Sends a byte through the SPI interface and return the byte
* @param  SPIx: To select the SPIx/I2Sx peripheral
* @param  temp: Data to be transmitted.l
* @retval The value of the received data.
*******************************************************************************/
uint8_t EEPROM_SPI_ReadWriteByte(SPI_TypeDef* SPIx,uint8_t temp)
{
	uint16_t 	u16TimeOut;
	 /* Send byte through the SPIx peripheral */
    SPI_I2S_SendData(SPIx,temp);
	
	/* Loop while DR register in not emplty */
	u16TimeOut = EEPROM_TIMEOUT_CNT;
    while (--u16TimeOut != 0u && SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET);
	
    /* Wait to receive a byte */
	u16TimeOut = EEPROM_TIMEOUT_CNT;
    while (--u16TimeOut != 0u && SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET);
    
    /* Return the byte read from the SPI bus */
    return (SPI_I2S_ReceiveData(SPIx));
}

/*******************************************************************************
* @brief  Read the Status Register of the EEPROM
* @param  void
* @retval The value of the Status Register
*******************************************************************************/
uint8_t EEPROM_SPI_ReadStatusRegister(void)
{
    uint8_t temp = 0;
    
    SPI_Cmd(SysSPI, ENABLE);
    GPIO_ResetBits(GPIO_PORT_EEPROM_CS,GPIO_Pin_EEPROM_CS);         // CS = 0
    
    EEPROM_SPI_ReadWriteByte(SysSPI,M95M01_RDSR);
    temp = EEPROM_SPI_ReadWriteByte(SysSPI,0);
    
    GPIO_SetBits(GPIO_PORT_EEPROM_CS,GPIO_Pin_EEPROM_CS);           // CS = 1
    SPI_Cmd(SysSPI, DISABLE);

    return temp; 
}

/*******************************************************************************
* @brief  Enables the write for the M95M02
* @param  void
* @retval void
*******************************************************************************/
void EEPROM_SPI_WriteEnable(void)
{
    SPI_Cmd(SysSPI, ENABLE);
	GPIO_ResetBits(GPIO_PORT_EEPROM_CS,GPIO_Pin_EEPROM_CS);       // CS = 0
    
    EEPROM_SPI_ReadWriteByte(SysSPI,M95M01_WREN);
    
    GPIO_SetBits(GPIO_PORT_EEPROM_CS,GPIO_Pin_EEPROM_CS);         // CS = 1
	SPI_Cmd(SysSPI, DISABLE);
}

/*******************************************************************************
* @brief  Disable the write for the M95M02
* @param  void
* @retval void
*******************************************************************************/
void EEPROM_SPI_WriteDisable(void)
{
    SPI_Cmd(SysSPI, ENABLE);
	GPIO_ResetBits(GPIO_PORT_EEPROM_CS,GPIO_Pin_EEPROM_CS);       // CS = 0
    
    EEPROM_SPI_ReadWriteByte(SysSPI,M95M01_WRDI);

    GPIO_SetBits(GPIO_PORT_EEPROM_CS,GPIO_Pin_EEPROM_CS);         // CS = 1
    SPI_Cmd(SysSPI, DISABLE);
}

/*******************************************************************************
* @brief   Write the memory data to the EEPROM
* @param   pSrcStr:The source buffer to write
* @param   pStartAddr:The first addr to read in EEPROM
* @param   len:The length to write in EEPROM
* @retval  void
*******************************************************************************/
void EEPROM_SPI_WriteToMemory(uint8_t *pSrcStr,uint32_t pStartAddr,uint16_t len)
{
 	uint16_t u16TimeOut;
    uint16_t index      = 0;
    uint32_t temp_Addr  = pStartAddr;

#ifdef M95M01_Debug
    printf("Now writing to M95m02,pStartAddr = %d,len = %d...\r\n",pStartAddr,len);
#endif
    
	u16TimeOut = EEPROM_TIMEOUT_CNT;
    while (--u16TimeOut != 0u &&(EEPROM_SPI_ReadStatusRegister() & M95M01_WIP_STATUS) == SET);
	
    EEPROM_SPI_WriteEnable();
    
    SPI_Cmd(SysSPI, ENABLE);
    GPIO_ResetBits(GPIO_PORT_EEPROM_CS,GPIO_Pin_EEPROM_CS);       // CS = 0

    EEPROM_SPI_ReadWriteByte(SysSPI,M95M01_WRITE);
    for(index = 0;index < 3;index ++)
    {
        temp_Addr <<= 8;
        EEPROM_SPI_ReadWriteByte(SysSPI,(uint8_t)(temp_Addr >> 24));
    }
    for(index = 0;index < len;index ++)
    {
        EEPROM_SPI_ReadWriteByte(SysSPI,*(pSrcStr + index));
    }
    
    GPIO_SetBits(GPIO_PORT_EEPROM_CS,GPIO_Pin_EEPROM_CS);         // CS = 1 
    SPI_Cmd(SysSPI, DISABLE);
    
    EEPROM_SPI_WriteDisable();
    
	u16TimeOut = EEPROM_TIMEOUT_CNT;
    while (--u16TimeOut != 0u &&(EEPROM_SPI_ReadStatusRegister() & M95M01_WIP_STATUS) == SET);
}

/*******************************************************************************
* @brief   Read the memory form the EEPROM
* @param   pDestStr:The buffer after read out
* @param   pStartAddr:The first addr to read in EEPROM
* @param   len:The length to read in EEPROM
* @retval  void
*******************************************************************************/
void EEPROM_SPI_ReadFromMemory(uint8_t *pDestStr,uint32_t pStartAddr,uint16_t len)
{
    uint16_t index      = 0;
    uint32_t temp_Addr  = pStartAddr;

#ifdef M95M01_Debug
    printf("\r\nNow reading from M95m02,pStartAddr = %d,len = %d...\r\n",pStartAddr,len);
#endif
    
    SPI_Cmd(SysSPI, ENABLE);
	SPI_I2S_ReceiveData(SysSPI);      //[Keke：读取之前清除标志位]
	GPIO_ResetBits(GPIO_PORT_EEPROM_CS,GPIO_Pin_EEPROM_CS);       // CS = 0

    EEPROM_SPI_ReadWriteByte(SysSPI,M95M01_READ);
    for(index = 0;index < 3;index ++)
    {
        temp_Addr <<= 8;
        EEPROM_SPI_ReadWriteByte(SysSPI,(uint8_t)(temp_Addr >> 24));
    }
    for(index = 0;index < len;index ++)
    {
        *(pDestStr + index) = EEPROM_SPI_ReadWriteByte(SysSPI,0);
    }
    
    GPIO_SetBits(GPIO_PORT_EEPROM_CS,GPIO_Pin_EEPROM_CS);         // CS = 1
    SPI_Cmd(SysSPI, DISABLE);
}



/*******************************************************************************
* @brief   Updates the counter in m95m02.
* @param   void
* @retval  void
*******************************************************************************/
void M95M01_CounterUpdate(void)
{
    uint8_t temp[40] = {0};
    
#ifdef M95M01_Debug
    printf("Now updating counter...\r\n");
#endif    
    
    Translateu32Tou8Array(m95m01_info.memory_info_contact_temp.pRead_m95m01,temp);
    Translateu32Tou8Array(m95m01_info.memory_info_contact_temp.pWrite_m95m01,&temp[4]);
    
    Translateu32Tou8Array(m95m01_info.memory_info_non_contact_temp.pRead_m95m01,&temp[8]);
    Translateu32Tou8Array(m95m01_info.memory_info_non_contact_temp.pWrite_m95m01,&temp[12]);
    
    Translateu32Tou8Array(m95m01_info.memory_info_steps.pRead_m95m01,&temp[16]);
    Translateu32Tou8Array(m95m01_info.memory_info_steps.pWrite_m95m01,&temp[20]);
    
    Translateu32Tou8Array(m95m01_info.memory_info_config_file.pRead_m95m01,&temp[24]);
    Translateu32Tou8Array(m95m01_info.memory_info_config_file.pWrite_m95m01,&temp[28]);
    
    Translateu32Tou8Array(m95m01_info.memory_info_monitor.pRead_m95m01,&temp[32]);
    Translateu32Tou8Array(m95m01_info.memory_info_monitor.pWrite_m95m01,&temp[36]); 
    EEPROM_SPI_WriteToMemory(temp,0,sizeof(temp));
	Delay_ms(10);
}

/*******************************************************************************
* @brief   Updates the counter in m95m02.
* @param   void
* @retval  void
*******************************************************************************/
static bool CheckAddrValid(uint8_t data_id,m95m01_info_t *p_m95m01_info,uint8_t len)
{
    bool state = false;
    
    /* Check the parameters */
    assert_param((data_id == CONTACT_TEMP_VALUE_ID) 
                || (data_id == NON_CONTACT_TEMP_VALUE_ID) 
                || (data_id == STEP_COUNT_VALUE_ID) 
                || (data_id == CONFIG_FILE_ID) 
                || (data_id == MONITOR_MODEL_VALUE_ID));
    assert_param(p_m95m01_info != NULL);
    assert_param(len <= 222);

    switch(data_id)
    {
        case CONTACT_TEMP_VALUE_ID:
            if(p_m95m01_info->memory_info_contact_temp.pRead_m95m01 > p_m95m01_info->memory_info_contact_temp.pWrite_m95m01)
            {
                if((p_m95m01_info->memory_info_contact_temp.pRead_m95m01 + len) > M95M01_PAGE_SPACE)
                {
                    if((p_m95m01_info->memory_info_contact_temp.pRead_m95m01 + len) % M95M01_PAGE_SPACE <= 
                        p_m95m01_info->memory_info_contact_temp.pWrite_m95m01)
                    {
                        state = true;
                    }
                }
                else
                {
                    state = true;
                }
            }
            else
            {
                if((p_m95m01_info->memory_info_contact_temp.pRead_m95m01 + len) <= p_m95m01_info->memory_info_contact_temp.pWrite_m95m01)
                {
                    state = true;
                }
            }
            break;
            
        case NON_CONTACT_TEMP_VALUE_ID:
            if(p_m95m01_info->memory_info_non_contact_temp.pRead_m95m01 > p_m95m01_info->memory_info_non_contact_temp.pWrite_m95m01)
            {
                if((p_m95m01_info->memory_info_non_contact_temp.pRead_m95m01 + len) > M95M01_PAGE_SPACE)
                {
                    if((p_m95m01_info->memory_info_non_contact_temp.pRead_m95m01 + len) % M95M01_PAGE_SPACE <= 
                        p_m95m01_info->memory_info_non_contact_temp.pWrite_m95m01)
                    {
                        state = true;
                    }
                }
                else
                {
                    state = true;
                }
            }
            else
            {
                if((p_m95m01_info->memory_info_non_contact_temp.pRead_m95m01 + len) <= p_m95m01_info->memory_info_non_contact_temp.pWrite_m95m01)
                {
                    state = true;
                }
            }
            break;
            
        case STEP_COUNT_VALUE_ID:
            if(p_m95m01_info->memory_info_steps.pRead_m95m01 > p_m95m01_info->memory_info_steps.pWrite_m95m01)
            {
                if((p_m95m01_info->memory_info_steps.pRead_m95m01 + len) > M95M01_PAGE_SPACE)
                {
                    if((p_m95m01_info->memory_info_steps.pRead_m95m01 + len) % M95M01_PAGE_SPACE <= 
                        p_m95m01_info->memory_info_steps.pWrite_m95m01)
                    {
                        state = true;
                    }
                }
                else
                {
                    state = true;
                }
            }
            else
            {
                if((p_m95m01_info->memory_info_steps.pRead_m95m01 + len) <= p_m95m01_info->memory_info_steps.pWrite_m95m01)
                {
                    state = true;
                }
            }
            break;
            
        case MONITOR_MODEL_VALUE_ID:
            if(p_m95m01_info->memory_info_monitor.pRead_m95m01 > p_m95m01_info->memory_info_monitor.pWrite_m95m01)
            {
				
				if((p_m95m01_info->memory_info_monitor.pRead_m95m01 + len) <= (p_m95m01_info->memory_info_monitor.pWrite_m95m01 + M95M01_ALL_SPACE - M95M01_MONITOR_MODEL_OFFSET))
				{
					state = true;
				}
//                if((p_m95m01_info->memory_info_monitor.pRead_m95m01 + len) > (M95M01_ALL_SPACE - M95M01_MONITOR_MODEL_OFFSET))
//                {
//                    if((p_m95m01_info->memory_info_monitor.pRead_m95m01 + len) % (M95M01_ALL_SPACE - M95M01_MONITOR_MODEL_OFFSET) <= 
//                        p_m95m01_info->memory_info_monitor.pWrite_m95m01)
//                    {
//                        state = true;
//                    }
//                }
//                else
//                {
//                    state = true;
//                }
            }
            else
            {
                if((p_m95m01_info->memory_info_monitor.pRead_m95m01 + len) <= p_m95m01_info->memory_info_monitor.pWrite_m95m01)
                {
                    state = true;
                }
            }
            break;
            
        default:
            break;
    }
    
    return state;
}

/*******************************************************************************
* @brief   Check out the counter in M95m02.
* @param   void
* @retval  void
*******************************************************************************/
void M95M01_CounterCheckOut(void)
{
    uint8_t temp[40] = {0};

#ifdef M95M01_Debug
    printf("Now checking out the counter...\r\n");
#endif  
    
    EEPROM_SPI_ReadFromMemory(temp,0,sizeof(temp));
    
    if((temp[3] == 0xFF) && (temp[11] == 0xFF) && (temp[19] == 0xFF))           // The first use
    {
        m95m01_info.memory_info_contact_temp.pRead_m95m01 = 0;
        m95m01_info.memory_info_contact_temp.pWrite_m95m01 = 0;
        
        m95m01_info.memory_info_non_contact_temp.pRead_m95m01 = 0;
        m95m01_info.memory_info_non_contact_temp.pWrite_m95m01 = 0;
        
        m95m01_info.memory_info_steps.pRead_m95m01 = 0;
        m95m01_info.memory_info_steps.pWrite_m95m01 = 0;
        
        m95m01_info.memory_info_config_file.pRead_m95m01 = 0;
        m95m01_info.memory_info_config_file.pWrite_m95m01 = 0;
        
        m95m01_info.memory_info_monitor.pRead_m95m01 = 0;
        m95m01_info.memory_info_monitor.pWrite_m95m01 = 0;
    }
    else
    {
        m95m01_info.memory_info_contact_temp.pRead_m95m01 = Translateu8ArrayTou32(temp);
        m95m01_info.memory_info_contact_temp.pWrite_m95m01 = Translateu8ArrayTou32(&temp[4]);
        
        m95m01_info.memory_info_non_contact_temp.pRead_m95m01 = Translateu8ArrayTou32(&temp[8]);
        m95m01_info.memory_info_non_contact_temp.pWrite_m95m01 = Translateu8ArrayTou32(&temp[12]);
        
        m95m01_info.memory_info_steps.pRead_m95m01 = Translateu8ArrayTou32(&temp[16]);
        m95m01_info.memory_info_steps.pWrite_m95m01 = Translateu8ArrayTou32(&temp[20]);
        
        m95m01_info.memory_info_config_file.pRead_m95m01 = Translateu8ArrayTou32(&temp[24]);
        m95m01_info.memory_info_config_file.pWrite_m95m01 = Translateu8ArrayTou32(&temp[28]);
        
        m95m01_info.memory_info_monitor.pRead_m95m01 = Translateu8ArrayTou32(&temp[32]);
        m95m01_info.memory_info_monitor.pWrite_m95m01 = Translateu8ArrayTou32(&temp[36]);
    } 
    
#ifdef M95M01_Debug
    printf("Initially,pRead_m95m01 = %d,pWrite_m95m01 = %d...\r\n",
    m95m01_info.memory_info_monitor.pRead_m95m01,m95m01_info.memory_info_monitor.pWrite_m95m01);
#endif     
	
}

/*******************************************************************************
* @brief   Write M95M02.
* @param   data_id:the type of data to be writed
* @param   len:the length of data to be writed
* @param   ptr:the source to be writed
* @retval  true --- Success;false --- Fail
*******************************************************************************/
bool M95M01_Write(uint8_t data_id,uint8_t len,uint8_t * ptr)
{   
    uint32_t pStartAddr = 0;
    uint8_t temp_len = 0;
    uint32_t temp_space = 0;
    
    switch (data_id)
    {
        case CONTACT_TEMP_VALUE_ID:
            if(((m95m01_info.memory_info_contact_temp.pWrite_m95m01 + 1)%M95M01_PAGE_SPACE) ==
                 m95m01_info.memory_info_contact_temp.pRead_m95m01)// FIFO is full,must be read out.
            {
                return ERR_WRITE_EEPROM;
            }
            else            
            {
                if(m95m01_info.memory_info_contact_temp.pWrite_m95m01 >= m95m01_info.memory_info_contact_temp.pRead_m95m01)
                {
                    temp_space = M95M01_PAGE_SPACE - (m95m01_info.memory_info_contact_temp.pWrite_m95m01 - 
                                            m95m01_info.memory_info_contact_temp.pRead_m95m01);
                }
                else
                {
                    temp_space = m95m01_info.memory_info_contact_temp.pRead_m95m01 - 
                                            m95m01_info.memory_info_contact_temp.pWrite_m95m01;
                }
                
                if(temp_space < len)
                {
                    /* First moves the pointer pRead_m95m01 to (pRead_m95m01 + len) */
                    m95m01_info.memory_info_contact_temp.pRead_m95m01 += len;
                    m95m01_info.memory_info_contact_temp.pRead_m95m01 %= M95M01_PAGE_SPACE;
                }
                
                pStartAddr = M95M01_CONTACT_TEMP_OFFSET + m95m01_info.memory_info_contact_temp.pWrite_m95m01;
                EEPROM_SPI_WriteToMemory(ptr,pStartAddr,len);
                
                m95m01_info.memory_info_contact_temp.pWrite_m95m01 += len;
                m95m01_info.memory_info_contact_temp.pWrite_m95m01 %= M95M01_PAGE_SPACE;
                
                M95M01_CounterUpdate();
                return APP_SUCCESS;
            }
        
        case NON_CONTACT_TEMP_VALUE_ID:
            if(((m95m01_info.memory_info_non_contact_temp.pWrite_m95m01 + 1)%M95M01_PAGE_SPACE) ==
                 m95m01_info.memory_info_non_contact_temp.pRead_m95m01)// FIFO is full,must be read out.
            {
                return ERR_WRITE_EEPROM;
            }
            else            
            {
                if(m95m01_info.memory_info_non_contact_temp.pWrite_m95m01 >= m95m01_info.memory_info_non_contact_temp.pRead_m95m01)
                {
                    temp_space = M95M01_PAGE_SPACE - (m95m01_info.memory_info_non_contact_temp.pWrite_m95m01 - 
                                            m95m01_info.memory_info_non_contact_temp.pRead_m95m01);
                }
                else
                {
                    temp_space = m95m01_info.memory_info_non_contact_temp.pRead_m95m01 - 
                                            m95m01_info.memory_info_non_contact_temp.pWrite_m95m01;
                }
                
                if(temp_space < len)
                {
                    /* First moves the pointer pRead_m95m01 to (pRead_m95m01 + len) */
                    m95m01_info.memory_info_non_contact_temp.pRead_m95m01 += len;
                    m95m01_info.memory_info_non_contact_temp.pRead_m95m01 %= M95M01_PAGE_SPACE;
                }
                
                pStartAddr = M95M01_NON_CONTACT_TEMP_OFFSET + m95m01_info.memory_info_non_contact_temp.pWrite_m95m01;
                EEPROM_SPI_WriteToMemory(ptr,pStartAddr,len);
                
                m95m01_info.memory_info_non_contact_temp.pWrite_m95m01 += len;
                m95m01_info.memory_info_non_contact_temp.pWrite_m95m01 %= M95M01_PAGE_SPACE;
                
                M95M01_CounterUpdate();
                return APP_SUCCESS;
            }
        
        case STEP_COUNT_VALUE_ID: 
            if(((m95m01_info.memory_info_steps.pWrite_m95m01 + 1)%M95M01_PAGE_SPACE) ==
                 m95m01_info.memory_info_steps.pRead_m95m01)// FIFO is full,must be read out.
            {
                return ERR_WRITE_EEPROM;
            }
            else            
            {
                if(m95m01_info.memory_info_steps.pWrite_m95m01 >= m95m01_info.memory_info_steps.pRead_m95m01)
                {
                    temp_space = M95M01_PAGE_SPACE - (m95m01_info.memory_info_steps.pWrite_m95m01 - 
                                            m95m01_info.memory_info_steps.pRead_m95m01);
                }
                else
                {
                    temp_space = m95m01_info.memory_info_steps.pRead_m95m01 - 
                                            m95m01_info.memory_info_steps.pWrite_m95m01;
                }
                
                if(temp_space < len)
                {
                    /* First moves the pointer pRead_m95m01 to (pRead_m95m01 + len) */
                    m95m01_info.memory_info_steps.pRead_m95m01 += len;
                    m95m01_info.memory_info_steps.pRead_m95m01 %= M95M01_PAGE_SPACE;
                }
                
                pStartAddr = M95M01_STEP_COUNT_OFFSET + m95m01_info.memory_info_steps.pWrite_m95m01;
                EEPROM_SPI_WriteToMemory(ptr,pStartAddr,len);
                
                m95m01_info.memory_info_steps.pWrite_m95m01 += len;
                m95m01_info.memory_info_steps.pWrite_m95m01 %= M95M01_PAGE_SPACE;
                
                M95M01_CounterUpdate();
                return APP_SUCCESS;
            }
        
        case CONFIG_FILE_ID: 
            if(((m95m01_info.memory_info_config_file.pWrite_m95m01 + 1)%M95M01_PAGE_SPACE) ==
                 m95m01_info.memory_info_config_file.pRead_m95m01)// FIFO is full,must be read out.
            {
                return ERR_WRITE_EEPROM;
            }
            else            
            {
                if(m95m01_info.memory_info_config_file.pWrite_m95m01 >= m95m01_info.memory_info_config_file.pRead_m95m01)
                {
                    temp_space = M95M01_PAGE_SPACE - (m95m01_info.memory_info_config_file.pWrite_m95m01 - 
                                            m95m01_info.memory_info_config_file.pRead_m95m01);
                }
                else
                {
                    temp_space = m95m01_info.memory_info_config_file.pRead_m95m01 - 
                                            m95m01_info.memory_info_config_file.pWrite_m95m01;
                }
                
                if(temp_space < len)
                {
                    /* First moves the pointer pRead_m95m01 to (pRead_m95m01 + len) */
                    m95m01_info.memory_info_config_file.pRead_m95m01 += len;
                    m95m01_info.memory_info_config_file.pRead_m95m01 %= M95M01_PAGE_SPACE;
                }
                
                pStartAddr = M95M01_CONFIG_FILE_OFFSET + m95m01_info.memory_info_config_file.pWrite_m95m01;
                EEPROM_SPI_WriteToMemory(ptr,pStartAddr,len);
                
                m95m01_info.memory_info_config_file.pWrite_m95m01 += len;
                m95m01_info.memory_info_config_file.pWrite_m95m01 %= M95M01_PAGE_SPACE;
                
                M95M01_CounterUpdate();
                return APP_SUCCESS;
            }
        
        case MONITOR_MODEL_VALUE_ID:
#ifdef M95M01_Debug
    printf("data_id = %d...\r\n",data_id);
#endif 
        
#ifdef M95M01_Debug
    printf("before writing,pRead_m95m01 = %d,pWrite_m95m01 = %d...\r\n",m95m01_info.memory_info_monitor.pRead_m95m01,m95m01_info.memory_info_monitor.pWrite_m95m01);
#endif 
            if(((m95m01_info.memory_info_monitor.pWrite_m95m01 + 1)%(M95M01_ALL_SPACE - M95M01_MONITOR_MODEL_OFFSET)) ==
                 m95m01_info.memory_info_monitor.pRead_m95m01)// FIFO is full,must be read out.
            {
                return ERR_WRITE_EEPROM;
            }
            else            
            {
                if(m95m01_info.memory_info_monitor.pWrite_m95m01 >= m95m01_info.memory_info_monitor.pRead_m95m01)
                {
                    temp_space = (M95M01_ALL_SPACE - M95M01_MONITOR_MODEL_OFFSET) - (m95m01_info.memory_info_monitor.pWrite_m95m01 - 
                                            m95m01_info.memory_info_monitor.pRead_m95m01);
                }
                else
                {
                    temp_space = m95m01_info.memory_info_monitor.pRead_m95m01 - m95m01_info.memory_info_monitor.pWrite_m95m01;                                           ;
                }
                
                if(temp_space < len)
                {
                    /* First moves the pointer pRead_m95m01 to (pRead_m95m01 + len) */
                    m95m01_info.memory_info_monitor.pRead_m95m01 += len;
                    m95m01_info.memory_info_monitor.pRead_m95m01 %= (M95M01_ALL_SPACE - M95M01_MONITOR_MODEL_OFFSET);
                }
                
                if(((m95m01_info.memory_info_monitor.pWrite_m95m01 + len) / M95M01_PAGE_SPACE) > 
                    m95m01_info.memory_info_monitor.pWrite_m95m01 / M95M01_PAGE_SPACE)
                {
#ifdef M95M01_Debug
                    printf("In the different page...\r\n");
#endif
                    pStartAddr = M95M01_MONITOR_MODEL_OFFSET + m95m01_info.memory_info_monitor.pWrite_m95m01;
                    temp_len = M95M01_PAGE_SPACE - (m95m01_info.memory_info_monitor.pWrite_m95m01 % M95M01_PAGE_SPACE);
                    EEPROM_SPI_WriteToMemory(ptr,pStartAddr,temp_len);
                    Delay_ms(10);
                    pStartAddr = M95M01_MONITOR_MODEL_OFFSET + 
                                 (((m95m01_info.memory_info_monitor.pWrite_m95m01 + len) / M95M01_PAGE_SPACE) * M95M01_PAGE_SPACE);
                    EEPROM_SPI_WriteToMemory(&(ptr[temp_len]),pStartAddr,(len - temp_len));
                }
                else            // In the same page
                {
#ifdef M95M01_Debug
                    printf("In the same page...\r\n");
#endif 
                    pStartAddr = M95M01_MONITOR_MODEL_OFFSET + m95m01_info.memory_info_monitor.pWrite_m95m01;
                    EEPROM_SPI_WriteToMemory(ptr,pStartAddr,len);
                }
                
                m95m01_info.memory_info_monitor.pWrite_m95m01 += len;
                m95m01_info.memory_info_monitor.pWrite_m95m01 %= (M95M01_ALL_SPACE - M95M01_MONITOR_MODEL_OFFSET);
#ifdef M95M01_Debug
                printf("After writen,pRead_m95m01 = %d,pWrite_m95m01 = %d...\r\n",m95m01_info.memory_info_monitor.pRead_m95m01,m95m01_info.memory_info_monitor.pWrite_m95m01);
#endif                 
                M95M01_CounterUpdate();
                return APP_SUCCESS;
            }

        default:
            break;
    }
    return ERR_WRITE_EEPROM;
}

/*******************************************************************************
* @brief   Read M95M02.
* @param   data_id:the type of data to be writed
* @param   len:the length of data to be writed
* @param   ptr:the source to be writed
* @retval  true --- Success;false --- Fail
*******************************************************************************/
bool M95M01_Read(uint8_t data_id,uint8_t len,uint8_t * ptr)
{   
    uint32_t pStartAddr = 0;
    uint32_t pStartAddr_temp = 0;
    uint32_t temp_space = 0;
	uint32_t index = 0;
     
    switch (data_id)
    {
        case CONTACT_TEMP_VALUE_ID:
            if(m95m01_info.memory_info_contact_temp.pRead_m95m01 != m95m01_info.memory_info_contact_temp.pWrite_m95m01)
            {
                if(m95m01_info.memory_info_contact_temp.pWrite_m95m01 > m95m01_info.memory_info_contact_temp.pRead_m95m01)
                {
                    temp_space = m95m01_info.memory_info_contact_temp.pWrite_m95m01 - 
                                            m95m01_info.memory_info_contact_temp.pRead_m95m01;
                }
                else
                {
                    temp_space = (m95m01_info.memory_info_contact_temp.pWrite_m95m01 + M95M01_PAGE_SPACE) - 
                                            m95m01_info.memory_info_contact_temp.pRead_m95m01;
                }
                
                if(temp_space < len)        // Invalid data
                {
                    pStartAddr = M95M01_CONTACT_TEMP_OFFSET + m95m01_info.memory_info_contact_temp.pRead_m95m01;
                    EEPROM_SPI_ReadFromMemory(ptr,pStartAddr,temp_space);
                    for(index = temp_space;index < len;index ++)
                    {
                         *(ptr + index) = *(ptr + temp_space - 1);
                    }
                    return ERR_READ_EEPROM;
                }
                else                // Direct Read
                {
                    if((m95m01_info.memory_info_contact_temp.pRead_m95m01 + len) > M95M01_PAGE_SPACE)
                    {
                        /* Need to read two times */
                        pStartAddr = M95M01_CONTACT_TEMP_OFFSET + m95m01_info.memory_info_contact_temp.pRead_m95m01;
                        pStartAddr_temp = M95M01_PAGE_SPACE - (m95m01_info.memory_info_contact_temp.pRead_m95m01);
                        EEPROM_SPI_ReadFromMemory(ptr,pStartAddr,pStartAddr_temp);
                        
                        pStartAddr = M95M01_CONTACT_TEMP_OFFSET;
                        EEPROM_SPI_ReadFromMemory(ptr + pStartAddr_temp,
                                                  pStartAddr,
                                                 (m95m01_info.memory_info_contact_temp.pRead_m95m01 + len)%M95M01_PAGE_SPACE);
                    }
                    else
                    {
                        pStartAddr = M95M01_CONTACT_TEMP_OFFSET + m95m01_info.memory_info_contact_temp.pRead_m95m01;
                        EEPROM_SPI_ReadFromMemory(ptr,pStartAddr,len);
                    }
                    return APP_SUCCESS;
                }
            }
            else                                        // FIFO is empty
            {
                return ERR_READ_EEPROM;
            }
        
        case NON_CONTACT_TEMP_VALUE_ID:
            if(m95m01_info.memory_info_non_contact_temp.pRead_m95m01 != m95m01_info.memory_info_non_contact_temp.pWrite_m95m01)
            {
                if(m95m01_info.memory_info_non_contact_temp.pWrite_m95m01 > m95m01_info.memory_info_non_contact_temp.pRead_m95m01)
                {
                    temp_space = m95m01_info.memory_info_non_contact_temp.pWrite_m95m01 - 
                                            m95m01_info.memory_info_non_contact_temp.pRead_m95m01;
                }
                else
                {
                    temp_space = (m95m01_info.memory_info_non_contact_temp.pWrite_m95m01 + M95M01_PAGE_SPACE) - 
                                            m95m01_info.memory_info_non_contact_temp.pRead_m95m01;
                }
                
                if(temp_space < len)        // Invalid data
                {
                    pStartAddr = M95M01_NON_CONTACT_TEMP_OFFSET + m95m01_info.memory_info_non_contact_temp.pRead_m95m01;
                    EEPROM_SPI_ReadFromMemory(ptr,pStartAddr,temp_space);
                    for(index = temp_space;index < len;index ++)
                    {
                         *(ptr + index) = *(ptr + temp_space - 1);
                    }
                    return ERR_READ_EEPROM;
                }
                else                // Direct Read
                {
                    if((m95m01_info.memory_info_non_contact_temp.pRead_m95m01 + len) > M95M01_PAGE_SPACE)
                    {
                        /* Need to read two times */
                        pStartAddr = M95M01_NON_CONTACT_TEMP_OFFSET + m95m01_info.memory_info_non_contact_temp.pRead_m95m01;
                        pStartAddr_temp = M95M01_PAGE_SPACE - (m95m01_info.memory_info_non_contact_temp.pRead_m95m01);
                        EEPROM_SPI_ReadFromMemory(ptr,pStartAddr,pStartAddr_temp);
                        
                        pStartAddr = M95M01_NON_CONTACT_TEMP_OFFSET;
                        EEPROM_SPI_ReadFromMemory(ptr + pStartAddr_temp,
                                                  pStartAddr,
                                                 (m95m01_info.memory_info_non_contact_temp.pRead_m95m01 + len)%M95M01_PAGE_SPACE);
                    }
                    else
                    {
                        pStartAddr = M95M01_NON_CONTACT_TEMP_OFFSET + m95m01_info.memory_info_non_contact_temp.pRead_m95m01;
                        EEPROM_SPI_ReadFromMemory(ptr,pStartAddr,len);
                    }
                    return APP_SUCCESS;
                }
            }
            else                                        // FIFO is empty
            {
                return ERR_READ_EEPROM;
            }
        
        case STEP_COUNT_VALUE_ID: 
            if(m95m01_info.memory_info_steps.pRead_m95m01 != m95m01_info.memory_info_steps.pWrite_m95m01)
            {
                if(m95m01_info.memory_info_steps.pWrite_m95m01 > m95m01_info.memory_info_steps.pRead_m95m01)
                {
                    temp_space = m95m01_info.memory_info_steps.pWrite_m95m01 - 
                                            m95m01_info.memory_info_steps.pRead_m95m01;
                }
                else
                {
                    temp_space = (m95m01_info.memory_info_steps.pWrite_m95m01 + M95M01_PAGE_SPACE) - 
                                            m95m01_info.memory_info_steps.pRead_m95m01;
                }
                
                if(temp_space < len)        // Invalid data
                {
                    pStartAddr = M95M01_STEP_COUNT_OFFSET + m95m01_info.memory_info_steps.pRead_m95m01;
                    EEPROM_SPI_ReadFromMemory(ptr,pStartAddr,temp_space);
                    for(index = temp_space;index < len;index ++)
                    {
                         *(ptr + index) = *(ptr + temp_space - 1);
                    }
                    return ERR_READ_EEPROM;
                }
                else                // Direct Read
                {
                    if((m95m01_info.memory_info_steps.pRead_m95m01 + len) > M95M01_PAGE_SPACE)
                    {
                        /* Need to read two times */
                        pStartAddr = M95M01_STEP_COUNT_OFFSET + m95m01_info.memory_info_steps.pRead_m95m01;
                        pStartAddr_temp = M95M01_PAGE_SPACE - (m95m01_info.memory_info_steps.pRead_m95m01);
                        EEPROM_SPI_ReadFromMemory(ptr,pStartAddr,pStartAddr_temp);
                        
                        pStartAddr = M95M01_STEP_COUNT_OFFSET;
                        EEPROM_SPI_ReadFromMemory(ptr + pStartAddr_temp,
                                                  pStartAddr,
                                                 (m95m01_info.memory_info_steps.pRead_m95m01 + len)%M95M01_PAGE_SPACE);
                    }
                    else
                    {
                        pStartAddr = M95M01_STEP_COUNT_OFFSET + m95m01_info.memory_info_steps.pRead_m95m01;
                        EEPROM_SPI_ReadFromMemory(ptr,pStartAddr,len);
                    }
                    return APP_SUCCESS;
                }
            }
            else                                        // FIFO is empty
            {
                return ERR_READ_EEPROM;
            }
        
        case CONFIG_FILE_ID: 
            /*if(m95m01_info.memory_info_config_file.pRead_m95m01 != m95m01_info.memory_info_config_file.pWrite_m95m01)
            {
                if(m95m01_info.memory_info_config_file.pWrite_m95m01 > m95m01_info.memory_info_config_file.pRead_m95m01)
                {
                    temp_space = m95m01_info.memory_info_config_file.pWrite_m95m01 - 
                                            m95m01_info.memory_info_config_file.pRead_m95m01;
                }
                else
                {
                    temp_space = (m95m01_info.memory_info_config_file.pWrite_m95m01 + M95M01_PAGE_SPACE) - 
                                            m95m01_info.memory_info_config_file.pRead_m95m01;
                }
                
                if(temp_space < len)        // Invalid data
                {
                    pStartAddr = M95M01_CONFIG_FILE_OFFSET + m95m01_info.memory_info_config_file.pRead_m95m01;
                    EEPROM_SPI_ReadFromMemory(ptr,pStartAddr,temp_space);
                    for(index = temp_space;index < len;index ++)
                    {
                         *(ptr + index) = *(ptr + temp_space - 1);
                    }
                    return ERR_READ_EEPROM;
                }
                else                // Direct Read
                {
                    if((m95m01_info.memory_info_steps.pRead_m95m01 + len) > M95M01_PAGE_SPACE)
                    {
                        // Need to read two times 
                        pStartAddr = M95M01_CONFIG_FILE_OFFSET + m95m01_info.memory_info_config_file.pRead_m95m01;
                        pStartAddr_temp = (m95m01_info.memory_info_config_file.pRead_m95m01 + len) - M95M01_PAGE_SPACE;
                        EEPROM_SPI_ReadFromMemory(ptr,pStartAddr,pStartAddr_temp);
                        
                        pStartAddr = M95M01_CONFIG_FILE_OFFSET;
                        EEPROM_SPI_ReadFromMemory(ptr + pStartAddr_temp,
                                                  M95M01_CONFIG_FILE_OFFSET,
                                                 (m95m01_info.memory_info_config_file.pRead_m95m01 + len)%M95M01_PAGE_SPACE);
                    }
                    else
                    {
                        pStartAddr = M95M01_CONFIG_FILE_OFFSET + m95m01_info.memory_info_config_file.pRead_m95m01;
                        EEPROM_SPI_ReadFromMemory(ptr,pStartAddr,len);
                    }
                    return APP_SUCCESS;
                }
            }
            else                                        // FIFO is empty
            {
                return ERR_READ_EEPROM;
            }*/
            break;
        
        case MONITOR_MODEL_VALUE_ID:
            if(m95m01_info.memory_info_monitor.pRead_m95m01 != m95m01_info.memory_info_monitor.pWrite_m95m01)
            {
                if(m95m01_info.memory_info_monitor.pWrite_m95m01 > m95m01_info.memory_info_monitor.pRead_m95m01)
                {
                    temp_space = m95m01_info.memory_info_monitor.pWrite_m95m01 - 
                                            m95m01_info.memory_info_monitor.pRead_m95m01;
                }
                else
                {
                    temp_space = (m95m01_info.memory_info_monitor.pWrite_m95m01 + (M95M01_ALL_SPACE - M95M01_MONITOR_MODEL_OFFSET)) - 
                                            m95m01_info.memory_info_monitor.pRead_m95m01;
                }
                
                if(temp_space < len)        // Invalid data
                {
                    pStartAddr = M95M01_MONITOR_MODEL_OFFSET + m95m01_info.memory_info_monitor.pRead_m95m01;
                    EEPROM_SPI_ReadFromMemory(ptr,pStartAddr,temp_space);
                    for(index = temp_space;index < len;index ++)
                    {
                         *(ptr + index) = *(ptr + temp_space - 1);
                    }
                    return ERR_READ_EEPROM;
                }
                else                // Direct Read
                {   
                    if((m95m01_info.memory_info_monitor.pRead_m95m01 + len) > (M95M01_ALL_SPACE - M95M01_MONITOR_MODEL_OFFSET))
                    {   
                        /* Need to read two times */
                        pStartAddr = M95M01_MONITOR_MODEL_OFFSET + m95m01_info.memory_info_monitor.pRead_m95m01;
                        pStartAddr_temp = (M95M01_ALL_SPACE - M95M01_MONITOR_MODEL_OFFSET) - (m95m01_info.memory_info_monitor.pRead_m95m01);
                        EEPROM_SPI_ReadFromMemory(ptr,pStartAddr,pStartAddr_temp);

                        pStartAddr = M95M01_MONITOR_MODEL_OFFSET;
                        EEPROM_SPI_ReadFromMemory(ptr + pStartAddr_temp,
                                                  pStartAddr,
                                                 (m95m01_info.memory_info_monitor.pRead_m95m01 + len)%(M95M01_ALL_SPACE - M95M01_MONITOR_MODEL_OFFSET));
                    }
                    else
                    {
                        pStartAddr = M95M01_MONITOR_MODEL_OFFSET + m95m01_info.memory_info_monitor.pRead_m95m01;
                        EEPROM_SPI_ReadFromMemory(ptr,pStartAddr,len);
                    }
                    return APP_SUCCESS;
                }
            }
            else                                        // FIFO is empty
            {
                return ERR_READ_EEPROM;
            }
        
        default:
            break;
    }
    return ERR_READ_EEPROM;
}

/*******************************************************************************
* @brief   returns the state of reading M95M02.
* @param   data_id:the type of data read out
* @param   len:the length of data read out
* @param   state:the state(1:read success,0:read fail)
* @retval  true --- Success;false --- Fail
*******************************************************************************/
void M95M01_ReadOK(uint8_t data_id,uint16_t len,uint8_t state)
{
    if(state)                                               // Success
    {   
        switch (data_id)
        {
            case CONTACT_TEMP_VALUE_ID:
                if(CheckAddrValid(CONTACT_TEMP_VALUE_ID,&m95m01_info,len))
                {
                    m95m01_info.memory_info_contact_temp.pRead_m95m01 += len;
                    m95m01_info.memory_info_contact_temp.pRead_m95m01 %= M95M01_PAGE_SPACE;
                    M95M01_CounterUpdate();
                }
                break;
            
            case NON_CONTACT_TEMP_VALUE_ID:
                if(CheckAddrValid(NON_CONTACT_TEMP_VALUE_ID,&m95m01_info,len))
                {
                    m95m01_info.memory_info_non_contact_temp.pRead_m95m01 += len;
                    m95m01_info.memory_info_non_contact_temp.pRead_m95m01 %= M95M01_PAGE_SPACE;
                    M95M01_CounterUpdate();
                }
                break;
            
            case STEP_COUNT_VALUE_ID: 
                if(CheckAddrValid(STEP_COUNT_VALUE_ID,&m95m01_info,len))
                {
                    m95m01_info.memory_info_steps.pRead_m95m01 += len;
                    m95m01_info.memory_info_steps.pRead_m95m01 %= M95M01_PAGE_SPACE;
                    M95M01_CounterUpdate();
                }
                break;
            
            case CONFIG_FILE_ID: 
                if(CheckAddrValid(CONFIG_FILE_ID,&m95m01_info,len))
                {
                    m95m01_info.memory_info_config_file.pRead_m95m01 += len;
                    m95m01_info.memory_info_config_file.pRead_m95m01 %= M95M01_PAGE_SPACE;
                    M95M01_CounterUpdate();
                }
                break;
            
            case MONITOR_MODEL_VALUE_ID:
                if(CheckAddrValid(MONITOR_MODEL_VALUE_ID,&m95m01_info,len))
                {

                    m95m01_info.memory_info_monitor.pRead_m95m01 += len;
                    m95m01_info.memory_info_monitor.pRead_m95m01 %= (M95M01_ALL_SPACE - M95M01_MONITOR_MODEL_OFFSET);
                    M95M01_CounterUpdate();
                }
                else
                {

                }
                break;
            
            default:
                break;
        }
    }
}

/*******************************************************************************
* @brief   returns the M95M02's information.
* @param   data_id:the type of data read out
* @param   getType:Must be one of the following values: 
                   M95M01_CAPACITY_SPACE,M95M01_IS_BUSY,M95M01_READ_AVAILABLE_SPACE
* @retval  
*******************************************************************************/
uint32_t GetM95M01State(uint8_t data_id,uint8_t getType)
{
	uint8_t Err_Code=0;
    switch (data_id)
    {
        case CONTACT_TEMP_VALUE_ID:
            if(getType == M95M01_CAPACITY_SPACE)
            {
                if(m95m01_info.memory_info_contact_temp.pRead_m95m01 <= m95m01_info.memory_info_contact_temp.pWrite_m95m01)
                {
                    return (m95m01_info.memory_info_contact_temp.pRead_m95m01 + M95M01_PAGE_SPACE - 
                            m95m01_info.memory_info_contact_temp.pWrite_m95m01);
                }
                else
                {
                    return (m95m01_info.memory_info_contact_temp.pRead_m95m01 - 
                            m95m01_info.memory_info_contact_temp.pWrite_m95m01);
                }
            }
            else if(getType == M95M01_IS_BUSY)
            {
                if((EEPROM_SPI_ReadStatusRegister() & M95M01_WIP_STATUS))           // Busy
                {
                    return false;
                }    
                else
                {
                    return true;
                }
            }
            else if(getType == M95M01_READ_AVAILABLE_SPACE)
            {
                if(m95m01_info.memory_info_contact_temp.pRead_m95m01 > m95m01_info.memory_info_contact_temp.pWrite_m95m01)
                {
                    return (m95m01_info.memory_info_contact_temp.pWrite_m95m01 + M95M01_PAGE_SPACE - 
                            m95m01_info.memory_info_contact_temp.pRead_m95m01);
                }
                else
                {
                    return (m95m01_info.memory_info_contact_temp.pWrite_m95m01 - 
                            m95m01_info.memory_info_contact_temp.pRead_m95m01);
                }
            }
            else
            {}
            break;
        
        case NON_CONTACT_TEMP_VALUE_ID:
            if(getType == M95M01_CAPACITY_SPACE)
            {
                if(m95m01_info.memory_info_non_contact_temp.pRead_m95m01 <= m95m01_info.memory_info_non_contact_temp.pWrite_m95m01)
                {
                    return (m95m01_info.memory_info_non_contact_temp.pRead_m95m01 + M95M01_PAGE_SPACE - 
                            m95m01_info.memory_info_non_contact_temp.pWrite_m95m01);
                }
                else
                {
                    return (m95m01_info.memory_info_non_contact_temp.pRead_m95m01 - 
                            m95m01_info.memory_info_non_contact_temp.pWrite_m95m01);
                }
            }
            else if(getType == M95M01_IS_BUSY)
            {
                if((EEPROM_SPI_ReadStatusRegister() & M95M01_WIP_STATUS))           // Busy
                {
                    return false;
                }    
                else
                {
                    return true;
                }
            }
            else if(getType == M95M01_READ_AVAILABLE_SPACE)
            {
                if(m95m01_info.memory_info_non_contact_temp.pRead_m95m01 > m95m01_info.memory_info_non_contact_temp.pWrite_m95m01)
                {
                    return (m95m01_info.memory_info_non_contact_temp.pWrite_m95m01 + M95M01_PAGE_SPACE - 
                            m95m01_info.memory_info_non_contact_temp.pRead_m95m01);
                }
                else
                {
                    return (m95m01_info.memory_info_non_contact_temp.pWrite_m95m01 - 
                            m95m01_info.memory_info_non_contact_temp.pRead_m95m01);
                }
            }
            else
            {}
            break;
        
        case STEP_COUNT_VALUE_ID: 
            if(getType == M95M01_CAPACITY_SPACE)
            {
                if(m95m01_info.memory_info_steps.pRead_m95m01 <= m95m01_info.memory_info_steps.pWrite_m95m01)
                {
                    return (m95m01_info.memory_info_steps.pRead_m95m01 + M95M01_PAGE_SPACE - 
                            m95m01_info.memory_info_steps.pWrite_m95m01);
                }
                else
                {
                    return (m95m01_info.memory_info_steps.pRead_m95m01 - 
                            m95m01_info.memory_info_steps.pWrite_m95m01);
                }
            }
            else if(getType == M95M01_IS_BUSY)
            {
                if((EEPROM_SPI_ReadStatusRegister() & M95M01_WIP_STATUS))           // Busy
                {
                    return false;
                }    
                else
                {
                    return true;
                }
            }
            else if(getType == M95M01_READ_AVAILABLE_SPACE)
            {
                if(m95m01_info.memory_info_steps.pRead_m95m01 > m95m01_info.memory_info_steps.pWrite_m95m01)
                {
                    return (m95m01_info.memory_info_steps.pWrite_m95m01 + M95M01_PAGE_SPACE - 
                            m95m01_info.memory_info_steps.pRead_m95m01);
                }
                else
                {
                    return (m95m01_info.memory_info_steps.pWrite_m95m01 - 
                            m95m01_info.memory_info_steps.pRead_m95m01);
                }
            }
            else
            {}
            break;
        
        case CONFIG_FILE_ID: 
            if(getType == M95M01_CAPACITY_SPACE)
            {
                if(m95m01_info.memory_info_config_file.pRead_m95m01 <= m95m01_info.memory_info_config_file.pWrite_m95m01)
                {
                    return (m95m01_info.memory_info_config_file.pRead_m95m01 + M95M01_PAGE_SPACE - 
                            m95m01_info.memory_info_config_file.pWrite_m95m01);
                }
                else
                {
                    return (m95m01_info.memory_info_config_file.pRead_m95m01 - 
                            m95m01_info.memory_info_config_file.pWrite_m95m01);
                }
            }
            else if(getType == M95M01_IS_BUSY)
            {
                if((EEPROM_SPI_ReadStatusRegister() & M95M01_WIP_STATUS))           // Busy
                {
                    return false;
                }    
                else
                {
                    return true;
                }
            }
            else if(getType == M95M01_READ_AVAILABLE_SPACE)
            {
                if(m95m01_info.memory_info_config_file.pRead_m95m01 > m95m01_info.memory_info_config_file.pWrite_m95m01)
                {
                    return (m95m01_info.memory_info_config_file.pWrite_m95m01 + M95M01_PAGE_SPACE - 
                            m95m01_info.memory_info_config_file.pRead_m95m01);
                }
                else
                {
                    return (m95m01_info.memory_info_config_file.pWrite_m95m01 - 
                            m95m01_info.memory_info_config_file.pRead_m95m01);
                }
            }
            else
            {}
            break;
        
        case MONITOR_MODEL_VALUE_ID:

            if(getType == M95M01_CAPACITY_SPACE)
            {
                if(m95m01_info.memory_info_monitor.pRead_m95m01 <= m95m01_info.memory_info_monitor.pWrite_m95m01)
                {
                    return (m95m01_info.memory_info_monitor.pRead_m95m01 + (M95M01_ALL_SPACE - M95M01_MONITOR_MODEL_OFFSET) - 
                            m95m01_info.memory_info_monitor.pWrite_m95m01);
                }
                else
                {
                    return (m95m01_info.memory_info_monitor.pRead_m95m01 - 
                            m95m01_info.memory_info_monitor.pWrite_m95m01);
                }
            }
            else if(getType == M95M01_IS_BUSY)
            {
				Err_Code = EEPROM_SPI_ReadStatusRegister();
                if((Err_Code & M95M01_WIP_STATUS))           // Busy
                {
					#ifdef EEPROM_DEBUG
						printf("M95M02 Read Busy\r\n");
					#endif					
                    return false;
                }    
                else
                {
                    return true;
                }
            }
            else if(getType == M95M01_READ_AVAILABLE_SPACE)
            {
                if(m95m01_info.memory_info_monitor.pRead_m95m01 > m95m01_info.memory_info_monitor.pWrite_m95m01)
                {
                    return (m95m01_info.memory_info_monitor.pWrite_m95m01 + (M95M01_ALL_SPACE - M95M01_MONITOR_MODEL_OFFSET) - 
                            m95m01_info.memory_info_monitor.pRead_m95m01);
                }
                else
                {	
                    return (m95m01_info.memory_info_monitor.pWrite_m95m01 - 
                            m95m01_info.memory_info_monitor.pRead_m95m01);
                }
            }
            else
            {}
            break;
        
        default:
            break;
    } 
    return false;    
}
/*******************************************************************************
* @brief   Clear a page in M95M02
* @param   page_addr:the page addr to be cleared
* @retval  void
*******************************************************************************/
void ClearM95M01(uint16_t page_addr)
{
    uint16_t index      = 0;
    uint32_t temp_Addr  = page_addr << 8;
	uint8_t WriteEEPROMTimeOutCnt = 80;

	while((WriteEEPROMTimeOutCnt !=0) && GetM95M01State(MONITOR_MODEL_VALUE_ID,M95M01_IS_BUSY) == false)
	{
		WriteEEPROMTimeOutCnt--;
		Delay_ms(10);
	}
 
    EEPROM_SPI_WriteEnable();
    
    SPI_Cmd(SysSPI, ENABLE);
    GPIO_ResetBits(GPIO_PORT_EEPROM_CS,GPIO_Pin_EEPROM_CS);       // CS = 0

    EEPROM_SPI_ReadWriteByte(SysSPI,M95M01_WRITE);
    for(index = 0;index < 3;index ++)
    {
        temp_Addr <<= 8;
        EEPROM_SPI_ReadWriteByte(SysSPI,(uint8_t)(temp_Addr >> 24));
    }
    for(index = 0;index < 256;index ++)
    {
        EEPROM_SPI_ReadWriteByte(SysSPI,0);
    }
	
    GPIO_SetBits(GPIO_PORT_EEPROM_CS,GPIO_Pin_EEPROM_CS);         // CS = 1 
    SPI_Cmd(SysSPI, DISABLE);
    
    EEPROM_SPI_WriteDisable();
}

/*******************************************************************************
* @brief   Erase a page in M95M01
* @param   page_addr:the page addr to be erased
* @retval  void
*******************************************************************************/
void EraseM95M01(uint16_t page_addr)
{
    uint16_t index      = 0;
    uint32_t temp_Addr  = page_addr << 8;
	uint8_t WriteEEPROMTimeOutCnt = 80;

	while((WriteEEPROMTimeOutCnt !=0) && GetM95M01State(MONITOR_MODEL_VALUE_ID,M95M01_IS_BUSY) == false)
	{
		WriteEEPROMTimeOutCnt--;
		Delay_ms(10);
	}
 
    EEPROM_SPI_WriteEnable();
    
    SPI_Cmd(SysSPI, ENABLE);
    GPIO_ResetBits(GPIO_PORT_EEPROM_CS,GPIO_Pin_EEPROM_CS);       // CS = 0

    EEPROM_SPI_ReadWriteByte(SysSPI,M95M01_WRITE);
    for(index = 0;index < 3;index ++)
    {
        temp_Addr <<= 8;
        EEPROM_SPI_ReadWriteByte(SysSPI,(uint8_t)(temp_Addr >> 24));
    }
    for(index = 0;index < 256;index ++)
    {
        EEPROM_SPI_ReadWriteByte(SysSPI,0xff);
    }
	
    GPIO_SetBits(GPIO_PORT_EEPROM_CS,GPIO_Pin_EEPROM_CS);         // CS = 1 
    SPI_Cmd(SysSPI, DISABLE);
    
    EEPROM_SPI_WriteDisable();
    Delay_ms(10);
#ifdef M95M01_Debug
    printf("EraseM95M02,OK,page_addr = %d...\r\n",page_addr);
#endif 
}
/*******************************************************************************
* Function Name  : M95M01HW_SelfTest
* Description    : EEPROM硬件检测，仅用于App发起的检测
* Input          : void
* Output         : None
* Return         : None
*******************************************************************************/
void M95M01HW_SelfTest(void)
{
    uint8_t wr_temp[20];
    uint8_t rd_temp[20] = {0};
    uint8_t index = 0;
	
	EEPROM_SPI_Configuration();
	
    for(index = 0;index < 20;index ++)
    {
        wr_temp[index] = index + 1;
    }
	EEPROM_SPI_WriteToMemory(wr_temp,M95M01_MONITOR_MODEL_OFFSET,20);

    Delay_ms(10);
	
	EEPROM_SPI_ReadFromMemory(rd_temp,M95M01_MONITOR_MODEL_OFFSET,20);
 
    for(index  = 0;index < 20;index ++)
    {
        if(rd_temp[index] != wr_temp[index])
        {
            break;
        }
    }
    if(index == 20)
    {
		SPI_AlarmTransmit(Alarm_ID_Reuse_EEPROM_CORRECT);  //EEPROM正确，暂时使用告警通道
    }
    else
    {
		SPI_AlarmTransmit(Alarm_ID_Reuse_EEPROM_ERROR);  //EEPROM错误，暂时使用告警通道
    }
}


