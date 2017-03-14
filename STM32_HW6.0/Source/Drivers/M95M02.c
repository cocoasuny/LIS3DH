#include "m95m02.h"
#include "stm32l1xx.h"
#include "SPI.h"
#include "common.h"

m95m02_info_t m95m02_info;

void M95m02_CounterCheckOut(void);
void M95m02_CounterUpdate(void);
void m95m02ReadOK(uint8_t data_id,uint8_t len,uint8_t state);
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
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; // (8M/32 =250KHz)
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
    
    EEPROM_SPI_ReadWriteByte(SysSPI,M95M02_RDSR);
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
    
    EEPROM_SPI_ReadWriteByte(SysSPI,M95M02_WREN);
    
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
    
    EEPROM_SPI_ReadWriteByte(SysSPI,M95M02_WRDI);

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

	u16TimeOut = EEPROM_TIMEOUT_CNT;
    while (--u16TimeOut != 0u &&(EEPROM_SPI_ReadStatusRegister() & M95M02_WIP_STATUS) == SET);
	
    EEPROM_SPI_WriteEnable();
    
    SPI_Cmd(SysSPI, ENABLE);
    GPIO_ResetBits(GPIO_PORT_EEPROM_CS,GPIO_Pin_EEPROM_CS);       // CS = 0

    EEPROM_SPI_ReadWriteByte(SysSPI,M95M02_WRITE);
    for(index = 0;index < 3;index ++)
    {
        temp_Addr <<= 8;
        EEPROM_SPI_ReadWriteByte(SysSPI,(uint8_t)(temp_Addr >> 24));
    }
    for(index = 0;index < len;index ++)
    {
        EEPROM_SPI_ReadWriteByte(SysSPI,*(pSrcStr + index));
    }
    
	u16TimeOut = EEPROM_TIMEOUT_CNT;
    while (--u16TimeOut != 0u &&(EEPROM_SPI_ReadStatusRegister() & M95M02_WIP_STATUS) == SET);
		
    GPIO_SetBits(GPIO_PORT_EEPROM_CS,GPIO_Pin_EEPROM_CS);         // CS = 1 
    SPI_Cmd(SysSPI, DISABLE);
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
    
    SPI_Cmd(SysSPI, ENABLE);
	GPIO_ResetBits(GPIO_PORT_EEPROM_CS,GPIO_Pin_EEPROM_CS);       // CS = 0

    EEPROM_SPI_ReadWriteByte(SysSPI,M95M02_READ);
    for(index = 0;index < 3;index ++)
    {
        temp_Addr <<= 8;
        EEPROM_SPI_ReadWriteByte(SysSPI,(uint8_t)(temp_Addr >> 24));
    }
    for(index = 0;index < (len);index ++)
    {
        *(pDestStr + index) = EEPROM_SPI_ReadWriteByte(SysSPI,0);
    }
    
    GPIO_SetBits(GPIO_PORT_EEPROM_CS,GPIO_Pin_EEPROM_CS);         // CS = 1
    SPI_Cmd(SysSPI, DISABLE);
}

/*******************************************************************************
* @brief   Translates 32-bit to 8-bit array.
* @param   u32Temp:the src data to be translated
* @param   pu8Array:the dest data addr
* @retval  void
*******************************************************************************/
void Translateu32Tou8Array(uint32_t u32Temp,uint8_t *pu8Array)
{
    *pu8Array = u32Temp;
    *(pu8Array + 1) = u32Temp >> 8;
    *(pu8Array + 2) = u32Temp >> 16;
    *(pu8Array + 3) = u32Temp >> 24;
}

/*******************************************************************************
* @brief   Translates 32-bit to 8-bit array.
* @param   u32Temp:the src data to be translated
* @param   pu8Array:the dest data addr
* @retval  void
*******************************************************************************/
uint32_t Translateu8ArrayTou32(uint8_t *pu8Array)
{
    return (((uint32_t)(*pu8Array)) + (((uint32_t)(*(pu8Array + 1))) >> 8) + 
            ((uint32_t)(*(pu8Array + 2)) >> 16) + ((uint32_t)(*(pu8Array + 3)) >> 24));
}

/*******************************************************************************
* @brief   Updates the counter in m95m02.
* @param   void
* @retval  void
*******************************************************************************/
void M95M02_CounterUpdate(void)
{
    uint8_t temp[40] = {0};
    
    Translateu32Tou8Array(m95m02_info.memory_info_contact_temp.pRead_m95m02,temp);
    Translateu32Tou8Array(m95m02_info.memory_info_contact_temp.pWrite_m95m02,&temp[4]);
    
    Translateu32Tou8Array(m95m02_info.memory_info_non_contact_temp.pRead_m95m02,&temp[8]);
    Translateu32Tou8Array(m95m02_info.memory_info_non_contact_temp.pWrite_m95m02,&temp[12]);
    
    Translateu32Tou8Array(m95m02_info.memory_info_steps.pRead_m95m02,&temp[16]);
    Translateu32Tou8Array(m95m02_info.memory_info_steps.pWrite_m95m02,&temp[20]);
    
    Translateu32Tou8Array(m95m02_info.memory_info_config_file.pRead_m95m02,&temp[24]);
    Translateu32Tou8Array(m95m02_info.memory_info_config_file.pWrite_m95m02,&temp[28]);
    
    Translateu32Tou8Array(m95m02_info.memory_info_monitor.pRead_m95m02,&temp[32]);
    Translateu32Tou8Array(m95m02_info.memory_info_monitor.pWrite_m95m02,&temp[36]); 
    EEPROM_SPI_WriteToMemory(temp,0,sizeof(temp));
	Delay_ms(10);
}

/*******************************************************************************
* @brief   Check out the counter in M95m02.
* @param   void
* @retval  void
*******************************************************************************/
void M95M02_CounterCheckOut(void)
{
    uint8_t temp[40] = {0};
    
    EEPROM_SPI_ReadFromMemory(temp,0,sizeof(temp));
    
    if((temp[3] == 0xFF) && (temp[11] == 0xFF) && (temp[19] == 0xFF))           // The first use
    {
        m95m02_info.memory_info_contact_temp.pRead_m95m02 = 0;
        m95m02_info.memory_info_contact_temp.pWrite_m95m02 = 0;
        
        m95m02_info.memory_info_non_contact_temp.pRead_m95m02 = 0;
        m95m02_info.memory_info_non_contact_temp.pWrite_m95m02 = 0;
        
        m95m02_info.memory_info_steps.pRead_m95m02 = 0;
        m95m02_info.memory_info_steps.pWrite_m95m02 = 0;
        
        m95m02_info.memory_info_config_file.pRead_m95m02 = 0;
        m95m02_info.memory_info_config_file.pWrite_m95m02 = 0;
        
        m95m02_info.memory_info_monitor.pRead_m95m02 = 0;
        m95m02_info.memory_info_monitor.pWrite_m95m02 = 0;
    }
    else
    {
        m95m02_info.memory_info_contact_temp.pRead_m95m02 = Translateu8ArrayTou32(temp);
        m95m02_info.memory_info_contact_temp.pWrite_m95m02 = Translateu8ArrayTou32(&temp[4]);
        
        m95m02_info.memory_info_non_contact_temp.pRead_m95m02 = Translateu8ArrayTou32(&temp[8]);
        m95m02_info.memory_info_non_contact_temp.pWrite_m95m02 = Translateu8ArrayTou32(&temp[12]);
        
        m95m02_info.memory_info_steps.pRead_m95m02 = Translateu8ArrayTou32(&temp[16]);
        m95m02_info.memory_info_steps.pWrite_m95m02 = Translateu8ArrayTou32(&temp[20]);
        
        m95m02_info.memory_info_config_file.pRead_m95m02 = Translateu8ArrayTou32(&temp[24]);
        m95m02_info.memory_info_config_file.pWrite_m95m02 = Translateu8ArrayTou32(&temp[28]);
        
        m95m02_info.memory_info_monitor.pRead_m95m02 = Translateu8ArrayTou32(&temp[32]);
        m95m02_info.memory_info_monitor.pWrite_m95m02 = Translateu8ArrayTou32(&temp[36]);
    }   
}

/*******************************************************************************
* @brief   Write M95M02.
* @param   data_id:the type of data to be writed
* @param   len:the length of data to be writed
* @param   ptr:the source to be writed
* @retval  true --- Success;false --- Fail
*******************************************************************************/
bool M95M02_Write(uint8_t data_id,uint8_t len,uint8_t * ptr)
{   
    uint32_t pStartAddr = 0;
    uint8_t temp_len = 0;
    uint32_t temp_space = 0;
    
    switch (data_id)
    {
        case CONTACT_TEMP_VALUE_ID:
            if(((m95m02_info.memory_info_contact_temp.pWrite_m95m02 + 1)%M95M02_PAGE_SPACE) ==
                 m95m02_info.memory_info_contact_temp.pRead_m95m02)// FIFO is full,must be read out.
            {
                return ERR_WRITE_EEPROM;
            }
            else            
            {
                if(m95m02_info.memory_info_contact_temp.pWrite_m95m02 > m95m02_info.memory_info_contact_temp.pRead_m95m02)
                {
                    temp_space = m95m02_info.memory_info_contact_temp.pWrite_m95m02 - 
                                            m95m02_info.memory_info_contact_temp.pRead_m95m02;
                }
                else
                {
                    temp_space = (m95m02_info.memory_info_contact_temp.pWrite_m95m02 + M95M02_PAGE_SPACE) - 
                                            m95m02_info.memory_info_contact_temp.pRead_m95m02;
                }
                
                if(temp_space < len)
                {
                    /* First moves the pointer pRead_m95m02 to (pRead_m95m02 + len) */
                    m95m02_info.memory_info_contact_temp.pRead_m95m02 += len;
                    m95m02_info.memory_info_contact_temp.pRead_m95m02 %= M95M02_PAGE_SPACE;
                }
                
                pStartAddr = M95M02_CONTACT_TEMP_OFFSET + m95m02_info.memory_info_contact_temp.pWrite_m95m02;
                EEPROM_SPI_WriteToMemory(ptr,pStartAddr,len);
                
                m95m02_info.memory_info_contact_temp.pWrite_m95m02 += len;
                m95m02_info.memory_info_contact_temp.pWrite_m95m02 %= M95M02_PAGE_SPACE;
                
                M95M02_CounterUpdate();
                return APP_SUCCESS;
            }
        
        case NON_CONTACT_TEMP_VALUE_ID:
            if(((m95m02_info.memory_info_non_contact_temp.pWrite_m95m02 + 1)%M95M02_PAGE_SPACE) ==
                 m95m02_info.memory_info_non_contact_temp.pRead_m95m02)// FIFO is full,must be read out.
            {
                return ERR_WRITE_EEPROM;
            }
            else            
            {
                if(m95m02_info.memory_info_non_contact_temp.pWrite_m95m02 > m95m02_info.memory_info_non_contact_temp.pRead_m95m02)
                {
                    temp_space = m95m02_info.memory_info_non_contact_temp.pWrite_m95m02 - 
                                            m95m02_info.memory_info_non_contact_temp.pRead_m95m02;
                }
                else
                {
                    temp_space = (m95m02_info.memory_info_non_contact_temp.pWrite_m95m02 + M95M02_PAGE_SPACE) - 
                                            m95m02_info.memory_info_non_contact_temp.pRead_m95m02;
                }
                
                if(temp_space < len)
                {
                    /* First moves the pointer pRead_m95m02 to (pRead_m95m02 + len) */
                    m95m02_info.memory_info_non_contact_temp.pRead_m95m02 += len;
                    m95m02_info.memory_info_non_contact_temp.pRead_m95m02 %= M95M02_PAGE_SPACE;
                }
                
                pStartAddr = M95M02_NON_CONTACT_TEMP_OFFSET + m95m02_info.memory_info_non_contact_temp.pWrite_m95m02;
                EEPROM_SPI_WriteToMemory(ptr,pStartAddr,len);
                
                m95m02_info.memory_info_non_contact_temp.pWrite_m95m02 += len;
                m95m02_info.memory_info_non_contact_temp.pWrite_m95m02 %= M95M02_PAGE_SPACE;
                
                M95M02_CounterUpdate();
                return APP_SUCCESS;
            }
        
        case STEP_COUNT_VALUE_ID: 
            if(((m95m02_info.memory_info_steps.pWrite_m95m02 + 1)%M95M02_PAGE_SPACE) ==
                 m95m02_info.memory_info_steps.pRead_m95m02)// FIFO is full,must be read out.
            {
                return ERR_WRITE_EEPROM;
            }
            else            
            {
                if(m95m02_info.memory_info_steps.pWrite_m95m02 > m95m02_info.memory_info_steps.pRead_m95m02)
                {
                    temp_space = m95m02_info.memory_info_steps.pWrite_m95m02 - 
                                            m95m02_info.memory_info_steps.pRead_m95m02;
                }
                else
                {
                    temp_space = (m95m02_info.memory_info_steps.pWrite_m95m02 + M95M02_PAGE_SPACE) - 
                                            m95m02_info.memory_info_steps.pRead_m95m02;
                }
                
                if(temp_space < len)
                {
                    /* First moves the pointer pRead_m95m02 to (pRead_m95m02 + len) */
                    m95m02_info.memory_info_steps.pRead_m95m02 += len;
                    m95m02_info.memory_info_steps.pRead_m95m02 %= M95M02_PAGE_SPACE;
                }
                
                pStartAddr = M95M02_STEP_COUNT_OFFSET + m95m02_info.memory_info_steps.pWrite_m95m02;
                EEPROM_SPI_WriteToMemory(ptr,pStartAddr,len);
                
                m95m02_info.memory_info_steps.pWrite_m95m02 += len;
                m95m02_info.memory_info_steps.pWrite_m95m02 %= M95M02_PAGE_SPACE;
                
                M95M02_CounterUpdate();
                return APP_SUCCESS;
            }
        
        case CONFIG_FILE_ID: 
            if(((m95m02_info.memory_info_config_file.pWrite_m95m02 + 1)%M95M02_PAGE_SPACE) ==
                 m95m02_info.memory_info_config_file.pRead_m95m02)// FIFO is full,must be read out.
            {
                return ERR_WRITE_EEPROM;
            }
            else            
            {
                if(m95m02_info.memory_info_config_file.pWrite_m95m02 > m95m02_info.memory_info_config_file.pRead_m95m02)
                {
                    temp_space = m95m02_info.memory_info_config_file.pWrite_m95m02 - 
                                            m95m02_info.memory_info_config_file.pRead_m95m02;
                }
                else
                {
                    temp_space = (m95m02_info.memory_info_config_file.pWrite_m95m02 + M95M02_PAGE_SPACE) - 
                                            m95m02_info.memory_info_config_file.pRead_m95m02;
                }
                
                if(temp_space < len)
                {
                    /* First moves the pointer pRead_m95m02 to (pRead_m95m02 + len) */
                    m95m02_info.memory_info_config_file.pRead_m95m02 += len;
                    m95m02_info.memory_info_config_file.pRead_m95m02 %= M95M02_PAGE_SPACE;
                }
                
                pStartAddr = M95M02_CONFIG_FILE_OFFSET + m95m02_info.memory_info_config_file.pWrite_m95m02;
                EEPROM_SPI_WriteToMemory(ptr,pStartAddr,len);
                
                m95m02_info.memory_info_config_file.pWrite_m95m02 += len;
                m95m02_info.memory_info_config_file.pWrite_m95m02 %= M95M02_PAGE_SPACE;
                
                M95M02_CounterUpdate();
                return APP_SUCCESS;
            }
        
        case MONITOR_MODEL_VALUE_ID:
            if(((m95m02_info.memory_info_monitor.pWrite_m95m02 + 1)%(M95M02_ALL_SPACE - M95M02_MONITOR_MODEL_OFFSET)) ==
                 m95m02_info.memory_info_monitor.pRead_m95m02)// FIFO is full,must be read out.
            {
                return ERR_WRITE_EEPROM;
            }
            else            
            {
                if(m95m02_info.memory_info_monitor.pWrite_m95m02 > m95m02_info.memory_info_monitor.pRead_m95m02)
                {
                    temp_space = m95m02_info.memory_info_monitor.pWrite_m95m02 - 
                                            m95m02_info.memory_info_monitor.pRead_m95m02;
                }
                else
                {
                    temp_space = (m95m02_info.memory_info_monitor.pWrite_m95m02 + (M95M02_ALL_SPACE - M95M02_MONITOR_MODEL_OFFSET)) - 
                                            m95m02_info.memory_info_monitor.pRead_m95m02;
                }
                
                if(temp_space < len)
                {
                    /* First moves the pointer pRead_m95m02 to (pRead_m95m02 + len) */
                    m95m02_info.memory_info_monitor.pRead_m95m02 += len;
                    m95m02_info.memory_info_monitor.pRead_m95m02 %= (M95M02_ALL_SPACE - M95M02_MONITOR_MODEL_OFFSET);
                }
                
                if(((m95m02_info.memory_info_monitor.pWrite_m95m02 + len) / M95M02_PAGE_SPACE) > 
                    m95m02_info.memory_info_monitor.pWrite_m95m02 / M95M02_PAGE_SPACE)
                {
                    pStartAddr = M95M02_MONITOR_MODEL_OFFSET + m95m02_info.memory_info_monitor.pWrite_m95m02;
                    temp_len = M95M02_PAGE_SPACE - (m95m02_info.memory_info_monitor.pWrite_m95m02 % M95M02_PAGE_SPACE);
                    EEPROM_SPI_WriteToMemory(ptr,pStartAddr,temp_len);
                    Delay_ms(10);
                    pStartAddr = M95M02_MONITOR_MODEL_OFFSET + 
                                 (((m95m02_info.memory_info_monitor.pWrite_m95m02 + len) / M95M02_PAGE_SPACE) * M95M02_PAGE_SPACE);
                    EEPROM_SPI_WriteToMemory(&(ptr[temp_len]),pStartAddr,(len - temp_len));
                }
                else            // In the same page
                {
                    pStartAddr = M95M02_MONITOR_MODEL_OFFSET + m95m02_info.memory_info_monitor.pWrite_m95m02;
                    EEPROM_SPI_WriteToMemory(ptr,pStartAddr,len);
                }
                
                m95m02_info.memory_info_monitor.pWrite_m95m02 += len;
                m95m02_info.memory_info_monitor.pWrite_m95m02 %= (M95M02_ALL_SPACE - M95M02_MONITOR_MODEL_OFFSET);
                
                M95M02_CounterUpdate();
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
bool M95M02_Read(uint8_t data_id,uint8_t len,uint8_t * ptr)
{   
    uint32_t pStartAddr = 0;
    uint32_t pStartAddr_temp = 0;
    uint32_t temp_space = 0,index;
     
    switch (data_id)
    {
        case CONTACT_TEMP_VALUE_ID:
            if(m95m02_info.memory_info_contact_temp.pRead_m95m02 != m95m02_info.memory_info_contact_temp.pWrite_m95m02)
            {
                if(m95m02_info.memory_info_contact_temp.pWrite_m95m02 > m95m02_info.memory_info_contact_temp.pRead_m95m02)
                {
                    temp_space = m95m02_info.memory_info_contact_temp.pWrite_m95m02 - 
                                            m95m02_info.memory_info_contact_temp.pRead_m95m02;
                }
                else
                {
                    temp_space = (m95m02_info.memory_info_contact_temp.pWrite_m95m02 + M95M02_PAGE_SPACE) - 
                                            m95m02_info.memory_info_contact_temp.pRead_m95m02;
                }
                
                if(temp_space < len)        // Invalid data
                {
                    pStartAddr = M95M02_CONTACT_TEMP_OFFSET + m95m02_info.memory_info_contact_temp.pRead_m95m02;
                    EEPROM_SPI_ReadFromMemory(ptr,pStartAddr,temp_space);
                    for(index = temp_space;index < len;index ++)
                    {
                         *(ptr + index) = *(ptr + temp_space - 1);
                    }
                    return ERR_READ_EEPROM;
                }
                else                // Direct Read
                {
                    if((m95m02_info.memory_info_contact_temp.pRead_m95m02 + len) > M95M02_PAGE_SPACE)
                    {
                        /* Need to read two times */
                        pStartAddr = M95M02_CONTACT_TEMP_OFFSET + m95m02_info.memory_info_contact_temp.pRead_m95m02;
                        pStartAddr_temp = (m95m02_info.memory_info_contact_temp.pRead_m95m02 + len) - M95M02_PAGE_SPACE;
                        EEPROM_SPI_ReadFromMemory(ptr,pStartAddr,pStartAddr_temp);
                        
                        pStartAddr = M95M02_CONTACT_TEMP_OFFSET;
                        EEPROM_SPI_ReadFromMemory(ptr + pStartAddr_temp,
                                                  M95M02_CONTACT_TEMP_OFFSET,
                                                 (m95m02_info.memory_info_contact_temp.pRead_m95m02 + len)%M95M02_PAGE_SPACE);
                    }
                    else
                    {
                        pStartAddr = M95M02_CONTACT_TEMP_OFFSET + m95m02_info.memory_info_contact_temp.pRead_m95m02;
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
            if(m95m02_info.memory_info_non_contact_temp.pRead_m95m02 != m95m02_info.memory_info_non_contact_temp.pWrite_m95m02)
            {
                if(m95m02_info.memory_info_non_contact_temp.pWrite_m95m02 > m95m02_info.memory_info_non_contact_temp.pRead_m95m02)
                {
                    temp_space = m95m02_info.memory_info_non_contact_temp.pWrite_m95m02 - 
                                            m95m02_info.memory_info_non_contact_temp.pRead_m95m02;
                }
                else
                {
                    temp_space = (m95m02_info.memory_info_non_contact_temp.pWrite_m95m02 + M95M02_PAGE_SPACE) - 
                                            m95m02_info.memory_info_non_contact_temp.pRead_m95m02;
                }
                
                if(temp_space < len)        // Invalid data
                {
                    pStartAddr = M95M02_NON_CONTACT_TEMP_OFFSET + m95m02_info.memory_info_non_contact_temp.pRead_m95m02;
                    EEPROM_SPI_ReadFromMemory(ptr,pStartAddr,temp_space);
                    for(index = temp_space;index < len;index ++)
                    {
                         *(ptr + index) = *(ptr + temp_space - 1);
                    }
                    return ERR_READ_EEPROM;
                }
                else                // Direct Read
                {
                    if((m95m02_info.memory_info_non_contact_temp.pRead_m95m02 + len) > M95M02_PAGE_SPACE)
                    {
                        /* Need to read two times */
                        pStartAddr = M95M02_NON_CONTACT_TEMP_OFFSET + m95m02_info.memory_info_non_contact_temp.pRead_m95m02;
                        pStartAddr_temp = (m95m02_info.memory_info_non_contact_temp.pRead_m95m02 + len) - M95M02_PAGE_SPACE;
                        EEPROM_SPI_ReadFromMemory(ptr,pStartAddr,pStartAddr_temp);
                        
                        pStartAddr = M95M02_CONTACT_TEMP_OFFSET;
                        EEPROM_SPI_ReadFromMemory(ptr + pStartAddr_temp,
                                                  M95M02_NON_CONTACT_TEMP_OFFSET,
                                                 (m95m02_info.memory_info_non_contact_temp.pRead_m95m02 + len)%M95M02_PAGE_SPACE);
                    }
                    else
                    {
                        pStartAddr = M95M02_NON_CONTACT_TEMP_OFFSET + m95m02_info.memory_info_non_contact_temp.pRead_m95m02;
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
            if(m95m02_info.memory_info_steps.pRead_m95m02 != m95m02_info.memory_info_steps.pWrite_m95m02)
            {
                if(m95m02_info.memory_info_steps.pWrite_m95m02 > m95m02_info.memory_info_steps.pRead_m95m02)
                {
                    temp_space = m95m02_info.memory_info_steps.pWrite_m95m02 - 
                                            m95m02_info.memory_info_steps.pRead_m95m02;
                }
                else
                {
                    temp_space = (m95m02_info.memory_info_steps.pWrite_m95m02 + M95M02_PAGE_SPACE) - 
                                            m95m02_info.memory_info_steps.pRead_m95m02;
                }
                
                if(temp_space < len)        // Invalid data
                {
                    pStartAddr = M95M02_STEP_COUNT_OFFSET + m95m02_info.memory_info_steps.pRead_m95m02;
                    EEPROM_SPI_ReadFromMemory(ptr,pStartAddr,temp_space);
                    for(index = temp_space;index < len;index ++)
                    {
                         *(ptr + index) = *(ptr + temp_space - 1);
                    }
                    return ERR_READ_EEPROM;
                }
                else                // Direct Read
                {
                    if((m95m02_info.memory_info_steps.pRead_m95m02 + len) > M95M02_PAGE_SPACE)
                    {
                        /* Need to read two times */
                        pStartAddr = M95M02_STEP_COUNT_OFFSET + m95m02_info.memory_info_steps.pRead_m95m02;
                        pStartAddr_temp = (m95m02_info.memory_info_steps.pRead_m95m02 + len) - M95M02_PAGE_SPACE;
                        EEPROM_SPI_ReadFromMemory(ptr,pStartAddr,pStartAddr_temp);
                        
                        pStartAddr = M95M02_CONTACT_TEMP_OFFSET;
                        EEPROM_SPI_ReadFromMemory(ptr + pStartAddr_temp,
                                                  M95M02_STEP_COUNT_OFFSET,
                                                 (m95m02_info.memory_info_steps.pRead_m95m02 + len)%M95M02_PAGE_SPACE);
                    }
                    else
                    {
                        pStartAddr = M95M02_STEP_COUNT_OFFSET + m95m02_info.memory_info_steps.pRead_m95m02;
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
            if(m95m02_info.memory_info_config_file.pRead_m95m02 != m95m02_info.memory_info_config_file.pWrite_m95m02)
            {
                if(m95m02_info.memory_info_config_file.pWrite_m95m02 > m95m02_info.memory_info_config_file.pRead_m95m02)
                {
                    temp_space = m95m02_info.memory_info_config_file.pWrite_m95m02 - 
                                            m95m02_info.memory_info_config_file.pRead_m95m02;
                }
                else
                {
                    temp_space = (m95m02_info.memory_info_config_file.pWrite_m95m02 + M95M02_PAGE_SPACE) - 
                                            m95m02_info.memory_info_config_file.pRead_m95m02;
                }
                
                if(temp_space < len)        // Invalid data
                {
                    pStartAddr = M95M02_CONFIG_FILE_OFFSET + m95m02_info.memory_info_config_file.pRead_m95m02;
                    EEPROM_SPI_ReadFromMemory(ptr,pStartAddr,temp_space);
                    for(index = temp_space;index < len;index ++)
                    {
                         *(ptr + index) = *(ptr + temp_space - 1);
                    }
                    return ERR_READ_EEPROM;
                }
                else                // Direct Read
                {
                    if((m95m02_info.memory_info_steps.pRead_m95m02 + len) > M95M02_PAGE_SPACE)
                    {
                        /* Need to read two times */
                        pStartAddr = M95M02_CONFIG_FILE_OFFSET + m95m02_info.memory_info_config_file.pRead_m95m02;
                        pStartAddr_temp = (m95m02_info.memory_info_config_file.pRead_m95m02 + len) - M95M02_PAGE_SPACE;
                        EEPROM_SPI_ReadFromMemory(ptr,pStartAddr,pStartAddr_temp);
                        
                        pStartAddr = M95M02_CONFIG_FILE_OFFSET;
                        EEPROM_SPI_ReadFromMemory(ptr + pStartAddr_temp,
                                                  M95M02_CONFIG_FILE_OFFSET,
                                                 (m95m02_info.memory_info_config_file.pRead_m95m02 + len)%M95M02_PAGE_SPACE);
                    }
                    else
                    {
                        pStartAddr = M95M02_CONFIG_FILE_OFFSET + m95m02_info.memory_info_config_file.pRead_m95m02;
                        EEPROM_SPI_ReadFromMemory(ptr,pStartAddr,len);
                    }
                    return APP_SUCCESS;
                }
            }
            else                                        // FIFO is empty
            {
                return ERR_READ_EEPROM;
            }
        
        case MONITOR_MODEL_VALUE_ID:
            if(m95m02_info.memory_info_monitor.pRead_m95m02 != m95m02_info.memory_info_monitor.pWrite_m95m02)
            {
                if(m95m02_info.memory_info_monitor.pWrite_m95m02 > m95m02_info.memory_info_monitor.pRead_m95m02)
                {
                    temp_space = m95m02_info.memory_info_monitor.pWrite_m95m02 - 
                                            m95m02_info.memory_info_monitor.pRead_m95m02;
                }
                else
                {
                    temp_space = (m95m02_info.memory_info_monitor.pWrite_m95m02 + (M95M02_ALL_SPACE - M95M02_MONITOR_MODEL_OFFSET)) - 
                                            m95m02_info.memory_info_monitor.pRead_m95m02;
                }
                
                if(temp_space < len)        // Invalid data
                {
                    pStartAddr = M95M02_MONITOR_MODEL_OFFSET + m95m02_info.memory_info_monitor.pRead_m95m02;
                    EEPROM_SPI_ReadFromMemory(ptr,pStartAddr,temp_space);
                    for(index = temp_space;index < len;index ++)
                    {
                         *(ptr + index) = *(ptr + temp_space - 1);
                    }
                    return ERR_READ_EEPROM;
                }
                else                // Direct Read
                {
                    if((m95m02_info.memory_info_monitor.pRead_m95m02 + len) > (M95M02_ALL_SPACE - M95M02_MONITOR_MODEL_OFFSET))
                    {
                        /* Need to read two times */
                        pStartAddr = M95M02_MONITOR_MODEL_OFFSET + m95m02_info.memory_info_monitor.pRead_m95m02;
                        pStartAddr_temp = (m95m02_info.memory_info_monitor.pRead_m95m02 + len) - (M95M02_ALL_SPACE - M95M02_MONITOR_MODEL_OFFSET);
                        EEPROM_SPI_ReadFromMemory(ptr,pStartAddr,pStartAddr_temp);
                        
                        pStartAddr = M95M02_MONITOR_MODEL_OFFSET;
                        EEPROM_SPI_ReadFromMemory(ptr + pStartAddr_temp,
                                                  M95M02_MONITOR_MODEL_OFFSET,
                                                 (m95m02_info.memory_info_monitor.pRead_m95m02 + len)%(M95M02_ALL_SPACE - M95M02_MONITOR_MODEL_OFFSET));
                    }
                    else
                    {
                        pStartAddr = M95M02_MONITOR_MODEL_OFFSET + m95m02_info.memory_info_monitor.pRead_m95m02;
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
void M95M02_ReadOK(uint8_t data_id,uint8_t len,uint8_t state)
{
    if(state)                                               // Success
    {   
        switch (data_id)
        {
            case CONTACT_TEMP_VALUE_ID:
                m95m02_info.memory_info_contact_temp.pRead_m95m02 += len;
                m95m02_info.memory_info_contact_temp.pRead_m95m02 %= M95M02_PAGE_SPACE;
                M95M02_CounterUpdate();
                break;
            
            case NON_CONTACT_TEMP_VALUE_ID:
                m95m02_info.memory_info_non_contact_temp.pRead_m95m02 += len;
                m95m02_info.memory_info_non_contact_temp.pRead_m95m02 %= M95M02_PAGE_SPACE;
                M95M02_CounterUpdate();
                break;
            
            case STEP_COUNT_VALUE_ID: 
                m95m02_info.memory_info_steps.pRead_m95m02 += len;
                m95m02_info.memory_info_steps.pRead_m95m02 %= M95M02_PAGE_SPACE;
                M95M02_CounterUpdate();
                break;
            
            case CONFIG_FILE_ID: 
                m95m02_info.memory_info_config_file.pRead_m95m02 += len;
                m95m02_info.memory_info_config_file.pRead_m95m02 %= M95M02_PAGE_SPACE;
                M95M02_CounterUpdate();
                break;
            
            case MONITOR_MODEL_VALUE_ID:
                m95m02_info.memory_info_monitor.pRead_m95m02 += len;
                m95m02_info.memory_info_monitor.pRead_m95m02 %= (M95M02_ALL_SPACE - M95M02_MONITOR_MODEL_OFFSET);
                M95M02_CounterUpdate();
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
                   M95M02_CAPACITY_SPACE,M95M02_IS_BUSY,M95M02_READ_AVAILABLE_SPACE
* @retval  
*******************************************************************************/
uint32_t GetM95M02State(uint8_t data_id,uint8_t getType)
{
	uint8_t Err_Code=0;
    switch (data_id)
    {
        case CONTACT_TEMP_VALUE_ID:
            if(getType == M95M02_CAPACITY_SPACE)
            {
                if(m95m02_info.memory_info_contact_temp.pRead_m95m02 <= m95m02_info.memory_info_contact_temp.pWrite_m95m02)
                {
                    return (m95m02_info.memory_info_contact_temp.pRead_m95m02 + M95M02_PAGE_SPACE - 
                            m95m02_info.memory_info_contact_temp.pWrite_m95m02);
                }
                else
                {
                    return (m95m02_info.memory_info_contact_temp.pRead_m95m02 - 
                            m95m02_info.memory_info_contact_temp.pWrite_m95m02);
                }
            }
            else if(getType == M95M02_IS_BUSY)
            {
                if((EEPROM_SPI_ReadStatusRegister() & M95M02_WIP_STATUS))           // Busy
                {
                    return false;
                }    
                else
                {
                    return true;
                }
            }
            else if(getType == M95M02_READ_AVAILABLE_SPACE)
            {
                if(m95m02_info.memory_info_contact_temp.pRead_m95m02 > m95m02_info.memory_info_contact_temp.pWrite_m95m02)
                {
                    return (m95m02_info.memory_info_contact_temp.pWrite_m95m02 + M95M02_PAGE_SPACE - 
                            m95m02_info.memory_info_contact_temp.pRead_m95m02);
                }
                else
                {
                    return (m95m02_info.memory_info_contact_temp.pWrite_m95m02 - 
                            m95m02_info.memory_info_contact_temp.pRead_m95m02);
                }
            }
            else
            {}
            break;
        
        case NON_CONTACT_TEMP_VALUE_ID:
            if(getType == M95M02_CAPACITY_SPACE)
            {
                if(m95m02_info.memory_info_non_contact_temp.pRead_m95m02 <= m95m02_info.memory_info_non_contact_temp.pWrite_m95m02)
                {
                    return (m95m02_info.memory_info_non_contact_temp.pRead_m95m02 + M95M02_PAGE_SPACE - 
                            m95m02_info.memory_info_non_contact_temp.pWrite_m95m02);
                }
                else
                {
                    return (m95m02_info.memory_info_non_contact_temp.pRead_m95m02 - 
                            m95m02_info.memory_info_non_contact_temp.pWrite_m95m02);
                }
            }
            else if(getType == M95M02_IS_BUSY)
            {
                if((EEPROM_SPI_ReadStatusRegister() & M95M02_WIP_STATUS))           // Busy
                {
                    return false;
                }    
                else
                {
                    return true;
                }
            }
            else if(getType == M95M02_READ_AVAILABLE_SPACE)
            {
                if(m95m02_info.memory_info_non_contact_temp.pRead_m95m02 > m95m02_info.memory_info_non_contact_temp.pWrite_m95m02)
                {
                    return (m95m02_info.memory_info_non_contact_temp.pWrite_m95m02 + M95M02_PAGE_SPACE - 
                            m95m02_info.memory_info_non_contact_temp.pRead_m95m02);
                }
                else
                {
                    return (m95m02_info.memory_info_non_contact_temp.pWrite_m95m02 - 
                            m95m02_info.memory_info_non_contact_temp.pRead_m95m02);
                }
            }
            else
            {}
            break;
        
        case STEP_COUNT_VALUE_ID: 
            if(getType == M95M02_CAPACITY_SPACE)
            {
                if(m95m02_info.memory_info_steps.pRead_m95m02 <= m95m02_info.memory_info_steps.pWrite_m95m02)
                {
                    return (m95m02_info.memory_info_steps.pRead_m95m02 + M95M02_PAGE_SPACE - 
                            m95m02_info.memory_info_steps.pWrite_m95m02);
                }
                else
                {
                    return (m95m02_info.memory_info_steps.pRead_m95m02 - 
                            m95m02_info.memory_info_steps.pWrite_m95m02);
                }
            }
            else if(getType == M95M02_IS_BUSY)
            {
                if((EEPROM_SPI_ReadStatusRegister() & M95M02_WIP_STATUS))           // Busy
                {
                    return false;
                }    
                else
                {
                    return true;
                }
            }
            else if(getType == M95M02_READ_AVAILABLE_SPACE)
            {
                if(m95m02_info.memory_info_steps.pRead_m95m02 > m95m02_info.memory_info_steps.pWrite_m95m02)
                {
                    return (m95m02_info.memory_info_steps.pWrite_m95m02 + M95M02_PAGE_SPACE - 
                            m95m02_info.memory_info_steps.pRead_m95m02);
                }
                else
                {
                    return (m95m02_info.memory_info_steps.pWrite_m95m02 - 
                            m95m02_info.memory_info_steps.pRead_m95m02);
                }
            }
            else
            {}
            break;
        
        case CONFIG_FILE_ID: 
            if(getType == M95M02_CAPACITY_SPACE)
            {
                if(m95m02_info.memory_info_config_file.pRead_m95m02 <= m95m02_info.memory_info_config_file.pWrite_m95m02)
                {
                    return (m95m02_info.memory_info_config_file.pRead_m95m02 + M95M02_PAGE_SPACE - 
                            m95m02_info.memory_info_config_file.pWrite_m95m02);
                }
                else
                {
                    return (m95m02_info.memory_info_config_file.pRead_m95m02 - 
                            m95m02_info.memory_info_config_file.pWrite_m95m02);
                }
            }
            else if(getType == M95M02_IS_BUSY)
            {
                if((EEPROM_SPI_ReadStatusRegister() & M95M02_WIP_STATUS))           // Busy
                {
                    return false;
                }    
                else
                {
                    return true;
                }
            }
            else if(getType == M95M02_READ_AVAILABLE_SPACE)
            {
                if(m95m02_info.memory_info_config_file.pRead_m95m02 > m95m02_info.memory_info_config_file.pWrite_m95m02)
                {
                    return (m95m02_info.memory_info_config_file.pWrite_m95m02 + M95M02_PAGE_SPACE - 
                            m95m02_info.memory_info_config_file.pRead_m95m02);
                }
                else
                {
                    return (m95m02_info.memory_info_config_file.pWrite_m95m02 - 
                            m95m02_info.memory_info_config_file.pRead_m95m02);
                }
            }
            else
            {}
            break;
        
        case MONITOR_MODEL_VALUE_ID:
			#ifdef EEPROM_DEBUG
				printf("Get Type=%d\r\n",getType);
				printf("Read m95m02 = %x\r\n",m95m02_info.memory_info_monitor.pRead_m95m02);
				printf("Write m95m02 = %x\r\n",m95m02_info.memory_info_monitor.pWrite_m95m02);
			#endif 
            if(getType == M95M02_CAPACITY_SPACE)
            {
                if(m95m02_info.memory_info_monitor.pRead_m95m02 <= m95m02_info.memory_info_monitor.pWrite_m95m02)
                {
                    return (m95m02_info.memory_info_monitor.pRead_m95m02 + (M95M02_ALL_SPACE - M95M02_MONITOR_MODEL_OFFSET) - 
                            m95m02_info.memory_info_monitor.pWrite_m95m02);
                }
                else
                {
                    return (m95m02_info.memory_info_monitor.pRead_m95m02 - 
                            m95m02_info.memory_info_monitor.pWrite_m95m02);
                }
            }
            else if(getType == M95M02_IS_BUSY)
            {
				Err_Code = EEPROM_SPI_ReadStatusRegister();
                if((Err_Code & M95M02_WIP_STATUS))           // Busy
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
            else if(getType == M95M02_READ_AVAILABLE_SPACE)
            {
                if(m95m02_info.memory_info_monitor.pRead_m95m02 > m95m02_info.memory_info_monitor.pWrite_m95m02)
                {
                    return (m95m02_info.memory_info_monitor.pWrite_m95m02 + (M95M02_ALL_SPACE - M95M02_MONITOR_MODEL_OFFSET) - 
                            m95m02_info.memory_info_monitor.pRead_m95m02);
                }
                else
                {
                    return (m95m02_info.memory_info_monitor.pWrite_m95m02 - 
                            m95m02_info.memory_info_monitor.pRead_m95m02);
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
void ClearM95M02(uint8_t page_addr)
{
    uint16_t index      = 0;
    uint32_t temp_Addr  = page_addr << 8;
	uint8_t WriteEEPROMTimeOutCnt = 80;

	while((WriteEEPROMTimeOutCnt !=0) && GetM95M02State(MONITOR_MODEL_VALUE_ID,M95M02_IS_BUSY) == false)
	{
		WriteEEPROMTimeOutCnt--;
		Delay_ms(10);
	}
 
    EEPROM_SPI_WriteEnable();
    
    SPI_Cmd(SysSPI, ENABLE);
    GPIO_ResetBits(GPIO_PORT_EEPROM_CS,GPIO_Pin_EEPROM_CS);       // CS = 0

    EEPROM_SPI_ReadWriteByte(SysSPI,M95M02_WRITE);
    for(index = 0;index < 3;index ++)
    {
        temp_Addr <<= 8;
        EEPROM_SPI_ReadWriteByte(SysSPI,(uint8_t)(temp_Addr >> 24));
    }
    for(index = 0;index < 256;index ++)
    {
        EEPROM_SPI_ReadWriteByte(SysSPI,0);
    }
	
    EEPROM_SPI_WriteDisable();
	
    GPIO_SetBits(GPIO_PORT_EEPROM_CS,GPIO_Pin_EEPROM_CS);         // CS = 1 
    SPI_Cmd(SysSPI, DISABLE);
}
