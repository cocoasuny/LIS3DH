/*
 * COPYRIGHT (c) 2010-2014 MACRONIX INTERNATIONAL CO., LTD
 * SPI Flash Low Level Driver (LLD) Sample Code
 *
 * SPI interface command hex code, type definition and function prototype.
 *
 * $Id: MX25_CMD.h,v 1.20 2013/11/08 01:41:48 modelqa Exp $
 */
#ifndef    __MX25L1606E_H__
#define    __MX25L1606E_H__

#include "MX25_DEF.h"
#include "stm32l1xx.h"


/* EEPROM SPI define */
#define RCC_AHBPeriph_FLASH_CS         		PT_RCC_AHBPeriph_FLASH_CS
#define GPIO_PORT_FLASH_CS                 	PT_GPIO_PORT_FLASH_CS
#define GPIO_Pin_FLASH_CS                  	PT_GPIO_Pin_FLASH_CS


//typedef enum{MX25_SUCCESS,MX25_WRITE_ERROR,MX25_READ_ERROR}MX25_Status;

/*** MX25 series command hex code definition ***/
//ID comands
#define    FLASH_CMD_RDID      				0x9F    //RDID (Read Identification)
#define    FLASH_CMD_RES       				0xAB    //RES (Read Electronic ID)
#define    FLASH_CMD_REMS      				0x90    //REMS (Read Electronic & Device ID)

//Register comands
#define    FLASH_CMD_WRSR      				0x01    //WRSR (Write Status Register)
#define    FLASH_CMD_RDSR      				0x05    //RDSR (Read Status Register)
#define    FLASH_CMD_WRSCUR    				0x2F    //WRSCUR (Write Security Register)
#define    FLASH_CMD_RDSCUR    				0x2B    //RDSCUR (Read Security Register)

//READ comands
#define    FLASH_CMD_READ        			0x03    //READ (1 x I/O)
#define    FLASH_CMD_FASTREAD    			0x0B    //FAST READ (Fast read data)
#define    FLASH_CMD_DREAD       			0x3B    //DREAD (1In/2 Out fast read)
#define    FLASH_CMD_RDSFDP      			0x5A    //RDSFDP (Read SFDP)

//Program comands
#define    FLASH_CMD_WREN     				0x06    //WREN (Write Enable)
#define    FLASH_CMD_WRDI     				0x04    //WRDI (Write Disable)
#define    FLASH_CMD_PP       				0x02    //PP (page program)

//Erase comands
#define    FLASH_CMD_SE       				0x20    //SE (Sector Erase)
#define    FLASH_CMD_BE       				0xD8    //BE (Block Erase)
#define    FLASH_CMD_CE       				0x60    //CE (Chip Erase) hex code: 60 or C7

//Mode setting comands
#define    FLASH_CMD_DP       				0xB9    //DP (Deep Power Down)
#define    FLASH_CMD_RDP      				0xAB    //RDP (Release form Deep Power Down)
#define    FLASH_CMD_ENSO     				0xB1    //ENSO (Enter Secured OTP)
#define    FLASH_CMD_EXSO     				0xC1    //EXSO  (Exit Secured OTP)
#ifdef SBL_CMD_0x77
#else
#endif

//Reset comands

//Security comands
#ifdef LCR_CMD_0xDD_0xD5
#else
#endif

//Suspend/Resume comands

// Return Message
typedef enum {
    FlashOperationSuccess,
    FlashWriteRegFailed,
    FlashTimeOut,
    FlashIsBusy,
    FlashQuadNotEnable,
    FlashAddressInvalid
}ReturnMsg;

// Flash status structure define
struct sFlashStatus{
    /* Mode Register:
     * Bit  Description
     * -------------------------
     *  7   RYBY enable
     *  6   Reserved
     *  5   Reserved
     *  4   Reserved
     *  3   Reserved
     *  2   Reserved
     *  1   Parallel mode enable
     *  0   QPI mode enable
    */
    uint8    ModeReg;
    BOOL     ArrangeOpt;
};

typedef struct sFlashStatus FlashStatus;

void Flash_PowerCtl(uint8_t power);
void MX25_GPIO_DeInit(void);
void MX25_SPI_Configuration(void);
void MX25L1606EHW_SelfTest(void);

/* Basic functions */
void MX25_CS_High(void);
void MX25_CS_Low(void);
void InsertDummyCycle( uint8 dummy_num );
ReturnMsg MX25_SPI_ReadWriteByte(SPI_TypeDef* SPIx,uint8_t tx_temp,uint8_t * rx_temp);


/* Utility functions */
void Wait_Flash_WarmUp(void);
void Initial_Spi(void);
BOOL WaitFlashReady( uint32 ExpectTime );
BOOL WaitRYBYReady( uint32 ExpectTime );
BOOL IsFlashBusy( void );
BOOL IsFlashQIO( void );
BOOL IsFlash4Byte( void );
void SendFlashAddr( uint32 flash_address, uint8 io_mode, uint8 addr_mode );
uint8 GetDummyCycle( uint32 default_cycle );

/* Flash commands */
ReturnMsg CMD_ReadID( uint32 *Identification );
ReturnMsg CMD_RES( uint8 *ElectricIdentification );
ReturnMsg CMD_REMS( uint16 *REMS_Identification, FlashStatus *fsptr );

ReturnMsg CMD_RDSR( uint8 *StatusReg );
#ifdef SUPPORT_WRSR_CR
   ReturnMsg CMD_WRSR( uint16 UpdateValue );
#else
   ReturnMsg CMD_WRSR( uint8 UpdateValue );
#endif
ReturnMsg CMD_RDSCUR( uint8 *SecurityReg );
ReturnMsg CMD_WRSCUR( void );

ReturnMsg CMD_READ( uint32 flash_address, uint8 *target_address, uint32 byte_length  );
ReturnMsg CMD_FASTREAD( uint32 flash_address, uint8 *target_address, uint32 byte_length );
ReturnMsg CMD_DREAD( uint32 flash_address, uint8 *target_address, uint32 byte_length );
ReturnMsg CMD_RDSFDP( uint32 flash_address, uint8 *target_address, uint32 byte_length );

ReturnMsg CMD_WREN( void );
ReturnMsg CMD_WRDI( void );
ReturnMsg CMD_PageProgram( uint32 flash_address, void * source_address, uint32 byte_length );
ReturnMsg CMD_SE( uint32 flash_address );
ReturnMsg CMD_BE( uint32 flash_address );
ReturnMsg CMD_CE( void );

ReturnMsg CMD_DP( void );
ReturnMsg CMD_RDP( void );
ReturnMsg CMD_ENSO( void );
ReturnMsg CMD_EXSO( void );


#endif    /* __MX25_CMD_H__ */
