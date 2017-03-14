#ifndef __APPERROR_H
#define __APPERROR_H

#include "common.h"

#define 	APP_SUCCESS          0x00
#define     ERR_WRITE_EEPROM     0x01
#define     ERR_READ_EEPROM      0x02

#define APP_ERROR_CHECK(ERR_CODE)                           \
    do                                                      \
    {                                                       \
        const uint8_t LOCAL_ERR_CODE = (ERR_CODE);         \
        if (LOCAL_ERR_CODE != APP_SUCCESS)                  \
        {                                                   \
            printf("Err_Code=%x\r\n",ERR_CODE);             \
			printf("file=%s,func=%s,line=%d\r\n",__FILE__,__FUNCTION__,__LINE__); \
        }                                                   \
    } while (0)         


#endif /* __APPERROR_H */

