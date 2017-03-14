/*
* Copyright(c) 2015, CloudCare Healthcare Co.,Ltd
* All rights reserved
* File Name:                arrayOperaFunction.h
* Descprition:              the functions for array operations
* Created Date:             2016/03/11
* Author:                   yun.zhu@ywkang.com  
* Current Reversion:        V1.0
* Reversion History:
*   Rev *.*    
*     Author:     
*     Change Date:
*     Change List:
*/

#ifndef _ARRAY_FUNCTIONS_H
#define _ARRAY_FUNCTIONS_H

#include "cc_alg.h"


/*  structure for uint8_t data      */
typedef struct{
    uint8_t     *       m_pui8Data;
    uint16_t            m_ui16DataLen;
    uint16_t            m_ui16DataInBuf;
} st_Arr_ui8_Typedef;

/*  structure for uint16_t data      */
typedef struct{
    uint16_t     *       m_pui16Data;
    uint16_t            m_ui16DataLen;
    uint16_t            m_ui16DataInBuf;
} st_Arr_ui16_Typedef;

/*  structure for int32_t data      */
typedef struct{
    int32_t     *       m_pi32Data;
    uint16_t            m_ui16DataLen;
    uint16_t            m_ui16DataInBuf;
} st_Arr_i32_Typedef;


/*  functions definition            */
FUNC_RETURN_TYPE arr_init_func_ui8(st_Arr_ui8_Typedef * ui8ArrDat_pst, uint8_t * pui8DataArr, uint16_t ui16DataArrLen);
FUNC_RETURN_TYPE arr_push_data_func_ui8(st_Arr_ui8_Typedef * ui8ArrDat_pst, uint8_t * pui8DataIn, uint16_t ui16DataInLen);
FUNC_RETURN_TYPE arr_get_mean_func_ui8(st_Arr_ui8_Typedef * ui8ArrDat_pst, uint8_t * pui8DatAve);


/*  functions definition            */
FUNC_RETURN_TYPE arr_init_func_ui16(st_Arr_ui16_Typedef * ui16ArrDat_pst, uint16_t * pui16DataArr, uint16_t ui16DataArrLen);
FUNC_RETURN_TYPE arr_push_data_func_ui16(st_Arr_ui16_Typedef * ui16ArrDat_pst, uint16_t * pui16DataIn, uint16_t ui16DataInLen);
FUNC_RETURN_TYPE arr_get_mean_func_ui16(st_Arr_ui16_Typedef * ui16ArrDat_pst, uint16_t * pui16DatAve);

/*  functions definition            */
FUNC_RETURN_TYPE arr_init_func_i32(st_Arr_i32_Typedef * i32ArrDat_pst, int32_t * pi32DataArr, uint16_t ui16DataArrLen);
FUNC_RETURN_TYPE arr_push_data_func_i32(st_Arr_i32_Typedef * i32ArrDat_pst, int32_t * pi32DataIn, uint16_t ui16DataInLen);
FUNC_RETURN_TYPE arr_get_mean_func_i32(st_Arr_i32_Typedef * i32ArrDat_pst, int32_t * pi32DatAve);


#endif //end of _ARRAY_FUNCTIONS_H
//end file
