/*
* Copyright(c) 2015, CloudCare Healthcare Co.,Ltd
* All rights reserved
* File Name:                _CC_APP_GSENSOR_DRV_INTERFACE.h
* Descprition:              the _CC_APP_GSENSOR_DRV_INTERFACE.h files for translate layer
* Created Date:             2016/03/18
* Author:                   yun.zhu@ywkang.com  
* Current Reversion:        V1.0
* Reversion History:
*   Rev *.*    
*     Author:     
*     Change Date:
*     Change List:
*/

#ifndef _CC_APP_GSENSOR_DRV_INTERFACE
#define _CC_APP_GSENSOR_DRV_INTERFACE

#include "bma250e.h"
#include "bma2x2.h"
#include "Process_Acc_Data.h"

#define GSEN_MOVECHECK_START_CNT_LIMIT 		(3)




void cc_app_gsensor_drv_interface_init(void);
void cc_sys_gsensor_spo2_motion_notice_handle(bool isMotionCheckPass);




#endif // _CC_APP_GSENSOR_DRV_INTERFACE

// end file
