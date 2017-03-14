#include "DeviceCtl.h"

void DeviceControl_Handler(event_t DevCtl_Event)
{
	if(DevCtl_Event & gDevCtlAppModifySNEvent)  //App Modify the SN
	{
		StoreAppModifySN();
	}
    else if(DevCtl_Event & gDevCtlAppModifyWorkModeEvent)
    {
        SetDeviceModeConfigInfo();
    }
//	else if()
//	{

//	}
		
}

/*******************************************************************************
* Function Name  : StoreAppModifySN
* Description    : StoreAppModifySN
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void StoreAppModifySN(void)
{
	SysConfigInfo_t DeviceCFInfo;      //Device configure information
	
	/* 	Read device config file from DataMemory	*/
    GetSysConfigInfo(&DeviceCFInfo);
	
	/* Store App Modify SN info */
	DeviceCFInfo.AppModifySNStatus = M95M01_APP_MODIFYSN;  //flag of App is to modify the SN 	
	DeviceCFInfo.AppModifySNInfo[0] = u8AppModifySeriNum[0];
	DeviceCFInfo.AppModifySNInfo[1] = u8AppModifySeriNum[1];
	DeviceCFInfo.AppModifySNInfo[2] = u8AppModifySeriNum[2];
	DeviceCFInfo.AppModifySNInfo[3] = u8AppModifySeriNum[3];	
	
	/* 	Write Modify SN status into EEPROM */
    SetSysConfigInfo(DeviceCFInfo);
	
	AppModifySN();
}

