#include "call.h"
//#include "common.h"

void Call_Handler(event_t event)
{
	static uint8_t VibrateSwitchStatus = DISABLE;
	
    switch(event)
    {
        case gCallIncomingCallAdded:
            #ifdef ANCS_DEBUG
				printf("Handler_INCOMING_CALL,Added...\r\n");	
			#endif
			
            Device_Mode_pre = Device_Mode;
            Device_Mode = Device_Mode_IncomingCall;
        
			VibrateSwitchStatus = MonitorTemplate.VibrateSwitch;  //Save Vibrate Status
			MonitorTemplate.VibrateSwitch = ENABLE;   //Enable Virbrate
			StartMotor(5);
		
			OLED_DisplayInconmingCallFlash(ON);
            break;
        
        case gCallIncomingCallRemoved:
			#ifdef ANCS_DEBUG
				printf("Handler_INCOMING_CALL,Removed...\r\n");	
			#endif
		
			MonitorTemplate.VibrateSwitch = VibrateSwitchStatus;  //Recover the status of Virbrate
			KillMotor();
			/* Add code here to send event for Call Removed */
			//if(OccupiedDisCall == OLEDDisplay_Stat_Get())  
			{
//				TS_SendEvent(gOledDisTaskID,gOledDisEventDisplayAllRAM_c); 
//				Set_OLED_Dis_Status(OLEDDisON);		
				OLED_DisplayInconmingCallFlash(OFF);
				TS_SendEvent(gOledDisTaskID,gOledDisEventClearModeDis_c);  //关闭屏幕显示计时
			    Device_Mode = Device_Mode_pre;
			}
            break;
        
        default:
            break; 
    }      
}
