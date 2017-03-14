#include "stm32l1xx.h"
#include "Usart.h"
  
#define uchar unsigned char    
#define uint unsigned int    
#define Ntp 141  				//Ntp is the number of Vtp_r25 table  
#define Nntc 141					//Nntc is the number of Rtx_ref table
#define T25 25						//reference temperature
#define TCsens -0.0005			//Temperature Coefficient of Thermopile Sensitivity
#define Sconv 1.05				//Sensitivity Conversion Factor
#define R25 96.8					//R25 is the standand DC impedance at 25 degree under testing; Unit is Kohm
#define Rref 50.2					//Rref is the serial resistance with the NTC in the thermopile;  Unit is Kohm	
#define Vref 0.995

//float R25 = 96.8;     //R25 is the standand DC impedance at 25 degree under testing; Unit is Kohm
//float T25 = 25;       //reference temperature.
//float Sconv = 0.982;  //Sensitivity Conversion Factor 
//float TCsens = 0.005; //Temperature Coefficient of Thermopile Sensitivity 
//float TCF;   					//TCF is TC Factor which is TCF = 1+DeltaT*TCsens
//float Vtp_offs;				//Offset, which is the voltage distance between reference curve (stored in LUT) and the actual Vtp(Tsens) 
//float Vtp_corr;				//Vtp corrected by TCF 
//float Tobj;						//Object temperature
//float Tobj_precise;		//precise object temperature
//float Vtp_t,Vtp_t+1; 	//Vtp_t at t degree in the lookup table;Vtp_t+1 at t+1 degree in the lookup table;
//float Rref = 50.2;    //Rref is the serial resistance with the NTC in the thermopile;  Unit is Kohm

  
float const Rtx_ref[Nntc]={    
    318292,318292,318292,318292,318292,318292,318292,318292,318292,318292,  
    318292,318292,318292,318292,318292,318292,318292,318292,318292,318292,
    318292,318292,318292,318292,318292,318292,318292,318292,318292,318292,
    318292,318292,318292,318292,318292,318292,318292,318292,318292,318292,
    318292,302894,288325,274535,261479,249114,237400,226300,215778,205802,
    196340,187363,178845,170759,163081,155789,148862,142279,136022,130073,
    124416,119034,113913,109039,104399,99980,95772,91762,87941,84299,
    80826,77515,74356,71342,68465,65719,63098,60594,58202,55917,
    53733,51645,49649,47740,45914,44167,42496,40895,39363,37896,
    36491,35145,33856,32620,31435,30299,29209,28164,27162,26200,
    25277,24390,23539,22722,21938,21184,20459,19763,19094,18451,
    17832,17237,16665,16114,15584,15075,14584,14111,13657,13218,
    12796,12390,11998,11621,11257,10906,10568,10242,9927,9624,
    9331,9049,8776,8514,8260,8015,7778,7550,7329,7116,
    6910
    };  
 
float const Vtp_r25[Ntp]={   
   -1.8954,-1.8760,-1.8563,-1.8363,-1.8160,-1.7955,-1.7747,-1.7537,-1.7324,-1.7107, 
    -1.6889,-1.6667,-1.6442,-1.6215,-1.5985,-1.5752,-1.5516,-1.5277,-1.5035,-1.4790,
    -1.4542,-1.4291,-1.4038,-1.3781,-1.3521,-1.3257,-1.2991,-1.2722,-1.2450,-1.2174, 
    -1.1895,-1.1613,-1.1328,-1.1039,-1.0748,-1.0453,-1.0154,-0.9853,-0.9548,-0.9240, 
    -0.8928,-0.8613,-0.8295,-0.7973,-0.7648,-0.7319,-0.6987,-0.6652,-0.6313,-0.5970,
    -0.5624,-0.5274,-0.4921,-0.4564,-0.4204,-0.3840,-0.3473,-0.3102,-0.2727,-0.2348, 
    -0.1966,-0.1580,-0.1191,-0.0798,-0.0401,0.0000,0.0404,0.0813,0.1225,0.1640, 
    0.2060,0.2484,0.2911,0.3342,0.3777,0.4216,0.4659,0.5105,0.5556,0.6010, 
    0.6469,0.6931,0.7397,0.7868,0.8342,0.8820,0.9303,0.9789,1.0279,1.0774, 
    1.1272,1.1775,1.2281,1.2792,1.3307,1.3825,1.4348,1.4875,1.5407,1.5942, 
    1.6481,1.7025,1.7573,1.8125,1.8681,1.9241,1.9806,2.0375,2.0948,2.1525, 
    2.2106,2.2692,2.3282,2.3876,2.4475,2.5078,2.5685,2.6296,2.6912,2.7531, 
    2.8156,2.8784,2.9417,3.0054,3.0696,3.1342,3.1992,3.2647,3.3306,3.3969, 
    3.4637,3.5309,3.5985,3.6666,3.7351,3.8041,3.8735,3.9433,4.0136,4.0844, 
    4.1555
    };   

          
float Temp_Amb_Resistance_calc(float Vrx)
{
		float Rtx;
        Vrx = (Vrx/3);
		Rtx = (Rref * Vrx)/(Vref - Vrx);  			//Vref is the reference voltage of the anolog circuit; Rtx is the current DC impedance of NTC in thermopile
		Rtx = Rtx*1000;
        return(Rtx);
}
   
float Temp_Amb_Lookup(float Rtx)   
{   
		uint num;
		float Tamb;
		
    for(num=0;num<Nntc;num++)   
    {   
        if(Rtx > Rtx_ref[num])break;
    }   
    if(num == 0)return(-40);
    		else 
    				{
    				Tamb = (num-41) - (Rtx - Rtx_ref[num])/(Rtx_ref[num] - Rtx_ref[num+1]);
    				return(Tamb);
    				}
}   
   
float Vtp_Thermopile_voltage_lookup(float Tamb)   
{    
    uint i; 
    float Vtp_offs;  
    
    for(i=0;i<Ntp;i++)   
    		{
    		if((Tamb+40) <= i)break;
    		} 
    Vtp_offs = Vtp_r25[i-1]+(Vtp_r25[i]-Vtp_r25[i-1])*(Tamb-i+41);        
    return(Vtp_offs);
        
}      

float Obj_Temperature_lookup(float Vtp_corr)
{
		uint i; 
		float Tobj;
		    
		for(i=0;i<Ntp;i++)   
		    {
		    if(Vtp_r25[i] >= Vtp_corr)break;
		    } 
		Tobj = (i-41)+(Vtp_corr - Vtp_r25[i-1])/(Vtp_r25[i]-Vtp_r25[i-1]);
		return(Tobj);
}
float Obj_temp_calc(float Vrx,float Vtp)   	//Vrx(V) = Ambient NTC ADC OUTPUT VOLTAGE; Vtp(mV) = The output voltage of thermopile ADC
{   
    float Rtx;
    float Tamb;   													//Tamb is he current ambient tempeture
    float TCF;															//TCF is TC Factor which is TCF = 1+DeltaT*TCsens
    float Vtp_corr;													//Vtp corrected by Sconv
    float Vtp_offs;													//Lookup offset Vtp by Tamb							
    float Tobj;															//Actual object temperature
    
    Vtp = ((Vtp-Vref*1000)/1056.25);
    Vtp = Vtp/Sconv;
    Rtx = Temp_Amb_Resistance_calc(Vrx);
    //printf("Rtx=%f\r\n",Rtx);
    Tamb = Temp_Amb_Lookup(Rtx);
    printf("Tamb=%f\r\n",Tamb);
    
    TCF = 1 + (Tamb - T25)*TCsens;
    //printf("TCF=%f\r\n",TCF);
    Vtp_offs = Vtp_Thermopile_voltage_lookup(Tamb); 
    //printf("Vtp_offs=%f\r\n",Vtp_offs);
    Vtp_corr = Vtp/TCF + Vtp_offs;
    //printf("Vtp_corr=%f\r\n",Vtp_corr);
    
    Tobj = Obj_Temperature_lookup(Vtp_corr);
    return(Tobj);  
}


