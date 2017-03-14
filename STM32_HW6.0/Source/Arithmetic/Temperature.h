#ifndef __TEMPERATURE_H
#define __TEMPERATURE_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"

/* Exported macro ------------------------------------------------------------*/


/* Exported functions ------------------------------------------------------- */
float Obj_temp_calc(float Vrx,float Vtp);
float Temp_Amb_Resistance_calc(float Vrx);
float Temp_Amb_Lookup(float Rtx) ;
float Vtp_Thermopile_voltage_lookup(float Tamb);
float Obj_Temperature_lookup(float Vtp_corr);

#endif /*__TEMPERATURE_H*/
