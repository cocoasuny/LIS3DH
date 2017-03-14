#ifndef __MOTORCTL_H
#define __MOTORCTL_H 

#include "common.h"

/* Port define */
#define RCC_MotorPeriph            PT_RCC_MotorPeriph
#define GPIO_Pin_Motor             PT_GPIO_Pin_Motor
#define GPIO_Motor                 PT_GPIO_Motor
#define Periph_Motor               PT_Periph_Motor

void Motor_Init(void);
void Motor_DeInit(void);
static void MotorCtl(uint8_t status);
static void Motor_Vibrate(Timer_ID_Typedef TIMID);
void StartMotor(uint8_t MotorVibrateTime);
void StopMotor(void);
void KillMotor(void);


#endif /* MotorCtl */
