#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f1xx_hal.h"

#define MOTOR_PWM_LIMIT 5000

void Load(int motor1,int motor2);
void Limit(int *MOTOR1,int *MOTOR2);

#endif
