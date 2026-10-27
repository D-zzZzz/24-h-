#ifndef __PID_H__
#define __PID_H__

#include "stm32f1xx_hal.h"

#define TILT_STOP_ANGLE_DEG 75.0f

extern volatile float Vertical_Kp, Vertical_Kd;
extern volatile float Velocity_Kp, Velocity_Ki;
extern volatile float Med_Angle;
extern volatile int Target_Speed;
extern volatile float Turn_Kp, Turn_Kp2, Turn_Kd;
extern volatile int Target_Turn;
extern int Turn_out;
extern volatile uint8_t stop;
/* Set by excessive pitch; only an MCU reset clears the motor interlock. */
extern volatile uint8_t tilt_stop_latched;

extern volatile float Vertical_Kp_Component;
extern volatile float Vertical_Kd_Component;
extern volatile float Velocity_Kp_Component;
extern volatile float Velocity_Ki_Component;

void control(void);


#endif
