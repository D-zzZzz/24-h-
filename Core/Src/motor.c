#include "motor.h"

#define PWM_MAX  MOTOR_PWM_LIMIT
#define PWM_MIN  (-MOTOR_PWM_LIMIT)
extern TIM_HandleTypeDef htim1;

int abs(int p)
{
    if(p>0)
        return p;
    else
        return -p;

}


void Load(int motor1,int motor2)  //PB12 PA4决定了电机转动方向
{
    if(motor1<0)
    {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
    }
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,abs(motor1));

    if(motor2<0)
    {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
    }
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,abs(motor2));
}

void Limit(int *MOTOR1,int *MOTOR2)
{
    if(*MOTOR1>PWM_MAX) *MOTOR1=PWM_MAX;
    if(*MOTOR1<PWM_MIN) *MOTOR1=PWM_MIN;
    if(*MOTOR2>PWM_MAX) *MOTOR2=PWM_MAX;
    if(*MOTOR2<PWM_MIN) *MOTOR2=PWM_MIN;
}
