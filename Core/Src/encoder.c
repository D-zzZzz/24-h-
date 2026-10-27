#include "encoder.h"
//TIM_HandleTypeDef  编码器所使用的定时器句柄地址
//通常把它放在固定周期的定时器中断或控制循环里，例如每 5 ms 或 10 ms 调用一次
int Read_Speed(TIM_HandleTypeDef *htim)
{
    int temp;
    temp=(short)__HAL_TIM_GetCounter(htim);
    __HAL_TIM_SetCounter(htim,0);
    return temp;

}
