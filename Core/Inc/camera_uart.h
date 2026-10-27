#ifndef CAMERA_UART_H
#define CAMERA_UART_H

#include "stm32f1xx_hal.h"

/* Standard Yahboom color packet: $length,1,x,y,w,h# (ASCII). */
#define CAMERA_FUNCTION_ID 1U
#define CAMERA_TIMEOUT_MS 500U

extern volatile int Camera_Error;
extern volatile int Camera_TurnSign;
extern volatile uint8_t Camera_Valid;
extern volatile uint8_t Camera_Enabled;
extern volatile uint32_t Camera_FrameCount;
extern volatile uint32_t Camera_BadFrames;
extern volatile uint32_t Camera_Overruns;

HAL_StatusTypeDef Camera_Init(void);
void Camera_Process(void);
/* Return the effective speed; Target_Speed remains the requested speed. */
int Camera_ControlTargets(void);
void Camera_SetEnabled(uint8_t enabled);
void Camera_UART_IRQHandler(void);
void Camera_DMA_IRQHandler(void);
void Camera_UART_ErrorCallback(UART_HandleTypeDef *uart);
void Camera_UART_RxCpltCallback(UART_HandleTypeDef *uart);

#endif
