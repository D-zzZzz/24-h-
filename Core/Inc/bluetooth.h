#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "stm32f1xx_hal.h"

/* JDY-31: PB10 -> RXD, PB11 <- TXD, 9600 baud, 8N1. */
HAL_StatusTypeDef Bluetooth_Init(void);
void Bluetooth_Process(void);
void Bluetooth_Delay(uint32_t milliseconds);
void Bluetooth_UART_IRQHandler(void);
void Bluetooth_DMA_IRQHandler(void);

#endif
