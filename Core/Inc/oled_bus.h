#ifndef __OLED_BUS_H
#define __OLED_BUS_H

#include "stm32f1xx_hal.h"

/* OLED software I2C: PC14=SCL, PC15=SDA. */
HAL_StatusTypeDef OLED_I2C_Write(uint8_t control,
                                 const uint8_t *data,
                                 uint16_t size);

#endif /* __OLED_BUS_H */
