/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
#include "oled_bus.h"

/* USER CODE BEGIN 0 */

/* Atomic open-drain writes; leave the conservative bus delays unchanged. */
#define OLED_SCL_HIGH() (GPIOC->BSRR = GPIO_PIN_14)
#define OLED_SCL_LOW()  (GPIOC->BSRR = (uint32_t)GPIO_PIN_14 << 16U)
#define OLED_SDA_HIGH() (GPIOC->BSRR = GPIO_PIN_15)
#define OLED_SDA_LOW()  (GPIOC->BSRR = (uint32_t)GPIO_PIN_15 << 16U)

/* Keep the software-I2C clock comfortably below the OLED's 400 kHz limit.
   PC14/PC15 are slow, low-drive pins, so every bus edge needs settling time. */
static void OLED_I2C_Delay(void)
{
  volatile uint32_t i;

  for (i = 0; i < 100U; i++)
  {
    __NOP();
  }
}

static HAL_StatusTypeDef OLED_I2C_WriteByte(uint8_t data)
{
  uint8_t bit;

  for (bit = 0; bit < 8; bit++)
  {
    if ((data & 0x80U) != 0U)
      OLED_SDA_HIGH();
    else
      OLED_SDA_LOW();
    OLED_I2C_Delay();
    OLED_SCL_HIGH();
    OLED_I2C_Delay();
    OLED_SCL_LOW();
    OLED_I2C_Delay();
    data <<= 1;
  }

  /* Release SDA while the OLED drives ACK. */
  OLED_SDA_HIGH();
  OLED_I2C_Delay();
  OLED_SCL_HIGH();
  OLED_I2C_Delay();
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) != GPIO_PIN_RESET)
  {
    OLED_SCL_LOW();
    OLED_I2C_Delay();
    return HAL_ERROR;
  }
  OLED_SCL_LOW();
  OLED_I2C_Delay();

  return HAL_OK;
}

HAL_StatusTypeDef OLED_I2C_Write(uint8_t control,
                                 const uint8_t *data,
                                 uint16_t size)
{
  uint16_t i;
  HAL_StatusTypeDef status = HAL_OK;

  /* START */
  OLED_SDA_HIGH();
  OLED_SCL_HIGH();
  OLED_I2C_Delay();
  OLED_SDA_LOW();
  OLED_I2C_Delay();
  OLED_SCL_LOW();
  OLED_I2C_Delay();

  if (OLED_I2C_WriteByte(0x78U) != HAL_OK)
  {
    status = HAL_ERROR;
    goto stop;
  }
  if (OLED_I2C_WriteByte(control) != HAL_OK)
  {
    status = HAL_ERROR;
    goto stop;
  }
  for (i = 0; i < size; i++)
  {
    if (OLED_I2C_WriteByte(data[i]) != HAL_OK)
    {
      status = HAL_ERROR;
      goto stop;
    }
  }

stop:
  /* STOP */
  OLED_SDA_LOW();
  OLED_I2C_Delay();
  OLED_SCL_HIGH();
  OLED_I2C_Delay();
  OLED_SDA_HIGH();
  OLED_I2C_Delay();

  return status;
}

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /* Release the MPU6050 software-I2C bus before enabling open-drain mode. */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);

  /* Release the OLED software-I2C bus. */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pins : PA4 PA5 (TB6612 AIN2/BIN2) */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 (TB6612 AIN1/BIN1) */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 (MPU6050 software I2C) */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 (MPU6050 INT, active low) */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* PC14/PC15: open-drain, 2 MHz maximum per STM32F103 datasheet.
     External pull-ups must be compatible with the MCU's 3.3 V I/O levels. */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
