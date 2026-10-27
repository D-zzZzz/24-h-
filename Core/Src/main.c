/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "IIC.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "stdio.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "bluetooth.h"
#include "camera_uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 传感器、编码器和 DMP 状态变量由其他模块定义，这里只引用。 */
extern int Encoder_L,Encoder_R;
extern float pitch,roll,yaw;//陀螺仪
extern short gyrox,gyroy,gyroz;
extern short aacx,aacy,aacz;
extern volatile uint8_t dmp_read_error;
extern volatile unsigned char dmp_fw_error;
extern volatile unsigned short dmp_fw_error_addr;
extern volatile unsigned char dmp_fw_expected;
extern volatile unsigned char dmp_fw_actual;
extern volatile unsigned char mpu_init_error_stage;

/* OLED 显示缓冲区及 MPU6050 初始化结果。 */
uint8_t display_buf[32];
static volatile uint8_t control_ready;

uint32_t sys_tick;
extern uint8_t rx_buf[2];


//int Encoder_L,Encoder_R;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Read(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static float OLED_LimitPidComponent(float value, float limit)
{
    /* 限制显示数值，避免异常 PID 参数撑满 OLED 显示区域。 */
    if (value > limit)
        return limit;
    if (value < -limit)
        return -limit;
    return value;
}

static void OLED_FormatPidComponent(char *text, float value, uint8_t decimals)
{
    int length;

    /* 每个数值占 6 个字符；较大数值减少小数位以保留符号和整数。 */
    while (1)
    {
        length = snprintf(text, 7U, "%6.*f", (int)decimals, value);
        if (length <= 6 || decimals == 0U)
            break;
        decimals--;
    }
}

static void OLED_ShowSmallLine(uint8_t page)
{
    uint8_t i = 0U;

    /* 6x8 字体每行显示 21 个字符，补空格覆盖上次的长数值。 */
    while (i < 21U && display_buf[i] != '\0')
        i++;
    while (i < 21U)
        display_buf[i++] = ' ';
    display_buf[21] = '\0';
    OLED_ShowString(0, page, display_buf, 8);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    uint8_t mpu6050_Error_Return;
    char pid_p_text[7], pid_di_text[7];
    uint32_t oled_refresh_tick;

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* HAL 初始化：复位外设、配置 Flash 接口并启动 SysTick。 */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* 配置系统时钟：外部高速时钟经 PLL 后得到 72 MHz 系统时钟。 */
    SystemClock_Config();

    /* 初始化 GPIO、PWM 定时器和左右轮编码器定时器。 */
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_TIM1_Init();
    MX_TIM4_Init();
    /* USER CODE BEGIN 2 */
    /* Keep communication independent of OLED/MPU initialization results. */
    if (Bluetooth_Init() != HAL_OK)
        Error_Handler();
    if (Camera_Init() != HAL_OK)
        Error_Handler();

    /* 初始化 OLED，并清屏准备显示运行状态。 */
    OLED_Init();
    OLED_Clear();

    /* 初始化 MPU6050，读取设备信息和校准/版本数据用于诊断。 */
    /* This call initializes I2C and the selected MPU before enabling DMP. */
    mpu6050_Error_Return = mpu_dmp_init();
    if (mpu6050_Error_Return != 0U)
    {
        Load(0, 0);
        OLED_Clear();
        snprintf((char *)display_buf, sizeof(display_buf), "MPU ERR:%u", mpu6050_Error_Return);
        OLED_ShowSmallLine(0);
        snprintf((char *)display_buf, sizeof(display_buf), "STAGE:%u", mpu_init_error_stage);
        OLED_ShowSmallLine(1);

        /* Do not start motor PWM without valid attitude feedback. */
        while (1)
            Bluetooth_Delay(100U);
    }

    /* 启动电机 PWM 和左右轮编码器采集。 */
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
    HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
    /* 上电时保持电机停止，避免初始化完成瞬间误动作。 */
    Load(0,0);
    control_ready = 1U;
    OLED_Clear();

    /* USER CODE END 2 */

    oled_refresh_tick = HAL_GetTick() - 100U;

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        Bluetooth_Process();
        /* Schedule at most 10 display frames/s; keep servicing Bluetooth while idle. */
        if (HAL_GetTick() - oled_refresh_tick < 100U)
        {
            HAL_Delay(1U);
            continue;
        }
        oled_refresh_tick = HAL_GetTick();
        /* 小字体显示原有 PID 实时分量、倾角和左右编码器计数。 */
        OLED_FormatPidComponent(pid_p_text,
                OLED_LimitPidComponent(Vertical_Kp_Component, 9999.9f), 1);
        OLED_FormatPidComponent(pid_di_text,
                OLED_LimitPidComponent(Vertical_Kd_Component, 9999.9f), 1);
        snprintf((char *)display_buf, sizeof(display_buf), "ver P:%s D:%s",
                pid_p_text, pid_di_text);
        OLED_ShowSmallLine(0);

        Bluetooth_Process();

        OLED_FormatPidComponent(pid_p_text,
                OLED_LimitPidComponent(Velocity_Kp_Component, 999.99f), 3);
        OLED_FormatPidComponent(pid_di_text,
                OLED_LimitPidComponent(Velocity_Ki_Component, 999.99f), 3);
        snprintf((char *)display_buf, sizeof(display_buf), "vel P:%s I:%s",
                pid_p_text, pid_di_text);
        OLED_ShowSmallLine(1);
        Bluetooth_Process();
        snprintf((char *)display_buf, sizeof(display_buf), "pitch:%.2f", pitch);
        OLED_ShowSmallLine(2);
        Bluetooth_Process();

        snprintf((char *)display_buf, sizeof(display_buf), "EL:%6d ER:%6d", Encoder_L, Encoder_R);
        OLED_ShowSmallLine(3);
        Bluetooth_Process();
        snprintf((char *)display_buf, sizeof(display_buf), "Turn:%5d GZ:%6d", Turn_out, gyroz);
        OLED_ShowSmallLine(4);
        Bluetooth_Process();
        snprintf((char *)display_buf, sizeof(display_buf), "%s",
                tilt_stop_latched ? "TILT STOP:RESET" : "Tilt:OK");
        OLED_ShowSmallLine(5);
        Bluetooth_Process();
        snprintf((char *)display_buf, sizeof(display_buf), "CAM E:%4d V:%u M:%u",
                Camera_Error, Camera_Valid, Camera_Enabled);
        OLED_ShowSmallLine(6);
        Bluetooth_Process();
        snprintf((char *)display_buf, sizeof(display_buf), "CAM F:%lu B:%lu",
                (unsigned long)Camera_FrameCount, (unsigned long)Camera_BadFrames);
        OLED_ShowSmallLine(7);
				
        /* 实际平衡控制仍由 MPU 外部中断回调触发。 */


//        sprintf((char *)display_buf,
//                "D:%u C:%u E:%u ",
//                (unsigned int)mpu6050_Error_Return,
//                (unsigned int)mpu_init_error_stage,
//                (unsigned int)dmp_fw_error);
//        OLED_ShowString(0, 4, display_buf, 16);

//        sprintf((char *)display_buf,
//                "A:%04X %02X/%02X ",
//                (unsigned int)dmp_fw_error_addr,
//                (unsigned int)dmp_fw_expected,
//                (unsigned int)dmp_fw_actual);
//        OLED_ShowString(0, 6, display_buf, 16);

//        sprintf((char *)display_buf, "I:%u R:%u      ",
//                (unsigned int)mpu6050_Error_Return,
//                (unsigned int)dmp_read_error);
//        OLED_ShowString(0,0,display_buf,16);

//        sprintf((char *)display_buf,"P:%7.2f     ",pitch);
//        OLED_ShowString(0,2,display_buf,16);
//        sprintf((char *)display_buf,"R:%7.2f     ",roll);
//        OLED_ShowString(0,4,display_buf,16);
//        sprintf((char *)display_buf,"Y:%7.2f     ",yaw);
//        OLED_ShowString(0,6,display_buf,16);

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_OFF; /* PC14/PC15 are OLED GPIO, not LSE. */
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* 配置 CPU、AHB 和 APB 总线分频。 */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    /* Do not read the MPU bus or drive motors from an IRQ during initialization. */
    if(GPIO_Pin==GPIO_PIN_14 && control_ready != 0U)
        control();
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();

    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
