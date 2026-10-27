#include "bluetooth.h"
#include "pid.h"
#include "pid_protocol.h"
#include "camera_uart.h"
#include <string.h>

#define RX_DMA_SIZE 512U
#define RX_LINE_SIZE 128U

static UART_HandleTypeDef bluetooth_uart;
static DMA_HandleTypeDef bluetooth_rx_dma;
static uint8_t rx_dma[RX_DMA_SIZE];
static uint32_t rx_consumed;
static volatile uint32_t rx_wraps;
static char rx_line[RX_LINE_SIZE];
static uint16_t line_length;
static uint8_t discard_line;
static uint8_t initialized;
static volatile uint8_t receive_error;
static uint32_t last_byte_tick;

static HAL_StatusTypeDef Bluetooth_StartReceive(void)
{
    rx_consumed = 0U;
    rx_wraps = 0U;
    return HAL_UART_Receive_DMA(&bluetooth_uart, rx_dma, RX_DMA_SIZE);
}

HAL_StatusTypeDef Bluetooth_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_AFIO_REMAP_USART3_DISABLE();

    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio);

    bluetooth_uart.Instance = USART3;
    bluetooth_uart.Init.BaudRate = 9600U;
    bluetooth_uart.Init.WordLength = UART_WORDLENGTH_8B;
    bluetooth_uart.Init.StopBits = UART_STOPBITS_1;
    bluetooth_uart.Init.Parity = UART_PARITY_NONE;
    bluetooth_uart.Init.Mode = UART_MODE_TX_RX;
    bluetooth_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    bluetooth_uart.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&bluetooth_uart) != HAL_OK)
        return HAL_ERROR;

    /* DMA keeps receiving while the higher-priority balance ISR is running. */
    bluetooth_rx_dma.Instance = DMA1_Channel3;
    bluetooth_rx_dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    bluetooth_rx_dma.Init.PeriphInc = DMA_PINC_DISABLE;
    bluetooth_rx_dma.Init.MemInc = DMA_MINC_ENABLE;
    bluetooth_rx_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    bluetooth_rx_dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    bluetooth_rx_dma.Init.Mode = DMA_CIRCULAR;
    bluetooth_rx_dma.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&bluetooth_rx_dma) != HAL_OK)
        return HAL_ERROR;
    __HAL_LINKDMA(&bluetooth_uart, hdmarx, bluetooth_rx_dma);

    HAL_NVIC_SetPriority(USART3_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    initialized = (Bluetooth_StartReceive() == HAL_OK);
    return initialized ? HAL_OK : HAL_ERROR;
}

static void Bluetooth_HandleLine(void)
{
    float values[PID_PARAMETER_COUNT];
    const char *reply;
    values[5] = (float)Target_Speed;
    values[6] = Turn_Kp;
    values[7] = Turn_Kd;
    values[8] = (float)Target_Turn;
    if (strcmp(rx_line, "CAM,1") == 0 || strcmp(rx_line, "CAM,0") == 0)
    {
        if (rx_line[4] == '1' && tilt_stop_latched)
            reply = "ERR,TILT\r\n";
        else
        {
            Camera_SetEnabled(rx_line[4] == '1');
            reply = "OK\r\n";
        }
        (void)HAL_UART_Transmit(&bluetooth_uart, (uint8_t *)reply,
                                (uint16_t)strlen(reply), 100U);
        return;
    }
    reply = PID_ProtocolParse(rx_line, values);

    if (strcmp(reply, "OK\r\n") == 0)
    {
        uint32_t primask = __get_PRIMASK();
        uint8_t reset_velocity = (Velocity_Kp != values[2] || Velocity_Ki != values[3] ||
                                  Target_Speed != (int)values[5]);
        /* A control cycle must see all old values or all new values. */
        __disable_irq();
        Camera_Enabled = 0U; /* Manual PID commands take ownership until CAM,1/reset. */
        Vertical_Kp = values[0];
        Vertical_Kd = values[1];
        Velocity_Kp = values[2];
        Velocity_Ki = values[3];
        Med_Angle = values[4];
        Target_Speed = tilt_stop_latched ? 0 : (int)values[5];
        Turn_Kp = values[6];
        Turn_Kd = values[7];
        Target_Turn = tilt_stop_latched ? 0 : (int)values[8];
        /* Steering-only adjustments must not reset the speed-loop integrator. */
        if (reset_velocity)
            stop = 1U;
        __set_PRIMASK(primask);
    }
    /* Replies are sent in the foreground, leaving balance interrupts enabled. */
    (void)HAL_UART_Transmit(&bluetooth_uart, (uint8_t *)reply,
                            (uint16_t)strlen(reply), 100U);
}

void Bluetooth_Process(void)
{
    uint32_t budget = RX_DMA_SIZE;

    Camera_Process();

    if (!initialized)
        return;
    if (receive_error)
    {
        (void)HAL_UART_DMAStop(&bluetooth_uart);
        __HAL_UART_CLEAR_OREFLAG(&bluetooth_uart);
        line_length = 0U;
        discard_line = 1U; /* Never apply the suffix of a damaged frame. */
        receive_error = 0U;
        if (Bluetooth_StartReceive() != HAL_OK)
            receive_error = 1U;
        return;
    }
    if (line_length != 0U && HAL_GetTick() - last_byte_tick > 1000U)
    {
        line_length = 0U;
        discard_line = 1U;
    }

    while (budget-- != 0U)
    {
        uint32_t primask = __get_PRIMASK();
        uint32_t wraps, remaining, produced;
        uint8_t byte;

        /* Account for a DMA wrap even if its IRQ has not run yet. */
        __disable_irq();
        wraps = rx_wraps;
        remaining = __HAL_DMA_GET_COUNTER(&bluetooth_rx_dma);
        if (__HAL_DMA_GET_FLAG(&bluetooth_rx_dma, DMA_FLAG_TC3))
        {
            wraps++;
            remaining = __HAL_DMA_GET_COUNTER(&bluetooth_rx_dma);
        }
        __set_PRIMASK(primask);
        produced = wraps * RX_DMA_SIZE + RX_DMA_SIZE - remaining;
        if (produced == rx_consumed)
            break;
        if (produced - rx_consumed > RX_DMA_SIZE)
        {
            static const uint8_t overflow[] = "ERR,OVERFLOW\r\n";
            rx_consumed = produced;
            line_length = 0U;
            discard_line = 1U;
            (void)HAL_UART_Transmit(&bluetooth_uart, (uint8_t *)overflow,
                                    sizeof(overflow) - 1U, 100U);
            return;
        }
        byte = rx_dma[rx_consumed % RX_DMA_SIZE];
        rx_consumed++;
        last_byte_tick = HAL_GetTick();
        if (byte == '\n')
        {
            if (!discard_line && line_length != 0U)
            {
                if (rx_line[line_length - 1U] == '\r')
                    line_length--;
                rx_line[line_length] = '\0';
                Bluetooth_HandleLine();
            }
            line_length = 0U;
            discard_line = 0U;
        }
        else if (!discard_line)
        {
            if (line_length >= RX_LINE_SIZE - 1U)
            {
                static const uint8_t overflow[] = "ERR,OVERFLOW\r\n";
                discard_line = 1U;
                line_length = 0U;
                (void)HAL_UART_Transmit(&bluetooth_uart, (uint8_t *)overflow,
                                        sizeof(overflow) - 1U, 100U);
            }
            else if (byte == 0U)
            {
                /* Embedded NUL must not hide trailing bytes from the parser. */
                static const uint8_t format[] = "ERR,FORMAT\r\n";
                discard_line = 1U;
                line_length = 0U;
                (void)HAL_UART_Transmit(&bluetooth_uart, (uint8_t *)format,
                                        sizeof(format) - 1U, 100U);
            }
            else
                rx_line[line_length++] = (char)byte;
        }
    }
}

void Bluetooth_Delay(uint32_t milliseconds)
{
    uint32_t start = HAL_GetTick();
    do
    {
        Bluetooth_Process();
        HAL_Delay(1U);
    } while (HAL_GetTick() - start < milliseconds);
}

void Bluetooth_UART_IRQHandler(void)
{
    HAL_UART_IRQHandler(&bluetooth_uart);
}

void Bluetooth_DMA_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&bluetooth_rx_dma);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    Camera_UART_ErrorCallback(uart);
    if (uart == &bluetooth_uart)
        receive_error = 1U;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    Camera_UART_RxCpltCallback(uart);
    if (uart == &bluetooth_uart)
        rx_wraps++;
}
