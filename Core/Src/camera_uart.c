#include "camera_uart.h"
#include "pid.h"

#define CAMERA_DMA_SIZE 512U
#define CAMERA_FRAME_SIZE 64U

volatile int Camera_Error;
volatile int Camera_TurnSign = 1;
volatile uint8_t Camera_Valid;
volatile uint8_t Camera_Enabled = 1U;
volatile uint32_t Camera_FrameCount;
volatile uint32_t Camera_BadFrames;
volatile uint32_t Camera_Overruns;

static UART_HandleTypeDef camera_uart;
static DMA_HandleTypeDef camera_rx_dma;
static uint8_t camera_dma_buffer[CAMERA_DMA_SIZE];
static uint32_t camera_consumed;
static volatile uint32_t camera_wraps;
static volatile uint8_t camera_receive_error;
static uint8_t camera_initialized;
static uint8_t camera_frame[CAMERA_FRAME_SIZE];
static uint16_t camera_frame_length;
static uint32_t camera_last_byte_tick;
static volatile uint32_t camera_last_frame_tick;

static void Camera_Invalidate(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    Camera_Valid = 0U;
    Camera_Error = 0;
    __set_PRIMASK(primask);
}

static HAL_StatusTypeDef Camera_StartReceive(void)
{
    HAL_StatusTypeDef status;
    camera_consumed = 0U;
    camera_wraps = 0U;
    status = HAL_UART_Receive_DMA(&camera_uart, camera_dma_buffer, CAMERA_DMA_SIZE);
    if (status == HAL_OK)
        __HAL_DMA_DISABLE_IT(&camera_rx_dma, DMA_IT_HT);
    return status;
}

HAL_StatusTypeDef Camera_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_AFIO_REMAP_USART2_DISABLE();

    gpio.Pin = GPIO_PIN_2;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);

    camera_uart.Instance = USART2;
    camera_uart.Init.BaudRate = 115200U;
    camera_uart.Init.WordLength = UART_WORDLENGTH_8B;
    camera_uart.Init.StopBits = UART_STOPBITS_1;
    camera_uart.Init.Parity = UART_PARITY_NONE;
    camera_uart.Init.Mode = UART_MODE_TX_RX;
    camera_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    camera_uart.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&camera_uart) != HAL_OK)
        return HAL_ERROR;

    /* DMA receives bytes while the balance interrupt is running. */
    camera_rx_dma.Instance = DMA1_Channel6;
    camera_rx_dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    camera_rx_dma.Init.PeriphInc = DMA_PINC_DISABLE;
    camera_rx_dma.Init.MemInc = DMA_MINC_ENABLE;
    camera_rx_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    camera_rx_dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    camera_rx_dma.Init.Mode = DMA_CIRCULAR;
    camera_rx_dma.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&camera_rx_dma) != HAL_OK)
        return HAL_ERROR;
    __HAL_LINKDMA(&camera_uart, hdmarx, camera_rx_dma);
    HAL_NVIC_SetPriority(USART2_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

    Camera_SetEnabled(1U);
    camera_initialized = (Camera_StartReceive() == HAL_OK);
    return camera_initialized ? HAL_OK : HAL_ERROR;
}

static void Camera_ParseFrame(void)
{
    uint32_t fields[6] = {0};
    uint32_t primask;
    uint16_t i;
    uint8_t field = 0U, digits = 0U;

    /* No sscanf/strtod: reject empty fields, signs, NUL and extra fields. */
    for (i = 1U; i + 1U < camera_frame_length; i++)
    {
        uint8_t byte = camera_frame[i];
        if (byte >= '0' && byte <= '9')
        {
            if (++digits > 5U)
                goto invalid;
            fields[field] = fields[field] * 10U + byte - '0';
        }
        else if (byte == ',' && digits != 0U && field < 5U)
        {
            field++;
            digits = 0U;
        }
        else
            goto invalid;
    }
    if (field != 5U || digits == 0U || fields[0] != camera_frame_length ||
        fields[1] != CAMERA_FUNCTION_ID || fields[2] < 680U || fields[2] > 1319U ||
        fields[3] > 639U || fields[4] > 479U || fields[5] > 1U)
        goto invalid;
    /* This sender uses x=error+1000 and y=target_x=error+320. */
    if (fields[5] == 1U && fields[2] != fields[3] + 680U)
        goto invalid;

    primask = __get_PRIMASK();
    __disable_irq();
    Camera_Error = fields[5] ? (int)fields[2] - 1000 : 0;
    camera_last_frame_tick = HAL_GetTick();
    Camera_Valid = (uint8_t)fields[5];
    Camera_FrameCount++;
    __set_PRIMASK(primask);
    return;
invalid:
    Camera_BadFrames++;
    Camera_Invalidate();
}

static void Camera_ReceiveByte(uint8_t byte)
{
    if (byte == '$')
    {
        camera_frame[0] = byte;
        camera_frame_length = 1U;
        return;
    }
    if (camera_frame_length == 0U)
        return;
    if (camera_frame_length >= CAMERA_FRAME_SIZE)
    {
        camera_frame_length = 0U;
        Camera_BadFrames++;
        Camera_Invalidate();
        return;
    }
    camera_frame[camera_frame_length++] = byte;
    if (byte == '#')
    {
        Camera_ParseFrame();
        camera_frame_length = 0U;
    }
}

void Camera_Process(void)
{
    uint32_t budget = CAMERA_DMA_SIZE;
    if (!camera_initialized)
        return;
    if (camera_receive_error)
    {
        Camera_Invalidate();
        (void)HAL_UART_DMAStop(&camera_uart);
        __HAL_UART_CLEAR_OREFLAG(&camera_uart);
        camera_frame_length = 0U;
        camera_receive_error = 0U;
        if (Camera_StartReceive() != HAL_OK)
            camera_receive_error = 1U;
        return;
    }
    if (camera_frame_length && HAL_GetTick() - camera_last_byte_tick > CAMERA_TIMEOUT_MS)
        camera_frame_length = 0U;

    while (budget-- != 0U)
    {
        uint32_t primask = __get_PRIMASK();
        uint32_t wraps, remaining, produced;
        uint8_t byte;
        __disable_irq();
        wraps = camera_wraps;
        remaining = __HAL_DMA_GET_COUNTER(&camera_rx_dma);
        if (__HAL_DMA_GET_FLAG(&camera_rx_dma, DMA_FLAG_TC6))
        {
            wraps++;
            remaining = __HAL_DMA_GET_COUNTER(&camera_rx_dma);
        }
        __set_PRIMASK(primask);
        produced = wraps * CAMERA_DMA_SIZE + CAMERA_DMA_SIZE - remaining;
        if (produced == camera_consumed)
            break;
        if (produced - camera_consumed > CAMERA_DMA_SIZE)
        {
            camera_consumed = produced;
            camera_frame_length = 0U;
            Camera_Overruns++;
            Camera_Invalidate();
            return;
        }
        byte = camera_dma_buffer[camera_consumed % CAMERA_DMA_SIZE];
        camera_consumed++;
        camera_last_byte_tick = HAL_GetTick();
        Camera_ReceiveByte(byte);
    }
}

int Camera_ControlTargets(void)
{
    static int camera_last_speed;
    int speed = Target_Speed;
    int turn = 0;
    if (camera_receive_error ||
        (Camera_Valid && HAL_GetTick() - camera_last_frame_tick > CAMERA_TIMEOUT_MS))
        Camera_Invalidate();
    if (Camera_Enabled)
    {
        if (Camera_Valid && !tilt_stop_latched)
            turn = Camera_TurnSign < 0 ? -Camera_Error : Camera_Error;
        else
            speed = 0; /* Pause on lost input without overwriting Target_Speed. */
        Target_Turn = turn;
    }
    if (camera_last_speed != speed)
    {
        camera_last_speed = speed;
        stop = 1U;
    }
    return speed;
}

void Camera_SetEnabled(uint8_t enabled)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    Camera_Enabled = enabled && !tilt_stop_latched ? 1U : 0U;
    camera_frame_length = 0U;
    Camera_Valid = 0U; /* Starting requires a fresh complete camera frame. */
    Camera_Error = 0;
    if (!Camera_Enabled)
        Target_Speed = 0; /* An explicit CAM,0 command requests a stop. */
    Target_Turn = 0;
    stop = 1U;
    __set_PRIMASK(primask);
}

void Camera_UART_IRQHandler(void) { HAL_UART_IRQHandler(&camera_uart); }
void Camera_DMA_IRQHandler(void) { HAL_DMA_IRQHandler(&camera_rx_dma); }
void Camera_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == &camera_uart)
        camera_receive_error = 1U;
}
void Camera_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == &camera_uart)
        camera_wraps++;
}
