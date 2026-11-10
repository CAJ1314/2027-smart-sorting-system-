#include "VisionUART.h"
#include "Delay.h"

volatile uint8_t Vision_Shape = 0U;
volatile uint8_t Vision_Color = 0U;
volatile uint8_t Vision_UART_Status = 0U;

static volatile char rx_line[64];
static volatile uint8_t rx_length = 0U;
static volatile uint8_t rx_ready = 0U;
static volatile uint8_t rx_bad = 0U;

static uint8_t TextEqual(const char *text, uint8_t length, const char *expected)
{
    uint8_t i = 0U;

    while (expected[i] != '\0')
    {
        if (i >= length || text[i] != expected[i]) return 0U;
        ++i;
    }
    return (uint8_t)(i == length);
}

static uint8_t ParseShape(const char *text, uint8_t length)
{
    if (TextEqual(text, length, "zhengfangti")) return 1U;
    if (TextEqual(text, length, "liulengzhu")) return 2U;
    if (TextEqual(text, length, "sanlengzhu")) return 3U;
    if (TextEqual(text, length, "qiu")) return 4U;
    if (TextEqual(text, length, "wulengzhu")) return 5U;
    if (TextEqual(text, length, "yuanzhu")) return 6U;
    if (TextEqual(text, length, "changfangti")) return 7U;
    return 0U;
}

static uint8_t ParseColor(const char *text, uint8_t length)
{
    if (TextEqual(text, length, "red")) return 1U;
    if (TextEqual(text, length, "black")) return 2U;
    if (TextEqual(text, length, "yellow")) return 3U;
    if (TextEqual(text, length, "dark blue")) return 4U;
    if (TextEqual(text, length, "light blue")) return 5U;
    if (TextEqual(text, length, "orange")) return 6U;
    if (TextEqual(text, length, "green")) return 7U;
    return 0U;
}

/* Accept: SHAPE:zhengfangti,COLOR:red */
static uint8_t ParseVisionLine(const char *line, uint8_t *shape, uint8_t *color)
{
    uint8_t i, comma = 0U, shape_start, shape_length, color_start, color_length;
    uint8_t line_length = 0U;

    while (line[line_length] != '\0' && line_length < 63U) ++line_length;

    if (line_length >= 6U && line[0] == 'S' && line[1] == 'H' && line[2] == 'A' &&
        line[3] == 'P' && line[4] == 'E' && line[5] == ':')
    {
        shape_start = 6U;
        for (i = shape_start; line[i] != '\0'; ++i)
        {
            if (line[i] == ',')
            {
                comma = i;
                break;
            }
        }
        if (comma == 0U) return 0U;
        /* Need ",COLOR:" plus at least one character. */
        if ((uint8_t)(comma + 8U) > line_length) return 0U;
        shape_length = (uint8_t)(comma - shape_start);
        if (line[comma + 1U] != 'C' || line[comma + 2U] != 'O' ||
            line[comma + 3U] != 'L' || line[comma + 4U] != 'O' ||
            line[comma + 5U] != 'R' || line[comma + 6U] != ':')
        {
            return 0U;
        }
        color_start = (uint8_t)(comma + 7U);
        color_length = 0U;
        while (line[color_start + color_length] != '\0') ++color_length;

        *shape = ParseShape(&line[shape_start], shape_length);
        *color = ParseColor(&line[color_start], color_length);
        return (uint8_t)(*shape != 0U);
    }

    /* Backward compatibility: old single-character shape result. */
    if (line[0] >= '1' && line[0] <= '7' && line[1] == '\0')
    {
        *shape = (uint8_t)(line[0] - '0');
        *color = 0U;
        return 1U;
    }

    return 0U;
}

void USART2_IRQHandler(void)
{
    uint32_t status = USART2->SR;
    char c;

    if (status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE))
    {
        c = (char)USART2->DR;
        if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE))
        {
            rx_bad = 1U;
            return;
        }

        if (rx_ready) return;

        if (c == '\n')
        {
            if (rx_bad) rx_line[0] = '\0';
            else rx_line[rx_length] = '\0';
            rx_ready = 1U;
        }
        else if (c != '\r')
        {
            if (rx_length < sizeof(rx_line) - 1U) rx_line[rx_length++] = c;
            else rx_bad = 1U;
        }
    }
}

void VisionUART_Init(void)
{
    GPIO_InitTypeDef g;
    RCC_ClocksTypeDef clocks;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    g.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    g.GPIO_Mode = GPIO_Mode_AF;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &g);

    RCC_GetClocksFreq(&clocks);
    USART2->CR1 = 0U;
    USART2->CR2 = 0U;
    USART2->CR3 = 0U;
    USART2->BRR = (clocks.PCLK1_Frequency + 57600U) / 115200U;
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    NVIC_SetPriority(USART2_IRQn, 4U);
    NVIC_EnableIRQ(USART2_IRQn);
}

/* Clear stale bytes and send the capture request. Does not wait for a reply. */
uint8_t VisionUART_BeginRequest(void)
{
    uint32_t elapsed;
    uint32_t irq;

    Vision_Shape = 0U;
    Vision_Color = 0U;
    Vision_UART_Status = 0U;

    irq = __get_PRIMASK();
    __disable_irq();
    rx_length = 0U;
    rx_ready = 0U;
    rx_bad = 0U;
    (void)USART2->SR;
    (void)USART2->DR;
    __set_PRIMASK(irq);

    for (elapsed = 0U; !(USART2->SR & USART_SR_TXE); ++elapsed)
    {
        if (elapsed >= 100U) return Vision_UART_Status = VISION_UART_TIMEOUT;
        Delay_ms(1);
    }
    USART2->DR = (uint16_t)'s';
    return VISION_UART_OK;
}

/* Parse one complete line if present. Returns 0 while still waiting. */
uint8_t VisionUART_Poll(void)
{
    uint32_t irq;
    uint8_t i;
    uint8_t shape, color;
    char copy[64];

    if (!rx_ready) return 0U;

    irq = __get_PRIMASK();
    __disable_irq();
    for (i = 0U; i < sizeof(copy); ++i)
    {
        copy[i] = rx_line[i];
        if (!copy[i]) break;
    }
    copy[sizeof(copy) - 1U] = '\0';
    rx_length = 0U;
    rx_ready = 0U;
    rx_bad = 0U;
    __set_PRIMASK(irq);

    shape = 0U;
    color = 0U;
    if (ParseVisionLine(copy, &shape, &color))
    {
        Vision_Shape = shape;
        Vision_Color = color;
        return Vision_UART_Status = VISION_UART_OK;
    }

    /* Ignore K230 status lines and keep waiting for the result. */
    if (copy[0] == 'n' && copy[1] == 'o' && copy[2] == 'n' && copy[3] == 'e' && copy[4] == '\0')
    {
        return Vision_UART_Status = VISION_UART_INVALID;
    }

    return 0U;
}

uint8_t VisionUART_WaitResult(uint32_t timeout_ms)
{
    uint32_t elapsed;
    uint8_t status;

    for (elapsed = 0U; elapsed < timeout_ms; ++elapsed)
    {
        status = VisionUART_Poll();
        if (status != 0U) return status;
        Delay_ms(1);
    }

    return Vision_UART_Status = VISION_UART_TIMEOUT;
}

uint8_t VisionUART_RequestShape(uint32_t timeout_ms)
{
    uint8_t status;

    status = VisionUART_BeginRequest();
    if (status != VISION_UART_OK) return status;
    return VisionUART_WaitResult(timeout_ms);
}
