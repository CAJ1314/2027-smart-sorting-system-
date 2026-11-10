#include "BoardUART.h"
#include <string.h>

static volatile char rx_line[32];
static volatile uint8_t rx_length, rx_ready, rx_bad;

void USART3_IRQHandler(void)
{
    uint32_t status = USART3->SR;
    char c;
    if (status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE))
    {
        c = (char)USART3->DR;
        if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) { rx_bad = 1U; return; }
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

uint8_t BoardUART_ReadLine(char *out)
{
    uint32_t irq;
    uint8_t i;
    if (!rx_ready) return 0U;
    irq = __get_PRIMASK();
    __disable_irq();
    for (i = 0U; i < sizeof(rx_line); ++i) { out[i] = rx_line[i]; if (!out[i]) break; }
    rx_length = 0U; rx_ready = 0U; rx_bad = 0U;
    __set_PRIMASK(irq);
    return 1U;
}

void BoardUART_Send(const char *text)
{
    while (*text)
    {
        while (!(USART3->SR & USART_SR_TXE)) {}
        USART3->DR = (uint8_t)*text++;
    }
    while (!(USART3->SR & USART_SR_TC)) {}
}

#include "Delay.h"
void BoardUART_Init(void)
{
    GPIO_InitTypeDef g;
    RCC_ClocksTypeDef clocks;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);
    g.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    g.GPIO_Mode = GPIO_Mode_AF; g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz; g.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &g);
    RCC_GetClocksFreq(&clocks);
    USART3->CR1 = 0U; USART3->CR2 = 0U; USART3->CR3 = 0U;
    USART3->BRR = (clocks.PCLK1_Frequency + 57600U) / 115200U;
    USART3->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    NVIC_SetPriority(USART3_IRQn, 4U); NVIC_EnableIRQ(USART3_IRQn);
}

volatile uint8_t BoardUART_ReleaseStatus = 0U;

uint8_t BoardUART_RequestRelease(uint32_t timeout_ms)
{
    uint32_t elapsed;
    char line[32];
    while (BoardUART_ReadLine(line)) {}
    BoardUART_ReleaseStatus = 0U;
    BoardUART_Send("RELEASE\r\n");
    for (elapsed = 0U; elapsed < timeout_ms; ++elapsed)
    {
        if (BoardUART_ReadLine(line) && strcmp(line, "DONE") == 0)
            return BoardUART_ReleaseStatus = 1U;
        Delay_ms(1);
    }
    /* No automatic retry: avoid releasing the same load twice. */
    return BoardUART_ReleaseStatus = 2U;
}

uint8_t BoardUART_RequestGateReset(uint32_t timeout_ms)
{
    uint32_t elapsed;
    char line[32];
    while (BoardUART_ReadLine(line)) {}
    BoardUART_ReleaseStatus = 0U;
    BoardUART_Send("RESET\r\n");
    for (elapsed = 0U; elapsed < timeout_ms; ++elapsed)
    {
        if (BoardUART_ReadLine(line) && strcmp(line, "DONE") == 0)
            return BoardUART_ReleaseStatus = 1U;
        Delay_ms(1);
    }
    return BoardUART_ReleaseStatus = 2U;
}

static uint8_t SensorHexValue(char c)
{
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    return 0xFFU;
}

uint8_t BoardUART_PollSensor(uint8_t *mask)
{
    char line[32];
    uint8_t high, low;

    while (BoardUART_ReadLine(line))
    {
            if (strlen(line) == 9U && strncmp(line, "SENSOR:", 7U) == 0)
        {
            high = SensorHexValue(line[7]);
            low = SensorHexValue(line[8]);
            if (high != 0xFFU && low != 0xFFU)
            {
                *mask = (uint8_t)((high << 4) | low);
                return 1U;
            }
        }
    }
    return 0U;
}

