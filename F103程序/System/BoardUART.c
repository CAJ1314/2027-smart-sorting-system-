#include "BoardUART.h"
#include <string.h>

static volatile char rx_line[32];
static volatile uint8_t rx_length, rx_ready, rx_bad;

void USART1_IRQHandler(void)
{
    uint32_t status = USART1->SR;
    char c;
    if (status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE))
    {
        c = (char)USART1->DR;
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
        while (!(USART1->SR & USART_SR_TXE)) {}
        USART1->DR = (uint8_t)*text++;
    }
    while (!(USART1->SR & USART_SR_TC)) {}
}

void BoardUART_Init(void)
{
    GPIO_InitTypeDef g;
    RCC_ClocksTypeDef clocks;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    g.GPIO_Pin = GPIO_Pin_9; g.GPIO_Mode = GPIO_Mode_AF_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz; GPIO_Init(GPIOA, &g);
    g.GPIO_Pin = GPIO_Pin_10; g.GPIO_Mode = GPIO_Mode_IPU; GPIO_Init(GPIOA, &g);
    RCC_GetClocksFreq(&clocks);
    USART1->CR1 = 0U; USART1->CR2 = 0U; USART1->CR3 = 0U;
    USART1->BRR = (clocks.PCLK2_Frequency + 57600U) / 115200U;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    NVIC_SetPriority(USART1_IRQn, 2U); NVIC_EnableIRQ(USART1_IRQn);
}
