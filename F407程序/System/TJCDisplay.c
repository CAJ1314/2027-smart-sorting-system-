#include "TJCDisplay.h"

#define TJC_BAUD_RATE 115200UL

static void TJCDisplay_DiscardRx(void)
{
    while ((UART5->SR & (USART_SR_RXNE | USART_SR_ORE)) != 0U)
    {
        volatile uint32_t discard = UART5->DR;
        (void)discard;
    }
}

static void TJCDisplay_WriteByte(uint8_t value)
{
    TJCDisplay_DiscardRx();
    while ((UART5->SR & USART_SR_TXE) == 0U) {}
    UART5->DR = value;
}

static void TJCDisplay_WriteText(const char *text)
{
    while (*text != '\0')
    {
        TJCDisplay_WriteByte((uint8_t)*text++);
    }
}

static void TJCDisplay_WriteUnsigned(uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do
    {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U && count < sizeof(digits));

    while (count != 0U)
    {
        TJCDisplay_WriteByte((uint8_t)digits[--count]);
    }
}

static void TJCDisplay_EndCommand(void)
{
    TJCDisplay_WriteByte(0xFFU);
    TJCDisplay_WriteByte(0xFFU);
    TJCDisplay_WriteByte(0xFFU);
    while ((UART5->SR & USART_SR_TC) == 0U) {}
}

void TJCDisplay_Init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_ClocksTypeDef clocks;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);

    gpio.GPIO_Pin = GPIO_Pin_12;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOD, &gpio);

    RCC_GetClocksFreq(&clocks);
    UART5->CR1 = 0U;
    UART5->CR2 = 0U;
    UART5->CR3 = 0U;
    UART5->BRR = (clocks.PCLK1_Frequency + (TJC_BAUD_RATE / 2U)) / TJC_BAUD_RATE;
    UART5->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

void TJCDisplay_SendCommand(const char *command)
{
    TJCDisplay_WriteText(command);
    TJCDisplay_EndCommand();
}

void TJCDisplay_Page(const char *page)
{
    TJCDisplay_WriteText("page ");
    TJCDisplay_WriteText(page);
    TJCDisplay_EndCommand();
}

void TJCDisplay_SetText(const char *component, const char *text)
{
    TJCDisplay_WriteText(component);
    TJCDisplay_WriteText(".txt=\"");
    TJCDisplay_WriteText(text);
    TJCDisplay_WriteByte((uint8_t)'\"');
    TJCDisplay_EndCommand();
}

void TJCDisplay_SetValue(const char *component, uint32_t value)
{
    TJCDisplay_WriteText(component);
    TJCDisplay_WriteText(".val=");
    TJCDisplay_WriteUnsigned(value);
    TJCDisplay_EndCommand();
}

void TJCDisplay_SetPicture(const char *component, uint16_t picture_id)
{
    TJCDisplay_WriteText(component);
    TJCDisplay_WriteText(".pic=");
    TJCDisplay_WriteUnsigned((uint32_t)picture_id);
    TJCDisplay_EndCommand();
}

void TJCDisplay_SetVisible(const char *component, uint8_t visible)
{
    TJCDisplay_WriteText("vis ");
    TJCDisplay_WriteText(component);
    TJCDisplay_WriteByte((uint8_t)',');
    TJCDisplay_WriteByte((uint8_t)(visible ? '1' : '0'));
    TJCDisplay_EndCommand();
}
