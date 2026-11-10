#include "ObjectUART.h"
#include "TB6600.h"
#include "Delay.h"
#include <string.h>

#define OBJECT_X_MAX_CENTI_MM 17000L /* 对齐K230夹爪坐标上限17cm，单位0.01mm */
#define OBJECT_Y_MAX_CENTI_MM 13000L /* 对齐K230夹爪坐标上限13cm，单位0.01mm */

volatile int32_t Object_X_centi_mm = 0;
volatile int32_t Object_Y_centi_mm = 0;
volatile uint8_t Object_IsCube = 0U;
volatile uint8_t Object_IsCylinder = 0U;
volatile uint8_t Object_UART_Status = 0U;
static volatile char line[64];
static volatile uint8_t length = 0U, ready = 0U, overflow = 0U;

void ObjectUART_Init(void)
{
    GPIO_InitTypeDef g;
    RCC_ClocksTypeDef clocks;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
    g.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    g.GPIO_Mode = GPIO_Mode_AF;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &g);
    RCC_GetClocksFreq(&clocks);
    USART1->CR1 = 0U;
    USART1->CR2 = 0U;
    USART1->CR3 = 0U;
    USART1->BRR = (clocks.PCLK2_Frequency + 57600U) / 115200U;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    NVIC_SetPriority(USART1_IRQn, 3U);
    NVIC_EnableIRQ(USART1_IRQn);
}

void USART1_IRQHandler(void)
{
    uint32_t status = USART1->SR;
    char c;
    if (status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
        c = (char)USART1->DR;
        if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
            overflow = 1U;
            return;
        }
        if (ready) return;
        if (c == '\n') {
            if (overflow) { line[0] = '\0'; ready = 1U; }
            else if (length) { line[length] = '\0'; ready = 1U; }
        } else if (c != '\r') {
            if (length < sizeof(line) - 1U) line[length++] = c;
            else overflow = 1U;
        }
    }
}

/* Parse K230 decimal centimeters (up to two decimals) without floats.
 * The result is 0.01 cm units; convert to 0.01 mm when storing.
 */
static uint8_t ParseNumber(const char **text, int32_t *value)
{
    const char *p = *text;
    int32_t whole = 0, fraction = 0;
    uint8_t negative = 0U, digits = 0U, decimals = 0U;
    if (*p == '-' || *p == '+') { negative = (*p == '-'); ++p; }
    while (*p >= '0' && *p <= '9') {
        if (++digits > 5U) return 0U;
        whole = whole * 10 + (*p++ - '0');
    }
    if (!digits) return 0U;
    if (*p == '.') {
        ++p;
        while (*p >= '0' && *p <= '9') {
            if (++decimals > 2U) return 0U;
            fraction = fraction * 10 + (*p++ - '0');
        }
        if (!decimals) return 0U;
        if (decimals == 1U) fraction *= 10;
    }
    *value = whole * 100 + fraction;
    if (negative) *value = -*value;
    *text = p;
    return 1U;
}

uint8_t ObjectUART_RequestPump(uint32_t timeout_ms, void (*pump)(void))
{
    uint32_t elapsed;
    uint32_t irq;
    char copy[64];
    uint8_t i;
    const char *p;
    int32_t x, y;
    char *field[7];
    char *scan;
    uint8_t fieldIndex;
    Object_UART_Status = 0U;
    if (!TB6600_RunPressed()) return Object_UART_Status = OBJECT_UART_STOPPED;
    irq = __get_PRIMASK();
    __disable_irq();
    length = ready = overflow = 0U;
    (void)USART1->SR;
    (void)USART1->DR;
    __set_PRIMASK(irq);
    for (elapsed = 0U; !(USART1->SR & USART_SR_TXE); ++elapsed) {
        if (!TB6600_RunPressed()) return Object_UART_Status = OBJECT_UART_STOPPED;
        if (elapsed >= 100U) return Object_UART_Status = OBJECT_UART_TIMEOUT;
        if (pump != 0) pump();
        Delay_ms(1);
    }
    USART1->DR = (uint16_t)'s';
    for (elapsed = 0U; elapsed < timeout_ms; ++elapsed) {
        if (!TB6600_RunPressed()) return Object_UART_Status = OBJECT_UART_STOPPED;
        if (ready) {
            for (i = 0U; i < sizeof(copy); ++i) copy[i] = line[i];
            copy[sizeof(copy) - 1U] = '\0';
            /* K230: TARGET,1,<shape>,<color>,<x>,<y>,CM */
            field[0] = copy;
            scan = copy;
            for (fieldIndex = 0U; fieldIndex < 6U; ++fieldIndex)
            {
                scan = strchr(scan, ',');
                if (scan == 0) return Object_UART_Status = OBJECT_UART_INVALID;
                *scan++ = '\0';
                field[fieldIndex + 1U] = scan;
            }
            if (strcmp(field[0], "TARGET") != 0 || strcmp(field[1], "1") != 0 ||
                field[2][0] == '\0' || field[3][0] == '\0' ||
                strcmp(field[6], "CM") != 0)
                return Object_UART_Status = OBJECT_UART_INVALID;
            p = field[4];
            if (!ParseNumber(&p, &x) || *p != '\0')
                return Object_UART_Status = OBJECT_UART_INVALID;
            p = field[5];
            if (!ParseNumber(&p, &y) || *p != '\0')
                return Object_UART_Status = OBJECT_UART_INVALID;
            Object_X_centi_mm = x * 10;
            Object_Y_centi_mm = y * 10;
            Object_IsCube = (strcmp(field[2], "zhengfangti") == 0) ? 1U : 0U;
            Object_IsCylinder = (strcmp(field[2], "yuanzhu") == 0) ? 1U : 0U;
            return Object_UART_Status = OBJECT_UART_OK;
        }
        if (pump != 0) pump();
        Delay_ms(1);
    }
    return Object_UART_Status = OBJECT_UART_TIMEOUT;
}

uint8_t ObjectUART_Request(uint32_t timeout_ms)
{
    return ObjectUART_RequestPump(timeout_ms, 0);
}

uint8_t ObjectUART_IsValid(void)
{
    /* Coordinates are measured from the XY home origin, in positive mm. */
    if (Object_UART_Status != OBJECT_UART_OK) return 0U;
    if (Object_X_centi_mm < 0 || Object_Y_centi_mm < 0) return 0U;
    if (Object_X_centi_mm > OBJECT_X_MAX_CENTI_MM ||
        Object_Y_centi_mm > OBJECT_Y_MAX_CENTI_MM) return 0U;
    return 1U;
}

