#include "DRV8825.h"
#include "Delay.h"

static void InitPins(void)
{
    GPIO_InitTypeDef g;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    g.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    g.GPIO_Mode = GPIO_Mode_OUT;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &g);
    GPIO_ResetBits(GPIOB, GPIO_Pin_5 | GPIO_Pin_6);
    GPIO_SetBits(GPIOB, GPIO_Pin_7);
}

void DRV8825_Upper_Init(void)
{
    InitPins();
    GPIO_ResetBits(DRV8825_UPPER_EN_PORT, DRV8825_UPPER_EN_PIN);
}

void DRV8825_Upper_Run(int32_t steps, uint16_t hz)
{
    uint32_t i, count;
    uint32_t half;
    if (steps < 0) {
        GPIO_SetBits(DRV8825_UPPER_DIR_PORT, DRV8825_UPPER_DIR_PIN);
        count = (uint32_t)(-steps);
    } else {
        GPIO_ResetBits(DRV8825_UPPER_DIR_PORT, DRV8825_UPPER_DIR_PIN);
        count = (uint32_t)steps;
    }
    if (hz < 50) hz = 50;
    if (hz > 5000) hz = 5000;
    half = 500000U / hz;
    if (half < 10) half = 10;
    Delay_ms(2);
    for (i = 0; i < count; ++i) {
        GPIO_SetBits(DRV8825_UPPER_STP_PORT, DRV8825_UPPER_STP_PIN);
        Delay_us(half);
        GPIO_ResetBits(DRV8825_UPPER_STP_PORT, DRV8825_UPPER_STP_PIN);
        Delay_us(half);
    }
}

void DRV8825_Upper_Stop(void)
{
    GPIO_ResetBits(DRV8825_UPPER_STP_PORT, DRV8825_UPPER_STP_PIN);
}
