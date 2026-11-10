#include "ZStepper.h"
#include "Delay.h"

/* 24BYJ48 (5 V, 1:64) + ULN2003 eight-state half-step sequence.
 * Bit order follows IN1, IN2, IN3, IN4. Positive steps lower the Z axis.
 */
#define Z_IN1_PORT GPIOD
#define Z_IN1_PIN  GPIO_Pin_15
#define Z_IN2_PORT GPIOG
#define Z_IN2_PIN  GPIO_Pin_3
#define Z_IN3_PORT GPIOB
#define Z_IN3_PIN  GPIO_Pin_11
#define Z_IN4_PORT GPIOE
#define Z_IN4_PIN  GPIO_Pin_15

static const uint8_t step_table[8] = {0x09U,0x01U,0x03U,0x02U,0x06U,0x04U,0x0CU,0x08U};
static int8_t current_step = 0;
static uint32_t async_steps_remaining = 0U;
static uint16_t async_interval_us = 3000U;
static uint16_t async_elapsed_us = 0U;
static int8_t async_direction = 1;
static uint8_t async_active = 0U;

static void ZStepper_Output(uint8_t index)
{
    uint8_t value = step_table[index];
    GPIO_WriteBit(Z_IN1_PORT, Z_IN1_PIN, (value & 0x01U) ? Bit_SET : Bit_RESET);
    GPIO_WriteBit(Z_IN2_PORT, Z_IN2_PIN, (value & 0x02U) ? Bit_SET : Bit_RESET);
    GPIO_WriteBit(Z_IN3_PORT, Z_IN3_PIN, (value & 0x04U) ? Bit_SET : Bit_RESET);
    GPIO_WriteBit(Z_IN4_PORT, Z_IN4_PIN, (value & 0x08U) ? Bit_SET : Bit_RESET);
}

void ZStepper_Init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOG |
                           RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOE, ENABLE);
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Pin = Z_IN1_PIN; GPIO_Init(Z_IN1_PORT, &gpio);
    gpio.GPIO_Pin = Z_IN2_PIN; GPIO_Init(Z_IN2_PORT, &gpio);
    gpio.GPIO_Pin = Z_IN3_PIN; GPIO_Init(Z_IN3_PORT, &gpio);
    gpio.GPIO_Pin = Z_IN4_PIN; GPIO_Init(Z_IN4_PORT, &gpio);
    ZStepper_Stop();
}

void ZStepper_RunWithService(int32_t steps, uint16_t step_interval_us, void (*service)(uint32_t elapsed_ms))
{
    int8_t direction = (steps >= 0) ? 1 : -1;
    uint32_t count = (steps >= 0) ? (uint32_t)steps : (uint32_t)(-steps);
    uint32_t elapsed_us = 0U;
    uint32_t serviced_elapsed_ms = 0U;
    if (step_interval_us == 0U) step_interval_us = 1000U;
    for (uint32_t i = 0U; i < count; ++i)
    {
        current_step += direction;
        if (current_step > 7) current_step = 0;
        if (current_step < 0) current_step = 7;
        ZStepper_Output((uint8_t)current_step);
        Delay_us(step_interval_us);
        elapsed_us += step_interval_us;
        while (service != 0 && serviced_elapsed_ms < (elapsed_us / 1000U))
        {
            service(++serviced_elapsed_ms);
        }
    }
    ZStepper_Stop();
}

void ZStepper_Run(int32_t steps, uint16_t step_interval_us)
{
    ZStepper_RunWithService(steps, step_interval_us, 0);
}

void ZStepper_StartRun(int32_t steps, uint16_t step_interval_us)
{
    if (step_interval_us < 1000U) step_interval_us = 1000U;
    async_direction = (steps >= 0) ? 1 : -1;
    async_steps_remaining = (steps >= 0) ? (uint32_t)steps : (uint32_t)(-steps);
    async_interval_us = step_interval_us;
    async_elapsed_us = 0U;
    async_active = (async_steps_remaining != 0U) ? 1U : 0U;
    if (async_active == 0U) ZStepper_Stop();
}

void ZStepper_Service1ms(void)
{
    if (async_active == 0U) return;
    async_elapsed_us += 1000U;
    if (async_elapsed_us < async_interval_us) return;
    async_elapsed_us -= async_interval_us;

    current_step += async_direction;
    if (current_step > 7) current_step = 0;
    if (current_step < 0) current_step = 7;
    ZStepper_Output((uint8_t)current_step);

    if (--async_steps_remaining == 0U)
    {
        async_active = 0U;
        ZStepper_Stop();
    }
}

uint8_t ZStepper_IsBusy(void)
{
    return async_active;
}

void ZStepper_Stop(void)
{
    async_active = 0U;
    GPIO_ResetBits(Z_IN1_PORT, Z_IN1_PIN);
    GPIO_ResetBits(Z_IN2_PORT, Z_IN2_PIN);
    GPIO_ResetBits(Z_IN3_PORT, Z_IN3_PIN);
    GPIO_ResetBits(Z_IN4_PORT, Z_IN4_PIN);
}
