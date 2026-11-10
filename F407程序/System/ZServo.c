#include "ZServo.h"
#include "Delay.h"

/* PA15 AF1 = TIM2_CH1. Hardware PWM: 1 us ticks, 20 ms period.
 * Continuous rotation: pulse width controls speed/direction, not angle.
 */

static uint8_t pwm_started = 0U;

void ZServo_Init(void)
{
    GPIO_InitTypeDef g;
    RCC_ClocksTypeDef clocks;
    uint32_t timer_clock;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM2, DISABLE);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);
    g.GPIO_Pin = GPIO_Pin_15;
    g.GPIO_Mode = GPIO_Mode_AF;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &g);
    RCC_GetClocksFreq(&clocks);
    timer_clock = clocks.PCLK1_Frequency;
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U) timer_clock *= 2U;
    TIM2->CR1 = TIM_CR1_ARPE;
    TIM2->PSC = timer_clock / 1000000U - 1U;
    TIM2->ARR = 19999U;
    TIM2->CCR1 = 0U;
    /* PWM mode 1 (OC1M=110), compare preload enabled. */
    TIM2->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;
    TIM2->CCER = 0U;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0U;
    pwm_started = 0U;
    ZServo_Stop();
}

void ZServo_SetPulse(uint16_t pulse_us)
{
    if (pulse_us < 500U) pulse_us = 500U;
    if (pulse_us > 2500U) pulse_us = 2500U;
    TIM2->CCR1 = pulse_us;
    if (!pwm_started) {
        TIM2->EGR = TIM_EGR_UG;
        TIM2->SR = 0U;
        TIM2->CNT = 0U;
        TIM2->CCER = TIM_CCER_CC1E;
        TIM2->CR1 |= TIM_CR1_CEN;
        pwm_started = 1U;
    }
}

void ZServo_Stop(void)
{
    ZServo_SetPulse(Z_SERVO_STOP_US);
}

uint8_t ZServo_RunTimed(uint16_t pulse_us, uint32_t duration_ms)
{
    uint32_t elapsed;
    ZServo_SetPulse(pulse_us);
    for (elapsed = 0U; elapsed < duration_ms; ++elapsed) {
        Delay_ms(1);
    }
    ZServo_Stop();
    return 1U;
}
