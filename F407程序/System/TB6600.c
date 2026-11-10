#include "TB6600.h"
#include "Delay.h"

#define TICK_US 10U
#define LIMIT_DEBOUNCE_TICKS 1000U /* 10 ms */
#define HOME_TIMEOUT_TICKS(ms) ((uint32_t)(ms) * 100U)

static volatile uint8_t lower_async_active = 0U;
static volatile uint8_t lower_async_pulse_high = 0U;
static volatile uint32_t lower_async_steps = 0U;

volatile int32_t TB6600_X_Position = 0;
volatile int32_t TB6600_Y_Position = 0;
volatile uint8_t TB6600_HomeError = 0U;

static void InitOutput(GPIO_TypeDef *port, uint16_t pins)
{
    GPIO_InitTypeDef g;
    g.GPIO_Pin = pins;
    g.GPIO_Mode = GPIO_Mode_OUT;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(port, &g);
}

static void InitLimitInputs(void)
{
    GPIO_InitTypeDef g;
    g.GPIO_Pin = TB6600_X_LIMIT_PIN | TB6600_Y_LIMIT_PIN;
    g.GPIO_Mode = GPIO_Mode_IN;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    g.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &g);
}

/* X uses NO-to-GND: contact closes on press, internal pull-up reads low. */
static uint8_t XLimitRaw(void)
{
    return (GPIO_ReadInputDataBit(TB6600_X_LIMIT_PORT, TB6600_X_LIMIT_PIN) == Bit_RESET) ? 1U : 0U;
}
/* Y uses NO-to-GND: contact closes on press, internal pull-up reads low. */
static uint8_t YLimitRaw(void)
{
    return (GPIO_ReadInputDataBit(TB6600_Y_LIMIT_PORT, TB6600_Y_LIMIT_PIN) == Bit_RESET) ? 1U : 0U;
}

static void InitRunInput(void)
{
    GPIO_InitTypeDef g;
    g.GPIO_Pin = GPIO_Pin_7;
    g.GPIO_Mode = GPIO_Mode_IN;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    g.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &g);
}

uint8_t TB6600_RunPressed(void)
{
    return (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7) == Bit_RESET) ? 1U : 0U;
}

void TB6600_Init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
    InitOutput(GPIOB, GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);
    InitOutput(GPIOA, GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);
    InitLimitInputs();
    InitRunInput();
    GPIO_ResetBits(GPIOB, GPIO_Pin_5 | GPIO_Pin_6);
    GPIO_ResetBits(GPIOA, GPIO_Pin_5 | GPIO_Pin_6);
    /* TB6600 ENA assumed active-low; verify driver input wiring before powering motors. */
    GPIO_ResetBits(GPIOB, GPIO_Pin_7);
    GPIO_ResetBits(GPIOA, GPIO_Pin_7);
    TB6600_X_Position = 0;
    TB6600_Y_Position = 0;
    TB6600_HomeError = 0U;
}

static uint32_t HalfPeriod(uint16_t hz)
{
    if (hz < 50U) hz = 50U;
    if (hz > 14000U) hz = 14000U;
    return 500000U / (uint32_t)hz;
}

/*
 * Homing: axes seek opposite their established positive directions.
 * A raw switch assertion immediately suppresses further pulses for that axis;
 * it is accepted as home only after 10 ms stable. Search timeout is per-axis.
 */
uint8_t TB6600_HomeBoth(uint16_t pulse_hz, uint32_t timeout_ms)
{
    return TB6600_HomeBothPump(pulse_hz, timeout_ms, 0);
}

uint8_t TB6600_HomeBothPump(uint16_t pulse_hz, uint32_t timeout_ms, void (*pump)(void))
{
    uint32_t half, elapsed = 0U;
    uint32_t x_phase = 0U, y_phase = 0U;
    uint32_t x_debounce = 0U, y_debounce = 0U;
    uint8_t x_high = 0U, y_high = 0U;
    uint8_t x_done, y_done, x_candidate = 0U, y_candidate = 0U;
    uint32_t timeout_ticks;

    TB6600_HomeError = 0U;
    TB6600_X_Position = 0;
    TB6600_Y_Position = 0;
    half = HalfPeriod(pulse_hz);
    timeout_ticks = HOME_TIMEOUT_TICKS(timeout_ms);

    GPIO_ResetBits(TB6600_X_DIR_PORT, TB6600_X_DIR_PIN); /* X home is opposite positive (DIR=0). */
    GPIO_ResetBits(TB6600_Y_DIR_PORT, TB6600_Y_DIR_PIN); /* Y positive is DIR=1, so home DIR=0. */
    Delay_ms(2);

    x_done = XLimitRaw();
    y_done = YLimitRaw();
    if (x_done) TB6600_X_Position = 0;
    if (y_done) TB6600_Y_Position = 0;

    while ((!x_done || !y_done) && elapsed < timeout_ticks)
    {
        uint8_t x_raw, y_raw;
        if (!TB6600_RunPressed()) {
            TB6600_StopBoth();
            TB6600_HomeError = TB6600_RUN_ABORTED;
            return TB6600_HomeError;
        }
        x_raw = XLimitRaw();
        y_raw = YLimitRaw();

        if (!x_done) {
            if (x_raw) {
                if (!x_candidate) { x_candidate = 1U; x_debounce = 0U; }
                if (x_debounce >= LIMIT_DEBOUNCE_TICKS) {
                    x_done = 1U; GPIO_ResetBits(TB6600_X_PUL_PORT, TB6600_X_PUL_PIN);
                    TB6600_X_Position = 0; x_high = 0U;
                }
            } else { x_candidate = 0U; x_debounce = 0U; }
        }
        if (!y_done) {
            if (y_raw) {
                if (!y_candidate) { y_candidate = 1U; y_debounce = 0U; }
                if (y_debounce >= LIMIT_DEBOUNCE_TICKS) {
                    y_done = 1U; GPIO_ResetBits(TB6600_Y_PUL_PORT, TB6600_Y_PUL_PIN);
                    TB6600_Y_Position = 0; y_high = 0U;
                }
            } else { y_candidate = 0U; y_debounce = 0U; }
        }

        if (!x_done && !x_candidate) {
            x_phase += TICK_US;
            if (x_phase >= half) {
                x_phase -= half;
                if (!x_high) { GPIO_SetBits(TB6600_X_PUL_PORT, TB6600_X_PUL_PIN); x_high = 1U; }
                else { GPIO_ResetBits(TB6600_X_PUL_PORT, TB6600_X_PUL_PIN); x_high = 0U; }
            }
        }
        if (!y_done && !y_candidate) {
            y_phase += TICK_US;
            if (y_phase >= half) {
                y_phase -= half;
                if (!y_high) { GPIO_SetBits(TB6600_Y_PUL_PORT, TB6600_Y_PUL_PIN); y_high = 1U; }
                else { GPIO_ResetBits(TB6600_Y_PUL_PORT, TB6600_Y_PUL_PIN); y_high = 0U; }
            }
        }
        if (x_candidate) x_debounce += TICK_US;
        if (y_candidate) y_debounce += TICK_US;
        Delay_us(TICK_US);
        elapsed += 1U;
        /* ~1 ms service point for non-blocking UART work during homing. */
        if ((pump != 0) && ((elapsed % 100U) == 0U)) pump();
    }

    GPIO_ResetBits(TB6600_X_PUL_PORT, TB6600_X_PUL_PIN);
    GPIO_ResetBits(TB6600_Y_PUL_PORT, TB6600_Y_PUL_PIN);
    if (!x_done) TB6600_HomeError |= TB6600_HOME_ERR_X;
    if (!y_done) TB6600_HomeError |= TB6600_HOME_ERR_Y;
    return TB6600_HomeError;
}

/*
 * Synchronized step-and-limit service. A switch hit stops and zeroes only
 * the axis travelling toward home; the other axis continues independently.
 * Return bit 0/1 indicates X/Y was limit-stopped before completing its move.
 */
uint8_t TB6600_MovePump(int32_t x_steps, int32_t y_steps, uint16_t x_hz, uint16_t y_hz, void (*pump)(void))
{
    uint32_t x_total, y_total, x_count = 0U, y_count = 0U;
    uint32_t x_phase = 0U, y_phase = 0U, x_half, y_half;
    uint8_t x_dir, y_dir, x_high = 0U, y_high = 0U;
    uint8_t x_active, y_active, stopped = 0U;
    uint32_t x_debounce = 0U, y_debounce = 0U;
    uint32_t service_ticks = 0U;


    x_total = (x_steps < 0) ? (uint32_t)(-x_steps) : (uint32_t)x_steps;
    y_total = (y_steps < 0) ? (uint32_t)(-y_steps) : (uint32_t)y_steps;
    x_dir = (x_steps < 0) ? TB6600_DIR_HOME_X : TB6600_DIR_POSITIVE_X;
    y_dir = (y_steps < 0) ? TB6600_DIR_HOME_Y : TB6600_DIR_POSITIVE_Y;
    x_half = HalfPeriod(x_hz); y_half = HalfPeriod(y_hz);
    x_active = (x_total != 0U); y_active = (y_total != 0U);

    if (x_dir) GPIO_SetBits(TB6600_X_DIR_PORT, TB6600_X_DIR_PIN); else GPIO_ResetBits(TB6600_X_DIR_PORT, TB6600_X_DIR_PIN);
    if (y_dir) GPIO_SetBits(TB6600_Y_DIR_PORT, TB6600_Y_DIR_PIN); else GPIO_ResetBits(TB6600_Y_DIR_PORT, TB6600_Y_DIR_PIN);
    Delay_ms(2);

    while (x_active || y_active)
    {
        if (!TB6600_RunPressed()) {
            TB6600_StopBoth();
            return stopped | TB6600_RUN_ABORTED;
        }
        if (x_active && x_dir == TB6600_DIR_HOME_X && XLimitRaw()) {
            GPIO_ResetBits(TB6600_X_PUL_PORT, TB6600_X_PUL_PIN); x_high = 0U;
            x_debounce += TICK_US;
            if (x_debounce >= LIMIT_DEBOUNCE_TICKS) { x_active = 0U; stopped |= TB6600_HOME_ERR_X; TB6600_X_Position = 0; }
        } else if (x_active) {
            x_debounce = 0U;
            x_phase += TICK_US;
            if (x_phase >= x_half) {
                x_phase -= x_half;
                if (!x_high) { GPIO_SetBits(TB6600_X_PUL_PORT, TB6600_X_PUL_PIN); x_high = 1U; }
                else { GPIO_ResetBits(TB6600_X_PUL_PORT, TB6600_X_PUL_PIN); x_high = 0U; ++x_count; TB6600_X_Position += (x_dir == TB6600_DIR_POSITIVE_X) ? 1 : -1; }
                if (x_count >= x_total) x_active = 0U;
            }
        }
        if (y_active && y_dir == TB6600_DIR_HOME_Y && YLimitRaw()) {
            GPIO_ResetBits(TB6600_Y_PUL_PORT, TB6600_Y_PUL_PIN); y_high = 0U;
            y_debounce += TICK_US;
            if (y_debounce >= LIMIT_DEBOUNCE_TICKS) { y_active = 0U; stopped |= TB6600_HOME_ERR_Y; TB6600_Y_Position = 0; }
        } else if (y_active) {
            y_debounce = 0U;
            y_phase += TICK_US;
            if (y_phase >= y_half) {
                y_phase -= y_half;
                if (!y_high) { GPIO_SetBits(TB6600_Y_PUL_PORT, TB6600_Y_PUL_PIN); y_high = 1U; }
                else { GPIO_ResetBits(TB6600_Y_PUL_PORT, TB6600_Y_PUL_PIN); y_high = 0U; ++y_count; TB6600_Y_Position += (y_dir == TB6600_DIR_POSITIVE_Y) ? 1 : -1; }
                if (y_count >= y_total) y_active = 0U;
            }
        }
        Delay_us(TICK_US);
        if (pump != 0 && ++service_ticks >= 100U)
        {
            service_ticks = 0U;
            pump();
        }
    }
    GPIO_ResetBits(TB6600_X_PUL_PORT, TB6600_X_PUL_PIN);
    GPIO_ResetBits(TB6600_Y_PUL_PORT, TB6600_Y_PUL_PIN);
    return stopped;
}

uint8_t TB6600_Move(int32_t x_steps, int32_t y_steps, uint16_t x_hz, uint16_t y_hz)
{
    return TB6600_MovePump(x_steps, y_steps, x_hz, y_hz, 0);
}

void TB6600_StopBoth(void)
{
    GPIO_ResetBits(TB6600_X_PUL_PORT, TB6600_X_PUL_PIN);
    GPIO_ResetBits(TB6600_Y_PUL_PORT, TB6600_Y_PUL_PIN);
}






/* Lower turntable: PUL=PA1, DIR=PA0, EN=PF8.
 * Standalone-test configuration drives EN+ low to enable; if the
 * specific TB6600 board is active-low, change TB6600_LOWER_EN_ACTIVE_HIGH.
 */
void TB6600_LowerInit(void)
{
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOF, ENABLE);
    GPIO_ResetBits(GPIOA, GPIO_Pin_1);
    GPIO_WriteBit(GPIOA, GPIO_Pin_0, TB6600_LOWER_HOME_DIR_LEVEL ? Bit_SET : Bit_RESET);
#if TB6600_LOWER_EN_ACTIVE_HIGH
    GPIO_ResetBits(GPIOF, GPIO_Pin_8);
#else
    GPIO_SetBits(GPIOF, GPIO_Pin_8);
#endif
    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_8;
    GPIO_Init(GPIOF, &gpio);
}

void TB6600_LowerEnable(uint8_t enable)
{
#if TB6600_LOWER_EN_ACTIVE_HIGH
    if (enable) GPIO_SetBits(TB6600_LOWER_EN_PORT, TB6600_LOWER_EN_PIN);
    else GPIO_ResetBits(TB6600_LOWER_EN_PORT, TB6600_LOWER_EN_PIN);
#else
    if (enable) GPIO_ResetBits(TB6600_LOWER_EN_PORT, TB6600_LOWER_EN_PIN);
    else GPIO_SetBits(TB6600_LOWER_EN_PORT, TB6600_LOWER_EN_PIN);
#endif
}

void TB6600_LowerSetDirection(uint8_t high)
{
    if (high) GPIO_SetBits(TB6600_LOWER_DIR_PORT, TB6600_LOWER_DIR_PIN);
    else GPIO_ResetBits(TB6600_LOWER_DIR_PORT, TB6600_LOWER_DIR_PIN);
}

void TB6600_LowerStep(uint32_t half_period_us)
{
    GPIO_SetBits(TB6600_LOWER_PUL_PORT, TB6600_LOWER_PUL_PIN);
    Delay_us(half_period_us);
    GPIO_ResetBits(TB6600_LOWER_PUL_PORT, TB6600_LOWER_PUL_PIN);
    Delay_us(half_period_us);
}

void TB6600_LowerStop(void)
{
    TB6600_LowerAsyncStop();
    GPIO_ResetBits(TB6600_LOWER_PUL_PORT, TB6600_LOWER_PUL_PIN);
}

void TB6600_LowerAsyncStart(uint8_t direction, uint16_t half_period_us)
{
    RCC_ClocksTypeDef clocks;
    uint32_t timer_clock;

    if (half_period_us < 2U) half_period_us = 2U;
    TB6600_LowerAsyncStop();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
    RCC_GetClocksFreq(&clocks);
    timer_clock = clocks.PCLK1_Frequency;
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U) timer_clock *= 2U;

    TB6600_LowerSetDirection(direction);
    TB6600_LowerEnable(1U);
    lower_async_steps = 0U;
    lower_async_pulse_high = 0U;
    GPIO_ResetBits(TB6600_LOWER_PUL_PORT, TB6600_LOWER_PUL_PIN);
    TIM7->CR1 = 0U;
    TIM7->PSC = timer_clock / 1000000U - 1U;
    TIM7->ARR = (uint32_t)half_period_us - 1U;
    TIM7->EGR = TIM_EGR_UG;
    TIM7->CNT = 0U;
    TIM7->SR = 0U;
    NVIC_ClearPendingIRQ(TIM7_IRQn);
    NVIC_SetPriority(TIM7_IRQn, 2U);
    TIM7->DIER = TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM7_IRQn);
    lower_async_active = 1U;
    TIM7->CR1 = TIM_CR1_CEN;
}

void TB6600_LowerAsyncStop(void)
{
    if (lower_async_active != 0U)
    {
        TIM7->DIER = 0U;
        TIM7->CR1 = 0U;
        TIM7->SR = 0U;
    }
    lower_async_active = 0U;
    lower_async_pulse_high = 0U;
    GPIO_ResetBits(TB6600_LOWER_PUL_PORT, TB6600_LOWER_PUL_PIN);
}

uint32_t TB6600_LowerAsyncSteps(void)
{
    return lower_async_steps;
}

void TB6600_LowerTimerIRQ(void)
{
    if ((TIM7->SR & TIM_SR_UIF) == 0U) return;
    TIM7->SR = 0U;
    if (lower_async_active == 0U) return;

    if (lower_async_pulse_high == 0U)
    {
        GPIO_SetBits(TB6600_LOWER_PUL_PORT, TB6600_LOWER_PUL_PIN);
        lower_async_pulse_high = 1U;
    }
    else
    {
        GPIO_ResetBits(TB6600_LOWER_PUL_PORT, TB6600_LOWER_PUL_PIN);
        lower_async_pulse_high = 0U;
        ++lower_async_steps;
    }
}

