#include "GripperServo.h"
#include "Delay.h"

/* PE6 GPIO PWM driven by TIM6, separate from PA15/TIM2 and SysTick.
 * Two interrupts per 20 ms frame keep PWM running during blocking motion.
 * 500..2500 us maps nominally to 0..180 degrees; 90 degrees = 1500 us.
 */
#define GRIPPER_FRAME_US 20000U
#define GRIPPER_FRAME_MS 20UL
#define GRIPPER_MIN_US 500U
#define GRIPPER_MAX_US 2500U

static volatile uint16_t current_pulse_us = 1500U;
static volatile uint8_t pulse_high = 0U;
static volatile uint8_t ramp_active = 0U;
static int32_t ramp_start_us;
static int32_t ramp_delta_us;
static uint32_t ramp_frames;
static uint32_t ramp_frame_index;

/* The gripper linkage uses a slightly wider opening than 180 deg. */
#define GRIPPER_MAX_ANGLE_DEG 190U

static uint16_t AngleToPulse(uint16_t angle_deg)
{
    if (angle_deg > GRIPPER_MAX_ANGLE_DEG) angle_deg = GRIPPER_MAX_ANGLE_DEG;
    return (uint16_t)(GRIPPER_MIN_US +
        ((uint32_t)angle_deg * (GRIPPER_MAX_US - GRIPPER_MIN_US) + 90U) / 180U);
}

void GripperServo_Init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_ClocksTypeDef clocks;
    uint32_t timer_clock;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM6, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM6, DISABLE);
    GPIO_ResetBits(GPIOE, GPIO_Pin_6);
    gpio.GPIO_Pin = GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOE, &gpio);

    current_pulse_us = AngleToPulse(GRIPPER_OPEN_ANGLE_DEG);
    ramp_active = 0U;
    pulse_high = 1U;
    RCC_GetClocksFreq(&clocks);
    timer_clock = clocks.PCLK1_Frequency;
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U) timer_clock *= 2U;
    TIM6->CR1 = 0U; /* ARR changes take effect immediately for each edge. */
    TIM6->PSC = timer_clock / 1000000U - 1U;
    TIM6->ARR = current_pulse_us - 1U;
    TIM6->EGR = TIM_EGR_UG;
    TIM6->SR = 0U;
    TIM6->CNT = 0U;
    NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
    NVIC_SetPriority(TIM6_DAC_IRQn, 0U);
    TIM6->DIER = TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    GPIO_SetBits(GPIOE, GPIO_Pin_6);
    TIM6->CR1 = TIM_CR1_CEN;
}

void GripperServo_StartMove(uint16_t angle_deg, uint32_t duration_ms)
{
    uint16_t target_us;
    uint32_t irq_state;

    target_us = AngleToPulse(angle_deg);
    irq_state = __get_PRIMASK();
    __disable_irq();
    if (target_us == current_pulse_us || duration_ms == 0U)
    {
        current_pulse_us = target_us;
        ramp_active = 0U;
    }
    else
    {
        ramp_start_us = current_pulse_us;
        ramp_delta_us = (int32_t)target_us - ramp_start_us;
        ramp_frames = duration_ms / GRIPPER_FRAME_MS;
        if ((duration_ms % GRIPPER_FRAME_MS) != 0U) ++ramp_frames;
        ramp_frame_index = 0U;
        ramp_active = 1U;
    }
    __set_PRIMASK(irq_state);
}

uint8_t GripperServo_IsBusy(void)
{
    return ramp_active;
}

void GripperServo_MoveTimed(uint16_t angle_deg, uint32_t duration_ms)
{
    GripperServo_MoveTimedPump(angle_deg, duration_ms, 0);
}

void GripperServo_MoveTimedPump(uint16_t angle_deg, uint32_t duration_ms, void (*pump)(void))
{
    GripperServo_StartMove(angle_deg, duration_ms);
    /* Timer schedules the trajectory; PC7 does not interrupt this stage. */
    while (ramp_active != 0U)
    {
        if (pump != 0) pump();
        Delay_ms(1);
    }
}

void TIM6_DAC_IRQHandler(void)
{
    if ((TIM6->SR & TIM_SR_UIF) == 0U) return;
    TIM6->SR = 0U;
    if (pulse_high != 0U)
    {
        GPIO_ResetBits(GPIOE, GPIO_Pin_6);
        TIM6->ARR = GRIPPER_FRAME_US - current_pulse_us - 1U;
        pulse_high = 0U;
    }
    else
    {
        if (ramp_active != 0U)
        {
            ++ramp_frame_index;
            current_pulse_us = (uint16_t)(ramp_start_us +
                ((int64_t)ramp_delta_us * ramp_frame_index) / ramp_frames);
            if (ramp_frame_index >= ramp_frames) ramp_active = 0U;
        }
        GPIO_SetBits(GPIOE, GPIO_Pin_6);
        TIM6->ARR = current_pulse_us - 1U;
        pulse_high = 1U;
    }
    TIM6->CNT = 0U;
}
