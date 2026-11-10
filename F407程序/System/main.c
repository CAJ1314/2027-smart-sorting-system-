#include "stm32f4xx.h"
#include "Delay.h"
#include "TB6600.h"

#define MOVE_PULSES 15686U
#define MOVE_SPEED_HZ 400U

int main(void)
{
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000000U);
    TB6600_Init();
    Delay_ms(100);

    /* Skip homing and move both axes together once after power-up. */
    TB6600_Move((int32_t)MOVE_PULSES, (int32_t)MOVE_PULSES, MOVE_SPEED_HZ, MOVE_SPEED_HZ);
    while (1) { Delay_ms(1000); }
}
