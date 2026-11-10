#ifndef Z_SERVO_H
#define Z_SERVO_H
#include "stm32f4xx.h"
#define Z_SERVO_STOP_US 1500U /* Tune if the continuous servo creeps. */
void ZServo_Init(void);
void ZServo_SetPulse(uint16_t pulse_us);
void ZServo_Stop(void);
uint8_t ZServo_RunTimed(uint16_t pulse_us, uint32_t duration_ms);
#endif
