#ifndef __Z_STEPPER_H
#define __Z_STEPPER_H
#include "stm32f4xx.h"

void ZStepper_Init(void);
void ZStepper_Run(int32_t steps, uint16_t step_interval_us);
void ZStepper_RunWithService(int32_t steps, uint16_t step_interval_us, void (*service)(uint32_t elapsed_ms));
void ZStepper_StartRun(int32_t steps, uint16_t step_interval_us);
void ZStepper_Service1ms(void);
uint8_t ZStepper_IsBusy(void);
void ZStepper_Stop(void);

#endif
