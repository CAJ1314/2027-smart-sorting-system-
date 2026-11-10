#ifndef __DRV8825_H
#define __DRV8825_H
#include "stm32f4xx.h"
#define DRV8825_UPPER_STP_PORT GPIOB
#define DRV8825_UPPER_STP_PIN GPIO_Pin_5
#define DRV8825_UPPER_DIR_PORT GPIOB
#define DRV8825_UPPER_DIR_PIN GPIO_Pin_6
#define DRV8825_UPPER_EN_PORT GPIOB
#define DRV8825_UPPER_EN_PIN GPIO_Pin_7
void DRV8825_Upper_Init(void);
void DRV8825_Upper_Run(int32_t steps, uint16_t hz);
void DRV8825_Upper_Stop(void);
#endif
