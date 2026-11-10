#ifndef __TB6600_H
#define __TB6600_H
#include "stm32f4xx.h"

/* X axis: PUL=PB5, DIR=PB6, EN=PB7; positive direction is DIR high. */
#define TB6600_X_PUL_PORT GPIOB
#define TB6600_X_PUL_PIN  GPIO_Pin_5
#define TB6600_X_DIR_PORT GPIOB
#define TB6600_X_DIR_PIN  GPIO_Pin_6
#define TB6600_X_EN_PORT  GPIOB
#define TB6600_X_EN_PIN   GPIO_Pin_7
#define TB6600_X_LIMIT_PORT GPIOD
#define TB6600_X_LIMIT_PIN  GPIO_Pin_0 /* NO to GND: released=1, pressed=0 */

/* Y axis: PUL=PA5, DIR=PA6, EN=PA7; positive direction remains DIR high. */
#define TB6600_Y_PUL_PORT GPIOA
#define TB6600_Y_PUL_PIN  GPIO_Pin_5
#define TB6600_Y_DIR_PORT GPIOA
#define TB6600_Y_DIR_PIN  GPIO_Pin_6
#define TB6600_Y_EN_PORT  GPIOA
#define TB6600_Y_EN_PIN   GPIO_Pin_7
#define TB6600_Y_LIMIT_PORT GPIOD
#define TB6600_Y_LIMIT_PIN  GPIO_Pin_1 /* NO to GND: released=1, pressed=0 */

/* Lower turntable: PUL=PA1, DIR=PA0, EN=PF8. */
#define TB6600_LOWER_PUL_PORT GPIOA
#define TB6600_LOWER_PUL_PIN  GPIO_Pin_1
#define TB6600_LOWER_DIR_PORT GPIOA
#define TB6600_LOWER_DIR_PIN  GPIO_Pin_0
#define TB6600_LOWER_EN_PORT  GPIOF
#define TB6600_LOWER_EN_PIN   GPIO_Pin_8

/* Match the working standalone motor test: PF8 low enables, high disables. */
#define TB6600_LOWER_EN_ACTIVE_HIGH 0U
/* Reference direction for seeking sensor 1. Flip after mechanical test if needed. */
#define TB6600_LOWER_HOME_DIR_LEVEL 0U

#define TB6600_DIR_POSITIVE_X 1U
#define TB6600_DIR_HOME_X     0U
#define TB6600_DIR_POSITIVE_Y 1U
#define TB6600_DIR_HOME_Y     0U
/* Hold-to-run: PC7 NO contact to GND, released=high. */
#define TB6600_RUN_ABORTED 0x80U
#define TB6600_HOME_ERR_X     0x01U
#define TB6600_HOME_ERR_Y     0x02U

extern volatile int32_t TB6600_X_Position;
extern volatile int32_t TB6600_Y_Position;
extern volatile uint8_t TB6600_HomeError;

void TB6600_Init(void);
uint8_t TB6600_RunPressed(void);
/* Returns zero on success; bit 0=X timeout, bit 1=Y timeout. */
uint8_t TB6600_HomeBoth(uint16_t pulse_hz, uint32_t timeout_ms);
/* Same homing loop, but calls pump() about once per millisecond so UART
 * reception can run while both axes are still returning to zero. */
uint8_t TB6600_HomeBothPump(uint16_t pulse_hz, uint32_t timeout_ms, void (*pump)(void));
/* Signed steps: positive moves in configured positive direction; limits stop only the axis moving toward home. */
uint8_t TB6600_Move(int32_t x_steps, int32_t y_steps, uint16_t x_hz, uint16_t y_hz);
/* Same move with a roughly 1 ms service callback during motion. */
uint8_t TB6600_MovePump(int32_t x_steps, int32_t y_steps, uint16_t x_hz, uint16_t y_hz, void (*pump)(void));
void TB6600_StopBoth(void);
void TB6600_LowerInit(void);
void TB6600_LowerEnable(uint8_t enable);
void TB6600_LowerSetDirection(uint8_t high);
void TB6600_LowerStep(uint32_t half_period_us);
void TB6600_LowerStop(void);
void TB6600_LowerAsyncStart(uint8_t direction, uint16_t half_period_us);
void TB6600_LowerAsyncStop(void);
uint32_t TB6600_LowerAsyncSteps(void);
void TB6600_LowerTimerIRQ(void);
#endif


