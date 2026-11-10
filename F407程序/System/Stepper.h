#ifndef __STEPPER_H
#define __STEPPER_H
#include "stm32f4xx.h"

/* ================================================================
   TB6600 步进电机驱动 — 适用于上下层转盘
   ----------------------------------------------------------------
   硬件连接:
     STM32 → TB6600:
       下层: PB0=PUL  PE0=DIR  PB2=EN
       上层: PB5=PUL  PB6=DIR  PB7=EN
       TB6600 的 PUL/DIR/ENA 分别接 PUL/DIR/EN；共阳/共阴接法按驱动器端子说明接线
     TB6600 → 步进电机 (万用表实测线圈):
       1A = 黑      1B = 红      ← A相 (导通)
       2A = 绿      2B = 蓝      ← B相 (导通)
       注意: 电机线标A+/A-/B+/B-不一定对应实际线圈, 以实测为准!
   ----------------------------------------------------------------
   ENA 极性: LOW=使能(电机工作), HIGH=释放(电机脱机)
   PUL: 上升沿触发一步
   细分: 1/32 (TB6600 拨码开关)
   校准: 转一圈=33400个脉冲（当前机械系统实测值）
   调用关系: GPIO位带 + __disable_irq 保证时序
   ================================================================ */

/* ---- 下层转盘 (盒子输送) 引脚定义 ---- */
#define Stepper_LOWER_PUL_PORT     GPIOB
#define Stepper_LOWER_PUL_PIN      GPIO_Pin_0
#define Stepper_LOWER_DIR_PORT     GPIOE
#define Stepper_LOWER_DIR_PIN      GPIO_Pin_0
#define Stepper_LOWER_EN_PORT      GPIOB
#define Stepper_LOWER_EN_PIN       GPIO_Pin_2

/* ---- 上层转盘 (存料区送料) 引脚定义 ---- */
#define Stepper_UPPER_PUL_PORT     GPIOB
#define Stepper_UPPER_PUL_PIN      GPIO_Pin_5
#define Stepper_UPPER_DIR_PORT     GPIOB
#define Stepper_UPPER_DIR_PIN      GPIO_Pin_6
#define Stepper_UPPER_EN_PORT      GPIOB
#define Stepper_UPPER_EN_PIN       GPIO_Pin_7

/* ---- 参数 ---- */
/* 转一圈实测: 33400 个脉冲（TB6600，1/32细分） */
#define Stepper_STEPS_PER_REV      33400

/* 方向宏 */
#define Stepper_DIR_CW             0   /* 顺时针 */
#define Stepper_DIR_CCW            1   /* 逆时针 */

/* ---- 下层转盘 API ---- */
void Stepper_Lower_Init(void);
void Stepper_Lower_Run(int32_t steps, uint16_t hz);
void Stepper_Lower_RunOneRev(uint16_t hz);
uint8_t Stepper_Lower_RunUntilSensor(uint8_t sensorIndex, uint16_t hz);
uint8_t Stepper_Lower_RunUntilSensorTimeout(uint8_t sensorIndex, uint16_t hz, uint32_t maxSteps);
void Stepper_Lower_Stop(void);
uint8_t Stepper_Lower_Reset(uint16_t hz, uint32_t maxSteps);

/* ---- 上层转盘 API ---- */
void Stepper_Upper_Init(void);
void Stepper_Upper_Run(int32_t steps, uint16_t hz);
void Stepper_Upper_RotateDeg(uint16_t degrees, uint16_t hz);
void Stepper_Upper_Stop(void);

#endif







