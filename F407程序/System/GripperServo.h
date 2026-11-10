#ifndef GRIPPER_SERVO_H
#define GRIPPER_SERVO_H

#include "stm32f4xx.h"

/* SG90 on PE6. Nominal angle mapping; calibrate with the actual linkage. */
#define GRIPPER_OPEN_ANGLE_DEG 170U /* 夹爪张开目标角度(度)；当前机构数值越大张开越大，驱动上限190度 */
#define GRIPPER_CLOSE_ANGLE_DEG 140U /* 夹爪最终闭合目标角度(度)；当前机构数值越小夹得越紧，注意机械堵转 */
#define GRIPPER_MOVE_TIME_MS 1000UL /* 普通夹爪动作时间(ms)；越大越慢，第二段闭合有独立时间计算 */
void GripperServo_Init(void);
/* Blocking move: returns when the ramp has reached the target angle. */
void GripperServo_MoveTimed(uint16_t angle_deg, uint32_t duration_ms);
void GripperServo_MoveTimedPump(uint16_t angle_deg, uint32_t duration_ms, void (*pump)(void));
/* Non-blocking move: starts the TIM6 ramp and returns immediately.
 * Poll GripperServo_IsBusy() to know when the angle is reached. */
void GripperServo_StartMove(uint16_t angle_deg, uint32_t duration_ms);
uint8_t GripperServo_IsBusy(void);

#endif

