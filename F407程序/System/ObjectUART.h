#ifndef OBJECT_UART_H
#define OBJECT_UART_H
#include "stm32f4xx.h"
#define OBJECT_UART_OK 1U
#define OBJECT_UART_TIMEOUT 2U
#define OBJECT_UART_STOPPED 3U
#define OBJECT_UART_INVALID 4U
extern volatile int32_t Object_X_centi_mm;
extern volatile int32_t Object_Y_centi_mm;
extern volatile uint8_t Object_IsCube; /* 本次第一摄像头目标是否为正方体 */
extern volatile uint8_t Object_IsCylinder; /* 本次第一摄像头目标是否为圆柱 */
extern volatile uint8_t Object_UART_Status;
uint8_t ObjectUART_IsValid(void);
void ObjectUART_Init(void);
uint8_t ObjectUART_Request(uint32_t timeout_ms);
uint8_t ObjectUART_RequestPump(uint32_t timeout_ms, void (*pump)(void));
#endif
