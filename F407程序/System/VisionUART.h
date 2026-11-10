#ifndef VISION_UART_H
#define VISION_UART_H
#include "stm32f4xx.h"

#define VISION_UART_OK      1U
#define VISION_UART_TIMEOUT 2U
#define VISION_UART_INVALID 4U

/* Shape: 1..7. Color: 0 unknown, 1 red,2 black,3 yellow,4 dark blue,5 light blue,6 orange,7 green. */
extern volatile uint8_t Vision_Shape;
extern volatile uint8_t Vision_Color;
extern volatile uint8_t Vision_UART_Status;

void VisionUART_Init(void);
/* Blocking request kept for compatibility. */
uint8_t VisionUART_RequestShape(uint32_t timeout_ms);
/* Non-blocking split: Begin sends 's', Poll parses a complete reply,
 * WaitResult polls until a result or timeout. */
uint8_t VisionUART_BeginRequest(void);
uint8_t VisionUART_Poll(void);
uint8_t VisionUART_WaitResult(uint32_t timeout_ms);

#endif
