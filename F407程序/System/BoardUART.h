#ifndef BOARD_UART_H
#define BOARD_UART_H
#include "stm32f4xx.h"
void BoardUART_Init(void);
void BoardUART_Send(const char *text);
/* Caller provides a buffer of at least 32 bytes. */
uint8_t BoardUART_ReadLine(char *out);
/* Poll one or more lines; returns 1 when SENSOR:XX was parsed. */
uint8_t BoardUART_PollSensor(uint8_t *mask);
extern volatile uint8_t BoardUART_ReleaseStatus;
uint8_t BoardUART_RequestRelease(uint32_t timeout_ms);
uint8_t BoardUART_RequestGateReset(uint32_t timeout_ms);
#endif
