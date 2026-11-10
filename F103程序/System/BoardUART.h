#ifndef BOARD_UART_H
#define BOARD_UART_H
#include "stm32f10x.h"
void BoardUART_Init(void);
void BoardUART_Send(const char *text);
/* Caller provides a buffer of at least 32 bytes. */
uint8_t BoardUART_ReadLine(char *out);
#endif
