#ifndef TJC_DISPLAY_H
#define TJC_DISPLAY_H

#include "stm32f4xx.h"

/*
 * TJC intelligent display link:
 *   UART5_TX = PC12 -> display RX
 *   UART5_RX = PD2  <- display TX
 *   115200 baud, 8 data bits, no parity, 1 stop bit.
 *
 * Every TJC command is terminated by three 0xFF bytes.
 */
void TJCDisplay_Init(void);
void TJCDisplay_SendCommand(const char *command);
void TJCDisplay_Page(const char *page);
void TJCDisplay_SetText(const char *component, const char *text);
void TJCDisplay_SetValue(const char *component, uint32_t value);
void TJCDisplay_SetPicture(const char *component, uint16_t picture_id);
void TJCDisplay_SetVisible(const char *component, uint8_t visible);

#endif
