#ifndef SCREEN_TABLE_H
#define SCREEN_TABLE_H

#include "stm32f4xx.h"

/* Initialize the SPI TFT display using SDI=PE9, CLK=PE8, CS=PG1, DC=PF15. */
void ScreenTable_Init(void);

/* Select page "main" and show six empty data rows. */
void ScreenTable_DrawFrame(void);
void ScreenTable_ResetData(void);

/* category = color * 8 + shape; rows are created in first-appearance order. */
void ScreenTable_UpdateItem(uint8_t shape, uint8_t category);

#endif
