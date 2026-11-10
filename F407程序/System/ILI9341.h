#ifndef __ILI9341_H
#define __ILI9341_H

#include "stm32f4xx.h"

/* ==================== 颜色定义 (RGB565) ==================== */
#define ILI9341_RED       0xF800
#define ILI9341_GREEN     0x07E0
#define ILI9341_BLUE      0x001F
#define ILI9341_WHITE     0xFFFF
#define ILI9341_BLACK     0x0000
#define ILI9341_YELLOW    0xFFE0
#define ILI9341_CYAN      0x07FF
#define ILI9341_MAGENTA   0xF81F
#define ILI9341_GRAY      0x8410
#define ILI9341_ORANGE    0xFC00

/* ==================== 屏幕参数 ==================== */
#define ILI9341_WIDTH     240
#define ILI9341_HEIGHT    320

/* ==================== 硬件引脚 (软件SPI, current wiring) ==================== */
#define ILI9341_CS_LOW()   GPIO_ResetBits(GPIOG, GPIO_Pin_1)
#define ILI9341_CS_HIGH()  GPIO_SetBits(GPIOG, GPIO_Pin_1)

/* ==================== 函数声明 ==================== */
void ILI9341_Init(void);
void ILI9341_SetWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void ILI9341_FillScreen(uint16_t color);
void ILI9341_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ILI9341_WriteCmd(uint8_t cmd);
void ILI9341_WriteData(uint8_t data);
void ILI9341_WriteData16(uint16_t data);
void ILI9341_DrawString(uint16_t x, uint16_t y, const char *str,
                        uint16_t color, uint16_t bg, uint8_t scale);

void ILI9341_DrawImageRGB565(uint16_t x, uint16_t y,
    uint16_t w, uint16_t h, const uint8_t *pixels, uint32_t length);

#endif
