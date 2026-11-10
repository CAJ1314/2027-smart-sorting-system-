#include "Delay.h"

/**
  * @brief  微秒级延时 (HCLK=168MHz)
  * @param  xus 延时时长, 范围 0~233015
  * @retval 无
  */
void Delay_us(uint32_t xus)
{
	SysTick->LOAD = 168 * xus;
	SysTick->VAL  = 0x00;
	SysTick->CTRL = 0x00000005;
	while (!(SysTick->CTRL & 0x00010000));
	SysTick->CTRL = 0x00000004;
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
	while (xms--) Delay_us(1000);
}

/**
  * @brief  秒级延时
  * @param  xs 延时时长
  * @retval 无
  */
void Delay_s(uint32_t s)
{
	while (s--) Delay_ms(1000);
}
