#include "Sensor.h"

/**
  * PD0~PD5: 1~6号NPN光电传感器, 用于下层转盘盒子定位
  * 遮挡 → NPN导通拉低=0, 无遮挡 → 上拉=1
  */

void Sensor_Init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitTypeDef g;
	g.GPIO_Mode = GPIO_Mode_IN;
	g.GPIO_PuPd = GPIO_PuPd_UP;
	g.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_Init(GPIOD, &g);
}

/**
  * @brief  读取指定编号传感器状态
  * @param  index 1~6
  * @retval 0=有遮挡, 1=无遮挡(含index非法情况)
  */
uint8_t Sensor_Get(uint8_t index)
{
	uint16_t pin;
	switch (index)
	{
		case 1: pin = GPIO_Pin_0; break;
		case 2: pin = GPIO_Pin_1; break;
		case 3: pin = GPIO_Pin_2; break;
		case 4: pin = GPIO_Pin_3; break;
		case 5: pin = GPIO_Pin_4; break;
		case 6: pin = GPIO_Pin_5; break;
		default: return 1;
	}
	return GPIO_ReadInputDataBit(GPIOD, pin);
}
