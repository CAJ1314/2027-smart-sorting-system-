#include "stm32f10x.h"                  // Device header
#include "Delay.h"
void Key_Init(void)//初始化按键
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
uint8_t Key_GetNum(void)//按键读取函数
{
	uint8_t KeyNum = 0;
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0)//GPIO_ReadInputDataBit是读取引脚输入电平
	{
		Delay_ms(20);//消除抖动
		while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0)//等待松手
		Delay_ms(20);
		KeyNum = 1;//取值1
	}
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 0)
	{
		Delay_ms(20);
		while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 0)
		Delay_ms(20);
		KeyNum = 2;
	}
	return KeyNum;
}
