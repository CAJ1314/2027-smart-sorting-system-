#include "stm32f10x.h"                  // Device header

void PWM_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	TIM_InternalClockConfig(TIM2);//选择时钟
	
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;//计数器模式(向上计数)
	TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;//ARR自动重装
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;//PSC预分频
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;//高级计数器暂时用不到
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode =TIM_OCMode_PWM1;//输出比较模式
	TIM_OCInitStructure.TIM_OCPolarity =TIM_OCPolarity_High;//输出比较极性
	TIM_OCInitStructure.TIM_OutputState =TIM_OutputState_Enable;//输出比较使能
	TIM_OCInitStructure.TIM_Pulse =0;//设置CCR
	TIM_OC2Init(TIM2,&TIM_OCInitStructure);
	
	TIM_Cmd(TIM2,ENABLE);//定时器使能
}

void PWM_SetCompare2(uint16_t Compare)//单独更改CCR值
{
	TIM_SetCompare2(TIM2,Compare);
}
