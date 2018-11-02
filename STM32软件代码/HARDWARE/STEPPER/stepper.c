#include "bsp.h"

/*
步进电机1.8度/脉冲
转过360度需要200脉冲，即200HZ
200HZ----360度----一圈
*/

void Stepper_Init(void)
{
	GPIO_InitTypeDef 			GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  			TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO | RCC_APB2Periph_TIM1, ENABLE);
	
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2 | GPIO_FullRemap_TIM1, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_0; //PB3:TIM2_CH2   STP  PB0:DIR
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11; //PE9:TIM1_CH1   STP  PE11:DIR
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = 2000-1; //50Hz 1S90度
	TIM_TimeBaseStructure.TIM_Prescaler = 72-1; //1us
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //时钟为72M
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	TIM_TimeBaseStructure.TIM_Period = 20000-1; //50Hz 1S90度
	TIM_TimeBaseStructure.TIM_Prescaler = 72-1; //1us
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //时钟为72M
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//TIM2_CH3 
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//TIM1_CH1 
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);

	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable); 
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); 
 
	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM1, ENABLE);
	
	DIR1 = 0;
	DIR2 = 0;
}

