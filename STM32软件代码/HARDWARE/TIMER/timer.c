#include "bsp.h"	

u8 QueZhuJiuEN = 1;
u8 HuiXuanJiuEN = 1;
u8 AutoModeEN = 1;

//通用定时器3中断初始化
void TIM3_Int_Init(void)
{
    TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
	NVIC_InitTypeDef 			NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
	
	//定时器TIM3初始化
	TIM_TimeBaseStructure.TIM_Period = 100 - 1; //10ms	
	TIM_TimeBaseStructure.TIM_Prescaler = 7200-1; //10kHz  100us
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); 

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);  

	TIM_Cmd(TIM3, ENABLE);  				 
}

void TIM3_IRQHandler(void)  
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  
	{  
		KEY_UP_SCAN();
		Display();

		
		if(ControlMode == ManualMode)
		{
			ikine_7bot(&RHS_7Bot, D4, DHAND, A1, A2);
			Set_Angle(&Joint);
		}
		
		if(ControlMode == AutoMode)//雀啄灸
		{
			if(AutoModeEN)
			{
				QueZhuJiuInit();
				AutoModeEN = 0;
			}
			Auto_Run(5.0);
			ikine_7bot(&RHS_7Bot, D4, DHAND, A1, A2);
			Set_Angle(&Joint);
		}
		
		if(ControlMode == QueZhuJiu)//雀啄灸
		{
			if(QueZhuJiuEN)
			{
				QueZhuJiuInit();
				QueZhuJiuEN = 0;
			}
			QueZhuJiuProcess(0.04,200);//0.04:雀啄灸的距离 20：插值个数
			ikine_7bot(&RHS_7Bot, D4, DHAND, A1, A2);
			Set_Angle(&Joint);
		}
		
		if(ControlMode == HuiXuanJiu)//回旋灸
		{
			if(HuiXuanJiuEN)
			{
				HuiXuanJiuInit();
				HuiXuanJiuEN = 0;
			}
			
			HuiXuanJiuProcess(0.03,300,0);//0.02:回旋灸的半径 20：插值个数 1：转动方向
			ikine_7bot(&RHS_7Bot, D4, DHAND, A1, A2);
			Set_Angle(&Joint);
		}
		
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );
	}
}


