#include "bsp.h"	

u8 QueZhuJiuEN = 1;
u8 HuiXuanJiuEN = 1;
u8 AutoModeEN = 1;

//ͨ�ö�ʱ��3�жϳ�ʼ��
void TIM3_Int_Init(void)
{
    TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
	NVIC_InitTypeDef 			NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
	
	//��ʱ��TIM3��ʼ��
	TIM_TimeBaseStructure.TIM_Period = 100 - 1; //10ms	
	TIM_TimeBaseStructure.TIM_Prescaler = 7200-1; //10kHz  100us
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); 

	//�ж����ȼ�NVIC����
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
		
		if(ControlMode == AutoMode)//ȸ�ľ�
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
		
		if(ControlMode == QueZhuJiu)//ȸ�ľ�
		{
			if(QueZhuJiuEN)
			{
				QueZhuJiuInit();
				QueZhuJiuEN = 0;
			}
			QueZhuJiuProcess(0.04,200);//0.04:ȸ�ľĵľ��� 20����ֵ����
			ikine_7bot(&RHS_7Bot, D4, DHAND, A1, A2);
			Set_Angle(&Joint);
		}
		
		if(ControlMode == HuiXuanJiu)//������
		{
			if(HuiXuanJiuEN)
			{
				HuiXuanJiuInit();
				HuiXuanJiuEN = 0;
			}
			
			HuiXuanJiuProcess(0.03,300,0);//0.02:�����ĵİ뾶 20����ֵ���� 1��ת������
			ikine_7bot(&RHS_7Bot, D4, DHAND, A1, A2);
			Set_Angle(&Joint);
		}
		
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );
	}
}


