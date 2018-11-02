#include "bsp.h"

void KEY_Init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
 	GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOF,GPIO_PinSource0);	
  	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	  

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOF,GPIO_PinSource1);
  	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOF,GPIO_PinSource2);
  	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOF,GPIO_PinSource3);
  	EXTI_InitStructure.EXTI_Line = EXTI_Line3;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOF,GPIO_PinSource4);
  	EXTI_InitStructure.EXTI_Line = EXTI_Line4;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;				
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;							
  	NVIC_Init(&NVIC_InitStructure); 
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;			
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;				
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;							
  	NVIC_Init(&NVIC_InitStructure); 
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;			
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;				
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;							
  	NVIC_Init(&NVIC_InitStructure); 
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;			
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;				
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;							
  	NVIC_Init(&NVIC_InitStructure); 
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;			
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;				
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;							
  	NVIC_Init(&NVIC_InitStructure); 
}


void EXTI0_IRQHandler(void)
{
	delay_ms(10);
	if(KEY_MODE == 0)	 	
	{		
		LED1 = !LED1;
		Key_Mode_Hander();
	}
	EXTI_ClearITPendingBit(EXTI_Line0);  
}

void EXTI1_IRQHandler(void)
{
	delay_ms(10);
	if(KEY_DOWN == 0)	 	
	{	
		LED1 = !LED1;
		Key_Down_Hander();
	}
	EXTI_ClearITPendingBit(EXTI_Line1);  
}

void EXTI2_IRQHandler(void)
{
	delay_ms(10);
	if(KEY_BACK == 0)	 	
	{	
		LED1 = !LED1;
		Key_Back_Hander();
	}
	EXTI_ClearITPendingBit(EXTI_Line2);  
}

void EXTI3_IRQHandler(void)
{
	delay_ms(10);
	if(KEY_OK == 0)	 	
	{
		LED1 = !LED1;
		Key_Ok_Hander();
	}
	EXTI_ClearITPendingBit(EXTI_Line3);  
}


void EXTI4_IRQHandler(void)
{
	delay_ms(10);
	if(KEY_FORWARD == 0)	 	
	{
		LED1 = !LED1;		
		Key_Forward_Hander();
	}
	EXTI_ClearITPendingBit(EXTI_Line4);  
}


void KEY_UP_SCAN(void)
{
	if(KEY_UP == 0)	
	{	
		delay_ms(100);
		if(KEY_UP == 0)	 		
		{
			LED1 = !LED1;
			Key_Up_Hander();
		}
	}
}
 
