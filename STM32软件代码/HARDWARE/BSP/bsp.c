#include "bsp.h"

void bsp_Init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	uart_init(115200);
	delay_init();
	KEY_Init();
	LED_Init(); 
	BEEP_Init();
	Servo_Init();
	Adc_Init();
	Smbus_Init();
	UltrasonicRanging_Init();
	TRIG_Init();
	OLED_Init();
	Robot_Init();
	fkine_7bot(&Joint, D4, DHAND, A1, A2);
	//Stepper_Init();
	TIM3_Int_Init();
}
