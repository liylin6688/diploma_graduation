#ifndef	__BSP_H__
#define	__BSP_H__

#include <stm32f10x.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "sys.h"
#include "usart.h"
#include "delay.h"

#include "key.h"
#include "led.h"
#include "beep.h"
#include "pwm.h"
#include "adc.h"
#include "smbus.h"
#include "ultrasonic.h"
#include "oled.h"
#include "timer.h"
#include "stepper.h"

#include "robot.h"
#include "keyhander.h"
#include "sensor.h"
#include "KalmanFilter.h"
#include "display.h"

extern RHS_t 			RHS_7Bot;
extern Joint_Param_t	Joint;
extern ControlMode_e 	ControlMode;
extern ParamState_e 	ParamState;
extern u8  				TIM5CH1_CAPTURE_STA;		    				
extern u32				TIM5CH1_CAPTURE_VAL;
extern float 			QueZhuArray1[3];
extern float 			QueZhuArray2[3];
extern u8 				QueZhuJiuEN;
extern u8 				HuiXuanJiuEN;
extern u8 				AutoModeEN ;
extern float 			dist_ideal;

void bsp_Init(void);

#endif
