#ifndef __KEY_H
#define __KEY_H

#include "sys.h"

//***************************°´¼ü²Ù×÷***********************************************
#define KEY_MODE     			GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_0)
#define KEY_DOWN     			GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_1)
#define KEY_BACK     			GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_2)
#define KEY_OK       			GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_3)
#define KEY_FORWARD  			GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_4)
#define KEY_UP       			GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_5)

void KEY_Init(void);
void KEY_UP_Init(void);
void KEY_UP_SCAN(void);

#endif

