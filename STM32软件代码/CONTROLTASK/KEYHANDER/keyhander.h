#ifndef __KEYHANDER_H
#define __KEYHANDER_H

#include "sys.h"

typedef enum
{
	TransxState,
	TransyState,
	TranszState,
	RotatexState,
	RotateyState,
	RotatezState,
}ParamState_e;


typedef enum
{
	ManualMode,
	AutoMode,
	WriteMode,
	TeachingMode,
	QueZhuJiu,
	HuiXuanJiu
}ControlMode_e;



void Key_Mode_Hander(void);
void Key_Ok_Hander(void);
void Key_Up_Hander(void);
void Key_Down_Hander(void);
void Key_Back_Hander(void);
void Key_Forward_Hander(void);
void GetControlMode(int mode);
void GetParamState(int paramconstate);
void GetTraj(ParamState_e ParamState, int dir);

#endif
