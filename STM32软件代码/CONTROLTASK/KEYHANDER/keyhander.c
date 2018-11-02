#include "bsp.h"

ControlMode_e ControlMode = ManualMode;
ParamState_e ParamState = TransxState;
static int paramconstate = 0;
static int mode = 0;
int ACK = 0;

void Key_Mode_Hander()
{
	ACK = 0;
	mode++;
	if(mode == 6)
	{
		mode = 0;
	}
}


void Key_Ok_Hander()
{
	ACK = 1;
	GetControlMode(mode);
	GetParamState(paramconstate);
	PrintfRobot();
	printf("\r\n当前模式：%d\r\n",ControlMode);
	printf("当前移动的变量：%d\r\n\r\n",ParamState);
}


void Key_Up_Hander()
{
	ACK = 0;
	if(ControlMode == ManualMode)
	{
		paramconstate++;
		if(paramconstate == 6)
		{
			paramconstate = 0;
		}
	}
}


void Key_Down_Hander()
{
	ACK = 0;
	if(ControlMode == ManualMode)
	{
		if(paramconstate == 0)
		{
			paramconstate = 6;
		}	
		paramconstate--;
	}
}


void Key_Back_Hander()
{
	if(ControlMode == ManualMode)
	{
		GetTraj(ParamState, -1);
		PrintfRobot();
	}
}


void Key_Forward_Hander()
{
	if(ControlMode == ManualMode)
	{
		GetTraj(ParamState, 1);
		PrintfRobot();
	}
}

void GetControlMode(int mode)
{
	if((mode == 0)&&(ACK == 1))
	{
		ControlMode = ManualMode;
	}
	
	if((mode == 1)&&(ACK == 1))
	{
		ControlMode = AutoMode;
		AutoModeEN = 1;
	}
	
	if((mode == 2)&&(ACK == 1))
	{
		ControlMode = WriteMode;
		Robot_Init();
	}
	
	if((mode == 3)&&(ACK == 1))
	{
		ControlMode = TeachingMode;
	}
	
	if((mode == 4)&&(ACK == 1))
	{
		ControlMode = QueZhuJiu;
		QueZhuJiuEN = 1;
	}
	
	if((mode == 5)&&(ACK == 1))
	{
		ControlMode = HuiXuanJiu;
		HuiXuanJiuEN = 1;
	}
}

void GetParamState(int paramconstate)
{
	if((paramconstate == 0)&&(ACK == 1))
	{
		ParamState = TransxState;
	}
	
	if((paramconstate == 1)&&(ACK == 1))
	{
		ParamState = TransyState;
	}
	
	if((paramconstate == 2)&&(ACK == 1))
	{
		ParamState = TranszState;
	}
	
	if((paramconstate == 3)&&(ACK == 1))
	{
		ParamState = RotatexState;
	}
	
	if((paramconstate == 4)&&(ACK == 1))
	{
		ParamState = RotateyState;
	}
	
	if((paramconstate == 5)&&(ACK == 1))
	{
		ParamState = RotatezState;
	}
}

void GetTraj(ParamState_e ParamState, int dir)
{
	if(ParamState == TransxState)
	{
		Transl_x(dir*TRANSSTEP);
	}
	if(ParamState == TransyState)
	{
		Transl_y(dir*TRANSSTEP);
	}
	if(ParamState == TranszState)
	{
		Transl_z(dir*TRANSSTEP);
	}
	if(ParamState == RotatexState)
	{
		Rot_x(dir*ROTATESTEP);
	}
	if(ParamState == RotateyState)
	{
		Rot_y(dir*ROTATESTEP);
	}
	if(ParamState == RotatezState)
	{
		Rot_z(dir*ROTATESTEP);
	}
}


