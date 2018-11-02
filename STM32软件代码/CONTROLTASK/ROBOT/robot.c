#include "bsp.h"

RHS_t 			RHS_7Bot;
Joint_Param_t	Joint = {90,115,65,90,120,120};
float QueZhuArray1[3] = {0, 0, 0};
float QueZhuArray2[3] = {0, 0, 0};
float QueZhuArray0[3] = {0, 0, 0};
float HXJState[12];

//基于六自由度机械臂7BOT建立的D-H模型，正运动学方程，输入为机械度的六个角度（0-180），输出为位姿矩阵
void fkine_7bot(Joint_Param_t* Joint_degree, float d4, float d6, float a1, float a2)
{
	float theta1 = ang2rad(Joint_degree->rad1);
	float theta2 = ang2rad(Joint_degree->rad2);
	float theta3 = ang2rad(Joint_degree->rad3 - 90);
	float theta4 = ang2rad(Joint_degree->rad4 - 90);
	float theta5 = ang2rad(Joint_degree->rad5 - 90);
	float theta6 = ang2rad(Joint_degree->rad6);
	
	RHS_7Bot.nx = cos(theta4)*sin(theta1)*sin(theta6) + 												\
				  cos(theta5)*cos(theta6)*sin(theta1)*sin(theta4) - 									\
				  cos(theta1)*cos(theta2)*cos(theta3)*sin(theta4)*sin(theta6) - 						\
				  cos(theta1)*cos(theta2)*cos(theta6)*sin(theta3)*sin(theta5) - 						\
				  cos(theta1)*cos(theta3)*cos(theta6)*sin(theta2)*sin(theta5) + 						\
				  cos(theta1)*sin(theta2)*sin(theta3)*sin(theta4)*sin(theta6) + 						\
				  cos(theta1)*cos(theta2)*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta6) - 			\
				  cos(theta1)*cos(theta4)*cos(theta5)*cos(theta6)*sin(theta2)*sin(theta3);
	
	RHS_7Bot.ny = sin(theta1)*sin(theta2)*sin(theta3)*sin(theta4)*sin(theta6) - 
				  cos(theta1)*cos(theta5)*cos(theta6)*sin(theta4) - 									\
	   			  cos(theta2)*cos(theta3)*sin(theta1)*sin(theta4)*sin(theta6) - 						\
				  cos(theta2)*cos(theta6)*sin(theta1)*sin(theta3)*sin(theta5) - 						\
				  cos(theta3)*cos(theta6)*sin(theta1)*sin(theta2)*sin(theta5) - 						\
				  cos(theta1)*cos(theta4)*sin(theta6) + 												\
				  cos(theta2)*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta6)*sin(theta1) - 			\
				  cos(theta4)*cos(theta5)*cos(theta6)*sin(theta1)*sin(theta2)*sin(theta3);
	
	RHS_7Bot.nz = cos(theta2)*cos(theta3)*cos(theta6)*sin(theta5) - 									\
				  cos(theta2)*sin(theta3)*sin(theta4)*sin(theta6) - 									\
				  cos(theta3)*sin(theta2)*sin(theta4)*sin(theta6) - 									\
				  cos(theta6)*sin(theta2)*sin(theta3)*sin(theta5) + 									\
				  cos(theta2)*cos(theta4)*cos(theta5)*cos(theta6)*sin(theta3) + 						\
				  cos(theta3)*cos(theta4)*cos(theta5)*cos(theta6)*sin(theta2);
				
	RHS_7Bot.ox = cos(theta4)*cos(theta6)*sin(theta1) - 												\
				  cos(theta5)*sin(theta1)*sin(theta4)*sin(theta6) - 									\
			 	  cos(theta1)*cos(theta2)*cos(theta3)*cos(theta6)*sin(theta4) + 						\
				  cos(theta1)*cos(theta6)*sin(theta2)*sin(theta3)*sin(theta4) + 						\
				  cos(theta1)*cos(theta2)*sin(theta3)*sin(theta5)*sin(theta6) + 						\
				  cos(theta1)*cos(theta3)*sin(theta2)*sin(theta5)*sin(theta6) - 						\
				  cos(theta1)*cos(theta2)*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta6) + 			\
				  cos(theta1)*cos(theta4)*cos(theta5)*sin(theta2)*sin(theta3)*sin(theta6);	
	
	RHS_7Bot.oy = cos(theta1)*cos(theta5)*sin(theta4)*sin(theta6) - 									\
				  cos(theta1)*cos(theta4)*cos(theta6) - 												\
				  cos(theta2)*cos(theta3)*cos(theta6)*sin(theta1)*sin(theta4) + 						\
				  cos(theta6)*sin(theta1)*sin(theta2)*sin(theta3)*sin(theta4) + 						\
				  cos(theta2)*sin(theta1)*sin(theta3)*sin(theta5)*sin(theta6) + 						\
				  cos(theta3)*sin(theta1)*sin(theta2)*sin(theta5)*sin(theta6) - 						\
				  cos(theta2)*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta1)*sin(theta6) + 			\
				  cos(theta4)*cos(theta5)*sin(theta1)*sin(theta2)*sin(theta3)*sin(theta6);
	
	RHS_7Bot.oz = sin(theta2)*sin(theta3)*sin(theta5)*sin(theta6) - 									\
				  cos(theta3)*cos(theta6)*sin(theta2)*sin(theta4) - 									\
				  cos(theta2)*cos(theta3)*sin(theta5)*sin(theta6) - 									\
				  cos(theta2)*cos(theta6)*sin(theta3)*sin(theta4) - 									\
				  cos(theta2)*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta6) - 						\
				  cos(theta3)*cos(theta4)*cos(theta5)*sin(theta2)*sin(theta6);
	
	RHS_7Bot.ax = sin(theta1)*sin(theta4)*sin(theta5) + 												\
				  cos(theta1)*cos(theta2)*cos(theta5)*sin(theta3) + 									\
				  cos(theta1)*cos(theta3)*cos(theta5)*sin(theta2) + 									\
				  cos(theta1)*cos(theta2)*cos(theta3)*cos(theta4)*sin(theta5) - 						\
				  cos(theta1)*cos(theta4)*sin(theta2)*sin(theta3)*sin(theta5);		
	
	RHS_7Bot.ay = cos(theta2)*cos(theta5)*sin(theta1)*sin(theta3) - 									\
				  cos(theta1)*sin(theta4)*sin(theta5) + 												\
				  cos(theta3)*cos(theta5)*sin(theta1)*sin(theta2) + 									\
				  cos(theta2)*cos(theta3)*cos(theta4)*sin(theta1)*sin(theta5) - 						\
				  cos(theta4)*sin(theta1)*sin(theta2)*sin(theta3)*sin(theta5);			
	
	RHS_7Bot.az = cos(theta5)*sin(theta2)*sin(theta3) - 												\
				  cos(theta2)*cos(theta3)*cos(theta5) + 												\
				  cos(theta2)*cos(theta4)*sin(theta3)*sin(theta5) + 									\
				  cos(theta3)*cos(theta4)*sin(theta2)*sin(theta5);
				
	RHS_7Bot.px = a1*cos(theta1) + 																	\
				  a2*cos(theta1)*cos(theta2) + 														\
				  d4*cos(theta1)*cos(theta2)*sin(theta3) + 											\
				  d4*cos(theta1)*cos(theta3)*sin(theta2) + 											\
				  d6*sin(theta1)*sin(theta4)*sin(theta5) + 											\
				  d6*cos(theta1)*cos(theta2)*cos(theta5)*sin(theta3) + 								\
				  d6*cos(theta1)*cos(theta3)*cos(theta5)*sin(theta2) + 								\
				  d6*cos(theta1)*cos(theta2)*cos(theta3)*cos(theta4)*sin(theta5) - 					\
				  d6*cos(theta1)*cos(theta4)*sin(theta2)*sin(theta3)*sin(theta5);			
	
	RHS_7Bot.py = a1*sin(theta1) + 																	\
				  a2*cos(theta2)*sin(theta1) + 														\
				  d4*cos(theta2)*sin(theta1)*sin(theta3) + 											\
				  d4*cos(theta3)*sin(theta1)*sin(theta2) - 											\
				  d6*cos(theta1)*sin(theta4)*sin(theta5) + 											\
				  d6*cos(theta2)*cos(theta5)*sin(theta1)*sin(theta3) + 								\
				  d6*cos(theta3)*cos(theta5)*sin(theta1)*sin(theta2) + 								\
				  d6*cos(theta2)*cos(theta3)*cos(theta4)*sin(theta1)*sin(theta5) - 					\
				  d6*cos(theta4)*sin(theta1)*sin(theta2)*sin(theta3)*sin(theta5);
	
	RHS_7Bot.pz = a2*sin(theta2) - 																	\
				  d4*cos(theta2)*cos(theta3) + 														\
				  d4*sin(theta2)*sin(theta3) - 														\
				  d6*cos(theta2)*cos(theta3)*cos(theta5) + 											\
				  d6*cos(theta5)*sin(theta2)*sin(theta3) + 											\
				  d6*cos(theta2)*cos(theta4)*sin(theta3)*sin(theta5) + 								\
				  d6*cos(theta3)*cos(theta4)*sin(theta2)*sin(theta5);
}


//在X轴方向平移distance
void Transl_x(float distance)
{
	RHS_7Bot.px += distance;
}	


//在Y轴方向平移distance
void Transl_y(float distance)
{
	RHS_7Bot.py += distance;
}


//在Z轴方向平移distance
void Transl_z(float distance)
{
	RHS_7Bot.pz -= distance;
}


//绕X轴旋转theta
void Rot_x(float theta)
{
	theta = ang2rad(theta);
	
	RHS_7Bot.ny = RHS_7Bot.ny*cos(theta) - RHS_7Bot.nz*sin(theta);
	RHS_7Bot.nz = RHS_7Bot.nz*cos(theta) + RHS_7Bot.ny*sin(theta);
	RHS_7Bot.oy = RHS_7Bot.oy*cos(theta) - RHS_7Bot.oz*sin(theta);
	RHS_7Bot.oz = RHS_7Bot.oz*cos(theta) + RHS_7Bot.oy*sin(theta);
	RHS_7Bot.ay = RHS_7Bot.ay*cos(theta) - RHS_7Bot.az*sin(theta);
	RHS_7Bot.az = RHS_7Bot.az*cos(theta) + RHS_7Bot.ay*sin(theta);
	RHS_7Bot.py = RHS_7Bot.py*cos(theta) - RHS_7Bot.pz*sin(theta);
	RHS_7Bot.pz = RHS_7Bot.pz*cos(theta) + RHS_7Bot.py*sin(theta);
}


//绕Y轴旋转theta
void Rot_y(float theta)
{
	theta = ang2rad(theta);
	
	RHS_7Bot.nx = RHS_7Bot.nx*cos(theta) + RHS_7Bot.nz*sin(theta);
	RHS_7Bot.nz = RHS_7Bot.nz*cos(theta) - RHS_7Bot.nx*sin(theta);
	RHS_7Bot.ox = RHS_7Bot.ox*cos(theta) + RHS_7Bot.oz*sin(theta);
	RHS_7Bot.oz = RHS_7Bot.oz*cos(theta) - RHS_7Bot.ox*sin(theta);
	RHS_7Bot.ax = RHS_7Bot.ax*cos(theta) + RHS_7Bot.az*sin(theta);
	RHS_7Bot.az = RHS_7Bot.az*cos(theta) - RHS_7Bot.ax*sin(theta);
	RHS_7Bot.px = RHS_7Bot.px*cos(theta) + RHS_7Bot.pz*sin(theta);
	RHS_7Bot.pz = RHS_7Bot.pz*cos(theta) - RHS_7Bot.px*sin(theta);
}


//绕Z轴旋转theta
void Rot_z(float theta)
{
	theta = ang2rad(theta);
	
	RHS_7Bot.nx = RHS_7Bot.nx*cos(theta) - RHS_7Bot.ny*sin(theta);
	RHS_7Bot.ny = RHS_7Bot.ny*cos(theta) + RHS_7Bot.nx*sin(theta);
	RHS_7Bot.ox = RHS_7Bot.ox*cos(theta) - RHS_7Bot.oy*sin(theta);
	RHS_7Bot.oy = RHS_7Bot.oy*cos(theta) + RHS_7Bot.ox*sin(theta);
	RHS_7Bot.ax = RHS_7Bot.ax*cos(theta) - RHS_7Bot.ay*sin(theta);
	RHS_7Bot.ay = RHS_7Bot.ay*cos(theta) + RHS_7Bot.ax*sin(theta);
	RHS_7Bot.px = RHS_7Bot.px*cos(theta) - RHS_7Bot.py*sin(theta);
	RHS_7Bot.py = RHS_7Bot.py*cos(theta) + RHS_7Bot.px*sin(theta); 
}


//逆运动学方程求解
void ikine_7bot(RHS_t* RHS_Component, float d4, float d6, float a1, float a2)
{
	float nx = RHS_Component->nx;
	float ny = RHS_Component->ny;
	float nz = RHS_Component->nz;
	float ox = RHS_Component->ox;
	float oy = RHS_Component->oy;
	float oz = RHS_Component->oz;
	float ax = RHS_Component->ax;
	float ay = RHS_Component->ay;
	float az = RHS_Component->az;
	float px = RHS_Component->px;
	float py = RHS_Component->py;
	float pz = RHS_Component->pz;
	float theta1,theta2_1,theta2_2,theta3_1,theta3_2,theta23_1,theta23_2,theta4,degree[6];
	float s1,c1,s2_1,c2_1,s2_2,c2_2,s23,c23,s23_1,c23_1,s23_2,c23_2,s4,s5,c5,s6,c6;
	float eq1,eq2,eq3,a,b,c,delta;
	
	theta1 = atan((ay*d6 - py)/(ax*d6 - px));
	degree[0] = (fabs(rad2ang(theta1) - Joint.rad1) < fabs(rad2ang(theta1) + 180 - Joint.rad1) ? rad2ang(theta1) : (rad2ang(theta1) + 180));
	
	s1 = sin(theta1);
	c1 = cos(theta1);
	eq1 = px*c1 + py*s1 - a1 - d6*ax*c1 - d6*ay*s1;
	eq2 = pz - d6*az;
	eq3 = (pow(a2,2) - pow(eq1,2) - pow(d4,2) - pow(eq2,2))/(2*d4);
	a = pow(eq1,2) + pow(eq2,2);
	b = 2*eq1*eq3;
	c = pow(eq3,2) - pow(eq2,2);
	delta = pow(b,2) - 4*a*c;
	
	if (delta >= 0)
	{
		s23_1 = (-b + sqrt(delta))/(2*a);
		c23_1 = (eq3 + eq1*s23_1)/eq2;
		theta23_1 = atan2(s23_1,c23_1);
		s2_1 = (eq2 + d4*c23_1)/a2;
		c2_1 = (eq1 - d4*s23_1)/a2;
		theta2_1 = atan2(s2_1,c2_1);
		theta3_1 = theta23_1 - theta2_1;
		
		s23_2 = (-b - sqrt(delta))/(2*a);
		c23_2 = (eq3 + eq1*s23_2)/eq2;
		theta23_2 = atan2(s23_2,c23_2);
		s2_2 = (eq2 + d4*c23_2)/a2;
		c2_2 = (eq1 - d4*s23_2)/a2;
		theta2_2 = atan2(s2_2,c2_2);
		theta3_2 = theta23_2 - theta2_2;
		
		if ((fabs(rad2ang(theta2_1) - Joint.rad2) + fabs(rad2ang(theta3_1) - Joint.rad3 + 90)) <= (fabs(rad2ang(theta2_2) - Joint.rad2) + fabs(rad2ang(theta3_2) - Joint.rad3 + 90)))
		{
			degree[1] = rad2ang(theta2_1);
			degree[2] = rad2ang(theta3_1) + 90;
			s23 = s23_1;
			c23 = c23_1;
		}
		else 
		{
			degree[1] = rad2ang(theta2_2);
			degree[2] = rad2ang(theta3_2) + 90;	
			s23 = s23_2;
			c23 = c23_2;
		}	
		
		theta4 = atan((ax*s1 - ay*c1)/(az*s23 + ax*c1*c23 + ay*s1*c23));
		degree[3] = ((fabs(rad2ang(theta4) - Joint.rad4 + 90) < fabs(rad2ang(theta4) + 180 - Joint.rad4 + 90)) ? rad2ang(theta4) : (rad2ang(theta4) + 180)) + 90;
		
		s4 = sin(theta4);
		c5 = -az*c23 + ax*c1*s23 + ay*s1*s23;
		s5 = (ax*s1 - ay*c1)/s4;
		degree[4] = rad2ang(atan2(s5,c5)) + 90;
		
		c6 = (-nz*c23 + nx*c1*s23 + ny*s1*s23)/(-s5);
		s6 = (-oz*c23 + ox*c1*s23 + oy*s1*s23)/s5;
		degree[5] = rad2ang(atan2(s6,c6));
	}
	else
	{
		ShowError();
		LED2 = 0;
		printf("\r\n*************ERROR!!!**************\r\n");
	}
	
	if (IsNumber(degree[0]) && IsNumber(degree[1]) && IsNumber(degree[2]) && IsNumber(degree[3]) && IsNumber(degree[4]) && IsNumber(degree[5]))
	{
		Joint.rad1 = degree[0];
		Joint.rad2 = degree[1];
		Joint.rad3 = degree[2];
		Joint.rad4 = degree[3];
		Joint.rad5 = degree[4];
		Joint.rad6 = degree[5];
	}
}

//将角度赋值给舵机
void Set_Angle(Joint_Param_t* Joint_degree)
{
	TIM_SetCompare4(TIM2,mapping(0,180,500,2500,Joint_degree->rad1));	
	TIM_SetCompare3(TIM2,mapping(0,180,500,2500,Joint_degree->rad2));	
	TIM_SetCompare4(TIM4,mapping(0,180,500,2500,Joint_degree->rad3));	
	TIM_SetCompare3(TIM4,mapping(0,180,500,2500,Joint_degree->rad4));
	TIM_SetCompare2(TIM4,mapping(0,180,500,2500,Joint_degree->rad5));	
	TIM_SetCompare1(TIM4,mapping(0,180,500,2500,Joint_degree->rad6));		
}


//将机械臂角度重映射为PWM占空比
u16 mapping(u16 old_min,u16 old_max,u16 new_min,u16 new_max,float value)
{
	return (u16)(value * (new_max - new_min) / (old_max - old_min) + new_min);
}


//机械臂初始化
void Robot_Init(void)
{
	Set_Angle(&Joint);
}

float ang2rad(float angle)
{
	return angle*PI/180.0f;
}

float rad2ang(float radian)
{
	return radian*180.0f/PI;
}


//雀啄灸初始化
void QueZhuJiuInit(void)
{	
	fkine_7bot(&Joint, D4, D6, A1, A2);
	QueZhuArray0[0] = RHS_7Bot.px;
	QueZhuArray0[1] = RHS_7Bot.py;
	QueZhuArray0[2] = RHS_7Bot.pz;
	
	fkine_7bot(&Joint, D4, DHAND, A1, A2);
	QueZhuArray1[0] = RHS_7Bot.px;
	QueZhuArray1[1] = RHS_7Bot.py;
	QueZhuArray1[2] = RHS_7Bot.pz;
}


//雀啄灸轨迹生成
void QueZhuJiuProcess(float quezhudist_xm, u16 StepNum)
{
	static u16 i=0,IncDir=1;
	float InterpInc = 0;
	
	InterpInc = quezhudist_xm/StepNum;
	
	QueZhuArray2[0] = i*InterpInc/HANDLENGTH*(QueZhuArray1[0] - QueZhuArray0[0]) + QueZhuArray1[0];
	QueZhuArray2[1] = i*InterpInc/HANDLENGTH*(QueZhuArray1[1] - QueZhuArray0[1]) + QueZhuArray1[1];
	QueZhuArray2[2] = i*InterpInc/HANDLENGTH*(QueZhuArray1[2] - QueZhuArray0[2]) + QueZhuArray1[2];	
	
	if (IncDir == 1)
	{	
		i++;
		if(i == StepNum)
		{
			IncDir = 0;
		}
	}
	
	if (IncDir == 0)
	{	
		i--;
		if(i == 0)
		{
			IncDir = 1;
		}
	}
	
	RHS_7Bot.px = QueZhuArray2[0];
	RHS_7Bot.py = QueZhuArray2[1];
	RHS_7Bot.pz = QueZhuArray2[2];
}


void HuiXuanJiuInit(void)
{
	fkine_7bot(&Joint, D4, DHAND, A1, A2);
	HXJState[0] = RHS_7Bot.nx;
	HXJState[1] = RHS_7Bot.ny;
	HXJState[2] = RHS_7Bot.nz;
	HXJState[3] = RHS_7Bot.ox;
	HXJState[4] = RHS_7Bot.oy;
	HXJState[5] = RHS_7Bot.oz;
	HXJState[6] = RHS_7Bot.ax;
	HXJState[7] = RHS_7Bot.ay;
	HXJState[8] = RHS_7Bot.az;
	HXJState[9] = RHS_7Bot.px;
	HXJState[10] = RHS_7Bot.py;
	HXJState[11] = RHS_7Bot.pz;
}

void HuiXuanJiuProcess(float radius_xm, u16 StepNum, u8 RotDir)
{
	static u16 k = 0;
	float InterpInc = 0;
	
	InterpInc = 2*PI/StepNum;

	if (RotDir == 1)
	{
		RHS_7Bot.px = HXJState[0]*radius_xm*cos(k*InterpInc) + HXJState[3]*radius_xm*sin(k*InterpInc) + HXJState[9];
		RHS_7Bot.py = HXJState[1]*radius_xm*cos(k*InterpInc) + HXJState[4]*radius_xm*sin(k*InterpInc) + HXJState[10];
		RHS_7Bot.pz = HXJState[2]*radius_xm*cos(k*InterpInc) + HXJState[5]*radius_xm*sin(k*InterpInc) + HXJState[11];
	}
	
	if (RotDir == 0)
	{
		RHS_7Bot.px = HXJState[0]*radius_xm*cos((StepNum - k)*InterpInc) + HXJState[3]*radius_xm*sin((StepNum - k)*InterpInc) + HXJState[9];
		RHS_7Bot.py = HXJState[1]*radius_xm*cos((StepNum - k)*InterpInc) + HXJState[4]*radius_xm*sin((StepNum - k)*InterpInc) + HXJState[10];
		RHS_7Bot.pz = HXJState[2]*radius_xm*cos((StepNum - k)*InterpInc) + HXJState[5]*radius_xm*sin((StepNum - k)*InterpInc) + HXJState[11];
	}
	
	k++;
	if (k == StepNum)
	{
		k = 0;
	}
}
	
int IsNumber(float d)
{
	return (d == d);
}


void Auto_Run(float set_distance)
{
	QueZhuArray2[0] = (set_distance - dist_ideal)*0.01/HANDLENGTH*(QueZhuArray1[0] - QueZhuArray0[0]) + QueZhuArray1[0];
	QueZhuArray2[1] = (set_distance - dist_ideal)*0.01/HANDLENGTH*(QueZhuArray1[1] - QueZhuArray0[1]) + QueZhuArray1[1];
	QueZhuArray2[2] = (set_distance - dist_ideal)*0.01/HANDLENGTH*(QueZhuArray1[2] - QueZhuArray0[2]) + QueZhuArray1[2];	

	RHS_7Bot.px = QueZhuArray2[0];
	RHS_7Bot.py = QueZhuArray2[1];
	RHS_7Bot.pz = QueZhuArray2[2];
}


