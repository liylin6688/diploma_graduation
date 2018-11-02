#ifndef __ROBOT_H__
#define __ROBOT_H__

#include "bsp.h"	

//***************************杆件参数**********************************************
#define D4										0.2206f//0.25f				
#define D6										0.012f//0.03f			//手部长度（不含艾条）			
#define A1 										0.035f//0.1f				
#define A2										0.15f//0.25f	

#define DHAND									0.155f			//手部长度（含艾条）
#define PI										3.1415926f
#define TRANSSTEP								0.01f
#define ROTATESTEP								3.0f
#define HANDLENGTH								0.125f			//艾条长度

//***************************六自由度机械臂*****************************************
#define ROBOT_6DOF								1	

typedef struct
{
	float rad1;
	float rad2;
	float rad3;
	float rad4;
	float rad5;
	float rad6;
} Joint_Param_t;					//关节变量


typedef struct
{
	float nx;					
	float ny;
	float nz;
	float ox;
	float oy;
	float oz;
	float ax;
	float ay;
	float az;
	float px;
	float py;
	float pz;
} RHS_t;							//位姿矩阵变量


void fkine_7bot(Joint_Param_t* Joint_degree, float d4, float d6, float a1, float a2);
void ikine_7bot(RHS_t* RHS_Component, float d4, float d6, float a1, float a2);
void Transl_x(float distance);
void Transl_y(float distance);
void Transl_z(float distance);
void Rot_x(float theta);
void Rot_y(float theta);
void Rot_z(float theta);
void Set_Angle(Joint_Param_t* Joint_degree);
u16 mapping(u16 old_min,u16 old_max,u16 new_min,u16 new_max,float value);
void Robot_Init(void);
float ang2rad(float angle);
float rad2ang(float radian);
void QueZhuJiuInit(void);
void QueZhuJiuProcess(float quezhudist_xm, u16 StepNum);
void HuiXuanJiuInit(void);
void HuiXuanJiuProcess(float radius_xm, u16 StepNum, u8 RotDir);
int IsNumber(float d);
void Auto_Run(float set_distance);

#endif

