#ifndef __KALMAN_H
#define __KALMAN_H

#include "sys.h"

#define DEEPTH  					5  			// 滤波深度--滑动窗口滤波器
#define MEDIAN_FILTER_N 			101			//中位值滤波个数
#define AVERAGE_FILTER_N 			12			//平均值滤波采样个数
#define MEDIAN_AVERAGE_FILTER_N 	100			//中位值平均滤波法（又称防脉冲干扰平均滤波法）采样个数
#define WEIGHTING_AVERAGE_FILTER_N 	12			//加权平均值滤波器个数

float KalmanFilter(float Measure_Value);
float LowPassFilter(float oldData, float newData, float CONST);
float SlideFilter(float in);
int MedianFilter(int filter_buf[MEDIAN_FILTER_N]);
int AverageFilter(int filter_buf[AVERAGE_FILTER_N]);
int MedianAverageFilter(int filter_buf[MEDIAN_AVERAGE_FILTER_N]);
int Filter(int filter_buf[WEIGHTING_AVERAGE_FILTER_N + 1]);

#endif


