#ifndef __KALMAN_H
#define __KALMAN_H

#include "sys.h"

#define DEEPTH  					5  			// �˲����--���������˲���
#define MEDIAN_FILTER_N 			101			//��λֵ�˲�����
#define AVERAGE_FILTER_N 			12			//ƽ��ֵ�˲���������
#define MEDIAN_AVERAGE_FILTER_N 	100			//��λֵƽ���˲������ֳƷ��������ƽ���˲�������������
#define WEIGHTING_AVERAGE_FILTER_N 	12			//��Ȩƽ��ֵ�˲�������

float KalmanFilter(float Measure_Value);
float LowPassFilter(float oldData, float newData, float CONST);
float SlideFilter(float in);
int MedianFilter(int filter_buf[MEDIAN_FILTER_N]);
int AverageFilter(int filter_buf[AVERAGE_FILTER_N]);
int MedianAverageFilter(int filter_buf[MEDIAN_AVERAGE_FILTER_N]);
int Filter(int filter_buf[WEIGHTING_AVERAGE_FILTER_N + 1]);

#endif


