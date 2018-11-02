#include "bsp.h"
/*
x_last    前一时刻最优值
x_mid    当前时刻预测值
x_now    当前时刻最优值
p_last    前一时刻最优值的偏差
p_mid     当前时刻预测值的偏差
p_now    当前时刻最优值的偏差
Q     预测噪声
R    测量噪声
kg    增益系数
Measure_Value  测量值
*/
float KalmanFilter(float Measure_Value)
{
	static float x_last=1;
	static float p_last=1;
	static float Q=0.1;
	static float R=0.1;
	float kg,x_mid,x_now,p_mid,p_now;
	x_mid=x_last;    //x_last=x(k-1|k-1),x_mid=x(k|k-1) ***********公式1   预测结果
	p_mid=sqrt(p_last*p_last+Q*Q);  //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=预测噪声 *********公式2  预测系数
	kg=sqrt((p_mid*p_mid)/(p_mid*p_mid+R*R)); //kg为 kalman filter，R为测量噪声 ************公式4  增益系数
	x_now=x_mid+kg*(Measure_Value-x_mid);//估计出的最优值 ******************公式3  预测与测量组合结果
	p_now=(1-kg)*p_mid;//最优值对应的 covariance ********************公式5  更新预测系数
	p_last = p_now;  //更新covariance值 
	x_last = x_now;  //更新系统状态值 
	return x_now;
}

/*
A、名称：一阶滞后滤波法(**一阶低通滤波器)
B、方法：
    取a=0-1，本次滤波结果=(1-a)*本次采样值+a*上次滤波结果。
C、优点：
    对周期性干扰具有良好的抑制作用；
    适用于波动频率较高的场合。
D、缺点：
    相位滞后，灵敏度低；
    滞后程度取决于a值大小；
    不能消除滤波频率高于采样频率1/2的干扰信号。

 * @param oldData:上次滤波输出
 * @param newData:新采样值
 * @param CONST:滤波系数
 * @return 滤波输出
 */
float LowPassFilter(float oldData, float newData, float CONST)
{
    return (1-CONST)*oldData + CONST*newData;
}



/*
A、名称：递推平均滤波法（又称滑动平均滤波法）
B、方法：
    把连续取得的N个采样值看成一个队列，队列的长度固定为N，
    每次采样到一个新数据放入队尾，并扔掉原来队首的一次数据（先进先出原则），
    把队列中的N个数据进行算术平均运算，获得新的滤波结果。
    N值的选取：流量，N=12；压力，N=4；液面，N=4-12；温度，N=1-4。
C、优点：
    对周期性干扰有良好的抑制作用，平滑度高；
    适用于高频振荡的系统。
D、缺点：
    灵敏度低，对偶然出现的脉冲性干扰的抑制作用较差；
    不易消除由于脉冲干扰所引起的采样值偏差；
    不适用于脉冲干扰比较严重的场合；
    比较浪费RAM。
*/

float data[DEEPTH]; 
float SlideFilter(float in)
{
    uint8_t i = 0;
    double sum = 0;

    data[DEEPTH-1] = in;  // 新数据放到最后
    for (i=1; i<DEEPTH; i++) 
	{
        data[i-1] = data[i];  // 滑动窗口
	}
    for (i=0; i<DEEPTH; i++)
	{
        sum += data[i];
	}

    return (sum / (float)DEEPTH);
}



/*
A、名称：中位值滤波法
B、方法：
    连续采样N次（N取奇数），把N次采样值按大小排列，
    取中间值为本次有效值。
C、优点：
    能有效克服因偶然因素引起的波动干扰；
    对温度、液位的变化缓慢的被测参数有良好的滤波效果。
D、缺点：
    对流量、速度等快速变化的参数不宜。
*/
int MedianFilter(int filter_buf[MEDIAN_FILTER_N]) 
{
  int i, j;
  int filter_temp;

  // 采样值从小到大排列（冒泡法）
  for(j = 0; j < MEDIAN_FILTER_N - 1; j++) 
	{
		for(i = 0; i < MEDIAN_FILTER_N - 1 - j; i++) 
		{
			if(filter_buf[i] > filter_buf[i + 1]) 
			{
				filter_temp = filter_buf[i];
				filter_buf[i] = filter_buf[i + 1];
				filter_buf[i + 1] = filter_temp;
			}
		}
	}
	
  return filter_buf[(MEDIAN_FILTER_N - 1) / 2];
}


/*
A、名称：算术平均滤波法
B、方法：
    连续取N个采样值进行算术平均运算：
    N值较大时：信号平滑度较高，但灵敏度较低；
    N值较小时：信号平滑度较低，但灵敏度较高；
    N值的选取：一般流量，N=12；压力：N=4。
C、优点：
    适用于对一般具有随机干扰的信号进行滤波；
    这种信号的特点是有一个平均值，信号在某一数值范围附近上下波动。
D、缺点：
    对于测量速度较慢或要求数据计算速度较快的实时控制不适用；
    比较浪费RAM。
*/

int AverageFilter(int filter_buf[AVERAGE_FILTER_N]) 
{
	int i;
	int filter_sum = 0;
	
	for(i = 0; i < AVERAGE_FILTER_N; i++) 
	{
		filter_sum += filter_buf[i];
	}
	
  return (int)(filter_sum / AVERAGE_FILTER_N);
}



/*
A、名称：中位值平均滤波法（又称防脉冲干扰平均滤波法）
B、方法：
    采一组队列去掉最大值和最小值后取平均值，
    相当于“中位值滤波法”+“算术平均滤波法”。
    连续采样N个数据，去掉一个最大值和一个最小值，
    然后计算N-2个数据的算术平均值。
    N值的选取：3-14。
C、优点：
    融合了“中位值滤波法”+“算术平均滤波法”两种滤波法的优点。
    对于偶然出现的脉冲性干扰，可消除由其所引起的采样值偏差。
    对周期干扰有良好的抑制作用。
    平滑度高，适于高频振荡的系统。
D、缺点：
    计算速度较慢，和算术平均滤波法一样。
    比较浪费RAM。
*/

int MedianAverageFilter(int filter_buf[MEDIAN_AVERAGE_FILTER_N]) 
{
	int i, j;
	int filter_temp, filter_sum = 0;
	
  // 采样值从小到大排列（冒泡法）
	for(j = 0; j < MEDIAN_AVERAGE_FILTER_N - 1; j++) 
	{
		for(i = 0; i < MEDIAN_AVERAGE_FILTER_N - 1 - j; i++) 
		{
			if(filter_buf[i] > filter_buf[i + 1]) 
			{
				filter_temp = filter_buf[i];
				filter_buf[i] = filter_buf[i + 1];
				filter_buf[i + 1] = filter_temp;
			}
		}
	}
  // 去除最大最小极值后求平均
	for(i = 1; i < MEDIAN_AVERAGE_FILTER_N - 1; i++) 
	{
		filter_sum += filter_buf[i];
	}
	
  return filter_sum / (MEDIAN_AVERAGE_FILTER_N - 2);
}


/*
A、名称：加权递推平均滤波法
B、方法：
    是对递推平均滤波法的改进，即不同时刻的数据加以不同的权；
    通常是，越接近现时刻的数据，权取得越大。
    给予新采样值的权系数越大，则灵敏度越高，但信号平滑度越低。
C、优点：
    适用于有较大纯滞后时间常数的对象，和采样周期较短的系统。
D、缺点：
    对于纯滞后时间常数较小、采样周期较长、变化缓慢的信号；
    不能迅速反应系统当前所受干扰的严重程度，滤波效果差。
*/
int coe[WEIGHTING_AVERAGE_FILTER_N] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};    // 加权系数表
int sum_coe = 1 + 2 + 3 + 4 + 5 + 6 + 7 + 8 + 9 + 10 + 11 + 12; // 加权系数和
int Filter(int filter_buf[WEIGHTING_AVERAGE_FILTER_N + 1]) 
{
	int i;
	int filter_sum = 0;
	
	for(i = 0; i < WEIGHTING_AVERAGE_FILTER_N; i++) 
	{
		filter_buf[i] = filter_buf[i + 1]; // 所有数据左移，低位仍掉
		filter_sum += filter_buf[i] * coe[i];
	}
	filter_sum /= sum_coe;
	
	return filter_sum;
}


