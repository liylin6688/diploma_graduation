#include "bsp.h"
/*
x_last    ǰһʱ������ֵ
x_mid    ��ǰʱ��Ԥ��ֵ
x_now    ��ǰʱ������ֵ
p_last    ǰһʱ������ֵ��ƫ��
p_mid     ��ǰʱ��Ԥ��ֵ��ƫ��
p_now    ��ǰʱ������ֵ��ƫ��
Q     Ԥ������
R    ��������
kg    ����ϵ��
Measure_Value  ����ֵ
*/
float KalmanFilter(float Measure_Value)
{
	static float x_last=1;
	static float p_last=1;
	static float Q=0.1;
	static float R=0.1;
	float kg,x_mid,x_now,p_mid,p_now;
	x_mid=x_last;    //x_last=x(k-1|k-1),x_mid=x(k|k-1) ***********��ʽ1   Ԥ����
	p_mid=sqrt(p_last*p_last+Q*Q);  //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=Ԥ������ *********��ʽ2  Ԥ��ϵ��
	kg=sqrt((p_mid*p_mid)/(p_mid*p_mid+R*R)); //kgΪ kalman filter��RΪ�������� ************��ʽ4  ����ϵ��
	x_now=x_mid+kg*(Measure_Value-x_mid);//���Ƴ�������ֵ ******************��ʽ3  Ԥ���������Ͻ��
	p_now=(1-kg)*p_mid;//����ֵ��Ӧ�� covariance ********************��ʽ5  ����Ԥ��ϵ��
	p_last = p_now;  //����covarianceֵ 
	x_last = x_now;  //����ϵͳ״ֵ̬ 
	return x_now;
}

/*
A�����ƣ�һ���ͺ��˲���(**һ�׵�ͨ�˲���)
B��������
    ȡa=0-1�������˲����=(1-a)*���β���ֵ+a*�ϴ��˲������
C���ŵ㣺
    �������Ը��ž������õ��������ã�
    �����ڲ���Ƶ�ʽϸߵĳ��ϡ�
D��ȱ�㣺
    ��λ�ͺ������ȵͣ�
    �ͺ�̶�ȡ����aֵ��С��
    ���������˲�Ƶ�ʸ��ڲ���Ƶ��1/2�ĸ����źš�

 * @param oldData:�ϴ��˲����
 * @param newData:�²���ֵ
 * @param CONST:�˲�ϵ��
 * @return �˲����
 */
float LowPassFilter(float oldData, float newData, float CONST)
{
    return (1-CONST)*oldData + CONST*newData;
}



/*
A�����ƣ�����ƽ���˲������ֳƻ���ƽ���˲�����
B��������
    ������ȡ�õ�N������ֵ����һ�����У����еĳ��ȹ̶�ΪN��
    ÿ�β�����һ�������ݷ����β�����ӵ�ԭ�����׵�һ�����ݣ��Ƚ��ȳ�ԭ�򣩣�
    �Ѷ����е�N�����ݽ�������ƽ�����㣬����µ��˲������
    Nֵ��ѡȡ��������N=12��ѹ����N=4��Һ�棬N=4-12���¶ȣ�N=1-4��
C���ŵ㣺
    �������Ը��������õ��������ã�ƽ���ȸߣ�
    �����ڸ�Ƶ�񵴵�ϵͳ��
D��ȱ�㣺
    �����ȵͣ���żȻ���ֵ������Ը��ŵ��������ýϲ
    �������������������������Ĳ���ֵƫ�
    ��������������űȽ����صĳ��ϣ�
    �Ƚ��˷�RAM��
*/

float data[DEEPTH]; 
float SlideFilter(float in)
{
    uint8_t i = 0;
    double sum = 0;

    data[DEEPTH-1] = in;  // �����ݷŵ����
    for (i=1; i<DEEPTH; i++) 
	{
        data[i-1] = data[i];  // ��������
	}
    for (i=0; i<DEEPTH; i++)
	{
        sum += data[i];
	}

    return (sum / (float)DEEPTH);
}



/*
A�����ƣ���λֵ�˲���
B��������
    ��������N�Σ�Nȡ����������N�β���ֵ����С���У�
    ȡ�м�ֵΪ������Чֵ��
C���ŵ㣺
    ����Ч�˷���żȻ��������Ĳ������ţ�
    ���¶ȡ�Һλ�ı仯�����ı�����������õ��˲�Ч����
D��ȱ�㣺
    ���������ٶȵȿ��ٱ仯�Ĳ������ˡ�
*/
int MedianFilter(int filter_buf[MEDIAN_FILTER_N]) 
{
  int i, j;
  int filter_temp;

  // ����ֵ��С�������У�ð�ݷ���
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
A�����ƣ�����ƽ���˲���
B��������
    ����ȡN������ֵ��������ƽ�����㣺
    Nֵ�ϴ�ʱ���ź�ƽ���Ƚϸߣ��������Ƚϵͣ�
    Nֵ��Сʱ���ź�ƽ���Ƚϵͣ��������Ƚϸߣ�
    Nֵ��ѡȡ��һ��������N=12��ѹ����N=4��
C���ŵ㣺
    �����ڶ�һ�����������ŵ��źŽ����˲���
    �����źŵ��ص�����һ��ƽ��ֵ���ź���ĳһ��ֵ��Χ�������²�����
D��ȱ�㣺
    ���ڲ����ٶȽ�����Ҫ�����ݼ����ٶȽϿ��ʵʱ���Ʋ����ã�
    �Ƚ��˷�RAM��
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
A�����ƣ���λֵƽ���˲������ֳƷ��������ƽ���˲�����
B��������
    ��һ�����ȥ�����ֵ����Сֵ��ȡƽ��ֵ��
    �൱�ڡ���λֵ�˲�����+������ƽ���˲�������
    ��������N�����ݣ�ȥ��һ�����ֵ��һ����Сֵ��
    Ȼ�����N-2�����ݵ�����ƽ��ֵ��
    Nֵ��ѡȡ��3-14��
C���ŵ㣺
    �ں��ˡ���λֵ�˲�����+������ƽ���˲����������˲������ŵ㡣
    ����żȻ���ֵ������Ը��ţ�����������������Ĳ���ֵƫ�
    �����ڸ��������õ��������á�
    ƽ���ȸߣ����ڸ�Ƶ�񵴵�ϵͳ��
D��ȱ�㣺
    �����ٶȽ�����������ƽ���˲���һ����
    �Ƚ��˷�RAM��
*/

int MedianAverageFilter(int filter_buf[MEDIAN_AVERAGE_FILTER_N]) 
{
	int i, j;
	int filter_temp, filter_sum = 0;
	
  // ����ֵ��С�������У�ð�ݷ���
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
  // ȥ�������С��ֵ����ƽ��
	for(i = 1; i < MEDIAN_AVERAGE_FILTER_N - 1; i++) 
	{
		filter_sum += filter_buf[i];
	}
	
  return filter_sum / (MEDIAN_AVERAGE_FILTER_N - 2);
}


/*
A�����ƣ���Ȩ����ƽ���˲���
B��������
    �ǶԵ���ƽ���˲����ĸĽ�������ͬʱ�̵����ݼ��Բ�ͬ��Ȩ��
    ͨ���ǣ�Խ�ӽ���ʱ�̵����ݣ�Ȩȡ��Խ��
    �����²���ֵ��Ȩϵ��Խ����������Խ�ߣ����ź�ƽ����Խ�͡�
C���ŵ㣺
    �������нϴ��ͺ�ʱ�䳣���Ķ��󣬺Ͳ������ڽ϶̵�ϵͳ��
D��ȱ�㣺
    ���ڴ��ͺ�ʱ�䳣����С���������ڽϳ����仯�������źţ�
    ����Ѹ�ٷ�Ӧϵͳ��ǰ���ܸ��ŵ����س̶ȣ��˲�Ч���
*/
int coe[WEIGHTING_AVERAGE_FILTER_N] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};    // ��Ȩϵ����
int sum_coe = 1 + 2 + 3 + 4 + 5 + 6 + 7 + 8 + 9 + 10 + 11 + 12; // ��Ȩϵ����
int Filter(int filter_buf[WEIGHTING_AVERAGE_FILTER_N + 1]) 
{
	int i;
	int filter_sum = 0;
	
	for(i = 0; i < WEIGHTING_AVERAGE_FILTER_N; i++) 
	{
		filter_buf[i] = filter_buf[i + 1]; // �����������ƣ���λ�Ե�
		filter_sum += filter_buf[i] * coe[i];
	}
	filter_sum /= sum_coe;
	
	return filter_sum;
}


