#include "bsp.h"

float dist_measure,dist_ideal,dist_old;
float temperature;

int main(int argc, char *argv[])
{
	bsp_Init();	
	PulseTrig();
	dist_measure = UltrasonicRanging();
	dist_old = dist_measure;
	while(1)
	{
		temperature = Object_Temp_Read();
		PulseTrig();
		dist_measure = UltrasonicRanging();
		if ((dist_measure>2)&&(dist_measure<20))
		{
//			dist_ideal = LowPassFilter(dist_old,dist_measure,0.3);
//			dist_old = dist_ideal;
			dist_ideal = dist_measure;
		}
		printf("%.2f \r\n",dist_ideal);
		printf("current temperature £º\t %.2f  ¡ãC\r\n",temperature);
		printf("current distance    £º\t %.2f  cm \r\n",dist_ideal);
		PrintfRobot();
		delay_ms(1000);	
	}
}
