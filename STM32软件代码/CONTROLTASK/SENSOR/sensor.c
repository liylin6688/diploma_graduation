#include "bsp.h"

void  SMBus_Start(void)
{
	SDA_OUT();     //sda线输出
	SMBUS_SDA = 1;
	SMBUS_SCL = 1;
	delay_us(5);
	SMBUS_SDA = 0;
	delay_us(5);
	SMBUS_SCL = 0;
	delay_us(5);
}

void  SMBus_Stop(void)
{ 
	SDA_OUT();     //sda线输出
	delay_us(5);
	SMBUS_SDA = 0;
	delay_us(5);
	SMBUS_SCL = 1;
	delay_us(5);
	SMBUS_SDA = 1;
	delay_us(5);
}

void  SMBus_Send_Byte(uint8_t byteData)
{
	uint8_t  i;
	uint8_t  temp = 0x80;
	
	SDA_OUT();
	for(i = 0; i < 8; i++)
	{
		SMBUS_SCL = 0;
		delay_us(6);
		if((byteData & temp) == 0)	
			SMBUS_SDA = 0;	
		else	
			SMBUS_SDA = 1;
		temp >>= 1;
		delay_us(6);
		SMBUS_SCL = 1;
		delay_us(6);
	}
}

uint8_t  SMBus_S_To_M_Ack(void)
{	
	SMBUS_SCL = 0;
	delay_us(6);
	SMBUS_SDA = 1;	// Master must release the SDA to wait the Ack from slaver !!!
	delay_us(6);
	SDA_IN();      //SDA设置为输入  
	if(READ_SDA == 0)
	{
		SMBUS_SCL = 1;
		delay_us(6);
		if(READ_SDA == 0)
		{
			SMBUS_SCL = 0;
			delay_us(6);
			if(READ_SDA == 1)
			{
				return 0;   // Slaver Ack 
			}
			return 1;
		}
		return 2;
	}
	return 3;  
}

uint8_t  SMBus_Read_Byte(void)
{
	uint8_t  i;
	uint8_t  byteData;
	uint8_t  temp = 0x80;
		
	delay_us(6);
	SDA_IN();      //SDA设置为输入
	for(i = 0; i < 8; i++)
	{
		SMBUS_SCL = 0;
		delay_us(6);
		if(READ_SDA == 0)
			byteData &= ~temp;
		else	
			byteData |= temp;	
		temp >>= 1;
		SMBUS_SCL = 1;
		delay_us(6);		
	}
	return 	byteData;
}

void  SMBus_M_To_S_Ack(void)
{
	SDA_OUT();     //sda线输出
	SMBUS_SCL = 0;
	SMBUS_SDA = 0;
	delay_us(6);
	SMBUS_SCL = 1;
	delay_us(6);
	SMBUS_SCL = 0;
	delay_us(6);
	SMBUS_SDA = 1;	// Master must release the SDA after Ack !!!
	delay_us(6);
}


float Infrared_Temperature_Sensor(uint8_t ambient_or_object)
{
	uint32_t  temp_data;	
	float  T;	
	uint8_t     RxBuffer[3];
	SMBus_Start(); 
	SMBus_Send_Byte(MLX90614_ADDR_WRITE);
	SMBus_S_To_M_Ack();
	SMBus_Send_Byte(ambient_or_object);
	SMBus_S_To_M_Ack();
	SMBus_Start(); 
	SMBus_Send_Byte(MLX90614_ADDR_READ);
	SMBus_S_To_M_Ack();
	RxBuffer[0] = SMBus_Read_Byte();
	SMBus_M_To_S_Ack();
	RxBuffer[1] = SMBus_Read_Byte();
	SMBus_M_To_S_Ack();
	RxBuffer[2] = SMBus_Read_Byte();
	SMBus_M_To_S_Ack();
	SMBus_Stop();
	
	temp_data = RxBuffer[1]*256 + RxBuffer[0];	
	if(temp_data >= 0x8000)	   
		return 0;
	
	if(temp_data == 0)return 0;
	
	T = temp_data*0.02;
	if(T >= 273.15)
		return (T - 273.15);
	else 
		return  -(273.15 - T);		
}

float Object_Temp_Read(void)
{
	return Infrared_Temperature_Sensor(MLX90614_RAM | OBJECT_TEMP);
}

float Ambient_Temp_Read(void)
{
	return Infrared_Temperature_Sensor(MLX90614_RAM | AMBIENT_TEMP);
}


void PulseTrig(void)
{
	TRIG = 0;
	delay_us(10);
	TRIG = 1;
	delay_us(15);
	TRIG = 0;
}


float UltrasonicRanging(void)
{
	u32 temp = 0;
	
 	if(TIM5CH1_CAPTURE_STA&0X80)        //成功捕获到了一次高电平
	{
		temp = TIM5CH1_CAPTURE_STA&0X3F; 
		temp *= 0XFFFFFFFF;		 		         //溢出时间总和
		temp += TIM5CH1_CAPTURE_VAL;		   //得到总的高电平时间
		TIM5CH1_CAPTURE_STA = 0;			     //开启下一次捕获
		return (float)temp*340/200/100; //测量距离单位cm
	}
}


