#ifndef __SENSOR_H
#define __SENSOR_H

#include "bsp.h"

//***************************非接触式红外传感器寄存器地址****************************
#define MLX90614_ADDR_WRITE   					0x00	
#define MLX90614_ADDR_READ    					0x01
#define MLX90614_READ_RAM_AMBIENT_TEMP    		0x06
#define MLX90614_READ_RAM_OBJECT_TEMP     		0x07
#define MLX90614_RAM    						0x00
#define AMBIENT_TEMP    						0x06
#define OBJECT_TEMP     						0x07

//************************非接触式红外传感器IO方向设置********************************
#define SDA_IN()  				{GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=(u32)8<<4;}
#define SDA_OUT() 				{GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=(u32)3<<4;}

#define SMBUS_SCL    			PBout(8) 
#define SMBUS_SDA    			PBout(9) 	 
#define READ_SDA   	 			PBin(9)  
#define TRIG 					PAout(1)	
#define ECHO 					PAout(0)	

float Infrared_Temperature_Sensor(uint8_t ambient_or_object);
void SMBus_M_To_S_Ack(void);
uint8_t SMBus_Read_Byte(void);
uint8_t SMBus_S_To_M_Ack(void);
void SMBus_Send_Byte(uint8_t byteData);
void SMBus_Stop(void);
void SMBus_Start(void);
float Object_Temp_Read(void);
float Ambient_Temp_Read(void);
void PulseTrig(void);
float UltrasonicRanging(void);

#endif

