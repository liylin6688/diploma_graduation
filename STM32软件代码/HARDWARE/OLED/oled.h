#ifndef __OLED_H
#define __OLED_H

#include "bsp.h"

#define OLED_CS  				PDout(6)
#define OLED_RST 				PGout(15) 	
#define OLED_RS  				PDout(3)
#define OLED_WR  				PGout(14)		  
#define OLED_RD  				PGout(13)
#define OLED_SCLK 				PCout(0)
#define OLED_SDIN 				PCout(1)

//***************************OLED**************************************************
#define OLED_MODE 								1 	//OLEDģʽ���� 0: 4�ߴ���ģʽ;1: ����8080ģʽ
#define OLED_CMD  								0	//д����
#define OLED_DATA 								1	//д����	

void OLED_Init(void);

#endif  
	 
