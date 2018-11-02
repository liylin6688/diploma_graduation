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
#define OLED_MODE 								1 	//OLED模式设置 0: 4线串行模式;1: 并行8080模式
#define OLED_CMD  								0	//写命令
#define OLED_DATA 								1	//写数据	

void OLED_Init(void);

#endif  
	 
