#ifndef __DISPLAY_H
#define __DISPLAY_H

#include "bsp.h"

#define DATAOUT(x) 				GPIO_Write(GPIOC,x);//��� 

void OLED_WR_Byte(u8 dat,u8 cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);  		    
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_Fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size);
void OLED_ShowString(u8 x,u8 y,const u8 *p,u8 size);	
void Display(void);
void ModeShowing(void);
void ManualModeShowing(void);
void PRShowing(void);
void PrintfRobot(void);
void ShowError(void);

#endif  


