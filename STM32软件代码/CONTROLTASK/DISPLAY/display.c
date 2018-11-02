#include "bsp.h"
#include "oledfont.h" 

//OLED���Դ�
//��Ÿ�ʽ����.
//[0]0 1 2 3 ... 127	
//[1]0 1 2 3 ... 127	
//[2]0 1 2 3 ... 127	
//[3]0 1 2 3 ... 127	
//[4]0 1 2 3 ... 127	
//[5]0 1 2 3 ... 127	
//[6]0 1 2 3 ... 127	
//[7]0 1 2 3 ... 127 		   
u8 OLED_GRAM[128][8];	 

//�����Դ浽LCD		 
void OLED_Refresh_Gram(void)
{
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //����ҳ��ַ��0~7��
		OLED_WR_Byte (0x00,OLED_CMD);      //������ʾλ�á��е͵�ַ
		OLED_WR_Byte (0x10,OLED_CMD);      //������ʾλ�á��иߵ�ַ   
		for(n=0;n<128;n++)OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA); 
	}   
}
#if OLED_MODE==1	//8080���� 
//��SSD1306д��һ���ֽڡ�
//dat:Ҫд�������/����
//cmd:����/�����־ 0,��ʾ����;1,��ʾ����;
void OLED_WR_Byte(u8 dat,u8 cmd)
{
	DATAOUT(dat);	    
 	OLED_RS=cmd;
	OLED_CS=0;	   
	OLED_WR=0;	 
	OLED_WR=1;
	OLED_CS=1;	  
	OLED_RS=1;	 
} 	    	    
#else
//��SSD1306д��һ���ֽڡ�
//dat:Ҫд�������/����
//cmd:����/�����־ 0,��ʾ����;1,��ʾ����;
void OLED_WR_Byte(u8 dat,u8 cmd)
{	
	u8 i;			  
	OLED_RS=cmd; //д���� 
	OLED_CS=0;		  
	for(i=0;i<8;i++)
	{			  
		OLED_SCLK=0;
		if(dat&0x80)OLED_SDIN=1;
		else OLED_SDIN=0;
		OLED_SCLK=1;
		dat<<=1;   
	}				 
	OLED_CS=1;		  
	OLED_RS=1;   	  
} 
#endif
	  	  
//����OLED��ʾ    
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//�ر�OLED��ʾ     
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   			 
//��������,������,������Ļ�Ǻ�ɫ��!��û����һ��!!!	  
void OLED_Clear(void)  
{  
	u8 i,n;  
	for(i=0;i<8;i++)for(n=0;n<128;n++)OLED_GRAM[n][i]=0X00;  
	OLED_Refresh_Gram();//������ʾ
}
//���� 
//x:0~127
//y:0~63
//t:1 ��� 0,���				   
void OLED_DrawPoint(u8 x,u8 y,u8 t)
{
	u8 pos,bx,temp=0;
	if(x>127||y>63)return;//������Χ��.
	pos=7-y/8;
	bx=y%8;
	temp=1<<(7-bx);
	if(t)OLED_GRAM[x][pos]|=temp;
	else OLED_GRAM[x][pos]&=~temp;	    
}
//x1,y1,x2,y2 �������ĶԽ�����
//ȷ��x1<=x2;y1<=y2 0<=x1<=127 0<=y1<=63	 	 
//dot:0,���;1,���	  
void OLED_Fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot)  
{  
	u8 x,y;  
	for(x=x1;x<=x2;x++)
	{
		for(y=y1;y<=y2;y++)OLED_DrawPoint(x,y,dot);
	}													    
	OLED_Refresh_Gram();//������ʾ
}
//��ָ��λ����ʾһ���ַ�,���������ַ�
//x:0~127
//y:0~63
//mode:0,������ʾ;1,������ʾ				 
//size:ѡ������ 12/16/24
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode)
{      			    
	u8 temp,t,t1;
	u8 y0=y;
	u8 csize=(size/8+((size%8)?1:0))*(size/2);		//�õ�����һ���ַ���Ӧ������ռ���ֽ���
	chr=chr-' ';//�õ�ƫ�ƺ��ֵ		 
    for(t=0;t<csize;t++)
    {   
		if(size==12)temp=asc2_1206[chr][t]; 	 	//����1206����
		else if(size==16)temp=asc2_1608[chr][t];	//����1608����
		else if(size==24)temp=asc2_2412[chr][t];	//����2412����
		else return;								//û�е��ֿ�
        for(t1=0;t1<8;t1++)
		{
			if(temp&0x80)OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			temp<<=1;
			y++;
			if((y-y0)==size)
			{
				y=y0;
				x++;
				break;
			}
		}  	 
    }          
}
//m^n����
u32 mypow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}				  
//��ʾ2������
//x,y :�������	 
//len :���ֵ�λ��
//size:�����С
//mode:ģʽ	0,���ģʽ;1,����ģʽ
//num:��ֵ(0~4294967295);	 		  
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size)
{         	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size/2)*t,y,' ',size,1);
				continue;
			}else enshow=1; 
		 	 
		}
	 	OLED_ShowChar(x+(size/2)*t,y,temp+'0',size,1); 
	}
} 
//��ʾ�ַ���  x:col  y:row
//x,y:�������  
//size:�����С 
//*p:�ַ�����ʼ��ַ 
void OLED_ShowString(u8 x,u8 y,const u8 *p,u8 size)
{	
    while((*p<='~')&&(*p>=' '))//�ж��ǲ��ǷǷ��ַ�!
    {       
        if(x>(128-(size/2))){x=0;y+=size;}
        if(y>(64-size)){y=x=0;OLED_Clear();}
        OLED_ShowChar(x,y,*p,size,1);	 
        x+=size/2;
        p++;
    }  
}	   


void Display(void)
{
	ModeShowing();
	if (ManualMode == ManualMode)
	{
		ManualModeShowing();
	}	
	PRShowing();
  
	OLED_Refresh_Gram();		//������ʾ��OLED 
}

void PRShowing(void)
{
	OLED_ShowString(0,24,"P(cm) X:",12); 
	OLED_ShowNum(48,24,(u32)(RHS_7Bot.px*100),2,12);
	OLED_ShowString(66,24,"Y:",12); 
	OLED_ShowNum(78,24,(u32)(RHS_7Bot.py*100),2,12);
	OLED_ShowString(96,24,"Z:",12); 
	OLED_ShowNum(108,24,(u32)(RHS_7Bot.py*100),2,12);
}

void ModeShowing(void)
{
	OLED_ShowString(0,0,"MODE:",12);  
	switch (ControlMode)
	{
		case ManualMode:
			OLED_ShowString(35,0,"ManualMode",12);
			break;
		
		case AutoMode:
			OLED_ShowString(35,0,"Auto__Mode",12);
			break;
		
		case WriteMode:
			OLED_ShowString(35,0,"Write_Mode",12);
			break;

		case TeachingMode:
			OLED_ShowString(35,0,"Teach_Mode",12);
			break;
		
		case QueZhuJiu:
			OLED_ShowString(35,0,"QueZhu_Jiu",12);
			break;
		
		case HuiXuanJiu:
			OLED_ShowString(35,0,"HuiXuanJiu",12);
			break;
		
		default:
			break;
	}
}


void ManualModeShowing(void)
{
	OLED_ShowString(0,12,"STATE:",12);
	switch (ParamState)
	{
		case TransxState:
			OLED_ShowString(40,12,"Trans__X",12);
			break;
		
		case TransyState:
			OLED_ShowString(40,12,"Trans__Y",12);
			break;
		
		case TranszState:
			OLED_ShowString(40,12,"Trans__Z",12);
			break;
		
		case RotatexState:
			OLED_ShowString(40,12,"Rotate_X",12);
			break;
		
		case RotateyState:
			OLED_ShowString(40,12,"Rotate_Y",12);
			break;

		case RotatezState:
			OLED_ShowString(40,12, "Rotate_Z",12);
			break;
		
		default:
			break;
	}
}

void PrintfRobot(void)
{
	printf("Pose matrix��\r\n");
	printf("\t %.4f \t %.4f \t %.4f \t %.4f \r\n",RHS_7Bot.nx, RHS_7Bot.ox, RHS_7Bot.ax, RHS_7Bot.px);
	printf("\t %.4f \t %.4f \t %.4f \t %.4f \r\n",RHS_7Bot.ny, RHS_7Bot.oy, RHS_7Bot.ay, RHS_7Bot.py);
	printf("\t %.4f \t %.4f \t %.4f \t %.4f \r\n",RHS_7Bot.nz, RHS_7Bot.oz, RHS_7Bot.az, RHS_7Bot.pz);
	printf("\t 0 \t\t 0 \t\t 0 \t\t 1 \r\n");
	printf("Joint variable:\r\n");
	printf("\t %.4f \t %.4f \t %.4f \t %.4f \t %.4f \t %.4f \r\n",Joint.rad1, Joint.rad2, Joint.rad3, Joint.rad4, Joint.rad5, Joint.rad6);
}


void ShowError(void)
{

}
