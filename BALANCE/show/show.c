#include "show.h"
unsigned char i;          //计数变量
unsigned char Send_Count; //串口需要发送的数据个数
/**************************************************************************
函数功能：OLED显示
入口参数：无
返回  值：无
**************************************************************************/

void oled_show(void)
{
		OLED_Display_On();  //显示屏打开
//		//=============显示超声波测试距离=======================//	
//		if (Bi_zhang)
//		{	   
//			OLED_ShowString(0, 0, "Distance");
//			OLED_ShowNumber(65, 0, Distance,5,12);
//			OLED_ShowString(100, 0, "cm");
//		}
//        else
//	   //===============显示滤波算法============================//
//		{
//			OLED_ShowString(00,0,"WAY-");
//		    OLED_ShowNumber(30,0, Way_Angle,1,12);
//			if(Way_Angle==1)	    OLED_ShowString(45,0,"DMP");
//			else if(Way_Angle==2)	OLED_ShowString(45,0,"Kalman");
//			else if(Way_Angle==3)	OLED_ShowString(45,0,"Hubu");
//		}
//		//=============显示温度=======================//	
//		                      OLED_ShowString(00,12,"Wendu");
//		                      OLED_ShowNumber(45,12,Temperature/10,2,12);
//		                      OLED_ShowNumber(68,12,Temperature%10,1,12);
//		                      OLED_ShowString(58,12,".");
//		                      OLED_ShowString(80,12,"`C");
//		//=============显示编码器1=======================//	
//		                      OLED_ShowString(00,22,"Enco1");
//		if( Encoder_Left<0)		OLED_ShowString(45,22,"-"),
//		                      OLED_ShowNumber(65,22,-Encoder_Left,5,12);
//		else                 	OLED_ShowString(45,22,"+"),
//		                      OLED_ShowNumber(65,22, Encoder_Left,5,12);
//  	//=============显示编码器2=======================//		
//		                      OLED_ShowString(00,32,"Enco2");
//		if(Encoder_Right<0)		OLED_ShowString(45,32,"-"),
//		                      OLED_ShowNumber(65,32,-Encoder_Right,5,12);
//		else               		OLED_ShowString(45,32,"+"),
//		                      OLED_ShowNumber(65,32,Encoder_Right,5,12);	
//		//=============显示电压=======================//
//		                      OLED_ShowString(00,42,"Volta");
//		                      OLED_ShowString(58,42,".");
//		                      OLED_ShowString(80,42,"V");
//		                      OLED_ShowNumber(45,42,Voltage/100,2,12);
//		                      OLED_ShowNumber(68,42,Voltage%100,2,12);
//		 if(Voltage%100<10) 	OLED_ShowNumber(62,42,0,2,12);
//		//=============显示角度=======================//
//		                      OLED_ShowString(0,52,"Angle");
//		if(Angle_Balance<0)	  OLED_ShowNumber(45,52,Angle_Balance+360,3,12);
//		else				  OLED_ShowNumber(45,52,Angle_Balance,3,12);
//		//=============刷新=======================//
//		OLED_Refresh_Gram();
//=============第一行显示小车模式=======================//	
		if (Way_Angle == 1)	    OLED_ShowString(0,0,"DMP");
		else if (Way_Angle == 2)	OLED_ShowString(0,0,"Kalman");
		else if(Way_Angle == 3)	OLED_ShowString(0,0,"Hubu");
                   
	    if(Distance < 30)	OLED_ShowString(60,0,"Bizhang");
		else             OLED_ShowString(60,0,"CCD");
	  //=============第二行显示温度和距离===============//	
													OLED_ShowNumber(0,10,Temperature/10,2,12);
													OLED_ShowNumber(23,10,Temperature%10,1,12);
													OLED_ShowString(13,10,".");
													OLED_ShowString(35,10,"`C");
													OLED_ShowNumber(70,10,(u16)Distance,5,12);
			                    OLED_ShowString(105,10,"cm");
		//=============第三行显示编码器1=======================//	
		                      OLED_ShowString(00,20,"EncoLEFT");
		if( Encoder_Left<0)		OLED_ShowString(80,20,"-"),
		                      OLED_ShowNumber(95,20,-Encoder_Left,3,12);
		else                 	OLED_ShowString(80,20,"+"),
		                      OLED_ShowNumber(95,20, Encoder_Left,3,12);
  	//=============第四行显示编码器2=======================//		
		                      OLED_ShowString(00,30,"EncoRIGHT");
		if(Encoder_Right<0)		OLED_ShowString(80,30,"-"),
		                      OLED_ShowNumber(95,30,-Encoder_Right,3,12);
		else               		OLED_ShowString(80,30,"+"),
		                      OLED_ShowNumber(95,30,Encoder_Right,3,12);	
		//=============第五行显示电压=======================//
		                      OLED_ShowString(00,40,"Volta");
		                      OLED_ShowString(58,40,".");
		                      OLED_ShowString(80,40,"V");
		                      OLED_ShowNumber(45,40,Voltage/100,2,12);
		                      OLED_ShowNumber(68,40,Voltage%100,2,12);
		 if(Voltage%100<10) 	OLED_ShowNumber(62,40,0,2,12);
		//=============第六行显示角度=======================//
		                      OLED_ShowString(0,50,"Angle");
		if(Angle_Balance<0)		OLED_ShowNumber(45,50,Angle_Balance+360,3,12);
		else					        OLED_ShowNumber(45,50,Angle_Balance,3,12);
		//=============刷新=======================//
		OLED_Refresh_Gram();	
}
/**************************************************************************
函数功能：向APP发送数据
入口参数：无
返回  值：无
**************************************************************************/
void APP_Show(void)
{    
	int app_2, app_3, app_4;
	
	app_4 = (Voltage - 1110) * 2 / 3;
	if (app_4 < 0) app_4 = 0;
	if (app_4 > 100) app_4 = 100;
	app_3 = Encoder_Right*1.1; 
	if (app_3 < 0) app_3 = -app_3;			
	app_2 = Encoder_Left * 1.1;
	if (app_2 < 0) app_2 = -app_2;
	printf("Z%d:%d:%d:%dL$", (u8)app_2, (u8)app_3, (u8)app_4, (int)Angle_Balance);
}
