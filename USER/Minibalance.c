#include "sys.h"

u8 Way_Angle = 2;                             //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波 默认搭载卡尔曼滤波
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu=2; //蓝牙遥控相关的变量
u8 Flag_Stop = 1, Flag_Show = 0;                 //停止标志位和 显示标志位 默认停止 显示关闭
int Encoder_Left,Encoder_Right;             //左右编码器的脉冲计数
int Moto1,Moto2;                            //电机PWM变量
int Temperature;                            //显示温度
int Voltage;                                //电池电压采样相关的变量

int main(void)
{ 
	Stm32_Clock_Init(9);            //系统时钟设置
	delay_init(72);                 //延时初始化
	JTAG_Set(JTAG_SWD_DISABLE);     //=====关闭JTAG接口
	JTAG_Set(SWD_ENABLE);           //=====打开SWD接口 可以利用主板的SWD接口调试
	//KEY_Init();                     //按键初始化
	OLED_Init();                    //OLED初始化
	Beep_init();                    //蜂鸣器初始化
	uart_init(72,115200);           //初始化串口1
	uart3_init(36,9600);            //串口3蓝牙中断初始化
	Adc_Init();                     //ADC初始化
	MiniBalance_PWM_Init(7199,0);   //=====初始化PWM 10KHZ 高频可以防止电机低频时的尖叫声
	TIM1_Cap_Init(0XFFFF,71);   //=====如果MODE_BIZHANG置1，则把TIM1初始化为超声波接口
	LandzoCCD_init();                     //CCD摄像头接口，ADC初始化
	Encoder_Init_TIM4();            //初始化编码器4
   // Black_line_extraction(Pixel);                        //提取实际中线的坐标	
    #if Adjust
		Timer2_Init(200, 71);       //CCD曝光定时器
    #else
	    Encoder_Init_TIM2();            //初始化编码器2
    #endif	
	IIC_Init();                     //模拟IIC初始化
  MPU6050_initialize();           //=====MPU6050初始化	
	DMP_Init();                     //初始化DMP     
	//di();                      //电源开启蜂鸣器报警
	EXTI_Init();                    //=====5MS进一次中断服务函数，中断服务函数在control.c
	//Timer2_Init(100, 7199);            //10ms定时中断

	while (1)
	{ 
		/************************发送传感器相关数据信息***************/
		if (Flag_Show == 1)
		{
			if ((cnt % senser_cnt) == (senser_cnt-1))
				send_senser = 1;	
	
			if ((cnt % status_cnt) == (status_cnt-1))
				send_status = 1;	
		
			cnt++;
		
			if (send_senser)
			{
				send_senser = 0;
				Send_Data((int16_t *)gyro,(int16_t *)accel);
			}
			else if (send_status)
			{
				send_status = 0;
				if (Way_Angle == 1)
					Data_Send_Status(Pitch,Roll,Yaw);
				else if (Way_Angle == 2)
					Data_Send_Status(Angle_Balance, 0, 0);
			}
			if (send_pid1)
			{
				send_pid1 = 0;
				Data_Send_PID1();
			}	
		}
		/**********************OLED显示和发送APP显示信息***************/
		else if (Flag_Show == 0)
		{
			//APP_Show();
			oled_show(); //OLED显示
			/*******************在上位机上读取当前PID值****************/
			if (send_pid1)
			{
				send_pid1 = 0;
				Data_Send_PID1();
			}
		}
       /****************************蜂鸣器报警***********************/		
		if (Voltage < 1100)
			Beep = 0;
        else
            Beep = 1;
	  /************************CCD图像数据发送************************/
    if (TIME1flag_20ms == 1)
		{
			TIME1flag_20ms = 0;
			//SendImageData(Pixel);	
		}
//        if (ready == 1)
//		{
//			ready = 0;
//			threshold = GetOSTUThreshold(Pixel);
//			Black_line_extraction(Pixel);
            //printf("The threshold is %d\r\n", threshold);	
//            //printf ("The error is %d \r\n", error);			
//		}	
        //oled_show(); //OLED显示	
		//printf("The threshold is %d\r\n", threshold);	
       //printf ("The error is %d Actual_midline is%d \r\n", error, Actual_midline);	
	} 
}
