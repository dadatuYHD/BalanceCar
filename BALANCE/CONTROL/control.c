#include "control.h"	
#include "filter.h"	

int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
float Balance_kp = 450, Balance_ki = 0, Balance_kd = 1.2; 
float Velocity_kp = 80, Velocity_ki = 0.4, Velocity_kd = 0; 
float Turn_kp = 30, Turn_ki = 0, Turn_kd = 0; 
float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡倾角 平衡角速度 转向角速度
u8 Flag_Target;
int ultrasoniccount, CCDcount, Velocitycount;
/*****CCD相关变量**************/
uint8_t Pixel[128] = {0};                  //CCDAD数据保存数组
uint8_t Pixelbinarization[128];            //二值化后的数据
u8 send_data_cnt = 0;                      //CCD数据发送间隔计数
u8 threshold;                              //动态阈值
uint8_t Theory_midline = 64;               //理论上的中线坐标
uint8_t Actual_midline;                    //实际上的中线坐标
int ready;
/**************************************************************************
函数功能：所有的控制代码都在这里面
         10ms定时中断由定时器更新中断触发
         严格保证采样和数据处理的时间同步				 
**************************************************************************/
#if 0
void TIM2_IRQHandler(void)  
{    
	if (TIM2->SR & 0X0001)//10ms定时中断
	{   
		TIM2->SR &= ~(1<<0);                                            //清楚中断标志位
		ultrasoniccount++;                                              //超声波测距周期计数
		CCDcount++;                                                     //CCD计数标志
		send_data_cnt++;                                                //CCD数据发送标志位
		Velocitycount++;                                                //速度环控制标志位
		
        Get_Angle(Way_Angle);		                                    //===用于滤波算法更新姿态	
		Encoder_Right = Read_Encoder(4);                                //===读取编码器的值
		#if Adjust                                                         //用右轮编码器近似等于左轮编码器
			Encoder_Left = Encoder_Right;
		#else
			Encoder_Left = -Read_Encoder(2);                                //===读取编码器的值，因为两个电机的旋转了180度的，所以对其中一个取反，保证输出极性一致
		#endif

		
		/*************************
        相关外设数据读取
        *************************/	
        {	
			//Key();
			if (send_data_cnt >= 2)
			{
				send_data_cnt = 0;	
				TIME1flag_20ms = 1;
			}
			if (CCDcount >= 1)
			{
				CCDcount = 0;
				ImageCapture(Pixel);                                 //CCD采样  Sampling CCD data 
			 /* Calculate Integration Time */
				//CalculateIntegrationTime(); 
                threshold = GetOSTUThreshold(Pixel);                 //计算出动态阈值
				//binarization(Pixel);                                 //二值化
				//Black_line_extraction(Pixel);                        //提取实际中线的坐标
				//ready = 1;
			}
			if (ultrasoniccount >= 6)
			{
				ultrasoniccount = 0;
				Read_Distane();                                         //BIZHANG_MODE=1才能读取超声波数据
			}  				
			Voltage=Get_battery_volt();                              //===获取电池电压
			Temperature=Read_Temperature();		                       //===读取MPU6050内置温度传感器数据，近似表示主板温度。
		}	
 		Balance_Pwm = balance(Angle_Balance, Gyro_Balance);        //===平衡PID控制
      	if (Velocitycount >= 1)
		{
			Velocitycount = 0;
			Velocity_Pwm = velocity(Encoder_Left, Encoder_Right);       //===速度环PID控制	 记住，速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点
		}	
 	    Turn_Pwm = turn(Encoder_Left, Encoder_Right, Gyro_Turn); //===转向环PID控制     
 		Moto1 = Balance_Pwm + Velocity_Pwm + Turn_Pwm + Incremental_PI(Encoder_Left , Movement);                 //===计算左轮电机最终PWM
 	  	Moto2 = Balance_Pwm + Velocity_Pwm - Turn_Pwm + Incremental_PI(Encoder_Right , Movement);                 //===计算右轮电机最终PWM
//		Moto1 = Balance_Pwm;//平衡环调试
//		Moto2 = Balance_Pwm;
//		Moto1 = Balance_Pwm + Velocity_Pwm;
//	    Moto2 = Balance_Pwm + Velocity_Pwm;
//		Moto1 = Velocity_Pwm;
//		Moto2 = Velocity_Pwm;
   		Xianfu_Pwm();                                            //===PWM限幅
        if (Turn_Off(Angle_Balance) == 0)                    //===如果不存在异常
			Set_Pwm(Moto1,Moto2);                                      //===赋值	                           
	}		   
} 
#endif
/**************************************************************************
函数功能：所有的控制代码都在这里面
         5ms定时中断由6050中断引脚触发
         严格保证采样和数据处理的时间同步				
float Balance_kp = 450, Balance_ki = 0, Balance_kd = 1.2; 
float Velocity_kp = 80, Velocity_ki = 0.4, Velocity_kd = 0; 
float Turn_kp = 15, Turn_ki = 0, Turn_kd = 0; 
**************************************************************************/
int EXTI3_IRQHandler(void) 
{
	if (PAin(3) == 0)
	{
		EXTI->PR = 1 << 3;                                               //清除LINE3上的中断标志位 
		ultrasoniccount++;                                              //超声波测距周期计数
		CCDcount++;                                                     //CCD计数标志
		send_data_cnt++;                                                //CCD数据发送标志位
		if (Way_Angle == 1)
			Get_Angle(Way_Angle);
		else
		{
			Flag_Target =! Flag_Target;
			if (Flag_Target == 1)                                       //5ms读取一次陀螺仪和加速度计的值，更高的采样频率可以改善卡尔曼滤波和互补滤波的效果
			{
				Get_Angle(Way_Angle);                                   //===更新姿态	
				return 0;	
			}  
		}
		Encoder_Right = Read_Encoder(4);                                //===读取编码器的值
		#if Adjust                                                         //用右轮编码器近似等于左轮编码器
			Encoder_Left = Encoder_Right;
		#else
			Encoder_Left = -Read_Encoder(2);                                //===读取编码器的值，因为两个电机的旋转了180度的，所以对其中一个取反，保证输出极性一致
		#endif
		Get_Angle(Way_Angle);		                                    //===用于滤波算法更新姿态	
		
		/*************************
        相关外设数据读取
        *************************/	
        {	
			//Key();
			if (send_data_cnt >= 1)
			{
				send_data_cnt = 0;	
				TIME1flag_20ms = 1;
			}
			if (CCDcount >= 2)
			{
				CCDcount = 0;
				ImageCapture(Pixel);                                 //CCD采样  Sampling CCD data 
			 /* Calculate Integration Time */
				//CalculateIntegrationTime(); 
                threshold = GetOSTUThreshold(Pixel);                 //计算出动态阈值
				Black_line_extraction(Pixel);                        //提取实际中线的坐标
				//ready = 1;
			}
			if (ultrasoniccount == 12)
			{
				ultrasoniccount = 0;
				Read_Distane();                                         //BIZHANG_MODE=1才能读取超声波数据
			}  				
			Voltage=Get_battery_volt();                              //===获取电池电压
			Temperature=Read_Temperature();		                       //===读取MPU6050内置温度传感器数据，近似表示主板温度。
		}	
 		Balance_Pwm = balance(Angle_Balance, Gyro_Balance);        //===平衡PID控制	
		Velocity_Pwm = velocity(Encoder_Left, Encoder_Right);       //===速度环PID控制	 记住，速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点
 	    Turn_Pwm = turn(Encoder_Left, Encoder_Right, Gyro_Turn); //===转向环PID控制     
 		Moto1 = Balance_Pwm + Velocity_Pwm + Turn_Pwm /*+ Incremental_PI(Encoder_Left , Movement)*/;                 //===计算左轮电机最终PWM
 	  	Moto2 = Balance_Pwm + Velocity_Pwm - Turn_Pwm  /*Incremental_PI(Encoder_Right , Movement)*/;                 //===计算右轮电机最终PWM
//		Moto1 = Balance_Pwm;//平衡环调试
//		Moto2 = Balance_Pwm;
//		Moto1 = Balance_Pwm + Velocity_Pwm;
//	    Moto2 = Balance_Pwm + Velocity_Pwm;
//		Moto1 = Velocity_Pwm;
//		Moto2 = Velocity_Pwm;
   		Xianfu_Pwm();                                            //===PWM限幅
        if (Turn_Off(Angle_Balance) == 0)                    //===如果不存在异常
			Set_Pwm(Moto1,Moto2);
		//===赋值	 
	}	
	return 0;	 		 
}
/**************************************************************************
函数功能: 按键外部中断函数
入口参数： 无
返回  值： 0
**************************************************************************/
//int EXTI9_5_IRQHandler(void)
//{
//	if (PAin(7) == 0)
//	{ 
//		delay_ms(10);
//		if (PAin(7) == 0)
//		{
//			while(!PAin(7));
//		    Key();	                                                          //按键更改模式			
//		}
//	}
//	EXTI->PR =1 << 7;
//	
//	return 0;
//}

/**************************************************************************
函数功能：直立PD控制
入口参数：角度、角速度
返回  值：直立控制PWM
**************************************************************************/
int balance(float Angle,float Gyro)
{  
	int balance;
    float Bias;                      ;                                                                                                                                                                                                                                                                                                    ;
	
	Bias = Angle - 1;              //===求出平衡的角度中值 和机械相关 -0意味着身重中心在0度附近 如果身重中心在5度附近 那就应该减去5
	balance = Balance_kp * Bias + Gyro * Balance_kd;   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数
	
	return balance;
}
/**************************************************************************
函数功能：速度PI控制 修改前进后退速度，请修改Movement的值，比如，改成-60和60就比较慢了
入口参数：左轮编码器、右轮编码器
返回  值：速度控制PWM
**************************************************************************/
int velocity(int encoder_left,int encoder_right)
{  

    static float Velocity, Encoder_Least, Encoder, Movement;
	static float Encoder_Integral, Target_Velocity;
	//=============遥控前进后退部分=======================//
    if (Distance < 80)             Target_Velocity = 10;                 //如果进入避障模式,自动进入低速模式
	if (error>10 || error <-10) Target_Velocity = 100; 
    Target_Velocity = 120;
	if (1 == Flag_Qian)	Movement = -Target_Velocity / Flag_sudu;	             //===如果前进标志位置1 位移为负
	else if (1 == Flag_Hou) Movement = Target_Velocity / Flag_sudu;        //===如果后退标志位置1 位移为正
	else if (Flag_Stop==0) Movement = -Target_Velocity;
	else  Movement = 0;
//    if (Bi_zhang==1 && Distance<30 && Flag_Left!=1 && Flag_Right!=1 && Flag_sudu==2)        //避障标志位置1且非遥控转弯的时候，进入避障模式
//		Movement = Target_Velocity/Flag_sudu;	
    //=============速度PI控制器=======================//	
	Encoder_Least = (Encoder_Left + Encoder_Right) - 0;         //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
	Encoder *= 0.7;		                                       //===一阶低通滤波器       
	Encoder += Encoder_Least * 0.3;	                               //===一阶低通滤波器    
	Encoder_Integral += Encoder;                                  //===积分出位移 积分时间：10ms
	Encoder_Integral = Encoder_Integral -  Movement;                  //===接收遥控器数据，控制前进后退
	if (Encoder_Integral > 10000) Encoder_Integral = 10000;         //===积分限幅
	if(Encoder_Integral < -10000) Encoder_Integral = -10000;         //===积分限幅	
	Velocity = Encoder * Velocity_kp + Encoder_Integral * Velocity_ki; //===速度控制	
	if (Turn_Off(Angle_Balance) == 1) Encoder_Integral = 0;    //===电机关闭后清除积分
	
    return Velocity;
}

/**************************************************************************
函数功能：转向PD控制
入口参数：左轮编码器、右轮编码器、Z轴陀螺仪
返回  值：转向控制PWM
**************************************************************************/
int turn(int encoder_left, int encoder_right, float gyro)//转向控制
{
	  static float Turn_Target,Turn,Encoder_temp,Turn_Convert=0.9,Turn_Count;
	  float Turn_Amplitude = 60;     
	  //=============遥控左右旋转部分=======================//
  	if (1==Flag_Left || 1==Flag_Right || Distance < 30 || error != 0)                      //这一部分主要是根据旋转前的速度调整速度的起始速度，增加小车的适应性
	{
		if (++Turn_Count == 1)
			Encoder_temp = myabs(encoder_left + encoder_right);
			Turn_Convert = 50 / Encoder_temp;
			if(Turn_Convert < 0.6) Turn_Convert = 0.6;
			if(Turn_Convert > 3) Turn_Convert = 3;
	}	
	else
	{
			Turn_Convert = 0.9;
			Turn_Count = 0;
			Encoder_temp = 0;
	}			
	if (1 == Flag_Left)	                       Turn_Target -= Turn_Convert;
	else if (1 == Flag_Right)	               Turn_Target += Turn_Convert;                  
	else if (Distance < 30)                    Turn_Target -= Turn_Convert;
    else if (error>0 && Flag_Stop==0 ) 	       Turn_Target += Turn_Convert; 
	else if (error<0 && Flag_Stop==0 )          Turn_Target -= Turn_Convert; 
	else Turn_Target = 0;
	
    if (Turn_Target > Turn_Amplitude)  Turn_Target = Turn_Amplitude;    //===转向速度限幅
	if (Turn_Target < -Turn_Amplitude) Turn_Target = -Turn_Amplitude;
	if (Flag_Qian==1 || Flag_Hou==1)  Turn_kd = 1;        
	else ;   //转向的时候取消陀螺仪的纠正 有点模糊PID的思想
//	if (error<20 || error>-20) Turn_kp = 10;
//	else Turn_kp = 12;
  	//=============转向PD控制器=======================//
	Turn = -Turn_Target * Turn_kp - gyro * Turn_kd;                 //===结合Z轴陀螺仪进行PD控制
	return Turn;
}

/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int moto1,int moto2)
{
    if (moto1 > 0)	  AIN1 = 0,AIN2 = 1;
	else 	          AIN1 = 1,AIN2 = 0;	
	PWMA = myabs(moto1);
	if (moto2 > 0)	  BIN1 = 0,	BIN2 = 1;
	else              BIN1 = 1,	BIN2 = 0;
	PWMB = myabs(moto2);	
}

/**************************************************************************
函数功能：限制PWM赋值 
入口参数：无
返回  值：无
**************************************************************************/
void Xianfu_Pwm(void)
{	
    int Amplitude = 7100;    //===PWM满幅是7200 限制在7100
	
    if (Moto1 < -Amplitude) Moto1 = -Amplitude;	
	if (Moto1 > Amplitude)  Moto1 = Amplitude;	
	if (Moto2 < -Amplitude) Moto2 = -Amplitude;	
	if (Moto2 > Amplitude)  Moto2 = Amplitude;		
	
}
/**************************************************************************
函数功能：按键修改小车运行状态 
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{	
	u8 tmp;
	
	tmp = click_N_Double(50); 
	
	if (tmp == 1) 
	{
		Flag_Stop =~ Flag_Stop;
		if (Distance < 30)
		{
			OLED_Clear(); 
		}
		else
		{
		    OLED_Clear(); 
			;                             //=====如果Bi_zhang置0，则把TIM1初始化CCD控制模式
		}	                            
	}
	if (tmp == 2) 
	{
		Flag_Show = ~Flag_Show;
		if (Flag_Show)
		{
			OLED_Clear(); 
		}
		else
		{
			OLED_Clear(); 
		}   			
	}	
}

/**************************************************************************
函数功能：异常关闭电机
入口参数：倾角和电压
返回  值：1：异常  0：正常
**************************************************************************/
u8 Turn_Off(float angle)
{
	u8 temp;
	
	if (angle<=-40 || angle>=40)
	{	                     
		temp = 1;                                        
		AIN1 = 0;                                           
		AIN2 = 0;
		BIN1 = 0;
		BIN2 = 0;
		PWMA = 0;
        PWMB = 0;
    }
	else
    temp = 0;
	
    return temp;			
}
	
/**************************************************************************
函数功能：获取角度
入口参数：获取角度的算法 1：无  2：卡尔曼 3：互补滤波
返回  值：无
**************************************************************************/
void Get_Angle(u8 way)
{ 
    float Accel_Y, Accel_X, Accel_Z, Gyro_Y, Gyro_Z;
	    
	  if (way ==1)                                      //DMP没有涉及到严格的时序问题，在主函数读取
	  {	
	      Read_DMP();                      //===读取角速度和倾角
		  Angle_Balance = Pitch;             //===更新平衡倾角
		  Gyro_Balance = gyro[1];            //===更新平衡角速度
		  Gyro_Turn = gyro[2];               //===更新转向角速度
	  }			
      else
      {
		 Gyro_Y = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_L); //读取Y轴陀螺仪
		 Gyro_Z = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_L); //读取Z轴陀螺仪
		 Accel_X = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_L); //读取X轴加速度记
	  	 Accel_Z = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度记
		 if (Gyro_Y > 32768)  Gyro_Y -= 65536;     //数据类型转换  也可通过short强制类型转换
	     if (Gyro_Z > 32768)  Gyro_Z -= 65536;     //数据类型转换
	  	 if (Accel_X >32768)  Accel_X -= 65536;    //数据类型转换
		 if (Accel_Z > 32768) Accel_Z -= 65536;    //数据类型转换
		 Gyro_Balance = -Gyro_Y;                                  //更新平衡角速度
	   	 Accel_Y = atan2(Accel_X,Accel_Z) * 180 / PI;                 //计算加速度计Y轴加速度	
		 Gyro_Y = Gyro_Y / 16.4;                                    //陀螺仪量程转换	
         if (Way_Angle == 2) Kalman_Filter(Accel_Y, -Gyro_Y);//卡尔曼滤波	
		 else if (Way_Angle==3) Yijielvbo(Accel_Y, -Gyro_Y);    //互补滤波
	     Angle_Balance = angle;                                   //更新平衡倾角
		 Gyro_Turn =Gyro_Z;                                      //更新转向角速度
		 /****传感器发送数据更新*******/ 
		 gyro[1] = Gyro_Balance;             //===更新平衡角速度
		 gyro[2] = Gyro_Turn;               //===更新转向角速度   
	  }
}
/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if (a < 0)  temp = -a;  
	  else temp = a;
	
	  return temp;
}
/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI (int Encoder,int Target)
{ 	
    float Kp = 100,Ki = 100;
	
	static int Bias, Pwm, Last_bias;
	
	Bias = Encoder - Target;                //计算偏差
	Pwm += Kp * (Bias - Last_bias) + Ki * Bias;   //增量式PI控制器
	Last_bias = Bias;	                   //保存上一次偏差 
	
	return Pwm;                         //增量输出
}
