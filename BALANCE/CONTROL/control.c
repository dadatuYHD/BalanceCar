#include "control.h"	
#include "filter.h"	

int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
float Balance_kp = 450, Balance_ki = 0, Balance_kd = 1.2; 
float Velocity_kp = 80, Velocity_ki = 0.4, Velocity_kd = 0; 
float Turn_kp = 30, Turn_ki = 0, Turn_kd = 0; 
float Angle_Balance,Gyro_Balance,Gyro_Turn; //ƽ����� ƽ����ٶ� ת����ٶ�
u8 Flag_Target;
int ultrasoniccount, CCDcount, Velocitycount;
/*****CCD��ر���**************/
uint8_t Pixel[128] = {0};                  //CCDAD���ݱ�������
uint8_t Pixelbinarization[128];            //��ֵ���������
u8 send_data_cnt = 0;                      //CCD���ݷ��ͼ������
u8 threshold;                              //��̬��ֵ
uint8_t Theory_midline = 64;               //�����ϵ���������
uint8_t Actual_midline;                    //ʵ���ϵ���������
int ready;
/**************************************************************************
�������ܣ����еĿ��ƴ��붼��������
         10ms��ʱ�ж��ɶ�ʱ�������жϴ���
         �ϸ�֤���������ݴ����ʱ��ͬ��				 
**************************************************************************/
#if 0
void TIM2_IRQHandler(void)  
{    
	if (TIM2->SR & 0X0001)//10ms��ʱ�ж�
	{   
		TIM2->SR &= ~(1<<0);                                            //����жϱ�־λ
		ultrasoniccount++;                                              //������������ڼ���
		CCDcount++;                                                     //CCD������־
		send_data_cnt++;                                                //CCD���ݷ��ͱ�־λ
		Velocitycount++;                                                //�ٶȻ����Ʊ�־λ
		
        Get_Angle(Way_Angle);		                                    //===�����˲��㷨������̬	
		Encoder_Right = Read_Encoder(4);                                //===��ȡ��������ֵ
		#if Adjust                                                         //�����ֱ��������Ƶ������ֱ�����
			Encoder_Left = Encoder_Right;
		#else
			Encoder_Left = -Read_Encoder(2);                                //===��ȡ��������ֵ����Ϊ�����������ת��180�ȵģ����Զ�����һ��ȡ������֤�������һ��
		#endif

		
		/*************************
        ����������ݶ�ȡ
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
				ImageCapture(Pixel);                                 //CCD����  Sampling CCD data 
			 /* Calculate Integration Time */
				//CalculateIntegrationTime(); 
                threshold = GetOSTUThreshold(Pixel);                 //�������̬��ֵ
				//binarization(Pixel);                                 //��ֵ��
				//Black_line_extraction(Pixel);                        //��ȡʵ�����ߵ�����
				//ready = 1;
			}
			if (ultrasoniccount >= 6)
			{
				ultrasoniccount = 0;
				Read_Distane();                                         //BIZHANG_MODE=1���ܶ�ȡ����������
			}  				
			Voltage=Get_battery_volt();                              //===��ȡ��ص�ѹ
			Temperature=Read_Temperature();		                       //===��ȡMPU6050�����¶ȴ��������ݣ����Ʊ�ʾ�����¶ȡ�
		}	
 		Balance_Pwm = balance(Angle_Balance, Gyro_Balance);        //===ƽ��PID����
      	if (Velocitycount >= 1)
		{
			Velocitycount = 0;
			Velocity_Pwm = velocity(Encoder_Left, Encoder_Right);       //===�ٶȻ�PID����	 ��ס���ٶȷ�����������������С�����ʱ��Ҫ����������Ҫ���ܿ�һ��
		}	
 	    Turn_Pwm = turn(Encoder_Left, Encoder_Right, Gyro_Turn); //===ת��PID����     
 		Moto1 = Balance_Pwm + Velocity_Pwm + Turn_Pwm + Incremental_PI(Encoder_Left , Movement);                 //===�������ֵ������PWM
 	  	Moto2 = Balance_Pwm + Velocity_Pwm - Turn_Pwm + Incremental_PI(Encoder_Right , Movement);                 //===�������ֵ������PWM
//		Moto1 = Balance_Pwm;//ƽ�⻷����
//		Moto2 = Balance_Pwm;
//		Moto1 = Balance_Pwm + Velocity_Pwm;
//	    Moto2 = Balance_Pwm + Velocity_Pwm;
//		Moto1 = Velocity_Pwm;
//		Moto2 = Velocity_Pwm;
   		Xianfu_Pwm();                                            //===PWM�޷�
        if (Turn_Off(Angle_Balance) == 0)                    //===����������쳣
			Set_Pwm(Moto1,Moto2);                                      //===��ֵ	                           
	}		   
} 
#endif
/**************************************************************************
�������ܣ����еĿ��ƴ��붼��������
         5ms��ʱ�ж���6050�ж����Ŵ���
         �ϸ�֤���������ݴ����ʱ��ͬ��				
float Balance_kp = 450, Balance_ki = 0, Balance_kd = 1.2; 
float Velocity_kp = 80, Velocity_ki = 0.4, Velocity_kd = 0; 
float Turn_kp = 15, Turn_ki = 0, Turn_kd = 0; 
**************************************************************************/
int EXTI3_IRQHandler(void) 
{
	if (PAin(3) == 0)
	{
		EXTI->PR = 1 << 3;                                               //���LINE3�ϵ��жϱ�־λ 
		ultrasoniccount++;                                              //������������ڼ���
		CCDcount++;                                                     //CCD������־
		send_data_cnt++;                                                //CCD���ݷ��ͱ�־λ
		if (Way_Angle == 1)
			Get_Angle(Way_Angle);
		else
		{
			Flag_Target =! Flag_Target;
			if (Flag_Target == 1)                                       //5ms��ȡһ�������Ǻͼ��ٶȼƵ�ֵ�����ߵĲ���Ƶ�ʿ��Ը��ƿ������˲��ͻ����˲���Ч��
			{
				Get_Angle(Way_Angle);                                   //===������̬	
				return 0;	
			}  
		}
		Encoder_Right = Read_Encoder(4);                                //===��ȡ��������ֵ
		#if Adjust                                                         //�����ֱ��������Ƶ������ֱ�����
			Encoder_Left = Encoder_Right;
		#else
			Encoder_Left = -Read_Encoder(2);                                //===��ȡ��������ֵ����Ϊ�����������ת��180�ȵģ����Զ�����һ��ȡ������֤�������һ��
		#endif
		Get_Angle(Way_Angle);		                                    //===�����˲��㷨������̬	
		
		/*************************
        ����������ݶ�ȡ
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
				ImageCapture(Pixel);                                 //CCD����  Sampling CCD data 
			 /* Calculate Integration Time */
				//CalculateIntegrationTime(); 
                threshold = GetOSTUThreshold(Pixel);                 //�������̬��ֵ
				Black_line_extraction(Pixel);                        //��ȡʵ�����ߵ�����
				//ready = 1;
			}
			if (ultrasoniccount == 12)
			{
				ultrasoniccount = 0;
				Read_Distane();                                         //BIZHANG_MODE=1���ܶ�ȡ����������
			}  				
			Voltage=Get_battery_volt();                              //===��ȡ��ص�ѹ
			Temperature=Read_Temperature();		                       //===��ȡMPU6050�����¶ȴ��������ݣ����Ʊ�ʾ�����¶ȡ�
		}	
 		Balance_Pwm = balance(Angle_Balance, Gyro_Balance);        //===ƽ��PID����	
		Velocity_Pwm = velocity(Encoder_Left, Encoder_Right);       //===�ٶȻ�PID����	 ��ס���ٶȷ�����������������С�����ʱ��Ҫ����������Ҫ���ܿ�һ��
 	    Turn_Pwm = turn(Encoder_Left, Encoder_Right, Gyro_Turn); //===ת��PID����     
 		Moto1 = Balance_Pwm + Velocity_Pwm + Turn_Pwm /*+ Incremental_PI(Encoder_Left , Movement)*/;                 //===�������ֵ������PWM
 	  	Moto2 = Balance_Pwm + Velocity_Pwm - Turn_Pwm  /*Incremental_PI(Encoder_Right , Movement)*/;                 //===�������ֵ������PWM
//		Moto1 = Balance_Pwm;//ƽ�⻷����
//		Moto2 = Balance_Pwm;
//		Moto1 = Balance_Pwm + Velocity_Pwm;
//	    Moto2 = Balance_Pwm + Velocity_Pwm;
//		Moto1 = Velocity_Pwm;
//		Moto2 = Velocity_Pwm;
   		Xianfu_Pwm();                                            //===PWM�޷�
        if (Turn_Off(Angle_Balance) == 0)                    //===����������쳣
			Set_Pwm(Moto1,Moto2);
		//===��ֵ	 
	}	
	return 0;	 		 
}
/**************************************************************************
��������: �����ⲿ�жϺ���
��ڲ����� ��
����  ֵ�� 0
**************************************************************************/
//int EXTI9_5_IRQHandler(void)
//{
//	if (PAin(7) == 0)
//	{ 
//		delay_ms(10);
//		if (PAin(7) == 0)
//		{
//			while(!PAin(7));
//		    Key();	                                                          //��������ģʽ			
//		}
//	}
//	EXTI->PR =1 << 7;
//	
//	return 0;
//}

/**************************************************************************
�������ܣ�ֱ��PD����
��ڲ������Ƕȡ����ٶ�
����  ֵ��ֱ������PWM
**************************************************************************/
int balance(float Angle,float Gyro)
{  
	int balance;
    float Bias;                      ;                                                                                                                                                                                                                                                                                                    ;
	
	Bias = Angle - 1;              //===���ƽ��ĽǶ���ֵ �ͻ�е��� -0��ζ������������0�ȸ��� �������������5�ȸ��� �Ǿ�Ӧ�ü�ȥ5
	balance = Balance_kp * Bias + Gyro * Balance_kd;   //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ��
	
	return balance;
}
/**************************************************************************
�������ܣ��ٶ�PI���� �޸�ǰ�������ٶȣ����޸�Movement��ֵ�����磬�ĳ�-60��60�ͱȽ�����
��ڲ��������ֱ����������ֱ�����
����  ֵ���ٶȿ���PWM
**************************************************************************/
int velocity(int encoder_left,int encoder_right)
{  

    static float Velocity, Encoder_Least, Encoder, Movement;
	static float Encoder_Integral, Target_Velocity;
	//=============ң��ǰ�����˲���=======================//
    if (Distance < 80)             Target_Velocity = 10;                 //����������ģʽ,�Զ��������ģʽ
	if (error>10 || error <-10) Target_Velocity = 100; 
    Target_Velocity = 120;
	if (1 == Flag_Qian)	Movement = -Target_Velocity / Flag_sudu;	             //===���ǰ����־λ��1 λ��Ϊ��
	else if (1 == Flag_Hou) Movement = Target_Velocity / Flag_sudu;        //===������˱�־λ��1 λ��Ϊ��
	else if (Flag_Stop==0) Movement = -Target_Velocity;
	else  Movement = 0;
//    if (Bi_zhang==1 && Distance<30 && Flag_Left!=1 && Flag_Right!=1 && Flag_sudu==2)        //���ϱ�־λ��1�ҷ�ң��ת���ʱ�򣬽������ģʽ
//		Movement = Target_Velocity/Flag_sudu;	
    //=============�ٶ�PI������=======================//	
	Encoder_Least = (Encoder_Left + Encoder_Right) - 0;         //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩 
	Encoder *= 0.7;		                                       //===һ�׵�ͨ�˲���       
	Encoder += Encoder_Least * 0.3;	                               //===һ�׵�ͨ�˲���    
	Encoder_Integral += Encoder;                                  //===���ֳ�λ�� ����ʱ�䣺10ms
	Encoder_Integral = Encoder_Integral -  Movement;                  //===����ң�������ݣ�����ǰ������
	if (Encoder_Integral > 10000) Encoder_Integral = 10000;         //===�����޷�
	if(Encoder_Integral < -10000) Encoder_Integral = -10000;         //===�����޷�	
	Velocity = Encoder * Velocity_kp + Encoder_Integral * Velocity_ki; //===�ٶȿ���	
	if (Turn_Off(Angle_Balance) == 1) Encoder_Integral = 0;    //===����رպ��������
	
    return Velocity;
}

/**************************************************************************
�������ܣ�ת��PD����
��ڲ��������ֱ����������ֱ�������Z��������
����  ֵ��ת�����PWM
**************************************************************************/
int turn(int encoder_left, int encoder_right, float gyro)//ת�����
{
	  static float Turn_Target,Turn,Encoder_temp,Turn_Convert=0.9,Turn_Count;
	  float Turn_Amplitude = 60;     
	  //=============ң��������ת����=======================//
  	if (1==Flag_Left || 1==Flag_Right || Distance < 30 || error != 0)                      //��һ������Ҫ�Ǹ�����תǰ���ٶȵ����ٶȵ���ʼ�ٶȣ�����С������Ӧ��
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
	
    if (Turn_Target > Turn_Amplitude)  Turn_Target = Turn_Amplitude;    //===ת���ٶ��޷�
	if (Turn_Target < -Turn_Amplitude) Turn_Target = -Turn_Amplitude;
	if (Flag_Qian==1 || Flag_Hou==1)  Turn_kd = 1;        
	else ;   //ת���ʱ��ȡ�������ǵľ��� �е�ģ��PID��˼��
//	if (error<20 || error>-20) Turn_kp = 10;
//	else Turn_kp = 12;
  	//=============ת��PD������=======================//
	Turn = -Turn_Target * Turn_kp - gyro * Turn_kd;                 //===���Z�������ǽ���PD����
	return Turn;
}

/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ���������PWM������PWM
����  ֵ����
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
�������ܣ�����PWM��ֵ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Xianfu_Pwm(void)
{	
    int Amplitude = 7100;    //===PWM������7200 ������7100
	
    if (Moto1 < -Amplitude) Moto1 = -Amplitude;	
	if (Moto1 > Amplitude)  Moto1 = Amplitude;	
	if (Moto2 < -Amplitude) Moto2 = -Amplitude;	
	if (Moto2 > Amplitude)  Moto2 = Amplitude;		
	
}
/**************************************************************************
�������ܣ������޸�С������״̬ 
��ڲ�������
����  ֵ����
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
			;                             //=====���Bi_zhang��0�����TIM1��ʼ��CCD����ģʽ
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
�������ܣ��쳣�رյ��
��ڲ�������Ǻ͵�ѹ
����  ֵ��1���쳣  0������
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
�������ܣ���ȡ�Ƕ�
��ڲ�������ȡ�Ƕȵ��㷨 1����  2�������� 3�������˲�
����  ֵ����
**************************************************************************/
void Get_Angle(u8 way)
{ 
    float Accel_Y, Accel_X, Accel_Z, Gyro_Y, Gyro_Z;
	    
	  if (way ==1)                                      //DMPû���漰���ϸ��ʱ�����⣬����������ȡ
	  {	
	      Read_DMP();                      //===��ȡ���ٶȺ����
		  Angle_Balance = Pitch;             //===����ƽ�����
		  Gyro_Balance = gyro[1];            //===����ƽ����ٶ�
		  Gyro_Turn = gyro[2];               //===����ת����ٶ�
	  }			
      else
      {
		 Gyro_Y = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_L); //��ȡY��������
		 Gyro_Z = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_L); //��ȡZ��������
		 Accel_X = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_L); //��ȡX����ٶȼ�
	  	 Accel_Z = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_L); //��ȡZ����ٶȼ�
		 if (Gyro_Y > 32768)  Gyro_Y -= 65536;     //��������ת��  Ҳ��ͨ��shortǿ������ת��
	     if (Gyro_Z > 32768)  Gyro_Z -= 65536;     //��������ת��
	  	 if (Accel_X >32768)  Accel_X -= 65536;    //��������ת��
		 if (Accel_Z > 32768) Accel_Z -= 65536;    //��������ת��
		 Gyro_Balance = -Gyro_Y;                                  //����ƽ����ٶ�
	   	 Accel_Y = atan2(Accel_X,Accel_Z) * 180 / PI;                 //������ٶȼ�Y����ٶ�	
		 Gyro_Y = Gyro_Y / 16.4;                                    //����������ת��	
         if (Way_Angle == 2) Kalman_Filter(Accel_Y, -Gyro_Y);//�������˲�	
		 else if (Way_Angle==3) Yijielvbo(Accel_Y, -Gyro_Y);    //�����˲�
	     Angle_Balance = angle;                                   //����ƽ�����
		 Gyro_Turn =Gyro_Z;                                      //����ת����ٶ�
		 /****�������������ݸ���*******/ 
		 gyro[1] = Gyro_Balance;             //===����ƽ����ٶ�
		 gyro[2] = Gyro_Turn;               //===����ת����ٶ�   
	  }
}
/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if (a < 0)  temp = -a;  
	  else temp = a;
	
	  return temp;
}
/**************************************************************************
�������ܣ�����PI������
��ڲ���������������ֵ��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI (int Encoder,int Target)
{ 	
    float Kp = 100,Ki = 100;
	
	static int Bias, Pwm, Last_bias;
	
	Bias = Encoder - Target;                //����ƫ��
	Pwm += Kp * (Bias - Last_bias) + Ki * Bias;   //����ʽPI������
	Last_bias = Bias;	                   //������һ��ƫ�� 
	
	return Pwm;                         //�������
}
