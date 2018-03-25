#include "sys.h"

u8 Way_Angle = 2;                             //��ȡ�Ƕȵ��㷨��1����Ԫ��  2��������  3�������˲� Ĭ�ϴ��ؿ������˲�
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu=2; //����ң����صı���
u8 Flag_Stop = 1, Flag_Show = 0;                 //ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ�ر�
int Encoder_Left,Encoder_Right;             //���ұ��������������
int Moto1,Moto2;                            //���PWM����
int Temperature;                            //��ʾ�¶�
int Voltage;                                //��ص�ѹ������صı���

int main(void)
{ 
	Stm32_Clock_Init(9);            //ϵͳʱ������
	delay_init(72);                 //��ʱ��ʼ��
	JTAG_Set(JTAG_SWD_DISABLE);     //=====�ر�JTAG�ӿ�
	JTAG_Set(SWD_ENABLE);           //=====��SWD�ӿ� �������������SWD�ӿڵ���
	//KEY_Init();                     //������ʼ��
	OLED_Init();                    //OLED��ʼ��
	Beep_init();                    //��������ʼ��
	uart_init(72,115200);           //��ʼ������1
	uart3_init(36,9600);            //����3�����жϳ�ʼ��
	Adc_Init();                     //ADC��ʼ��
	MiniBalance_PWM_Init(7199,0);   //=====��ʼ��PWM 10KHZ ��Ƶ���Է�ֹ�����Ƶʱ�ļ����
	TIM1_Cap_Init(0XFFFF,71);   //=====���MODE_BIZHANG��1�����TIM1��ʼ��Ϊ�������ӿ�
	LandzoCCD_init();                     //CCD����ͷ�ӿڣ�ADC��ʼ��
	Encoder_Init_TIM4();            //��ʼ��������4
   // Black_line_extraction(Pixel);                        //��ȡʵ�����ߵ�����	
    #if Adjust
		Timer2_Init(200, 71);       //CCD�عⶨʱ��
    #else
	    Encoder_Init_TIM2();            //��ʼ��������2
    #endif	
	IIC_Init();                     //ģ��IIC��ʼ��
  MPU6050_initialize();           //=====MPU6050��ʼ��	
	DMP_Init();                     //��ʼ��DMP     
	//di();                      //��Դ��������������
	EXTI_Init();                    //=====5MS��һ���жϷ��������жϷ�������control.c
	//Timer2_Init(100, 7199);            //10ms��ʱ�ж�

	while (1)
	{ 
		/************************���ʹ��������������Ϣ***************/
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
		/**********************OLED��ʾ�ͷ���APP��ʾ��Ϣ***************/
		else if (Flag_Show == 0)
		{
			//APP_Show();
			oled_show(); //OLED��ʾ
			/*******************����λ���϶�ȡ��ǰPIDֵ****************/
			if (send_pid1)
			{
				send_pid1 = 0;
				Data_Send_PID1();
			}
		}
       /****************************����������***********************/		
		if (Voltage < 1100)
			Beep = 0;
        else
            Beep = 1;
	  /************************CCDͼ�����ݷ���************************/
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
        //oled_show(); //OLED��ʾ	
		//printf("The threshold is %d\r\n", threshold);	
       //printf ("The error is %d Actual_midline is%d \r\n", error, Actual_midline);	
	} 
}
