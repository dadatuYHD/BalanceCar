#include "timer.h"
#include "led.h"
//��ʱ�������ݱ�־λ
u8 cnt = 0;
u8 senser_cnt  = 10;
u8 status_cnt  = 15;
u8 send_senser = 0;
u8 send_status = 0;
//u8 rcdata_cnt  = 20;
//u8 motopwm_cnt = 20;
//u8 power_cnt   = 50;
u8 TIME1flag_20ms; 
/**************************************************************************
�������ܣ���ʱ�жϳ�ʼ��
��ڲ�����arr���Զ���װֵ  psc��ʱ��Ԥ��Ƶ�� 
����  ֵ����
**************************************************************************/
void Timer2_Init(u16 arr,u16 psc)  
{  
	RCC->APB1ENR |= 1<<0;     //TIM2ʱ��ʹ��    
 	TIM2->ARR = arr;      //�趨�������Զ���װֵ   
	TIM2->PSC = psc;      //Ԥ��Ƶ��7200,�õ�10Khz�ļ���ʱ��
	TIM2->DIER |= 1<<0;   //��������ж�				
	TIM2->DIER |= 1<<6;   //�������ж�	   
	TIM2->CR1 |= 0x01;    //ʹ�ܶ�ʱ��
	MY_NVIC_Init(2,0,TIM2_IRQn,3);
}  

/**************************************************************************
�������ܣ���ʱ��3ͨ��1���벶��
��ڲ�������ڲ�����arr���Զ���װֵ  psc��ʱ��Ԥ��Ƶ�� 
����  ֵ����
**************************************************************************/
void TIM1_Cap_Init(u16 arr,u16 psc)	
{	 
	RCC->APB2ENR |= 1<<11;   	//TIM1 ʱ��ʹ�� 
	RCC->APB2ENR |= 1<<2;    	//ʹ��PORTAʱ��   	 
	GPIOA->CRL &= 0XFFFFF0FF; 
	GPIOA->CRL |= 0X00000200;//PA.2 �������   	PA.8 ���� 
	GPIOA->CRH &= 0XFFFFFFF0; 
	GPIOA->CRH |= 0X00000008;//PA.2 �������   	PA.8 ���� 
	
  TIM1->ARR = arr;  		//�趨�������Զ���װֵ   
	TIM1->PSC = psc;  		//Ԥ��Ƶ�� 
	TIM1->CCMR1 |= 1<<0;		//CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
 	TIM1->CCMR1 |= 0<<4; 		//IC1F=0000 ���������˲��� ���˲�
 	TIM1->CCMR1 |= 0<<10; 	//IC2PS=00 	���������Ƶ,����Ƶ 

	TIM1->CCER |= 0<<1; 		//CC1P=0	�����ز���
	TIM1->CCER |= 1<<0; 		//CC1E=1 	�������������ֵ������Ĵ�����

	TIM1->DIER |= 1<<1;   	//�������ж�				
	//TIM1->DIER |= 1<<0;   	//��������ж�	
	TIM1->CR1 |= 0x01;    	//ʧ���ܶ�ʱ��1
	MY_NVIC_Init(3, 1, TIM1_CC_IRQn , 3);
}

/**************************************************************************
�������ܣ����������ջز�����
��ڲ�������
����  ֵ����
**************************************************************************/
u16 TIM1CH1_CAPTURE_STA; 
u16 TIM1CH1_CAPTURE_UPVAL;
u16 TIM1CH1_CAPTURE_DOWNVAL;

u32 tempup1 = 0;	//�����ܸߵ�ƽ��ʱ��
u32 Distance;
u32 tim1_T1;
void Read_Distane(void)
{   
	 PAout(2) = 1;
	 delay_us(20);  
	 PAout(2) = 0;			
}

/**************************************************************************
�������ܣ��������ز������ȡ�ж�
��ڲ�������
����  ֵ����
**************************************************************************/
void TIM1_CC_IRQHandler(void)
{ 		    		  			    
	u16 tsr;
	
	tsr = TIM1->SR;
	if ((TIM1CH1_CAPTURE_STA & 0X80) == 0)//��δ�ɹ�����	
	{
		if (tsr & 0x02)//����1���������¼�
	   {	
		    TIM1->SR = 0;//����жϱ�־λ
		   
	        if (TIM1CH1_CAPTURE_STA & 0X40)		//����һ���½��� 		
			{	  			
				//TIM2CH4_CAPTURE_STA |= 0X80;		//��ǳɹ�����һ�θߵ�ƽ����
				TIM1CH1_CAPTURE_DOWNVAL = TIM1->CCR1;	//��ȡ��ǰ�Ĳ���ֵ.
				if (TIM1CH1_CAPTURE_DOWNVAL < TIM1CH1_CAPTURE_UPVAL)
				{/* �����������ʼֵ����ĩβֵ��˵������������� */
					tim1_T1 = 65535;
				}
				else
					tim1_T1 = 0;
				tempup1 = TIM1CH1_CAPTURE_DOWNVAL - TIM1CH1_CAPTURE_UPVAL
						+ tim1_T1;		//�õ��ܵĸߵ�ƽ��ʱ��
				Distance = tempup1 *17/1000;//�������&&UltrasonicWave_Distance<85
			    //printf("%d\r\n", tempup1);
				TIM1CH1_CAPTURE_STA = 0;		//�����־λ���㣬��һ������Ҫ��
				TIM1->CCER &= ~(1<<1);			//CC1P=0 ����Ϊ�����ز���
			}
			else  								//��δ��ʼ,��һ�β���������
			{
				TIM1CH1_CAPTURE_STA = 0;			//���
				TIM1CH1_CAPTURE_UPVAL = TIM1->CCR1;
				TIM1CH1_CAPTURE_STA |= 0X40;		//��ǲ�����������
			    //TIM1->CNT = 0;					//���������
			    TIM1->CCER |= 1<<1; 				//CC1P=1 ����Ϊ�½��ز���
			}		    
	   }			     	    			
   }
}

/**************************************************************************
�������ܣ�CCD�ع�ʱ�����ó���
��ڲ�������
����  ֵ����
**************************************************************************/
//void TIM2_IRQHandler(void)
//{
//	extern u8 IntegrationTime ;                    //�ع�ʱ��
//	static unsigned char TimerCnt20ms = 0;
//    u8 integration_piont;
//	
//	if (TIM2->SR & 0X0001)
//	{
//		TIM2->SR &= ~(1<<0);                      //����жϱ�־λ
//		TimerCnt20ms++;
//		
//        integration_piont = 100 - IntegrationTime; 
//		if(integration_piont >= 2) {      /* �ع�ʱ��С��2�򲻽������ع� */
//			if(integration_piont == TimerCnt20ms)
//			StartIntegration();          ///�ع⿪ʼ
//		}
//	}
//}
