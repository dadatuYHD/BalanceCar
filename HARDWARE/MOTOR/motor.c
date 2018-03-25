#include "motor.h"
void MiniBalance_Motor_Init(void)
{
	RCC->APB2ENR |= 1<<3;       //PORTBʱ��ʹ�� 
    RCC->APB2ENR |= 1<<2;       //PORTAʱ��ʹ�� 
	GPIOB->CRL &= 0XFF000FFF;   //PORTB3 4 5�������
	GPIOB->CRL |= 0X00222000;   //PORTB3 4 5�������
	GPIOA->CRH &= 0X0FFFFFFF;   //PORTA15�������
	GPIOA->CRH |= 0X20000000;   //PORTA15�������
}
void MiniBalance_PWM_Init(u16 arr,u16 psc)
{		 					 
	MiniBalance_Motor_Init(); //��ʼ�������������IO
	RCC->APB1ENR |= 1<<1;       //TIM3ʱ��ʹ��    
	RCC->APB2ENR |= 1<<3;       //PORTBʱ��ʹ��     
	GPIOB->CRL &= 0XFFFFFF00;   //PORTB0 1�������
	GPIOB->CRL |= 0X000000BB;   //PORTB0 1�������
	TIM3->ARR = arr;//�趨�������Զ���װֵ 
	TIM3->PSC = psc;//Ԥ��Ƶ������Ƶ
	TIM3->CCMR2 |= 6<<12;//CH4 PWM1ģʽ	
	TIM3->CCMR2 |= 6<<4; //CH3 PWM1ģʽ	
	TIM3->CCMR2 |= 1<<11;//CH4Ԥװ��ʹ��	 
	TIM3->CCMR2 |= 1<<3; //CH3Ԥװ��ʹ��	  
	TIM3->CCER |= 1<<12; //CH4���ʹ��	   
	TIM3->CCER |= 1<<8;  //CH3���ʹ��	
	TIM3->CR1 = 0x8000;  //ARPEʹ�� 
	TIM3->CR1 |= 0x01;   //ʹ�ܶ�ʱ��3 										  
} 

