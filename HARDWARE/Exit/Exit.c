#include "Exit.h"

void EXTI_Init(void)
{
	RCC->APB2ENR |= 1<<2;    //ʹ��PORTAʱ��	   	 
	GPIOA->CRL &= 0XFFFF0FFF; 
	GPIOA->CRL |= 0X00008000;//PA3 ��������
    GPIOA->ODR |= 1<<3; //PA3 ����	
	Ex_NVIC_Config(GPIO_A, 3, FTIR);		//�½��ش���
	MY_NVIC_Init(2, 0, EXTI3_IRQn, 3);  	//��ռ2�������ȼ�1����2
}
