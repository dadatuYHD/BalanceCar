#include "Beep.h"
//������IO��ʼ��
void Beep_init(void)
{
	RCC->APB2ENR |= 1 << 3;    //ʹ��PORTBʱ��	   	 
	GPIOB->CRH &= 0X0FFFFFFF; 
	GPIOB->CRH |= 0X30000000;//PB15 �������
    GPIOB->ODR |= 1<<15;	
}
//��Դ�������������α�����
void di(void)
{
	Beep = 0;
	delay_ms(1500);
	Beep = 1;
}
