#include "Exit.h"

void EXTI_Init(void)
{
	RCC->APB2ENR |= 1<<2;    //使能PORTA时钟	   	 
	GPIOA->CRL &= 0XFFFF0FFF; 
	GPIOA->CRL |= 0X00008000;//PA3 上拉输入
    GPIOA->ODR |= 1<<3; //PA3 上拉	
	Ex_NVIC_Config(GPIO_A, 3, FTIR);		//下降沿触发
	MY_NVIC_Init(2, 0, EXTI3_IRQn, 3);  	//抢占2，子优先级1，组2
}
