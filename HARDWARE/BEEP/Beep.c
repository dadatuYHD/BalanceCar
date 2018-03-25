#include "Beep.h"
//蜂鸣器IO初始化
void Beep_init(void)
{
	RCC->APB2ENR |= 1 << 3;    //使能PORTB时钟	   	 
	GPIOB->CRH &= 0X0FFFFFFF; 
	GPIOB->CRH |= 0X30000000;//PB15 推免输出
    GPIOB->ODR |= 1<<15;	
}
//电源开启蜂鸣器“滴报警”
void di(void)
{
	Beep = 0;
	delay_ms(1500);
	Beep = 1;
}
