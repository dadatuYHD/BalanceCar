#include "usart.h"	  


//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	if (Flag_Show == 1)
	{	
	while ((USART3->SR & 0X40) == 0);//循环发送,直到发送完毕   
	USART3->DR = (u8)ch;      
	}
	else
	{	
	while((USART1->SR & 0X40) == 0);//循环发送,直到发送完毕   
	USART1->DR = (u8)ch;      
	}	
	return ch;
}
#endif 
//end
//////////////////////////////////////////////////////////////////
void uart_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	
	
	temp = (float)(pclk2*1000000) / (bound*16);//得到USARTDIV
	mantissa = temp;				 //得到整数部分
	fraction = (temp - mantissa) * 16; //得到小数部分	 
  mantissa <<= 4;
	mantissa += fraction; 
	RCC->APB2ENR |= 1 << 2;   //使能PORTA口时钟  
	RCC->APB2ENR |= 1 << 14;  //使能串口时钟 
	GPIOA->CRH &= 0XFFFFF00F;//IO状态设置
	GPIOA->CRH |= 0X000008B0;//IO状态设置 
	RCC->APB2RSTR |= 1 << 14;   //复位串口1
	RCC->APB2RSTR &= ~(1 << 14);//停止复位	   	   
	//波特率设置
 	USART1->BRR = mantissa; // 波特率设置	 
	USART1->CR1 |= 0X200C;  //1位停止,无校验位.
#if EN_USART1_RX		  //如果使能了接收
	//使能接收中断 
	//USART1->CR1|=1<<8;    //PE中断使能
	USART1->CR1|=1<<5;    //接收缓冲区非空中断使能	
	MY_NVIC_Init(0,0,USART1_IRQn,3);//组3，最低优先级 
//	//使能发送中断
//	USART1->CR1|=1<<6;  
//    USART1->SR&=(~(1<<6));//清除中断标志位
#endif
}

#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
u8 RxState = 0, com_data;
u8 _data_cnt = 0, _data_len, send_pid1;
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目  
  
void USART1_IRQHandler(void)
{
	if(USART1->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART1->DR;
	}
	if (USART1->SR&(1<<5))	//接收到数据
	{	 
		com_data = USART1->DR;
		if (RxState==0 && com_data==0xAA)
		{
			RxState=1;
			USART_RX_BUF[0] = com_data;
		}
		else if (RxState==1 && com_data == 0xAF)
		{
			RxState=2;
			USART_RX_BUF[1] = com_data;
		}
		else if (RxState==2 && com_data<0XF1)
		{
			RxState=3;
			USART_RX_BUF[2]=com_data;
		}
		else if (RxState==3&&com_data<50)
		{
			RxState = 4;
			USART_RX_BUF[3]=com_data;
			_data_len = com_data;
			_data_cnt = 0;
		}
		else if (RxState==4&&_data_len>0)
		{
			_data_len--;
			USART_RX_BUF[4+_data_cnt++] = com_data;
			if(_data_len == 0)
				RxState = 5;
		}
		else if (RxState==5)
		{
			RxState = 0;
			USART_RX_BUF[4+_data_cnt] = com_data;
			Data_Receive_Anl(USART_RX_BUF,_data_cnt+5);
		}
		else
			RxState = 0;
	}		
} 
#endif										 
