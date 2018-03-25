#include "usart.h"	  


//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef�� d in stdio.h. */ 
FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	if (Flag_Show == 1)
	{	
	while ((USART3->SR & 0X40) == 0);//ѭ������,ֱ���������   
	USART3->DR = (u8)ch;      
	}
	else
	{	
	while((USART1->SR & 0X40) == 0);//ѭ������,ֱ���������   
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
	
	temp = (float)(pclk2*1000000) / (bound*16);//�õ�USARTDIV
	mantissa = temp;				 //�õ���������
	fraction = (temp - mantissa) * 16; //�õ�С������	 
  mantissa <<= 4;
	mantissa += fraction; 
	RCC->APB2ENR |= 1 << 2;   //ʹ��PORTA��ʱ��  
	RCC->APB2ENR |= 1 << 14;  //ʹ�ܴ���ʱ�� 
	GPIOA->CRH &= 0XFFFFF00F;//IO״̬����
	GPIOA->CRH |= 0X000008B0;//IO״̬���� 
	RCC->APB2RSTR |= 1 << 14;   //��λ����1
	RCC->APB2RSTR &= ~(1 << 14);//ֹͣ��λ	   	   
	//����������
 	USART1->BRR = mantissa; // ����������	 
	USART1->CR1 |= 0X200C;  //1λֹͣ,��У��λ.
#if EN_USART1_RX		  //���ʹ���˽���
	//ʹ�ܽ����ж� 
	//USART1->CR1|=1<<8;    //PE�ж�ʹ��
	USART1->CR1|=1<<5;    //���ջ������ǿ��ж�ʹ��	
	MY_NVIC_Init(0,0,USART1_IRQn,3);//��3��������ȼ� 
//	//ʹ�ܷ����ж�
//	USART1->CR1|=1<<6;  
//    USART1->SR&=(~(1<<6));//����жϱ�־λ
#endif
}

#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
u8 RxState = 0, com_data;
u8 _data_cnt = 0, _data_len, send_pid1;
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ  
  
void USART1_IRQHandler(void)
{
	if(USART1->SR & USART_SR_ORE)//ORE�ж�
	{
		com_data = USART1->DR;
	}
	if (USART1->SR&(1<<5))	//���յ�����
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
