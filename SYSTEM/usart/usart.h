#ifndef __USART_H
#define __USART_H
#include "sys.h"
#include "stdio.h"	

#define USART_REC_LEN  			50  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����

extern u8  USART_RX_BUF[USART_REC_LEN], send_pid1; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
void uart_init(u32 pclk2,u32 bound);
void Data_Send_Check(u8 head, u8 check_sum);
void Data_Receive_Anl(u8 *data_buf,u8 num);
#endif	   
















