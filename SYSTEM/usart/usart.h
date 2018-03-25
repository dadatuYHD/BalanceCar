#ifndef __USART_H
#define __USART_H
#include "sys.h"
#include "stdio.h"	

#define USART_REC_LEN  			50  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收

extern u8  USART_RX_BUF[USART_REC_LEN], send_pid1; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
void uart_init(u32 pclk2,u32 bound);
void Data_Send_Check(u8 head, u8 check_sum);
void Data_Receive_Anl(u8 *data_buf,u8 num);
#endif	   
















