#ifndef __PROTOCOL2__
#define __PROTOCOL2__
#include "sys.h"
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
	
void usart_send_char(uint8_t c);
void Data_Send_Status(float Pitch,float Roll,float Yaw);
void Send_Data(int16_t *Gyro,int16_t *Accel);
void Data_Send_PID1(void);
#endif
