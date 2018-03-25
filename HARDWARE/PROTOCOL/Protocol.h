#ifndef __PROTOCOL__
#define __PROTOCOL__
#include "sys.h"
void usart1_send_char(u8 c);
void usart1_niming_report(u8 fun,u8*data,u8 len);
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz);
void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw);
#endif
