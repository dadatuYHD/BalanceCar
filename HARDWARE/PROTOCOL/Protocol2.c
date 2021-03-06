#include "Protocol2.h"
/**
  * @brief  控制串口发送1个字符
  * @param  c:要发送的字符
  * @retval none
  */
void usart_send_char(u8 c)
{
	while ((USART1->SR & 0X40) == 0);//等待上一次发送完毕   
	USART1->DR = c;   	
} 
/*函数功能：根据匿名最新上位机协议写的显示姿态的程序（上位机0512版本）
 *具体协议说明请查看上位机软件的帮助说明。
 */
void Data_Send_Status(float Pitch,float Roll,float Yaw)
{
	unsigned char i = 0;
	unsigned char _cnt = 0,sum = 0;
	unsigned int _temp;
	u8 data_to_send[50];

	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0x01;
	data_to_send[_cnt++] = 0;
	
	_temp = (int)(Roll*100);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = 0-(int)(Pitch*100);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (int)(Yaw*100);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = 0;
	data_to_send[_cnt++] = BYTE3(_temp);
	data_to_send[_cnt++] = BYTE2(_temp);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	
	data_to_send[_cnt++] = 0xA0;
	
	data_to_send[3] = _cnt-4;
	//和校验
	for(i=0; i<_cnt; i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	//串口发送数据
	for(i=0; i<_cnt; i++)
	{
		usart_send_char(data_to_send[i]);
		delay_ms(20);
	}
}

/*函数功能：根据匿名最新上位机协议写的显示传感器数据（上位机0512版本）
 *具体协议说明请查看上位机软件的帮助说明。
 */
void Send_Data(int16_t *Gyro,int16_t *Accel)
{
	unsigned char i = 0;
	unsigned char _cnt = 0,sum = 0;
	u8 data_to_send[50];

	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0x02;
	data_to_send[_cnt++] = 0;
	
	data_to_send[_cnt++] = BYTE1(Accel[0]);
	data_to_send[_cnt++] = BYTE0(Accel[0]);
	data_to_send[_cnt++] = BYTE1(Accel[1]);
	data_to_send[_cnt++] = BYTE0(Accel[1]);
	data_to_send[_cnt++] = BYTE1(Accel[2]);
	data_to_send[_cnt++] = BYTE0(Accel[2]);
	
	data_to_send[_cnt++] = BYTE1(Gyro[0]);
	data_to_send[_cnt++] = BYTE0(Gyro[0]);
	data_to_send[_cnt++] = BYTE1(Gyro[1]);
	data_to_send[_cnt++] = BYTE0(Gyro[1]);
	data_to_send[_cnt++] = BYTE1(Gyro[2]);
	data_to_send[_cnt++] = BYTE0(Gyro[2]);
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;
	
	data_to_send[3] = _cnt-4;
	//和校验
	for(i=0; i<_cnt; i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	//串口发送数据
	for(i=0; i<_cnt; i++)
	{
		usart_send_char(data_to_send[i]);
		delay_ms(20);
	}
}
void Data_Send_PID1(void)
{
	unsigned char i = 0;
	unsigned char _cnt = 0,sum = 0;
	unsigned int _temp;
	u8 data_to_send[50];
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x12;
	data_to_send[_cnt++]=0;
	
	_temp = (Balance_kp * 10);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (Balance_ki * 10);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (Balance_kd) * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (Velocity_kp) * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (Velocity_ki) * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (Velocity_kd) * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (Turn_kp) * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (Turn_ki) * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (Turn_kd) * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	//和校验
	for (i=0; i<_cnt; i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	//串口发送数据
	for (i=0; i<_cnt; i++)
	{
		usart_send_char(data_to_send[i]);
		delay_ms(20);
	}

}
/*****发送校验数据****/
void Data_Send_Check(u8 head, u8 check_sum)
{
    unsigned char i = 0;
	unsigned char sum = 0;
	u8 data_to_send[50];
	
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	
	data_to_send[5]=check_sum;
	
	for (i=0; i<6; i++)
		sum += data_to_send[i];
	
	data_to_send[6]=sum;

	for (i=0; i<7; i++)
	{
		usart_send_char(data_to_send[i]);
		delay_ms(5);
	}	
}
/*******对上位机发送过来的数据进行解析**********/
void Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0, i;
	
	for (i=0; i<(num-1); i++)
		sum += *(data_buf+i);
	if (!(sum == *(data_buf+num-1)))		return;		//判断sum
	if (!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	
	if (*(data_buf+2) == 0X02)
	{
		if (*(data_buf+4) == 0X01)
		{
			send_pid1 = 1;
		}
	}
	if (*(data_buf+2) == 0X10)								//PID1
	{
		Data_Send_Check(*(data_buf+2),sum);
	}
	if (*(data_buf+2) == 0X11)								//PID2
	{
		Data_Send_Check(*(data_buf+2),sum);
	}
	if (*(data_buf+2) == 0X12)								//PID3
	{
		Balance_kp  = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/10;
		Balance_ki  = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/10;
		Balance_kd  = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/10;
		Velocity_kp = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/10;
		Velocity_ki = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/10;
		Velocity_kd = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/10;
		Turn_kp     = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/10;
		Turn_ki     = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/10;
		Turn_kd     = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/10;
		Data_Send_Check(*(data_buf+2),sum);
	}
	if (*(data_buf+2) == 0X13)								//PID4
	{
		Data_Send_Check(*(data_buf+2),sum);
	}
	if (*(data_buf+2) == 0X14)								//PID5
	{
		Data_Send_Check(*(data_buf+2),sum);
	}
	if (*(data_buf+2) == 0X15)								//PID6
	{
		Data_Send_Check(*(data_buf+2),sum);
	}

}
