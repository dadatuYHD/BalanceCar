#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
#define PI 3.14159265
extern	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
extern float Balance_kp, Balance_ki, Balance_kd;  
extern float Velocity_kp, Velocity_ki, Velocity_kd;  
extern float Turn_kp, Turn_ki, Turn_kd;
extern int ultrasoniccount;
extern u8 send_data_cnt;
extern u8 threshold;                          //动态阈值
extern uint8_t Theory_midline;               //理论上的中线坐标
extern uint8_t Actual_midline;                    //实际上的中线坐标
extern int ready;
int TIM1_UP_IRQHandler(void);  
int balance(float angle,float gyro);
int velocity(int encoder_left,int encoder_right);
int turn(int encoder_left,int encoder_right,float gyro);
void Set_Pwm(int moto1,int moto2);
void Key(void);
void Xianfu_Pwm(void);
u8 Turn_Off(float angle);
void Get_Angle(u8 way);
int myabs(int a);
int Incremental_PI (int Encoder,int Target);
#endif
