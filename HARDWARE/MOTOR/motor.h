#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	 

#define PWMA   TIM3->CCR4
#define AIN2   PAout(15)
#define AIN1   PBout(3)
#define BIN1   PBout(5)
#define BIN2   PBout(4)
#define PWMB   TIM3->CCR3
void MiniBalance_PWM_Init(u16 arr,u16 psc);
void MiniBalance_Motor_Init(void);
#endif
