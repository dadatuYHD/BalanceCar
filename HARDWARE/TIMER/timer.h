#ifndef __TIMER_H
#define __TIMER_H
#include <sys.h>	 

void Timer2_Init(u16 arr,u16 psc);  
void TIM1_Cap_Init(u16 arr,u16 psc)	;
void Read_Distane(void);
void TIM2_IRQHandler(void);
/********数据发送相关变量***********/
extern u8 cnt;
extern u8 senser_cnt;
extern u8 status_cnt;
extern u8 send_senser;
extern u8 send_status;
extern u32 Distance;
/**********CCD采集相关定义************/
extern u8 TIME1flag_5ms;
extern u8 TIME1flag_10ms;
extern u8 TIME1flag_15ms;
extern u8 TIME1flag_20ms; 
//extern u8 rcdata_cnt;
//extern u8 motopwm_cnt;
//extern u8 power_cnt; 
#endif
