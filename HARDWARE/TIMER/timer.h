#ifndef __TIMER_H
#define __TIMER_H
#include <sys.h>	 

void Timer2_Init(u16 arr,u16 psc);  
void TIM1_Cap_Init(u16 arr,u16 psc)	;
void Read_Distane(void);
void TIM2_IRQHandler(void);
/********���ݷ�����ر���***********/
extern u8 cnt;
extern u8 senser_cnt;
extern u8 status_cnt;
extern u8 send_senser;
extern u8 send_status;
extern u32 Distance;
/**********CCD�ɼ���ض���************/
extern u8 TIME1flag_5ms;
extern u8 TIME1flag_10ms;
extern u8 TIME1flag_15ms;
extern u8 TIME1flag_20ms; 
//extern u8 rcdata_cnt;
//extern u8 motopwm_cnt;
//extern u8 power_cnt; 
#endif
