#include "timer.h"
#include "led.h"
//定时发送数据标志位
u8 cnt = 0;
u8 senser_cnt  = 10;
u8 status_cnt  = 15;
u8 send_senser = 0;
u8 send_status = 0;
//u8 rcdata_cnt  = 20;
//u8 motopwm_cnt = 20;
//u8 power_cnt   = 50;
u8 TIME1flag_20ms; 
/**************************************************************************
函数功能：定时中断初始化
入口参数：arr：自动重装值  psc：时钟预分频数 
返回  值：无
**************************************************************************/
void Timer2_Init(u16 arr,u16 psc)  
{  
	RCC->APB1ENR |= 1<<0;     //TIM2时钟使能    
 	TIM2->ARR = arr;      //设定计数器自动重装值   
	TIM2->PSC = psc;      //预分频器7200,得到10Khz的计数时钟
	TIM2->DIER |= 1<<0;   //允许更新中断				
	TIM2->DIER |= 1<<6;   //允许触发中断	   
	TIM2->CR1 |= 0x01;    //使能定时器
	MY_NVIC_Init(2,0,TIM2_IRQn,3);
}  

/**************************************************************************
函数功能：定时器3通道1输入捕获
入口参数：入口参数：arr：自动重装值  psc：时钟预分频数 
返回  值：无
**************************************************************************/
void TIM1_Cap_Init(u16 arr,u16 psc)	
{	 
	RCC->APB2ENR |= 1<<11;   	//TIM1 时钟使能 
	RCC->APB2ENR |= 1<<2;    	//使能PORTA时钟   	 
	GPIOA->CRL &= 0XFFFFF0FF; 
	GPIOA->CRL |= 0X00000200;//PA.2 推挽输出   	PA.8 输入 
	GPIOA->CRH &= 0XFFFFFFF0; 
	GPIOA->CRH |= 0X00000008;//PA.2 推挽输出   	PA.8 输入 
	
  TIM1->ARR = arr;  		//设定计数器自动重装值   
	TIM1->PSC = psc;  		//预分频器 
	TIM1->CCMR1 |= 1<<0;		//CC1S=01 	选择输入端 IC1映射到TI1上
 	TIM1->CCMR1 |= 0<<4; 		//IC1F=0000 配置输入滤波器 不滤波
 	TIM1->CCMR1 |= 0<<10; 	//IC2PS=00 	配置输入分频,不分频 

	TIM1->CCER |= 0<<1; 		//CC1P=0	上升沿捕获
	TIM1->CCER |= 1<<0; 		//CC1E=1 	允许捕获计数器的值到捕获寄存器中

	TIM1->DIER |= 1<<1;   	//允许捕获中断				
	//TIM1->DIER |= 1<<0;   	//允许更新中断	
	TIM1->CR1 |= 0x01;    	//失能能定时器1
	MY_NVIC_Init(3, 1, TIM1_CC_IRQn , 3);
}

/**************************************************************************
函数功能：超声波接收回波函数
入口参数：无
返回  值：无
**************************************************************************/
u16 TIM1CH1_CAPTURE_STA; 
u16 TIM1CH1_CAPTURE_UPVAL;
u16 TIM1CH1_CAPTURE_DOWNVAL;

u32 tempup1 = 0;	//捕获总高电平的时间
u32 Distance;
u32 tim1_T1;
void Read_Distane(void)
{   
	 PAout(2) = 1;
	 delay_us(20);  
	 PAout(2) = 0;			
}

/**************************************************************************
函数功能：超声波回波脉宽读取中断
入口参数：无
返回  值：无
**************************************************************************/
void TIM1_CC_IRQHandler(void)
{ 		    		  			    
	u16 tsr;
	
	tsr = TIM1->SR;
	if ((TIM1CH1_CAPTURE_STA & 0X80) == 0)//还未成功捕获	
	{
		if (tsr & 0x02)//捕获1发生捕获事件
	   {	
		    TIM1->SR = 0;//清除中断标志位
		   
	        if (TIM1CH1_CAPTURE_STA & 0X40)		//捕获到一个下降沿 		
			{	  			
				//TIM2CH4_CAPTURE_STA |= 0X80;		//标记成功捕获到一次高电平脉宽
				TIM1CH1_CAPTURE_DOWNVAL = TIM1->CCR1;	//获取当前的捕获值.
				if (TIM1CH1_CAPTURE_DOWNVAL < TIM1CH1_CAPTURE_UPVAL)
				{/* 如果计数器初始值大于末尾值，说明计数器有溢出 */
					tim1_T1 = 65535;
				}
				else
					tim1_T1 = 0;
				tempup1 = TIM1CH1_CAPTURE_DOWNVAL - TIM1CH1_CAPTURE_UPVAL
						+ tim1_T1;		//得到总的高电平的时间
				Distance = tempup1 *17/1000;//计算距离&&UltrasonicWave_Distance<85
			    //printf("%d\r\n", tempup1);
				TIM1CH1_CAPTURE_STA = 0;		//捕获标志位清零，这一步很重要！
				TIM1->CCER &= ~(1<<1);			//CC1P=0 设置为上升沿捕获
			}
			else  								//还未开始,第一次捕获上升沿
			{
				TIM1CH1_CAPTURE_STA = 0;			//清空
				TIM1CH1_CAPTURE_UPVAL = TIM1->CCR1;
				TIM1CH1_CAPTURE_STA |= 0X40;		//标记捕获到了上升沿
			    //TIM1->CNT = 0;					//计数器清空
			    TIM1->CCER |= 1<<1; 				//CC1P=1 设置为下降沿捕获
			}		    
	   }			     	    			
   }
}

/**************************************************************************
函数功能：CCD曝光时间设置程序
入口参数：无
返回  值：无
**************************************************************************/
//void TIM2_IRQHandler(void)
//{
//	extern u8 IntegrationTime ;                    //曝光时间
//	static unsigned char TimerCnt20ms = 0;
//    u8 integration_piont;
//	
//	if (TIM2->SR & 0X0001)
//	{
//		TIM2->SR &= ~(1<<0);                      //清楚中断标志位
//		TimerCnt20ms++;
//		
//        integration_piont = 100 - IntegrationTime; 
//		if(integration_piont >= 2) {      /* 曝光时间小于2则不进行再曝光 */
//			if(integration_piont == TimerCnt20ms)
//			StartIntegration();          ///曝光开始
//		}
//	}
//}
