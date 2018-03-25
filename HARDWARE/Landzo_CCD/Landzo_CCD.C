#include "Landzo_CCD.h"
int Leftedge, Rightedge, error;
int white_flag1, black_flag1;              //全黑全白标志位  
uint8_t Mid_Black;
/*************************************************************************
*  函数名称：CCD_init
*  功能说明：CCD初始化
*  参数说明：
*  函数返回：无
*  修改时间：2016-5-12
*  备    注：
*************************************************************************/
void LandzoCCD_init(void)
{
    //先初始化IO口
 	RCC->APB2ENR |= 1<<2;    //使能PORTA口时钟 
	RCC->APB2ENR |= 1<<3;    //使能PORTB口时钟 
	GPIOA->CRL &= 0X0FFFFFFF;//PA7 anolog输入 
	
	GPIOB->CRH &= 0XF0FFFFFF;//PB14 
	GPIOB->CRH |= 0X03000000;//PB14 推挽输出 50MHZ
    GPIOB->ODR |= 1<<14;	 //输出高电平
	GPIOA->CRH &= 0XFFFF0FFF;//PA11 
	GPIOA->CRH |= 0X00003000;//PA11 推挽输出 50MHZ
    GPIOA->ODR |= 1<<11;	
	//通道10/11设置			 
	RCC->APB2ENR |= 1<<9;    //ADC1时钟使能	  
	RCC->APB2RSTR |= 1<<9;   //ADC1复位
	RCC->APB2RSTR &= ~(1<<9);//复位结束	    
	RCC->CFGR &= ~(3<<14);   //分频因子清零	
	//SYSCLK/DIV2=12M ADC时钟设置为12M,ADC最大时钟不能超过14M!
	//否则将导致ADC准确度下降! 
	RCC->CFGR |= 2<<14;      	 
	ADC1->CR1 &= 0XF0FFFF;   //工作模式清零
	ADC1->CR1 |= 0<<16;      //独立工作模式  
	ADC1->CR1 &= ~(1<<8);    //非扫描模式	  
	ADC1->CR2 &= ~(1<<1);    //单次转换模式
	ADC1->CR2 &= ~(7<<17);	   
	ADC1->CR2 |= 7<<17;	   //软件控制转换  
	ADC1->CR2 |= 1<<20;      //使用用外部触发(SWSTART)!!!	必须使用一个事件来触发
	ADC1->CR2 &= ~(1<<11);   //右对齐	 
	ADC1->SQR1 &= ~(0XF<<20);
	ADC1->SQR1 &= 0<<20;     //1个转换在规则序列中 也就是只转换规则序列1 			   
	//设置通道7的采样时间
	ADC1->SMPR2 &= 0X0FFFFFFF;//通道7采样时间清空	  
	ADC1->SMPR2 |= 6<<21;      //通道7  239.5周期,提高采样时间可以提高精确度	 

	ADC1->CR2 |= 1<<0;	      //开启AD转换器	 
	ADC1->CR2 |= 1<<3;        //使能复位校准  
	while(ADC1->CR2 & 1<<3);  //等待校准结束 			 
    //该位由软件设置并由硬件清除。在校准寄存器被初始化后该位将被清除。 		 
	ADC1->CR2 |= 1<<2;        //开启AD校准	   
	while(ADC1->CR2 & 1<<2);  //等待校准结束
	delay_ms(1);
}

/*************************************************************************
*  函数名称：StartIntegration
*  功能说明：CCD启动程序
*  参数说明：
*  函数返回：无
*  修改时间：2012-10-20
*  备    注：
*************************************************************************/
void StartIntegration(void) {

    unsigned char i;

    TSL_SI = 1;            /* SI  = 1 */
    SamplingDelay();
    TSL_CLK = 1;           /* CLK = 1 */
    SamplingDelay();
    TSL_SI = 0;            /* SI  = 0 */
    SamplingDelay();
    TSL_CLK = 0;           /* CLK = 0 */

    for(i=0; i<127; i++) {
        SamplingDelay();
        SamplingDelay();
        TSL_CLK = 1;       /* CLK = 1 */
        SamplingDelay();
        SamplingDelay();
       TSL_CLK = 0;       /* CLK = 0 */
    }
    SamplingDelay();
    SamplingDelay();
    TSL_CLK = 1;           /* CLK = 1 */
    SamplingDelay();
    SamplingDelay();
    TSL_CLK = 0;           /* CLK = 0 */
}

/*************************************************************************
*  函数名称：ImageCapture
*  功能说明：CCD采样程序
*  参数说明：* ImageData   采样数组
*  函数返回：无
*  修改时间：2012-10-20
*  备    注：
*ImageData =  ad_once(ADC1, AD6a, ADC_8bit);
*************************************************************************/

void ImageCapture(uint8_t * ImageData) 
{
    uint8_t i;
    extern uint8_t AtemP ;

    TSL_SI = 1;            /* SI  = 1 */
    delay_us(2);
    TSL_CLK = 1;           /* CLK = 1 */
	  delay_us(2);
    TSL_SI = 0;            /* SI  = 0 */
    delay_us(5);
//Delay 10us for sample the first pixel
    for(i = 0; i < 200; i++) 
	{                    //更改250，让CCD的图像看上去比较平滑，
		SamplingDelay();  //200ns                  //把该值改大或者改小达到自己满意的结果。
    }
//Sampling Pixel 1

    *ImageData =  Get_Adc(7)>>4;
    ImageData ++ ;
    TSL_CLK = 0;           /* CLK = 0 */

    for(i=0; i<127; i++) {
        delay_us(1);
        TSL_CLK = 1;       /* CLK = 1 */
        delay_us(1);
        //Sampling Pixel 2~128
       *ImageData =  Get_Adc(7)>>4;	 
        ImageData ++ ;
       TSL_CLK = 0;       /* CLK = 0 */
    }
    delay_us(5);
    TSL_CLK = 1;           /* CLK = 1 */
	  delay_us(5);
    TSL_CLK = 0;           /* CLK = 0 */
}


/*************************************************************************
*  函数名称：CalculateIntegrationTime
*  功能说明：计算曝光时间
*  参数说明：
*  函数返回：无
*  修改时间：2012-10-20
*  备    注：
*************************************************************************/

/* 曝光时间，单位ms */
uint8_t IntegrationTime = 10;
void CalculateIntegrationTime(void) {
/* 128个像素点的平均AD值 */
uint8_t PixelAverageValue;
/* 128个像素点的平均电压值的10倍 */
uint8_t PixelAverageVoltage;
/* 设定目标平均电压值，实际电压的10倍 */
int16_t TargetPixelAverageVoltage = 20;
/* 设定目标平均电压值与实际值的偏差，实际电压的10倍 */
int16_t PixelAverageVoltageError = 0;
/* 设定目标平均电压值允许的偏差，实际电压的10倍 */
int16_t TargetPixelAverageVoltageAllowError = 2;

    /* 计算128个像素点的平均AD值 */
    PixelAverageValue = PixelAverage(128,Pixel);
    /* 计算128个像素点的平均电压值,实际值的10倍 */
    PixelAverageVoltage = (uint8_t)((int16_t)PixelAverageValue * 25 / 194);

    PixelAverageVoltageError = TargetPixelAverageVoltage - PixelAverageVoltage;
    if(PixelAverageVoltageError < -TargetPixelAverageVoltageAllowError)
    {
      PixelAverageVoltageError = 0- PixelAverageVoltageError ;
      PixelAverageVoltageError /= 5;
      if(PixelAverageVoltageError > 10 )
         PixelAverageVoltageError = 10 ;
       IntegrationTime -= PixelAverageVoltageError;
    }
    if(PixelAverageVoltageError > TargetPixelAverageVoltageAllowError)
    { 
        PixelAverageVoltageError /= 5;
        if(PixelAverageVoltageError > 10 )
           PixelAverageVoltageError = 10 ;
        IntegrationTime += PixelAverageVoltageError;}
 
    if(IntegrationTime <= 1)
        IntegrationTime = 1;
    if(IntegrationTime >= 100)
        IntegrationTime = 100;
}

/*************************************************************************
*  函数名称：PixelAverage
*  功能说明：求数组的均值程序
*  参数说明：
*  函数返回：无
*  修改时间：2012-10-20
*  备    注：
*************************************************************************/
uint8_t PixelAverage(uint8_t len, uint8_t *data) {
  unsigned char i;
  unsigned int sum = 0;
  for(i = 0; i<len; i++) {
    sum = sum + *data++;
  }
  return ((unsigned char)(sum/len));
}
/*************************************************************************
*  函数名称：SendHex
*  功能说明：采集发数程序
*  参数说明：
*  函数返回：无
*  修改时间：2012-10-20
*  备    注：
*************************************************************************/
void SendHex(unsigned char hex) {
  unsigned char temp;
  temp = hex >> 4;
	
  if(temp < 10) {
    usart_send_char(temp + '0');
	while ((USART1->SR & 0X40) == 0);//等待发送结束
	//delay_us(100);
  } else {
   usart_send_char(temp - 10 + 'A');
   while ((USART1->SR & 0X40) == 0);//等待发送结束
  // delay_us(100);
  }
  temp = hex & 0x0F;
  if(temp < 10) {
   usart_send_char(temp + '0');
	while ((USART1->SR & 0X40) == 0);//等待发送结束
	//delay_us(100);
  } else {
  usart_send_char(temp - 10 + 'A');
   while ((USART1->SR & 0X40) == 0);//等待发送结束
  // delay_us(100);
  }
}
/*************************************************************************
*  函数名称：SendImageData
*  功能说明：
*  参数说明：
*  函数返回：无
*  修改时间：2012-10-20
*  备    注：
*************************************************************************/
void SendImageData(unsigned char * ImageData) {

    unsigned char i;
    unsigned char crc = 0;

    /* Send Data */
    usart_send_char('*');//向串口1发送数据//uart_putchar(UART0,'*');
    while ((USART1->SR & 0X40) == 0); //等待发送结束
	//delay_us(100);
    usart_send_char('L');
	while ((USART1->SR & 0X40) == 0); //等待发送结束
	//delay_us(100);
    usart_send_char('D');
	while ((USART1->SR & 0X40) == 0); //等待发送结束
	//delay_us(100);
    SendHex(0);
    SendHex(0);
    SendHex(0);
    SendHex(0);

    for(i=0; i<128; i++) {
      SendHex(*ImageData++);
    }

    SendHex(crc);
    usart_send_char('#');
    while ((USART1->SR & 0X40) == 0);//等待发送结束
	//delay_us(100);
}
/*************************************************************************
*  函数名称：SamplingDelay
*  功能说明：CCD延时程序 
*  参数说明：
*  函数返回：无
*  修改时间：2012-10-20
*  备    注：
*************************************************************************/
void SamplingDelay(void){
   volatile uint8_t i ;
   for(i=0;i<1;i++) {
    __NOP();
    __NOP();}
   
}
/*************************************************************************
*  函数名称：GetOSTUThreshold;
*  功能说明：获取动态阈值
*  参数说明：128个像素点
*  函数返回：阈值
*  修改时间：2016-5-20
*  备    注：
*************************************************************************/
u8 GetOSTUThreshold(u8 *p)
{
	u32 g, max = 0;
    u16 total = 0, total_low = 0;
    u8 u0 = 0, u1 = 0, count = 0, tr = 0, cnt = 0;
	u8 pc[256] = {0};
	u8 j;
	
	for (j=5; j<=122; j++)
	{
		pc[*(p+j)]++;
		total += *(p+j);
	}
	for (j=0; j<=254; j++)
	{
		cnt = pc[j];
		if (cnt == 0) continue;//优化加速
		count += pc[j];
		total_low += cnt * j;
		u0 = total_low / count;
		u1 = (total - total_low) / (118 - count);
		g = ((u32)(u0 - u1) * (u0 - u1)) * ((count * (118 - count))) / 16384;
		if (g > max)
		{
			max = g;
			tr = j;
		}
		if (count >= 118) break;//优化加速 122-5+1
	}
	return tr;
}

/*************************************************************************
*  函数名称：Black_p_extraction
*  功能说明：提取黑线
*  参数说明：128个像素点
*  函数返回：无
*  修改时间：2016-5-21
*  备    注：
*************************************************************************/
void Black_line_extraction(u8 *p)
{
////	u8 looptemp, looptemp2;
////	u8 PreMidp = 128, maxfind = 128;
//	int fall_num = 0, rase_num = 0;
//    int fall_index[128] ={0},rase_index[128] ={0}; 
	int black_num, white_num;
	uint8_t LxQ5 = 0, RxQ5 = 0, LxQ2 = 0, RxQ2 = 0;
	uint8_t mid_flag1 = 0, mid_flag2 = 0; 
	uint8_t mid_flag3 = 0,mid_flag4 = 0;
	uint8_t i = 0, Lc = 0, Lc2 = 0;
	uint8_t mid_left = 0, mid_right = 0, mid_left2 = 0, mid_right2 = 0; 
	uint8_t single_mid = 0, single_mid2 = 0;
    
//	//全黑全白背景检测
//	for (i=0; i<128; i++)
//	{
//		if (p[i] <= threshold-30) black_num++;
//		else black_num = 0;
//		
//		if (p[i] >= threshold-30) white_num++;
//		else white_num = 0;
//		
//		if (black_num >= 100) black_flag1 = 1;
//		else black_flag1 = 0;
//		
//		if (white_num >= 100) white_flag1 = 1;
//		else white_flag1 = 0;
//	}
//	
//    if (white_flag1 == 0 && black_flag1==0)   //不是全白且不是全黑的情况下
//    {
//		for (i=5; i<122; i++) 
//		{
//			if ((p[i]-p[i+1])==150 && (p[i]-p[i+2])==150)   //从白到黑的下降沿(方向从左往右)
//			{
//				fall_index[fall_num] = i;      //记录所有的下降沿位置下标
//				fall_num++;
//			}
//		}
//	

//	    for (i=122; i>7; i--)
//        {
//			if ((p[i]-p[i-1])==150 && (p[i]-p[i-2])==150)   //从黑到白的上升沿(方向从左往右)
//			{ 
//				rase_index[rase_num] = i;   //记录所有的上升沿的位置下标
//				rase_num++;
//            }
//        }

//		for (i=0; i<fall_num; i++)
//		{
//			for (j=0; j<rase_num; j++)
//			{
//				if ((rase_index[j]-fall_index[i])<=13 && (rase_index[j]-fall_index[i])>=3)    //找到了单线
//				{
//					Actual_midp = (int)((fall_index[i] + rase_index[j])/2);
//              //Oneflag = 1;
//					break;
//				}
//				else
//				{
//             //Oneflag = 0;
//				}
//			} 
//		}
//	//找右边凹槽
//    Rightedge = 129;
//    looptemp = (PreMidp>80) ? 100 : PreMidp;
//	looptemp2 = (PreMidp-maxfind)>20 ? (PreMidp-maxfind) : 20;
//	for (i=looptemp; i>=looptemp2; i--)
//		if (/*p[i]+4<p[i+1] && p[i+1]+4<p[i+2] && p[i+2]+4<p[i+3] && */Pixelbinarization[i+3]-Pixelbinarization[i]>=150)
//		{
//			Rightedge = i;
//			break;
//		}
//	//找左边凹槽
//	Leftedge = -1;
//	looptemp = (PreMidp<3) ? PreMidp : 3;
//	looptemp2 = (PreMidp+maxfind)<125 ? (PreMidp+maxfind) : 125;
//	for (i=looptemp; i<=looptemp2; i++)
//		if (/*p[i]+4<p[i-1] && p[i-1]+4<p[i-2] && p[i-2]+4<p[i-3] && */Pixelbinarization[i-3]-Pixelbinarization[i]>=150)
//		{
//			Leftedge = i;
//			break;
//		}


		
		//Actual_midp = (Leftedge + Rightedge) / 2;      //提取实际中线值
		//error = Actual_midp - Theory_midp;
	//}
	//else error = 0;
    //下次提取时初始化为0
	//single_mid = 0, single_mid2 = 0;
	
	 //往右扫描提取实际中心线
	//if (white_flag1==0 && black_flag1==0)
	//{
		for (i=6 ; i<121; i++)
		{
			if ((*(p + i)-*(p + i + 4)) >= (threshold-20))
			{
				mid_flag1++;
				if (mid_flag1 >= 2) 
				{
					mid_left = i;
					LxQ2 = 1;
					//printf("LxQ2 is %d\r\n", LxQ2 );
					for (i=mid_left; i<121; i++) 
					{
						if ((*(p+i+4)- *(p+i)) >= (threshold-20))
						{
							mid_flag2++ ;
							if (mid_flag2 >= 2) 
							{
								mid_right = i;
								Lc = mid_right - mid_left;
								if (Lc>=2 && Lc<=12) 
								{
									single_mid = (mid_right + mid_left ) >> 1;
									RxQ2 = 1;
									//printf("RxQ2 is %d\r\n", RxQ2 );
								}
								break;
							}
						}
						else 
						{
							mid_flag2 = 0;
						}
					}
					break;
				}
			}
			else
			{
				mid_flag1 = 0;
			}
		}
	    //提左扫描提取中心线
		for (i=121; i>6; i--)
		{
			if ((*(p+i)-*(p+i-4)) >= (threshold-20))
			{
				mid_flag3++;
				if (mid_flag3 >= 2)
				{
					mid_left2 = i;
					LxQ5 = 1;
					//printf("LxQ5 is %d\r\n", LxQ5 );
					for (i=mid_left2; i>6; i--) 
					{
						if ((*(p+i-4)-*(p+i)) >= (threshold-20))
						{
							mid_flag4++;
							if (mid_flag4 >= 2)
							{
								mid_right2 = i;
								Lc2 = mid_left2 - mid_right2;
								if (Lc2>=2 && Lc2<=12) 
								{
									single_mid2 = (mid_right2 + mid_left2 ) >> 1;
									RxQ5 = 1;
									//printf("RxQ5 is %d\r\n", RxQ5);
								}
								break;
							}
						}
						else 
						{
							mid_flag4 = 0;
						}
				   }
					 break;
			  }	
		   }		
		   else
		  {
			mid_flag3 = 0;
		  }
	   }
	  //printf("the single_mid is%d the single_mid2 is %d\r\n", single_mid, single_mid2);
	  if (LxQ2!=0 && RxQ2!=0 && (myabs(single_mid2-single_mid)<=5) && LxQ5!=0 && RxQ5!=0) 
	  {
		 LxQ5 = 0, RxQ5 = 0, LxQ2 = 0, RxQ2 = 0;
		 Actual_midline = single_mid; 
		 error = Actual_midline - Theory_midline;
		 Mid_Black = 1;
		 //printf("the single_mid is%d\r\n", single_mid);
      }             
	  else 
      {
		 Mid_Black = 0;
	  }
	//}
	//else
	//{
//		error = 0;
//		printf("the error is %d\r\n", error);
////		 Actual_midline = single_mid; 
////		 error = Actual_midline - Theory_midline;
//	}
   
	
}
   
/*************************************************************************
*  函数名称：binarization
*  功能说明：二值化
*  参数说明：128个像素点
*  函数返回：无
*  修改时间：2016-5-23
*  备    注：
*************************************************************************/
void binarization(uint8_t *p)
{
	u8 i;
	
	//二值化
	for (i=0; i<128; i++)
	{
		if (p[i]>threshold) Pixelbinarization[i] = 200;
		else Pixelbinarization[i] = 50;
	}
}
