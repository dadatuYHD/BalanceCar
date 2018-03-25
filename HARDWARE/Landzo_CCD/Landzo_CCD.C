#include "Landzo_CCD.h"
int Leftedge, Rightedge, error;
int white_flag1, black_flag1;              //ȫ��ȫ�ױ�־λ  
uint8_t Mid_Black;
/*************************************************************************
*  �������ƣ�CCD_init
*  ����˵����CCD��ʼ��
*  ����˵����
*  �������أ���
*  �޸�ʱ�䣺2016-5-12
*  ��    ע��
*************************************************************************/
void LandzoCCD_init(void)
{
    //�ȳ�ʼ��IO��
 	RCC->APB2ENR |= 1<<2;    //ʹ��PORTA��ʱ�� 
	RCC->APB2ENR |= 1<<3;    //ʹ��PORTB��ʱ�� 
	GPIOA->CRL &= 0X0FFFFFFF;//PA7 anolog���� 
	
	GPIOB->CRH &= 0XF0FFFFFF;//PB14 
	GPIOB->CRH |= 0X03000000;//PB14 ������� 50MHZ
    GPIOB->ODR |= 1<<14;	 //����ߵ�ƽ
	GPIOA->CRH &= 0XFFFF0FFF;//PA11 
	GPIOA->CRH |= 0X00003000;//PA11 ������� 50MHZ
    GPIOA->ODR |= 1<<11;	
	//ͨ��10/11����			 
	RCC->APB2ENR |= 1<<9;    //ADC1ʱ��ʹ��	  
	RCC->APB2RSTR |= 1<<9;   //ADC1��λ
	RCC->APB2RSTR &= ~(1<<9);//��λ����	    
	RCC->CFGR &= ~(3<<14);   //��Ƶ��������	
	//SYSCLK/DIV2=12M ADCʱ������Ϊ12M,ADC���ʱ�Ӳ��ܳ���14M!
	//���򽫵���ADC׼ȷ���½�! 
	RCC->CFGR |= 2<<14;      	 
	ADC1->CR1 &= 0XF0FFFF;   //����ģʽ����
	ADC1->CR1 |= 0<<16;      //��������ģʽ  
	ADC1->CR1 &= ~(1<<8);    //��ɨ��ģʽ	  
	ADC1->CR2 &= ~(1<<1);    //����ת��ģʽ
	ADC1->CR2 &= ~(7<<17);	   
	ADC1->CR2 |= 7<<17;	   //��������ת��  
	ADC1->CR2 |= 1<<20;      //ʹ�����ⲿ����(SWSTART)!!!	����ʹ��һ���¼�������
	ADC1->CR2 &= ~(1<<11);   //�Ҷ���	 
	ADC1->SQR1 &= ~(0XF<<20);
	ADC1->SQR1 &= 0<<20;     //1��ת���ڹ��������� Ҳ����ֻת����������1 			   
	//����ͨ��7�Ĳ���ʱ��
	ADC1->SMPR2 &= 0X0FFFFFFF;//ͨ��7����ʱ�����	  
	ADC1->SMPR2 |= 6<<21;      //ͨ��7  239.5����,��߲���ʱ�������߾�ȷ��	 

	ADC1->CR2 |= 1<<0;	      //����ADת����	 
	ADC1->CR2 |= 1<<3;        //ʹ�ܸ�λУ׼  
	while(ADC1->CR2 & 1<<3);  //�ȴ�У׼���� 			 
    //��λ���������ò���Ӳ���������У׼�Ĵ�������ʼ�����λ��������� 		 
	ADC1->CR2 |= 1<<2;        //����ADУ׼	   
	while(ADC1->CR2 & 1<<2);  //�ȴ�У׼����
	delay_ms(1);
}

/*************************************************************************
*  �������ƣ�StartIntegration
*  ����˵����CCD��������
*  ����˵����
*  �������أ���
*  �޸�ʱ�䣺2012-10-20
*  ��    ע��
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
*  �������ƣ�ImageCapture
*  ����˵����CCD��������
*  ����˵����* ImageData   ��������
*  �������أ���
*  �޸�ʱ�䣺2012-10-20
*  ��    ע��
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
	{                    //����250����CCD��ͼ����ȥ�Ƚ�ƽ����
		SamplingDelay();  //200ns                  //�Ѹ�ֵ�Ĵ���߸�С�ﵽ�Լ�����Ľ����
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
*  �������ƣ�CalculateIntegrationTime
*  ����˵���������ع�ʱ��
*  ����˵����
*  �������أ���
*  �޸�ʱ�䣺2012-10-20
*  ��    ע��
*************************************************************************/

/* �ع�ʱ�䣬��λms */
uint8_t IntegrationTime = 10;
void CalculateIntegrationTime(void) {
/* 128�����ص��ƽ��ADֵ */
uint8_t PixelAverageValue;
/* 128�����ص��ƽ����ѹֵ��10�� */
uint8_t PixelAverageVoltage;
/* �趨Ŀ��ƽ����ѹֵ��ʵ�ʵ�ѹ��10�� */
int16_t TargetPixelAverageVoltage = 20;
/* �趨Ŀ��ƽ����ѹֵ��ʵ��ֵ��ƫ�ʵ�ʵ�ѹ��10�� */
int16_t PixelAverageVoltageError = 0;
/* �趨Ŀ��ƽ����ѹֵ������ƫ�ʵ�ʵ�ѹ��10�� */
int16_t TargetPixelAverageVoltageAllowError = 2;

    /* ����128�����ص��ƽ��ADֵ */
    PixelAverageValue = PixelAverage(128,Pixel);
    /* ����128�����ص��ƽ����ѹֵ,ʵ��ֵ��10�� */
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
*  �������ƣ�PixelAverage
*  ����˵����������ľ�ֵ����
*  ����˵����
*  �������أ���
*  �޸�ʱ�䣺2012-10-20
*  ��    ע��
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
*  �������ƣ�SendHex
*  ����˵�����ɼ���������
*  ����˵����
*  �������أ���
*  �޸�ʱ�䣺2012-10-20
*  ��    ע��
*************************************************************************/
void SendHex(unsigned char hex) {
  unsigned char temp;
  temp = hex >> 4;
	
  if(temp < 10) {
    usart_send_char(temp + '0');
	while ((USART1->SR & 0X40) == 0);//�ȴ����ͽ���
	//delay_us(100);
  } else {
   usart_send_char(temp - 10 + 'A');
   while ((USART1->SR & 0X40) == 0);//�ȴ����ͽ���
  // delay_us(100);
  }
  temp = hex & 0x0F;
  if(temp < 10) {
   usart_send_char(temp + '0');
	while ((USART1->SR & 0X40) == 0);//�ȴ����ͽ���
	//delay_us(100);
  } else {
  usart_send_char(temp - 10 + 'A');
   while ((USART1->SR & 0X40) == 0);//�ȴ����ͽ���
  // delay_us(100);
  }
}
/*************************************************************************
*  �������ƣ�SendImageData
*  ����˵����
*  ����˵����
*  �������أ���
*  �޸�ʱ�䣺2012-10-20
*  ��    ע��
*************************************************************************/
void SendImageData(unsigned char * ImageData) {

    unsigned char i;
    unsigned char crc = 0;

    /* Send Data */
    usart_send_char('*');//�򴮿�1��������//uart_putchar(UART0,'*');
    while ((USART1->SR & 0X40) == 0); //�ȴ����ͽ���
	//delay_us(100);
    usart_send_char('L');
	while ((USART1->SR & 0X40) == 0); //�ȴ����ͽ���
	//delay_us(100);
    usart_send_char('D');
	while ((USART1->SR & 0X40) == 0); //�ȴ����ͽ���
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
    while ((USART1->SR & 0X40) == 0);//�ȴ����ͽ���
	//delay_us(100);
}
/*************************************************************************
*  �������ƣ�SamplingDelay
*  ����˵����CCD��ʱ���� 
*  ����˵����
*  �������أ���
*  �޸�ʱ�䣺2012-10-20
*  ��    ע��
*************************************************************************/
void SamplingDelay(void){
   volatile uint8_t i ;
   for(i=0;i<1;i++) {
    __NOP();
    __NOP();}
   
}
/*************************************************************************
*  �������ƣ�GetOSTUThreshold;
*  ����˵������ȡ��̬��ֵ
*  ����˵����128�����ص�
*  �������أ���ֵ
*  �޸�ʱ�䣺2016-5-20
*  ��    ע��
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
		if (cnt == 0) continue;//�Ż�����
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
		if (count >= 118) break;//�Ż����� 122-5+1
	}
	return tr;
}

/*************************************************************************
*  �������ƣ�Black_p_extraction
*  ����˵������ȡ����
*  ����˵����128�����ص�
*  �������أ���
*  �޸�ʱ�䣺2016-5-21
*  ��    ע��
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
    
//	//ȫ��ȫ�ױ������
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
//    if (white_flag1 == 0 && black_flag1==0)   //����ȫ���Ҳ���ȫ�ڵ������
//    {
//		for (i=5; i<122; i++) 
//		{
//			if ((p[i]-p[i+1])==150 && (p[i]-p[i+2])==150)   //�Ӱ׵��ڵ��½���(�����������)
//			{
//				fall_index[fall_num] = i;      //��¼���е��½���λ���±�
//				fall_num++;
//			}
//		}
//	

//	    for (i=122; i>7; i--)
//        {
//			if ((p[i]-p[i-1])==150 && (p[i]-p[i-2])==150)   //�Ӻڵ��׵�������(�����������)
//			{ 
//				rase_index[rase_num] = i;   //��¼���е������ص�λ���±�
//				rase_num++;
//            }
//        }

//		for (i=0; i<fall_num; i++)
//		{
//			for (j=0; j<rase_num; j++)
//			{
//				if ((rase_index[j]-fall_index[i])<=13 && (rase_index[j]-fall_index[i])>=3)    //�ҵ��˵���
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
//	//���ұ߰���
//    Rightedge = 129;
//    looptemp = (PreMidp>80) ? 100 : PreMidp;
//	looptemp2 = (PreMidp-maxfind)>20 ? (PreMidp-maxfind) : 20;
//	for (i=looptemp; i>=looptemp2; i--)
//		if (/*p[i]+4<p[i+1] && p[i+1]+4<p[i+2] && p[i+2]+4<p[i+3] && */Pixelbinarization[i+3]-Pixelbinarization[i]>=150)
//		{
//			Rightedge = i;
//			break;
//		}
//	//����߰���
//	Leftedge = -1;
//	looptemp = (PreMidp<3) ? PreMidp : 3;
//	looptemp2 = (PreMidp+maxfind)<125 ? (PreMidp+maxfind) : 125;
//	for (i=looptemp; i<=looptemp2; i++)
//		if (/*p[i]+4<p[i-1] && p[i-1]+4<p[i-2] && p[i-2]+4<p[i-3] && */Pixelbinarization[i-3]-Pixelbinarization[i]>=150)
//		{
//			Leftedge = i;
//			break;
//		}


		
		//Actual_midp = (Leftedge + Rightedge) / 2;      //��ȡʵ������ֵ
		//error = Actual_midp - Theory_midp;
	//}
	//else error = 0;
    //�´���ȡʱ��ʼ��Ϊ0
	//single_mid = 0, single_mid2 = 0;
	
	 //����ɨ����ȡʵ��������
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
	    //����ɨ����ȡ������
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
*  �������ƣ�binarization
*  ����˵������ֵ��
*  ����˵����128�����ص�
*  �������أ���
*  �޸�ʱ�䣺2016-5-23
*  ��    ע��
*************************************************************************/
void binarization(uint8_t *p)
{
	u8 i;
	
	//��ֵ��
	for (i=0; i<128; i++)
	{
		if (p[i]>threshold) Pixelbinarization[i] = 200;
		else Pixelbinarization[i] = 50;
	}
}