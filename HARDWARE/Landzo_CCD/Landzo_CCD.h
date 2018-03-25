#ifndef _CCD_H_
#define _CCD_H_
#include "sys.h"

#define TSL_SI   PBout(14)   //SI
#define TSL_CLK   PAout(11)   //CLK 
/***********************��غ�������*************************/
void StartIntegration(void);   
void ImageCapture(unsigned char * ImageData);
void SendHex(unsigned char hex);
void SamplingDelay(void);
void LandzoCCD_init(void);
void CalculateIntegrationTime(void);
uint8_t PixelAverage(uint8_t len, uint8_t *data);
void SendImageData(unsigned char * ImageData);
u8 GetOSTUThreshold(u8 *p);
void Black_line_extraction(u8 *p);
void binarization(uint8_t *p);
extern uint8_t Pixel[128];
extern uint8_t Pixelbinarization[128];
/****************************��ر�������*******************/
extern uint8_t Theory_midline;                    //����������������
extern uint8_t Actual_midline;                    //ʵ���������������
extern u8 threshold;                              //��̬��ֵ������
extern int Leftedge, Rightedge, error;
extern int white_flag1, black_flag1;              //ȫ��ȫ�ױ�־λ 
extern uint8_t Mid_Black;                         //���ĺ�����ȡ�ɹ���־λ
#endif 
