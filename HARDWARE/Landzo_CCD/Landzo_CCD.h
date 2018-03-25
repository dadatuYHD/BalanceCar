#ifndef _CCD_H_
#define _CCD_H_
#include "sys.h"

#define TSL_SI   PBout(14)   //SI
#define TSL_CLK   PAout(11)   //CLK 
/***********************相关函数声明*************************/
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
/****************************相关变量声明*******************/
extern uint8_t Theory_midline;                    //理论中线坐标声明
extern uint8_t Actual_midline;                    //实际中线坐标的声明
extern u8 threshold;                              //动态阈值的声明
extern int Leftedge, Rightedge, error;
extern int white_flag1, black_flag1;              //全黑全白标志位 
extern uint8_t Mid_Black;                         //中心黑线提取成功标志位
#endif 
