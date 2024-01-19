#ifndef __DATA_PROCESSING_H
#define	__DATA_PROCESSING_H
#include "stm32f4xx.h"

#include "stm32f4xx.h"
#include "ads1256.h"
#include "./usart/bsp_debug_usart.h"

#define enable          	1
#define disable         	0
#define storage_number    12

extern int32_t deviation[4];   //调零偏移量
extern const 	int32_t init_value[4];
extern int64_t channel[4][storage_number]; //将读取的部分ad值存储在这个数组中

void set_zero(void);
void Setzero_Init(void);
void Delays1(uint32_t x);
void relay17_Config(void);
void Acquisition_channel(char channel_0,char channel_1,char channel_2,char channel_3);
void Acquisition_channel1(char channel_0,char channel_1,char channel_2,char channel_3);

long double smoothing(int64_t weilvadzhi,uint8_t channels);




#endif /* __LED_H */




