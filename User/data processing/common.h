#ifndef __COMMON_H
#define __COMMON_H

//#include "bitband.h"
#include "stm32f4xx.h"

#define EN_USART_RX         //ʹ�ܴ���1�����ж�
#define EN_USART3_RX
//#define X0  	PAin(2)
//#define Y0  	PBout(12)

//#define LED1		BIT_ADDR(GPIOB_ODR_Addr,0)
//#define LED1_READ	BIT_ADDR(GPIOB_IDR_Addr,0)

//#define LED2		BIT_ADDR(GPIOB_ODR_Addr,1)
//#define LED2_READ	BIT_ADDR(GPIOB_IDR_Addr,1)

#define KEY			BIT_ADDR(GPIOB_IDR_Addr,2)



extern uint8_t USART_RxDat;       //��������
extern uint8_t USART_RxFlag;      //���ձ�־λ
//void Delay_Init(void);       //��ʱ��ʼ������
//void Delay_ms(uint16_t ms);       //��ʱX���뺯��
//void Delay_us(uint32_t us);       //��ʱX΢���
//void USART_Config(USART_TypeDef* USARTx,uint32_t Baud);  //���ڳ�ʼ��
void RTC_Configuration(void);
uint8_t UartSendData(USART_TypeDef* USARTx,uint8_t ch);
void UartSendString(USART_TypeDef* USARTx,char *str);
void Delay(vu32 nCount);

void SysTick_Init(void);
void Delay_us(__IO u32 nTime);
void TimingDelay_Decrement(void);
#define Delay_ms(x) Delay_us(1000*x)	 //��λms

#endif




