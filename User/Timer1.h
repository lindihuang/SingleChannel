#ifndef __TIM1_H
#define	__TIM1_H

#include "stm32f4xx.h"

//#define ADVANCE_TIM           		TIM1
//#define ADVANCE_TIM_CLK       		RCC_APB2Periph_TIM1

//#define ADVANCE_TIM_IRQn					TIM1_UP_TIM10_IRQn
//#define ADVANCE_TIM_IRQHandler    TIM1_UP_TIM10_IRQHandler

#define GENERAL_TIM           		TIM3
#define GENERAL_TIM_CLK       		RCC_APB1Periph_TIM3

#define GENERAL_TIM_IRQn					TIM3_IRQn
#define GENERAL_TIM_IRQHandler    TIM3_IRQHandler

#define GENERAL_TIM4           		TIM4
#define GENERAL_TIM4_CLK       		RCC_APB1Periph_TIM4

#define GENERAL_TIM4_IRQn					TIM4_IRQn
#define GENERAL_TIM4_IRQHandler    TIM4_IRQHandler



void TIMx_Configuration(void);

void TIM4_Configuration(void);

#endif /* __ADVANCE_TIM_H */




