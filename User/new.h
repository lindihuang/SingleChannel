#ifndef __NEW_H
#define	__NEW_H

#include "stm32f4xx.h"

extern u8  now_step;//µ±«∞≤Ω÷Ë
void STEP_chek(float pro_num);

void Step(void);
void working_action(void);
void STEP1_data(void);
void STEP2_data(void);
void STEP3_data(void);
void STEP4_data(void);
void STEP5_data(void);
void STEP6_data(void);
void STEP7_data(void);
void STEP8_data(void);
void STEP9_data(void);
void STEP10_data(void);
void STEP11_data(void);
void STEP12_data(void);
void Stop_step(void);
void Stop_step_action(float pro_num_action,float detime);

#endif /* __EXTI_H */



