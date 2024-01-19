#ifndef __RELAY_H
#define	__RELAY_H

#include "stm32f4xx.h"



/* 直接操作寄存器的方法控制IO */
#define	digitalHi(p,i)			 {p->BSRRL=i;}		//设置为高电平
#define digitalLo(p,i)			 {p->BSRRH=i;}		//输出低电平
#define digitalToggle(p,i)	 {p->ODR ^=i;}		//输出反转状态

/* 定义控制IO的宏 */
#define relay16_TOGGLE		digitalToggle(GPIOG,GPIO_Pin_11)
#define relay16_OFF				digitalHi(GPIOG,GPIO_Pin_11)
#define relay16_ON				digitalLo(GPIOG,GPIO_Pin_11)

#define relay15_TOGGLE		digitalToggle(GPIOA,GPIO_Pin_3)
#define relay15_OFF				digitalHi(GPIOA,GPIO_Pin_3)
#define relay15_ON				digitalLo(GPIOA,GPIO_Pin_3)

#define relay14_TOGGLE		digitalToggle(GPIOI,GPIO_Pin_2)
#define relay14_OFF				digitalHi(GPIOI,GPIO_Pin_2)
#define relay14_ON				digitalLo(GPIOI,GPIO_Pin_2)

#define relay13_TOGGLE		digitalToggle(GPIOE,GPIO_Pin_4)
#define relay13_OFF				digitalHi(GPIOE,GPIO_Pin_4)
#define relay13_ON				digitalLo(GPIOE,GPIO_Pin_4)


#define relay12_TOGGLE		digitalToggle(GPIOI,GPIO_Pin_0)
#define relay12_OFF				digitalHi(GPIOI,GPIO_Pin_0)
#define relay12_ON				digitalLo(GPIOI,GPIO_Pin_0)

#define relay11_TOGGLE		digitalToggle(GPIOH,GPIO_Pin_15)
#define relay11_OFF				digitalHi(GPIOH,GPIO_Pin_15)
#define relay11_ON				digitalLo(GPIOH,GPIO_Pin_15)

#define relay10_TOGGLE		digitalToggle(GPIOG,GPIO_Pin_10)
#define relay10_OFF				digitalHi(GPIOG,GPIO_Pin_10)
#define relay10_ON				digitalLo(GPIOG,GPIO_Pin_10)


#define relay9_TOGGLE			digitalToggle(GPIOH,GPIO_Pin_13)
#define relay9_OFF				digitalHi(GPIOH,GPIO_Pin_13)
#define relay9_ON					digitalLo(GPIOH,GPIO_Pin_13)

#define relay8_TOGGLE			digitalToggle(GPIOE,GPIO_Pin_6)
#define relay8_OFF				digitalHi(GPIOE,GPIO_Pin_6)
#define relay8_ON					digitalLo(GPIOE,GPIO_Pin_6)

#define relay7_TOGGLE			digitalToggle(GPIOE,GPIO_Pin_5)
#define relay7_OFF				digitalHi(GPIOE,GPIO_Pin_5)
#define relay7_ON					digitalLo(GPIOE,GPIO_Pin_5)



#define relay6_TOGGLE			digitalToggle(GPIOG,GPIO_Pin_9)
#define relay6_OFF				digitalHi(GPIOG,GPIO_Pin_9)
#define relay6_ON					digitalLo(GPIOG,GPIO_Pin_9)

#define relay5_TOGGLE			digitalToggle(GPIOB,GPIO_Pin_1)
#define relay5_OFF				digitalHi(GPIOB,GPIO_Pin_1)
#define relay5_ON					digitalLo(GPIOB,GPIO_Pin_1)
  
#define relay4_TOGGLE			digitalToggle(GPIOC,GPIO_Pin_8)
#define relay4_OFF				digitalHi(GPIOC,GPIO_Pin_8)
#define relay4_ON					digitalLo(GPIOC,GPIO_Pin_8)

#define relay3_TOGGLE			digitalToggle(GPIOI,GPIO_Pin_11)
#define relay3_OFF				digitalHi(GPIOI,GPIO_Pin_11)
#define relay3_ON					digitalLo(GPIOI,GPIO_Pin_11)

#define relay2_TOGGLE			digitalToggle(GPIOH,GPIO_Pin_2)
#define relay2_OFF				digitalHi(GPIOH,GPIO_Pin_2)
#define relay2_ON					digitalLo(GPIOH,GPIO_Pin_2)

#define relay1_TOGGLE			digitalToggle(GPIOH,GPIO_Pin_3)
#define relay1_OFF				digitalHi(GPIOH,GPIO_Pin_3)
#define relay1_ON					digitalLo(GPIOH,GPIO_Pin_3)

void Relay_Config(void);





				
#endif /* __RELAY_H */








