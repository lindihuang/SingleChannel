#include "relay.h"
#include "mb.h"

void Relay_Config(void)
{
	/* 定义一个GPIO_InitTypeDef类型的结构体 */
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 开启LED相关的GPIO外设时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOE
	| RCC_AHB1Periph_GPIOI | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC
	| RCC_AHB1Periph_GPIOH | RCC_AHB1Periph_GPIOG, ENABLE); 

	/* 选择要控制的GPIO引脚 */															   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;	

	/* 设置引脚模式为输出模式 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;   
	
	/* 设置引脚的输出类型为推挽输出 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	
	/* 设置引脚为上拉模式 */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	/* 设置引脚速率为2MHz */   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 

	/* 调用库函数，使用上面配置的GPIO_InitStructure初始化GPIO */
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_11 | GPIO_Pin_2;
	GPIO_Init(GPIOI, &GPIO_InitStructure);  
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_12;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_2 | GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_Init(GPIOH, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	/**
	 * 光栅引脚
	 * 选择要控制的GPIO引脚
	 */															   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	

	/* 设置引脚模式为输入模式 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;   
		
	/* 设置引脚为上拉模式 */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

	/* 设置引脚速率为2MHz */   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	
	/* 调用库函数，使用上面配置的GPIO_InitStructure初始化GPIO */
	GPIO_Init(GPIOA, &GPIO_InitStructure);  
	GPIO_ResetBits(GPIOA,GPIO_Pin_11);
	
	relay1_OFF;
	relay2_OFF;
	relay3_OFF;
	relay4_OFF;
	relay5_OFF;
	relay6_OFF;
	relay7_OFF;
	relay8_OFF;
	relay9_OFF;
	relay10_OFF;
	relay11_OFF;
	relay12_OFF;
	relay13_OFF;
	relay14_OFF;
	relay15_OFF;
	relay16_OFF;
}


