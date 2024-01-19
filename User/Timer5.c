#include "Timer5.h"
#include "mb.h"

void Timer5_config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
  
  TIM_DeInit(TIM5);

	/**
	 * 3.5个字符时间区分不同的帧，即接收到的两个字符之间时间间隔小于3.5个字符
	 * 时间时认为是同一个帧的，如果间隔大于3.5个字符时间则认为是不同帧的
	 * 在一般的串口通信中，发送1个字符需要：1位起始位，8位数据位，1位校验位(可无),
	 * 1位停止位,总共 1+8+1+1 = 11位，3.5个字符时间就是 3.5 * 11 = 38.5位，
	 * 假如波特率是9600,那么传输1位的时间是1000/9600 = 0.10416667(ms) ,
	 * 这样，3.5个字符时间就大约是 4 ms ,即定时器需要的中断时间
	 * 
	 * 这个就是预分频系数9000/180M = 0.00005,即每50us计数值加1
	 * 50us x 2000 = 100ms,即100ms中断一次
	 */	
  TIM_TimeBaseStructure.TIM_Period = 2000-1;
  TIM_TimeBaseStructure.TIM_Prescaler = (9000 - 1);	
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	
	/* 预装载使能 */
	TIM_ARRPreloadConfig(TIM5, ENABLE);
	/*====================================中断初始化===========================*/
	
	/* 设置NVIC优先级分组为Group2：0-3抢占式优先级，0-3的响应式优先级 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);
	
	/* 清除溢出中断标志位 */
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	
	/* 定时器3溢出中断关闭 */
	TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);
	
	/* 定时器3禁能 */
	TIM_Cmd(TIM5, DISABLE);
}



