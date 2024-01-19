/**
  ******************************************************************************
  * @file    bsp_advance_tim.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   高级控制定时器定时范例
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火  STM32 F429 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */  
#include "Timer1.h"

/****************************************定时器3*********************************************************************/
static void TIMx_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	/* 设置中断组为0 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);	
	
	/* 设置中断来源 */
	NVIC_InitStructure.NVIC_IRQChannel = GENERAL_TIM_IRQn; 
	
	/* 设置抢占优先级 */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	
	
	/* 设置子优先级 */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  高级控制定时器 TIMx,x[1,8]中断优先级配置
  * @param  无
  * @retval 无
  */
static void TIM_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	/* 开启TIMx_CLK,x[6,7] */
  RCC_APB1PeriphClockCmd(GENERAL_TIM_CLK, ENABLE); 

  /**
   * 累计 TIM_Period个后产生一个更新或者中断
	 *
	 * 当定时器从0计数到4999，即为5000次，为一个定时周期
	 */	
  TIM_TimeBaseStructure.TIM_Period = 2000-1;       
	
	/** 
	 * 高级控制定时器时钟源 TIMxCLK = HCLK/2=90MHz 
	 * 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=10000Hz
	 */
  TIM_TimeBaseStructure.TIM_Prescaler = 9000-1;	
	
  /* 采样时钟分频 */
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	
  /* 计数方式 */
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	
	/* 初始化定时器TIMx, x[1,8] */
	TIM_TimeBaseInit(GENERAL_TIM, &TIM_TimeBaseStructure);
		
	/* 清除定时器更新中断标志位 */
	TIM_ClearFlag(GENERAL_TIM, TIM_FLAG_Update);
	
	/* 开启定时器更新中断 */
	TIM_ITConfig(GENERAL_TIM,TIM_IT_Update,ENABLE);
	
	/* 使能定时器 */
	TIM_Cmd(GENERAL_TIM, ENABLE);	
}

/**
  * @brief  初始化高级控制定时器定时，1ms产生一次中断
  * @param  无
  * @retval 无
  */
void TIMx_Configuration(void)
{
	TIMx_NVIC_Configuration();	
  
  TIM_Mode_Config();
}

/****************************************定时器4*********************************************************************/
static void TIM4_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	/* 设置中断组为0 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
	/* 设置中断来源 */
	NVIC_InitStructure.NVIC_IRQChannel = GENERAL_TIM4_IRQn; 
	
	/* 设置抢占优先级 */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	 
	
	/* 设置子优先级 */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  高级控制定时器 TIMx,x[1,8]中断优先级配置
  * @param  无
  * @retval 无
  */
static void TIM4_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	/* 开启TIMx_CLK,x[6,7] */ 
  RCC_APB1PeriphClockCmd(GENERAL_TIM4_CLK, ENABLE); 

  /** 
	 * 累计 TIM_Period个后产生一个更新或者中断
	 * 当定时器从0计数到4999，即为5000次，为一个定时周期
	 */	
  TIM_TimeBaseStructure.TIM_Period = 2000-1;       
	
	/**
   * 高级控制定时器时钟源TIMxCLK = HCLK/2=90MHz 
	 * 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=10000Hz
	 */
  TIM_TimeBaseStructure.TIM_Prescaler = 9000-1;	
	
  /* 采样时钟分频 */
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	
  /* 计数方式 */
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	
	/* 初始化定时器TIMx, x[1,8] */
	TIM_TimeBaseInit(GENERAL_TIM4, &TIM_TimeBaseStructure);
	
	/* 清除定时器更新中断标志位 */
	TIM_ClearFlag(GENERAL_TIM4, TIM_FLAG_Update);
	
	/* 开启定时器更新中断 */
	TIM_ITConfig(GENERAL_TIM4,TIM_IT_Update,ENABLE);
	
	/* 使能定时器 */
	TIM_Cmd(GENERAL_TIM4, ENABLE);	
}

/**
  * @brief  初始化高级控制定时器定时，1ms产生一次中断
  * @param  无
  * @retval 无
  */
void TIM4_Configuration(void)
{
	TIM4_NVIC_Configuration();	
  
  TIM4_Mode_Config();
}

