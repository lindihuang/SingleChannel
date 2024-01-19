/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   用1.5.1版本库建的工程模板
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火  STM32 F429 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
#include "bsp_exti.h" 
#include "stm32f4xx.h"
#include "./usart/bsp_rs232_usart.h" 
#include "bsp_led.h"   
#include "string.h"
#include "Timer5.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "relay.h"
#include "bsp_SysTick.h"

#include "bsp_debug_usart.h"
#include "string.h"
#include "math.h"
#include "modbus.h"
#include <float.h>
#include <stdlib.h>

#include "KEY.h" 
#include "Timer1.h"
#include "new.h"
#include "data_processing.h" 
#include "ADS1256.h"
/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */

int standard_AD = 3000;

uint16_t press_z_flag = 0;	//压力正标志位
uint16_t press_f_flag = 0;	//压力负标志位
uint16_t Start_Over_Flag_1;

void set_zero(void);
void Press_z_f(void);
void Setzero_Init(void);
void STOP_INTERRUP(void);
void out_start_stop(void);
void deviationchange(uint8_t *array,uint32_t num);

int main(void)
{
	press_f_flag = 0;
	press_z_flag = 0;
	Timer5_cout = 0;

	/** 
	 * 程序来到main函数之前，启动文件：statup_stm32f429xx.s已经调用
	 * SystemInit()函数把系统时钟初始化成180MHZ
	 * SystemInit()在system_stm32f4xx.c中定义
	 * 如果用户想修改系统时钟，可自行编写程序修改
	 */
	
	/** 
	 * 配置SysTick 为1us中断一次,时间到后触发定时中断，
	 * 进入stm32fxx_it.c文件的SysTick_Handler处理，通过数中断次数计时
	 */
	SysTick_Init();
	
  /*初始化USART1*/
  DEBUG_USART_Config();
	
	/*继电器初始化*/
	Relay_Config();
	
	/*定时器5初始化*/
	Timer5_config();
	
	/*定时器1初始化*/
	TIMx_Configuration();

	/*LED初始化*/
	LED_GPIO_Config();
	
	/*按键初始化*/
	Key_GPIO_Config();
	
	relay17_Config();					//2020.11.18加入接近开关使能  
	
	eMBInit(MB_RTU, 1, 1, 38400, MB_PAR_NONE);

	eMBEnable();
	
	/* 初始化ADS1256 GPIO管脚 */
	Init_ADS1256_GPIO(); 
	
	Delay_ms(200); 
	
	/**
	 * 2020.10.14添加reset();，如果AD有问题马上注释掉reset();
	 * 该函数作用：在AD芯片上电后先对其复位一次，防止AD每次开机对不上
	 */
	reset();
	
  Delay_ms(200);

	ADS1256_Init();
	
	Delay_ms(200);
	
	/* 启动·调零 */
	Setzero_Init();
	
	ucSCoilBuf[0] &= (~(1 << 2));	//0xFD
	
	Start_Over_Flag_1 = 0;

  ucSCoilBuf[0] = 0x00;		

  GS_Flag = 0;
   
  while(1)
	{
		/* 手动按键调零 */
		set_zero();

		/**
  	 * 接近开关部分，没有单独写函数  出现问题马上屏蔽掉 
		 * 
		 * 远离调压阀侧接近开关不亮时为1
		 */  
		if(GPIO_ReadInputDataBit(GPIOH,GPIO_Pin_10) == 1)	
		{
			ucSCoilBuf[10] |= 0x10;	//85=1	
		}
		else
		{
			ucSCoilBuf[10] &= 0xef;	//85=0
		}
			
		/* 靠近调压阀侧接近开关不亮时为1 */
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_3) == 1)		
		{
			ucSCoilBuf[10] |= 0x20;	//86=1
		}
		else
		{
			ucSCoilBuf[10] &= 0xdf; //86=0
		}
		/* 接近开关部分结束，没有单独写函数  出现问题马上屏蔽掉 */  

	  Step();						//光栅检测
		STOP_INTERRUP();	//外部按键灯控制
	  Step();
		out_start_stop();	//外部启动停止按键	
	  Step();
		Press_z_f();			//压力设置正负压
	  Step();
		
    (void)eMBPoll(); 

		/***************MPM270****************/		 
		Acquisition_channel(1,disable,0,0);	//使能需要的交叉采集通道
		/***************END****************/
		
		Step();
		
		if(!(ucSCoilBuf[0] & 0x02))
		{
			ucSCoilBuf[1] |= 0x02;
			
			TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
			TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
			TIM_SetCounter(TIM3, 0);
			
			Timer3_cout = 0;
		}
		
//		/**********************蜂鸣器报警（2023.5.12日添加）****************************/
		if(ucSCoilBuf[4] & 0x02)
		{
			ucSCoilBuf[4] |= 0x80;
		}
		else
		{
			ucSCoilBuf[4] &= 0x7F;
		}
//		/****************************不需要时屏蔽掉*************************************/
		
		AUTO_ZERO();		//自动调零
		Step();					//光栅检测
		Relay_control();//非测试过程继电器控制 			 
  }	
}

/*********************************************************************
 * 函数定义	：void STOP_INTERRUP(void)
 * 描    述	：外部按键灯控制
 * 功    能	：
 * Edieor		：
 * 备    注	：Jinpeng.Guo
 **********************************************************************/
void STOP_INTERRUP(void)
{
	if(ucSCoilBuf[4] & 0x01)
	{
		relay9_ON;
		Timer5_cout = 0;
	}
	else 
	{
		relay9_OFF;
	}
	if(ucSCoilBuf[4] & 0x02)
	{
		relay10_ON;
	}
	else 
	{
		relay10_OFF;
	}
}

/*********************************************************************
 * 函数定义	：void out_start_stop(void)
 * 描    述	：外部启动停止按键
 * 功    能	：
 * Edieor		：
 * 备    注	：Jinpeng.Guo
 **********************************************************************/
void out_start_stop(void)
{ 
	if((!(ucSCoilBuf[0] & 0x02)))
	{
		if(ucSCoilBuf[0] & 0x08)
		{
			/* PA0--WKUP   唤醒MCU */
			if((Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON) && (GS_Flag == 0))
			{
				/**
					* LED1反转
					*	PC13	- K2(外部急停键-P2)
					* 急停摁下
					*/
				if(Key_Scan(KEY2_GPIO_PORT,KEY2_PIN) == KEY_OFF)
				{
				}
				else
				{      
					ucSCoilBuf[7] |= 0x01;
				}
			}  
		}
		else
		{
			/**
				* LED1反转
				*	PB9 - K3 (外部启动键-P1)
				* 屏幕没有启动屏蔽
				*/
			if((Key_Scan(KEY9_GPIO_PORT,KEY9_PIN) == KEY_ON) && (GS_Flag == 0) && (!(ucSCoilBuf[0] & 0x08)))
			{
				/**
					* LED1反转
					* PC13	- K2(外部急停键-P2)
					*/
				if(Key_Scan(KEY2_GPIO_PORT,KEY2_PIN) == KEY_OFF)//急停摁下
				{
				}
				else
				{      
					ucSCoilBuf[7] |= 0x01;
				}
			}   
			
			/* PA0--WKUP   唤醒MCU */
			if((Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON) && (GS_Flag == 0))
			{
				/**
					* LED1反转
					* PC13	- K2(外部急停键-P2)
					*/
				if( Key_Scan(KEY2_GPIO_PORT,KEY2_PIN) == KEY_OFF)
				{
				}
				else
				{      
					ucSCoilBuf[7] |= 0x01;
				}
			}  
		}
	}

	/**
		* LED2反转
		* PC13	- K2(外部急停键-P2)
		*/	
	if(Key_Scan(KEY2_GPIO_PORT,KEY2_PIN) == KEY_OFF)
	{
		/* 测试停止 */
		if(ucSCoilBuf[0] & 0x02)
		{	
			ucSCoilBuf[3] = 0x00;		//进气阀关闭
			relay2_OFF;					
			
			ucSCoilBuf[3] = 0x10;
			ucSCoilBuf[3] = 0x20;
			relay5_ON;
			relay6_ON;							//高压先排气
			Delay_ms(50);
					
			relay11_OFF;						//优先关闭动作1
			ucSCoilBuf[4] &= 0xf7;	//优先关闭动作1
			ucSCoilBuf[0] |= 0x01; 
			ucSCoilBuf[0] &= 0xf9; 
			ucSCoilBuf[7] &= 0xfe;  //参数准备状态归零
			
			TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
			TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
			TIM_SetCounter(TIM3, 0);
			Timer3_cout = 0;
			relay5_OFF;
			relay6_OFF;							//高压先排气
		}
	}
}

/*********************************************************************
 * 函数定义	：void Press_z_f(void) 
 * 描    述	：压力设置正负压
 * 功    能	：
 * Edieor		：
 * 备    注	：Jinpeng.Guo
 **********************************************************************/
void Press_z_f(void) 
{
	/* 正压 */
	if((ucSCoilBuf[5] & 0x08) && (!(ucSCoilBuf[5] & 0x10)))  
	{
		press_z_flag = 1;
		
		relay1_OFF;
		relay2_ON;	
		relay4_OFF;	
		relay6_ON;
		
		ucSCoilBuf[3] &= (~0x09);	//0xF6 ->1111 0110
		ucSCoilBuf[3] |= 0x22;
	}
	
	if((!(ucSCoilBuf[5] & 0x08)) && (press_z_flag == 1))
	{
		relay1_OFF;
		relay2_OFF;	
		relay4_OFF;	
		relay6_OFF;
		
		press_z_flag = 0;
		
		ucSCoilBuf[3] &= (~0x22);	//0xDD->1101 1101
	}
	
	/* 负压 */
	if((!(ucSCoilBuf[5] & 0x08)) && (ucSCoilBuf[5] & 0x10)) 
	{
		press_f_flag = 1;
		
		relay1_ON;
		relay2_OFF;	
		relay4_ON;	
		relay6_ON;
		
		ucSCoilBuf[3] &= (~0x02);	//0xFD
		ucSCoilBuf[3] |= 0x29;
	}
	
	if((!(ucSCoilBuf[5] & 0x10)) && (press_f_flag == 1))	
	{
		relay1_OFF;
		relay2_OFF;	
		relay4_OFF;	
		relay6_OFF;
		
		press_f_flag = 0;
		
		ucSCoilBuf[3] &= (~0x29);	//0xD6
	}
}

/*****************************END OF FILE**********************/

