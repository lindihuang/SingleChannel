/**
  ******************************************************************************
  * @file    FMC_SDRAM/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    11-November-2013
  * @brief   Main Interrupt Service Routines.
  *         This file provides template for all exceptions handler and
  *         peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "port.h"
#include "./usart/bsp_rs232_usart.h"

#include "bsp_SysTick.h"
#include "Timer5.h"
#include "relay.h"
#include "mb.h"
#include "bsp_led.h"   
#include "modbus.h"
#include "bsp_exti.h"
#include "Timer1.h"
/** @addtogroup STM32F429I_DISCOVERY_Examples
  * @{
  */

/** @addtogroup FMC_SDRAM
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
   TimingDelay_Decrement();
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f429_439xx.s).                         */
/******************************************************************************/

/**
  * @}
  */ 
/*******************************************************************************
* Function Name  : USART1_IRQHandler
* Description    : This function handles USART1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RS232_USART_IRQHandler(void)
{
	if(USART_GetITStatus(RS232_USART, USART_IT_RXNE) == SET)
	{		
		prvvUARTRxISR();
		USART_ClearITPendingBit(RS232_USART, USART_IT_RXNE);
//	  Rxflag=1;		
//		ucTemp = USART_ReceiveData( RS232_USART );
	}

	if(USART_GetITStatus(RS232_USART, USART_IT_TXE) == SET)
	{
		prvvUARTTxReadyISR();
// 		USART_ClearITPendingBit(USART1, USART_IT_TXE);
	}
}
/*******************************************************************************
* Function Name  : TIM2_IRQHandler
* Description    : This function handles TIM2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

//void ADVANCE_TIM_IRQHandler(void)
//{
//		if ( TIM_GetITStatus( ADVANCE_TIM, TIM_IT_Update) != RESET ) 
//	{	
//		Timer1_cout++;
//		if(Timer1_cout>10)
//		{
//			LED3_TOGGLE;
//		}	
//		TIM_ClearITPendingBit(ADVANCE_TIM , TIM_IT_Update); 
//	}
//}


void TIM2_IRQHandler(void)
{
	TIMERExpiredISR();
	
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}
void GENERAL_TIM_IRQHandler(void)           //Timer4
{
	Timer3_cout++;
	
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}

void GENERAL_TIM4_IRQHandler(void)
{
	Timer4_cout++;

	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
}

union LianHeTi test_data9;
union LianHeTi test_data10;

union LianHeTi test_data11;
union LianHeTi test_data12;
union LianHeTi test_data13;

union LianHeTi test_data15;

union LianHeTi test_data16;
union LianHeTi test_data17;
union LianHeTi test_data18;
union LianHeTi test_data19;

uint16_t jieduan_Flag_1;
uint16_t Mode_Flag_1;
//USHORT   usSRegHoldBuf[100];
void TIM5_IRQHandler(void)
{
	Timer5_cout = Timer5_cout + 1;
	
	/** 清除TIMx的中断挂起位 */
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	
	if(Timer5_cout > 30)
	{
		/** 清除TIMx的中断挂起位 */
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
		
		/** 启用或禁用指定的TIM中断 */
		TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);
		
		/** 设置TIMx计数器寄存器值 */
		TIM_SetCounter(TIM5, 0);
		
		/** 启用或禁用指定TIM外设 */
		TIM_Cmd(TIM5, DISABLE);
		
		ucSCoilBuf[4] &= 0x7f;
		Timer5_cout = 0;
		relay15_OFF;
	}
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f429_439xx.s).                         */
/******************************************************************************/

/**
  * @}
  */ 



//static void Delay(__IO uint32_t nCount)	 //简单的延时函数
//{
//	for(; nCount != 0; nCount--);
//}
//void KEY1_IRQHandler(void)
//{
//  //确保是否产生了EXTI Line中断
//	if(EXTI_GetITStatus(KEY1_INT_EXTI_LINE) != RESET) 
//	{
//		// LED1 取反		
////		LED1_TOGGLE;
//		LED_RGBOFF;
//		LED_WHITE;

//		
//		ucSCoilBuf[5]|=0x04;
//    //清除中断标志位
//		EXTI_ClearITPendingBit(KEY1_INT_EXTI_LINE);     
//	}  
//}

//void KEY2_IRQHandler(void)
//{
//  //确保是否产生了EXTI Line中断
//	if(EXTI_GetITStatus(KEY2_INT_EXTI_LINE) != RESET) 
//	{
//		// LED2 取反	
////    DelaySCLK(123123123132)	;
////		LED1_ON;
//				ucSCoilBuf[0]&=0xF9;
//		 		relay6_OFF;
//				Delay(1000000);
//				relay5_OFF;
////		LED2_TOGGLE;
//		LED_RGBOFF;
//		LED_YELLOW;
//    //清除中断标志位
//		EXTI_ClearITPendingBit(KEY2_INT_EXTI_LINE);     
//	}  
//}
/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
