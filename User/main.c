/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   ��1.5.1�汾�⽨�Ĺ���ģ��
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:����  STM32 F429 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
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
  * @brief  ������
  * @param  ��
  * @retval ��
  */

int standard_AD = 3000;

uint16_t press_z_flag = 0;	//ѹ������־λ
uint16_t press_f_flag = 0;	//ѹ������־λ
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
	 * ��������main����֮ǰ�������ļ���statup_stm32f429xx.s�Ѿ�����
	 * SystemInit()������ϵͳʱ�ӳ�ʼ����180MHZ
	 * SystemInit()��system_stm32f4xx.c�ж���
	 * ����û����޸�ϵͳʱ�ӣ������б�д�����޸�
	 */
	
	/** 
	 * ����SysTick Ϊ1us�ж�һ��,ʱ�䵽�󴥷���ʱ�жϣ�
	 * ����stm32fxx_it.c�ļ���SysTick_Handler����ͨ�����жϴ�����ʱ
	 */
	SysTick_Init();
	
  /*��ʼ��USART1*/
  DEBUG_USART_Config();
	
	/*�̵�����ʼ��*/
	Relay_Config();
	
	/*��ʱ��5��ʼ��*/
	Timer5_config();
	
	/*��ʱ��1��ʼ��*/
	TIMx_Configuration();

	/*LED��ʼ��*/
	LED_GPIO_Config();
	
	/*������ʼ��*/
	Key_GPIO_Config();
	
	relay17_Config();					//2020.11.18����ӽ�����ʹ��  
	
	eMBInit(MB_RTU, 1, 1, 38400, MB_PAR_NONE);

	eMBEnable();
	
	/* ��ʼ��ADS1256 GPIO�ܽ� */
	Init_ADS1256_GPIO(); 
	
	Delay_ms(200); 
	
	/**
	 * 2020.10.14���reset();�����AD����������ע�͵�reset();
	 * �ú������ã���ADоƬ�ϵ���ȶ��临λһ�Σ���ֹADÿ�ο����Բ���
	 */
	reset();
	
  Delay_ms(200);

	ADS1256_Init();
	
	Delay_ms(200);
	
	/* ���������� */
	Setzero_Init();
	
	ucSCoilBuf[0] &= (~(1 << 2));	//0xFD
	
	Start_Over_Flag_1 = 0;

  ucSCoilBuf[0] = 0x00;		

  GS_Flag = 0;
   
  while(1)
	{
		/* �ֶ��������� */
		set_zero();

		/**
  	 * �ӽ����ز��֣�û�е���д����  ���������������ε� 
		 * 
		 * Զ���ѹ����ӽ����ز���ʱΪ1
		 */  
		if(GPIO_ReadInputDataBit(GPIOH,GPIO_Pin_10) == 1)	
		{
			ucSCoilBuf[10] |= 0x10;	//85=1	
		}
		else
		{
			ucSCoilBuf[10] &= 0xef;	//85=0
		}
			
		/* ������ѹ����ӽ����ز���ʱΪ1 */
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_3) == 1)		
		{
			ucSCoilBuf[10] |= 0x20;	//86=1
		}
		else
		{
			ucSCoilBuf[10] &= 0xdf; //86=0
		}
		/* �ӽ����ز��ֽ�����û�е���д����  ���������������ε� */  

	  Step();						//��դ���
		STOP_INTERRUP();	//�ⲿ�����ƿ���
	  Step();
		out_start_stop();	//�ⲿ����ֹͣ����	
	  Step();
		Press_z_f();			//ѹ����������ѹ
	  Step();
		
    (void)eMBPoll(); 

		/***************MPM270****************/		 
		Acquisition_channel(1,disable,0,0);	//ʹ����Ҫ�Ľ���ɼ�ͨ��
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
		
//		/**********************������������2023.5.12����ӣ�****************************/
		if(ucSCoilBuf[4] & 0x02)
		{
			ucSCoilBuf[4] |= 0x80;
		}
		else
		{
			ucSCoilBuf[4] &= 0x7F;
		}
//		/****************************����Ҫʱ���ε�*************************************/
		
		AUTO_ZERO();		//�Զ�����
		Step();					//��դ���
		Relay_control();//�ǲ��Թ��̵̼������� 			 
  }	
}

/*********************************************************************
 * ��������	��void STOP_INTERRUP(void)
 * ��    ��	���ⲿ�����ƿ���
 * ��    ��	��
 * Edieor		��
 * ��    ע	��Jinpeng.Guo
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
 * ��������	��void out_start_stop(void)
 * ��    ��	���ⲿ����ֹͣ����
 * ��    ��	��
 * Edieor		��
 * ��    ע	��Jinpeng.Guo
 **********************************************************************/
void out_start_stop(void)
{ 
	if((!(ucSCoilBuf[0] & 0x02)))
	{
		if(ucSCoilBuf[0] & 0x08)
		{
			/* PA0--WKUP   ����MCU */
			if((Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON) && (GS_Flag == 0))
			{
				/**
					* LED1��ת
					*	PC13	- K2(�ⲿ��ͣ��-P2)
					* ��ͣ����
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
				* LED1��ת
				*	PB9 - K3 (�ⲿ������-P1)
				* ��Ļû����������
				*/
			if((Key_Scan(KEY9_GPIO_PORT,KEY9_PIN) == KEY_ON) && (GS_Flag == 0) && (!(ucSCoilBuf[0] & 0x08)))
			{
				/**
					* LED1��ת
					* PC13	- K2(�ⲿ��ͣ��-P2)
					*/
				if(Key_Scan(KEY2_GPIO_PORT,KEY2_PIN) == KEY_OFF)//��ͣ����
				{
				}
				else
				{      
					ucSCoilBuf[7] |= 0x01;
				}
			}   
			
			/* PA0--WKUP   ����MCU */
			if((Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON) && (GS_Flag == 0))
			{
				/**
					* LED1��ת
					* PC13	- K2(�ⲿ��ͣ��-P2)
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
		* LED2��ת
		* PC13	- K2(�ⲿ��ͣ��-P2)
		*/	
	if(Key_Scan(KEY2_GPIO_PORT,KEY2_PIN) == KEY_OFF)
	{
		/* ����ֹͣ */
		if(ucSCoilBuf[0] & 0x02)
		{	
			ucSCoilBuf[3] = 0x00;		//�������ر�
			relay2_OFF;					
			
			ucSCoilBuf[3] = 0x10;
			ucSCoilBuf[3] = 0x20;
			relay5_ON;
			relay6_ON;							//��ѹ������
			Delay_ms(50);
					
			relay11_OFF;						//���ȹرն���1
			ucSCoilBuf[4] &= 0xf7;	//���ȹرն���1
			ucSCoilBuf[0] |= 0x01; 
			ucSCoilBuf[0] &= 0xf9; 
			ucSCoilBuf[7] &= 0xfe;  //����׼��״̬����
			
			TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
			TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
			TIM_SetCounter(TIM3, 0);
			Timer3_cout = 0;
			relay5_OFF;
			relay6_OFF;							//��ѹ������
		}
	}
}

/*********************************************************************
 * ��������	��void Press_z_f(void) 
 * ��    ��	��ѹ����������ѹ
 * ��    ��	��
 * Edieor		��
 * ��    ע	��Jinpeng.Guo
 **********************************************************************/
void Press_z_f(void) 
{
	/* ��ѹ */
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
	
	/* ��ѹ */
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

