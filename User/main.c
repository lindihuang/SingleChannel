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
	
uint16_t Start_Over_Flag_1;

//USHORT   usSRegInBuf[100];
//union LianHeTi test_data1;
void pianyiliangchange(uint8_t *array,uint32_t num);
void Setzero_Init(void);
void set_zero(void);
void STOP_INTERRUP(void);
//const 	uint16_t chushizhi=3000;  //��ʼ����ѹ�µ�adֵ
//uint16_t pianyiliang=0;   //����ƫ����
void out_start_stop(void);
void Press_z_f(void);
uint16_t press_z_flag=0;
uint16_t press_f_flag=0;
int standard_AD=3000;

int main(void)
{
	

	uint16_t len;
//	int16_t unit, polarity, point;
//	float data = 0, point_t;
//	const char str_read_datas[]  = {'$', 'H', X1+0x30, 'E',  X2+0x30, 'L', 'C',  'D', 'M'};//��ȡ����
	char *pbuf;
	press_f_flag=0;
	press_z_flag=0;
	Timer5_cout=0;

//	uint16_t usRxCount=0; 
		/* ��������main����֮ǰ�������ļ���statup_stm32f429xx.s�Ѿ�����
		* SystemInit()������ϵͳʱ�ӳ�ʼ����180MHZ
		* SystemInit()��system_stm32f4xx.c�ж���
		* ����û����޸�ϵͳʱ�ӣ������б�д�����޸�
		*/
		  /* add your code here ^_^. */
	  /* ����SysTick Ϊ10us�ж�һ��,ʱ�䵽�󴥷���ʱ�жϣ�
	*����stm32fxx_it.c�ļ���SysTick_Handler����ͨ�����жϴ�����ʱ
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
//	TIM4_Configuration();
//	
	/*LED��ʼ��*/
	LED_GPIO_Config();
//	EXTI_Key_Config();
////	/*������ʼ��*/
	Key_GPIO_Config();
	
	eMBInit(MB_RTU, 1, 1, 9600, MB_PAR_NONE);

	eMBEnable();
	
	Init_ADS1256_GPIO(); //��ʼ��ADS1256 GPIO�ܽ� 

  Delay_ms(200);

	ADS1256_Init();
	
	Delay_ms(200);
	
	ADS1256_Init();
	
	Delay_ms(200);
	
	ADS1256_Init();
	
	Delay_ms(200);
	
	ADS1256_Init();
	
	Delay_ms(200);
	
	ADS1256_Init();
	
	Delay_ms(200);
	
	Setzero_Init();//����������
	ucSCoilBuf[0]&=(~(1<<2));
	Start_Over_Flag_1=0;

  ucSCoilBuf[0]=0x00;		


  GS_Flag=0;
   
  while(1)
	{

		set_zero();



			STOP_INTERRUP();

			out_start_stop();	
			
			Press_z_f();

       ( void)eMBPoll(); 

///***************************************************MPM270*************************************************************/		 
			  caijitongdao(1,disable,0,0);
///***************************************************MPM270*************************************************************/
			 Step();
		   if(!(ucSCoilBuf[0]&0x02))
			 {
				 ucSCoilBuf[1]|=0x02;
				TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
				TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
				TIM_SetCounter(TIM3, 0);
				 Timer3_cout=0;
			 }
			 AUTO_ZERO();
					Relay_control();
	
//			 
			 
   }

		
}

//**********************************************************************************
//��������: void Setzero_Init(void)	
//��    �����ⲿ�����ƿ���

//��    ע��
//**********************************************************************************
void STOP_INTERRUP(void)
{
////		if((!GPIO_ReadInputDataBit(GPIOI,GPIO_Pin_4))||(!(ucSCoilBuf[5]&0x01)))
////		{
////			
////			ucSCoilBuf[0]&=0xfd;
////			relay11_OFF;
////			relay12_OFF;
////			relay13_OFF;
////			relay14_OFF;
////			relay15_OFF;
////			relay16_OFF;
	  if(ucSCoilBuf[4]&0x01)
		{
			relay9_ON;
			Timer5_cout=0;
		}
		else 
		{
			relay9_OFF;
		}
		if(ucSCoilBuf[4]&0x02)
		{
			relay10_ON;
	
		}
		else 
		{
			relay10_OFF;
		}

////		}
}  
//**********************************************************************************
//��������: static void Delay(__IO uint32_t nCount)	
//��    �����򵥵���ʱ����

//��    ע��
//**********************************************************************************
static void Delay(__IO uint32_t nCount)	 //�򵥵���ʱ����
{
	for(; nCount != 0; nCount--);
}

//**********************************************************************************
//��������: void out_start_stop(void)
//��    �����ⲿ����ֹͣ����

//��    ע��
//**********************************************************************************

void out_start_stop(void)
{
		if(( Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON  )&&(GS_Flag==0))
		{
			/*LED1��ת*/


      ucSCoilBuf[7]|=0x01;
			
			
		}   
    
    if( Key_Scan(KEY2_GPIO_PORT,KEY2_PIN) == KEY_ON  )
		{
			/*LED2��ת*/

			if(ucSCoilBuf[0]&0x02)
			{

				ucSCoilBuf[0]|=0x01; 
				ucSCoilBuf[0]&=0xf1; 
				ucSCoilBuf[7]&=0xfe;            //����׼��״̬����
				TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
				TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
				TIM_SetCounter(TIM3, 0);
				Timer3_cout=0;

	
		   }   
			else
			{
				relay5_ON;
				relay6_ON;
				Delay_ms(500);
				relay6_OFF;
				Delay_ms(100);
				relay5_OFF;
			}
		}
}
//**********************************************************************************
//��������: void out_start_stop(void)
//��    ����ѹ����������ѹ

//��    ע��
//**********************************************************************************
void Press_z_f(void) 
{
		if((ucSCoilBuf[5]&0x08)&&(!(ucSCoilBuf[5]&0x10)))  //��ѹ
	{
	    press_z_flag=1;
			relay1_OFF;
			relay2_ON;	
			relay4_OFF;	
			relay6_ON;
		  ucSCoilBuf[3]&=(~0x09);
		
			ucSCoilBuf[3]|=0x22;

	}
	if((!(ucSCoilBuf[5]&0x08))&&(press_z_flag==1))
	{
			relay1_OFF;
			relay2_OFF;	
			relay4_OFF;	
			relay6_OFF;
			press_z_flag=0;
		ucSCoilBuf[3]&=(~0x22);
		
	}
	
		if((!(ucSCoilBuf[5]&0x08))&&(ucSCoilBuf[5]&0x10) ) //��ѹ
	{
	    press_f_flag=1;
			relay1_ON;
			relay2_OFF;	
			relay4_ON;	
			relay6_ON;
			ucSCoilBuf[3]&=(~0x02);
			ucSCoilBuf[3]|=0x29;

	}
	if((!(ucSCoilBuf[5]&0x10))&&(press_f_flag==1))
	{
			relay1_OFF;
			relay2_OFF;	
			relay4_OFF;	
			relay6_OFF;
			press_f_flag=0;
		  ucSCoilBuf[3]&=(~0x29);
		
	}
}

/*********************************************END OF FILE**********************/

