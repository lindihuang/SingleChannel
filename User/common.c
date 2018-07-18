#include "stm32f4xx.h"
#include "stdio.h"
#include "common.h"

//static uint8_t  fac_us=0; //us��ʱ������
//static uint16_t fac_ms=0; //ms��ʱ������


/*******************************************************************************
* ��������       :  
* ��������       : Printf֧�ִ���   printf("����a��ֵ��: %d\r\n\r\n",a);
* ��ڲ���       : ��
* ���ڲ���       : ��
*******************************************************************************/
#if 1
#pragma import(__use_no_semihosting)             
                
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
 
_sys_exit(int x) //����_sys_exit()�Ա���ʹ�ð�����ģʽ   
{ 
	x = x; 
} 

//int fputc(int ch, FILE *f)//�ض���fputc���� 
//{      
//	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); 
//    USART_SendData(USART1,(uint8_t)ch);   
//	return ch;
//}
#endif 


static __IO u32 TimingDelay;
 
/**
  * @brief  ����ϵͳ�δ�ʱ�� SysTick
  * @param  ��
  * @retval ��
  */
void SysTick_Init(void)
{
	/* SystemFrequency / 1000    1ms�ж�һ��
	 * SystemFrequency / 100000	 10us�ж�һ��
	 * SystemFrequency / 1000000 1us�ж�һ��
	 */
	if (SysTick_Config(SystemCoreClock / 1000000))
	{ 
		/* Capture error */ 
		while (1);
	}
}

/**
  * @brief   us��ʱ����,10usΪһ����λ
  * @param  
  *		@arg nTime: Delay_us( 1 ) ��ʵ�ֵ���ʱΪ 1 * 10us = 10us
  * @retval  ��
  */
void Delay_us(__IO u32 nTime)
{ 
	TimingDelay = nTime;	

	while(TimingDelay != 0);
}


/**
  * @brief  ��ȡ���ĳ���
  * @param  ��
  * @retval ��
  * @attention  �� SysTick �жϺ��� SysTick_Handler()����
  */
void TimingDelay_Decrement(void)
{
	if (TimingDelay != 0x00)
	{ 
		TimingDelay--;
	}
}


///*******************************************************************************
//* ��������       : Delay_Init
//* ��������       : ��ʱ��ʼ������
//* ��ڲ���       : ��
//* ���ڲ���       : ��
//*******************************************************************************/
//void Delay_Init(void)
//{
//	SysTick->CTRL&=0xfffffffb;//bit2���,ѡ���ⲿʱ��  HCLK/8
//	fac_us=9;		    
////	fac_us=SYSCLK/8;
//	fac_ms=(uint16_t)fac_us*1000;
//}								    

///*******************************************************************************
//* ��������       : Delay_ms
//* ��������       : ��ʱ������72M������ms<=1864
//* ��ڲ���       : ms ��ʱ��С
//* ���ڲ���       : ��
//*******************************************************************************/
//void Delay_ms(uint16_t ms)
//{	 		  	  
//	uint32_t temp;		   
//	SysTick->LOAD=(uint32_t)ms*fac_ms; //ʱ�����(SysTick->LOADΪ24bit)
//	SysTick->VAL =0x00;            //��ռ�����
//	SysTick->CTRL=0x01 ;           //��ʼ����  
//	do
//	{
//		temp=SysTick->CTRL;
//	}
//	while(temp&0x01&&!(temp&(1<<16))); //�ȴ�ʱ�䵽��   
//	SysTick->CTRL=0x00;                //�رռ�����
//	SysTick->VAL =0X00;                //��ռ�����	  	    
//}   

///*******************************************************************************
//* ��������       : Delay_us
//* ��������       : ��ʱ������72M������ms<=1864000
//* ��ڲ���       : us ��ʱ��С
//* ���ڲ���       : ��
//*******************************************************************************/
//void Delay_us(uint32_t us)
//{		
//	uint32_t temp;	    	 
//	SysTick->LOAD=us*fac_us; //ʱ�����	  		 
//	SysTick->VAL=0x00;        //��ռ�����
//	SysTick->CTRL=0x01 ;      //��ʼ���� 	 
//	do
//	{
//		temp=SysTick->CTRL;
//	}
//	while(temp&0x01&&!(temp&(1<<16))); //�ȴ�ʱ�䵽��   
//	SysTick->CTRL=0x00;                //�رռ�����
//	SysTick->VAL =0X00;                //��ռ�����	 
//}

void Delay(vu32 nCount)
{
  for(; nCount != 0; nCount--);
}

/*******************************************************************************
* ��������       : USART_Config
* ��������       : ���ô���
* ��ڲ���       : USARTx ���ں� 
                   Baud   ������
* ���ڲ���       : ��
*******************************************************************************/
//void USART_Config(USART_TypeDef* USARTx,u32 Baud)
//{
//	USART_InitTypeDef USART_InitStructure;
//	USART_InitStructure.USART_BaudRate =Baud;	//������
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //����λ8λ
//	USART_InitStructure.USART_StopBits = USART_StopBits_1; //ֹͣλ1λ
//	USART_InitStructure.USART_Parity = USART_Parity_No; //��У��λ
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ������
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //�շ�ģʽ

//	USART_Init(USARTx, &USART_InitStructure);	//���ô��ڲ�������
//	
//#ifdef EN_USART_RX
//	USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);  //ʹ�ܽ����ж�
//#endif
//	//USART_ITConfig(USARTx, USART_IT_TXE, ENABLE); //ʹ�ܷ��ͻ�����ж�
//	//USART_ITConfig(USART1, USART_IT_TC, ENABLE);  //ʹ�ܷ�������ж�

//	/* Enable the USART1 */
//	USART_Cmd(USARTx, ENABLE); //ʹ�ܴ���	
//	/*CPU��Сȱ�ݣ��������úã����ֱ��Send�����1���ֽڷ��Ͳ���ȥ
//	�����������1���ֽ��޷���ȷ���ͳ�ȥ������  */
//	USART_ClearFlag(USARTx, USART_FLAG_TXE);/* �巢����ɱ�־��Transmission Complete flag */ 
//}


/*******************************************************************************
* ��������       : UartSendData
* ��������       : ����һ���ֽڵ�����
* ��ڲ���       : USARTx ���ںţ�����
* ���ڲ���       : ���ط��͵�����
*******************************************************************************/
u8 UartSendData(USART_TypeDef* USARTx,uint8_t ch)
{
	/* Write a character to the USART */
	USART_SendData(USARTx, ch);
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET){}
	return ch;
}

//����һ���ַ���
void UartSendString(USART_TypeDef* USARTx,char *str)
{

	 while(*str!='\0')
	{
		USART_SendData(USARTx, *str++);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET){}	
	}
}	


