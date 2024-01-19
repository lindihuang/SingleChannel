/**
  ******************************************************************************
  * @file    bsp_rs232_usart.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   使用串口2，重定向c库printf函数到usart端口，中断接收模式
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火  STM32 F429 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 
  
#include "./usart/bsp_rs232_usart.h"


 /**
  * @brief  配置嵌套向量中断控制器NVIC
  * @param  无
  * @retval 无
  */
//static void NVIC_Configuration(void)
//{
//  NVIC_InitTypeDef NVIC_InitStructure;
//    
//	  
//	  /* Set the Vector Table base location at 0x08000000 */
//  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
//	
//	
//  /* Configure one bit for preemption priority */
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
//  
//  /* 配置中断源 */
//  NVIC_InitStructure.NVIC_IRQChannel = RS232_USART_IRQ;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
//	// 	//TIM2 
//	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
//	NVIC_Init(&NVIC_InitStructure);	
//  
//}


 /**
  * @brief  RS232_USART GPIO 配置,工作模式配置。115200 8-N-1 ，中断接收模式
  * @param  无
  * @retval 无
  */
//void Debug_USART_Config(void)
//{
//  GPIO_InitTypeDef GPIO_InitStructure;
//  USART_InitTypeDef USART_InitStructure;
//		
//  RCC_AHB1PeriphClockCmd( RS232_USART_RX_GPIO_CLK|RS232_USART_TX_GPIO_CLK, ENABLE);

//  /* 使能 UART 时钟 */
//  RCC_APB1PeriphClockCmd(RS232_USART_CLK, ENABLE);
//  
//  /* 连接 PXx 到 USARTx_Tx*/
//  GPIO_PinAFConfig(RS232_USART_RX_GPIO_PORT,RS232_USART_RX_SOURCE, RS232_USART_RX_AF);

//  /*  连接 PXx 到 USARTx__Rx*/
//  GPIO_PinAFConfig(RS232_USART_TX_GPIO_PORT,RS232_USART_TX_SOURCE,RS232_USART_TX_AF);

//  /* 配置Tx引脚为复用功能  */
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

//  GPIO_InitStructure.GPIO_Pin = RS232_USART_TX_PIN  ;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(RS232_USART_TX_GPIO_PORT, &GPIO_InitStructure);

//  /* 配置Rx引脚为复用功能 */
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_Pin = RS232_USART_RX_PIN;
//  GPIO_Init(RS232_USART_RX_GPIO_PORT, &GPIO_InitStructure);
//			
//  /* 配置串口RS232_USART 模式 */
//  USART_InitStructure.USART_BaudRate = RS232_USART_BAUDRATE;
//  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//  USART_InitStructure.USART_StopBits = USART_StopBits_1;
//  USART_InitStructure.USART_Parity = USART_Parity_No ;
//  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//  USART_Init(RS232_USART, &USART_InitStructure); 
//	
////	NVIC_Configuration();
//	/*配置串口接收中断*/
//	USART_ITConfig(RS232_USART, USART_IT_RXNE, ENABLE);
//	
//  USART_Cmd(RS232_USART, ENABLE);
//}

//void NVIC_Configuration(void)
//{
//  NVIC_InitTypeDef NVIC_InitStructure;

////#ifdef  VECT_TAB_RAM
//  /* Set the Vector Table base location at 0x20000000 */
////  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
////#else  /* VECT_TAB_FLASH  */
//  /* Set the Vector Table base location at 0x08000000 */
//  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
////#endif

//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);	
//	
//	//Usart1 NVIC 配置
//  NVIC_InitStructure.NVIC_IRQChannel = RS232_USART_IRQ;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级0
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级2
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器	

//// 	//TIM2 
//	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
//	NVIC_Init(&NVIC_InitStructure);	
//}

/*****************  发送一个字符 **********************/
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch )
{
	/* 发送一个字节数据到USART1 */
	USART_SendData(pUSARTx,ch);
		
	/* 等待发送完毕 */
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

/*****************  指定长度的发送字符串 **********************/
void Usart_SendStr_length( USART_TypeDef * pUSARTx, uint8_t *str,uint32_t strlen )
{
	unsigned int k = 0;
	
	do 
	{
		Usart_SendByte(pUSARTx, *(str + k));
		k++;
		
	} 
	while(k < strlen);
}

/*****************  发送字符串 **********************/
void Usart_SendString( USART_TypeDef * pUSARTx, uint8_t *str)
{
	unsigned int k = 0;
	
	do 
	{
		Usart_SendByte(pUSARTx, *(str + k));
		k++;
	} 
	while(*(str + k)!='\0');
}



/*********************************************END OF FILE**********************/


