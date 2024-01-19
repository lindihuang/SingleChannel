#include <stdio.h>
#include "stm32f4xx_gpio.h"
#include "ADS1256.h"
#include "bsp_led.h"
#include "bsp_SysTick.h"

/***************************
 *		Pin assign	   	
 *		STM32			ADS1256		
 *		GPIOB_Pin_11 		<--- DRDY
 *		GPIOB_Pin_12 		---> CS
 *		GPIOB_Pin_13 		---> SCK
 *		GPIOB_Pin_14(MISO)  <--- DOUT
 *		GPIOB_Pin_15(MOSI)  ---> DIN
 ***************************/	

/************�˿ڶ���************/ 
#define RCC_DRDY			RCC_AHB1Periph_GPIOB
#define PORT_DRDY			GPIOB
#define PIN_DRDY			GPIO_Pin_12
#define PORT_CS				GPIOA
#define PIN_CS				GPIO_Pin_4

#define ads1256_SPI_SCK_GPIO_PORT                 GPIOA
#define ads1256_SPI_SCK_AF                       	GPIO_AF_SPI1
#define ads1256_SPI_SCK_SOURCE                   	GPIO_PinSource5

#define ads1256_SPI_MISO_GPIO_PORT                GPIOA
#define ads1256_SPI_MISO_AF                       GPIO_AF_SPI1
#define ads1256_SPI_MISO_SOURCE                   GPIO_PinSource6

#define ads1256_SPI_MOSI_GPIO_PORT                GPIOA
#define ads1256_SPI_MOSI_AF                       GPIO_AF_SPI1
#define ads1256_SPI_MOSI_SOURCE                   GPIO_PinSource7

#define CS_0()						GPIO_ResetBits(PORT_CS, PIN_CS);
#define CS_1()						GPIO_SetBits(PORT_CS, PIN_CS);
#define ADS1256_DRDY  		(PORT_DRDY->IDR & PIN_DRDY)


void SPI1_Init(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	/****Initial SPI1******************/
 
	/* Enable SPI1 and GPIOB clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	/* Configure SPI1 pins: NSS, SCK, MISO and MOSI */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
 
	/* ���� ����Դ*/
  GPIO_PinAFConfig(ads1256_SPI_SCK_GPIO_PORT,ads1256_SPI_SCK_SOURCE,ads1256_SPI_SCK_AF);
  GPIO_PinAFConfig(ads1256_SPI_MISO_GPIO_PORT,ads1256_SPI_MISO_SOURCE,ads1256_SPI_MISO_AF);
  GPIO_PinAFConfig(ads1256_SPI_MOSI_GPIO_PORT,ads1256_SPI_MOSI_SOURCE,ads1256_SPI_MOSI_AF);
	
  /**
   * SPI1 configuration 
	 * SPI1����Ϊ����ȫ˫��
	 */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	
	/* ����SPI1Ϊ��ģʽ */
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master; 
	
	/* SPI���ͽ���8λ֡�ṹ */
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;

	/* ����ʱ���ڲ�����ʱ��ʱ��Ϊ�͵�ƽ */
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;

	/* ��һ��ʱ���ؿ�ʼ�������� */
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; 

	/* NSS�ź��������ʹ��SSIλ������ */
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; 

	/* ���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ8 */
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; 
	
	/* ���ݴ����MSBλ��ʼ */
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; 

	/* CRCֵ����Ķ���ʽ */
	SPI_InitStructure.SPI_CRCPolynomial = 7;   
	
	SPI_Init(SPI1, &SPI_InitStructure);
	
	/* Enable SPI1  */
	SPI_Cmd(SPI1, ENABLE);
}  

/*********************************************************************
 * ��������	��void Init_ADS1256_GPIO(void)
 * ��    ��	����ʼ��ADS1256 GPIO
 * ��    ��	��
 * Edieor		��
 * ��    ע	��Guo
 **********************************************************************/
void Init_ADS1256_GPIO(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_DRDY , ENABLE); 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE); 

	GPIO_InitStructure.GPIO_Pin = PIN_DRDY; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PORT_DRDY, &GPIO_InitStructure);
	
	//SPI1 NSS 
	CS_1();
	
	GPIO_InitStructure.GPIO_Pin = PIN_CS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(PORT_CS, &GPIO_InitStructure);
	
	CS_1();
	SPI1_Init();
}

/*-----------------------------------------------------------------//
 *	��    ��: ģ��SPIͨ��
 *	��ڲ���: 	���͵�SPI����
 *	���ڲ���: 	���յ�SPI����
 *	ȫ�ֱ���: 
 *	��    ע: ���ͽ��պ���
 *-----------------------------------------------------------------*/
unsigned char SPI_WriteByte(unsigned char TxData)
{
  unsigned char RxData = 0;
	
  while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(SPI1,TxData);
	Delay_us(25);
  while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE) == RESET);
  RxData = SPI_I2S_ReceiveData(SPI1);
	
  return RxData;
} 

/*-----------------------------------------------------------------
 *	��    �ܣ�ADS1256 д����
 *	��ڲ���: 
 *	���ڲ���: 
 *	ȫ�ֱ���: 
 *	��    ע: ��ADS1256�е�ַΪregaddr�ļĴ���д��һ���ֽ�databyte
 *-----------------------------------------------------------------*/
void ADS1256WREG(unsigned char regaddr,unsigned char databyte)
{ 
	CS_0();
	
	/* ��ADS1256_DRDYΪ��ʱ����д�Ĵ��� */
	while(ADS1256_DRDY);
	
	/* ��Ĵ���д�����ݵ�ַ */
	SPI_WriteByte(ADS1256_CMD_WREG | (regaddr & 0x0F));
	
	/* д�����ݵĸ���n-1 */
	SPI_WriteByte(0x00);
	
	/* ��regaddr��ַָ��ļĴ���д������databyte */
	SPI_WriteByte(databyte);
	
	CS_1();
}

/*********************************************************************
 * ��������	��void ADS1256_Init(void)
 * ��    ��	����ʼ��ADS1256
 * ��    ��	��
 * Edieor		��
 * ��    ע	��Guo
 **********************************************************************/
void ADS1256_Init(void)
{
	/*************��У׼****************/
  while(ADS1256_DRDY);
	
	LED_BLUE;	
	
	CS_0();
	
	SPI_WriteByte(ADS1256_CMD_SELFCAL);
	
	while(ADS1256_DRDY);
	
	CS_1();
	/**********************************/
 
	ADS1256WREG(ADS1256_STATUS,0x06);               // ��λ��ǰ��ʹ�û���
//	ADS1256WREG(ADS1256_STATUS,0x04);               // ��λ��ǰ����ʹ�û���

//	ADS1256WREG(ADS1256_MUX,0x08);                  // ��ʼ���˿�A0Ϊ��+����AINCOMλ��-��
	ADS1256WREG(ADS1256_ADCON,ADS1256_GAIN_4);                // �Ŵ���4
//	ADS1256WREG(ADS1256_ADCON,ADS1256_GAIN_8);                // �Ŵ���8 ��ѹ��
	ADS1256WREG(ADS1256_DRATE,ADS1256_DRATE_100SPS);  // ����100sps    

	ADS1256WREG(ADS1256_IO,0x00);               

	/*************��У׼****************/
	while(ADS1256_DRDY);
	
	CS_0();
	
	SPI_WriteByte(ADS1256_CMD_SELFCAL);
	
	while(ADS1256_DRDY);
	
	CS_1(); 
	/**********************************/
}

/*********************************************************************
 * ��������	��void ADS1256ReadData(void)
 * ��    ��	����ȡADֵ
 * ��    ��	��
 * Edieor		��
 * ��    ע	��Guo
 **********************************************************************/
signed int ADS1256ReadData(unsigned char channel)  
{
  unsigned int sum = 0;
	
	/* ��ADS1256_DRDYΪ��ʱ����д�Ĵ��� */
	while(ADS1256_DRDY);
	
	/* ����ͨ�� */
	ADS1256WREG(ADS1256_MUX,channel);		
	CS_0();
	
	SPI_WriteByte(ADS1256_CMD_SYNC);
	SPI_WriteByte(ADS1256_CMD_WAKEUP);	               
	SPI_WriteByte(ADS1256_CMD_RDATA);
	Delay_us(8);
	
  sum |= (SPI_WriteByte(0xff) << 16);
	sum |= (SPI_WriteByte(0xff) << 8);
	sum |= SPI_WriteByte(0xff);
	CS_1();

	if(sum > 0x7FFFFF)        // if MSB=1, 
	{
		sum -= 0x1000000;       // do 2's complement
	}
	
  return sum;
}

/*********************************************************************
 * ��������	��void reset(void)
 * ��    ��	����ADоƬ�ϵ���ȶ��临λһ�Σ���ֹADÿ�ο����Բ���
 * ��    ��	��
 * Edieor		��
 * ��    ע	��Guo
 **********************************************************************/
void reset(void)
{
  while(ADS1256_DRDY);
	LED_BLUE;	
	CS_0();
   
	SPI_WriteByte(ADS1256_CMD_REST);
	while(ADS1256_DRDY);
	CS_1();		
}


