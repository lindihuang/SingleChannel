#include "modbus.h"
#include "data_processing.h" 
#include "mb.h"
#include "bsp_led.h"  
#include "bsp_SysTick.h"
#include "relay.h"

int AD_10;
int AD_100;
int measure_AD; 											//标准AD值
int Adc[4] = {0};
int measure_AD1 = 3890;

int8_t sampling_number = 0;  					//统计取样个数
int32_t deviation[4] = {0};  					//调零偏移量
int64_t channel[4][storage_number]; 	//将读取的部分ad值存储在这个数组中

float pressure_10;
float pressure_100;
float range_1001;
float range_101 = 0.1162655;					//量程
float mole_coeffi = -0.00000000030936;//分子系数

union LianHeTi test_data1;
union LianHeTi test_data20;
union LianHeTi test_data21;
union LianHeTi test_data22;
union LianHeTi test_data23;
union LianHeTi test_data24;
union LianHeTi test_data25;
union LianHeTi test_data26;

const int32_t init_value[4] = {3890,0,0,0};  //mpm270 大气压ad值

/*******************************************************************************
 * @Function		: long double smoothing(int64_t weilvadzhi,uint8_t channels)
 * @Description	: 滤波函数
 * @Input				:	 	
 * @Output			: 
 * @Others			:
 * @Author			: Jinpeng.Guo
 * @Version			: V1.0
 * @Date				: 2023-11-07
 * @Brief 			: 
*******************************************************************************/
long double smoothing(int64_t weilvadzhi,uint8_t channels)
{
	int8_t j;
	int64_t adzhi1;
	long double result;	

	adzhi1 = weilvadzhi;

	if(sampling_number < storage_number)
	{
		for(j = sampling_number; j < storage_number; j++)
		{
			channel[channels][j] = adzhi1;
		}
		sampling_number++;
	}
	else
	{
		/* 存储数组a向前移动一位 */
		for(j = 0; j < storage_number-1; j++)  
		{
			channel[channels][j] = channel[channels][j + 1];
		}

		channel[channels][j] = adzhi1;
	}
	
	/* 高斯滤波 */
	result = channel[channels][11]*0.16		+ channel[channels][10]*0.1536
				 + channel[channels][9]*0.1420	+ channel[channels][8]*0.1262
				 + channel[channels][7]*0.1078	+ channel[channels][6]*0.0886
				 + channel[channels][5]*0.0696	+ channel[channels][4]*0.0534
				 + channel[channels][3]*0.0392	+ channel[channels][2]*0.0276
				 + channel[channels][1]*0.0192	+ channel[channels][0]*0.0128;  

//	printf(" %d a[0]=%d %d %d %d %d %d ",sampling_number,a[0],a[1],a[2],a[3],a[4],a[5]);
	return result;
}

/*******************************************************************************
 * @Function		: void Setzero_Init(void)
 * @Description	: MPM270程序启动初始调零
 * @Input				:	 	
 * @Output			: 
 * @Others			:
 * @Author			: Jinpeng.Guo
 * @Version			: V1.0
 * @Date				: 2023-11-07
 * @Brief 			: 
*******************************************************************************/
void Setzero_Init(void)
{
	long ulResult;
	long ulSum=0;
	int i;

	for(i = 0; i < 10; i++)
	{
		ulResult = ADS1256ReadData(ADS1256_MUXP_AIN0 | ADS1256_MUXN_AIN1);	
		
		printf(" %ld 校零操作成功！\n",ulResult);
		
		if(i >= 9)				
		{
			ulSum = ulSum + ulResult ;
		}
		Delays1(0xffff);			
	}
	ulResult = ulSum / 1;
	deviation[0] = ulResult - measure_AD1;
	
	printf("%d %ld \n",deviation[0],ulResult);

	Delays1(0xffff);
}

/*******************************************************************************
 * @Function		: void set_zero(void)
 * @Description	: 手动按键调零
 * @Input				:	 	
 * @Output			: 
 * @Others			:
 * @Author			: Jinpeng.Guo
 * @Version			: V1.0
 * @Date				: 2023-11-07
 * @Brief 			: 
*******************************************************************************/
void set_zero(void)
{
	int i;
	long ulResult;
	long ulSum=0;

	if(ucSCoilBuf[2] & 0x80)
	{
		for(i = 0; i < 10; i++)
		{
			ulResult = ADS1256ReadData(ADS1256_MUXP_AIN0 | ADS1256_MUXN_AIN1);	
			
			printf(" %ld 校零操作成功！\n",ulResult);
			
			if(i >= 9)				
			{
				ulSum = ulSum + ulResult ;
			}
			Delays1(0xffff);			
		}
		ulResult = ulSum / 1;
		deviation[0] = ulResult - measure_AD;
		printf("deviation %d %ld \n",deviation[0],ulResult);

		Delays1(0xffff);
		LED_BLUE;
		ucSCoilBuf[2] &= 0x7f;  //关闭手动调零
	}
}

/*******************************************************************************
 * @Function		: void Acquisition_channel(void)
 * @Description	: 使能需要的交叉采集通道
 * @Input				:	 	
 * @Output			: 
 * @Others			:
 * @Author			: Jinpeng.Guo
 * @Version			: V1.0
 * @Date				: 2023-11-07
 * @Brief 			: 
*******************************************************************************/
void Acquisition_channel(char channel_0,char channel_1,char channel_2,char channel_3)  
{	
	char last=0;  //统计使能通道个数
	char first=0;  
	float Pressure;

	if(channel_0 == 1)
	{
		last++;
	}
	
	if(channel_1 == 1) 
	{
		last++;
	}
	if(channel_2 == 1) 
	{
		last++;
	}
	
	if(ucSCoilBuf[5] & 0x20)
	{
		test_data21.value[0] = usSRegHoldBuf[31];				//容积充气	
		test_data21.value[1] = (usSRegHoldBuf[31] >> 8);
		test_data21.value[2] = usSRegHoldBuf[30];
		test_data21.value[3] = (usSRegHoldBuf[30] >> 8);
		measure_AD = test_data21.d;//+deviation ;
	
		test_data23.value[0] = usSRegHoldBuf[25];				//直接测试
		test_data23.value[1] = (usSRegHoldBuf[25] >> 8);
		test_data23.value[2] = usSRegHoldBuf[24];
		test_data23.value[3] = (usSRegHoldBuf[24] >> 8);
		pressure_100 = test_data23.d*1000 ;
		pressure_10 = pressure_100*0.1F;
			
		test_data24.value[0] = usSRegHoldBuf[27];				//容积测试
		test_data24.value[1] = (usSRegHoldBuf[27] >> 8);
		test_data24.value[2] = usSRegHoldBuf[26];
		test_data24.value[3] = (usSRegHoldBuf[26] >> 8);
		AD_10 = test_data24.d ;//+deviation;
//		printf("AD_10:::%d \n",AD_10);
		
		test_data25.value[0] = usSRegHoldBuf[29];				//超微测试
		test_data25.value[1] = (usSRegHoldBuf[29] >> 8);
		test_data25.value[2] = usSRegHoldBuf[28];
		test_data25.value[3] = (usSRegHoldBuf[28] >> 8);
		AD_100 = test_data25.d ;//+deviation;
//		printf("AD_100:::%d \n",AD_100);
				
		measure_AD1 = measure_AD;
		range_101 = pressure_10 / (AD_10 - measure_AD);
		range_1001 = pressure_100 / (AD_100 - measure_AD);
		mole_coeffi = (range_1001 - range_101) / (AD_100 - AD_10);
//		printf("measure_AD1:::%d \n",measure_AD1);
//		printf("mole_coeffi:::%.20f \n",range_101);
//		printf("pressure_100:::%.20f \n",pressure_100);		
	}
  
	printf("  0 channel:%d %.3f\n",Adc[first],Pressure);
		
	if(channel_3 == 1) 
	{
		last++;
	}
	
	/* 如果通道0使能 */
  if(channel_0 == 1)   
	{
		/**
		 * 差分采集方式
		 * P = AIN0 ,N = AIN1 差分方式
		 */
		Adc[last-1] = ADS1256ReadData(ADS1256_MUXP_AIN0 | ADS1256_MUXN_AIN1); 
		Adc[first] = smoothing(Adc[first],0) - deviation[0];
		
		/**************************6.12 16:14*******************************/
		/* mpm270 0-100KPa  4X */
		Pressure = (Adc[first] - measure_AD1)*(range_101 + (Adc[first] - AD_10)*mole_coeffi) / 1000;    
			
		/* mpm270 0-350Kpa 4倍 */
//		Pressure = (Adc[first] + 2503)*(0.686087667 + (Adc[first]-48919)*0.00000000355789772) / 1000; 
			
		/* mpm270 0-600Kpa  8倍 */
//		Pressure = (Adc[first] + 2371)*(1.58339113 - (Adc[first]-42328)*0.0000000356142249) / 1000;  
		
//		test_data20.d = test_data1.d = Adc[first]*0.000000598;
		test_data20.d = test_data1.d = Adc[first];
		usSRegInBuf[18] = test_data20.value[3];
		usSRegInBuf[18] = (usSRegInBuf[18] << 8) + test_data20.value[2];
		usSRegInBuf[19] = test_data20.value[1];
		usSRegInBuf[19] = (usSRegInBuf[19] << 8) + test_data20.value[0];
		
		/* 限制异常气压 */
		if(Pressure < 1000)         
		{
			test_data1.d = Pressure; 
			
			usSRegInBuf[0] = test_data1.value[3];
			usSRegInBuf[0] = (usSRegInBuf[0] << 8) + test_data1.value[2];
			usSRegInBuf[1] = test_data1.value[1];
			usSRegInBuf[1] = (usSRegInBuf[1] << 8) + test_data1.value[0];
		}
		
//		printf("deviation %d  \n",deviation[0]);
	}
	
	if(channel_1 == 1)
	{
	  Adc[first] = ADS1256ReadData(ADS1256_MUXP_AIN2 | ADS1256_MUXN_AIN3);
		
		first++;
		Adc[first] = smoothing(Adc[first],1) - deviation[1];
    Pressure = (Adc[first] - 3890)*(0.1162655 + (Adc[first] - 89900)*(-0.00000000030936)) / 1000;
	  printf("  1 channel:%d %.3f\n",Adc[first],Pressure);	
	}
	
	if(channel_2 == 1)
	{
		Adc[first] = ADS1256ReadData(ADS1256_MUXP_AIN4 | ADS1256_MUXN_AIN5);
		first++;
		Adc[first] = smoothing(Adc[first],2) - deviation[2];
//		 Pressure = (Adc - 3890)*(0.1162655 + (Adc - 89900)*(-0.00000000030936)) / 1000;
		Pressure = Adc[first]*0.000000598;
		printf("  2 channel:%d %.3f",Adc[first],Pressure);	
	}
	
	if(channel_3 == 1)
	{
		Adc[first] = ADS1256ReadData(ADS1256_MUXP_AIN6 | ADS1256_MUXN_AIN7);
		first++;
		Adc[first] = smoothing(Adc[first],3) - deviation[3];
		Pressure = (Adc[first] - 3890)*(0.1162655 + (Adc[first] - 89900)*(-0.00000000030936)) / 1000;
		printf("  3 channel:%d %.3f",Adc[first],Pressure); 
	}
}

/*******************************************************************************
 * @Function		: void Acquisition_channel1(void)
 * @Description	: 使能需要的交叉采集通道
 * @Input				:	 	
 * @Output			: 
 * @Others			:
 * @Author			: Jinpeng.Guo
 * @Version			: V1.0
 * @Date				: 2023-11-07
 * @Brief 			: 
*******************************************************************************/
void Acquisition_channel1(char channel_0,char channel_1,char channel_2,char channel_3)  
{
	float Pressure;
	char first=0;
	
	/* 如果通道0使能 */
	if(channel_0 == 1)   
	{
		Delay_ms(1);
		/*差分采集方式*/
		Adc[first] = ADS1256ReadData(ADS1256_MUXP_AIN0 | ADS1256_MUXN_AIN1); //P = AIN0 ,N = AIN1 差分方式*/
		Delay_us(5);
		Adc[first] = ADS1256ReadData(ADS1256_MUXP_AIN0 | ADS1256_MUXN_AIN1);
		Adc[first] = smoothing(Adc[first],0) - deviation[0];
		Pressure = (Adc[first] - 3890)*(0.1162655 + (Adc[first] - 89900)*(-0.00000000030936)) / 1000;
	
		printf("  0 channel:%d %.3f",Adc[first],Pressure);
	}
	
	if(channel_1 == 1)
	{
		Delay_ms(1);
		first++;
	  Adc[first] = ADS1256ReadData(ADS1256_MUXP_AIN2 | ADS1256_MUXN_AIN3);
		Delay_us(5);
	  Adc[first] = ADS1256ReadData(ADS1256_MUXP_AIN2 | ADS1256_MUXN_AIN3);
		Adc[first] = smoothing(Adc[first],1) - deviation[1];
    Pressure = (Adc[first] - 3890)*(0.1162655 + (Adc[first] - 89900)*(-0.00000000030936)) / 1000;
	  printf("  1 channel:%d %.3f",Adc[first],Pressure);
	}
	
	if(channel_2 == 1)
	{
		Delay_ms(1);
		first++;
		Adc[first] = ADS1256ReadData(ADS1256_MUXP_AIN4 | ADS1256_MUXN_AIN5);
		Delay_us(5);
		Adc[first] = ADS1256ReadData(ADS1256_MUXP_AIN4 | ADS1256_MUXN_AIN5);
		Adc[first] = smoothing(Adc[first],2)-deviation[2];
//		Pressure = (Adc - 3890)*(0.1162655 + (Adc - 89900)*(-0.00000000030936)) / 1000;
		Pressure = Adc[first]*0.000000598;
		printf("  2 channel:%d %.3f",Adc[first],Pressure);
	}
	
	if(channel_3 == 1)
	{
		Delay_ms(1);
		first++;
		Adc[first] = ADS1256ReadData(ADS1256_MUXP_AIN6 | ADS1256_MUXN_AIN7);
		Delay_us(5);
		Adc[first] = ADS1256ReadData(ADS1256_MUXP_AIN6 | ADS1256_MUXN_AIN7);
		Adc[first] = smoothing(Adc[first],3) - deviation[3];
//    Pressure = (Adc[first] - 3890)*(0.1162655 + (Adc[first] - 89900)*(-0.00000000030936)) / 1000;
		Pressure = Adc[first]*0.000000598;
		printf("  3 channel:%d %.3f",Adc[first],Pressure); 
	}
}

/*******************************************************************************
 * @Function		: void Delays1(uint32_t x)
 * @Description	: 延时函数
 * @Input				:	 	
 * @Output			: 
 * @Others			:
 * @Author			: Jinpeng.Guo
 * @Version			: V1.0
 * @Date				: 2023-11-07
 * @Brief 			: 
*******************************************************************************/
void Delays1(uint32_t x)
{
	uint32_t nCount;
	
  for(nCount=x; nCount != 0; nCount--);
}

/*******************************************************************************
 * @Function		: void relay17_Config(void)
 * @Description	: 接近开关函数引脚定义
 * @Input				:	 	
 * @Output			: 
 * @Others			:
 * @Author			: Jinpeng.Guo
 * @Version			: V1.0
 * @Date				: 2023-11-07
 * @Brief 			: 
*******************************************************************************/
void relay17_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* 开启按键GPIO口的时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);
	
  /* 选择按键的引脚 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14; 
  
  /* 设置引脚为输入模式 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
  
  /* 设置引脚不上拉也不下拉 */
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	
  /* 使用上面的结构体初始化按键 */
	GPIO_Init(GPIOH, &GPIO_InitStructure);  
	
	/* 开启按键GPIO口的时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);
	
  /* 选择按键的引脚 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
	
  /* 设置引脚为输入模式 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
	
  /* 设置引脚不上拉也不下拉 */
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	
	GPIO_Init(GPIOH, &GPIO_InitStructure);  
}


/*********************************************END OF FILE**********************/



