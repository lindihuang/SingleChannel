#include "mb.h"
#include "stm32f4xx.h"
#include "bsp_led.h" 
#include "relay.h"
#include "modbus.h"
#include "Timer5.h"
#include "mbrtu.h"
#include "main.h"
#include "bsp_SysTick.h"
#include "new.h"


float  Timer5_cout;
float  Timer4_cout;
float  Timer3_cout;
float  Timer1_cout;

uint16_t GS_Flag=0;

/* -----------------------Slave Defines -------------------------------------*/
#define S_DISCRETE_INPUT_START        1		//离散输入启动
#define S_DISCRETE_INPUT_NDISCRETES   16	
#define S_COIL_START                  1
#define S_COIL_NCOILS                 104
#define S_REG_INPUT_START             1
#define S_REG_INPUT_NREGS             100
#define S_REG_HOLDING_START           1
#define S_REG_HOLDING_NREGS           100

/* 从机模式：在保持寄存器中，各个地址对应的功能定义 */
#define          S_HD_RESERVE                     0		  //保留
#define          S_HD_CPU_USAGE_MAJOR             1     //当前CPU利用率的整数位
#define          S_HD_CPU_USAGE_MINOR             2     //当前CPU利用率的小数位

/* 从机模式：在输入寄存器中，各个地址对应的功能定义 */
#define          S_IN_RESERVE                     0		  //保留

/* 从机模式：在线圈中，各个地址对应的功能定义 */
#define          S_CO_RESERVE                     2		  //保留

/* 从机模式：在离散输入中，各个地址对应的功能定义 */
#define          S_DI_RESERVE                     1		  //保留

/*------------------------Slave mode use these variables----------------------*/
/**
 * Slave mode:DiscreteInputs variables
 *
 * 从模式:离散输入变量 
 */
USHORT   usSDiscInStart = S_DISCRETE_INPUT_START;

#if S_DISCRETE_INPUT_NDISCRETES%8
	UCHAR    ucSDiscInBuf[S_DISCRETE_INPUT_NDISCRETES/8+1];
#else
	UCHAR    ucSDiscInBuf[S_DISCRETE_INPUT_NDISCRETES/8];
#endif

/**
 * Slave mode:Coils variables
 *
 * 从模式:线圈变量 
 */
USHORT   usSCoilStart = S_COIL_START;

#if S_COIL_NCOILS%8
	UCHAR    ucSCoilBuf[S_COIL_NCOILS/8+1];	//N*=输出数量/8 如果余数不等于0  那么N=N+1
#else
	UCHAR    ucSCoilBuf[14];
#endif

/**
 * Slave mode:InputRegister variables 
 *
 * 从模式:输入寄存器变量
 */
USHORT   usSRegInStart = S_REG_INPUT_START;
USHORT   usSRegInBuf[100];  
//float   usSRegInBuf[S_REG_INPUT_NREGS];

/**
 * Slave mode:HoldingRegister variables
 *
 * 从模式:保持寄存器变量 
 */
USHORT   usSRegHoldStart = S_REG_HOLDING_START;
USHORT   usSRegHoldBuf[250];

//UCHAR    ucSCoilBuf[8] ;

/******************************输入寄存器回调函数************************************************
 * 函数定义: eMBErrorCode eMBRegInputCB(UCHAR * pucRegBuffer,USHORT usAddress,USHORT usNRegs)
 * 描    述:输入寄存器相关的功能（读、连续读）
 * 功    能:读取输入寄存器，对应功能码是 04 eMBFuncReadInputRegister
 * 入口参数: 
 *				pucRegBuffer		: 回调函数将Modbus寄存器的当前值写入的缓冲区
 *				usAddress				: 寄存器的起始地址，输入寄存器的地址范围是1-65535。
 *				usNRegs  				: 寄存器数量
 * 出口参数：eMBErrorCode : 这个函数将返回的错误码
 * 备    注：Editor：Armink 2010-10-31    Company: BXXJS   3区
 ************************************************************************************************/
eMBErrorCode eMBRegInputCB(UCHAR * pucRegBuffer,USHORT usAddress,USHORT usNRegs)
{
	eMBErrorCode    eStatus = MB_ENOERR;//无错误
	int             iRegIndex;					//寄存器指针
	USHORT *        pusRegInputBuf;			//公共寄存器输入buf
	UCHAR           REG_INPUT_START;		//寄存器输入开始
	UCHAR           REG_INPUT_NREGS;		//寄存器输入数量
	UCHAR           usRegInStart;
	  
	pusRegInputBuf = usSRegInBuf;
	REG_INPUT_START = S_REG_INPUT_START;
	REG_INPUT_NREGS = S_REG_INPUT_NREGS;
	usRegInStart = usSRegInStart;
	
	if((usAddress >= REG_INPUT_START) && (usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS))
	{
		iRegIndex = (int)(usAddress - usRegInStart);
	
		while(usNRegs > 0)
		{
			*pucRegBuffer++ = (unsigned char)(pusRegInputBuf[iRegIndex] >> 8);
			*pucRegBuffer++ = (unsigned char)(pusRegInputBuf[iRegIndex] & 0xFF);

			iRegIndex++;
			usNRegs--;
		}
	}
	else
	{
		eStatus = MB_ENOREG;
	}

	return eStatus;
}

/******************************保持寄存器回调函数****************************************************************************
 * 函数定义: eMBErrorCode eMBRegHoldingCB(UCHAR * pucRegBuffer,USHORT usAddress,USHORT usNRegs,eMBRegisterMode eMode)
 * 描    述：保持寄存器相关的功能（读、连续读、写、连续写）
 * 功    能：对应功能码有：
 * 					 06 写保持寄存器 				eMBFuncWriteHoldingRegister 
 *					 16 写多个保持寄存器 		eMBFuncWriteMultipleHoldingRegister
 *					 03 读保持寄存器 				eMBFuncReadHoldingRegister
 *					 23 读写多个保持寄存器 	eMBFuncReadWriteMultipleHoldingRegister
 * 入口参数：
 * 					pucRegBuffer : 如果需要更新用户寄存器数值，这个缓冲区必须指向新的寄存器数值。
 *                         如果协议栈想知道当前的数值，回调函数必须将当前值写入这个缓冲区
 * 					usAddress    : 寄存器的起始地址。
 *					usNRegs      : 寄存器数量
 *          eMode        : 如果该参数为eMBRegisterMode::MB_REG_WRITE，用户的应用数值将从pucRegBuffer中得到更新。
 *                         如果该参数为eMBRegisterMode::MB_REG_READ，用户需要将当前的应用数据存储在pucRegBuffer中
 * 出口参数：eMBErrorCode : 这个函数将返回的错误码
 * 备    注：Editor：Armink 2010-10-31    Company: BXXJS   4区
 ******************************************************************************************************************************/
eMBErrorCode eMBRegHoldingCB(UCHAR * pucRegBuffer,USHORT usAddress,USHORT usNRegs,eMBRegisterMode eMode)
{
	eMBErrorCode    eStatus = MB_ENOERR;
	int             iRegIndex;
	USHORT *        pusRegHoldingBuf;
	UCHAR           REG_HOLDING_START;
	UCHAR           REG_HOLDING_NREGS;
	UCHAR           usRegHoldStart;
      	
	pusRegHoldingBuf = usSRegHoldBuf;
	REG_HOLDING_START = S_REG_HOLDING_START;
	REG_HOLDING_NREGS = S_REG_HOLDING_NREGS;
	usRegHoldStart = usSRegHoldStart;
	
	if((usAddress >= REG_HOLDING_START) && (usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS))
	{
		iRegIndex = (int)(usAddress - usRegHoldStart);
		
		switch (eMode)
		{
			/**
			 * Pass current register values to the protocol stack.
			 * 将当前寄存器值传递给协议栈。
			 */
			case MB_REG_READ:
					while(usNRegs > 0)
					{
						/** 将存储进来的寄存器值存储到原来寄存器值得后面 */
						*pucRegBuffer++ = (unsigned char)(pusRegHoldingBuf[iRegIndex] >> 8);
						/** 将存储进来的寄存器值标记成一个单元 0xFF表示单元标识符 */
						*pucRegBuffer++ = (unsigned char)(pusRegHoldingBuf[iRegIndex] & 0xFF);
						
						iRegIndex++;	//寄存器索引自加
						usNRegs--;		//寄存器数量自减
					}
					break;

			/** 
			 * Update current register values with new values from the
			 * protocol stack. 
			 */
			case MB_REG_WRITE:
					while(usNRegs > 0)
					{
						/** 更新保存在寄存器里的数据 */
						pusRegHoldingBuf[iRegIndex]  = *pucRegBuffer++ << 8;
						pusRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
						
						iRegIndex++;
						usNRegs--;
					}
					break;
				}
	}
	else
	{
		eStatus = MB_ENOREG;
	}
	
	return eStatus;
}

/****************************线圈状态寄存器回调函数*******************************************************************
 * 函数定义: eMBErrorCode eMBRegCoilsCB(UCHAR * pucRegBuffer,USHORT usAddress,USHORT usNCoils,eMBRegisterMode eMode)
 * 描    述：线圈状态寄存器相关的功能（读、连续读、写、连续写）
 * 功    能：
 * 对应功能码有：
 *						01 读线圈 			eMBFuncReadCoils
 *						05 写线圈 			eMBFuncWriteCoil
 *						15 写多个线圈 	eMBFuncWriteMultipleCoils
 * 入口参数：
 *				pucRegBuffer : 位组成一个字节，起始寄存器对应的位处于该字节pucRegBuffer的最低位LSB。
 *                       如果回调函数要写这个缓冲区，没有用到的线圈（例如不是8个一组的线圈状态）
 *											 对应的位的数值必须设置位0。
 *				usAddress    : 第一个线圈地址。
 *				usNCoils     : 请求的线圈个数
 *        eMode        : 如果该参数为eMBRegisterMode::MB_REG_WRITE，用户的应用数值将从pucRegBuffer中得到更新。
 *                       如果该参数为eMBRegisterMode::MB_REG_READ，用户需要将当前的应用数据存储在pucRegBuffer中
 * 出口参数：eMBErrorCode : 这个函数将返回的错误码
 * 备    注：Editor：Armink 2010-10-31    Company: BXXJS   0区
 **********************************************************************************************************************/
eMBErrorCode eMBRegCoilsCB(UCHAR * pucRegBuffer,USHORT usAddress,USHORT usNCoils,eMBRegisterMode eMode)
{
	eMBErrorCode  eStatus = MB_ENOERR;
	int           iRegIndex ,iRegBitIndex ,iNReg;

	iNReg =  usNCoils / 8 + 1;        //占用寄存器数量

	if((usAddress >= S_COIL_START) && (usAddress + usNCoils <= S_COIL_START + S_COIL_NCOILS))
	{
		iRegIndex    = (int)(usAddress - S_COIL_START) / 8 ;    //每个寄存器存8个
		iRegBitIndex = (int)(usAddress - usSCoilStart) % 8 ;	  //相对于寄存器内部的位地址
		
		switch(eMode)
		{
			/**
			 * Pass current coil values to the protocol stack.
			 * 将电流线圈值传递给协议栈。
			 */
			case MB_REG_READ:	//读取寄存器的值并传递给协议栈
					while(iNReg > 0)
					{
						*pucRegBuffer++ = xMBUtilGetBits(&ucSCoilBuf[iRegIndex++] , iRegBitIndex , 8);
						iNReg --;	//减掉占用的寄存器
					}
					pucRegBuffer --;
					usNCoils = usNCoils % 8;                         //余下的线圈数	
					*pucRegBuffer = *pucRegBuffer << (8 - usNCoils); //高位补零
					*pucRegBuffer = *pucRegBuffer >> (8 - usNCoils);
					break;

			/** 
			 * Update current coil values with new values from the
			 * protocol stack. 
			 * 用协议栈中的新值更新当前线圈值。
			 */
			case MB_REG_WRITE:	//更新寄存器的值
					while(iNReg > 1)									 //最后面余下来的数单独算
					{
						xMBUtilSetBits(&ucSCoilBuf[iRegIndex++],iRegBitIndex ,8,*pucRegBuffer++);
						iNReg--;
					}
					usNCoils = usNCoils % 8;//余下的线圈数
					
					if (usNCoils != 0)      //xMBUtilSetBits方法 在操作位数量为0时存在bug
					{
						xMBUtilSetBits(&ucSCoilBuf[iRegIndex++],iRegBitIndex, usNCoils,
						*pucRegBuffer++);
					}
					break;
			}
	}
	else
	{
		eStatus = MB_ENOREG;	//非法注册地址
	}
	
	return eStatus;
}

/****************************离散输入寄存器回调函数*******************************************************************
 * 函数定义: eMBErrorCode eMBRegDiscreteCB(UCHAR * pucRegBuffer,USHORT usAddress,USHORT usNDiscrete)
 * 功    能：读取离散寄存器，
 * 对应功能码有：
 * 						02 读离散寄存器 eMBFuncReadDiscreteInputs
 * 入口参数：
 *				pucRegBuffer : 用当前的线圈数据更新这个寄存器，起始寄存器对应的位处于该字节pucRegBuffer的最低位LSB。
 *                       如果回调函数要写这个缓冲区，没有用到的线圈（例如不是8个一组的线圈状态）
 *											 对应的位的数值必须设置为0。
 *				usAddress    : 离散输入的起始地址
 *				usNDiscrete  : 离散输入点数量
 * 出口参数：eMBErrorCode : 这个函数将返回的错误码
 * 备    注：Editor：Armink 2010-10-31    Company: BXXJS      1区
 *********************************************************************************************************************/
eMBErrorCode eMBRegDiscreteCB(UCHAR * pucRegBuffer,USHORT usAddress,USHORT usNDiscrete)
{
  eMBErrorCode   eStatus = MB_ENOERR;
	int             iRegIndex , iRegBitIndex , iNReg;
	UCHAR *         pucDiscreteInputBuf;
	UCHAR           DISCRETE_INPUT_START;
	UCHAR           DISCRETE_INPUT_NDISCRETES;
	UCHAR           usDiscreteInputStart;
	iNReg =  usNDiscrete / 8 + 1;        //占用寄存器数量
  
	pucDiscreteInputBuf = ucSDiscInBuf;
	DISCRETE_INPUT_START = S_DISCRETE_INPUT_START;
	DISCRETE_INPUT_NDISCRETES = S_DISCRETE_INPUT_NDISCRETES;
	usDiscreteInputStart = usSDiscInStart;	//离散输入开始
	pucDiscreteInputBuf[0] = 0x07;	//公共离散输入buf 从0x07开始存-0xF5结束
	
	if((usAddress >= DISCRETE_INPUT_START) && (usAddress + usNDiscrete <= DISCRETE_INPUT_START + DISCRETE_INPUT_NDISCRETES))
	{
		iRegIndex    = (int)(usAddress - usDiscreteInputStart) / 8 ;    //每个寄存器存8个
		iRegBitIndex = (int)(usAddress - usDiscreteInputStart) % 8 ;	  //相对于寄存器内部的位地址
		
		while(iNReg > 0)
		{
			*pucRegBuffer++ = xMBUtilGetBits(&pucDiscreteInputBuf[iRegIndex++],iRegBitIndex,8);
			iNReg --;
		}
		pucRegBuffer --;
		usNDiscrete = usNDiscrete % 8;                      //余下的线圈数
		*pucRegBuffer = *pucRegBuffer << (8 - usNDiscrete); //高位补零
		*pucRegBuffer = *pucRegBuffer >> (8 - usNDiscrete);
	}
	else
	{
		eStatus = MB_ENOREG;
	}

	return eStatus;
}

/**********************************************************************************
 * 函数定义: void Step(void)
 * 描    述:光栅检测
 * ucSCoilBuf[2] = 0x0   (0        0            0         0)
 * 通道地址：             20       19           18
 * 功    能：读取离散寄存器，对应功能码有：02 读离散寄存器 eMBFuncReadDiscreteInputs
 * 备    注：
 ***********************************************************************************/
union LianHeTi test_data27;

void Step(void)
{
	/* 20206.17修改  02=0时对光栅标志位进行清0操作防止光栅时间有问题 */ 
	if(!(ucSCoilBuf[0] & 0x02))
	{
		/** 清除TIM3的中断挂起位 */
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		
		/** 禁用指定的TIM3中断 */
		TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
		
		/** 设置TIM3计数器寄存器值 */
		TIM_SetCounter(TIM3, 0);
		
		Timer3_cout = 0;
	}
	/* 20206.17修改结束 */

//	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)==1)
	
	/**
	 * 2021-3-19：光栅信号线改为常闭线，即信号为1时表示正常状态，
	 * 信号为0时表示光栅失效或光栅检测到障碍物 
	 */
	if(ucSCoilBuf[5] & 0x01)
	{
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11) == 1)
		{
		 ucSCoilBuf[8] &= 0xef;             //没有障碍物
		}		
		else
		{	
			ucSCoilBuf[8] |= 0x10;            //有障碍物
		}
		
		if((ucSCoilBuf[0] & 0x02))
		{
			/* 设备0_读写00003    判断循环开关 */
			if(ucSCoilBuf[0] & 0x04)
			{
				/** 
				 * 2021-3-19：光栅信号线改为常闭线，即信号为1时表示正常状态，
				 * 信号为0时表示光栅失效或光栅检测到障碍物 
				 * 
				 * 检测到障碍物
				 */
				if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11) == 0)
				{ 
					ucSCoilBuf[8] |= 0x10;//有障碍物
					relay11_OFF;					//优先关闭动作1
					ucSCoilBuf[4] &= 0xf7;//优先关闭动作1
					ucSCoilBuf[0] |= 0x01; 
					ucSCoilBuf[0] &= 0xf9; 
					ucSCoilBuf[7] &= 0xfe;//参数准备状态归零
				}
			}
			else
			{
				TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
				TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
//				printf("Timer3_cout:::%f \n",Timer3_cout);
				TIM_Cmd(TIM3, ENABLE);
				test_data27.value[0] = usSRegHoldBuf[34];
				test_data27.value[1] = (usSRegHoldBuf[34] >> 8);
				test_data27.value[2] = usSRegHoldBuf[33];
				test_data27.value[3] = (usSRegHoldBuf[33] >> 8);
				test_data27.d = test_data27.d*10;
				
				/* 检测到障碍物 */
				if((Timer3_cout <= test_data27.d) && (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11) == 0) && (ucSCoilBuf[5] & 0x01))
				{
					ucSCoilBuf[8] |= 0x10;//有障碍物
					relay11_OFF;					//优先关闭动作1
					ucSCoilBuf[4] &= 0xf7;//优先关闭动作1
					ucSCoilBuf[0] |= 0x01; 
					ucSCoilBuf[0] &= 0xf9; 
					ucSCoilBuf[7] &= 0xfe;//参数准备状态归零
					TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
					TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
					TIM_SetCounter(TIM3, 0);
					Timer3_cout = 0;					
				}
				
				if(Timer3_cout > test_data27.d)
				{
					TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
					TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
					TIM_SetCounter(TIM3, 0);
				}
			}
		}
	}
	else
	{
		ucSCoilBuf[8] &= 0xef;    //没有障碍物
	}
}
 
/**********************************************************************************
 * 函数定义: void Press_Set(void)
 * 描    述：非测试过程继电器控制
 * 功    能：
 * 备    注：Editor：
 **********************************************************************************/
void  Relay_control(void)
{
	if((ucSCoilBuf[3] & 0x01))
	{
		relay1_ON;
	}
	else 
	{
		relay1_OFF;
	}
	
	if((ucSCoilBuf[3] & 0x02))
	{
		relay2_ON;
	}
	else 
	{
		relay2_OFF;
	}
	
	if((ucSCoilBuf[3] & 0x04))
	{
		relay3_ON;
	}
	else 
	{
		relay3_OFF;
	}
	
	if((ucSCoilBuf[3] & 0x08))
	{
		relay4_ON;
	}
	else 
	{
		relay4_OFF;
	}
	
	if((ucSCoilBuf[3] & 0x10))
	{
		relay5_ON;
	}
	else 
	{
		relay5_OFF;
	}
	
	if((ucSCoilBuf[3] & 0x20))
	{
		relay6_ON;
	}
	else 
	{
		relay6_OFF;
	}
	
	if((ucSCoilBuf[3] & 0x40))
	{
		relay7_ON;
	}
	else 
	{
		relay7_OFF;
	}
	
	if((ucSCoilBuf[3] & 0x80))
	{
		relay8_ON;
	}
	else 
	{
		relay8_OFF;
	}
	
	if((ucSCoilBuf[4] & 0x08))
	{
		relay11_ON;
	}
	else 
	{
		relay11_OFF;
	}
	
	if((ucSCoilBuf[4] & 0x10))
	{
		relay12_ON;
	}
	else 
	{
		relay12_OFF;
	}
	
	if((ucSCoilBuf[4] & 0x20))
	{
		relay13_ON;
	}
	else 
	{
		relay13_OFF;
	}
	
	if(ucSCoilBuf[4] & 0x40)
	{
		relay14_ON;
	}
	else 
	{
		relay14_OFF;
	}
	
	if(ucSCoilBuf[4] & 0x80)
	{
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
		TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
		TIM_Cmd(TIM5, ENABLE);
		relay15_ON;
	}
	else 
	{
		relay15_OFF;
	}
	
	if(ucSCoilBuf[5] & 0x01)
	{
		relay16_ON;
	}
	else 
	{
		relay16_OFF;
	}
}

/**********************************************************************************
 * 函数定义: void AUTO_ZERO(void)
 * 描    述：自动调零
 * 功    能：
 * 备    注：Editor：
 **********************************************************************************/
uint16_t time = 0;  //自动调零扫描次数

void AUTO_ZERO(void)
{
	/** 设备0_读写00024=1 */
	if(ucSCoilBuf[8] & 0x01)
	{
		time++;
		
		if(time > 2)
		{
			time = 0;
			Setzero_Init();
			time = 0;
			ucSCoilBuf[8] &= 0xf0;
		}
	}
}


