#ifndef __MODBUS_H
#define	__MODBUS_H

#include "stm32f4xx.h"
#include <stdio.h>
#include "mb.h"

union LianHeTi
{
	char  value[4];
  float d;
};

extern union LianHeTi test_data1;
extern union LianHeTi test_data2;
extern union LianHeTi test_data3;
extern union LianHeTi test_data4;
extern union LianHeTi test_data5;
extern union LianHeTi test_data6;
extern union LianHeTi test_data7;
extern union LianHeTi test_data8;
extern union LianHeTi test_data9;
extern union LianHeTi test_data10;
extern union LianHeTi test_data11;
extern union LianHeTi test_data12;
extern union LianHeTi test_data13;
extern union LianHeTi test_data14;
extern union LianHeTi test_data27;
extern union LianHeTi test_data27;


extern uint16_t GS_Flag;             //光栅是否检测到障碍物的标志：0，没有；1，有
extern uint16_t Mode_Flag_1;
extern uint16_t jieduan_Flag_1;
extern uint16_t Start_Over_Flag_1;

extern UCHAR    ucSCoilBuf[14] ;
extern USHORT   usSRegInBuf[100];
extern USHORT  usSRegHoldBuf[250];

void Step(void);
void AUTO_ZERO(void);
void  Relay_control(void);


#endif /* __MODBUS_H */


