#include "port.h"

void EnterCriticalSection(void)
{
	//�ر�ȫ���ж�
	__disable_irq();
}

void ExitCriticalSection(void)
{
	//����ȫ���ж�
	__enable_irq();
}






