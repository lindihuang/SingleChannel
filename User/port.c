#include "port.h"
/* ----------------------- Variables ----------------------------------------*/

/* ----------------------- Start implementation -----------------------------*/
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
