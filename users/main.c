#include "stm32f10x.h"
#include "bsp.h"
//#include "I2C.h"
#include "app.h"
#include "main.h"
#include  "ads1115.h"
#include "oled.h"
#include "ads1115.h"
extern int robot_status;   //���Ա�ʶ��ǰ�����˵Ĺ���״̬��0Ϊ������1Ϊ�����ϵ繤��

#define STARTUP_TASK_STK_SIZE 128

/************************************************************/
//����ջ����Task_Start����ʹ��
OS_STK startup_task_stk[STARTUP_TASK_STK_SIZE];		  
 
/***********************************************************/
int main(void)
{
	float f=3.141592;
	u16 temp_test=0;
	u16 vol_test=0;
	int i=0;
	SystemInit();
	BSP_Init();
	ADS1115_Init();
	OLED_Init();
	robot_status=0;
	
	OSInit();
	OSTaskCreate(Task_Start, (void *)0, &startup_task_stk[STARTUP_TASK_STK_SIZE-1], STARTUP_TASK_PRIO);//��������
  
	OSTimeSet(0);//���ļ���������nn

	OSStart();	//����ϵͳ��ʼ����

	return 0;
}



/**********************************************************************************/
/**********************************************************************************/
/**********************************************************************************/

