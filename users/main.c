#include "stm32f10x.h"
#include "bsp.h"
//#include "I2C.h"
#include "app.h"
#include "main.h"
#include  "ads1115.h"
#include "oled.h"
#include "ads1115.h"
extern int robot_status;   //用以标识当前机器人的工作状态。0为待机，1为正常上电工作

#define STARTUP_TASK_STK_SIZE 128

/************************************************************/
//定义栈，供Task_Start任务使用
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
	OSTaskCreate(Task_Start, (void *)0, &startup_task_stk[STARTUP_TASK_STK_SIZE-1], STARTUP_TASK_PRIO);//建立任务
  
	OSTimeSet(0);//节拍计数器清零nn

	OSStart();	//操作系统开始工作

	return 0;
}



/**********************************************************************************/
/**********************************************************************************/
/**********************************************************************************/

