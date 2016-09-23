 #include "stm32f10x.h"
 #include "app.h"
// #include "stdio.h"
 #include "bsp.h"
 #include "oled.h"
// #include "main.h"

extern int mail;
static int time=0;
int robot_status=0;    //机器人的状态，0为待机态，1为正常工作态，2为低压状态
extern OS_EVENT  *OS_Mbox_Tick;
extern OS_EVENT  *OS_Mbox_PowerOff;
extern OS_EVENT  *OS_Mbox_PowerOn;

u8 OS_Mbox_Tick_Buf[2]={0};
u8 hand=0;

void NMI_Handler(void)
{
 
}


void HardFault_Handler(void)
{
 
  while (1)
  {
		OLED_Printf(0,0,"HardFault");
  }
}

void MemManage_Handler(void)
{
 
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
 
  while (1)
  {
  }
}


void UsageFault_Handler(void)
{
  
  while (1)
  {
  }
}


void SVC_Handler(void)
{
}


void DebugMon_Handler(void)
{
}












void SysTick_Handler(void)
{
  OSIntEnter();

	
  OSMboxPost(OS_Mbox_Tick,&mail);	
	
	OSTimeTick();
	OS_Sched();
  OSIntExit(); 
}



void EXTI9_5_IRQHandler(void)
{
	OSIntEnter();
	

	EXTI_ClearITPendingBit(EXTI_Line9);
	OSIntExit();
}




