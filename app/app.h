#ifndef _APP_H
#define _APP_H

#include "stm32f10x.h"
#include "includes.h"

#define TICKTASK_STK_SIZE 128
#define TASK_BATTERY_STK_SIZE 128
#define TASK_POWEER_STK_SIZE 128


#define STARTUP_TASK_PRIO 10
#define TICKTASK_PRIO 11

#define POWER_ON_PRIO 13
#define POWER_OFF_PRIO 14
#define TASK_BATTERY_PRIO 16

static u16 comp_value;    //p波的占空比


/******************************declare struction variable************************************/
union float_trans
{
	float float_form;
	u8 u8_form[4];
};


typedef struct POS_PID
{
	float P;
	float I;
	float D;
	u16 Current_Err;
	u16 Pre_Err;
	u16 PrePre_Err;
}POS_PID;

typedef struct Kalman_Parameter
{
	float Noise_Variance;             // 2
	float Deduce_Variance;            // 0.5
	float R_Variance;                 // 0
	float Result_Variance;            //
  float K;                          //卡氏增益	
	u16 Deduce_Value;
	u16 Result_Value;
}Kalman_Parameter;


/******************************declare extern variable*************************************/
extern u8 Temperature;
extern u16 I_Motor;
extern u16 I_Power;
extern u16 Current_Pos;
extern int CAN_Send_Flag;
extern int CAN_Receive_Flag;
extern u16 Aim_Position;
extern POS_PID pos_pid;

/******************************declare function*************************************/

void Data_Convert(void);
u16 Kalman_Filter(u16 input);
void delay_us(u16 nus);
void ADCtoVoltage(float *Voltage);
void ADCtoTemparature(float *Temperature);
void temperature_adc_switch(int chananel);
void voltage_adc_switch(int chananel);
void  OS_CPU_SysTickInit (void);
u16 Get_Adc_Average(u8 ch,u8 times);
/******************************declare task************************************/
void Task_Start(void *p_arg);
void TickTask(void *p_arg);
void BatteryTask(void *p_arg);
void Power_ON_Task(void *p_arg);
void Power_OFF_Task(void *p_arg);


#endif

