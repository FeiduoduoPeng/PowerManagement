#include "stm32f10x.h"
#include "app.h"
#include "bsp.h"
#include "spi.h"
#include "stdio.h"
#include "includes.h"
#include "oled.h"

#define BAT_LOW_TOTAL 19.2f
#define BAT_LOW_THRESH 3.2f
#define TEMP_HIGH_THRESH 0.94f     //����ѹС��0.94vʱ�����¶ȵ���С��4kŷ�����¶ȸ���47��C

#define PC_OFF_WAIT 15

u8 err;
extern int robot_status;
extern u8 hand;
int mail=2;

float test=3.14159;
int low_vol[6]={0};                     //�͵�ѹʱ��
int high_temp[6]={0};                     //����ʱ��
	
	
	
	
/*****************************����ջ��������ʹ�ã����������������涨�壩*******************************/

OS_STK TickTask_stk[TICKTASK_STK_SIZE];														//ʱ������
OS_STK Task_Battery_stk[TASK_BATTERY_STK_SIZE];											//�ֱ�
OS_STK Task_PowerOn_stk[TASK_POWEER_STK_SIZE];											//�ֱ�
OS_STK Task_PowerOff_stk[TASK_POWEER_STK_SIZE];											//�ֱ�


#define Dead_Zone 10

/****************�ź�����ʱ���־�飬��Ϣ���䣬��Ϣ���е�����ͨ�ű���!!ȫ�ֱ���**************************/

OS_EVENT    		*OS_Mbox_Tick;
extern INT8U        	OS_Mbox_Tick_Buf[30]; 											//����ʱ�ӽ��ĵ��ж������ź�

OS_FLAG_GRP     			*OS_Flag_5VSensor;													//5V������״̬λ����Ϊ1
OS_FLAG_GRP     			*OS_Flag_12VSensor;													//12V������״̬λ����Ϊ1

OS_EVENT        			*OS_Mbox_Battery;
OS_EVENT        			*OS_Mbox_PowerOn;
OS_EVENT        			*OS_Mbox_PowerOff;




/********************************************************************************************************/
/****************************************������*******************************************************/
/****************************************������*******************************************************/
/****************************************������*******************************************************/
/********************************************************************************************************/
void Task_Start(void *p_arg)
{
	while(1)
	{
		OS_Mbox_Tick    = OSMboxCreate((void*) 0);											//��Ϣ���䣨��Ϣ���ݣ���ʼΪ�գ�
		OS_Mbox_Battery = OSMboxCreate((void*) 0);
		OS_Mbox_PowerOn = OSMboxCreate((void*) 0);
		OS_Mbox_PowerOff= OSMboxCreate((void*) 0);
//	  OS_Flag_5VSensor   = OSFlagCreate(0x0000,&err);								//5V��������־��
//	  OS_Flag_12VSensor  = OSFlagCreate(0x0000,&err);								//12V��������־��
			
		DISABLE_ANDROID; 
		DISABLE_CHASISS;
		DISABLE_PC;
		DISABLE_HEAD;
		
/*********************************����10ms�жϷ������************************************************/
		OS_CPU_SysTickInit();//ϵͳʱ��SysTick
    OSTaskCreate(TickTask, (void *)0, &TickTask_stk[TICKTASK_STK_SIZE-1], TICKTASK_PRIO);//ʱ�ӽ����ж�����
/*****************************************************************************************************/
		
		OSTaskCreate(BatteryTask, (void *)0, &Task_Battery_stk[TASK_BATTERY_STK_SIZE-1], TASK_BATTERY_PRIO);	
		OSTaskCreate(Power_ON_Task, (void *)0, &Task_PowerOn_stk[TASK_POWEER_STK_SIZE-1], POWER_ON_PRIO);							
		OSTaskCreate(Power_OFF_Task, (void *)0, &Task_PowerOff_stk[TASK_POWEER_STK_SIZE-1], POWER_OFF_PRIO);				
		OSTaskDel(STARTUP_TASK_PRIO); //ɾ������
	}
}


/*****************************************************************************/
void TickTask(void *p_arg)
{
	static int time=0;
	static int bat=0;
	while(1)
	{
		OSMboxPend(OS_Mbox_Tick,0,&err);
		//��ѯ���ִ��ؿ��ص�״̬
		
		if( GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9)==0)  //�ִ�����
		{		
			time++;
		}
		else //���뿪
		{
			time=0;
		}
		
		bat++;
		if(bat>=10 && robot_status)   //��������ȫ�������ϵ��ÿ100ms����һ�ε�ع�������
		{
			OSMboxPost(OS_Mbox_Battery,&mail);
			bat=0;
		}
/*************************************************************/
/*************************************************************/
		if(time>200 && robot_status==0)  //������ʱ�����2�룬����robot���ڴ���״̬��������Դ
		{
			time=0;
			OSMboxPost(OS_Mbox_PowerOn,&mail);  //�����������ֵĵ�Դ
		}
		if(time>200 && robot_status==1)  //������ʱ�����2�룬����robot���ڹ���״̬���رյ�Դ
		{
			time=0;
			OSMboxPost(OS_Mbox_PowerOff,&mail);  //�����ػ�����
		}
		
		
	}
}


void BatteryTask(void *p_arg)
{
	u8 can_test[5]={0};
	union float_trans fti;
	float vol_cell[6]={0};
	float temp_cell[6]={0};
	int c=0;
	int i=0;

	while(1)
	{
		OSMboxPend(OS_Mbox_Battery,0,&err);
		
	
		OLED_Printf(0,0,"running...");	
	
				//��ѯ��صĵ�ѹ���¶�
		for(c=0;c<6;c++)
		{
//			OSTimeDlyHMSM(0,0,0,2000);
			temperature_adc_switch(c);
		  voltage_adc_switch(c);
			OSTimeDlyHMSM(0,0,0,10);
			vol_cell[c]=11.2*Get_Adc_Average(ADC_Channel_4,10)*3.3/4096;
//			vol_cell[c]=(float)Get_Adc_Average(ADC_Channel_4,10)*3.3/4096;
			OSTimeDlyHMSM(0,0,0,10);
			temp_cell[c]=(float)Get_Adc_Average(ADC_Channel_3,10)*3.3/4096;
			
//			printf("vol%d: %f\n",c,vol_cell[c]);
//			printf("tem%d: %f\n",c,temp_cell[c]);
		}
		
		fti.float_form=vol_cell[5]; //������ܵ�ѹ������λ���ư�
		can_test[0]=fti.u8_form[0];
		can_test[1]=fti.u8_form[1];
		can_test[2]=fti.u8_form[2];
		can_test[3]=fti.u8_form[3];
		can_test[4]=fti.u8_form[0]+fti.u8_form[1]+fti.u8_form[2]+fti.u8_form[3];
		Can_Sent(can_test);
		
		for(i=5;i;i--)
		{
			vol_cell[i]=vol_cell[i]-vol_cell[i-1];
		}
		//��������¶ȳ��޵�ѹ����
		for(i=0;i<6;i++)
		{
//			OLED_Printf(0,0,"t%d: %f",i,temp_cell[i]);
//			OLED_Printf(1,0,"v%d: %f",i,vol_cell[i]);
//			OSTimeDlyHMSM(0,0,0,500);
			
			if(vol_cell[i]<BAT_LOW_THRESH) 
        {
				  low_vol[i]++;
				}  //100msһ�Σ�
			else {low_vol[i]=0;}
			
			if(temp_cell[i]<TEMP_HIGH_THRESH)
        {
				  high_temp[i]++;
				}//���������¶ȸ����¶��ٽ�ֵ���ػ�
			else {low_vol[i]=0;}
			
			if(low_vol[i]>200) { low_vol[i]=0;robot_status=2;OSMboxPost(OS_Mbox_PowerOff,&mail);}  //������20s��ؿ�С�ڵ�ѹ�ٽ�ֵ���ػ�
			if(high_temp[i]>200) {high_temp[i]=0;robot_status=2;OSMboxPost(OS_Mbox_PowerOff,&mail);}//������20s����¶ȸ����¶��ٽ�ֵ���ػ�
		}
	}
}


void Power_ON_Task(void *p_arg)
{
	float vol;
	while(1)
	{
		OSMboxPend(OS_Mbox_PowerOn,0,&err);
		
		/********************************����Ԥ����*****************************************/
		OLED_Printf(3,0,"taskON");
		POWER_ON;            //�򿪲����������Դ 
		
		BUZZER_ON;OSTimeDlyHMSM(0,0,0,150);BUZZER_OFF;OSTimeDlyHMSM(0,0,0,80);  //�����Դ������ʾ��
		BUZZER_ON;OSTimeDlyHMSM(0,0,0,150);BUZZER_OFF;OSTimeDlyHMSM(0,0,0,80);
		BUZZER_ON;OSTimeDlyHMSM(0,0,0,150);BUZZER_OFF;OSTimeDlyHMSM(0,0,0,80);
		BUZZER_ON;OSTimeDlyHMSM(0,0,0,150);BUZZER_OFF;
		
		voltage_adc_switch(5);//�����Լ죬������ܵ�ѹ
		vol=11.2*Get_Adc_Average(ADC_Channel_4,20)*3.3/4096;
		if(vol<BAT_LOW_TOTAL)  //����ܵ�ѹС������ֵ����ֹ����
		{
			OSTaskSuspend(TICKTASK_PRIO);
			OSTaskSuspend(TASK_BATTERY_PRIO);
			OSTaskSuspend(POWER_OFF_PRIO);
			while(1) {BUZZER_ON;POWER_OFF;}//�������������������� 
		}
		/**************************************************************************************/
		
		
	  //�������ϵ�
		ENABLE_ANDROID;
		OSTimeDlyHMSM(0,0,1,0);
		ENABLE_HEAD;
		OSTimeDlyHMSM(0,0,1,0);
		ENABLE_PC;           
		OSTimeDlyHMSM(0,0,1,0);
		ENABLE_CHASISS;
		
		{ PC_ON;  OSTimeDlyHMSM(0,0,0,100);  PC_OFF;	} //�㰴���Եĵ�Դ�������ԵĿ���/�ػ��ź�
		
		robot_status=1;
		OSTaskSuspend(OS_PRIO_SELF); //������������ɾ����������ֻ�ܽ���ػ�����
	}
}


void Power_OFF_Task(void *p_arg)
{
	INT8U err;
	while(1)
	{
		OSMboxPend(OS_Mbox_PowerOff,0,&err);
		
		/*********************************************�ػ�����Ԥ����********************************************/
		//����������Ĺ��𡣱�֤˳���ػ�
		err=OSTaskSuspend(TICKTASK_PRIO);
		if(err!=OS_ERR_NONE){OLED_Printf(3,0,"TaskSuspendErr");OSTaskSuspend(POWER_OFF_PRIO);}
		
		err=OSTaskSuspend(TASK_BATTERY_PRIO);
		if(err!=OS_ERR_NONE){OLED_Printf(3,0,"TaskSuspendErr");OSTaskSuspend(POWER_OFF_PRIO);}
		
		err=OSTaskSuspend(POWER_ON_PRIO);
		if(err!=OS_ERR_NONE){OLED_Printf(3,0,"TaskSuspendErr");OSTaskSuspend(POWER_OFF_PRIO);}
		
		OLED_Printf(3,0,"taskOFF");
		/*******************************************************************************************************/
		
//		OLED_Printf(3,0,"taskOFF");
		//�رհ�׿/���/���̹���
		DISABLE_ANDROID; 
		DISABLE_CHASISS;
		
		{ PC_ON;OSTimeDlyHMSM(0,0,0,100);  PC_OFF;} //�㰴���Եĵ�Դ�������ԵĿ���/�ػ��ź�
		
		BUZZER_ON;OSTimeDlyHMSM(0,0,0,80);BUZZER_OFF;OSTimeDlyHMSM(0,0,0,60);
		BUZZER_ON;OSTimeDlyHMSM(0,0,0,160);BUZZER_OFF;OSTimeDlyHMSM(0,0,0,60);
		BUZZER_ON;OSTimeDlyHMSM(0,0,0,320);BUZZER_OFF;OSTimeDlyHMSM(0,0,0,60);
		BUZZER_ON;OSTimeDlyHMSM(0,0,0,640);BUZZER_OFF;
		
		OSTimeDlyHMSM(0,0,PC_OFF_WAIT,0);               //�ȴ�������ɹػ�
		
//		DISABLE_PC;
//		DISABLE_HEAD;
		POWER_OFF;                                      //�ص������Դ
	}
}







/********************************************************************************************************/
/****************************************��ͨ����*******************************************************/
/****************************************��ͨ����*******************************************************/
/****************************************��ͨ����*******************************************************/
/********************************************************************************************************/



/********************************************************************************************************
Function Name: delay_us
Author       : 
Date         : 2016.7.18
Description  : 
Inputs       : None
Outputs      : None 
********************************************************************************************************/
void delay_us(u16 nus)
{
	u16 i=0;
	while(nus--)
	{
		i=11*4;
		while(i--);
	}
} 


/********************************************************************************************************
Function Name: Kalman_Filter
Author       : 
Date         : 2016.7.19
Description  : 
Inputs       : original data
Outputs      : the result of kalman_filter 
********************************************************************************************************/
u16 Kalman_Filter(u16 input)
{
//	kal.Deduce_Value = kal.Result_Value; //����ǰ������ʱ�̵Ĳ�����ͬ�����ƶ�
//	kal.Deduce_Variance=kal.Result_Value+kal.R_Variance;
//	
//	kal.K=kal.Deduce_Variance/(kal.Deduce_Variance+kal.Noise_Variance);
//	
//	kal.Result_Value=kal.Deduce_Value + kal.K * (input-kal.Deduce_Value);  //���Ź���ֵ
//	kal.Result_Variance=(1-kal.K)*kal.Deduce_Variance;
	return 0;
//	return kal.K;
	
}


/********************************************************************************************************
Function Name: ADCtoVoltage
Author       : 
Date         : 2016.8.1
Description  : 
Inputs       : result of ADC
Outputs      : Voltage
********************************************************************************************************/
void ADCtoVoltage(float *Voltage)
{	
}



/********************************************************************************************************
Function Name: ADCtoTemparature
Author       : 
Date         : 2016.8.2
Description  : 
Inputs       : result of ADC
Outputs      : temperature
********************************************************************************************************/
void ADCtoTemparature(float *Temperature)
{
}


/********************************************************************************************************
Function Name: temperature_adc_switch
Author       : 
Date         : 2016.8.22
Description  : 
Inputs       : ѡ���ͨ��
Outputs      : 
********************************************************************************************************/
void temperature_adc_switch(int channel)
{
	switch (channel)
	{
		case 0: {PA2_OFF;PA1_OFF;PA0_OFF;}break;
		case 1: {PA2_OFF;PA1_OFF;PA0_ON;} break;
		case 2: {PA2_OFF;PA1_ON;PA0_OFF;} break;
		case 3: {PA2_OFF;PA1_ON;PA0_ON;}  break;
		case 4: {PA2_ON;PA1_OFF;PA0_OFF;} break;
		case 5: {PA2_ON;PA1_OFF;PA0_ON;}  break;
		case 6: {PA2_ON;PA1_ON;PA0_OFF;}  break;
		case 7: {PA2_ON;PA1_ON;PA0_ON;}  break;
	}
}



/********************************************************************************************************
Function Name: temperature_adc_switch
Author       : 
Date         : 2016.8.22
Description  : 
Inputs       : ѡ���ͨ��
Outputs      : 
********************************************************************************************************/
void voltage_adc_switch(int channel)
{
	switch (channel)
	{
		case 0: {PA5_OFF;PA4_OFF;PA3_OFF;}break;
		case 1: {PA5_OFF;PA4_OFF;PA3_ON;} break;
		case 2: {PA5_OFF;PA4_ON;PA3_OFF;} break;
		case 3: {PA5_OFF;PA4_ON;PA3_ON;}  break;
		case 4: {PA5_ON;PA4_OFF;PA3_OFF;} break;
		case 5: {PA5_ON;PA4_OFF;PA3_ON;}  break;
	}
	
}


void  OS_CPU_SysTickInit (void)
{
  INT32U  cnts;
	RCC_ClocksTypeDef  rcc_clocks;

	RCC_GetClocksFreq(&rcc_clocks);	//���ϵͳʱ��Ƶ�ʡ�
  cnts = rcc_clocks.HCLK_Frequency / OS_TICKS_PER_SEC;

	SysTick_Config(cnts);
}



u16 Get_Adc(u8 ch)   
{
  	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADCͨ��,����ʱ��Ϊ239.5����	  			    
  
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת������

	return ADC_GetConversionValue(ADC1);	//�������һ��ADC1�������ת�����
}

u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	float Max=0;
	float Min=0;
	//�޳����ֵ����Сֵ�ľ�ֵ�˲�
	for(t=0;t<times;t++)
	{
		if(t==0)
		{
			Max=Get_Adc(ch);     //��һ�������������ֵ��Ҳ����Сֵ��
    	Min=Max;
			temp_val=temp_val+Max;
		}
		//��times��ѭ���У��ҳ����ֵ����Сֵ��
		temp_val+=Get_Adc(ch);
		if(Get_Adc(ch)>Max)
		{
			Max=Get_Adc(ch);
		}
		if(Get_Adc(ch)<Min)
		{
			Min=Get_Adc(ch);
		}
		delay_us(500);
	}
	//�������п۳����ֵ����Сֵ
	temp_val=temp_val-Max-Min;
	return temp_val/(times-2);
} 	 

