#include "bsp.h"
#include "stm32f10x.h"
//#include "app.h"

/********************************************************************************************************
Function Name: BSP_Init
Author       : 
Date         : 2016.7.12
Description  : 
Inputs       : None
Outputs      : None 
********************************************************************************************************/
void BSP_Init(void)
{
	RCC_Configuration();
	GPIO_Configuration();
	USART1_Configuration();
	CAN1_Configuration();
	PWM_Configuration();
//	TIM3_Configuration();
	EXIT_Configuration();
//	SPI2_Configuration();
//	USART3_Configuration();
	NVIC_Configuration();
//	DMA_Configuration();
	ADC_Configuration();
	
}

/********************************************************************************************************
Function Name: RCC_Configuration
Author       : 
Date         : 2016.7.12
Description  : 
Inputs       : None
Outputs      : None 
********************************************************************************************************/
void RCC_Configuration(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2 ,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
}



/********************************************************************************************************
Function Name: GPIO_Configuration
Author       : 普通io口，常用语驱动led、buzzer等
Date         : 2016.7.12
Description  : 
Inputs       : None
Outputs      : None 
********************************************************************************************************/
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
 /**********************多路复用开关****************************/
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
//	GPIO_ResetBits(GPIOA, GPIO_Pin_12);// 全部默认第一路
	
	
 /*********************各部分电源启动控制****************************/
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_8 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);


 /*********************BUZZER***************************/
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, GPIO_Pin_8);	
	
	/*********************PC_ON**************************/
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, GPIO_Pin_15);
	
}


/********************************************************************************************************
Function Name: ADC_Configuration
Author       : 
Date         : 2016.8.30
Description  : 
Inputs       : None
Outputs      : None 
********************************************************************************************************/
void ADC_Configuration(void)
{
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;  //三四通道
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	ADC_DeInit(ADC1); 
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//模数转换工作在单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在单次转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   

//  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_7Cycles5);
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_7Cycles5);
	
	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1
	
	ADC_ResetCalibration(ADC1);	//使能复位校准  
	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//等待复位校准结束
	
	ADC_StartCalibration(ADC1);	 //开启AD校准
 
	while(ADC_GetCalibrationStatus(ADC1));	 //等待校准结束
	
}
/********************************************************************************************************
Function Name: EXIT_Configuration
Author       : 
Date         : 2016.8.23
Description  : 
Inputs       : None
Outputs      : None 
********************************************************************************************************/
void EXIT_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStructure;
	
	/***********************触摸启动信号***************************/
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource9);
//	
//	EXTI_InitStructure.EXTI_Line=EXTI_Line9;
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;//上升/下降沿触发
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

}
/********************************************************************************************************
Function Name: NVIC_Configuration
Author       : 
Date         : 2016.7.12
Description  : 
Inputs       : None
Outputs      : None 
********************************************************************************************************/
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef  NVIC_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;              //定时器中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;   //CAN1收数中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

//	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;           //触摸按键中断
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
}


/********************************************************************************************************
Function Name: CAN1_Configuration
Author       : pf
Date         : 2016.8.22
Description  : 
Inputs       : None
Outputs      : None 
********************************************************************************************************/
void CAN1_Configuration(void)
{
	CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
 
   /* Configure CAN pin: RX */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   
   /* Configure CAN pin: TX */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   
//   GPIO_PinRemapConfig(GPIO_Remap1_CAN1 , ENABLE);
   
   /* CAN register init */
   CAN_DeInit(CAN1);
   CAN_StructInit(&CAN_InitStructure);

   /* CAN cell init */
   CAN_InitStructure.CAN_TTCM = DISABLE;
   CAN_InitStructure.CAN_ABOM = DISABLE;
   CAN_InitStructure.CAN_AWUM = DISABLE;
   CAN_InitStructure.CAN_NART = DISABLE;
   CAN_InitStructure.CAN_RFLM = DISABLE;
   CAN_InitStructure.CAN_TXFP = DISABLE;
   CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
   
   /* CAN Baudrate = 1MBps*/
   CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
   CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
   CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
   CAN_InitStructure.CAN_Prescaler = 6;
   CAN_Init(CAN1, &CAN_InitStructure);
 

   CAN_FilterInitStructure.CAN_FilterNumber = 0;
   CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
   CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
   CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
   CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
   CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
   CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
   CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FilterFIFO0;
   CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
   CAN_FilterInit(&CAN_FilterInitStructure);
   
	 CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
   /* Transmit */
	 
//   TxMessage.StdId = 0x321;
//   TxMessage.ExtId = 0x01;
//   TxMessage.RTR = CAN_RTR_DATA;
//   TxMessage.IDE = CAN_ID_STD;
//   TxMessage.DLC = 1;

}


/********************************************************************************************************
Function Name: USART3_Configuration
Author       : pf
Date         : 2016.8.1
Description  : 
Inputs       : None
Outputs      : None 
********************************************************************************************************/
void USART1_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
	
	USART_DeInit(USART1);
	USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	
	USART_Cmd(USART1, ENABLE);
	
}



/********************************************************************************************************
Function Name: RCC_Configuration
Author       : pf
Date         : 2016.7.12
Description  : 
Inputs       : None
Outputs      : None 
********************************************************************************************************/
void DMA_Configuration(void)
{
//	 DMA_InitTypeDef  DMA_InitStructure;
//	
//	/**********************ADC1的DMA_channel_1**************************/
//	 DMA_DeInit(DMA1_Channel1);
//   DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&(ADC1->DR)); //0x4001244c  
//   DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(ADC1_BUFFER);
//   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
//   DMA_InitStructure.DMA_BufferSize = 4;
//   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//   DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//   DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
//   DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//   DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//   DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  
//	 
//	 DMA_Init(DMA1_Channel1, &DMA_InitStructure);  
//	 DMA_Cmd(DMA1_Channel1, ENABLE);
}


/********************************************************************************************************
Function Name: PWM_Configuration
Author       : pf
Date         : 2016.7.12
Description  : Configuration of PWM_Configuration of time3  电风扇的驱动
Inputs       : None
Outputs      : None 
********************************************************************************************************/
void PWM_Configuration(void)  
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);

   /* Time Base configuration    20kHz*/
   TIM_TimeBaseStructure.TIM_Prescaler = 0;
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseStructure.TIM_Period = 360;
   TIM_TimeBaseStructure.TIM_ClockDivision = 0;
 
   TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
 
   TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
   TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
   TIM_OCInitStructure.TIM_Pulse = 00;
   TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
   TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	 
	 TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	 
	 TIM_ARRPreloadConfig(TIM2, ENABLE);
	 
   /* TIM3 counter enable */
   TIM_Cmd(TIM2, ENABLE);
 
   /* TIM3 Main Output Enable */
   TIM_CtrlPWMOutputs(TIM2, ENABLE);
}

void TIM3_Configuration(void )  //用以保证触摸开关的持续触摸，防止误触发
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period=4999;       //产生一个500ms的中断
	TIM_TimeBaseStructure.TIM_Prescaler=7199;
	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
//	TIM_Cmd(TIM2,ENABLE);
}


/********************************************************************************************************
Function Name: Can_Sent
Author       : pf
Date         : 2016.7.12
Description  : 广播发送消息
Inputs       : data to send
Outputs      : None 
********************************************************************************************************/
void Can_Sent(u8* data)
{
	CanTxMsg tx_message;
	int i=0;
	u8 mbox;
	
	tx_message.StdId = 0x402;
	tx_message.DLC = 0x05;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.IDE = CAN_ID_STD;

//	for(i=0;i<8;i++)
//	{tx_message.Data[0] = data[i];}
	tx_message.Data[0] = *data;
	tx_message.Data[1] = *(data+1);
	tx_message.Data[2] = *(data+2);
	tx_message.Data[3] = *(data+3);
	tx_message.Data[4] = *(data+4);
	
	mbox=CAN_Transmit(CAN1,&tx_message);
	while(CAN_TransmitStatus(CAN1,mbox)==CAN_TxStatus_Ok); //等待发送成功
}



/********************************************************************************************************
中断服务函数
********************************************************************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;
	u8 data[8]={0};
	u8 i=0;
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
		CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	}
	if(RxMessage.StdId==0x00)
	{
		for(i=0;i<8;i++)
		{data[i]=RxMessage.Data[i];}
	}
}