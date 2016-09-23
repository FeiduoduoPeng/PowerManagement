#ifndef BSP_H_
#define BSP_H_

#include "stm32f10x.h"
/************************选通开关*********************************/
#define PA0_ON GPIO_SetBits(GPIOA,GPIO_Pin_0)
#define PA1_ON GPIO_SetBits(GPIOA,GPIO_Pin_1)
#define PA2_ON GPIO_SetBits(GPIOA,GPIO_Pin_2)
#define PA3_ON GPIO_SetBits(GPIOA,GPIO_Pin_5)
#define PA4_ON GPIO_SetBits(GPIOA,GPIO_Pin_6)
#define PA5_ON GPIO_SetBits(GPIOA,GPIO_Pin_7)
#define PA0_OFF GPIO_ResetBits(GPIOA,GPIO_Pin_0)
#define PA1_OFF GPIO_ResetBits(GPIOA,GPIO_Pin_1)
#define PA2_OFF GPIO_ResetBits(GPIOA,GPIO_Pin_2)
#define PA3_OFF GPIO_ResetBits(GPIOA,GPIO_Pin_5)
#define PA4_OFF GPIO_ResetBits(GPIOA,GPIO_Pin_6)
#define PA5_OFF GPIO_ResetBits(GPIOA,GPIO_Pin_7)

/****************************各部件的电源开关接口***********************************/
#define PC_OFF GPIO_ResetBits(GPIOA,GPIO_Pin_15)
#define DISABLE_PC GPIO_ResetBits(GPIOB,GPIO_Pin_14)
#define DISABLE_ANDROID GPIO_ResetBits(GPIOB,GPIO_Pin_15)
#define DISABLE_HEAD GPIO_ResetBits(GPIOB,GPIO_Pin_13)
#define DISABLE_CHASISS GPIO_ResetBits(GPIOB,GPIO_Pin_12)
#define POWER_OFF GPIO_ResetBits(GPIOB,GPIO_Pin_8)

#define PC_ON GPIO_SetBits(GPIOA,GPIO_Pin_15)
#define ENABLE_PC GPIO_SetBits(GPIOB,GPIO_Pin_14)
#define ENABLE_ANDROID GPIO_SetBits(GPIOB,GPIO_Pin_15)
#define ENABLE_HEAD GPIO_SetBits(GPIOB,GPIO_Pin_13)
#define ENABLE_CHASISS GPIO_SetBits(GPIOB,GPIO_Pin_12)
#define POWER_ON GPIO_SetBits(GPIOB,GPIO_Pin_8)
/************************************************************************************/


#define BUZZER_OFF GPIO_ResetBits(GPIOA,GPIO_Pin_8)
#define BUZZER_ON GPIO_SetBits(GPIOA,GPIO_Pin_8)

#define STATUS_ST GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)

#define fanner(n) TIM_SetCompare1(TIM2,n)

void BSP_Init(void);
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void CAN1_Configuration(void);
void USART1_Configuration(void);
void IIC_Configuration(void);
void ADC_Configuration(void);
void DMA_Configuration(void);
void PWM_Configuration(void);
void USART3_Configuration(void);
void TIM3_Configuration(void);
void EXIT_Configuration(void);

void Can_Sent(u8* data);

#endif
/***************************end of file*****************************************/

