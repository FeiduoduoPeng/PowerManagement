#ifndef __SPI_H
#define __SPI_H
#include "stm32f10x.h"
	  
#define DEV_ADDR1 0x01   //0000 0010�Ľṹ��ʽ     
#define DEV_ADDR2 0x02   //0000 0010�Ľṹ��ʽ  
#define DEVICE_STATUS 0x00
#define VCELL1 0x03
#define TEMPERATURE1 0x0f
#define TEMPERATURE2 0x11
#define ALERT 0x20
#define FAULT 0x21
#define ADC_CONTROL 0x30
#define CB_CTRL 0x32
#define CB_TIME 0x33
#define RESET_REG 0x3c
#define USER1 0x48
#define ADDRESS_CONTROL 0x3b


void SPI2_Configuration(void);			 //��ʼ��SPI��
void SPI2_SetSpeed(u8 SpeedSet); //����SPI�ٶ�   
u8 SPI2_ReadWriteByte(u8 TxData);//SPI���߶�дһ���ֽ�
void SPI2_Write_Reg(u8 Len,u8 *Buff);
void SPI2_Read_Reg(u8 Len,u8 ADDR,u8 Reg,u8 *Buff);
u8 SPI2_CRC_Cacul(u8 *buffer, u8 length);

#endif

