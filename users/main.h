#ifndef _MAIN_H
#define _MAIN_H

#include "stm32f10x.h"
#include <stdio.h>

extern int SCAN_FLAG;

void Start_BatteryManagent(void);

int fputc(int ch, FILE *f)
{
  /* Place your implementation of fputc here */
  USART_SendData(USART1, (u8)ch);
  while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
  return ch;
}

#endif

