#ifndef OLED_H
#define OLED_H
#include "stm32f10x.h"

#define SDA_GPIO   GPIOB
#define SDA_BIT    GPIO_Pin_7

#define SCK_GPIO   GPIOB
#define SCK_BIT    GPIO_Pin_6

#define DC_GPIO    GPIOB
#define DC_BIT     GPIO_Pin_5

#define RST_GPIO   GPIOB
#define RST_BIT    GPIO_Pin_4

#define OLED_RST_H  GPIO_SetBits(RST_GPIO,RST_BIT)
#define OLED_RST_L  GPIO_ResetBits(RST_GPIO,RST_BIT)

#define OLED_SDA_H  GPIO_SetBits(SDA_GPIO,SDA_BIT)
#define OLED_SDA_L  GPIO_ResetBits(SDA_GPIO,SDA_BIT)

#define OLED_SCK_H  GPIO_SetBits(SCK_GPIO,SCK_BIT)
#define OLED_SCK_L  GPIO_ResetBits(SCK_GPIO,SCK_BIT)

#define OLED_DC_H   GPIO_SetBits(DC_GPIO,DC_BIT)
#define OLED_DC_L   GPIO_ResetBits(DC_GPIO,DC_BIT)

#define OLED_GPIO_CLK (RCC_APB2Periph_GPIOB)

void OLED_Init(void);
void OLED_Pant(void);
void OLED_Clear(void );
void OLED_ShowEnglish(u8 ,u8 ,u8 *);
void OLED_Printf(u8 ,u8, char *,...);
void OLED_HorizontalScroll(u8 ,u8 ,u8);

#endif



