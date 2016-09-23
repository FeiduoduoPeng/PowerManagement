#ifndef ADS1115_H
#define ADS1115_H

#include "stm32f10x.h"
#include "stm32f10x_i2c.h"
//#include "systick.h"
//#include "delay_timer.h"


#define MUX_AIN0_AIN1  (0x0000<<12) 
#define MUX_AIN0_AIN3  (0x0001<<12)
#define MUX_AIN1_AIN3  (0x0002<<12)
#define MUX_AIN2_AIN3  (0x0003<<12)
#define MUX_AIN0_GND   (0x0004<<12)
#define MUX_AIN1_GND   (0x0005<<12)
#define MUX_AIN2_GND   (0x0006<<12)  
#define MUX_AIN3_GND   (0x0007<<12)

#define PGA_6144       (0x0000<<9)
#define PGA_4096       (0x0001<<9)
#define PGA_2048       (0x0002<<9)
#define PGA_1024       (0x0003<<9) 
#define PGA_0512       (0x0004<<9)
#define PGA_0256       (0x0005<<9)

#define DR_008         (0x0000<<5)
#define DR_016         (0x0001<<5)
#define DR_032         (0x0002<<5)
#define DR_064         (0x0003<<5)
#define DR_128         (0x0004<<5)
#define DR_250         (0x0005<<5)
#define DR_475         (0x0006<<5)
#define DR_860         (0x0007<<5) 


#define FULL_RANGE     4.096

#define CONFIG_REG     0X01
#define CONVER_REG     0x00

#define OS            (0x0001<<15)            // 1 bit  单次转换,工作在连续转换模式下,此为没有影响
#define MUX           (MUX_AIN0_GND)          // 3 bit P:AIN0,N:GND
#define PGA           (PGA_4096)              // 3 bit FS=4.096
#define MODE          (0x0000)                // 1 bit 连续转换模式

#define DR            (DR_860)                // 3 bit转换速率 860SPS
#define COMP_MODE     (0x0000)                // 1 bit
#define COMP_POL      (0x0000)                // 1 bit 
#define COMP_LAT      (0x0000)                // 1 bit 
#define COMP_QUE      (0x0003)                // 2 bit     
//#define COMP_QUE      (0x0002)                // 2 bit     

#define CURRENT_CHANNEL   1


int16_t ads1115_readreg(u8 reg,u8 n);
int ads1115_getvalue(u8 n);
void ADS1115_Init(void );
float ADS1115_GetVol(u8 n);
float ADS1115_GetVolFromChannel(int channel,u8 n);

#endif

