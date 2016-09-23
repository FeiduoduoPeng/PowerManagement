#include "ads1115.h"
#include "app.h"
#include "oled.h"
//#include "delay_timer.h"

/****************************��һ·IIC�����Ե�ص�ѹ���¶ȵĲ���************************************/
#define SCL_HIGH     GPIO_SetBits(GPIOA,GPIO_Pin_3)
#define SCL_LOW      GPIO_ResetBits(GPIOA,GPIO_Pin_3)
#define SDA_HIGH     GPIO_SetBits(GPIOA,GPIO_Pin_4)
#define SDA_LOW      GPIO_ResetBits(GPIOA,GPIO_Pin_4)
#define SDA          GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)
#define SDAOUT       //{GPIOA->CRL&=0XFFF0FFFF;GPIOA->CRL|=3<<16;}  //�Լ�д�ģ���һ����ȷ
#define SDAIN        //{GPIOA->CRL&=0XFFF0FFFF;GPIOA->CRL|=8<<16;}

/****************************�ڶ�·IIC�����Ը�������ĵ�ѹ������Ĳ���**********************************/
#define SCL_HIGH2     GPIO_SetBits(GPIOB,GPIO_Pin_0)
#define SCL_LOW2      GPIO_ResetBits(GPIOB,GPIO_Pin_0)
#define SDA_HIGH2     GPIO_SetBits(GPIOB,GPIO_Pin_1)
#define SDA_LOW2      GPIO_ResetBits(GPIOB,GPIO_Pin_1)
#define SDA2          GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)
#define SDAOUT2       //{GPIOB->CRL&=0XFFFFFF0F;GPIOB->CRL|=3<<1;}  //�Լ�д�ģ���һ����ȷ
#define SDAIN2        //{GPIOB->CRL&=0XFFFFFF0F;GPIOB->CRL|=8<<1;}


void ads1115_io_init(void )
{
	GPIO_InitTypeDef GPIO_InitStruct;	
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_3|GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;
	GPIO_Init(GPIOB,&GPIO_InitStruct);
}
/**********************************************************/
/**********************************************************/
void I2C_Start(void )
{
	SDAOUT;
	SDA_HIGH;
	SCL_HIGH;
	delay_us(2);
	SDA_LOW;
	delay_us(2);
	SCL_LOW; //��������
}
void I2C_Start2(void )
{
	SDAOUT2;
	SDA_HIGH2;
	SCL_HIGH2;
	delay_us(2);
	SDA_LOW2;
	delay_us(2);
	SCL_LOW2; //��������
}


/**********************************************************/
/**********************************************************/
void I2C_Stop(void )
{
	SDAOUT;
	SCL_HIGH;
	SDA_LOW;
	delay_us(2);
	SDA_HIGH;
	delay_us(2);
}
void I2C_Stop2(void )
{
	SDAOUT2;
	SCL_HIGH2;
	SDA_LOW2;
	delay_us(2);
	SDA_HIGH2;
	delay_us(2);
}


/**********************************************************/
/**********************************************************/
void I2C_ACK(void )
{
	SDAOUT;
	SCL_LOW;
	SDA_LOW;
	delay_us(2);
	SCL_HIGH;
	delay_us(2);
	SCL_LOW;
}
void I2C_ACK2(void )
{
	SDAOUT2;
	SCL_LOW2;
	SDA_LOW2;
	delay_us(2);
	SCL_HIGH2;
	delay_us(2);
	SCL_LOW2;
}


/**********************************************************/
/**********************************************************/
u8 I2C_Wait_ACK(void )
{
	u8 count=0;
	SDAIN;
	
	SCL_HIGH;//�ͷ����� 
	delay_us(1);
	SDA_HIGH; //����SDA 
	delay_us(1);
	
	while(SDA) //��ȡSDA״̬��0ΪӦ��
	{
		count++;
		if(count>250)
		{
			return 1;
		}
	}
	SCL_LOW;
//	ACK++;    //��¼оƬ����Ӧ�ô�����������
	return 0;
}
u8 I2C_Wait_ACK2(void )
{
	u8 count=0;
	SDAIN2;
	
	SCL_HIGH2;//�ͷ����� 
	delay_us(1);
	SDA_HIGH2; //����SDA 
	delay_us(1);
	
	while(SDA2) //��ȡSDA״̬��0ΪӦ��
	{
		count++;
		if(count>250)
		{
			return 1;
		}
	}
	SCL_LOW2;
//	ACK++;    //��¼оƬ����Ӧ�ô�����������
	return 0;
}


/**********************************************************/
/**********************************************************/
void I2C_WriteByte(u8 cmd, u8 n)  //nָ��ʹ����һ·��I2C
{
	u8 i;
	if(n==0)
	{
		SDAOUT;
		SCL_LOW;         //scl=0,�������ݱ仯
	//	delay_us(5);
		for(i=0;i<8;i++)
		{
			if(cmd&0x80)SDA_HIGH;
			else SDA_LOW;
			cmd<<=1;
			delay_us(2);
			
			SCL_HIGH;    //SCL=1,���ݱ���
			delay_us(2);
			
			SCL_LOW;
			delay_us(1);
		}
	}
	else if(n==1)
	{
		SDAOUT2;
		SCL_LOW2;         //scl=0,�������ݱ仯
	//	delay_us(5);
		for(i=0;i<8;i++)
		{
			if(cmd&0x80)SDA_HIGH2;
			else SDA_LOW2;
			cmd<<=1;
			delay_us(2);
			
			SCL_HIGH2;    //SCL=1,���ݱ���
			delay_us(2);
			
			SCL_LOW2;
			delay_us(1);
		}
	}
}

u8 I2C_ReadByte(u8 n)
{
	u8 i,ans=0;
	if(n==0)
	{
		SDAIN;
		for(i=0;i<8;i++)
		{
			SCL_LOW;    //�������ݱ仯
			delay_us(2);
			
			SCL_HIGH;   //�������ݲ���
			delay_us(1);
			
			ans<<=1;     //��������⣬���簡������
			if(SDA)ans++;
			delay_us(1);
		}
		I2C_ACK();
	}
	else if(n==1)
	{
		SDAIN2;
		for(i=0;i<8;i++)
		{
			SCL_LOW2;    //�������ݱ仯
			delay_us(2);
			
			SCL_HIGH2;   //�������ݲ���
			delay_us(1);
			
			ans<<=1;     //��������⣬���簡������
			if(SDA2)ans++;
			delay_us(1);
		}
		I2C_ACK2();
	}
	
	return ans;
}

void ads1115_config(u16 config,u8 n)
{
	if(n==0)
	{
		I2C_Start();
		I2C_WriteByte(0x90,0);  //д�ź�
		I2C_Wait_ACK();
		I2C_WriteByte(0x01,0);  //ָ�����üĴ���
		I2C_Wait_ACK();
		I2C_WriteByte(config>>8,0);
		I2C_Wait_ACK();
		I2C_WriteByte(config,0);
		I2C_Wait_ACK();
		I2C_Stop();
	}
	
	if(n==1)
	{
		I2C_Start2();
		I2C_WriteByte(0x90,1);  //д�ź�
		I2C_Wait_ACK2();
		I2C_WriteByte(0x01,1);  //ָ�����üĴ���
		I2C_Wait_ACK2();
		I2C_WriteByte(config>>8,1);
		I2C_Wait_ACK2();
		I2C_WriteByte(config,1);
		I2C_Wait_ACK2();
		I2C_Stop2();
	}
	
}

int16_t ads1115_readreg(u8 reg,u8 n)
{
	int16_t value_h,value_l;
	if(n==0)
	{
		I2C_Start();
		
		I2C_WriteByte(0x90,0);  //д�ź�
		I2C_Wait_ACK();
		
		I2C_WriteByte(reg,0);  //ָ�����üĴ���
		I2C_Wait_ACK();
		
		I2C_Start();
		
		I2C_WriteByte(0x91,0);  //���ź�
		I2C_Wait_ACK();
		
		value_h=I2C_ReadByte(0);
		value_l=I2C_ReadByte(0);
	}
	else if(n==1)
	{
		I2C_Start2();
		
		I2C_WriteByte(0x90,1);  //д�ź�
		I2C_Wait_ACK2();
		
		I2C_WriteByte(reg,1);  //ָ�����üĴ���
		I2C_Wait_ACK2();
		
		I2C_Start2();
		
		I2C_WriteByte(0x91,1);  //���ź�
		I2C_Wait_ACK2();
		
		value_h=I2C_ReadByte(1);
		value_l=I2C_ReadByte(1);
	}
	
	value_h<<=8;
	value_h|=value_l;
	return value_h;
}

int ads1115_getvalue(u8 n)
{
	int value=0;
	value=ads1115_readreg(CONVER_REG,n);
	return value;
}

float ADS1115_GetVol(u8 n)
{
	float Vol;
	Vol = ads1115_getvalue(n);
	Vol = Vol * FULL_RANGE / 32767;
	return Vol;
}



/*��ĳһ��ͨ���õ�AD����ֵ*/
/*channel ��ͨ����    n�ǵڼ���оƬ*/
float ADS1115_GetVolFromChannel(int channel,u8 n) 
{
	int reg_config = 0;
	int mux = 0;
	float Vol;
	
	if(channel == 0)
	{
		mux = MUX_AIN0_GND;
	}
	else if(channel == 1)
	{
		mux = MUX_AIN1_GND;
	}	
	else if(channel == 2)
	{
		mux = MUX_AIN2_GND;
	}	
	else if(channel == 3)
	{
		mux = MUX_AIN3_GND;
	}	
	//���üĴ���
	reg_config = OS|
				mux|
				PGA|
			   MODE|
				 DR|
		  COMP_MODE|
		   COMP_POL|
		   COMP_LAT|
		   COMP_QUE;
	ads1115_config(reg_config,n);
	delay_us(50000);
	
	Vol = ADS1115_GetVol(n);
	return Vol;
}


void ADS1115_Init(void )
{
	int reg_config=0;
	ads1115_io_init();
	reg_config=OS|
						 MUX_AIN1_GND|
						 PGA|
						 MODE|
						 DR|
						 COMP_MODE|
						 COMP_POL|
						 COMP_LAT|
						 COMP_QUE;
	ads1115_config(reg_config,0);
	ads1115_config(reg_config,1);
}
