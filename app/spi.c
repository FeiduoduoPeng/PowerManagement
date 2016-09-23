#include "spi.h"
#include "bsp.h"
#include "app.h"


u8 CRC_Table[]={
0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3};



SPI_InitTypeDef  SPI_InitStructure;

void SPI2_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 |GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);

 	GPIO_SetBits(GPIOB,GPIO_Pin_13 |GPIO_Pin_14| GPIO_Pin_15);
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//ѡ���˴���ʱ�ӵ���̬:ʱ�����ո�
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//���ݲ����ڵڶ���ʱ����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
	SPI_Init(SPI2, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
 
//	SPI_CalculateCRC(SPI2, ENABLE);
	SPI_Cmd(SPI2, ENABLE); //ʹ��SPI���� 
}   
//SPI �ٶ����ú���
//SpeedSet:
//SPI_BaudRatePrescaler_2   2��Ƶ   (SPI 36M@sys 72M)
//SPI_BaudRatePrescaler_8   8��Ƶ   (SPI 9M@sys 72M)
//SPI_BaudRatePrescaler_16  16��Ƶ  (SPI 4.5M@sys 72M)
//SPI_BaudRatePrescaler_256 256��Ƶ (SPI 281.25K@sys 72M)
  
void SPI2_SetSpeed(u8 SpeedSet)
{
	SPI_InitStructure.SPI_BaudRatePrescaler = SpeedSet ;
  SPI_Init(SPI2, &SPI_InitStructure);
	SPI_Cmd(SPI2,ENABLE);
} 

//SPIx ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
u8 SPI2_ReadWriteByte(u8 TxData)
{		
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
		{
		retry++;
		if(retry>200)return 0;
		}			  
	SPI_I2S_SendData(SPI2, TxData); //ͨ������SPIx����һ������
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)//���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
		{
		retry++;
		if(retry>200)return 0;
		}	  						    
	return SPI_I2S_ReceiveData(SPI2); //����ͨ��SPIx������յ�����					    
}

/********************************************************************
*name:SPI2_Read_Reg
*auther:pf
*date:2016.8.1
*describe:�ԼĴ�����������д
*input��Dev:��������ַ��ĩβ0Ϊ����1Ϊд; LenΪ���ȡ�����ݵĳ��� ; ��ȡ�����ݴ����*Buff���ݻ�����
*return��null
********************************************************************/
void SPI2_Read_Reg(u8 Len,u8 ADDR,u8 Reg,u8 *Buff)
{
	u8 i=0;
	CS_OFF;
	delay_us(1000); //��ʱ1ms
	CS_ON;
	ADDR=ADDR<<1;
	delay_us(1000); //��ʱ1ms
	SPI2_ReadWriteByte(ADDR);   //����������ַ
	SPI2_ReadWriteByte(Reg);     //���ͼĴ�����ַ
	SPI2_ReadWriteByte(Len);     //���Ͷ�ȡ�ó���
	
	for(i=0;i<Len;i++)              //����0x00ռλ������÷��ص�����
	{
//		*(Buff+i)=SPI2_ReadWriteByte(0x00);
		Buff[i]=SPI2_ReadWriteByte(0x00);
	}
	SPI2_ReadWriteByte(0x00);//CRCռλ  
	delay_us(1000); //��ʱ1ms
	CS_OFF;
}



/********************************************************************
*name:SPI2_Write_Reg
*auther:pf
*date:2016.8.1
*describe:�ԼĴ�����������д
*input��Dev:LenΪд�����ݵ��ܳ��� ����������ַ��Ĵ�����ַ; *Buff���ݻ�����
*return��null
********************************************************************/
void SPI2_Write_Reg(u8 Len,u8 *Buff)
{
	u8 crc=0;
	u8 i=0;
	CS_OFF;
	delay_us(1000); //��ʱ1ms
	CS_ON;
	delay_us(1000); //��ʱ1ms
	Buff[0]=(Buff[0]<<1)+1;//������ַ����һλ����һ����ʾҪд����
	
  crc=SPI2_CRC_Cacul(Buff,Len); //����CRC��ֵ
	
	SPI2_ReadWriteByte(Buff[0]); //����������ַ
	SPI2_ReadWriteByte(Buff[1]); //���ͼĴ����ĵ�ַ
	
	for(i=2;i<Len;i++)            //��������
	{
		SPI2_ReadWriteByte(Buff[i]);
	}
	SPI2_ReadWriteByte(crc);       //����crcУ���
	delay_us(1000); //��ʱ1ms�
	CS_OFF;
}


/********************************************************************
*name:SPI2_CRC_Cacul
*auther:pf
*date:2016.8.2
*describe:CRC����
*input��Dev:��������е��׵�ַ�����г���
*return��null
********************************************************************/
u8 SPI2_CRC_Cacul(u8 *buffer, u8 length)
{
	u8 Crc=0;
	int temp=0;
	int i=0;
	for(i=0;i<length;i++)
	{
		temp=Crc^buffer[i];
		Crc=CRC_Table[temp];
	}
	return Crc;
}
