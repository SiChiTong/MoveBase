
#ifndef __I2C_H
#define __I2C_H
#include "stm32f10x.h"
#include "common.h"

#define I2C_SPEED_1K		5000	//���ݴ������ٶ����ã����ﴦ�����ٶ���72MHz

//I2C�˿ڶ���
#define I2C_SCL0    GPIOout(GPIOB, 10)	//SCL--PB10
#define I2C_SDA0    GPIOout(GPIOB, 11)	//SDA--PB11
#define READ_SDA0   GPIOin( GPIOB, 11)	//����SDA



//����SDA:PD11�������
#define SDA0_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}		//�ο�stm32�Ĵ�������
#define SDA0_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}

//I2C�˿ڶ���
#define I2C_SCL1    GPIOout(GPIOE, 11)	//SCL--PE11
#define I2C_SDA1    GPIOout(GPIOE, 12)	//SDA--PE12
#define READ_SDA1   GPIOin( GPIOE, 12)	//����SDA



//����SDA:PE12�������
#define SDA1_IN()  {GPIOE->CRH&=0XFFF0FFFF;GPIOE->CRH|=8<<16;}		//�ο�stm32�Ĵ�������
#define SDA1_OUT() {GPIOE->CRH&=0XFFF0FFFF;GPIOE->CRH|=3<<16;}

typedef enum
{
	I2C_SUCCESS = 0,
	I2C_TIMEOUT,
	I2C_ERROR,
}I2C_StatusTypeDef;

extern uint32_t i2c_speed;	//I2C�����ٶ� = I2C_SPEED_1K / i2c_speed

/* ---------------------------����I2CЭ���д��ʱ����------------------------------*/
void I2C0_Init(void);				//��ʼ��I2C��IO��				 
void I2C0_Start(void);				//����I2C��ʼ�ź�
void I2C0_Stop(void);				//����I2Cֹͣ�ź�
uint8_t I2C0_Wait_ACK(void);	//I2C�ȴ�ACK�ź�
void I2C0_ACK(void);					//I2C����ACK�ź�
void I2C0_NACK(void);				//I2C������ACK�ź�
void I2C0_Send_Byte(uint8_t data);		//I2C����һ���ֽ�
uint8_t I2C0_Read_Byte(uint8_t ack);	//I2C��ȡһ���ֽ�


uint16_t I2C_SetSpeed(uint16_t speed);//����I2C�ٶ�(1Kbps~400Kbps,speed��λ��Kbps)

/* ---------------------------���²����Ƿ�װ�õ�I2C��д����--------------------------- */

//���嵽ĳһ������������ϸ�Ķ�������������I2C���ֵ�˵������ΪĳЩ������I2C�Ķ�д������
//��һЩ���죬����Ĵ��������ھ��������I2C�����У�������֤OK�ģ�
I2C_StatusTypeDef I2C0_WriteOneByte(uint8_t DevAddr, uint8_t DataAddr, uint8_t Data);//��I2C���豸д��һ���ֽ�

I2C_StatusTypeDef I2C0_ReadOneByte(uint8_t DevAddr, uint8_t DataAddr, uint8_t* Data);//��I2C���豸��ȡһ���ֽ�

I2C_StatusTypeDef I2C0_WriteBit(uint8_t DevAddr, uint8_t DataAddr, uint8_t Bitx, uint8_t BitSet);

I2C_StatusTypeDef single_write_I2C0(uint8_t DevAddr, uint16_t DataAddr, uint8_t pData);									//��I2C���豸��д��һ���ֽ�

I2C_StatusTypeDef double_write_I2C0(uint8_t DevAddr, uint16_t DataAddr, uint16_t pData);									//��I2C���豸����д��2���ֽ�

I2C_StatusTypeDef multi_write_I2C0(uint8_t DevAddr, uint16_t DataAddr, uint32_t pData);									//��I2C���豸����д��4���ֽ�

I2C_StatusTypeDef single_read_I2C0(uint8_t DevAddr, uint16_t DataAddr, uint8_t* pData);									//��I2C�豸�϶�ȡһ���ֽ�

I2C_StatusTypeDef multi_read_I2C0(uint8_t DevAddr, uint16_t DataAddr, uint8_t* pData, uint32_t Num);			//��I2C�豸������ȡNum���ֽ�


I2C_StatusTypeDef I2C1_WriteOneByte(uint8_t DevAddr, uint8_t DataAddr, uint8_t Data);//��I2C���豸д��һ���ֽ�

I2C_StatusTypeDef I2C1_ReadOneByte(uint8_t DevAddr, uint8_t DataAddr, uint8_t* Data);//��I2C���豸��ȡһ���ֽ�

I2C_StatusTypeDef I2C1_WriteBit(uint8_t DevAddr, uint8_t DataAddr, uint8_t Bitx, uint8_t BitSet);

I2C_StatusTypeDef single_write_I2C1(uint8_t DevAddr, uint16_t DataAddr, uint8_t pData);									//��I2C���豸��д��һ���ֽ�

I2C_StatusTypeDef double_write_I2C1(uint8_t DevAddr, uint16_t DataAddr, uint16_t pData);									//��I2C���豸����д��2���ֽ�

I2C_StatusTypeDef multi_write_I2C1(uint8_t DevAddr, uint16_t DataAddr, uint32_t pData);									//��I2C���豸����д��4���ֽ�

I2C_StatusTypeDef single_read_I2C1(uint8_t DevAddr, uint16_t DataAddr, uint8_t* pData);									//��I2C�豸�϶�ȡһ���ֽ�

I2C_StatusTypeDef multi_read_I2C1(uint8_t DevAddr, uint16_t DataAddr, uint8_t* pData, uint32_t Num);			//��I2C�豸������ȡNum���ֽ�

#endif

/********************* (C) COPYRIGHT 2014 WWW.UCORTEX.COM **********END OF FILE**********/
