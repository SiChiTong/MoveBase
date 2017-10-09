/*=============================================================
@brief		STM32通用IO口模拟I2C底层驱动
@author		ZGJ
@date			2015-10-16
@statement	All rights reserved
==============================================================*/
#ifndef		_I2CIO_H
#define		_I2CIO_H
/*-------------------Include Files-----------------------------*/
#include	"stm32f10x.h"
#ifdef	__cplusplus
extern	"C"
{
#endif
/*------------------typedef-------------------------------*/
typedef enum _BOOL_T
{
	bool_false = 0,
	bool_true = 1
}Bool;
/*--------------------define -----------------------------*/
#define		I2C_SCL			GPIO_Pin_10
#define		I2C_SDA			GPIO_Pin_11
#define		GPIO_I2C		GPIOB
#define		I2C_SCL_H		GPIO_SetBits(GPIO_I2C,I2C_SCL)
#define		I2C_SCL_L		GPIO_ResetBits(GPIO_I2C,I2C_SCL)
#define		I2C_SDA_H		GPIO_SetBits(GPIO_I2C,I2C_SDA)
#define		I2C_SDA_L		GPIO_ResetBits(GPIO_I2C,I2C_SDA)
	
/*---------------------Global Functions----------------------*/
void I2C_delay(void);
void I2C_GPIO_Init(void);
void I2C_SDA_Out(void);
void I2C_SDA_In(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NAck(void);
unsigned char I2C_Wait_Ack(void);
void I2C_Send_Byte(unsigned char txd);
unsigned char I2C_Read_Byte(unsigned char ack);	
Bool Single_Write(unsigned char SlaveAddress,unsigned short addr,unsigned char data);
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char addr);
Bool I2C_Read(unsigned char SlaveAddress,unsigned char addr,unsigned char *buf,unsigned short num);
Bool I2C_Write(unsigned char SlaveAddress,unsigned char addr,unsigned char *buf,unsigned short num);
	
#ifdef	__cplusplus
}
#endif

#endif
/*=======================End of File============================*/
