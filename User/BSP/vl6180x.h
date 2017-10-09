/**-------------------------------------------------------------------------
*@ file				vl6180x.h
*@ brief			激光传感器驱动,I2C Interface
*@ author 		ZGJ
*@ vertion		V0.01
*@ date				2015-12-20
*@ statement	(C)	COPYRIGHT 2015 KSITRI.
--------------------------------------------------------------------------*/
#ifndef __VL6180X_H
#define __VL6180X_H
/* Include ---------------------------------------------------------*/
#include "stm32f10x.h"
#ifdef __cplusplus
extern "C"
{
#endif
/* typedef --------------------------------------------------------------*/
typedef enum 
{
    UI2C_ERR_NONE        =       0,
    UI2C_ERR_TIMEOUT     =       1,
    UI2C_ERR_FAILED      =       2
}_I2C_ERR;
/* define ----------------------------------------------------------------*/
#define VL6180X_I2C				I2C2
#define I2C_PORT					GPIOB
#define I2C_SDA_Pin				GPIO_Pin_11
#define I2C_SCL_Pin				GPIO_Pin_10	

#define	SLAVE_ADDRESS1			0x2A
#define VL6180X_ADDRESS1		0x54
#define	SLAVE_ADDRESS2			0x28
#define VL6180X_ADDRESS2		0x50
#define	SLAVE_ADDRESS3			0x27
#define VL6180X_ADDRESS3		0x4E
#define	SLAVE_ADDRESS4			0x26
#define VL6180X_ADDRESS4		0x4C
#define	SLAVE_ADDRESS5			0x25
#define VL6180X_ADDRESS5		0x4A
#define	SLAVE_ADDRESS6			0x24
#define VL6180X_ADDRESS6		0x48
#define I2C_SPEED					300000			//300k

#define     UI2C_FLAG_TIMEOUT        ((uint32_t)0x1000)
#define     UI2C_LONG_TIMEOUT        ((uint32_t)(10 * UI2C_FLAG_TIMEOUT))

#define RESULT__RANGE_STATUS						0x04D
#define RESULT__INTERRUPT_STATUS_GPIO		0x04F
#define RESULT__RANGE_VAL								0x062

#define VL6180X_CE1_OPEN	GPIOF->BSRR=GPIO_Pin_4
#define VL6180X_CE1_CLOSE GPIOF->BRR=GPIO_Pin_4
#define VL6180X_CE2_OPEN	GPIOE->BSRR=GPIO_Pin_6
#define VL6180X_CE2_CLOSE GPIOE->BRR=GPIO_Pin_6
#define VL6180X_CE3_OPEN  GPIOE->BSRR=GPIO_Pin_5
#define VL6180X_CE3_CLOSE GPIOE->BRR=GPIO_Pin_5
#define VL6180X_CE4_OPEN  GPIOE->BSRR=GPIO_Pin_3
#define VL6180X_CE4_CLOSE GPIOE->BRR=GPIO_Pin_3
#define VL6180X_CE5_OPEN	GPIOE->BSRR=GPIO_Pin_1
#define VL6180X_CE5_CLOSE GPIOE->BRR=GPIO_Pin_1
#define VL6180X_CE6_OPEN	GPIOB->BSRR=GPIO_Pin_9
#define VL6180X_CE6_CLOSE GPIOB->BRR=GPIO_Pin_9


/* variables -------------------------------------------------------------*/
	
/* functions ------------------------------------------------------------*/	
void Vl6180x_I2C_LowLevel_Init(void);
_I2C_ERR single_read(unsigned char DeviceAddr, unsigned short WriteAddr, unsigned char *pBuffer);
_I2C_ERR single_write(unsigned char DeviceAddr, unsigned short WriteAddr, unsigned char data);
_I2C_ERR double_write(unsigned char DeviceAddr, unsigned short WriteAddr, unsigned short data);
_I2C_ERR multi_write(unsigned char DeviceAddr, unsigned short WriteAddr, unsigned int data);
void VL6180X_LowLevel_Init(unsigned char DeviceAddr);
//void Vl6180x_I2C_LowLevel_Init(void);
void Vl6180X_Init(void);
	
	
#ifdef __cplusplus
}
#endif

#endif









/*----------------------The End of File---------------------------------------------------------*/
