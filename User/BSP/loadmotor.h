#ifndef __LOADMOTOR_H
#define __LOADMOTOR_H
#include "stm32f10x.h"


#define HLS_READ   0X03
#define HLS_WRITE  0X06
#define HLS_WRITES 0X16
#define HLS_WR     0X50

typedef enum
{
	ERR_NONE = 0X00,
	ERR_OVERTIME = 0X01,
	ERR_NOACK = 0X02,
	ERR_ENCODER_AB = 0X03,
	ERR_ENCODER_UVW = 0X04,
	ERR_ELECGEAR = 0X05,
	ERR_LOWPOWER = 0X06,
	ERR_OVERCURRENT_A = 0X07,
	ERR_OVERCURRENT_B = 0X08,
	ERR_OVERCURRENT_C = 0X09,
	ERR_OVER_SPEED = 0X0A,
	ERR_OVER_POSCOUNT = 0X0B,
	ERR_OVER_POS = 0X0C,
	ERR_NOSPEED = 0X0D,
	ERR_ENCODERZ_LOST = 0X0E,
	ERR_ENCODERZ_OVER = 0X0F,
	
	FAULT_LOWRES = 0X10,
	FAULT_FRAM_ERR = 0X11,
	FAULT_OVER_LOAD = 0X12,
	FAULT_OVER_POWER = 0X13,
	FAULT_PARAMSERR = 0X14,
	FAULT_1PHASE_ERR = 0X15,
	FAULT_2PHASE_ERR = 0X16,
	FAULT_MATCH_ERR = 0X17,
	
	WRR_DISABLE = 0X20,
	WRR_PWMOFF = 0X40,
	WRR_CONNECT = 0x80
}LOCAL_ERRCODE_ENUM;

typedef enum
{
	/****Dn****/
	Dn01_freq = 0x01,
	Dn1D_pos = 0x1D,
	Dn36_err = 0x36,
	Dn5A_state = 0x5A,
	
	/***Fn***/
	Fn010_enable = 0x2720,
	Fn011_clearErr = 0x2721
	
}DRIVER_CMD_ENUM;

typedef enum
{
	E_0000 = 0X0000,
	F_1200 = 0X1200,
	E_1301 = 0X1301,
	E_1401 = 0X1401,
	E_1500 = 0X1500,
	E_1510 = 0X1510,
	F_1600 = 0X1600,
	E_1700 = 0X1700,
	E_2200 = 0X2200,
	E_2500 = 0X2500,
	E_2501 = 0X2501,
	E_2502 = 0X2502,
	E_2510 = 0X2510,
	E_2520 = 0X2520,
	F_2530 = 0X2530,
	F_2600 = 0X2600,
	E_2610 = 0X2610,
	E_2645 = 0X2645,
	F_2660 = 0X2660,
	F_2661 = 0X2661,
	E_2900 = 0X2900,
	F_3110 = 0X3110,
	E_3600 = 0X3600,
	E_3601 = 0X3601
	
}DRIVER_ERROR_ENUM;

//extern uint16_t errCodes[24][2];

uint32_t crc_chk(uint8_t *data,uint8_t length);
void UART5_Config(uint32_t baud);
void LOAD8_PWM_Init(int period,int prescaler);
void  LoadPowerSwitch_Init(void);
void UART5_SendDatas(uint8_t *data,uint8_t length);
void UART5_SendCMD(USART_TypeDef *USARTx,uint8_t mode,uint16_t add,uint16_t nums,uint16_t *data);
void LoadSpeed(float v);
uint8_t GetSwitchsState(void);

void Collisions_Init(void);
uint16_t GetCollisionsState(void);
uint8_t GetCollisionsState_3(void);

void Limits_Init(void);
uint8_t GetLimitsState(void);

uint8_t ErrCode(uint16_t in);

#endif

