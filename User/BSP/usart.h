/**-------------------------------------------------------------------------
*@ file				usart.h
*@ brief			USART driver,communicate with PC
*@ author 		ZGJ
*@ vertion		V0.01
*@ date				2015-12-15
*@ statement	(C)	COPYRIGHT 2015 KSITRI.
--------------------------------------------------------------------------*/
#ifndef __USART_H
#define __USART_H

#include "stm32f10x.h"
#include "ucos_ii.h"
#include <string.h>
#include <stdbool.h>
#include "modbus.h"
#include "sys_config.h"

#ifdef __cplusplus
extern "C"
{
#endif
/* typedef -------------------------------------------------------------*/
	
////协议选择
//#define OLD_FEEDBACK  1
//#define NEW_FEEDBACK  0
	
typedef struct
{
	unsigned char header;
	unsigned char vx[2];
	unsigned char vrotate[2];
	unsigned char cmd;
	unsigned char sensor_limit_up;
	unsigned char sensor_linit_down;
	unsigned char reserve[2];
	unsigned char sum;
	unsigned char frame_end;	
}_RECV_DATA;

typedef union
{
	unsigned char fc[4];
	float f;
}FLOAT;

//typedef struct			//底盘状态
//{
//	bool sta            : 4;
//	bool safemode_on    : 1;
//	bool sys_on         : 1;
//	bool io_stop        : 1;
//	bool emergency_stop : 1;
//}_RobotSta;

typedef union
{
	struct
	{
		bool safe_mode : 1;	
		bool power_on  : 1;
		bool normal    : 1;
		bool io_stop   : 1;
		bool switch_stop : 1;
		bool low_power : 1;
		bool over_speed   : 1;
		bool res          :1;

	}ROBOT_STATE_STRUCT;
	char ROBOT_STATE;
}ROBOT_STATE_UNION;



#if NEW_FEEDBACK
typedef struct
{
	FLOAT dx;
	FLOAT dy;
	FLOAT dthta;
	short vx;
	short vthta;
	//爱控驱动器
	MOTOR_DRIVER_UNION LEFT;
	MOTOR_DRIVER_UNION RIGHT;
	
	ZL_DRIVER_UNION LEFT_DRIVER;
  ZL_DRIVER_UNION RIGHT_DRIVER;
	ZL_DRIVER_UNION UP_DRIVER;
	
	//driver working state
	uint8_t leftDriver;
	uint8_t rightDriver;
	uint8_t loadDriver;
	//driver enable state
	uint8_t l_enable;
	uint8_t r_enable;
	uint8_t load_enable;
	
	ROBOT_STATE_UNION  WORK_STATE;			//底盘状态
	uint8_t sensor_sta;		//碰撞开关与急停开关状态，0-未触发，1-触发
	uint8_t lightsensordata[5];
	uint8_t battery;
	uint8_t comu;			//通讯质量
	uint8_t exception;
	uint8_t workState;//0 -- ready 1 -- pause 2 -- run 
//	uint8_t pause;      //暂停
//	uint8_t run;        //运行
	uint8_t collisions; //碰撞开关
	uint8_t loadSwitchs; //顶升有关传感器
	
	short l_current;
	short r_current;
    
    uint8_t log;
}_CMD_FEEDBACK;
#endif

typedef enum
{
	CMD_NONE=0x00,
	CMD_OPEN_SAFEMODE=0x01,
	CMD_CLOSE_SAFEMODE=0x02,
	CMD_SENSOR_PARAM_CFG=0x03,
	CMD_CLEAR_SYS_ON_FLAG=0x04,
	CMD_CLEAR_ERR_FLAG=0x05
}_REV_CMD;

typedef enum
{
	MOTOR_ENABLE=0x00,

	POSITION_CTLMODE=0x02,
	
	CONTROL_MODE = 0x36,
	
	SPEED_P=0x40,
	SPEED_I=0x41,
	SPEED_D=0x42,
	
	POSITION_P=0x1A,
	POSOTION_D=0x1B,
	POSITION_FB=0x1C,//位置模式前馈
	POSITION_VLIMIT=0x1D,//位置模式下的速度限幅
	
	SPEED_ACDC=0x0A,
	
	POSITION_H=0x50,
	POSITION_L=0x05,
	
	POSITION_MODE=0x52,//绝对位置、相对位置设定
	SEARCH_Z=0x53,
	
	CLEAR_ERR=0x4A,
	GETSTATE=0x80
}DRIVERADD_ENUM;



typedef union
{
	struct
	{
		char tar_add;
		char tar_data;
		char status_add;
		char status_data;
		
		//母线电压
		char U_add;
		short U_data;
		char U_sum;
		//输出电流
		char I_add;
		short I_data;
		char I_sum;
		//输出转速
		char RPM_add;
		short RPM_data;
		char RPM_sum;
		//给定位置高位
		char TPosH_add;
		short TPosH_data;
		char TPosH_sum;
		//给定位置低位
		char TPosL_add;
		short TPosL_data;
		char TPosL_sum;
		//反馈位置高位
		char FPosH_add;
		short FPosH_data;
		char FPosH_sum;
		//反馈位置低位
		char FPosL_add;
		short FPosL_data;
		char FPosL_sum;

	}DATA;
	uint8_t UsartRevData[32];
}DRIVERDATA_UNION;

typedef struct
{
	char port;
	char sendflag;
	char sendcount;
	char revcount;
}DRIVERSTATE_STRUCT;

typedef struct
{
	DRIVERADD_ENUM add;
	char L_h_value;
	char L_l_value;
	
	char R_h_value;
	char R_l_value;
}USARTCMD_STRUCT;


typedef enum
{
	ZERO_ENUM = 0X00,
	//基本指令
	CLEAR_POWERSTATE = 0x60,
	READ_POWERSTATE  = 0x61,
	
//	SET_FALLENLIMIT  = 0x62,
//	READ_FALLENLIMIT = 0x63,
	CLEAR_SWITCH_STATE  = 0x62,
	READ_LOADSWITCHS = 0X63,
	
	SET_SENSOR       = 0x64,
	READ_SENSOR      = 0x65,
	
	CLEAR_ERRSTATE   = 0x66,
	READ_ERRSTATE    = 0x67,
	
	ROBOT_MOVE       = 0x68,
	
	LINEAR_LOAD      = 0x69,
	READ_MPU9250     = 0x6A,
	
	//特殊指令
	READY2UPDATE     = 0x6F,
	GETVERSION       = 0x6E,
	CMDERROR         = 0x6D,
    CHARGING_STATE   = 0x6C
}USART2CMD_ENUM;

/* define -------------------------------------------------------------*/
#define		PROTOCOL_SEND_SIZE			255		//32
#define		PROTOCOL_RECIVE_SIZE		255
#define		MODBUS_UART							UART4

typedef struct _USART2_REV_BUF
{
	unsigned char RevBuf[PROTOCOL_RECIVE_SIZE];
	unsigned char	length;
	struct _USART2_REV_BUF *next;
}USART2RevBuf_t;
	
/* variables ----------------------------------------------------------*/
extern _CMD_FEEDBACK	cmdfeed;
extern unsigned char Uart4RevBuffer[64];
extern unsigned char Usart2RevBuffer[PROTOCOL_RECIVE_SIZE];
extern OS_MEM	*USART2RevList;
extern USART2RevBuf_t	USART2RevPart[11][1];
extern USART2RevBuf_t *usart2revbufhead;

extern unsigned char Usart1SendBuffer[];
//extern unsigned char Usart1RevBuffer[];

extern unsigned char Usart3SendBuffer[];
//extern unsigned char Usart3RevBuffer[];

extern unsigned int revcnt;
extern unsigned int procnt;

extern DRIVERDATA_UNION Usart1Data;
extern DRIVERDATA_UNION Usart3Data;

extern DRIVERSTATE_STRUCT Usart1State;
extern DRIVERSTATE_STRUCT Usart3State;

extern USARTCMD_STRUCT UsartCMDMbox;

/* functions ----------------------------------------------------------*/
void USART_LowLevel_Init(void);
void USART2_SendEnable(unsigned char NumToSend);
void USART_SendStr(USART_TypeDef *USARTx, uint8_t *str);
void Cmd_Send(_CMD_FEEDBACK *pcmd);
void Usart_SendBuffer(USART_TypeDef *USARTx,unsigned char *pBuf, unsigned short len);

void USART2RevHeadInit(void);
void Usart2AddRevBuf(unsigned char *buf,unsigned char len);
unsigned char GetUsart2RevListLen(void);

void USART1_Config(int baud);
void USART3_Config(int baud);
void UART5_Config(uint32_t baud);

void SendCMD(USART_TypeDef *USARTx,DRIVERADD_ENUM add,char hvalue,char lvalue);
void HLSSendCMD(USART_TypeDef *USARTx,uint8_t mode,uint16_t add,uint16_t nums,uint16_t *data);
//void HLSSendCMD(USART_TypeDef *USARTx,uint8_t mode,uint16_t add,uint16_t len);
void HLSGetStates(uint8_t mode,uint16_t add,uint16_t nums,uint16_t *data);

//void Usart2Feedback(USART2CMD_ENUM cmd,char len,char *data);
void Usart2Feedback(USART2CMD_ENUM cmd,uint8_t len,uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif

