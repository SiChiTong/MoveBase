/**-------------------------------------------------------------------------
*@ file				common.h
*@ brief			非模块化，初始化与配置
*@ author 		ZGJ
*@ vertion		V0.01
*@ date				2015-12-15
*@ statement	(C)	COPYRIGHT 2015 KSITRI.
--------------------------------------------------------------------------*/
#ifndef __COMMON_H
#define	__COMMON_H
/* Include files---------------------------------------------------------*/
#include "stm32f10x.h"
#include "sys_config.h"
#ifdef	__cplusplus
extern "C"
{
	

#endif
/* typedef ---------------------------------------------------------*/


	
typedef enum
{
	PZKEY_NONE=0,
	PZKEY1=1,
	PZKEY2=2,
	PZKEY3=3,
	PZKEY1_NO=4,
	PZKEY2_NO=5,
	PZKEY3_NO=6
}_PZ_KEY;
typedef enum
{
	EXCEPTION_NONE=0,		//正常
	MOTOR_NO_SPEED=1,		//堵转
	DRIVER_OVER_TEMP,
	DRIVER_POWER_ERR,
	OVER_POWER,
	LOW_POWER,
	DRIVER_OVER_CURRENT,		//驱动器过流
	MOTOR_OVER_LOAD,
	MOTOR_OVER_THMP,
	MOTOR_WINDING_OPEN,		//电机绕组开路
	DRIVER_COMMU_ERR,
	DRIVER_HALL_ERR,
	ENCODER_ERR,
	ERR_OTHERS,
	PZKEY_ON,		//碰撞开关
	CMD_OVERTIME,			//通讯超时
	STOPSWITCH_ON,		//急停按钮
	LIGHT_SENSOR_ON,		//激光传感器触发
	V_ERR,
	IOCTRLSTOP_ON,
	OVER_SPEED,
	OVER_CURRENT,
	PWM_CUTOFF,
	
	OVER_POSITION,
	DRIVER_ERR
}_EXCEPTION_KIND;

typedef struct
{
	unsigned char FallSensorUpThread;
	unsigned char FallSensorDownThread;
}_FallSensor_Thread;

typedef enum
{
	STA_STOP_STANDBY=0,		//静止待机
	STA_MOVE_NORMAL=1,		//正常运动
	STA_STOPSWITCH_ON=2,		//急停开关触发
	STA_ERR=3								//错误
}_ROBOTSTA;

typedef union
{
	 struct{
		uint32_t start_address; // the address of the bin saved on flash.
		uint32_t length; // file real length
		uint8_t version[8];
		uint8_t type; // B:bootloader, P:boot_table, A:application, 
		uint8_t upgrade_type; //u:upgrade, 
		uint8_t reserved[6];
	}boot_table_t;
 char data[24];
 }BOOTLOADER_UNION;

/* define -----------------------------------------------------------*/
#define		OpenBeep()		GPIOB->BSRR=GPIO_Pin_15
#define		CloseBeep()		GPIOB->BRR=GPIO_Pin_15
	
	//电源使能控制
#define OpenSysPower() (GPIO_SetBits(GPIOB,GPIO_Pin_0))
#define CloseSysPower() (GPIO_ResetBits(GPIOB,GPIO_Pin_0))
//开关机按键检测
#define IsPlayOnPush() (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1))
#define IsPlayNotOnPush() (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1) == 0)

#define Work_LedOn()		GPIOG->BRR=GPIO_Pin_9
#define Work_LedOff()		GPIOG->BSRR=GPIO_Pin_9

#if MC_V1_1_BOARD 
	#define ReadStopSwitch	(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9))
	#define RS485_DIR_PIN		GPIO_Pin_8
	#define RS485_DIR_PORT	GPIOA
#endif


	#define RS485DataOut()	RS485_DIR_PORT->BSRR=RS485_DIR_PIN
	#define RS485DataIn()		RS485_DIR_PORT->BRR=RS485_DIR_PIN

#define IO_CTRL_PIN1		GPIO_Pin_12
#define IO_CTRL_PIN2		GPIO_Pin_11
#define IO_CTRL_PORT		GPIOD

#define	ReadIOCtrlPin(x)	GPIO_ReadInputDataBit(IO_CTRL_PORT,x)
/* variables --------------------------------------------------------*/
//自建的系统状态寄存器,位带操作区.
//把“位带地址+位序号”转换成别名地址
#define BITBAND(addr, bitnum) ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
//把地址转换成指针
#define MEM_ADDR(addr) *((volatile unsigned long *) (addr))
	
#define GPIOout(GPIOx,bit)				MEM_ADDR(BITBAND((uint32_t)(&GPIOx->ODR), bit))
#define GPIOin(GPIOx,bit)					MEM_ADDR(BITBAND((uint32_t)(&GPIOx->IDR), bit))
	
extern volatile unsigned long SysStateRegister; //系统状态寄存器
	
#define		SafeModeSta			0			//bit0-安全模式标志位。1-打开，0-关闭
#define		PZKeySta				1			//bit1-碰撞开关标志。 1-碰撞开关使能，0-碰撞开关失能
#define		FallSensorSta		2			  //bit2-防跌落标志。1-触发，0-未触发
#define		EmergencyStop		3			//bit3-急停按钮。1-触发，0-未触发
#define		IOCtrlStop			4			//bit4-IO控制急停。1-触发，0-未触发
#define		MotorEnFlag			5			//bit5-电机是否允许运行,1-不允许运行,0-允许运行
#define		DriverInitFlag	    6			//bit6-驱动器初始化,1-初始化完成，0-初始化未完成
#define		SafeModeStop		7			//bit7-安全模式急停,1-触发急停，0-未触发急停

#define		SetSysStaBit(x)		do {MEM_ADDR(BITBAND((int)(&SysStateRegister), x)) = 0x1;} while(0)
#define		ClearSysStaBit(x)	do {MEM_ADDR(BITBAND((int)(&SysStateRegister), x)) = 0x0;} while(0)
#define		ReadSysStaBit(x)	MEM_ADDR(BITBAND((int)(&SysStateRegister), x))
	
#define PZKey1_Pin		GPIO_Pin_12
#define PZKey2_Pin		GPIO_Pin_13
#define	PZKey3_Pin		GPIO_Pin_14
#define PZKey_Port		GPIOB

#define ReadPZKey(x)	GPIO_ReadInputDataBit(PZKey_Port,x)			//x只能为PZkey的Pin脚

extern _FallSensor_Thread	gfallSensorThread;

extern int lt_count;
extern int rt_count;
extern char BoardID;
extern char UpdateFlag;
extern char UpdateData[];


	/* global functions--------------------------------------------------*/
void HardWare_LowLevel_Init(void);
_PZ_KEY PZKey_Detect(void);
unsigned char StopSwitchDet(void);
unsigned char IOCtrlDet1(void);
//unsigned char IOCtrlDet2(void);
unsigned char GetKey(void);
	
//void LEFT_PWM_Init(int period,int prescaler);
//void RIGHT8_PWM_Init(int period,int prescaler);
//void LEFT2_PWM_Init(int period,int prescaler);

//void MoveDirevtion(void);

void ID_NumberInit(void);
char GetIDnum(void);
void DisChargeInit(void);

unsigned char GetEStopState(void);
uint8_t GetCollisionState(void);
void Switchs_Init(void);

void Delayus(uint32_t us);
void _delay(unsigned int cnt);

//void TIM2_PWM_Init(int period,int prescaler);

	#ifdef __cplusplus
}
#endif

#endif
/*-----------------------The End of File----------------------------------*/
