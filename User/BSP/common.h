/**-------------------------------------------------------------------------
*@ file				common.h
*@ brief			��ģ�黯����ʼ��������
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
	EXCEPTION_NONE=0,		//����
	MOTOR_NO_SPEED=1,		//��ת
	DRIVER_OVER_TEMP,
	DRIVER_POWER_ERR,
	OVER_POWER,
	LOW_POWER,
	DRIVER_OVER_CURRENT,		//����������
	MOTOR_OVER_LOAD,
	MOTOR_OVER_THMP,
	MOTOR_WINDING_OPEN,		//������鿪·
	DRIVER_COMMU_ERR,
	DRIVER_HALL_ERR,
	ENCODER_ERR,
	ERR_OTHERS,
	PZKEY_ON,		//��ײ����
	CMD_OVERTIME,			//ͨѶ��ʱ
	STOPSWITCH_ON,		//��ͣ��ť
	LIGHT_SENSOR_ON,		//���⴫��������
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
	STA_STOP_STANDBY=0,		//��ֹ����
	STA_MOVE_NORMAL=1,		//�����˶�
	STA_STOPSWITCH_ON=2,		//��ͣ���ش���
	STA_ERR=3								//����
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
	
	//��Դʹ�ܿ���
#define OpenSysPower() (GPIO_SetBits(GPIOB,GPIO_Pin_0))
#define CloseSysPower() (GPIO_ResetBits(GPIOB,GPIO_Pin_0))
//���ػ��������
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
//�Խ���ϵͳ״̬�Ĵ���,λ��������.
//�ѡ�λ����ַ+λ��š�ת���ɱ�����ַ
#define BITBAND(addr, bitnum) ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
//�ѵ�ַת����ָ��
#define MEM_ADDR(addr) *((volatile unsigned long *) (addr))
	
#define GPIOout(GPIOx,bit)				MEM_ADDR(BITBAND((uint32_t)(&GPIOx->ODR), bit))
#define GPIOin(GPIOx,bit)					MEM_ADDR(BITBAND((uint32_t)(&GPIOx->IDR), bit))
	
extern volatile unsigned long SysStateRegister; //ϵͳ״̬�Ĵ���
	
#define		SafeModeSta			0			//bit0-��ȫģʽ��־λ��1-�򿪣�0-�ر�
#define		PZKeySta				1			//bit1-��ײ���ر�־�� 1-��ײ����ʹ�ܣ�0-��ײ����ʧ��
#define		FallSensorSta		2			  //bit2-�������־��1-������0-δ����
#define		EmergencyStop		3			//bit3-��ͣ��ť��1-������0-δ����
#define		IOCtrlStop			4			//bit4-IO���Ƽ�ͣ��1-������0-δ����
#define		MotorEnFlag			5			//bit5-����Ƿ���������,1-����������,0-��������
#define		DriverInitFlag	    6			//bit6-��������ʼ��,1-��ʼ����ɣ�0-��ʼ��δ���
#define		SafeModeStop		7			//bit7-��ȫģʽ��ͣ,1-������ͣ��0-δ������ͣ

#define		SetSysStaBit(x)		do {MEM_ADDR(BITBAND((int)(&SysStateRegister), x)) = 0x1;} while(0)
#define		ClearSysStaBit(x)	do {MEM_ADDR(BITBAND((int)(&SysStateRegister), x)) = 0x0;} while(0)
#define		ReadSysStaBit(x)	MEM_ADDR(BITBAND((int)(&SysStateRegister), x))
	
#define PZKey1_Pin		GPIO_Pin_12
#define PZKey2_Pin		GPIO_Pin_13
#define	PZKey3_Pin		GPIO_Pin_14
#define PZKey_Port		GPIOB

#define ReadPZKey(x)	GPIO_ReadInputDataBit(PZKey_Port,x)			//xֻ��ΪPZkey��Pin��

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
