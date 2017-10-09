/**-------------------------------------------------------------------------
*@ file				modbus.c
*@ brief			motor driver
*@ author 		ZGJ
*@ vertion		V0.01
*@ date				2015-12-15
*@ statement	(C)	COPYRIGHT 2015 KSITRI.
--------------------------------------------------------------------------*/
#ifndef __MODBUS_H
#define __MODBUS_H
#include <stdbool.h>
#ifdef __cplusplus
extern "C"
{
#endif
	
#define MAXFREQ 1800
	
typedef enum {
	MDERR_SUCCESS,		// �ɹ�
/*����ΪMODBUS����*/
	MDERR_FUNC = 0x01,	// ��Ч������
	MDERR_ADDR,			// ��Ч�Ĵ�����ַ
	MDERR_VALUE,		// ��Ч�Ĵ�������
	MDERR_FAIL,			// ����ʧ��
	MDERR_ENSURE,		// �����ѱ�ȷ��
	MDERR_BUSY,			// �豸æ
	MDERR_PARITY = 0x08,	// ��ż�������
	MDERR_BADGATEWAY = 0x0a, // �����õ�����·��
	MDERR_GATEWAYERR,		// ����Ŀ���豸��Ӧʧ��
/*����Ϊ��չ����*/
	MDERR_FORBIDDEN = 0x40,	 // ��ֹ����
	MDERR_NOTSTUDY = 0x60,	 // ��δѧϰ
	MDERR_NOTOPEN = 0x80, // ����δ��
	MDERR_COMM,			// ͨѶ����	(�ӻ����صĵ�ַ����Ӧ���һ��)
	MDERR_TIMEOUT,		// ������ʱ
	MDERR_BUFFER,		// ���ݳ��ȳ���������������
	MDERR_UNDEF = 0xff	// δ�������
} MDRESULT;

typedef enum {
	FUNC_READMULTIREG = 0x03,
	FUNC_WRITESINGLEREG = 0x06,
	FUNC_WRITEMULTIREG = 0x10,
	FUNC_READDEVID = 0x2b,
} FUNCCODE;

typedef enum {
	DRIVER_ERR_NONE=0,
	DRIVER_HAVENOT_STUDY=1,
	DRIVER_NO_SPEED=2,	//��תֹͣ
	DRIVER_HOWER_ERR=3,	//��������
	DRIVER_SPEED_NOTACHIVE=4,//�ﲻ��Ŀ���ٶ�
	DRIVER_WINDING_OPEN=5
} DRIVERERR;

typedef union
{
 struct
	{
/*		
		bool res :2;
		bool connect : 1;
		bool target_speed : 1;
		bool hall  : 1;
		bool encoder :1 ;
		bool temp  : 1;
		bool block : 1;	
*/
		
	volatile	bool block          : 1 ;	
	volatile 	bool temp  					: 1 ;
	volatile 	bool encoder			  : 1 ;
	volatile 	bool hall  					: 1 ;
	volatile 	bool target_speed 	: 1 ;
	volatile	bool connect 				: 1 ;
	volatile  bool winding_open	  : 1 ;	
		bool res 						: 1 ;
				
	}MOTOR_DRIVER_STRUCT;
	char MOTOR_DRIVER_STATE;
}MOTOR_DRIVER_UNION;



typedef union
{
	struct
	{
		bool over_i    : 1;	
		bool encoder   : 1;
		bool position  : 1;
		bool over_load : 1;
		bool temp      : 1;	
		bool connected : 1;
		bool pwmoff :  1 ;
		bool res       : 1;
	}MOTOR_DRIVER_STRUCT;
	char MOTOR_DRIVER_STATE;
}ZL_DRIVER_UNION;


//typedef struct
//{
//	float SetPoint;
//	float Kp;
//	float Ki;
//	float Kd;
//	float LastErr;
//	float PreErr;
//}PID_STRUCT;
/*------define -------------------------------------------*/

//extern PID_STRUCT PID_Data;

typedef struct
{
	short Current_I;//��ǰ����
	short Delt_I;//��������
}GETCURRENT_STRUCT;

extern char CleaerLeftErrFlag;
extern char CleaerRightErrFlag;

extern ZL_DRIVER_UNION LEFT_DRIVER;
extern ZL_DRIVER_UNION RIGHT_DRIVER;

/* ---------------------------Function ------------------------------------------*/
MDRESULT ReadDevId(unsigned char bDevAddr, unsigned char bDevIdCode, unsigned char bObjectId, unsigned char *pRecvBuf, unsigned short *pwBufSize);
unsigned char IsFrame(unsigned char addr, unsigned char *pBuf, unsigned short wLen);
MDRESULT WriteMultiRegs(unsigned char bDevAddr, unsigned short wRegAddr, unsigned short *pwRegBuf, unsigned short wRegNum, unsigned short *pwActAddr , unsigned short *pwActNum);
MDRESULT WriteSingleReg(unsigned char bDevAddr, unsigned short wRegAddr, unsigned short wRegValue, unsigned short *pwActAddr, unsigned short *pwActNum);
MDRESULT ReadRegs(unsigned char bDevAddr, unsigned short wRegAddr,  unsigned short wRegNum, unsigned char *pRecvBuf, unsigned short *pwBufSize);
unsigned short CalcCrc16(unsigned char *pBuf, unsigned short dwLen,  unsigned char *pCRCHi, unsigned char *pCRCLo);
const unsigned char * GetSendBuf(unsigned short *pdwLen);
const unsigned char * GetRecvBuf(unsigned short *pdwLen);
	
unsigned char * FindFrame(unsigned char addr, unsigned char *pBuf, unsigned short wLen, unsigned short *pwLen);

void stopMotor(short stop_mode);
void setMotorSpeed(float sp_l, float sp_r);
int DriverConfig(void);
void LockMotor(void);
void Delayus(unsigned int us);
void _delay(unsigned int cnt);
//float PID_Calc(PID_STRUCT *p,float SetPoint,float Current);

char GetDriverState(char add);
void ClearDriverState(void);
short  GetCurrent(char add);

#ifdef __cplusplus
}
#endif
#endif

/*---------------------The End of File-----------------------------------------------------------*/
