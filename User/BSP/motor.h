/**-------------------------------------------------------------------------
*@ file				motor.h
*@ brief			motor driver
*@ author 		ZGJ
*@ vertion		V0.01
*@ date				2015-12-15
*@ statement	(C)	COPYRIGHT 2015 KSITRI.
--------------------------------------------------------------------------*/
#ifndef __MOTOR_H
#define __MOTOR_H
/* Include Files------------------------------------------------*/
#include "stm32f10x.h"

#ifdef __cplusplus
extern "C"
{
#endif
//*************************������������**********************//
#define N_STOP      0x0000
#define E_STOP      0x0001
#define F_STOP      0X0002	
	
//���ҵ����Ŷ���
typedef  enum _MOTOR_NUM
{
    MOTOR_LEFT = 0x01,
    MOTOR_RIGHT = 0x02
}MOTOR_NUM;

//��������壬���Ե������ῴΪ׼
typedef enum _MOTOR_DIR
{
	MOTOR_DIR_NONE=0x00,
    MOTOR_DIR_CW = 0x01,
    MOTOR_DIR_CCW = 0x02
}MOTOR_DIR;
//�˶�����
typedef enum
{
	MOVE_TYPE_STOP=0x00,
	MOVE_TYPE_FORWARD=0x01,
	MOVE_TYPE_TURNLEFT=0x02,
	MOVE_TYPE_BACK=0x03,
	MOVE_TYPE_TURNRIGHT=0x04
}_MOVE_TYPE;

typedef struct 
{
	unsigned int EncoderSetNum;			//�˴��˶������̼���ֵ
	float LMotorV;				//�����ٶ�
	float RMotorV;				//�����ٶ�
	MOTOR_DIR LMotorDir;		//���ַ���
	MOTOR_DIR RMotorDir;		//���ַ���	
	unsigned char MotorEN;		//����Ƿ����˶�״̬
	unsigned char MotorLockSta;		//ִ��ֹͣ�������Ҫִ�е������0-ִ�е��˶����1-ִ��ֹͣ�����Ҫ����
	void (*move)(float,float,unsigned char);
}_MOTION_CTRL_PRM;

typedef struct
{
	int	LEncoderTotalCnt;
	int REncoderTotalCnt;
	void (*CalcPos) (int,int);
}_ENCODER_INF;

typedef struct
{
	float x;			//mm
	float y;			//mm
	float angle;		//1000*rad/s
}_ROBOT_POS;

typedef struct
{
	float vx;
	float vthta;
	unsigned char stop_mode;
}_MOTION_PARAM;

typedef struct
{
	int L_EncoderCycCnt;
	int R_EncoderCycCnt;
}_EncoderCycCnt;


/* Define ---------------------------------------------------------------*/

/*
 * �����0.2R�ĵ�����в�������ת����Ϊ2A����������������Ϊ1A�ı�����1Aʱ����ѹΪ0.2V��
 * ��ʱAD�ɼ�������ֵΪ248,������ֵ���ٶ��йأ��ٶ�Сʱ������Ҳ�Ƚ�С��ʵ�ʲ��ԣ��ذ壬
��ש����̺��, ��ֵΪ>=80
*/
#define MAX_MOTOR_CURRENT         80
#define MIDDLE_CODER_CNT          0x7FFF   //��������������ֵ��ֵ
//#define MIDDLE_CODER_CNT          0x00   //20160426,Zero

#define MOVECMDLISTLEN        sizeof(Movectrl_list)
#define MOVECMDLEN            sizeof(MoveCtrl_t)

//��������ʱ������,���Ʊ������Ƿ����
#define OpenRightEncoder() 	(TIM_Cmd(TIM4,ENABLE)) //���������
#define CloseRightEncoder() (TIM_Cmd(TIM4,DISABLE))
#define OpenLeftEncoder() 	(TIM_Cmd(TIM3,ENABLE)) //���ұ�����
#define CloseLeftEncoder() 	(TIM_Cmd(TIM3,DISABLE))

//PID �������ڿ��ؿ���
#define OpenMotorCLoopControl()		do{TIM_Cmd(TIM5, ENABLE);TIM5->CNT = 0;}while(0)
#define CloseMotorCLoopControl()	do{TIM_Cmd(TIM5, DISABLE);TIM5->CNT = 0;}while(0)

//����������Դ
#define OpenDriverPower()				GPIOC->BSRR=GPIO_Pin_0
#define CloseDriverPower()			GPIOC->BRR=GPIO_Pin_0
#define ReadDriverPowerSta()		GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_0)


//��������-----------------------------------------------------------------
extern int32_t LEncoderCycCnt;
extern int32_t REncoderCycCnt;

extern _MOTION_CTRL_PRM	gMotionCtrl;
extern _ENCODER_INF	gEncoderInf;
extern _ROBOT_POS	gRobotPos;
extern _EncoderCycCnt gEncoderCycCnt;

extern _MOTION_PARAM gMotiondata;

extern float PWM_L,PWM_R;
//*************************************ȫ�ֺ�������***************************************************//
void Motor_Config(void);
uint8_t ReadMotorCurEN(void);
//xdid--x�����˶��ľ��룬vx--x�����˶����ٶȣ�thta--��ת�����ĵ���ת�Ƕ�, vthta--��ת�ٶ�
//���ںò����ݲ�֧��ͬʱ����ֱ���˶���ת��.xid > 0:��ǰ��xid < 0:���thta > 0:��ת��
//thta<0, ��ת��xdis��thtaͬʱΪ0�����ʾֹͣ.
void RobotMove(float vx,float vthta,unsigned char stopsta);
void CalaRobotPos(int LEncoderCnt,int REncoderCnt);
void SetMotorPWMPulse(MOTOR_NUM motor, int32_t pulse);
void RobotMoveCtrl(float vx,float vthta, unsigned char stopsta);
void Motor_LowLevel_Init(void);

//20160413,Zero
void QuickStopMotor(short mode);

void SetPWMFreq(float vx,float vthta);
#ifdef __cplusplus
}
#endif

#endif

