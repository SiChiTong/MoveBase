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
//*************************数据类型声明**********************//
#define N_STOP      0x0000
#define E_STOP      0x0001
#define F_STOP      0X0002	
	
//左右电机序号定义
typedef  enum _MOTOR_NUM
{
    MOTOR_LEFT = 0x01,
    MOTOR_RIGHT = 0x02
}MOTOR_NUM;

//电机方向定义，正对电机输出轴看为准
typedef enum _MOTOR_DIR
{
	MOTOR_DIR_NONE=0x00,
    MOTOR_DIR_CW = 0x01,
    MOTOR_DIR_CCW = 0x02
}MOTOR_DIR;
//运动类型
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
	unsigned int EncoderSetNum;			//此次运动的码盘计数值
	float LMotorV;				//左轮速度
	float RMotorV;				//右轮速度
	MOTOR_DIR LMotorDir;		//左轮方向
	MOTOR_DIR RMotorDir;		//右轮方向	
	unsigned char MotorEN;		//电机是否处在运动状态
	unsigned char MotorLockSta;		//执行停止命令后需要执行电机自锁0-执行的运动命令，1-执行停止命令，需要自锁
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
 * 电机由0.2R的电阻进行采样，堵转电流为2A，这里设置最大电流为1A的保护，1A时，电压为0.2V，
 * 此时AD采集的理论值为248,但电流值与速度有关，速度小时，电流也比较小。实际测试（地板，
地砖，地毯）, 阈值为>=80
*/
#define MAX_MOTOR_CURRENT         80
#define MIDDLE_CODER_CNT          0x7FFF   //正交编码器计数值中值
//#define MIDDLE_CODER_CNT          0x00   //20160426,Zero

#define MOVECMDLISTLEN        sizeof(Movectrl_list)
#define MOVECMDLEN            sizeof(MoveCtrl_t)

//编码器定时器控制,控制编码器是否计数
#define OpenRightEncoder() 	(TIM_Cmd(TIM4,ENABLE)) //打开左编码器
#define CloseRightEncoder() (TIM_Cmd(TIM4,DISABLE))
#define OpenLeftEncoder() 	(TIM_Cmd(TIM3,ENABLE)) //打开右编码器
#define CloseLeftEncoder() 	(TIM_Cmd(TIM3,DISABLE))

//PID 调节周期开关控制
#define OpenMotorCLoopControl()		do{TIM_Cmd(TIM5, ENABLE);TIM5->CNT = 0;}while(0)
#define CloseMotorCLoopControl()	do{TIM_Cmd(TIM5, DISABLE);TIM5->CNT = 0;}while(0)

//打开驱动器电源
#define OpenDriverPower()				GPIOC->BSRR=GPIO_Pin_0
#define CloseDriverPower()			GPIOC->BRR=GPIO_Pin_0
#define ReadDriverPowerSta()		GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_0)


//变量定义-----------------------------------------------------------------
extern int32_t LEncoderCycCnt;
extern int32_t REncoderCycCnt;

extern _MOTION_CTRL_PRM	gMotionCtrl;
extern _ENCODER_INF	gEncoderInf;
extern _ROBOT_POS	gRobotPos;
extern _EncoderCycCnt gEncoderCycCnt;

extern _MOTION_PARAM gMotiondata;

extern float PWM_L,PWM_R;
//*************************************全局函数声明***************************************************//
void Motor_Config(void);
uint8_t ReadMotorCurEN(void);
//xdid--x方向运动的距离，vx--x方向运动的速度，thta--绕转动中心的旋转角度, vthta--旋转速度
//对于好博特暂不支持同时进行直线运动与转动.xid > 0:向前，xid < 0:向后，thta > 0:左转，
//thta<0, 右转。xdis与thta同时为0，则表示停止.
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

