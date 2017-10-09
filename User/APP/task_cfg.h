/**-------------------------------------------------------------------------
*@ file				task_cfg.h
*@ brief			user tasks and system objects define
*@ author 		ZGJ
*@ vertion		V0.01
*@ date				2015-12-15
*@ statement	(C)	COPYRIGHT 2015 KSITRI.
--------------------------------------------------------------------------*/
#ifndef __TASK_CFG_H
#define __TASK_CFG_H
/*---------------------------------------------
*			name					 prio
*	task_AdcDetect			16
* task_PidCalc				6       
* task_Protocol				10
* task_Sensordet			14
* task_EHandler				8       
* task_MotionCtrl			9       

---------------------------------------------*/

#ifdef 	EXTERN_GLOBALS
#define EXTERN
#else
#define EXTERN extern
#endif

/* Include files-------------------------------------------------------------*/
#include "ucos_ii.h"
/* typdef ----------------------------------------------------------*/

/* define ----------------------------------------------------------*/

/* variables ------------------------------------------------------*/
#ifdef __cplusplus
extern "C"
{
#endif
	
EXTERN		OS_EVENT *CmdOverTimeSem;
EXTERN		OS_EVENT *CommuQuerySem;
EXTERN		OS_EVENT *MoveSem;
	
EXTERN		OS_EVENT *StopSem;
EXTERN		OS_EVENT *StopEncoderSem;
	
EXTERN		OS_EVENT *PowerUpdateSem;
	
EXTERN		OS_EVENT *StopLockSem;
EXTERN		OS_EVENT *MoveLockSem;
	
#define MODEBUS_REV_QSIZE	10
EXTERN  OS_EVENT	*ModBusRevQBOX;
EXTERN  void* ModBusRevQBuffer[MODEBUS_REV_QSIZE];
/**-----------------------------------------------------------------
*@ ADC检测任务:	1.周期执行
*@              2.执行ADC检测任务     
-------------------------------------------------------------------*/
#define   TASK_ADCDETECT_PRIO        16   //任务优先级
#define   TASK_ADCDETECT_STKSIZE     256 //堆栈大小
EXTERN    OS_STK    TaskAdcDetectStk[TASK_ADCDETECT_STKSIZE]; //堆栈区
void Task_AdcDetect(void *pdata);
	
EXTERN    OS_EVENT *Adc1CycSem;
EXTERN		OS_EVENT *Adc3CycSem;

EXTERN    OS_EVENT *Usart1Cal;
EXTERN    OS_EVENT *Usart3Cal;
	
/**-----------------------------------------------------------------
*@ PID计算任务:
*@              1.由定时器控制周期执行，周期为20ms    
-------------------------------------------------------------------*/
#define   TASK_PIDCALC_PRIO        6   //任务优先级
#define   TASK_PIDCALC_STKSIZE     512 //堆栈大小
EXTERN    OS_STK    TaskPidCalcStk[TASK_PIDCALC_STKSIZE]; //堆栈区
void Task_PidCalc(void *pdata);
	
EXTERN    OS_EVENT *PidCalcSem;

/**-----------------------------------------------------------------
*@ 协议解析任务：
*@              1.PC每50ms发送一条指令给本任务    
-------------------------------------------------------------------*/
#define   TASK_PROTOCOL_PRIO        10   //任务优先级
#define   TASK_PROTOCOL_STKSIZE     512 //堆栈大小
EXTERN    OS_STK    TaskProtocolStk[TASK_PROTOCOL_STKSIZE]; //堆栈区
void Task_Protocol(void *pdata);

#define		TASK_PROTOCOL_QSIZE				10u
EXTERN		OS_EVENT	*ProtocolQBOX;
EXTERN		void*		ProtocolQBuffer[TASK_PROTOCOL_QSIZE];

/**-----------------------------------------------------------------
*@ 跌落于碰撞传感器检测任务：
*@              1.周期执行    
-------------------------------------------------------------------*/
#define   TASK_SENSORDET_PRIO        14   //任务优先级
#define   TASK_SENSORDET_STKSIZE     512 //堆栈大小
EXTERN    OS_STK    TaskSensorDetStk[TASK_SENSORDET_STKSIZE]; //堆栈区
void Task_SensorDet(void *p_arg);

/**-----------------------------------------------------------------
*@ 运动控制任务：
*@              1.由消息队列驱动    
-------------------------------------------------------------------*/
#define   TASK_MOTIONCTRL_PRIO        9   //任务优先级
#define   TASK_MOTIONCTRL_STKSIZE     512 //堆栈大小
EXTERN    OS_STK    TaskMotionCtrlStk[TASK_MOTIONCTRL_STKSIZE]; //堆栈区
void Task_MotionCtrl(void *pdata);

#define		TASK_MOTIONCTRL_QSIZE				10u
EXTERN		OS_EVENT	*MotionCtrlQBOX;
EXTERN		void*		MotionCtrlQBuffer[TASK_MOTIONCTRL_QSIZE];
/**-----------------------------------------------------------------
*@ 异常处理任务：
*@              1.由消息队列触发  
-------------------------------------------------------------------*/
#define   TASK_EHANDLER_PRIO        8   //任务优先级
#define   TASK_EHANDLER_STKSIZE     256 //堆栈大小
EXTERN    OS_STK    TaskEHandlerStk[TASK_EHANDLER_STKSIZE]; //堆栈区
void Task_EHandler(void *p_arg);

#define		TASK_EHANDLER_QSIZE				10u
EXTERN		OS_EVENT	*EHandlerQBOX;
EXTERN		void*		EHandlerQBuffer[TASK_EHANDLER_QSIZE];

EXTERN    OS_EVENT  *Usart1Cal;
EXTERN    OS_EVENT  *Usart3Cal;


#define   TASK_DRIVERSTATE_PRIO        5   //任务优先级
#define   TASK_DRIVERSTATE_STKSIZE     512 //堆栈大小
EXTERN    OS_STK    TaskDriverStateStk[TASK_DRIVERSTATE_STKSIZE]; //堆栈区
void Task_DriverState(void *pdata);

EXTERN    OS_EVENT  *UsartsCMD;

EXTERN    OS_EVENT *Usart2CMDSem;//工控机命令信号量

#define   TASK_LINEARACTUATOR_PRIO        18   //任务优先级
#define   TASK_LINEARACTUATOR_STKSIZE     128 //堆栈大小
EXTERN    OS_STK    TaskLinearActuatorStk[TASK_LINEARACTUATOR_STKSIZE]; //堆栈区
void Task_LinearActuator(void *pdata);

	
/* Functions ----------------------------------------------------------*/
void OS_SysObject_Create(void);
void OS_Task_Create(void);

#ifdef __cplusplus
}
#endif
#endif

/*-------------------The End of File---------------------------------------*/
