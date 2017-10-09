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
*@ ADC�������:	1.����ִ��
*@              2.ִ��ADC�������     
-------------------------------------------------------------------*/
#define   TASK_ADCDETECT_PRIO        16   //�������ȼ�
#define   TASK_ADCDETECT_STKSIZE     256 //��ջ��С
EXTERN    OS_STK    TaskAdcDetectStk[TASK_ADCDETECT_STKSIZE]; //��ջ��
void Task_AdcDetect(void *pdata);
	
EXTERN    OS_EVENT *Adc1CycSem;
EXTERN		OS_EVENT *Adc3CycSem;

EXTERN    OS_EVENT *Usart1Cal;
EXTERN    OS_EVENT *Usart3Cal;
	
/**-----------------------------------------------------------------
*@ PID��������:
*@              1.�ɶ�ʱ����������ִ�У�����Ϊ20ms    
-------------------------------------------------------------------*/
#define   TASK_PIDCALC_PRIO        6   //�������ȼ�
#define   TASK_PIDCALC_STKSIZE     512 //��ջ��С
EXTERN    OS_STK    TaskPidCalcStk[TASK_PIDCALC_STKSIZE]; //��ջ��
void Task_PidCalc(void *pdata);
	
EXTERN    OS_EVENT *PidCalcSem;

/**-----------------------------------------------------------------
*@ Э���������
*@              1.PCÿ50ms����һ��ָ���������    
-------------------------------------------------------------------*/
#define   TASK_PROTOCOL_PRIO        10   //�������ȼ�
#define   TASK_PROTOCOL_STKSIZE     512 //��ջ��С
EXTERN    OS_STK    TaskProtocolStk[TASK_PROTOCOL_STKSIZE]; //��ջ��
void Task_Protocol(void *pdata);

#define		TASK_PROTOCOL_QSIZE				10u
EXTERN		OS_EVENT	*ProtocolQBOX;
EXTERN		void*		ProtocolQBuffer[TASK_PROTOCOL_QSIZE];

/**-----------------------------------------------------------------
*@ ��������ײ�������������
*@              1.����ִ��    
-------------------------------------------------------------------*/
#define   TASK_SENSORDET_PRIO        14   //�������ȼ�
#define   TASK_SENSORDET_STKSIZE     512 //��ջ��С
EXTERN    OS_STK    TaskSensorDetStk[TASK_SENSORDET_STKSIZE]; //��ջ��
void Task_SensorDet(void *p_arg);

/**-----------------------------------------------------------------
*@ �˶���������
*@              1.����Ϣ��������    
-------------------------------------------------------------------*/
#define   TASK_MOTIONCTRL_PRIO        9   //�������ȼ�
#define   TASK_MOTIONCTRL_STKSIZE     512 //��ջ��С
EXTERN    OS_STK    TaskMotionCtrlStk[TASK_MOTIONCTRL_STKSIZE]; //��ջ��
void Task_MotionCtrl(void *pdata);

#define		TASK_MOTIONCTRL_QSIZE				10u
EXTERN		OS_EVENT	*MotionCtrlQBOX;
EXTERN		void*		MotionCtrlQBuffer[TASK_MOTIONCTRL_QSIZE];
/**-----------------------------------------------------------------
*@ �쳣��������
*@              1.����Ϣ���д���  
-------------------------------------------------------------------*/
#define   TASK_EHANDLER_PRIO        8   //�������ȼ�
#define   TASK_EHANDLER_STKSIZE     256 //��ջ��С
EXTERN    OS_STK    TaskEHandlerStk[TASK_EHANDLER_STKSIZE]; //��ջ��
void Task_EHandler(void *p_arg);

#define		TASK_EHANDLER_QSIZE				10u
EXTERN		OS_EVENT	*EHandlerQBOX;
EXTERN		void*		EHandlerQBuffer[TASK_EHANDLER_QSIZE];

EXTERN    OS_EVENT  *Usart1Cal;
EXTERN    OS_EVENT  *Usart3Cal;


#define   TASK_DRIVERSTATE_PRIO        5   //�������ȼ�
#define   TASK_DRIVERSTATE_STKSIZE     512 //��ջ��С
EXTERN    OS_STK    TaskDriverStateStk[TASK_DRIVERSTATE_STKSIZE]; //��ջ��
void Task_DriverState(void *pdata);

EXTERN    OS_EVENT  *UsartsCMD;

EXTERN    OS_EVENT *Usart2CMDSem;//���ػ������ź���

#define   TASK_LINEARACTUATOR_PRIO        18   //�������ȼ�
#define   TASK_LINEARACTUATOR_STKSIZE     128 //��ջ��С
EXTERN    OS_STK    TaskLinearActuatorStk[TASK_LINEARACTUATOR_STKSIZE]; //��ջ��
void Task_LinearActuator(void *pdata);

	
/* Functions ----------------------------------------------------------*/
void OS_SysObject_Create(void);
void OS_Task_Create(void);

#ifdef __cplusplus
}
#endif
#endif

/*-------------------The End of File---------------------------------------*/
