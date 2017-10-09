/**-------------------------------------------------------------------------
*@ file				task_cfg.c
*@ brief			user tasks and system objects define
*@ author 		ZGJ
*@ vertion		V0.01
*@ date				2015-12-15
*@ statement	(C)	COPYRIGHT 2015 KSITRI.
--------------------------------------------------------------------------*/
#define EXTERN_GLOBALS
/* Include Files-------------------------------------------------*/
#include "task_cfg.h"

#include "usart.h"
#include "common.h"

/* typedef ---------------------------------------------------*/

/* define ----------------------------------------------------*/

/* variables -------------------------------------------------*/

/* functions ---------------------------------------------------*/

/**
*@ brief 		create system object
*@ para			none
*@ retval		none
*/
void OS_SysObject_Create(void)
{
	unsigned char err=0;
	Adc1CycSem=OSSemCreate(0);
	Adc3CycSem=OSSemCreate(0);
	PidCalcSem=OSSemCreate(0);
	
	Usart1Cal=OSSemCreate(0);
	Usart3Cal=OSSemCreate(0);
	UsartsCMD=OSMboxCreate(0);
	Usart2CMDSem = OSSemCreate(0);
	
	CmdOverTimeSem=OSSemCreate(25);		//500ms³¬Ê±
	CommuQuerySem=OSSemCreate(0);
	MoveSem=OSSemCreate(1);
	StopSem=OSSemCreate(0);
	StopEncoderSem=OSSemCreate(0);
	PowerUpdateSem=OSSemCreate(0);
	StopLockSem=OSSemCreate(0);
	MoveLockSem=OSSemCreate(0);
	ProtocolQBOX=OSQCreate(&ProtocolQBuffer[0],TASK_PROTOCOL_QSIZE);
	EHandlerQBOX=OSQCreate(&EHandlerQBuffer[0],TASK_EHANDLER_QSIZE);
	ModBusRevQBOX=OSQCreate(&ModBusRevQBuffer[0],MODEBUS_REV_QSIZE);
	MotionCtrlQBOX=OSQCreate(&MotionCtrlQBuffer[0],TASK_MOTIONCTRL_QSIZE);
	USART2RevList = OSMemCreate((void *)&USART2RevPart[0][0], 11, sizeof(USART2RevBuf_t), &err);
}
/**
*@ brief 		create system tasks
*@ para			none
*@ retval		none
*/
void OS_Task_Create(void)
{

	OSTaskCreateExt(Task_AdcDetect,
									(void *)0,
									&TaskAdcDetectStk[TASK_ADCDETECT_STKSIZE - 1],
                  TASK_ADCDETECT_PRIO,
									TASK_ADCDETECT_PRIO,
									&TaskAdcDetectStk[0],
                  TASK_ADCDETECT_STKSIZE,
									(void *)0,
									OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
	OSTaskCreateExt(Task_PidCalc,
									(void *)0,
									&TaskPidCalcStk[TASK_PIDCALC_STKSIZE - 1],
                  TASK_PIDCALC_PRIO,
									TASK_PIDCALC_PRIO,
									&TaskPidCalcStk[0],
                  TASK_PIDCALC_STKSIZE,
									(void *)0,
									OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
	OSTaskCreateExt(Task_SensorDet,
									(void *)0,
									&TaskSensorDetStk[TASK_SENSORDET_STKSIZE - 1],
                  TASK_SENSORDET_PRIO,
									TASK_SENSORDET_PRIO,
									&TaskSensorDetStk[0],
                  TASK_SENSORDET_STKSIZE,
									(void *)0,
									OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
		/*							
	 OSTaskCreateExt(Task_EHandler,
									(void *)0,
									&TaskEHandlerStk[TASK_EHANDLER_STKSIZE - 1],
                  TASK_EHANDLER_PRIO,
									TASK_EHANDLER_PRIO,
									&TaskEHandlerStk[0],
                  TASK_EHANDLER_STKSIZE,
									(void *)0,
									OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
		*/							
	OSTaskCreateExt(Task_MotionCtrl,
									(void *)0,
									&TaskMotionCtrlStk[TASK_MOTIONCTRL_STKSIZE - 1],
                  TASK_MOTIONCTRL_PRIO,
									TASK_MOTIONCTRL_PRIO,
									&TaskMotionCtrlStk[0],
                  TASK_MOTIONCTRL_STKSIZE,
									(void *)0,
									OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
									
	/*							
	if(BoardID==0)
	{		
		OSTaskCreateExt(Task_DriverState,
										(void *)0,
										&TaskDriverStateStk[TASK_DRIVERSTATE_STKSIZE - 1],
										TASK_DRIVERSTATE_PRIO,
										TASK_DRIVERSTATE_PRIO,
										&TaskDriverStateStk[0],
										TASK_DRIVERSTATE_STKSIZE,
										(void *)0,
										OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
	}
	*/
		OSTaskCreateExt(Task_LinearActuator,
									(void *)0,
									&TaskLinearActuatorStk[TASK_LINEARACTUATOR_STKSIZE - 1],
									TASK_LINEARACTUATOR_PRIO,
									TASK_LINEARACTUATOR_PRIO,
									&TaskLinearActuatorStk[0],
									TASK_LINEARACTUATOR_STKSIZE,
									(void *)0,
									OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

}


/*--------------------The End of File----------------------------------------*/
