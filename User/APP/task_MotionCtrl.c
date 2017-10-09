/**-------------------------------------------------------------------------
*@ file				task_MotionCtrl.c
*@ brief			电机运动控制任务，以逻辑控制停止自锁比较稳定，但反应灵敏度需添加一个控制任务来提高
*@ author 		ZGJ
*@ vertion		V0.01
*@ date				2016-01-27
*@ statement	(C)	COPYRIGHT 2016 KSITRI.
--------------------------------------------------------------------------*/
/* Include Files -----------------------------------------------------*/
#include "motor.h"
#include "common.h"
#include "task_cfg.h"
#include "ucos_ii.h"
#include "modbus.h"
#include <stdio.h>
#include "usart.h"

extern _CMD_FEEDBACK	cmdfeed;


void Task_MotionCtrl(void* p_arg)
{
	_MOTION_PARAM *pMPara=NULL;
	unsigned char err=0;
	OSTimeDly(50);
	
	//爱控驱动器初始化
//	if(BoardID==0)
//	{
//		DriverConfig();
//		LockMotor();
//		
//	}
	SetSysStaBit(DriverInitFlag);
	
	for(;;)	
	{
		IWDG_ReloadCounter();
		pMPara = (_MOTION_PARAM*)OSQPend(MotionCtrlQBOX,50,&err);
					
		if(err==OS_ERR_NONE)
		{
			if((pMPara->vx==0)&&(pMPara->vthta==0))
			{
				{
					gMotionCtrl.move(pMPara->vx,pMPara->vthta,pMPara->stop_mode);
				}
			}
			else
			{
				gMotionCtrl.move(pMPara->vx,pMPara->vthta,pMPara->stop_mode);
			}
		}
		else
		{
			gMotionCtrl.move(0,0,0);
//			OSTimeDly(500);
//			gMotionCtrl.move(100,0,0);
//			OSTimeDly(500);
//			gMotionCtrl.move(-800,0,0);
		}
		
	}
}

/*------------------------The End of File --------------------------------------------*/

