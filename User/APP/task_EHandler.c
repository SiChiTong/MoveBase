/**-------------------------------------------------------------------------
*@ file				task_Ehandler.c
*@ brief			task for exception handler
*@ author 		ZGJ
*@ vertion		V0.01
*@ date				2015-12-15
*@ statement	(C)	COPYRIGHT 2015 KSITRI.
--------------------------------------------------------------------------*/
/* Include files -------------------------------------------------*/
#include "ucos_ii.h"
#include "task_cfg.h"
#include "usart.h"
#include "motor.h"
#include "common.h"
/* define ---------------------------------------------------------*/

/* variables -----------------------------------------------------*/

/* functions ------------------------------------------------------*/
void Task_EHandler(void *p_arg)
{
	unsigned char err=0;
	
	_EXCEPTION_KIND		ekind=EXCEPTION_NONE;
	for(;;)
	{
		IWDG_ReloadCounter();
		ekind=(_EXCEPTION_KIND)OSQPend(EHandlerQBOX,0,&err);
	
		cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.normal = 0;
		
		if(ekind != EXCEPTION_NONE)
		{
			gMotionCtrl.move(0,0,N_STOP);
		}
			
#if 0
		switch(ekind)
		{		
			case MOTOR_NO_SPEED:			//堵转停止
			{
				
				if(ReadSysStaBit(DriverInitFlag))
				{
					gMotionCtrl.move(0,0,N_STOP);
				}
			}
				break;
			case DRIVER_OVER_TEMP:		//驱动器过温
			{
				if(ReadSysStaBit(DriverInitFlag))
				{
					gMotionCtrl.move(0,0,N_STOP);
				}
			}
				break;

			case LOW_POWER:		//欠压
			{
				if(ReadSysStaBit(DriverInitFlag))
				{
					gMotionCtrl.move(0,0,N_STOP);
				}
			}
				break;

			case MOTOR_OVER_THMP:				//电机过温
			{
				if(ReadSysStaBit(DriverInitFlag))
				{
					gMotionCtrl.move(0,0,N_STOP);
				}				
			}
				break;
			
			case MOTOR_WINDING_OPEN :
			{
				if(ReadSysStaBit(DriverInitFlag))
				{
					gMotionCtrl.move(0,0,E_STOP);
				}
			}
				break;

			case DRIVER_HALL_ERR:
			{
				if(ReadSysStaBit(DriverInitFlag))
				{
					gMotionCtrl.move(0,0,N_STOP);
				}
			}
				break;
			case DRIVER_COMMU_ERR:
			{
				if(ReadSysStaBit(DriverInitFlag))
				{
					gMotionCtrl.move(0,0,N_STOP);
				}
			}
				break;
			case ENCODER_ERR:
			{
				if(ReadSysStaBit(DriverInitFlag))
				{
					gMotionCtrl.move(0,0,N_STOP);
				}
			}
				break;
		
			case PZKEY_ON:		//碰撞开关
			{
				if(sys_config.sensor.collision)	//20160520,Zero
				{
					if(ReadSysStaBit(DriverInitFlag))
					{
						gMotionCtrl.move(0,0,E_STOP);
					}
				}
			}
				break;
			
			case CMD_OVERTIME:	//超时停止
			{

				if(ReadSysStaBit(DriverInitFlag))
				{
					gMotionCtrl.move(0,0,E_STOP);
				}
			}
				break;
			case STOPSWITCH_ON:		//急停开关
			{
				if(sys_config.sensor.e_stop)//20160520,Zero
				{
					if(ReadSysStaBit(DriverInitFlag))
					{
						gMotionCtrl.move(0,0,F_STOP);
					}
				}
			}
				break;
			case IOCTRLSTOP_ON:		//IO急停开关
			{
				if(sys_config.sensor.laser)//20160520,Zero
				{
					if(ReadSysStaBit(DriverInitFlag))
					{
						gMotionCtrl.move(0,0,E_STOP);
					}
				}
			}
				break;
			case LIGHT_SENSOR_ON:
			{
				if(sys_config.sensor.fallen)//20160520,Zero
				{
					if(ReadSysStaBit(DriverInitFlag))
					{
						gMotionCtrl.move(0,0,E_STOP);		//碰撞开关触发，停止运动
					}
				}
			}
				break;	
	/*		
			case OVER_SPEED:
			{

				if(ReadSysStaBit(DriverInitFlag))
				{
					gMotionCtrl.move(0,0,E_STOP);		//碰撞开关触发，停止运动
				}
			}
				break;
	*/		
			case OVER_CURRENT:
			{

				if(ReadSysStaBit(DriverInitFlag))
				{
//					gMotionCtrl.move(0,0,E_STOP);		//碰撞开关触发，停止运动
//					CloseDriverPower();
					gMotionCtrl.move(0,0,0);	
				}
			}
				break;
			
			case OVER_POSITION:
			{

				if(ReadSysStaBit(DriverInitFlag))
				{
//					gMotionCtrl.move(0,0,E_STOP);		//碰撞开关触发，停止运动
//					CloseDriverPower();
					gMotionCtrl.move(0,0,0);	
				}
			}
				break;
			
		case MOTOR_OVER_LOAD:
			{

//				if(ReadSysStaBit(DriverInitFlag))
				{
//					gMotionCtrl.move(0,0,E_STOP);		//碰撞开关触发，停止运动
//					CloseDriverPower();
					gMotionCtrl.move(0,0,0);	
				}
			}
				break;
			
		case PWM_CUTOFF:
			gMotionCtrl.move(0,0,0);	
		break;
		
		case DRIVER_ERR :
				gMotionCtrl.move(0,0,0);	
		break;
			
			default:
				gMotionCtrl.move(0,0,0);	
				break;
		}
#endif
	}
}


/*-------------------The End Of File--------------------------------------------*/
