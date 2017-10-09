/**-------------------------------------------------------------------------
*@ file				task_SensorDet.c
*@ brief			跌落与碰撞传感器检测任务
*@ author 		ZGJ
*@ vertion		V0.01
*@ date				2015-12-15
*@ statement	(C)	COPYRIGHT 2015 KSITRI.
--------------------------------------------------------------------------*/
/* Include Files -----------------------------------------------------------*/
#include "task_cfg.h"
#include "vl6180x.h"
#include "common.h"
#include "usart.h"
#include "motor.h"
//#include "fallen.h"
#include "loadmotor.h"
/* define ------------------------------------------------------------------*/
#define MAX_RANGE    100
#define	DATA_ERR	   0xFF
#define DEVICE_ERR  0xFE
#define DEVICE_NUM   3
/* variables ----------------------------------------------------------------*/

//static unsigned char range_value = 0;
//static float MAXOffset[5]={2,2,1,1.6,1.6};
//static float MINOffset[5]={0,0,0,0,0};
//char AddressTable[5]={0x54,0x50,0x4E,0x4C,0x4A};
//extern unsigned char LaserOffset ;

//static unsigned char GetLaserData(char address)
//{
//		int status;
//		int new_switch_state;
//		int switch_state = -1;
//		unsigned char pdata = 0;
//		unsigned char pdata1 = 0;
//		unsigned char pdata_3 = 0;
//		
//		unsigned short range = 0; 
//		int alpha =(int)(0.1*(1<<16));    /* range distance running average cofs */
//	
//			status = single_read(address,0x000,&pdata1);
//			if(pdata1 == 0xB4)
//			{
//				single_write(address, 0x015, 0x01);							//清中断
//				single_read(address,RESULT__RANGE_STATUS,&pdata); //等待开始
//				if(pdata==0)
//				{
////					single_read_I2C0(DeviceAddr,RESULT__RANGE_STATUS,&pdata);
//					range = DEV_ERR;
//					return range;
//				}
//				single_write(address, 0x018, 0x01);		//开始测量
//				OSTimeDly(6);
//				status = single_read(address,0x000,&pdata1);
//				if(pdata1 == 0xB4)
//				{
//					single_read(address,RESULT__INTERRUPT_STATUS_GPIO,&pdata);
//					if(pdata == 4)
//					{
//						single_read(address,RESULT__RANGE_VAL,&pdata1);
//						if(pdata1 != 255)
//						{
//							pdata_3 = pdata1;
//							range = pdata1;
//							range_value = range + LaserOffset;		
//						}
//						else
//						{
//							range_value = DATA_ERR;
//						}
//					}
//					else
//					{
//						range_value=DATA_ERR;
//					}
//				}
//				else
//				{
//					range_value = DATA_ERR;
//				}
//			}
//			else
//			{
//				range_value=DATA_ERR;
//			}
//			return range_value;
//}

/* functions --------------------------------------------------------------*/

uint8_t ChargingFlag = 0;
uint8_t ChargedFlag = 1 ;

void Task_SensorDet(void *p_arg)
{
//	_PZ_KEY  pzkey=PZKEY_NONE;
//	int status;
//	int new_switch_state;
//	int switch_state = -1;
	
//	unsigned short range = 0; 
//	int alpha =(int)(0.1*(1<<16));    /* range distance running average cofs */
//	static unsigned char stamachin=0;
//	static unsigned int DriverEnableCount=0;
	
	static uint8_t l_driverEnable = 0;
	static uint8_t r_driverEnable = 0;
	
	static uint8_t l_enableCount = 0;
	static uint8_t r_enableCount = 0;
    
    uint16_t enable = 0x01;
	uint16_t disable = 0x00;
	
	gfallSensorThread.FallSensorUpThread = 0x3C;
	gfallSensorThread.FallSensorDownThread =  0x1B;
	
	
//	Vl6180X_Init();
   
	for(;;)
	{

		OSTimeDly(1);	
		IWDG_ReloadCounter();
		
//		if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_8) == 0)
//		{
//			cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.io_stop = 1;
//		}
//		else
//		{
//			cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.io_stop = 0;
//		}
		
		//检查传感器状态字
		if(sys_config.sensor.e_stop)
		{
			//检查急停开关状态
			if(GetEStopState())
			{
				//DISCHARGE
				GPIOG->BRR=GPIO_Pin_5;	

				//20170832,Zero if set power off ,else dosable
				if( DRIVER_POWER_OFF )
				{
					CloseDriverPower();
				}
				else
				{
						if(cmdfeed.l_enable == 1)
						{
								l_enableCount++;
								if( l_enableCount%5 == 0)
								{
									  OSTimeDly(1);	
										HLSSendCMD(USART1,0X06,Fn010_enable,0X00,&disable);
								}
						}
						else
						{
							l_enableCount = 0;
						}
						
						if(cmdfeed.r_enable == 1)
						{
								r_enableCount++;
								if( r_enableCount%5 == 0)
								{
									  OSTimeDly(1);	
										HLSSendCMD(USART3,0X06,Fn010_enable,0X00,&disable);					
								}
						}
						else
						{
							r_enableCount = 0;
						}
//						HLSSendCMD(USART1,0X06,Fn010_enable,0X00,&disable);
//						HLSSendCMD(USART3,0X06,Fn010_enable,0X00,&disable);	
				}
				SetSysStaBit(EmergencyStop);
//			DriverEnableCount = 0;
				cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.switch_stop = 1;
				cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.normal = 0;
//				cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.over_speed = 0 ;
				
				//急停后恢复PWM断线故障，Zero,20170206
//				cmdfeed.LEFT_DRIVER.MOTOR_DRIVER_STRUCT.pwmoff = 0;
//				cmdfeed.RIGHT_DRIVER.MOTOR_DRIVER_STRUCT.pwmoff = 0;
                
				//按下急停，清除PWM断线错误
				cmdfeed.leftDriver &= (~WRR_PWMOFF);
				cmdfeed.rightDriver &= (~WRR_PWMOFF);
				
//				if(sys_config.sensor.laser_mode)
//				{					
//					cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.io_stop = (GPIOD->IDR&(1<<11));	
//				}					
				
				OSQPost(EHandlerQBOX,(void*)STOPSWITCH_ON);	
				l_driverEnable = 0;
				r_driverEnable = 0;				
			}
			else
			{

				IWDG_ReloadCounter();	
				GPIOG->BSRR=GPIO_Pin_5;
				//20170518
				if( DRIVER_POWER_OFF )
				{
					OpenDriverPower();		
				}					
				ClearSysStaBit(EmergencyStop);
				cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.switch_stop = 0;	
				
				//get driver state,if not enable,enable it
				//20170815,Zero check charging state , if charged   -- enable it
				if( ChargedFlag == 1 )
				{
//                    ChargedFlag = 0 ;
						if(cmdfeed.l_enable == 0)
						{
								l_enableCount++;
								if( l_enableCount%5 == 0)
								{
										OSTimeDly(1);	
										HLSSendCMD(USART1,0X06,Fn010_enable,0X00,&enable);
								}
						}
						else
						{
							l_enableCount = 0;
						}
						
						if(cmdfeed.r_enable == 0)
						{
								r_enableCount++;
								if( r_enableCount%5 == 0)
								{
										OSTimeDly(1);	
										HLSSendCMD(USART3,0X06,Fn010_enable,0X00,&enable);					
								}
						}
						else
						{
							r_enableCount = 0;
						}
				}
				
				//if charging , disbale it 
				if( ChargingFlag == 1 )
				{
//                    ChargingFlag = 0 ;
						if(cmdfeed.l_enable == 1)
						{
								l_enableCount++;
								if( l_enableCount%5 == 0)
								{ 
									  OSTimeDly(1);	
										HLSSendCMD(USART1,0X06,Fn010_enable,0X00,&disable);
								}
						}
						else
						{
							l_enableCount = 0;
						}
						
						if(cmdfeed.r_enable == 1)
						{
								r_enableCount++;
								if( r_enableCount%5 == 0)
								{ 
										OSTimeDly(1);	
										HLSSendCMD(USART3,0X06,Fn010_enable,0X00,&disable);					
								}
						}
						else
						{
							r_enableCount = 0;
						}					
				}			
			}
		}
		else
		{
			cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.switch_stop = 0;
		}
		
		if(sys_config.sensor.collision)
		{
			//20170317,Zero,clear collision satae by estop
			if(sys_config.sensor.collision_mode)
			{
//				cmdfeed.sensor_sta |=  (GetCollisionState() |(cmdfeed.sensor_sta&0x0F));
				cmdfeed.collisions |= GetCollisionsState_3();
				if(GetEStopState()==1)
				{
//					cmdfeed.sensor_sta = (GetCollisionState() |(cmdfeed.sensor_sta&0x0F));
					cmdfeed.collisions = GetCollisionsState_3();
				}
			}
			else
			{
//			  cmdfeed.sensor_sta = (GetCollisionState() |(cmdfeed.sensor_sta&0x0F));
					cmdfeed.collisions = GetCollisionsState_3();
			}
		}
		else
		{
//			cmdfeed.sensor_sta &= 0x0F ; 
			cmdfeed.collisions = 0x00;
		}
		      
    }
 }
/*-----------------------The End Of File----------------------------------*/
