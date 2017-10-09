#include "task_cfg.h"
#include "linear_actuator.h"
#include "loadmotor.h"
#include "motor.h"
#include "sys_config.h"
#include "usart.h"


uint8_t loadMotorState = 0;

extern uint8_t Uart5RxBuff[];;

LINEAR_ACTUATOR_UNION LinearData;
void Task_LinearActuator(void* p_arg)
{
//	uint8_t err;
	uint16_t getLoadDatas[] = {0x0001,0x001D,0X001E,0x0036,0x2720};
	uint16_t loadState = 0;
	uint8_t UP_PWMCut = 0;
//	uint16_t drvState = 0;
	uint16_t enable = 0x0001;
	uint16_t disable = 0x0000;
	uint16_t loadFreq = 0;
    uint8_t switchs = 0;
    uint8_t limits= 0;
	uint8_t loadCount = 0 ;
	uint8_t unloadCount = 0;
	uint8_t moveDir = 0;
    uint8_t upCount = 0,lowCount = 0;
  p_arg=p_arg;
	OSTimeDly(10);
//	PowerOFF();
    
   	PowerON();
	while(1)
	{
			
		OSTimeDly(5);
		IWDG_ReloadCounter();		
        switchs = GetSwitchsState();
		limits = GetLimitsState();
		cmdfeed.loadSwitchs = switchs | (limits<<5);
			
#if 1 //20170715,Zero		
   //disable
		if((switchs&0x01) == 0)
		{
			loadMotorState = 2;
		}
		
		//loading
		if((switchs&0x08) == 0X00)
		{
			loadMotorState = 1;
		}
		
//		//unloading
//		if((switchs&0x04) == 0X00)
//		{
//			loadMotorState = 0;
//		}
//		
//		//power on
//		if((switchs&0x02) == 0X00)
//		{
//			loadMotorState = 3;
//		}
//		
//		//power off
//		if((switchs&0x10) == 0)
//		{
//			loadMotorState = 4;
//		}
#endif

#if 0
		   //disable
		if((switchs&0x01) == 0)
		{
			loadMotorState = 1;
		}
		
		//loading
		if((switchs&0x08) == 0X08)
		{
			loadMotorState = 2;
		}
		
		//unloading
		if((switchs&0x04) == 0X04)
		{
			loadMotorState = 0;
		}
		
		//power on
		if((switchs&0x02) == 0X00)
		{
			loadMotorState = 3;
		}
		
		//power off
		if((switchs&0x10) == 0)
		{
			loadMotorState = 4;
		}
		
#endif		
		
		UART5_SendCMD(UART5,0x50,0x0000,0X0005,getLoadDatas);	//×´Ì¬¶ÁÈ¡
		
		cmdfeed.load_enable = Uart5RxBuff[13];//enable state
		
		loadState = (Uart5RxBuff[10]<<8) + Uart5RxBuff[11];
		cmdfeed.loadDriver = ErrCode(loadState);
				
		loadFreq = (Uart5RxBuff[4]<<8) + Uart5RxBuff[5];
		if(((loadMotorState == 1 || loadMotorState == 2)&&((loadFreq)==0)))
		{
			UP_PWMCut++;
			if(UP_PWMCut>=100)
			{
				UP_PWMCut=100;
//				OSQPost(EHandlerQBOX,(void*)PWMCounts);
//				cmdfeed.UP_DRIVER.MOTOR_DRIVER_STRUCT.pwmoff = 1;//PWM¿ØÖÆ¶ÏÏß
				cmdfeed.loadDriver = WRR_PWMOFF;
			}
		}
		else
			{
				UP_PWMCut=0;
			}
			
#if 1	
		switch(loadMotorState)
		{
			case 0:
                LinearData.Linear_State.state = 0;
                LoadSpeed(0);
                OSTimeDly(50);
                UART5_SendCMD(UART5,0X06,Fn010_enable,0X00,&disable);
                loadCount = 0;
                unloadCount = 0;
                break;
			
			case 1:
				if(GetUplimittate() == 0)
				{
					LinearData.Linear_State.uplimit = 0;
					LinearData.Linear_State.state = 1;
					OSTimeDly(1);
					UART5_SendCMD(UART5,0X06,Fn010_enable,0X00,&enable);
					LoadSpeed(-20);
					loadCount ++;
					unloadCount = 0;

				}
				else if(GetUplimittate())
				{
					LinearData.Linear_State.uplimit = 1;
					LinearData.Linear_State.lowlimit = 0;
					LinearData.Linear_State.state = 0;
					LoadSpeed(0);
					OSTimeDly(50);
					UART5_SendCMD(UART5,0X06,Fn010_enable,0X00,&disable);
					loadCount = 0;
					unloadCount = 0;
					loadMotorState = 0;
				}
				break;
			
			case 2:
				if(GetLowlimittate() == 0)	
				{
					LinearData.Linear_State.lowlimit = 0;
					{
						LinearData.Linear_State.state = 2;
						OSTimeDly(1);
						UART5_SendCMD(UART5,0X06,Fn010_enable,0X00,&enable);
						LoadSpeed(20);
						loadCount = 0;
						unloadCount ++;
					}
				}
				else if(GetLowlimittate())		
				{
					LinearData.Linear_State.uplimit = 0;
					LinearData.Linear_State.lowlimit = 1;
					LinearData.Linear_State.state = 0;
					LoadSpeed(0);
					OSTimeDly(50);
					UART5_SendCMD(UART5,0X06,Fn010_enable,0X00,&disable);
					loadCount = 0;
					unloadCount = 0;
					loadMotorState = 0;
				}
				break;
			
			case 3:
                LinearData.Linear_State.state = 3;
                PowerON();
                loadCount = 0;
                unloadCount = 0;
                OSTimeDly(1);
				break;
			
			case 4:
                LinearData.Linear_State.state = 4;
                OSTimeDly(1);
                loadCount = 0;
                unloadCount = 0;
                PowerOFF();
				break;
			
			case 5:
                LinearData.Linear_State.state = 1;
                OSTimeDly(1);
                UART5_SendCMD(UART5,0X06,Fn010_enable,0X00,&enable);
                LoadSpeed(-20);
                loadCount ++;
                unloadCount = 0;
			break;
			
			case 6:
                LinearData.Linear_State.state = 2;
                OSTimeDly(1);
                UART5_SendCMD(UART5,0X06,Fn010_enable,0X00,&enable);
                LoadSpeed(20);
                loadCount = 0;
                unloadCount ++;
			break;
            
            case 7:
                
                if(GetUplimittate())
                {
                    //moveDir = 2;
                    LinearData.Linear_State.uplimit = 1;
                    LinearData.Linear_State.lowlimit = 0;
                  
                    upCount++;
                    lowCount = 0;
                    if(upCount<100)
                    {
                        LinearData.Linear_State.state = 0; 
                    }
                    else
                    {
                        LinearData.Linear_State.state = 2; 
                    }

                }
                
                if(GetLowlimittate())
                {
                    //moveDir = 1;
                    LinearData.Linear_State.uplimit = 0;
                    LinearData.Linear_State.lowlimit = 1;
                    upCount = 0;
                    lowCount ++;
                    if(lowCount<100)
                    {
                        LinearData.Linear_State.state = 0;
                    }
                    else
                    {
                        LinearData.Linear_State.state = 1;
                    }
                }
                
                if(LinearData.Linear_State.state == 0)
                {
                    LoadSpeed(0);
                }
                else if(LinearData.Linear_State.state == 1)
                {
                    LoadSpeed(-20);
                }
                else 
                {
                    LoadSpeed(20);
                }
                
                break;
                    
            case 8 :
                OSTimeDly(1);
				UART5_SendCMD(UART5,0X06,Fn010_enable,0X00,&enable);
                break;
                
			
			default :
                loadCount = 0;
                unloadCount = 0;
				break;	
		}
#endif	
		//if timeout,disable it 
		if( loadCount >= 100 || unloadCount >= 100)
		{
            LoadSpeed(0);
            OSTimeDly(1);			
            UART5_SendCMD(UART5,0X06,Fn010_enable,0X00,&disable);
            loadCount = 0;
            unloadCount = 0;
            loadMotorState = 0;
		}
	}
}
