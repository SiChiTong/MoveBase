#include "motor.h"
#include "common.h"
#include "task_cfg.h"
#include "ucos_ii.h"
#include "modbus.h"
#include <stdio.h>
#include "usart.h"
#include "loadmotor.h"

static void DriverInit(void);

void Task_DriverState(void* p_arg)
{
//	unsigned char err;

//	USARTCMD_STRUCT *UsartCmdData;
	static uint8_t DrvInitFlag = 0;
	static uint8_t DrvInitCount = 0;
	uint16_t enable = 0x01;
	uint16_t disable = 0x00;
	uint8_t DriverState=0;
  p_arg=p_arg;
	OSTimeDly(10);
	while(1)
	{
//		uint16_t getDatas[] = {0x0001,0x001D,0X001E,0x0036,0x2720};
		IWDG_ReloadCounter();
		OSTimeDly(5);
//		HLSGetStates(0x50,0x0000,0X0005,getDatas);
//		UsartCmdData=OSMboxPend(UsartsCMD,0,&err);

//		if(err==OS_ERR_NONE)
		{	
//			uint16_t getDatas[] = {0x0001,0x001D,0X001E,0x0036,0x2720};
//			HLSSendCMD(USART1,0x50,0x0000,0X0005,getDatas);	
//			HLSSendCMD(USART3,0x50,0x0000,0X0005,getDatas);	
			//GET DRIVER STATE
//			HLSSendCMD(USART1,0x03,0x0036,0X0001,0x00);
//			HLSSendCMD(USART3,0x03,0x0036,0X0001,0x00);
//		
//			cmdfeed.leftDriver = (Usart1Data.UsartRevData[4]<<8) + Usart1Data.UsartRevData[5];
//			cmdfeed.rightDriver = (Usart3Data.UsartRevData[4]<<8) + Usart3Data.UsartRevData[5];
//			
//			if((cmdfeed.leftDriver !=0)||(cmdfeed.rightDriver!=0 ))
//			{	
//				OSQPost(EHandlerQBOX,(void*)DRIVER_ERR);
//			}
		}
	}
}
	