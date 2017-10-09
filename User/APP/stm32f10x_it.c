/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "ucos_ii.h"
#include "task_cfg.h"
#include "usart.h"
#include "common.h"
#include "motor.h"
#include "linear_actuator.h"

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/
//static unsigned char tim_index=0;

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
	if (CoreDebug->DHCSR & 1) {  //check C_DEBUGEN == 1 -> Debugger Connected  
      __breakpoint(0);  // halt program execution here         
  }  
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
/*
void PendSV_Handler(void)
{
}
*/
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	OS_CPU_SR  cpu_sr;
  OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
  OSIntNesting++;
  OS_EXIT_CRITICAL();
  OSTimeTick();                                /* Call uC/OS-II's OSTimeTick()                       */
  OSIntExit();
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/
/**
  * @brief  This function handles ADC interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Channel4_5_IRQHandler(void)
{
		OSIntEnter();
    if (DMA_GetITStatus(DMA2_IT_TC5) != RESET)
    {
        DMA_ClearFlag(DMA2_FLAG_TC5);
        DMA_ClearITPendingBit(DMA2_IT_TC5);  //清除标志
        OSSemPost(Adc3CycSem);
    }
    OSIntExit();
}

/**
  * @brief  This function handles ADC interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Channel1_IRQHandler(void)
{
		OSIntEnter();
    if (DMA_GetITStatus(DMA1_IT_TC1) != RESET)
    {
        DMA_ClearFlag(DMA1_FLAG_TC1);
        DMA_ClearITPendingBit(DMA1_IT_TC1);  //清除标志
        OSSemPost(Adc1CycSem);
    }
    OSIntExit();
}
/**
  * @brief  This function handles TIM5 interrupt request, for motor cycle ctrl
  * @param  None
  * @retval None
  */
void TIM5_IRQHandler(void)
{
    OSIntEnter();  //通知系统进中断
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
    {
        TIM_ClearFlag(TIM5, TIM_FLAG_Update);   //清标志
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);  //清中断
			
//			if(BoardID==0)
//			{
//				OSSemPost(PidCalcSem);
//			}
			
			if(BoardID==0)
			{
				DMA1_Channel2->CCR |= DMA_CCR1_EN;//enable	
				DMA1_Channel4->CCR |= DMA_CCR1_EN;//enable	
				
				UsartCMDMbox.add=GETSTATE;
				UsartCMDMbox.L_h_value=0;
				UsartCMDMbox.L_l_value=0;
				UsartCMDMbox.R_h_value=0;
				UsartCMDMbox.R_l_value=0;
				OSMboxPost(UsartsCMD,&UsartCMDMbox);
			}
    }
    OSIntExit();  //通知系统出中断
}

/**
  * @brief  This function handles TIM6 interrupt request, for PZ key detect,20ms
  * @param  None
  * @retval None
  */
void TIM6_IRQHandler(void)
{
    static uint8_t cnt=0;
    volatile static uint8_t power_cnt=0;
    uint16_t Ecnt=0;
    uint8_t err=0;

    OSIntEnter();  //通知系统进中断
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
        TIM_ClearFlag(TIM6, TIM_FLAG_Update);   //清标志
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);  //清中断
        IWDG_ReloadCounter();
    //触碰位
//				GetKey();
        
            //运动超时检测
        if(ReadSysStaBit(SafeModeSta))		//安全模式开启
        {
            if(OSSemAccept(CmdOverTimeSem)==0)
            {
                //通知超时停止
                OSQPost(EHandlerQBOX,(void*)CMD_OVERTIME);
                OSSemSet(CmdOverTimeSem,25,&err);
            }
        }
        //通讯质量统计
        cnt++;
        if(cnt>=50)		//1s
        {
            Ecnt=OSSemAccept(CommuQuerySem);
            cmdfeed.comu=Ecnt*5;
            OSSemSet(CommuQuerySem,0,&err);
            cnt=0;
        }
        
#if POWER_ALARM				
        power_cnt++;
        if(power_cnt>=50)
        {
            OSSemPost(PowerUpdateSem);
            power_cnt=0;
        }			
#endif
    }
    OSIntExit();  //通知系统出中断
}

#if 0
void EXTI15_10_IRQHandler(void)
{
	OSIntEnter();
//	unsigned short key=0;
	//急停开关,
//	    IWDG_ReloadCounter();
	//激光
		if(EXTI_GetITStatus(EXTI_Line11) != RESET)
		{
			EXTI_ClearITPendingBit(EXTI_Line11);
			if(sys_config.sensor.laser)
			{
				if(((GPIOD->IDR&(1<<11))==0))
				{
					//20170317,Zero,通过配置实现对避障触发后的动作
					if(sys_config.sensor.laser_mode)
					{
						if(GetEStopState()==1)
						{
							ClearSysStaBit(IOCtrlStop);				
							cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.io_stop = 0;
						}
					}
					else
					{
						ClearSysStaBit(IOCtrlStop);				
						cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.io_stop = 0;		
					}
				}
				else
				{
					SetSysStaBit(IOCtrlStop);							
					cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.io_stop = 1;
					cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.normal = 0;				
					OSQPost(EHandlerQBOX,(void*)IOCTRLSTOP_ON);
				}
			}
			else
			{
				cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.io_stop = 0;
			}

		}

		 OSIntExit();  //í¨?a?μí33??D??
}
#endif
/**
  * @brief  This function handles USART2 IDLE interrupt request.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)
{
	volatile static unsigned char temp=0;
	volatile unsigned char temper=0;
    OSIntEnter();
    if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
        temper = USART2->SR;  		
        temper = USART2->DR;	//清USART_IT_IDLE
        IWDG_ReloadCounter();
        DMA_Cmd(DMA1_Channel6,DISABLE);
        temp=PROTOCOL_RECIVE_SIZE-DMA_GetCurrDataCounter(DMA1_Channel6);
        Usart2AddRevBuf(Usart2RevBuffer,temp);
        revcnt++;

        //发送命令处理信号量
        OSSemPost(Usart2CMDSem);
        DMA1_Channel6->CMAR = (uint32_t)&Usart2RevBuffer[0];
        DMA_SetCurrDataCounter(DMA1_Channel6,PROTOCOL_RECIVE_SIZE);

        DMA_Cmd(DMA1_Channel6,ENABLE);
    }
    OSIntExit();
}

/**
  * @brief  This function handles USART2-TX-DMA interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Channel7_IRQHandler(void)					//USART2-TX
{
    OSIntEnter();
    if (DMA_GetITStatus(DMA1_IT_TC7) != RESET)
    {
        DMA_Cmd(DMA1_Channel7, DISABLE);
        DMA_ClearFlag(DMA1_FLAG_TC7);
        DMA_ClearITPendingBit(DMA1_IT_TC7);  // 清除中断标志位
        while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET); //等待一包数据发送完成
    }
    OSIntExit();
}

void UART4_IRQHandler(void)
{
	static unsigned char temp=0;
	volatile unsigned char temper=0;
	OSIntEnter();
	
	if(USART_GetITStatus(UART4,USART_IT_IDLE)!=RESET)
	{
		temper = UART4->SR;  		
        temper = UART4->DR;	//清USART_IT_IDLE
		DMA_Cmd(DMA2_Channel3,DISABLE);
		temp=32-DMA_GetCurrDataCounter(DMA2_Channel3);
//		if(temp>3)
//		{
//			OSQPost(ModBusRevQBOX,(void*)&temp);
//		}
		DMA_SetCurrDataCounter(DMA2_Channel3,32);
		DMA_Cmd(DMA2_Channel3,ENABLE);
	}
	OSIntExit();
}

uint8_t Uart5RxBuff[64] = { 0 };
uint8_t Uart5RxCount = 0;
void UART5_IRQHandler(void)
{
	static uint8_t temp=0;
	volatile uint8_t temper=0;
	OSIntEnter();
	
	if(USART_GetITStatus(UART5,USART_IT_RXNE)!=RESET)
	{
		Uart5RxBuff[Uart5RxCount++] = UART5->DR;
	
	}
	
	if(USART_GetITStatus(UART5,USART_IT_IDLE)!=RESET)
	{
		temper = UART5->SR;  		
        temper = UART5->DR;	//清USART_IT_IDLE

		Uart5RxCount = 0;
		OSQPost(ModBusRevQBOX,(void*)&temp);
		
	}
	OSIntExit();
}

//void TIM7_IRQHandler(void)
//{
//	static unsigned temp=0;
//	OSIntEnter();
//	if(TIM_GetITStatus(TIM7,TIM_IT_Update)!=RESET)
//	{
//		TIM_ClearFlag(TIM7,TIM_FLAG_Update);
//		TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
//		tim_index++;
//		if(tim_index>2)
//		{
//			TIM_Cmd(TIM7,DISABLE);
//		//	temp=uart_index;
//			DMA_Cmd(DMA2_Channel3,DISABLE);
//			temp=64-DMA_GetCurrDataCounter(DMA2_Channel3);
//			if(temp>3)
//			{
//				OSQPost(ModBusRevQBOX,(void*)&temp);
//			}
//			DMA_SetCurrDataCounter(DMA2_Channel3,64);
//			DMA_Cmd(DMA2_Channel3,ENABLE);
//	//		OSQPost(ModBusRevQBOX,(void*)&temp);
//	//		uart_index=0;
//			tim_index=0;
//		}
//	}
//	OSIntExit();
//}

//void DMA1_Channel3_IRQHandler(void)					//USART3-RX
//{
//	  volatile int count;
//		OSIntEnter();
//    if (DMA_GetITStatus(DMA1_IT_TC3) != RESET)
//    {
//			  count=32-DMA_GetCurrDataCounter(DMA1_Channel3);
//        DMA_Cmd(DMA1_Channel3, DISABLE);
//        DMA_ClearFlag(DMA1_FLAG_TC3);
//        DMA_ClearITPendingBit(DMA1_IT_TC3);  // 清除中断标志位

//				DMA1_Channel3->CCR &= (uint16_t)(~DMA_CCR1_EN);//disable
//				DMA1_Channel3->CNDTR = 32;
//				DMA1_Channel3->CCR |= DMA_CCR1_EN;//enable
//				
//				Usart3State.sendflag=0;
////			  OSSemPost(Usart3Cal);
//    }
//		
//		if(DMA_GetITStatus(DMA1_IT_TE3) != RESET)
//		{
//			 DMA_ClearFlag(DMA1_FLAG_TE3);
//       DMA_ClearITPendingBit(DMA1_IT_TE3);  // 清除中断标志位
//		}
//		
//		OSIntExit();
//}

void USART3_IRQHandler(void)
{
		volatile static unsigned char temp=0;
		volatile unsigned char temper=0;
	  volatile int count;
    OSIntEnter();
    if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
        temper = USART3->SR;  		
        temper = USART3->DR;	//清USART_IT_IDLE
        IWDG_ReloadCounter();

        count=32-DMA_GetCurrDataCounter(DMA1_Channel3);
        DMA_Cmd(DMA1_Channel3, DISABLE);
        //			DMA_ClearFlag(DMA1_FLAG_TC3);
        //			DMA_ClearITPendingBit(DMA1_IT_TC3);  // 清除中断标志位

        DMA1_Channel3->CCR &= (uint16_t)(~DMA_CCR1_EN);//disable
        DMA1_Channel3->CNDTR = 32;
        DMA1_Channel3->CCR |= DMA_CCR1_EN;//enable

        Usart3State.sendflag=0;
    }
    OSIntExit();
}


//void DMA1_Channel5_IRQHandler(void)					//USART1-RX
//{
//	  volatile int count;
//		OSIntEnter();
//    if (DMA_GetITStatus(DMA1_IT_TC5) != RESET)
//    {
//			count=32-DMA_GetCurrDataCounter(DMA1_Channel5);
//      DMA_Cmd(DMA1_Channel5, DISABLE);
//      DMA_ClearFlag(DMA1_FLAG_TC5);
//      DMA_ClearITPendingBit(DMA1_IT_TC5);  // 清除中断标志位			
//				
//			Usart1State.sendflag=0;

//			DMA1_Channel5->CCR &= (uint16_t)(~DMA_CCR1_EN);//disable
//			DMA1_Channel5->CNDTR = 32;
//			DMA1_Channel5->CCR |= DMA_CCR1_EN;//enable	

////			OSSemPost(Usart1Cal);			
//    }
//		
//		if(DMA_GetITStatus(DMA1_IT_TE5) != RESET)
//		{
//			 DMA_ClearFlag(DMA1_FLAG_TE5);
//       DMA_ClearITPendingBit(DMA1_IT_TE5);  // 清除中断标志位
//		}
//		OSIntExit();
//}

void USART1_IRQHandler(void)
{
    volatile static unsigned char temp=0;
    volatile unsigned char temper=0;
    volatile int count;
    OSIntEnter();
    if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        temper = USART1->SR;  		
        temper = USART1->DR;	//清USART_IT_IDLE
        IWDG_ReloadCounter();

        count=32-DMA_GetCurrDataCounter(DMA1_Channel5);
        DMA_Cmd(DMA1_Channel5, DISABLE);
        //      DMA_ClearFlag(DMA1_FLAG_TC5);
        //      DMA_ClearITPendingBit(DMA1_IT_TC5);  // 清除中断标志位			

        Usart1State.sendflag=0;

        DMA1_Channel5->CCR &= (uint16_t)(~DMA_CCR1_EN);//disable
        DMA1_Channel5->CNDTR = 32;
        DMA1_Channel5->CCR |= DMA_CCR1_EN;//enable	
    }
    OSIntExit();
}



void EXTI0_IRQHandler(void)
{
	OSIntEnter();
  if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {

    EXTI_ClearITPendingBit(EXTI_Line0);	
  }
	OSIntExit();
}


//extern	uint8_t LoadFlag ;
//extern	uint8_t UnloadFlag ;
extern uint8_t loadMotorState;
void EXTI2_IRQHandler(void)
{
	OSIntEnter();
  if(EXTI_GetITStatus(LOW_LIMIT_EXTI_LINE) != RESET)
  {
		if(loadMotorState == 2)
		{
			if(GPIO_ReadInputDataBit(LOW_LIMIT_PORT,LOW_LIMIT_PIN))
//			if(GetLowlimittate())
			{
				CloseDriverPower();
			}
		}
    EXTI_ClearITPendingBit(LOW_LIMIT_EXTI_LINE);	
  }
	OSIntExit();
}


void EXTI3_IRQHandler(void)
{
	OSIntEnter();
  if(EXTI_GetITStatus(UP_LIMIT_EXTI_LINE) != RESET)
  {
		if(loadMotorState == 1)
		{
		if(GPIO_ReadInputDataBit(UP_LIMIT_PORT,UP_LIMIT_PIN))
//			if(GetUplimittate())
			{
				CloseDriverPower();
			}
		}
    EXTI_ClearITPendingBit(UP_LIMIT_EXTI_LINE);	
  }
	OSIntExit();
}


void EXTI9_5_IRQHandler(void)
{
	OSIntEnter();
  if(EXTI_GetITStatus(LOW_LIMIT_EXTI_LINE) != RESET)
  {

    EXTI_ClearITPendingBit(LOW_LIMIT_EXTI_LINE);	
  }
	
	 if(EXTI_GetITStatus(UP_LIMIT_EXTI_LINE) != RESET)
  {
    EXTI_ClearITPendingBit(UP_LIMIT_EXTI_LINE);		
  }
//	CloseDriverPower();
	OSIntExit();
	
}
/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/


