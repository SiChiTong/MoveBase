#include "stm32f10x.h"
#include "loadmotor.h"
#include "usart.h"
#include <math.h>

void UART5_Config(uint32_t baud)
{
		USART_InitTypeDef USART_InitStructure;
		GPIO_InitTypeDef	GPIO_InitStructure;
				//USART5-RX-PD2, USART5-TX-PC12  
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
	
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);				//---RS485
		//主通信接口USART5
		USART_InitStructure.USART_BaudRate = baud;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(UART5, &USART_InitStructure);
		USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
		USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);
		USART_Cmd(UART5, ENABLE);
}

//LOAD MOTOR PWM -- PC7 DIR -- PC6
void LOAD8_PWM_Init(int period,int prescaler)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
 // GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);	
	
//		 PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
  /* Time base configuration */
		//T = (1+ prescaler)/72M*(1+period)
  TIM_TimeBaseStructure.TIM_Period = period;
  TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

/* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;

  TIM_OC2Init(TIM8, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM8, ENABLE);

  /* TIM3 enable counter */	
	TIM_CtrlPWMOutputs(TIM8,ENABLE);
	
//	TIM_SelectOnePulseMode(TIM4,TIM_OPMode_Single);

  TIM_Cmd(TIM8, ENABLE);

}

void  LoadPowerSwitch_Init(void)
{
    GPIO_InitTypeDef   GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOG, &GPIO_InitStructure);
}

void Switchs_Init(void)
{
    GPIO_InitTypeDef   GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 |GPIO_Pin_11 |GPIO_Pin_12 |GPIO_Pin_13 |GPIO_Pin_14 ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING;//GPIO_Mode_IPD;//
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 ;
    // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
}


uint8_t GetSwitchsState(void)
{
	static uint8_t SwitchsState = 0;
	static uint8_t TrigH_count[5]={0};
	static uint8_t TrigL_count[5]={0};
	uint8_t i;
	
			//碰撞开关检测	
	for(i=0;i<5;i++)
	{
		if(GPIO_ReadInputDataBit(GPIOE,(0x0400<<i)))
		{
            TrigL_count[i] = 0;
			TrigH_count[i]++;
			if(TrigH_count[i]>=2)
			{
				TrigH_count[i] = 2;
				
				SwitchsState |= (0x01<<i);//set bit			
			}
		}
		else
		{
            TrigH_count[i] = 0;
			TrigL_count[i]++;
			if(TrigL_count[i]>=2)
			{
				
				TrigL_count[i] = 2;
				SwitchsState &= (~(0x01<<i));
			}
		}
	}
 return SwitchsState;
}

void Collisions_Init(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 |GPIO_Pin_13 |GPIO_Pin_14|GPIO_Pin_15 ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 |GPIO_Pin_9 |GPIO_Pin_10|GPIO_Pin_11 | GPIO_Pin_12;
 // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}


uint16_t GetCollisionsState(void)
{
	static uint16_t CollisionsState = 0;
	static uint8_t TrigH_count[9]={0};
	static uint8_t TrigL_count[9]={0};
	uint8_t i;
	
			//碰撞开关检测	
	for(i=0;i<5;i++)
	{
		if(GPIO_ReadInputDataBit(GPIOD,(0x0100<<i)))
		{
			TrigH_count[i]++;
            TrigL_count[i] = 0;
			if(TrigH_count[i]>=2)
			{
				TrigH_count[i] = 2;
				
				CollisionsState |= (0x01<<i);//set bit			
			}
		}
		else
		{
			TrigL_count[i]++;
            TrigH_count[i] = 0;
			if(TrigL_count[i]>=2)
			{
				
				TrigL_count[i] = 2;
				CollisionsState &= (~(0x01<<i));
			}
		}
	}
	
				//碰撞开关检测	
	for(i=0;i<4;i++)
	{
		if(GPIO_ReadInputDataBit(GPIOB,(0x1000<<i)))
		{
			TrigH_count[i+5]++;
            TrigL_count[i+5] = 0;
			if(TrigH_count[i+5]>=2)
			{
				TrigH_count[i+5] = 2;
				
				CollisionsState |= (0x20<<i);//set bit			
			}
		}
		else
		{
			TrigL_count[i+5]++;
            TrigH_count[i+5] = 0;
			if(TrigL_count[i+5]>=2)
			{
				
				TrigL_count[i+5] = 2;
				CollisionsState &= (~(0x20<<i));
			}
		}
	}
 return CollisionsState;
}


uint8_t GetCollisionsState_3(void)
{
	static uint8_t CollisionsState_3 = 0;
	static uint8_t TrigH_count[3]={0};
	static uint8_t TrigL_count[3]={0};
	uint8_t i;
	
				//碰撞开关检测	
	for(i=0;i<3;i++)
	{
		if(GPIO_ReadInputDataBit(GPIOB,(0x1000<<i)))
		{
			TrigH_count[i]++;
      TrigL_count[i] = 0;
			if(TrigH_count[i]>=2)
			{
				TrigH_count[i] = 2;			
				CollisionsState_3 &= (~(0x20<<i));//set bit			
			}
		}
		else
		{
			TrigL_count[i]++;
            TrigH_count[i] = 0;
			if(TrigL_count[i]>=2)
			{				
				TrigL_count[i] = 2;
				CollisionsState_3 |= ((0x20<<i));
			}
		}
	}
 return CollisionsState_3;
}


void Limits_Init(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 |GPIO_Pin_4  ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

uint8_t GetLimitsState(void)
{
	static uint8_t LimitsState = 0;
	static uint8_t TrigH_count[2]={0};
	static uint8_t TrigL_count[2]={0};
	uint8_t i;
	
			//碰撞开关检测	
	for(i=0;i<2;i++)
	{
		if(GPIO_ReadInputDataBit(GPIOD,(0x0008<<i)))
		{
			TrigH_count[i]++;
			if(TrigH_count[i]>=2)
			{
				TrigH_count[i] = 2;
				TrigL_count[i] = 0;
				LimitsState |= (0x01<<i);//set bit			
			}
		}
		else
		{
			TrigL_count[i]++;
			if(TrigL_count[i]>=2)
			{
				TrigH_count[i] = 0;
				TrigL_count[i] = 2;
				LimitsState &= (~(0x01<<i));
			}
		}
	}
 return LimitsState;
}


//uint8_t GetUnloadState(void)
//{
//	static	uint8_t Unload_flag=0;
//	static  uint8_t UnloadH_count=0;
//	static  uint8_t UnloadL_count=0;

////		if(UNLOAD_SWITCH_PORT->IDR&&UNLOAD_SWITCH_PIN)
//		if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3))
//		{
//			UnloadH_count++;
//			if(UnloadH_count>=2)
//			{
//				UnloadH_count = 2;
//				UnloadL_count = 0;
//				Unload_flag =1;

//			}
//		}
//		else
//		{
//			UnloadL_count++;
//			if(UnloadL_count>=2)
//			{
//				UnloadL_count = 2;
//				UnloadH_count = 0;
//				Unload_flag = 0;
//			}
//		}
//		return Unload_flag;
//}


//uint8_t GetUplimittate(void)
//{
//	static	uint8_t Uplimit_flag=0;
//	static  uint8_t UplimitH_count=0;
//	static  uint8_t UplimitL_count=0;

////		if(LOW_LIMIT_PORT->IDR&&LOW_LIMIT_PIN)
//	if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3))
//		{
//			UplimitH_count++;
//			if(UplimitH_count>=2)
//			{
//				UplimitH_count = 2;
//				UplimitL_count = 0;
//				Uplimit_flag =1;

//			}
//		}
//		else
//		{
//			UplimitL_count++;
//			if(UplimitL_count>=2)
//			{
//				UplimitL_count = 2;
//				UplimitH_count = 0;
//				Uplimit_flag = 0;
//			}
//		}
//		return Uplimit_flag;
//}

//uint8_t GetLowlimittate(void)
//{
//	static	uint8_t Lowlimit_flag=0;
//	static  uint8_t LowlimitH_count=0;
//	static  uint8_t LowlimitL_count=0;

////		if(UNLOAD_SWITCH_PORT->IDR&&UNLOAD_SWITCH_PIN)
//		if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_4))
//		{
//			LowlimitH_count++;
//			if(LowlimitH_count>=2)
//			{
//				LowlimitH_count = 2;
//				LowlimitL_count = 0;
//				Lowlimit_flag =1;

//			}
//		}
//		else
//		{
//			LowlimitL_count++;
//			if(LowlimitL_count>=2)
//			{
//				LowlimitL_count = 2;
//				LowlimitH_count = 0;
//				Lowlimit_flag = 0;
//			}
//		}
//		return Lowlimit_flag;
//}

void LoadSpeed(float v)
{
	uint16_t peroid;
	uint32_t freq;
  float   speed;

	speed = v;
	freq=fabs(v*1000);
	
		//设置运动方向		
	if(speed<0)
	{
		GPIO_ResetBits(GPIOC,GPIO_Pin_6);
		
	}
	else
	{
		GPIO_SetBits(GPIOC,GPIO_Pin_6);
	}
	
	if(speed==0)
	{
		TIM8->CCR2 = 0;
	}
	else
	{
		peroid=1000000/freq;	
		TIM8->ARR = peroid;
		TIM8->CCR2 = peroid/2;
	}	
}

uint32_t crc_chk(uint8_t *data,uint8_t length)
{
	int32_t j;
	uint32_t reg_crc = 0xFFFF;
	
	while(length--)
	{
		reg_crc ^= *data++;
		for(j=0;j<8;j++)
		{
			if(reg_crc &0x01)
			{
				reg_crc = (reg_crc >> 1)^0xA001;
			}
			else
			{
				reg_crc = (reg_crc >> 1);
			}
		}
	}
	return reg_crc;
}

void UART5_SendDatas(uint8_t *data,uint8_t length)
{
	uint32_t crc;
	crc = crc_chk(data, length - 2);
	data[length-2] = crc;
	data[length-1] = crc>>8;
	//USART_SendStr(UART5,data);
	Usart_SendBuffer(UART5,data,length);
}



void UART5_SendCMD(USART_TypeDef *USARTx,uint8_t mode,uint16_t add,uint16_t nums,uint16_t *data)
{
	uint8_t TxBuff[16] = {0};
	uint16_t dataLen =nums*2 ;
	uint8_t i = 0;
	uint32_t crc;
	uint16_t txLen = 0;
//	uint16_t rxLen = 0;
	
	switch(mode)
	{
		case HLS_READ:
				i = 0;
				TxBuff[i] = 0x01;i++;
				TxBuff[i] = mode;i++;
				TxBuff[i] = (add>>8);i++;
				TxBuff[i] = add;i++;
				TxBuff[i] = (nums>>8);i++;
				TxBuff[i] = nums;i++;				
				crc = crc_chk(TxBuff, 6 );
				TxBuff[i]  = crc;i++;
				TxBuff[i]  = crc>>8;i++;
				txLen = i;
	//	    rxLen = dataLen + 4;		
			
			break;
		case HLS_WRITE:
				i = 0;
				TxBuff[i] = 0x01;i++;
				TxBuff[i] = mode;i++;
				TxBuff[i] = (add>>8);i++;
				TxBuff[i] = add;i++;
				TxBuff[i] = ((*data)>>8);i++;
				TxBuff[i] = (*data);i++;					
				crc = crc_chk(TxBuff, 6 );
				TxBuff[i]  = crc;i++;
				TxBuff[i]  = crc>>8;i++;
				txLen = i;
//		    rxLen = 8;		
			break;
		case HLS_WRITES:
			   i = 0;
				TxBuff[i] = 0x01;i++;
				TxBuff[i] = mode;i++;
				TxBuff[i] = (add>>8);i++;
				TxBuff[i] = add;i++;
				TxBuff[i] = (nums>>8);i++;
				TxBuff[i] = nums;i++;			
				memcpy(&TxBuff[i],data,nums);			
				crc = crc_chk(TxBuff, 6+dataLen );
				TxBuff[i+dataLen]  = crc;i++;
				TxBuff[i+dataLen]  = crc>>8;i++;
				txLen = i;
	//	    rxLen = 8;		
			break;
		
				case HLS_WR:
				{
					uint8_t j = 0;
					i = 0;
					TxBuff[i] = 0x01;i++;
					TxBuff[i] = mode;i++;	
					TxBuff[i] = add;i++;	
					TxBuff[i] = nums;i++;				
					//memcpy(&TxBuff[i],data,dataLen);		
					for(j = 0;j<nums;j++)
					{
						TxBuff[i] = (*(data+j)>>8);i++;
						TxBuff[i] = *(data+j);i++;
					}
					crc = crc_chk(TxBuff, i );
					TxBuff[i]  = crc;i++;
					TxBuff[i]  = crc>>8;i++;
					txLen = i;
//					rxLen = dataLen + 6;
				}
			break;
		
		default :
			break;

	}

	//USART_SendStr(UART5,data);
	Usart_SendBuffer(UART5,TxBuff,txLen);
}

uint8_t ErrCode(uint16_t in)
{
	uint8_t out;
	switch(in)
	{
		case E_0000:
			out = ERR_NONE;
			break;
		
		case F_1200: 
			out = FAULT_LOWRES;
			break;
		
	 case E_1301: 
			out = ERR_OVERTIME;
			break;
	 
	 	case E_1401: 
			out = ERR_NOACK;
			break;
		
		case E_1500: 
			out = ERR_ENCODER_AB;
			break;
		
	case E_1510: 
			out = ERR_ENCODER_UVW;
			break;
		
	 case F_1600: 
			out = FAULT_FRAM_ERR;
			break;
	 
	 	case E_1700: 
			out = ERR_ELECGEAR;
			break;
		
		case E_2200: 
			out = ERR_LOWPOWER;
			break;
		
	case E_2500: 
			out = ERR_OVERCURRENT_A;
			break;
		
	case E_2501: 
			out = ERR_OVERCURRENT_B;
			break;
		
	 case E_2502: 
			out = ERR_OVERCURRENT_C;
			break;
	 
	 	case E_2510: 
			out = FAULT_OVER_LOAD;
			break;
		
		case E_2520: 
			out = ERR_OVER_SPEED;
			break;
		
				case F_2530: 
			out = FAULT_OVER_POWER;
			break;
		
	case F_2600: 
			out = FAULT_PARAMSERR;
			break;
		
	case E_2610: 
			out = ERR_OVER_POSCOUNT;
			break;
		
	 case E_2645: 
			out = ERR_OVER_POS;
			break;
	 
	 	case F_2660: 
			out = FAULT_1PHASE_ERR;
			break;
		
		case F_2661: 
			out = FAULT_2PHASE_ERR;
			break;
		
			case E_2900: 
			out = ERR_NOSPEED;
			break;
		
	 case F_3110: 
			out = FAULT_MATCH_ERR;
			break;
	 
	 	case E_3600: 
			out = ERR_ENCODERZ_LOST;
			break;
		
		case E_3601: 
			out = ERR_ENCODERZ_OVER;
			break;
		
		default:
			break;

	}
	return out; 
}

