#include "linear_actuator.h"
#include "motor.h"

/**************up limit*******************/
void UpLimitInit(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd( UP_LIMIT_CLK , ENABLE);
	 
	GPIO_InitStructure.GPIO_Pin = UP_LIMIT_PIN ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_IPD;
  GPIO_Init(UP_LIMIT_PORT, &GPIO_InitStructure);	
	
	  /* Enable AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  /* Connect EXTI0 Line to PA.00 pin */
  GPIO_EXTILineConfig(UP_LIMIT_PORT_SOURCE, UP_LIMIT_PIN_SOURCE);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = UP_LIMIT_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = DISABLE;//ENABLE;//
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = UP_LIMIT_EXTI_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;//ENABLE;//
  NVIC_Init(&NVIC_InitStructure);
	
}

void LowLimitInit(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd( LOW_LIMIT_CLK , ENABLE);
	 
	GPIO_InitStructure.GPIO_Pin = LOW_LIMIT_PIN ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_IPD;
  GPIO_Init(LOW_LIMIT_PORT, &GPIO_InitStructure);	
	
		  /* Enable AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  /* Connect EXTI0 Line to PA.00 pin */
  GPIO_EXTILineConfig(LOW_LIMIT_PORT_SOURCE, LOW_LIMIT_PIN_SOURCE);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = LOW_LIMIT_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = DISABLE;//ENABLE;//
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = LOW_LIMIT_EXTI_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 13;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;//ENABLE;//
  NVIC_Init(&NVIC_InitStructure);
}

void LoadSwitchInit(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	RCC_APB2PeriphClockCmd( LOAD_SWITCH_CLK , ENABLE);
	 
	GPIO_InitStructure.GPIO_Pin = LOAD_SWITCH_PIN ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(LOAD_SWITCH_PORT, &GPIO_InitStructure);	
	
			  /* Enable AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  /* Connect EXTI0 Line to PA.00 pin */
  GPIO_EXTILineConfig(LOAD_SWITCH_PORT_SOURCE, LOAD_SWITCH_PIN_SOURCE);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = LOAD_SWITCH_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = DISABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = LOAD_SWITCH_EXTI_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStructure);
}


void UnloadSwitchInit(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	RCC_APB2PeriphClockCmd( UNLOAD_SWITCH_CLK , ENABLE);
	 
	GPIO_InitStructure.GPIO_Pin = UNLOAD_SWITCH_PIN ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(UNLOAD_SWITCH_PORT, &GPIO_InitStructure);	
	
			  /* Enable AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  /* Connect EXTI0 Line to PA.00 pin */
  GPIO_EXTILineConfig(UNLOAD_SWITCH_PORT_SOURCE, UNLOAD_SWITCH_PIN_SOURCE);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = UNLOAD_SWITCH_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = DISABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = UNLOAD_SWITCH_EXTI_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void DirSwitchInit(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	RCC_APB2PeriphClockCmd( DIR_SWITCH_CLK , ENABLE);
	 
	GPIO_InitStructure.GPIO_Pin = DIR_SWITCH_PIN ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_IPD;
  GPIO_Init(DIR_SWITCH_PORT, &GPIO_InitStructure);	
	
			  /* Enable AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  /* Connect EXTI0 Line to PA.00 pin */
  GPIO_EXTILineConfig(DIR_SWITCH_PORT_SOURCE, DIR_SWITCH_PIN_SOURCE);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = DIR_SWITCH_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = DISABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = DIR_SWITCH_EXTI_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void PowerSwitchInit(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	RCC_APB2PeriphClockCmd( POWER_SWITCH_CLK , ENABLE);
	 
	GPIO_InitStructure.GPIO_Pin = POWER_SWITCH_PIN ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_IPD;
  GPIO_Init(POWER_SWITCH_PORT, &GPIO_InitStructure);	
	
			  /* Enable AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  /* Connect EXTI0 Line to PA.00 pin */
  GPIO_EXTILineConfig(POWER_SWITCH_PORT_SOURCE, POWER_SWITCH_PIN_SOURCE);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = POWER_SWITCH_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = DISABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = POWER_SWITCH_EXTI_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void PowerON(void)
{
	POWER_SWITCH_PORT->ODR |= POWER_SWITCH_PIN;
}

void PowerOFF(void)
{
	POWER_SWITCH_PORT->ODR &= (~POWER_SWITCH_PIN);
}

void Load(void)
{
	POWER_SWITCH_PORT->ODR |= POWER_SWITCH_PIN;
	DIR_SWITCH_PORT->ODR |= DIR_SWITCH_PIN;
}

void UnLoad(void)
{
	POWER_SWITCH_PORT->ODR |= POWER_SWITCH_PIN;
	DIR_SWITCH_PORT->ODR &= (~DIR_SWITCH_PIN);
}


uint8_t GetLoadState(void)
{
	static	uint8_t Load_flag=0;
	static  uint8_t LoadH_count=0;
	static  uint8_t LoadL_count=0;

//		if(LOAD_SWITCH_PORT->IDR&&LOAD_SWITCH_PIN)
	if(GPIO_ReadInputDataBit(LOAD_SWITCH_PORT,LOAD_SWITCH_PIN))
		{
			LoadH_count++;
			if(LoadH_count>=2)
			{
				LoadH_count = 2;
				LoadL_count = 0;
				Load_flag =1;

			}
		}
		else
		{
			LoadL_count++;
			if(LoadL_count>=2)
			{
				LoadL_count = 2;
				LoadH_count = 0;
				Load_flag = 0;
			}
		}
		
		return Load_flag;
}

uint8_t GetUnloadState(void)
{
	static	uint8_t Unload_flag=0;
	static  uint8_t UnloadH_count=0;
	static  uint8_t UnloadL_count=0;

//		if(UNLOAD_SWITCH_PORT->IDR&&UNLOAD_SWITCH_PIN)
		if(GPIO_ReadInputDataBit(UNLOAD_SWITCH_PORT,UNLOAD_SWITCH_PIN))
		{
			UnloadH_count++;
			if(UnloadH_count>=2)
			{
				UnloadH_count = 2;
				UnloadL_count = 0;
				Unload_flag =1;

			}
		}
		else
		{
			UnloadL_count++;
			if(UnloadL_count>=2)
			{
				UnloadL_count = 2;
				UnloadH_count = 0;
				Unload_flag = 0;
			}
		}
		return Unload_flag;
}


uint8_t GetUplimittate(void)
{
	static	uint8_t Uplimit_flag=0;
	static  uint8_t UplimitH_count=0;
	static  uint8_t UplimitL_count=0;

//		if(LOW_LIMIT_PORT->IDR&&LOW_LIMIT_PIN)
	if(GPIO_ReadInputDataBit(UP_LIMIT_PORT,UP_LIMIT_PIN))
		{
			UplimitH_count++;
			if(UplimitH_count>=2)
			{
				UplimitH_count = 2;
				UplimitL_count = 0;
				Uplimit_flag =1;

			}
		}
		else
		{
			UplimitL_count++;
			if(UplimitL_count>=2)
			{
				UplimitL_count = 2;
				UplimitH_count = 0;
				Uplimit_flag = 0;
			}
		}
		return Uplimit_flag;
}

uint8_t GetLowlimittate(void)
{
	static	uint8_t Lowlimit_flag=0;
	static  uint8_t LowlimitH_count=0;
	static  uint8_t LowlimitL_count=0;

//		if(UNLOAD_SWITCH_PORT->IDR&&UNLOAD_SWITCH_PIN)
		if(GPIO_ReadInputDataBit(LOW_LIMIT_PORT,LOW_LIMIT_PIN))
		{
			LowlimitH_count++;
			if(LowlimitH_count>=2)
			{
				LowlimitH_count = 2;
				LowlimitL_count = 0;
				Lowlimit_flag =1;

			}
		}
		else
		{
			LowlimitL_count++;
			if(LowlimitL_count>=2)
			{
				LowlimitL_count = 2;
				LowlimitH_count = 0;
				Lowlimit_flag = 0;
			}
		}
		return Lowlimit_flag;
}


void UpActuatorInit(void)
{
 UpLimitInit();
 LowLimitInit();
 LoadSwitchInit();
 UnloadSwitchInit();
 PowerSwitchInit();
 DirSwitchInit();
	
}

