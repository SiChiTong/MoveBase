/**-------------------------------------------------------------------------
*@ file				common.c
*@ brief			非独立模块的初始化与配置
*@ author 		ZGJ
*@ vertion		V0.01
*@ date				2015-12-15
*@ statement	(C)	COPYRIGHT 2015 KSITRI.
--------------------------------------------------------------------------*/
/* Include files---------------------------------------------------------*/
#include "common.h"
#include "adc.h"
#include "stm32f10x.h"
#include "usart.h"
#include "ucos_ii.h"
#include "motor.h"
#include "vl6180x.h"
#include "i2cio.h"
#include "modbus.h"
#include "sys_config.h"
#include "iic.h"
#include "linear_actuator.h"
#include "loadmotor.h"
#include "mpu9250.h"



/* variables-------------------------------------------------------------*/
volatile unsigned long SysStateRegister = 0;
char DriverPowerState=1;
_FallSensor_Thread	gfallSensorThread={67,27};

SYS_SENSOR_STRUCT sys_config;

char HardwareVer[]="0201";
char SoftVer[]="NoahC001M06A001";

char BoardID=0;//板卡ID号
char UpdateFlag=0;//升级标志位
char UpdateData[255]={0};//用于存储升级的数据包


/* functions------------------------------------------------------------*/
static void RCC_Configuration(void);
static void PeriphClock_Configuration(void);
static void NVIC_Configuration(void);
static void GPIO_Configuration(void);
static void Iwdg_Configuration(void);
static void SystemTick_Configuration(void);
static void PZKey_LowLevel_Init(void);
static void Timer7_Init(void);
void Collision_Init(void);
//void Laser_Init(void);
void StopSwitch_Init(void);

//static void LOAD8_PWM_Init(int period,int prescaler);
static void RIGHT3_PWM_Init(int period,int prescaler);
static void LEFT2_PWM_Init(int period,int prescaler);
static void MoveDirevtion(void);
/*--------------------------------------------------------------------------
--------------------------------------------------------------------------*/
/**
*@ brief		硬件底层驱动
*@ para			none
*@ retval		none
*/
void HardWare_LowLevel_Init(void)
{
	sys_config.state=0X08;
	RCC_Configuration();
	PeriphClock_Configuration();
	SystemTick_Configuration();
	GPIO_Configuration();
	NVIC_Configuration();
	Delayus(1000);
	ADC_Configuration();				//ADC
	USART_LowLevel_Init();			//串口
	Iwdg_Configuration();				//看门狗
	
	
	Timer7_Init();
	Collision_Init();
//	Laser_Init();
	StopSwitch_Init();
	
	ID_NumberInit();
	BoardID=GetIDnum();
	DisChargeInit();
	
	PZKey_LowLevel_Init();
	Motor_LowLevel_Init();			//电机
//	I2C0_Init();
//	Vl6180X_Init();
	
	
	if(BoardID==0)
	{
		//RIGHT8_PWM_Init(PERIOD-1,PRESCALER-1);//右轮PWM控制初始化
		//LEFT2_PWM_Init(PERIOD-1,PRESCALER-1);//左轮PWM控制初始化
        LOAD8_PWM_Init(PERIOD-1,PRESCALER-1);//
        RIGHT3_PWM_Init(PERIOD-1,PRESCALER-1);
        LEFT2_PWM_Init(PERIOD-1,PRESCALER-1);
        MoveDirevtion();	
        USART1_Config(115200);
        USART3_Config(115200);
	}
	
	UpActuatorInit();
	UART5_Config(115200);
	Switchs_Init();
	Collisions_Init();
    
  UART4_Config(115200);
}

/**
*@ brief		系统时钟配置
*@ para			none
*@ retval		none
*/
static void RCC_Configuration(void)
{
    ErrorStatus HSEStartUpStatus;
    /*SYSCLK,HCLK,PCLK1,PCLK2 configuration*/
    /*RCC system reset (for debug purpose)*/
    RCC_DeInit();
    /*enable HSE*/
    RCC_HSEConfig(RCC_HSE_ON);
    /*wait till HSE is ready*/
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    if (HSEStartUpStatus == SUCCESS)
    {
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable); //FLASH预读缓冲，加速
        FLASH_SetLatency(FLASH_Latency_2);  /*FLASH时序延迟，系统频率0-24MHz时，取Latency=0；
                                             24-48MHz时，取Latency=1；48-72MHz时，取Latency=2*/
        /*HCLK = SYSCLK*/
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        /*PLCK1 = HCLK/2*/
        RCC_PCLK1Config(RCC_HCLK_Div2);
        /*PLCK2 = HCLK */
        RCC_PCLK2Config(RCC_HCLK_Div1);
        /*PLLCLK = HSE*9 = 72MHz */
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
        /*Enable PLLCLK */
        RCC_PLLCmd(ENABLE);
        /*wait till PLL is Ready*/
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
        /*Select PLL as system CLK*/
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        /*wait till PLL is used as system clock source */
        while (RCC_GetSYSCLKSource() != 0x08);
    }
		
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		
//		RCC_HSICmd(ENABLE);
    /* Enable Clock Security System(CSS): this will generate an NMI exception
     when HSE clock fails */
  //  RCC_ClockSecuritySystemCmd(ENABLE);  //Enable clock security sysytem (CSS)
}
/**
*@ brief		外设时钟配置
*@ para			none
*@ retval		none
*/
static void PeriphClock_Configuration(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);		//TIM3,4--encoder feedback
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);		//PID cycle
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);		//PZ key
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);		//motor PWM
    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);				//---RS485
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,	ENABLE);			//--com0
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);
}

/**
*@ brief		中断配置
*@ para			none
*@ retval		none
*/
static void NVIC_Configuration(void)
{
    //系统中的中断有：USART2串口接收中断，TIM3、TIM4正交编码器中断，TIM2输入捕获中断
    //ADC传输DMA中断，TIM5定时中断
    NVIC_InitTypeDef NVIC_InitStructure;
    //优先级组设置，4位用于抢占优先级，0位用于响应优先级,此时
    //NVIC_IRQChannelSubPriority的值无效
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
		
	//USART2串口接收中断
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		//USART-TX_DMA
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		//ADC传输DMA中断
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel4_5_IRQn;;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		//PID cycle
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		//PZKey cycle time,10ms
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
//	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 11;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//#if MC_V1_1_BOARD		
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//#endif
//    NVIC_Init(&NVIC_InitStructure);
		
//	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//#if MC_V1_1_BOARD
//    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
//#endif
//    NVIC_Init(&NVIC_InitStructure);
		
		
//NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
		
//	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 11;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

		
}
/**
*@ brief		中断配置
*@ para			none
*@ retval		none
*/
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	//BEEP
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_15;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	//PG5,PG7 DInit
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5|GPIO_Pin_7;
	GPIO_Init(GPIOG,&GPIO_InitStructure);
	
	//电源使能-PB0
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB, GPIO_Pin_0);
  //关机检测-PB1
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//work led
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOG, &GPIO_InitStructure);
  GPIO_SetBits(GPIOG, GPIO_Pin_9);
	//急停开关检测
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//  GPIO_Init(GPIOC, &GPIO_InitStructure);	
#if 0
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		//IO控制引脚
	GPIO_InitStructure.GPIO_Pin = IO_CTRL_PIN1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(IO_CTRL_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = IO_CTRL_PIN2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(IO_CTRL_PORT, &GPIO_InitStructure);
#endif
	
	//485方向控制
	GPIO_InitStructure.GPIO_Pin = RS485_DIR_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(RS485_DIR_PORT, &GPIO_InitStructure);
	RS485DataOut();

}

void Collision_Init(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;
	  /* Enable AFIO clock */
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = PZKey1_Pin|PZKey2_Pin|PZKey3_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(PZKey_Port, &GPIO_InitStructure);


}
#if 0
void Laser_Init(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	  /* Enable AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
  /* Connect EXTI8 Line to PG.08 pin */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource11);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* Configure EXTI8 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line11;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = DISABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);

}
#endif
void StopSwitch_Init(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

}


/**
*@ brief		OS clock configuration
*@ para			none
*@ retval		none
*/
static void SystemTick_Configuration(void)
{
    RCC_ClocksTypeDef rcc_clocks;
    RCC_GetClocksFreq(&rcc_clocks);
    SysTick_Config(rcc_clocks.HCLK_Frequency / OS_TICKS_PER_SEC);
}

/**
 * @brief   IWDG configuration
 * @param   None
 * @retval  None
 */
static void Iwdg_Configuration(void)
{
    /* IWDG timeout equal to 250 ms (the timeout may varies due to LSI frequency
     dispersion) */
    /* Enable write access to IWDG_PR and IWDG_RLR registers */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

    /* IWDG counter clock: LSI/32 */
    IWDG_SetPrescaler(IWDG_Prescaler_32);   // 40K/32 = 1.25K --- 0.8ms

    /* the reload value is between 0--0x0FFF(4095)*/
    IWDG_SetReload(400);       //320ms

    /* Reload IWDG counter */
    IWDG_ReloadCounter();

    /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
    //IWDG_Enable();
}

/**
  * @brief  PZ key detect lowlevel init
  * @param  None
  * @retval None
  */
static void PZKey_LowLevel_Init(void)
{
//		GPIO_InitTypeDef	GPIO_InitStructure;
		TIM_TimeBaseInitTypeDef TIM_TimBaseInitStructure;
	
//		GPIO_InitStructure.GPIO_Pin = PZKey1_Pin|PZKey2_Pin|PZKey3_Pin;
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//		GPIO_Init(PZKey_Port, &GPIO_InitStructure);
	
    TIM_TimBaseInitStructure.TIM_Period = 20000 - 1;   //20ms定时
    TIM_TimBaseInitStructure.TIM_Prescaler = 71;
    TIM_TimBaseInitStructure.TIM_ClockDivision = 0;
    TIM_TimBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM6, &TIM_TimBaseInitStructure);
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);
    TIM_Cmd(TIM6, DISABLE);   //初始化,先关闭计数器
}

static void Timer7_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimBaseInitStructure;
	TIM_TimBaseInitStructure.TIM_Period = 1000 - 1;   //250us定时
  TIM_TimBaseInitStructure.TIM_Prescaler = 71;		//1us
  TIM_TimBaseInitStructure.TIM_ClockDivision = 0;
  TIM_TimBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM7, &TIM_TimBaseInitStructure);
  TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
  TIM_ClearFlag(TIM7, TIM_FLAG_Update);
  TIM_Cmd(TIM7, DISABLE);
}


void LEFT_PWM_Init(int period,int prescaler)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
 // GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);	
	
//		 PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
  /* Time base configuration */
		//T = (1+ prescaler)/72M*(1+period)
  TIM_TimeBaseStructure.TIM_Period = period;
  TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

/* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM4, ENABLE);

  /* TIM4 enable counter */	
	TIM_CtrlPWMOutputs(TIM4,ENABLE);
	
//	TIM_SelectOnePulseMode(TIM4,TIM_OPMode_Single);

  TIM_Cmd(TIM4, ENABLE);

}

// RIGHT MOTOR PWM -- PB1 DIR -- PB1
static void RIGHT3_PWM_Init(int period,int prescaler)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
		//T = (1+ prescaler)/72M*(1+period)
  TIM_TimeBaseStructure.TIM_Period = period;
  TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

/* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM3, ENABLE);

  /* TIM4 enable counter */	
	TIM_CtrlPWMOutputs(TIM3,ENABLE);
	
//	TIM_SelectOnePulseMode(TIM4,TIM_OPMode_Single);

  TIM_Cmd(TIM3, ENABLE);

}

// LEFT MOTOR PWM -- PA1 DIR -- PG6
static void LEFT2_PWM_Init(int period,int prescaler)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
 // GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);	
	
//		 PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
  /* Time base configuration */
		//T = (1+ prescaler)/72M*(1+period)
  TIM_TimeBaseStructure.TIM_Period = period;
  TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

/* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC2Init(TIM2, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM2, ENABLE);

  /* TIM4 enable counter */	
	TIM_CtrlPWMOutputs(TIM2,ENABLE);
	
//	TIM_SelectOnePulseMode(TIM4,TIM_OPMode_Single);

  TIM_Cmd(TIM2, ENABLE);

}


//驱动器正反控制信号
static void MoveDirevtion(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	
	//LOAD DIR
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	//PG6 LEFT DIR
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	
	//PB1 RIGHT DIR
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}


static void ID_NumberInit()
{

	GPIO_InitTypeDef   GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

char GetIDnum()
{
	char IDNum;
	IDNum=(GPIOD->IDR&0x03);
	return IDNum;
}


void DisChargeInit(void)
{

	GPIO_InitTypeDef   GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOG, &GPIO_InitStructure);
}
/**
 * @brief 	PZKey detect
 * @param 	None
 * @retval	_PZ_KEY,can be one of following values
						PZKEY_NONE,PZKEY1,PZKEY2,PZKEY3
 */
_PZ_KEY PZKey_Detect(void)
{
	static unsigned int key1_cnt=0;
	static unsigned int key2_cnt=0;
	static unsigned int key3_cnt=0;
	static unsigned char stamachin=0;
	if(stamachin==0)
	{
		if(!ReadPZKey(PZKey1_Pin))
		{
			if(key1_cnt<=2)
			{
				key1_cnt++;
			}
			else
			{
				stamachin=1;
				return(PZKEY1);
			}
		}
		else
		{
			key1_cnt=0;
		}
		stamachin=1;
		return(PZKEY1_NO);
	}
	else if(stamachin==1)
	{
		if(!ReadPZKey(PZKey2_Pin))
		{
			if(key2_cnt<=2)
			{
				key2_cnt++;
			}
			else
			{
				stamachin=2;
				return(PZKEY2);
			}
		}
		else
		{
			key2_cnt=0;
		}
		stamachin=2;
		return(PZKEY2_NO);
	}
	else if(stamachin==2)
	{
		if(!ReadPZKey(PZKey3_Pin))
		{
			if(key3_cnt<=2)
			{
				key3_cnt++;
			}
			else
			{
				stamachin=0;
				return(PZKEY3);
			}
		}
		else
		{
			key3_cnt=0;
		}
		stamachin=0;
		return(PZKEY3_NO);
	}
	
	return (PZKEY_NONE);
}

/**
 * @brief 	Stop Switch detect
 * @param 	None
 * @retval	1-ON, 0-OFF
 */
unsigned char StopSwitchDet(void)
{
	//20160413,Zero
	if(ReadStopSwitch==0)
	{
//		Delayus(10000);
		//急停开关按下
		if(ReadStopSwitch==0)
		{

			//DISCHARGE
			GPIOG->BRR=GPIO_Pin_5;
			
			CloseDriverPower();
			cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.switch_stop=1;
			
//			if(BoardID==0)
//			{
//				cmdfeed.RIGHT.MOTOR_DRIVER_STRUCT.connect = 1;
//				cmdfeed.LEFT.MOTOR_DRIVER_STRUCT.connect = 1;
//			}
//			
//			if(BoardID==2)
			{
				//复位编码器计数值
					lt_count = 0;					
					rt_count = 0;

			}
		}
		return 1;
	
	}
	else 
	{
//		Delayus(10000);
		//急停开关松开
		if(ReadStopSwitch==1)
		{

			
			//DISCHARGE
			GPIOG->BSRR=GPIO_Pin_5;
			cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.switch_stop=0;
			//20170518
			OpenDriverPower();
		}
		return 0;
	}
	
}

/**
 * @brief 	IO Ctrl detect
 * @param 	none
 * @retval	1-ON, 0-OFF
 */
unsigned char IOCtrlDet1(void)
{
	static unsigned char cnt=0;
	if(ReadIOCtrlPin(IO_CTRL_PIN1)==0x0)
	{
		if(cnt<=3)
		{
			cnt++;
		}
		else
		{
			return 1;
		}
	}
	else
	{
		cnt=0;
	}
	return 0;
}


//unsigned char GetKey(void)
//{
//	unsigned short key=0;

//	if(sys_config.sensor.e_stop)
//	{
//		key=StopSwitchDet();
//	}
//#if NEW_FEEDBACK	
//	if(key==1)
//	{
//			cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.switch_stop=1;
//			cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.normal=0;
//	}
//	else
//	{
//		cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.switch_stop=0;
//	}
//#endif
//	
//	
//		#if NORMALLY_CLOSE
//					key = (PZKey_Port->IDR & 0x7000)>>8;//碰撞开关
//		#endif

//				
//	#if NEW_FEEDBACK			
//		if(key)
//		{
//			cmdfeed.sensor_sta |= key;
//			cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.normal=0;
//		}
//		else
//		{
//			cmdfeed.sensor_sta &= 0x0F;
////			ClearSysStaBit(PZKeySta);
//		}
//		key = (IO_CTRL_PORT->IDR & 0x800);//IO急停
//		if(key)
//		{
//			cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.io_stop=1;
//			cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.normal=0;
//		}
//		else
//		{
//			cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.io_stop=0;
//	//		cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.normal=0;
//		}
//	#endif
//	return key;
//}


 unsigned char GetEStopState(void)
{
	static	unsigned char EStop_flag=0;
	static  unsigned char EStopH_count=0;
	static  unsigned char EStopL_count=0;
			//急停开关
		if(ReadStopSwitch==0)
		{
			EStopL_count++;
			EStopH_count = 0;
			if(EStopL_count>=2)
			{
				EStopL_count = 2;
				EStop_flag =1;
			}
		}
		else
		{
			EStopH_count++;
			EStopL_count = 0;
			if(EStopH_count>=2)
			{
				EStopH_count = 2;
				EStop_flag = 0;
			}
		}	
		return EStop_flag;	
}


uint8_t GetCollisionState(void)
{
	
	static uint8_t ColState = 0;
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
				ColState |= (0x10<<i);//set bit			
			}
		}
		else
		{
			TrigL_count[i]++;
			TrigH_count[i] = 0;
			if(TrigL_count[i]>=2)
			{
				TrigL_count[i] = 2;
				ColState &= (~(0x10<<i));
			}
		}
	}
 return ColState;
}


void Delayus(uint32_t us)
{
    int i, j;
    for (i=0; i<us; i++)
    {
        for (j=0; j<6; j++);
    }
}

void _delay(unsigned int cnt)
{
	unsigned int i=0;
	for(i=0;i<cnt; i++)
	{
		cnt--;
	}
}
/*----------------The End of File-------------------------------------------*/
