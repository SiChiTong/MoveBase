#ifndef _LINEAR_ACTUATOR_H
#define _LINEAR_ACTUATOR_H

#include "stm32f10x.h"

/*****************up limit*********************/
//#define UP_LIMIT_PORT             GPIOE
//#define UP_LIMIT_CLK              RCC_APB2Periph_GPIOE
//#define UP_LIMIT_PIN              GPIO_Pin_3
//#define UP_LIMIT_PORT_SOURCE      GPIO_PortSourceGPIOE
//#define UP_LIMIT_PIN_SOURCE       GPIO_PinSource3
//#define UP_LIMIT_EXTI_LINE        EXTI_Line3
//#define UP_LIMIT_EXTI_IRQ         EXTI3_IRQn
//#define UP_OFFSET_BITS            3

//#define LOW_LIMIT_PORT             GPIOE
//#define LOW_LIMIT_CLK              RCC_APB2Periph_GPIOE
//#define LOW_LIMIT_PIN              GPIO_Pin_2
//#define LOW_LIMIT_PORT_SOURCE      GPIO_PortSourceGPIOE
//#define LOW_LIMIT_PIN_SOURCE       GPIO_PinSource2
//#define LOW_LIMIT_EXTI_LINE        EXTI_Line2
//#define LOW_LIMIT_EXTI_IRQ         EXTI2_IRQn
//#define LOW_OFFSET_BITS            2

//#define LOAD_SWITCH_PORT             GPIOB
//#define LOAD_SWITCH_CLK              RCC_APB2Periph_GPIOB
//#define LOAD_SWITCH_PIN              GPIO_Pin_8
//#define LOAD_SWITCH_PORT_SOURCE      GPIO_PortSourceGPIOB
//#define LOAD_SWITCH_PIN_SOURCE       GPIO_PinSource8
//#define LOAD_SWITCH_EXTI_LINE        EXTI_Line8
//#define LOAD_SWITCH_EXTI_IRQ         EXTI9_5_IRQn
//#define LOAD_SWITCH_OFFSET_BITS      8

//#define UNLOAD_SWITCH_PORT             GPIOB
//#define UNLOAD_SWITCH_CLK              RCC_APB2Periph_GPIOB
//#define UNLOAD_SWITCH_PIN              GPIO_Pin_9
//#define UNLOAD_SWITCH_PORT_SOURCE      GPIO_PortSourceGPIOB
//#define UNLOAD_SWITCH_PIN_SOURCE       GPIO_PinSource9
//#define UNLOAD_SWITCH_EXTI_LINE        EXTI_Line9
//#define UNLOAD_SWITCH_EXTI_IRQ         EXTI9_5_IRQn
//#define UNLOAD_SWITCH_OFFSET_BITS      9


#define UP_LIMIT_PORT             GPIOD
#define UP_LIMIT_CLK              RCC_APB2Periph_GPIOD
#define UP_LIMIT_PIN              GPIO_Pin_3
#define UP_LIMIT_PORT_SOURCE      GPIO_PortSourceGPIOD
#define UP_LIMIT_PIN_SOURCE       GPIO_PinSource3
#define UP_LIMIT_EXTI_LINE        EXTI_Line3
#define UP_LIMIT_EXTI_IRQ         EXTI3_IRQn
#define UP_OFFSET_BITS            3

#define LOW_LIMIT_PORT             GPIOD
#define LOW_LIMIT_CLK              RCC_APB2Periph_GPIOD
#define LOW_LIMIT_PIN              GPIO_Pin_4
#define LOW_LIMIT_PORT_SOURCE      GPIO_PortSourceGPIOD
#define LOW_LIMIT_PIN_SOURCE       GPIO_PinSource4
#define LOW_LIMIT_EXTI_LINE        EXTI_Line4
#define LOW_LIMIT_EXTI_IRQ         EXTI4_IRQn
#define LOW_OFFSET_BITS            4

#define LOAD_SWITCH_PORT             GPIOB
#define LOAD_SWITCH_CLK              RCC_APB2Periph_GPIOB
#define LOAD_SWITCH_PIN              GPIO_Pin_8
#define LOAD_SWITCH_PORT_SOURCE      GPIO_PortSourceGPIOB
#define LOAD_SWITCH_PIN_SOURCE       GPIO_PinSource8
#define LOAD_SWITCH_EXTI_LINE        EXTI_Line8
#define LOAD_SWITCH_EXTI_IRQ         EXTI9_5_IRQn
#define LOAD_SWITCH_OFFSET_BITS      8

#define UNLOAD_SWITCH_PORT             GPIOE
#define UNLOAD_SWITCH_CLK              RCC_APB2Periph_GPIOE
#define UNLOAD_SWITCH_PIN              GPIO_Pin_10
#define UNLOAD_SWITCH_PORT_SOURCE      GPIO_PortSourceGPIOE
#define UNLOAD_SWITCH_PIN_SOURCE       GPIO_PinSource10
#define UNLOAD_SWITCH_EXTI_LINE        EXTI_Line10
#define UNLOAD_SWITCH_EXTI_IRQ         EXTI15_10_IRQn
#define UNLOAD_SWITCH_OFFSET_BITS      10

#define DIR_SWITCH_PORT             GPIOE
#define DIR_SWITCH_CLK              RCC_APB2Periph_GPIOE
#define DIR_SWITCH_PIN              GPIO_Pin_0
#define DIR_SWITCH_PORT_SOURCE      GPIO_PortSourceGPIOE
#define DIR_SWITCH_PIN_SOURCE       GPIO_PinSource0
#define DIR_SWITCH_EXTI_LINE        EXTI_Line0
#define DIR_SWITCH_EXTI_IRQ         EXTI0_IRQn
#define DIR_SWITCH_OFFSET_BITS      0

//#define POWER_SWITCH_PORT             GPIOE
//#define POWER_SWITCH_CLK              RCC_APB2Periph_GPIOE
//#define POWER_SWITCH_PIN              GPIO_Pin_1
//#define POWER_SWITCH_PORT_SOURCE      GPIO_PortSourceGPIOE
//#define POWER_SWITCH_PIN_SOURCE       GPIO_PinSource1
//#define POWER_SWITCH_EXTI_LINE        EXTI_Line1
//#define POWER_SWITCH_EXTI_IRQ         EXTI1_IRQn
//#define POWER_SWITCH_OFFSET_BITS      1

#define POWER_SWITCH_PORT             GPIOG
#define POWER_SWITCH_CLK              RCC_APB2Periph_GPIOG
#define POWER_SWITCH_PIN              GPIO_Pin_12
#define POWER_SWITCH_PORT_SOURCE      GPIO_PortSourceGPIOG
#define POWER_SWITCH_PIN_SOURCE       GPIO_PinSource12
#define POWER_SWITCH_EXTI_LINE        EXTI_Line12
#define POWER_SWITCH_EXTI_IRQ         EXTI15_10_IRQn
#define POWER_SWITCH_OFFSET_BITS      12

typedef union
{
	struct
	{
		uint8_t state;
		uint8_t uplimit;
		uint8_t lowlimit;
	}Linear_State;
	char data[3];
}LINEAR_ACTUATOR_UNION;
//void UpLimitInit(void);
//void LowLimitInit(void);
//void LoadSwitchInit(void);
//void UnloadSwitchInit(void);
//void PowerSwitchInit(void);
void UpActuatorInit(void);

void PowerON(void);
void PowerOFF(void);

void Load(void);
void UnLoad(void);

uint8_t GetLoadState(void);
uint8_t GetUnloadState(void);
uint8_t GetUplimittate(void);
uint8_t GetLowlimittate(void);


#endif

