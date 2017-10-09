#include "stm32f10x.h"
#include "common.h"

//typedef enum 
//{
//    UI2C_ERR_NONE        =       0,
//    UI2C_ERR_TIMEOUT     =       1,
//    UI2C_ERR_FAILED      =       2
//}_I2C_ERR;

#define	DATA_ERR	 0xFF
#define DEV_ERR    0xFE

#define VL6180X_ADDRESS												0x52 
#define EXPANDER_I2C_ADDRESS    						(0x42*2)
#define SYSTEM__MODE_GPIO0   									0x10
#define SYSTEM__HISTORY_CTRL   								0x12
#define SYSTEM__INTERRUPT_CONFIG_GPIO   			0x14
#define I2C_SLAVE__DEVICE_ADDRESS							0x212
#define RESULT__RANGE_VAL											0x062
#define READOUT__AVERAGING_SAMPLE_PERIOD    	0x10A
#define SYSRANGE__EARLY_CONVERGENCE_ESTIMATE 	0x22
#define SYSRANGE__START												0x18
#define RESULT__INTERRUPT_STATUS_GPIO					0x4F
#define SYSTEM__INTERRUPT_CLEAR								0x15

#define SYSRANGE_MAX_CONVERGENCE_TIME         0x01C
#define SYSTEM_MODE_GPIO1                     0x011
#define SYSTEM_INTERRUPT_CLEAR                0x015
#define RESULT__RANGE_STATUS									0x4D

#define SYSRANGE__RANGE_IGNORE_VALID_HEIGHT		0x025
#define RANGE_SCALER                          0x096
#define SYSRANGE_PART_TO_PART_RANGE_OFFSET    0x024

#define CHIP_NUM_I2C0			5

#define CHIP1         0
#define CHIP2         1
#define CHIP3         2
#define CHIP4         3
#define CHIP5         4
#define CHIP6         5
#define CHIP7         6



#define CHIPEN1_I2C0  GPIOout(GPIOF, 4)//CS1
#define CHIPEN2_I2C0 	GPIOout(GPIOE, 6)//CS2
#define CHIPEN3_I2C0 	GPIOout(GPIOE, 5)//CS3
#define CHIPEN4_I2C0 	GPIOout(GPIOE, 3)//CS4
#define CHIPEN5_I2C0 	GPIOout(GPIOE, 1)//CS5
#define CHIPEN6_I2C0 	GPIOout(GPIOB, 9)//CS6


#define     UI2C_FLAG_TIMEOUT        ((uint32_t)0x1000)
#define     UI2C_LONG_TIMEOUT        ((uint32_t)(10 * UI2C_FLAG_TIMEOUT))

#define VL6180X_CE1_OPEN	GPIOF->BSRR=GPIO_Pin_4
#define VL6180X_CE1_CLOSE GPIOF->BRR=GPIO_Pin_4
#define VL6180X_CE2_OPEN	GPIOE->BSRR=GPIO_Pin_6
#define VL6180X_CE2_CLOSE GPIOE->BRR=GPIO_Pin_6
#define VL6180X_CE3_OPEN  GPIOE->BSRR=GPIO_Pin_5
#define VL6180X_CE3_CLOSE GPIOE->BRR=GPIO_Pin_5
#define VL6180X_CE4_OPEN  GPIOE->BSRR=GPIO_Pin_3
#define VL6180X_CE4_CLOSE GPIOE->BRR=GPIO_Pin_3
#define VL6180X_CE5_OPEN	GPIOE->BSRR=GPIO_Pin_1
#define VL6180X_CE5_CLOSE GPIOE->BRR=GPIO_Pin_1
#define VL6180X_CE6_OPEN	GPIOB->BSRR=GPIO_Pin_9
#define VL6180X_CE6_CLOSE GPIOB->BRR=GPIO_Pin_9

#define	SLAVE_ADDRESS1			0x2A
#define VL6180X_ADDRESS1		0x54
#define	SLAVE_ADDRESS2			0x28
#define VL6180X_ADDRESS2		0x50
#define	SLAVE_ADDRESS3			0x27
#define VL6180X_ADDRESS3		0x4E
#define	SLAVE_ADDRESS4			0x26
#define VL6180X_ADDRESS4		0x4C
#define	SLAVE_ADDRESS5			0x25
#define VL6180X_ADDRESS5		0x4A
#define	SLAVE_ADDRESS6			0x24
#define VL6180X_ADDRESS6		0x48


extern int test_vl6180x(void);

void init_vl6180x_I2C0(unsigned char DeviceAddr);

extern void start_vl6180x_I2C0(void);

extern void end_vl6180x_I2C0(void);

unsigned char GetFallenData(unsigned char DeviceAddr);
void Vl6180X_Init(void);
void GPIO_I2C0_Init(void);



