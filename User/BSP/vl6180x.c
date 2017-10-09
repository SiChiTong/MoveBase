/**-------------------------------------------------------------------------
*@ file				vl6180x.h
*@ brief			激光传感器驱动,I2C Interface
*@ author 		ZGJ
*@ vertion		V0.01
*@ date				2015-12-20
*@ statement	(C)	COPYRIGHT 2015 KSITRI.
--------------------------------------------------------------------------*/
/* Include Files ---------------------------------------------------------*/
#include "vl6180x.h"
#include "stm32f10x.h"
#include "i2cio.h"
#include "ucos_ii.h"

/* typedef ---------------------------------------------------------------*/

/* define ----------------------------------------------------------------*/

/* variables ------------------------------------------------------------*/

/* functions -------------------------------------------------------------*/
/**
 * @brief  I2C lowlevel init
 * @param  None
 * @retval None
 */
void Vl6180x_I2C_LowLevel_Init(void)
{
		I2C_InitTypeDef I2C_InitStructure;
    GPIO_InitTypeDef    GPIO_InitStructure;

    /* IO Config */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Pin = I2C_SCL_Pin|I2C_SDA_Pin;
    GPIO_Init(I2C_PORT, &GPIO_InitStructure);
		//CE1
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOF, &GPIO_InitStructure);
		//CE2,3,4,5
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_5|GPIO_Pin_3|GPIO_Pin_1;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
		//CE6
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
//    I2C_InitStructure.I2C_OwnAddress1 = DeviceAddr;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    //set OAR1 regester bitt15 to chose 7(0) addr or 10 addr(1) mode, and the bit14 must be set 1 in code.
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;   //300K

    I2C_Cmd(VL6180X_I2C, ENABLE);
    I2C_Init(VL6180X_I2C, &I2C_InitStructure);
}

/**
  * @brief:  Reads a byte of data from the VL6180X
  * @param:  pBuffer-pointer to the buffer that receives the data form
  *          VL6180X.
  * @param:  ReadAddr-VL6180X internal address to start reading .
  * @retval: UI2C_ERR_NONE(0) if operation is correctly performed, else return value
  *          different from UI2C_ERR_NONE(0) or the timeout user callback.
  */
_I2C_ERR single_read(unsigned char DeviceAddr, unsigned short ReadAddr, unsigned char *pBuffer)
{
    uint32_t timeout = 0;
    /* write MSB first, then LSB */
    uint8_t  Addr = 0;
    /* Wait EEPROM StandBy State */
	
    timeout = UI2C_LONG_TIMEOUT;
    do
    {
        I2C_GenerateSTART(I2C2, ENABLE);
        I2C_ReadRegister(I2C2, I2C_Register_SR1);
        I2C_Send7bitAddress(I2C2, DeviceAddr, I2C_Direction_Transmitter);
        if ((timeout --) == 0)
        {
            return (UI2C_ERR_TIMEOUT);
        }
    }
    while(!(I2C_ReadRegister(I2C2, I2C_Register_SR1) & 0x0002));
	
    I2C_ClearFlag(I2C2, I2C_FLAG_AF);
    I2C_GenerateSTOP(I2C2, ENABLE);

    timeout = UI2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY))
    {
        if ((timeout--) == 0)
        {
            return(UI2C_ERR_TIMEOUT);
        }
    }

    /* Send start condition */
    I2C_GenerateSTART(I2C2, ENABLE);

    timeout = UI2C_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
    {
        if ((timeout--) == 0)
        {
            return(UI2C_ERR_TIMEOUT);
        }
    }

    /* Send EEPROM address for write */
    I2C_Send7bitAddress(I2C2, DeviceAddr, I2C_Direction_Transmitter);

    timeout = UI2C_LONG_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
        if ((timeout--) == 0)
        {
            return (UI2C_ERR_TIMEOUT);
        }
    }

    Addr = (uint8_t)((ReadAddr & 0xFF00) >> 8);

    I2C_SendData(I2C2, Addr);

    timeout = UI2C_FLAG_TIMEOUT;
    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_TXE) == RESET)
    {
        if ((timeout --) == 0)
        {
            return (UI2C_ERR_TIMEOUT);
        }
    }

    Addr = (uint8_t) (ReadAddr & 0x00FF);

    I2C_SendData(I2C2, Addr);

    timeout = UI2C_FLAG_TIMEOUT;
    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BTF) == RESET)
    {
        if ((timeout--) == 0)
        {
            return (UI2C_ERR_TIMEOUT);
        }
    }

    /* Send start condition a second time */
    I2C_GenerateSTART(I2C2, ENABLE);

    timeout = UI2C_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
    {
        if ((timeout --) == 0)
        {
            return (UI2C_ERR_TIMEOUT);
        }
    }

    /* Send EEPROM address for read */
    I2C_Send7bitAddress(I2C2, DeviceAddr, I2C_Direction_Receiver);

    /* wait on ADDR flag to be set */
    timeout = UI2C_FLAG_TIMEOUT;
    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_ADDR) == RESET)
    {
        if ((timeout --) == 0)
        {
            return (UI2C_ERR_TIMEOUT);
        }
    }

    /* Clear ADDR Flag */
    I2C_ReadRegister(I2C2, I2C_Register_SR1);
    I2C_ReadRegister(I2C2, I2C_Register_SR2);

    /* Disable Acknowledgement */
    I2C_AcknowledgeConfig(I2C2, DISABLE);

    /* Generate STOP Pluse */
    I2C_GenerateSTOP(I2C2, ENABLE);

    /* Waite for the byte to be received */
    timeout = UI2C_FLAG_TIMEOUT;
    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_RXNE) == RESET)
    {
        if ((timeout --) == 0)
        {
            return (UI2C_ERR_FAILED);
        }
    }

    /* Read the byte received from the EEPROM */
    *pBuffer = I2C_ReceiveData(I2C2);

    /* Wait to make sure the STOP control bit has been cleared */
    timeout = UI2C_FLAG_TIMEOUT;
    while (I2C2->CR1 & I2C_CR1_STOP)
    {
        if ((timeout --) == 0)
        {
            return (UI2C_ERR_FAILED);
        }
    }

    /* Re-Enable ACK to be ready for another reception */
    I2C_AcknowledgeConfig(I2C2, ENABLE);

    return(UI2C_ERR_NONE);
}

/**
 * @brief write a byte to the VL6180X
 * @param pBuffer-pointer to the byte to be write
 * @param WriteAddr-the address to be write
 * @retval perfomance result
*/
_I2C_ERR single_write(unsigned char DeviceAddr, unsigned short WriteAddr, unsigned char data)
{
    uint32_t timeout = 0;
    /* write MSB first, then LSB */
    uint8_t  Addr = 0;
    /* Wait EEPROM StandBy State */
		
    timeout = UI2C_LONG_TIMEOUT;
    do
    {
        I2C_GenerateSTART(I2C2, ENABLE);
        I2C_ReadRegister(I2C2, I2C_Register_SR1);
        I2C_Send7bitAddress(I2C2, DeviceAddr, I2C_Direction_Transmitter);
        if ((timeout --) == 0)
        {
            return (UI2C_ERR_TIMEOUT);
        }
    }
    while(!(I2C_ReadRegister(I2C2, I2C_Register_SR1) & 0x0002));

    I2C_ClearFlag(I2C2, I2C_FLAG_AF);
    I2C_GenerateSTOP(I2C2, ENABLE);

    timeout = UI2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY))
    {
        if ((timeout--) == 0)
        {
            return(UI2C_ERR_TIMEOUT);
        }
    }

    /* Send start condition */
    I2C_GenerateSTART(I2C2, ENABLE);

		
    timeout = UI2C_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
    {
        if ((timeout--) == 0)
        {
            return(UI2C_ERR_TIMEOUT);
        }
    }
		

    /* Send EEPROM address for write */
    I2C_Send7bitAddress(I2C2, DeviceAddr, I2C_Direction_Transmitter);

    timeout = 10 * UI2C_LONG_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
        if ((timeout--) == 0)
        {
            return (UI2C_ERR_TIMEOUT);
        }
    }

    Addr = (uint8_t)((WriteAddr & 0xFF00) >> 8);

    I2C_SendData(I2C2, Addr);

    timeout = UI2C_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if ((timeout --) == 0)
        {
            return (UI2C_ERR_TIMEOUT);
        }
    }

    Addr = (uint8_t) (WriteAddr & 0x00FF);

    I2C_SendData(I2C2, Addr);

    timeout = UI2C_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if ((timeout--) == 0)
        {
            return (UI2C_ERR_TIMEOUT);
        }
    }

    I2C_SendData(I2C2, (uint8_t)(data));

    /* Waite for the byte to be send */
    timeout = UI2C_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if ((timeout--) == 0)
        {
            return (UI2C_ERR_TIMEOUT);
        }
    }

    /* Generate STOP Pluse */
    I2C_GenerateSTOP(I2C2, ENABLE);
    /* Wait to make sure the STOP control bit has been cleared */
    timeout = UI2C_FLAG_TIMEOUT;
    while (I2C2->CR1 & I2C_CR1_STOP)
    {
        if ((timeout --) == 0)
        {
            return (UI2C_ERR_FAILED);
        }
    }

    /* Re-Enable ACK to be ready for another reception */
   // I2C_AcknowledgeConfig(I2C2, ENABLE);

    return(UI2C_ERR_NONE);
}

/**
 * @brief write 2 bytes to the VL6180X
 * @param pBuffer-pointer to the byte to be write
 * @param WriteAddr-the address to be write
 * @retval perfomance result
*/
_I2C_ERR double_write(unsigned char DeviceAddr, unsigned short WriteAddr, unsigned short data)
{
		uint32_t timeout = 0;
    /* write MSB first, then LSB */
    uint8_t  Addr = 0;
    /* Wait EEPROM StandBy State */
	
    timeout = UI2C_LONG_TIMEOUT;
    do
    {
        I2C_GenerateSTART(I2C2, ENABLE);
        I2C_ReadRegister(I2C2, I2C_Register_SR1);
        I2C_Send7bitAddress(I2C2, DeviceAddr, I2C_Direction_Transmitter);
        if ((timeout --) == 0)
        {
            return (UI2C_ERR_TIMEOUT);
        }
    }
    while(!(I2C_ReadRegister(I2C2, I2C_Register_SR1) & 0x0002));
	
    I2C_ClearFlag(I2C2, I2C_FLAG_AF);
    I2C_GenerateSTOP(I2C2, ENABLE);

    timeout = UI2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY))
    {
        if ((timeout--) == 0)
        {
            return(UI2C_ERR_TIMEOUT);
        }
    }

    /* Send start condition */
    I2C_GenerateSTART(I2C2, ENABLE);

    timeout = UI2C_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
    {
        if ((timeout--) == 0)
        {
            return(UI2C_ERR_TIMEOUT);
        }
    }

    /* Send EEPROM address for write */
    I2C_Send7bitAddress(I2C2, DeviceAddr, I2C_Direction_Transmitter);

    timeout = 10 * UI2C_LONG_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
        if ((timeout--) == 0)
        {
            return (UI2C_ERR_TIMEOUT);
        }
    }

    Addr = (uint8_t)((WriteAddr & 0xFF00) >> 8);

    I2C_SendData(I2C2, Addr);

    timeout = UI2C_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if ((timeout --) == 0)
        {
            return (UI2C_ERR_TIMEOUT);
        }
    }

    Addr = (uint8_t) (WriteAddr & 0x00FF);

    I2C_SendData(I2C2, Addr);

    timeout = UI2C_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if ((timeout--) == 0)
        {
            return (UI2C_ERR_TIMEOUT);
        }
    }

    I2C_SendData(I2C2, (uint8_t)((data>>8)&0xFF));

    /* Waite for the byte to be send */
    timeout = UI2C_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if ((timeout--) == 0)
        {
            return (UI2C_ERR_TIMEOUT);
        }
    }
		
		I2C_SendData(I2C2, (uint8_t)(data&0xFF));

    /* Waite for the byte to be send */
    timeout = UI2C_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if ((timeout--) == 0)
        {
            return (UI2C_ERR_TIMEOUT);
        }
    }
		
		/* Generate STOP Pluse */
    I2C_GenerateSTOP(I2C2, ENABLE);
    /* Wait to make sure the STOP control bit has been cleared */
    timeout = UI2C_FLAG_TIMEOUT;
    while (I2C2->CR1 & I2C_CR1_STOP)
    {
        if ((timeout --) == 0)
        {
            return (UI2C_ERR_FAILED);
        }
    }

    /* Re-Enable ACK to be ready for another reception */
   // I2C_AcknowledgeConfig(I2C2, ENABLE);

    return(UI2C_ERR_NONE);
}

/**
 * @brief write n bytes to the VL6180X
 * @param pBuffer-pointer to the byte to be write
 * @param WriteAddr-the address to be write
 * @retval perfomance result
*/
_I2C_ERR multi_write(unsigned char DeviceAddr, unsigned short WriteAddr, unsigned int data)
{
		uint32_t timeout = 0;
    /* write MSB first, then LSB */
    uint8_t  Addr = 0;
    /* Wait EEPROM StandBy State */
	
    timeout = UI2C_LONG_TIMEOUT;
    do
    {
        I2C_GenerateSTART(I2C2, ENABLE);
        I2C_ReadRegister(I2C2, I2C_Register_SR1);
        I2C_Send7bitAddress(I2C2, DeviceAddr, I2C_Direction_Transmitter);
        if ((timeout --) == 0)
        {
            return (UI2C_ERR_TIMEOUT);
        }
    }
    while(!(I2C_ReadRegister(I2C2, I2C_Register_SR1) & 0x0002));
	
    I2C_ClearFlag(I2C2, I2C_FLAG_AF);
    I2C_GenerateSTOP(I2C2, ENABLE);

    timeout = UI2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY))
    {
        if ((timeout--) == 0)
        {
            return(UI2C_ERR_TIMEOUT);
        }
    }

    /* Send start condition */
    I2C_GenerateSTART(I2C2, ENABLE);

    timeout = UI2C_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
    {
        if ((timeout--) == 0)
        {
            return(UI2C_ERR_TIMEOUT);
        }
    }

    /* Send EEPROM address for write */
    I2C_Send7bitAddress(I2C2, DeviceAddr, I2C_Direction_Transmitter);

    timeout = 10 * UI2C_LONG_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
        if ((timeout--) == 0)
        {
            return (UI2C_ERR_TIMEOUT);
        }
    }

    Addr = (uint8_t)((WriteAddr & 0xFF00) >> 8);

    I2C_SendData(I2C2, Addr);

    timeout = UI2C_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if ((timeout --) == 0)
        {
            return (UI2C_ERR_TIMEOUT);
        }
    }

    Addr = (uint8_t) (WriteAddr & 0x00FF);

    I2C_SendData(I2C2, Addr);

    timeout = UI2C_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if ((timeout--) == 0)
        {
            return (UI2C_ERR_TIMEOUT);
        }
    }

    I2C_SendData(I2C2, (uint8_t)((data>>24)&0xFF));

    /* Waite for the byte to be send */
    timeout = UI2C_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if ((timeout--) == 0)
        {
            return (UI2C_ERR_TIMEOUT);
        }
    }
		
		I2C_SendData(I2C2, (uint8_t)((data>>16)&0xFF));

    /* Waite for the byte to be send */
    timeout = UI2C_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if ((timeout--) == 0)
        {
            return (UI2C_ERR_TIMEOUT);
        }
    }
		
		I2C_SendData(I2C2, (uint8_t)((data>>8)&0xFF));

    /* Waite for the byte to be send */
    timeout = UI2C_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if ((timeout--) == 0)
        {
            return (UI2C_ERR_TIMEOUT);
        }
    }
		
		I2C_SendData(I2C2, (uint8_t)(data&0xFF));

    /* Waite for the byte to be send */
    timeout = UI2C_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if ((timeout--) == 0)
        {
            return (UI2C_ERR_TIMEOUT);
        }
    }
		
		/* Generate STOP Pluse */
    I2C_GenerateSTOP(I2C2, ENABLE);
    /* Wait to make sure the STOP control bit has been cleared */
    timeout = UI2C_FLAG_TIMEOUT;
    while (I2C2->CR1 & I2C_CR1_STOP)
    {
        if ((timeout --) == 0)
        {
            return (UI2C_ERR_FAILED);
        }
    }

    /* Re-Enable ACK to be ready for another reception */
   // I2C_AcknowledgeConfig(I2C2, ENABLE);

    return(UI2C_ERR_NONE);
}

/**
 * @brief write n bytes to the VL6180X
 * @param pBuffer-pointer to the byte to be write
 * @param WriteAddr-the address to be write
 * @retval perfomance result
*/
void VL6180X_LowLevel_Init(unsigned char DeviceAddr)
{
		single_write( DeviceAddr, 0x0207, 0x01);
    single_write( DeviceAddr, 0x0208, 0x01);
    single_write( DeviceAddr, 0x0096, 0x00);
    single_write( DeviceAddr, 0x0097, 0x54);
    single_write( DeviceAddr, 0x00e3, 0x00);
    single_write( DeviceAddr, 0x00e4, 0x04);
    single_write( DeviceAddr, 0x00e5, 0x02);
    single_write( DeviceAddr, 0x00e6, 0x01);
    single_write( DeviceAddr, 0x00e7, 0x03);
    single_write( DeviceAddr, 0x00f5, 0x02);
    single_write( DeviceAddr, 0x00d9, 0x05);

		multi_write(	DeviceAddr,0x00da,0x00CF03F7);

    single_write( DeviceAddr, 0x009f, 0x00);
    single_write( DeviceAddr, 0x00a3, 0x28);
    single_write( DeviceAddr, 0x00b7, 0x00);
    single_write( DeviceAddr, 0x00bb, 0x28);
    single_write( DeviceAddr, 0x00b2, 0x09);
    single_write( DeviceAddr, 0x00ca, 0x09);
    single_write( DeviceAddr, 0x0198, 0x01);
    single_write( DeviceAddr, 0x01b0, 0x17);
    single_write( DeviceAddr, 0x01ad, 0x00);
    single_write( DeviceAddr, 0x00ff, 0x05);
    single_write( DeviceAddr, 0x0100, 0x05);
    single_write( DeviceAddr, 0x0199, 0x05);
    single_write( DeviceAddr, 0x01a6, 0x1b);
    single_write( DeviceAddr, 0x01ac, 0x3e);
    single_write( DeviceAddr, 0x01a7, 0x1f);
    single_write( DeviceAddr, 0x0030, 0x00);
    single_write( DeviceAddr, 0x0011, 0x10);
    single_write( DeviceAddr, 0x010a, 0x30);
    single_write( DeviceAddr, 0x003f, 0x46);
    single_write( DeviceAddr, 0x0031, 0xFF);
    single_write( DeviceAddr, 0x0040, 0x63);
    single_write( DeviceAddr, 0x002e, 0x01);
    single_write( DeviceAddr, 0x002c, 0xff);
    single_write( DeviceAddr, 0x001b, 0x09);
    single_write( DeviceAddr, 0x003e, 0x31);
    single_write( DeviceAddr, 0x0014, 0x24);

    single_write(	DeviceAddr, 0x001C, 0x32);
    double_write(	DeviceAddr, 0x0022, 0x0060);
    double_write(	DeviceAddr, 0x0096, 0x00fd);
		single_write( DeviceAddr, 0x024, 0x08);
		
		single_write( DeviceAddr, 0x016, 0x00);


		single_write( DeviceAddr, 0x03f, 0x46);
		single_write( DeviceAddr, 0x019, 0xc8);
		single_write( DeviceAddr, 0x01A, 0x0a);
    double_write( DeviceAddr, 0x0040, 0x0063);
    single_write( DeviceAddr, 0x003e, 0x14);
    single_write( DeviceAddr, 0x003f, 0x40);
    single_write( DeviceAddr, 0x003c, 0x00);
    single_write( DeviceAddr, 0x003A, 0xff);
		single_write( DeviceAddr, 0x015, 0x07);
		single_write( DeviceAddr, 0x01AC, 0x3c);
		single_write( DeviceAddr, 0x0f2, 0x05);
/***********改变量程*******************/

//    double_write(	DeviceAddr, 0x0096, 0x0054);
//		single_write( DeviceAddr, 0x024, 0x02);
/**************************************/
}

#if 0
void Vl6180X_Init(void)
{
	VL6180X_CE1_OPEN;
	VL6180X_CE2_CLOSE;
	VL6180X_CE3_CLOSE;
	VL6180X_CE4_CLOSE;
	VL6180X_CE5_CLOSE;
	VL6180X_CE6_CLOSE;
	
	OSTimeDly(1);
	single_write(0x52, 0x0212, SLAVE_ADDRESS1);
	OSTimeDly(2);
	
	VL6180X_CE2_OPEN;
	OSTimeDly(1);
	single_write(0x52, 0x0212, SLAVE_ADDRESS2);
	OSTimeDly(2);
	
	VL6180X_CE3_OPEN;
	OSTimeDly(1);
	single_write(0x52, 0x0212, SLAVE_ADDRESS3);
	OSTimeDly(2);
	
	VL6180X_CE4_OPEN;
	OSTimeDly(1);
	single_write(0x52, 0x0212, SLAVE_ADDRESS4);
	OSTimeDly(2);
	
	VL6180X_CE5_OPEN;
	OSTimeDly(1);
	single_write(0x52, 0x0212, SLAVE_ADDRESS5);
	OSTimeDly(2);
	
	VL6180X_LowLevel_Init(VL6180X_ADDRESS1);
	OSTimeDly(2);
	VL6180X_LowLevel_Init(VL6180X_ADDRESS2);
	OSTimeDly(2);
	VL6180X_LowLevel_Init(VL6180X_ADDRESS3);
	OSTimeDly(2);
	VL6180X_LowLevel_Init(VL6180X_ADDRESS4);
	OSTimeDly(2);
	VL6180X_LowLevel_Init(VL6180X_ADDRESS5);
	OSTimeDly(2);
}
#endif


/*------------------The End of File----------------------------------------------------*/
