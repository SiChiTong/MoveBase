/*=============================================================
@brief      STM32通用IO口模拟I2C底层驱动
@author     ZGJ
@date           2015-10-16
@statement  All rights reserved
==============================================================*/
/*---------------------Include files------------------------*/
#include "i2cio.h"
#include "stm32f10x.h"

/*---------------------Private Fuctions---------------------*/

void I2C_delay(void)
{
  unsigned int i, j;
  unsigned int sum = 0;
  i = 20;
  while(i--)
  {
     for (j = 0; j < 10; j++)
     sum += i;
  }
  sum = i;
}
void I2C_GPIO_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin =  I2C_SCL | I2C_SDA;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIO_I2C, &GPIO_InitStructure);
    I2C_SCL_H;
    I2C_SDA_H;
}

void I2C_SDA_Out(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin =  I2C_SDA;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIO_I2C, &GPIO_InitStructure);
}

void I2C_SDA_In(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin =  I2C_SDA;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIO_I2C, &GPIO_InitStructure);
}

//产生起始信号
void I2C_Start(void)
{
    I2C_SDA_Out();
    I2C_SDA_H;
    I2C_SCL_H;
    I2C_delay();
    I2C_SDA_L;
    I2C_delay();
    I2C_SCL_L;
}

//产生停止信号
void I2C_Stop(void)
{
    I2C_SDA_Out();
    I2C_SCL_L;
    I2C_SDA_L;
    I2C_SCL_H;
    I2C_delay();
    I2C_SDA_H;
    I2C_delay();
}

//主机产生应答信号ACK
void I2C_Ack(void)
{
    I2C_SDA_Out();
    I2C_SCL_L;
    I2C_SDA_L;
    I2C_delay();
    I2C_SCL_H;
    I2C_delay();
    I2C_SCL_L;
}
//主机不产生应答信号NACK
void I2C_NAck(void)
{
    I2C_SDA_Out();
    I2C_SCL_L;
    I2C_SDA_H;
    I2C_delay();
    I2C_SCL_H;
    I2C_delay();
    I2C_SCL_L;
}

//等待从机应答信号
//返回值：0 接收应答失败
//        1 接收应答成功
unsigned char I2C_Wait_Ack(void)
{
    unsigned char temptime=0;
    I2C_SDA_H;
    I2C_SDA_In();
    I2C_delay();
    I2C_SCL_H;
    I2C_delay();
    while(GPIO_ReadInputDataBit(GPIO_I2C,I2C_SDA))
    {
        temptime++;
        if(temptime>250)
        {
            I2C_Stop();
            return 0;
        }
    }
    I2C_SCL_L;
    return 1;
}

//发送一个字节
void I2C_Send_Byte(unsigned char txd)
{
    unsigned char i=0;
    I2C_SDA_Out();
    I2C_SCL_L;  //拉低时钟开始传输数据,只在时钟拉低时SDA数据可变
    for(i=0;i<8;i++)
    {
        if((txd & 0x80)>0)
        {
            I2C_SDA_H;
        }
        else
        {
            I2C_SDA_L;
        }
        txd <<= 1;
        I2C_SCL_H;
        I2C_delay();
        I2C_SCL_L;
        I2C_delay();
    }
}

//读取一个字节
unsigned char I2C_Read_Byte(unsigned char ack)
{
    unsigned char i=0;
    unsigned char receive=0;
    I2C_SDA_In();
    for(i=0;i<8;i++)
    {
        I2C_SCL_L;
        I2C_delay();
        I2C_SCL_H;
        receive<<=1;
        if(GPIO_ReadInputDataBit(GPIO_I2C,I2C_SDA))
        {
            receive++;
        }
        I2C_delay();
    }
    if(ack==0)
    {
        I2C_NAck();
    }
    else
    {
        I2C_Ack();
    }
    return receive;
}

//向从机写入一个字节
Bool Single_Write(unsigned char SlaveAddress,unsigned short addr,unsigned char data)
{
    I2C_Start();
    I2C_Send_Byte(SlaveAddress);
    if(!I2C_Wait_Ack()){
		I2C_Stop();
		return bool_false;
		}
    I2C_Send_Byte((addr>>8)&0xFF);
    I2C_Wait_Ack();
		I2C_Send_Byte((addr&0xFF));
    I2C_Wait_Ack();
    I2C_Send_Byte(data);
    I2C_Wait_Ack();
    I2C_Stop();
    I2C_delay();
    return bool_true;
}

//从从机读出一个字节
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char addr)
{
    unsigned char rxd=0;
    I2C_Start();
    I2C_Send_Byte(SlaveAddress);
    if(!I2C_Wait_Ack()){I2C_Stop(); return bool_false;}
    I2C_Send_Byte(addr);
    I2C_Wait_Ack();
    I2C_Start();
    I2C_Send_Byte(SlaveAddress+1);       //SlaveAddress+1  中的1表示读操作
    I2C_Wait_Ack();

    rxd=I2C_Read_Byte(0);
    I2C_NAck();
    I2C_Stop();
    return rxd;
}

//可以读取多字节
Bool I2C_Read(unsigned char SlaveAddress,unsigned char addr,unsigned char *buf,unsigned short num)
{
    if(num==0)
    return bool_false;
    while(num)
    {
        *buf=Single_Read(SlaveAddress,addr);
        if(num != 1)
        {
            buf ++;
            addr ++;
        }
        num--;
    }
    return bool_true;
}
//写入多字节
Bool I2C_Write(unsigned char SlaveAddress,unsigned char addr,unsigned char *buf,unsigned short num)
{
    if(num==0)
    return bool_false;
    while(num)
    {
        if(Single_Write(SlaveAddress,addr,*buf))
        {
            if(num != 1)
            {
                buf ++;
                addr ++;
            }
        }
        else
        {
            return bool_false;
        }
    }
    return bool_true;
}
/*======================End of file===========================*/

