/*=============================================================
@brief      STM32ͨ��IO��ģ��I2C�ײ�����
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

//������ʼ�ź�
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

//����ֹͣ�ź�
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

//��������Ӧ���ź�ACK
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
//����������Ӧ���ź�NACK
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

//�ȴ��ӻ�Ӧ���ź�
//����ֵ��0 ����Ӧ��ʧ��
//        1 ����Ӧ��ɹ�
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

//����һ���ֽ�
void I2C_Send_Byte(unsigned char txd)
{
    unsigned char i=0;
    I2C_SDA_Out();
    I2C_SCL_L;  //����ʱ�ӿ�ʼ��������,ֻ��ʱ������ʱSDA���ݿɱ�
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

//��ȡһ���ֽ�
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

//��ӻ�д��һ���ֽ�
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

//�Ӵӻ�����һ���ֽ�
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char addr)
{
    unsigned char rxd=0;
    I2C_Start();
    I2C_Send_Byte(SlaveAddress);
    if(!I2C_Wait_Ack()){I2C_Stop(); return bool_false;}
    I2C_Send_Byte(addr);
    I2C_Wait_Ack();
    I2C_Start();
    I2C_Send_Byte(SlaveAddress+1);       //SlaveAddress+1  �е�1��ʾ������
    I2C_Wait_Ack();

    rxd=I2C_Read_Byte(0);
    I2C_NAck();
    I2C_Stop();
    return rxd;
}

//���Զ�ȡ���ֽ�
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
//д����ֽ�
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

