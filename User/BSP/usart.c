/**-------------------------------------------------------------------------
*@ file				usart.c
*@ brief			USART driver,communicate with PC
*@ author 		ZGJ
*@ vertion		V0.01
*@ date				2015-12-15
*@ statement	(C)	COPYRIGHT 2015 KSITRI.
--------------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "usart.h"
#include "ucos_ii.h"
#include "modbus.h"
#include <stdio.h>
#include "common.h"
#include "task_cfg.h"
#include "loadmotor.h"


#pragma import(__use_no_semihosting)

_sys_exit(int x)

{
x = x;
}

struct __FILE
{
int handle;
};

FILE __stdout;

/* Define -------------------------------------------------------*/

/* Variables ------------------------------------------------------*/
unsigned char Usart2SendBuffer[PROTOCOL_SEND_SIZE]={0};
unsigned char Usart2RevBuffer[PROTOCOL_RECIVE_SIZE]={0};
//unsigned char Uart4RevBuffer[64]={0};
_CMD_FEEDBACK	cmdfeed;

OS_MEM	*USART2RevList;
USART2RevBuf_t	USART2RevPart[11][1];
USART2RevBuf_t *usart2revbufhead=NULL;



unsigned char Usart1SendBuffer[32]={0x80,0x00,0x80};
//unsigned char Usart1RevBuffer[32]={0x01,0x02,0x03,0x04,0x05,0x06};

unsigned char Usart3SendBuffer[32]={0x80,0x00,0x80};
//unsigned char Usart3RevBuffer[32]={0x01,0x02,0x03,0x04,0x05,0x06};

unsigned int revcnt=0;
unsigned int procnt=0;

DRIVERDATA_UNION Usart1Data;
DRIVERDATA_UNION Usart3Data;

DRIVERSTATE_STRUCT Usart1State;
DRIVERSTATE_STRUCT Usart3State;

USARTCMD_STRUCT UsartCMDMbox;

/* functions ------------------------------------------------------*/
void USART_LowLevel_Init(void)
{
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    GPIO_InitTypeDef	GPIO_InitStructure;

    //IO configuration
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    //USART2-TX-PA2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    //USART2-RX-PA3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//主通信接口USART2
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART2, &USART_InitStructure);
    //允许串口总线空闲中断，IDLE的门限值是一个字节的传输时间，对于38400波特率，门限值约为208.3us
    //因此，给驱动板发送指令，两个字节之间的时间间隔不能超过门限值。
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);  //
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
 //   USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE);
				
	//DMA configuration,  COM1-TX
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);    // 外设地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&Usart2SendBuffer[0];    // 内存地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                  // 内存-->外设
    DMA_InitStructure.DMA_BufferSize = PROTOCOL_SEND_SIZE;             // 缓存大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;    // 外设地址不增加
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;             // 内存地址增加
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 数据宽度8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; // 内存数据宽度
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                     // 循环缓存模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                 // DMA优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                        // MEM到MEM传输禁止
    DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);                     // 允许DMA传输完中断
    DMA_Init(DMA1_Channel7, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel7, DISABLE);
		
	//COM1-RX
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);    // 外设地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&Usart2RevBuffer[0];    // 内存地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                  // 内存-->外设
    DMA_InitStructure.DMA_BufferSize = PROTOCOL_RECIVE_SIZE;             // 缓存大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;    // 外设地址不增加
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;             // 内存地址增加
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 数据宽度8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; // 内存数据宽度
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                     // 循环缓存模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // DMA优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                        // MEM到MEM传输禁止
   // DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);                     // 允许DMA传输完中断
    DMA_Init(DMA1_Channel6, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel6, ENABLE);

}


void USART1_Config(int baud)
{
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    GPIO_InitTypeDef	GPIO_InitStructure;
    //	    NVIC_InitTypeDef NVIC_InitStructure;
    //优先级组设置，4位用于抢占优先级，0位用于响应优先级,此时
    //NVIC_IRQChannelSubPriority的值无效
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,	ENABLE);			//--com0
    //	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    //USART1-RX-PA10, USART1-TX-PA9  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    //COM1-TX
    DMA_DeInit(DMA1_Channel4);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);    // 外设地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&(Usart1SendBuffer[0]);    // 内存地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                  // 内存-->外设
    DMA_InitStructure.DMA_BufferSize = 8;             // 缓存大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;    // 外设地址不增加
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;             // 内存地址增加
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 数据宽度8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; // 内存数据宽度
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                     // 循环缓存模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                 // DMA优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                        // MEM到MEM传输禁止
    //    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);                     // 允许DMA传输完中断
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel4, DISABLE);
    //*************************UART1-DMA1_channel5接收*********************//
    //RX
    DMA_DeInit(DMA1_Channel5);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);    // 外设地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&(Usart1Data.UsartRevData[0]);    // 内存地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                  // 内存<--外设
    DMA_InitStructure.DMA_BufferSize = 32;             // 缓存大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;    // 外设地址不增加
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;             // 内存地址增加
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 数据宽度8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; // 内存数据宽度
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                     // 循环缓存模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                 // DMA优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                        // MEM到MEM传输禁止
    //    DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);                     // 允许DMA传输完中断
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel5, DISABLE);

    //USART1---COM0
    USART_InitStructure.USART_BaudRate = baud;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;		
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);  //
    USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
    USART_Cmd(USART1, ENABLE);
}


//驱动器通信接口
void USART3_Config(int baud)
{
		USART_InitTypeDef USART_InitStructure;
		DMA_InitTypeDef DMA_InitStructure;
		GPIO_InitTypeDef	GPIO_InitStructure;
//	  NVIC_InitTypeDef NVIC_InitStructure;
	  
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,	ENABLE);			//--com0
//	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		
		//USART3-RX-PB10, USART1-TX-PB11  
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		//COM1-TX
		DMA_DeInit(DMA1_Channel2);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);    // 外设地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&(Usart3SendBuffer[0]);    // 内存地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                  // 内存-->外设
    DMA_InitStructure.DMA_BufferSize = 8;             // 缓存大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;    // 外设地址不增加
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;             // 内存地址增加
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 数据宽度8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; // 内存数据宽度
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                     // 循环缓存模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                 // DMA优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                        // MEM到MEM传输禁止
 //   DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);                     // 允许DMA传输完中断
    DMA_Init(DMA1_Channel2, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel2, DISABLE);
		//*************************UART1-DMA1_channel5接收*********************//
		//RX
		DMA_DeInit(DMA1_Channel3);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);    // 外设地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&(Usart3Data.UsartRevData[0]);    // 内存地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                  // 内存<--外设
    DMA_InitStructure.DMA_BufferSize = 32;             // 缓存大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;    // 外设地址不增加
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;             // 内存地址增加
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 数据宽度8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; // 内存数据宽度
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                     // 循环缓存模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                 // DMA优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                        // MEM到MEM传输禁止
//    DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);                     // 允许DMA传输完中断
    DMA_Init(DMA1_Channel3, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel3, DISABLE);
		
		//USART3
		USART_InitStructure.USART_BaudRate = baud;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;		
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);
		USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);  //
		USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
    USART_Cmd(USART3, ENABLE);
}




void USART2_SendEnable(unsigned char NumToSend)
{
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    DMA1_Channel7->CNDTR = NumToSend;
    DMA_Cmd(DMA1_Channel7, ENABLE);
}

/*****************************
说明：串口字符串发送函数
入口参数：u8 *str
出口参数：无
******************************/
void USART_SendStr(USART_TypeDef *USARTx, uint8_t *str)
{
    while(*str!='\0')
    {
        USART_SendData(USARTx,*str);
        while(USART_GetFlagStatus(USARTx,USART_FLAG_TXE)!= SET);
        str++;
    }
}


#if NEW_FEEDBACK
void Cmd_Send(_CMD_FEEDBACK *pcmd)
{
	unsigned char i=0;
	unsigned char sum=0;
	Usart2SendBuffer[0]=0x55;
	Usart2SendBuffer[1]=pcmd->dx.fc[3];
	Usart2SendBuffer[2]=pcmd->dx.fc[2];
	Usart2SendBuffer[3]=pcmd->dx.fc[1];
	Usart2SendBuffer[4]=pcmd->dx.fc[0];
	
	Usart2SendBuffer[5]=pcmd->dy.fc[3];
	Usart2SendBuffer[6]=pcmd->dy.fc[2];
	Usart2SendBuffer[7]=pcmd->dy.fc[1];
	Usart2SendBuffer[8]=pcmd->dy.fc[0];
	
	Usart2SendBuffer[9]=pcmd->dthta.fc[3];
	Usart2SendBuffer[10]=pcmd->dthta.fc[2];
	Usart2SendBuffer[11]=pcmd->dthta.fc[1];
	Usart2SendBuffer[12]=pcmd->dthta.fc[0];
	
	Usart2SendBuffer[13]=(pcmd->vx>>8)&0xFF;
	Usart2SendBuffer[14]=pcmd->vx&0xFF;
	Usart2SendBuffer[15]=(pcmd->vthta>>8)&0xFF;
	Usart2SendBuffer[16]=pcmd->vthta&0xFF;
	

	//中菱驱动器底盘状态
	if(BoardID==0)
	{
		Usart2SendBuffer[17]=pcmd->LEFT_DRIVER.MOTOR_DRIVER_STATE;
		Usart2SendBuffer[18]=pcmd->RIGHT_DRIVER.MOTOR_DRIVER_STATE;
	}
	
	Usart2SendBuffer[19]=pcmd->WORK_STATE.ROBOT_STATE;
	Usart2SendBuffer[20]=pcmd->sensor_sta;
	
	Usart2SendBuffer[21]=pcmd->lightsensordata[2];
	Usart2SendBuffer[22]=pcmd->lightsensordata[1];
	Usart2SendBuffer[23]=pcmd->lightsensordata[0];
	Usart2SendBuffer[24]=0x00;;
	Usart2SendBuffer[25]=pcmd->battery;
	Usart2SendBuffer[26]=pcmd->comu;
	
	for(i=0;i<27;i++)
	{
		sum+=Usart2SendBuffer[i];
	}
	Usart2SendBuffer[27]=sum;
	Usart2SendBuffer[28]=0xAA;
	
	USART2_SendEnable(29);	
}
#endif



void Usart_SendBuffer(USART_TypeDef *USARTx,unsigned char *pBuf, unsigned short len)
{
	unsigned short i=0;
	for(i=0;i<len;i++)
	{
		USART_SendData(USARTx,*pBuf++);
		while(USART_GetFlagStatus(USARTx,USART_FLAG_TXE)==RESET);
	}
	while(USART_GetFlagStatus(USARTx,USART_FLAG_TC)==RESET);
	Delayus(1);
}
void USART2RevHeadInit(void)
{
		unsigned char err;
		unsigned char	i=0;
    while (usart2revbufhead == NULL)
    {
        usart2revbufhead = (USART2RevBuf_t *)OSMemGet(USART2RevList, &err);
    }
    for(i=0;i<PROTOCOL_RECIVE_SIZE;i++)
		{
			usart2revbufhead->RevBuf[i]=0;
		}
		usart2revbufhead->length=0;
    usart2revbufhead->next = NULL;
}

unsigned char GetUsart2RevListLen(void)
{
	USART2RevBuf_t	*ptr=NULL;
	unsigned char i=0;
	ptr=usart2revbufhead->next;
	while(ptr!=NULL)
	{
		ptr=ptr->next;
		i++;
	}
	return i;
}

void Usart2AddRevBuf(unsigned char *buf,unsigned char len)
{
	unsigned char err=0;
	USART2RevBuf_t *ptr=NULL;
	USART2RevBuf_t *node=NULL;
	unsigned char i=0;
	if(GetUsart2RevListLen()<10)
	{
		ptr=OSMemGet(USART2RevList,&err);
		if(ptr!=NULL)
		{
			ptr->length=len;
			for(i=0;i<len;i++)
			{
				ptr->RevBuf[i]=*(buf+i);
			}
			ptr->next=NULL;
			node=usart2revbufhead;
			while(node->next!=NULL)
			{
				node=node->next;
			}
			node->next=ptr;
		}
	}
}
//命令下发
void SendCMD(USART_TypeDef *USARTx,DRIVERADD_ENUM add,char hvalue,char lvalue)
{
//	char i=0;
	volatile short sum=0;
	char t_add,t_hvalue,t_lvalue;
	t_add=add;
	t_hvalue=hvalue;
	t_lvalue=lvalue;
	sum=add+t_hvalue+t_lvalue;

	
	if(USARTx==USART1)
	{
		Usart1State.port=1;
		Usart1State.sendflag++;
		//读取状态
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		if(add==GETSTATE)
		{
				Usart1SendBuffer[0]=GETSTATE;
				Usart1SendBuffer[1]=0x00;
				Usart1SendBuffer[2]=Usart1SendBuffer[0]+Usart1SendBuffer[1];
					
				DMA1_Channel5->CCR &= (uint16_t)(~DMA_CCR1_EN);//disable
				DMA1_Channel5->CNDTR = 32;
				DMA1_Channel5->CCR |= DMA_CCR1_EN;//enable
			
				DMA1_Channel4->CCR &= (uint16_t)(~DMA_CCR1_EN);//disable
				DMA1_Channel4->CNDTR = 3;
				DMA1_Channel4->CCR |= DMA_CCR1_EN;//enable
			
			 OSSemPost(Usart1Cal);
			
		}
		else
		{
				Usart1SendBuffer[0]=t_add;
				Usart1SendBuffer[1]=t_hvalue;
				Usart1SendBuffer[2]=t_lvalue;
				Usart1SendBuffer[3]=Usart1SendBuffer[0]+Usart1SendBuffer[1]+Usart1SendBuffer[2];
			
				DMA1_Channel5->CCR &= (uint16_t)(~DMA_CCR1_EN);//disable
				DMA1_Channel5->CNDTR = 2;
				DMA1_Channel5->CCR |= DMA_CCR1_EN;//enable
				
				DMA1_Channel4->CCR &= (uint16_t)(~DMA_CCR1_EN);//disable
				DMA1_Channel4->CNDTR = 4;
				DMA1_Channel4->CCR |= DMA_CCR1_EN;//enable
		}
		
		if(Usart1State.sendflag >= 50)
		{
			Usart1State.sendflag = 50;
			cmdfeed.LEFT_DRIVER.MOTOR_DRIVER_STRUCT.connected=1;
		}
		else
		{
			cmdfeed.LEFT_DRIVER.MOTOR_DRIVER_STRUCT.connected=0;
		}
	}
	
	  if(USARTx==USART3)
		{
			Usart3State.port=3;
		  Usart3State.sendflag++;
			while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
			//读取状态
			if(add==GETSTATE)
			{
					Usart3SendBuffer[0]=GETSTATE;
					Usart3SendBuffer[1]=0x00;
					Usart3SendBuffer[2]=Usart3SendBuffer[0]+Usart3SendBuffer[1];
				
					DMA1_Channel3->CCR &= (uint16_t)(~DMA_CCR1_EN);//disable
					DMA1_Channel3->CNDTR = 32;
					DMA1_Channel3->CCR |= DMA_CCR1_EN;//enable		
				
					DMA1_Channel2->CCR &= (uint16_t)(~DMA_CCR1_EN);//disable
					DMA1_Channel2->CNDTR = 3;
					DMA1_Channel2->CCR |= DMA_CCR1_EN;//enable
				
				 OSSemPost(Usart3Cal);
				

			}
			else
			{
					Usart3SendBuffer[0]=t_add;
					Usart3SendBuffer[1]=t_hvalue;
					Usart3SendBuffer[2]=t_lvalue;
					Usart3SendBuffer[3]=Usart3SendBuffer[0]+Usart3SendBuffer[1]+Usart3SendBuffer[2];

					DMA1_Channel3->CCR &= (uint16_t)(~DMA_CCR1_EN);//disable
					DMA1_Channel3->CNDTR = 2;
					DMA1_Channel3->CCR |= DMA_CCR1_EN;//enable
				
					DMA1_Channel2->CCR &= (uint16_t)(~DMA_CCR1_EN);//disable
					DMA1_Channel2->CNDTR = 4;
					DMA1_Channel2->CCR |= DMA_CCR1_EN;//enable
			}
			
			if(Usart3State.sendflag >= 50)
			{
				Usart3State.sendflag= 50;
				cmdfeed.RIGHT_DRIVER.MOTOR_DRIVER_STRUCT.connected=1;
			}
						
			else
			{
				cmdfeed.RIGHT_DRIVER.MOTOR_DRIVER_STRUCT.connected=0;
			}
		}
		
		if(cmdfeed.RIGHT_DRIVER.MOTOR_DRIVER_STRUCT.connected||cmdfeed.LEFT_DRIVER.MOTOR_DRIVER_STRUCT.connected)
		{
			OSQPost(EHandlerQBOX,(void*)DRIVER_COMMU_ERR);
		}
}


//void HLSSendCMD(USART_TypeDef *USARTx,uint8_t mode,uint16_t add,uint16_t len)
//{
//	uint8_t TxBuff[64] = {0};
//	uint16_t dataLen =len ;
//	uint8_t i = 0;
//	uint32_t crc;
//	TxBuff[0] = 0x01;i++;
//	TxBuff[1] = mode;i++;
//	TxBuff[2] = (add>>8);i++;
//	TxBuff[3] = add;i++;
//	TxBuff[4] = (dataLen>>8);i++;
//	TxBuff[5] = dataLen;i++;
//	crc = crc_chk(TxBuff, 6);
//	TxBuff[6]  = crc;i++;
//	TxBuff[7]  = crc>>8;
//	
//	
//	
//	if(USARTx==USART1)
//	{
//		memcpy(Usart1SendBuffer,TxBuff,8);
//		Usart1State.port=1;
//		Usart1State.sendflag++;
//		//读取状态
//		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
//			
//		DMA1_Channel5->CCR &= (uint16_t)(~DMA_CCR1_EN);//disable
//		DMA1_Channel5->CNDTR = dataLen*2 + 5;
//		DMA1_Channel5->CCR |= DMA_CCR1_EN;//enable

//		DMA1_Channel4->CCR &= (uint16_t)(~DMA_CCR1_EN);//disable
//		DMA1_Channel4->CNDTR = 8;
//		DMA1_Channel4->CCR |= DMA_CCR1_EN;//enable
//		OSSemPost(Usart1Cal);
//		
//		if(Usart1State.sendflag >= 50)
//		{
//			Usart1State.sendflag = 50;
//			cmdfeed.LEFT_DRIVER.MOTOR_DRIVER_STRUCT.connected=1;
//		}
//		else
//		{
//			cmdfeed.LEFT_DRIVER.MOTOR_DRIVER_STRUCT.connected=0;
//		}
//	}
//	
//	if(USARTx==USART3)
//		{
//			memcpy(Usart3SendBuffer,TxBuff,8);
//			Usart3State.port=3;
//		  Usart3State.sendflag++;
//			while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
//			//读取状态

//			
//			DMA1_Channel3->CCR &= (uint16_t)(~DMA_CCR1_EN);//disable
//			DMA1_Channel3->CNDTR = dataLen*2 + 5;;
//			DMA1_Channel3->CCR |= DMA_CCR1_EN;//enable		

//			DMA1_Channel2->CCR &= (uint16_t)(~DMA_CCR1_EN);//disable
//			DMA1_Channel2->CNDTR = 8;
//			DMA1_Channel2->CCR |= DMA_CCR1_EN;//enable

//			OSSemPost(Usart3Cal);
//							
//			if(Usart3State.sendflag >= 50)
//			{
//				Usart3State.sendflag= 50;
//				cmdfeed.RIGHT_DRIVER.MOTOR_DRIVER_STRUCT.connected=1;
//			}
//						
//			else
//			{
//				cmdfeed.RIGHT_DRIVER.MOTOR_DRIVER_STRUCT.connected=0;
//			}
//		}
//		
//		if(cmdfeed.RIGHT_DRIVER.MOTOR_DRIVER_STRUCT.connected||cmdfeed.LEFT_DRIVER.MOTOR_DRIVER_STRUCT.connected)
//		{
//			OSQPost(EHandlerQBOX,(void*)DRIVER_COMMU_ERR);
//		}
//}



void HLSSendCMD(USART_TypeDef *USARTx,uint8_t mode,uint16_t add,uint16_t nums,uint16_t *data)
{
	uint8_t TxBuff[64] = {0};
	uint16_t dataLen =nums*2 ;
	uint8_t i = 0;
	uint32_t crc;
	uint16_t txLen = 0;
	uint16_t rxLen = 0;
	
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
		    rxLen = dataLen + 5;		

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
		    rxLen = 8;
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
		    txLen = i+dataLen;
		    rxLen = 8;
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
					rxLen = dataLen + 6;
				}
			break;
		default :
			break;

	}
	
	
	if(USARTx==USART1)
	{
		memcpy(Usart1SendBuffer,TxBuff,txLen);
		Usart1State.port=1;
		Usart1State.sendflag++;
		//读取状态
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
			
		DMA1_Channel5->CCR &= (uint16_t)(~DMA_CCR1_EN);//disable
		DMA1_Channel5->CNDTR = rxLen;
		DMA1_Channel5->CCR |= DMA_CCR1_EN;//enable

		DMA1_Channel4->CCR &= (uint16_t)(~DMA_CCR1_EN);//disable
		DMA1_Channel4->CNDTR = txLen;
		DMA1_Channel4->CCR |= DMA_CCR1_EN;//enable
		OSSemPost(Usart1Cal);
		
		if(Usart1State.sendflag >= 50)
		{
			Usart1State.sendflag = 50;
			//cmdfeed.LEFT_DRIVER.MOTOR_DRIVER_STRUCT.connected=1;
			cmdfeed.leftDriver |= WRR_CONNECT;
		}
		else
		{
			//cmdfeed.LEFT_DRIVER.MOTOR_DRIVER_STRUCT.connected=0;
			cmdfeed.leftDriver &= (~WRR_CONNECT);
		}

	}
	
		if(USARTx==USART3)
		{
			memcpy(Usart3SendBuffer,TxBuff,txLen);
			Usart3State.port=3;
		  Usart3State.sendflag++;
			while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
			//读取状态

			
			DMA1_Channel3->CCR &= (uint16_t)(~DMA_CCR1_EN);//disable
			DMA1_Channel3->CNDTR = rxLen;
			DMA1_Channel3->CCR |= DMA_CCR1_EN;//enable		

			DMA1_Channel2->CCR &= (uint16_t)(~DMA_CCR1_EN);//disable
			DMA1_Channel2->CNDTR = txLen;
			DMA1_Channel2->CCR |= DMA_CCR1_EN;//enable

			OSSemPost(Usart3Cal);
							
			if(Usart3State.sendflag >= 50)
			{
				Usart3State.sendflag= 50;
				//cmdfeed.RIGHT_DRIVER.MOTOR_DRIVER_STRUCT.connected=1;
				cmdfeed.rightDriver |= WRR_CONNECT;
			}
						
			else
			{
				//cmdfeed.RIGHT_DRIVER.MOTOR_DRIVER_STRUCT.connected=0;
				cmdfeed.rightDriver  &= (~WRR_CONNECT);
			}
		}
		
		if((cmdfeed.leftDriver&WRR_CONNECT)||(cmdfeed.rightDriver&WRR_CONNECT))
		{
			OSQPost(EHandlerQBOX,(void*)DRIVER_COMMU_ERR);
		}
		

}


void HLSGetStates(uint8_t mode,uint16_t add,uint16_t nums,uint16_t *data)
{
	uint8_t TxBuff[64] = {0};
	uint16_t dataLen =nums*2 ;
	uint8_t i = 0;
	uint32_t crc;
	uint16_t txLen = 0;
	uint16_t rxLen = 0;
	
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
		    rxLen = dataLen + 5;		

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
		    rxLen = 8;
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
		    txLen = i+dataLen;
		    rxLen = 8;
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
					rxLen = dataLen + 6;
				}
			break;
		default :
			break;

	}
	
	
//	if(USARTx==USART1)
	{
		memcpy(Usart1SendBuffer,TxBuff,txLen);
		Usart1State.port=1;
		Usart1State.sendflag++;
		//读取状态
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
			
		DMA1_Channel5->CCR &= (uint16_t)(~DMA_CCR1_EN);//disable
		DMA1_Channel5->CNDTR = rxLen;
		DMA1_Channel5->CCR |= DMA_CCR1_EN;//enable

		DMA1_Channel4->CCR &= (uint16_t)(~DMA_CCR1_EN);//disable
		DMA1_Channel4->CNDTR = txLen;
		DMA1_Channel4->CCR |= DMA_CCR1_EN;//enable
		OSSemPost(Usart1Cal);
		
		if(Usart1State.sendflag >= 50)
		{
			Usart1State.sendflag = 50;
			//cmdfeed.LEFT_DRIVER.MOTOR_DRIVER_STRUCT.connected=1;
			cmdfeed.leftDriver |= WRR_CONNECT;
		}
		else
		{
			//cmdfeed.LEFT_DRIVER.MOTOR_DRIVER_STRUCT.connected=0;
			cmdfeed.leftDriver &= (~WRR_CONNECT);
		}
	}
	
//	if(USARTx==USART3)
		{
			memcpy(Usart3SendBuffer,TxBuff,txLen);
			Usart3State.port=3;
		    Usart3State.sendflag++;
			while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
			//读取状态

			
			DMA1_Channel3->CCR &= (uint16_t)(~DMA_CCR1_EN);//disable
			DMA1_Channel3->CNDTR = rxLen;
			DMA1_Channel3->CCR |= DMA_CCR1_EN;//enable		

			DMA1_Channel2->CCR &= (uint16_t)(~DMA_CCR1_EN);//disable
			DMA1_Channel2->CNDTR = txLen;
			DMA1_Channel2->CCR |= DMA_CCR1_EN;//enable

			OSSemPost(Usart3Cal);
							
			if(Usart3State.sendflag >= 50)
			{
				Usart3State.sendflag= 50;
				//cmdfeed.RIGHT_DRIVER.MOTOR_DRIVER_STRUCT.connected=1;
				cmdfeed.rightDriver |= WRR_CONNECT;
			}
						
			else
			{
				//cmdfeed.RIGHT_DRIVER.MOTOR_DRIVER_STRUCT.connected=0;
				cmdfeed.rightDriver  &= (~WRR_CONNECT);
			}
		}
		
		if((cmdfeed.leftDriver&WRR_CONNECT)||(cmdfeed.rightDriver&WRR_CONNECT))
		{
			OSQPost(EHandlerQBOX,(void*)DRIVER_COMMU_ERR);
		}
}


void Usart2Feedback(USART2CMD_ENUM cmd,uint8_t len,uint8_t *data)
{
	unsigned char i=0;//,num=0;
	unsigned char sum=0;
	
	Usart2SendBuffer[0]=0x5A;
	Usart2SendBuffer[1]=len+5;
	Usart2SendBuffer[2]=cmd;
	
	//数据拷贝
	memcpy(&Usart2SendBuffer[3],data,len);
	
	for(i=0;i< len+ 3;i++)
	{
		sum+=Usart2SendBuffer[i];
	}
	
	Usart2SendBuffer[len+3]=sum;
	Usart2SendBuffer[len+4]=0xA5;//帧尾	

	USART2_SendEnable(len+5);
}

int fputc(int ch,FILE *f)
 {
	USART_SendData(USART2, ch);

	while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET) { }

	return(ch);
 }
/*-----------------------The end of file----------------------------------------*/
