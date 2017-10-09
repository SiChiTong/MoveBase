#include "mpu9250.h"

uint8_t Uart4SendBuffer[12]={0};
uint8_t Uart4RevBuffer[32]={0};

void UART4_Config(uint32_t baud)
{
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    GPIO_InitTypeDef	GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    

    /*-------------USART4-TX-PC10, USART4-RX-PC11-----------------*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;			//	------RS485
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,	ENABLE);
    //UART4
    USART_InitStructure.USART_BaudRate = baud;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;		//偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx;// | USART_Mode_Tx;
    USART_Init(UART4, &USART_InitStructure);
    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);		//
    USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);		//DMA接收
    //		USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);		//DMA-USART_TX
    USART_Cmd(UART4, ENABLE);


    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
    //RX
    DMA_DeInit(DMA2_Channel3);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&UART4->DR);    // 外设地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&(Uart4RevBuffer[0]);    // 内存地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                  // 内存<--外设
    DMA_InitStructure.DMA_BufferSize = 32;             // 缓存大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;    // 外设地址不增加
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;             // 内存地址增加
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 数据宽度8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; // 内存数据宽度
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                     // 循环缓存模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                 // DMA优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                        // MEM到MEM传输禁止
    //   DMA_ITConfig(DMA2_Channel3, DMA_IT_TC|DMA_IT_TE, ENABLE);                     // 允许DMA传输完中断
    DMA_Init(DMA2_Channel3, &DMA_InitStructure);
    DMA_Cmd(DMA2_Channel3, ENABLE);
}


