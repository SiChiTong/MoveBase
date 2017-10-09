/**-------------------------------------------------------------------------
*@ file				adc.c
*@ brief			adc part
*@ author 		ZGJ
*@ vertion		V0.01
*@ date				2015-12-15
*@ statement	(C)	COPYRIGHT 2015 KSITRI.
--------------------------------------------------------------------------*/
#include "adc.h"
#include "stdint.h"
#include "stm32f10x.h"
#include <stdio.h>
#include <stdlib.h>
/* define ---------------------------------------------------------*/

/* variables ------------------------------------------------------*/
unsigned int  ADCDataBuffer[ADC_TOTAL_CHANNELS]; //ADC3的10次采样数据接收区

/* functions -------------------------------------------------------*/

/**
*@ brief		ADC 设置
*@ para			none
*@ retval		none
*/
void ADC_Configuration(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
		GPIO_InitTypeDef	GPIO_InitStructure;

		//ADC3
		GPIO_InitStructure.GPIO_Pin = TMRD_Pin | TMLD_Pin | TMRM_Pin | TMLM_Pin;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOF, &GPIO_InitStructure);
		//ADC1
		GPIO_InitStructure.GPIO_Pin = VBAT_DET_Pin;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOC, &GPIO_InitStructure);		
		//ADC3
    DMA_InitStructure.DMA_PeripheralBaseAddr = (unsigned int)&(ADC3->DR);    // 外设地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (unsigned int)&ADCDataBuffer[0];    // 内存地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                  // 外设-->内存
    DMA_InitStructure.DMA_BufferSize = 4;             // 缓存大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;    // 来源地址不增加
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;             // 内存地址增加
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // 数据宽度16位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word; // 内存数据宽度
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                     // 循环缓存模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                 // DMA优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                        // MEM到MEM传输禁止
    DMA_ITConfig(DMA2_Channel5, DMA_IT_TC, ENABLE);                     // 允许DMA传输完中断
    DMA_Init(DMA2_Channel5, &DMA_InitStructure);
    DMA_Cmd(DMA2_Channel5, DISABLE);
		
    //******************************************************************//

		ADC_DeInit(ADC3);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;                  // ADC1通道独立
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;                        // 扫描模式使能
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                 // 连续转换模式
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // 不使用外部触发
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;              // 右对齐
    ADC_InitStructure.ADC_NbrOfChannel = 4;            // 总共AD转换的路数
    ADC_Init(ADC3, &ADC_InitStructure);
		
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);    //APB2 Clock 8分频 ADC_clock = 72/8 = 9MHz
		
    ADC_RegularChannelConfig(ADC3, TMRD_ADC_Channel, 1, ADC_SampleTime_55Cycles5);  //
    ADC_RegularChannelConfig(ADC3, TMLD_ADC_Channel, 2, ADC_SampleTime_55Cycles5);  //
    ADC_RegularChannelConfig(ADC3, TMRM_ADC_Channel, 3, ADC_SampleTime_55Cycles5);  //
    ADC_RegularChannelConfig(ADC3, TMLM_ADC_Channel, 4, ADC_SampleTime_55Cycles5);  //

    ADC_DMACmd(ADC3, ENABLE);
    DMA_Cmd(DMA2_Channel5, ENABLE);
    ADC_Cmd(ADC3, ENABLE);
    ADC_ResetCalibration(ADC3);
    while (ADC_GetResetCalibrationStatus(ADC3));
    ADC_StartCalibration(ADC3);
    while (ADC_GetCalibrationStatus(ADC3));
		//ADC1
		DMA_InitStructure.DMA_PeripheralBaseAddr = (unsigned int)&(ADC1->DR);    // 外设地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (unsigned int)&ADCDataBuffer[4];    // 内存地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                  // 外设-->内存
    DMA_InitStructure.DMA_BufferSize = 1;            									 // 缓存大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;    // 来源地址不增加
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;             // 内存地址增加
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // 数据宽度16位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; // 内存数据宽度
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                     // 循环缓存模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                 // DMA优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                        // MEM到MEM传输禁止
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);                     // 允许DMA传输完中断
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel1, DISABLE);

    //******************************************************************//

		ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;                  // ADC1通道独立
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;                        // 扫描模式使能
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                 // 连续转换模式
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // 不使用外部触发
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;              // 右对齐
    ADC_InitStructure.ADC_NbrOfChannel = 1;            // 总共AD转换的路数
    ADC_Init(ADC1, &ADC_InitStructure);
		
 //   RCC_ADCCLKConfig(RCC_PCLK2_Div8);    //APB2 Clock 8分频 ADC_clock = 72/8 = 9MHz
		
    ADC_RegularChannelConfig(ADC1, VBAT_DET_ADC_Channel, 1, ADC_SampleTime_55Cycles5);

    ADC_DMACmd(ADC1, ENABLE);
    DMA_Cmd(DMA1_Channel1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));
}

/**
*@ brief		数据滤波算法
*@ para			srcData-the pointor point to src data buffer
						dstdata-the pointor point to dst data buffer
*@ retval		none
*/
void DataFilter(unsigned int *srcData, unsigned int *destData)
{
    static unsigned int srcIndex = 0;   // 来源数据次数指示
    unsigned int i, j;
		static unsigned int rtdingFilterBuff[FILTER_CHANNELS][FILTER_QLENGTH];
		static unsigned int rtdingFilterMax[FILTER_CHANNELS];
		static unsigned int rtdingFilterMin[FILTER_CHANNELS];
		static unsigned int rtdingFilterTotal[FILTER_CHANNELS];

#if (FILTER_QLENGTH < 3)
#error RT_FILTER_QLENGTH Must bigger than 2.
#endif

    // 获取数据
    for (i = 0; i < FILTER_CHANNELS; i++)
    {
        rtdingFilterBuff[i][srcIndex] = *srcData;
        srcData++;
    }

    // 递推
    srcIndex++;
    srcIndex %= FILTER_QLENGTH;

    // 每一通道逐一处理
    for (i = 0; i < FILTER_CHANNELS; i++)
    {
        rtdingFilterMax[i] = rtdingFilterBuff[i][0];
        rtdingFilterMin[i] = rtdingFilterBuff[i][0];
        rtdingFilterTotal[i] = rtdingFilterBuff[i][0];

        // 求最小与最大值，并求和
        for (j = 1; j <= FILTER_QLENGTH; j++)
        {
            if (rtdingFilterMax[i] < rtdingFilterBuff[i][j])
            {
                rtdingFilterMax[i] = rtdingFilterBuff[i][j];
            }
            if (rtdingFilterMin[i] > rtdingFilterBuff[i][j])
            {
                rtdingFilterMin[i] = rtdingFilterBuff[i][j];
            }
            rtdingFilterTotal[i] += rtdingFilterBuff[i][j];
        }
        // 去峰
        rtdingFilterTotal[i] -= rtdingFilterMax[i];
        rtdingFilterTotal[i] -= rtdingFilterMin[i];
        // 求均值
        rtdingFilterTotal[i] /= (FILTER_QLENGTH - 2);
        *destData = rtdingFilterTotal[i];
        destData++;
    }
}

