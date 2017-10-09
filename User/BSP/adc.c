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
unsigned int  ADCDataBuffer[ADC_TOTAL_CHANNELS]; //ADC3��10�β������ݽ�����

/* functions -------------------------------------------------------*/

/**
*@ brief		ADC ����
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
    DMA_InitStructure.DMA_PeripheralBaseAddr = (unsigned int)&(ADC3->DR);    // �����ַ
    DMA_InitStructure.DMA_MemoryBaseAddr = (unsigned int)&ADCDataBuffer[0];    // �ڴ��ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                  // ����-->�ڴ�
    DMA_InitStructure.DMA_BufferSize = 4;             // �����С
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;    // ��Դ��ַ������
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;             // �ڴ��ַ����
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // ���ݿ��16λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word; // �ڴ����ݿ��
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                     // ѭ������ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                 // DMA���ȼ�
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                        // MEM��MEM�����ֹ
    DMA_ITConfig(DMA2_Channel5, DMA_IT_TC, ENABLE);                     // ����DMA�������ж�
    DMA_Init(DMA2_Channel5, &DMA_InitStructure);
    DMA_Cmd(DMA2_Channel5, DISABLE);
		
    //******************************************************************//

		ADC_DeInit(ADC3);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;                  // ADC1ͨ������
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;                        // ɨ��ģʽʹ��
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                 // ����ת��ģʽ
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // ��ʹ���ⲿ����
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;              // �Ҷ���
    ADC_InitStructure.ADC_NbrOfChannel = 4;            // �ܹ�ADת����·��
    ADC_Init(ADC3, &ADC_InitStructure);
		
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);    //APB2 Clock 8��Ƶ ADC_clock = 72/8 = 9MHz
		
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
		DMA_InitStructure.DMA_PeripheralBaseAddr = (unsigned int)&(ADC1->DR);    // �����ַ
    DMA_InitStructure.DMA_MemoryBaseAddr = (unsigned int)&ADCDataBuffer[4];    // �ڴ��ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                  // ����-->�ڴ�
    DMA_InitStructure.DMA_BufferSize = 1;            									 // �����С
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;    // ��Դ��ַ������
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;             // �ڴ��ַ����
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // ���ݿ��16λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; // �ڴ����ݿ��
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                     // ѭ������ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                 // DMA���ȼ�
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                        // MEM��MEM�����ֹ
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);                     // ����DMA�������ж�
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel1, DISABLE);

    //******************************************************************//

		ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;                  // ADC1ͨ������
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;                        // ɨ��ģʽʹ��
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                 // ����ת��ģʽ
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // ��ʹ���ⲿ����
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;              // �Ҷ���
    ADC_InitStructure.ADC_NbrOfChannel = 1;            // �ܹ�ADת����·��
    ADC_Init(ADC1, &ADC_InitStructure);
		
 //   RCC_ADCCLKConfig(RCC_PCLK2_Div8);    //APB2 Clock 8��Ƶ ADC_clock = 72/8 = 9MHz
		
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
*@ brief		�����˲��㷨
*@ para			srcData-the pointor point to src data buffer
						dstdata-the pointor point to dst data buffer
*@ retval		none
*/
void DataFilter(unsigned int *srcData, unsigned int *destData)
{
    static unsigned int srcIndex = 0;   // ��Դ���ݴ���ָʾ
    unsigned int i, j;
		static unsigned int rtdingFilterBuff[FILTER_CHANNELS][FILTER_QLENGTH];
		static unsigned int rtdingFilterMax[FILTER_CHANNELS];
		static unsigned int rtdingFilterMin[FILTER_CHANNELS];
		static unsigned int rtdingFilterTotal[FILTER_CHANNELS];

#if (FILTER_QLENGTH < 3)
#error RT_FILTER_QLENGTH Must bigger than 2.
#endif

    // ��ȡ����
    for (i = 0; i < FILTER_CHANNELS; i++)
    {
        rtdingFilterBuff[i][srcIndex] = *srcData;
        srcData++;
    }

    // ����
    srcIndex++;
    srcIndex %= FILTER_QLENGTH;

    // ÿһͨ����һ����
    for (i = 0; i < FILTER_CHANNELS; i++)
    {
        rtdingFilterMax[i] = rtdingFilterBuff[i][0];
        rtdingFilterMin[i] = rtdingFilterBuff[i][0];
        rtdingFilterTotal[i] = rtdingFilterBuff[i][0];

        // ����С�����ֵ�������
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
        // ȥ��
        rtdingFilterTotal[i] -= rtdingFilterMax[i];
        rtdingFilterTotal[i] -= rtdingFilterMin[i];
        // ���ֵ
        rtdingFilterTotal[i] /= (FILTER_QLENGTH - 2);
        *destData = rtdingFilterTotal[i];
        destData++;
    }
}

