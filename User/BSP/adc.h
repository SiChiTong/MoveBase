/**-------------------------------------------------------------------------
*@ file				adc.h
*@ brief			adc part
*@ author 		ZGJ
*@ vertion		V0.01
*@ date				2015-12-15
*@ statement	(C)	COPYRIGHT 2015 KSITRI.
--------------------------------------------------------------------------*/
#ifndef __ADC_H
#define __ADC_H

#include "stm32f10x.h"

#ifdef __cplusplus
extern "C"
{
#endif
/* typedef------------------------------------------------------------------*/
typedef enum
{
	ADC_TMRD_SQ=0,
	ADC_TMLD_SQ=1,
	ADC_TMRM_SQ=2,
	ADC_TMLM_SQ=3,
	ADC_VBAT_SQ=4
}_ADC_SEQUENCE;
//****************系统状态******************//
typedef struct
{
	unsigned int TMRD_Value;
	unsigned int TMLD_Value;
	unsigned int TMRM_Value;
	unsigned int TMLM_Value;
	unsigned int VBAT_Value;
}_ADC_DETECT_DATA;

/* define------------------------------------------------------------------------------------------*/
#define TMRD_Pin			GPIO_Pin_7					//PF
#define TMLD_Pin			GPIO_Pin_8
#define TMRM_Pin			GPIO_Pin_9
#define TMLM_Pin			GPIO_Pin_10
#define VBAT_DET_Pin	GPIO_Pin_5			//PC

#define TMRD_ADC_Channel	ADC_Channel_5				//ADC3
#define TMLD_ADC_Channel	ADC_Channel_6
#define TMRM_ADC_Channel	ADC_Channel_7
#define TMLM_ADC_Channel	ADC_Channel_8
#define VBAT_DET_ADC_Channel	ADC_Channel_15			//ADC1	

#define ADC3_DR_Address    ((uint32_t)0x40013C4C)  //ADC_DT的地址
#define StartNextADC3()     ADC_SoftwareStartConvCmd(ADC3, ENABLE) //启动AD检测
#define StartNextADC1()			ADC_SoftwareStartConvCmd(ADC1, ENABLE)

#define ADC_TOTAL_CHANNELS	5
#define FILTER_CHANNELS			5			//FILTER_CHANNELS must equal ADC_TOTAL_CHANNELS
#define FILTER_QLENGTH			10		//the groups of datas to filter, it must bigger than 2

/* 新电池， 500mA放电电流 */
#define VBAT_STATE_10		30000
#define VBAT_STATE_9		29000
#define VBAT_STATE_8    27790       //90%
#define VBAT_STATE_7    27440       //85%
#define VBAT_STATE_6    27090       //80%
#define VBAT_STATE_5    26530       //70%
#define VBAT_STATE_4    26110       //60%
#define VBAT_STATE_3    25760       //50%
//#define VBAT_STATE_2    25550       //40%
#define VBAT_STATE_2    25000        //20160506,Zero
#define VBAT_STATE_1		24000				//关机电压

#define VBAT_LEVEL_10		10
#define VBAT_LEVEL_9		9
#define VBAT_LEVEL_8		8
#define VBAT_LEVEL_7		7
#define VBAT_LEVEL_6		6
#define VBAT_LEVEL_5		5
#define VBAT_LEVEL_4		4
#define VBAT_LEVEL_3		3
#define VBAT_LEVEL_2		2
#define VBAT_LEVEL_1		1
#define VBAT_LEVEL_0		0

#define MOTOR_TEMP_THRESHOLD_HIGH			100
#define MOTOR_TEMP_THRESHOLD_LOW			-15

/* variables-----------------------------------------------------------------------------------------------*/

extern unsigned int  ADCDataBuffer[ADC_TOTAL_CHANNELS];

//*********************接口函数*********************//
void ADC_Configuration(void);
void DataFilter(unsigned int *srcData, unsigned int *destData);

#ifdef __cplusplus
}
#endif

#endif

