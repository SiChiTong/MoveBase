/**-------------------------------------------------------------------------
*@ file				task_AdcDetect.c
*@ brief			user task for adc detect
*@ author 		ZGJ
*@ vertion		V0.01
*@ date				2015-12-15
*@ statement	(C)	COPYRIGHT 2015 KSITRI.
--------------------------------------------------------------------------*/
/* Include Files---------------------------------------------------------*/
#include "ucos_ii.h"
#include "adc.h"
#include "task_cfg.h"
#include "common.h"
#include "usart.h"

#include "motor.h"
#include "vl6180x.h"
#include <stdio.h>

const float R_temp[12][12]={
	486.08,458.63,432.91,408.79,386.17,364.94,345.02,326.3, 308.71,292.18,276.64,262.02,
	248.26,235.31,223.11,211.61,200.78,190.57,180.93,171.84,163.26,155.16,147.51,140.28,
	133.45,126.99,120.88,115.09,109.62,104.44,99.53, 94.88, 90.48, 86.3,  82.34, 78.58,
	75.02, 71.64, 68.42, 65.37, 62.48, 59.72, 57.11, 54.62, 52.25, 50,    47.86, 45.82,
	43.88, 42.03, 40.27, 38.59, 36.99, 35.47, 34.01, 32.62, 31.3,  30.04, 28.83, 27.68,
	26.58, 25.53, 24.53, 23.57, 22.66, 21.78, 20.94, 20.14, 19.38, 18.64, 17.94, 17.27,
	16.62, 16.01, 15.42, 14.85, 14.31, 13.79, 13.29, 12.81, 12.35, 11.91, 11.49, 11.09,
	10.7,  10.32, 9.964, 9.619, 9.288, 8.97,  8.66,  8.37,  8.09,  7.82,  7.55,  7.3,
	7.06,  6.83,  6.61,  6.39,  6.18,  5.98,  5.79,  5.6,   5.43,  5.25,  5.09,  4.93,
	4.77,  4.62,  4.48,  4.34,  4.21,  4.08,  3.96,  3.84,  3.72,  3.61,  3.5,   3.4,
	3.3,   3.2,   3.1,   3.01,  2.93,  2.84,  2.76,  2.68,  2.6,   2.53,  2.46,  2.388,
	2.321, 2.256, 2.193, 2.132, 2.073, 2.016, 1.961, 1.908, 1.856, 0,			0,     0
};
signed char RToTemp(float rtemp)
{
	unsigned char i=0;
	unsigned char j=0;
	signed char temp=0;
	if(rtemp>=R_temp[0][0])
	{
		return (-20);
	}
	if(rtemp<=R_temp[11][8])
	{
		return(120);
	}
	for(i=0;i<12;i++)
	{
		if(rtemp > R_temp[i][0])
			break;		
	}
	if(i==0) return (-20);	//err
	i-=1;
	for(j=0;j<12;j++)
	{
		if(rtemp > R_temp[i][j])
			break;
	}
	temp=i*12+j-20;	
	return temp;
}

void Task_AdcDetect(void *p_arg)
{
	unsigned char err=0;
	unsigned int DstData[ADC_TOTAL_CHANNELS]={0};
	_ADC_DETECT_DATA	sAdcData={0,0,0,0};
	volatile static unsigned short power_cnt=0;
	static unsigned short work_cnt=0;
	unsigned int DVBAT=0;
	volatile unsigned char pwrstate=0;
	volatile unsigned char pwrstate_pre=0;
	volatile static unsigned char statemachin=0;
	volatile static unsigned char power_on_sta=1;		//开机电量检测状态，低于25.5V不允许开机。如开机成功，低于24V关机保护电池
	static signed char motor_temp_L=0;
	static signed char motor_temp_R=0;
	float  r_mf52=0;
	static unsigned char stabletime=0;
	
	OSSemPost(PowerUpdateSem);
	statemachin=6;
	
	StartNextADC1();
	StartNextADC3();
	OSTimeDly(1);
	for(;;)
	{
		OSSemPend(Adc1CycSem,0,&err);
		OSSemPend(Adc3CycSem,0,&err);
		IWDG_ReloadCounter();					//喂狗200ms
		DataFilter(ADCDataBuffer,DstData);
		sAdcData.TMRD_Value=DstData[ADC_TMRD_SQ];
		sAdcData.TMLD_Value=DstData[ADC_TMLD_SQ];
		sAdcData.TMRM_Value=DstData[ADC_TMRM_SQ];
		sAdcData.TMLM_Value=DstData[ADC_TMLM_SQ];
		sAdcData.VBAT_Value=DstData[ADC_VBAT_SQ];
		
		if(stabletime<10)
		{
			stabletime++;
		}
		else
		{
			r_mf52=sAdcData.TMLM_Value*3300/4608;			
			r_mf52=r_mf52*49.9/(3300-r_mf52);
			motor_temp_L=(signed char)RToTemp(r_mf52);
			
			//电机过温
#if NEW_FEEDBACK
			if (motor_temp_L>MOTOR_TEMP_THRESHOLD_HIGH)
			{
				cmdfeed.LEFT.MOTOR_DRIVER_STRUCT.temp=1;
			}
//			else
//				if(CleaerLeftErrFlag==1)
//				{
//					cmdfeed.LEFT.MOTOR_DRIVER_STRUCT.temp=0;
//				}
				
			if (motor_temp_R>MOTOR_TEMP_THRESHOLD_HIGH)
			{
				cmdfeed.RIGHT.MOTOR_DRIVER_STRUCT.temp=1;
			}
//			else
//				if(CleaerRightErrFlag==1)
//				{
//					cmdfeed.RIGHT.MOTOR_DRIVER_STRUCT.temp=0;
//				}
			
#endif
			
			r_mf52=sAdcData.TMRM_Value*3300/4608;
			r_mf52=r_mf52*49.9/(3300-r_mf52);
			motor_temp_R=(signed char)RToTemp(r_mf52);
		
			if((motor_temp_L>MOTOR_TEMP_THRESHOLD_HIGH) || (motor_temp_R>MOTOR_TEMP_THRESHOLD_HIGH))
			{
				OSQPost(EHandlerQBOX,(void*)MOTOR_OVER_THMP);
			}

		}
		
		//电池电量检测
		DVBAT=(unsigned int)((float)(sAdcData.VBAT_Value)*3300/4608);
		DVBAT=DVBAT*12;
		
		if(DVBAT>25000)
			cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.low_power = 0;
		
			//20160503,Zero，电池电压反馈
		if(DVBAT > VBAT_STATE_10)
			cmdfeed.battery = (DVBAT-30000)/1000+200;//y=x+170;x--电池电压，单位V
		else if(DVBAT>20000&&DVBAT<=30000)
			cmdfeed.battery = 10*(DVBAT - 20000)/1000+100;//y=10x-100;
		else if (DVBAT>0&&DVBAT<=20000)
			cmdfeed.battery = 5*DVBAT/1000;//y=5*x
		
//20160812,Zero		
#if POWER_ALARM		
		if (OSSemAccept(PowerUpdateSem)>0)
    {
       if (DVBAT >= VBAT_STATE_10)       pwrstate = VBAT_LEVEL_10;
			 else if (DVBAT >= VBAT_STATE_9)  pwrstate = VBAT_LEVEL_9;
			 else if (DVBAT >= VBAT_STATE_8)  pwrstate = VBAT_LEVEL_8;
       else if (DVBAT >= VBAT_STATE_7)  pwrstate = VBAT_LEVEL_7;
       else if (DVBAT >= VBAT_STATE_6)  pwrstate = VBAT_LEVEL_6;
       else if (DVBAT >= VBAT_STATE_5)  pwrstate = VBAT_LEVEL_5;
       else if (DVBAT >= VBAT_STATE_4)  pwrstate = VBAT_LEVEL_4;
       else if (DVBAT >= VBAT_STATE_3)  pwrstate = VBAT_LEVEL_3;
       else if (DVBAT >= VBAT_STATE_2)  pwrstate = VBAT_LEVEL_2;
			 else if (DVBAT >= VBAT_STATE_1)  pwrstate = VBAT_LEVEL_1;
       else                             pwrstate = VBAT_LEVEL_0;
       if (pwrstate == pwrstate_pre)
       {
         statemachin++;
       }
       else
       {
         statemachin = 0;
       }
       if (statemachin >= 6)
       {
				 statemachin = 0;
          switch (pwrstate)
          {
			//20160412,Zero,去掉高压断电
						case VBAT_LEVEL_10:
						case VBAT_LEVEL_9:		
						case VBAT_LEVEL_8:
						case VBAT_LEVEL_7:
						case VBAT_LEVEL_6:
						case VBAT_LEVEL_5:
						case VBAT_LEVEL_4:
						case VBAT_LEVEL_3:
						case VBAT_LEVEL_2:
//				{
//					if(ReadSysStaBit(MotorEnFlag))
//					{
//						ClearSysStaBit(MotorEnFlag);
//					}
//					if(power_on_sta==1)
//					{
//						power_on_sta=0;
//					}
//				}
//			#if NEW_FEEDBACK
						if(cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.low_power==0x01)
								cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.low_power = 0;
//			#endif
					break;
			case VBAT_LEVEL_1:
			{
				if(power_on_sta==1)
				{
				#if NEW_FEEDBACK
					cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.low_power = 1;
				#endif
					OSQPost(EHandlerQBOX,(void*)LOW_POWER);
					SetSysStaBit(MotorEnFlag);		//开机电压低于25V，不允许电机运行
					power_on_sta=0;
				}
				else
				{
					if(ReadSysStaBit(MotorEnFlag))
					{
						ClearSysStaBit(MotorEnFlag);
					}
				}
			}
			break;
			case VBAT_LEVEL_0:
			{
			#if NEW_FEEDBACK
				cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.low_power = 1;
			#endif
				OSQPost(EHandlerQBOX,(void*)LOW_POWER);
				if(power_on_sta==1)
				{
					power_on_sta=0;
				}
				SetSysStaBit(MotorEnFlag);		//开机电压低于24V，不允许电机运行
			}
			break;
			default:
				
				break;
		}
		statemachin = 0;
       }
       pwrstate_pre = pwrstate;
    }
		//开关机键检测
		if(IsPlayOnPush())
		{
			power_cnt++;
			if(power_cnt>=32)
			{
				power_cnt=0;
				OpenBeep();
				CloseSysPower();
			}
		}
		else if(power_cnt!=0)
		{
			power_cnt=0;
		}		
#endif
		//系统运行LED显示
		work_cnt++;

		
		if(work_cnt>=12)
		{
			work_cnt=0;
			GPIOG->ODR ^= GPIO_Pin_9;
		}
		
		StartNextADC1();
		StartNextADC3();
		OSTimeDly(2);
		
		
	}
}


/*---------------------The End of File---------------------------------------*/
