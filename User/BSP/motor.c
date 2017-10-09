/*
******************************************************************
*FileName: motor.c
*
*Description: 电机控制的基本函数实现
*
*Author: Zhangguojia
*
*Date: 2014-02-11
*
*Statement: All Rights Reserved
******************************************************************
*/
//******************Include Files************************//
#include "motor.h"
#include "stm32f10x.h"
#include "ucos_ii.h"
#include "task_cfg.h"
#include "common.h"
#include "usart.h"
#include "modbus.h"
#include "fuzzypid.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "stm32f10x_tim.h"

#include "kalman.h"

//// define --------------------------------------------------------------
//#define	ratio				32									  //减速比
//#define D_wheel			200									//轮直径mm
//#define	L_wheel			426							    //轮间距mm
//#define	HL_wheel		L_wheel/2						//轮间距的一半，mm
//#define	PI					3.14159
//#define N_encoder		1000							//T1计数，单边沿触发//双边沿计数，Zero，20160825
//#define E_ratio     10               //电子齿轮比

//#define V_MAX       2000              //mm/s
//#define CYC_TIME    0.05                //s
//#define MAXCOUNT    (V_MAX/PI/D_wheel*N_encoder*E_ratio*ratio*CYC_TIME)	//v*1000/pi/w_r*encoder_cyc*ratio*cyc_time


// variables -----------------------------------------------------------

extern PID_STRUCT PID_Data;

int32_t LEncoderCycCnt = 1;
int32_t REncoderCycCnt = 1;
_MOTION_CTRL_PRM    gMotionCtrl =
{
    0,
    0,
    0,
    MOTOR_DIR_NONE,
    MOTOR_DIR_NONE,
    0,
		0,
    RobotMoveCtrl
};
_ENCODER_INF    gEncoderInf =
{
    0,
    0,
    CalaRobotPos
};
_ROBOT_POS  gRobotPos =
{
    0,
    0,
    0
};
_EncoderCycCnt gEncoderCycCnt=
{
	0,
	0,
};

_MOTION_PARAM gMotiondata;


float PWM_L,PWM_R;

static float l_lastSpeed;
static float r_lastSpeed;

//****************************模块内部函数声明******************************//
static void Motor_GPIOInit(void);
//static void Encoder_LowLevel_Init(void);
static void PIDCyc_LowLevel_Init(void);

//***************************函数实现********************************//
/**
  * @brief  读取电机当前使能状态
  * @param  None
  * @retval 1-运动状态，0-停止状态
  */
uint8_t ReadMotorCurEN(void)
{
    return (gMotionCtrl.MotorEN);
}
/**
  * @brief  电机控制相关设置
  * @param  None
  * @retval None
  */
void Motor_LowLevel_Init(void)
{
    Motor_GPIOInit();
	
	
//	//使用爱控驱动器时编码器接口初始化
//		if(BoardID==0)
//		{
//			Encoder_LowLevel_Init();
//		}
	
		PIDCyc_LowLevel_Init();
}
/**
  * @brief  Motor Ctrl IO configuration
  * @param  None
  * @retval None
  */
static void Motor_GPIOInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		//驱动器电源
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
		GPIOC->BRR=GPIO_Pin_0;
}


/**
  * @brief  左右编码器配置
  * @param  None
  * @retval None
  */
#if 0
static void Encoder_LowLevel_Init(void)
{
		GPIO_InitTypeDef	GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;

		//码盘检测：左1-PA6，左2-PA7,TIM3,右1-PB6，右2-PB7,TIM4
		GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_Init(GPIOA, &GPIO_InitStructure);

		//右编码器
    TIM_DeInit(TIM4);
    TIM_TimeBaseInitStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);
    // 配置编码模式
    TIM_EncoderInterfaceConfig(TIM4,
															 TIM_EncoderMode_TI12,  //TI1和TI2同时计数
                               TIM_ICPolarity_Rising,  //双边沿触发
                               TIM_ICPolarity_Rising);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 9;   //
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    TIM4->CNT = MIDDLE_CODER_CNT;
    TIM_Cmd(TIM4, DISABLE);
		
		//左编码器
		TIM_DeInit(TIM3);
    TIM_TimeBaseInitStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
    // 配置编码模式
    TIM_EncoderInterfaceConfig(TIM3,
                               TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising,
                               TIM_ICPolarity_Rising);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 9;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);
    TIM3->CNT = MIDDLE_CODER_CNT;
    TIM_Cmd(TIM3, DISABLE);
}
#endif
/**
  * @brief  PID adjust cycle time
  * @param  None
  * @retval None
  */
static void PIDCyc_LowLevel_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimBaseInitStructure;
    TIM_TimBaseInitStructure.TIM_Period = 20000 - 1;   //20ms定时
    TIM_TimBaseInitStructure.TIM_Prescaler = 71;
    TIM_TimBaseInitStructure.TIM_ClockDivision = 0;
    TIM_TimBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &TIM_TimBaseInitStructure);
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
    TIM_ClearFlag(TIM5, TIM_FLAG_Update);
    TIM_Cmd(TIM5, DISABLE);   //初始化,先关闭计数器，待需要控制电机时再打开
}

/*
*@ brief        电机控制，防止函数重入出错
*/
void RobotMoveCtrl(float vx, float vthta,unsigned char stopsta)
{
	unsigned char err=0;
	OSSemPend(MoveSem,0,&err);
	RobotMove(vx,vthta,stopsta);
	OSSemPost(MoveSem);
}

void RobotMove(float vx, float vthta,unsigned char stopsta)
{
		float vL=0;
		float vR=0;
	  volatile unsigned char stop_mode=0;
	

	
		if(sys_config.sensor.e_stop)
		{
			if(ReadStopSwitch==0)
			{
				cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.switch_stop = 1;
				cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.normal = 0;
				vx=0;
				vthta=0;
				stop_mode = F_STOP;
			}
		}
		else
		{
			cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.switch_stop = 0;
		}
				
		if(sys_config.sensor.collision)
		{
//				if((cmdfeed.sensor_sta&0x70)!=0)
				if((cmdfeed.collisions)!=0)
				{
					cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.normal = 0;			
					vx=0;
					vthta=0;
					stop_mode = E_STOP;
				}
				else
				{
					cmdfeed.collisions = 0x00;
				}					
		}
		
//		if(cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.io_stop)
//		{		
//			vx=0;				
//			vthta=0;
//		}
//		
		if((cmdfeed.leftDriver != 0) || (cmdfeed.rightDriver != 0))
		{
			vx=0;
			vthta=0;
			stop_mode = E_STOP;
		}	

//			if((vx > MAX_SPEED_X * 1.1 ) || (vthta  > MAX_SPEED_THTA ))
//			{
//				 vx=0;
//			   vthta=0;
//				cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.over_speed = 1 ;		
//			}
//			else
//			{
//				cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.over_speed = 0 ;		
//			}
//		if((cmdfeed.sensor_sta&&0x0001))
//		if(cmdfeed.workState == 1)
//		{	
//			vx=0;
//			vthta=0;
//		}

	//中菱驱动器的运动设置
	if(BoardID==0)
	{
		vL = vx;
		vR = vthta;
		SetPWMFreq(vL,vR);
	}
}
//计算机器人的位姿和速度。起始运动点作为原点，正前方为+x，正左为+y方向 ，
//旋转，逆时针为正，顺时针为负。位姿和速度均为两驱动轮轴中心点的数值。
//位姿中，角度的范围为-180-180度

void CalaRobotPos(int LEncoderCnt,int REncoderCnt)
{
    float Ldis = 0;
		float	Rdis=0;
		float dis=0;
		float vL=0;
		float vR=0;
		float vRotate=0;
		short	vx=0;
		short vthta=0;
		float angle;
		float dangle=0;
		float dx=0;
		float dy=0;
		float RR=0;
		static char stop_count=0;
	  static char push_count=0;
	
	  IWDG_ReloadCounter();
		
		Ldis=LEncoderCnt;
		Rdis=-REncoderCnt;					//轮子反装，编码器线序是一样的
	
//	//use kalmanfilter 
//		Ldis = KalmanFilter(&l_kalmanParam,l_lastSpeed*0.05);
//		Rdis = KalmanFilter(&r_kalmanParam,r_lastSpeed*0.05);
		//计算左右轮运动的距离
		Ldis=Ldis/N_encoder/E_ratio;
		Ldis=Ldis/ratio;
		Ldis=Ldis*PI*D_wheel;

		Rdis=Rdis/N_encoder/E_ratio;
		Rdis=Rdis/ratio;
		Rdis=Rdis*PI*D_wheel;

		dis=(Ldis+Rdis)/2;
		//dangle=(Rdis-Ldis)/L_wheel;
		dangle=(-Rdis+Ldis)/L_wheel;
		
				//20170731,Zero
//		if(gMotiondata.vx == 0)
//		{
//		  dx = 0;
//		}
//		if(gMotiondata.vthta == 0)
//		{
//			dangle = 0;
//			dy=0;
//		}

		if(fabs(dangle) < 0.0001)	
		{
			dangle = 0;
			dx=dis;
			dy=0;
		}
		
		else
		{
			RR=dis/dangle;
//			RR = (-REncoderCnt +LEncoderCnt)*L_wheel/(2*( -LEncoderCnt - REncoderCnt ));		
			dx=RR*sin(dangle);
			dy=RR*(1-cos(dangle));
		}
		
		//计算速度
		vL=Ldis*20;				//mm/s
		vR=Rdis*20;
		
		//20160721,Zero
		if((gMotiondata.vx==0)&&(gMotiondata.vthta==0))
		{
			//反馈速度为零，设定速度为零，置停止位
			if((vL==0)&&(vR==0))
			{
				stop_count++;
				if( stop_count >= 10 )
				{
					stop_count = 10 ;
					cmdfeed.exception |= 0x04;
				}
			}
		
			//设定速度为零反馈速度不为零表示被推动
			if(fabs(vL)+fabs(vR)>100)
			{
				push_count++;
				if(push_count>=25)
				{
					push_count=25;
					cmdfeed.exception |= 0x01;//停止时被推动
				}
			}
			else
			{
				push_count=0;
				cmdfeed.exception &= 0xFE;
			}
		}
		//设定速度不为零
		else
		{
			stop_count = 0;
			cmdfeed.exception &= 0xFB;
		}
		
		vRotate=(vR-vL)*1000/L_wheel;			//(rad/s)*1000
//		vRotate=(vR-vL)*1000/L_Encoder;
		
		vx=(short)((vL+vR)/2);			//mm/s
		vthta=(short)vRotate;			//(rad/s)*1000
		
		gRobotPos.x += cos(gRobotPos.angle+dangle/2)*dx-dy*sin(gRobotPos.angle+dangle/2);
		gRobotPos.y += sin(gRobotPos.angle+dangle/2)*dx+dy*cos(gRobotPos.angle+dangle/2);
		gRobotPos.angle+=dangle;
		
		angle = gRobotPos.angle;
		//规整到-PI--PI
		while(angle>PI)
		{
			angle-=2*PI;
		}
		while(angle<-PI)
		{
			angle+=2*PI;
		}
		gRobotPos.angle=angle;
		cmdfeed.vx=vx;
		cmdfeed.vthta=-vthta;//20170720,Zero
		//cmdfeed.vthta=vthta;
		(cmdfeed.dx).f=gRobotPos.x;
		(cmdfeed.dy).f=gRobotPos.y;
		(cmdfeed.dthta).f=gRobotPos.angle;
			
}


//void SetPWMFreq(float vx,float vthta)
//{
//	unsigned short peroid;
//	unsigned int l_freq,r_freq;
//	float vL,vR;//设定的左右轮速度
////	float f_vL,f_vR;//反馈的左右轮速度
////	static char L_PWMCut=0,R_PWMCut=0;
//	
//	vL=vx-vthta*HL_wheel/1000;			//mm/s
//	vR=vx+vthta*HL_wheel/1000;
//				
//			//计算转速
//	vL=ratio*vL/(PI*D_wheel);			//RPS
//	vR=ratio*vR/(PI*D_wheel);
//	
//	l_freq=fabs(vL*N_encoder);
//	r_freq=fabs(vR*N_encoder);
//	
//	//记录全局速度
//	PWM_L = l_freq;
//	PWM_R = r_freq;
//	
//		//设置运动方向		
//	if(vL<0)
//	{
//		GPIO_SetBits(GPIOG,GPIO_Pin_6);		

//	}
//	else
//	{
//		GPIO_ResetBits(GPIOG,GPIO_Pin_6);

//	}
//	
//		//设置运动方向
//	if(vR<0)
//	{
//		GPIO_ResetBits(GPIOB,GPIO_Pin_1);
//	}
//	else
//	{
//		GPIO_SetBits(GPIOB,GPIO_Pin_1);
//	}
//	
//	if(l_freq==0)
//	{
//		TIM2->CCR2 = 0;
//	}
//	else
//	{
//		peroid=1000000/l_freq;	
//		TIM2->ARR = peroid;
//		TIM2->CCR2 = peroid/2;
//	}	
//		
//	if(r_freq==0)
//	{
//		//TIM8->CCR2 = 0;
//		TIM3->CCR3 = 0;
//	}
//	else
//	{
//		peroid=1000000/r_freq;	
////		TIM8->ARR = peroid;
////		TIM8->CCR2 = peroid/2;
//		TIM3->ARR = peroid;
//		TIM3->CCR3 = peroid/2;
//	}			
//}

void SetPWMFreq(float vx,float vthta)
{
	unsigned short l_peroid,r_peroid;
	unsigned short l_duty,r_duty;
	unsigned int l_freq,r_freq;
	float vL,vR;//设定的左右轮速度
//	float f_vL,f_vR;//反馈的左右轮速度
//	static char L_PWMCut=0,R_PWMCut=0;
	
	vL=vx-vthta*HL_wheel/1000;			//mm/s
	vR=vx+vthta*HL_wheel/1000;
	
	l_lastSpeed = vL;
	r_lastSpeed = vR;
				
			//计算转速
	vL=ratio*vL/(PI*D_wheel);			//RPS
	vR=ratio*vR/(PI*D_wheel);
	
	l_freq=fabs(vL*N_encoder);
	r_freq=fabs(vR*N_encoder);
	
	//记录全局速度,plus/s
	PWM_L = l_freq;
	PWM_R = r_freq;
	
		//设置运动方向		
	if(vL<0)
	{
		GPIO_SetBits(GPIOG,GPIO_Pin_6);		
	}
	else
	{
		GPIO_ResetBits(GPIOG,GPIO_Pin_6);
	}
	
		//设置运动方向
	if(vR<0)
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_1);
	}
	else
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_1);
	}

	
	if(l_freq==0)
	{
		l_peroid = 0xFF;
		l_duty = 0;
	}
	else
	{
		l_peroid=1000000/l_freq;	
		l_duty = l_peroid/2;
	}	
	

	
	if(r_freq==0)
	{
		r_peroid = 0xFF;
		r_duty = 0;

	}
	else
	{
		r_peroid=1000000/r_freq;	
		r_duty = r_peroid/2;
	}	
	
	TIM2->ARR = l_peroid;
	TIM2->CCR2 = l_duty;
	
	TIM3->ARR = r_peroid;
	TIM3->CCR3 = r_duty;
	
}
/*--------------------------The End of File----------------------------------------------------------*/

