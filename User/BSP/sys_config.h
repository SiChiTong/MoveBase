#ifndef _SYS_CONFIG_H
#define _SYS_CONFIG_H

#include <stdbool.h>


// define --------------------------------------------------------------
#define	ratio			  32								//���ٱ�
#define D_wheel			200								//��ֱ��mm
#define	L_wheel			426							    //�ּ��mm
#define	HL_wheel		L_wheel/2						//�ּ���һ�룬mm
#define	PI				  3.14159
#define N_encoder		1000		  //T1�����������ش���//˫���ؼ�����Zero��20160825
#define E_ratio     10            //���ӳ��ֱ�

#define V_MAX       4000              //mm/s
#define V_MIN       5                 //mm/s   used for check PWM connect
#define CYC_TIME    0.05              //s
#define MAXCOUNT        (float)(V_MAX/PI/D_wheel*N_encoder*E_ratio*ratio*CYC_TIME)	//v*1000/pi/w_r*encoder_cyc*ratio*cyc_time
#define LIMITS          (int)((0x01<<31)-1)
#define PWM_MINLIMIT    (float)(V_MIN/PI/D_wheel*N_encoder*ratio)          //PLUS/S  V/PI/D*32*10000

#define MAX_SPEED_X      700
#define MAX_SPEED_THTA   3000


#define APPSIZE         0x20000
#define UPDATEADDRESS   (uint32_t)0x8026000
#define PARAMSSIZE      0x1000
#define PARAMSADDRESS   (uint32_t)0x08005000 

//Ӳ���忨����
#define MC_V1_1_BOARD   1      //MC_V1.1�忨
#define MC_V1_0X_BOARD  0      //MC_V1.1�忨֮ǰ�忨

#define SWITCH_OFF      0
#define SWITCH_ON       1

//Э��ѡ��
#define OLD_FEEDBACK  0
#define NEW_FEEDBACK  1

//���������뼱ͣ����ѡ�񣬳����볣��
#define NORMALLY_OPEN   0
#define NORMALLY_CLOSE  1

#define USE_PID          0

#define PERIOD          1000
#define PRESCALER       72

//ʹ������������
#define ZLDRIVER        1

#define POWER_ALARM     0

#define DRIVER_POWER_OFF 0


typedef union
{
	int count;
	char fcount[4];
}PWMCOUNT_UNION;

typedef union
{
	struct
	{
	char fallen            :1;
	char collision         :1;
	char laser             :1;
	char e_stop            :1;
	char fallen_mode       :1;
	char collision_mode    :1;
	char laser_mode        :1;
	char e_stop_mode       :1;
	}sensor;
	char state;
}SYS_SENSOR_STRUCT;


extern SYS_SENSOR_STRUCT sys_config;
extern char HardwareVer[];
extern char SoftVer[];
#endif

