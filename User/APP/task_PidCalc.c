/**-------------------------------------------------------------------------
*@ file				task_PidCalc.c
*@ brief			PID计算线程，每20ms执行一次
*@ author 		ZGJ
*@ vertion		V0.01
*@ date				2015-12-15
*@ statement	(C)	COPYRIGHT 2015 KSITRI.
--------------------------------------------------------------------------*/
#include "ucos_ii.h"
#include "task_cfg.h"
#include "motor.h"

#include "usart.h"
#include "common.h"
#include "sys_config.h"
#include <math.h>
#include <stdlib.h>
#include "loadmotor.h"

#include "kalman.h"


extern _MOTION_PARAM	gSpeed;
extern float PWM_L,PWM_R;
//保存驱动器获取的编码器当前计数值
int lt_count;
int rt_count;
//驱动器收到的设定位置
int lset_count;
int rset_count;

int LSet_EncoderCycleCnt=0;
int RSet_EncoderCycleCnt=0;


//typedef union
//{
//	int count;
//	char fcount[4];
//}PWMCOUNT_UNION;

static PWMCOUNT_UNION L_Count;		// Read the position from the driver
static PWMCOUNT_UNION R_Count;

PWMCOUNT_UNION LSet_Count;
PWMCOUNT_UNION RSet_Count;
		
void Task_PidCalc(void *p_arg)
{
    uint8_t err;
    uint16_t enable = 0x01;
//		unsigned short LCnt=0;
//		unsigned short RCnt=0;
    static int L_EncoderCycleCnt=0;   // calcu. the dx dy
    static int R_EncoderCycleCnt=0;

    static int delt_L;
    static int delt_R;

    uint16_t DriverState=0;
    uint16_t getDatas[] = {0x0000,0x001D,0X001E,0x0036,0x2720};
    int i=0;

    static char L_PWMCut=0,R_PWMCut=0;
    uint16_t l_speedBack=0,r_speedBack=0;

    uint16_t l_crc,r_crc;
		
//		KalmanInit(&l_kalmanParam,0,0,1,1);
//		KalmanInit(&r_kalmanParam,0,0,1,1);
			
    for (;;)
    {

        IWDG_ReloadCounter();
        HLSGetStates(0x50,0x0000,0X0005,getDatas);	
        OSTimeDly(5);

        //中菱驱动器位置计算程序
//        if(BoardID==0)
        {					
            OSSemPend(Usart1Cal, 0, &err);
            OSSemPend(Usart3Cal, 0, &err);
					
//						if(cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.switch_stop ==0)
              if((cmdfeed.leftDriver & WRR_CONNECT )==0 && ((cmdfeed.rightDriver & WRR_CONNECT)==0))
              {             
                    if((Usart1Data.UsartRevData[1] == 0x50) &&(Usart3Data.UsartRevData[1] == 0x50))
                    {							
                        l_crc = crc_chk(Usart1Data.UsartRevData,14);							
                        if(l_crc == ((Usart1Data.UsartRevData[15]<<8)+Usart1Data.UsartRevData[14]))
                        {												
                            L_Count.fcount[3] = Usart1Data.UsartRevData[8];
                            L_Count.fcount[2] = Usart1Data.UsartRevData[9];
                            L_Count.fcount[1] = Usart1Data.UsartRevData[6];
                            L_Count.fcount[0] = Usart1Data.UsartRevData[7];
                                                                                                                        
                            delt_L = L_Count.count-lt_count;
                            
                            //溢出判断
                            if(abs(delt_L) > ( 0x7FFFFFFE - MAXCOUNT))
                            {
                                if(delt_L > 0)
                                {
                                    delt_L = delt_L - 0x7FFFFFFF;
//                                            cmdfeed.log |= 0x01;
                                }
                                else
                                {
                                    delt_L = delt_L + 0x7FFFFFFF;
                                   cmdfeed.log |= 0x02;
                                }
                            }
                                                                
                            L_EncoderCycleCnt = delt_L ;
                            lt_count = L_Count.count;		
//									printf("L ok \n");
                                                                    
                  }
                    //crc error
                    else 
                    {
//								delt_L = L_EncoderCycleCnt;
//								lt_count += delt_L;
                        cmdfeed.log |= 0x04;
                    }	
                                                
                    r_crc = crc_chk(Usart3Data.UsartRevData,14);
                    if(r_crc == ((Usart3Data.UsartRevData[15]<<8)+Usart3Data.UsartRevData[14]))
                    {								
                        R_Count.fcount[3] = Usart3Data.UsartRevData[8];
                        R_Count.fcount[2] = Usart3Data.UsartRevData[9];
                        R_Count.fcount[1] = Usart3Data.UsartRevData[6];
                        R_Count.fcount[0] = Usart3Data.UsartRevData[7];
                    
                        delt_R = R_Count.count-rt_count;								
                        //溢出判断
                        if(abs(delt_R) > ( 0x7FFFFFFE - MAXCOUNT))
                        {
                            if(delt_R > 0)
                            {
                                delt_R = delt_R - 0x7FFFFFFF;
//                                        cmdfeed.log |= 0x10;
                            }
                            else
                            {
                                delt_R = delt_R + 0x7FFFFFFF;
                                cmdfeed.log |= 0x20;
                            }
                        }
                                                    
                        R_EncoderCycleCnt = delt_R ;
                        rt_count = R_Count.count;		
//								printf("R ok \n");
                        
                    }
                    else 
                    {
//								delt_R = R_EncoderCycleCnt;
//								rt_count += delt_R;
                        cmdfeed.log |= 0x40;
                    }	
                                                                            
                    DriverState = (Usart1Data.UsartRevData[10]<<8) + Usart1Data.UsartRevData[11];
                    cmdfeed.leftDriver = ((cmdfeed.leftDriver & 0xE0) | ErrCode(DriverState));
//							cmdfeed.leftDriver |= ErrCode(DriverState);

                    
                    DriverState = (Usart3Data.UsartRevData[10]<<8) + Usart3Data.UsartRevData[11];
                    cmdfeed.rightDriver = ((cmdfeed.rightDriver &0xE0)|  ErrCode(DriverState));
//						cmdfeed.rightDriver |= ErrCode(DriverState);
                
                    if((cmdfeed.rightDriver !=0)||(cmdfeed.leftDriver  !=0 ))
                    {
                            OSQPost(EHandlerQBOX,(void*)DRIVER_ERR);
                    }
										
										 cmdfeed.l_enable = Usart1Data.UsartRevData[13];
                    if(cmdfeed.l_enable == 0)
                    {
                        cmdfeed.leftDriver |= WRR_DISABLE;
                    }
                    else
                    {
                        cmdfeed.leftDriver &= (~WRR_DISABLE);
                    }
                    cmdfeed.r_enable = Usart3Data.UsartRevData[13];
                    if(cmdfeed.r_enable == 0)
                    {
                        cmdfeed.rightDriver |= WRR_DISABLE;
                    }
                    else
                    {
                        cmdfeed.rightDriver &= (~WRR_DISABLE);
                    }
                
                    l_speedBack = (Usart1Data.UsartRevData[4]<<8) + Usart1Data.UsartRevData[5];
                    r_speedBack = (Usart3Data.UsartRevData[4]<<8) + Usart3Data.UsartRevData[5];
        
                            //PWM断线判断
                    if(((cmdfeed.l_enable )&&(PWM_L>PWM_MINLIMIT)&&((l_speedBack)==0)))
                    {
                        L_PWMCut++;
                        if(L_PWMCut>=100)
                        {
                            L_PWMCut=100;
                            OSQPost(EHandlerQBOX,(void*)PWM_CUTOFF);
//								cmdfeed.LEFT_DRIVER.MOTOR_DRIVER_STRUCT.pwmoff = 1;//PWM控制断线
                            cmdfeed.leftDriver |= WRR_PWMOFF;
                        }
                    }
                    else
                    {
                        L_PWMCut=0;
//								cmdfeed.leftDriver &= (~WRR_PWMOFF);
                    }
                        
                    if(((cmdfeed.r_enable )&&(PWM_R>PWM_MINLIMIT)&&((r_speedBack)==0)))
                    {
                        R_PWMCut++;
                        if(R_PWMCut>=100)
                        {
                            R_PWMCut=100;
                            OSQPost(EHandlerQBOX,(void*)PWM_CUTOFF);
//								cmdfeed.RIGHT_DRIVER.MOTOR_DRIVER_STRUCT.pwmoff = 1;//PWM控制断线
                            cmdfeed.rightDriver |= WRR_PWMOFF;
                        }
                    }
                    else
                    {
                        R_PWMCut = 0;
                    }
                    

                    
                }
                else
                {                  
                    L_EncoderCycleCnt = 0;
                    R_EncoderCycleCnt = 0;
                }
            }
            else
            {
                if(cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.switch_stop ==1)
                {
									if( DRIVER_POWER_OFF)
									{
                    delt_L = 0;
                    delt_R = 0;
                    lt_count = 0;
										rt_count = 0;
                    L_Count.count=0;
                    R_Count.count=0;
                    								
                    for(i=0;i<32;i++)
                    {
                        Usart1Data.UsartRevData[i]=0;
                        Usart3Data.UsartRevData[i]=0;					
                    }                  

                     cmdfeed.log |= 0x11;
									}
                }
                
                L_EncoderCycleCnt = 0;
                R_EncoderCycleCnt = 0;
//						gEncoderInf.CalcPos(0,0);
                cmdfeed.log |= 0x88;
            }
						L_PWMCut=0;
						R_PWMCut=0;
            gEncoderInf.CalcPos(-L_EncoderCycleCnt,-R_EncoderCycleCnt);                    
        }				
    }
}

/*------------------------The End of File------------------------------------------------------------*/
