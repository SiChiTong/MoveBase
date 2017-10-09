/**-------------------------------------------------------------------------
*@ file				task_protocol.c
*@ brief			Э���������
*@ author 		ZGJ
*@ vertion		V0.01
*@ date				2015-12-15
*@ statement	(C)	COPYRIGHT 2015 KSITRI.
--------------------------------------------------------------------------*/
/* Include Files---------------------------------------------------------*/
#include "task_cfg.h"
#include "ucos_ii.h"
#include "usart.h"
#include "common.h"
#include "motor.h"
#include "vl6180x.h"
#include "modbus.h"
#include "sys_config.h"
#include "libmd5.h"
#include "stm32f10x_flash.h"
#include "linear_actuator.h"
#include "loadmotor.h"
#include <stdio.h>
#include <math.h>


BOOTLOADER_UNION       bootTable;
extern LINEAR_ACTUATOR_UNION LinearData;
//extern uint8_t PCLoadFlag ;
//extern uint8_t PCUnloadFlag ;
//extern uint8_t PCLinearOff ;
//extern uint8_t PCLinearOn ;
//extern uint8_t PCLinearDisable ;
extern uint8_t loadMotorState;

extern uint8_t ChargingFlag ;
extern uint8_t ChargedFlag ;

void Task_Protocol(void *p_arg)
{
	unsigned char err=0;
	//_RECV_DATA		Pro;
	//_RECV_DATA		*pPro=&Pro;
	float vx=0;
	float vthta=0;
	//uint8_t *pRev=Usart2RevBuffer;
	//uint8_t sum=0;
	//USART2RevBuf_t  *ptr = NULL;
	//uint8_t u2RecvCh = 0;               // �洢���յ����ַ�
  //static uint8_t u2StaMach = 0;
  //static uint8_t revindex = 0;
	//static uint8_t dataindex=0;
	
	uint8_t Usart2TempBuff[255] = {0};
	uint8_t Usart2TempBuff2[255] = {0};
	USART2CMD_ENUM CMDType=ZERO_ENUM;
	uint8_t i=0;
	char CheckSum=0;
	uint8_t FeedbackkData[32]={0};
	char PageNum=0;
	int DataNum=0;      //��������ֵ
	char DeltAdd=0;     //ÿ����������
	char MD5Check[16]={0};     //MD5У��
	int DataSize=0;    //�̼�����С
    	
	uint8_t EraseState=0;
	uint8_t WriteState=0;
	
	int32_t usedTime1 = 0;
	int32_t usedTime2 = 0;
	
	Md5Context MD5Ctx;
	MD5_HASH md5_ret;
	
	FLASH_Status FlashEraseState; 
	FLASH_Status FlashWriteState; 
	
	_MOTION_PARAM	mPara={0,0,0};

	HardWare_LowLevel_Init();
	OpenDriverPower();		//��ʼ��.
	USART_Cmd(UART4,ENABLE);
	
	OS_ENTER_CRITICAL();
	OS_SysObject_Create();
	OS_Task_Create();
	OS_EXIT_CRITICAL();
		
	Work_LedOn();
	OpenSysPower();
	USART2RevHeadInit();
	
	SetSysStaBit(SafeModeSta);		//������ȫģʽ
	
	cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.safe_mode = 1;
	OpenMotorCLoopControl();
	
	TIM_Cmd(TIM6,ENABLE);

	USART_Cmd(USART2,ENABLE);
	IWDG_Enable();
	
	cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.power_on = 1;
	
	OSTimeDly(1);
//	Cmd_Send(&cmdfeed);
	for(;;)
	{
		OSSemPend(Usart2CMDSem,100,&err);
		
//		usedTime1 = OSTimeGet();
//		printf("usedTime1 = %d \n" ,usedTime1);
		
		if(err==OS_ERR_NONE)
		{

				//��������ʱ����
				memcpy(Usart2TempBuff,Usart2RevBuffer,255);
			
        IWDG_ReloadCounter();
				OSSemPost(CommuQuerySem);
				OSSemSet(CmdOverTimeSem,25,&err);

				CheckSum = 0;
				for(i=0;i<Usart2TempBuff[1]-2;i++)
				{				
					CheckSum += Usart2TempBuff[i];
				}	
	
				
					//֡ͷ��֡β�ж�
				if((Usart2TempBuff[0]==0x5A)&&(Usart2TempBuff[Usart2TempBuff[1]-1]==0xA5)&&( CheckSum == Usart2TempBuff[Usart2TempBuff[1]-2]))
				{
                    CMDType = (USART2CMD_ENUM)Usart2TempBuff[2];
                    switch(CMDType)
                    {
                        //��������ź�
                        case CLEAR_POWERSTATE:																										
                            cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.power_on = 0;
                            FeedbackkData[0] = cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.power_on;
                            Usart2Feedback(CMDType,1,FeedbackkData);						
                            break;
                        //��ȡ�����ź�						
                        case READ_POWERSTATE:
                            FeedbackkData[0] = cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.power_on;
                            Usart2Feedback(CMDType,1,FeedbackkData);						
                            break;
                        
//							//���÷������������
//							case SET_FALLENLIMIT:
//								gfallSensorThread.FallSensorUpThread=Usart2TempBuff[3];
//								gfallSensorThread.FallSensorDownThread=Usart2TempBuff[4];
//							
//							  FeedbackkData[0] = gfallSensorThread.FallSensorUpThread;
//							  FeedbackkData[1] = gfallSensorThread.FallSensorDownThread;
//							  Usart2Feedback(CMDType,2,FeedbackkData);							
//								break ;
//							
//							case READ_FALLENLIMIT:
//								FeedbackkData[0] = gfallSensorThread.FallSensorUpThread;
//							  FeedbackkData[1] = gfallSensorThread.FallSensorDownThread;
//							  Usart2Feedback(CMDType,2,FeedbackkData);		
//								break;
                        
                        case CLEAR_SWITCH_STATE:
                            cmdfeed.workState = Usart2TempBuff[3];
                          //cmdfeed.loadSwitchs = Usart2TempBuff[3];
                          FeedbackkData[0] = cmdfeed.workState;//cmdfeed.loadSwitchs;
                            Usart2Feedback(CMDType,1,FeedbackkData);	
                            break;
                        
                        //
                        case READ_LOADSWITCHS:
                          FeedbackkData[0] = cmdfeed.loadSwitchs;	//cmdfeed.collisions;
                            Usart2Feedback(CMDType,1,FeedbackkData);	
                            break;
                        
                        //���÷����䡢������IO����ͣ�Ŀ�����ر�
                        case SET_SENSOR:
                            //20170717,Zero
                            sys_config.state = 0x08;//Usart2TempBuff[3];
                            FeedbackkData[0] = sys_config.state ;
                            Usart2Feedback(CMDType,1,FeedbackkData);
                            break;
                        
                        //������״̬
                        case READ_SENSOR:
                            FeedbackkData[0] = sys_config.state ;
                            Usart2Feedback(CMDType,1,FeedbackkData);
                            break;
                        
                        case CLEAR_ERRSTATE:						
                            //��������������
                            if(BoardID==0)
                            {
                                if(cmdfeed.leftDriver <= ERR_ENCODERZ_OVER||cmdfeed.rightDriver <= ERR_ENCODERZ_OVER)		
                                {								
                                                
                                }
                                FeedbackkData[0]=cmdfeed.leftDriver;
                                FeedbackkData[1]=cmdfeed.rightDriver;
                                Usart2Feedback(CMDType,2,FeedbackkData);				
                            }
                                                        
                            break;
                            
                        case READ_ERRSTATE:
                                    //�������������ص���״̬
//										if(BoardID==0)
//										{
//											FeedbackkData[0]=cmdfeed.LEFT.MOTOR_DRIVER_STATE;
//											FeedbackkData[1]=cmdfeed.RIGHT.MOTOR_DRIVER_STATE;
//										}
//										//��������������״̬
//										if(BoardID==2)
                                    {
                                        FeedbackkData[0]=cmdfeed.leftDriver;
                                        FeedbackkData[1]=cmdfeed.rightDriver;
                                    }
                          Usart2Feedback(CMDType,2,FeedbackkData);
                            break;
                            
                        case ROBOT_MOVE:
                            
                            vx=(float)((Usart2TempBuff[3]<<8)+Usart2TempBuff[4]);
                            vthta=(float)((Usart2TempBuff[5]<<8)+Usart2TempBuff[6]);
                
                            if(vx>0x7FFF) 
                                vx=-(0xFFFF-vx+1);
                            if(vthta>0x7FFF) 
                                vthta=-(0xFFFF-vthta+1);
                            
                            mPara.vx=vx;
                            mPara.vthta=vthta;										
                            gMotiondata.vx=mPara.vx;
                            gMotiondata.vthta=mPara.vthta;
                            OSQPost(MotionCtrlQBOX,(void*)&mPara);
                            
                            FeedbackkData[0]=cmdfeed.dx.fc[3];
                            FeedbackkData[1]=cmdfeed.dx.fc[2];
                            FeedbackkData[2]=cmdfeed.dx.fc[1];
                            FeedbackkData[3]=cmdfeed.dx.fc[0];
                            
                            FeedbackkData[4]=cmdfeed.dy.fc[3];
                            FeedbackkData[5]=cmdfeed.dy.fc[2];
                            FeedbackkData[6]=cmdfeed.dy.fc[1];
                            FeedbackkData[7]=cmdfeed.dy.fc[0];
                            
                            FeedbackkData[8]=cmdfeed.dthta.fc[3];
                            FeedbackkData[9]=cmdfeed.dthta.fc[2];
                            FeedbackkData[10]=cmdfeed.dthta.fc[1];
                            FeedbackkData[11]=cmdfeed.dthta.fc[0];
                            
                            FeedbackkData[12]=(cmdfeed.vx>>8)&0xFF;
                            FeedbackkData[13]=cmdfeed.vx&0xFF;
                            FeedbackkData[14]=(cmdfeed.vthta>>8)&0xFF;
                            FeedbackkData[15]=cmdfeed.vthta&0xFF;
                            
//										//��������������״̬
//										if(BoardID==0)
                            {
                                FeedbackkData[16]=cmdfeed.leftDriver;
                                FeedbackkData[17]=cmdfeed.rightDriver;
                            }
                            
                            FeedbackkData[18]=cmdfeed.WORK_STATE.ROBOT_STATE;
                            FeedbackkData[19]=cmdfeed.collisions;									
                            FeedbackkData[20]=cmdfeed.loadSwitchs;									
                            FeedbackkData[21]=sys_config.state;
                            //FeedbackkData[22]=0x00;	
                            cmdfeed.workState  = LinearData.Linear_State.state ; //20170822,Zero                          
                            FeedbackkData[22]=cmdfeed.workState;											
                            //FeedbackkData[23]=0x00;
                            FeedbackkData[23] = cmdfeed.loadDriver;//cmdfeed.log;
                            
                            Usart2Feedback(CMDType,24,FeedbackkData);									
                            break;
                        
                    /****************** used for linar actuator *****************/									
                    case LINEAR_LOAD:
                        {
                            uint8_t cmd = Usart2TempBuff[3];
                            loadMotorState = cmd;
                            FeedbackkData[0] = cmdfeed.loadDriver ;
                            FeedbackkData[1] = loadMotorState;
                            FeedbackkData[2] = cmdfeed.loadSwitchs ;
                            Usart2Feedback(CMDType,3,FeedbackkData);
                       }
                            break;		
                    
                   /***************************************************************/		
                  /****************** used for read MPU9250 *****************/									
                    case READ_MPU9250:   
                    {     
                            int16_t Yaw,Roll,Pitch;
                            int16_t X_Out,Y_Out,Z_Out;
                            float x,y,z;
                        
                            X_Out = Uart4RevBuffer[3]*256+Uart4RevBuffer[4];
                            Y_Out = Uart4RevBuffer[5]*256+Uart4RevBuffer[6];
                            Z_Out = Uart4RevBuffer[7]*256+Uart4RevBuffer[8];
                        
                            x = X_Out * 360 / 65536.0;
                            y = Y_Out * 180 / 65536.0;
                            z = Z_Out * 360 / 65536.0;
                            printf("X_Out = %f ,Y_Out = %f , Z = %f \n",x,y,z);
//                        printf("X_Out = %f,Y_Out = %f , Z = %f \n",(float)X_Out/65536.0,(float)Y_Out/65536.0,(float)Z_Out/65536.0);

//                            Yaw = Uart4RevBuffer[3]*256+Uart4RevBuffer[4]+ Uart4RevBuffer[5]*256+Uart4RevBuffer[6];
//                            Roll = Uart4RevBuffer[7]*256+Uart4RevBuffer[8]+Uart4RevBuffer[9]*256+Uart4RevBuffer[10];
//                            Pitch = Uart4RevBuffer[11]*256+Uart4RevBuffer[12]+Uart4RevBuffer[13]*256+Uart4RevBuffer[14];
//                            memcpy(FeedbackkData,&Uart4RevBuffer[3],12);
//                          Usart2Feedback(CMDType,12,FeedbackkData);
//                          printf("yaw = %d ,z = %d ,roll = %d , y = %d , pitch = %d ,x = %d \n",Uart4RevBuffer[3]*256+Uart4RevBuffer[4],Uart4RevBuffer[5]*256+Uart4RevBuffer[6],\
                    Uart4RevBuffer[7]*256+Uart4RevBuffer[8],Uart4RevBuffer[9]*256+Uart4RevBuffer[10],Uart4RevBuffer[11]*256+Uart4RevBuffer[12],Uart4RevBuffer[13]*256+Uart4RevBuffer[14]);
//                            printf("Yaw = %d ,Roll = %d , Pitch = %d \n",Yaw,Roll,Pitch);
                    }
                          break;		
                    
                /***************************************************************/	
                       
                    
                        case READY2UPDATE:
                                switch(Usart2TempBuff[3])
                                {
                                    //����׼��
                                    case 0x00:
                                        EraseState=0;
                                        DataNum=0;
                                        memcpy(MD5Check,&Usart2TempBuff[4],16);
                                        DataSize = (Usart2TempBuff[20]<<24)+(Usart2TempBuff[21]<<16)+(Usart2TempBuff[22]<<8)+Usart2TempBuff[23];
                                        //������С
                                        if(DataSize>128*1024)
                                        {
                                            EraseState |= 0x01;
                                        }
                                        else
                                        {
                                            EraseState &= 0x0E;
//														FLASH_UnlockBank1();
                                            FLASH_Unlock();
                                            RCC_HSICmd(ENABLE);																									
                                            FLASH_SetLatency(FLASH_Latency_2);
                                            FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
//														FLASH_EraseOptionBytes();

                                            for(PageNum=0;PageNum<0x40;PageNum++)
                                            {
                                                IWDG_ReloadCounter();
                                                
                                                FlashEraseState=FLASH_ErasePage(UPDATEADDRESS+PageNum*0x800);
                                                if(FlashEraseState!=4)
                                                {
                                                    EraseState =0x02;
                                                    break;
                                                }
                                                else
                                                {
                                                    EraseState &=0x0D;
                                                }
                                            }				
                                            RCC_HSICmd(DISABLE);	
                                        }
                                        FeedbackkData[0] = Usart2TempBuff[3];
                                        FeedbackkData[1] = EraseState;
                                        Usart2Feedback(CMDType,2,FeedbackkData);	
                                        

                                        break;
                                    
                                    //�������ݰ�
                                    case 0x01:
                                        WriteState=0;
                                        DeltAdd = Usart2TempBuff[1]-6;	
                                        IWDG_ReloadCounter();										
//													OS_ENTER_CRITICAL();					
                                    //����ǰ���յ��������ϴε����ݽ��бȽ�
                                        if(memcmp(Usart2TempBuff,Usart2TempBuff2,255))
                                        {									
                                            //����д��
                                            for(i=0;i<DeltAdd;i+=2)
                                            {
                                                unsigned short tempData;														
                                                RCC_HSICmd(ENABLE);
                                                tempData = (Usart2TempBuff[i+5]<<8)+Usart2TempBuff[i+4];
                                                FlashWriteState=FLASH_ProgramHalfWord(UPDATEADDRESS+DataNum*2,tempData);																											
                                                if(FlashWriteState!= 4)
                                                {
                                                    DataNum-=i/2;
                                                    WriteState = 0x01;
//																WriteState |= FlashWriteState;
                                                    break;
                                                }
                                                else
                                                {
                                                    DataNum ++;
                                                    WriteState &= 0x0E;
                                                }
//														RCC_HSICmd(DISABLE);
                                            }
                                            
                                            if(DeltAdd%2)
                                            {
                                                unsigned short tempData;
//														RCC_HSICmd(ENABLE);
                                                tempData = (0xFF00)+Usart2TempBuff[i*2+4];
                                                FlashWriteState=FLASH_ProgramHalfWord(UPDATEADDRESS+DataNum*2,tempData);	
                                                if(FlashWriteState!= 4)
                                                {
                                                    DataNum-=i/2;
                                                    WriteState |= 0x02;
//																WriteState |= FlashWriteState;
                                                    break;
                                                }
                                                else
                                                {
                                                    DataNum ++;
                                                    WriteState &= 0x0D;
                                                }
                                                RCC_HSICmd(DISABLE);
                                            }
                                        //�����ݱ����������������´ν��յ����ݽ��бȽ�
                                            if(WriteState==0)
                                            {
                                                memcpy(Usart2TempBuff2,Usart2TempBuff,255);		
                                            }															
                                        }
                                        else
                                        {
                                            //�ַ����ظ�
                                           WriteState = 0x00;
                                        }
                                        FeedbackkData[0] = Usart2TempBuff[3];
                                        FeedbackkData[1] = WriteState;
                                        Usart2Feedback(CMDType,2,FeedbackkData);
//													OS_EXIT_CRITICAL();
                                                                                                    
                                        break;
                                    //�������
                                    case 0x02:

                                        UpdateFlag=0;
                                        DataNum=0;
                                        WriteState=0;
                                        Md5Initialise(&MD5Ctx);
                                        Md5Update( &MD5Ctx, (uint8_t *)UPDATEADDRESS, DataSize);
                                        Md5Finalise( &MD5Ctx, &md5_ret );
                                        if(memcmp(md5_ret.bytes, MD5Check, 16)!=0)
                                        {
                                                WriteState |= 0x01;																											
                                        }							
                                        else
                                        {
                                            WriteState &=0x0E;
                                            bootTable.boot_table_t.length = DataSize;
                                            bootTable.boot_table_t.start_address = UPDATEADDRESS;
                                            bootTable.boot_table_t.type = 'A';
                                            bootTable.boot_table_t.upgrade_type = 'U';
                                            RCC_HSICmd(ENABLE);
                                            //��������ҳ
                                            for(PageNum=0;PageNum<0x02;PageNum++)
                                            {
                                                IWDG_ReloadCounter();
                                                FlashWriteState=FLASH_ErasePage(PARAMSADDRESS+PageNum*0x800);
                                                if(FlashWriteState!= 4)
                                                {
                                                    DataNum-=i/2;
                                                    WriteState |= 0x02;
//																WriteState |= FlashWriteState;
                                                    break;
                                                }
                                                else
                                                {
                                                    WriteState &= 0x0D;
                                                }
                                            }
                                            
                                            if(WriteState==0x00)
                                            {
                                                //д������
                                                for(i=0;i<16;i+=4)
                                                {
                                                    unsigned int paraData;
                                                  paraData = (bootTable.data[i+3]<<24)+(bootTable.data[i+2]<<16)+(bootTable.data[i+1]<<8)+bootTable.data[i];
                                                    FlashWriteState=FLASH_ProgramWord(PARAMSADDRESS+DataNum*4,paraData);
                                                    if(FlashWriteState!= 4)
                                                    {
                                                        DataNum-=i/2;
                                                        WriteState |= 0x04;
//																	WriteState |= FlashWriteState;
                                                        break;
                                                    }
                                                    else
                                                    {
                                                        DataNum ++;
                                                        WriteState &= 0x0B;
                                                    }
                                                }
                                                
                                                for(i=16;i<24;i+=4)
                                                {
                                                    unsigned int paraData;
                                                  paraData = (bootTable.data[i+3]<<24)+(bootTable.data[i+2]<<16)+(bootTable.data[i+1]<<8)+bootTable.data[i];
                                                    FlashWriteState=FLASH_ProgramWord(PARAMSADDRESS+DataNum*4,paraData);
                                                    if(FlashWriteState!= 4)
                                                    {
                                                        DataNum-=i/2;
                                                        WriteState |= 0x08;
//																	WriteState |= FlashWriteState;
                                                        break;
                                                    }
                                                    else
                                                    {
                                                        DataNum ++;
                                                        WriteState &= 0x07;
                                                    }
                                                }																
                                            }	
                                            RCC_HSICmd(DISABLE);																					
                                        }	
                                        
                                        FeedbackkData[1] = WriteState;	
                                        FeedbackkData[0] = 0x02;												
                                        Usart2Feedback(CMDType,2,FeedbackkData);
                                        FLASH_Lock();
//													RCC_HSICmd(DISABLE);	
                                        DataNum = 0;													
                                                
                                        break;
                                                                        
                                    default:
                                        Usart2Feedback(CMDType,12,"UPDATE ERROR");
                                        break;
                                }						
                            break;
                                
                       /*********20170522,Zero*********************/
						case CHARGING_STATE:
                            if(Usart2TempBuff[3]==0)
                            {
                                ChargingFlag = 0;
                                ChargedFlag = 1;
                                FeedbackkData[0] = ChargedFlag ;				
                            }
                            else
                                if(Usart2TempBuff[3]==1)
                                {
                                    ChargingFlag = 1;
                                    ChargedFlag =  0 ;
                                    FeedbackkData[0] = ChargingFlag ;
                                }
                                Usart2Feedback(CMDType,1,FeedbackkData);
                            break;
						/********************************************/
                                
                        case GETVERSION:
                            memcpy(FeedbackkData,HardwareVer,4);
                            memcpy(&FeedbackkData[4],SoftVer,15);
                            Usart2Feedback(CMDType,19,FeedbackkData);							
                            break;
                        default:
                          Usart2Feedback(CMDERROR,9,"CMD ERROR");		
                            break;														
                    }				
                }
				
//			usedTime2 = OSTimeGet()  ;		
//			printf("usedTime2 = %d \n" ,usedTime2);
		}
		else
		{
			
				mPara.vx=0;
				mPara.vthta=0;										
				gMotiondata.vx=mPara.vx;
				gMotiondata.vthta=mPara.vthta;
	//			OSQPost(MotionCtrlQBOX,(void*)&mPara);	
		}

	}
	
}

/*---------------------------The End of File-------------------------------*/
