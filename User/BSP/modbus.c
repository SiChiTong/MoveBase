/**-------------------------------------------------------------------------
*@ file				modbus.c
*@ brief			motor driver
*@ author 		ZGJ
*@ vertion		V0.01
*@ date				2015-12-15
*@ statement	(C)	COPYRIGHT 2015 KSITRI.
--------------------------------------------------------------------------*/

#include "modbus.h"
#include "usart.h"
#include "task_cfg.h"
#include "ucos_ii.h"
#include "common.h"
#include "motor.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
/* CRC 高位字节值表 */ 
#define OVER_TIMES  8

const unsigned char auchCRCHi[] = { 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40 
} ; 
/* CRC低位字节值表*/ 
const unsigned char auchCRCLo[] = { 
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 
0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 
0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 
0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 
0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 
0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 
0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 
0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 
0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 
0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 
0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 
0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 
0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 
0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 
0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 
0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 
0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 
0x43, 0x83, 0x41, 0x81, 0x80, 0x40 
} ;

unsigned short m_nTimeOut=0;
static unsigned char ltimeout_cnt=0;
static unsigned char rtimeout_cnt=0;
static unsigned char motor_flag=0;
unsigned int vl_cur,vr_cur;
char CleaerLeftErrFlag=0;
char CleaerRightErrFlag=0;

GETCURRENT_STRUCT L_Driver;
GETCURRENT_STRUCT R_Driver;
//PID_STRUCT PID_Data;

void SetTimeOut(unsigned short dwMs)
{
	m_nTimeOut = dwMs;
}

/*
unsigned char* FindFrame(unsigned char addr, unsigned char *pBuf, unsigned short wLen, unsigned short *pwLen)
{
	//unsigned char bRet = FALSE;
	unsigned char bFind = 0;
	unsigned char *pFind = NULL;
	unsigned short i, j, wFindLen = 0;
	if (wLen >= 4) {
		unsigned char *pCrcLo = new unsigned char[wLen];
		unsigned char *pCrcHi = new unsigned char[wLen];
		
		for (i = 0; i < wLen - 4; i++) {
			if (pBuf[i] != addr) {
				continue;
			}
			CalcCrc16(pBuf + i, wLen - i, pCrcHi + i, pCrcLo + i);
			for (j = i; j < wLen - 2; j++) {
				if (pCrcHi[j] == pBuf[j + 1] && pCrcLo[j] == pBuf[j + 2]) {
					bFind = TRUE;
					pFind = pBuf + i;
					wFindLen = j + 3;
					break;
				}
			}
			if (bFind) break;
		}

		delete[] pCrcLo;
		delete[] pCrcHi;
	}
	if (pwLen) *pwLen = wFindLen;

	return pFind;
}
*/

unsigned short CalcCrc16(unsigned char *pBuf, unsigned short dwLen, unsigned char *pCRCHi, unsigned char *pCRCLo)
{
	 unsigned char i, index, bCRCLo = 0xff, bCRCHi = 0xff;

	 for (i = 0; i < dwLen; i++) {
	    index = bCRCHi ^ *pBuf++;
	    bCRCHi = bCRCLo ^ auchCRCHi[index];
	    bCRCLo = auchCRCLo[index];
		if (pCRCHi) *pCRCHi++ = bCRCHi;
		if (pCRCLo) *pCRCLo++ = bCRCLo;
	 }

	 return bCRCLo | (bCRCHi << 8);
}

MDRESULT ReadRegs(unsigned char bDevAddr, unsigned short wRegAddr, unsigned short wRegNum, unsigned char *pRecvBuf, unsigned short *pwBufSize)
{
	unsigned char buf[64];
	unsigned char chk = 0;
//	unsigned short i;
	unsigned short val;
	unsigned char bRet;
	MDRESULT result = MDERR_SUCCESS;
	unsigned short m_nSendLen=0;
	unsigned char err=0;
	unsigned char *pFind=Uart4RevBuffer;
	int nBufLen = 0;
	
	Delayus(1);
	RS485DataOut();
	Delayus(1);
	buf[0] = bDevAddr;
	buf[1] = FUNC_READMULTIREG;
	buf[2] = wRegAddr >> 8;
	buf[3] = (unsigned char)wRegAddr;
	buf[4] = wRegNum >> 8;
	buf[5] = (unsigned char)wRegNum;
	val = CalcCrc16(buf, 6, (unsigned char*)NULL,(unsigned char *)NULL);
	buf[6] = val >> 8;
	buf[7] = (unsigned char)val;
	m_nSendLen = 8;

	Usart_SendBuffer(MODBUS_UART,buf,m_nSendLen);
	
	RS485DataIn();
	bRet = *((unsigned char*)OSQPend(ModBusRevQBOX,30,&err));
	if (err==OS_ERR_NONE) {
		if (IsFrame(bDevAddr, pFind, bRet) && 
			pFind[0] == bDevAddr && (pFind[1] & 0x7f) == FUNC_READMULTIREG) { // 判断是否帧存在，并且从机地址和返回的命令与发送的一致
			if (pFind[1] & 0x80) {	// 错误信息
				result = (MDRESULT)pFind[2];
			} else {
				if (pRecvBuf && pwBufSize) {
					nBufLen = *pwBufSize;
					if (nBufLen > pFind[2]) {
						nBufLen = pFind[2];
					}
					memcpy(pRecvBuf, pFind + 3, nBufLen);
					*pwBufSize = nBufLen;
				}
			}
		} else {
			result = MDERR_COMM;
		}
		if(motor_flag==0)
		{
			ltimeout_cnt=0;
			if(CleaerLeftErrFlag==1)
			{
				cmdfeed.LEFT.MOTOR_DRIVER_STRUCT.connect = 0;
//				CleaerLeftErrFlag=0;
			}
		}
		else if(motor_flag==1)
		{
			rtimeout_cnt=0;
			if(CleaerRightErrFlag==1)
			{
			cmdfeed.RIGHT.MOTOR_DRIVER_STRUCT.connect = 0;
//				CleaerRightErrFlag=0;
			}
		}
	} else {
		if(motor_flag==0)
		{
			ltimeout_cnt++;
		#if NEW_FEEDBACK
			if( ltimeout_cnt >= OVER_TIMES)
			{
//				ltimeout_cnt = 0;
			  cmdfeed.LEFT.MOTOR_DRIVER_STRUCT.connect = 1;	
			}
		#endif
		}
		else if(motor_flag==1)
		{
			rtimeout_cnt++;
		#if NEW_FEEDBACK
			if(rtimeout_cnt >= OVER_TIMES)
			{
//				rtimeout_cnt = 0;
				
				cmdfeed.RIGHT.MOTOR_DRIVER_STRUCT.connect = 1;
			}
		#endif
		}
		result = MDERR_TIMEOUT;

		
	#if NEW_FEEDBACK
		if((cmdfeed.LEFT.MOTOR_DRIVER_STRUCT.connect==1)||(cmdfeed.RIGHT.MOTOR_DRIVER_STRUCT.connect==1))
		{
			OSQPost(EHandlerQBOX,(void*)DRIVER_COMMU_ERR);
		}
	#endif
	}	
	return result;
}

MDRESULT WriteSingleReg(unsigned char bDevAddr, unsigned short wRegAddr, unsigned short wRegValue, unsigned short *pwActAddr, unsigned short *pwActValue)
{

	unsigned char buf[64];
	unsigned char chk = 0;
//	unsigned short i;
	unsigned short val;
	unsigned char bRet;
	MDRESULT result = MDERR_SUCCESS;
	unsigned char m_nSendLen=0;
	unsigned char err=0;
	unsigned char *pFind=Uart4RevBuffer;
	
	Delayus(1);
	RS485DataOut();
	Delayus(1);
	buf[0] = bDevAddr;
	buf[1] = FUNC_WRITESINGLEREG;
	buf[2] = wRegAddr >> 8;
	buf[3] = (unsigned char)wRegAddr;
	buf[4] = wRegValue >> 8;
	buf[5] = (unsigned char)wRegValue;
	val = CalcCrc16(buf, 6, (void*)NULL,(void*)NULL);
	buf[6] = val >> 8;
	buf[7] = (unsigned char)val;
	m_nSendLen = 8;

	Usart_SendBuffer(MODBUS_UART,buf,m_nSendLen);

	
	RS485DataIn();
	bRet = *((unsigned char*)OSQPend(ModBusRevQBOX,50,&err));
	if (err==OS_ERR_NONE) {
		val = 0;
		if (IsFrame(bDevAddr, pFind, bRet) && 
			pFind[0] == bDevAddr && (pFind[1] & 0x7f) == FUNC_WRITESINGLEREG) { // 判断是否帧存在，并且从机地址和返回的命令与发送的一致
			if (pFind[1] & 0x80) {	// 错误信息
				result = (MDRESULT)pFind[2];
			} else {
				if (pwActAddr) {
					*pwActAddr = (pFind[2] << 8) | pFind[3];
				}
				if (pwActValue) {
					*pwActValue = (pFind[4] << 8) | pFind[5];
				}
			}
		} else {
			result = MDERR_COMM;
		}
		if(motor_flag==0)
		{
			ltimeout_cnt=0;
			if(CleaerLeftErrFlag==1)
			{
				cmdfeed.LEFT.MOTOR_DRIVER_STRUCT.connect = 0;
//				CleaerLeftErrFlag=0;
			}
		}
		else if(motor_flag==1)
		{
			rtimeout_cnt=0;
			if(CleaerRightErrFlag==1)
			{
			cmdfeed.RIGHT.MOTOR_DRIVER_STRUCT.connect = 0;
//				CleaerRightErrFlag=0;
			}
		}
	} else {
		if(motor_flag==0)
		{
			ltimeout_cnt++;
		#if NEW_FEEDBACK
			if( ltimeout_cnt >= OVER_TIMES)
			{
//				ltimeout_cnt = 0;
			  cmdfeed.LEFT.MOTOR_DRIVER_STRUCT.connect = 1;
			}
		#endif
		}
		else if(motor_flag==1)
		{
			rtimeout_cnt++;
		#if NEW_FEEDBACK
			if(rtimeout_cnt >= OVER_TIMES)
			{
//				rtimeout_cnt = 0;
				cmdfeed.RIGHT.MOTOR_DRIVER_STRUCT.connect = 1;
			}
		#endif
		}
		result = MDERR_TIMEOUT;

		
	#if NEW_FEEDBACK
		if((cmdfeed.LEFT.MOTOR_DRIVER_STRUCT.connect==1)||(cmdfeed.RIGHT.MOTOR_DRIVER_STRUCT.connect==1))
		{
			OSQPost(EHandlerQBOX,(void*)DRIVER_COMMU_ERR);
		}
	#endif
	}	
	return result;	
}

MDRESULT WriteMultiRegs(unsigned char bDevAddr, unsigned short wRegAddr, unsigned short *pwRegBuf, unsigned short wRegNum, unsigned short *pwActAddr, unsigned short *pwActNum)
{
	unsigned char buf[64];
	unsigned char chk = 0;
	unsigned short i;
	unsigned short val;
	unsigned char bRet;
	MDRESULT result = MDERR_SUCCESS;
	unsigned char m_nSendLen=0;
	unsigned char err=0;
	unsigned char *pFind=Uart4RevBuffer;
	
	Delayus(1);
	RS485DataOut();
	Delayus(1);
	buf[0] = bDevAddr;
	buf[1] = FUNC_WRITEMULTIREG;
	buf[2] = wRegAddr >> 8;
	buf[3] = (unsigned char)wRegAddr;
	buf[4] = wRegNum >> 8;
	buf[5] = (unsigned char)wRegNum;
	buf[6] = wRegNum * 2;
	for (i = 0; i < wRegNum; i++) {
		*(unsigned short *)(buf + i * 2 + 7) = ((pwRegBuf[i] & 0xff00) >> 8) | ((pwRegBuf[i] & 0x00ff) << 8); 
	}
	val = CalcCrc16(buf, wRegNum * 2 + 7,(void*)NULL,(void*)NULL);
	buf[wRegNum * 2 + 7] = val >> 8;
	buf[wRegNum * 2 + 8] = (unsigned char)val;
	m_nSendLen = wRegNum * 2 + 9;
	
	Usart_SendBuffer(MODBUS_UART,buf,m_nSendLen);

	
	RS485DataIn();
	bRet = *((unsigned char*)OSQPend(ModBusRevQBOX,30,&err));
	if (err==OS_ERR_NONE) {
		if (IsFrame(bDevAddr, pFind,bRet) && 
			pFind[0] == bDevAddr && (pFind[1] & 0x7f) == FUNC_WRITEMULTIREG) { // 判断是否帧存在，并且从机地址和返回的命令与发送的一致
			if (pFind[1] & 0x80) {	// 错误信息
				result = (MDRESULT)pFind[2];
			} else {
				if (pwActAddr) {
					*pwActAddr = (pFind[2] << 8) | pFind[3];
				}
				if (pwActNum) {
					*pwActNum = (pFind[4] << 8) | pFind[5];
				}
			}
		} else {
			result = MDERR_COMM;
		}
		if(motor_flag==0)
		{
			ltimeout_cnt=0;
			if(CleaerLeftErrFlag==1)
			{
				cmdfeed.LEFT.MOTOR_DRIVER_STRUCT.connect = 0;
//				CleaerLeftErrFlag=0;
			}
		}
		else if(motor_flag==1)
		{
			rtimeout_cnt=0;
			if(CleaerRightErrFlag==1)
			{
				cmdfeed.RIGHT.MOTOR_DRIVER_STRUCT.connect = 0;
//				CleaerRightErrFlag=0;
			}
		}
	} else {
		if(motor_flag==0)
		{
			ltimeout_cnt++;
			#if NEW_FEEDBACK
			if( ltimeout_cnt >= OVER_TIMES)
			{
//				ltimeout_cnt = 0;
			  cmdfeed.LEFT.MOTOR_DRIVER_STRUCT.connect = 1;
			}
			#endif
		}
		else if(motor_flag==1)
		{
			rtimeout_cnt++;
			#if NEW_FEEDBACK
			if(rtimeout_cnt >= OVER_TIMES)
			{
//				rtimeout_cnt = 0;
				cmdfeed.RIGHT.MOTOR_DRIVER_STRUCT.connect = 1;
			}
			#endif
		}
		result = MDERR_TIMEOUT;

		
	#if NEW_FEEDBACK
		if((cmdfeed.LEFT.MOTOR_DRIVER_STRUCT.connect==1)||(cmdfeed.RIGHT.MOTOR_DRIVER_STRUCT.connect==1))
		{
			OSQPost(EHandlerQBOX,(void*)DRIVER_COMMU_ERR);
		}
	#endif
	}	
	return result;	
}

unsigned char IsFrame(unsigned char addr, unsigned char *pBuf, unsigned short wLen)
{
	unsigned char bRet = 0;
	unsigned short wCrcLen = wLen - 2;
	unsigned char pCrcLo[64]={0};
	unsigned char pCrcHi[64]={0};

	if (wLen >= 4 && pBuf[0] == addr) {
		
		CalcCrc16(pBuf, wLen - 2, pCrcHi, pCrcLo);
		if (pCrcHi[wCrcLen - 1] == pBuf[wLen - 2] && pCrcLo[wCrcLen - 1] == pBuf[wLen - 1]) {
			bRet = 1;
		}
	}

	return bRet;
}

MDRESULT ReadDevId(unsigned char bDevAddr, unsigned char bDevIdCode, unsigned char bObjectId, unsigned char *pRecvBuf, unsigned short *pwBufSize)
{
	unsigned char buf[64];
	unsigned char chk = 0;
//	unsigned short i;
	unsigned short val;
	unsigned char bRet;
	MDRESULT result = MDERR_SUCCESS;
	unsigned char m_nSendLen=0;
	unsigned char err=0;
	unsigned char *pFind=Uart4RevBuffer;

	Delayus(1);
	RS485DataOut();
	Delayus(1);
	buf[0] = bDevAddr;
	buf[1] = FUNC_READDEVID;
	buf[2] = 0x0e;
	buf[3] = bDevIdCode;
	buf[4] = bObjectId;
	val = CalcCrc16(buf, 5,(void*)NULL,(void*)NULL);
	buf[5] = val >> 8;
	buf[6] = (unsigned char)val;
	m_nSendLen = 7;
	
	Usart_SendBuffer(MODBUS_UART,buf,m_nSendLen);


	RS485DataIn();
	bRet = *((unsigned char*)OSQPend(ModBusRevQBOX,30,&err));
	
	if (err==OS_ERR_NONE) {
		if (IsFrame(bDevAddr, pFind, bRet) && 
			pFind[0] == bDevAddr && (pFind[1] & 0x7f) == FUNC_READDEVID) { // 判断是否帧存在，并且从机地址和返回的命令与发送的一致
			if (pFind[1] & 0x80) {	// 错误信息
				result = (MDRESULT)pFind[2];
			} else {
				if (pRecvBuf && pwBufSize) {
					int nBufLen = *pwBufSize;
					if (nBufLen > bRet - 7) {
						nBufLen = bRet- 7;
					}
					memcpy(pRecvBuf, pFind + 5, nBufLen);
					*pwBufSize = nBufLen;
				}
			}
		} else {
			result = MDERR_COMM;
		}
		if(motor_flag==0)
		{
			ltimeout_cnt=0;
		}
		else if(motor_flag==1)
		{
			rtimeout_cnt=0;
		}
	} else {
		if(motor_flag==0)
		{
			ltimeout_cnt++;
			#if NEW_FEEDBACK
			if( ltimeout_cnt >= OVER_TIMES)
			{
//				ltimeout_cnt = 0;
			  cmdfeed.LEFT.MOTOR_DRIVER_STRUCT.connect = 1;
			}
			#endif
		}
		else if(motor_flag==1)
		{
			rtimeout_cnt++;
			#if NEW_FEEDBACK
			if(rtimeout_cnt >= OVER_TIMES)
			{
//				rtimeout_cnt = 0;
				cmdfeed.RIGHT.MOTOR_DRIVER_STRUCT.connect = 1;
			}
			#endif
		}
		result = MDERR_TIMEOUT;

		
	#if NEW_FEEDBACK
		if((cmdfeed.LEFT.MOTOR_DRIVER_STRUCT.connect==1)||(cmdfeed.RIGHT.MOTOR_DRIVER_STRUCT.connect==1))
		{
			OSQPost(EHandlerQBOX,(void*)DRIVER_COMMU_ERR);
		}
	#endif
	}	
	return result;
}

// @brief ???????,??1,ref: AQMD6020BLS_UM_V0.91.pdf, p104,?4.57
int DriverConfig(void)
{
		unsigned char i=0;
    int ret;
		unsigned char slaveAddr=0;
    for (i = 0; i < 2; i++)				
    {
				OSTimeDly(10);
				slaveAddr=0x01+ i;
				if(slaveAddr==1)
				{
					motor_flag=0;
				}
				else if(slaveAddr==2)
				{
					motor_flag=1;
				}
				//工作模式位置闭环
        ret = WriteSingleReg(slaveAddr, 0x00b0, 3,(unsigned short*)NULL,(unsigned short*)NULL);

				ret = WriteSingleReg(slaveAddr, 0x00b0, 3,(unsigned short*)NULL,(unsigned short*)NULL);

        /* ??????,ref: AQMD6020BLS_UM_V0.91.pdf, p122,6.3.4 */
        // ????????????????
        // ret = modbus_write_register(modbus_ctx_, 0x0060, 50);

        // ????????????????
        // ret = modbus_write_register(modbus_ctx_, 0x0061, 50);

        // ????????????????????
        // ret = modbus_write_register(modbus_ctx_, 0x0062, 100);

        // 初始加速度
        ret = WriteSingleReg(slaveAddr, 0x0063, 1000,(unsigned short*)NULL,NULL);

        // ????????????????????
        // ret = modbus_write_register(modbus_ctx_, 0x0064, 100);

        // 初始减速加速度
        ret = WriteSingleReg(slaveAddr, 0x0065, 1500,NULL,NULL);

        // 最大速度
        ret = WriteSingleReg(slaveAddr, 0x0066, 2400,NULL,NULL);		//最大速度  2367

        // ?????????/??????????(????)
        // ret = modbus_write_register(modbus_ctx_, 0x0067, 5000);

        // 停止方式:正常停止方式
//        ret = WriteSingleReg(slaveAddr, 0x0040, 0,NULL,NULL);

        // ?????
        // ret = modbus_write_register(modbus_ctx_, 0x0042, 500);

        // 速度闭环控制目标
        ret = WriteSingleReg(slaveAddr, 0x0043, 0, NULL, NULL);

        // 位置闭环控制目标
        ret = WriteSingleReg(slaveAddr, 0x0044, 0,NULL,NULL);

        // 位置闭环控制类型，0-绝对位置，1-相对位置
        ret = WriteSingleReg(slaveAddr, 0x0045, 1,NULL,NULL);

        // ????????????,??
        // ret = modbus_write_register(modbus_ctx_, 0x0046, 0);

        // ????????????,??
        // ret = modbus_write_register(modbus_ctx_, 0x0047, 10000);

        // ???????????????
        // ret = modbus_write_register(modbus_ctx_, 0x0050, 50);

        // ???????????????
        // ret = modbus_write_register(modbus_ctx_, 0x0051, 50);

        // 加速加速度，100
        ret = WriteSingleReg(slaveAddr, 0x0052, 1000,NULL,NULL);

        // 减速加速度,调整刹车距离，值越大，刹车距离越短，150
        ret = WriteSingleReg(slaveAddr, 0x0053, 1500,NULL,NULL);

        //常态自锁电流额定电流,*0.01A
        ret = WriteSingleReg(slaveAddr, 0x0068, 100,NULL,NULL);

        // 额定电流,*0.01A
        ret = WriteSingleReg(slaveAddr, 0x006a, 1400,NULL,NULL);

        // 最大负载电流
        ret = WriteSingleReg(slaveAddr, 0x006b, 1400,NULL,NULL);

        // 最大制动电流
        ret = WriteSingleReg(slaveAddr, 0x006c, 600,NULL,NULL);

        // 速度闭环控制算法,0:速度闭环,1:时间-位置闭环,2--时间-位置速率
        ret = WriteSingleReg(slaveAddr, 0x0070, 2,NULL,NULL);	//1,0

        // ??????????
       // ret = WriteSingleReg(slaveAddr, 0x0071, 1000,NULL,NULL);

        // 位置闭环超调后修正，0-不修正，1-修正
        ret = WriteSingleReg(slaveAddr, 0x0072, 0,NULL,NULL);

        // 电机极个数
        ret = WriteSingleReg(slaveAddr, 0x0073, 20,NULL,NULL);

        // 电机减速比
        ret = WriteSingleReg(slaveAddr, 0x0074, 10,NULL,NULL);
				//通信中断停止时间 500ms
				ret = WriteSingleReg(slaveAddr, 0x0095, 5,NULL,NULL);		//5
				//堵转停止时间 500ms
				ret = WriteSingleReg(slaveAddr, 0x008E, 5,NULL,NULL);
				
				//禁用驱动器报警
				ret = WriteSingleReg(slaveAddr, 0x0099, 1,NULL,NULL);
				//保存参数
				ret = WriteSingleReg(slaveAddr, 0x0160, 1,NULL,NULL);
				
				ret = WriteSingleReg(slaveAddr, 0x0180, 1,NULL,NULL);
    }
}

#if NEW_FEEDBACK
void setMotorSpeed(float sp_l, float sp_r)
{
		unsigned char i=0;  // 换相频率设定值=(转速(RPM)*电机极数)*10/20
		unsigned char slaveAddr=0;
		short speed=0;
		float sw_freq[2]={0};
		unsigned short write_buf_l[4]={0};
		unsigned short write_buf_r[4]={0};
		unsigned char kMotorPolarNum=20;
		unsigned char read_buf[2]={0};
		unsigned short read_len=2;
		unsigned short read_data=0;
		float kPi=3.141;
		MDRESULT	ret=MDERR_SUCCESS;
		short vl,vr;
	//	unsigned int vl_cur,vr_cur;
		static unsigned int prevl=0;
		static unsigned int prevr=0;
		static unsigned char cntl=0;
		static unsigned char cntr=0;
		
		unsigned char lfreq[2],rfreq[2];
	   unsigned short len=2;
	   signed short lspeed,rspeed;
		short l_cur,r_cur;
		
		Delayus(1000);

		//限速保护

		
		sw_freq[0] = sp_l *600;			//sp_l,sp_r,:单位转每秒，(RPS)
		sw_freq[1] = -sp_r * 600;

		vl=(short)sw_freq[0];
		vr=(short)sw_freq[1];


		
		
		if(cmdfeed.LEFT.MOTOR_DRIVER_STRUCT.connect != 1)
		{
			motor_flag=0;
			ret=WriteSingleReg(0x01, 0x0043, vl,NULL,NULL);
		}
		
		Delayus(1500);
		if(cmdfeed.RIGHT.MOTOR_DRIVER_STRUCT.connect != 1)
		{
			motor_flag=1;
			ret=WriteSingleReg(0x02, 0x0043, vr,NULL,NULL);
		}
			
		if(cmdfeed.RIGHT.MOTOR_DRIVER_STRUCT.connect != 1)
		{
			motor_flag=1;
		//读取实时状态
		ret=ReadRegs(0x02, 0x0033, 1,read_buf,&read_len);
		if(ret==MDERR_SUCCESS)
		{
			read_data=((read_buf[0]<<8) + read_buf[1]);
			switch(read_data)
			{
				case DRIVER_ERR_NONE:
					if(CleaerRightErrFlag==1)
					{
						cmdfeed.RIGHT.MOTOR_DRIVER_STATE=0;
//						CleaerRightErrFlag=0;
					}
					break;
				case DRIVER_NO_SPEED:		//堵转停止
				{
					cmdfeed.RIGHT.MOTOR_DRIVER_STRUCT.block = 1;
					cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.normal = 0;
					OSQPost(EHandlerQBOX,(void*)MOTOR_NO_SPEED);
				}
				break;
				case DRIVER_HOWER_ERR:	//霍尔错误
				{
					cmdfeed.RIGHT.MOTOR_DRIVER_STRUCT.hall = 1 ;
					cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.normal = 0;
					OSQPost(EHandlerQBOX,(void*)DRIVER_HALL_ERR);
				}
				break;
				case DRIVER_SPEED_NOTACHIVE:			//达不到指定速度
				{
					cmdfeed.RIGHT.MOTOR_DRIVER_STRUCT.target_speed = 1 ;
					cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.normal = 0;
					OSQPost(EHandlerQBOX,(void*)MOTOR_WINDING_OPEN);
				}
				break;
				
				case DRIVER_WINDING_OPEN:
				{
					cmdfeed.RIGHT.MOTOR_DRIVER_STRUCT.winding_open = 1 ;
					cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.normal = 0;
					OSQPost(EHandlerQBOX,(void*)MOTOR_WINDING_OPEN);
				}
					break;
				default:
					break;
			}
		}
		if(cmdfeed.RIGHT.MOTOR_DRIVER_STRUCT.connect != 1)
		{
			ret=ReadRegs(0x02, 0x0035, 1,read_buf,&read_len);
			if(ret==MDERR_SUCCESS)
			{
				vr_cur=((read_buf[0]<<8) + read_buf[1]);		//实时转速
				ret=ReadRegs(0x02, 0x0034, 1,read_buf,&read_len);
				if(ret==MDERR_SUCCESS)
				{
					read_data=((read_buf[0]<<8) + read_buf[1]);
					if(vr_cur==1)
					{
						vr_cur=read_data*10;		//RPM
					}
					else
					{
						vr_cur=read_data;
					}
					if((fabs(gEncoderCycCnt.R_EncoderCycCnt*50*60*10)<(vr_cur*1024))&&(vr_cur+400>prevr))		//50%
					{
						if(cntr<10)
						{
							cntr++;
						}
						else
						{
							cmdfeed.RIGHT.MOTOR_DRIVER_STRUCT.encoder = 1;
							cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.normal = 0;
							OSQPost(EHandlerQBOX,(void*)ENCODER_ERR);
						}
						
					}
					else
					{
						cntr=0;
						if(CleaerRightErrFlag==1)
							{
								cmdfeed.RIGHT.MOTOR_DRIVER_STRUCT.encoder = 0;
							}
					}
					prevr=vr_cur;
				}
			}
		}
	}
		
	Delayus(1500);
	if(cmdfeed.LEFT.MOTOR_DRIVER_STRUCT.connect != 1)
		{
			motor_flag=0;
		//读取实时状态
		ret=ReadRegs(0x01, 0x0033, 1,read_buf,&read_len);
		if(ret==MDERR_SUCCESS)
		{
			read_data=((read_buf[0]<<8) + read_buf[1]);
			switch(read_data)
			{
				case DRIVER_ERR_NONE:
					if(CleaerLeftErrFlag==1)
					{
						cmdfeed.LEFT.MOTOR_DRIVER_STATE=0;
//						CleaerLeftErrFlag=0;
					}
					break;
				case DRIVER_NO_SPEED:		//堵转停止
				{
					cmdfeed.LEFT.MOTOR_DRIVER_STRUCT.block = 1;
					cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.normal = 0;
					OSQPost(EHandlerQBOX,(void*)MOTOR_NO_SPEED);
				}
				break;
				case DRIVER_HOWER_ERR:	//霍尔错误
				{
					cmdfeed.LEFT.MOTOR_DRIVER_STRUCT.hall = 1 ;
					cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.normal = 0;
					OSQPost(EHandlerQBOX,(void*)DRIVER_HALL_ERR);
				}
				break;
				case DRIVER_SPEED_NOTACHIVE:			//达不到指定速度
				{
					cmdfeed.LEFT.MOTOR_DRIVER_STRUCT.target_speed = 1 ;
					cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.normal = 0;
					OSQPost(EHandlerQBOX,(void*)MOTOR_WINDING_OPEN);
				}
				break;
				case DRIVER_WINDING_OPEN:
				{
					cmdfeed.LEFT.MOTOR_DRIVER_STRUCT.winding_open = 1 ;
					cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.normal = 0;
					OSQPost(EHandlerQBOX,(void*)MOTOR_WINDING_OPEN);
				}
					break;
				default:
					break;
			}
		}
		if(cmdfeed.LEFT.MOTOR_DRIVER_STRUCT.connect != 1)
		{
			ret=ReadRegs(0x01, 0x0035, 1,read_buf,&read_len);
			if(ret==MDERR_SUCCESS)
			{
				vl_cur=((read_buf[0]<<8) + read_buf[1]);		//实时转速
				ret=ReadRegs(0x01, 0x0034, 1,read_buf,&read_len);
				if(ret==MDERR_SUCCESS)
				{
					read_data=((read_buf[0]<<8) + read_buf[1]);
					if(vl_cur==1)
					{
						vl_cur=read_data*10;		//RPM
					}
					else
					{
						vl_cur=read_data;
					}
					if((fabs(gEncoderCycCnt.L_EncoderCycCnt*50*60*10)<(vl_cur*1024))&&(vl_cur+400>prevr))		//50%
					{
						if(cntl<10)
						{
							cntl++;
						}
						else
						{
							cmdfeed.LEFT.MOTOR_DRIVER_STRUCT.encoder = 1;
							cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.normal = 0;
							OSQPost(EHandlerQBOX,(void*)ENCODER_ERR);
						}
						
					}
					else
					{
						cntl=0;
						if(CleaerLeftErrFlag==1)
							{
								cmdfeed.LEFT.MOTOR_DRIVER_STRUCT.encoder = 0;
							}
					}
					prevl=vl_cur;
				}
			}
		}
		//20160721,Zero
		l_cur=GetCurrent(0x01);
		L_Driver.Current_I=l_cur;
		L_Driver.Delt_I=l_cur-L_Driver.Current_I;
		
		r_cur=GetCurrent(0x02);
		R_Driver.Current_I=r_cur;
		R_Driver.Delt_I=r_cur-R_Driver.Current_I;
		
		if(L_Driver.Delt_I>100||R_Driver.Delt_I>100)
			cmdfeed.exception |= 0x02;
	  else
			cmdfeed.exception &= 0xFC;
		
		
	}	
}
#endif

// @brief 电机停止函数
// @param[in] stop_mode 0:正常停止,1:紧急制动,2:自由停止
void stopMotor(short stop_mode)
{
		unsigned char i=0;
		unsigned char slaveAddr=0;
		MDRESULT	ret=MDERR_SUCCESS;
		unsigned char err=0;
		unsigned short buf[4]={0x1388,0x0001,0x0000,0x0000};
		static unsigned char cnt=0;
		short l_cur,r_cur;
		char state;
		
		Delayus(10);
    for (i = 0; i < 2; i++)		
    {
				slaveAddr=0x01+i;

				if(slaveAddr==1)
				{
					motor_flag=0;
				}
				else if(slaveAddr==2)
				{
					motor_flag=1;
				}
        switch (stop_mode)
        {
            case 0:
                ret = WriteSingleReg(slaveAddr, 0x0040, 0,NULL,NULL);				
            break;
            case 1:
                ret = WriteSingleReg(slaveAddr, 0x0040, 1,NULL,NULL);
            break;
            case 2:
                ret = WriteSingleReg(slaveAddr, 0x0040, 2,NULL,NULL);
            break;
            default:
            break;
        }
    }
		//
		if((CleaerLeftErrFlag==1)||(CleaerRightErrFlag==1))
		{
			//清编码器、电机过温错误位
			if(cmdfeed.LEFT.MOTOR_DRIVER_STATE!=0)
			{
				state = GetDriverState(0x01);
				if(state==0)
          cmdfeed.LEFT.MOTOR_DRIVER_STATE = 0;
			}
			if(cmdfeed.RIGHT.MOTOR_DRIVER_STATE!=0)
			{
				state = GetDriverState(0x02);
				if(state==0)
					cmdfeed.RIGHT.MOTOR_DRIVER_STATE = 0;
			}
		}
		
	
		OSSemPost(StopSem);
		
		#if 0
		OSSemSet(MoveLockSem,0,&err);
		OSSemSet(StopLockSem,0,&err);
		cnt=0;
		if(stop_mode!=1)
		{
			OSSemPost(StopSem);
			while(1)
			{
				cnt++;
				if(OSSemAccept(MoveLockSem)>0)
				{
					cnt=0;
					break;
				}
				if(OSSemAccept(StopEncoderSem)>0)
				{
					break;
				}
				if(cnt>=150)
				{
					break;
				}				
				OSTimeDly(2);
			}
			if(cnt>0)
			{
				motor_flag=0;
				ret = WriteSingleReg(0x01, 0x0040, 1,NULL,NULL);
				Delayus(1500);
				motor_flag=1;
				ret = WriteSingleReg(0x02, 0x0040, 1,NULL,NULL);
			}
		}
		else
		{
			cnt=1;
		}
		
		if(cnt>0)
		{
			cnt=0;
			while(1)
			{
				cnt++;
				if(OSSemAccept(MoveLockSem)>0)
				{
					cnt=0;
					break;
				}
				if(OSSemAccept(StopLockSem)>0)
				{
					break;
				}
				if(cnt>=250)
				{
					break;
				}
				OSTimeDly(2);
			}
			if(cnt!=0)
			{
				motor_flag=0;
				ret=WriteMultiRegs(0x01,0x0044,buf,0x04,NULL,NULL);
				Delayus(1500);
				motor_flag=1;
				ret=WriteMultiRegs(0x02,0x0044,buf,0x04,NULL,NULL);
			}
			cnt=0;
		}	
		#endif
}

void LockMotor(void)
{
	MDRESULT	ret=MDERR_SUCCESS;
	unsigned short buf[4]={0x1388,0x0001,0x0000,0x0000};
	motor_flag=0;
	ret=WriteMultiRegs(0x01,0x0044,buf,0x04,NULL,NULL);
	Delayus(1000);
	motor_flag=1;
	ret=WriteMultiRegs(0x02,0x0044,buf,0x04,NULL,NULL);
}

void QuickStopMotor(short mode)
{
	MDRESULT	ret=MDERR_SUCCESS;
	unsigned short buf[4]={0x00,0x0001,0x0000,0x0000};
	motor_flag=0;
	ret = WriteSingleReg(0x01, 0x0040, mode,NULL,NULL);
	Delayus(1500);
	motor_flag=1;
	ret = WriteSingleReg(0x02, 0x0040, mode,NULL,NULL);
}

char GetDriverState(char add)
{
	MDRESULT	ret=MDERR_SUCCESS;
	unsigned char read_buf[2]={0};
	unsigned short read_len=2;
	unsigned short read_data=0;
	Delayus(1500);
	ret=ReadRegs(add, 0x0033, 1,read_buf,&read_len);
	if(ret==MDERR_SUCCESS)
	{
			read_data=((read_buf[0]<<8) + read_buf[1]);
			switch(read_data)
			{
				case DRIVER_ERR_NONE:
					break;
				case DRIVER_NO_SPEED:		//堵转停止
				{
					cmdfeed.LEFT.MOTOR_DRIVER_STRUCT.block = 1;
					cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.normal = 0;
					OSQPost(EHandlerQBOX,(void*)MOTOR_NO_SPEED);
				}
				break;
				case DRIVER_HOWER_ERR:	//霍尔错误
				{
					cmdfeed.LEFT.MOTOR_DRIVER_STRUCT.hall = 1 ;
					cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.normal = 0;
					OSQPost(EHandlerQBOX,(void*)DRIVER_HALL_ERR);
				}
				break;
				case DRIVER_SPEED_NOTACHIVE:			//达不到指定速度
				{
					cmdfeed.LEFT.MOTOR_DRIVER_STRUCT.target_speed = 1 ;
					cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.normal = 0;
					OSQPost(EHandlerQBOX,(void*)MOTOR_WINDING_OPEN);
				}
				break;
				case DRIVER_WINDING_OPEN:
				{
					cmdfeed.LEFT.MOTOR_DRIVER_STRUCT.winding_open = 1 ;
					cmdfeed.WORK_STATE.ROBOT_STATE_STRUCT.normal = 0;
					OSQPost(EHandlerQBOX,(void*)MOTOR_WINDING_OPEN);
				}
					break;
				default:
					break;
			}
		}
	else
	{
		//通信断线错误码
		return 0x20;
	}
	
	return read_data;
}

void ClearDriverState()
{
	MDRESULT	ret=MDERR_SUCCESS;
	unsigned char read_buf[2]={0};
	unsigned short read_len=2;
	unsigned short read_data=0;
	
	Delayus(1500);
	ret=ReadRegs(0x02, 0x0033, 1,read_buf,&read_len);
	if(ret==MDERR_SUCCESS)
	{
			read_data=((read_buf[0]<<8) + read_buf[1]);
			if(read_data==0x00)
			{
				cmdfeed.RIGHT.MOTOR_DRIVER_STATE = 0;
			}
	}
	Delayus(1500);
	ret=ReadRegs(0x01, 0x0033, 1,read_buf,&read_len);
	if(ret==MDERR_SUCCESS)
	{
			read_data=((read_buf[0]<<8) + read_buf[1]);
			if(read_data==0x00)
			{
				cmdfeed.LEFT.MOTOR_DRIVER_STATE = 0;
			}
	}
}

short  GetCurrent(char add)
{
	MDRESULT	ret=MDERR_SUCCESS;
	unsigned char read_buf[2]={0};
	unsigned short read_len=2;
	unsigned short read_data=0;
	char t_add=add;
	static short current;
		if(cmdfeed.LEFT.MOTOR_DRIVER_STRUCT.encoder != 1)
		{
			ret=ReadRegs(t_add, 0x0021, 1,read_buf,&read_len);
			if(ret==MDERR_SUCCESS)
			{
				current=((read_buf[0]<<8) + read_buf[1]);		//实时电流		
			}
		}	
		return current;
}


void Delayus(uint32_t us)
{
    int i, j;
    for (i=0; i<us; i++)
    {
        for (j=0; j<6; j++);
    }
}

void _delay(unsigned int cnt)
{
	unsigned int i=0;
	for(i=0;i<cnt; i++)
	{
		cnt--;
	}
}



