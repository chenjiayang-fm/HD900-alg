/******************************************************************************
Copyright (C) 2022-2024 广州敏视数码科技有限公司版权所有.

文件名：rs485.c

作者: szp       版本: v1.0.0(初始版本号)   日期: 2022-12-15

文件功能描述: 定义485通讯接口

*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <dirent.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/prctl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <errno.h>
#include <time.h>
#include <fcntl.h>
#include <termios.h>

#include "print.h"
#include "../../../include/board.h"
#include "safefunc.h"
#include "op.h"
#include "msg.h"
#include "utils.h"
#include "rs485.h"

#define RS485_PORT              "/dev/ttyS3"              		/* RS485串口设备 */
#define RS485_FRAME_MAX_LEN     1024                            /* RS485帧最大长度 */

/* RS485模块控制信息 */
typedef struct
{
	sint32						s32SerialFd;		/* 串口设备描述符 */
    sint32                      s32RS485Baudrate;   /* 串口波特率 */
    SV_BOOL                    	bRunning;           /* 线程运行状态 */
    pthread_t                   RecvTid;            /* 接收线程ID */
    pthread_mutex_t				mutexSendData;      /* 发送数据互斥锁 */
}
STRS485Info;

STRS485Info m_stRS485Info = {0};

/*
// 打印年月日时分秒毫秒
static void print_time()
{
    struct timeval tv;
	int BEIJINGTIME = 8;
	int DAY = (60*60*24);
	int YEARFIRST = 2001;
	int YEARSTART = (365*(YEARFIRST-1970) + 8);
	int YEAR400 = (365*4*100 + (4*(100/4 - 1) + 1));
	int YEAR100 = (365*100 + (100/4 - 1));
	int YEAR004 = (365*4 + 1);
	int YEAR001 = 365;
	
    long sec = 0, usec = 0;
    int yy = 0, mm = 0, dd = 0, hh = 0, mi = 0, ss = 0, ms = 0;
    int ad = 0;
    int y400 = 0, y100 = 0, y004 = 0, y001 = 0;
    int m[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    int i;
    memset(&tv, 0, sizeof(struct timeval));
    gettimeofday(&tv, NULL);
    sec = tv.tv_sec;
    usec = tv.tv_usec;
    sec = sec + (60*60)*BEIJINGTIME;
    ad = sec/DAY;
    ad = ad - YEARSTART;
    y400 = ad/YEAR400;
    y100 = (ad - y400*YEAR400)/YEAR100;
    y004 = (ad - y400*YEAR400 - y100*YEAR100)/YEAR004;
    y001 = (ad - y400*YEAR400 - y100*YEAR100 - y004*YEAR004)/YEAR001;
    yy = y400*4*100 + y100*100 + y004*4 + y001*1 + YEARFIRST;
    dd = (ad - y400*YEAR400 - y100*YEAR100 - y004*YEAR004)%YEAR001;
    //月 日
    if(0 == yy%1000)
    {
        if(0 == (yy/1000)%4)
        {
            m[1] = 29;
        }
    }
    else
    {
        if(0 == yy%4)
        {
            m[1] = 29;
        }
    }
    for(i = 1; i <= 12; i++)
    {
        if(dd - m[i] < 0)
        {
            break;
        }
        else
        {
            dd = dd -m[i];
        }
    }
    mm = i;
    //小时
    hh = sec/(60*60)%24;
    //分
    mi = sec/60 - sec/(60*60)*60;
    //秒
    ss = sec - sec/60*60;
    ms = usec;
    printf("%d-%02d-%02d %02d:%02d:%02d.%06d ==> ", yy, mm, dd, hh, mi, ss, ms);
}

sint32 callbackRS485Send(MSG_PACKET_S *pstMsgPkt, MSG_PACKET_S *pstRetPkt)
{
    sint32 s32Ret = 0;
    MSG_RS485_DATA_S *pstMsgRS485Data = (MSG_RS485_DATA_S *)pstMsgPkt->pu8Data;
    
    s32Ret = rs485_SendData(pstMsgRS485Data->szRS485Data, pstMsgRS485Data->s32DataLen);
    if (s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "rs485_SendData failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }
    
    return SV_SUCCESS;
}
*/
sint32 rs485_CtrlPinInit(uint8 u8Band, uint8 u8PinNum)
{
    sint32 s32Ret = 0;
    uint32_t u32GpioNum = 0;
    char szCmd[256];
    
    if(u8PinNum > 32)
    {
        return SV_FAILURE;
    }
    
    u32GpioNum = u8Band*32+u8PinNum;

    sprintf(szCmd, "echo %d > /sys/class/gpio/export", u32GpioNum);
    s32Ret = SAFE_System(szCmd, NORMAL_WAIT_TIME);
    if (0 != s32Ret)
    {
        print_level(SV_ERROR, "cmd: %s failed.\n", szCmd);
        return SV_FAILURE;
    }
    
    sprintf(szCmd, "echo out > /sys/class/gpio/gpio%d/direction", u32GpioNum);
    s32Ret = SAFE_System(szCmd, NORMAL_WAIT_TIME);
    if (0 != s32Ret)
    {
        print_level(SV_ERROR, "cmd: %s failed.\n", szCmd);
        return SV_FAILURE;
    }

    return SV_SUCCESS;
}

sint32 rs485_CtrlPinSetValue(uint32_t u32GpioNum, uint8_t u8Status)
{
	if (u8Status > 1)
	{
		print_level(SV_ERROR, "s32Edge is out of range!\n");
		return SV_FAILURE;
	}
	
	static char s_szBufStat[] = {'0', '1'};
	sint32 s32Fd = -1, s32Ret;
	char path[64] = {0};

	snprintf(path, 64, "/sys/class/gpio/gpio%d/value", u32GpioNum);
	s32Fd  = open(path, O_RDWR|O_CLOEXEC);
	if(s32Fd < 0)
	{
		print_level(SV_ERROR, "open[%s] fail error[%d]\n", path, errno);
		return SV_FAILURE;
	}

	s32Ret = write(s32Fd, &s_szBufStat[u8Status], sizeof(s_szBufStat[u8Status]));
	if(s32Ret < 0)
	{
		print_level(SV_ERROR, "write [%s] fail error[%d]\n", s_szBufStat[u8Status], errno);
		close(s32Fd);
		return SV_FAILURE;
	}
    
	close(s32Fd);
	return SV_SUCCESS;
}

sint32 rs485_ReadData(char *pszData, uint32 *pu32Len)
{
	sint32 s32Ret = 0;
	uint32 u32Count = 0;
    uint32 u32TimCnt = 0;
    uint32_t u32TimCntLimit = 0;

	if (pszData == NULL)
	{
		print_level(SV_ERROR, "null pointer!\n");
		return SV_FAILURE;
	}

    switch (m_stRS485Info.s32RS485Baudrate)
    {
        case 1200:
            u32TimCntLimit = 60;
            break;
        case 2400:
            u32TimCntLimit = 30;
            break;
        case 4800:
            u32TimCntLimit = 20;
            break;
        case 9600:
            u32TimCntLimit = 12;
            break;
        case 19200:
            u32TimCntLimit = 8;
            break;
        case 38400:
            u32TimCntLimit = 6;
            break;
        case 57600:
            u32TimCntLimit = 4;
            break;
        case 115200:
            u32TimCntLimit = 3;
            break;
        default:
            u32TimCntLimit = 3;
            break;
    }
    
    while (u32Count < RS485_FRAME_MAX_LEN)
    {
        s32Ret = read(m_stRS485Info.s32SerialFd, pszData+u32Count, RS485_FRAME_MAX_LEN-u32Count);
        if (s32Ret < 0)
        {
            if (errno != EAGAIN && errno != EINTR)
            {
                print_level(SV_ERROR, "recv error\n");
                return SV_FAILURE;
            }
            
            //print_level(SV_INFO, "EAGAIN or EINTR: recv no data\n");
            
			u32TimCnt++;
			if (u32TimCnt > u32TimCntLimit)
			{
				//print_level(SV_INFO, "data end\n");
				break;
			}
            else
            {
                sleep_ms(5);
            }
        }
        else if(s32Ret == 0)
        {
			u32TimCnt++;
			if (u32TimCnt > u32TimCntLimit)
			{
				//print_level(SV_INFO, "data end\n");
				break;
			}
            else
            {
                sleep_ms(5);
            }
        }
        else
        {
            u32Count += s32Ret;
            u32TimCnt = 0;
        }
    }

    *pu32Len = u32Count;
	return SV_SUCCESS;
}

sint32 rs485_WriteData(char *pszData, uint32 u32Len)
{
    sint32 i = 0, s32Ret = 0;
	
    if (u32Len > RS485_FRAME_MAX_LEN)
    {
        print_level(SV_ERROR, "RS485 frame is too long\n");
        return SV_FAILURE;
    }

	if (!m_stRS485Info.bRunning)
	{
		return SV_SUCCESS;
	}
    
    pthread_mutex_lock(&m_stRS485Info.mutexSendData);
    s32Ret = serialWriteRaw(m_stRS485Info.s32SerialFd, pszData, u32Len);
    if (s32Ret != u32Len)
	{
		print_level(SV_ERROR, "serialWriteRaw failed.\n");
		pthread_mutex_unlock(&m_stRS485Info.mutexSendData);
		return SV_FAILURE;
	}
#if 0
    for (i=0; i<u32Len; i++)
    {
        s32Ret = serialWriteChar(m_stRS485Info.s32SerialFd, pszData[i]);
		if (s32Ret < 0)
		{
			print_level(SV_ERROR, "serialWriteChar failed.\n");
			pthread_mutex_unlock(&m_stRS485Info.mutexSendData);
			return SV_FAILURE;
		}
    }  
#endif
    pthread_mutex_unlock(&m_stRS485Info.mutexSendData);

	return SV_SUCCESS;
}

void* rs485_Recv_Body(void *pvArg)
{
    sint32 s32Ret;
    STRS485Info *pstRS485Info = (STRS485Info *)pvArg;
    MSG_RS485_DATA_S stRS485RecvData = {0};
    MSG_PACKET_S stMsgPkt = {0};
    struct timeval timeout;
	sint32 i;
    
    while (pstRS485Info->bRunning)
    {			
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        fd_set UartFd_Set;
        FD_ZERO(&UartFd_Set);
        FD_SET(pstRS485Info->s32SerialFd, &UartFd_Set);

        s32Ret = select(pstRS485Info->s32SerialFd+1, &UartFd_Set, 0, 0, &timeout);
        if(s32Ret < 0)
        {
            print_level(SV_ERROR, "%s\n", strerror(errno));
        }
        else if(s32Ret == 0)
        {
            //print_level(SV_INFO, "485 no data\n");
            continue;
        }
        else
        {
            memset(&stRS485RecvData, 0, sizeof(MSG_RS485_DATA_S));
            s32Ret = rs485_ReadData(stRS485RecvData.szRS485Data, &stRS485RecvData.s32DataLen);
            if (SV_SUCCESS != s32Ret)
            {
                print_level(SV_ERROR, "rs485_ReadData failed. Len:%d\n", stRS485RecvData.s32DataLen);
                continue;
            }

#if 1
            printf("rs485 recv data: ");
            for (i=0; i<stRS485RecvData.s32DataLen; i++)
            {
                printf("0x%x ", stRS485RecvData.szRS485Data[i]);
            }
            printf("\r\n");
#endif
/*
            stMsgPkt.pu8Data = &stRS485RecvData;
            stMsgPkt.u32Size = sizeof(MSG_RS485_DATA_S);
            s32Ret = Msg_submitEvent(EP_ALG, OP_EVENT_RS485_RECV, &stMsgPkt);
            if (SV_SUCCESS != s32Ret)
            {
                print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
                return SV_FAILURE;
            }
*/
        }
    }
}

sint32 RS485_Init(CFG_SYS_PARAM *pstSysParam)
{
	sint32 s32Ret, s32SerialFd = -1;
    memset(&m_stRS485Info, 0, sizeof(STRS485Info));
    
	s32Ret = pthread_mutex_init(&m_stRS485Info.mutexSendData, NULL);
    if (0 != s32Ret)
    {
        print_level(SV_ERROR, "pthread_mutex_init failed! [err=%#x]\n", s32Ret);
        return ERR_SYS_NOTREADY;
    }

	s32Ret = serialOpen(RS485_PORT, pstSysParam->s32RS485Baudrate, SV_TRUE, &s32SerialFd);
	if (SV_SUCCESS != s32Ret)
	{
		print_level(SV_ERROR, "serial_open %s error! baud: %d\n", RS485_PORT, pstSysParam->s32RS485Baudrate);
		return SV_FAILURE;
	}
    
	m_stRS485Info.s32SerialFd = s32SerialFd;
	m_stRS485Info.s32RS485Baudrate = pstSysParam->s32RS485Baudrate;
    
    s32Ret = rs485_CtrlPinInit(3, 20);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "rs485_CtrlPinInit failed! [err=%#x]\n", s32Ret);
    }
    
    return SV_SUCCESS;
}

sint32 RS485_Fini()
{
	if(m_stRS485Info.s32SerialFd > 0)
	{
        serialClose(m_stRS485Info.s32SerialFd);
	}

    pthread_mutex_destroy(&m_stRS485Info.mutexSendData);
    memset(&m_stRS485Info, 0, sizeof(STRS485Info));
    return SV_SUCCESS;
}

sint32 RS485_Start()
{
    sint32 s32Ret = 0;
    m_stRS485Info.bRunning = SV_TRUE;

    pthread_t RecvTid;

    s32Ret = pthread_create(&RecvTid, NULL, rs485_Recv_Body, &m_stRS485Info);
    if(s32Ret != 0)
    {
        printf("pthread_create failed! [err: %s]\n", strerror(errno));
        return SV_FAILURE;
    }
/*
    s32Ret = MSG_ReciverStart(EP_RS485);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "MSG_ReciverStart failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }

    s32Ret = Msg_registerOpCallback(EP_RS485, OP_EVENT_RS485_SEND, callbackRS485Send);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_registerOpCallback_ThreadExec failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }
*/
    m_stRS485Info.RecvTid = RecvTid;
    
    return SV_SUCCESS;
}

sint32 RS485_Stop()
{
    sint32 s32Ret = 0;
    pthread_t RecvTid = m_stRS485Info.RecvTid;
    void *pvRetval = NULL;

    m_stRS485Info.bRunning = SV_FALSE;
    
    s32Ret = pthread_join(RecvTid, &pvRetval);
    if (0 != s32Ret)
    {
        printf("pthread_join failed. [err: %s]\n", strerror(errno));
        return SV_FAILURE;
    }

    return SV_SUCCESS;
}

sint32 RS485_SetConfig(CFG_SYS_PARAM *pstSysParam)
{
    sint32 s32Ret = -1, s32SerialFd = -1;
    
    if (NULL == pstSysParam)
    {
        return ERR_NULL_PTR;
    }
	
    if (m_stRS485Info.s32RS485Baudrate != pstSysParam->s32RS485Baudrate)
    {
        if (m_stRS485Info.s32SerialFd > 0)
        {
            serialClose(m_stRS485Info.s32SerialFd);
        }
        
        s32Ret = serialOpen(RS485_PORT, pstSysParam->s32RS485Baudrate, SV_TRUE, &s32SerialFd);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "serial_open %s error! baud: %d\n", RS485_PORT, pstSysParam->s32RS485Baudrate);
            s32Ret = serialOpen(RS485_PORT, m_stRS485Info.s32RS485Baudrate, SV_TRUE, &s32SerialFd);
            if (SV_SUCCESS != s32Ret)
            {
                print_level(SV_ERROR, "serial_open %s error! baud: %d\n", RS485_PORT, m_stRS485Info.s32RS485Baudrate);
            }
            else
            {
                print_level(SV_ERROR, "serial_open %s success! baud: %d\n", RS485_PORT, m_stRS485Info.s32RS485Baudrate);
            }
            m_stRS485Info.s32SerialFd = s32SerialFd;
            return SV_FAILURE;
        }
        else
        {
            print_level(SV_INFO, "RS485 %s reinit success! baudrate: %d\n", RS485_PORT, pstSysParam->s32RS485Baudrate);
            m_stRS485Info.s32SerialFd = s32SerialFd;
            m_stRS485Info.s32RS485Baudrate = pstSysParam->s32RS485Baudrate;
        }
    }
	
    return SV_SUCCESS;
}

sint32 RS485_SendData(char *pszData, uint32 u32Len)
{
    sint32 s32Ret = 0;
    sint32 u32Bytes = 0;
    uint8 u8Times = 0;
    uint64 u64SleepTime = 0;
    uint8 i = 0;

    if (pszData == NULL || u32Len > RS485_FRAME_MAX_LEN)
    {
        print_level(SV_ERROR, "serial frame is too long\n");
        return SV_FAILURE;
    }

    /* 485发送 */
    s32Ret = rs485_CtrlPinSetValue(116, 1);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "BOARD_RK_SetGPIO failed.\n");
		return SV_FAILURE;
    }

    /* 串口数据写入 */
    s32Ret = rs485_WriteData(pszData, u32Len);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "rs485_WriteData failed.\n");
        return SV_FAILURE;
    }

    /* 判断是否发送完成 */
    s32Ret = ioctl(m_stRS485Info.s32SerialFd, TIOCOUTQ, &u32Bytes);
    print_level(SV_DEBUG, "wait send len:%d ret:%d\n", u32Bytes, s32Ret);

    while((u32Bytes > 0) && (u8Times < 5))
    {
        u8Times++;
        u64SleepTime = (double)u32Bytes * 11 / m_stRS485Info.s32RS485Baudrate * 1000000;
        print_level(SV_INFO, "bytes:[%d], sleep time:[%lf], baudrate:%d \n", u32Bytes, u64SleepTime, m_stRS485Info.s32RS485Baudrate);
        usleep(u64SleepTime);
        s32Ret = ioctl(m_stRS485Info.s32SerialFd, TIOCOUTQ, &u32Bytes);
    }

    /* 串口缓存发送完成,等待485发送完成 */
    u64SleepTime = (double)(u32Len*11*1000000/m_stRS485Info.s32RS485Baudrate);
    usleep(u64SleepTime);
    
    /* 485接收 */
    s32Ret = rs485_CtrlPinSetValue(116, 0);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "BOARD_RK_SetGPIO failed.\n");
		return SV_FAILURE;
    }
    
	return SV_SUCCESS;
}

