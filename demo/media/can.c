/******************************************************************************
Copyright (C) 2021-2023 广州敏视数码科技有限公司版权所有.

文件名：can.c

日期: 2023-02-17

文件功能描述: 定义can功能demo接口

其他: // 其他内容说明

版本: v1.0.0(最新版本号)

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
#include <arpa/inet.h>
#include <net/if.h>
#include <linux/socket.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>


#include "print.h"
#include "../../../include/board.h"
#include "safefunc.h"
#include "op.h"
#include "msg.h"
#include "cJSON.h"
#include "alarm.h"
#include "utils.h"
#include "gps.h"
#include "can.h"

#define CAN_UPGRADE_FILE_PERFIX	"DMS31V2_upgrade"

/* CAN 模块控制信息 */
typedef struct tag_CanInfo_S
{
	sint32						s32CanFd;				/* can口设备描述符 */
    CAN_FORMAT_E                enFrameFormat;          /* can帧格式 */
	sint32						s32Baudrate;			/* can波特率 */
	sint8       				szDmmCanid[32];     	/* DMM canid */
	sint8       				szFrsCanid[32];			/* FRS canid*/
    sint8        				szHeartCanid[32];   	/* Heart canid */
	sint8						szPdsCanid[32];		    /* PDS canid */
    SV_BOOL                    	bRunning;           	/* 线程运行状态 */    
    pthread_mutex_t				mutexSendData;      	/* 发送数据互斥锁 */
}CAN_INFO_S;

CAN_INFO_S m_stCanInfo = {0};

sint32 can_GetFd(sint32 *s32CanFd)
{
	sint32 s32Ret, s32Fd;
    sint32 loopback = 0; //0 = disabled, 1 = enabled
	sint32 recvOwnMsg = 0; //0 = disabled, 1 = enabled
    struct ifreq ifr;
    strcpy(ifr.ifr_name, "can0");
    struct sockaddr_can addr;
	char szCmd[128] = {0};
	
	s32Fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s32Fd < 0) 
	{
         print_level(SV_ERROR, "Error while opening can socket, %s\n", strerror(errno));
         return SV_FAILURE;
    }

    setsockopt(s32Fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
    setsockopt(s32Fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recvOwnMsg, sizeof(recvOwnMsg));

    strcpy(ifr.ifr_name, "can0");
    ioctl(s32Fd, SIOCGIFINDEX, &ifr);

    addr.can_family  = PF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
	s32Ret = bind(s32Fd, (struct sockaddr *)&addr, sizeof(addr));
    if (s32Ret < 0) 
	{
        print_level(SV_ERROR, "Error in socket bind, %s\n", strerror(errno));
        close(s32Fd);
        s32Fd = -1;
        return SV_FAILURE;
    }

	*s32CanFd = s32Fd;

    return SV_SUCCESS;
}

sint32 can_GetClkFreq(sint32 s32CanFd, ulng32 *u32ClkFreq)
{
    sint32 s32Ret;
    struct ifreq ifr;
    strcpy(ifr.ifr_name, "can0");

    if (s32CanFd <= 0 || NULL == u32ClkFreq)
    {
        print_level(SV_ERROR, "get invalid param: fd=%d\n", s32CanFd);
        return SV_FAILURE;
    }

    s32Ret = ioctl(s32CanFd, SIOCGCANCLKFREQ, &ifr);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "ioctl SIOCGCANCLKFREQ failed [err=%#x, reason: %s]\n", s32Ret, strerror(errno));
        return SV_FAILURE;
    }
    *u32ClkFreq = (ulng32)ifr.ifr_ifru.ifru_data;
    
    return SV_SUCCESS;
}

sint32 can_SetClkFreq(sint32 s32CanFd, ulng32 u32ClkFreq)
{
    sint32 s32Ret;
    struct ifreq ifr;
    strcpy(ifr.ifr_name, "can0");

    if (s32CanFd <= 0 || u32ClkFreq < CAN_CLK_FREQ_LOW || u32ClkFreq > CAN_CLK_FREQ_HIGH)
    {
        print_level(SV_ERROR, "get invalid param: fd=%d, clkFreq=%ld\n", s32CanFd, u32ClkFreq);
        return SV_FAILURE;
    }

    ifr.ifr_ifru.ifru_data = (void *)u32ClkFreq;
    s32Ret = ioctl(s32CanFd, SIOCSCANCLKFREQ, &ifr);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "ioctl %d failed [err=%#x, reason: %s]\n", u32ClkFreq, s32Ret, strerror(errno));
        return SV_FAILURE;
    }
    
    return SV_SUCCESS;
}

sint32 can_CheckClkFreq(SV_BOOL bNeedGetFd, sint32 s32Baudrate)
{
	sint32 s32Ret, s32CanFd = -1;
    ulng32 u32ClkFreq = 0;
    SV_BOOL bChangeFreq = SV_FALSE;

    if (bNeedGetFd)
    {
        s32Ret = can_GetFd(&s32CanFd);
    	if (SV_SUCCESS != s32Ret)
    	{
    		print_level(SV_ERROR, "can_GetFd error!\n");
    		return SV_FAILURE;
    	}
    }
    else
    {
        s32CanFd = m_stCanInfo.s32CanFd;
    }

    if (s32CanFd <= 0)
    {
        print_level(SV_ERROR, "get invalid fd: %d\n", s32CanFd);
        return SV_FAILURE;
    }

    s32Ret = can_GetClkFreq(s32CanFd, &u32ClkFreq);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "can_GetClkFreq failed.\n");
		return SV_FAILURE;
    }

#if defined(BOARD_ADA47V1)
    if ((s32Baudrate <= CAN_BAUDRATE_LIMIT_LOW && u32ClkFreq > CAN_CLK_FREQ_LOW)
        || (s32Baudrate >= CAN_BAUDRATE_LIMIT_MIDDLE && u32ClkFreq < CAN_CLK_FREQ_HIGH)
        || (s32Baudrate > CAN_BAUDRATE_LIMIT_LOW && s32Baudrate < CAN_BAUDRATE_LIMIT_MIDDLE && u32ClkFreq != CAN_CLK_FREQ_MIDDLE))
#else
    if ((s32Baudrate < CAN_BAUDRATE_LIMIT && u32ClkFreq > CAN_CLK_FREQ_LOW)
        || (s32Baudrate >= CAN_BAUDRATE_LIMIT && u32ClkFreq <= CAN_CLK_FREQ_LOW))
#endif
    {
        bChangeFreq = SV_TRUE;
    }
        
    if (bChangeFreq)
    {
#if defined(BOARD_ADA47V1)
        u32ClkFreq = (s32Baudrate <= CAN_BAUDRATE_LIMIT_LOW) ? (void *)CAN_CLK_FREQ_LOW : (s32Baudrate >= CAN_BAUDRATE_LIMIT_MIDDLE) ? (void *)CAN_CLK_FREQ_HIGH : (void *)CAN_CLK_FREQ_MIDDLE;
#else
        u32ClkFreq = (s32Baudrate < CAN_BAUDRATE_LIMIT) ? (void *)CAN_CLK_FREQ_LOW : (void *)CAN_CLK_FREQ_HIGH;
#endif
        print_level(SV_INFO, "this baudrate need to change clock freq, set can clock freq to %ldMHz\n", u32ClkFreq / 1000000);
        s32Ret = can_SetClkFreq(s32CanFd, u32ClkFreq);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "can_SetClkFreq failed.\n");
    		return SV_FAILURE;
        }
    }

    return SV_SUCCESS;
}


sint32 can_InitServer(sint32 s32Baudrate, sint32 *s32CanFd)
{
	sint32 s32Ret, s32Fd;
    sint32 s32Rate = s32Baudrate * 1000;
    ulng32 u32ClkFreq = 0;
    sint32 loopback = 0; //0 = disabled, 1 = enabled
	sint32 recvOwnMsg = 0; //0 = disabled, 1 = enabled
    struct ifreq ifr;
    strcpy(ifr.ifr_name, "can0");
	char szCmd[128] = {0};
    struct sockaddr_can addr;

	strncpy(szCmd, "io -4 0xfe010050 0x00ff0033", 128);
	s32Ret = SAFE_System(szCmd, NORMAL_WAIT_TIME);
	if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "cmd: %s failed.\n", szCmd);
		return SV_FAILURE;
    }
    
    strncpy(szCmd, "/root/ip link set can0 down", 128);
    s32Ret = SAFE_System(szCmd, NORMAL_WAIT_TIME);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "cmd: %s failed.\n", szCmd);
        return SV_FAILURE;
    }
    
	strncpy(szCmd, "ifconfig can0 txqueuelen 1000", 128);
	s32Ret = SAFE_System(szCmd, NORMAL_WAIT_TIME);
	if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "cmd: %s failed.\n", szCmd);
		return SV_FAILURE;
    }

	sprintf(szCmd, "/root/ip link set can0 type can bitrate %d triple-sampling on", s32Rate);
	s32Ret = SAFE_System(szCmd, NORMAL_WAIT_TIME);
	if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "cmd: %s failed.\n", szCmd);
		return SV_FAILURE;
    }

	strncpy(szCmd, "/root/ip link set can0 up", 128);
	s32Ret = SAFE_System(szCmd, NORMAL_WAIT_TIME);
	if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "cmd: %s failed.\n", szCmd);
		return SV_FAILURE;
    }

	s32Fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s32Fd < 0) 
	{
         print_level(SV_ERROR, "Error while opening can socket, %s\n", strerror(errno));
         return SV_FAILURE;
    }

    setsockopt(s32Fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
    setsockopt(s32Fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recvOwnMsg, sizeof(recvOwnMsg));

    strcpy(ifr.ifr_name, "can0");
    ioctl(s32Fd, SIOCGIFINDEX, &ifr);

    addr.can_family  = PF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
	s32Ret = bind(s32Fd, (struct sockaddr *)&addr, sizeof(addr));
    if (s32Ret < 0) 
	{
        print_level(SV_ERROR, "Error in socket bind, %s\n", strerror(errno));
        close(s32Fd);
        s32Fd = -1;
        return SV_FAILURE;
    }

	*s32CanFd = s32Fd;

	print_level(SV_INFO, "Bind can socket finish\n");
    return SV_SUCCESS;
}

sint32 can_FiniServer(sint32 s32CanFd)
{
	if (s32CanFd > 0)
	    close(s32CanFd);

    return SV_SUCCESS;
}

/******************************************************************************
 * 函数功能: can接收主线程
 * 输入参数: pvArg -- 用户传入参数
 * 输出参数: 无
 * 返回值  : NULL
 * 注意    : 无
 *****************************************************************************/
void* can_Recv_Body(void *pvArg)
{
	sint32 s32Ret, s32MaxFd, i = 0;
	sint32 s32FreeSpace, s32ReadSize = 0;
	uint8 u8ErrCnt  = 0;
	sint64 s64UpdateCanid = 0, s64GpsCanid = 0;
    fd_set fdset;
	struct timeval timeout;
	struct can_frame stCanFrame;
	CAN_INFO_S *pstCanInfo = (CAN_INFO_S *)pvArg;

	s64UpdateCanid = 0x18FAD5FE | CAN_EFF_FLAG;
	s64GpsCanid = 0x18FBD0EE | CAN_EFF_FLAG;

	/* 设置过滤规则 */
	struct can_filter stCanFilter[2];
	stCanFilter[0].can_id = s64UpdateCanid;
	stCanFilter[0].can_mask = CAN_EFF_MASK;
	stCanFilter[1].can_id = s64GpsCanid;
	stCanFilter[1].can_mask = CAN_EFF_MASK;

	setsockopt(pstCanInfo->s32CanFd, SOL_CAN_RAW, CAN_RAW_FILTER, &stCanFilter, sizeof(stCanFilter));
	print_level(SV_INFO, "setsockopt can fd %d\n", pstCanInfo->s32CanFd);

	while (pstCanInfo->bRunning)
	{
		s32MaxFd = 0;
        FD_ZERO(&fdset);
		FD_SET(pstCanInfo->s32CanFd, &fdset);
        if (pstCanInfo->s32CanFd > s32MaxFd)
        {
            s32MaxFd = pstCanInfo->s32CanFd+1;
        }
		else
		{
			sleep_ms(1000);
            print_level(SV_INFO, "s32MaxFd: %d, fd: %d\n", s32MaxFd, pstCanInfo->s32CanFd);
			continue;
		}

		timeout.tv_sec = 0;
        timeout.tv_usec = 500000;
        s32Ret = select(s32MaxFd, &fdset, NULL, NULL, &timeout);
        if (s32Ret < 0)
        {
            if(errno != EINTR)
            {
                print_level(SV_ERROR, "select can fd failed\n");
            }
            print_level(SV_ERROR, "select can fd failed.\n");
        }
        else if (s32Ret == 0)
        {
			continue;
            print_level(SV_ERROR, "select can fd timeout\n");
        }
		else
		{
			if (FD_ISSET(pstCanInfo->s32CanFd, &fdset)) 
            {
            	s32ReadSize = read(pstCanInfo->s32CanFd, &stCanFrame, sizeof(stCanFrame));
				if (s32ReadSize <= 0)
				{
					continue;
				}
				
				if (stCanFrame.can_id == s64GpsCanid)
				{
					print_level(SV_INFO, "recv gps canid msg.\n");
				}
				else if (stCanFrame.can_id == s64UpdateCanid)
				{
					print_level(SV_INFO, "recv update canid msg.\n");
                    #if 0
                    for (i = 0; i < stCanFrame.can_dlc; i++)
                    {
                        printf("%X ", stCanFrame.data[i]);
                    }
                    printf("\n");
                    #endif
				}
				else
				{
					print_level(SV_ERROR, "cnaid error ID=%#X \n", stCanFrame.can_id);
					if (++u8ErrCnt >= 20)  
					{
						u8ErrCnt = 0;
						print_level(SV_INFO, "can retry to setsockopt\n");
						setsockopt(pstCanInfo->s32CanFd, SOL_CAN_RAW, CAN_RAW_FILTER, &stCanFilter, sizeof(stCanFilter));
					}
				}
            }
		}
	}
	
    return NULL;
}

sint32 CAN_Init(CFG_SYS_PARAM *pstSysParam)
{
	sint32 s32Ret, s32CanFd = -1;
	uint32 u32FreeSpace = 0;
    ulng32 u32ClkFreq = 0;
    memset(&m_stCanInfo, 0, sizeof(CAN_INFO_S));

	s32Ret = pthread_mutex_init(&m_stCanInfo.mutexSendData, NULL);
    if (0 != s32Ret)
    {
        print_level(SV_ERROR, "pthread_mutex_init failed! [err=%#x]\n", s32Ret);
        return ERR_SYS_NOTREADY;
    }

	s32Ret = can_CheckClkFreq(SV_TRUE, pstSysParam->s32Baudrate);
	if (SV_SUCCESS != s32Ret)
	{
		print_level(SV_ERROR, "can_CheckClkFreq failed!\n");
		return SV_FAILURE;
	}

    s32Ret = can_InitServer(pstSysParam->s32Baudrate, &s32CanFd);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_INFO, "can_InitServer failed.\n");
        return SV_FAILURE;
    }
    
	m_stCanInfo.s32CanFd = s32CanFd;
    m_stCanInfo.enFrameFormat = pstSysParam->enFrameFormat;
	m_stCanInfo.s32Baudrate = pstSysParam->s32Baudrate;
	strncpy(m_stCanInfo.szDmmCanid, pstSysParam->pszDmmCanid, 31);
	strncpy(m_stCanInfo.szFrsCanid, pstSysParam->pszFrsCanid, 31);
	strncpy(m_stCanInfo.szHeartCanid, pstSysParam->pszHeartCanid, 31);
	strncpy(m_stCanInfo.szPdsCanid, pstSysParam->pszPdsCanid, 31);
	
    return SV_SUCCESS;
}

sint32 CAN_Fini()
{
    sint32 s32Ret;
    s32Ret = can_FiniServer(m_stCanInfo.s32CanFd);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "can_FiniServer failed.\n");
        return SV_FAILURE;
    }

    pthread_mutex_destroy(&m_stCanInfo.mutexSendData);	
    memset(&m_stCanInfo, 0, sizeof(CAN_INFO_S));
    return SV_SUCCESS;
}

sint32 CAN_Reset()
{
    sint32 s32Ret;
    pthread_mutex_lock(&m_stCanInfo.mutexSendData);
	sleep_ms(1000);
	system("canconfig can0 stop >> /dev/null");
	sleep_ms(50);
	system("canconfig can0 start >> /dev/null");
    pthread_mutex_unlock(&m_stCanInfo.mutexSendData);
    
    return SV_SUCCESS;
}

uint32 CAN_GetCanid(uint32 u32Canid)
{
    return (E_CAN_STANDARD == m_stCanInfo.enFrameFormat) ? u32Canid : (u32Canid | CAN_EFF_FLAG);
}

sint32 CAN_SendFrame(struct can_frame *pstFrame)
{
    sint32 s32Ret;
	
    if (pstFrame == NULL)
    {
		print_level(SV_ERROR, "null pointer.\n");
        return SV_FAILURE;
    }

	pthread_mutex_lock(&m_stCanInfo.mutexSendData);

    s32Ret = write(m_stCanInfo.s32CanFd, pstFrame, sizeof(struct can_frame));
	if (s32Ret < 0)
	{
		print_level(SV_ERROR, "write can frame error\n");
        pthread_mutex_unlock(&m_stCanInfo.mutexSendData);
		return SV_FAILURE;
    }
#if 0
	else
	{
		print_level(SV_INFO, "write size: %d\n", s32Ret);
		for (int i = 0; i < pstFrame->can_dlc; i++)
			printf("%X ", pstFrame->data[i]);
		printf("\n");
	}
#endif
    pthread_mutex_unlock(&m_stCanInfo.mutexSendData);

    return SV_SUCCESS;
}

