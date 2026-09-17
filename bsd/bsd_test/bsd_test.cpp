/******************************************************************************
Copyright (C) 2017-2019 广州敏视数码科技有限公司版权所有.

文件名：bsd.cpp

作者: 王玲    版本: v1.0.0(初始版本号)   日期: 2019-11-07

文件功能描述: 定义变道辅助BSD图像算法入口程序

*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/syscall.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <pthread.h>
#include <ctype.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>

#include "print.h"
#include "common.h"
#include "hi_common.h"
#include "mpi_sys.h"
#include "mpi_vpss.h"
#include "config.h"
#include "board.h"
#include "op.h"
#include "msg.h"
#include "media.h"
#include "safefunc.h"
#include "sharefifo.h"
#include "control.h"

/* 模块控制信息 */
typedef struct tagBsdInfo_S
{
    
    uint32          u32Tid;             /* 处理线程ID */
    SV_BOOL         bRunning;           /* 线程是否正在运行 */
} BSD_INFO_S;

BSD_INFO_S m_stBsdInfo = {0};
#if 0
/******************************************************************************
 * 函数功能: 获取通道一张灰阶图
 * 输入参数: 无
 * 输出参数: ppvBuf --- 数据缓存指针
             pu32Width --- 画面宽度
             pu32Height --- 画面高度
 * 返回值  : SV_SUCCESS - 成功
             SV_FAILURE - 失败
 * 注意    : 读取完数据后需要调用 bsd_ReleaseGrayFrame 释放数据
 *****************************************************************************/
sint32 bsd_GetGrayFrame(void **ppvBuf, uint32 *pu32Width, uint32 *pu32Height)
{
    sint32 s32Ret = 0;
    VPSS_GRP VpssGrp = 0;
    VPSS_CHN VpssChn = 2;
    sint32 s32GetFrameMilliSec = 2000;
    VIDEO_FRAME_INFO_S stVideoFrame = {0};

    if (NULL == ppvBuf || NULL == pu32Width || NULL == pu32Height)
    {
        return ERR_NULL_PTR;
    }

    s32Ret = HI_MPI_VPSS_GetChnFrame(VpssGrp, VpssChn, &stVideoFrame, s32GetFrameMilliSec);
    if (HI_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "HI_MPI_VPSS_GetChnFrame failed. [err=%#x]\n", s32Ret);
        return SV_FAILURE;
    }

    print_level(SV_DEBUG, "u32PoolId:%d, u32Field:%d, enPixelFormat:%d, enVideoFormat:%d, enCompressMode:%d, u32PhyAddr:%#x, pVirAdd:%#x, u32Stride:%d, pHeaderVirAddr:%#x, u32HeaderStride:%d\n", m_stVideoFrame.u32PoolId, m_stVideoFrame.stVFrame.u32Field, \
                    m_stVideoFrame.stVFrame.enPixelFormat, m_stVideoFrame.stVFrame.enVideoFormat, m_stVideoFrame.stVFrame.enCompressMode, m_stVideoFrame.stVFrame.u32PhyAddr[0], m_stVideoFrame.stVFrame.pVirAddr[0], \
                    m_stVideoFrame.stVFrame.u32Stride[0], m_stVideoFrame.stVFrame.pHeaderVirAddr[0], m_stVideoFrame.stVFrame.u32HeaderStride[0]);
    m_pvAddr = HI_MPI_SYS_Mmap(m_stVideoFrame.stVFrame.u32PhyAddr[0], m_stVideoFrame.stVFrame.u32Width*m_stVideoFrame.stVFrame.u32Height);
    *ppvBuf = m_pvAddr;//m_stVideoFrame.stVFrame.pVirAddr[0];
    *pu32Width = m_stVideoFrame.stVFrame.u32Width;
    *pu32Height = m_stVideoFrame.stVFrame.u32Height;
    
    return SV_SUCCESS;
}

/******************************************************************************
 * 函数功能: 释放灰阶图数据
 * 输入参数: 无
 * 输出参数: 无
 * 返回值  : SV_SUCCESS - 成功
             SV_FAILURE - 失败
 * 注意    : 无
 *****************************************************************************/
sint32 bsd_ReleaseGrayFrame()
{
    sint32 s32Ret = 0;
    VPSS_GRP VpssGrp = 0;
    VPSS_CHN VpssChn = 2;

    HI_MPI_SYS_Munmap(m_pvAddr, m_stVideoFrame.stVFrame.u32Width*m_stVideoFrame.stVFrame.u32Height);
    s32Ret = HI_MPI_VPSS_ReleaseChnFrame(VpssGrp, VpssChn, &m_stVideoFrame);
    if (HI_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "HI_MPI_VPSS_ReleaseChnFrame failed. [err=%#x]\n", s32Ret);
        return SV_FAILURE;
    }
    
    return SV_SUCCESS;
}
#endif

/******************************************************************************
 * 函数功能: 释放灰阶图数据
 * 输入参数: 无
 * 输出参数: 无
 * 返回值  : SV_SUCCESS - 成功
             SV_FAILURE - 失败
 * 注意    : 无
 *****************************************************************************/
void bsd_SubmitEvent()
{
    sint32 s32Ret = 0;
    static uint16 u16Serial = 0;
    MSG_BUF_S stMsgBuf = {0};

    stMsgBuf.u16DestId = EP_LOG;
    stMsgBuf.u16SrcId = EP_ALL;
    stMsgBuf.u16OpCode = OP_EVENT_BSD;
    stMsgBuf.u16Serial = u16Serial;
    //s32Ret = MSG_Write(0, &stMsgBuf, NULL, 0);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "MSG_Write failed. [err=%#x]\n", s32Ret);
        return;
    }
    
    u16Serial++;
}


void * bsd_Handle_Body(void *pvArg)
{
    sint32 s32Ret = 0;
    uint32 u32Cnt = 0;
    sint32 s32Fd = -1;
    void *pvBuf = NULL;
    BSD_INFO_S *pstBsdInfo = (BSD_INFO_S *)pvArg;
    VPSS_GRP VpssGrp = 0;
    VPSS_CHN VpssChn = 2;
    sint32 s32GetFrameMilliSec = 2000;
    VIDEO_FRAME_INFO_S stVideoFrame = {0};

    s32Ret = prctl(PR_SET_NAME, "bsd_body");
    if (0 != s32Ret)
    {
        print_level(SV_ERROR, "prctl PR_SET_NAME failed! [err:%s]\n", strerror(errno));
    }
    
    while (pstBsdInfo->bRunning)
    {
        //print_level(SV_DEBUG, "bsd_Handle_Body running...\n");
        sleep_ms(1000);

        s32Ret = HI_MPI_VPSS_GetChnFrame(VpssGrp, VpssChn, &stVideoFrame, s32GetFrameMilliSec);
        if (HI_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "HI_MPI_VPSS_GetChnFrame failed. [err=%#x]\n", s32Ret);
            continue;
        }

        print_level(SV_DEBUG, "u32PoolId:%d, enModId:%d, (%dx%d), enField:%d, enPixelFormat:%d, enVideoFormat:%d, enCompressMode:%d, enDynamicRange:%d, enColorGamut:%d\n u32HeaderStride:%d %d %d, u32Stride:%d %d %d, u64PhyAddr:%#x %#x %#x, u64VirAddr:%#x %#x %#x\n", \
                    stVideoFrame.u32PoolId, stVideoFrame.enModId, stVideoFrame.stVFrame.u32Width, stVideoFrame.stVFrame.u32Height, \
                    stVideoFrame.stVFrame.enField, stVideoFrame.stVFrame.enPixelFormat, stVideoFrame.stVFrame.enVideoFormat, \
                    stVideoFrame.stVFrame.enCompressMode, stVideoFrame.stVFrame.enDynamicRange, stVideoFrame.stVFrame.enColorGamut, \
                    stVideoFrame.stVFrame.u32HeaderStride[0], stVideoFrame.stVFrame.u32HeaderStride[1], stVideoFrame.stVFrame.u32HeaderStride[2], \
                    stVideoFrame.stVFrame.u32Stride[0], stVideoFrame.stVFrame.u32Stride[1], stVideoFrame.stVFrame.u32Stride[2], \
                    stVideoFrame.stVFrame.u64PhyAddr[0], stVideoFrame.stVFrame.u64PhyAddr[1], stVideoFrame.stVFrame.u64PhyAddr[2], \
                    stVideoFrame.stVFrame.u64VirAddr[0], stVideoFrame.stVFrame.u64VirAddr[1], stVideoFrame.stVFrame.u64VirAddr[2]);

        s32Fd = open("/var/tmp.yuv", O_CREAT | O_RDWR, S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH);
        if (s32Fd > 0)
        {
            // 提取y分量
            pvBuf = HI_MPI_SYS_Mmap(stVideoFrame.stVFrame.u64PhyAddr[0], stVideoFrame.stVFrame.u32Width*stVideoFrame.stVFrame.u32Height);
            if (NULL == pvBuf)
            {
                print_level(SV_ERROR, "HI_MPI_SYS_Mmap failed.\n");
            }
            else
            {
                s32Ret = write(s32Fd, pvBuf, stVideoFrame.stVFrame.u32Width*stVideoFrame.stVFrame.u32Height);
                if (s32Ret < 0)
                {
                    print_level(SV_ERROR, "write failed. [err:%s]\n", strerror(errno));
                }
                HI_MPI_SYS_Munmap(pvBuf, stVideoFrame.stVFrame.u32Width*stVideoFrame.stVFrame.u32Height);
#if 1   // 加入uv分量
                pvBuf = HI_MPI_SYS_Mmap(stVideoFrame.stVFrame.u64PhyAddr[1], stVideoFrame.stVFrame.u32Width*stVideoFrame.stVFrame.u32Height);
                if (NULL == pvBuf)
                {
                    print_level(SV_ERROR, "HI_MPI_SYS_Mmap failed.\n");
                }
                else
                {
                    s32Ret = write(s32Fd, pvBuf, stVideoFrame.stVFrame.u32Width*stVideoFrame.stVFrame.u32Height);
                    if (s32Ret < 0)
                    {
                        print_level(SV_ERROR, "write failed. [err:%s]\n", strerror(errno));
                    }
                    HI_MPI_SYS_Munmap(pvBuf, stVideoFrame.stVFrame.u32Width*stVideoFrame.stVFrame.u32Height);
                }
#endif
            }
            close(s32Fd);
            rename("/var/tmp.yuv", "/var/bsd.yuv");
        }

        s32Ret = HI_MPI_VPSS_ReleaseChnFrame(VpssGrp, VpssChn, &stVideoFrame);
        if (HI_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "HI_MPI_VPSS_ReleaseChnFrame failed. [err=%#x]\n", s32Ret);
        }

        u32Cnt++;
        if (0 == u32Cnt % 5)
        {
            //bsd_SubmitEvent();
            s32Ret = Msg_submitEvent(EP_CONTROL, OP_EVENT_BSD, NULL);
            if (SV_SUCCESS != s32Ret)
            {
                print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
            }
        }
    }

    return NULL;
}

/******************************************************************************
 * 函数功能: 初始化BSD模块
 * 输入参数: 无
 * 输出参数: 无
 * 返回值  : SV_SUCCESS --- 成功
             SV_FAILURE --- 其它错误
 * 注意    : 无
 *****************************************************************************/
sint32 BSD_Init()
{
    sint32 s32Ret = 0;

    memset(&m_stBsdInfo, 0, sizeof(BSD_INFO_S));
    
    return SV_SUCCESS;
}

/******************************************************************************
 * 函数功能: 去初始化BSD模块
 * 输入参数: 无
 * 输出参数: 无
 * 返回值  : SV_SUCCESS --- 成功
             SV_FAILURE --- 其它错误
 * 注意    : 无
 *****************************************************************************/
sint32 BSD_Fini()
{
    
    return SV_SUCCESS;
}

/******************************************************************************
 * 函数功能: 启动BSD模块
 * 输入参数: 无
 * 输出参数: 无
 * 返回值  : SV_SUCCESS --- 成功
             SV_FAILURE --- 其它错误
 * 注意    : 无
 *****************************************************************************/
sint32 BSD_Start()
{
    sint32 s32Ret = 0;
    pthread_t thread = 0;

    m_stBsdInfo.bRunning = SV_TRUE;
    s32Ret = pthread_create(&thread, NULL, bsd_Handle_Body, &m_stBsdInfo);
    if (0 != s32Ret)
    {
        print_level(SV_ERROR, "pthread_create failed. [err: %s]\n", strerror(errno));
        return SV_FAILURE;
    }

    m_stBsdInfo.u32Tid = thread;
    
    return SV_SUCCESS;
}

/******************************************************************************
 * 函数功能: 停止BSD模块
 * 输入参数: 无
 * 输出参数: 无
 * 返回值  : SV_SUCCESS --- 成功
             SV_FAILURE --- 其它错误
 * 注意    : 无
 *****************************************************************************/
sint32 BSD_Stop()
{
    sint32 s32Ret = 0;
    pthread_t thread = m_stBsdInfo.u32Tid;
    void *pvRetval = NULL;

    m_stBsdInfo.bRunning = SV_FALSE;
    s32Ret = pthread_join(thread, &pvRetval);
    if (0 != s32Ret)
    {
        print_level(SV_ERROR, "pthread_join failed. [err: %s]\n", strerror(errno));
        return SV_FAILURE;
    }
    
    return SV_SUCCESS;
}


/* 中断退出 */
static void exit_handle(int signalnum)
{
    BSD_Stop();
    BSD_Fini();
    MSG_ReciverStop(EP_ALG);
    exit(EXIT_FAILURE);
}

int ipsys_log_level = SV_DEBUG;

int main(int argc, char **argv)
{
    sint32 s32Ret = 0;

    if (argc == 2 && atoi(argv[1]) >= SV_ERROR && atoi(argv[1]) <= SV_ALWAYS)
    {
        ipsys_log_level = atoi(argv[1]);
    }
    
    /*捕获进程退出的系统消息*/
    if (SIG_ERR == signal(SIGINT, exit_handle))
    {
        printf("catch signal SIGKILL Error: %d, %s\n", errno, strerror(errno));
    }

    /*忽略PIPE消息*/
    if (SIG_ERR == signal(SIGPIPE, SIG_IGN))
    {
        printf("catch signal SIGPIPE Error: %d, %s\n", errno, strerror(errno));
    }

    s32Ret = MSG_SysInit(SV_FALSE);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "MSG_SysInit failed. [err=%#x]\n", s32Ret);
        return -1;
    }

    s32Ret = MSG_ReciverStart(EP_ALG);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "MSG_ReciverStart failed. [err=%#x]\n", s32Ret);
        return -1;
    }

    s32Ret = BSD_Init();
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "BSD_Init failed. [err=%#x]\n", s32Ret);
        return -1;
    }
    
    s32Ret = BSD_Start();
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "BSD_Start failed. [err=%#x]\n", s32Ret);
        return -1;
    }
    
    while (1)
    {
        sleep(1);
    }

    s32Ret = BSD_Stop();
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "BSD_Stop failed. [err=%#x]\n", s32Ret);
    }
    
    s32Ret = BSD_Fini();
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "BSD_Fini failed. [err=%#x]\n", s32Ret);
    }

    s32Ret = MSG_ReciverStop(EP_ALG);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "MSG_ReciverStop failed. [err=%#x]\n", s32Ret);
    }

    return 0;
}

