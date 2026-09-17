/******************************************************************************
Copyright (C) 广州敏视数码科技有限公司版权所有.

文件名：DynamicTrace.c

作者: 郑南城    版本: v1.0.0(初始版本号)   日期: 2020-08-26

文件功能描述: 利用OSD模块和Track模块进行倒车轨迹在IPC上实时显示

*******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>

#include <sys/types.h>

#include <sys/stat.h>

#include <math.h>

#ifndef __HuaweiLite__ 
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/msg.h>
#include <linux/fb.h>
#include <error.h>
#endif

#include <sys/ioctl.h>
#include <sys/prctl.h>
#include <fcntl.h>
#include <time.h>
#include <pthread.h>
#include <errno.h>

#include "print.h"
#include "common.h"
#include "hi_common.h"
#include "mpi_region.h"
#include "hi_type.h"

#include <signal.h>
#include <hi_comm_sys.h>

#include <mpi_sys.h>
#include "./common/trace_common.h"
#include "./common/camera_map.h"

#include "Track.h"


#define WIDTH  1920
#define HEIGHT 1080



/* 区域状态 */
typedef struct tagRgnStat_S
{
    SV_BOOL     bCreated;       /* 是否被创建 */
    SV_BOOL     bAttached;      /* 是否被绑定 */
    RGN_HANDLE  u32Handle;      /* 区域句柄值 */
    HI_S32      s32ChnId;       /* 绑定通道 */
    SV_BOOL     bShow;          /* 区域是否显示 */
    SV_RECT_S   stRect;         /* 区域位置及大小 */
} MPP_VOSD_RGN_S;

/* 视频叠加区域 */
typedef struct tagOverlayRgn_S
{
    MPP_VOSD_RGN_S stCarTraceRgn;   /* 倒车轨迹叠加区域 */
} MPP_OVERLAY_RGN_S;


//叠加车辆轨迹
typedef struct tagCarTraceState_s
{
    double      radius;             //半径
    SV_BOOL     bUpdate;            //更新标志位
    HI_VOID    *pvir_car;           //位图缓冲区指针，虚拟地址
    HI_U64      pphy_car;           //位图缓冲，物理地址

} MPP_CarTrace_S;

//视频遮挡与控制信息
typedef struct tagVosdInfo_CarTrace
{
    uint32 u32ChnNum;                   //通道数
    MPP_OVERLAY_RGN_S   astChnOverlay;  //视频叠加区域
    MPP_CarTrace_S   astCarTrace;       //车辆轨迹叠加

    
    uint32 u32TID;                      //视频叠加线程ID
    SV_BOOL bRunning;                   //视频线程是否运行
    SV_BOOL bException;                 //线程是否异常
    uint32 u32TID_Trace;                //车辆追迹线程
    uint32 bRunning_Trace;              //车辆追迹线程是否运行
    SV_BOOL bException_Trace;                 //车辆追迹线程是否异常

    pthread_mutex_t mutexLock;//参数设置线程互斥锁
} MPP_VOSD_INFO_CarTrace;

sint32 DynamicTrace_Init(MPP_VOSD_INFO_CarTrace *m_stVosdTraceInfo);
sint32 DynamicTrace_Fini(MPP_VOSD_INFO_CarTrace *m_stVosdTraceInfo);
sint32 DynamicTrace_Start(MPP_VOSD_INFO_CarTrace *m_stVosdTraceInfo);
sint32 DynamicTrace_Stop(MPP_VOSD_INFO_CarTrace *m_stVosdTraceInfo);
void   *DynamicTrace_mpp_vosd_Body(MPP_VOSD_INFO_CarTrace *m_stVosdTraceInfo);
void   *DynamicTrace_trace_body();

sint32 mpp_vosd_CreateOverlayRgn_CarTrace(MPP_VOSD_INFO_CarTrace *m_stVosdTraceInfo);
sint32 mpp_vosd_DestroyOverlayRgn_CarTrace(MPP_VOSD_INFO_CarTrace *m_stVosdTraceInfo);
sint32 mpp_vosd_UpdateCarTraceBmp(MPP_VOSD_INFO_CarTrace *m_stVosdTraceInfo);

sint32 DynamicTrace_curve_Paint(MPP_VOSD_INFO_CarTrace *m_stVosdTraceInfo);

// 全局控制信息
extern MPP_VOSD_INFO_CarTrace m_stVosdTraceInfo = {0};
// 相机俯仰角和翻滚角
double mpp_roll,mpp_pitch;
// ipsys信息配置
int ipsys_log_level = SV_INFO;
// 画布尺寸
int MPP_VOSD_CARTRACE_BUFSIZE; 


/******************************************************************************
 * 函数功能: VOSD模块和追迹初始化
 * 输入参数: m_stVosdTraceInfo --- 视频遮挡叠加配置参数
 * 输出参数: 无
 * 返回值  : SV_SUCCESS - 成功
             SV_FAILURE - 其它错误
 * 注意    : 无
 *****************************************************************************/
sint32 DynamicTrace_Init(MPP_VOSD_INFO_CarTrace *m_stVosdTraceInfo)
{
    sint32 i;
    sint32 s32Ret = 0;
    HI_VOID **pvir_car;
    HI_U64  *pphy_car;

    printf("perform DynamicTrace_Init\n");
    if(NULL == m_stVosdTraceInfo)
    {
        return ERR_NULL_PTR;
    }
    //重置设备信息，防止绑定错误
    DynamicTrace_Fini(m_stVosdTraceInfo);

    //-----------------------------------------------------------------------------------------------
    //VOSD模块初始化
    pphy_car = &(m_stVosdTraceInfo->astCarTrace.pphy_car);
    pvir_car = &(m_stVosdTraceInfo->astCarTrace.pvir_car);
    
    *pvir_car = malloc(MPP_VOSD_CARTRACE_BUFSIZE);

    s32Ret = pthread_mutex_init(&(m_stVosdTraceInfo->mutexLock), NULL);
    if (0 != s32Ret)
    {
        print_level(SV_ERROR, "pthreada_mutex_init faild! [err=%#x]\n", s32Ret);
        HI_MPI_SYS_MmzFree(*pphy_car,*pvir_car);
        return ERR_SYS_NOTREADY;
    }

    s32Ret = mpp_vosd_CreateOverlayRgn_CarTrace(m_stVosdTraceInfo);
    if(SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "mpp_vosd_CreateOverlayRgn_CarTrace faild! [err=%#x]\n",s32Ret);
        HI_MPI_SYS_MmzFree(*pphy_car,*pvir_car);
        return s32Ret;
    }
    //保持画布更新update更新
    m_stVosdTraceInfo->astCarTrace.bUpdate = SV_TRUE;
    //-----------------------------------------------------------------------------------------------
    

    //-----------------------------------------------------------------------------------------------
    //Trace 踪迹初始化和信息配置
    s32Ret = Track_Init();
    if(s32Ret == SV_SUCCESS)
    {
        printf("Init Track Success!!\n");
    }

    IMU_getRoll(&mpp_roll);
    IMU_getPitch(&mpp_pitch);
    Camera_Angle_Set(mpp_roll, mpp_pitch);
    Camera_Canvas_Set((int)(m_stVosdTraceInfo->astChnOverlay.stCarTraceRgn.stRect.s32X),
                      (int)(m_stVosdTraceInfo->astChnOverlay.stCarTraceRgn.stRect.s32Y));
    //-----------------------------------------------------------------------------------------------
    

    return SV_SUCCESS;
}

/******************************************************************************
 * 函数功能: 去初始化VOSD模块和Track模块
 * 输入参数: 无
 * 输出参数: 无
 * 返回值  : SV_SUCCESS - 成功
             SV_FAILURE - 其它错误
 * 注意    : 无
 *****************************************************************************/
sint32 DynamicTrace_Fini(MPP_VOSD_INFO_CarTrace *m_stVosdTraceInfo)
{
    sint32 i;
    sint32 s32Ret = 0;
    printf("perform DynamicTrace_Fini\n");
    //-----------------------------------------------------------------------------------------------
    //OSD模块去初始化
    s32Ret = mpp_vosd_DestroyOverlayRgn_CarTrace(m_stVosdTraceInfo);
    if(SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "mpp_vosd_DestroyOverlayRgn_CarTrace failed! [err=%#x]\n",s32Ret);
        //return s32Ret;
    }

    s32Ret = pthread_mutex_destroy(&m_stVosdTraceInfo->mutexLock);
    if(SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "pthread_mutex_destroy failed! [err=%#x]\n", s32Ret);
        //return SV_FAILURE;
    }

    s32Ret = HI_MPI_SYS_MmzFree(m_stVosdTraceInfo->astCarTrace.pphy_car,
                                m_stVosdTraceInfo->astCarTrace.pvir_car);
    if(SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "HI_MPI_SYS_MmzFree failed! [err=%#x]\n", s32Ret);
        //return s32Ret;
    }
    m_stVosdTraceInfo->astCarTrace.pvir_car = NULL;
    //-----------------------------------------------------------------------------------------------

    Track_Exit();
    return s32Ret;
}

/******************************************************************************
 * 函数功能: 启动 DynamicTrace， 动态更新轨迹
 * 输入参数: 无
 * 输出参数: 无
 * 返回值  : SV_SUCCESS - 成功
 * 注意    : 无
 *****************************************************************************/
sint32 DynamicTrace_Start(MPP_VOSD_INFO_CarTrace *m_stVosdTraceInfo)
{
    sint32 s32Ret = 0;
    pthread_t thread;
    pthread_t trace_thread;
    printf("perform DynamicTrace_Start\n");
    m_stVosdTraceInfo->bRunning = SV_TRUE;
    m_stVosdTraceInfo->bException = SV_FAILURE;
    m_stVosdTraceInfo->bRunning_Trace = SV_TRUE;
    m_stVosdTraceInfo->bException_Trace = SV_FAILURE;
    
    s32Ret = pthread_create(&thread, NULL, DynamicTrace_mpp_vosd_Body, m_stVosdTraceInfo);
    if(SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Start thread for VOSD failed! [err=%#x] in mpp_vosd_Body\n",s32Ret);
        return s32Ret;
    }
    
   
    s32Ret = pthread_create(&trace_thread, NULL, DynamicTrace_trace_body,NULL);
    if(SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Start thread for VOSD failed! [err=%#x] in track_body\n",s32Ret);
        return s32Ret;
    }
    
    m_stVosdTraceInfo->u32TID = thread;
    m_stVosdTraceInfo->u32TID_Trace = trace_thread;
    return SV_SUCCESS;
}

/******************************************************************************
 * 函数功能: 阻塞 DynamicTrace 模块
 * 输入参数: 无
 * 输出参数: 无
 * 返回值  : SV_SUCCESS - 成功
 * 注意    : 无
 *****************************************************************************/
sint32 DynamicTrace_Stop(MPP_VOSD_INFO_CarTrace *m_stVosdTraceInfo)
{
    sint32 s32Ret = 0;
    void *pvRetval = NULL;
    printf("perform DynamicTrace_Stop\n");
    //阻塞当前进程，获取返回值
    s32Ret = pthread_join(m_stVosdTraceInfo->u32TID, &pvRetval);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Stop thread for VOSD failed! [err=%#x] in u32TID\n",s32Ret);
        return s32Ret;
    }
    s32Ret = pthread_join(m_stVosdTraceInfo->u32TID_Trace, &pvRetval);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Stop thread for VOSD failed! [err=%#x] in u32TID_Trace\n",s32Ret);
        return s32Ret;
    }
    return SV_SUCCESS;
}

/******************************************************************************
 * 函数功能: OSD 轨迹显示模块线程体
 * 输入参数: m_stVosdTraceInfo --- 视频编码控制信息
 * 输出参数: 无
 * 返回值  : 无
 * 注意    : 无
 *****************************************************************************/
void* DynamicTrace_mpp_vosd_Body(MPP_VOSD_INFO_CarTrace *m_stVosdTraceInfo)
{
    sint32 s32Ret = 0;
    double pre_radius;

    //设置进程名
    s32Ret = prctl(PR_SET_NAME, "DynamicTrace_mpp_vosd_Body");
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "prctl PR_SET_NAME failed! [err:%s]\n",strerror(errno));
    }
    printf("perform DynamicTrace_mpp_vosd_Body\n");
    //return NULL;
    m_stVosdTraceInfo->astCarTrace.bUpdate = SV_TRUE;
    m_stVosdTraceInfo->astCarTrace.radius = 1e3;    //赋予较大的初值
    mpp_vosd_UpdateCarTraceBmp(m_stVosdTraceInfo);
    
    while (m_stVosdTraceInfo->bRunning)
    {
        
        select_sleep(0,4*1000);
        
        pre_radius = m_stVosdTraceInfo->astCarTrace.radius;
        //半径太大时不更新
        
        if(abs(Track_getRadius(&(m_stVosdTraceInfo->astCarTrace.radius)))>=1e3 && pre_radius >= 1e3)
        {
            continue;
        }
        
        
        m_stVosdTraceInfo->astCarTrace.bUpdate = SV_TRUE;
        mpp_vosd_UpdateCarTraceBmp(m_stVosdTraceInfo);
        
    }
    
    
    return NULL;
}
/******************************************************************************
 * 函数功能: Track 更新轨迹半径线程体
 * 输入参数: 无
 * 输出参数: 无
 * 返回值  : 无
 * 注意    : 无
 *****************************************************************************/
void* DynamicTrace_trace_body()
{
    sint32 s32Ret;
    printf("perform DynamicTrace_trace_body\n");
    s32Ret = prctl(PR_SET_NAME, "DynamicTrace_trace_body");
    
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "prctl PR_SET_NAME failed! [err:%s]\n",strerror(errno));
    }
    //实时刷新
    Track_Realtime_Update();
    return NULL;
}
/******************************************************************************
 * 函数功能: 创建通道的视频叠加区域
 * 输入参数: 无
 * 输出参数: 无
 * 返回值  : 无
 * 注意    : 无
 *****************************************************************************/
sint32 mpp_vosd_CreateOverlayRgn_CarTrace(MPP_VOSD_INFO_CarTrace *m_stVosdTraceInfo)
{
    sint32 s32Ret;
    RGN_HANDLE u32Handle;
    MPP_CHN_S stChn;
    RGN_ATTR_S stRgnAttr;
    RGN_CHN_ATTR_S stChnAttr;

    if (NULL == m_stVosdTraceInfo)
    {
        return ERR_NULL_PTR;
    }
    
    stChn.enModId = HI_ID_VENC;
    stChn.s32DevId = 0;
    stChn.s32ChnId = m_stVosdTraceInfo->astChnOverlay.stCarTraceRgn.s32ChnId;
    u32Handle = m_stVosdTraceInfo->astChnOverlay.stCarTraceRgn.u32Handle;

    stRgnAttr.enType = OVERLAY_RGN;     //设定叠加区域类型
    stRgnAttr.unAttr.stOverlay.enPixelFmt = PIXEL_FORMAT_ARGB_1555;     //设定像素格式
    stRgnAttr.unAttr.stOverlay.stSize.u32Width = m_stVosdTraceInfo->astChnOverlay.stCarTraceRgn.stRect.u32Width;
    stRgnAttr.unAttr.stOverlay.stSize.u32Height = m_stVosdTraceInfo->astChnOverlay.stCarTraceRgn.stRect.u32Height;
    stRgnAttr.unAttr.stOverlay.u32BgColor = 0;     //设定区域背景色
    stRgnAttr.unAttr.stOverlay.u32CanvasNum = 2;        //设定内存范围

    s32Ret = HI_MPI_RGN_Create(u32Handle, &stRgnAttr);
    if(HI_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "mpp_vosd_CreateOverlayRgn_CarTrace failed! [err=%#x]\n",s32Ret);
        return s32Ret;
    }
    
    stChnAttr.bShow  = 1;
    stChnAttr.enType = OVERLAY_RGN;
    stChnAttr.unChnAttr.stOverlayChn.stPoint.s32X = m_stVosdTraceInfo->astChnOverlay.stCarTraceRgn.stRect.s32X;
    stChnAttr.unChnAttr.stOverlayChn.stPoint.s32Y = m_stVosdTraceInfo->astChnOverlay.stCarTraceRgn.stRect.s32Y;

    stChnAttr.unChnAttr.stOverlayChn.u32BgAlpha   = 0;
    stChnAttr.unChnAttr.stOverlayChn.u32FgAlpha   = 127;
    stChnAttr.unChnAttr.stOverlayChn.u32Layer     = 1;
    stChnAttr.unChnAttr.stOverlayChn.stQpInfo.bQpDisable = HI_FALSE;
    stChnAttr.unChnAttr.stOverlayChn.stQpInfo.bAbsQp = HI_FALSE;
    stChnAttr.unChnAttr.stOverlayChn.stQpInfo.s32Qp  = 0;
    stChnAttr.unChnAttr.stOverlayChn.stInvertColor.stInvColArea.u32Height = 16;
    stChnAttr.unChnAttr.stOverlayChn.stInvertColor.stInvColArea.u32Width  = 16;
    stChnAttr.unChnAttr.stOverlayChn.stInvertColor.u32LumThresh = 128;
    stChnAttr.unChnAttr.stOverlayChn.stInvertColor.enChgMod     = LESSTHAN_LUM_THRESH;
    stChnAttr.unChnAttr.stOverlayChn.stInvertColor.bInvColEn    = HI_FALSE;
    stChnAttr.unChnAttr.stOverlayChn.enAttachDest = ATTACH_JPEG_MAIN;
    stChnAttr.unChnAttr.stOverlayChn.u16ColorLUT[0] = 0x2abc;
    stChnAttr.unChnAttr.stOverlayChn.u16ColorLUT[1] = 0x7FF0;
    print_level(SV_DEBUG, "handle:%d, time(%d,%d)\n", u32Handle, m_stVosdTraceInfo->astChnOverlay.stCarTraceRgn.stRect.s32X, 
                                                      m_stVosdTraceInfo->astChnOverlay.stCarTraceRgn.stRect.s32Y);
	
    s32Ret = HI_MPI_RGN_AttachToChn(u32Handle, &stChn, &stChnAttr);
    if(HI_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "HI_MPI_RGN_AttachToChn failed! [err=%#x]\n",s32Ret);
        return SV_FAILURE;
    }

    m_stVosdTraceInfo->astChnOverlay.stCarTraceRgn.bCreated = SV_TRUE;
    m_stVosdTraceInfo->astChnOverlay.stCarTraceRgn.bAttached = SV_TRUE;

    
    return SV_SUCCESS;
}
/******************************************************************************
 * 函数功能: 销毁通道的视频叠加区域
 * 输入参数: m_stVosdTraceInfo
 * 输出参数: 无
 * 返回值  : 无
 * 注意    : 无
 *****************************************************************************/
sint32 mpp_vosd_DestroyOverlayRgn_CarTrace(MPP_VOSD_INFO_CarTrace *m_stVosdTraceInfo)
{
    sint32 s32Ret = 0;
    RGN_HANDLE u32Handel = 12;
    MPP_CHN_S stChn;
    
    u32Handel = m_stVosdTraceInfo->astChnOverlay.stCarTraceRgn.u32Handle;
    stChn.enModId = HI_ID_VENC;
    stChn.s32DevId = 0;
    stChn.s32ChnId = m_stVosdTraceInfo->astChnOverlay.stCarTraceRgn.s32ChnId;

    s32Ret = HI_MPI_RGN_DetachFromChn(u32Handel, &stChn);

    if(HI_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "HI_MPI_RGN_Destroy failed! [err=%#x]\n",s32Ret);
    }
    s32Ret = HI_MPI_RGN_Destroy(m_stVosdTraceInfo->astChnOverlay.stCarTraceRgn.u32Handle);
    if(HI_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "HI_MPI_RGN_Destroy failed [err=%#x]\n",s32Ret);
    }
    m_stVosdTraceInfo->astChnOverlay.stCarTraceRgn.bCreated = SV_FALSE;
    return SV_SUCCESS;
}
/******************************************************************************
 * 函数功能: 更新倒车轨迹位图
 * 输入参数: m_stVosdTraceInfo
 * 输出参数: 无
 * 返回值  : 无
 * 注意    : 无
 *****************************************************************************/
sint32 mpp_vosd_UpdateCarTraceBmp(MPP_VOSD_INFO_CarTrace *m_stVosdTraceInfo)
{
    sint32 i;
    sint32 s32Ret = 0;
    RGN_HANDLE u32Handel;
    BITMAP_S stBitmap;

    RECT_S rect;
    rect.s32X = 50;
    rect.s32Y = 50;
    rect.u32Height = 50;
    rect.u32Width = 50;
    

    if(SV_FALSE==m_stVosdTraceInfo->astCarTrace.bUpdate)
    {
        return SV_FAILURE;
    }

    s32Ret = DynamicTrace_paint_Curve(m_stVosdTraceInfo);
    if(SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "mpp_ted_Fill failed! [err=%#x]\n", s32Ret);
        return SV_FAILURE;
    }

    u32Handel = m_stVosdTraceInfo->astChnOverlay.stCarTraceRgn.u32Handle;
    stBitmap.enPixelFormat = PIXEL_FORMAT_ARGB_1555;
    stBitmap.u32Width = m_stVosdTraceInfo->astChnOverlay.stCarTraceRgn.stRect.u32Width;
    stBitmap.u32Height = m_stVosdTraceInfo->astChnOverlay.stCarTraceRgn.stRect.u32Height;
    stBitmap.pData = m_stVosdTraceInfo->astCarTrace.pvir_car;

    s32Ret = HI_MPI_RGN_SetBitMap(u32Handel, &stBitmap);
    if(SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "HI_MPI_RGN_SetBitMap failed! [err=%#x]\n", s32Ret);
    }
    return SV_SUCCESS;
}
/******************************************************************************
 * 函数功能: 动态绘制倒车轨迹图像
 * 输入参数: m_stVosdTraceInfo
 * 输出参数: 无
 * 返回值  : 无
 * 注意    : 无
 *****************************************************************************/
sint32 DynamicTrace_paint_Curve(MPP_VOSD_INFO_CarTrace *m_stVosdTraceInfo)
{
    if(SV_FALSE == m_stVosdTraceInfo->astCarTrace.bUpdate)
    {
        return SV_FAILURE;
    }
    
    sint32 s32Ret;
    HI_U16 *pbmp = m_stVosdTraceInfo->astCarTrace.pvir_car;
    HI_U16 u32FillData = 0x8fe0;
    HI_U32 width, height;
    pixel_point *points;
    int lens = 30;
    
    double theta_start = 0, theta_end=M_PI/2.1;
    double radius = m_stVosdTraceInfo->astCarTrace.radius;
    points = (pixel_point*)malloc(sizeof(pixel_point)*lens);

    width = m_stVosdTraceInfo->astChnOverlay.stCarTraceRgn.stRect.u32Width;
    height = m_stVosdTraceInfo->astChnOverlay.stCarTraceRgn.stRect.u32Height;
    
    s32Ret = hi_memset(pbmp,MPP_VOSD_CARTRACE_BUFSIZE,0x0,MPP_VOSD_CARTRACE_BUFSIZE);
    if(EOK != s32Ret)
    {
        print_level(SV_ERROR, "hi_memset failed!");
        free(points);
        return -1;
    }
    
    Camera_LeftTrace_Map(points,lens,theta_start,theta_end,radius);
    paint_MultiLine_uint16(pbmp,(int)width,(int)height,points,lens,u32FillData,20);
    Camera_RightTrace_Map(points,lens,theta_start,theta_end,radius);
    paint_MultiLine_uint16(pbmp,(int)width,(int)height,points,lens,u32FillData,20);
    free(points);

    return SV_SUCCESS;
}
/******************************************************************************
 * 函数功能: 进程退出处理
 * 输入参数: sig 退出信号
 * 输出参数: 无
 * 返回值  : 无
 * 注意    : 无
 *****************************************************************************/
void process_exit(int sig){
	//删除通道叠加区域
	sint32 s32Ret = 0;
	VENC_CHN VeChnId = 2;
	printf("\ninterrupt process:%d !!\n",sig);
	//s32Ret = mpp_vosd_DestroyOverlayRgn_CarTrace(&m_stVosdTraceInfo);
    
    
    s32Ret = DynamicTrace_Fini(&m_stVosdTraceInfo);
	if(SV_SUCCESS != s32Ret)
	{
		print_level(SV_ERROR, "mpp_vosd_DestroyOverlayRgn_CarTrace failed! [err=%x]\n",s32Ret);
	}
	exit(0);
}

/******************************************************************************
 * 函数功能: 进程退出信号注册
 * 输入参数: 无
 * 输出参数: 无
 * 返回值  : 无
 * 注意    : 无
 *****************************************************************************/
void signal_registed()
{
    printf("perform signal_registed\n");
	signal(SIGINT,process_exit);
	signal(SIGFPE,process_exit);
	signal(SIGILL,process_exit);
	signal(SIGABRT,process_exit);
	signal(SIGABRT,process_exit);
	signal(SIGTERM,process_exit);
}



int main()
{
    printf("DynamicTrace main start\n");
	signal_registed();
    
    m_stVosdTraceInfo.astChnOverlay.stCarTraceRgn.u32Handle = 12;          //叠加句柄
    m_stVosdTraceInfo.astChnOverlay.stCarTraceRgn.s32ChnId  = 2;           //2通道对应IPC
    m_stVosdTraceInfo.astChnOverlay.stCarTraceRgn.stRect.s32X = 0;
    m_stVosdTraceInfo.astChnOverlay.stCarTraceRgn.stRect.s32Y = 600;
    m_stVosdTraceInfo.astChnOverlay.stCarTraceRgn.stRect.u32Height = 1080 - 600;
    m_stVosdTraceInfo.astChnOverlay.stCarTraceRgn.stRect.u32Width = 1920;//1920 - 2*(m_stVosdTraceInfo.astChnOverlay.stCarTraceRgn.stRect.s32X);
    m_stVosdTraceInfo.astChnOverlay.stCarTraceRgn.bShow = HI_TRUE;
    m_stVosdTraceInfo.astCarTrace.bUpdate = SV_TRUE;
    
    MPP_VOSD_CARTRACE_BUFSIZE = 2*(m_stVosdTraceInfo.astChnOverlay.stCarTraceRgn.stRect.u32Width*m_stVosdTraceInfo.astChnOverlay.stCarTraceRgn.stRect.u32Height);
    DynamicTrace_Init(&m_stVosdTraceInfo);
    DynamicTrace_Start(&m_stVosdTraceInfo);
    DynamicTrace_Stop(&m_stVosdTraceInfo);
	
    return 0;
}








