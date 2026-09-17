/******************************************************************************
Copyright (C) 2021-2023 广州敏视数码科技有限公司版权所有.

文件名：pd.c

日期: 2021-08-03

文件功能描述: 定义行人检测算法功能接口

其他: // 其他内容说明

版本: v1.0.0(最新版本号)

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
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>
#include <dirent.h>

#include "print.h"
#include "../../../include/common.h"
#include "safefunc.h"
#include "op.h"
#include "msg.h"
#include "alarm.h"
#include "avalarmer.h"
#include "pds_alg.h"
#include "apc.h"

#include "config.h"
#include "media.h"
#include "media_sem.h"
#include "media_shm.h"
#include "alg.h"
#include "cJSON.h"
#include "utils.h"
#include "board.h"

#include "CaliEX.h"
#include <jpeg/jpeglib.h>
#include <jpeg/jerror.h>

#define min(x, y) (((x) < (y)) ? (x) : (y))
#define max(x, y) (((x) > (y)) ? (x) : (y))
#define abs(x) ((x)<0? -(x) : (x))

#define ALG_MAX(a,b)    (((a) > (b)) ? (a) : (b))
#define ALG_MIN(a,b)    (((a) < (b)) ? (a) : (b))


#define APC_MODEL_RGB_HEAD   "/root/RGB_HEAD.rknn"      /* 可见光点人数模型 */
#define APC_IMAGE_SRC_WIDTH     608                     /* 算法图像帧源宽度 */
#define APC_IMAGE_SRC_HEIGHT    352                     /* 算法图像帧源高度 */
#define APC_IMAGE_DST_WIDTH     608                     /* 算法图像帧目的宽度 */
#define APC_IMAGE_DST_HEIGHT    352                     /* 算法图像帧目的高度 */


#pragma  pack(1)
typedef struct tag_Type24
{
    char buf[3];
} TYPE24;
#pragma  pack()

typedef struct tagApcGuiImg_S
{
    char *pbmp;
    sint32 s32Width;
    sint32 s32Height;
} APC_GUI_IMG_S;

typedef struct tagApcDumpInfo_S
{
    sint64 s64TimeStamp;           /* 时间戳 */
    uint32 u32InNumber;     /* 上车人数 */
    uint32 u32OutNumber;    /* 下车人数 */
    uint32 u32RectNumber;   /* 探测区域内的人数 */
} APC_DUMP_INFO_S;


typedef enum tagApcAlarmType_E
{
    APC_ALARM_NONE = 0,     /* 无报警 */
    APC_ALARM_PERSON_ON,    /* 上车报警 */
    APC_ALARM_PERSON_OFF,   /* 下车报警 */
    APC_ALARM_OVERLOAD,     /* 超载报警 */
    APC_ALARM_CROWDED,      /* 拥挤报警 */

    APC_ALARM_BUTT,
} APC_ALARM_TYPE;

/* 音频状态 */
typedef struct tagApcAudioState
{
    APC_ALARM_TYPE enAlarm; /* 报警状态 */
} APC_AUDIO_STATE;

typedef struct tagAlarmOutState
{
    APC_ALARM_TYPE enAlarm; /* 报警状态 */
} APC_ALARMOUT_STATE;

typedef struct tagSubThreadState
{
    APC_AUDIO_STATE stAudio;        /* 音频线程状态 */
    APC_ALARMOUT_STATE stAlarmOut;  /* AlarmOut线程状态 */
} APC_SUB_THREAD_STATE;

/* 模块控制信息 */
typedef struct tagApcInfo_S
{
    uint32          u32ChnNum;              /* APC 算法通道数目 */
    pdsa32::CPdsAlg *apcsPdsAlg[ALG_MAX_CHN]; /* APC 算法对象指针 */
    pdsa32::STTrackParam stTrackParam;      /* PD算法跟踪参数 */
    pdsa32::STPostParam  stPostParam;       /* APC算法位置参数 */
#if ALG_MUTLIT_BUFFER
    sint32          as32MediaBufFd[4][3];      /* 媒体通道Media Buffer的文件描述符 */
#else
    sint32          as32MediaBufFd[4];      /* 媒体通道Media Buffer的文件描述符 */
#endif
    CFG_ALG_PARAM   stCfgParam;             /* 算法配置参数 */
    APC_SUB_THREAD_STATE stSubThreadState;  /* 子线程状态 */
    uint32          u32TidAlg;              /* 算法线程ID */
    uint32          u32TidAudio;            /* 音频报警线程ID */
    uint32          u32TidAlarmOut;         /* AlarmOut线程ID */
    SV_BOOL         bRunning;               /* 线程是否正在运行 */
    pthread_mutex_t mutexRunStat;           /* 算法运行状态互斥锁 */
} APC_INFO_S;


APC_INFO_S m_stApcInfo = {0};             /* 模块控制信息 */
extern int ipsys_log_level;


void apc_print_delay_time()
{
    static struct timeval pre_stTm = {0};
    static struct timeval now_stTm = {0};
    int delay_time = 0;
    
    gettimeofday(&now_stTm, NULL);
    delay_time = (now_stTm.tv_sec - pre_stTm.tv_sec) * 1000 + (now_stTm.tv_usec - pre_stTm.tv_usec) / 1000;

    printf("apc_print_delay_time:%d(ms)\n", delay_time);

    pre_stTm = now_stTm;
}


/******************************************************************************
 * 函数功能: 缩放图片
 * 输入参数: pstPdImgDst --- 目标位图数据
             pstPdImgSrc --- 源位图数据
 * 输出参数: 无
 * 返回值  : 无
 *****************************************************************************/
sint32 apc_Gui_Zoom(APC_GUI_IMG_S *pstApcImgDst, APC_GUI_IMG_S *pstApcImgSrc)
{
    sint32 s32Ret;

    if(pstApcImgDst == NULL || pstApcImgSrc == NULL)
        return ERR_NULL_PTR;

    if((0 == pstApcImgDst->s32Height) || (0 == pstApcImgDst->s32Width) ||
       (0 == pstApcImgSrc->s32Height) || (0 == pstApcImgSrc->s32Width))
        return SV_SUCCESS;

    unsigned long xrIntFloat_16 = (pstApcImgSrc->s32Width << 16) / pstApcImgDst->s32Width + 1;
    unsigned long yrIntFloat_16 = (pstApcImgSrc->s32Height << 16) / pstApcImgDst->s32Height + 1;
    unsigned long dst_width = pstApcImgDst->s32Width;

    TYPE24 *pDstLine, *pSrcLine;
    char *pbmp_dst, *pbmp_src;
    unsigned long srcy_16 = 0, srcx_16 = 0;
    pDstLine = (TYPE24 *)pstApcImgDst->pbmp;
    for(unsigned long y = 0; y < pstApcImgDst->s32Height; y++)
    {
        pSrcLine = (TYPE24 *)pstApcImgSrc->pbmp + pstApcImgSrc->s32Width*(srcy_16>>16);
        srcx_16 = 0;
        for(unsigned long x = 0; x < pstApcImgDst->s32Width; x++)
        {
            pDstLine[x] = pSrcLine[srcx_16>>16];
            srcx_16+=xrIntFloat_16;
        }
        srcy_16+=yrIntFloat_16;
        pDstLine+=pstApcImgDst->s32Width;
    }
    
    return SV_SUCCESS;
}

sint32 apc_Alarm_Out_Enable()
{
    sint32 s32Ret;
    uint8 u8Value = 1;

    s32Ret = BOARD_SetAlarmOut(u8Value);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "BOARD_SetAlarmOut fail![err=%#x]\n", s32Ret);
        return s32Ret;
    }

    return SV_SUCCESS;
}

sint32 apc_Alarm_Out_Reset()
{
    sint32 s32Ret;
    uint8 u8Value = 0;

    s32Ret = BOARD_SetAlarmOut(u8Value);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "BOARD_SetAlarmOut fail![err=%#x]\n", s32Ret);
        return s32Ret;
    }

    return SV_SUCCESS;
}

/* AlarmOut 初始化 */
sint32 apc_Alarm_Out_Init()
{
    sint32 s32Ret;

    s32Ret = apc_Alarm_Out_Reset();
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "pd_Alarm_Out_Reset fail![err=%#x]\n", s32Ret);
        return s32Ret;
    }

    return SV_SUCCESS;
}


sint32 apc_get_readIdx(sint32 s32Chn)
{
    sint32 s32Ret, s32Idx = -1;
    int try_time = 30;
    int i;

    while(try_time--)
    {
        /* P操作进入MediaBuffer临界区 */
        s32Ret = MS_P(s32Chn);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "MS_P failed. [err=%d]\n", s32Ret);
            return s32Idx;
        }

        for(i = 0; i < 3; i++)
        {
            if(MH_IsRead(s32Chn, i))
            {
                s32Idx = i;
                MH_SetRead(s32Chn, i);
                break;
            }
        }

        MS_V(s32Chn);
        if(s32Idx >= 0)
        {
            break;
        }
        else
        {
            sleep_ms(1);
        }
        
    }
    return s32Idx;
}


sint32 apc_release_readIdx(sint32 s32Chn, sint32 s32Idx)
{
    sint32 s32Ret;
    /* P操作进入MediaBuffer临界区 */
    s32Ret = MS_P(s32Chn);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "MS_P failed. [err=%d]\n", s32Ret);
        return s32Idx;
    }

    s32Ret = MH_SetWrite(s32Chn, s32Idx);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "MH_SetWrite failed. [err=%d]\n", s32Ret);
        MS_V(s32Chn);
        return s32Ret;
    }

    MS_V(s32Chn);
    return s32Ret;
}

/* APC 音频报警线程 */
void *apc_audio_Body(void *pvArg)
{
    sint32 s32Ret = 0, i, j;
    APC_INFO_S *pstApcInfo = (APC_INFO_S *)pvArg;
    APC_AUDIO_STATE *pstAudioState = &(pstApcInfo->stSubThreadState.stAudio);
    APC_ALARM_TYPE enAlarm;
    ALARM_EVENT_S stAlarmEvent = {0};
    MSG_PACKET_S stMsgPkt = {0};
    struct timeval tvAlarm;

    while(pstApcInfo->bRunning)
    {
        if (pstAudioState->enAlarm == APC_ALARM_NONE)
        {
            sleep_ms(10);
            continue;
        }

        enAlarm = pstAudioState->enAlarm;
        pstAudioState->enAlarm = APC_ALARM_NONE;    /* 恢复 */

        /* 音频播放事件提交 */
        gettimeofday(&tvAlarm, NULL);
        stAlarmEvent.enAlarmEvent = ALARM_EVENT_APC;
        switch(enAlarm)
        {
            case APC_ALARM_OVERLOAD:
                stAlarmEvent.enAlarmType = ALARM_APC_OVERLOAD;
                break;
            case APC_ALARM_CROWDED:
                stAlarmEvent.enAlarmType = ALARM_APC_CROWDED;
                break;
        }
        stAlarmEvent.s32TimeStamp = (sint32)tvAlarm.tv_sec;
        stAlarmEvent.s32Chn = 0;
        memset(&stMsgPkt, 0, sizeof(stMsgPkt));
        stMsgPkt.stMsg.u16OpCode = OP_EVENT_ALG_ALARM;
        stMsgPkt.pu8Data = (uint8 *)&stAlarmEvent;
        stMsgPkt.u32Size = sizeof(stAlarmEvent);
        print_level(SV_INFO, "crowded event post:%d!\n", stAlarmEvent.enAlarmType);
        s32Ret = Msg_submitEvent(EP_CONTROL, OP_EVENT_ALG_ALARM, &stMsgPkt);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
        }

        /* 播放报警 */
        ALARM_Play_APC_Alarm(enAlarm);
    }
    
    return NULL;
}


/* APC AlarmOut线程 */
void *apc_alarmout_Body(void *pvArg)
{
    sint32 s32Ret = 0, i, j;
    APC_INFO_S *pstApcInfo = (APC_INFO_S *)pvArg;
    APC_ALARMOUT_STATE *pstAlarmOutState = &(pstApcInfo->stSubThreadState.stAlarmOut);
    APC_ALARM_TYPE enAlarm = APC_ALARM_NONE;
    struct timespec tvStart = {0, 0};
    struct timespec tvNow = {0, 0};
    SV_BOOL bTrigger = SV_FALSE;
    sint32 s32Interval = 2*1000;

    while(pstApcInfo->bRunning)
    {
        if (pstAlarmOutState->enAlarm != APC_ALARM_NONE)
        {
            enAlarm = pstAlarmOutState->enAlarm;
            pstAlarmOutState->enAlarm = APC_ALARM_NONE;
            clock_gettime(CLOCK_MONOTONIC, &tvStart); /* 更新触发时间 */
            if (bTrigger != SV_TRUE)
            {
                bTrigger = SV_TRUE;
                apc_Alarm_Out_Enable();
            }
        }

        if (bTrigger == SV_TRUE)
        {
            clock_gettime(CLOCK_MONOTONIC, &tvNow);
            if ((1000*tvNow.tv_sec + tvNow.tv_nsec/1000000) - (1000*tvStart.tv_sec + tvStart.tv_nsec/1000000) > s32Interval)
            {
                apc_Alarm_Out_Reset();
                bTrigger = SV_FALSE;
            }
        }
        sleep_ms(10);
    }
    return NULL;
}


sint32 apc_alarm_event(APC_ALARM_TYPE enType)
{
    switch(enType)
    {
        case APC_ALARM_NONE:
        case APC_ALARM_PERSON_ON:
        case APC_ALARM_PERSON_OFF:
            break;
        case APC_ALARM_OVERLOAD:
            m_stApcInfo.stSubThreadState.stAlarmOut.enAlarm = APC_ALARM_OVERLOAD;
            m_stApcInfo.stSubThreadState.stAudio.enAlarm = APC_ALARM_OVERLOAD;
            break;
        case APC_ALARM_CROWDED:
            m_stApcInfo.stSubThreadState.stAlarmOut.enAlarm = APC_ALARM_CROWDED;
            m_stApcInfo.stSubThreadState.stAudio.enAlarm = APC_ALARM_CROWDED;
            break;
        default:
            print_level(SV_ERROR, "cannot parse event:%d\n", APC_ALARM_CROWDED);
            break;
    }

    return SV_SUCCESS;
}


/* 获取报警事件 */
APC_ALARM_TYPE apc_alarm_getEvent(sint32 s32TotalNum, sint32 s32CrowdedNum)
{
    APC_ALARM_TYPE enType = APC_ALARM_NONE;
    static SV_BOOL bOverloaded = SV_FALSE; /* 是否已经处于超载报警状态 */
    static SV_BOOL bCrowded = SV_FALSE;    /* 是否已经处于拥挤报警状态 */
    
    if (m_stApcInfo.stCfgParam.stAlgCh2.stApcParam.bOverLoadAlarm)
    {
        if (s32TotalNum > m_stApcInfo.stCfgParam.stAlgCh2.stApcParam.s32LoadNum)
        {
            if (bOverloaded == SV_FALSE)
            {
                bOverloaded = SV_TRUE;
                return APC_ALARM_OVERLOAD;
            }
        }
        else
        {
            bOverloaded = SV_FALSE;
        }
    }

    if (m_stApcInfo.stCfgParam.stAlgCh2.stApcParam.bCrowdedAlarm)
    {
        if (s32CrowdedNum > m_stApcInfo.stCfgParam.stAlgCh2.stApcParam.s32CrowdedNum)
        {
            if (bCrowded == SV_FALSE)
            {
                bCrowded = SV_TRUE;
                return APC_ALARM_CROWDED;
            }
        }
        else
        {
            bCrowded = SV_FALSE;
        }
    }

    return APC_ALARM_NONE;
}



static sint32 send_websocket_msg(char *buffer)
{
    MSG_PACKET_S stMsgPkt;
    MSG_WEBSOCKET_INFO_S stMsgWebsocket;
    sint32 s32Ret;
    
    memset(&stMsgPkt, 0, sizeof(stMsgPkt));
    strcpy(stMsgWebsocket.stWsInfo, buffer);
    stMsgPkt.stMsg.u16OpCode = OP_EVEN_WEBSOCKET_INFO;
    stMsgPkt.pu8Data = (uint8*)&stMsgWebsocket;
    stMsgPkt.u32Size = sizeof(stMsgWebsocket);

    // 回传web端显示报警信息
    s32Ret = Msg_submitEvent(EP_HTTPSERVER, OP_EVEN_WEBSOCKET_INFO, &stMsgPkt);
    if (SV_SUCCESS != s32Ret)
    {
       print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
       return SV_FAILURE;
    }

    return SV_SUCCESS;
}

sint32 apc_send_websocket_msg(uint32 u32In, uint32 u32Out, uint32 u32Rect, APC_ALARM_TYPE enType)
{
    sint32 s32Ret;
    char buffer[256];
    static APC_ALARM_TYPE enPreType = APC_ALARM_BUTT;   /* 作为初始化的报警 */
    static uint32 u32PreIn = 0, u32PreOut = 0, u32PreRect = 0;

    if (u32PreIn == u32In && u32PreOut == u32Out && u32PreRect == u32Rect && enPreType == enType)
    {
        return SV_SUCCESS;
    }

    switch(enType)
    {
        case APC_ALARM_OVERLOAD:
            sprintf(buffer, "{\"In\":%d,\"Out\":%d,\"Rect\":%d,\"Warning\":\"%s\"}", 
                               u32In,     u32Out,   u32Rect,     "overload");
            break;
        case APC_ALARM_CROWDED:
            sprintf(buffer, "{\"In\":%d,\"Out\":%d,\"Rect\":%d,\"Warning\":\"%s\"}",
                               u32In,     u32Out,   u32Rect,     "crowded");
            break;
        case APC_ALARM_NONE:
        default:
            sprintf(buffer, "{\"In\":%d,\"Out\":%d,\"Rect\":%d,\"Warning\":\"%s\"}",
                               u32In,     u32Out,   u32Rect,     "no_warning");
            break;
    }

    s32Ret = send_websocket_msg(buffer);
    if (s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "send_websocket_msg failed! [err=%d]\n", s32Ret);
        return s32Ret;
    }

    enPreType = enType;
    u32PreIn = u32In;
    u32PreOut = u32Out;
    u32PreRect = u32Rect;
    
    return SV_SUCCESS;
}

static sint32 string_to_file(char *file, char *string)
{
    int fdout;
    void *dst;
    int size = strlen(string);
    if((fdout = open(file, O_RDWR | O_CREAT | O_CLOEXEC | O_TRUNC | O_FSYNC, (S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH))) < 0)
    {
        printf("can't create %s for writing\n", file);
        return -1;
    }

    if(ftruncate(fdout, size) < 0) /* set output file size */
    {
        printf("ftruncate error");
        return -1;
    }

    if((dst = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fdout, 0)) == MAP_FAILED)
    {
        printf("mmap error for output");
        return -1;
    }

    memcpy(dst, string, size);
    close(fdout);
    //msync(dst, size, MS_SYNC | MS_INVALIDATE);
    munmap(dst, size);
    
    return 0;
}


sint32 apc_DumpInfo(sint32 s32Chn, APC_DUMP_INFO_S *pstApcDumpInfo)
{
    sint32 s32Ret = 0, i;
    uint32 u32ChnNum = 1;
    sint32 fd = -1;
    cJSON *pstJson = NULL;
    cJSON *pstTmp = NULL;
    cJSON *pstList = NULL, *pstItem = NULL;
    cJSON *pstTimeStamp = NULL, *pstPdWorkMode = NULL, *pstGreenRoiNum = NULL, *pstYellowRoiNum = NULL, *pstRedRoiNum = NULL;
    char szTmpInt[1024];
    char szBuf[1024];
    char echo_szBuf[2048];
    CFG_PDS_PARAM *apstPdsParam[ALG_MAX_CHN] = {0};
    static APC_DUMP_INFO_S astDumpInfo[ALG_MAX_CHN] = {0};

    if (s32Chn >= ALG_MAX_CHN)
    {
        return ERR_ILLEGAL_PARAM;
    }

    if (NULL == pstApcDumpInfo)
    {
        return ERR_NULL_PTR;
    }
    
    pstJson = cJSON_CreateObject();
    if (NULL == pstJson)
    {
        print_level(SV_ERROR, "cJSON_CreateObject fail!\n");
        return SV_FAILURE;
    }

    sprintf(szTmpInt, "%lld", (pstApcDumpInfo->s64TimeStamp));
    cJSON_AddItemToObject(pstJson, "TimeStamp", cJSON_CreateRaw(szTmpInt));
    
    sprintf(szTmpInt, "%u", (pstApcDumpInfo->u32InNumber));
    cJSON_AddItemToObject(pstJson, "InNum", cJSON_CreateRaw(szTmpInt));
    
    sprintf(szTmpInt, "%u", (pstApcDumpInfo->u32OutNumber));
    cJSON_AddItemToObject(pstJson, "OutNum", cJSON_CreateRaw(szTmpInt));

    sprintf(szTmpInt, "%u", (pstApcDumpInfo->u32RectNumber));
    cJSON_AddItemToObject(pstJson, "RectNum", cJSON_CreateRaw(szTmpInt));

    memset(szBuf, 0, 1024);
    cJSON_PrintPreallocated(pstJson, szBuf, 1024, 0);
    
    string_to_file("/var/info/apc-tmp", szBuf);
    rename("/var/info/apc-tmp", "/var/info/apc");

exit:
    cJSON_Delete(pstJson);
    return SV_SUCCESS;

}



/* APC 算法驱动线程 */
void * apc_alg_Body(void *pvArg)
{
    sint32 s32Ret = 0, i, j;
    sint32 s32CurChn = 0, s32Idx;
    sint32 s32AlarmMode = -1;
    uint32 u32RectCnt = 0;
    APC_INFO_S *pstApcInfo = (APC_INFO_S *)pvArg;
    sint32 s32TotalNum;
    struct timespec tvNow = {0, 0};
    APC_DUMP_INFO_S stApcDumpInfo;
    DUMP_APC_S      stDumpApc;

    pdsa32::STAlgInfo stApcResult = {0};
    MSG_PACKET_S stMsgPkt = {0};
    MEDIA_GUI_DRAW_S stMediaGuiDraw = {0};
    MEDIA_GUI_PERSON_S stGuiRect = {0};
    MEDIA_GUI_LINE_S stGuiLine = {0};
    MEDIA_GUI_STRING_S stGuiString = {0};
    MEDIA_GUI_APC_COUNT_S stGuiApcCount = {0};
    MEDIA_GUI_APC_ARROW_S stGuiApcArow = {0};
    MEDIA_GUI_NULL_S stGuiNull;
    CHN_ALG_E *apenChnAlg[ALG_MAX_CHN] = {0};
    CFG_PDS_PARAM *apstPdsParam[ALG_MAX_CHN] = {0};
    APC_ALARM_TYPE enAlarmType;

    uint32 u32InNum, u32OutNum;
    sint32 s32Total;
    float xmiddle, ymiddle;
    
    uint16 u16mask;
#if ALG_MUTLIT_BUFFER
    void *apvBuf[4][3] = {NULL};
#else
    void *apvBuf[4] = {NULL};
#endif

    uint32 u32BufLen = APC_IMAGE_SRC_WIDTH*APC_IMAGE_SRC_HEIGHT*3;
    uint32 u32Cnt = 0;
    uint8 u8InValue;

    s32Ret = prctl(PR_SET_NAME, "apc_body");
    if (0 != s32Ret)
    {
        print_level(SV_ERROR, "prctl PR_SET_NAME failed! [err:%s]\n", strerror(errno));
    }

    APC_GUI_IMG_S stAlgImg, stAlgImgSrc;  // 缩放


    apstPdsParam[0] = &pstApcInfo->stCfgParam.stAlgCh2.stPdsParam;
    for (i = 0; i < pstApcInfo->u32ChnNum; i++)
    {
#if ALG_MUTLIT_BUFFER
        for(j = 0; j < 3; j++)
        {
            apvBuf[i][j] = mmap(NULL, u32BufLen, PROT_READ, MAP_SHARED, pstApcInfo->as32MediaBufFd[i][j], 0);
            if (MAP_FAILED == apvBuf[i][j])
            {
                print_level(SV_ERROR, "mmap[%d] failed.\n", i);
                return NULL;
            }
        }
#else
        apvBuf[i] = mmap(NULL, u32BufLen, PROT_READ, MAP_SHARED, pstApcInfo->as32MediaBufFd[i], 0);
        if (MAP_FAILED == apvBuf[i])
        {
            print_level(SV_ERROR, "mmap[%d] failed.\n", i);
            return NULL;
        }
#endif
    }


    // 詹博的RGB_PCZL.rknn的输入
    stAlgImg.s32Width = APC_IMAGE_DST_WIDTH;
    stAlgImg.s32Height = APC_IMAGE_DST_HEIGHT;
    stAlgImg.pbmp = malloc(stAlgImg.s32Width * stAlgImg.s32Height * 3);
    stAlgImgSrc.s32Width = APC_IMAGE_SRC_WIDTH;
    stAlgImgSrc.s32Height = APC_IMAGE_SRC_HEIGHT;


    print_level(SV_INFO, "enter APC detection.\n");
    while (pstApcInfo->bRunning)
    {
        memset(&stApcResult, 0x00, sizeof(pdsa32::STAlgInfo));
        memset(&stMediaGuiDraw, 0x00, sizeof(stMediaGuiDraw));
        memset(&stGuiLine, 0x00, sizeof(stGuiLine));
        memset(&stGuiString, 0x00, sizeof(stGuiString));
        memset(&stGuiRect, 0x00, sizeof(stGuiRect));

        clock_gettime(CLOCK_MONOTONIC, &tvNow);
        stApcDumpInfo.s64TimeStamp = tvNow.tv_sec * 1000 + tvNow.tv_nsec /1000000;
        stDumpApc.s32TimeStamp = tvNow.tv_sec * 1000 + tvNow.tv_nsec /1000000;
        
        u32RectCnt = 0;
        u8InValue = 1;

#if (BOARD == BOARD_ADA900V1)
        s32Ret = BOARD_GetAlarmIn(&u8InValue);
        if (s32Ret != SV_SUCCESS)
        {
            print_level(SV_ERROR, "BOARD_GetAlarmIn failed! [err=%#x]\n", s32Ret);
        }
        else if(!u8InValue) /* AlarmIn 反向 */
        {
            //s32Ret = apc_audio_thread();
            if (s32Ret != SV_SUCCESS)
            {
                print_level(SV_ERROR, "apc_audio_thread failed! [err=%#x]\n", s32Ret);
            }
        }
#endif

        /* 清空原来的画板 */
        u16mask = MEDIA_GUI_GET_MASK(s32CurChn, 0, MEDIA_GUI_OP_CLEAR);
        s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiNull);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
        }

        if (!u8InValue)
        {
            //print_level(SV_INFO, "u8InValue:%d\n", u8InValue);
            stGuiString.color = GUI_COLOR_RED;
            stGuiString.fontsize = 3;
            strcpy(stGuiString.string, "ALARM IN");
            u16mask = MEDIA_GUI_GET_MASK(s32CurChn, 0, MEDIA_GUI_OP_DRAW_STRING);
            s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiString);
            if (SV_SUCCESS != s32Ret)
            {
                print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
            }
        }

        /* 锁住计算资源,保证同一时刻只跑一个算法*/
        s32Ret = ALG_Calculate_Lock();
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "ALG_Calculate_Lock failed. [err=%d]\n", s32Ret);
            sleep_ms(1);
            continue;
        }

#if ALG_MUTLIT_BUFFER
        s32Idx = apc_get_readIdx(s32CurChn);
        if(s32Idx < 0)
        {
            //print_level(SV_ERROR, "pd_get_readIdx failed!\n");
            ALG_Calculate_unLock();
            sleep_ms(1);
            continue;
        }
#else
        /* P操作进入MediaBuffer临界区 */
        s32Ret = MS_P(s32CurChn);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "MS_P failed. [err=%d]\n", s32Ret);
            ALG_Calculate_unLock();
            sleep_ms(1);
            continue;
        }
#endif
        if(stAlgImgSrc.s32Width != stAlgImg.s32Width || stAlgImgSrc.s32Height != stAlgImg.s32Height)
        {
#if ALG_MUTLIT_BUFFER
            stAlgImgSrc.pbmp = (char *)apvBuf[s32CurChn][s32Idx];
#else
            stAlgImgSrc.pbmp = (char *)apvBuf[s32CurChn];
#endif
            s32Ret = apc_Gui_Zoom(&stAlgImg, &stAlgImgSrc);
            if(s32Ret != SV_SUCCESS)
            {
                print_level(SV_ERROR, "pd_Gui_Zoom fail!\n");
                return s32Ret;
            }
        }
        else
        {
#if ALG_MUTLIT_BUFFER
            stAlgImg.pbmp = (char *)apvBuf[s32CurChn][s32Idx];
#else
            stAlgImg.pbmp = (char *)apvBuf[s32CurChn];
#endif
        }

#if ALG_MUTLIT_BUFFER
        s32Ret = pstApcInfo->apcsPdsAlg[s32CurChn]->AlgForward((char *)stAlgImg.pbmp, APC_IMAGE_DST_WIDTH, APC_IMAGE_DST_HEIGHT, pstApcInfo->as32MediaBufFd[s32CurChn][s32Idx]);
#else
        s32Ret = pstApcInfo->apcsPdsAlg[s32CurChn]->AlgForward((char *)stAlgImg.pbmp, APC_IMAGE_DST_WIDTH, APC_IMAGE_DST_HEIGHT, pstApcInfo->as32MediaBufFd[s32CurChn]);
#endif

        //s32Ret = pstApcInfo->apcsPdsAlg[s32CurChn]->AlgForward((char *)stAlgImg.pbmp, APC_IMAGE_DST_WIDTH, APC_IMAGE_DST_WIDTH);
        if (0 != s32Ret)
        {
            print_level(SV_ERROR, "ALGPDS_forward failed. [err=%d]\n", s32Ret);
#if ALG_MUTLIT_BUFFER
            apc_release_readIdx(s32CurChn, s32Idx);
#else
            MS_V(s32CurChn);
#endif

            ALG_Calculate_unLock();
            sleep_ms(10);
            continue;
        }

#if ALG_MUTLIT_BUFFER
        apc_release_readIdx(s32CurChn, s32Idx);
#else
        MS_V(s32CurChn);
#endif
        ALG_Calculate_unLock();

        //print_level(SV_INFO, "readIdx:%d\n", s32Idx);
        //apc_print_delay_time();

        
        pstApcInfo->apcsPdsAlg[s32CurChn]->AlgResult(&stApcResult, pdsa32::E_POST_NONE, pstApcInfo->stTrackParam, s32CurChn);
        if (!u8InValue) /* 关门信号 */
        {
            pstApcInfo->apcsPdsAlg[s32CurChn]->AlgPostProcess(&stApcResult, pdsa32::E_POST_TRACK_COUNT_HEAD_CLOSE_DOOR, pstApcInfo->stPostParam);
        }
        else
        {
            pstApcInfo->apcsPdsAlg[s32CurChn]->AlgPostProcess(&stApcResult, pdsa32::E_POST_TRACK_COUNT_HEAD, pstApcInfo->stPostParam);
        }
        
        if (0 != s32Ret)
        {
            print_level(SV_ERROR, "AlgResult failed. [err=%d]\n", s32Ret);
            sleep_ms(10);
            continue;
        }

        u32InNum = pstApcInfo->stPostParam.stCountHeadParam.u32InNums;
        u32OutNum = pstApcInfo->stPostParam.stCountHeadParam.u32OutNums;
        if(pstApcInfo->stCfgParam.stAlgCh2.stApcParam.enApcDirection == CFG_APC_DIRECTION_DOWM ||
           pstApcInfo->stCfgParam.stAlgCh2.stApcParam.enApcDirection == CFG_APC_DIRECTION_RIGHT)
        {
            u32InNum = pstApcInfo->stPostParam.stCountHeadParam.u32OutNums;
            u32OutNum = pstApcInfo->stPostParam.stCountHeadParam.u32InNums;
        }

        s32TotalNum = u32InNum - u32OutNum;
        stGuiApcCount.incount = u32InNum;
        stGuiApcCount.outcount = u32OutNum;
        stGuiApcCount.rectcount = stApcResult.u32Nums;
        u16mask = MEDIA_GUI_GET_MASK(s32CurChn, 0, MEDIA_GUI_OP_APC_COUNT);
        s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiApcCount);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
        }

        CFG_APC_PARAM *pstApcParam = &pstApcInfo->stCfgParam.stAlgCh2.stApcParam;
        switch(pstApcInfo->stCfgParam.stAlgCh2.stApcParam.enApcDirection)
        {
            case CFG_APC_DIRECTION_DOWM:
                stGuiApcArow.xpos1 = pstApcParam->astApcDetectionPoints[0].dX;
                stGuiApcArow.ypos1 = pstApcParam->stApcDivider.dY;
                stGuiApcArow.xpos2 = pstApcParam->astApcDetectionPoints[1].dX;
                stGuiApcArow.ypos2 = pstApcParam->stApcDivider.dY;
                stGuiApcArow.direction = CFG_APC_DIRECTION_DOWM;
                break;
            case CFG_APC_DIRECTION_UP:
                stGuiApcArow.xpos1 = pstApcParam->astApcDetectionPoints[0].dX;
                stGuiApcArow.ypos1 = pstApcParam->stApcDivider.dY;
                stGuiApcArow.xpos2 = pstApcParam->astApcDetectionPoints[1].dX;
                stGuiApcArow.ypos2 = pstApcParam->stApcDivider.dY;
                stGuiApcArow.direction = CFG_APC_DIRECTION_UP;
                break;
            case CFG_APC_DIRECTION_LEFT:
                stGuiApcArow.xpos1 = pstApcParam->stApcDivider.dX;
                stGuiApcArow.ypos1 = pstApcParam->astApcDetectionPoints[0].dY;
                stGuiApcArow.xpos2 = pstApcParam->stApcDivider.dX;
                stGuiApcArow.ypos2 = pstApcParam->astApcDetectionPoints[1].dY;
                stGuiApcArow.direction = CFG_APC_DIRECTION_LEFT;
                break;
            case CFG_APC_DIRECTION_RIGHT:
                stGuiApcArow.xpos1 = pstApcParam->stApcDivider.dX;
                stGuiApcArow.ypos1 = pstApcParam->astApcDetectionPoints[0].dY;
                stGuiApcArow.xpos2 = pstApcParam->stApcDivider.dX;
                stGuiApcArow.ypos2 = pstApcParam->astApcDetectionPoints[1].dY;
                stGuiApcArow.direction = CFG_APC_DIRECTION_RIGHT;
                break;
        }
        u16mask = MEDIA_GUI_GET_MASK(s32CurChn, 0, MEDIA_GUI_OP_APC_ARROW);
        s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiApcArow);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
        }

#if 0
        {
            static int runtime = 0;
            static int temp_cpu, temp_npu;
            if(runtime % 10 == 0)
            {
                char  string[32] = {0};

                SAFE_System_Recv("cat /sys/class/thermal/thermal_zone0/temp", string, 32);
                temp_cpu = atoi(string) / 1000;
                SAFE_System_Recv("cat /sys/class/thermal/thermal_zone1/temp", string, 32);
                temp_npu = atoi(string) / 1000;
            }
            stGuiString.y = 0.1;
            stGuiString.color = GUI_COLOR_L_GREEN;
            stGuiString.fontsize = 3;
            sprintf(stGuiString.string, "CPU:%d NPU:%d", temp_cpu, temp_npu);
            u16mask = MEDIA_GUI_GET_MASK(s32CurChn, 0, MEDIA_GUI_OP_DRAW_STRING);
            s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiString);
            if (SV_SUCCESS != s32Ret)
            {
                print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
            }
            runtime++;
        }
#endif
/* 将算法结果 stPdResult.u32Nums = 0 作为零发送 */
skip_alg:
        for (i = 0; i < stApcResult.u32Nums; i++)
        {
            
            if(u32RectCnt >= 20)
            {
                break;
            }

            if(pstApcInfo->stCfgParam.stAlgCh2.stApcParam.enApcDirection == CFG_APC_DIRECTION_DOWM ||
               pstApcInfo->stCfgParam.stAlgCh2.stApcParam.enApcDirection == CFG_APC_DIRECTION_UP)
            {
                ymiddle = (stApcResult.stResults[i].fY1 + stApcResult.stResults[i].fY2) / 2;
                if(pstApcInfo->stCfgParam.stAlgCh2.stApcParam.enApcDirection == CFG_APC_DIRECTION_DOWM)
                {
                    stGuiRect.astPersonsRect[u32RectCnt].color = ymiddle > pstApcInfo->stCfgParam.stAlgCh2.stApcParam.stApcDivider.dY ? \
                                                                 GUI_COLOR_ORANGE : GUI_COLOR_GREEN;
                }
                else
                {
                    stGuiRect.astPersonsRect[u32RectCnt].color = ymiddle < pstApcInfo->stCfgParam.stAlgCh2.stApcParam.stApcDivider.dY ? \
                                                                 GUI_COLOR_ORANGE : GUI_COLOR_GREEN;
                }
            }
            else
            {
                xmiddle = (stApcResult.stResults[i].fX1 + stApcResult.stResults[i].fX2) / 2;
                if(pstApcInfo->stCfgParam.stAlgCh2.stApcParam.enApcDirection == CFG_APC_DIRECTION_LEFT)
                {
                    stGuiRect.astPersonsRect[u32RectCnt].color = xmiddle > pstApcInfo->stCfgParam.stAlgCh2.stApcParam.stApcDivider.dX ? \
                                                                 GUI_COLOR_ORANGE : GUI_COLOR_GREEN;
                }
                else
                {
                    stGuiRect.astPersonsRect[u32RectCnt].color = xmiddle < pstApcInfo->stCfgParam.stAlgCh2.stApcParam.stApcDivider.dX ? \
                                                                 GUI_COLOR_ORANGE : GUI_COLOR_GREEN;
                }
            }

            stGuiRect.astPersonsRect[u32RectCnt].stick = 3;
            stGuiRect.classes[u32RectCnt] = (PdsAlgObjectClass)stApcResult.stResults[i].classes;
            stGuiRect.astPersonsRect[u32RectCnt].x1 = stApcResult.stResults[i].fX1;
            stGuiRect.astPersonsRect[u32RectCnt].y1 = stApcResult.stResults[i].fY1;
            stGuiRect.astPersonsRect[u32RectCnt].x2 = stApcResult.stResults[i].fX2;
            stGuiRect.astPersonsRect[u32RectCnt].y2 = stApcResult.stResults[i].fY2;
            stGuiRect.astPersonsRect[u32RectCnt].fontscale = apstPdsParam[s32CurChn]->s32PdOsdFontSize;
            if(apstPdsParam[s32CurChn]->bPdTestMode == SV_TRUE)
            {
                stGuiRect.au32Score[u32RectCnt] = (uint16)(stApcResult.stResults[i].fConfidence*1000);
            }
            else
            {
                stGuiRect.au32Score[u32RectCnt] = 0;
            }

            stGuiRect.as32DistanceX[u32RectCnt] = 0;
            stGuiRect.as32DistanceY[u32RectCnt] = -1;

            // 测试用
            // stGuiRect.as32DistanceX[u32RectCnt] = stApcResult.stResults[i].fConfidence*1000;  // 得分
            // stGuiRect.as32DistanceY[u32RectCnt] = stApcResult.stResults[i].u32ID*1000;  // ID号
            
            u32RectCnt++;
        }
        stGuiRect.u32PersonNum = u32RectCnt;

        //stApcDumpInfo.u32InNumber = u32InNum;
        //stApcDumpInfo.u32OutNumber = u32OutNum;
        //stApcDumpInfo.u32RectNumber = u32RectCnt;
        stDumpApc.s32InNumber = u32InNum;
        stDumpApc.s32OutNumber = u32OutNum;
        stDumpApc.s32RectNumber = u32RectCnt;
        //s32Ret = apc_DumpInfo(0, &stApcDumpInfo);
        if (s32Ret != SV_SUCCESS)
        {
            print_level(SV_ERROR, "apc_DumpInfo failed! [err=%d]\n", s32Ret);
        }

        s32Ret = dump_SetApcInfo(&stDumpApc);
        if (s32Ret != SV_SUCCESS)
        {
            print_level(SV_ERROR, "dump_SetApcInfo failed! [err=%d]\n", s32Ret);
        }

        enAlarmType = apc_alarm_getEvent(s32TotalNum, u32RectCnt);
        s32Ret = apc_alarm_event(enAlarmType);
        if (s32Ret != SV_SUCCESS)
        {
            print_level(SV_ERROR, "apc_alarm_event failed! [err=%d]\n", s32Ret);
        }

        s32Ret = apc_send_websocket_msg(u32InNum, u32OutNum, u32RectCnt, enAlarmType);
        if (s32Ret != SV_SUCCESS)
        {
            print_level(SV_ERROR, "apc_send_websocket_msg failed! [err=%d]\n", s32Ret);
        }

        /* 添加行人矩形绘制操作 */
        u16mask = MEDIA_GUI_GET_MASK(s32CurChn, 0, MEDIA_GUI_OP_PERSON_RECT);
        s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiRect);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
        }

        memset(&stMsgPkt, 0, sizeof(stMsgPkt));
        stMsgPkt.pu8Data = (uint8*)&stMediaGuiDraw;
        stMsgPkt.u32Size = MEDIA_GUI_SIZE(stMediaGuiDraw);
        s32Ret = Msg_submitEvent(EP_CONTROL, OP_EVENT_MEDIA_GUI, &stMsgPkt);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
        }

//        if (stApcResult.u32Nums > 0)
//        {
//            s32Ret = apc_alarm_thread();
//            if (SV_SUCCESS != s32Ret)
//            {
//                print_level(SV_ERROR, "apc_alarm_thread failed. [err=%#x]\n", s32Ret);
//            }
//        }
    }

    print_level(SV_INFO, "exit PD detection.\n");
    for (i = 0; i < pstApcInfo->u32ChnNum; i++)
    {
#if ALG_MUTLIT_BUFFER
        for(j = 0; j < 3; j++)
        {
            munmap(apvBuf[i][j], u32BufLen);
        }
#else
        munmap(apvBuf[i], u32BufLen);
#endif
    }

    return NULL;
}


sint32 APC_Init(APC_CFG_PARAM_S *pstInitParam)
{
    sint32 s32Ret = 0, i, j;
    sint32 s32CenterGaze = 0;
    float afThresholds[4];
    uint32 u32ThrNum = 4;
    SV_BOOL bValidFd = SV_FALSE;
    CHN_ALG_E *apenChnAlg[ALG_MAX_CHN] = {0};
    CFG_PDS_PARAM *apstApcsParam[ALG_MAX_CHN] = {0};
    STCaliParams stCaliParams = {0};
    char *pszModelFile = NULL;
    pdsa32::EAlgType enAlgType = pdsa32::E_PDSALG_TYPE_RGB_HEAD;
    pdsa32::CPdsAlg *apcsPdsAlgRgbHEAD = NULL;

    if (NULL == pstInitParam)
    {
        return ERR_NULL_PTR;
    }

    for (i = 0; i < 4; i++)
    {
#if ALG_MUTLIT_BUFFER
        for(j = 0; j < 3; j++)
        {
            if (pstInitParam->as32MediaBufFd[i][j] > 0)
            {
                bValidFd = SV_TRUE;
            }
        }
#else
        if (pstInitParam->as32MediaBufFd[i] > 0)
        {
            bValidFd = SV_TRUE;
        }
#endif
    }

    if (!bValidFd)
    {
        return ERR_ILLEGAL_PARAM;
    }

    memset(&m_stApcInfo, 0, sizeof(APC_INFO_S));
    s32Ret = pthread_mutex_init(&m_stApcInfo.mutexRunStat, NULL);
    if (0 != s32Ret)
    {
        print_level(SV_ERROR, "pthread_mutex_init failed! [err=%#x]\n", s32Ret);
        return ERR_SYS_NOTREADY;
    }

    s32Ret = MS_Init();
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "MS_Init failed. [err=%#x]\n", s32Ret);
        return ERR_SYS_NOTREADY;
    }
#if ALG_MUTLIT_BUFFER
    s32Ret = MH_Init();
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "MH_Init failed. [err=%#x]\n", s32Ret);
        return ERR_SYS_NOTREADY;
    }

    MH_PrintState(0);
#endif


    apenChnAlg[0] = &pstInitParam->stAlgParam.stAlgCh2.enAlgType;
    apstApcsParam[0] = &pstInitParam->stAlgParam.stAlgCh2.stPdsParam;


    m_stApcInfo.u32ChnNum = pstInitParam->u32ChnNum;
    for (i = 0; i < 4; i++)
    {
#if ALG_MUTLIT_BUFFER
        for (j = 0; j < 3; j++)
        {
            m_stApcInfo.as32MediaBufFd[i][j] = pstInitParam->as32MediaBufFd[i][j];
        }
#else
        m_stApcInfo.as32MediaBufFd[i] = pstInitParam->as32MediaBufFd[i];
#endif
    }

    pdsa32::STAlgParam stAlgParam(0.1, 0.45);              // 设置算法参数
    apcsPdsAlgRgbHEAD = new pdsa32::CPdsAlg(pdsa32::E_PDSALG_TYPE_RGB_HEAD);
    if (NULL == apcsPdsAlgRgbHEAD)
    {
        print_level(SV_ERROR, "new pdsa32::CPdsAlg failed. [err=%d]\n", s32Ret);
        return SV_FAILURE;
    }

    s32Ret = apcsPdsAlgRgbHEAD->AlgInit(APC_MODEL_RGB_HEAD, stAlgParam);
    if (0 != s32Ret)
    {
        print_level(SV_ERROR, "csPdsAlg.AlgInit failed. [err=%d]\n", s32Ret);
        return SV_FAILURE;
    }


    m_stApcInfo.apcsPdsAlg[0] = apcsPdsAlgRgbHEAD;
    m_stApcInfo.stCfgParam = pstInitParam->stAlgParam;
    m_stApcInfo.stPostParam.stCountHeadParam.fRectScope1.fX = pstInitParam->stAlgParam.stAlgCh2.stApcParam.astApcDetectionPoints[0].dX;
    m_stApcInfo.stPostParam.stCountHeadParam.fRectScope1.fY = pstInitParam->stAlgParam.stAlgCh2.stApcParam.astApcDetectionPoints[0].dY;
    m_stApcInfo.stPostParam.stCountHeadParam.fRectScope2.fX = pstInitParam->stAlgParam.stAlgCh2.stApcParam.astApcDetectionPoints[1].dX;
    m_stApcInfo.stPostParam.stCountHeadParam.fRectScope2.fY = pstInitParam->stAlgParam.stAlgCh2.stApcParam.astApcDetectionPoints[1].dY;
    switch(pstInitParam->stAlgParam.stAlgCh2.stApcParam.enApcDirection)
    {
        case CFG_APC_DIRECTION_LEFT:
        case CFG_APC_DIRECTION_RIGHT:
            m_stApcInfo.stPostParam.stCountHeadParam.eMode = pdsa32::E_COUNT_HEAD_X;
            m_stApcInfo.stPostParam.stCountHeadParam.fLinePosition = pstInitParam->stAlgParam.stAlgCh2.stApcParam.stApcDivider.dY;
            break;

        case CFG_APC_DIRECTION_DOWM:
        case CFG_APC_DIRECTION_UP:
        default:
            m_stApcInfo.stPostParam.stCountHeadParam.eMode = pdsa32::E_COUNT_HEAD_Y;
            m_stApcInfo.stPostParam.stCountHeadParam.fLinePosition = pstInitParam->stAlgParam.stAlgCh2.stApcParam.stApcDivider.dY;
            break;
    }

    apc_Alarm_Out_Init();

    return SV_SUCCESS;
}

sint32 APC_Fini()
{
    sint32 i;

    for (i = 0; i < ALG_MAX_CHN; i++)
    {
        if (NULL != m_stApcInfo.apcsPdsAlg[i])
        {
            delete m_stApcInfo.apcsPdsAlg[i];
        }
    }
    pthread_mutex_destroy(&m_stApcInfo.mutexRunStat);

    return SV_SUCCESS;
}

sint32 APC_Start()
{
    sint32 s32Ret = 0;
    pthread_t thread_alg, thread_audio, thread_alarmout;

    m_stApcInfo.bRunning = SV_TRUE;

    s32Ret = pthread_create(&thread_alg, NULL, apc_alg_Body, &m_stApcInfo);
    if (0 != s32Ret)
    {
        print_level(SV_ERROR, "pthread_create failed. [err: %s]\n", strerror(errno));
        if (SV_SUCCESS != s32Ret)
        {
            return s32Ret;
        }
    }

    s32Ret = pthread_create(&thread_audio, NULL, apc_audio_Body, &m_stApcInfo);
    if (0 != s32Ret)
    {
        print_level(SV_ERROR, "pthread_create failed. [err: %s]\n", strerror(errno));
        if (SV_SUCCESS != s32Ret)
        {
            return s32Ret;
        }
    }

    s32Ret = pthread_create(&thread_alarmout, NULL, apc_alarmout_Body, &m_stApcInfo);
    if (0 != s32Ret)
    {
        print_level(SV_ERROR, "pthread_create failed. [err: %s]\n", strerror(errno));
        if (SV_SUCCESS != s32Ret)
        {
            return s32Ret;
        }
    }


    m_stApcInfo.u32TidAlg       = thread_alg;
    m_stApcInfo.u32TidAudio     = thread_audio;
    m_stApcInfo.u32TidAlarmOut  = thread_alarmout;
    return SV_SUCCESS;
}


sint32 APC_Stop()
{
    sint32 s32Ret = 0;
    pthread_t thread = m_stApcInfo.u32TidAlg;
    void *pvRetval = NULL;

    m_stApcInfo.bRunning = SV_FALSE;
    s32Ret = pthread_join(thread, &pvRetval);
    if (0 != s32Ret)
    {
        print_level(SV_ERROR, "pthread_join failed. [err: %s]\n", strerror(errno));
        return SV_FAILURE;
    }

    return SV_SUCCESS;
}

sint32 APC_ConfigSet(CFG_ALG_PARAM *pstCfgParam)
{
    if (NULL == pstCfgParam)
    {
        return ERR_NULL_PTR;
    }

    pthread_mutex_lock(&m_stApcInfo.mutexRunStat);
    m_stApcInfo.stCfgParam = *pstCfgParam;

    m_stApcInfo.stPostParam.stCountHeadParam.fRectScope1.fX = pstCfgParam->stAlgCh2.stApcParam.astApcDetectionPoints[0].dX;
    m_stApcInfo.stPostParam.stCountHeadParam.fRectScope1.fY = pstCfgParam->stAlgCh2.stApcParam.astApcDetectionPoints[0].dY;
    m_stApcInfo.stPostParam.stCountHeadParam.fRectScope2.fX = pstCfgParam->stAlgCh2.stApcParam.astApcDetectionPoints[1].dX;
    m_stApcInfo.stPostParam.stCountHeadParam.fRectScope2.fY = pstCfgParam->stAlgCh2.stApcParam.astApcDetectionPoints[1].dY;

    switch(pstCfgParam->stAlgCh2.stApcParam.enApcDirection)
    {
        case CFG_APC_DIRECTION_LEFT:
        case CFG_APC_DIRECTION_RIGHT:
            m_stApcInfo.stPostParam.stCountHeadParam.eMode = pdsa32::E_COUNT_HEAD_X;
            m_stApcInfo.stPostParam.stCountHeadParam.fLinePosition = pstCfgParam->stAlgCh2.stApcParam.stApcDivider.dY;
            break;

        case CFG_APC_DIRECTION_DOWM:
        case CFG_APC_DIRECTION_UP:
        default:
            m_stApcInfo.stPostParam.stCountHeadParam.eMode = pdsa32::E_COUNT_HEAD_Y;
            m_stApcInfo.stPostParam.stCountHeadParam.fLinePosition = pstCfgParam->stAlgCh2.stApcParam.stApcDivider.dY;
            break;
    }
    m_stApcInfo.stPostParam.stCountHeadParam.bClean = SV_TRUE;
    pthread_mutex_unlock(&m_stApcInfo.mutexRunStat);
    
    return SV_SUCCESS;
}

