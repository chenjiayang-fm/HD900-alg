/******************************************************************************
Copyright (C) 2021-2023 广州敏视数码科技有限公司版权所有.

文件名：targettrack.cpp

日期: 2022-2-03

文件功能描述: 定义目标跟踪算法功能接口

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
#include <sys/mman.h>
#include <sys/prctl.h>
#include <pthread.h>
#include <ctype.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include "print.h"
#include "../../../include/common.h"
#include "safefunc.h"
#include "op.h"
#include "msg.h"
#include "alglib.h"
#include "track.h"

#include "config.h"
#include "media.h"
#include "media_sem.h"
#include "alg.h"
#include "cJSON.h"
#include "board.h"

#define TRACK_IMAGE_WIDTH      1280                      /* 算法图像帧宽度 */
#define TRACK_IMAGE_HEIGHT     720                       /* 算法图像帧高度 */

#define TRACK_PTZ_MODEL_PATH   "/root/model/track.rknn" /* 跟踪算法模型路径 */

/* 模块控制信息 */
typedef struct tagPdInfo_S
{
    uint32          u32ChnNum;              /* 跟踪算法通道数目 */
    sint32          s32MediaBufFd;          /* 媒体通道Media Buffer的文件描述符 */
    sint32          s32MeidaBufChn;         /* 媒体通道Media Buffer的通道 */
    CFG_ALG_PARAM   stCfgParam;             /* 算法配置参数 */
    float           fConfidenceThr;         /* 算法得分阈值 */
    float           fNmsThreshold;          /* 算法NMS阈值 */
    float           fTrackThreshold;        /* 跟踪模块的阈值 */
    float           fTrackMaxAge;           /* 跟踪模块的参数 */
    uint32          u32TidAlg;              /* 算法线程ID */
    SV_BOOL         bRunning;               /* 线程是否正在运行 */
    pthread_mutex_t mutexRunStat;           /* 算法运行状态互斥锁 */
    SV_BOOL         bTrackConf;             /* 是否初始化跟踪目标 */
    SV_BOOL         bIsFinishConf;          /* 是否完成 */
    PTZ_POS_S       stTrackPos;             /* 跟踪对象坐标 */
} TARGET_TRACK_INFO_S;

TARGET_TRACK_INFO_S g_stTargetTrackInfo = {0};      /* 模块控制信息 */
extern int ipsys_log_level;

/******************************************************************************
 * 函数功能: 获取当前时刻（微秒）
 * 输入参数: 无
 * 输出参数: 无
 * 返回值  : 微秒
 * 注意    : 无
 *****************************************************************************/
static long long int microtime(void)
{
  struct timeval time;
  gettimeofday(&time, NULL); //This actually returns a struct that has microsecond precision.
  long long int microsec = ((long long)time.tv_sec * 1000000) + time.tv_usec;
  return microsec;
}

/******************************************************************************
 * 函数功能: 对跟踪算法识别的矩形框进行滤波防抖
 * 输入参数: pstNewResult --- 最新帧识别的结果
             u32Chn --- 通道号
             fThresholdX --- X轴方向跨度阈值系数
             fThresholdY --- Y轴方向跨度阈值系数
 * 输出参数: 无
 * 返回值  : 无
 * 注意    : 该函数是不可重入函数，只允许一个线程调用
 *****************************************************************************/
void target_track_RectAntiShakeFilter(TARGET_TRACK_RECT_S *pstNewResult, uint32 u32Chn, float fThresholdX, float fThresholdY)
{
    sint32 i = 0, j = 0, k = 0;
    SFCP_BOX1_T stNewRect;
    SFCP_BOX1_T stOldRect;
    static TARGET_TRACK_RECT_S s_stOldResult[4];
    float fThrDistWidth = 0; 
    float fThrDistHeight = 0; 
    float fMoveDist = 0;

    if (NULL == pstNewResult || u32Chn >= 4)
    {
        return;
    }

    for (i = 0; i < pstNewResult->s32Num; i++)
    {
        stNewRect.x1 = pstNewResult->stRect[i].fX1 * TRACK_IMAGE_WIDTH;
        stNewRect.y1 = pstNewResult->stRect[i].fY1 * TRACK_IMAGE_HEIGHT;
        stNewRect.x2 = pstNewResult->stRect[i].fX2 * TRACK_IMAGE_WIDTH;
        stNewRect.y2 = pstNewResult->stRect[i].fY2 * TRACK_IMAGE_HEIGHT;
        for (j = 0; j < s_stOldResult[u32Chn].s32Num; j++)
        {
            stOldRect.x1 = s_stOldResult[u32Chn].stRect[j].fX1 * TRACK_IMAGE_WIDTH;
            stOldRect.y1 = s_stOldResult[u32Chn].stRect[j].fY1 * TRACK_IMAGE_HEIGHT;
            stOldRect.x2 = s_stOldResult[u32Chn].stRect[j].fX2 * TRACK_IMAGE_WIDTH;
            stOldRect.y2 = s_stOldResult[u32Chn].stRect[j].fY2 * TRACK_IMAGE_HEIGHT;
            fThrDistWidth = fThresholdX * (stNewRect.x2 - stNewRect.x1);
            fThrDistHeight = fThresholdY * (stNewRect.y2 - stNewRect.y1);
            fMoveDist = sqrtf(powf((stOldRect.x1 - stNewRect.x1),2) + powf((stOldRect.y1 - stNewRect.y1),2));
            if (fMoveDist < fThrDistWidth)
            {
                fMoveDist = sqrtf(powf((stOldRect.x2 - stNewRect.x2),2) + powf((stOldRect.y2 - stNewRect.y2),2));
                if (fMoveDist < fThrDistHeight)
                {
                    pstNewResult->stRect[i].fX1 = s_stOldResult[u32Chn].stRect[j].fX1;
                    pstNewResult->stRect[i].fY1 = s_stOldResult[u32Chn].stRect[j].fY1;
                    pstNewResult->stRect[i].fX2 = s_stOldResult[u32Chn].stRect[j].fX2;
                    pstNewResult->stRect[i].fY2 = s_stOldResult[u32Chn].stRect[j].fY2;

                    /* 排队该旧帧矩形框减少下个新矩形的匹配遍历数 */
                    for (k = j; k < s_stOldResult[u32Chn].s32Num - 1; k++)
                    {
                        s_stOldResult[u32Chn].stRect[k] = s_stOldResult[u32Chn].stRect[k+1];
                    }
                    s_stOldResult[u32Chn].s32Num--;
                    break;
                }
            }
        }
    }

    s_stOldResult[u32Chn] = *pstNewResult;
}

/******************************************************************************
 * 函数功能: 从输入的json格式字符串中解析出指定项对应的值，保存为字符串
 * 输入参数: ps8Input --- json格式字符串
             pBuf --- 要解析的名称
 * 输出参数: ps8Output --- 解析结果
 * 返回值  : 无
 * 注意    : 无
 *****************************************************************************/
sint32 target_track_param2string(sint8 *ps8Input, sint8 *ps8Output, const sint8 *pBuf)
{
    cJSON *pstJson, *pstParam;
    
    pstJson = cJSON_Parse(ps8Input); // 解析参数
    if (!pstJson) 
    {
        print_level(SV_ERROR, "%s\n", "Error before target_track_param2string cJSON_Parse");
        goto err;
    }   
    else    
    {   
        pstParam = cJSON_GetObjectItem(pstJson, pBuf);
        if(pstParam == NULL)
        {
            print_level(SV_ERROR, "%s\n", "pstParam is NULL!");
            goto err;
        }
        
        if(strlen(pstParam->valuestring) < 128)
        {
            memset(ps8Output, 0, 128);
            memcpy(ps8Output, pstParam->valuestring, strlen(pstParam->valuestring));
        }
        else
        {
            print_level(SV_ERROR, "%s\n", "string length is too long!");
            goto err;
        }
    }

    cJSON_Delete(pstJson);     
    return 0;
    
err:
    cJSON_Delete(pstJson);
    return -1;
}

/******************************************************************************
 * 函数功能: 从输入的json格式字符串中解析出指定项对应的值，保存为数字
 * 输入参数: ps8Input --- json格式字符串
             pBuf --- 要解析的名称
 * 输出参数: pdNum --- 解析结果
 * 返回值  : 无
 * 注意    : 无
 *****************************************************************************/
sint32 target_track_param2double(sint8 *ps8Input, double *pdNum, const sint8 *pBuf)
{
    cJSON *pstJson, *pstParam;
    
    pstJson = cJSON_Parse(ps8Input); // 解析参数
    if (!pstJson) 
    {
        print_level(SV_ERROR, "%s\n", "Error before param2num:cJSON_Parse");//PRINT_ERROR("Error before: [%s]\n",cJSON_GetErrorPtr());
        goto err;
    }   
    else    
    {   
        pstParam = cJSON_GetObjectItem(pstJson, pBuf);
        if(pstParam == NULL)
        {
            print_level(SV_ERROR, "%s\n", "pstParam is NULL!");//PRINT_ERROR("pstParam is NULL!\n");
            goto err;
        }

        *pdNum = pstParam->valuedouble;
    }

    cJSON_Delete(pstJson);
    return 0;
    
err:
    cJSON_Delete(pstJson);
    return -1;
}

/* 跟踪算法驱动线程 */
void * track_alg_Body(void *pvArg)
{
    sint32 s32Ret = 0, i;
    sint32 s32CurChn = 0;
    uint32 u32RectCnt = 0;
    SV_BOOL bLoss = SV_FALSE;
    SV_BOOL bClear = SV_FALSE;
    CHN_ALG_E *apenChnAlg[ALG_MAX_CHN] = {0};
    TARGET_TRACK_INFO_S *pstTargetTrackInfo = (TARGET_TRACK_INFO_S *)pvArg;
    
    TARGET_TRACK_RECT_S stResult = {0};
    MEDIA_GUI_DRAW_S stMediaGuiDraw = {0};
    MEDIA_GUI_RECT_S stGuiRect = {0};
    MEDIA_GUI_NULL_S stGuiNull;
    MSG_PACKET_S stMsgPkt = {0};
    PTZ_POS_S   stPtzPos = {0};

    uint16 u16mask;
    void *apvBuf = {NULL};
    uint32 u32BufLen = TRACK_IMAGE_WIDTH * TRACK_IMAGE_HEIGHT * 3;

    sint8 s8Cmd[512] = {0}; 
    sint8 s8Buf[512] = {0}; 

    SFCP cSFCP = SFCP();
    SFCP_BOX stFirstBox;
    SFCP_BOX stNextBox;    
    SFCP_BOX stLastBox;
    float fScore;
    sint32 s32PointLeftX = 0, s32PointLeftY = 0, s32PointRightX = 0, s32PointRightY = 0;
    sint32 s32ImgWidth  = TRACK_IMAGE_WIDTH;
    sint32 s32ImgHeight = TRACK_IMAGE_HEIGHT;   
    long long s64CurTime = 0;
    long long s64LastTime = 0;

    s32Ret = prctl(PR_SET_NAME, "target_track_body");
    if (0 != s32Ret)
    {
        print_level(SV_ERROR, "prctl PR_SET_NAME failed! [err:%s]\n", strerror(errno));
    }

    apvBuf = mmap(NULL, u32BufLen, PROT_READ, MAP_SHARED, pstTargetTrackInfo->s32MediaBufFd, 0);
    if (MAP_FAILED == apvBuf)
    {
        print_level(SV_ERROR, "mmap[%d] failed.\n", i);
        return ;
    }

    print_level(SV_INFO, "enter target track.\n");
    
    s32Ret = cSFCP.init(TRACK_PTZ_MODEL_PATH, false);
    if (s32Ret < 0)
    {
        print_level(SV_ERROR, "SFCPInit failed. [err=%#x]\n", s32Ret);
        return SV_FAILURE;
    }
    else
    {
        print_level(SV_INFO, "SFCPInit success. [ret=%#x]\n", s32Ret);
    }
    s32CurChn = pstTargetTrackInfo->s32MeidaBufChn;
    
    while (pstTargetTrackInfo->bRunning)
    {
  
        memset(&stResult, 0x00, sizeof(TARGET_TRACK_RECT_S));
        memset(&stMediaGuiDraw, 0x00, sizeof(stMediaGuiDraw));
		memset(&stGuiRect, 0x00, sizeof(stGuiRect));
		u32RectCnt = 0;
		
        /* 清空原来的画板 */
        u16mask = MEDIA_GUI_GET_MASK(s32CurChn, 0, MEDIA_GUI_OP_CLEAR);
        s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiNull);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
        }    

        /* 锁住计算资源,保证同一时刻只跑一个算法 */
        s32Ret = ALG_Calculate_Lock();
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "ALG_Calculate_Lock failed. [err=%d]\n", s32Ret);
            sleep_ms(5);
            continue;
        }
        s64LastTime = microtime();

        /* P操作进入MediaBuffer临界区 */
        s32Ret = MS_P(s32CurChn);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "MS_P failed. [err=%d]\n", s32Ret);
            ALG_Calculate_unLock();
            sleep_ms(5);
            continue;
        }

        if (!pstTargetTrackInfo->stTrackPos.bVaild)
        {
            pstTargetTrackInfo->bTrackConf = SV_FALSE;
            pstTargetTrackInfo->bIsFinishConf = SV_FALSE;
            pstTargetTrackInfo->stTrackPos.bVaild = SV_TRUE;
            bLoss = SV_FALSE;
            bClear = SV_FALSE;
            memset(&stMsgPkt, 0, sizeof(stMsgPkt));
            stMsgPkt.pu8Data = (uint8*)&stMediaGuiDraw;
            stMsgPkt.u32Size = sizeof(stMediaGuiDraw);
            s32Ret = Msg_submitEvent(EP_CONTROL, OP_EVENT_MEDIA_GUI, &stMsgPkt);
            if (SV_SUCCESS != s32Ret)
            {
                print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
            }            
        }

        if(SV_TRUE == pstTargetTrackInfo->bTrackConf)
        {            
            s32PointLeftX = (int)(pstTargetTrackInfo->stTrackPos.dLeftX * s32ImgWidth);
            s32PointLeftY = (int)(pstTargetTrackInfo->stTrackPos.dLeftY * s32ImgHeight);
            s32PointRightX = (int)(pstTargetTrackInfo->stTrackPos.dRightX * s32ImgWidth);
            s32PointRightY = (int)(pstTargetTrackInfo->stTrackPos.dRightY * s32ImgHeight);

            stFirstBox.cx = (s32PointLeftX + s32PointRightX) >> 1;
            stFirstBox.cy = (s32PointLeftY + s32PointRightY) >> 1;
            stFirstBox.w = abs(s32PointRightX - s32PointLeftX);
            stFirstBox.h = abs(s32PointRightY - s32PointLeftY);
            print_level(SV_INFO, "first box X:%d Y:%d W:%d H:%d\n",stFirstBox.cx, stFirstBox.cy, stFirstBox.w, stFirstBox.h);
            cSFCP.prepare((char *)apvBuf, pstTargetTrackInfo->s32MediaBufFd, s32ImgWidth, s32ImgHeight, stFirstBox);  /* 准备要跟踪的对象 */
            pstTargetTrackInfo->bTrackConf = SV_FALSE;
            pstTargetTrackInfo->bIsFinishConf = SV_TRUE;
            bLoss = SV_FALSE;
        }

        if (SV_TRUE == pstTargetTrackInfo->bIsFinishConf)
        {
            memcpy(&stLastBox, &stNextBox, sizeof(SFCP_BOX));
            cSFCP.update((char *)apvBuf, pstTargetTrackInfo->s32MediaBufFd, stNextBox, fScore, 0.75);
            if (fScore < 0.3)
            {
                bLoss = SV_TRUE;
                bClear = SV_FALSE;
                memcpy(&stNextBox, &stLastBox, sizeof(SFCP_BOX));
                fScore = 0.3;
            }

            if (bLoss)
            {
                if (fScore > 0.6)
                {
                    bLoss = SV_FALSE;
                }
                else
                {
                   memcpy(&stNextBox, &stLastBox, sizeof(SFCP_BOX));
                   fScore = 0.3;
                }
            }

            if (bLoss)
            {
                if (!bClear)
                {
                    bClear = SV_TRUE;
                    memset(&stMsgPkt, 0, sizeof(stMsgPkt));
                    stMsgPkt.pu8Data = (uint8*)&stMediaGuiDraw;
                    stMsgPkt.u32Size = sizeof(stMediaGuiDraw);
                    s32Ret = Msg_submitEvent(EP_CONTROL, OP_EVENT_MEDIA_GUI, &stMsgPkt);
                    if (SV_SUCCESS != s32Ret)
                    {
                        print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
                    }
                }
                
                MS_V(s32CurChn);
                ALG_Calculate_unLock();
                sleep_ms(5);
                stResult.s32Num = 0;
                continue;
            }

            //print_level(SV_INFO, "next box X:%d Y:%d W:%d H:%d\n",stNextBox.cx, stNextBox.cy, stNextBox.w, stNextBox.h);
            stResult.s32Num = 1;
            stResult.stRect[0].fX1 = ((float)(stNextBox.cx - ((stNextBox.w)>>1)))/s32ImgWidth;
            stResult.stRect[0].fX2 = ((float)(stNextBox.cx + ((stNextBox.w)>>1)))/s32ImgWidth;
            stResult.stRect[0].fY1 = ((float)(stNextBox.cy - ((stNextBox.h)>>1)))/s32ImgHeight;
            stResult.stRect[0].fY2 = ((float)(stNextBox.cy + ((stNextBox.h)>>1)))/s32ImgHeight;
            //print_level(SV_INFO, "s32ImgWidth %d s32ImgHeight %d X1:%f X2:%f Y1:%f Y2:%f next box X:%d Y:%d W:%d H:%d %f\n", s32ImgWidth, s32ImgHeight, stResult.stRect[0].fX1, stResult.stRect[0].fX2, stResult.stRect[0].fY1, stResult.stRect[0].fY2, stNextBox.cx, stNextBox.cy, stNextBox.w, stNextBox.h,fScore);
            stPtzPos.dLeftX = stResult.stRect[0].fX1;
            stPtzPos.dLeftY = stResult.stRect[0].fY1;
            stPtzPos.dRightX = stResult.stRect[0].fX2;
            stPtzPos.dRightY = stResult.stRect[0].fY2;
            stPtzPos.fScore = fScore;
            stMsgPkt.pu8Data = (uint8 *)&stPtzPos;
            stMsgPkt.u32Size = sizeof(PTZ_POS_S);
            stMsgPkt.stMsg.u16OpCode = OP_EVENT_PTZ_UPDATE;
            s32Ret = Msg_submitEvent(EP_CONTROL, OP_EVENT_PTZ_UPDATE, &stMsgPkt);
            if (SV_SUCCESS != s32Ret)
            {
                print_level(SV_ERROR, "OP_EVENT_ALG_SPLIT_UPDATE failed. [err:%#x]\n", s32Ret);
                return s32Ret;
            }
        }
        else
        {
            MS_V(s32CurChn);
            ALG_Calculate_unLock();
            sleep_ms(5);
            stResult.s32Num = 0;
            continue;
        }
        s64CurTime = microtime();
        //print_level(SV_DEBUG, "alg frame: %lld %lld %lld\n", s64CurTime, s64LastTime, (s64CurTime - s64LastTime));

        MS_V(s32CurChn);
        ALG_Calculate_unLock();

        target_track_RectAntiShakeFilter(&stResult, s32CurChn, 0.1, 0.2);

        for (i = 0; i < stResult.s32Num; i++)
        {       
            stGuiRect.color = GUI_COLOR_L_YELLOW;
            stGuiRect.stick = 3;
            stGuiRect.x1 = stResult.stRect[i].fX1;
            stGuiRect.y1 = stResult.stRect[i].fY1;
            stGuiRect.x2 = stResult.stRect[i].fX2;
            stGuiRect.y2 = stResult.stRect[i].fY2;
            stGuiRect.fontscale = 2;
            
            u32RectCnt++;
        }

        /* 添加矩形绘制操作 */
        u16mask = MEDIA_GUI_GET_MASK(s32CurChn, 0, MEDIA_GUI_OP_DRAW_RECT);
        s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiRect);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
        }

        memset(&stMsgPkt, 0, sizeof(stMsgPkt));
        stMsgPkt.pu8Data = (uint8*)&stMediaGuiDraw;
        stMsgPkt.u32Size = sizeof(stMediaGuiDraw);
        s32Ret = Msg_submitEvent(EP_CONTROL, OP_EVENT_MEDIA_GUI, &stMsgPkt);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
        }
    }

    munmap(apvBuf, u32BufLen);

    return NULL;
}

sint32 TARGET_TRACK_Init(TARGET_TRACK_CFG_PARAM_S *pstInitParam)
{
    sint32 s32Ret = 0;
    sint32 i = 0;

    if (NULL == pstInitParam)
    {
        return ERR_NULL_PTR;
    }

    if (pstInitParam->s32MediaBufFd < 0)
    {
        return ERR_ILLEGAL_PARAM;
    }

    memset(&g_stTargetTrackInfo, 0, sizeof(TARGET_TRACK_INFO_S));
    s32Ret = pthread_mutex_init(&g_stTargetTrackInfo.mutexRunStat, NULL);
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
    MS_V(pstInitParam->s32MediaBufChn);

    g_stTargetTrackInfo.u32ChnNum = pstInitParam->u32ChnNum;
    g_stTargetTrackInfo.s32MediaBufFd = pstInitParam->s32MediaBufFd;
    g_stTargetTrackInfo.s32MeidaBufChn = pstInitParam->s32MediaBufChn;
    
    g_stTargetTrackInfo.bTrackConf = SV_FALSE;
    g_stTargetTrackInfo.bIsFinishConf = SV_FALSE;
    
    return SV_SUCCESS;
}

sint32 TARGET_TRACK_Fini()
{
    pthread_mutex_destroy(&g_stTargetTrackInfo.mutexRunStat);

    return SV_SUCCESS;
}

sint32 TARGET_TRACK_Start()
{
    sint32 s32Ret = 0;
    pthread_t thread;

    g_stTargetTrackInfo.bRunning = SV_TRUE;
    s32Ret = pthread_create(&thread, NULL, track_alg_Body, &g_stTargetTrackInfo);
    if (0 != s32Ret)
    {
        print_level(SV_ERROR, "pthread_create failed. [err: %s]\n", strerror(errno));
        if (EAGAIN == s32Ret)
        {
            return ERR_SYS_NOTREADY;
        }
        else
        {
            return SV_FAILURE;
        }
    }

    g_stTargetTrackInfo.u32TidAlg = thread;
    return SV_SUCCESS;
}

sint32 TARGET_TRACK_Stop()
{
    sint32 s32Ret = 0;
    pthread_t thread = g_stTargetTrackInfo.u32TidAlg;
    void *pvRetval = NULL;

    g_stTargetTrackInfo.bRunning = SV_FALSE;
    s32Ret = pthread_join(thread, &pvRetval);
    if (0 != s32Ret)
    {
        print_level(SV_ERROR, "pthread_join failed. [err: %s]\n", strerror(errno));
        return SV_FAILURE;
    }

    return SV_SUCCESS;
}

/******************************************************************************
 * 函数功能: 手动更新目标位置
 * 输入参数: stTrackPos -- 初始跟踪目标位置
 * 输出参数: 无
 * 返回值  : SV_SUCCESS - 成功
             SV_FAILURE - 其它错误
 * 注意    : 
 *****************************************************************************/
sint32 TrackPosUpdate(PTZ_POS_S stTrackPos)
{
    //print_level(SV_WARN, "TrackPosUpdate %lf %lf %lf %lf\n", stTrackPos.dLeftX, stTrackPos.dLeftY, stTrackPos.dRightX, stTrackPos.dRightY);
    g_stTargetTrackInfo.stTrackPos.dLeftX = stTrackPos.dLeftX;
    g_stTargetTrackInfo.stTrackPos.dLeftY = stTrackPos.dLeftY;
    g_stTargetTrackInfo.stTrackPos.dRightX = stTrackPos.dRightX;
    g_stTargetTrackInfo.stTrackPos.dRightY = stTrackPos.dRightY;
    g_stTargetTrackInfo.stTrackPos.bVaild = stTrackPos.bVaild;
    g_stTargetTrackInfo.bTrackConf = SV_TRUE;
    return SV_SUCCESS;
}

