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
#include "media_sem.h"
#include "media_shm.h"
#include "alg.h"
#include "cJSON.h"
#include "utils.h"
#include "board.h"
#include "utils.h"
#include "msg.h"
#include "zoom.h"

/* TODO:2023-08-02 先借用行人检测算法 待后续变焦算开发后更改*/
#define ZOOM_ALG_TEST   1

#if ZOOM_ALG_TEST
#include "pd.h"
#include "pds_alg.h"
#endif

#define ZOOM_IMAGE_SRC_WIDTH     608                     /* 算法图像帧源宽度 */
#define ZOOM_IMAGE_SRC_HEIGHT    352                     /* 算法图像帧源高度 */
#define ZOOM_IMAGE_DST_WIDTH     608                     /* 算法图像帧目的宽度 */
#define ZOOM_IMAGE_DST_HEIGHT    352                     /* 算法图像帧目的高度 */

#define ZOOM_ALG_CHN             0                       /* 变焦算法通道 */

/* 模块控制信息 */
typedef struct
{
    SV_BOOL         bKeyAuth;               /* 密钥是否有效 */
    SV_BOOL         bRunning;               /* 线程运行标志 */
    SV_BOOL         bEnable;                /* 算法是否开启 */
    sint32          s32MediaBufFd;          /* 媒体通道Media Buffer的文件描述符 */
    CFG_ZOOM_PARAM  stZoomParam;            /* 变焦算法配置参数 */
#if ZOOM_ALG_TEST    
    pdsa32::CPdsAlg *pdsAlg;                /* PD算法对象指针 */
#endif
    pthread_mutex_t mutexlock;              /* 算法运行状态互斥锁 */
} ZOOM_INFO_S;

ZOOM_INFO_S m_stZoomInfo = {0};

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

void* zoom_alg_Body(void *pvArg)
{
    sint32 s32Ret = 0, i;
    ZOOM_INFO_S *pstZoomInfo = (ZOOM_INFO_S *)pvArg;
    void *pbuf = NULL;
    uint32 u32BufLen = ZOOM_IMAGE_SRC_WIDTH*ZOOM_IMAGE_SRC_HEIGHT*3;

#if ZOOM_ALG_TEST
    pdsa32::STAlgInfo stPdResult = {0};
    pdsa32::STTrackParam stTrackParam;
    MEDIA_GUI_DRAW_S stMediaGuiDraw = {0};
    MEDIA_GUI_PERSON_S stGuiRect = {0};
    MEDIA_GUI_NULL_S stGuiNull;    
    MSG_PACKET_S stMsgPkt = {0};
    uint32 u32RectCnt = 0;
    uint16 u16mask;
#endif

    pbuf = mmap(NULL, u32BufLen, PROT_READ, MAP_SHARED, pstZoomInfo->s32MediaBufFd, 0);
    if (MAP_FAILED == pbuf)
    {
        print_level(SV_ERROR, "zoom mmap failed.\n");
        return NULL;
    }

    while(pstZoomInfo->bRunning)
    {
#if ZOOM_ALG_TEST
        memset(&stPdResult, 0x00, sizeof(pdsa32::STAlgInfo));
        memset(&stMediaGuiDraw, 0x00, sizeof(stMediaGuiDraw));
        memset(&stGuiRect, 0x00, sizeof(stGuiRect));
        u32RectCnt = 0;

        /* 清空原来的画板 */
        u16mask = MEDIA_GUI_GET_MASK(ZOOM_ALG_CHN, 0, MEDIA_GUI_OP_CLEAR);
        s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiNull);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
        }
#endif
        s32Ret = ALG_Calculate_Lock();
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "ALG_Calculate_Lock failed. [err=%d]\n", s32Ret);
            sleep_ms(1);
            continue;
        }    
        s32Ret = MS_P(ZOOM_ALG_CHN);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "MS_P failed. [err=%d]\n", s32Ret);
            ALG_Calculate_unLock();
            sleep_ms(1);
            continue;
        }
#if ZOOM_ALG_TEST
        s32Ret = pstZoomInfo->pdsAlg->AlgForward((char *)pbuf, \
                ZOOM_IMAGE_SRC_WIDTH, ZOOM_IMAGE_SRC_HEIGHT, \
                (uint32)pstZoomInfo->s32MediaBufFd, ZOOM_ALG_CHN);
#endif
                
        if (0 != s32Ret)
        {
            print_level(SV_ERROR, "ALGPDS_forward failed. [err=%d]\n", s32Ret);
            MS_V(ZOOM_ALG_CHN);
            ALG_Calculate_unLock();
            sleep_ms(10);
            continue;
        }
        
        MS_V(ZOOM_ALG_CHN);
        ALG_Calculate_unLock();
#if ZOOM_ALG_TEST
        pstZoomInfo->pdsAlg->AlgResult(&stPdResult, pdsa32::E_POST_NONE, stTrackParam, ZOOM_ALG_CHN);
        if (0 != s32Ret)
        {
            print_level(SV_ERROR, "AlgResult failed. [err=%d]\n", s32Ret);
            sleep_ms(10);
            continue;
        }
        
        for (i = 0; i < stPdResult.u32Nums; i++)
        {
            if (stPdResult.stResults[i].classes != pdsa32::E_CLS_PERSON)
            {
                continue;
            }
            if (u32RectCnt >= 20)
            {
                break;
            }            
            stGuiRect.astPersonsRect[u32RectCnt].stick = 3;
            stGuiRect.classes[u32RectCnt] = (PdsAlgObjectClass)stPdResult.stResults[i].classes;
            stGuiRect.astPersonsRect[u32RectCnt].x1 = stPdResult.stResults[i].fX1;
            stGuiRect.astPersonsRect[u32RectCnt].y1 = stPdResult.stResults[i].fY1;
            stGuiRect.astPersonsRect[u32RectCnt].x2 = stPdResult.stResults[i].fX2;
            stGuiRect.astPersonsRect[u32RectCnt].y2 = stPdResult.stResults[i].fY2;
            stGuiRect.astPersonsRect[u32RectCnt].fontscale = 1;
            stGuiRect.au32Score[u32RectCnt] = 0;
            stGuiRect.as32DistanceX[u32RectCnt] = 0;
            stGuiRect.as32DistanceY[u32RectCnt] = -1;
            stGuiRect.astPersonsRect[u32RectCnt].color = GUI_COLOR_GREEN;
            u32RectCnt++;
        }        
        stGuiRect.u32PersonNum = u32RectCnt;
        
        /* 添加行人矩形绘制操作 */
        u16mask = MEDIA_GUI_GET_MASK(ZOOM_ALG_CHN, 0, MEDIA_GUI_OP_PERSON_RECT);
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
#endif

    }
    munmap(pbuf, u32BufLen);
    return NULL;
}

sint32 Zoom_Init(ZOOM_CFG_PARAM_S *pstInitParam)
{
    print_level(SV_WARN, "zoom init\n");
    sint32 s32Ret = 0;
    CHN_ALG_E *penAlgType;
    CFG_ZOOM_PARAM *pstZoomParam = {0};
    
    if (NULL == pstInitParam)
    {
        return ERR_NULL_PTR;
    }
    
    memset(&m_stZoomInfo, 0, sizeof(ZOOM_INFO_S));
    s32Ret = pthread_mutex_init(&m_stZoomInfo.mutexlock, NULL);
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
    
    penAlgType = &pstInitParam->stAlgParam.stAlgCh2.enAlgType;
    pstZoomParam = &pstInitParam->stAlgParam.stAlgCh2.stZoomParam;

#if ZOOM_ALG_TEST
    pdsa32::CPdsAlg *apcsPdsAlgRgbP = NULL;
    pdsa32::STAlgParam stAlgParam(0.52, 0.45); 

    apcsPdsAlgRgbP = new pdsa32::CPdsAlg(pdsa32::E_PDSALG_TYPE_RGB_P);
    if (NULL == apcsPdsAlgRgbP)
    {
        print_level(SV_ERROR, "new pdsa32::CPdsAlg failed. [err=%d]\n", s32Ret);
        return SV_FAILURE;
    }
    
    s32Ret = apcsPdsAlgRgbP->AlgInit("/root/model/RGB_P.rknn", stAlgParam, []{return 0;});
    if (0 != s32Ret)
    {
        print_level(SV_ERROR, "csPdsAlg.AlgInit failed. [err=%d]\n", s32Ret);
        return SV_FAILURE;
    }
    m_stZoomInfo.pdsAlg = apcsPdsAlgRgbP;
#endif
    m_stZoomInfo.s32MediaBufFd = pstInitParam->as32MediaBufFd[0];
    m_stZoomInfo.stZoomParam = pstInitParam->stAlgParam.stAlgCh2.stZoomParam;
    print_level(SV_INFO, "alg type[%d]\n",*penAlgType);
    return SV_SUCCESS;
}

sint32 Zoom_Fini()
{
    return SV_SUCCESS;
}

sint32 Zoom_Start()
{
    print_level(SV_WARN, "zoom start\n");
    sint32 s32Ret = SV_SUCCESS;
    pthread_t thread;
    m_stZoomInfo.bRunning = SV_TRUE;
    s32Ret = pthread_create(&thread, NULL, zoom_alg_Body, &m_stZoomInfo);
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
    return SV_SUCCESS;
}

sint32 Zoom_Stop()
{
    return SV_SUCCESS;
}


