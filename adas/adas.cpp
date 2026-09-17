/******************************************************************************
Copyright (C) 2021-2023 广州敏视数码科技有限公司版权所有.

文件名：adas.c

日期: 2022-03-03

文件功能描述: 定义ADAS算法功能接口

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
#include "lane.hpp"
#include "laneprocess.h"
#include "adas.h"
#include "config.h"
#include "media.h"
#include "media_sem.h"
#include "alg.h"
#include "cJSON.h"
#include "board.h"

#include "CaliEX.h"
#include <jpeg/jpeglib.h>
#include <jpeg/jerror.h>

using namespace std;

#define min(x, y) (((x) < (y)) ? (x) : (y))
#define max(x, y) (((x) > (y)) ? (x) : (y))
#define abs(x) ((x)<0? -(x) : (x))

#define ADAS_ALARM_PIN_BAND 3
#define ADAS_ALARM_PIN_NUM  20

#define ADAS_MODEL_RGB_P        "/root/RGB_P.rknn"      /* 可见光检人模型 */
#define ADAS_MODEL_RGB_P_OW     "/root/RGB_P_OW.rknn"   /* 可见光俯视大角度检人模型 */
#define ADAS_MODEL_RGB_PC       "/root/RGB_PC.rknn"     /* 可见光检人和车模型 */
#define ADAS_MODEL_IR_PC        "/root/RED_PC.rknn"     /* 红外热成像检人和车模型 */
#define ADAS_MODEL_LANE         "/root/LANE.rknn"       /* 车道线检测模型 */
#define ADAS_IMAGE_WIDTH        608                     /* 算法图像帧宽度 */
#define ADAS_IMAGE_HEIGHT       352                     /* 算法图像帧高度 */
#define LDW_IMAGE_WIDTH         416                     /* 轨道检测图像帧宽度 */
#define LDW_IMAGE_HEIGHT        224                     /* 轨道检测图像帧高度 */

#pragma  pack(1)
typedef struct tag_Type24
{
    char buf[3];
} TYPE24;
#pragma  pack()

typedef struct tagAdasGuiImg_S
{
    char *pbmp;
    sint32 s32Width;
    sint32 s32Height;
} ADAS_GUI_IMG_S;

/* 模块控制信息 */
typedef struct tagAdasInfo_S
{
    CLane          *pcsLaneAlg;             /* 车道线算法对你指针 */
    pdsa32::CPdsAlg *pcsPdsAlg;             /* PD算法对象指针 */
    pdsa32::STTrackParam stTrackParam;      /* PD算法跟踪参数 */
    sint32          s32MediaBufFd;          /* 媒体通道Media Buffer的文件描述符 */
    CFG_ALG_PARAM   stCfgParam;             /* 算法配置参数 */
    float           fConfidenceThr;         /* 算法得分阈值 */
    float           fNmsThreshold;          /* 算法NMS阈值 */
    float           fTrackThreshold;        /* 跟踪模块的阈值 */
    float           fTrackMaxAge;           /* 跟踪模块的参数 */
    uint32          u32TidAlg;              /* 算法线程ID */
    SV_BOOL         bRunning;               /* 线程是否正在运行 */
    pthread_mutex_t mutexRunStat;           /* 算法运行状态互斥锁 */
} ADAS_INFO_S;

/* ADAS 输出信息 */
typedef struct tagAdasDumpInfo_S
{
    sint64          s64TimeStamp;           /* 时间戳 */
    sint32          s32GreenRoiNum;         /* 绿色ROI区域检测数量 */
    sint32          s32YellowRoiNum;        /* 黄色ROI区域检测数量 */
    sint32          s32RedRoiNum;           /* 红色ROI区域检测数量 */
    sint32          s32DistanceNum;         /* 目标数量 */
    sint32          s32DistanceXY[20*2];    /* 检测到的目标的距离坐标(x,y),上限为20个 */
} ADAS_DUMP_INFO_S;

ADAS_INFO_S m_stAdasInfo = {0};             /* 模块控制信息 */
extern int ipsys_log_level;

sint32 adas_Alarm_Out_Enable()
{
    sint32 s32Ret;
    uint8 u8Value = 1;
#if (defined(BOARD_ADA32V2) || defined(BOARD_ADA32IR) || defined(BOARD_ADA47V1) || defined(BOARD_ADA32C4))
    if(BOARD_IsCustomer(BOARD_C_ADA32V2_WXKY) || BOARD_IsCustomer(BOARD_C_ADA32V2_201819))
        u8Value = 0;
    s32Ret = BOARD_RK_SetGPIO(PD_ALARM_PIN_BAND, PD_ALARM_PIN_NUM , u8Value);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "BOARD_RK_SetGPIO fail![err=%#x]\n", s32Ret);
        return s32Ret;
    }
#endif

    return SV_SUCCESS;
}

sint32 adas_Alarm_Out_Reset()
{
    sint32 s32Ret;
    uint8 u8Value = 0;
#if (defined(BOARD_ADA32V2) || defined(BOARD_ADA32IR) || defined(BOARD_ADA47V1) || defined(BOARD_ADA32C4))
    if(BOARD_IsCustomer(BOARD_C_ADA32V2_WXKY) || BOARD_IsCustomer(BOARD_C_ADA32V2_201819))
        u8Value = 1;
    s32Ret = BOARD_RK_SetGPIO(PD_ALARM_PIN_BAND, PD_ALARM_PIN_NUM , u8Value);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "BOARD_RK_SetGPIO fail![err=%#x]\n", s32Ret);
        return s32Ret;
    }
#endif

    return SV_SUCCESS;
}

sint32 adas_string_to_file(char *file, char *string)
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


sint32 adas_DumpInfo(ADAS_DUMP_INFO_S *pstPdDumpInfo)
{
    sint32 s32Ret = 0;
    sint32 fd = -1;
    cJSON *pstJson = NULL, *pstTmp = NULL;
    cJSON *pstTimeStamp = NULL, *pstPdWorkMode = NULL, *pstGreenRoiNum = NULL, *pstYellowRoiNum = NULL, *pstRedRoiNum = NULL;
    char szBuf[1024];
    char echo_szBuf[2048];

    pstJson = cJSON_CreateObject();
    if (NULL == pstJson)
    {
        print_level(SV_ERROR, "cJSON_CreateObject fail!\n");
        return SV_FAILURE;
    }

    pstTimeStamp = cJSON_CreateNumber((double)pstPdDumpInfo->s64TimeStamp);
    if(NULL == pstTimeStamp)
    {
        print_level(SV_ERROR, "cJSON_CreateNumber failed.\n");
        goto exit;
    }
    cJSON_AddItemToObject(pstJson, "TimeStamp", pstTimeStamp);

    pstPdWorkMode = cJSON_CreateNumber(m_stAdasInfo.stCfgParam.stAlgCh2.stPdsParam.enPDWorkMode);
    if(NULL == pstTimeStamp)
    {
        print_level(SV_ERROR, "cJSON_CreateNumber failed.\n");
        goto exit;
    }
    cJSON_AddItemToObject(pstJson, "ADASWorkMode", pstPdWorkMode);

    pstGreenRoiNum = cJSON_CreateNumber((double)pstPdDumpInfo->s32GreenRoiNum);
    if(NULL == pstGreenRoiNum)
    {
        print_level(SV_ERROR, "cJSON_CreateNumber failed.\n");
        goto exit;
    }
    cJSON_AddItemToObject(pstJson, "GreenRoiNum", pstGreenRoiNum);

    pstYellowRoiNum = cJSON_CreateNumber((double)pstPdDumpInfo->s32YellowRoiNum);
    if(NULL == pstYellowRoiNum)
    {
        print_level(SV_ERROR, "cJSON_CreateNumber failed.\n");
        goto exit;
    }
    cJSON_AddItemToObject(pstJson, "YellowRoiNum", pstYellowRoiNum);

    pstRedRoiNum = cJSON_CreateNumber((double)pstPdDumpInfo->s32RedRoiNum);
    if(NULL == pstRedRoiNum)
    {
        print_level(SV_ERROR, "cJSON_CreateNumber failed.\n");
        goto exit;
    }
    cJSON_AddItemToObject(pstJson, "RedRoiNum", pstRedRoiNum);

#if 0
    if(pstPdDumpInfo->s32DistanceNum > 0)
    {
        cJSON_AddItemToObject(pstJson, "DistanceXY", cJSON_CreateIntArray((int*)pstPdDumpInfo->s32DistanceXY, pstPdDumpInfo->s32DistanceNum*2));
    }
#endif

    memset(szBuf, 0, 1024);
    cJSON_PrintPreallocated(pstJson, szBuf, 1024, 0);
    
    adas_string_to_file("/var/info/adas-tmp", szBuf);
    rename("/var/info/adas-tmp", "/var/info/adas");

exit:
    cJSON_Delete(pstJson);
    return SV_SUCCESS;
}


/******************************************************************************
 * 函数功能: 缩放图片
 * 输入参数: pstPdImgDst --- 目标位图数据
             pstPdImgSrc --- 源位图数据
 * 输出参数: 无
 * 返回值  : 无
 *****************************************************************************/
sint32 adas_Gui_Zoom(ADAS_GUI_IMG_S *pstPdImgDst, ADAS_GUI_IMG_S *pstPdImgSrc)
{
    sint32 s32Ret;

    if(pstPdImgDst == NULL || pstPdImgSrc == NULL)
        return ERR_NULL_PTR;

    if((0 == pstPdImgDst->s32Height) || (0 == pstPdImgDst->s32Width) ||
       (0 == pstPdImgSrc->s32Height) || (0 == pstPdImgSrc->s32Width))
        return SV_SUCCESS;

    unsigned long xrIntFloat_16 = (pstPdImgSrc->s32Width << 16) / pstPdImgDst->s32Width + 1;
    unsigned long yrIntFloat_16 = (pstPdImgSrc->s32Height << 16) / pstPdImgDst->s32Height + 1;
    unsigned long dst_width = pstPdImgDst->s32Width;

    TYPE24 *pDstLine, *pSrcLine;
    char *pbmp_dst, *pbmp_src;
    unsigned long srcy_16 = 0, srcx_16 = 0;
    pDstLine = (TYPE24 *)pstPdImgDst->pbmp;
    for(unsigned long y = 0; y < pstPdImgDst->s32Height; y++)
    {
        pSrcLine = (TYPE24 *)pstPdImgSrc->pbmp + pstPdImgSrc->s32Width*(srcy_16>>16);
        srcx_16 = 0;
        for(unsigned long x = 0; x < pstPdImgDst->s32Width; x++)
        {
            pDstLine[x] = pSrcLine[srcx_16>>16];
            srcx_16+=xrIntFloat_16;
        }
        srcy_16+=yrIntFloat_16;
        pDstLine+=pstPdImgDst->s32Width;
    }
    
    return SV_SUCCESS;
}


sint32 adas_PointImageToReal(SV_POINT2_S *pstPointImage, SV_3DPOINT2_S *pstPointReal)
{
    sint32 s32Ret = 0;
    STImgPoint stImg = {0};
    STWorldPoint stWorld = {0};

    stImg.fX = pstPointImage->dX * 1920;
    stImg.fY = pstPointImage->dY * 1080;

    //s32Ret = Cali_distance(stImg, stWorld);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "Cali_projection failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }

    pstPointReal->dX = stWorld.fX;
    pstPointReal->dY = stWorld.fY;
    pstPointReal->dZ = stWorld.fZ;


    return s32Ret;
}

/******************************************************************************
 * 函数功能: 对行人算法识别的矩形框进行滤波防抖
 * 输入参数: pstNewResult --- 最新帧识别的结果
             fThresholdX --- X轴方向跨度阈值系数
             fThresholdY --- Y轴方向跨度阈值系数
 * 输出参数: 无
 * 返回值  : 无
 * 注意    : 该函数是不可重入函数，只允许一个线程调用
 *****************************************************************************/
void adas_RectAntiShakeFilter(pdsa32::STAlgInfo *pstNewResult, float fThresholdX, float fThresholdY)
{
    sint32 i, j, k;
    pdsa32::STAlgResult stNewRect, stOldRect;
    static pdsa32::STAlgInfo stOldResult;
    float fThrDistWidth, fThrDistHeight, fMoveDist;

    if (NULL == pstNewResult)
    {
        return;
    }

    for (i = 0; i < pstNewResult->u32Nums; i++)
    {
        stNewRect.fX1 = pstNewResult->stResults[i].fX1 * ADAS_IMAGE_WIDTH;
        stNewRect.fY1 = pstNewResult->stResults[i].fY1 * ADAS_IMAGE_HEIGHT;
        stNewRect.fX2 = pstNewResult->stResults[i].fX2 * ADAS_IMAGE_WIDTH;
        stNewRect.fY2 = pstNewResult->stResults[i].fY2 * ADAS_IMAGE_HEIGHT;
        for (j = 0; j < stOldResult.u32Nums; j++)
        {
            stOldRect.fX1 = stOldResult.stResults[j].fX1 * ADAS_IMAGE_WIDTH;
            stOldRect.fY1 = stOldResult.stResults[j].fY1 * ADAS_IMAGE_HEIGHT;
            stOldRect.fX2 = stOldResult.stResults[j].fX2 * ADAS_IMAGE_WIDTH;
            stOldRect.fY2 = stOldResult.stResults[j].fY2 * ADAS_IMAGE_HEIGHT;
            fThrDistWidth = fThresholdX * (stNewRect.fX2 - stNewRect.fX1);
            fThrDistHeight = fThresholdY * (stNewRect.fY2 - stNewRect.fY1);
            fMoveDist = sqrtf(powf((stOldRect.fX1 - stNewRect.fX1),2) + powf((stOldRect.fY1 - stNewRect.fY1),2));
            if (fMoveDist < fThrDistWidth)
            {
                fMoveDist = sqrtf(powf((stOldRect.fX2 - stNewRect.fX2),2) + powf((stOldRect.fY2 - stNewRect.fY2),2));
                if (fMoveDist < fThrDistHeight)
                {
                    pstNewResult->stResults[i].fX1 = stOldResult.stResults[j].fX1;
                    pstNewResult->stResults[i].fY1 = stOldResult.stResults[j].fY1;
                    pstNewResult->stResults[i].fX2 = stOldResult.stResults[j].fX2;
                    pstNewResult->stResults[i].fY2 = stOldResult.stResults[j].fY2;

                    /* 排队该旧帧矩形框减少下个新矩形的匹配遍历数 */
                    for (k = j; k < stOldResult.u32Nums - 1; k++)
                    {
                        stOldResult.stResults[k] = stOldResult.stResults[k+1];
                    }
                    stOldResult.u32Nums--;
                    break;
                }
            }
        }
    }

    stOldResult = *pstNewResult;
}


/* ADAS算法驱动线程 */
void * adas_alg_Body(void *pvArg)
{
    sint32 s32Ret = 0, i;
    sint32 s32AlarmMode = -1;
    uint32 u32RectCnt = 0;
    ADAS_INFO_S *pstAdasInfo = (ADAS_INFO_S *)pvArg;
    ADAS_DUMP_INFO_S stPdDumpInfo;
    uint32 u32StepTimeMs = 0;
    struct timespec tvLast = {0, 0};
    struct timespec tvNow = {0, 0};
    struct timespec tvBegin, tvEnd;
    pdsa32::STAlgInfo stPdResult = {0};
    MSG_PACKET_S stMsgPkt = {0};
    MEDIA_GUI_DRAW_S stMediaGuiDraw = {0};
    MEDIA_GUI_PERSON_S stGuiRect = {0};
    MEDIA_GUI_LINE_S stGuiLine = {0};
    MEDIA_GUI_LINE_S astGuiLines2[18] = {0};
    MEDIA_GUI_NULL_S stGuiNull;
    ADAS_GUI_IMG_S stAdsaImgSrc, stLaneImgDst;
    vector<STLaneInfo> vec_LaneInfoList1 {};
    vector<STLaneInfo> vec_LaneInfoList2 {};
    sint32 s32Len1 = 0, s32Len2 = 0;

  	uint16 u16mask;
    void *pvBuf = NULL;
    uint32 u32BufLen = ADAS_IMAGE_WIDTH*ADAS_IMAGE_HEIGHT*3;

    s32Ret = prctl(PR_SET_NAME, "adas_body");
    if (0 != s32Ret)
    {
        print_level(SV_ERROR, "prctl PR_SET_NAME failed! [err:%s]\n", strerror(errno));
    }

    pvBuf = mmap(NULL, u32BufLen, PROT_READ, MAP_SHARED, pstAdasInfo->s32MediaBufFd, 0);
    if (MAP_FAILED == pvBuf)
    {
        print_level(SV_ERROR, "mmap[%d] failed.\n", i);
        return NULL;
    }

    stAdsaImgSrc.s32Width = ADAS_IMAGE_WIDTH;
    stAdsaImgSrc.s32Height = ADAS_IMAGE_HEIGHT;
    stAdsaImgSrc.pbmp = pvBuf;
    stLaneImgDst.s32Width = SV_WIDTH;
    stLaneImgDst.s32Height = SV_HEIGHT;
    stLaneImgDst.pbmp = malloc(stLaneImgDst.s32Width * stLaneImgDst.s32Height * 3);
    if (stLaneImgDst.pbmp == NULL)
    {
        print_level(SV_ERROR, "malloc failed.\n", i);
        return NULL;
    }
    
    print_level(SV_INFO, "enter ADAS detection.\n");
    clock_gettime(CLOCK_MONOTONIC, &tvLast);
    while (pstAdasInfo->bRunning)
    {
        //print_level(SV_DEBUG, "adas_alg_Body running...\n");
        if (ALG_ADAS != pstAdasInfo->stCfgParam.stAlgCh1.enAlgType)
        {
            sleep_ms(1000);
            continue;
        }
        
        clock_gettime(CLOCK_MONOTONIC, &tvNow);
        u32StepTimeMs = ((tvNow.tv_sec*1000 + tvNow.tv_nsec/1000000) - (tvLast.tv_sec*1000 + tvLast.tv_nsec/1000000));
        tvLast = tvNow;
        memset(&stPdDumpInfo, 0x00, sizeof(ADAS_DUMP_INFO_S));
        s32AlarmMode = -1;

        stPdDumpInfo.s64TimeStamp = tvNow.tv_sec * 1000 + tvNow.tv_nsec /1000000;

        /* 锁住计算资源,保证同一时刻只跑一个算法*/
        s32Ret = ALG_Calculate_Lock();
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "ALG_Calculate_Lock failed. [err=%d]\n", s32Ret);
            sleep_ms(1);
            continue;
        }

        /* P操作进入MediaBuffer临界区 */
        s32Ret = MS_P(0);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "MS_P failed. [err=%d]\n", s32Ret);
            ALG_Calculate_unLock();
            sleep_ms(1);
            continue;
        }

        clock_gettime(CLOCK_MONOTONIC, &tvBegin);
        s32Ret = pstAdasInfo->pcsPdsAlg->AlgForward((char *)pvBuf, ADAS_IMAGE_WIDTH, ADAS_IMAGE_HEIGHT);
        if (0 != s32Ret)
        {
            print_level(SV_ERROR, "ALGPDS_forward failed. [err=%d]\n", s32Ret);
            MS_V(0);
            ALG_Calculate_unLock();
            sleep_ms(10);
            continue;
        }

        if (pstAdasInfo->stCfgParam.stAlgCh1.stAdasParam.bLdwEnable)
        {
            s32Ret = adas_Gui_Zoom(&stLaneImgDst, &stAdsaImgSrc);
            if(s32Ret != SV_SUCCESS)
            {
                print_level(SV_ERROR, "adas_Gui_Zoom fail!\n");
            }
            else
            {
                pstAdasInfo->pcsLaneAlg->LaneDetect(stLaneImgDst.pbmp, vec_LaneInfoList1, vec_LaneInfoList2);
                LaneProcess(vec_LaneInfoList1, vec_LaneInfoList2);
                s32Len1 = vec_LaneInfoList1.size();
                s32Len2 = vec_LaneInfoList2.size();
                //print_level(SV_DEBUG, "lane1:%d, lane2:%d\n", s32Len1, s32Len1);
            }
        }

        MS_V(0);
        ALG_Calculate_unLock();
        clock_gettime(CLOCK_MONOTONIC, &tvEnd);
        //print_level(SV_INFO, "eclipse: %dms\n", (tvEnd.tv_sec*1000 + tvEnd.tv_nsec/1000000) - (tvBegin.tv_sec*1000 + tvBegin.tv_nsec/1000000));

        memset(&stPdResult, 0, sizeof(pdsa32::STAlgInfo));
        pstAdasInfo->pcsPdsAlg->AlgResult(&stPdResult, pdsa32::E_POST_TRACK, pstAdasInfo->stTrackParam);
        if (0 != s32Ret)
        {
            print_level(SV_ERROR, "pcsPdsAlg->AlgResult failed. [err=%d]\n", s32Ret);
            sleep_ms(10);
            continue;
        }

        adas_RectAntiShakeFilter(&stPdResult, 0.1, 0.2);
        memset(&stMediaGuiDraw, 0x00, sizeof(stMediaGuiDraw));

        /* 清空原来的画板 */
        u16mask = MEDIA_GUI_GET_MASK(0, 0, MEDIA_GUI_OP_CLEAR);
        s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiNull);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
        }

        /* 添加轨道线 */
        for (i = 0; i < s32Len1 - 1; i++)
        {
            stGuiLine.x1 = 1.f * vec_LaneInfoList1[i].u64x / SV_WIDTH;
            stGuiLine.y1 = 1.f * vec_LaneInfoList1[i].u64y / SV_HEIGHT;
            stGuiLine.x2 = 1.f * vec_LaneInfoList1[i+1].u64x / SV_WIDTH;
            stGuiLine.y2 = 1.f * vec_LaneInfoList1[i+1].u64y / SV_HEIGHT;
            stGuiLine.color = GUI_COLOR_L_GREEN;
            stGuiLine.stick = 10;
            //print_level(SV_DEBUG, "(%f, %f) (%f, %f)\n", stGuiLine.x1, stGuiLine.y1, stGuiLine.x2, stGuiLine.y2);
            u16mask = MEDIA_GUI_GET_MASK(0, 0, MEDIA_GUI_OP_DRAW_LINE);
            s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiLine);
            if (SV_SUCCESS != s32Ret)
            {
                print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
            }
        }
        for (i = 0; i < s32Len2 - 1; i++)
        {
            stGuiLine.x1 = 1.f * vec_LaneInfoList2[i].u64x / SV_WIDTH;
            stGuiLine.y1 = 1.f * vec_LaneInfoList2[i].u64y / SV_HEIGHT;
            stGuiLine.x2 = 1.f * vec_LaneInfoList2[i+1].u64x / SV_WIDTH;
            stGuiLine.y2 = 1.f * vec_LaneInfoList2[i+1].u64y / SV_HEIGHT;
            stGuiLine.color = (vec_LaneInfoList2[i].u64state == 0) ? GUI_COLOR_L_GREEN : GUI_COLOR_RED;
            stGuiLine.stick = 10;
            //print_level(SV_DEBUG, "(%f, %f) (%f, %f)\n", stGuiLine.x1, stGuiLine.y1, stGuiLine.x2, stGuiLine.y2);
            u16mask = MEDIA_GUI_GET_MASK(0, 0, MEDIA_GUI_OP_DRAW_LINE);
            s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiLine);
            if (SV_SUCCESS != s32Ret)
            {
                print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
            }
        }
        
        s32Len1 = 0;
        s32Len2 = 0;
        memset(&stMsgPkt, 0, sizeof(stMsgPkt));
        stMsgPkt.pu8Data = (uint8*)&stMediaGuiDraw;
        stMsgPkt.u32Size = sizeof(stMediaGuiDraw);
        s32Ret = Msg_submitEvent(EP_CONTROL, OP_EVENT_MEDIA_GUI, &stMsgPkt);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
        }

        /* 添加人车检测矩形框 */
        u32RectCnt = 0;
        for (i = 0; i < stPdResult.u32Nums; i++)
        {
            //print_level(SV_DEBUG, "confidence:%f, (%f,%f) (%f,%f)\n", stPdResult.stResults[i].fConfidence, stPdResult.stResults[i].fX1, stPdResult.stResults[i].fY1, stPdResult.stResults[i].fX2, stPdResult.stResults[i].fY2);
            if(u32RectCnt >= 20)
            {
                break;
            }

            stGuiRect.astPersonsRect[u32RectCnt].color = GUI_COLOR_GREEN;
            stGuiRect.astPersonsRect[u32RectCnt].stick = 3;
            stGuiRect.classes[u32RectCnt] = (sint32)stPdResult.stResults[i].classes;
            stGuiRect.astPersonsRect[u32RectCnt].x1 = stPdResult.stResults[i].fX1;
            stGuiRect.astPersonsRect[u32RectCnt].y1 = stPdResult.stResults[i].fY1;
            stGuiRect.astPersonsRect[u32RectCnt].x2 = stPdResult.stResults[i].fX2;
            stGuiRect.astPersonsRect[u32RectCnt].y2 = stPdResult.stResults[i].fY2;
            stGuiRect.astPersonsRect[u32RectCnt].fontscale = 2;
            u32RectCnt++;
        }

        //print_level(SV_DEBUG, "u32PersonNum:%d\n", stGuiRect.u32PersonNum);
        memset(&stMediaGuiDraw, 0x00, sizeof(stMediaGuiDraw));
        u16mask = MEDIA_GUI_GET_MASK(0, 0, MEDIA_GUI_OP_PERSON_RECT);
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
        
        stGuiRect.u32PersonNum = u32RectCnt;
    }

    print_level(SV_INFO, "exit ADAS detection.\n");
    munmap(pvBuf, u32BufLen);
    free(stLaneImgDst.pbmp);

    return NULL;
}

sint32 ADAS_Init(ADAS_CFG_PARAM_S *pstInitParam)
{
    sint32 s32Ret = 0, i;
    sint32 s32CenterGaze = 0;
    float afThresholds[4];
    uint32 u32ThrNum = 4;
    char *pszModelFile = NULL;

    if (NULL == pstInitParam)
    {
        return ERR_NULL_PTR;
    }

    if (pstInitParam->s32MediaBufFd < 0)
    {
        return ERR_ILLEGAL_PARAM;
    }

    memset(&m_stAdasInfo, 0, sizeof(ADAS_INFO_S));
    s32Ret = pthread_mutex_init(&m_stAdasInfo.mutexRunStat, NULL);
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

    m_stAdasInfo.s32MediaBufFd = pstInitParam->s32MediaBufFd;
    adas_Alarm_Out_Reset();
    if (pstInitParam->stAlgParam.stAlgCh1.stAdasParam.bLdwEnable)
    {
        m_stAdasInfo.pcsLaneAlg = new CLane(SV_WIDTH, SV_HEIGHT, SV_GRIDING, SV_NUMLANES, ADAS_MODEL_LANE);
        if (NULL == m_stAdasInfo.pcsLaneAlg)
        {
            print_level(SV_ERROR, "new CLane failed. [err=%d]\n", s32Ret);
            return SV_FAILURE;
        }
    }
    
    pszModelFile = ADAS_MODEL_RGB_PC;
    m_stAdasInfo.stCfgParam = pstInitParam->stAlgParam;
    m_stAdasInfo.fConfidenceThr = 0.3f;
    m_stAdasInfo.fNmsThreshold = 0.45f;
    m_stAdasInfo.fTrackThreshold = 0.6f;
    m_stAdasInfo.fTrackMaxAge = 4;
    afThresholds[0] = m_stAdasInfo.fConfidenceThr;
    afThresholds[1] = m_stAdasInfo.fNmsThreshold = 0.45f;
    afThresholds[2] = m_stAdasInfo.fTrackThreshold = 0.6f;
    afThresholds[3] = m_stAdasInfo.fTrackMaxAge = 4;
    m_stAdasInfo.pcsPdsAlg = new pdsa32::CPdsAlg(pdsa32::E_PDSALG_TYPE_RGB_PC);
    if (NULL == m_stAdasInfo.pcsPdsAlg)
    {
        print_level(SV_ERROR, "new pdsa32::CPdsAlg failed. [err=%d]\n", s32Ret);
        return SV_FAILURE;
    }
    
    pdsa32::STAlgParam stAlgParam(0.52, 0.45);              // 设置算法参数
    pdsa32::STTrackParam stTacrkParam;    // 跟踪参数
    s32Ret = m_stAdasInfo.pcsPdsAlg->AlgInit(pszModelFile, stAlgParam);
    if (0 != s32Ret)
    {
        print_level(SV_ERROR, "csPdsAlg.AlgInit failed. [err=%d]\n", s32Ret);
        return SV_FAILURE;
    }
    
    m_stAdasInfo.stTrackParam = stTacrkParam;

    return SV_SUCCESS;
}

sint32 ADAS_Fini()
{
    if (NULL != m_stAdasInfo.pcsLaneAlg)
    {
        m_stAdasInfo.pcsLaneAlg->LaneRelease();
        delete m_stAdasInfo.pcsLaneAlg;
    }
    if (NULL != m_stAdasInfo.pcsPdsAlg)
    {
        delete m_stAdasInfo.pcsPdsAlg;
    }
    pthread_mutex_destroy(&m_stAdasInfo.mutexRunStat);

    return SV_SUCCESS;
}

sint32 ADAS_Start()
{
    sint32 s32Ret = 0;
    pthread_t thread;

    m_stAdasInfo.bRunning = SV_TRUE;
    s32Ret = pthread_create(&thread, NULL, adas_alg_Body, &m_stAdasInfo);
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

    m_stAdasInfo.u32TidAlg = thread;
    
    return SV_SUCCESS;
}

sint32 ADAS_Stop()
{
    sint32 s32Ret = 0;
    pthread_t thread = m_stAdasInfo.u32TidAlg;
    void *pvRetval = NULL;

    m_stAdasInfo.bRunning = SV_FALSE;
    s32Ret = pthread_join(thread, &pvRetval);
    if (0 != s32Ret)
    {
        print_level(SV_ERROR, "pthread_join failed. [err: %s]\n", strerror(errno));
        return SV_FAILURE;
    }

    return SV_SUCCESS;
}

sint32 ADAS_ConfigSet(CFG_ALG_PARAM *pstCfgParam)
{
    if (NULL == pstCfgParam)
    {
        return ERR_NULL_PTR;
    }

    pthread_mutex_lock(&m_stAdasInfo.mutexRunStat);
    m_stAdasInfo.stCfgParam = *pstCfgParam;
    pthread_mutex_unlock(&m_stAdasInfo.mutexRunStat);
    
    return SV_SUCCESS;
}



