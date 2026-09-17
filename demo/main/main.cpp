/******************************************************************************
Copyright (C) 2021-2023 广州敏视数码科技有限公司版权所有.

文件名：main.cpp

日期: 2021-08-03

文件功能描述: 定义媒体接口访问demo

其他: // 其他内容说明

版本: v1.0.0(最新版本号)

*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
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
#include <sys/socket.h>
#include <sys/fcntl.h>
#include <sys/un.h>

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
#include "config.h"
#include "../../../../include/board.h"
#include "op.h"
#include "msg.h"
#include "alarm.h"
#include "unsocket.h"
#include "gps.h"
#include "media.h"
#include "media_sem.h"

#define _BASETSD_H

#include "opencv2/core/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include "alglib.h"

#define MEDIA_IMAGE_WIDTH      608                  /* 算法图像帧宽度 */
#define MEDIA_IMAGE_HEIGHT     352                  /* 算法图像帧高度 */

#define SP_ALARM_PIN_BAND       3
#define SP_ALARM_PIN_NUM        20

typedef enum tagSampleAlarm_Type_E
{
    SP_ALARM_TYPE_NULL = 0, /* 无报警触发 */
    SP_ALARM_TYPE_GREEN,
    SP_ALARM_TYPE_YELLOW,
    SP_ALARM_TYPE_RED,

    SP_ALARM_TYPE_BUTT,
} SP_ALARM_TYPE;


static int ipsys_log_level = SV_DEBUG;
static int g_as32MediaBufFd[4] = {-1, -1, -1, -1}; // 各物理通道的Media Buffer的文件描述符 // The file descriptor of the Media Buffer of each physical channel 
static int g_s32SocketFd = -1;
static int g_running = 0;
static char *g_modelPath = NULL;

sint32 MEDIA_GET_ALG_FD(int *pfd, int idx){ return SV_SUCCESS;}


struct type_24
{
    char buf[3];
};

typedef struct IMG_S
{
    char *pbmp;
    sint32 s32Width;
    sint32 s32Height;
};

static sint32 resize(IMG_S *pstApcImgDst, IMG_S *pstApcImgSrc)
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

    type_24 *pDstLine, *pSrcLine;
    char *pbmp_dst, *pbmp_src;
    unsigned long srcy_16 = 0, srcx_16 = 0;
    pDstLine = (type_24 *)pstApcImgDst->pbmp;
    for(unsigned long y = 0; y < pstApcImgDst->s32Height; y++)
    {
        pSrcLine = (type_24 *)pstApcImgSrc->pbmp + pstApcImgSrc->s32Width*(srcy_16>>16);
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

static sint32 callbackConfigUpdate(MSG_PACKET_S *pstMsgPkt, MSG_PACKET_S *pstRetPkt)
{

    sint32 s32Ret = 0;
    CFG_ALG_PARAM stAlgParam = {0};
    CFG_SYS_PARAM stSysParam = {0};
    ALARM_CFG_PARAM_S stAlarmParam = {0};
    print_level(SV_INFO, "recive: OP_EVENT_CFG_UPDATE\n");

    s32Ret = CONFIG_ReloadFile();
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "CONFIG_ReloadFile failed. [err=%#x]\n", s32Ret);
        return MSG_DEFAULT_FAIL;
    }

    s32Ret = CONFIG_GetAlgParam(&stAlgParam);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "CONFIG_GetAlgParam failed. [err=%#x]\n", s32Ret);
        return MSG_DEFAULT_FAIL;
    }

    print_level(SV_INFO, "stAlgParam.bAlgEnable:%d\n", stAlgParam.bAlgEnable);
    s32Ret = CONFIG_GetSystemParam(&stSysParam);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "CONFIG_GetAlgParam failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }

    print_level(SV_INFO, "stSysParam.enLang:%d\n", stSysParam.enLang);

    return SV_SUCCESS;
}

static sint32 callbackUpdateGpsData(MSG_PACKET_S *pstMsgPkt, MSG_PACKET_S *pstRetPkt)
{	
    sint32 s32Ret = 0;
	GPS_DATA_S stGpsData = {0};
	memcpy(&stGpsData, pstMsgPkt->pu8Data, sizeof(GPS_DATA_S));
	print_level(SV_INFO, "(%f, %f)\n", stGpsData.latitude, stGpsData.longitude);

    return SV_SUCCESS;
}

/******************************************************************************
 * 函数功能: 通过socket获取MediaBuf Fd
 * 输入参数: s32SocketFd --- socket fd
               s32Chn --- 通道号
 * 输出参数: ps32MediaBufFd --- MediaBuf Fd
 * 返回值  : SV_SUCCESS - 成功
             SV_FAILURE - 其它错误
 * 说明    : 无
 *****************************************************************************/
sint32 media_GetMediaBufFd(sint32 s32SocketFd, sint32 s32Chn, sint32 *ps32MediaBufFd)
{
    sint32 s32Ret = 0, i;
    SocketPacket stSocketPkt = {0};
    fd_set read_fds, write_fds;
    struct timeval timeout;

    if (s32SocketFd < 0 || s32Chn < 0 || s32Chn >= 4)
    {
        return ERR_ILLEGAL_PARAM;
    }

    if (NULL == ps32MediaBufFd)
    {
        return ERR_NULL_PTR;
    }
    
    FD_ZERO(&write_fds);
    FD_SET(s32SocketFd, &write_fds);
    timeout.tv_sec=3;
    timeout.tv_usec=0;
    s32Ret = select(s32SocketFd + 1, NULL, &write_fds, NULL, &timeout);
    if (s32Ret <= 0)
    {
        print_level(SV_WARN, "select write failed. [err=%d]\n", s32Ret);
        return SV_FAILURE;
    }

    stSocketPkt.header.startode = MSG_STARTCODE;
    stSocketPkt.header.opcode = SOCKET_OP_GET_FD;
    stSocketPkt.header.params = s32Chn;
    s32Ret = unsock_write(s32SocketFd, &stSocketPkt, sizeof(stSocketPkt));
    if (s32Ret < 0)
    {
        print_level(SV_ERROR, "unsock_write failed. [err=%#x]\n", s32Ret);
        return SV_FAILURE;
    }

    print_level(SV_INFO, "unsock_write successful. fd:%d\n", s32SocketFd);
    FD_ZERO(&read_fds);
    for (i = 0; i < 5; i++)
    {
        timeout.tv_sec=1;
        timeout.tv_usec=0;
        FD_SET(s32SocketFd, &read_fds);
        s32Ret = select(s32SocketFd + 1, &read_fds, NULL, NULL, &timeout);
        if (s32Ret <= 0)
        {
            print_level(SV_WARN, "select failed. [err=%d]\n", s32Ret);
            continue;
        }

        s32Ret = unsock_recvPacket(s32SocketFd, &stSocketPkt, sizeof(stSocketPkt), ps32MediaBufFd);
        if (s32Ret < 0 || ps32MediaBufFd < 0)
        {
            print_level(SV_ERROR, "unsock_write failed. [err=%#x, fd=%d]\n", s32Ret, ps32MediaBufFd);
            return SV_FAILURE;
        }
        else
        {
            break;
        }
    }
    
    if (i >= 5)
    {
        print_level(SV_ERROR, "wait for unsock_recvPacket timeout.\n");
        close(s32SocketFd);
        return SV_FAILURE;
    }
    
    return SV_SUCCESS;
}

sint32 sample_Alarm_Out_Enable()
{
    sint32 s32Ret;
    uint8 u8Value = 1;
#if (defined(BOARD_ADA32V2) || defined(BOARD_ADA32C4))
    s32Ret = BOARD_RK_SetGPIO(SP_ALARM_PIN_BAND, SP_ALARM_PIN_NUM , u8Value);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "BOARD_RK_SetGPIO fail![err=%#x]\n", s32Ret);
        return s32Ret;
    }
#endif

    return SV_SUCCESS;
}

sint32 sample_Alarm_Out_Reset()
{
    sint32 s32Ret;
    uint8 u8Value = 0;
#if (defined(BOARD_ADA32V2)|| defined(BOARD_ADA32C4))
    s32Ret = BOARD_RK_SetGPIO(SP_ALARM_PIN_BAND, SP_ALARM_PIN_NUM , u8Value);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "BOARD_RK_SetGPIO fail![err=%#x]\n", s32Ret);
        return s32Ret;
    }
#endif
    return SV_SUCCESS;
}

/* 播放报警音频 */
void sample_Audio_Play(void *pvArg)
{
    SP_ALARM_TYPE enMode = *((SP_ALARM_TYPE*)pvArg);
    sint32 s32Ret;
    static int mode = -1;
    static int bPlay = 0;
    ALARM_EVENT_S stAlarmEvent = {0};
    MSG_PACKET_S stMsgPkt = {0};
    struct timeval tvAlarm;
    struct timezone tz;

    s32Ret = pthread_detach(pthread_self());
    if(s32Ret!=0)
    {
        pthread_exit(NULL); // 显式退出
    }

    //print_level(SV_DEBUG, "bPlay:%d, mode:%d, new_mode:%d\n", bPlay, mode, enMode);
    if(enMode == SP_ALARM_TYPE_NULL)
    {
        return;
    }
    
    if(bPlay && mode >= enMode)
    {
        return;
    }


    memset(&stAlarmEvent, 0, sizeof(stAlarmEvent));
    gettimeofday(&tvAlarm, &tz);
    stAlarmEvent.enAlarmEvent = ALARM_EVENT_PD;
    stAlarmEvent.enAlarmType = ALARM_PD_ROI1;
    stAlarmEvent.enAlarmType = ALARM_PD_ROI1;
    stAlarmEvent.s32TimeStamp = (sint32)tvAlarm.tv_sec;
    stAlarmEvent.s32Chn = 0;

    memset(&stMsgPkt, 0, sizeof(stMsgPkt));
    stMsgPkt.stMsg.u16OpCode = OP_EVENT_ALG_ALARM;
    stMsgPkt.pu8Data = (uint8 *)&stAlarmEvent;
    stMsgPkt.u32Size = sizeof(stAlarmEvent);
    s32Ret = Msg_submitEvent(EP_CONTROL, OP_EVENT_ALG_ALARM, &stMsgPkt);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
    }

    bPlay = 1;
    mode = enMode;
    //print_level(SV_DEBUG, "mode:%d\n", mode);

    ALARM_PlayAlarm(0, mode, SV_FALSE);
    if(mode == enMode)
    {
        bPlay = 0;
    }
    
    return;
}

/* 触发线使能 */
void sample_Alarm_Out(void *pvArg)
{
    sint32 s32Ret, pwm, i = 0;
    sint32 s32AlarmOutInterval = 5000;
    static int bTrigger = 0;
    static struct timespec tvStart = {0, 0};
    static struct timespec tvNow = {0, 0};
    static SP_ALARM_TYPE lmode;   /* 实时保存相应的触发模式 */
    static SV_BOOL st_bStop;
    SP_ALARM_TYPE enMode = *((SP_ALARM_TYPE*)pvArg);
    
    s32Ret = pthread_detach(pthread_self());
    if(s32Ret!=0)
    {
        pthread_exit(NULL); // 显式退出
    }

    if(enMode == SP_ALARM_TYPE_NULL)
    {
        st_bStop = SV_TRUE;
        return;
    }

    st_bStop = SV_FALSE;
    lmode = enMode;   /* 更新报警模式 */
    if(bTrigger)
    {
        clock_gettime(CLOCK_MONOTONIC, &tvStart);   //重置时间
        pthread_exit(NULL);
        return;
    }

    bTrigger = 1;
    clock_gettime(CLOCK_MONOTONIC, &tvStart);
    tvNow = tvStart;

    sample_Alarm_Out_Enable();
    while(((1000*tvNow.tv_sec + tvNow.tv_nsec/1000000) - (1000*tvStart.tv_sec + tvStart.tv_nsec/1000000) < s32AlarmOutInterval) 
            || (st_bStop != SV_TRUE))
    {
        i = (i + 1) % 10;
        /* 设置PWM占用比      绿色占空比为30% 黄色占空比为60% 红色占空比为100%*/
        if(BOARD_IsCustomer(BOARD_C_ADA32V2_FTC_A))
        {
            if(lmode == SP_ALARM_TYPE_GREEN)        pwm = 3;
            else if(lmode == SP_ALARM_TYPE_YELLOW)  pwm = 6;
            else if(lmode == SP_ALARM_TYPE_RED)     pwm = 10;
            else                                    pwm = 10;
            if(i < pwm)
            {
                sample_Alarm_Out_Enable();
            }
            else
            {
                sample_Alarm_Out_Reset();
            }
        }
        sleep_ms(10);
        clock_gettime(CLOCK_MONOTONIC, &tvNow);
    }

    sample_Alarm_Out_Reset();
    bTrigger = 0;

    return;
}

sint32 sample_Alarm_Post(SP_ALARM_TYPE enAlarmType)
{
    sint32 s32Ret;
    pthread_t thread_sound, thread_trigger;
    static SP_ALARM_TYPE enMode = SP_ALARM_TYPE_NULL;

    enMode = enAlarmType;
    s32Ret = pthread_create(&thread_sound, NULL, sample_Audio_Play, &enMode);
    if (s32Ret < 0)
    {
        print_level(SV_ERROR, "pthread_create  failed. [err: %s]\n", strerror(errno));
        return SV_FAILURE;
    }

    s32Ret = pthread_create(&thread_trigger, NULL, sample_Alarm_Out, &enMode);
    if (s32Ret < 0)
    {
        print_level(SV_ERROR, "pthread_create  failed. [err: %s]\n", strerror(errno));
        return SV_FAILURE;
    }

    return SV_SUCCESS;
}


/* 算法执行线程 */
void * sample_alg_body(void *pvArg)
{
    sint32 s32Ret = 0, n, i;
    sint32 s32CurChn = 0;
    void *pvBuf = NULL;
    uint32 u32BufLen = MEDIA_IMAGE_WIDTH*MEDIA_IMAGE_HEIGHT*3;  // RGB888

    cv::Mat orig_img(MEDIA_IMAGE_WIDTH, MEDIA_IMAGE_HEIGHT, CV_8UC3);
    cv::Mat resize_img(640, 640, CV_8UC3);
    
    IMG_S img_ori = {NULL, MEDIA_IMAGE_WIDTH, MEDIA_IMAGE_HEIGHT};
    IMG_S img_dst = {resize_img.data, 640, 640};

    const float vis_threshold = 0.1;
    const float nms_threshold = 0.5;
    const float conf_threshold = 0.3;
    float thres[10] = {0};
    ALGPDS_INFO_S AlgResutl;
    
    thres[0] = conf_threshold;
    thres[1] = nms_threshold;
    thres[2] = vis_threshold;

    pvBuf = mmap(NULL, u32BufLen, PROT_READ, MAP_SHARED, g_as32MediaBufFd[0], 0);
    if (MAP_FAILED == pvBuf)
    {
        print_level(SV_ERROR, "mmap failed.\n");
        g_running = 0;
        return;
    }

    s32Ret = ALGPDS_init(thres, 3, g_modelPath);
    if (s32Ret < 0)
    {
        print_level(SV_ERROR, "ALGPDS_init failed.\n");
        munmap(pvBuf, u32BufLen);
        g_running = 0;
        return;
    }
    
    while (g_running)
    {
        /* P操作进入MediaBuffer临界区 */
        s32Ret = MS_P(s32CurChn);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "MS_P failed. [err=%d]\n", s32Ret);
            g_running = 0;
            return;
        }

        img_ori.pbmp = pvBuf;
        resize(&img_dst, &img_ori);

        s32Ret = ALGPDS_forward(img_dst.pbmp);
        if (s32Ret < 0)
        {
            print_level(SV_ERROR, "ALGPDS_forward failed.\n");
            MS_V(s32CurChn);
            continue;
        }

        s32Ret = ALGPDS_get_result(&AlgResutl);
        if (s32Ret < 0)
        {
            print_level(SV_ERROR, "ALGPDS_get_result failed.\n");
            MS_V(s32CurChn);
            continue;
        }

        sint32 s32CurChn = 0;
        uint16 u16mask;
        MSG_PACKET_S stMsgPkt = {0};
        MEDIA_GUI_DRAW_S stMediaGuiDraw = {0};
        MEDIA_GUI_PERSON_S stGuiRect = {0};
        MEDIA_GUI_NULL_S stGuiNull;

        memset(&stMediaGuiDraw, 0x00, sizeof(stMediaGuiDraw));
        memset(&stGuiRect, 0x00, sizeof(stGuiRect));
        
        /* 清空原来的画板 */
        u16mask = MEDIA_GUI_GET_MASK(s32CurChn, 0, MEDIA_GUI_OP_CLEAR);
        s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiNull);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
        }
        
        stGuiRect.u32PersonNum = AlgResutl.num;
        for (i = 0; i < AlgResutl.num; i++)
        {
            ALGPDS_RECT_S algBox = AlgResutl.Rect_t[i];
            print_level(SV_INFO, "[%d], (%f, %f), (%f, %f), confidence:%f\n",i, algBox.x1, algBox.y1, algBox.x2, algBox.y2, algBox.confidence);
            stGuiRect.astPersonsRect[i].x1 = algBox.x1;
            stGuiRect.astPersonsRect[i].y1 = algBox.y1;
            stGuiRect.astPersonsRect[i].x2 = algBox.x2;
            stGuiRect.astPersonsRect[i].y2 = algBox.y2;
            stGuiRect.astPersonsRect[i].color = GUI_COLOR_GREEN;
            stGuiRect.astPersonsRect[i].stick = 3;
        }

        /* V操作退出MediaBuffer临界区 */
        MS_V(s32CurChn);
        if (AlgResutl.num == 0)
        {
            sample_Alarm_Post(SP_ALARM_TYPE_NULL);
        }
        else
        {
            sample_Alarm_Post(SP_ALARM_TYPE_YELLOW);
        }

        u16mask = MEDIA_GUI_GET_MASK(s32CurChn, 0, MEDIA_GUI_OP_PERSON_RECT);
        s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiRect);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
        }

        /* 发送绘图消息 */
        memset(&stMsgPkt, 0, sizeof(stMsgPkt));
        stMsgPkt.pu8Data = (uint8*)&stMediaGuiDraw;
        stMsgPkt.u32Size = sizeof(stMediaGuiDraw);
        s32Ret = Msg_submitEvent(EP_CONTROL, OP_EVENT_MEDIA_GUI, &stMsgPkt);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
        }
    }

    ALGPDS_release();
    munmap(pvBuf, u32BufLen);

    return NULL;
}

/* 中断退出 */
static void exit_handle(int signalnum)
{
    sint32 s32Ret = 0;
    
    printf("catch signalnum %d!\n", signalnum);
    g_running = 0;
    sleep_ms(200);
    exit(EXIT_FAILURE);
}

int main(int argc, char **argv)
{
    sint32 s32Ret = 0, n, i;
    sint32 s32ChnNum;
    sint32 s32Tmp = 0;
    char *pszConfigFile = CONFIG_XML;
    char *pszConfigBak1 = CONFIG_BAK1;
    char *pszConfigBak2 = CONFIG_BAK2;
    char *pszConfigDefault = CONFIG_DEFAULT;
    CFG_ALG_PARAM stAlgParam = {0};
    CFG_SYS_PARAM stSysParam = {0};
    ALARM_CFG_PARAM_S stAlarmParam = {0};
    SocketPacket stSocketPkt = {0};
    fd_set read_fds, write_fds;
    struct timeval timeout;
    int as32MediaBufFd[4] = {-1, -1, -1, -1};
    pthread_t thread;
    void *pvRetval = NULL;
    sint32 chChoice;

    s32ChnNum = 1;
    if (argc != 2){
        printf("Usage: %s <rknn model>\n", argv[0]);
        return -1;
    }

    g_modelPath = argv[1];
    if (0 != access(g_modelPath, F_OK))
    {
        print_level(SV_ERROR, "not found model file: %s\n", g_modelPath);
        return -1;
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

    s32Ret = BOARD_Init();
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "BOARD_Init failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }

    s32Ret = CONFIG_Init(pszConfigFile, pszConfigBak1, pszConfigBak2, pszConfigDefault);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "CONFIG_Init failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }

    s32Ret = CONFIG_GetAlgParam(&stAlgParam);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "CONFIG_GetAlgParam failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }

    for (i = 0; i < 5; i++)
    {
        g_s32SocketFd = cli_connect(CS_PATH_CAM_STREAM, 'c');
        if (g_s32SocketFd > 0)
        {
            break;
        }
        print_level(SV_WARN, "cli_connect %s failed.\n", CS_PATH_CAM_STREAM);
        sleep_ms(1000);
    }
    if (i >= 5)
    {
        print_level(SV_ERROR, "wait for cli_connect %s timeout.\n", CS_PATH_CAM_STREAM);
        return -1;
    }

    print_level(SV_INFO, "cli_connect successful. fd:%d\n", g_s32SocketFd);
    for(n = 0; n < s32ChnNum; n++)
    {
        s32Ret = media_GetMediaBufFd(g_s32SocketFd, n, &g_as32MediaBufFd[n]);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "alg_GetMediaBufFd failed.[err=%#x]\n", s32Ret);
            close(g_s32SocketFd);
            return -1;
        }
     
        print_level(SV_INFO, "got media buffer fd:%d\n", g_as32MediaBufFd[n]);
    }

    /* 初步化MediaBuffer临界区 */
    s32Ret = MS_Init();
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "MS_Init failed. [err=%#x]\n", s32Ret);
        return ERR_SYS_NOTREADY;
    }

    s32Ret = CONFIG_GetSystemParam(&stSysParam);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "CONFIG_GetAlgParam failed. [err=%#x]\n", s32Ret);
        return s32Ret;
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
        return s32Ret;
    }

    stAlarmParam.enLanguage = stSysParam.enLang;
    stAlarmParam.aenPdsAudioType[0] = stAlgParam.stAlgCh1.stPdsParam.enAudioType;
    stAlarmParam.aenPdsAudioType[1] = stAlgParam.stAlgCh2.stPdsParam.enAudioType;
    stAlarmParam.aenPdsAudioType[2] = stAlgParam.stAlgCh3.stPdsParam.enAudioType;
    stAlarmParam.enAdasAudioType = stAlgParam.stAlgCh1.stAdasParam.enAudioType;
    stAlarmParam.enDmsAudioType = stAlgParam.stAlgCh2.stDmsParam.enAudioType;
    stAlarmParam.s32AudioVolume = stAlgParam.s32AudioVolume;
    s32Ret = ALARM_Init(&stAlarmParam);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "ALARM_Init failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }

    s32Ret = Msg_registerOpCallback(EP_ALG, OP_EVENT_CFG_UPDATE, callbackConfigUpdate);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_registerOpCallback_ThreadExec failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }

    s32Ret = Msg_registerOpCallback(EP_ALG, OP_EVENT_GPS_DATA, callbackUpdateGpsData);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_registerOpCallback failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }

    g_running = 1;
    s32Ret = pthread_create(&thread, NULL, sample_alg_body, NULL);
    while (g_running)
    {
        sleep(1);

        FD_ZERO(&read_fds);
        timeout.tv_sec=5;
        timeout.tv_usec=0;
        FD_SET(g_s32SocketFd, &read_fds);
        s32Ret = select(g_s32SocketFd + 1, &read_fds, NULL, NULL, &timeout);
        if (s32Ret <= 0)
        {
            //print_level(SV_WARN, "select failed. [err=%d]\n", s32Ret);
            continue;
        }

        s32Ret = unsock_recvPacket(g_s32SocketFd, &stSocketPkt, sizeof(stSocketPkt), &s32Tmp);
        if (s32Ret == 0)
        {
            print_level(SV_INFO, "media fd update! \n");
            for (n = 0; n < s32ChnNum; n++)
            {

                s32Ret = media_GetMediaBufFd(g_s32SocketFd, n, &as32MediaBufFd[n]);
                if (SV_SUCCESS != s32Ret)
                {
                    print_level(SV_ERROR, "alg_GetMediaBufFd failed.[err=%#x]\n", s32Ret);
                }

                if (g_as32MediaBufFd[n] != as32MediaBufFd[n])
                {
                    print_level(SV_INFO, "ch%d fd: %d -> %d\n", n, g_as32MediaBufFd[n], as32MediaBufFd[n]);
                }
            }
        }
    }

    MS_Fini();

    return 0;
}

