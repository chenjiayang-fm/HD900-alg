/******************************************************************************
Copyright (C) 2021-2023 广州敏视数码科技有限公司版权所有.

文件名：dmm.c

日期: 2021-08-03

文件功能描述: 定义DMS算法功能接口

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
#include "DSM.h"
#include "stateAnalyzer.h"
#include "FRAnalyzer.h"
#include "op.h"
#include "msg.h"
#include "cJSON.h"
#include "board.h"
#include "utils.h"
#include "alarm.h"
#include "dmm.h"
#include "media.h"
#include "media_sem.h"
#include "alg.h"
//#include "YOLOpostprocess.h"
#include "gpio_pwm.h"

#if defined(BOARD_DMS31V2)
/*****************************
* old alarm out --> GPIO1_B7 
* new alarm out --> GPIO1_C5
******************************/
#define DMM_ALARM_PIN_BAND 1
#define DMM_ALARM_OLD_PIN  15
#define DMM_ALARM_NEW_PIN  21

/* FHD685壳体DMS的alarmout引脚和A32的一致 */
#define DMM_ALARM_FHD685_PIN_BAND 3
#define DMM_ALARM_FHD685_PIN_NUM  20
#else
/*****************************
* old alarm out --> GPIO2_A6
* new alarm out --> GPIO2_A7
******************************/
#define DMM_ALARM_PIN_BAND 2
#define DMM_ALARM_OLD_PIN  6
#define DMM_ALARM_NEW_PIN  7
#endif

/* LED灯控制口 -- SPI1_CSN0_M1 -- GPIO1_C7 */
#define DMM_ALARM_LED_BAND 1
#define DMM_ALARM_LED_PIN  23

#define DMM_SAMPLE_NUM      55              /* 校准采样点数 */
#define DMM_MAX_ANGLE       35              /* 允许中心角正向移植值 */
#define DMM_MIN_ANGLE       -35             /* 允许中心角负向移植值 */
#define DMM_IMAGE_WIDTH     1280            /* 算法图像帧宽度 */
#define DMM_IMAGE_HEIGHT    720             /* 算法图像帧高度 */
#define DMM_NO_FACE_TIME_MS 10000           /* 连续无人脸时间 */ 

/* 模块控制信息 */
typedef struct tagDmmInfo_S
{    
    SV_BOOL         bKeyAuth;               /* 密钥是否有效 */
    sint32          s32MediaBufFd;          /* 媒体通道Media Buffer的文件描述符 */
    sint32          s32Chn;                 /* DMS算法媒体通道号 */
    sint32          s32PwmFd;               /* alarmout模拟输出pwm波驱动设备fd */
    sint32          s32DmsCenterAngle;      /* DMS设备安装的中心角(由算法程序写入) */
    DMM_RUN_E       enRunStat;              /* 算法当前运行状态 */
    DMM_RES_E       enRunResult;            /* 上次次执行(标定,注册,登陆)的结果 */
    USER_INFO_S     stUserInfo;             /* 最近一次登陆或添加的用户信息 */
    CFG_ALG_PARAM   stCfgParam;             /* 算法配置参数 */
    CDSM            *pcsDmmDetector;        /* DMS算法检测器 */
    CStateAnalyzer  *pcsAlgStatus;          /* 算法检测状态 */
    CFRAnalyzer     *pcsFRAnalyzer;         /* 人脸识别分析器 */
    char            szUserDir[64];          /* 当前注册用户目录路径 */
    uint32          u32TidAlg;              /* 算法线程ID */
    SV_BOOL         bRunning;               /* 线程是否正在运行 */
    pthread_mutex_t mutexRunStat;           /* 算法运行状态互斥锁 */
    SV_BOOL         bFirstFace;             /* 是否为上电后第一次出现的人脸 */
    SV_BOOL         bPlayLoginAudio;        /* 是否播放登录音频 */
    DUMP_DMM_S      stDumpDmmInfo;          /* dump信息 */
} DMM_INFO_S;

DMM_INFO_S m_stDmmInfo = {0};           /* 模块控制信息 */


typedef struct tagDmmConf_S
{
    sint32  s32WorkSpeed;               /* 算法工作速度 */
    sint32  s32DectectInterval;         /* 算法检测间隔 */
    bool    bIntervalChanged;           /* 算法检测间隔是否被修改 */
    SV_BOOL bDmsAlarmOut;               /* 是否使能报警输出 */
    SV_BOOL bDmsAudioEnable;            /* 是否使能报警声音 */
}DMM_CONF_S;

DMM_CONF_S m_stDmmConf[ALARM_DMS_BUFF] = {0};


#define DMM_PALY_LIST_NUM   5           /* 播放列表数量 */
typedef struct tagDmmPlayInfo_S
{
    SV_BOOL bPlay;
    sint32 s32PlayingIndex;
    sint64 tLastPlay;
    ALARM_TYPE_E enAlarmTypePlaying;
    ALARM_TYPE_E enAlarmTypeLast;
    ALARM_TYPE_E enAlarmTypeList[DMM_PALY_LIST_NUM];
}DMM_PLAY_INFO_S;

static inline void cutLineBreak(char *pszStr)
{
    char *pcTmp = NULL;
    
    pcTmp = strchr(pszStr, '\r');
    if(NULL == pcTmp)
        pcTmp = strchr(pszStr, '\n');
    if(NULL != pcTmp)
        *pcTmp = '\0';
}

/* MS_V回调操作函数
  说明: 算法ForwardGroupDSM/ForwardGroupFR 内部将图像数据拷贝完后的回调
       用于快速释放锁
 */
inline void dmm_MS_V()
{
    MS_V(0);    // 退出临界区
    ALG_Calculate_unLock();
}

sint32 dmm_Pwm_Init(sint32 *s32PwmFd)
{
    sint32 s32Ret = 0;
    sint32 s32Fd = -1;
    char szCmd[128] = {0};
    PWM_DUTY_CYCLE_S stPwmDutyCycle = {0};

    s32Ret = BOARD_RK_SetGPIO(DMM_ALARM_PIN_BAND, DMM_ALARM_NEW_PIN, 0);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "BOARD_RK_SetGPIO fail![err=%#x]\n", s32Ret);
        return s32Ret;
    }

    sprintf(szCmd, "rmmod gpio_pwm");
    s32Ret = SAFE_System(szCmd, NORMAL_WAIT_TIME);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_WARN, "exec cmd: %s failed.\n", szCmd);
    }

    stPwmDutyCycle.ton = 2500 * m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsPwmDutyCycle;	// 客户要求PWM频率为4kHz，也就是周期为250000ns
    sprintf(szCmd, "insmod /root/ko/extdrv/gpio_pwm.ko ton=%d", stPwmDutyCycle.ton);
    s32Ret = SAFE_System(szCmd, NORMAL_WAIT_TIME);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_WARN, "exec cmd: %s failed.\n", szCmd);
    }

    s32Fd = open(PWM_DEVICE_PATH, O_RDWR);
    if (s32Fd < 0)
    {
        print_level(SV_ERROR, "can't open %s!\n", PWM_DEVICE_PATH);
        return SV_FAILURE;
    }

    *s32PwmFd = s32Fd;
    return SV_SUCCESS;
}

sint32 dmm_Alarm_Out_Enable()
{
    sint32 s32Ret, i = 0;
    uint8 u8Value;
#if (defined(BOARD_DMS31V2) || defined(BOARD_ADA47V1))
    if (BOARD_IsCustomer(BOARD_C_DMS31V2_202032) && BOARD_DMS31V2_V1 == BOARD_GetVersion())
    {
        for (i = 0; i < 2; i++)
        {
            s32Ret = ioctl(m_stDmmInfo.s32PwmFd, PWM_START_WORKING);
            if (SV_SUCCESS != s32Ret)
            {
                print_level(SV_WARN, "pwm_ioctl %d error[%d], try again...\n", _IOC_NR(PWM_START_WORKING), s32Ret);
                continue;
            }
            else
            {
                break;
            }
        }
        if (i >= 2)
        {            
            print_level(SV_ERROR, "pwm_ioctl %d error[%d]\n", _IOC_NR(PWM_START_WORKING), s32Ret);
            return s32Ret;
        }
    }
    else
    {
        u8Value = 1;
        if (BOARD_DMS31V2_V1 == BOARD_GetVersion())
        {
            s32Ret = BOARD_RK_SetGPIO(DMM_ALARM_PIN_BAND, DMM_ALARM_OLD_PIN, u8Value);
            if(s32Ret != SV_SUCCESS)
            {
                print_level(SV_ERROR, "BOARD_RK_SetGPIO fail![err=%#x]\n", s32Ret);
                return s32Ret;
            }

            s32Ret = BOARD_RK_SetGPIO(DMM_ALARM_PIN_BAND, DMM_ALARM_NEW_PIN, u8Value);
            if(s32Ret != SV_SUCCESS)
            {
                print_level(SV_ERROR, "BOARD_RK_SetGPIO fail![err=%#x]\n", s32Ret);
                return s32Ret;
            }
        }
        else if (BOARD_DMS31V2_V2 == BOARD_GetVersion())
        {
            s32Ret = BOARD_RK_SetGPIO(DMM_ALARM_FHD685_PIN_BAND, DMM_ALARM_FHD685_PIN_NUM, u8Value);
            if(s32Ret != SV_SUCCESS)
            {
                print_level(SV_ERROR, "BOARD_RK_SetGPIO fail![err=%#x]\n", s32Ret);
                return s32Ret;
            }
        }
    }
#endif

    return SV_SUCCESS;
}

sint32 dmm_Alarm_Out_Reset()
{
    sint32 s32Ret, i = 0;
    uint8 u8Value;
#if (defined(BOARD_DMS31V2) || defined(BOARD_ADA47V1))
    if (BOARD_IsCustomer(BOARD_C_DMS31V2_202032) && BOARD_DMS31V2_V1 == BOARD_GetVersion())
    {
        for (i = 0; i < 2; i++)
        {
            s32Ret = ioctl(m_stDmmInfo.s32PwmFd, PWM_STOP_WORKING);
            if (SV_SUCCESS != s32Ret)
            {
                print_level(SV_WARN, "pwm_ioctl %d error[%d], try again...\n", _IOC_NR(PWM_STOP_WORKING), s32Ret);
                continue;
            }
            else
            {
                break;
            }
        }
        if (i >= 2)
        {            
            print_level(SV_ERROR, "pwm_ioctl %d error[%d]\n", _IOC_NR(PWM_STOP_WORKING), s32Ret);
            return s32Ret;
        }
    }
    else
    {
        u8Value = 0;
        if (BOARD_DMS31V2_V1 == BOARD_GetVersion())
        {
            s32Ret = BOARD_RK_SetGPIO(DMM_ALARM_PIN_BAND, DMM_ALARM_OLD_PIN, u8Value);
            if(s32Ret != SV_SUCCESS)
            {
                print_level(SV_ERROR, "BOARD_RK_SetGPIO fail![err=%#x]\n", s32Ret);
                return s32Ret;
            }

            s32Ret = BOARD_RK_SetGPIO(DMM_ALARM_PIN_BAND, DMM_ALARM_NEW_PIN, u8Value);
            if(s32Ret != SV_SUCCESS)
            {
                print_level(SV_ERROR, "BOARD_RK_SetGPIO fail![err=%#x]\n", s32Ret);
                return s32Ret;
            }
        }
        else if (BOARD_DMS31V2_V2 == BOARD_GetVersion())
        {
            s32Ret = BOARD_RK_SetGPIO(DMM_ALARM_FHD685_PIN_BAND, DMM_ALARM_FHD685_PIN_NUM, u8Value);
            if(s32Ret != SV_SUCCESS)
            {
                print_level(SV_ERROR, "BOARD_RK_SetGPIO fail![err=%#x]\n", s32Ret);
                return s32Ret;
            }
        }
    }
#endif

    return SV_SUCCESS;
}

/* 触发线使能 */
void dmm_Alarm_Out(void *pvArg)
{
    sint32 s32Ret;
    sint32 s32Cnt = 0;
    static uint8 u8PinValue = 0;
    static SV_BOOL bTrigger = SV_FALSE;
    static struct timespec tvStart = {0, 0};
    static struct timespec tvNow = {0, 0};
    ALARM_TYPE_E enAlarmType = *((ALARM_TYPE_E *)pvArg);

    if (SV_FALSE == m_stDmmConf[enAlarmType].bDmsAlarmOut)
    {
        return;
    }

    if (bTrigger)
    {
        clock_gettime(CLOCK_MONOTONIC, &tvStart);   //重置时间
        pthread_exit(NULL);
        return;
    }

    bTrigger = SV_TRUE;
    clock_gettime(CLOCK_MONOTONIC, &tvStart);
    tvNow = tvStart;

    dmm_Alarm_Out_Enable();
    while ((1000*tvNow.tv_sec + tvNow.tv_nsec/1000000) - (1000*tvStart.tv_sec + tvStart.tv_nsec/1000000) \
          < m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsAlarmOutInterval)
    {
        sleep_ms(10);
        clock_gettime(CLOCK_MONOTONIC, &tvNow);
        if (BOARD_IsCustomer(BOARD_C_DMS31V2_SEB))
        {
            if (s32Cnt++ % 25 == 0)
            {
                u8PinValue = !u8PinValue;
                BOARD_RK_SetGPIO(DMM_ALARM_LED_BAND, DMM_ALARM_LED_PIN, u8PinValue);
            }
        }
    }
    dmm_Alarm_Out_Reset();
    bTrigger = SV_FALSE;

    if (BOARD_IsCustomer(BOARD_C_DMS31V2_SEB))
    {
        s32Cnt = 0;
        u8PinValue = 0;
        BOARD_RK_SetGPIO(DMM_ALARM_LED_BAND, DMM_ALARM_LED_PIN, u8PinValue);
    }

    return;
}

sint32 dmm_Alarm_Post(ALARM_TYPE_E enAlarmType)
{
    sint32 s32Ret;
    pthread_t thread_trigger;
    static ALARM_TYPE_E enAlarmTypePost = ALARM_NOTHING;
    enAlarmTypePost = enAlarmType;
    
    pthread_attr_t 	attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);       //设置为分离线程
    s32Ret = pthread_create(&thread_trigger, &attr, dmm_Alarm_Out, (void *)&enAlarmTypePost);
    if (s32Ret < 0)
    {
        print_level(SV_ERROR, "pthread_create  failed. [err: %s]\n", strerror(errno));
        return SV_FAILURE;
    }
    pthread_attr_destroy(&attr);

    return SV_SUCCESS;
}

sint64 dmm_GetTimeTickMs()
{
	struct timespec time = {0, 0};
	
	clock_gettime(CLOCK_MONOTONIC, &time);
	return ((sint64)time.tv_sec) * 1000 + time.tv_nsec/1000000;
}

/* 播放音频 */
void dmm_Alarm_Play(void *pvArg)
{
    sint32 s32Ret, i = 0;
    sint32 s32PlayListSize = 5;    
    sint32 s32TimeoutCnt = 60;
    sint64 tNow = dmm_GetTimeTickMs();
    ALARM_TYPE_E enAlarmType = *((ALARM_TYPE_E *)pvArg);    
    static DMM_PLAY_INFO_S stDmmPlayInfo = {0};
    
    s32Ret = pthread_detach(pthread_self());
    if (s32Ret != 0)
    {
        pthread_exit(NULL); // 显式退出
    }

    sint64 s64TimeInterval = llabs(tNow - stDmmPlayInfo.tLastPlay);
    if (enAlarmType == stDmmPlayInfo.enAlarmTypeLast && s64TimeInterval < 1500)
    {
        for (i = 0; i < DMM_PALY_LIST_NUM; i++)
        {
            if (ALARM_NOTHING != stDmmPlayInfo.enAlarmTypeList[i] && enAlarmType != stDmmPlayInfo.enAlarmTypeList[i])
            {
                if (stDmmPlayInfo.bPlay && stDmmPlayInfo.s32PlayingIndex == i)
                {
                    print_level(SV_INFO, "list[%d] has another alarm%d is playing, so continue to add\n", i, stDmmPlayInfo.enAlarmTypeList[i]);
                    
                }
                else
                {
                    print_level(SV_WARN, "continuous alarm%d, list[%d] is playing alarm%d, interval time: %lld\n", enAlarmType, i, stDmmPlayInfo.enAlarmTypeList[i], s64TimeInterval);
                    return;
                }
            }
        }
    }

    if (enAlarmType == stDmmPlayInfo.enAlarmTypePlaying)
    {
        print_level(SV_WARN, "aplay is playing the same alarm%d, return\n", enAlarmType);
        return;
    }

    for (i = 0; i < DMM_PALY_LIST_NUM; i++)
    {
        if (ALARM_NOTHING == stDmmPlayInfo.enAlarmTypeList[i])
        {
            //print_level(SV_INFO, "add alarm%d to play list[%d]\n", enAlarmType, i);
            stDmmPlayInfo.enAlarmTypeList[i] = enAlarmType;
            break;
        }
    }

    if (i >= DMM_PALY_LIST_NUM)
    {
        print_level(SV_ERROR, "playlist is full, skip to play alarm%d\n", enAlarmType);
        return;
    }

    while (s32TimeoutCnt > 0)
    {
        if (stDmmPlayInfo.bPlay)
        {
            print_level(SV_WARN, "alarm%d is waiting for alarm%d!!!\n", enAlarmType, stDmmPlayInfo.enAlarmTypePlaying);
            s32TimeoutCnt--;
            sleep_ms(50);
            continue;
        }
        else
        {
            break;
        }
    }

    if (0 == s32TimeoutCnt)
    {
        print_level(SV_WARN, "alarm%d timeout wait for aplay, return\n", enAlarmType);
        goto exit;
    }

    if (0 != access(ALARM_WELCOME_FILE, F_OK))
    {
        print_level(SV_WARN, "aplay is still playing welcome message, wait a minute!!!\n");
        goto exit;
    }
    
    ALARM_EnableSpk(SV_TRUE);

    stDmmPlayInfo.bPlay = SV_TRUE;
    stDmmPlayInfo.s32PlayingIndex = i;
    stDmmPlayInfo.enAlarmTypePlaying = enAlarmType;
    
    ALARM_PlayAudio(enAlarmType);
    
    stDmmPlayInfo.enAlarmTypePlaying = ALARM_NOTHING;
    stDmmPlayInfo.s32PlayingIndex = -1;
    stDmmPlayInfo.enAlarmTypeLast = enAlarmType;
    stDmmPlayInfo.tLastPlay = dmm_GetTimeTickMs();
    stDmmPlayInfo.bPlay = SV_FALSE;
    //print_level(SV_INFO, "playlist[%d] play alarm%d finish\n", i, enAlarmType);

    stDmmPlayInfo.enAlarmTypeList[i] = ALARM_NOTHING;
    for (i = 0; i < DMM_PALY_LIST_NUM; i++)
    {
        if (ALARM_NOTHING != stDmmPlayInfo.enAlarmTypeList[i])
        {
            break;
        }
    }
    if (i >= DMM_PALY_LIST_NUM)
    {
        sleep_ms(200);
        ALARM_EnableSpk(SV_FALSE);
    }
    return;

exit:
    stDmmPlayInfo.enAlarmTypeList[i] = ALARM_NOTHING;
    return;
}

sint32 dmm_Alarm_PlayAudio(ALARM_TYPE_E enAlarmType)
{
    sint32 s32Ret;
    pthread_t thread_trigger;
    static ALARM_TYPE_E enAlarmTypePlay = ALARM_NOTHING;

    enAlarmTypePlay = enAlarmType;
    s32Ret = pthread_create(&thread_trigger, NULL, dmm_Alarm_Play, (void *)&enAlarmTypePlay);
    if (s32Ret < 0)
    {
        print_level(SV_ERROR, "pthread_create  failed. [err: %s]\n", strerror(errno));
        return SV_FAILURE;
    }

    return SV_SUCCESS;
}

sint32 dmm_DumpInfo(DUMP_DMM_S *pstDumpDmmInfo)
{
    sint32 s32Ret = -1, s32Fd = -1, i = 0;
    char szBuf[DUMP_BUF_SIZE] = {0};
    cJSON *pstJson = NULL;
    cJSON *pstUsrId = NULL;
    cJSON *pstFrUserName = NULL;
    cJSON *pstHeadPos = NULL;
    cJSON *pstGaze = NULL;
    cJSON *pstEyeScore = NULL;
    cJSON *pstGlassScore = NULL;
    cJSON *pstTmp = NULL;
    uint8  u8EyeScore[2][24] = {"openEye", "closeEye"};
    uint8  u8GlassScore[2][24] = {"normalglass", "sunglass"};
    uint8  u8HeadPose[3][24] = {"pitch", "yaw", "roll"};
    uint8  u8Gaze[2][24] = {"yaw", "pitch"};
    uint8  u8UsrId[12] = {0};

    if (NULL == pstDumpDmmInfo)
    {
        return SV_FAILURE;
    }

    pstJson = cJSON_CreateObject();
    if (NULL == pstJson)
    {
        print_level(SV_ERROR, "cJSON_CreateObject failed.\n");
        goto error_exit;
    }

    pstTmp = cJSON_CreateNumber(pstDumpDmmInfo->enAlarmType);
    if (NULL == pstTmp)
    {
        print_level(SV_ERROR, "cJSON_CreateNumber %d failed.\n", pstDumpDmmInfo->enAlarmType);
        goto error_exit;
    }
    cJSON_AddItemToObject(pstJson, "alarmType", pstTmp);

    pstTmp = cJSON_CreateBool(pstDumpDmmInfo->bImageMirror);
    if (NULL == pstTmp)
    {
        print_level(SV_ERROR, "cJSON_CreateBool %d failed.\n", pstDumpDmmInfo->bImageMirror);
        goto error_exit;
    }
    cJSON_AddItemToObject(pstJson, "bImageMirror", pstTmp);

    pstTmp = cJSON_CreateBool(pstDumpDmmInfo->bDetectFace);
    if (NULL == pstTmp)
    {
        print_level(SV_ERROR, "cJSON_CreateBool %d failed.\n", pstDumpDmmInfo->bDetectFace);
        goto error_exit;
    }
    cJSON_AddItemToObject(pstJson, "bDetectFace", pstTmp);

    pstTmp = cJSON_CreateBool(pstDumpDmmInfo->bYawn);
    if (NULL == pstTmp)
    {
        print_level(SV_ERROR, "cJSON_CreateBool %d failed.\n", pstDumpDmmInfo->bYawn);
        goto error_exit;
    }
    cJSON_AddItemToObject(pstJson, "bYawn", pstTmp);

    pstTmp = cJSON_CreateBool(pstDumpDmmInfo->bNoMask);
    if (NULL == pstTmp)
    {
        print_level(SV_ERROR, "cJSON_CreateBool %d failed.\n", pstDumpDmmInfo->bNoMask);
        goto error_exit;
    }
    cJSON_AddItemToObject(pstJson, "bNoMask", pstTmp);

    pstTmp = cJSON_CreateBool(pstDumpDmmInfo->bShelter);
    if (NULL == pstTmp)
    {
        print_level(SV_ERROR, "cJSON_CreateBool %d failed.\n", pstDumpDmmInfo->bShelter);
        goto error_exit;
    }
    cJSON_AddItemToObject(pstJson, "bShelter", pstTmp);

    pstTmp = cJSON_CreateString(pstDumpDmmInfo->u8SmokeScore);
    if (NULL == pstTmp)
    {
        print_level(SV_ERROR, "cJSON_CreateString %s failed.\n", pstDumpDmmInfo->u8SmokeScore);
        goto error_exit;
    }
    cJSON_AddItemToObject(pstJson, "smokeScore", pstTmp);

    pstTmp = cJSON_CreateString(pstDumpDmmInfo->u8PhoneScore);
    if (NULL == pstTmp)
    {
        print_level(SV_ERROR, "cJSON_CreateString %s failed.\n", pstDumpDmmInfo->u8PhoneScore);
        goto error_exit;
    }
    cJSON_AddItemToObject(pstJson, "phoneScore", pstTmp);

    pstTmp = cJSON_CreateString(pstDumpDmmInfo->u8DrinkEatScore);
    if (NULL == pstTmp)
    {
        print_level(SV_ERROR, "cJSON_CreateString %s failed.\n", pstDumpDmmInfo->u8DrinkEatScore);
        goto error_exit;
    }
    cJSON_AddItemToObject(pstJson, "drinkEatScore", pstTmp);

    pstTmp = cJSON_CreateString(pstDumpDmmInfo->u8SeatbeltScore);
    if (NULL == pstTmp)
    {
        print_level(SV_ERROR, "cJSON_CreateString %s failed.\n", pstDumpDmmInfo->u8SeatbeltScore);
        goto error_exit;
    }
    cJSON_AddItemToObject(pstJson, "seatbeltScore", pstTmp);

    pstTmp = cJSON_CreateString(pstDumpDmmInfo->u8HelmetScore);
    if (NULL == pstTmp)
    {
        print_level(SV_ERROR, "cJSON_CreateString %s failed.\n", pstDumpDmmInfo->u8HelmetScore);
        goto error_exit;
    }
    cJSON_AddItemToObject(pstJson, "helmetScore", pstTmp);

    pstEyeScore = cJSON_CreateObject();
    if (NULL == pstEyeScore)
    {
        print_level(SV_ERROR, "cJSON_CreateObject failed.\n");
        goto error_exit;
    }
    cJSON_AddItemToObject(pstJson, "eyeScore", pstEyeScore);
    for (i = 0; i < 2; i++)
    {
        pstTmp = cJSON_CreateString(pstDumpDmmInfo->u8EyeScore[i]);
        if (NULL == pstTmp)
        {
            print_level(SV_ERROR, "cJSON_CreateString %s failed.\n", pstDumpDmmInfo->u8EyeScore[i]);
            goto error_exit;
        }
        cJSON_AddItemToObject(pstEyeScore, u8EyeScore[i], pstTmp);
    }

    pstGlassScore = cJSON_CreateObject();
    if (NULL == pstGlassScore)
    {
        print_level(SV_ERROR, "cJSON_CreateObject failed.\n");
        goto error_exit;
    }
    cJSON_AddItemToObject(pstJson, "glassScore", pstGlassScore);
    for (i = 0; i < 2; i++)
    {
        pstTmp = cJSON_CreateString(pstDumpDmmInfo->u8GlassScore[i]);
        if (NULL == pstTmp)
        {
            print_level(SV_ERROR, "cJSON_CreateString %s failed.\n", pstDumpDmmInfo->u8GlassScore[i]);
            goto error_exit;
        }
        cJSON_AddItemToObject(pstGlassScore, u8GlassScore[i], pstTmp);
    }

    pstHeadPos = cJSON_CreateObject();
    if (NULL == pstHeadPos)
    {
        print_level(SV_ERROR, "cJSON_CreateObject failed.\n");
        goto error_exit;
    }
    cJSON_AddItemToObject(pstJson, "headPos", pstHeadPos);
    for (i = 0; i < 3; i++)
    {
        pstTmp = cJSON_CreateString(pstDumpDmmInfo->u8HeadPose[i]);
        if (NULL == pstTmp)
        {
            print_level(SV_ERROR, "cJSON_CreateString %s failed.\n", pstDumpDmmInfo->u8HeadPose[i]);
            goto error_exit;
        }
        cJSON_AddItemToObject(pstHeadPos, u8HeadPose[i], pstTmp);
    }
    
    pstGaze = cJSON_CreateObject();
    if (NULL == pstGaze)
    {
        print_level(SV_ERROR, "cJSON_CreateObject failed.\n");
        goto error_exit;
    }
    cJSON_AddItemToObject(pstJson, "gazePos", pstGaze);
    for (i = 0; i < 2; i++)
    {
        pstTmp = cJSON_CreateString(pstDumpDmmInfo->u8Gaze[i]);
        if (NULL == pstTmp)
        {
            print_level(SV_ERROR, "cJSON_CreateString %s failed.\n", pstDumpDmmInfo->u8Gaze[i]);
            goto error_exit;
        }
        cJSON_AddItemToObject(pstGaze, u8Gaze[i], pstTmp);
    }

    pstUsrId = cJSON_CreateString(pstDumpDmmInfo->u8UsrId);
    if (NULL == pstUsrId)
    {
        print_level(SV_ERROR, "cJSON_CreateString %s failed.\n", pstDumpDmmInfo->u8UsrId);
        goto error_exit;
    }
    cJSON_AddItemToObject(pstJson, "usrId", pstUsrId);

    pstFrUserName = cJSON_CreateString(pstDumpDmmInfo->u8UsrName);
    if (NULL == pstFrUserName)
    {
        print_level(SV_ERROR, "cJSON_CreateString %s failed.\n", pstDumpDmmInfo->u8UsrName);
        goto error_exit;
    }
    cJSON_AddItemToObject(pstJson, "usrName", pstFrUserName);
    
    memset(szBuf, 0, DUMP_BUF_SIZE);
    cJSON_PrintPreallocated(pstJson, szBuf, DUMP_BUF_SIZE, 0);
    s32Fd = open("/var/info/dmm-tmp", O_CREAT|O_RDWR, S_IRUSR|S_IWUSR);
    if (s32Fd < 0)
    {
        print_level(SV_ERROR, "open file: /var/info/cellular-tmp failed. [err:%s]\n", strerror(errno));
        goto error_exit;
    }

    s32Ret = write(s32Fd, szBuf, strlen(szBuf)+1);
    if (s32Ret < 0)
    {
        print_level(SV_ERROR, "write file: /var/info/cellular-tmp failed. [err:%s]\n", strerror(errno));
        close(s32Fd);
        goto error_exit;
    }

    close(s32Fd);
    rename("/var/info/dmm-tmp", DUMP_INFO_DMM);
    
    cJSON_Delete(pstJson);
    return SV_SUCCESS;

error_exit:
    if (NULL != pstJson)
    {
        cJSON_Delete(pstJson); 
    }
    return SV_FAILURE;
}

sint32 dmm_DumpInfo_Mini(DUMP_DMM_S *pstDumpDmmInfo)
{
    sint32 s32Ret = -1, s32Fd = -1, i = 0;
    char szBuf[DUMP_BUF_SIZE] = {0};
    cJSON *pstJson = NULL;
    cJSON *pstUsrId = NULL;
    cJSON *pstFrUserName = NULL;
    cJSON *pstHeadPos = NULL;
    cJSON *pstGaze = NULL;
    cJSON *pstEyeScore = NULL;
    cJSON *pstGlassScore = NULL;
    cJSON *pstTmp = NULL;
    uint8  u8EyeScore[2][24] = {"openEye", "closeEye"};
    uint8  u8GlassScore[2][24] = {"normalglass", "sunglass"};
    uint8  u8HeadPose[3][24] = {"pitch", "yaw", "roll"};
    uint8  u8Gaze[2][24] = {"yaw", "pitch"};

    if (NULL == pstDumpDmmInfo)
    {
        return SV_FAILURE;
    }

    pstJson = cJSON_CreateObject();
    if (NULL == pstJson)
    {
        print_level(SV_ERROR, "cJSON_CreateObject failed.\n");
        goto error_exit;
    }

    pstTmp = cJSON_CreateBool(pstDumpDmmInfo->bImageMirror);
    if (NULL == pstTmp)
    {
        print_level(SV_ERROR, "cJSON_CreateBool %d failed.\n", pstDumpDmmInfo->bImageMirror);
        goto error_exit;
    }
    cJSON_AddItemToObject(pstJson, "bImageMirror", pstTmp);

    pstTmp = cJSON_CreateBool(pstDumpDmmInfo->bDetectFace);
    if (NULL == pstTmp)
    {
        print_level(SV_ERROR, "cJSON_CreateBool %d failed.\n", pstDumpDmmInfo->bDetectFace);
        goto error_exit;
    }
    cJSON_AddItemToObject(pstJson, "bDetectFace", pstTmp);

    pstHeadPos = cJSON_CreateObject();
    if (NULL == pstHeadPos)
    {
        print_level(SV_ERROR, "cJSON_CreateObject failed.\n");
        goto error_exit;
    }
    cJSON_AddItemToObject(pstJson, "headPos", pstHeadPos);
    for (i = 0; i < 3; i++)
    {
        pstTmp = cJSON_CreateString(pstDumpDmmInfo->u8HeadPose[i]);
        if (NULL == pstTmp)
        {
            print_level(SV_ERROR, "cJSON_CreateString %s failed.\n", pstDumpDmmInfo->u8HeadPose[i]);
            goto error_exit;
        }
        cJSON_AddItemToObject(pstHeadPos, u8HeadPose[i], pstTmp);
    }
    
    memset(szBuf, 0, DUMP_BUF_SIZE);
    cJSON_PrintPreallocated(pstJson, szBuf, DUMP_BUF_SIZE, 0);
    s32Fd = open("/var/info/dmm-tmp", O_CREAT|O_RDWR, S_IRUSR|S_IWUSR);
    if (s32Fd < 0)
    {
        print_level(SV_ERROR, "open file: /var/info/cellular-tmp failed. [err:%s]\n", strerror(errno));
        goto error_exit;
    }

    s32Ret = write(s32Fd, szBuf, strlen(szBuf)+1);
    if (s32Ret < 0)
    {
        print_level(SV_ERROR, "write file: /var/info/cellular-tmp failed. [err:%s]\n", strerror(errno));
        close(s32Fd);
        goto error_exit;
    }

    close(s32Fd);
    rename("/var/info/dmm-tmp", DUMP_INFO_DMM);
    
    cJSON_Delete(pstJson);
    return SV_SUCCESS;

error_exit:
    if (NULL != pstJson)
    {
        cJSON_Delete(pstJson); 
    }
    return SV_FAILURE;
}

void dmm_HandleDumpInfo(void *pvArg)
{
    sint32 s32Ret = 0;
    DUMP_DMM_S *pstDumpDmmInfo = (DUMP_DMM_S *)pvArg;
    
    s32Ret = pthread_detach(pthread_self());
    if (s32Ret != 0)
    {
        pthread_exit(NULL); // 显式退出
    }

    if (BOARD_IsCustomer(BOARD_C_DMS31V2_GBS))
    {
        s32Ret = dmm_DumpInfo_Mini(pstDumpDmmInfo);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "dmm_DumpInfo failed!\n");
        }
    }
    else
    {
        s32Ret = dmm_DumpInfo(pstDumpDmmInfo);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "dmm_DumpInfo failed!\n");
        }
    }

    return;
}

sint32 dmm_DumpInfoThread(DUMP_DMM_S *pstDumpInfo)
{
    sint32 s32Ret;
    pthread_t thread_trigger;
    static DUMP_DMM_S stDumpDmmInfo = {0};
    memcpy(&stDumpDmmInfo, pstDumpInfo, sizeof(DUMP_DMM_S));

    s32Ret = pthread_create(&thread_trigger, NULL, dmm_HandleDumpInfo, (void *)&stDumpDmmInfo);
    if (s32Ret < 0)
    {
        print_level(SV_ERROR, "pthread_create  failed. [err: %s]\n", strerror(errno));
        return SV_FAILURE;
    }

    return SV_SUCCESS;
}

sint32 dmm_PostDmmGui(MEDIA_GUI_ALARM_DMM_S *pstGuiAlarmDmm)
{
    sint32 s32Ret = 0;
    uint16 u16mask;
    MSG_PACKET_S stMsgPkt = {0};
    MEDIA_GUI_DRAW_S stMediaGuiDraw = {0};
    MEDIA_GUI_NULL_S stGuiNull;

    if (NULL == pstGuiAlarmDmm)
    {
        return SV_FAILURE;
    }

    memset(&stMsgPkt, 0, sizeof(stMsgPkt));
    stMsgPkt.pu8Data = (uint8*)&stMediaGuiDraw;
    stMsgPkt.u32Size = sizeof(MEDIA_GUI_DRAW_S);
    u16mask = MEDIA_GUI_GET_MASK(m_stDmmInfo.s32Chn, 1, MEDIA_GUI_OP_CLEAR);
    s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiNull);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
    }

    u16mask = MEDIA_GUI_GET_MASK(m_stDmmInfo.s32Chn, 1, MEDIA_GUI_OP_ALARM_DMM);
    s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, *pstGuiAlarmDmm);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
    }

    s32Ret = Msg_submitEvent(EP_CONTROL, OP_EVENT_MEDIA_GUI, &stMsgPkt);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
    }

    return SV_SUCCESS;
}

/******************************************************************************
 * 函数功能: 向 Control 发送进度条
 * 输入参数: string --- 进度条的字符串说明
             fprocess --- 进度条进度
             bClose --- 是否关闭进度条
 * 输出参数: 无
 * 返回值  : 无
 * 说明    : 无
 ******************************************************************************/
sint32 dmm_PostProcessBar(char *string, float fprocess, SV_BOOL bClose)
{
    sint32 s32Ret;
    MSG_PACKET_S stMsgPkt = {0};
    MEDIA_GUI_DRAW_S stMediaGuiDraw = {0};
    MEDIA_GUI_FACE_S stGuiFace = {0};
    MEDIA_GUI_PROCESS_BAR_S stGuiProcessBar = {0};
    MEDIA_GUI_NULL_S stGuiNull;
    uint16 u16mask;

    memset(&stMsgPkt, 0, sizeof(stMsgPkt));
    memset(&stMediaGuiDraw, 0, sizeof(stMediaGuiDraw));
    stMsgPkt.pu8Data = (uint8*)&stMediaGuiDraw;
    stMsgPkt.u32Size = sizeof(MEDIA_GUI_DRAW_S);
    
    u16mask = MEDIA_GUI_GET_MASK(m_stDmmInfo.s32Chn, 1, MEDIA_GUI_OP_CLEAR);
    s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiNull);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
    }

    if(!bClose && string != NULL)
    {
        strcpy(stGuiProcessBar.string, string);
        stGuiProcessBar.fprocess = fprocess;
        u16mask = MEDIA_GUI_GET_MASK(m_stDmmInfo.s32Chn, 1, MEDIA_GUI_OP_PROCESS_BAR);
        
        s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiProcessBar);
        if(s32Ret != SV_SUCCESS)
        {
            print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
        }
    }

    
    s32Ret = Msg_submitEvent(EP_CONTROL, OP_EVENT_MEDIA_GUI, &stMsgPkt);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
    }

    return SV_SUCCESS;
}

/******************************************************************************
 * 函数功能: 获取GPS信息数据
 * 输入参数: 无
 * 输出参数: pszMcuPort -- mcu对应的设备文件
 * 返回值  : SV_FAILURE -- 失败
 			 SV_SUCCESS -- 成功
 * 注意      : 无
 *****************************************************************************/
sint32 dmm_GetGpsResults(sint32 &s32status, sint32 &s32speed)
{
    sint32 s32Ret;
    char *tmpBuf = NULL;
    char szCmd[32] = {0};
    char szBuf[256] = {0};
    char szMcuPort[32] = {0};
    char szJsonRet[1024] = {0};
    cJSON *pstJson = NULL, *pstStatus = NULL, *pstSpeed = NULL;

    s32Ret = cJSON_GetJson(GPS_DUMP_INFO_FILE, szJsonRet);
    if (SV_SUCCESS != s32Ret)
    {
        return s32Ret;
    }

    pstJson = cJSON_Parse(szJsonRet);
    if (NULL == pstJson)
    {
        print_level(SV_ERROR, "cJSON_Parse failed.\n");
        return SV_FAILURE;
    }

    pstStatus = cJSON_GetObjectItemCaseSensitive(pstJson, "Status");
    if (NULL == pstStatus)
    {
        print_level(SV_ERROR, "keyword Status is not exist.\n");
        return SV_FAILURE;
    }
    s32status = pstStatus->valueint;

    pstSpeed = cJSON_GetObjectItemCaseSensitive(pstJson, "spk");
    if (NULL == pstSpeed)
    {
        print_level(SV_ERROR, "keyword spk is not exist.\n");
        return SV_FAILURE;
    }
    s32speed = pstSpeed->valueint;


    cJSON_Delete(pstJson);
    return SV_SUCCESS;
} 


/******************************************************************************
 * 函数功能: 计算角度平均值
 * 输入参数: ps32Angles --- 角度采样集指针
             u32Num --- 采样集数
 * 输出参数: 无
 * 返回值  : 无
 * 说明    : 将角度集按10度间隔分15类，取出现类别最频繁的点类前30个点的角度平均值
 *****************************************************************************/
sint32 dmm_CalcAnglesAverage(sint32 *pas32Angles, uint32 u32Num)
{
    /* 角度分类 */
    typedef struct tag_AngleClass_S
    {
        sint32 s32ClassAngle;                   /* 属于哪个角度类(-70 ~ 0 ~ 70) */
        uint32 u32PointNum;                     /* 该类的点集数 */
    } DMM_ANG_CLASS;
    
    sint32 i, j;
    sint32 s32Class;
    sint32 s32AngleSum = 0, s32AngleNum = 0;
    DMM_ANG_CLASS stAngleTmp;
    DMM_ANG_CLASS astAngleClass[15] = {{-70, 0}, {-60, 0}, {-50, 0}, {-40, 0}, {-30, 0}, {-20, 0}, {-10, 0}, \
                                       {0, 0}, {10, 0}, {20, 0}, {30, 0}, {40, 0}, {50, 0}, {60, 0}, {70, 0}};
    /* 统计各类出现点数 */
    for (i = 0; i < u32Num; i++)
    {
        s32Class = pas32Angles[i];
        s32Class += (s32Class > 0) ? 5 : -5;
        s32Class = s32Class / 10 * 10;
        //print_level(SV_DEBUG, "%d->%d\n", pas32Angles[i], s32Class);
        for (j = 0; j < 15; j++)
        {
            if (astAngleClass[j].s32ClassAngle == s32Class)
            {
                astAngleClass[j].u32PointNum++;
                break;
            }
        }
    }

    /* 按各类点数由高到低排序 */
    for (i = 0; i < 14; i++)
    {
        for (j = 0; j < 14-i; j++)
        {
            if (astAngleClass[j].u32PointNum < astAngleClass[j+1].u32PointNum)
            {
                stAngleTmp = astAngleClass[j];
                astAngleClass[j] = astAngleClass[j+1];
                astAngleClass[j+1] = stAngleTmp;
            }
        }
    }

    
    /* 计算类别最频繁的点类前30个点的角度平均值 */
    for (i = 0; i < 15; i++)
    {
        //print_level(SV_DEBUG, "%d->%d\n", astAngleClass[i].s32ClassAngle, astAngleClass[i].u32PointNum);
        for (j = 0; j < u32Num; j++)
        {
            s32Class = pas32Angles[j];
            s32Class += (s32Class > 0) ? 5 : -5;
            s32Class = s32Class / 10 * 10;
            if (astAngleClass[i].s32ClassAngle == s32Class)
            {
                s32AngleSum += pas32Angles[j];
                s32AngleNum++;
                if (s32AngleNum >= 30)
                {
                    break;
                }
            }
        }

        if (s32AngleNum >= 30)
        {
            break;
        }
    }

    if (s32AngleNum == 0)
    {
        return -1000;
    }

    return s32AngleSum / s32AngleNum;
}


/******************************************************************************
 * 函数功能: 计算视线方向投影在图像的终点
 * 输入参数: fPointStartX --- 投影方向起点x坐标
             fPointStartY --- 投影方向起点y坐标
             fYaw --- 视线方向水平角
             fPitch --- 视线方向俯仰角
 * 输出参数: fPointEndX --- 投影方向终点x坐标
             fPointEndY，投影方向终点y坐标
 * 返回值  : 无
 * 说明    : 用于计算坐标转换的视线旋转角度
 *****************************************************************************/
void dmm_GazeVecotrCompute(float fPointStartX, float fPointStartY, float fYaw, float fPitch, float& fPointEndX, float& fPointEndY)
{
    int32_t s32VectorLength = 112;
    fPointEndX = fPointStartX - s32VectorLength * sin(fYaw) * cos(fPitch);
    fPointEndY = fPointStartY - s32VectorLength * sin(fPitch);
}

/******************************************************************************
 * 函数功能: 向画布中添加视线跟踪的线段
 * 输入参数: pstDmmParam --- 算法返回的计算信息
 * 输出参数: pstMediaGuiDraw --- 画布
 * 返回值  : 无
 * 说明    : 用于向画布中添加视线跟踪的线段
 *****************************************************************************/
void dmm_DrawGazeLine(dmmParam *pstDmmParam, MEDIA_GUI_DRAW_S *pstMediaGuiDraw)
{
    sint32 s32Ret = 0;    
    uint16 u16mask;
    MEDIA_GUI_LINE_S stGuiLine;
    float fLEyeEndPoint[2] = {0};
    float fREyeEndPoint[2] = {0};
    uint8 u8LPointNum = 40;
    uint8 u8RPointNum = 48;

    if (SV_FALSE == m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsGazeTracking)
    {
        return;
    }
    
    dmm_GazeVecotrCompute(pstDmmParam->Ppoints[u8LPointNum], pstDmmParam->Ppoints[u8LPointNum+1], pstDmmParam->pfGaze2d[0], pstDmmParam->pfGaze2d[1], fLEyeEndPoint[0], fLEyeEndPoint[1]);
    dmm_GazeVecotrCompute(pstDmmParam->Ppoints[u8RPointNum], pstDmmParam->Ppoints[u8RPointNum+1], pstDmmParam->pfGaze2d[0], pstDmmParam->pfGaze2d[1], fREyeEndPoint[0], fREyeEndPoint[1]);

    u16mask = MEDIA_GUI_GET_MASK(m_stDmmInfo.s32Chn, 0, MEDIA_GUI_OP_DRAW_LINE);
    stGuiLine.x1 = pstDmmParam->Ppoints[u8LPointNum] / 1280.0;
    stGuiLine.y1 = pstDmmParam->Ppoints[u8LPointNum+1] / 720.0;
    stGuiLine.x2 = fLEyeEndPoint[0] / 1280.0;
    stGuiLine.y2 = fLEyeEndPoint[1] / 720.0;
    stGuiLine.color = GUI_COLOR_BLUE;
    stGuiLine.stick = 5;
    MEDIA_GUI_INSERT(*pstMediaGuiDraw, u16mask, stGuiLine);

    u16mask = MEDIA_GUI_GET_MASK(m_stDmmInfo.s32Chn, 0, MEDIA_GUI_OP_DRAW_LINE);
    stGuiLine.x1 = pstDmmParam->Ppoints[u8RPointNum] / 1280.0;
    stGuiLine.y1 = pstDmmParam->Ppoints[u8RPointNum+1] / 720.0;
    stGuiLine.x2 = fREyeEndPoint[0] / 1280.0;
    stGuiLine.y2 = fREyeEndPoint[1] / 720.0;
    MEDIA_GUI_INSERT(*pstMediaGuiDraw, u16mask, stGuiLine);
}

sint32 dmm_ClearOsd()
{    
    sint32 s32Ret;
    MEDIA_GUI_DRAW_S stMediaGuiDraw = {0};
    MEDIA_GUI_NULL_S stGuiNull;
    uint16 u16mask;
    MSG_PACKET_S stMsgPkt = {0};
    
    memset(&stMsgPkt, 0, sizeof(stMsgPkt));
    stMsgPkt.pu8Data = (uint8*)&stMediaGuiDraw;
    stMsgPkt.u32Size = sizeof(MEDIA_GUI_DRAW_S);
    memset(&stMediaGuiDraw, 0x00, sizeof(stMediaGuiDraw));

    //清空原来的画板
    u16mask = MEDIA_GUI_GET_MASK(m_stDmmInfo.s32Chn, 0, MEDIA_GUI_OP_CLEAR);
    s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiNull);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
        return SV_FAILURE;
    }

    s32Ret = Msg_submitEvent(EP_CONTROL, OP_EVENT_MEDIA_GUI, &stMsgPkt);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
        return SV_FAILURE;
    }
            
    return SV_SUCCESS;
}

/******************************************************************************
 * 函数功能: 安装预标定执行体
 * 输入参数: pcsDmmDetector --- DMS算法检测器
             pcsAlgStatus --- 算法检测状态
 * 输出参数: 无
 * 返回值  : 无
 * 说明    : 用于动态指示当前设备预安装位置的合适程度
 *****************************************************************************/
void dmm_CalibrationPreBody(CDSM *pcsDmmDetector, CStateAnalyzer *pcsAlgStatus)
{
    sint32 s32Ret = 0, i;
    sint32 s32SemChn = 0;
    dmmParam dmmP;
    cv::Rect referROi(390,145,500,375);
    sint32 s32PoseCenterAngle = 0;  // 头部姿势距中心角的偏移量
    sint32 s32PosePitchAngle = 0;   // 头部俯仰角
    MSG_PACKET_S stMsgPkt = {0};
    MEDIA_GUI_DRAW_S stMediaGuiDraw = {0};
    MEDIA_GUI_FACE_S stGuiFace;
    MEDIA_GUI_RECT_S stGuiRect;
    MEDIA_GUI_NULL_S stGuiNull;
    MEDIA_GUI_LINE_S stGuiLine;
    uint16 u16mask;
    void *pvBuf = NULL;
    uint32 u32BufLen = DMM_IMAGE_WIDTH*DMM_IMAGE_HEIGHT*1.5;
    time_t tEnterTime = 0;

    memset(dmmP.Ppoints,0,sizeof(float)*Face_Max_Point);
    if (NULL == pcsDmmDetector || NULL == pcsAlgStatus)
    {
        print_level(SV_ERROR, "input null ptr.\n");
        return ;
    }

    pvBuf = mmap(NULL, u32BufLen, PROT_READ, MAP_SHARED, m_stDmmInfo.s32MediaBufFd, 0);
    if (MAP_FAILED == pvBuf)
    {
        print_level(SV_ERROR, "mmap failed.\n");
        return ;
    }

#if (defined(BOARD_DMS31V2) || defined(BOARD_ADA47V1))
    s32SemChn = 0;
#else
    s32SemChn = 1;
#endif

    print_level(SV_INFO, "enter CalibrationPre.\n");
    tEnterTime = time(NULL);
    while (m_stDmmInfo.bRunning)
    {
        if (m_stDmmInfo.enRunStat != DMM_RUN_CALIBRATION_PRE)
        {
            print_level(SV_INFO, "exit CalibrationPre.\n");
            break;
        }

        /* 预标定状态最多保持3分钟（避免误操作无法回到检测状态）*/
        if (time(NULL) > tEnterTime + 180)
        {
            m_stDmmInfo.enRunStat = DMM_RUN_DETECTION;
        }

        /* 锁住计算资源,保证同一时刻只跑一个算法*/
        s32Ret = ALG_Calculate_Lock();
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "ALG_Calculate_Lock failed. [err=%d]\n", s32Ret);
            sleep_ms(1);
            continue;
        }
        
        /* P操作进入MediaBuffer临界区 */
        s32Ret = MS_P(s32SemChn);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "MS_P failed. [err=%d]\n", s32Ret);
            ALG_Calculate_unLock();
            sleep_ms(1);
            continue;
        }

        /* 模型向前推断函数 */
        s32Ret = pcsDmmDetector->ForwardGroupDSM((uint8*)pvBuf, dmmP, pcsAlgStatus->GetTelephoneS(), pcsAlgStatus->GetSmokeS(), pcsAlgStatus->GetNoMaskS(), true,true);
        if (s32Ret < 0)
        {
            print_level(SV_ERROR, "ForwardGroupDSM failed. [err=%d]\n", s32Ret);
            MS_V(s32SemChn);    // 退出临界区
            ALG_Calculate_unLock();
            sleep_ms(10);
            continue;
        }
        MS_V(s32SemChn);    // 退出临界区
        ALG_Calculate_unLock();

        s32PoseCenterAngle = dmmP.HeadPose[1];
        s32PosePitchAngle = dmmP.pfGaze2d[1] * (180.0 / M_PI);   // 从姿态俯仰角改为视线俯仰角，视线俯仰角单位是弧度，要转为角度
        print_level(SV_DEBUG, "detectFace:%d, center:%d, pitch:%d\n", dmmP.detectFace, s32PoseCenterAngle, s32PosePitchAngle);
        memset(&stMsgPkt, 0, sizeof(stMsgPkt));
        stMsgPkt.pu8Data = (uint8*)&stMediaGuiDraw;
        stMsgPkt.u32Size = sizeof(MEDIA_GUI_DRAW_S);
        memset(&stMediaGuiDraw, 0x00, sizeof(stMediaGuiDraw));
        u16mask = MEDIA_GUI_GET_MASK(m_stDmmInfo.s32Chn, 0, MEDIA_GUI_OP_CLEAR);
        s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiNull);
        
        if (dmmP.detectFace)
        {
            dmm_DrawGazeLine(&dmmP, &stMediaGuiDraw);
        
            s32PoseCenterAngle = (s32PoseCenterAngle + 5) / 10 * 10;
            stGuiFace.stFaceRect.color = GUI_COLOR_RED;
            stGuiFace.angle = (int)(dmmP.HeadPose[1] + 2.5 * ((dmmP.HeadPose[1] > 0) * 2 - 1))/5*5;
			stGuiFace.angle = dmmP.HeadPose[1] > 85? 90:stGuiFace.angle;
			stGuiFace.angle = dmmP.HeadPose[1] < -85? -90:stGuiFace.angle;
            stGuiFace.stFaceRect.x1 = 1.0 * (float)referROi.x / 1280.0;
            stGuiFace.stFaceRect.y1 = 1.0 * (float)referROi.y / 720.0;
            stGuiFace.stFaceRect.x2 = 1.0 * (float)(referROi.x+referROi.width) / 1280.0;
            stGuiFace.stFaceRect.y2 = 1.0 * (float)(referROi.y+referROi.height) / 720.0;
            stGuiFace.u32PointNum = 31;
            for (i = 0; i < 31; i++)
            {
                stGuiFace.astFacePoints[i].x = dmmP.Ppoints[i*2] / 1280.0;
                stGuiFace.astFacePoints[i].y = dmmP.Ppoints[i*2+1] / 720.0;
            }

            /* 图像中心参考区域与人脸区域的交叠区达到一定比例 */
            cv::Rect interRoi = referROi & dmmP.showFaceRoi;
            if (interRoi.area() > (0.15*referROi.area()) && (interRoi.area() > (0.8*dmmP.showFaceRoi.area())))
            {
                if (s32PoseCenterAngle <= 30 && s32PoseCenterAngle >= -30)
                {
                    stGuiFace.stFaceRect.color = GUI_COLOR_GREEN;
                } 
                else
                {
                    stGuiFace.stFaceRect.color = GUI_COLOR_YELLOW;
                }
            }
            
            u16mask = MEDIA_GUI_GET_MASK(m_stDmmInfo.s32Chn, 0, MEDIA_GUI_OP_DRAW_FACE);
            s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiFace);
        }
        else
        {
            stGuiRect.stick = 4;
            stGuiRect.color = GUI_COLOR_RED;
            stGuiRect.x1 = 1.0 * (float)referROi.x / 1280.0;
            stGuiRect.y1 = 1.0 * (float)referROi.y / 720.0;
            stGuiRect.x2 = 1.0 * (float)(referROi.x+referROi.width) / 1280.0;
            stGuiRect.y2 = 1.0 * (float)(referROi.y+referROi.height) / 720.0;
            u16mask = MEDIA_GUI_GET_MASK(m_stDmmInfo.s32Chn, 0, MEDIA_GUI_OP_DRAW_RECT);
            s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiRect);
        }

        s32Ret = Msg_submitEvent(EP_CONTROL, OP_EVENT_MEDIA_GUI, &stMsgPkt);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
        }
    }

    if (m_stDmmInfo.enRunStat != DMM_RUN_CALIBRATION)
    {
        memset(&stMediaGuiDraw, 0x00, sizeof(stMediaGuiDraw));
        u16mask = MEDIA_GUI_GET_MASK(m_stDmmInfo.s32Chn, 0, MEDIA_GUI_OP_CLEAR);
        s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiNull);
        s32Ret = Msg_submitEvent(EP_CONTROL, OP_EVENT_MEDIA_GUI, &stMsgPkt);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
        }
    }
    
    munmap(pvBuf, u32BufLen);
}

/******************************************************************************
 * 函数功能: 安装正式标定执行体
 * 输入参数: pcsDmmDetector --- DMS算法检测器
             pcsAlgStatus --- 算法检测状态
             pcsFRAnalyzer --- 人脸识别分析器
 * 输出参数: 无
 * 返回值  : 无
 * 说明    : 用于确认安装位置后,采集多点确认司机凝视角度
 *****************************************************************************/
void dmm_CalibrationBody(CDSM *pcsDmmDetector, CStateAnalyzer *pcsAlgStatus, CFRAnalyzer* pcsFRAnalyzer)
{
    sint32 s32Ret = 0, i;
    sint32 s32SemChn = 0;
    dmmParam dmmP;
    cv::Rect referROi(390,145,500,375);
    sint32 s32PoseCenterDeg = 0;
    sint32 s32PoseCenterAngle = 0;  // 头部姿势距中心偏移角
    sint32 s32PosePitchAngle = 0;   // 头部俯仰角
    sint32 s32CenterAverage, s32PitchAverage;
    sint32 as32CenterAngles[DMM_SAMPLE_NUM];    // 中心角采样集
    sint32 as32PitchAngles[DMM_SAMPLE_NUM];     // 俯仰角采样集
    uint32 u32SampleCnt = 0;
    MSG_PACKET_S stMsgPkt = {0};
    MEDIA_GUI_DRAW_S stMediaGuiDraw = {0};
    MEDIA_GUI_FACE_S stGuiFace;
    MEDIA_GUI_RECT_S stGuiRect;
    MEDIA_GUI_NULL_S stGuiNull;
    MEDIA_GUI_LINE_S stGuiLine;
    struct timespec tvLast = {0, 0};
    struct timespec tvNow = {0, 0};
    uint16 u16mask;
    void *pvBuf = NULL;
    uint32 u32BufLen = DMM_IMAGE_WIDTH*DMM_IMAGE_HEIGHT*1.5;

    memset(dmmP.Ppoints,0,sizeof(float)*Face_Max_Point);
    if (NULL == pcsDmmDetector || NULL == pcsAlgStatus || NULL == pcsFRAnalyzer)
    {
        print_level(SV_ERROR, "input null ptr.\n");
        return ;
    }

    pvBuf = mmap(NULL, u32BufLen, PROT_READ, MAP_SHARED, m_stDmmInfo.s32MediaBufFd, 0);
    if (MAP_FAILED == pvBuf)
    {
        print_level(SV_ERROR, "mmap failed.\n");
        return ;
    }

#if (defined(BOARD_DMS31V2) || defined(BOARD_ADA47V1))
    s32SemChn = 0;
#else
    s32SemChn = 1;
#endif

    clock_gettime(CLOCK_MONOTONIC, &tvLast);

    print_level(SV_INFO, "enter Calibration.\n");
    dmm_Alarm_PlayAudio(NOTIFY_CALIBRATION_START);
    //sleep_ms(2000);   // 延时2s移动到下面线程体内进行计算
    
    while (m_stDmmInfo.bRunning)
    {
        if (m_stDmmInfo.enRunStat != DMM_RUN_CALIBRATION)
        {
            print_level(SV_INFO, "exit Calibration.\n");
            break;
        }

        clock_gettime(CLOCK_MONOTONIC, &tvNow);
        if (tvNow.tv_sec - tvLast.tv_sec > 12)
        {
            print_level(SV_INFO, "Register timeout!\n");
            pthread_mutex_lock(&m_stDmmInfo.mutexRunStat);
            m_stDmmInfo.enRunStat = DMM_RUN_DETECTION;
            m_stDmmInfo.enRunResult = DMM_RES_TIMEOUT;
            pthread_mutex_unlock(&m_stDmmInfo.mutexRunStat);
            continue;
        }

        /* 锁住计算资源,保证同一时刻只跑一个算法*/
        s32Ret = ALG_Calculate_Lock();
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "ALG_Calculate_Lock failed. [err=%d]\n", s32Ret);
            sleep_ms(1);
            continue;
        }

        /* P操作进入MediaBuffer临界区 */
        s32Ret = MS_P(s32SemChn);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "MS_P failed. [err=%d]\n", s32Ret);
            ALG_Calculate_unLock();
            sleep_ms(1);
            continue;
        }

        /* 模型向前推断函数 */
        s32Ret = pcsDmmDetector->ForwardGroupDSM((uint8*)pvBuf, dmmP, pcsAlgStatus->GetTelephoneS(), pcsAlgStatus->GetSmokeS(), pcsAlgStatus->GetNoMaskS(), true, true);
        if (s32Ret < 0)
        {
            print_level(SV_ERROR, "ForwardGroupDSM failed. [err=%d]\n", s32Ret);
            MS_V(s32SemChn);    // 退出临界区
            ALG_Calculate_unLock();
            sleep_ms(10);
            continue;
        }
        MS_V(s32SemChn);    // 退出临界区
        ALG_Calculate_unLock();

        s32PoseCenterAngle = dmmP.HeadPose[1];
        s32PosePitchAngle = dmmP.pfGaze2d[1] * (180.0 / M_PI);   // 从姿态俯仰角改为视线俯仰角，视线俯仰角单位是弧度，要转为角度
        print_level(SV_DEBUG, "detectFace:%d, center:%d, pitch:%d\n", dmmP.detectFace, s32PoseCenterAngle, s32PosePitchAngle);
        memset(&stMsgPkt, 0, sizeof(stMsgPkt));
        stMsgPkt.pu8Data = (uint8*)&stMediaGuiDraw;
        stMsgPkt.u32Size = sizeof(MEDIA_GUI_DRAW_S);
        memset(&stMediaGuiDraw, 0x00, sizeof(stMediaGuiDraw));

        u16mask = MEDIA_GUI_GET_MASK(m_stDmmInfo.s32Chn, 0, MEDIA_GUI_OP_CLEAR);
        s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiNull);
        if (dmmP.detectFace)
        {
            dmm_DrawGazeLine(&dmmP, &stMediaGuiDraw);
        
            s32PoseCenterDeg = s32PoseCenterAngle;
            s32PoseCenterDeg += (s32PoseCenterDeg > 0) ? 5 : -5;
			s32PoseCenterDeg = (s32PoseCenterDeg + 2.5 * ((s32PoseCenterDeg > 0) * 2 - 1)) / 10 * 10;
			stGuiFace.stFaceRect.color = GUI_COLOR_RED;
            stGuiFace.angle = (int)(dmmP.HeadPose[1] + 2.5 * ((dmmP.HeadPose[1] > 0) * 2 - 1))/5*5;
			stGuiFace.angle = dmmP.HeadPose[1] > 85? 90:stGuiFace.angle;
			stGuiFace.angle = dmmP.HeadPose[1] < -85? -90:stGuiFace.angle;
            stGuiFace.stFaceRect.x1 = (1.0 * (float)referROi.x) / 1280.0;
            stGuiFace.stFaceRect.y1 = (1.0 * (float)referROi.y) / 720.0;
            stGuiFace.stFaceRect.x2 = (1.0 * (float)(referROi.x+referROi.width)) / 1280.0;
            stGuiFace.stFaceRect.y2 = (1.0 * (float)(referROi.y+referROi.height)) / 720.0;
            stGuiFace.u32PointNum = 31;
            for (i = 0; i < 31; i++)
            {
                stGuiFace.astFacePoints[i].x = dmmP.Ppoints[i*2] / 1280.0;
                stGuiFace.astFacePoints[i].y = dmmP.Ppoints[i*2+1] / 720.0;
            }

            /* 图像中心参考区域与人脸区域的交叠区达到一定比例 */
            cv::Rect interRoi = referROi & dmmP.showFaceRoi;
            if (interRoi.area() > (0.15*referROi.area()) && (interRoi.area() > (0.8*dmmP.showFaceRoi.area())))
            {
                if (-30 <= s32PoseCenterDeg && s32PoseCenterDeg <= 30)
                {
                    stGuiFace.stFaceRect.color = GUI_COLOR_GREEN;
                } 
                else
                {
                    stGuiFace.stFaceRect.color = GUI_COLOR_YELLOW;
                }
            }

            u16mask = MEDIA_GUI_GET_MASK(m_stDmmInfo.s32Chn, 0, MEDIA_GUI_OP_DRAW_FACE);
            s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiFace);
        }
        else
        {
            stGuiRect.stick = 4;
            stGuiRect.color = GUI_COLOR_RED;
            stGuiRect.x1 = 1.0 * (float)referROi.x / 1280.0;
            stGuiRect.y1 = 1.0 * (float)referROi.y / 720.0;
            stGuiRect.x2 = 1.0 * (float)(referROi.x+referROi.width) / 1280.0;
            stGuiRect.y2 = 1.0 * (float)(referROi.y+referROi.height) / 720.0;
            u16mask = MEDIA_GUI_GET_MASK(m_stDmmInfo.s32Chn, 0, MEDIA_GUI_OP_DRAW_RECT);
            s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiRect);
        }

        s32Ret = Msg_submitEvent(EP_CONTROL, OP_EVENT_MEDIA_GUI, &stMsgPkt);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
        }

        if(stGuiFace.stFaceRect.color != GUI_COLOR_GREEN && stGuiFace.stFaceRect.color != GUI_COLOR_YELLOW)
        {
            //print_level(SV_DEBUG, "color:%#x\n", stGuiFace.stFaceRect.color);
            continue;
        }

        /****************************************************************************************************************
         *  把上面的sleep_ms(2000)移到此处计算，解决NOTIFY_CALIBRATION_START语音播放后，直接sleep导致有2s的画框消失问题
         *  2s内一直会进行画框的操作，2s后再开始统计人脸角度
         ****************************************************************************************************************/
        if (tvNow.tv_sec - tvLast.tv_sec < 2)
        {
            continue;
        }

        as32CenterAngles[u32SampleCnt] = s32PoseCenterAngle;
        as32PitchAngles[u32SampleCnt] = s32PosePitchAngle;
        u32SampleCnt++;
        //print_level(SV_DEBUG, "u32SampleCnt:%d\n", u32SampleCnt);
        if (u32SampleCnt < DMM_SAMPLE_NUM)
        {
            continue;
        }

        s32CenterAverage = dmm_CalcAnglesAverage(as32CenterAngles, u32SampleCnt);
        s32PitchAverage = dmm_CalcAnglesAverage(as32PitchAngles, u32SampleCnt);
        print_level(SV_INFO, "s32CenterAverage:%d, s32PitchAverage:%d\n", s32CenterAverage, s32PitchAverage);
        if (s32CenterAverage == -1000 || s32PitchAverage == -1000)
        {
            print_level(SV_ERROR, "dmm_CalcAnglesAverage exception.\n");
            u32SampleCnt = 0;
            continue;
        }
        
        if (s32CenterAverage < DMM_MIN_ANGLE)
        {
            m_stDmmInfo.stCfgParam.bImageMirror ? dmm_Alarm_PlayAudio(NOTIFY_CALIBRATION_TOORIGHT) : dmm_Alarm_PlayAudio(NOTIFY_CALIBRATION_TOOLEFT);
            pthread_mutex_lock(&m_stDmmInfo.mutexRunStat);
            m_stDmmInfo.enRunStat = DMM_RUN_CALIBRATION_PRE;
            m_stDmmInfo.enRunResult = DMM_RES_FAILURE;
            pthread_mutex_unlock(&m_stDmmInfo.mutexRunStat);
        }
        else if (s32CenterAverage > DMM_MAX_ANGLE)
        {
            m_stDmmInfo.stCfgParam.bImageMirror ? dmm_Alarm_PlayAudio(NOTIFY_CALIBRATION_TOOLEFT) : dmm_Alarm_PlayAudio(NOTIFY_CALIBRATION_TOORIGHT);
            pthread_mutex_lock(&m_stDmmInfo.mutexRunStat);
            m_stDmmInfo.enRunStat = DMM_RUN_CALIBRATION_PRE;
            m_stDmmInfo.enRunResult = DMM_RES_FAILURE;
            pthread_mutex_unlock(&m_stDmmInfo.mutexRunStat);
        }
        else
        {
            s32Ret = CONFIG_ReloadFile();
            if (SV_SUCCESS != s32Ret)
            {
                print_level(SV_ERROR, "CONFIG_ReloadFile failed. [err=%#x]\n", s32Ret);
            }

            m_stDmmInfo.s32DmsCenterAngle = s32CenterAverage;
            pcsAlgStatus->DistractParamChange(s32CenterAverage, s32PitchAverage);//把标定结果回写给算法DMM
            pcsFRAnalyzer->CenterParamChange(s32CenterAverage); //把标定结果回写给算法FR
            m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsCalibrated = SV_TRUE;
            m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsCenterAngle = s32CenterAverage;
            m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsPitchAngle = s32PitchAverage;
            s32Ret = CONFIG_SetAlgParam(&m_stDmmInfo.stCfgParam);
            if (SV_SUCCESS != s32Ret)
            {
                print_level(SV_ERROR, "CONFIG_SetAlgParam failed. [err=%#x]\n", s32Ret);
            }
            pthread_mutex_lock(&m_stDmmInfo.mutexRunStat);
            m_stDmmInfo.enRunStat = DMM_RUN_DETECTION;
            m_stDmmInfo.enRunResult = DMM_RES_SUCCESS;
            pthread_mutex_unlock(&m_stDmmInfo.mutexRunStat);
			
            dmm_Alarm_PlayAudio(NOTIFY_CALIBRATION_SUCCESS);
        }
    }

    memset(&stMediaGuiDraw, 0x00, sizeof(stMediaGuiDraw));
    u16mask = MEDIA_GUI_GET_MASK(m_stDmmInfo.s32Chn, 0, MEDIA_GUI_OP_CLEAR);
    s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiNull);
    s32Ret = Msg_submitEvent(EP_CONTROL, OP_EVENT_MEDIA_GUI, &stMsgPkt);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
    }
    
    munmap(pvBuf, u32BufLen);
}

/******************************************************************************
 * 函数功能: 注册人脸执行体
 * 输入参数: pcsDmmDetector --- DMS算法检测器
             pcsFRAnalyzer --- 人脸识别分析器
 * 输出参数: 无
 * 返回值  : 无
 * 说明    : 用于人脸识别登陆功能
 *****************************************************************************/
void dmm_RegisterBody(CDSM *pcsDmmDetector, CFRAnalyzer *pcsFRAnalyzer)
{
    sint32 s32Ret = 0, i;
    sint32 s32SemChn = 0;
    frParam frP = {0};
    sint32 s32Progress = 0;
    float fProgress = 0;
    MSG_PACKET_S stMsgPkt = {0};
    struct timespec tvLast = {0, 0};
    struct timespec tvNow = {0, 0};
    void *pvBuf = NULL;
    uint32 u32BufLen = DMM_IMAGE_WIDTH*DMM_IMAGE_HEIGHT*1.5;

    if (NULL == pcsDmmDetector || NULL == pcsFRAnalyzer)
    {
        print_level(SV_ERROR, "input null ptr.\n");
        return ;
    }

    pvBuf = mmap(NULL, u32BufLen, PROT_READ, MAP_SHARED, m_stDmmInfo.s32MediaBufFd, 0);
    if (MAP_FAILED == pvBuf)
    {
        print_level(SV_ERROR, "mmap failed.\n");
        return ;
    }

#if (defined(BOARD_DMS31V2) || defined(BOARD_ADA47V1))
    s32SemChn = 0;
#else
    s32SemChn = 1;
#endif

    print_level(SV_INFO, "enter Register.\n");
	s32Ret = dmm_PostProcessBar("REGISTER", 0.0, SV_FALSE);
	if(s32Ret != SV_SUCCESS)
	{
		print_level(SV_ERROR, "dmm_PostProcessBar failed.\n");
	}
    dmm_Alarm_PlayAudio(NOTIFY_REGISTER_START);
    //sleep_ms(2000);     // 等待音频播放完成
    clock_gettime(CLOCK_MONOTONIC, &tvLast);
    frP.feature_val = false;
    strcpy(frP.userPath, m_stDmmInfo.szUserDir);
    while (m_stDmmInfo.bRunning)
    {
        if (m_stDmmInfo.enRunStat != DMM_RUN_REGISTER)
        {
            print_level(SV_INFO, "exit Register.\n");
            break;
        }

        clock_gettime(CLOCK_MONOTONIC, &tvNow);
        if (tvNow.tv_sec - tvLast.tv_sec > 10)
        {
            print_level(SV_INFO, "Register timeout!\n");
            pthread_mutex_lock(&m_stDmmInfo.mutexRunStat);
            m_stDmmInfo.enRunStat = DMM_RUN_DETECTION;
            m_stDmmInfo.enRunResult = DMM_RES_FAILURE;
            pthread_mutex_unlock(&m_stDmmInfo.mutexRunStat);
            continue;
        }

        /* 锁住计算资源,保证同一时刻只跑一个算法*/
        s32Ret = ALG_Calculate_Lock();
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "ALG_Calculate_Lock failed. [err=%d]\n", s32Ret);
            sleep_ms(1);
            continue;
        }

        /* P操作进入MediaBuffer临界区 */
        s32Ret = MS_P(s32SemChn);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "MS_P failed. [err=%d]\n", s32Ret);
            sleep_ms(1);
            continue;
        }

        /* 模型向前推断函数 */
        s32Ret = pcsDmmDetector->ForwardGroupFR((uint8*)pvBuf, frP);
        if (s32Ret < 0)
        {
            print_level(SV_ERROR, "ForwardGroupFR failed. [err=%d]\n", s32Ret);
            MS_V(s32SemChn);    // 退出临界区
            ALG_Calculate_unLock();
            sleep_ms(10);
            continue;
        }
        MS_V(s32SemChn);    // 退出临界区
        ALG_Calculate_unLock();

        if (frP.feature_val)
        {
            fProgress = pcsFRAnalyzer->FRRegister(frP);
            s32Progress = fProgress * 100;
            print_level(SV_INFO, "progress: %d%%\n", s32Progress);
			s32Ret = dmm_PostProcessBar("REGISTER", fProgress, SV_FALSE);
			if(s32Ret != SV_SUCCESS)
			{
				print_level(SV_ERROR, "dmm_PostProcessBar failed.\n");
			}
        }
        
        //s32Ret = Msg_submitEvent(EP_CONTROL, OP_EVENT_MEDIA_GUI, &stMsgPkt);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
        }

        if (s32Progress >= 100)
        {
            print_level(SV_INFO, "Register successful.\n");
            pcsFRAnalyzer->WriteInfo(frP);
            dmm_Alarm_PlayAudio(NOTIFY_REGISTRE_SUCCESS);
            pthread_mutex_lock(&m_stDmmInfo.mutexRunStat);
            m_stDmmInfo.enRunStat = DMM_RUN_DETECTION;
            m_stDmmInfo.enRunResult = DMM_RES_SUCCESS;
            pthread_mutex_unlock(&m_stDmmInfo.mutexRunStat);
        }
    }

	s32Ret = dmm_PostProcessBar("REGISTER", 0.0, SV_TRUE);
	if(s32Ret != SV_SUCCESS)
	{
		print_level(SV_ERROR, "dmm_PostProcessBar failed.\n");
	}
	
    munmap(pvBuf, u32BufLen);
}

/******************************************************************************
 * 函数功能: 人脸识别执行体
 * 输入参数: pcsDmmDetector --- DMS算法检测器
             pcsFRAnalyzer --- 人脸识别分析器
 * 输出参数: 无
 * 返回值  : 无
 * 说明    : 用于人脸识别登陆功能
 *****************************************************************************/
void dmm_RecognitionBody(CDSM *pcsDmmDetector, CFRAnalyzer *pcsFRAnalyzer)
{
    sint32 s32Ret = 0, i;
    sint32 s32SemChn = 0;
    uint16 warnState, u16UsrId;
    sint32 s32Fd = -1;
    frParam frP;
    sint32 s32Progress = 0;
    float fProgress = 0;
    char szUserNamePath[128];
    char szUserName[1024];
    MSG_PACKET_S stMsgPkt = {0};
    struct timespec tvLast = {0, 0};
    struct timespec tvNow = {0, 0};
    void *pvBuf = NULL;
    struct timeval tvAlarm;
    struct timezone tz;
    ALARM_EVENT_S stAlarmEvent = {0};
    ALARM_TYPE_E enAlarmType = ALARM_NOTHING;
    uint32 u32BufLen = DMM_IMAGE_WIDTH*DMM_IMAGE_HEIGHT*1.5;
    DUMP_DMM_S stDumpDmmInfo = {0};

    if (NULL == pcsDmmDetector || NULL == pcsFRAnalyzer)
    {
        print_level(SV_ERROR, "input null ptr.\n");
        return ;
    }

    pvBuf = mmap(NULL, u32BufLen, PROT_READ, MAP_SHARED, m_stDmmInfo.s32MediaBufFd, 0);
    if (MAP_FAILED == pvBuf)
    {
        print_level(SV_ERROR, "mmap failed.\n");
        return;
    }
    m_stDmmInfo.bFirstFace = SV_FALSE;
    memset(&m_stDmmInfo.stDumpDmmInfo, 0, sizeof(DUMP_DMM_S));

#if (defined(BOARD_DMS31V2) || defined(BOARD_ADA47V1))
    s32SemChn = 0;
#else
    s32SemChn = 1;
#endif

    print_level(SV_INFO, "enter Recognition.\n");
	s32Ret = dmm_PostProcessBar("LOGIN", 0.0, SV_FALSE);
	if(s32Ret != SV_SUCCESS)
	{
		print_level(SV_ERROR, "dmm_PostProcessBar failed.\n");
	}
    
    if (m_stDmmInfo.bPlayLoginAudio)
    {
        dmm_Alarm_PlayAudio(NOTIFY_LOGIN_START);
        //sleep_ms(2000);     // 等待音频播放完成
    }
    
    clock_gettime(CLOCK_MONOTONIC, &tvLast);
    frP.feature_val = false;
    pcsFRAnalyzer->ReadFeatrue(ALG_USERS_PATH);
    while (m_stDmmInfo.bRunning)
    {
        if (m_stDmmInfo.enRunStat != DMM_RUN_RECOGNITION)
        {
            print_level(SV_INFO, "exit Recognition.\n");
            break;
        }

        clock_gettime(CLOCK_MONOTONIC, &tvNow);
        if (tvNow.tv_sec - tvLast.tv_sec > 10)
        {
            print_level(SV_INFO, "Recognition timeout!\n");
            if (m_stDmmInfo.bPlayLoginAudio)
            {
                dmm_Alarm_PlayAudio(NOTIFY_LNGIN_FAILED);
            }
            
            dmm_DumpInfo(&m_stDmmInfo.stDumpDmmInfo);
            pthread_mutex_lock(&m_stDmmInfo.mutexRunStat);
            m_stDmmInfo.enRunStat = DMM_RUN_DETECTION;
            m_stDmmInfo.enRunResult = DMM_RES_TIMEOUT;
            pthread_mutex_unlock(&m_stDmmInfo.mutexRunStat);
            continue;
        }

        /* 锁住计算资源,保证同一时刻只跑一个算法*/
        s32Ret = ALG_Calculate_Lock();
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "ALG_Calculate_Lock failed. [err=%d]\n", s32Ret);
            sleep_ms(1);
            continue;
        }

        /* P操作进入MediaBuffer临界区 */
        s32Ret = MS_P(s32SemChn);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "MS_P failed. [err=%d]\n", s32Ret);
            sleep_ms(1);
            continue;
        }

        /* 模型向前推断函数 */
        s32Ret = pcsDmmDetector->ForwardGroupFR((uint8*)pvBuf, frP);
        if (s32Ret < 0)
        {
            print_level(SV_ERROR, "ForwardGroupFR failed. [err=%d]\n", s32Ret);
            MS_V(s32SemChn);    // 退出临界区
            ALG_Calculate_unLock();
            sleep_ms(10);
            continue;
        }
        MS_V(s32SemChn);    // 退出临界区
        ALG_Calculate_unLock();

        if (frP.feature_val)
        {
            fProgress = pcsFRAnalyzer->FRRecognition(frP);
            s32Progress = fProgress * 100;
            print_level(SV_INFO, "progress: %d%%\n", s32Progress);
			s32Ret = dmm_PostProcessBar("LOGIN", fProgress, SV_FALSE);
			if(s32Ret != SV_SUCCESS)
			{
				print_level(SV_ERROR, "dmm_PostProcessBar failed.\n");
			}
        }
        
        //s32Ret = Msg_submitEvent(EP_CONTROL, OP_EVENT_MEDIA_GUI, &stMsgPkt);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
        }

        if (s32Progress >= 100)
        {
            print_level(SV_INFO, "Face Recognition end\n");
            print_level(SV_INFO, "Face Recognition ID: %d\n", pcsFRAnalyzer->GetFRId());
            print_level(SV_INFO, "Face Recognition name: %s\n", pcsFRAnalyzer->GetIdName(pcsFRAnalyzer->GetFRId()));
            if (pcsFRAnalyzer->GetFRId() > 0)
            {
                if (m_stDmmInfo.bPlayLoginAudio)
                {
                    dmm_Alarm_PlayAudio(NOTIFY_LNGIN_SUCCESS);
                }
                snprintf(m_stDmmInfo.stUserInfo.szDir, DMM_MAX_DIRLEN, "%s/%s", FRS_USERINFOS_PATH, pcsFRAnalyzer->GetIdName(pcsFRAnalyzer->GetFRId()));
                m_stDmmInfo.enRunResult = DMM_RES_SUCCESS;
                //sprintf(szUserNamePath, "%s/%s/username", ALG_USERS_PATH, pcsFRAnalyzer->Get_IdName(pcsFRAnalyzer->Get_FR_ID()));
                sprintf(szUserNamePath, "%s/username", m_stDmmInfo.stUserInfo.szDir);
                s32Fd = open(szUserNamePath, O_RDONLY);
                if (s32Fd < 0)
                {
                    print_level(SV_ERROR, "open file: %s failed. [err:%s]\n", szUserNamePath, strerror(errno));
                }
                else
                {
                    s32Ret = read(s32Fd, szUserName, 1024);
                    if (s32Ret < 0)
                    {
                        print_level(SV_ERROR, "read file: %s failed. [err:%s]\n", szUserNamePath, strerror(errno));
                    }
                }

                cutLineBreak(szUserName);
                //print_level(SV_DEBUG, "szName:%s, szDir:%s\n", szUserName, pastFileList[i]->d_name);
                strcpy(m_stDmmInfo.stUserInfo.szName, &szUserName[strlen("username=")]);
				
                strcpy(m_stDmmInfo.stDumpDmmInfo.u8UsrId, pcsFRAnalyzer->GetIdName(pcsFRAnalyzer->GetFRId())+strlen("user"));
                strcpy(m_stDmmInfo.stDumpDmmInfo.u8UsrName, m_stDmmInfo.stUserInfo.szName);
            }
            else
            {
                if (m_stDmmInfo.bPlayLoginAudio)
                {
                    dmm_Alarm_PlayAudio(NOTIFY_LNGIN_FAILED);
                }
				m_stDmmInfo.enRunResult = DMM_RES_FAILURE;
            }
            dmm_DumpInfo(&m_stDmmInfo.stDumpDmmInfo);
			
            pthread_mutex_lock(&m_stDmmInfo.mutexRunStat);
            m_stDmmInfo.enRunStat = DMM_RUN_DETECTION;
            pthread_mutex_unlock(&m_stDmmInfo.mutexRunStat);
        }
    }
    
	if (m_stDmmInfo.enRunResult == DMM_RES_SUCCESS)
		enAlarmType = NOTIFY_LNGIN_SUCCESS;
	else if (m_stDmmInfo.enRunResult == DMM_RES_TIMEOUT)
		enAlarmType = NOTIFY_LNGIN_TIMEOUT;
	else
		enAlarmType = NOTIFY_LNGIN_FAILED;

    if (NOTIFY_LNGIN_SUCCESS != enAlarmType)
    {
        s32Ret = dmm_Alarm_Post(NOTIFY_LNGIN_FAILED);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "dmm_Alarm_Post failed.\n");
        }
    }
    
    // 提交登录报警事件
    memset(&stAlarmEvent, 0, sizeof(stAlarmEvent));
    gettimeofday(&tvAlarm, &tz);
    stAlarmEvent.enAlarmEvent = ALARM_EVENT_FR;
    stAlarmEvent.enAlarmType = enAlarmType;
    stAlarmEvent.u16UsrId = atoi(m_stDmmInfo.stDumpDmmInfo.u8UsrId);
    strcpy(stAlarmEvent.u8UsrName, m_stDmmInfo.stDumpDmmInfo.u8UsrName);
    stAlarmEvent.s32TimeStamp = (sint32)tvAlarm.tv_sec;
    
    memset(&stMsgPkt, 0, sizeof(stMsgPkt));
    stMsgPkt.stMsg.u16OpCode = OP_EVENT_ALG_ALARM;
    stMsgPkt.pu8Data = (uint8 *)&stAlarmEvent;
    stMsgPkt.u32Size = sizeof(stAlarmEvent);
    print_level(SV_DEBUG, "u16UsrId:%03u, warnState:%d\n", stAlarmEvent.u16UsrId, enAlarmType);
    
    s32Ret = Msg_submitEvent(EP_CONTROL, OP_EVENT_ALG_ALARM, &stMsgPkt);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
    }
    s32Ret = Msg_submitEvent(EP_HTTPSERVER, OP_EVENT_ALG_ALARM, &stMsgPkt); //回传到web端显示报警信息
    if (SV_SUCCESS != s32Ret)
    {
       print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
    }

    s32Ret = dmm_PostProcessBar("LOGIN", 0.0, SV_TRUE);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "dmm_PostProcessBar failed.\n");
    }

    munmap(pvBuf, u32BufLen);
}

/******************************************************************************
 * 函数功能: DMS算法检测执行体
 * 输入参数: pcsDmmDetector --- DMS算法检测器
             pcsAlgStatus --- 算法检测状态
 * 输出参数: 无
 * 返回值  : 无
 * 注意    : 无
 *****************************************************************************/
void dmm_DetctionBody(CDSM *pcsDmmDetector, CStateAnalyzer *pcsAlgStatus)
{
    sint32 s32Ret = 0, i = 0, j = 0;
    sint32 s32SemChn = 0;
    dmmParam dmmP;
    sint32 s32FatigueTimeMs = m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32FatigueInterval * 1000; // 检测倒计时(ms)
    sint32 s32FatigueLast = 0;   // Fatigue 报警后的持续时间,针对L2等级配置
    sint32 s32FatigueL2TimeMs = m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32FatigueL2Interval * 1000; // 检测倒计时(ms)
    sint32 s32DistractionTimeMs = m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DistractionInterval * 1000;
    sint32 s32NoDriverTimeMs = m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32NoDriverInterval * 1000;
    sint32 s32SmokeTimeMs = m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32SmokeInterval * 1000;
    sint32 s32PhoneTimeMs = m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32PhoneInterval * 1000;
    sint32 s32DrinkEatTimeMs = m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DrinkEatInterval * 1000;
    sint32 s32YawnTimeMs = m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32YawnInterval * 1000;
    sint32 s32NoMaskTimeMs = m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32NoMaskInterval * 1000;
    sint32 s32SunGlassTimeMs = m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32SunGlassInterval * 1000;
    sint32 s32SeatBeltTimeMs = m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32SeatBeltInterval * 1000;
    sint32 s32ShelterTimeMs  = m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32ShelterInterval * 1000;
    sint32 s32OverspeedTimeMs  = 0;
    sint32 s32AlarmTimeMs[ALARM_DMS_BUFF] = {0};
    sint32 s32AlarmTimeMsLast[ALARM_DMS_BUFF] = {0};       /* 上一次设置的检测检测 */
    SV_BOOL bAlarm[ALARM_DMS_BUFF] = {0};
    SV_BOOL bFatigue = SV_FALSE, bFatigueL2 = SV_FALSE, bDistraction = SV_FALSE, bNoDriver = SV_FALSE, bSmoke = SV_FALSE;  // 警报是否被触发
    SV_BOOL bPhone = SV_FALSE, bYawn = SV_FALSE, bNoMask = SV_FALSE, bSunGlass = SV_FALSE, bSeatBelt = SV_FALSE, bShelter = SV_FALSE, bDrinkEat = SV_FALSE;
    SV_BOOL bOverspeed = SV_FALSE;
    float fSensitivity;
    uint32 u32StepTimeMs = 0;
    uint32 u32NoFaceTimeMs = 0;   /* 连续无司机的时间 */
    SV_BOOL bCreareNoface = SV_FALSE;
    uint32 u32FaceDetectTimeMs = 0;     /* 人脸检测的时间计时，北京速力需求，计时到一分钟，进行一次人脸检测 */
    SV_BOOL bAngleOut = SV_FALSE;       /* 人脸角度是否超出合理范围，北京速力需求，能正常检测到人脸才跳转到人脸识别 */
    SV_BOOL bStartDetAngle = SV_FALSE;  /* 是否开始检测人脸角度 */
    uint32 u32CorrectAngleCnt = 0;      /* 人脸自动识别时，角度连续三次不超出合理范围才认为可以进行人脸识别 */
    static SV_BOOL s_bPlayGpsConn = SV_FALSE;
    struct timespec tvLast = {0, 0};
    struct timespec tvNow = {0, 0};
    struct timespec tvBegin, tvEnd;
    MSG_PACKET_S stMsgPkt = {0};
    MEDIA_GUI_DRAW_S stMediaGuiDraw = {0};
    MEDIA_GUI_FACE_S stGuiFace = {0};
    MEDIA_GUI_ALARM_DMM_S stGuiAlarmDmm = {0};
    MEDIA_GUI_NULL_S stGuiNull;
    MEDIA_GUI_LINE_S stGuiLine;
    uint16 u16mask;
    void *pvBuf = NULL;
    uint32 u32BufLen = DMM_IMAGE_WIDTH*DMM_IMAGE_HEIGHT*1.5;
    sint32 s32GpsStatus, s32GpsSpeed;     /* GPS状态, GPS速度 */
    WarnState WarnResult = 0, WarnResultLast = 0;
    ALARM_TYPE_E enAlarmType = ALARM_NOTHING, enAlarmTypeLast = ALARM_NOTHING;
    sint32 s32RepeatTime = 0;
    struct timeval tvAlarm;
    struct timezone tz;
    ALARM_EVENT_S stAlarmEvent = {0};
    SV_BOOL bWorkingLowSpeed = SV_FALSE;
    sint32 s32ClearOsdWtdTimeMs = -1;
    DUMP_DMM_S stDumpDmmInfo = {0};
    SV_BOOL bGetShelter = SV_FALSE;
    uint32 u32ShelterCnt = 0;    
    uint32 u32WorkSpeedIndex = 0;
    SV_BOOL bOsd = SV_FALSE;

    for (i = 0; i < ALARM_DMS_BUFF; i++)
    {
        if (i != ALARM_OVERSPEED)
        {
            s32AlarmTimeMs[i] = m_stDmmConf[i].s32DectectInterval;
        }
        s32AlarmTimeMsLast[i] = m_stDmmConf[i].s32DectectInterval;
    }
    
    memset(dmmP.Ppoints,0,sizeof(float)*Face_Max_Point);
    if (NULL == pcsDmmDetector || NULL == pcsAlgStatus)
    {
        print_level(SV_ERROR, "input null ptr.\n");
        return ;
    }

    pvBuf = mmap(NULL, u32BufLen, PROT_READ, MAP_SHARED, m_stDmmInfo.s32MediaBufFd, 0);
    if (MAP_FAILED == pvBuf)
    {
        print_level(SV_ERROR, "mmap failed.\n");
        return ;
    }

#if (defined(BOARD_DMS31V2) || defined(BOARD_ADA47V1))
    s32SemChn = 0;
#else
    s32SemChn = 1;
#endif

    print_level(SV_INFO, "enter DMS detection.\n");
    clock_gettime(CLOCK_MONOTONIC, &tvLast);
    while (m_stDmmInfo.bRunning)
    {
        if (m_stDmmInfo.enRunStat != DMM_RUN_DETECTION)
        {
            print_level(SV_INFO, "exit DMS detection.\n");
            dmm_ClearOsd();
            break;
        }

        s32Ret = dmm_GetGpsResults(s32GpsStatus, s32GpsSpeed);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "dmm_GetGpsResults fail.\n");
            sleep_ms(1);
            continue;
        }

#if 0
        if (s32GpsStatus != 0)
        {
            if (!s_bPlayGpsConn)
            {
                dmm_Alarm_PlayAudio(NOTIFY_GPS_CONNECT);
                s_bPlayGpsConn = SV_TRUE;
            }
        }
        else
        {
            s_bPlayGpsConn = SV_FALSE;
        }
#endif

        clock_gettime(CLOCK_MONOTONIC, &tvNow);
        u32StepTimeMs = ((tvNow.tv_sec*1000 + tvNow.tv_nsec/1000000) - (tvLast.tv_sec*1000 + tvLast.tv_nsec/1000000));
        tvLast = tvNow;
        //print_level(SV_INFO, "u32StepTimeMs: %d\n", u32StepTimeMs);
        
        // 针对视线跟踪残留做的处理，如果计时大于1000ms还没有被置零表示现在没有在刷新人脸轮廓点
        if (-1 != s32ClearOsdWtdTimeMs)
        {
            s32ClearOsdWtdTimeMs += u32StepTimeMs;

            if (s32ClearOsdWtdTimeMs >= 1000)
            {
                s32ClearOsdWtdTimeMs = -1;
                dmm_ClearOsd();
            }
        }
        
        for (i = 0; i < ALARM_DMS_BUFF; i++)
        {
            if (m_stDmmConf[i].bIntervalChanged)
            {
                print_level(SV_INFO, "dectectInterval[%d] %d change to %d\n", i, s32AlarmTimeMsLast[i], m_stDmmConf[i].s32DectectInterval);
                if (s32AlarmTimeMsLast[i] >= 0)
                {
                    bAlarm[i] = SV_TRUE;
                    s32AlarmTimeMs[i] = m_stDmmConf[i].s32DectectInterval;
                }
                s32AlarmTimeMsLast[i] = m_stDmmConf[i].s32DectectInterval;

                if (i == NOTIFY_LNGIN_CHANGE_GUARD)
                {
                    bCreareNoface = SV_FALSE;
                    u32NoFaceTimeMs = 0;
                }
                
                m_stDmmConf[i].bIntervalChanged = false;
            }
            
            switch (i)
            {
                case ALARM_FATIGUE:
                    if (m_stDmmConf[i].s32DectectInterval >= 0)
                    {
                        pcsAlgStatus->SetFatigueS(true);
                        if (bAlarm[i])
                        {
                            s32AlarmTimeMs[i] -= u32StepTimeMs;
                            pcsAlgStatus->SetFatigueS(s32AlarmTimeMs[i] <= 0);
                            if (m_stDmmConf[ALARM_FATIGUE_L2].s32DectectInterval >= 0)  // 针对二级疲劳使能
                            {
                                if(s32FatigueLast >= 0)
                                {
                                    s32FatigueLast -= u32StepTimeMs;
                                    pcsAlgStatus->SetFatigueS(s32FatigueLast >= 0);
                                }
                            }
                        }
                        if (bAlarm[i] && s32AlarmTimeMs[i] <= 0)  /* 报警触发超过一定时间后重新复位 */
                        {
                            bAlarm[i] = SV_FALSE;
                            bAlarm[ALARM_FATIGUE_L2] = SV_FALSE;
                            s32AlarmTimeMs[i] = m_stDmmConf[i].s32DectectInterval;
                        }
                    }
                    else
                    {
                        pcsAlgStatus->SetFatigueS(false);
                    }
                    break;
                case ALARM_DISTRACTION:
                    if (m_stDmmConf[i].s32DectectInterval >= 0)
                    {
                        pcsAlgStatus->SetDistractS(true);
                        if (bAlarm[i])
                        {
                            s32AlarmTimeMs[i] -= u32StepTimeMs;
                            pcsAlgStatus->SetDistractS(s32AlarmTimeMs[i] <= 0);
                        }
                        if (bAlarm[i] && s32AlarmTimeMs[i] <= 0)
                        {
                            bAlarm[i] = SV_FALSE;
                            s32AlarmTimeMs[i] = m_stDmmConf[i].s32DectectInterval;
                        }
                    }
                    else
                    {
                        pcsAlgStatus->SetDistractS(false);
                    }
                    break;
                case ALARM_NO_DRIVER:
                    if (m_stDmmConf[i].s32DectectInterval >= 0)
                    {
                        pcsAlgStatus->SetNodriverS(true);
                        if (bAlarm[i])
                        {
                            s32AlarmTimeMs[i] -= u32StepTimeMs;
                            pcsAlgStatus->SetNodriverS(s32AlarmTimeMs[i] <= 0);
                        }
                        if (bAlarm[i] && s32AlarmTimeMs[i] <= 0)
                        {
                            bAlarm[i] = SV_FALSE;
                            s32AlarmTimeMs[i] = m_stDmmConf[i].s32DectectInterval;
                        }
                    }
                    else
                    {
                        pcsAlgStatus->SetNodriverS(false);
                    }
                    break;
                case ALARM_SMOKE:
                    if (m_stDmmConf[i].s32DectectInterval >= 0)
                    {
                        pcsAlgStatus->SetSmokeS(true);
                        if (bAlarm[i])
                        {
                            s32AlarmTimeMs[i] -= u32StepTimeMs;
                            pcsAlgStatus->SetSmokeS(s32AlarmTimeMs[i] <= 0);
                        }
                        if (bAlarm[i] && s32AlarmTimeMs[i] <= 0)
                        {
                            bAlarm[i] = SV_FALSE;
                            s32AlarmTimeMs[i] = m_stDmmConf[i].s32DectectInterval;
                        }
                    }
                    else
                    {
                        pcsAlgStatus->SetSmokeS(false);
                    }
                    break;
                case ALARM_PHONE:
                    if (m_stDmmConf[i].s32DectectInterval >= 0)
                    {
                        pcsAlgStatus->SetTelephoneS(true);
                        if (bAlarm[i])
                        {
                            s32AlarmTimeMs[i] -= u32StepTimeMs;
                            pcsAlgStatus->SetTelephoneS(s32AlarmTimeMs[i] <= 0);
                        }
                        if (bAlarm[i] && s32AlarmTimeMs[i] <= 0)
                        {
                            bAlarm[i] = SV_FALSE;
                            s32AlarmTimeMs[i] = m_stDmmConf[i].s32DectectInterval;
                        }
                    }
                    else
                    {
                        pcsAlgStatus->SetTelephoneS(false);
                    }
                    break;
                case ALARM_YAWN:
                    if (m_stDmmConf[i].s32DectectInterval >= 0)
                    {
                        pcsAlgStatus->SetYawnS(true);
                        if (bAlarm[i])
                        {
                            s32AlarmTimeMs[i] -= u32StepTimeMs;                            
                            pcsAlgStatus->SetYawnS(s32AlarmTimeMs[i] <= 0);
                        }
                        if (bAlarm[i] && s32AlarmTimeMs[i] <= 0)
                        {
                            bAlarm[i] = SV_FALSE;
                            s32AlarmTimeMs[i] = m_stDmmConf[i].s32DectectInterval;
                        }
                    }
                    else
                    {
                        pcsAlgStatus->SetYawnS(false);
                    }
                    break;
                case ALARM_NO_MASK:
                    if (m_stDmmConf[i].s32DectectInterval >= 0)
                    {
                        pcsAlgStatus->SetnoMaskS(true);
                        if (bAlarm[i])
                        {
                            s32AlarmTimeMs[i] -= u32StepTimeMs;
                            pcsAlgStatus->SetnoMaskS(s32AlarmTimeMs[i] <= 0);
                        }
                        if (bAlarm[i] && s32AlarmTimeMs[i] <= 0)
                        {
                            bAlarm[i] = SV_FALSE;
                            s32AlarmTimeMs[i] = m_stDmmConf[i].s32DectectInterval;
                        }
                    }
                    else
                    {
                        pcsAlgStatus->SetnoMaskS(false);
                    }
                    break;
                case ALARM_SUNGLASS:
                    if (m_stDmmConf[i].s32DectectInterval >= 0)
                    {
                        pcsAlgStatus->SetSunGlassS(true);
                        if (bAlarm[i])
                        {
                            s32AlarmTimeMs[i] -= u32StepTimeMs;
                            pcsAlgStatus->SetSunGlassS(s32AlarmTimeMs[i] <= 0);
                        }
                        if (bAlarm[i] && s32AlarmTimeMs[i] <= 0)
                        {
                            bAlarm[i] = SV_FALSE;
                            s32AlarmTimeMs[i] = m_stDmmConf[i].s32DectectInterval;
                        }
                    }
                    else
                    {
                        pcsAlgStatus->SetSunGlassS(false);
                    }
                    break;
                case ALARM_NO_SEATBELT:
                    if (m_stDmmConf[i].s32DectectInterval >= 0)
                    {
                        pcsAlgStatus->SetSeatbeltS(true);
                        if (bAlarm[i])
                        {
                            s32AlarmTimeMs[i] -= u32StepTimeMs;
                            pcsAlgStatus->SetSeatbeltS(s32AlarmTimeMs[i] <= 0);
                        }
                        if (bAlarm[i] && s32AlarmTimeMs[i] <= 0)
                        {
                            bAlarm[i] = SV_FALSE;
                            s32AlarmTimeMs[i] = m_stDmmConf[i].s32DectectInterval;
                        }
                    }
                    else
                    {
                        pcsAlgStatus->SetSeatbeltS(false);
                    }
                    break;
                case ALARM_SHELTER:
                    if (m_stDmmConf[i].s32DectectInterval >= 0)
                    {
                        pcsAlgStatus->SetShelterS(true);
                        if (bAlarm[i])
                        {
                            s32AlarmTimeMs[i] -= u32StepTimeMs;
                            pcsAlgStatus->SetShelterS(s32AlarmTimeMs[i] <= 0);
                        }
                        if (bAlarm[i] && s32AlarmTimeMs[i] <= 0)
                        {
                            bAlarm[i] = SV_FALSE;
                            s32AlarmTimeMs[i] = m_stDmmConf[i].s32DectectInterval;
                        }
                    }
                    else
                    {
                        pcsAlgStatus->SetShelterS(false);
                    }
                    break;
                case ALARM_DRINK_EAT:
                    if (m_stDmmConf[i].s32DectectInterval >= 0)
                    {
                        pcsAlgStatus->SetDrinkEatS(true);
                        if (bAlarm[i])
                        {
                            s32AlarmTimeMs[i] -= u32StepTimeMs;
                            pcsAlgStatus->SetDrinkEatS(s32AlarmTimeMs[i] <= 0);
                        }
                        if (bAlarm[i] && s32AlarmTimeMs[i] <= 0)
                        {
                            bAlarm[i] = SV_FALSE;
                            s32AlarmTimeMs[i] = m_stDmmConf[i].s32DectectInterval;
                        }
                    }
                    else
                    {
                        pcsAlgStatus->SetDrinkEatS(false);
                    }
                    break;
                case ALARM_OVERSPEED:
                    if (m_stDmmConf[i].s32DectectInterval >= 0)
                    {
                        if (s32AlarmTimeMs[i] <= 0)
                        {
                            if (s32GpsStatus != 0)
                            {
                                if (s32GpsSpeed > m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsOverspeedLimit)
                                {
                                    enAlarmType = ALARM_OVERSPEED;
                                    goto submit_msg;
                                }
                                else
                                {
                                    s32AlarmTimeMs[i] = 0;
                                }
                            }
                            else
                            {
                                s32AlarmTimeMs[i] = 0;
                            }
                        }

                        if (bAlarm[i])
                        {
                            s32AlarmTimeMs[i] -= u32StepTimeMs;
                        }
                        if (bAlarm[i] && s32AlarmTimeMs[i] <= 0)
                        {
                            bAlarm[i] = SV_FALSE;
                        }
                    }
                    break;
                case ALARM_NO_HELMET:
                    if (m_stDmmConf[i].s32DectectInterval >= 0)
                    {
                        pcsAlgStatus->SetHelmetS(true);
                        if (bAlarm[i])
                        {
                            s32AlarmTimeMs[i] -= u32StepTimeMs;
                            pcsAlgStatus->SetHelmetS(s32AlarmTimeMs[i] <= 0);
                        }
                        if (bAlarm[i] && s32AlarmTimeMs[i] <= 0)
                        {
                            bAlarm[i] = SV_FALSE;
                            s32AlarmTimeMs[i] = m_stDmmConf[i].s32DectectInterval;
                        }
                    }
                    else
                    {
                        pcsAlgStatus->SetHelmetS(false);
                    }
                    break;
                default:
                    continue;
            }
        }

        /* 设置检测灵敏度 */
        switch (m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsSensitivity)
        {
            case -1:    /* 自动模式 */
                if(s32GpsSpeed < m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsMiddleSpeedThr)
                    fSensitivity = 1.0;
                else if(s32GpsSpeed < m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsHighSpeedThr)
                    fSensitivity = 0.66;
                else
                    fSensitivity = 0.33;
                break;
            case 0:
                fSensitivity = 1.0;
                break;
            case 1:
                fSensitivity = 0.66;
                break;
            case 2:
                fSensitivity = 0.33;
                break;
            default:
                fSensitivity = 1.0;
        }

        /* 锁住计算资源,保证同一时刻只跑一个算法*/
        s32Ret = ALG_Calculate_Lock();
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "ALG_Calculate_Lock failed. [err=%d]\n", s32Ret);
            sleep_ms(1);
            continue;
        }

        /* P操作进入MediaBuffer临界区 */
        s32Ret = MS_P(s32SemChn);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "MS_P failed. [err=%d]\n", s32Ret);
            ALG_Calculate_unLock();
            sleep_ms(1);
            continue;
        }

        clock_gettime(CLOCK_MONOTONIC, &tvBegin);
        
        /* 模型向前推断函数 */
        s32Ret = pcsDmmDetector->ForwardGroupDSM((uint8*)pvBuf, dmmP, pcsAlgStatus->GetTelephoneS(), pcsAlgStatus->GetSmokeS(), pcsAlgStatus->GetNoMaskS(), true,true);
        if (s32Ret < 0)
        {
            print_level(SV_ERROR, "ForwardGroupDSM failed. [err=%d]\n", s32Ret);
            MS_V(s32SemChn);    // 退出临界区
            ALG_Calculate_unLock();
            sleep_ms(10);
            continue;
        }
        MS_V(s32SemChn);    // 退出临界区
        ALG_Calculate_unLock();

        clock_gettime(CLOCK_MONOTONIC, &tvEnd);
        //print_level(SV_INFO, "eclipse: %dms\n", (tvEnd.tv_sec*1000 + tvEnd.tv_nsec/1000000) - (tvBegin.tv_sec*1000 + tvBegin.tv_nsec/1000000));

        /* 行为分析 */
        pcsAlgStatus->StateAnalysis(dmmP, fSensitivity);
        WarnResult = pcsAlgStatus->GetState();    //获取警报类型
        do{
            switch (WarnResult)
            {
                case NoWarn:
                    enAlarmType          = ALARM_NOTHING;
                    break;
                case Fatigue:
                    enAlarmType          = ALARM_FATIGUE;
                    break;
                case FatigueL2:
                    enAlarmType          = ALARM_FATIGUE_L2;
                    break;
                case Distractoin:
                    enAlarmType          = ALARM_DISTRACTION;
                    break;
                case No_Driver:
                    enAlarmType          = ALARM_NO_DRIVER;
                    break;
                case Smoke:
                    enAlarmType          = ALARM_SMOKE;
                    break;
                case Phone:
                    enAlarmType          = ALARM_PHONE;
                    break;
                case Yawn:
                    enAlarmType          = ALARM_YAWN;
                    break;
                case No_Mask:
                    enAlarmType          = ALARM_NO_MASK;
                    break;
                case SunGlass:
                    enAlarmType          = ALARM_SUNGLASS;
                    break;
                case No_Seatbelt:
                    enAlarmType          = ALARM_NO_SEATBELT;
                    break;
                case Camera_Occlusion:
                    if (BOARD_IsCustomer(BOARD_C_DMS31V2_CREARE))
                    {
                        enAlarmType      = ALARM_NO_DRIVER;
                    }
                    else
                    {
                        enAlarmType      = ALARM_SHELTER;
                    }                    
                    bGetShelter = SV_TRUE;
                    break;
                case DrinkEat:
                    enAlarmType          = ALARM_DRINK_EAT;
                    break;
                case No_Helmet:
                    enAlarmType          = ALARM_NO_HELMET;
                    break;
                default:
                    break;
            }

            if (dmmP.detectFace)
            {
                float angleDiff = abs(dmmP.HeadPose[1] - m_stDmmInfo.s32DmsCenterAngle);
                if (angleDiff >= 30)
                {
                    u32CorrectAngleCnt = 0;
                }
                else
                {
                    u32CorrectAngleCnt++;
                }
                
                if (u32CorrectAngleCnt >= 3)
                {
                    bAngleOut = SV_FALSE;
                }
                else
                {
                    bAngleOut = SV_TRUE;
                }
            }
            else
            {
                u32CorrectAngleCnt = 0;
            }

#if (defined(BOARD_DMS31V2) || defined(BOARD_ADA47V1))
            /* 定时人脸识别处理 */
            u32FaceDetectTimeMs += u32StepTimeMs;
            if (m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsFrInterval > 0 && u32FaceDetectTimeMs > m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsFrInterval * 1000)
            {
                if (!bStartDetAngle)
                {
                    bStartDetAngle = SV_TRUE;
                    u32CorrectAngleCnt = 0;
                    bAngleOut = SV_TRUE;
                }
            }
            else
            {
                bStartDetAngle = SV_FALSE;
            }

            if (bStartDetAngle && !bAngleOut)
            {
                pthread_mutex_lock(&m_stDmmInfo.mutexRunStat);
                m_stDmmInfo.enRunStat = DMM_RUN_RECOGNITION;
                m_stDmmInfo.bPlayLoginAudio = SV_FALSE;
                pthread_mutex_unlock(&m_stDmmInfo.mutexRunStat);
                break;
            }

            /* CREARE客户，连续一段时间无司机后，若人脸再次出现，则识别一次人脸，检测是否换司机 */
            if (BOARD_IsCustomer(BOARD_C_DMS31V2_CREARE) && m_stDmmConf[NOTIFY_LNGIN_CHANGE_GUARD].s32DectectInterval >= 0)
            {
                /* 上电后跳到人脸识别线程记录第一张出现的人脸 */
                if (dmmP.detectFace && m_stDmmInfo.bFirstFace)
                {
                    m_stDmmInfo.enRunStat = DMM_RUN_CHANGE_DRIVER;
                    print_level(SV_INFO, "record first face.\n");
                    break;
                }

                if (!bCreareNoface && !dmmP.detectFace)
                {
                    u32NoFaceTimeMs += u32StepTimeMs;
                    if (u32NoFaceTimeMs >= DMM_NO_FACE_TIME_MS)
                    {
                        print_level(SV_INFO, "no face time exceeds %dms!!!\n", DMM_NO_FACE_TIME_MS);
                        bCreareNoface = SV_TRUE;
                    }
                }
                else if (dmmP.detectFace)
                {
                    u32NoFaceTimeMs = 0;
                }

                if (bCreareNoface && dmmP.detectFace)
                {
                    m_stDmmInfo.enRunStat = DMM_RUN_CHANGE_DRIVER;
                    bCreareNoface = SV_FALSE;
                    print_level(SV_INFO, "ready to enter DMM_RUN_CHANGE_DRIVER....\n");
                    break;
                }
            }
#endif            
        
            /* 如果检测到无司机，则准备跳到登录阶段 */
            if (E_LOGIN_AUTO == m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.enDmsLoginMode && dmmP.detectFace && ((m_stDmmInfo.bFirstFace || ALARM_NO_DRIVER == enAlarmTypeLast || ALARM_SHELTER == enAlarmTypeLast)))
            {
                if (!bAngleOut)
                {
                    pthread_mutex_lock(&m_stDmmInfo.mutexRunStat);
                    print_level(SV_INFO, "ready to login....\n");
                    m_stDmmInfo.enRunStat = DMM_RUN_RECOGNITION;
                    m_stDmmInfo.bPlayLoginAudio = SV_TRUE;
                    pthread_mutex_unlock(&m_stDmmInfo.mutexRunStat);
                    break;
                }
                else
                {
                    print_level(SV_WARN, "waiting for center angle to fr....\n");
                    continue;
                }
            }
			
submit_msg:
			bOsd = SV_FALSE;
            if (enAlarmType == ALARM_NOTHING)
            {
                if (enAlarmTypeLast == ALARM_NO_DRIVER && s32RepeatTime > 1 && dmmP.detectFace)
                {
                    s32AlarmTimeMs[ALARM_NO_DRIVER] = m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32NoDriverInterval * 1000;
                    enAlarmTypeLast = ALARM_NOTHING;
                    s32RepeatTime = 0;
                }

                if (m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsWorkspeed[enAlarmType] <= 0 || m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsWorkspeed_almNoGPS)
                {
                    bOsd = SV_TRUE;
                }
                break;
            }
            else if (enAlarmType == ALARM_OVERSPEED)
            {
                bOverspeed = SV_TRUE;
                s32AlarmTimeMs[ALARM_OVERSPEED] = m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32OverspeedInterval * 1000;
            }

            /* 工作速度判断 */
            if (BOARD_IsCustomer(BOARD_C_DMS31V2_CREARE))
                u32WorkSpeedIndex = enAlarmType;
            else
                u32WorkSpeedIndex = 0;
            
            bWorkingLowSpeed = SV_FALSE;
            if (m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsWorkspeed[u32WorkSpeedIndex] > 0)
            {
                if (s32GpsStatus == 0)
                {
                    if (!m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsWorkspeed_almNoGPS)
                    {
                        enAlarmTypeLast = enAlarmType;
                        sleep_ms(30);
                        break;
                    }
                    else
                    {
                        if (BOARD_IsCustomer(BOARD_C_DMS31V2_VUE))
                        {
                            bWorkingLowSpeed = SV_TRUE;
                        }
                    }
                }
                else if (s32GpsStatus != 0 && s32GpsSpeed < m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsWorkspeed[u32WorkSpeedIndex])
                {
                    if (BOARD_IsCustomer(BOARD_C_DMS31V2_VUE))
                    {
                        bWorkingLowSpeed = SV_TRUE;
                    }
                    else
                    {
                        m_stDmmInfo.pcsAlgStatus->SetTimerZero();
                        enAlarmTypeLast = enAlarmType;
                        sleep_ms(30);
                        break;
                    }
                }
            }
            bOsd = SV_TRUE;
            
            bAlarm[enAlarmType] = SV_TRUE;
            s32AlarmTimeMs[enAlarmType] = m_stDmmConf[enAlarmType].s32DectectInterval;

            if (enAlarmType == ALARM_FATIGUE)
            {
                if(m_stDmmConf[ALARM_FATIGUE_L2].s32DectectInterval >= 0)
                {
                    s32FatigueLast = 5 * 1000;
                }
                else
                {
                    s32FatigueLast = -1;
                }
            }

            /* 低速状态下触发的报警不推送报警事件 */
            if (!bWorkingLowSpeed)
            {
                /* 通知外部的结果统一使用ALARM_TYPE_E宏，函数内使用WarnState宏 */
                s32Ret = dmm_Alarm_Post((sint32)enAlarmType);
                if (SV_SUCCESS != s32Ret)
                {
                    print_level(SV_ERROR, "dmm_Alarm_Post failed.\n");
                }

                memset(&stAlarmEvent, 0, sizeof(stAlarmEvent));            
                gettimeofday(&tvAlarm, &tz);
                stAlarmEvent.enAlarmEvent = ALARM_EVENT_DMM;
                stAlarmEvent.enAlarmType = enAlarmType;
                stAlarmEvent.u16UsrId = atoi(m_stDmmInfo.stDumpDmmInfo.u8UsrId);
                strcpy(stAlarmEvent.u8UsrName, m_stDmmInfo.stDumpDmmInfo.u8UsrName);
                stAlarmEvent.s32TimeStamp = (sint32)tvAlarm.tv_sec;
                
                memset(&stMsgPkt, 0, sizeof(stMsgPkt));
                stMsgPkt.stMsg.u16OpCode = OP_EVENT_ALG_ALARM;
                stMsgPkt.pu8Data = (uint8 *)&stAlarmEvent;
                stMsgPkt.u32Size = sizeof(stAlarmEvent);
                s32Ret = Msg_submitEvent(EP_CONTROL, OP_EVENT_ALG_ALARM, &stMsgPkt);
                if (SV_SUCCESS != s32Ret)
                {
                    print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
                }
                s32Ret = Msg_submitEvent(EP_HTTPSERVER, OP_EVENT_ALG_ALARM, &stMsgPkt); //回传到web端显示报警信息
                if (SV_SUCCESS != s32Ret)
                {
                   print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
                }
            }

            stGuiAlarmDmm.enAlarmType = enAlarmType;
            stGuiAlarmDmm.enAlarmLevel = bWorkingLowSpeed ? ALARM_LEVEL_LOW : ALARM_LEVEL_HIGH;
            stGuiAlarmDmm.contime = 2;  //持续两秒钟
            s32Ret = dmm_PostDmmGui(&stGuiAlarmDmm);
            if (SV_SUCCESS != s32Ret)
            {
                print_level(SV_ERROR, "dmm_PostDmmGui failed. [err=%#x]\n", s32Ret);
            }

            print_level(SV_DEBUG, "WarnResult: %d\n", WarnResult);
            if (enAlarmType == ALARM_NO_DRIVER && s32RepeatTime >= 5);            /* 多次无司机报警屏蔽 */
            else if (enAlarmType == ALARM_SHELTER && s32RepeatTime >= 5);         /* 多次遮挡报警屏蔽 */
            else
            {
                print_level(SV_DEBUG, "enAlarmType: %d\n", enAlarmType);
                if (m_stDmmConf[ALARM_NOTHING].bDmsAudioEnable && m_stDmmConf[enAlarmType].bDmsAudioEnable && !bWorkingLowSpeed)
                {
                    s32Ret = dmm_Alarm_PlayAudio(enAlarmType);
                    if (SV_SUCCESS != s32Ret)
                    {
                        print_level(SV_ERROR, "dmm_Alarm_PlayAudio failed. [type:%d]\n", enAlarmType);
                    }
                }
            }

            if (enAlarmType != enAlarmTypeLast)
            {
                s32RepeatTime = 1;
                enAlarmTypeLast = enAlarmType;
                break;
            }

            /* case enAlarmType == enAlarmTypeLast */
            s32RepeatTime++;
            if(BOARD_IsNotCustomer(BOARD_C_DMS31V2_SHIQI))
            {
                break;
            }

            if(enAlarmType == ALARM_NO_DRIVER)
            {
                if(s32RepeatTime % 9 == 0)
                {
                    s32AlarmTimeMs[ALARM_NO_DRIVER] = 30 * 60 * 1000; // 延时30分钟
                }
                else if(s32RepeatTime % 3 == 0)
                {
                    s32AlarmTimeMs[ALARM_NO_DRIVER] = 3 * 60 * 1000; // 延时3分钟
                }
            }
            else if(enAlarmType == ALARM_PHONE)
            {
                if(s32RepeatTime % 9 == 0)
                {
                    s32AlarmTimeMs[ALARM_PHONE] = 10 * 60 * 1000; // 延时10分钟
                }
                else if(s32RepeatTime % 3 == 0)
                {
                    s32AlarmTimeMs[ALARM_PHONE] = 3 * 60 * 1000; // 延时3分钟
                }
            }
            else if(enAlarmType == ALARM_SMOKE)
            {
                if(s32RepeatTime % 9 == 0)
                {
                    s32AlarmTimeMs[ALARM_SMOKE] = 10 * 60 * 1000; // 延时10分钟
                }
                else if(s32RepeatTime % 3 == 0)
                {
                    s32AlarmTimeMs[ALARM_SMOKE] = 3 * 60 * 1000; // 延时3分钟
                }
            }
        } while(0);

        m_stDmmInfo.stDumpDmmInfo.enAlarmType = enAlarmType;
        /* 遮挡算法不经过模型运算，使用CPU对图像进行运算，速度比较快，这里需要对遮挡手动添加多几帧dump数据 */
        if (bGetShelter)
        {
            if (enAlarmType == ALARM_NOTHING)
            {
                if (u32ShelterCnt <= 5)
                {
                    u32ShelterCnt++;
                    m_stDmmInfo.stDumpDmmInfo.enAlarmType = ALARM_SHELTER;
                }
            }
            else if (enAlarmType != ALARM_SHELTER)
            {
                bGetShelter = SV_FALSE;
                u32ShelterCnt = 0;
            }
            else
            {
                u32ShelterCnt = 0;
            }
        }
        
        m_stDmmInfo.stDumpDmmInfo.bImageMirror = m_stDmmInfo.stCfgParam.bImageMirror;
        m_stDmmInfo.stDumpDmmInfo.bDetectFace = dmmP.detectFace ? SV_TRUE : SV_FALSE;
        m_stDmmInfo.stDumpDmmInfo.bYawn = dmmP.Yawn ? SV_TRUE : SV_FALSE;
        m_stDmmInfo.stDumpDmmInfo.bNoMask = dmmP.NoMask ? SV_TRUE : SV_FALSE;
        m_stDmmInfo.stDumpDmmInfo.bShelter = dmmP.shelter ? SV_TRUE : SV_FALSE;
        sprintf(m_stDmmInfo.stDumpDmmInfo.u8SmokeScore, "%f", dmmP.Smoke);
        sprintf(m_stDmmInfo.stDumpDmmInfo.u8PhoneScore, "%f", dmmP.Phone);
        sprintf(m_stDmmInfo.stDumpDmmInfo.u8DrinkEatScore, "%f", dmmP.fDrinkEat);
        sprintf(m_stDmmInfo.stDumpDmmInfo.u8SeatbeltScore, "%f", dmmP.seatbeltScore);
        sprintf(m_stDmmInfo.stDumpDmmInfo.u8HelmetScore, "%f", dmmP.fHelmetScore);
        for (j = 0; j < 2; j++)
            sprintf(m_stDmmInfo.stDumpDmmInfo.u8EyeScore[j], "%f", dmmP.pfEyeScore[j]);
        for (j = 0; j < 2; j++)
            sprintf(m_stDmmInfo.stDumpDmmInfo.u8GlassScore[j], "%f", dmmP.pfGlassScore[j]);
        for (j = 0; j < 3; j++)
            sprintf(m_stDmmInfo.stDumpDmmInfo.u8HeadPose[j], "%f", dmmP.HeadPose[j]);
        for (j = 0; j < 2; j++)
            sprintf(m_stDmmInfo.stDumpDmmInfo.u8Gaze[j], "%f", dmmP.pfGaze2d[j] * (180.0 / M_PI));  // 弧度转角度
        dmm_DumpInfoThread(&m_stDmmInfo.stDumpDmmInfo);

        #if 0
        DUMP_DMM_S stDumpDmmInfoGet = {0};
        dump_GetDmmInfo(&stDumpDmmInfoGet);
        printf("%d %d %d %d,\n%s %s %s %s\n%s %s\n%s %s\n%s %s %s\n%s %s\n", stDumpDmmInfoGet.bDetectFace, stDumpDmmInfoGet.bYawn, stDumpDmmInfoGet.bNoMask,
            stDumpDmmInfoGet.bShelter, stDumpDmmInfoGet.u8SmokeScore, stDumpDmmInfoGet.u8PhoneScore, stDumpDmmInfoGet.u8DrinkEatScore, stDumpDmmInfoGet.u8SeatbeltScore,
            stDumpDmmInfoGet.u8EyeScore[0], stDumpDmmInfoGet.u8EyeScore[1],
            stDumpDmmInfoGet.u8GlassScore[0], stDumpDmmInfoGet.u8GlassScore[1],
            stDumpDmmInfoGet.u8HeadPose[0], stDumpDmmInfoGet.u8HeadPose[1], stDumpDmmInfoGet.u8HeadPose[2],
            stDumpDmmInfoGet.u8Gaze[0], stDumpDmmInfoGet.u8Gaze[1]);
        #endif

        if (BOARD_IsCustomer(BOARD_C_DMS31V2_CREARE) && enAlarmType == ALARM_NOTHING)
        {
            bOsd = SV_TRUE;
        }

        /* 发送检测结果OSD信息到ipsys进行叠加 */
        if (m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsOsdEnable && bOsd)
        {
            memset(&stMsgPkt, 0, sizeof(stMsgPkt));
            stMsgPkt.pu8Data = (uint8*)&stMediaGuiDraw;
            stMsgPkt.u32Size = sizeof(MEDIA_GUI_DRAW_S);
            memset(&stMediaGuiDraw, 0x00, sizeof(stMediaGuiDraw));

            //清空原来的画板
            u16mask = MEDIA_GUI_GET_MASK(m_stDmmInfo.s32Chn, 0, MEDIA_GUI_OP_CLEAR);
            s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiNull);
            if (SV_SUCCESS != s32Ret)
            {
                print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
            }
            
            if (dmmP.detectFace && dmmP.showFaceRoi.width > 10 && dmmP.showFaceRoi.height > 10)
            {
                stGuiFace.angle = (int)(dmmP.HeadPose[1] + 2.5 * ((dmmP.HeadPose[1] > 0) * 2 - 1))/5*5;
				stGuiFace.angle = dmmP.HeadPose[1] > 85? 90:stGuiFace.angle;
				stGuiFace.angle = dmmP.HeadPose[1] < -85? -90:stGuiFace.angle;
                stGuiFace.stFaceRect.color = GUI_COLOR_YELLOW;
                stGuiFace.stFaceRect.x1 = (1.0 * (float)dmmP.showFaceRoi.x) / 1280.0;
                stGuiFace.stFaceRect.y1 = (1.0 * (float)dmmP.showFaceRoi.y) / 720.0;
                stGuiFace.stFaceRect.x2 = (1.0 * (float)(dmmP.showFaceRoi.x + dmmP.showFaceRoi.width)) / 1280.0;
                stGuiFace.stFaceRect.y2 = (1.0 * (float)(dmmP.showFaceRoi.y + dmmP.showFaceRoi.height)) / 720.0;
                stGuiFace.u32PointNum = 31;
                for (i = 0; i < 31; i++)
                {
                    stGuiFace.astFacePoints[i].x = dmmP.Ppoints[i*2] / 1280.0;
                    stGuiFace.astFacePoints[i].y = dmmP.Ppoints[i*2+1] / 720.0;
                }
            }
            else
            {
                stGuiFace.stFaceRect.color = GUI_COLOR_YELLOW;
                stGuiFace.u32PointNum = 0;
            }

            if (dmmP.detectFace)
            {
                dmm_DrawGazeLine(&dmmP, &stMediaGuiDraw);
            }

            //添加人脸绘制操作
            u16mask = MEDIA_GUI_GET_MASK(m_stDmmInfo.s32Chn, 0, MEDIA_GUI_OP_DRAW_FACE);
            s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiFace);
            if (SV_SUCCESS != s32Ret)
            {
                print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
            }
            
            s32Ret = Msg_submitEvent(EP_CONTROL, OP_EVENT_MEDIA_GUI, &stMsgPkt);
            if (SV_SUCCESS != s32Ret)
            {
                print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
            }
            
            s32ClearOsdWtdTimeMs = 0;
        }
    }

    munmap(pvBuf, u32BufLen);
}

/******************************************************************************
 * 函数功能: 更换司机检测执行体
 * 输入参数: pcsDmmDetector --- DMS算法检测器
             pcsFRAnalyzer --- 人脸识别分析器
 * 输出参数: 无
 * 返回值  : 无
 * 说明    : 用于人脸识别登陆功能
 *****************************************************************************/
void dmm_ChangeDriverBody(CDSM *pcsDmmDetector, CFRAnalyzer *pcsFRAnalyzer)
{
    sint32 s32Ret = 0, i;
    sint32 s32SemChn = 0;
    uint16 warnState, u16UsrId;
    sint32 s32Fd = -1;
    frParam frP;
    sint32 s32Progress = 0;
    float fProgress = 0;
    char szUserNamePath[128];
    char szUserName[1024];
    MSG_PACKET_S stMsgPkt = {0};
    struct timespec tvLast = {0, 0};
    struct timespec tvNow = {0, 0};
    void *pvBuf = NULL;
    struct timeval tvAlarm;
    struct timezone tz;
    ALARM_EVENT_S stAlarmEvent = {0};
    uint32 u32BufLen = DMM_IMAGE_WIDTH*DMM_IMAGE_HEIGHT*1.5;
    MEDIA_GUI_ALARM_DMM_S stGuiAlarmDmm = {0};

    if (NULL == pcsDmmDetector || NULL == pcsFRAnalyzer)
    {
        print_level(SV_ERROR, "input null ptr.\n");
        return ;
    }

    pvBuf = mmap(NULL, u32BufLen, PROT_READ, MAP_SHARED, m_stDmmInfo.s32MediaBufFd, 0);
    if (MAP_FAILED == pvBuf)
    {
        print_level(SV_ERROR, "mmap failed.\n");
        return ;
    }

#if (defined(BOARD_DMS31V2) || defined(BOARD_ADA47V1))
    s32SemChn = 0;
#else
    s32SemChn = 1;
#endif

    print_level(SV_INFO, "enter FRMatch.\n");
    
    clock_gettime(CLOCK_MONOTONIC, &tvLast);
    frP.feature_val = false;

    while (m_stDmmInfo.bRunning)
    {
        if (m_stDmmInfo.enRunStat != DMM_RUN_CHANGE_DRIVER)
        {
            print_level(SV_INFO, "exit Recognition.\n");
            break;
        }

        clock_gettime(CLOCK_MONOTONIC, &tvNow);
        if (tvNow.tv_sec - tvLast.tv_sec > 30)
        {
            print_level(SV_INFO, "FRMatch timeout!\n");
            pthread_mutex_lock(&m_stDmmInfo.mutexRunStat);
            m_stDmmInfo.enRunStat = DMM_RUN_DETECTION;
            m_stDmmInfo.enRunResult = DMM_RES_TIMEOUT;
            pthread_mutex_unlock(&m_stDmmInfo.mutexRunStat);
            continue;
        }

        /* 锁住计算资源,保证同一时刻只跑一个算法*/
        s32Ret = ALG_Calculate_Lock();
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "ALG_Calculate_Lock failed. [err=%d]\n", s32Ret);
            sleep_ms(1);
            continue;
        }

        /* P操作进入MediaBuffer临界区 */
        s32Ret = MS_P(s32SemChn);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "MS_P failed. [err=%d]\n", s32Ret);
            sleep_ms(1);
            continue;
        }

        /* 模型向前推断函数 */
        s32Ret = pcsDmmDetector->ForwardGroupFR((uint8*)pvBuf, frP);
        if (s32Ret < 0)
        {
            print_level(SV_ERROR, "ForwardGroupFR failed. [err=%d]\n", s32Ret);
            MS_V(s32SemChn);    // 退出临界区
            ALG_Calculate_unLock();
            sleep_ms(10);
            continue;
        }
        MS_V(s32SemChn);    // 退出临界区
        ALG_Calculate_unLock();

        if (frP.feature_val)
        {
            fProgress = pcsFRAnalyzer->FRMatch(frP);
            s32Progress = fProgress * 100;
            print_level(SV_INFO, "progress: %d%%\n", s32Progress);
        }

        if (s32Progress >= 100)
        {
            print_level(SV_INFO, "Face FRMatch end\n");


            /* 获取司机检测结果,         0为同一个人, -1不是同一个人 */
            if (0 == pcsFRAnalyzer->GetMatchResult())
            {
                m_stDmmInfo.enRunResult = DMM_RES_SUCCESS;
                
            }
            else
            {
                m_stDmmInfo.enRunResult = DMM_RES_FAILURE;
            }
            
            pthread_mutex_lock(&m_stDmmInfo.mutexRunStat);
            m_stDmmInfo.enRunStat = DMM_RUN_DETECTION;
            pthread_mutex_unlock(&m_stDmmInfo.mutexRunStat);
        }
    }

    /* 不通结果产生不同的事件 */
    if (m_stDmmInfo.enRunResult == DMM_RES_FAILURE)
    {
        print_level(SV_INFO, "different drivers.\n");
        warnState = NOTIFY_LNGIN_CHANGE_GUARD;
    }
    else if (m_stDmmInfo.enRunResult == DMM_RES_TIMEOUT)
    {
        print_level(SV_INFO, "time out.\n");
        warnState = NOTIFY_LNGIN_TIMEOUT;
    }
    else
    {
        m_stDmmInfo.bFirstFace = SV_FALSE;
        print_level(SV_INFO, "same driver\n");
        goto exit;
    }   

    // 提交登录报警事件
    memset(&stAlarmEvent, 0, sizeof(stAlarmEvent));
    gettimeofday(&tvAlarm, &tz);
    stAlarmEvent.enAlarmEvent = ALARM_EVENT_FR;
    stAlarmEvent.enAlarmType = warnState;
    stAlarmEvent.u16UsrId = atoi(m_stDmmInfo.stDumpDmmInfo.u8UsrId);
    strcpy(stAlarmEvent.u8UsrName, m_stDmmInfo.stDumpDmmInfo.u8UsrName);
    stAlarmEvent.s32TimeStamp = (sint32)tvAlarm.tv_sec;
    
    memset(&stMsgPkt, 0, sizeof(stMsgPkt));
    stMsgPkt.stMsg.u16OpCode = OP_EVENT_ALG_ALARM;
    stMsgPkt.pu8Data = (uint8 *)&stAlarmEvent;
    stMsgPkt.u32Size = sizeof(stAlarmEvent);
    print_level(SV_DEBUG, "warnState:%u\n", warnState);
    s32Ret = Msg_submitEvent(EP_CONTROL, OP_EVENT_ALG_ALARM, &stMsgPkt);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
    }

    stGuiAlarmDmm.enAlarmType = warnState;
    stGuiAlarmDmm.enAlarmLevel = ALARM_LEVEL_HIGH;
    stGuiAlarmDmm.contime = 2;  //持续两秒钟
    s32Ret = dmm_PostDmmGui(&stGuiAlarmDmm);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "dmm_PostDmmGui failed. [err=%#x]\n", s32Ret);
    }
exit:

    munmap(pvBuf, u32BufLen);
}


/* DMS算法驱动线程 */
void * dmm_alg_Body(void *pvArg)
{
    sint32 s32Ret = 0;
    DMM_RUN_E enRunStat;
    DMM_INFO_S *pstDmmInfo = (DMM_INFO_S *)pvArg;

    s32Ret = prctl(PR_SET_NAME, "dmm_body");
    if (0 != s32Ret)
    {
        print_level(SV_ERROR, "prctl PR_SET_NAME failed! [err:%s]\n", strerror(errno));
    }

    enRunStat = pstDmmInfo->enRunStat;
    while (pstDmmInfo->bRunning)
    {
        //print_level(SV_DEBUG, "dmm_alg_Body running...\n");
        sleep_ms(1000);
#if (defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1))     
        if (ALG_DMS != pstDmmInfo->stCfgParam.stAlgCh2.enAlgType)
        {
            sleep_ms(1000);
            continue;
        }
#endif        
        if (enRunStat != pstDmmInfo->enRunStat)
        {
            print_level(SV_INFO, "dmm running status change: %d -> %d\n", enRunStat, pstDmmInfo->enRunStat);
            enRunStat = pstDmmInfo->enRunStat;
        }
        
        pstDmmInfo->pcsDmmDetector->FpInit();
        pstDmmInfo->pcsAlgStatus->InitStateParam();
        switch (enRunStat)
        {
            case DMM_RUN_IDEL:
                sleep_ms(3000);
                continue;

            case DMM_RUN_CALIBRATION_PRE:
                dmm_CalibrationPreBody(pstDmmInfo->pcsDmmDetector, pstDmmInfo->pcsAlgStatus);
                break;

            case DMM_RUN_CALIBRATION:
                dmm_CalibrationBody(pstDmmInfo->pcsDmmDetector, pstDmmInfo->pcsAlgStatus, pstDmmInfo->pcsFRAnalyzer);
                break;

            case DMM_RUN_REGISTER:
                dmm_RegisterBody(pstDmmInfo->pcsDmmDetector, pstDmmInfo->pcsFRAnalyzer);
                break;

            case DMM_RUN_RECOGNITION:
                dmm_RecognitionBody(pstDmmInfo->pcsDmmDetector, pstDmmInfo->pcsFRAnalyzer);
                break;

            case DMM_RUN_DETECTION:
                dmm_DetctionBody(pstDmmInfo->pcsDmmDetector, pstDmmInfo->pcsAlgStatus);
                break;

            case DMM_RUN_CHANGE_DRIVER:
                dmm_ChangeDriverBody(pstDmmInfo->pcsDmmDetector, pstDmmInfo->pcsFRAnalyzer);
                break;

            default:
                print_level(SV_WARN, "invalid running status: %d\n", enRunStat);
                sleep_ms(500);
                continue;
        }
    }

    return NULL;
}

sint32 dmm_CheckCallback(uint8_t* au8SourceAddr, uint8_t* au8ResultAddr)
{
    sint32 s32Ret, i;
    MSG_PACKET_S stMsgPkt = {0}, stRetPkt = {0};
    KEY_AUTH_S stAuthSrc = {0}, stAuthDes = {0};
    
    stAuthSrc.s32Len = 16;
    memcpy(stAuthSrc.au8Buf, au8SourceAddr, 16);
    stMsgPkt.pu8Data = (uint8 *)&stAuthSrc;
    stMsgPkt.u32Size = sizeof(KEY_AUTH_S);
    stRetPkt.pu8Data = (uint8 *)&stAuthDes;
    s32Ret = Msg_execRequestBlock(EP_ALG, EP_HTTPSERVER, OP_REQ_KEY_AUTH, &stMsgPkt, &stRetPkt, sizeof(KEY_AUTH_S));
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "OP_REQ_KEY_AUTH failed. [err=%#x]\n", s32Ret);
    }

    m_stDmmInfo.bKeyAuth = stRetPkt.stMsg.s32Param;
    memcpy(au8ResultAddr, stAuthDes.au8Buf, 16);
    print_level(SV_DEBUG, "bKeyAuth:%d\n", m_stDmmInfo.bKeyAuth);
    
    return 0;
}

sint32 DMM_Init(DMM_CFG_PARAM_S *pstInitParam)
{
    sint32 s32Ret = 0, i = 0;
    sint32 s32PwmFd = -1;
    sint32 s32CenterGaze = 0;
    sint32 s32PitchAngleGaze = 0;
    char szCmd[128] = {0};
    char *pszModelPath = "/root/model";
    char *pszUnencryptedModelPath = "/mnt/nfs/model_unencrypted";
    bool bEncrypt = true;

    if (NULL == pstInitParam)
    {
        return ERR_NULL_PTR;
    }

    if (pstInitParam->s32MediaBufFd < 0)
    {
        return ERR_ILLEGAL_PARAM;
    }

    print_level(SV_INFO, "s32MediaBufFd:%d\n", pstInitParam->s32MediaBufFd);
    memset(&m_stDmmInfo, 0, sizeof(DMM_INFO_S));
    s32Ret = pthread_mutex_init(&m_stDmmInfo.mutexRunStat, NULL);
    if (0 != s32Ret)
    {
        print_level(SV_ERROR, "pthread_mutex_init failed! [err=%#x]\n", s32Ret);
        return ERR_SYS_NOTREADY;
    }

    if (COMMON_IsPathExist(pszUnencryptedModelPath))
    {
        print_level(SV_INFO, "%s is exist, use unencrypted model to debug!\n", pszUnencryptedModelPath);
        sprintf(szCmd, "umount %s 2> /dev/null", pszModelPath);
        SAFE_System(szCmd, NORMAL_WAIT_TIME);
        
        sprintf(szCmd, "mount --bind %s %s", pszUnencryptedModelPath, pszModelPath);
        SAFE_System(szCmd, NORMAL_WAIT_TIME);
        
        bEncrypt = false;
    }

    s32Ret = MS_Init();
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "MS_Init failed. [err=%#x]\n", s32Ret);
        return ERR_SYS_NOTREADY;
    }

    if (BOARD_IsNotCustomer(BOARD_C_DMS31V2_CREARE))
    {
        s32Ret = dmm_Alarm_PlayAudio(NOTIFY_ALGSTART);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "dmm_Alarm_PlayAudio failed.\n");
        }
    }
    
    bool bDetNoDriver = pstInitParam->stAlgParam.stAlgCh2.stDmsParam.s32NoDriverInterval >= 0;
    bool bDetFatigue = pstInitParam->stAlgParam.stAlgCh2.stDmsParam.s32FatigueInterval >= 0;
    bool bDetFatigueL2 = pstInitParam->stAlgParam.stAlgCh2.stDmsParam.s32FatigueL2Interval >= 0;
    bool bDetDistraction = pstInitParam->stAlgParam.stAlgCh2.stDmsParam.s32DistractionInterval >= 0;
    bool bDetSmoke = pstInitParam->stAlgParam.stAlgCh2.stDmsParam.s32SmokeInterval >= 0;
    bool bDetPhone = pstInitParam->stAlgParam.stAlgCh2.stDmsParam.s32PhoneInterval >= 0;
    bool bDetYawn = pstInitParam->stAlgParam.stAlgCh2.stDmsParam.s32YawnInterval >= 0;
    bool bDetNoMask = pstInitParam->stAlgParam.stAlgCh2.stDmsParam.s32NoMaskInterval >= 0;
    bool bDetSunGlass = pstInitParam->stAlgParam.stAlgCh2.stDmsParam.s32SunGlassInterval >= 0;
    bool bDetSeatBelt = pstInitParam->stAlgParam.stAlgCh2.stDmsParam.s32SeatBeltInterval >= 0;
    bool bDetShelter  = pstInitParam->stAlgParam.stAlgCh2.stDmsParam.s32ShelterInterval >= 0;
    bool bDetDrinkEat = pstInitParam->stAlgParam.stAlgCh2.stDmsParam.s32DrinkEatInterval >= 0;
    bool bDetNoHelmet = pstInitParam->stAlgParam.stAlgCh2.stDmsParam.s32NoHelmetInterval >= 0;
    m_stDmmInfo.bFirstFace = SV_TRUE;

    if (pstInitParam->stAlgParam.stAlgCh2.stDmsParam.bDmsCalibrated)
    {
        s32CenterGaze = pstInitParam->stAlgParam.stAlgCh2.stDmsParam.s32DmsCenterAngle;
        s32PitchAngleGaze = pstInitParam->stAlgParam.stAlgCh2.stDmsParam.s32DmsPitchAngle;
    }
    m_stDmmInfo.s32MediaBufFd = pstInitParam->s32MediaBufFd;
    m_stDmmInfo.s32Chn = pstInitParam->s32Chn;
#if (defined(BOARD_DMS31V2) || defined(BOARD_ADA47V1))
    m_stDmmInfo.pcsDmmDetector = new CDSM(E_DMS31);
#elif defined(BOARD_ADA42V1)
    m_stDmmInfo.pcsDmmDetector = new CDSM(E_ADA42);
#else
    m_stDmmInfo.pcsDmmDetector = new CDSM();
#endif

    m_stDmmInfo.pcsAlgStatus = new CStateAnalyzer(s32CenterGaze, s32PitchAngleGaze, bDetNoDriver, bDetFatigue, bDetDistraction, bDetSmoke, bDetPhone, bDetYawn, bDetNoMask, bDetFatigueL2, bDetSunGlass, bDetSeatBelt, bDetShelter, bDetDrinkEat, bDetNoHelmet, BOARD_IsCustomer(BOARD_C_DMS31V2_EXHIBITION) ? E_EXHIBITION : E_GENERAL);
    m_stDmmInfo.pcsFRAnalyzer = new CFRAnalyzer(ALG_USERS_PATH, s32CenterGaze);
    m_stDmmInfo.stCfgParam = pstInitParam->stAlgParam;
    s32Ret = m_stDmmInfo.pcsDmmDetector->DSMInit(dmm_CheckCallback, bEncrypt);
    if (s32Ret < 0)
    {
        print_level(SV_ERROR, "DSMInit failed. [err=%#x]\n", s32Ret);
        return SV_FAILURE;
    }
    
    if (BOARD_IsNotCustomer(BOARD_C_DMS31V2_CREARE))
    {
        s32Ret = dmm_Alarm_PlayAudio(NOTIFY_ALGRUNNING);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "dmm_Alarm_PlayAudio failed.\n");
        }
    }

    for (i = 0; i < ALARM_DMS_BUFF; i++)
    {
        m_stDmmConf[i].s32WorkSpeed = m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsWorkspeed[i];
        /* CREARE客户遮挡当做无司机来处理 */
        if (BOARD_IsCustomer(BOARD_C_DMS31V2_CREARE) && i == ALARM_SHELTER)
        {
            m_stDmmConf[i].s32WorkSpeed = m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsWorkspeed[ALARM_NO_DRIVER];
        }
    }

    m_stDmmConf[ALARM_FATIGUE].s32DectectInterval             =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32FatigueInterval * 1000;
    m_stDmmConf[ALARM_DISTRACTION].s32DectectInterval         =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DistractionInterval * 1000;
    m_stDmmConf[ALARM_NO_DRIVER].s32DectectInterval           =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32NoDriverInterval * 1000;
    m_stDmmConf[ALARM_SMOKE].s32DectectInterval               =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32SmokeInterval * 1000;
    m_stDmmConf[ALARM_PHONE].s32DectectInterval               =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32PhoneInterval * 1000;
    m_stDmmConf[ALARM_YAWN].s32DectectInterval                =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32YawnInterval * 1000;
    m_stDmmConf[ALARM_NO_MASK].s32DectectInterval             =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32NoMaskInterval * 1000;
    m_stDmmConf[ALARM_SUNGLASS].s32DectectInterval            =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32SunGlassInterval * 1000;
    m_stDmmConf[ALARM_NO_SEATBELT].s32DectectInterval         =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32SeatBeltInterval * 1000;
    m_stDmmConf[ALARM_SHELTER].s32DectectInterval             =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32ShelterInterval * 1000;
    m_stDmmConf[ALARM_FATIGUE_L2].s32DectectInterval          =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32FatigueL2Interval * 1000;
    m_stDmmConf[ALARM_DRINK_EAT].s32DectectInterval           =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DrinkEatInterval * 1000;
    m_stDmmConf[ALARM_OVERSPEED].s32DectectInterval           =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32OverspeedInterval * 1000;
    m_stDmmConf[ALARM_NO_HELMET].s32DectectInterval           =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32NoHelmetInterval * 1000;
    m_stDmmConf[NOTIFY_LNGIN_CHANGE_GUARD].s32DectectInterval =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32ChangeGuardInterval * 1000;

    m_stDmmConf[ALARM_FATIGUE].bDmsAlarmOut             =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutFatigue;
    m_stDmmConf[ALARM_DISTRACTION].bDmsAlarmOut         =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutDistraction;
    m_stDmmConf[ALARM_NO_DRIVER].bDmsAlarmOut           =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutNoDriver;
    m_stDmmConf[ALARM_SMOKE].bDmsAlarmOut               =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutSmoke;
    m_stDmmConf[ALARM_PHONE].bDmsAlarmOut               =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutPhone;
    m_stDmmConf[ALARM_YAWN].bDmsAlarmOut                =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutYawn;
    m_stDmmConf[ALARM_NO_MASK].bDmsAlarmOut             =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutNoMask;
    m_stDmmConf[ALARM_SUNGLASS].bDmsAlarmOut            =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutSunGlass;
    m_stDmmConf[ALARM_NO_SEATBELT].bDmsAlarmOut         =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutSeatBelt;
    m_stDmmConf[ALARM_SHELTER].bDmsAlarmOut             =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutShelter;
    m_stDmmConf[ALARM_FATIGUE_L2].bDmsAlarmOut          =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutFatigueL2;
    m_stDmmConf[ALARM_DRINK_EAT].bDmsAlarmOut           =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutDrinkEat;
    m_stDmmConf[NOTIFY_LNGIN_FAILED].bDmsAlarmOut       =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutLoginFail;
    m_stDmmConf[ALARM_OVERSPEED].bDmsAlarmOut           =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutOverspeed;
    m_stDmmConf[ALARM_NO_HELMET].bDmsAlarmOut           =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutNoHelmet;

    m_stDmmConf[ALARM_NOTHING].bDmsAudioEnable          =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioEnable;
    m_stDmmConf[ALARM_FATIGUE].bDmsAudioEnable          =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioFatigue;
    m_stDmmConf[ALARM_DISTRACTION].bDmsAudioEnable      =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioDistraction;
    m_stDmmConf[ALARM_NO_DRIVER].bDmsAudioEnable        =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioNoDriver;
    m_stDmmConf[ALARM_SMOKE].bDmsAudioEnable            =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioSmoke;
    m_stDmmConf[ALARM_PHONE].bDmsAudioEnable            =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioPhone;
    m_stDmmConf[ALARM_YAWN].bDmsAudioEnable             =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioYawn;
    m_stDmmConf[ALARM_NO_MASK].bDmsAudioEnable          =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioNoMask;
    m_stDmmConf[ALARM_SUNGLASS].bDmsAudioEnable         =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioSunGlass;
    m_stDmmConf[ALARM_NO_SEATBELT].bDmsAudioEnable      =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioSeatBelt;
    m_stDmmConf[ALARM_SHELTER].bDmsAudioEnable          =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioShelter;
    m_stDmmConf[ALARM_FATIGUE_L2].bDmsAudioEnable       =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioFatigueL2;
    m_stDmmConf[ALARM_DRINK_EAT].bDmsAudioEnable        =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioDrinkEat;
    m_stDmmConf[ALARM_OVERSPEED].bDmsAudioEnable        =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioOverspeed;
    m_stDmmConf[ALARM_NO_HELMET].bDmsAudioEnable        =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioNoHelmet;

    
	m_stDmmInfo.pcsAlgStatus-> SetAlgParam(m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsEyelidClosure*1.0/100,
							  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsFatigueTimelimit,
							  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsDistractionAngleLeft,
							  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsDistractionAngleRight,
							  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsDistractionTimelimit,
							  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsNodriverTimelimit,
							  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsSmokeThreshold*1.0/100,
                			  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsSmokeTimelimit,
                			  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsPhoneThreshold*1.0/100,
                			  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsPhoneTimelimit,
                			  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsSeatBeltThreshold*1.0/100,
                			  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsSeatBeltTimelimit,
                			  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsShelterTimelimit,
                			  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsDrinkEatThreshold*1.0/100,
                			  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsDrinkEatTimelimit,
                			  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsYawnTimelimit,
                              m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsNoMaskTimelimit,
                              m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsSunGlassTimelimit,
                              m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsNoHelmetThreshold*1.0/100,
                              m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsNoHelmetTimelimit);
    
    if (BOARD_IsCustomer(BOARD_C_DMS31V2_202032))
    {
        s32Ret = dmm_Pwm_Init(&s32PwmFd);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "dmm_Pwm_Init failed\n");
            //return SV_FAILURE;
        }
        m_stDmmInfo.s32PwmFd = s32PwmFd;
    }
    
    return SV_SUCCESS;
}

sint32 DMM_Fini()
{
    delete m_stDmmInfo.pcsDmmDetector;
    delete m_stDmmInfo.pcsAlgStatus;
    delete m_stDmmInfo.pcsFRAnalyzer;
    pthread_mutex_destroy(&m_stDmmInfo.mutexRunStat);

    return SV_SUCCESS;
}

sint32 DMM_Start()
{
    sint32 s32Ret = 0;
    pthread_t thread;


    m_stDmmInfo.enRunStat = DMM_RUN_DETECTION;
    if (E_LOGIN_OFF != m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.enDmsLoginMode)
    {
        m_stDmmInfo.bPlayLoginAudio = SV_TRUE;
        if (E_LOGIN_BOOT == m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.enDmsLoginMode)
        {
            m_stDmmInfo.enRunStat = DMM_RUN_RECOGNITION;
        }
    }

    m_stDmmInfo.bRunning = SV_TRUE;
    s32Ret = pthread_create(&thread, NULL, dmm_alg_Body, &m_stDmmInfo);
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

    m_stDmmInfo.u32TidAlg = thread;
    
    return SV_SUCCESS;
}

sint32 DMM_Stop()
{
    sint32 s32Ret = 0;
    pthread_t thread = m_stDmmInfo.u32TidAlg;
    void *pvRetval = NULL;

    m_stDmmInfo.bRunning = SV_FALSE;
    s32Ret = pthread_join(thread, &pvRetval);
    if (0 != s32Ret)
    {
        print_level(SV_ERROR, "pthread_join failed. [err: %s]\n", strerror(errno));
        return SV_FAILURE;
    }

    return SV_SUCCESS;
}

sint32 DMM_ConfigSet(CFG_ALG_PARAM *pstCfgParam)
{
    sint32 s32Ret = 0, i = 0;
    PWM_DUTY_CYCLE_S stPwmDutyCycle = {0};
    if (NULL == pstCfgParam)
    {
        return ERR_NULL_PTR;
    }

    pthread_mutex_lock(&m_stDmmInfo.mutexRunStat);

    if (BOARD_IsCustomer(BOARD_C_DMS31V2_202032) && m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsPwmDutyCycle != pstCfgParam->stAlgCh2.stDmsParam.s32DmsPwmDutyCycle)
    {
        m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsPwmDutyCycle = pstCfgParam->stAlgCh2.stDmsParam.s32DmsPwmDutyCycle;
        stPwmDutyCycle.ton = 2500 * m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsPwmDutyCycle;
        s32Ret = ioctl(m_stDmmInfo.s32PwmFd, PWM_SET_DUTY_CYCLE, &stPwmDutyCycle);  // 客户要求PWM频率为4kHz，也就是周期为250000ns
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "pwm_ioctl %d error[%d]\n", _IOC_NR(PWM_SET_DUTY_CYCLE), s32Ret);
        }
    }
    
    m_stDmmConf[ALARM_FATIGUE].bIntervalChanged = (m_stDmmConf[ALARM_FATIGUE].s32DectectInterval != pstCfgParam->stAlgCh2.stDmsParam.s32FatigueInterval * 1000);
    m_stDmmConf[ALARM_DISTRACTION].bIntervalChanged = (m_stDmmConf[ALARM_DISTRACTION].s32DectectInterval != pstCfgParam->stAlgCh2.stDmsParam.s32DistractionInterval * 1000);
    m_stDmmConf[ALARM_NO_DRIVER].bIntervalChanged = (m_stDmmConf[ALARM_NO_DRIVER].s32DectectInterval != pstCfgParam->stAlgCh2.stDmsParam.s32NoDriverInterval * 1000);
    m_stDmmConf[ALARM_SMOKE].bIntervalChanged = (m_stDmmConf[ALARM_SMOKE].s32DectectInterval != pstCfgParam->stAlgCh2.stDmsParam.s32SmokeInterval * 1000);
    m_stDmmConf[ALARM_PHONE].bIntervalChanged = (m_stDmmConf[ALARM_PHONE].s32DectectInterval != pstCfgParam->stAlgCh2.stDmsParam.s32PhoneInterval * 1000);
    m_stDmmConf[ALARM_YAWN].bIntervalChanged = (m_stDmmConf[ALARM_YAWN].s32DectectInterval != pstCfgParam->stAlgCh2.stDmsParam.s32YawnInterval * 1000);
    m_stDmmConf[ALARM_NO_MASK].bIntervalChanged = (m_stDmmConf[ALARM_NO_MASK].s32DectectInterval != pstCfgParam->stAlgCh2.stDmsParam.s32NoMaskInterval * 1000);
    m_stDmmConf[ALARM_SUNGLASS].bIntervalChanged = (m_stDmmConf[ALARM_SUNGLASS].s32DectectInterval != pstCfgParam->stAlgCh2.stDmsParam.s32SunGlassInterval * 1000);
    m_stDmmConf[ALARM_NO_SEATBELT].bIntervalChanged = (m_stDmmConf[ALARM_NO_SEATBELT].s32DectectInterval != pstCfgParam->stAlgCh2.stDmsParam.s32SeatBeltInterval * 1000);
    m_stDmmConf[ALARM_SHELTER].bIntervalChanged = (m_stDmmConf[ALARM_SHELTER].s32DectectInterval != pstCfgParam->stAlgCh2.stDmsParam.s32ShelterInterval * 1000);
    m_stDmmConf[ALARM_FATIGUE_L2].bIntervalChanged = (m_stDmmConf[ALARM_FATIGUE_L2].s32DectectInterval != pstCfgParam->stAlgCh2.stDmsParam.s32FatigueL2Interval * 1000);
    m_stDmmConf[ALARM_DRINK_EAT].bIntervalChanged = (m_stDmmConf[ALARM_DRINK_EAT].s32DectectInterval != pstCfgParam->stAlgCh2.stDmsParam.s32DrinkEatInterval * 1000);
    m_stDmmConf[ALARM_OVERSPEED].bIntervalChanged = (m_stDmmConf[ALARM_OVERSPEED].s32DectectInterval != pstCfgParam->stAlgCh2.stDmsParam.s32OverspeedInterval * 1000);
    m_stDmmConf[ALARM_NO_HELMET].bIntervalChanged = (m_stDmmConf[ALARM_NO_HELMET].s32DectectInterval != pstCfgParam->stAlgCh2.stDmsParam.s32NoHelmetInterval * 1000);
    m_stDmmConf[NOTIFY_LNGIN_CHANGE_GUARD].bIntervalChanged = (m_stDmmConf[NOTIFY_LNGIN_CHANGE_GUARD].s32DectectInterval != pstCfgParam->stAlgCh2.stDmsParam.s32ChangeGuardInterval * 1000);

    m_stDmmInfo.stCfgParam = *pstCfgParam;

    for (i = 0; i < ALARM_DMS_BUFF; i++)
    {
        m_stDmmConf[i].s32WorkSpeed = m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsWorkspeed[i];
        /* CREARE客户遮挡当做无司机来处理 */
        if (BOARD_IsCustomer(BOARD_C_DMS31V2_CREARE) && i == ALARM_SHELTER)
        {
            m_stDmmConf[i].s32WorkSpeed = m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsWorkspeed[ALARM_NO_DRIVER];
        }
    }

    m_stDmmConf[ALARM_FATIGUE].s32DectectInterval             =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32FatigueInterval * 1000;
    m_stDmmConf[ALARM_DISTRACTION].s32DectectInterval         =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DistractionInterval * 1000;
    m_stDmmConf[ALARM_NO_DRIVER].s32DectectInterval           =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32NoDriverInterval * 1000;
    m_stDmmConf[ALARM_SMOKE].s32DectectInterval               =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32SmokeInterval * 1000;
    m_stDmmConf[ALARM_PHONE].s32DectectInterval               =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32PhoneInterval * 1000;
    m_stDmmConf[ALARM_YAWN].s32DectectInterval                =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32YawnInterval * 1000;
    m_stDmmConf[ALARM_NO_MASK].s32DectectInterval             =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32NoMaskInterval * 1000;
    m_stDmmConf[ALARM_SUNGLASS].s32DectectInterval            =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32SunGlassInterval * 1000;
    m_stDmmConf[ALARM_NO_SEATBELT].s32DectectInterval         =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32SeatBeltInterval * 1000;
    m_stDmmConf[ALARM_SHELTER].s32DectectInterval             =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32ShelterInterval * 1000;
    m_stDmmConf[ALARM_FATIGUE_L2].s32DectectInterval          =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32FatigueL2Interval * 1000;
    m_stDmmConf[ALARM_DRINK_EAT].s32DectectInterval           =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DrinkEatInterval * 1000;
    m_stDmmConf[ALARM_OVERSPEED].s32DectectInterval           =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32OverspeedInterval * 1000;
    m_stDmmConf[ALARM_NO_HELMET].s32DectectInterval           =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32NoHelmetInterval * 1000;
    m_stDmmConf[NOTIFY_LNGIN_CHANGE_GUARD].s32DectectInterval =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32ChangeGuardInterval * 1000;

    m_stDmmConf[ALARM_FATIGUE].bDmsAlarmOut             =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutFatigue;
    m_stDmmConf[ALARM_DISTRACTION].bDmsAlarmOut         =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutDistraction;
    m_stDmmConf[ALARM_NO_DRIVER].bDmsAlarmOut           =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutNoDriver;
    m_stDmmConf[ALARM_SMOKE].bDmsAlarmOut               =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutSmoke;
    m_stDmmConf[ALARM_PHONE].bDmsAlarmOut               =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutPhone;
    m_stDmmConf[ALARM_YAWN].bDmsAlarmOut                =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutYawn;
    m_stDmmConf[ALARM_NO_MASK].bDmsAlarmOut             =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutNoMask;
    m_stDmmConf[ALARM_SUNGLASS].bDmsAlarmOut            =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutSunGlass;
    m_stDmmConf[ALARM_NO_SEATBELT].bDmsAlarmOut         =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutSeatBelt;
    m_stDmmConf[ALARM_SHELTER].bDmsAlarmOut             =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutShelter;
    m_stDmmConf[ALARM_FATIGUE_L2].bDmsAlarmOut          =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutFatigueL2;
    m_stDmmConf[ALARM_DRINK_EAT].bDmsAlarmOut           =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutDrinkEat;
    m_stDmmConf[NOTIFY_LNGIN_FAILED].bDmsAlarmOut       =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutLoginFail;
    m_stDmmConf[ALARM_OVERSPEED].bDmsAlarmOut           =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutOverspeed;
    m_stDmmConf[ALARM_NO_HELMET].bDmsAlarmOut           =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAlarmOutNoHelmet;

    m_stDmmConf[ALARM_NOTHING].bDmsAudioEnable          =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioEnable;
    m_stDmmConf[ALARM_FATIGUE].bDmsAudioEnable          =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioFatigue;
    m_stDmmConf[ALARM_DISTRACTION].bDmsAudioEnable      =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioDistraction;
    m_stDmmConf[ALARM_NO_DRIVER].bDmsAudioEnable        =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioNoDriver;
    m_stDmmConf[ALARM_SMOKE].bDmsAudioEnable            =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioSmoke;
    m_stDmmConf[ALARM_PHONE].bDmsAudioEnable            =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioPhone;
    m_stDmmConf[ALARM_YAWN].bDmsAudioEnable             =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioYawn;
    m_stDmmConf[ALARM_NO_MASK].bDmsAudioEnable          =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioNoMask;
    m_stDmmConf[ALARM_SUNGLASS].bDmsAudioEnable         =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioSunGlass;
    m_stDmmConf[ALARM_NO_SEATBELT].bDmsAudioEnable      =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioSeatBelt;
    m_stDmmConf[ALARM_SHELTER].bDmsAudioEnable          =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioShelter;
    m_stDmmConf[ALARM_FATIGUE_L2].bDmsAudioEnable       =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioFatigueL2;
    m_stDmmConf[ALARM_DRINK_EAT].bDmsAudioEnable        =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioDrinkEat;
    m_stDmmConf[ALARM_OVERSPEED].bDmsAudioEnable        =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioOverspeed;
    m_stDmmConf[ALARM_NO_HELMET].bDmsAudioEnable        =  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.bDmsAudioNoHelmet;
    
	m_stDmmInfo.pcsAlgStatus->SetAlgParam(m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsEyelidClosure*1.0/100,
							  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsFatigueTimelimit,
							  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsDistractionAngleLeft,
							  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsDistractionAngleRight,
							  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsDistractionTimelimit,
							  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsNodriverTimelimit,
							  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsSmokeThreshold*1.0/100,
                			  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsSmokeTimelimit,
                			  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsPhoneThreshold*1.0/100,
                			  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsPhoneTimelimit,
                			  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsSeatBeltThreshold*1.0/100,
                			  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsSeatBeltTimelimit,
                			  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsShelterTimelimit,
                			  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsDrinkEatThreshold*1.0/100,
                			  m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsDrinkEatTimelimit,
                              m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsYawnTimelimit,
                              m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsNoMaskTimelimit,
                              m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsSunGlassTimelimit,
                              m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsNoHelmetThreshold*1.0/100,
                              m_stDmmInfo.stCfgParam.stAlgCh2.stDmsParam.s32DmsNoHelmetTimelimit);

    pthread_mutex_unlock(&m_stDmmInfo.mutexRunStat);
    
    return SV_SUCCESS;
}

sint32 DMM_SetRunStatus(DMM_RUN_E enRunStat)
{
    pthread_mutex_lock(&m_stDmmInfo.mutexRunStat);
    if (DMM_RUN_CALIBRATION == enRunStat
        || DMM_RUN_REGISTER == enRunStat
        || DMM_RUN_RECOGNITION == enRunStat)
    {
        m_stDmmInfo.enRunResult = DMM_RES_RUNNING;
    }
    m_stDmmInfo.enRunStat = enRunStat;
    if (DMM_RUN_RECOGNITION == enRunStat)
    {
        m_stDmmInfo.bPlayLoginAudio = SV_TRUE;
    }
    print_level(SV_INFO, "DMM_SetRunStatus:%d\n", m_stDmmInfo.enRunStat);
    pthread_mutex_unlock(&m_stDmmInfo.mutexRunStat);
    
    return SV_SUCCESS;
}

DMM_RUN_E DMM_GetRunStatus()
{
    return m_stDmmInfo.enRunStat;
}

DMM_RES_E DMM_GetRunResult(USER_INFO_S *pstUserInfo)
{
    if(NULL != pstUserInfo)
    {
        *pstUserInfo = m_stDmmInfo.stUserInfo;
    }
    return m_stDmmInfo.enRunResult;
}

sint32 DMM_RegisterUser(char *pszUserName)
{
    sint32 s32Ret = 0, i;
    USER_LIST_S stUserList = {0};
    char szDirOldFmt[DMM_MAX_DIRLEN];
    char szDir[DMM_MAX_DIRLEN];
    char szDirPath[128];
    char szCmd[128];
    
    if (NULL == pszUserName)
    {
        return ERR_NULL_PTR;
    }

    if (strlen(pszUserName) >= DMM_MAX_NAMELEN-1)
    {
        return ERR_ILLEGAL_PARAM;
    }

    s32Ret = DMM_GetUserList(&stUserList);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "DMM_GetUserList failed. [err=%#x]\n", s32Ret);
        return SV_FAILURE;
    }

    for (i = 0; i < stUserList.u32Num; i++)
    {
        if (0 == strcmp(stUserList.astUserList[i].szName, pszUserName))
        {
            print_level(SV_ERROR, "user name: %s is eixst.\n", pszUserName);
            return ERR_EXIST;
        }
    }

    for (i = 0; i < DMM_MAX_FR_NUM; i++)
    {
        sprintf(szDirOldFmt, "ID/user%02d", i);
		sprintf(szDir, "ID/user%03d", i);
        if (0 != strcmp(stUserList.astUserList[i].szDir, szDirOldFmt)
            && 0 != strcmp(stUserList.astUserList[i].szDir, szDir))    // 找到没有被占用的编号
        {
			sprintf(szDir, "user%03d", i);
            break;
        }
    }
    if (i >= DMM_MAX_FR_NUM)
    {
        print_level(SV_ERROR, "no free user buffer!\n");
        return ERR_BUF_FULL;
    }

    sprintf(szDirPath, "%s/%s", ALG_USERS_PATH, szDir);
    s32Ret = mkdir(szDirPath, 0755);
    if(0 != s32Ret)
    {
        print_level(SV_ERROR, "mkdir: %s failed.", szDirPath);
        return SV_FAILURE;
    }

    sprintf(szCmd, "echo \"username=%s\" > %s/username", pszUserName, szDirPath);
    s32Ret = SAFE_System(szCmd, NORMAL_WAIT_TIME);
    if(0 != s32Ret)
    {
        print_level(SV_ERROR, "cmd: %s failed.\n", szCmd);
        return SV_FAILURE;
    }
    
    strcpy(m_stDmmInfo.szUserDir, szDir);

    return SV_SUCCESS;
}

sint32 DMM_DeleteUser(char *pszUserName)
{
    sint32 s32Ret = 0, i;
    USER_LIST_S stUserList = {0};
    char szCmd[128];
    
    if (NULL == pszUserName)
    {
        return ERR_NULL_PTR;
    }

    s32Ret = DMM_GetUserList(&stUserList);
    if (SV_SUCCESS != s32Ret)
    {
        return SV_FAILURE;
    }

    for (i = 0; i < stUserList.u32Num; i++)
    {
        if (0 == strcmp(stUserList.astUserList[i].szName, pszUserName))
        {
            break;
        }
    }
    if (i >= stUserList.u32Num)
    {
        print_level(SV_ERROR, "not found user name:%s\n", pszUserName);
        return ERR_UNEXIST;
    }

    sprintf(szCmd, "rm -rf /root/%s", stUserList.astUserList[i].szDir);
    s32Ret = SAFE_System(szCmd, NORMAL_WAIT_TIME);
    if(0 != s32Ret)
    {
        print_level(SV_ERROR, "cmd: %s failed.\n", szCmd);
        return SV_FAILURE;
    }
    
    return SV_SUCCESS;
}

sint32 DMM_DeleteUserBatch(USER_LIST_S *pstDelUserList)
{
    sint32 s32Ret = 0, i, j;
    USER_LIST_S stUserList = {0};
    char szCmd[128];
    
    if (NULL == pstDelUserList)
    {
        return ERR_NULL_PTR;
    }

    s32Ret = DMM_GetUserList(&stUserList);
    if (SV_SUCCESS != s32Ret)
    {
        return SV_FAILURE;
    }

    for (i = 0; i < pstDelUserList->u32Num; i++)
    {
        for (j = 0; j < stUserList.u32Num; j++)
        {
            if (0 == strcmp(stUserList.astUserList[j].szName, pstDelUserList->astUserList[i].szName))
            {
                break;
            }
        }
        if (j >= stUserList.u32Num)
        {
            print_level(SV_ERROR, "not found user name: %s\n", pstDelUserList->astUserList[i].szName);
            continue;
        }

        sprintf(szCmd, "rm -rf /root/%s", stUserList.astUserList[j].szDir);
        s32Ret = SAFE_System(szCmd, NORMAL_WAIT_TIME);
        if(0 != s32Ret)
        {
            print_level(SV_ERROR, "cmd: %s failed.\n", szCmd);
            return SV_FAILURE;
        }
    }
    
    return SV_SUCCESS;
}

sint32 DMM_ModifyUser(char *pszUserName)
{
    sint32 s32Ret = 0, i;
    USER_LIST_S stUserList = {0};
    char szCmd[128];
    
    if (NULL == pszUserName)
    {
        return ERR_NULL_PTR;
    }

    s32Ret = DMM_GetUserList(&stUserList);
    if (SV_SUCCESS != s32Ret)
    {
        return SV_FAILURE;
    }

	printf("!!!!!!!!!! %s\n",pszUserName);
	char* tmp = strchr(pszUserName,'/');
	if(!tmp){
		print_level(SV_ERROR, "wrong name:%s with no / \n", pszUserName);
        return SV_FAILURE;
	}
	*tmp='\0';
	char* pszNewUserName = tmp+1;
	printf("~~~~~~~~ %s %s\n",pszUserName,pszNewUserName);

    for (i = 0; i < stUserList.u32Num; i++)
    {
        if (0 == strcmp(stUserList.astUserList[i].szName, pszUserName))
        {
            break;
        }
    }
    if (i >= stUserList.u32Num)
    {
        print_level(SV_ERROR, "not found user name:%s\n", pszUserName);
        return ERR_UNEXIST;
    }

	sprintf(szCmd, "echo \"username=%s\" > /root/%s/username", pszNewUserName, stUserList.astUserList[i].szDir);
    s32Ret = SAFE_System(szCmd, NORMAL_WAIT_TIME);
    if(0 != s32Ret)
    {
        print_level(SV_ERROR, "cmd: %s failed.\n", szCmd);
        return SV_FAILURE;
    }
    
    return SV_SUCCESS;
}


sint32 DMM_GetUserList(USER_LIST_S *pstUserList)
{
    sint32 s32Ret = 0, i;
    sint32 s32Num = 0;
    sint32 s32Fd = -1;
    uint32 u32UserCnt = 0;
    struct dirent **pastFileList;
    char szUserNamePath[128];
    char szUserName[1024];
    
    if (NULL == pstUserList)
    {
        return ERR_NULL_PTR;
    }

    s32Num = scandir(ALG_USERS_PATH, &pastFileList, 0, alphasort);
    if (s32Num < 0)
    {
        mkdir(ALG_USERS_PATH, 0755);
        pstUserList->u32Num = 0;
        return SV_SUCCESS;
    }

    for (i = 0; i < s32Num; i++)
    {
        if (!(pastFileList[i]->d_type & DT_DIR) || NULL == strstr(pastFileList[i]->d_name, "user"))
        {
            free(pastFileList[i]);
            continue;
        }

        sprintf(szUserNamePath, "%s/%s/username", ALG_USERS_PATH, pastFileList[i]->d_name);
        s32Fd = open(szUserNamePath, O_RDONLY);
        if (s32Fd < 0)
        {
            print_level(SV_ERROR, "open file: %s failed. [err:%s]\n", szUserNamePath, strerror(errno));
            free(pastFileList[i]);
            continue;
        }

        s32Ret = read(s32Fd, szUserName, 1024);
        if (s32Ret < 0)
        {
            print_level(SV_ERROR, "read file: %s failed. [err:%s]\n", szUserNamePath, strerror(errno));
            free(pastFileList[i]);
            close(s32Fd);
            continue;
        }
        close(s32Fd);

        cutLineBreak(szUserName);
        //print_level(SV_DEBUG, "szName:%s, szDir:%s\n", szUserName, pastFileList[i]->d_name);
        strcpy(pstUserList->astUserList[u32UserCnt].szName, &szUserName[strlen("username=")]);
        snprintf(pstUserList->astUserList[u32UserCnt].szDir,DMM_MAX_DIRLEN,"%s/%s",FRS_USERINFOS_DIR,pastFileList[i]->d_name);
        u32UserCnt++;
        free(pastFileList[i]);
    }

    free(pastFileList);
    pstUserList->u32Num = u32UserCnt;

    return SV_SUCCESS;
}

sint32 DMM_GetUserListDelFormJson(USER_LIST_S *pstUserList)
{
    sint32 s32Ret = 0, i;
    sint32 s32Fd = -1;
    uint32 u32UserCnt = 0;
    uint32 u32Size = 0;
    char szBuf[25*1024] = {0};
    cJSON *pstJson, *pstTmp, *pstDeleteUserList = NULL;

    
    if (access(DUMP_INFO_DEL_USER_LIST, F_OK) != 0)
    {
        print_level(SV_ERROR, "file %s is not exist!\n", DUMP_INFO_DEL_USER_LIST);
        goto error_exit;
    }
    
    s32Fd = open(DUMP_INFO_DEL_USER_LIST, O_RDONLY);
    if (s32Fd < 0)
    {
        print_level(SV_ERROR, "open file %s error!\n", DUMP_INFO_DEL_USER_LIST);
        goto error_exit;
    }
    
    s32Ret = read(s32Fd, szBuf, sizeof(szBuf));
    if (s32Ret < 0)
    {
        print_level(SV_ERROR, "read file %s error!\n", DUMP_INFO_DEL_USER_LIST);
        goto error_exit;
    }

    pstJson = cJSON_Parse(szBuf);
    if (NULL == pstJson)
    {
        print_level(SV_ERROR, "cJSON_Parse failed.\n");
        goto error_exit;
    }

    pstDeleteUserList = cJSON_GetObjectItemCaseSensitive(pstJson, "userList");
    if (NULL != pstDeleteUserList)
    {
        u32Size = cJSON_GetArraySize(pstDeleteUserList);
        for (i = 0; i < u32Size; i++)
        {
            pstTmp = cJSON_GetArrayItem(pstDeleteUserList, i);
            if (NULL != pstTmp)
            {
                strcpy(pstUserList->astUserList[u32UserCnt].szName, pstTmp->valuestring);
                u32UserCnt++;
            }
        }
    }

    close(s32Fd);
    cJSON_Delete(pstJson);
    pstUserList->u32Num = u32UserCnt;
    return SV_SUCCESS;

error_exit:
    if (-1 != s32Fd)
    {
        close(s32Fd);
    }
    return SV_FAILURE;
}

