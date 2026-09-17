/******************************************************************************
Copyright (C) 2021-2023 广州敏视数码科技有限公司版权所有.

文件名：alg.cpp

日期: 2021-08-03

文件功能描述: 定义算法进程入口程序

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
#include "../../include/board.h"
#include "op.h"
#include "msg.h"
#include "log.h"
#include "dmm.h"
#include "pd.h"
#include "apc.h"
#include "adas.h"
#include "alarm.h"
#include "unsocket.h"
#include "gps.h"
#include "zoom.h"
#if (defined(BOARD_ADA42PTZV1))
#include "track.h"
#endif
int ipsys_log_level = SV_DEBUG;
#if ALG_MUTLIT_BUFFER
// 各物理通道的Media Buffer的文件描述符 // The file descriptor of the Media Buffer of each physical channel 
int g_as32MediaBufFd[4][3] = 
    {
        {-1, -1, -1},
        {-1, -1, -1},
        {-1, -1, -1},
        {-1, -1, -1}
    };
#else
int g_as32MediaBufFd[4] = {-1, -1, -1, -1}; // 各物理通道的Media Buffer的文件描述符 // The file descriptor of the Media Buffer of each physical channel 
#endif
int g_s32SocketFd = -1;
pthread_mutex_t g_Mutex;            // 锁住计算资源,保证同一时刻只跑一个算法  // Lock computing resources to ensure that only one algorithm is run at the same time 


sint32 ALG_Init()
{
    sint32 s32Ret;
    s32Ret = pthread_mutex_init(&g_Mutex, NULL);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_INFO, "pthread_mutex_Init fail! [err=%#x]\n", s32Ret);
        return s32Ret;
    }
    return SV_SUCCESS;
}

sint32 ALG_Fini()
{
    sint32 s32Ret;
    s32Ret = pthread_mutex_destroy(&g_Mutex);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_INFO, "pthread_mutex_Init fail! [err=%#x]\n", s32Ret);
        return s32Ret;
    }
    return SV_SUCCESS;
}

sint32 ALG_Calculate_Lock()
{
    sint32 s32Ret;
    s32Ret = pthread_mutex_lock(&g_Mutex);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_INFO, "pthread_mutex_lock fail! [err=%#x]\n", s32Ret);
        return s32Ret;
    }
    return SV_SUCCESS;
}

sint32 ALG_Calculate_unLock()
{
    sint32 s32Ret;
    s32Ret = pthread_mutex_unlock(&g_Mutex);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_INFO, "pthread_mutex_lock fail! [err=%#x]\n", s32Ret);
        return s32Ret;
    }
    return SV_SUCCESS;
}


static sint32 callbackConfigUpdate(MSG_PACKET_S *pstMsgPkt, MSG_PACKET_S *pstRetPkt)
{

    sint32 s32Ret = 0;
    CFG_MEDIA_PARAM stMediaParam = {0};
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


    s32Ret = CONFIG_GetMediaParam(&stMediaParam);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "CONFIG_GetMediaParam failed. [err=%#x]\n", s32Ret);
        return MSG_DEFAULT_FAIL;
    }
    stAlgParam.bImageMirror = stMediaParam.astChnParam[0].bImageMirror;


    s32Ret = CONFIG_GetSystemParam(&stSysParam);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "CONFIG_GetAlgParam failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }

    print_level(SV_INFO, "stSysParam.enLang:%d\n", stSysParam.enLang);

    stAlarmParam.enLanguage = stSysParam.enLang;
    stAlarmParam.aenPdsAudioType[0] = stAlgParam.stAlgCh1.stPdsParam.enAudioType;
    stAlarmParam.aenPdsAudioType[1] = stAlgParam.stAlgCh2.stPdsParam.enAudioType;
    stAlarmParam.aenPdsAudioType[2] = stAlgParam.stAlgCh3.stPdsParam.enAudioType;
    stAlarmParam.enAdasAudioType = stAlgParam.stAlgCh1.stAdasParam.enAudioType;
    stAlarmParam.enDmsAudioType = stAlgParam.stAlgCh2.stDmsParam.enAudioType;
    stAlarmParam.s32AudioVolume = stAlgParam.s32AudioVolume;
    s32Ret = ALARM_ConfigSet(&stAlarmParam);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "DMM_Init failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }

#if (defined(BOARD_DMS31V2) || defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1) || defined(BOARD_ADA47V1))
    if(BOARD_GetVersion() != BOARD_ADA47V1_V1)
    {
        s32Ret = DMM_ConfigSet(&stAlgParam);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "DMM_ConfigSet failed. [err=%#x]\n", s32Ret);
            return MSG_DEFAULT_FAIL;
        }
    }
#endif

#if (defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1) || defined(BOARD_ADA32V2) || defined(BOARD_ADA32V3) || defined(BOARD_ADA32IR) || defined(BOARD_ADA32N1) || defined(BOARD_ADA47V1) || defined(BOARD_ADA32C4))
    if(BOARD_GetVersion() != BOARD_ADA47V1_V2)
    {
        s32Ret = PD_ConfigSet(&stAlgParam);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "DMM_ConfigSet failed. [err=%#x]\n", s32Ret);
            return MSG_DEFAULT_FAIL;
        }
    }
#endif

#if (defined(BOARD_ADA900V1))
    s32Ret = APC_ConfigSet(&stAlgParam);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "DMM_ConfigSet failed. [err=%#x]\n", s32Ret);
        return MSG_DEFAULT_FAIL;
    }
#endif

#if (defined(BOARD_ADA32V2) || defined(BOARD_ADA32N1) || defined(BOARD_ADA47V1) || defined(BOARD_ADA32C4))
    s32Ret = PD_RotateSet((stMediaParam.astChnParam[0].enRotateAngle == SV_ROTATION_90 || stMediaParam.astChnParam[0].enRotateAngle == SV_ROTATION_270) ? SV_TRUE : SV_FALSE);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "PD_RotateSet failed. [err=%#x]\n", s32Ret);
        return MSG_DEFAULT_FAIL;
    }
#endif

    return SV_SUCCESS;
}

static sint32 callbackCalibration(MSG_PACKET_S *pstMsgPkt, MSG_PACKET_S *pstRetPkt)
{
    sint32 s32Ret = DMM_RES_SUCCESS;
#if (defined(BOARD_DMS31V2) || defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1) || defined(BOARD_ADA47V1))
    DMM_RUN_E enRunStat;
    
    print_level(SV_INFO, "recive: OP_REQ_ALG_CALIBRATION %d\n", pstMsgPkt->stMsg.s32Param);
    enRunStat = DMM_GetRunStatus();
    switch (pstMsgPkt->stMsg.s32Param)
    {
        case 0:
            if (enRunStat == DMM_RUN_CALIBRATION_PRE || enRunStat == DMM_RUN_CALIBRATION)
            {
                DMM_SetRunStatus(DMM_RUN_DETECTION);
            }
            break;
        case 1:
            if (enRunStat != DMM_RUN_CALIBRATION_PRE)
            {
                DMM_SetRunStatus(DMM_RUN_CALIBRATION_PRE);
            }
            break;
        case 2:
            if (enRunStat != DMM_RUN_CALIBRATION)
            {
                sint32 s32Result;
                sint32 s32QueryTimes = 0;
                DMM_SetRunStatus(DMM_RUN_CALIBRATION);

                while(DMM_RUN_CALIBRATION == DMM_GetRunStatus() && DMM_GetRunResult(NULL) == DMM_RES_RUNNING)
                {
                    if(s32QueryTimes > 140)
                    {
                        print_level(SV_WARN, "calibration timeout!\n");
                        break;
                    }
                    s32QueryTimes++;
                    sleep_ms(100);
                }

                s32Ret = DMM_GetRunResult(NULL);
            }
            break;
    }
#endif

    return s32Ret;
}


static sint32 callbackQRCodeCalibration(MSG_PACKET_S * pstMsgPkt, MSG_PACKET_S * pstRetPkt)
{
#if (defined(BOARD_ADA32V2) || defined(BOARD_ADA32IR) || defined(BOARD_ADA32N1) || defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1) || defined(BOARD_ADA47V1) || defined(BOARD_ADA32C4))
    sint32 s32Ret = 0, i;
    sint32 s32Chn = 0;
    MSG_QRCode_CFG *pstQRCodeCfg = (MSG_QRCode_CFG *)pstMsgPkt->pu8Data;
    s32Ret = pd_QRCodeCaliberation(s32Chn, pstQRCodeCfg->QRCode1, pstQRCodeCfg->QRCode2);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "pd_QRCodeCaliberation failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }

#endif
    return SV_SUCCESS;
}

static sint32 callbackPointImageToReal(MSG_PACKET_S * pstMsgPkt, MSG_PACKET_S * pstRetPkt)
{
#if (defined(BOARD_ADA32V2) || defined(BOARD_ADA32IR) || defined(BOARD_ADA32N1) || defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1) || defined(BOARD_ADA47V1) || defined(BOARD_ADA32C4))
    sint32 s32Ret = 0, i;
    MSG_POINT_IMAGE *pstPointImage = (MSG_POINT_IMAGE *)pstMsgPkt->pu8Data;
    MSG_POINT_REAL  *pstPointReal = (MSG_POINT_REAL *)pstRetPkt->pu8Data;

    for(i = 0; i < pstPointImage->u32Num; i++)
    {
        s32Ret = pd_PointImageToReal(&pstPointImage->stPoint[i], &pstPointReal->stPoint[i], pstMsgPkt->stMsg.s32Param);
        if(s32Ret != SV_SUCCESS)
        {
            print_level(SV_ERROR, "pd_PointImageToReal failed. [err=%#x]\n", s32Ret);
        }
    }

    pstRetPkt->u32Size = sizeof(MSG_POINT_REAL);
#endif
    return SV_SUCCESS;
}

static sint32 callbackPointRealToImage(MSG_PACKET_S * pstMsgPkt, MSG_PACKET_S * pstRetPkt)
{
#if (defined(BOARD_ADA32V2) || defined(BOARD_ADA32IR) || defined(BOARD_ADA32N1) || defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1) || defined(BOARD_ADA47V1) || defined(BOARD_ADA32C4))
    sint32 s32Ret = 0, i;
    MSG_POINT_REAL  *pstPointReal = (MSG_POINT_REAL *)pstMsgPkt->pu8Data;
    MSG_POINT_IMAGE *pstPointImage = (MSG_POINT_IMAGE *)pstRetPkt->pu8Data;
    
    for(i = 0; i < pstPointImage->u32Num; i++)
    {
        s32Ret = pd_PointRealToImage(&pstPointReal->stPoint[i], &pstPointImage->stPoint[i], pstMsgPkt->stMsg.s32Param);
        if(s32Ret != SV_SUCCESS)
        {
            print_level(SV_ERROR, "pd_PointRealToImage failed. [err=%#x]\n", s32Ret);
        }
    }

    pstRetPkt->u32Size = sizeof(MSG_POINT_IMAGE);
#endif
    return SV_SUCCESS;
}

static sint32 callbackAddUser(MSG_PACKET_S *pstMsgPkt, MSG_PACKET_S *pstRetPkt)
{
#if (defined(BOARD_DMS31V2) || defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1) || defined(BOARD_ADA47V1))
    sint32 s32Ret = 0, i;
    DMM_RES_E enRes = 0;
    USER_INFO_S stUserInfo = {0};

    ALARM_CFG_PARAM_S stAlarmParam = {0};


    print_level(SV_INFO, "recive: OP_REQ_ALG_FR_ADDUSER: %s\n", (char*)pstMsgPkt->pu8Data);
    s32Ret = DMM_RegisterUser((char*)pstMsgPkt->pu8Data);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "DMM_RegisterUser failed. [err=%#x]\n", s32Ret);
        if (ERR_ILLEGAL_PARAM == s32Ret)
        {
            return MSG_RANGE_OUT;
        }
        else if (ERR_EXIST == s32Ret)
        {
            return MSG_RES_EXIST;
        }
        else if (ERR_BUF_FULL == s32Ret)
        {
            return MSG_NO_ENOUGH_MEMORY;
        }
        else
        {
            return MSG_DEFAULT_FAIL;
        }
    }

    DMM_SetRunStatus(DMM_RUN_REGISTER);
    for (i = 0; i < 20; i++)
    {
        enRes = DMM_GetRunResult(&stUserInfo);
        if (DMM_RES_RUNNING != enRes)
        {
            break;
        }
        sleep_ms(1000);
    }
    if (i >= 20 || enRes != DMM_RES_SUCCESS)
    {
        DMM_DeleteUser((char*)pstMsgPkt->pu8Data);
        return SV_FAILURE;
    }
#endif

    return SV_SUCCESS;
}

static sint32 callbackDelUser(MSG_PACKET_S *pstMsgPkt, MSG_PACKET_S *pstRetPkt)
{
#if (defined(BOARD_DMS31V2) || defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1) || defined(BOARD_ADA47V1))
    sint32 s32Ret = 0;
    USER_LIST_S stDelUserList = {0};

    print_level(SV_INFO, "recive: OP_REQ_ALG_FR_DELUSER, s32Param: %d\n", pstMsgPkt->stMsg.s32Param);

    if (0 == pstMsgPkt->stMsg.s32Param)
    {
        s32Ret = DMM_DeleteUser((char*)pstMsgPkt->pu8Data);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "DMM_DeleteUser failed. [err=%#x]\n", s32Ret);
            if (ERR_UNEXIST == s32Ret)
            {
                return MSG_RES_EXIST;
            }
            else
            {
                return MSG_DEFAULT_FAIL;
            }
        }
    }
    else if (1 == pstMsgPkt->stMsg.s32Param)
    {
        s32Ret = DMM_GetUserListDelFormJson(&stDelUserList);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "DMM_GetUserListDelFormJson failed.\n");
            return MSG_DEFAULT_FAIL;
        }

        s32Ret = DMM_DeleteUserBatch(&stDelUserList);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "DMM_DeleteUserBatch failed. [err=%#x]\n", s32Ret);
            if (ERR_UNEXIST == s32Ret)
            {
                return MSG_RES_EXIST;
            }
            else
            {
                return MSG_DEFAULT_FAIL;
            }
        }
    }
    else
    {
        print_level(SV_ERROR, "not support opcode!\n");
        return SV_FAILURE;
    }
#endif

    return SV_SUCCESS;
}

static sint32 callbackModifyUser(MSG_PACKET_S *pstMsgPkt, MSG_PACKET_S *pstRetPkt)
{
#if (defined(BOARD_DMS31V2) || defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1) || defined(BOARD_ADA47V1))
    sint32 s32Ret = 0;

    print_level(SV_INFO, "recive: OP_REQ_ALG_FR_MODUSER: %s\n", (char*)pstMsgPkt->pu8Data);
    s32Ret = DMM_ModifyUser((char*)pstMsgPkt->pu8Data);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "DMM_DeleteUser failed. [err=%#x]\n", s32Ret);
        if (ERR_UNEXIST == s32Ret)
        {
            return MSG_RES_EXIST;
        }
        else
        {
            return MSG_DEFAULT_FAIL;
        }
    }
#endif

    return SV_SUCCESS;
}


static sint32 callbackGetUsers(MSG_PACKET_S *pstMsgPkt, MSG_PACKET_S *pstRetPkt)
{
#if (defined(BOARD_DMS31V2) || defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1) || defined(BOARD_ADA47V1))
    sint32 s32Ret = 0;
    USER_LIST_S *pstUserList = (USER_LIST_S *)pstRetPkt->pu8Data;

    print_level(SV_INFO, "recive: OP_REQ_ALG_FR_GETUSERS\n");
    s32Ret = DMM_GetUserList(pstUserList);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "DMM_GetUserList failed. [err=%#x]\n", s32Ret);
        return MSG_DEFAULT_FAIL;
    }

    pstRetPkt->u32Size = sizeof(USER_LIST_S);
#endif

    return SV_SUCCESS;
}


static sint32 callbackLogin(MSG_PACKET_S *pstMsgPkt, MSG_PACKET_S *pstRetPkt)
{	
#if (defined(BOARD_DMS31V2) || defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1) || defined(BOARD_ADA47V1))
    sint32 s32Ret = 0, i, warnState;
    DMM_RES_E enRes = 0;
    USER_INFO_S stUserInfo = {0};
    USER_INFO_S *pstUserInfo = (USER_INFO_S *)pstRetPkt->pu8Data;

    print_level(SV_INFO, "recive: OP_REQ_ALG_FR_LOGIN\n");
    DMM_SetRunStatus(DMM_RUN_RECOGNITION);
    for (i = 0; i < 20; i++)
    {
        enRes = DMM_GetRunResult(&stUserInfo);
        if (DMM_RES_RUNNING != enRes)
        {
            break;
        }
        sleep_ms(1000);
    }
	
	if (i >= 20)
	{
		print_level(SV_WARN, "login recognition timeout!\n");
		enRes = DMM_RES_TIMEOUT;
	}

	if (enRes == DMM_RES_SUCCESS)
	{
		warnState = NOTIFY_LNGIN_SUCCESS;
		pstRetPkt->u32Size = sizeof(USER_INFO_S);
		*pstUserInfo = stUserInfo;
	}
	else
	{
		warnState = NOTIFY_LNGIN_FAILED;
	}

	if (DMM_RES_TIMEOUT == enRes)
		return MSG_TIMEOUT_FAIL;
	else if (DMM_RES_FAILURE == enRes)
		return MSG_DATA_UNMATCH;
#endif

    return MSG_SUCCESS_RES;
}
static sint32 callbackSplitNotify(MSG_PACKET_S *pstMsgPkt, MSG_PACKET_S *pstRetPkt)
{
    sint32 s32Ret = 0;
    MSG_VIDEO_CFG *pstVideoCfg = (MSG_VIDEO_CFG *)pstMsgPkt->pu8Data;
    SPLIT_MODE enSplitMode = pstVideoCfg->enVoSplitMode;
#if (defined(BOARD_ADA32IR))
    s32Ret = PD_SplitSet(enSplitMode);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "callbackSplitNotify failed \n");
    }
#endif
    return MSG_SUCCESS_RES;
}

static sint32 callbackPtzGetTrackPos(MSG_PACKET_S *pstMsgPkt, MSG_PACKET_S *pstRetPkt)
{
    sint32 s32Ret = 0;
    PTZ_POS_S *pstTrackPos = (PTZ_POS_S *)pstMsgPkt->pu8Data;

#if (defined(BOARD_ADA42PTZV1))
    s32Ret = TrackPosUpdate(*pstTrackPos);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "callbackSplitNotify failed \n");
    }
#endif
    return MSG_SUCCESS_RES;

}

static sint32 callbackUpdateGpsData(MSG_PACKET_S *pstMsgPkt, MSG_PACKET_S *pstRetPkt)
{	
#if (defined(BOARD_DMS31V2) || defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1) || defined(BOARD_ADA47V1))
    sint32 s32Ret = 0;
	GPS_DATA_S stGpsData = {0};
	memcpy(&stGpsData, pstMsgPkt->pu8Data, sizeof(GPS_DATA_S));
#endif

    return SV_SUCCESS;
}

/******************************************************************************
 * 函数功能: 隐藏图片流
 * 输入参数: 无
 * 输出参数: 无
 * 返回值  : SV_SUCCESS - 成功
             其他 - 其它错误
 * 注意    : 无
 *****************************************************************************/
sint32 ALG_Hide_Picstream()
{
    sint32 s32Ret;
    MSG_VIDEO_CFG stVideoCfg = {0};
    MSG_PACKET_S stMsgPkt = {0}, stRetPkt = {0};
    stRetPkt.pu8Data = (uint8 *)&stVideoCfg;
    s32Ret = Msg_execRequestBlock(EP_ALG, EP_CONTROL, OP_REQ_GET_VIDEO_CFG, NULL, &stRetPkt, sizeof(MSG_VIDEO_CFG));
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "OP_REQ_GET_VIDEO_CFG failed. [err=%#x]\n", s32Ret);
        return MSG_DEFAULT_FAIL;
    }

    if(!(stVideoCfg.astChnParam[0].bShowGuiMask & 0b0100))
    {
        return SV_SUCCESS;
    }

    stVideoCfg.astChnParam[0].bShowGuiMask &= 0b1011;

    stMsgPkt.pu8Data = (uint8 *)&stVideoCfg;
    stMsgPkt.u32Size = sizeof(MSG_VIDEO_CFG);
    //print_level(SV_WARN, "stMsgPkt.u32Size = %d\n", stMsgPkt.u32Size);
    s32Ret = Msg_execRequestBlock(EP_ALG, EP_CONTROL, OP_REQ_SET_VIDEO_CFG, &stMsgPkt, NULL, 0);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "OP_REQ_SET_VIDEO_CFG failed.\n");
        return MSG_DEFAULT_FAIL;
    }
    
    return SV_SUCCESS;
}

/******************************************************************************
 * 函数功能: 切换AlarmOut开关
 * 输入参数: bEnable 是否打开
 * 输出参数: 无
 * 返回值  : SV_SUCCESS - 成功
             其他 - 其它错误
 * 注意    : 无
 *****************************************************************************/
sint32 ALG_AlarmOut_Switch(SV_BOOL bEnable)
{
    printf("ALG_AlarmOut_Switch, bEnable=%d\n", bEnable);
    sint32 s32Ret;
    MSG_ALG_CFG stAlgCfg = {0};
    MSG_PACKET_S stMsgPkt = {0}, stRetPkt = {0};
    stRetPkt.pu8Data = (uint8 *)&stAlgCfg;
    s32Ret = Msg_execRequestBlock(EP_ALG, EP_CONTROL, OP_REQ_GET_ALG_CFG, NULL, &stRetPkt, sizeof(MSG_ALG_CFG));
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "OP_REQ_GET_VIDEO_CFG failed. [err=%#x]\n", s32Ret);
        return MSG_DEFAULT_FAIL;
    }

    if ( stAlgCfg.stAlgCh2.stPdsParam.bPdAlarmOutRed == bEnable
      && stAlgCfg.stAlgCh2.stPdsParam.bPdAlarmOutYellow == bEnable
      && stAlgCfg.stAlgCh2.stPdsParam.bPdAlarmOutGreen  == bEnable)
    {
        return SV_SUCCESS;
    }

    stAlgCfg.stAlgCh2.stPdsParam.bPdAlarmOutRed = bEnable;
    stAlgCfg.stAlgCh2.stPdsParam.bPdAlarmOutYellow = bEnable;
    stAlgCfg.stAlgCh2.stPdsParam.bPdAlarmOutGreen = bEnable;

    stMsgPkt.pu8Data = (uint8 *)&stAlgCfg;
    stMsgPkt.u32Size = sizeof(MSG_ALG_CFG);
    //print_level(SV_WARN, "stMsgPkt.u32Size = %d\n", stMsgPkt.u32Size);
    s32Ret = Msg_execRequestBlock(EP_ALG, EP_CONTROL, OP_REQ_SET_ALG_CFG, &stMsgPkt, NULL, 0);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "OP_REQ_SET_VIDEO_CFG failed.\n");
        return MSG_DEFAULT_FAIL;
    }
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
sint32 alg_GetMediaBufFd(sint32 s32SocketFd, sint32 s32Chn, sint32 *ps32MediaBufFd)
{
    sint32 s32Ret = 0, i, idx;
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

#if ALG_MUTLIT_BUFFER
    for(idx = 0; idx < 3; idx++)
    {
        stSocketPkt.header.startode = MSG_STARTCODE;
        stSocketPkt.header.opcode = SOCKET_OP_GET_FD;
        stSocketPkt.header.params = s32Chn;
        stSocketPkt.args[0] = idx;    /* 取得第一个BUFFER */
        s32Ret = unsock_write(s32SocketFd, &stSocketPkt, sizeof(stSocketPkt));
        if (s32Ret < 0)
        {
            print_level(SV_ERROR, "unsock_write failed. [err=%#x]\n", s32Ret);
            return SV_FAILURE;
        }

        print_level(SV_INFO, "unsock_write successful. fd:%d\n", s32SocketFd);
        FD_ZERO(&read_fds);
        for (i = 0; i < 10; i++)
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

            s32Ret = unsock_recvPacket(s32SocketFd, &stSocketPkt, sizeof(stSocketPkt), &ps32MediaBufFd[idx]);
            if (s32Ret < 0 || ps32MediaBufFd[idx] < 0)
            {
                print_level(SV_ERROR, "unsock_write failed. [err=%#x, fd=%d]\n", s32Ret, ps32MediaBufFd[idx]);
                return SV_FAILURE;
            }
            else
            {
                print_level(SV_INFO, "socket recvfd:%d\n", ps32MediaBufFd[idx]);
                break;
            }
        }
        
        if (i >= 10)
        {
            print_level(SV_ERROR, "wait for unsock_recvPacket timeout.\n");
            close(s32SocketFd);
            return SV_FAILURE;
        }
    }

#else
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
#endif
    
    return SV_SUCCESS;
}

/******************************************************************************
 * 函数功能: 通过socket获取媒体分辨率（宽高）
 * 输入参数: s32SocketFd --- socket fd
               s32Chn --- 通道号
 * 输出参数: u32Width --- 宽
             u32Height --- 高
 * 返回值  : SV_SUCCESS - 成功
             SV_FAILURE - 其它错误
 * 说明    : 无
 *****************************************************************************/
sint32 alg_GetMediaRes(sint32 s32SocketFd, sint32 s32Chn, uint32 *u32Width, uint32 *u32Height)
{
    sint32 s32Ret = 0, i;
    SocketPacket stSocketPkt = {0};
    fd_set read_fds, write_fds;
    struct timeval timeout;

    if (s32SocketFd < 0 || s32Chn < 0 || s32Chn >= 4)
    {
        return ERR_ILLEGAL_PARAM;
    }

    if (NULL == u32Width || NULL == u32Height)
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
    stSocketPkt.header.opcode = SOCKET_OP_GET_RES;
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

        s32Ret = unsock_recvPacket(s32SocketFd, &stSocketPkt, sizeof(stSocketPkt), NULL);
        if (s32Ret < 0)
        {
            print_level(SV_ERROR, "unsock_write failed. [err=%#x]\n", s32Ret);
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

    if (stSocketPkt.header.len == 2*sizeof(sint32))
    {
        memcpy(u32Width, &stSocketPkt.args[0], 4);
        memcpy(u32Height, &stSocketPkt.args[4], 4);
    }

    return SV_SUCCESS;
}

/* 中断退出 */
static void exit_handle(int signalnum)
{
    sint32 s32Ret = 0;
    
    printf("catch signalnum %d!\n", signalnum);

    exit(EXIT_FAILURE);
}

void sigHandle(int sig)
{
    switch (sig) {
    case SIGINT:
		printf("*** catch signal : SIGINT, value = %d\n", sig);
        break;
    case SIGTERM:
		printf("*** catch signal : SIGTERM, value = %d\n", sig);
        //这里回收资源
        break;
  
    case SIGSEGV:
		printf("*** catch signal : SIGSEGV, value = %d\n", sig);
        break;
    case SIGFPE:
		printf("*** catch signal : SIGFPE, value = %d\n", sig);
        break;
    case SIGABRT:
		printf("*** catch signal : SIGABRT, value = %d\n", sig);
        break;
    default:
		printf("*** catch unknown signal, value = %d\n", sig);
        break;
    }
    exit(0); //调用exit退出程序，会被捕获该事件，从而触发进程退出处理的回调函数
}

int main(int argc, char **argv)
{
    sint32 s32Ret = 0, n, i, j;
    sint32 s32ChnNum;
    uint32 u32Width = 0;
    uint32 u32Height = 0;
    sint32 s32Tmp = 0;
    char *pszConfigFile = CONFIG_XML;
    char *pszConfigBak1 = CONFIG_BAK1;
    char *pszConfigBak2 = CONFIG_BAK2;
    char *pszConfigDefault = CONFIG_DEFAULT;
    CFG_ALG_PARAM stAlgParam = {0};
    CFG_SYS_PARAM stSysParam = {0};
    CFG_SER_PARAM stSerParam = {0};
    CFG_MEDIA_PARAM stMediaParam = {0};
    ADAS_CFG_PARAM_S stAdasParam = {0};
    DMM_CFG_PARAM_S stDmmParam = {0};
    PD_CFG_PARAM_S stPdParam = {0};
    APC_CFG_PARAM_S stApcParam = {0};
    ZOOM_CFG_PARAM_S stZoomParam = {0};
#if (defined(BOARD_ADA42PTZV1))    
    TARGET_TRACK_CFG_PARAM_S stTrackParam = {0};
#endif
    ALARM_CFG_PARAM_S stAlarmParam = {0};
    SocketPacket stSocketPkt = {0};
    fd_set read_fds, write_fds;
    struct timeval timeout;
    SV_BOOL bAdas = SV_FALSE;
    SV_BOOL bDms = SV_FALSE;
    SV_BOOL bPds = SV_FALSE;
    SV_BOOL bApc = SV_FALSE;
    SV_BOOL bTrack = SV_FALSE;
    CHN_ALG_E aenChnAlg[ALG_MAX_CHN] = {0};
#if ALG_MUTLIT_BUFFER
    int as32MediaBufFd[4][3] = {-1};
#else
	int as32MediaBufFd[4] = {-1, -1, -1, -1};
#endif

#if (defined(BOARD_ADA42V1))
    s32ChnNum = ALG_MAX_CHN;
#elif (defined(BOARD_ADA32IR) || defined(BOARD_ADA42PTZV1))
    s32ChnNum = 2;
#else
    s32ChnNum = 1;
#endif

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

    signal(SIGINT, sigHandle);  //Ctrl + C
    signal(SIGTERM, sigHandle); //kill发出的软件终止
    signal(SIGSEGV, sigHandle); //非法内存访问
    signal(SIGFPE, sigHandle);  //数学相关的异常，如被0除，浮点溢出等
    signal(SIGABRT, sigHandle); //由调用abort函数产生，进程非正常退出


    s32Ret = ALG_Init();
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "ALG_Init failed. [err=%#x]\n", s32Ret);
        return s32Ret;
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

#if (defined(BOARD_ADA32V2) || defined(BOARD_ADA47V1) || defined(BOARD_ADA32C4))
    s32Ret = BOARD_SetAlarmOut(stAlgParam.enAlgTrigger ==  TRIGGER_UP ? 0 : 1);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "BOARD_SetAlarmOut failed. [err=%#x]\n", s32Ret);
    }
#endif

    aenChnAlg[0] = stAlgParam.stAlgCh1.enAlgType;
    aenChnAlg[1] = stAlgParam.stAlgCh2.enAlgType;
    aenChnAlg[2] = stAlgParam.stAlgCh3.enAlgType;
 #if 0
    const int reconnect_times = 10;
    for (i = 0; i < reconnect_times; i++)
    {
        g_s32SocketFd = cli_connect(CS_PATH_CAM_STREAM, 'c');
        if (g_s32SocketFd > 0)
        {
            break;
        }
        print_level(SV_WARN, "cli_connect %s failed.\n", CS_PATH_CAM_STREAM);
        sleep_ms(1000);
    }
    if (i >= reconnect_times)
    {
        print_level(SV_ERROR, "wait for cli_connect %s timeout.\n", CS_PATH_CAM_STREAM);
        return -1;
    }

    print_level(SV_INFO, "cli_connect successful. fd:%d\n", g_s32SocketFd);
    for(n = 0; n < s32ChnNum; n++)
    {
#if (defined(BOARD_ADA42V1))
        if (ALG_OFF == aenChnAlg[n])
        {
            continue;
        }
#endif

        switch (aenChnAlg[n])
        {
            case ALG_ADAS:
                print_level(SV_INFO, "ch%d run ADAS\n", n);
                bAdas = SV_TRUE;
                break;
            case ALG_DMS:
                print_level(SV_INFO, "ch%d run DMS\n", n);
                bDms = SV_TRUE;
                break;
            case ALG_PDS:
                print_level(SV_INFO, "ch%d run PDS\n", n);
                bPds = SV_TRUE;
                break;
            case ALG_APC:
                print_level(SV_INFO, "ch%d run APC\n", n);
                bApc = SV_TRUE;
                break;
            case ALG_TRACK:
                print_level(SV_INFO, "ch%d run TRACK\n", n);
                bTrack = SV_TRUE;
                break;
        }
        
#if ALG_MUTLIT_BUFFER
        s32Ret = alg_GetMediaBufFd(g_s32SocketFd, n, g_as32MediaBufFd[n]);
#else
		s32Ret = alg_GetMediaBufFd(g_s32SocketFd, n, &g_as32MediaBufFd[n]);
#endif
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "alg_GetMediaBufFd failed.[err=%#x]\n", s32Ret);
            close(g_s32SocketFd);
            return -1;
        }

        //print_level(SV_INFO, "got media buffer fd:%d\n", g_as32MediaBufFd[n]);

#if (defined(BOARD_ADA32V3) || defined(BOARD_ADA32N1))
        s32Ret = alg_GetMediaRes(g_s32SocketFd, n, &u32Width, &u32Height);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "alg_GetMediaRes failed.[err=%#x]\n", s32Ret);
            close(g_s32SocketFd);
            return -1;
        }
        print_level(SV_INFO, "get alg w: %d, h: %d\n", u32Width, u32Height);
#endif
    }
#endif

    s32Ret = CONFIG_GetServerParam(&stSerParam);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "CONFIG_GetServerParam failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }

    s32Ret = CONFIG_GetSystemParam(&stSysParam);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "CONFIG_GetAlgParam failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }
    

    // s32Ret = CONFIG_GetMediaParam(&stMediaParam);
    // if (SV_SUCCESS != s32Ret)
    // {
    //     print_level(SV_ERROR, "CONFIG_GetMediaParam failed. [err=%#x]\n", s32Ret);
    //     return s32Ret;
    // }

    stPdParam.s32SplitMode = (sint32)stMediaParam.enVoSplitMode;
    stPdParam.bRotate = (stMediaParam.astChnParam[0].enRotateAngle == SV_ROTATION_90 || stMediaParam.astChnParam[0].enRotateAngle == SV_ROTATION_270) ? SV_TRUE : SV_FALSE;

    s32Ret = LOG_Init(&stSerParam, SV_FALSE);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "LOG_Init failed. [err=%#x]\n", s32Ret);
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
    stAlarmParam.bPlayWelcome = stAlgParam.stAlgCh2.stDmsParam.bDmsAudioWelcome;
    s32Ret = ALARM_Init(&stAlarmParam);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "ALARM_Init failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }
    
#if (defined(BOARD_ADA42V1))
    stAdasParam.s32MediaBufFd = g_as32MediaBufFd[0];
    stAdasParam.stAlgParam = stAlgParam;
    if (bAdas)
    {
        s32Ret = ADAS_Init(&stAdasParam);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "ADAS_Init failed. [err=%#x]\n", s32Ret);
            return s32Ret;
        }
    }
#endif

#if (defined(BOARD_ADA42PTZV1))
    stTrackParam.s32MediaBufFd = g_as32MediaBufFd[1];
    stTrackParam.stAlgParam = stAlgParam;
    stTrackParam.u32ChnNum = s32ChnNum;
    stTrackParam.s32MediaBufChn = 1;
    if (bTrack)
    {
        s32Ret = TARGET_TRACK_Init(&stTrackParam);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "TARGET_TRACK_Init failed. [err=%#x]\n", s32Ret);
            return s32Ret;
        }
    }    
#endif

#if (defined(BOARD_DMS31V2) || defined(BOARD_ADA42V1) || defined(BOARD_ADA47V1))

#if (defined(BOARD_DMS31V2) || defined(BOARD_ADA47V1))
#if ALG_MUTLIT_BUFFER
    for(i = 0; i < 3; i++)
    {
        stDmmParam.s32MediaBufFd[i] = g_as32MediaBufFd[0][i];
    }
#else
	stDmmParam.s32MediaBufFd = g_as32MediaBufFd[0];
#endif
    stDmmParam.s32Chn = 0;
    bDms = SV_TRUE;
    if(BOARD_GetVersion() == BOARD_ADA47V1_V1)
    {
        bDms = SV_FALSE;
    }

#if defined(BOARD_ADA47V1)
    if (stAlgParam.stAlgCh2.enAlgType != ALG_DMS)
    {
        bDms = SV_FALSE;
    }
#endif

#else

#if ALG_MUTLIT_BUFFER
    for(i = 0; i < 3; i++)
    {
        stDmmParam.s32MediaBufFd[i] = g_as32MediaBufFd[1][i];
    }
#else
	stDmmParam.s32MediaBufFd = g_as32MediaBufFd[1];
#endif
    stDmmParam.s32Chn = 1;

#endif
    stDmmParam.stAlgParam = stAlgParam;
    stDmmParam.stAlgParam.bImageMirror = stMediaParam.astChnParam[0].bImageMirror;
    if (bDms)
    {
        s32Ret = DMM_Init(&stDmmParam);
        if (SV_SUCCESS != s32Ret)
        {
#if (defined(DMS31SDK) || defined(ADA32SDK) || defined(BOARD_ADA47V1))
            goto err_loop;
#endif        
            print_level(SV_ERROR, "DMM_Init failed. [err=%#x]\n", s32Ret);
            return s32Ret;
        }
    }
#endif

#if (defined(BOARD_ADA32V2) || defined(BOARD_ADA32V3) || defined(BOARD_ADA32IR) || defined(BOARD_ADA32N1) || defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1) || defined(BOARD_ADA47V1) || defined(BOARD_ADA32C4))
#if (defined(BOARD_ADA32V2) || defined(BOARD_ADA32V3) || defined(BOARD_ADA32IR) || defined(BOARD_ADA32N1) || defined(BOARD_ADA42PTZV1) || defined(BOARD_ADA47V1) || defined(BOARD_ADA32C4))
    bPds = SV_TRUE;
    if(BOARD_GetVersion() == BOARD_ADA47V1_V2)
    {
        bPds = SV_FALSE;
    }
#endif
    stPdParam.u32ChnNum = s32ChnNum;
    for (i = 0; i < s32ChnNum; i++)
    {
#if ALG_MUTLIT_BUFFER
        for (j = 0; j < 3; j++)
        {
            stPdParam.as32MediaBufFd[i][j] = g_as32MediaBufFd[i][j];
        }
#else
		stPdParam.as32MediaBufFd[i] = g_as32MediaBufFd[i];
#endif
    }
    stPdParam.u32Width = 608;
    stPdParam.u32Height = 352;
    if(BOARD_GetVersion() == BOARD_ADA47V1_V1)
    {
        stAlgParam.stAlgCh2.enAlgType = ALG_PDS;
        stAlgParam.stAlgCh2.stPdsParam.bPdTestMode = SV_TRUE;
    }
    stPdParam.stAlgParam = stAlgParam;
    if (bPds)
    {
        s32Ret = PD_Init(&stPdParam);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "PD_Init failed. [err=%#x]\n", s32Ret);
#if (defined(DMS31SDK) || defined(ADA32SDK) || defined(BOARD_ADA47V1))
            goto err_loop;
#endif
            if (BOARD_IsNeedKeyAuth())
            {
                goto err_loop;
            }

            return s32Ret;
        }
    }
#endif

#if (defined(BOARD_ADA900V1) || defined(BOARD_ADA47V1))
    if (stAlgParam.stAlgCh2.enAlgType == ALG_APC)
    {
        bApc = SV_TRUE;
    }

    stApcParam.u32ChnNum = s32ChnNum;
    for (i = 0; i < s32ChnNum; i++)
    {

#if ALG_MUTLIT_BUFFER
        for (j = 0; j < 3; j++)
        {
            stApcParam.as32MediaBufFd[i][j] = g_as32MediaBufFd[i][j];
        }
#else
        stApcParam.as32MediaBufFd[i] = g_as32MediaBufFd[i];
#endif
    }
    stApcParam.stAlgParam = stAlgParam;

    if (bApc)
    {
        s32Ret = APC_Init(&stApcParam);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "PD_Init failed. [err=%#x]\n", s32Ret);
            return s32Ret;
        }
    }

#endif

#if (defined(BOARD_HDW845V1))
        stZoomParam.u32ChnNum = s32ChnNum;
        for (i = 0; i < s32ChnNum; i++)
        {
            stZoomParam.as32MediaBufFd[i] = g_as32MediaBufFd[i];
        }
        stZoomParam.stAlgParam = stAlgParam;
    
        s32Ret = Zoom_Init(&stZoomParam);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "Zoom_Init failed. [err=%#x]\n", s32Ret);
            return s32Ret;
        }
#endif


    s32Ret = Msg_registerOpCallback(EP_ALG, OP_EVENT_CFG_UPDATE, callbackConfigUpdate);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_registerOpCallback_ThreadExec failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }

    s32Ret = Msg_registerOpCallback(EP_ALG, OP_REQ_ALG_CALIBRATION, callbackCalibration);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_registerOpCallback failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }

    s32Ret = Msg_registerOpCallback_ThreadExec(EP_ALG, OP_REQ_ALG_QRCODE_CALIBRATION, callbackQRCodeCalibration);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_registerOpCallback failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }

    s32Ret = Msg_registerOpCallback_ThreadExec(EP_ALG, OP_REQ_ALG_POINT_IMAGE_TO_REAL, callbackPointImageToReal);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_registerOpCallback failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }

    s32Ret = Msg_registerOpCallback_ThreadExec(EP_ALG, OP_REQ_ALG_POINT_REAL_TO_IMAGE, callbackPointRealToImage);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_registerOpCallback failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }

    s32Ret = Msg_registerOpCallback_ThreadExec(EP_ALG, OP_REQ_ALG_FR_ADDUSER, callbackAddUser);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_registerOpCallback failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }

    s32Ret = Msg_registerOpCallback(EP_ALG, OP_REQ_ALG_FR_DELUSER, callbackDelUser);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_registerOpCallback failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }

    s32Ret = Msg_registerOpCallback(EP_ALG, OP_REQ_ALG_FR_MODUSER, callbackModifyUser);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_registerOpCallback failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }

    s32Ret = Msg_registerOpCallback(EP_ALG, OP_REQ_ALG_FR_GETUSERS, callbackGetUsers);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_registerOpCallback failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }

    s32Ret = Msg_registerOpCallback_ThreadExec(EP_ALG, OP_REQ_ALG_FR_LOGIN, callbackLogin);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_registerOpCallback failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }

    s32Ret = Msg_registerOpCallback_ThreadExec(EP_ALG, OP_EVENT_ALG_SPLIT_UPDATE, callbackSplitNotify);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_registerOpCallback failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }

    s32Ret = Msg_registerOpCallback_ThreadExec(EP_ALG, OP_EVENT_RESET_POS, callbackPtzGetTrackPos);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_registerOpCallback failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }

#if (defined(BOARD_DMS31V2) || defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1) || defined(BOARD_ADA47V1))
    s32Ret = Msg_registerOpCallback(EP_ALG, OP_EVENT_GPS_DATA, callbackUpdateGpsData);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_registerOpCallback failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }
#endif

#if (defined(BOARD_ADA42V1))
    if (bAdas)
    {
        s32Ret = ADAS_Start();
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "ADAS_Start failed. [err=%#x]\n", s32Ret);
            return s32Ret;
        }
    }
#endif

#if (defined(BOARD_DMS31V2) || defined(BOARD_ADA42V1) || defined(BOARD_ADA47V1))
    if (bDms)
    {
        s32Ret = DMM_Start();
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "DMM_Start failed. [err=%#x]\n", s32Ret);
            return s32Ret;
        }
    }
#endif

#if (defined(BOARD_ADA32V2) || defined(BOARD_ADA32V3) || defined(BOARD_ADA32IR) || defined(BOARD_ADA32N1) || defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1) || defined(BOARD_ADA47V1) || defined(BOARD_ADA32C4))
    if (bPds)
    {
        s32Ret = PD_Start();
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "PD_Start failed. [err=%#x]\n", s32Ret);
            return s32Ret;
        }
    }
#endif    

#if (defined(BOARD_ADA42PTZV1))
    if (bTrack)
    {
        s32Ret = TARGET_TRACK_Start();
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "TARGET_TRACK_Start failed. [err=%#x]\n", s32Ret);
            return s32Ret;
        }
    }
#endif

#if (defined(BOARD_ADA900V1) || defined(BOARD_ADA47V1))
    if (bApc)
    {
        s32Ret = APC_Start();
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "APC_Start failed. [err=%#x]\n", s32Ret);
            return s32Ret;
        }
    }
#endif

#if (defined(BOARD_HDW845V1))
        s32Ret = Zoom_Start();
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "Zoom_Start failed. [err=%#x]\n", s32Ret);
            return s32Ret;
        }
#endif

    while (1)
    {
        sleep(1);
#if (defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1))
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
            for(n = 0; n < s32ChnNum; n++)
            {
                if (ALG_OFF == aenChnAlg[n])
                {
                    continue;
                }

                s32Ret = alg_GetMediaBufFd(g_s32SocketFd, n, &as32MediaBufFd[n]);
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
#endif
    }

err_loop:
    while (1)
    {
        print_level(SV_ERROR, "auth failed...\n");
        sleep(5);
    }

    return 0;
}

