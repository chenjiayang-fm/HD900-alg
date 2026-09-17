/******************************************************************************
Copyright (C) 2023-2025 广州敏视数码科技有限公司版权所有.

文件名：main.cpp

日期: 2023-03-10

文件功能描述: 实现ADA47 CAN口、RS485接口、蓝牙接口、SWITCH IN/OUT接口自动化测试

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
#include <linux/can.h>

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
#include "cJSON.h"

/* 行人检测输出信息 */
typedef struct tagPdDumpInfo_S
{
    sint64          s64TimeStamp;           /* 时间戳 */
    sint32          s32GreenRoiNum;         /* 绿色ROI区域检测数量 */
    sint32          s32YellowRoiNum;        /* 黄色ROI区域检测数量 */
    sint32          s32RedRoiNum;           /* 红色ROI区域检测数量 */
    sint32          s32DistanceNum;           /* 目标数量 */
    sint32          s32DistanceXY[20*2];      /* 检测到的目标的距离坐标(x,y),上限为20个 */
} PD_DUMP_INFO_S;

#define MEDIA_IMAGE_WIDTH      608                  /* 算法图像帧宽度 */
#define MEDIA_IMAGE_HEIGHT     352                  /* 算法图像帧高度 */

static int ipsys_log_level = SV_DEBUG;
static int g_as32MediaBufFd[4] = {-1, -1, -1, -1}; // 各物理通道的Media Buffer的文件描述符 // The file descriptor of the Media Buffer of each physical channel 
static int g_s32SocketFd = -1;
static int g_running = 0;

#if !defined(BOARD_ADA47V1)
sint32 MEDIA_GET_ALG_FD(int *pfd, int idx){ return SV_SUCCESS;}
#endif

extern "C" {
    extern sint32 CAN_Init(CFG_SYS_PARAM *pstSysParam);
    extern sint32 CAN_Fini();
    extern sint32 CAN_Reset();
    extern uint32 CAN_GetCanid(uint32 u32Canid);
    extern sint32 CAN_SendFrame(struct can_frame *pstFrame);
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

static sint32 callbackGetRS485Data(MSG_PACKET_S *pstMsgPkt, MSG_PACKET_S *pstRetPkt)
{	
#if defined(BOARD_ADA47V1)
    sint32 s32Ret = 0;
	MSG_RS485_DATA_S stRS485Data = {0};
    sint32 i = 0;
	memcpy(&stRS485Data, pstMsgPkt->pu8Data, sizeof(MSG_RS485_DATA_S));
    
    /* 处理消息 */
    print_level(SV_INFO, "Get RS485 Data Finish!\n");
    for (i=0; i<stRS485Data.s32DataLen; i++)
    {
        printf("0x%x ", stRS485Data.szRS485Data[i]);

        if ((i+1)%25 == 0)
        {
            printf("\r\n");
        }
    }
    printf("\r\n");
#endif

    return SV_SUCCESS;
}

static sint32 callbackGetBluetoothData(MSG_PACKET_S *pstMsgPkt, MSG_PACKET_S *pstRetPkt)
{	
#if defined(BOARD_ADA47V1)
    sint32 s32Ret = 0;
	MSG_BLUETOOTH_DATA_S stBluetoothData = {0};
    sint32 i = 0;
	memcpy(&stBluetoothData, pstMsgPkt->pu8Data, sizeof(MSG_BLUETOOTH_DATA_S));
    
    /* 处理消息 */
    print_level(SV_INFO, "Get Bluetooth Data Finish!\n");
    for (i=0; i<stBluetoothData.s32DataLen; i++)
    {
        printf("0x%x ", stBluetoothData.szBluetoothData[i]);

        if ((i+1)%25 == 0)
        {
            printf("\r\n");
        }
    }
    printf("\r\n");
#endif

    return SV_SUCCESS;
}


/******************************************************************************
 * 函数功能: 通过socket获取MediaBuf Fd / Get the MediaBuf Fd via socket
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

/* 中断退出 */
static void exit_handle(int signalnum)
{
    sint32 s32Ret = 0;
    
    printf("catch signalnum %d!\n", signalnum);
    exit(EXIT_FAILURE);
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

static sint32 pd_DumpInfo(sint32 s32Chn, PD_DUMP_INFO_S *pstPdDumpInfo)
{
    sint32 s32Ret = 0, i;
    uint32 u32ChnNum = 1;
    sint32 fd = -1;
    cJSON *pstJson = NULL, *pstList = NULL, *pstItem = NULL;
    cJSON *pstTimeStamp = NULL, *pstPdWorkMode = NULL, *pstGreenRoiNum = NULL, *pstYellowRoiNum = NULL, *pstRedRoiNum = NULL;
    char szBuf[1024];
    char echo_szBuf[2048];
    static PD_DUMP_INFO_S astDumpInfo[ALG_MAX_CHN] = {0};

    if (s32Chn >= ALG_MAX_CHN)
    {
        return ERR_ILLEGAL_PARAM;
    }

    if (NULL == pstPdDumpInfo)
    {
        return ERR_NULL_PTR;
    }

    astDumpInfo[s32Chn] = *pstPdDumpInfo;
    pstJson = cJSON_CreateObject();
    if (NULL == pstJson)
    {
        print_level(SV_ERROR, "cJSON_CreateObject fail!\n");
        return SV_FAILURE;
    }

    pstList = cJSON_CreateArray();
    if (NULL == pstList)
    {
        print_level(SV_ERROR, "cJSON_CreateArray fail!\n");
        goto exit;
    }

    u32ChnNum = 1;
    cJSON_AddItemToObject(pstJson, "pdsChn", pstList);
    for (i = 0; i < u32ChnNum; i++)
    {
        pstItem = cJSON_CreateObject();
        if (NULL == pstItem)
        {
            print_level(SV_ERROR, "cJSON_CreateObject fail!\n");
            goto exit;
        }

        cJSON_AddItemToArray(pstList, pstItem);
        pstTimeStamp = cJSON_CreateNumber((double)astDumpInfo[i].s64TimeStamp);
        if(NULL == pstTimeStamp)
        {
            print_level(SV_ERROR, "cJSON_CreateNumber failed.\n");
            goto exit;
        }
        cJSON_AddItemToObject(pstItem, "TimeStamp", pstTimeStamp);

        pstPdWorkMode = cJSON_CreateNumber(CFG_PD_WORK_NORMAL);
        if(NULL == pstTimeStamp)
        {
            print_level(SV_ERROR, "cJSON_CreateNumber failed.\n");
            goto exit;
        }
        cJSON_AddItemToObject(pstItem, "PDWorkMode", pstPdWorkMode);

        pstGreenRoiNum = cJSON_CreateNumber((double)astDumpInfo[i].s32GreenRoiNum);
        if(NULL == pstGreenRoiNum)
        {
            print_level(SV_ERROR, "cJSON_CreateNumber failed.\n");
            goto exit;
        }
        cJSON_AddItemToObject(pstItem, "GreenRoiNum", pstGreenRoiNum);

        pstYellowRoiNum = cJSON_CreateNumber((double)astDumpInfo[i].s32YellowRoiNum);
        if(NULL == pstYellowRoiNum)
        {
            print_level(SV_ERROR, "cJSON_CreateNumber failed.\n");
            goto exit;
        }
        cJSON_AddItemToObject(pstItem, "YellowRoiNum", pstYellowRoiNum);

        pstRedRoiNum = cJSON_CreateNumber((double)astDumpInfo[i].s32RedRoiNum);
        if(NULL == pstRedRoiNum)
        {
            print_level(SV_ERROR, "cJSON_CreateNumber failed.\n");
            goto exit;
        }
        cJSON_AddItemToObject(pstItem, "RedRoiNum", pstRedRoiNum);
    }

    memset(szBuf, 0, 1024);
    cJSON_PrintPreallocated(pstJson, szBuf, 1024, 0);
    
    string_to_file("/var/info/pd-tmp", szBuf);
    rename("/var/info/pd-tmp", "/var/info/pd");

exit:
    cJSON_Delete(pstJson);
    return SV_SUCCESS;
}

void Media_CaptureImage()
{
    sint32 s32Ret = 0, i;
    sint32 s32CurChn = 0;
    sint32 s32fd = -1;
    void *pvBuf = NULL;
    uint32 u32BufLen = MEDIA_IMAGE_WIDTH*MEDIA_IMAGE_HEIGHT*3;  // RGB888
    char szFilePath[256];

    pvBuf = mmap(NULL, u32BufLen, PROT_READ, MAP_SHARED, g_as32MediaBufFd[0], 0);
    if (MAP_FAILED == pvBuf)
    {
        print_level(SV_ERROR, "mmap failed.\n");
        return;
    }

    print_level(SV_INFO, "mmap pvBuf:%#x\n", pvBuf);
    for (i = 1; i <= 5; i++)
    {
        /* P操作进入MediaBuffer临界区 / The P operation enters the MediaBuffer critical zone */
        s32Ret = MS_P(s32CurChn);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "MS_P failed. [err=%d]\n", s32Ret);
            return;
        }

        sprintf(szFilePath, "capture_%d.rgb", i);
        remove(szFilePath);
        print_level(SV_INFO, "try capture file: %s\n", szFilePath);
        s32fd = open(szFilePath, O_CREAT | O_RDWR, S_IRWXU | S_IRGRP | S_IROTH);
        write(s32fd, pvBuf, u32BufLen);
        close(s32fd);

        /* 模拟算法运行耗时 / The simulation algorithm takes the same time to run */
        sleep_ms(50);

        /* V操作退出MediaBuffer临界区 / The V operation exits the MediaBuffer critical zone */
        MS_V(s32CurChn);
    }

    munmap(pvBuf, u32BufLen);
}

/***************************************************************
*-# 用例编号: Media_DrawImage
*-# 测试功能: 在输出的图像中叠加行人矩形框 / Overlay the pedestrian rectangular boxes in the output image
*-# 预期结果: 在输出的图像中可在对应设置的坐标上看到矩形框 / The rectangular box is visible on the corresponding set coordinates in the output image
* 
****************************************************************/
void Media_DrawImage()
{
    sint32 s32Ret = 0;
    sint32 s32CurChn = 0;
    uint16 u16mask;
    MSG_PACKET_S stMsgPkt = {0};
    MEDIA_GUI_DRAW_S stMediaGuiDraw = {0};
    MEDIA_GUI_PERSON_S stGuiRect = {0};
    MEDIA_GUI_STRING_S stGuiText = {0};
    MEDIA_GUI_NULL_S stGuiNull;

    memset(&stMediaGuiDraw, 0x00, sizeof(stMediaGuiDraw));
    memset(&stGuiRect, 0x00, sizeof(stGuiRect));

    /* 添加行人矩形绘制 / Add pedestrian rectangular plots */
    stGuiRect.u32PersonNum = 2;
    stGuiRect.astPersonsRect[0].x1 = 0.1;
    stGuiRect.astPersonsRect[0].y1 = 0.1;
    stGuiRect.astPersonsRect[0].x2 = 0.5;
    stGuiRect.astPersonsRect[0].y2 = 0.7;
    stGuiRect.astPersonsRect[0].color = GUI_COLOR_GREEN;
    stGuiRect.astPersonsRect[0].stick = 2;
    stGuiRect.astPersonsRect[1].x1 = 0.4;
    stGuiRect.astPersonsRect[1].y1 = 0.4;
    stGuiRect.astPersonsRect[1].x2 = 0.6;
    stGuiRect.astPersonsRect[1].y2 = 0.8;
    stGuiRect.astPersonsRect[1].color = GUI_COLOR_RED;
    stGuiRect.astPersonsRect[1].stick = 3;
    u16mask = MEDIA_GUI_GET_MASK(s32CurChn, 0, MEDIA_GUI_OP_PERSON_RECT);
    s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiRect);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
    }

    stGuiText.x = 0.3;
    stGuiText.y = 0.3;
    strcpy(stGuiText.string, "Hello");
    stGuiText.color = GUI_COLOR_YELLOW;
    stGuiText.fontsize = 3;
    u16mask = MEDIA_GUI_GET_MASK(s32CurChn, 0, MEDIA_GUI_OP_DRAW_STRING);
    s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiText);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
    }

    /* 发送绘图消息 / Send a drawing message */
    memset(&stMsgPkt, 0, sizeof(stMsgPkt));
    stMsgPkt.pu8Data = (uint8*)&stMediaGuiDraw;
    stMsgPkt.u32Size = sizeof(stMediaGuiDraw);
    s32Ret = Msg_submitEvent(EP_CONTROL, OP_EVENT_MEDIA_GUI, &stMsgPkt);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
    }

    sleep_ms(5000);

    /* 清空画板 / Clear the drawing board */
    memset(&stMediaGuiDraw, 0x00, sizeof(stMediaGuiDraw));
    u16mask = MEDIA_GUI_GET_MASK(s32CurChn, 0, MEDIA_GUI_OP_CLEAR);
    s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiNull);
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

/***************************************************************
*-# 用例编号: Media_PlayAlarmAudio
*-# 测试功能: 播放音频 / Play audio
*-# 预期结果: 在显示屏上可听到ding/ding/ding报警声音 / Hearing the ding/ding/ding alarm sound on the display screen
* 
****************************************************************/
void Media_PlayAlarmAudio()
{
    int mode;

    mode = 1;
    print_level(SV_INFO, "play audio mode: %d\n", mode);
    ALARM_PlayAlarm(0, mode, SV_TRUE);
    sleep_ms(2000);

    mode = 2;
    print_level(SV_INFO, "play audio mode: %d\n", mode);
    ALARM_PlayAlarm(0, mode, SV_TRUE);
    sleep_ms(2000);

    mode = 3;
    print_level(SV_INFO, "play audio mode: %d\n", mode);
    ALARM_PlayAlarm(0, mode, SV_TRUE);
    sleep_ms(2000);
}

/***************************************************************
*-# 用例编号: Media_SendAlarmEvent
*-# 测试功能: 发送报警事件 / Send Alarm Event
*-# 预期结果: 生成报警录像文件, 串口或CAN口发送报警数据帧 / Generate alarm video file and send alarm data frame to serial port or CAN port
* 
****************************************************************/
void Media_SendAlarmEvent()
{
    sint32 s32Ret;
    sint32 s32CurChn = 0;
    PD_DUMP_INFO_S stPdDumpInfo;
    MSG_PACKET_S stMsgPkt = {0};
    ALARM_EVENT_S stAlarmEvent = {0};
    struct timeval tvAlarm;
    struct timezone tz;

    memset(&stAlarmEvent, 0, sizeof(stAlarmEvent));
    gettimeofday(&tvAlarm, &tz);
    stAlarmEvent.enAlarmEvent = ALARM_EVENT_PD;
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

    /* A32行人检测事件使用以下方法发送CAN数据帧 /The A32 pedestrian detection event sends the CAN data frame using the following method */
    memset(&stPdDumpInfo, 0x00, sizeof(PD_DUMP_INFO_S));
    stPdDumpInfo.s32RedRoiNum = 1;  // 设置ROI区域检测到的行人数量 / Set the number of pedestrians detected in the ROI area
    stPdDumpInfo.s32YellowRoiNum = 1;
    stPdDumpInfo.s32GreenRoiNum = 1;
    stPdDumpInfo.s64TimeStamp = tvAlarm.tv_sec * 1000 + tvAlarm.tv_usec /1000;
    pd_DumpInfo(s32CurChn, &stPdDumpInfo);

    sleep_ms(2000);
    memset(&stPdDumpInfo, 0x00, sizeof(PD_DUMP_INFO_S));    // 清除报警状态 / Clear alarm status
    pd_DumpInfo(s32CurChn, &stPdDumpInfo);
    
    print_level(SV_INFO, "Send alarm event successful!\n");
}

/***************************************************************
*-# 用例编号: Media_SendCanData
*-# 测试功能: 发送CAN数据 / Send the CAN data
*-# 预期结果: 在设备的CAN接口线上可以侦测到CAN数据包 / The CAN packets can be detected on the device's CAN interface line
* 
****************************************************************/
void Media_SendCanData()
{
    sint32 s32Ret;
    MSG_PACKET_S stMsgPkt = {0};
    MSG_CAN_DATA_S stMsgCanData = {0};

    memset(&stMsgCanData, 0, sizeof(stMsgCanData));
    stMsgCanData.bSingleFrame = SV_TRUE;
    strcpy(stMsgCanData.szCanData, "Hello!");
    stMsgCanData.s32DataLen = strlen(stMsgCanData.szCanData);
    memset(&stMsgPkt, 0, sizeof(stMsgPkt));
    stMsgPkt.pu8Data = (uint8*)&stMsgCanData;
    stMsgPkt.u32Size = sizeof(stMsgCanData);
    s32Ret = Msg_submitEvent(EP_MCU, OP_EVENT_CAN_USERDATA, &stMsgPkt);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
    }

    s32Ret = Msg_submitEvent(EP_CAN, OP_EVENT_CAN_USERDATA, &stMsgPkt);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
    }
}

/***************************************************************
*-# 用例编号: Media_SetCanConfig
*-# 测试功能: 设置CAN配置参数 / Setting the CAN config
*-# 预期结果: 在设备的CAN接口发送的数据工作对应的配置模式 / The configuration mode corresponding to the data work sent at the CAN interface of the device
* 
****************************************************************/
void Media_SetCanConfig()
{
    sint32 s32Ret;
    MSG_PACKET_S stMsgPkt = {0}, stRetPkt = {0};
    MSG_SYS_CFG stSysCfg = {0};
    MSG_CAN_DATA_S stMsgCanData = {0};

    /* 获取当前的系统配置参数      /  Gets the current system configuration parameters */
    stRetPkt.pu8Data = (uint8 *)&stSysCfg;
    s32Ret = Msg_execRequestBlock(EP_ALG, EP_CONTROL, OP_REQ_GET_SYS_CFG, NULL, &stRetPkt, sizeof(MSG_SYS_CFG));
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "OP_REQ_GET_SYS_CFG failed. [err=%#x]\n", s32Ret);
        return;
    }

    /* 修改需要调整的部分参数 / Modify some of the parameters to be adjusted */    
    strcpy(stSysCfg.szDmmCanid, "0x18fad1fe");
    strcpy(stSysCfg.szFrsCanid, "0x18fad0fe");
    strcpy(stSysCfg.szHeartCanid, "0x18fad5fe");
    strcpy(stSysCfg.szPdsCanid, "0x18fad0fc");
    stSysCfg.enFrameFormat = E_CAN_STANDARD;
    stSysCfg.s32Baudrate = 250;
    stMsgPkt.pu8Data = (uint8 *)&stSysCfg;
    stMsgPkt.u32Size = sizeof(MSG_SYS_CFG);
    s32Ret = Msg_execRequestBlock(EP_ALG, EP_CONTROL, OP_REQ_SET_SYS_CFG, &stMsgPkt, NULL, 0);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "OP_REQ_SET_SYS_CFG failed.\n");
        return;
    }

    print_level(SV_INFO, "setting CAN config successful.\n");
}


/***************************************************************
*-# 用例编号: Media_SendRS485Data
*-# 测试功能: 发送RS485数据 / Send the RS485 data
*-# 预期结果: 在设备的RS485接口线上可以侦测到RS485数据包 / The RS485 packets can be detected on the device's RS485 interface line
* 
****************************************************************/
void Media_SendRS485Data()
{
    sint32 s32Ret;
    MSG_PACKET_S stMsgPkt = {0};
    MSG_RS485_DATA_S stMsgRS485Data = {0};
    
    memset(&stMsgRS485Data, 0, sizeof(stMsgRS485Data));
    strcpy(stMsgRS485Data.szRS485Data, "RS485 normal.\r\n");
    stMsgRS485Data.s32DataLen = strlen(stMsgRS485Data.szRS485Data);
    memset(&stMsgPkt, 0, sizeof(stMsgPkt));
    stMsgPkt.pu8Data = (uint8*)&stMsgRS485Data;
    stMsgPkt.u32Size = sizeof(stMsgRS485Data);
    s32Ret = Msg_submitEvent(EP_RS485, OP_EVENT_RS485_SEND, &stMsgPkt);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
    }
    sleep_ms(1000);
}

/***************************************************************
*-# 用例编号: Media_ExecActionDedust
*-# 测试功能: 执行除尘动作 / execute the dedusting action
*-# 预期结果: 可以观察到步进电机执行除尘动作 / The stepmotor can be observed to execute the dedusting action
* 
****************************************************************/
void Media_ExecActionDedust()
{
    sint32 s32Ret = 0;
    MSG_PACKET_S stMsgPkt = {0};
    s32Ret = Msg_submitEvent(EP_STEPMOTOR, OP_EVENT_STEPMOTOR_DEDUST, &stMsgPkt);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);

    }
}

/***************************************************************
*-# 用例编号: Media_TestSwitchInOut
*-# 测试功能: 获取两路SwitchIn的输入,设置两路SwitchOut的输出,每次执行都会翻转SwitchOut的输出
*-# 预期结果: 可以看到SwitchIn和SwitchOut数值的打印信息,其代表实际的输入输出电平(0低电平 1高电平)
* 
****************************************************************/
void Media_TestSwitchInOut()
{
#if defined(BOARD_ADA47V1)    
    sint32 s32Ret = 0;
    uint8 u8SwitchInVal = 0;
    uint8 s_u8SwitchOutVal = 0;
    MSG_PACKET_S stMsgPkt = {0};
    MSG_RS485_DATA_S stMsgRS485Data = {0};
    char szLog[64] = {0};

    s32Ret = BOARD_SetSwitchOut(1, s_u8SwitchOutVal);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "BOARD_SetSwitchOut failed. [err=%#x]\n", s32Ret);
    }
    else
    {
        print_level(SV_INFO, "SwitchOut1 value:%x\n", s_u8SwitchOutVal);
    }
    
    s32Ret = BOARD_GetSwitchIn(1, &u8SwitchInVal);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "BOARD_GetSwitchIn failed. [err=%#x]\n", s32Ret);
    }
    else
    {
        print_level(SV_INFO, "SwitchIn1 value:%x\n", u8SwitchInVal);
    }

    /* 测试不通过 */
    if (u8SwitchInVal == s_u8SwitchOutVal)
    {
        strcpy(szLog, "Switch in1 out1 test fail!!!\r\n");
    }
    else
    {
        s_u8SwitchOutVal = 1-s_u8SwitchOutVal;
        s32Ret = BOARD_SetSwitchOut(1, s_u8SwitchOutVal);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "BOARD_SetSwitchOut failed. [err=%#x]\n", s32Ret);
        }
        else
        {
            print_level(SV_INFO, "SwitchOut1 value:%x\n", s_u8SwitchOutVal);
        }
        
        s32Ret = BOARD_GetSwitchIn(1, &u8SwitchInVal);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "BOARD_GetSwitchIn failed. [err=%#x]\n", s32Ret);
        }
        else
        {
            print_level(SV_INFO, "SwitchIn1 value:%x\n", u8SwitchInVal);
        }

        if (u8SwitchInVal == s_u8SwitchOutVal)
        {
            strcpy(szLog, "Switch in1 out1 test fail!!!\r\n");
        }
        else
        {
            strcpy(szLog, "Switch in1 out1 test success.\r\n");
        }
    }
    
    memset(&stMsgRS485Data, 0, sizeof(stMsgRS485Data));
    strcpy(stMsgRS485Data.szRS485Data, szLog);
    stMsgRS485Data.s32DataLen = strlen(stMsgRS485Data.szRS485Data);
    memset(&stMsgPkt, 0, sizeof(stMsgPkt));
    stMsgPkt.pu8Data = (uint8*)&stMsgRS485Data;
    stMsgPkt.u32Size = sizeof(stMsgRS485Data);
    s32Ret = Msg_submitEvent(EP_RS485, OP_EVENT_RS485_SEND, &stMsgPkt);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
    }

    sleep_ms(1000);

#if 0
    s32Ret = BOARD_SetSwitchOut(2, s_u8SwitchOutVal);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "BOARD_SetSwitchOut failed. [err=%#x]\n", s32Ret);
    }
    else
    {
        print_level(SV_INFO, "SwitchOut2 value:%x\n", s_u8SwitchOutVal);
    }
    
    s32Ret = BOARD_GetSwitchIn(2, &u8SwitchInVal);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "BOARD_GetSwitchIn failed. [err=%#x]\n", s32Ret);
    }
    else
    {
        print_level(SV_INFO, "SwitchIn2 value:%x\n", u8SwitchInVal);
    }

    if (u8SwitchInVal == (1-s_u8SwitchOutVal))
    {
        memset(&stMsgRS485Data, 0, sizeof(stMsgRS485Data));
        strcpy(stMsgRS485Data.szRS485Data, "Switch in2 out2 test success.\r\n");
        stMsgRS485Data.s32DataLen = strlen(stMsgRS485Data.szRS485Data);
        memset(&stMsgPkt, 0, sizeof(stMsgPkt));
        stMsgPkt.pu8Data = (uint8*)&stMsgRS485Data;
        stMsgPkt.u32Size = sizeof(stMsgRS485Data);
        s32Ret = Msg_submitEvent(EP_RS485, OP_EVENT_RS485_SEND, &stMsgPkt);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
        }
    }
    else
    {
        memset(&stMsgRS485Data, 0, sizeof(stMsgRS485Data));
        strcpy(stMsgRS485Data.szRS485Data, "Switch in2 out2 test fail!!!\r\n");
        stMsgRS485Data.s32DataLen = strlen(stMsgRS485Data.szRS485Data);
        memset(&stMsgPkt, 0, sizeof(stMsgPkt));
        stMsgPkt.pu8Data = (uint8*)&stMsgRS485Data;
        stMsgPkt.u32Size = sizeof(stMsgRS485Data);
        s32Ret = Msg_submitEvent(EP_RS485, OP_EVENT_RS485_SEND, &stMsgPkt);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
        }
    }
    sleep_ms(1000);
#endif
#endif    
}

sint32 CAN_Send_Heart_Data(CFG_SYS_PARAM *pstSysParam)
{
	sint32 s32Ret, k;
	uint8 szHeartData[10] = {0};
    uint32 u32HeartCanid = 0;
	struct can_frame stCanFrame;

    s32Ret = CAN_Init(pstSysParam);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "CAN_Init failed.\n");
        return s32Ret;
    }

	u32HeartCanid = strtol(pstSysParam->pszHeartCanid, NULL, 16);
	stCanFrame.can_id = CAN_GetCanid(u32HeartCanid);

    // result frame
    stCanFrame.can_dlc = 2;
    stCanFrame.data[0] = 0xff;
    stCanFrame.data[1] = 0x00;
    s32Ret = CAN_SendFrame(&stCanFrame);
	if (SV_SUCCESS != s32Ret)
	{
		print_level(SV_ERROR, "CAN_SendFrame failed.\n");
		goto error_exit;
	}

    
	// starting frame
	stCanFrame.can_dlc = 8;
	memset(szHeartData, 0, sizeof(szHeartData));
	szHeartData[0] = 0x01;
	
	k = 0;
    stCanFrame.data[k++] = 0xfe;
    stCanFrame.data[k++] = 0x01;
    //stCanFrame.data[k++] = crc8_check(szHeartData, 1);
    stCanFrame.data[k++] = 0x31;
    while (k < 8)
	{
        stCanFrame.data[k++] = 0xfe;
    }
    s32Ret = CAN_SendFrame(&stCanFrame);
	if (SV_SUCCESS != s32Ret)
	{
		print_level(SV_ERROR, "CAN_SendFrame failed.\n");
		goto error_exit;
	}

    
	// data frame
	k = 0;
    stCanFrame.data[k++] = 0x0f;
    stCanFrame.data[k++] = 0x01;
    while (k < 8)
	{
        stCanFrame.data[k++] = 0x00;
    }
    s32Ret = CAN_SendFrame(&stCanFrame);
	if (SV_SUCCESS != s32Ret)
	{
		print_level(SV_ERROR, "CAN_SendFrame failed.\n");
		goto error_exit;
	}

    
	// ending frame
	k = 0;
	stCanFrame.data[k++] = 0xff;
    while (k < 8)
	{
        stCanFrame.data[k++] = 0xff;
    }
    s32Ret = CAN_SendFrame(&stCanFrame);
	if (SV_SUCCESS != s32Ret)
	{
		print_level(SV_ERROR, "CAN_SendFrame failed.\n");
		goto error_exit;
	}

    s32Ret = CAN_Fini();
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "CAN_Fini failed.\n");
        return s32Ret;
    }
    return SV_SUCCESS;

error_exit:
    s32Ret = CAN_Fini();
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "CAN_Fini failed.\n");
        return s32Ret;
    }
    return SV_FAILURE;
}

/* 监测媒体FD线程 */
void * media_Watch_BufferFd(void *pvArg)
{
    sint32 s32Ret = 0, n, i;
    sint32 s32Tmp = 0;
    sint32 s32ChnNum = 1;
    SocketPacket stSocketPkt = {0};
    fd_set read_fds, write_fds;
    struct timeval timeout;
    int as32MediaBufFd[4] = {-1, -1, -1, -1};
    
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

    return NULL;
}

int main(int argc, char **argv)
{
    sint32 s32Ret = 0, n, i;
#if 0
    sint32 s32ChnNum;
#endif
    char *pszConfigFile = CONFIG_XML;
    char *pszConfigBak1 = CONFIG_BAK1;
    char *pszConfigBak2 = CONFIG_BAK2;
    char *pszConfigDefault = CONFIG_DEFAULT;
    CFG_ALG_PARAM stAlgParam = {0};
    CFG_SYS_PARAM stSysParam = {0};
    ALARM_CFG_PARAM_S stAlarmParam = {0};
    pthread_t thread;
    void *pvRetval = NULL;
    sint32 chChoice;

#if 0
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
#if 0
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
#endif
#if 0
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
#endif
    /* 初始化MediaBuffer临界区 */
#if 0
    s32Ret = MS_Init();
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "MS_Init failed. [err=%#x]\n", s32Ret);
        return ERR_SYS_NOTREADY;
    }
#endif
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
#if 0
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
#endif
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

#if defined(BOARD_ADA47V1)
    s32Ret = Msg_registerOpCallback(EP_ALG, OP_EVENT_RS485_RECV, callbackGetRS485Data);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_registerOpCallback failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }

    s32Ret = Msg_registerOpCallback(EP_ALG, OP_EVENT_BLUETOOTH_RECV, callbackGetBluetoothData);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_registerOpCallback failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }
#endif

    g_running = 1;
#if 0
    s32Ret = pthread_create(&thread, NULL, media_Watch_BufferFd, NULL);
#endif

    while (1)
    {
        CAN_Send_Heart_Data(&stSysParam);
        Media_SendRS485Data();
        Media_TestSwitchInOut();
    }

    g_running = 0;
#if 0
    pthread_join(thread, &pvRetval);
    MS_Fini();
#endif
    return 0;
}

