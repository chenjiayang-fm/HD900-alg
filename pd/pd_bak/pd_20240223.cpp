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
#include <termios.h>

#include "print.h"
#include "../../../include/common.h"
#include "safefunc.h"
#include "op.h"
#include "msg.h"
#include "alarm.h"
#include "avalarmer.h"
#include "led.h"
#include "pds_alg.h"
#include "pd.h"
#include "alg.h"

#include "config.h"
#include "media.h"
#include "media_sem.h"
#include "media_shm.h"
#include "alg.h"
#include "cJSON.h"
#include "utils.h"
#include "board.h"
#include "utils.h"
#include "thpool.h"

#include "CaliEX.h"
#include <jpeg/jpeglib.h>
#include <jpeg/jerror.h>


#include <linux/videodev2.h> 
#include <sys/ioctl.h>
#include <sys/poll.h>

#include "../include/itc.h"
//#include"../include/mpp_vdec_link.h"

//#include "../../media/include/mpp_vdec.h"
//#include "../../media/rockchip/rv1126/include2/rkmedia/rkmedia_vdec.h"
#include "rkmedia_api.h"
//#include "mpp_vdec.h"
// #include "rga/rga.h"
// #include "rga/RgaApi.h"
// #include "im2d_api/im2d.h"



//#define REMOVE_ALARM_OUT


#define min(x, y) (((x) < (y)) ? (x) : (y))
#define max(x, y) (((x) > (y)) ? (x) : (y))
#define abs(x) ((x)<0? -(x) : (x))

#define ALG_MAX(a,b)    (((a) > (b)) ? (a) : (b))
#define ALG_MIN(a,b)    (((a) < (b)) ? (a) : (b))

#define ALG_RANGE_LIMIT(v, a, b) do{ \
    if (v < a) v=a; \
    if (v > b) v=b; \
}while(0)

#define PD_OVERTAKE_ALARM 0     /* 超车报警 */

#define PD_ALARM_PIN_BAND 3
#define PD_ALARM_PIN_NUM  20

#define PD_MODEL_RGB_P      "/root/model/RGB_P.rknn"                /* 可见光检人模型 */
#define PD_MODEL_RGB_P_OW   "/root/model/RGB_P_OW.rknn"             /* 可见光俯视大角度检人模型 */
#define PD_MODEL_RGB_P_201266B "/root/model/RGB_P_201266.rknn"      /* 可见光检测人, 躺下的人, 穿反光衣的人模型 */
#define PD_MODEL_RGB_P_FTC  "/root/model/RGB_P_FTC.rknn"            /* 可见光检测人模型,规避啤酒桶误检 */
#define PD_MODEL_RGB_P_201306 "/root/model/RGB_P_201306.rknn"       /* 可见光检测人模型，规避雪糕筒误检 */
#define PD_MODEL_RGB_PC     "/root/model/RGB_PC.rknn"               /* 可见光检人和车模型 */
#define PD_MODEL_RGB_PC_OW  "/root/model/RGB_PC_OW.rknn"            /* 可见光俯视大角度检人和车模型 */
#define PD_MODEL_IR_PC      "/root/model/RED_PC.rknn"               /* 红外热成像检人和车模型 */
#define PD_MODEL_RGB_MANHOLE "/root/model/RGB_MANHOLE.rknn"         /* 可见光检井盖模型 */
#define PD_MODEL_RGB_BEAR   "/root/model/RGB_LYQ_BEAR.rknn"         /* 检测熊的模型 */
#define PD_MODEL_RGB_IR_PC   "/root/model/RGB_NIR_PC.rknn"          /* 带IRCut红外灯模型 */
#define PD_MODEL_RGB_PC_90   "/root/model/RGB_PC_90.rknn"           /* 可见光检人旋转90度版本 */
#define PD_MODEL_RGB_SH     "/root/model/RGB_SH.rknn"               /* 可见光检人配带安全帽 */
#define PD_MODEL_RGB_CQQ_SIGN "/root/model/RGB_CQQ_SIGN.rknn"       /* 201165 客户检测SpeedZone标志*/
#define PD_MODEL_RGB_SSR    "/root/model/RGB_SSR.rknn"              /* 201338 客户检测限速标志*/
#define PD_MODEL_TRAFFIC    "/root/model/TRAFFIC.rknn"              /* 交通灯模型*/
#define PD_MODEL_RGB_PC_202406 "/root/model/RGB_PC_202406.rknn"     /* 202406 客户模型 */
#define PD_MODEL_RGB_TEST      "/root/model/RGB_PC_608x352.rknn" 

#if !defined(BOARD_ADA32V3)
#define PD_IMAGE_WIDTH      608                     /* 算法图像帧宽度 */
#define PD_IMAGE_HEIGHT     352                     /* 算法图像帧高度 */
#define PD_IMAGE_WIDTH_90   352                     /* 算法图像帧宽度 旋转90度 */
#define PD_IMAGE_HEIGHT_90  608                     /* 算法图像帧高度 旋转90度 */
//触发输入GPIO口 GPIO3_C3
#define PD_ALARM_IN_BIND    3                       /* RV1126 alarmIn bind */
#define PD_ALARM_IN_PIN     19                      /* RV1126 alarmIn pin */
#else
#define PD_IMAGE_WIDTH      608                     /* 算法图像帧宽度 */
#define PD_IMAGE_HEIGHT     352                     /* 算法图像帧高度 */
#define PD_IMAGE_WIDTH_90   352                     /* 算法图像帧宽度 旋转90度 */
#define PD_IMAGE_HEIGHT_90  608                     /* 算法图像帧高度 旋转90度 */
//触发输入GPIO口 GPIO1_B1
#define PD_ALARM_IN_BIND    1                       /* RV1106 alarmIn bind */
#define PD_ALARM_IN_PIN     9                       /* RV1106 alarmIn pin */
#endif

#define PD_IMAGE_BUF_SIZE   (PD_IMAGE_WIDTH*PD_IMAGE_HEIGHT*3)  /* PD图像buffer大小 */
#define PD_IR_IMG_WIDTH     512                     /* 红外算法图像宽 */
#define PD_IR_IMG_HEIGHT    384                     /* 红外算法图像高 */
#define PD_IR_BUF_SIZE      (PD_IR_IMG_WIDTH*PD_IR_IMG_HEIGHT*3)/* 红外图像buffer大小 */
#pragma  pack(1)
typedef struct tag_Type24
{
    char buf[3];
} TYPE24;
#pragma  pack()

typedef struct tagPdGuiImg_S
{
    char *pbmp;
    sint32 s32Width;
    sint32 s32Height;
} PD_GUI_IMG_S;

typedef enum tagPdAlarm_Type_E
{
    PD_ALARM_TYPE_NULL = 0, /* 无报警触发 */
    PD_ALARM_TYPE_GREEN,
    PD_ALARM_TYPE_YELLOW,
    PD_ALARM_TYPE_RED,

    PD_ALARM_TYPE_BUTT,
} PD_ALARM_TYPE;


/* 报警事件传递信息 */
typedef struct tagPdAlarmNotice_S
{
    PD_ALARM_TYPE   enMode;                 /* 报警模式 */
    SV_BOOL         bPerson;                /* 是否检测到行人(非车辆) */
    SV_BOOL         bCar;                   /* 是否检测到车辆 */
    sint32          s32Chn;                 /* 当前通道号 */
    SPLIT_MODE      enSplit;                /* 分屏方式 */
} PD_ALARM_NOTICE_S;

/* 安全帽颜色 */
typedef enum tagHatColor_S
{
    HAT_RED = 0,
    HAT_YELLOW,
    HAT_WHITE,
    HAT_BLUE,
    
    HAT_BUTT,
} HAT_COLOR_S;

/**********************/

typedef struct tagPdPersonSH_S
{
    sint32  s32Num;         /* 总人数 */
    SV_BOOL bHat[50];       /* 是否带安全帽 */
    float   x1[50];         /* 人左上角x坐标 */
    float   y1[50];         /* 人左上角y坐标 */
    float   x2[50];         /* 人右下角x坐标 */
    float   y2[50];         /* 人右下角y坐标 */

    float   hx1[50];        /* 帽子左上角x坐标 */
    float   hy1[50];        /* 帽子左上角y坐标 */
    float   hx2[50];        /* 帽子右下角x坐标 */
    float   hy2[50];        /* 帽子右下角y坐标 */
    HAT_COLOR_S color[50];  /* 安全帽颜色 */
} PD_PERSON_HAT_S;


/* 模块控制信息 */
typedef struct tagPdInfo_S
{
    SV_BOOL         bKeyAuth;               /* 密钥是否有效 */
    uint32          u32ChnNum;              /* PD算法通道数目 */
    pdsa32::CPdsAlg *apcsPdsAlg[ALG_MAX_CHN]; /* PD算法对象指针 */
    pdsa32::CPdsAlg *apcsPdsAlg_90;         /* PD算法对象指针(旋转90度) */
    pdsa32::STTrackParam stTrackParam;      /* PD算法跟踪参数 */
#if 0
    sint32          as32MediaBufFd[4][3];      /* 媒体通道Media Buffer的文件描述符 */
#else
	sint32          as32MediaBufFd[4];      /* 媒体通道Media Buffer的文件描述符 */
#endif
    uint32          u32Width;               /* 算法图片宽 */
    uint32          u32Height;              /* 算法图片高 */
    uint32          u32Width_90;            /* 算法图片宽(旋转90度) */
    uint32          u32Height_90;           /* 算法图片高(旋转90度) */
    CFG_ALG_PARAM   stCfgParam;             /* 算法配置参数 */
    float           fConfidenceThr;         /* 算法得分阈值 */
    float           fNmsThreshold;          /* 算法NMS阈值 */
    float           fTrackThreshold;        /* 跟踪模块的阈值 */
    float           fTrackMaxAge;           /* 跟踪模块的参数 */
    SV_BOOL         bSkipPerson[ALG_MAX_CHN];/* 跳过行人框 */
    SV_BOOL         bSkipCar[ALG_MAX_CHN];  /* 跳过车辆 */
    SPLIT_MODE      enSplitMode;            /* 显示模式 */
    SV_BOOL         bRotate;                /* 旋转90度 */
    uint32          u32TidAlg;              /* 算法线程ID */
    uint32          u32TidTool;             /* 工具线程ID */
    SV_BOOL         bRunning;               /* 线程是否正在运行 */
    SV_BOOL         bLimit;                 /* 是否在限速区 */
    char*           modelMessageList[5];    /* 模型信息列表 */
    pthread_mutex_t mutexRunStat;           /* 算法运行状态互斥锁 */
    uint32_t          s32SerialFd[4];
    threadpool      thpool;                 /* 线程池 */
} PD_INFO_S;

/* PD 输出信息 */
typedef struct tagPdDumpInfo_S
{
    sint64          s64TimeStamp;           /* 时间戳 */
    sint32          s32GreenRoiNum;         /* 绿色ROI区域检测数量 */
    sint32          s32YellowRoiNum;        /* 黄色ROI区域检测数量 */
    sint32          s32RedRoiNum;           /* 红色ROI区域检测数量 */

    sint32          s32RedHelmetNum;           /* 红色安全帽数量 */
    sint32          s32YellowHelmetNum;        /* 黄色安全帽数量 */
    sint32          s32WhiteHelmetNum;         /* 白色安全帽数量 */
    sint32          s32BlueHelmetNum;          /* 蓝色安全帽数量 */
    sint32          s32NoHelmetNum;            /* 不带安全帽数量 */
    
    sint32          s32DistanceNum;           /* 目标数量 */
    sint32          s32DistanceXY[20*2];      /* 检测到的目标的距离坐标(x,y),上限为20个 */
} PD_DUMP_INFO_S;

typedef struct
{
    float  fx1;
    float  fy1;
    float  fx2;
    float  fy2;
}Algo_Result_f;
//映射之前

typedef struct
{
	short x1;//左上角x坐标值
	short y1;//左上角y坐标值
	short x2;//右上角x坐标值
	short y2;//右上角y坐标值
    char  alarm_type;
}Algo_Result;
//串口发送到松翰的坐标结构

typedef enum tagDisplayType
{
    DISPLAY_SINGLE = 0, //单画面
    DISPLAY_DOUBLE1, //左边
    DISPLAY_DOUBLE2, //右边
    DISPLAY_QUAD1, //左上角
    DISPLAY_QUAD2, //右上角
    DISPLAY_QUAD3, //左下角
    DISPLAY_QUAD4, //右下角
} PD_DISPLAY_TYPE;
//显示模式

PD_INFO_S m_stPdInfo = {0};             /* 模块控制信息 */
extern int ipsys_log_level;

EPdsModel pd_model_renew(EPdsModel pdsmodel);
sint32 pd_model_file(EPdsModel pdsmodel, char **modefilelist);
/* 获取模型列表信息 */
sint32 getModelListMessage(char **modelFileList, char **stringList);
/* 将算法信息发送给ipsys */
sint32 postModelListMessage(char **stringList);


/* Sigmastar硬件看门狗 */
#define WATCHDOG_IOCTL_BASE 'W'
#define WDIOS_DISABLECARD 0x0001
#define WDIOC_SETTIMEOUT _IOWR(WATCHDOG_IOCTL_BASE, 6, int)
#define WDIOC_KEEPALIVE _IOR(WATCHDOG_IOCTL_BASE, 5, int)
#define WDIOC_SETOPTIONS _IOR(WATCHDOG_IOCTL_BASE, 4, int)

enum MPP_VDEC_H264RESOL_S{
    RESOL_480P = 0,
    RESOL_720P ,
};

//送进算法的帧末尾扩展3位数据
#define extern_num  3 

uint8 SOURCE_ID = 0;
uint8 DETECT_AREA_EN = 0;
uint8 DisplayMode = 0;


uint8 H264_RESOL = 0;

#define UVC_BUF_NUM     12

// #define UVC_CAM_WIDTH   640
// #define UVC_CAM_HEIGHT  480

#define UVC_CAM_WIDTH   640
#define UVC_CAM_HEIGHT  480

#define UVC_CAM_WIDTH_2   1280
#define UVC_CAM_HEIGHT_2  720

typedef struct tagVpssAlgUvcFrame_S
{
    sint32              s32AlgChnFd;            /* UVC摄像头视频节点描述符 */
    unsigned char       *mptr[UVC_BUF_NUM];     /* UVC摄像头映射内存地址 */
    uint32              size[UVC_BUF_NUM];
} MPP_VPSS_ALG_FRAME_S;

MPP_VPSS_ALG_FRAME_S m_stVpssAlgFrame = {0};    /* UVC摄像头算法控制信息 */ 


#define GREEN_TOP_x1 392
#define GREEN_TOP_x2 632
#define GREEN_TOP_y 180

#define GREEN_BOTTOM_x1 316
#define GREEN_BOTTOM_x2 710
#define GREEN_BOTTOM_y 314

#define YELLOW_TOP_x1 316
#define YELLOW_TOP_x2 710
#define YELLOW_TOP_y 315

#define YELLOW_BOTTOM_x1 237
#define YELLOW_BOTTOM_x2 788
#define YELLOW_BOTTOM_y 448

#define RED_TOP_x1 788
#define RED_TOP_x2 237
#define RED_TOP_y 449

#define RED_BOTTOM_x1 162
#define RED_BOTTOM_x2 580
#define RED_BOTTOM_y 580

typedef struct Detect_Area_Info {
    uint32_t line_top_x1;
    uint32_t line_top_x2;
    uint32_t line_bottom_x1;
    uint32_t line_bottom_x2;
    uint32_t line_top_y;
    uint32_t line_second_y;
    uint32_t line_third_y;
    uint32_t line_bottom_y;
} Area_Info;   //梯形检测区域

typedef struct Detect_Line_Info {
    float line_k1; 
    float line_b1; 
    float line_k2; 
    float line_b2; 
} Line_Info;   //梯形检测区域

Area_Info Area_Info_Get = {0};
Line_Info Line_Info_Get = {0};

float get_line_k(uint32 x1, uint32 y1, uint32 x2, uint32 y2)
{
    float line_k = 0;
    float fx1 = x1;
    float fy1 = y1;
    float fx2 = x2;
    float fy2 = y2;

    line_k = (fy1-fy2) / (fx1 - fx2);
    return line_k;
}

float get_line_b(uint32 x1, uint32 y1, uint32 x2, uint32 y2)
{
    float line_b = 0;
    float fx1 = x1;
    float fy1 = y1;
    float fx2 = x2;
    float fy2 = y2;

    line_b = (fy1*fx2 - fy2*fx1) / (fx2 - fx1);
    return line_b; 
}

void get_line_info(Area_Info AreaInfo)  //获得梯形两条边的k值，b值
{
    Line_Info_Get.line_k1 = get_line_k(AreaInfo.line_bottom_x1, AreaInfo.line_bottom_y, AreaInfo.line_top_x1, AreaInfo.line_top_y);
    Line_Info_Get.line_b1 = get_line_b(AreaInfo.line_bottom_x1, AreaInfo.line_bottom_y, AreaInfo.line_top_x1, AreaInfo.line_top_y);
    Line_Info_Get.line_k2 = get_line_k(AreaInfo.line_bottom_x2, AreaInfo.line_bottom_y, AreaInfo.line_top_x2, AreaInfo.line_top_y);
    Line_Info_Get.line_b2 = get_line_b(AreaInfo.line_bottom_x2, AreaInfo.line_bottom_y, AreaInfo.line_top_x2, AreaInfo.line_top_y);
}

void detect_area_init(Area_Info AreaInfo)
{
   Area_Info_Get = AreaInfo;
}

int frame_sequence_detect(uint8_t Frame_last, uint8_t Frame_current)
{
    if( (Frame_last - Frame_current ) == 30)
        return SV_TRUE;
    else if( (Frame_current - Frame_last ) == 1)
        return SV_TRUE;
    else return SV_FALSE;
}

sint32 area_detect(Algo_Result result , Area_Info AreaInfo)
{
        int xr_top=0,xl_top=0;
        int xr_bottom=0,xl_bottom=0;

    if(DETECT_AREA_EN)
    {    
        //1.用y2判断在哪个报警区域.  先用 y2 算出 xl ， xr 在矩形边上的左右边缘坐标
         if(result.y2 <= AreaInfo.line_bottom_y && result.y2 >= AreaInfo.line_top_y)
         {
            xl_top = (int)(((float)result.y2 - Line_Info_Get.line_b1) / Line_Info_Get.line_k1);
            xr_top = (int)(((float)result.y2 - Line_Info_Get.line_b2) / Line_Info_Get.line_k2);

            //2. //在矩形框外， 则 return PD_ALARM_TYPE_NULL;
            if (result.x2 < xl_top || result.x1 > xr_top )
                {
                return PD_ALARM_TYPE_NULL;
                }
            else
                { 
                    if (result.y2 <= AreaInfo.line_second_y)   //y2判断在哪个报警区域
                    {
                      return PD_ALARM_TYPE_GREEN;      
                    }
                    else if(result.y2 <= AreaInfo.line_third_y)
                    {
                       return PD_ALARM_TYPE_YELLOW;      
                    }
                    else
                    {
                        return PD_ALARM_TYPE_RED;     
                    }
                    
                }
         }
         else if (result.y1 <= AreaInfo.line_bottom_y && result.y1 >= AreaInfo.line_top_y ) //3.  y2 在梯形区域外部，y1在梯形内部的情况
         {
            xl_bottom = (int)(((float)result.y1 - Line_Info_Get.line_b1) / Line_Info_Get.line_k1);
            xr_bottom = (int)(((float)result.y1 - Line_Info_Get.line_b2) / Line_Info_Get.line_k2);
                        //2. //在矩形框外， 则 return PD_ALARM_TYPE_NULL;
            if (result.x2 < xl_bottom || result.x1 > xr_bottom )
                {
                return PD_ALARM_TYPE_NULL;
                }
            else
                { 
                return PD_ALARM_TYPE_RED;         
                }         
         }
         else  //在梯形外不报警
         {
           return  PD_ALARM_TYPE_NULL;     
         }
    }
    else
    {
        if(result.y2 <= PD_ALARM_TYPE_GREEN)
             return PD_ALARM_TYPE_GREEN; 
        else if(result.y2 <= PD_ALARM_TYPE_YELLOW)
            return PD_ALARM_TYPE_YELLOW; 
        else
            return PD_ALARM_TYPE_RED; 

    }


    return  PD_ALARM_TYPE_NULL;          
} 


void coordinate_remap(Algo_Result *stAlgResult_t,  Algo_Result_f stPdResult_t,uint8 Displaymode_t,uint32_t panel_width, uint32_t panel_height)
{

   switch (Displaymode_t)
   {
    case DISPLAY_SINGLE :
        stAlgResult_t->x1 = (short)(stPdResult_t.fx1 * panel_width);
        stAlgResult_t->y1 = (short)(stPdResult_t.fy1 * panel_height);
        stAlgResult_t->x2 = (short)(stPdResult_t.fx2 * panel_width);
        stAlgResult_t->y2 = (short)(stPdResult_t.fy2 * panel_height);
    break;

    case DISPLAY_DOUBLE1:
        stAlgResult_t->x1 = (short)(stPdResult_t.fx1 * panel_width/2);
        stAlgResult_t->y1 = (short)(stPdResult_t.fy1 * panel_height);
        stAlgResult_t->x2 = (short)(stPdResult_t.fx2 * panel_width/2);
        stAlgResult_t->y2 = (short)(stPdResult_t.fy2 * panel_height);
    break;

    case DISPLAY_DOUBLE2:
        stAlgResult_t->x1 = (short)(stPdResult_t.fx1 * panel_width/2 + panel_width/2);
        stAlgResult_t->y1 = (short)(stPdResult_t.fy1 * panel_height);
        stAlgResult_t->x2 = (short)(stPdResult_t.fx2 * panel_width/2 + panel_width/2);
        stAlgResult_t->y2 = (short)(stPdResult_t.fy2 * panel_height);
    break;

    case DISPLAY_QUAD1:
        stAlgResult_t->x1 = (short)(stPdResult_t.fx1 * panel_width /2 );
        stAlgResult_t->y1 = (short)(stPdResult_t.fy1 * panel_height/2);
        stAlgResult_t->x2 = (short)(stPdResult_t.fx2 * panel_width /2);
        stAlgResult_t->y2 = (short)(stPdResult_t.fy2 * panel_height/2);
    break;

    case DISPLAY_QUAD2:
        stAlgResult_t->x1 = (short)(stPdResult_t.fx1 * panel_width /2  + panel_width/2);
        stAlgResult_t->y1 = (short)(stPdResult_t.fy1 * panel_height/2);
        stAlgResult_t->x2 = (short)(stPdResult_t.fx2 * panel_width /2  + panel_width/2 );
        stAlgResult_t->y2 = (short)(stPdResult_t.fy2 * panel_height/2);
    break;

    case DISPLAY_QUAD3:
        stAlgResult_t->x1 = (short)(stPdResult_t.fx1 * panel_width /2 );
        stAlgResult_t->y1 = (short)(stPdResult_t.fy1 * panel_height/2 + panel_height/2);
        stAlgResult_t->x2 = (short)(stPdResult_t.fx2 * panel_width /2 );
        stAlgResult_t->y2 = (short)(stPdResult_t.fy2 * panel_height/2 + panel_height/2);
    break;    
    
    case DISPLAY_QUAD4:
        stAlgResult_t->x1 = (short)(stPdResult_t.fx1 * panel_width /2 + panel_width/2);
        stAlgResult_t->y1 = (short)(stPdResult_t.fy1 * panel_height/2 + panel_height/2);
        stAlgResult_t->x2 = (short)(stPdResult_t.fx2 * panel_width /2 + panel_width/2 );
        stAlgResult_t->y2 = (short)(stPdResult_t.fy2 * panel_height/2 + panel_height/2);
    break;
   
    default:
        stAlgResult_t->x1 = (short)(stPdResult_t.fx1 * panel_width);
        stAlgResult_t->y1 = (short)(stPdResult_t.fy1 * panel_height);
        stAlgResult_t->x2 = (short)(stPdResult_t.fx2 * panel_width);
        stAlgResult_t->y2 = (short)(stPdResult_t.fy2 * panel_height);
    break;
   }
}


#define RINGBUF_COUNT       12  //每路3帧
#define RINGBUF_SIZE       PD_IMAGE_WIDTH*PD_IMAGE_HEIGHT*3+extern_num//PD_IMAGE_WIDTH *  PD_IMAGE_HEIGHT *3 //   RGB //642048 //
//static uint8_t *ringbuf[RINGBUF_COUNT] = {0};
TRWRingQueue<int> ringque(RINGBUF_COUNT-1);


#define RINGBUF_VDEC_COUNT       12
#define RINGBUF_VDEC_SIZE       UVC_CAM_WIDTH*UVC_CAM_HEIGHT
#define RINGBUF_VDEC_SIZE_2     UVC_CAM_WIDTH_2*UVC_CAM_HEIGHT_2
TRWRingQueue<int> ringque_VDEC(RINGBUF_VDEC_COUNT-1);

uint32_t TEST_RINGCOUNT = 0;

// int Task_RingbufWriter(TRWRingQueue<int> &ringque, bool &wrun)
// {
//     int counter = 0;

//     //cout << "###### writer begin: " << GetMtimestamp() << " size:" << ringque.queue_.size() << endl;
//    // printf("###### writer begin: %llu, size: %u\n", GetMtimestamp(), ringque.queue_.size());
//     wrun = true;
//     while (wrun) {
//             int index = counter % RINGBUF_COUNT;
//             // *** request resource, get frame from socket here
//             *ringbuf[index] = counter;

//             // *** write resource
//             int rel = ringque.Write(index, [](int &release){ 
//                 return ++release; 
//             });


//             //cout << "  ~~~ writer ~~~ " << GetMtimestamp() << " size:" << ringque.queue_.size() << " index:" << index << " value:" << *ringbuf[index] << endl;
//         //    printf("  ~~~ writer ~~~ %llu, size: %u, index: %d, value: %#x\n", GetMtimestamp(), ringque.queue_.size(), index, (uint32_t)(*ringbuf[index]));
//             if (rel > 0) { // release dequeued resource
//                 printf("        --- relese dequeued resource: %d\n", --rel);
//             }
//             counter++;
//     }
    
//     printf("###### writer end\n");
//     return 0;
// }


// int Task_RingbufReader(TRWRingQueue<int> &ringque, bool &rrun)
// {
//     int chn = 0;    int chn = 0;
//     int32_t timeout = 500000; 
//     int index = -1;
//     int32_t timeout = 500000; 
//     int index = -1;

//     //cout << "$$$$$$ reader" << " begin: " << GetMtimestamp() << " rpos:" << ringque.rpos_[0] << endl;
//  //   printf("$$$$$$ reader begin: %llu, rpos: %d\n", GetMtimestamp(), ringque.rpos_[0]);
//     rrun = true;
//     while (rrun) {
//         // *** read resource
//         index = ringque.Read(chn, timeout, [/*UtilizeResource*/&ringque](int &current){
//             //cout << "    --- reader" << " --- " << GetMtimestamp() << " rpos:" << ringque.rpos_[0] << " index:" << current << " value:" << *ringbuf[current] << endl;
//             //printf("    --- reader --- %llu, rpos: %d, index: %d, value: %#x\n", GetMtimestamp(), ringque.rpos_[0], current, (uint32_t)(*ringbuf[current]));
//             return current; //0;
//         });

//         // *** utilize resource, put frame to alg here
//     //    printf("    --- reader --- %llu, rpos: %d, index: %d, value: %#x\n", GetMtimestamp(), ringque.rpos_[0], index, (uint32_t)(*ringbuf[index]));
//     }

//     printf("$$$$$$ reader end\n");
//     return 0;
// }
void Yuv420ToRGB_640x480(unsigned char *yuv , unsigned char *vir_addr_dst)
{
     int Y, Cr, Cb;
     int R, G, B;
     unsigned char vir_addr_src[640*480*3]	= {0};

   //  unsigned char img_rgb[640*480*4] = {0}, 
     unsigned char  Y_buf[640*480], U_buf[640*480], V_buf[640*480]; // 640 * 480 * 3
    // int R, G, B;
   // int Y, Cr, Cb;
    int k, p1, p2, p3;

    //printf("source ID :%d",*(yuv+32));
  //  sonix_source_ID = *(yuv+32);
  //  Shared_memory->video_info.source_ID = *(yuv+32);

    p1 = 0;
    p2 = 0;
    p3 = 0;
    for (int n = 0; n<240; n += 1) // 有240个2行
    {
        for (int i = 0; i<240; i += 1) // 每2行有 240x8个数据
        {
            if (i < 160)
            {
                for (int k = 0; k<8; k += 1)  //*(yuv + i*640*3/2 + j)
                {
                    if (k<4)
                    {
                        Y_buf[p1++] = *(yuv +  1920*n + 8*i + k);
                    }
                    else
                    {
                        if ((k-4)%2 == 0) // 数据U
                        {
                            U_buf[p2++] = *(yuv +  1920*n + 8*i + k);
                        }
                        else
                        {
                            V_buf[p3++] = *(yuv +  1920*n + 8*i + k);
                        }
                    }
                }
            }
            else
            {
                for (int k = 0; k<8; k += 1)
                {
                    Y_buf[p1++] = *(yuv + 1920*n + 8*i + k);
                }    
            }
        }
    }
    uint32 index_dst = 0; 

    for (int i = 0; i<480; i += 1) // 行数
    {
        if(i % 4 == 0)
        continue;
        if( (i+22) % 60 == 0)
        continue;

        for (int j = 0; j<640; j += 1) // 列数
        {
       if( j%20 == 0)
       continue;   

            Y = Y_buf[640*i + j];
            Cr = U_buf[320*(i/2) + j/2];
            Cb = V_buf[320*(i/2) + j/2];

            #if 1
            R = ((298*(Y-16) + 409*(Cr-128) + 128)>>8 > 255                ? 255 : (298*(Y-16) + 409*(Cr-128) + 128)>>8);
            G = ((298*(Y-16) - 100*(Cb-128) - 208*(Cr-128) + 128)>>8 > 255 ? 255 : (298*(Y-16) - 100*(Cb-128) - 208*(Cr-128) + 128)>>8);   
            B = ((298*(Y-16) + 516*(Cb-128) + 128)>>8 > 255                ? 255 : (298*(Y-16) + 516*(Cb-128) + 128)>>8);
            #else
            R = ((298*(Y-16) + 128)>>8 > 255 ? 255 : (298*(Y-16) + 128)>>8);
            G = ((298*(Y-16) + 128)>>8 > 255 ? 255 : (298*(Y-16) + 128)>>8);
            B = ((298*(Y-16) + 128)>>8 > 255 ? 255 : (298*(Y-16) + 128)>>8);
            #endif

            vir_addr_dst[index_dst ++] = B<0 ? 0 : B;
            vir_addr_dst[index_dst ++] = G<0 ? 0 : G;
            vir_addr_dst[index_dst ++] = R<0 ? 0 : R;

            if(index_dst > RINGBUF_SIZE)
                break;

        }
    }

           vir_addr_dst[0] = *(yuv+32); //第0位字节储存source_id(通道号)
           vir_addr_dst[1] = *(yuv+38); //第1位字节储存检测区域是否开启(通道号)
       //    printf("source_id:%d\n",vir_addr_dst[0]);
}

void mpp_vpss_Yuv420ToRGB(unsigned char *yuv , unsigned char *vir_addr_dst)
{
     int Y, Cr, Cb;
     int R, G, B;
     unsigned char vir_addr_src[640*480*3]	= {0};

   //  unsigned char img_rgb[640*480*4] = {0}, 
     unsigned char  Y_buf[640*480], U_buf[640*480], V_buf[640*480]; // 640 * 480 * 3
    // int R, G, B;
   // int Y, Cr, Cb;
    int k, p1, p2, p3;

    //printf("source ID :%d",*(yuv+32));
  //  sonix_source_ID = *(yuv+32);
  //  Shared_memory->video_info.source_ID = *(yuv+32);

    int width = UVC_CAM_WIDTH;
    int height = UVC_CAM_HEIGHT;

    p1 = 0;
    p2 = 0;
    p3 = 0;
      for (int n = 0; n<(height/2); n += 1) // 有240个2行
    {
        for (int i = 0; i<(3*width/8); i += 1) // 每2行有 240x8个数据
        {
            if (i < (width/4))
            {
                for (int k = 0; k<8; k += 1)  //*(yuv + i*640*3/2 + j)
                {
                    if (k<4)
                    {
                        Y_buf[p1++] = *(yuv +  3*width*n  + 8*i + k );
                    }
                    else
                    {
                        if ((k-4)%2 == 0) // 数据U
                        {
                            U_buf[p2++] = *(yuv +  3*width*n  + 8*i + k );
                        }
                        else
                        {
                            V_buf[p3++] = *(yuv +  3*width*n  + 8*i + k );
                        }
                    }
                }
            }
            else
            {
                for (int k = 0; k<8; k += 1)
                {
                    Y_buf[p1++] = *(yuv + 3*width*n  + 8*i + k);
                }    
            }
        }
    }

        int index_dst = 0;
    for (int i = 0; i<height; i += 1) // 行数
    {
        if(i % 45 == 0)
        continue;

        for (int j = 0; j<width; j += 1) // 列数
        {
            if( j%20 == 0)
            continue;     

            //这里丢列   丢32列
            Y = Y_buf[640*i + j];
            Cr = U_buf[320*(i/2) + j/2];
            Cb = V_buf[320*(i/2) + j/2];
            #if 1
            R = ((298*(Y-16) + 409*(Cr-128) + 128)>>8 > 255                ? 255 : (298*(Y-16) + 409*(Cr-128) + 128)>>8);
            G = ((298*(Y-16) - 100*(Cb-128) - 208*(Cr-128) + 128)>>8 > 255 ? 255 : (298*(Y-16) - 100*(Cb-128) - 208*(Cr-128) + 128)>>8);   
            B = ((298*(Y-16) + 516*(Cb-128) + 128)>>8 > 255                ? 255 : (298*(Y-16) + 516*(Cb-128) + 128)>>8);
            #else
            R = ((298*(Y-16) + 128)>>8 > 255 ? 255 : (298*(Y-16) + 128)>>8);
            G = ((298*(Y-16) + 128)>>8 > 255 ? 255 : (298*(Y-16) + 128)>>8);
            B = ((298*(Y-16) + 128)>>8 > 255 ? 255 : (298*(Y-16) + 128)>>8);
            #endif

            vir_addr_dst[index_dst ++] = B<0 ? 0 : B;
            vir_addr_dst[index_dst ++] = G<0 ? 0 : G;
            vir_addr_dst[index_dst ++] = R<0 ? 0 : R;

        }
    }

            vir_addr_dst[0] = *(yuv+32); //第0位字节储存source_id(通道号)
            vir_addr_dst[1] = *(yuv+38);
       //    printf("source_id:%d\n",vir_addr_dst[0]);
}


#define INBUF_SIZE 240*360
bool sendFrame_f = 0;
static void *GetMediaBuffer(void *arg) {
  (void)arg;
  MEDIA_BUFFER mb = NULL;
  int ret = 0;

  MPP_CHN_S VdecChn;
  VdecChn.enModId = RK_ID_VDEC;
  VdecChn.s32DevId = 0;
  VdecChn.s32ChnId = 0;

 //   while(1)
 //   {

     mb = RK_MPI_SYS_GetMediaBuffer(RK_ID_VDEC, 0, 5000);
     if (!mb) {
    printf("RK_MPI_SYS_GetMediaBuffer get null buffer in 5s...\n");
    return NULL;
    }


   remove("./test.yuv");
   FILE *fp = fopen("./test.yuv", "wb+");
   //fwrite(mb, 1, 1382400, fp);
   fwrite(RK_MPI_MB_GetPtr(mb), 1, RK_MPI_MB_GetSize(mb), fp);
   fclose(fp);

#if 0
  MB_IMAGE_INFO_S stImageInfo = {0};
  ret = RK_MPI_MB_GetImageInfo(mb, &stImageInfo);
  if (ret) {
    printf("Get image info failed! ret = %d\n", ret);
    RK_MPI_MB_ReleaseBuffer(mb);
    return NULL;
  }
  printf("Get Frame:ptr:%p, fd:%d, size:%zu, mode:%d, channel:%d, "
         "timestamp:%lld, ImgInfo:<wxh %dx%d, fmt 0x%x>\n",
         RK_MPI_MB_GetPtr(mb), RK_MPI_MB_GetFD(mb), RK_MPI_MB_GetSize(mb),
         RK_MPI_MB_GetModeID(mb), RK_MPI_MB_GetChannelID(mb),
         RK_MPI_MB_GetTimestamp(mb), stImageInfo.u32Width,
         stImageInfo.u32Height, stImageInfo.enImgType);
  RK_MPI_MB_ReleaseBuffer(mb);
#endif

 //   }

  return NULL;
}

void H264_test()
{
    printf("******H264 test******\n");
    int ret = 0; 
    bool quit = false;
    RK_U32 u32Loop = 0;
    RK_BOOL bIsHardware = RK_TRUE;

     FILE *infile = fopen("./example", "rb");
        if (!infile) {
          fprintf(stderr, "Could not open mpp_dec_test.h264\n");
          return 0;
        }

  RK_MPI_SYS_Init();


    VDEC_CHN_ATTR_S stVdecAttr;

    stVdecAttr.enCodecType = RK_CODEC_TYPE_H264;
    stVdecAttr.enMode = VIDEO_MODE_FRAME;
    stVdecAttr.enDecodecMode = VIDEO_DECODEC_HADRWARE;

    ret = RK_MPI_VDEC_CreateChn(0, &stVdecAttr);
      if (ret) {
    printf("Create Vdec[0] failed! ret=%d\n", ret);
    return -1;
  }

    pthread_t read_thread;
    pthread_create(&read_thread, NULL, GetMediaBuffer, NULL);

    int data_size;
  int read_size;
  if (stVdecAttr.enMode == VIDEO_MODE_STREAM) {
    data_size = INBUF_SIZE;
  } else if (stVdecAttr.enMode == VIDEO_MODE_FRAME) {
    fseek(infile, 0, SEEK_END);
    data_size = ftell(infile);
    fseek(infile, 0, SEEK_SET);
  }

  while (!quit) {
    MEDIA_BUFFER mb = RK_MPI_MB_CreateBuffer(data_size, RK_FALSE, 0);
  RETRY:
    /* read raw data from the input file */
    read_size = fread(RK_MPI_MB_GetPtr(mb), 1, data_size, infile);
    printf("*****read_size:%d*******\n",read_size);
    if (!read_size || feof(infile)) {  //feof 检测流文件上的结束符
      if (u32Loop) {
        fseek(infile, 0, SEEK_SET);
        goto RETRY;
      } else {
        RK_MPI_MB_ReleaseBuffer(mb);
        break;
      }
    }
    RK_MPI_MB_SetSize(mb, read_size);
    printf("#Send packet(%p, %zuBytes) to VDEC[0].\n", RK_MPI_MB_GetPtr(mb),
           RK_MPI_MB_GetSize(mb));
    ret = RK_MPI_SYS_SendMediaBuffer(RK_ID_VDEC, 0, mb);
    RK_MPI_MB_ReleaseBuffer(mb);

    usleep(30 * 1000);
  }

   quit = true;
  pthread_join(read_thread, NULL);

  RK_MPI_VDEC_DestroyChn(0);
  fclose(infile);


}

sint32 mpp_vpss_InitAlgChn()
{
    sint32 s32Ret = 0;
    uint32 u32BufNum = 0, ret;
    char path[64] = {0};
    //打开设备
    int path_id = 0;
    int fd = 0;
    struct v4l2_capability vcap;


while(1)
{   
    if(path_id > 50)
    {
       return SV_FAILURE;
    }

    sprintf(path, "/dev/video%d",path_id);
    printf("open video node :%s ",path);
    path_id ++;
    fd = open(path, O_RDWR);
    if (fd < 0)
    {
        //print_level(SV_ERROR, "open device fail\n");
        continue;
    }
    else
    {
        struct v4l2_capability vcap;
        ioctl(fd, VIDIOC_QUERYCAP, &vcap);
        if (!(V4L2_CAP_VIDEO_CAPTURE & vcap.capabilities)) 
        {
           // print_level(SV_ERROR, "vcap.capabilities = %d", vcap.capabilities);
           // print_level(SV_ERROR,"Error: No capture video device!\n");
            continue;
        }
        break;
    }
}  
    // //打开设备
    // int fd = open("/dev/video25", O_RDWR);
    // if (fd < 0)
    // {
    //     print_level(SV_ERROR, "open device fail\n");
    //     return SV_FAILURE;
    // }
    
    // struct v4l2_capability vcap;
    // ioctl(fd, VIDIOC_QUERYCAP, &vcap);
    // if (!(V4L2_CAP_VIDEO_CAPTURE & vcap.capabilities)) 
    // {
    //     print_level(SV_ERROR, "vcap.capabilities = %d", vcap.capabilities);
    //     perror("Error: No capture video device!\n");
    //     return SV_FAILURE;
    // }
    
    //获取摄像头支持的格式
    struct v4l2_fmtdesc v4fmt;
    v4fmt.index = 0;
    v4fmt.type  = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    print_level(SV_INFO, "support format list:\n");
    while (ioctl(fd,VIDIOC_ENUM_FMT, &v4fmt) == 0)
    {
        print_level(SV_INFO, "v4l2_format%d:%s\n",v4fmt.index, v4fmt.description);
        v4fmt.index++;
    }
    
    struct v4l2_frmsizeenum frmsize;
    frmsize.index = 0;
    frmsize.type  = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    print_level(SV_INFO, "support resolution list:\n");
    frmsize.pixel_format = V4L2_PIX_FMT_H264;
    while(ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) == 0)
    {
        print_level(SV_INFO, "frame_size<%d*%d>\n", frmsize.discrete.width, frmsize.discrete.height);
        frmsize.index++;
    }
    
    //设置摄像头支持的格式
    struct v4l2_format vFormat;
    vFormat.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    // if(H264_RESOL == RESOL_480P)
    // {
    // vFormat.fmt.pix.width   = UVC_CAM_WIDTH;
    // vFormat.fmt.pix.height  = UVC_CAM_HEIGHT;
    // }
    // else if(H264_RESOL == RESOL_720P)
    // {

    vFormat.fmt.pix.width   = UVC_CAM_WIDTH_2;
    vFormat.fmt.pix.height  = UVC_CAM_HEIGHT_2;   

    // }
    // else
    // {
    // vFormat.fmt.pix.width   = UVC_CAM_WIDTH;
    // vFormat.fmt.pix.height  = UVC_CAM_HEIGHT;    
    // }


    //vFormat.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    vFormat.fmt.pix.pixelformat  = V4L2_PIX_FMT_H264;
    ret = ioctl(fd, VIDIOC_S_FMT, &vFormat);
    if(ret < 0)
    {
        print_level(SV_ERROR, "set format fail\n");
        return SV_FAILURE;
    }
    
    //申请内核空间
    struct v4l2_requestbuffers vqbuff;
    vqbuff.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    vqbuff.count  = UVC_BUF_NUM;
    vqbuff.memory = V4L2_MEMORY_MMAP;
    ret = ioctl(fd, VIDIOC_REQBUFS, &vqbuff);
    if (ret<0)
    {
        print_level(SV_ERROR, "requrey buf fail\n");
        return SV_FAILURE;
    }

    //申请内存空间
    struct v4l2_buffer vbuff;
    for(int i=0; i<UVC_BUF_NUM; i++)
    {
        memset (&vbuff, 0, sizeof (struct v4l2_buffer));
        vbuff.index  = i;        
        vbuff.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        vbuff.memory = V4L2_MEMORY_MMAP;
        ret=ioctl(fd, VIDIOC_QUERYBUF, &vbuff);
        if (ret<0)
        {
            print_level(SV_ERROR, "requrey memory fail\n");
            continue;
        }
        
        m_stVpssAlgFrame.mptr[i]= (unsigned char *)mmap(NULL, vbuff.length, PROT_READ, MAP_SHARED, fd, vbuff.m.offset);
        if (m_stVpssAlgFrame.mptr[i] == MAP_FAILED)
        {
            print_level(SV_ERROR, "mmap failed. i=%d\n", i);
            continue;
        }

        m_stVpssAlgFrame.size[i] = vbuff.length;

        u32BufNum++;
    }

    if (u32BufNum <= 0)
    {
        print_level(SV_ERROR, "v4l2 mmap failed.\n");
        return SV_FAILURE;
    }
    
    for(int i=0; i<u32BufNum; i++)
    {
        vbuff.index  = i;        
        vbuff.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        vbuff.memory = V4L2_MEMORY_MMAP; //缓冲帧放入缓冲队列
        ret=ioctl(fd, VIDIOC_QBUF, &vbuff);
        if(ret<0)
        {
            print_level(SV_ERROR, "put fail\n");
            return SV_FAILURE;
        }
    }


    //开始采集
    enum v4l2_buf_type bufType;
    bufType = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ret = ioctl(fd, VIDIOC_STREAMON, &bufType);
    if(ret<0)
    {
        print_level(SV_ERROR, "open fail\n");
        return SV_FAILURE;
    }

    m_stVpssAlgFrame.s32AlgChnFd = fd;

    return SV_SUCCESS;
}

sint32 mpp_vpss_Deinitialize()
{
    sint32 ret = 0;
    enum v4l2_buf_type bufType;


    //停止采集
    bufType = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ret = ioctl(m_stVpssAlgFrame.s32AlgChnFd, VIDIOC_STREAMOFF, &bufType);

    if(ret<0)
    {
        print_level(SV_ERROR, "open fail\n");
        return SV_FAILURE;
    }

    //释放映射
    for(int i=0; i<UVC_BUF_NUM; i++)
    munmap(m_stVpssAlgFrame.mptr[i], m_stVpssAlgFrame.size[i]);  

    //关闭设备
    close(m_stVpssAlgFrame.s32AlgChnFd);

    printf("v4l2 Deinitialize succ\n");

    return SV_SUCCESS;
}

unsigned char dstData[RINGBUF_COUNT][RINGBUF_SIZE];
unsigned char MptrData[RINGBUF_VDEC_COUNT][RINGBUF_VDEC_SIZE_2];
int s32time_out = 0;

sint32 mpp_vpss_GetAlgFrame()
{
    sint32 s32Ret = 0 ,ret = 0;
    void *pbmp = NULL;
    sint32 s32Fd = m_stVpssAlgFrame.s32AlgChnFd;
    sint32 s32BufIndex = 0;
    int counter = 0;

    uint32 u32BufNum = 0;
    uint32 u32DstSize = 0; 

    static struct timespec tvNow = {0, 0};
    static struct timespec tvLast = {0, 0};
    clock_gettime(CLOCK_MONOTONIC, &tvNow);

    uint32_t  u32StepTimeMs = 0;
    uint32_t  u32cnt = 0;
    uint32_t  u32sum = 0;

 // uint32_t Getframe_CH = 0;
 //   uint32_t cnt = 0;
    sint32 Index_temp = 0;  //记录上一帧在哪个缓存
    uint8_t frame_sequence = 0; //=1 时 交换帧
    sint32 frame_skip = 10; //前10帧不做检测

    enum v4l2_buf_type bufType = 0;

    uint8_t resol_change = 0;  //分辨率变换后，帧序号会被跳过
    
    uint8_t Reset_Frame_done[4] = {0};

    uint8_t u8ResetFrameBuf[4] = {0};
    u8ResetFrameBuf[0] = 0Xff;
    u8ResetFrameBuf[1] = 0Xbb;

    uint8_t test_cnt = 0;

//  uint8_t skip_frame = 10; //初始的帧会不连续，需要跳过前几帧

    while(1)   
    {
    //     clock_gettime(CLOCK_MONOTONIC, &tvNow);
    //     u32StepTimeMs = ((tvNow.tv_sec*1000 + tvNow.tv_nsec/1000000) - (tvLast.tv_sec*1000 + tvLast.tv_nsec/1000000));
    //     tvLast = tvNow;
    //    // printf("****delay ms__get pic:%d ***** \n",u32StepTimeMs);

    //     u32sum += u32StepTimeMs;
    //     u32cnt ++;
    //     if(u32cnt >10) //100*50ms = 5s
    //     {
    //        printf("****delay ms__get pic:%d ***** \n",u32sum / u32cnt );
    //        u32cnt =  0;
    //        u32sum =  0;
    //     }

       if (s32Fd <= 0)
       {
        print_level(SV_ERROR, "uvc fd is invalid! fd: %d\n", s32Fd);
        return SV_FAILURE;
       }
       
       struct pollfd tFds[1];
       /* poll */
       tFds[0].fd     = s32Fd;
       tFds[0].events = POLLIN;
       s32Ret = poll(tFds, 1, -1);
       if (s32Ret <= 0)
       {
           print_level(SV_ERROR, "poll error!\n");
           return SV_FAILURE;
       }
       //printf("poll .....\n");

       //从队列中取数据
       struct v4l2_buffer readbuff;
       memset(&readbuff, 0, sizeof(struct v4l2_buffer));
       readbuff.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
       readbuff.memory = V4L2_MEMORY_MMAP;
       s32Ret = ioctl(s32Fd, VIDIOC_DQBUF, &readbuff);
       if(s32Ret < 0)
       {
           print_level(SV_ERROR, "read fail!\n");
           return SV_FAILURE;
       }
       s32BufIndex = readbuff.index; //12帧缓存

       //printf("VIDIOC_DQBUF .....\n");

    //    if(skip_frame > 0)
    //    {
    //         skip_frame --;
           
    //         s32Ret = ioctl(s32Fd, VIDIOC_QBUF, &readbuff);
    //         if (s32Ret < 0)
    //         {
    //             print_level(SV_ERROR, "put fail!\n");
    //             return SV_FAILURE;
    //         } 
            
    //         continue;
    //    }


        int rel = 0;

    uint8_t Getframe_CH = *(m_stVpssAlgFrame.mptr[s32BufIndex] +32);     

    //printf("___%d___\n",(*(m_stVpssAlgFrame.mptr[s32BufIndex] +34)) );   

    #if 1
        if(! (*(m_stVpssAlgFrame.mptr[s32BufIndex] +34)&0x80) ) //切到单通道变成720p
        {
            if(H264_RESOL !=  RESOL_720P)
            {
                H264_RESOL = RESOL_720P;
                // system("rm 480p_arg");
                // system("touch 720p_arg");
                // // system("touch /root/err_exit");
                printf("480p -> 720p:%d \n",Getframe_CH);
                Reset_Frame_done[Getframe_CH] = 0;

                resol_change = 1;

                // rel = ringque_VDEC.Clear([](int &release){ 
                //     return ++release; 
                // })

                s32Ret = ioctl(s32Fd, VIDIOC_QBUF, &readbuff);
                if (s32Ret < 0)
                {
                    print_level(SV_ERROR, "put fail!\n");
                    return SV_FAILURE;
                } 
                continue;
                // mpp_vpss_Deinitialize();
                // system("pkill -9  alg");              
            }
        }
        else   //切到分割画面变成480p
        {
            if(H264_RESOL !=  RESOL_480P)
            {
                H264_RESOL = RESOL_480P;
                // system("rm 720p_arg");
                // system("touch 480p_arg");
                // // system("touch /root/err_exit");
                printf("720p -> 480p:%d \n",Getframe_CH);
                for (uint8_t i = 0; i < 4; i++)
                {
                   Reset_Frame_done[i] = 0;
                }

                resol_change = 1;
                
                //Reset_Frame_done[Getframe_CH] = 0;

                // rel = ringque_VDEC.Clear([](int &release){ 
                //     return ++release; 
                // });

                s32Ret = ioctl(s32Fd, VIDIOC_QBUF, &readbuff);
                if (s32Ret < 0)
                {
                    print_level(SV_ERROR, "put fail!\n");
                    return SV_FAILURE;
                } 
                continue;
                // mpp_vpss_Deinitialize();
                // system("pkill -9  alg");
            }

        }
    #endif

    #if 1
        if(Reset_Frame_done[Getframe_CH] == 0)
        {
            if( *(m_stVpssAlgFrame.mptr[s32BufIndex] +6) == 0)
                {   
                Reset_Frame_done[Getframe_CH] = 1;
                printf("Reset_Frame_done:%d \n",Getframe_CH);
                }
            else
                {
                s32Ret = ioctl(s32Fd, VIDIOC_QBUF, &readbuff);
                    if (s32Ret < 0)
                    {
                        print_level(SV_ERROR, "put fail!\n");
                        return SV_FAILURE;
                    }     
                printf("wait_IFrame :%d \n",Getframe_CH);
                u8ResetFrameBuf[2] = Getframe_CH;
                sint32 s32WriteLen = write(m_stPdInfo.s32SerialFd[0], u8ResetFrameBuf, 4);
                continue;
                }
        }
    #endif

        int index = counter % RINGBUF_VDEC_COUNT; //V4L2 读帧不一定会按顺序执行，送环形队列的标签按实际 readbuff.index 


             if(H264_RESOL == RESOL_480P)
             {
                memcpy(MptrData[index],m_stVpssAlgFrame.mptr[s32BufIndex], RINGBUF_VDEC_SIZE);
             }
             else if(H264_RESOL == RESOL_720P)
             {
                memcpy(MptrData[index],m_stVpssAlgFrame.mptr[s32BufIndex], RINGBUF_VDEC_SIZE_2);    
             }
             else
             {
                memcpy(MptrData[index],m_stVpssAlgFrame.mptr[s32BufIndex], RINGBUF_VDEC_SIZE);  
             }

      printf("CH[%d]:[%d]-[%d]\n",  m_stVpssAlgFrame.mptr[s32BufIndex][32],m_stVpssAlgFrame.mptr[s32BufIndex][6],m_stVpssAlgFrame.mptr[s32BufIndex][7]);

    // if(frame_skip != 0)
    // {
    //     frame_skip --;
    // }
    // else
    // {
    //   if( !(frame_sequence_detect(test_cnt,m_stVpssAlgFrame.mptr[s32BufIndex][7])))
    //        {
    //         print_level(SV_ERROR, "frame sequence err now:%d,last:%d!\n", m_stVpssAlgFrame.mptr[s32BufIndex][7] ,test_cnt);
    //         if( (m_stVpssAlgFrame.mptr[s32BufIndex][7] - test_cnt) > 2)
    //         system("pkill -9  alg");
    //        }
    // }
    //     test_cnt = m_stVpssAlgFrame.mptr[s32BufIndex][7];
    

    #if 1    //帧倒序检测
        if(frame_sequence)
        {    //先发当前帧
            int rel1 = ringque_VDEC.Write(index, [](int &release){     
                    return ++release; 
                });
            //再发上一帧
            int rel2 = ringque_VDEC.Write(Index_temp, [](int &release){ 
                    return ++release; 
                });
            MptrData[index][7] =  MptrData[Index_temp][7];
            frame_sequence = 0;
        }
        else
        { 
           if(resol_change)
           {
                frame_skip = 10; //跳过10帧
                resol_change = 0;
           }

           if(frame_skip > 0) //开始，两者都为0    //前10帧不做检测
           {
                    frame_skip --;
                       // *** write resource
                    rel = ringque_VDEC.Write(index, [](int &release){ 
                        return ++release; 
                    });

                    if (rel > 0) { // release dequeued resource
                       // printf("--- relese dequeued resource: %d\n", --rel);
                    }
           }
        //    else if()
        //     {

        //     }
            else if( !(frame_sequence_detect(MptrData[Index_temp][7],MptrData[index][7])))
           {
               print_level(SV_ERROR, "frame sequence err now[%d]:%d,last[%d]:%d!\n", index,MptrData[index][7] , Index_temp,MptrData[Index_temp][7]);
               frame_sequence = 1;
           }
           else
           {
                // *** write resource
                rel = ringque_VDEC.Write(index, [](int &release){ 
                    return ++release; 
                });

                if (rel > 0) { // release dequeued resource
                   // printf("        --- relese dequeued resource: %d\n", --rel);
                }
           }
       }

        Index_temp = index; 
    #else
        rel = ringque_VDEC.Write(index, [](int &release){ 
        return ++release; 
    });

    #endif
        counter ++;

    // if(*(m_stVpssAlgFrame.mptr[s32BufIndex] +6) == 0)
    //     printf("** vdec write[%d] I:%d  \n",index, *(m_stVpssAlgFrame.mptr[s32BufIndex] +7));
    // else
    //     printf("** vdec write[%d] P:%d  \n",index, *(m_stVpssAlgFrame.mptr[s32BufIndex] +7));    

       //将缓冲区重新放入队列
       // MS_P(1);
       s32Ret = ioctl(s32Fd, VIDIOC_QBUF, &readbuff);
       if (s32Ret < 0)
       {
           print_level(SV_ERROR, "put fail!\n");
           return SV_FAILURE;
       }    

     // clock_gettime(CLOCK_MONOTONIC, &tvNow);
     //  u32StepTimeMs = ((tvNow.tv_sec*1000 + tvNow.tv_nsec/1000000) - (tvLast.tv_sec*1000 + tvLast.tv_nsec/1000000));


    }

    return SV_SUCCESS;
}

sint32 mpp_vdec_init()
{
    sint32 s32Ret = 0 ,ret = 0;
    printf("mpp_vdec_init\n");
    VDEC_CHN_ATTR_S stVdecAttr; 
    stVdecAttr.enCodecType =  RK_CODEC_TYPE_H264 ;
    stVdecAttr.enMode = VIDEO_MODE_FRAME; //VIDEO_MODE_STREAM    VIDEO_MODE_FRAME   JPEG流才用FRAME
    stVdecAttr.enDecodecMode = VIDEO_DECODEC_HADRWARE ;  //VIDEO_DECODEC_HADRWARE    VIDEO_DECODEC_SOFTWARE
    uint32 u32Size;

    MPP_CHN_S stSrcChn;
    MPP_CHN_S stDestChn;

    RK_MPI_SYS_Init();

    for(int i = 0; i<4 ; i++)
    {
        s32Ret = RK_MPI_VDEC_CreateChn(i, &stVdecAttr);
        if(s32Ret != RK_SUCCESS)
        {
            print_level(SV_ERROR, "RK_MPI_VDEC_CreateChn[%d] failed. [err=%d]\n",i, s32Ret);
        }

            uint32_t u32Width,u32Height = 0;

             u32Width   = UVC_CAM_WIDTH;
             u32Height  = UVC_CAM_HEIGHT;


            RGA_ATTR_S stRgaAttr;
            stRgaAttr.bEnBufPool = RK_TRUE;
            stRgaAttr.u16BufPoolCnt = 2;
            stRgaAttr.u16Rotaion = 0;
            stRgaAttr.stImgIn.u32X = 0;
            stRgaAttr.stImgIn.u32Y = 0;
            stRgaAttr.stImgIn.imgType = IMAGE_TYPE_NV12;
            stRgaAttr.stImgIn.u32Width = u32Width;
            stRgaAttr.stImgIn.u32Height = u32Height;
            stRgaAttr.stImgIn.u32HorStride = u32Width;
            stRgaAttr.stImgIn.u32VirStride = u32Height;
            stRgaAttr.stImgOut.u32X = 0;
            stRgaAttr.stImgOut.u32Y = 0;
            stRgaAttr.stImgOut.imgType = IMAGE_TYPE_BGR888;//IMAGE_TYPE_RGB888;
            stRgaAttr.stImgOut.u32Width =     PD_IMAGE_WIDTH;
            stRgaAttr.stImgOut.u32Height =    PD_IMAGE_HEIGHT;
            stRgaAttr.stImgOut.u32HorStride = PD_IMAGE_WIDTH;
            stRgaAttr.stImgOut.u32VirStride = PD_IMAGE_HEIGHT;
            ret = RK_MPI_RGA_CreateChn(i, &stRgaAttr);
            if (ret) {
               print_level(SV_ERROR, "RK_MPI_RGA_CreateChn[%d] failed. [err=%d]\n",i, s32Ret);
              return -1;
            }

              stSrcChn.enModId = RK_ID_VDEC;
              stSrcChn.s32DevId = 0;
              stSrcChn.s32ChnId = i;
              
              stDestChn.enModId = RK_ID_RGA;
              stDestChn.s32DevId = 0;
              stDestChn.s32ChnId = i;
              ret = RK_MPI_SYS_Bind(&stSrcChn, &stDestChn);    
    }

    for(int i = 4; i<8 ; i++)
    {
        s32Ret = RK_MPI_VDEC_CreateChn(i, &stVdecAttr);
        if(s32Ret != RK_SUCCESS)
        {
            print_level(SV_ERROR, "RK_MPI_VDEC_CreateChn[%d] failed. [err=%d]\n",i, s32Ret);
        }

            uint32_t u32Width,u32Height = 0;

             u32Width   = UVC_CAM_WIDTH_2;
             u32Height  = UVC_CAM_HEIGHT_2;    
      

            RGA_ATTR_S stRgaAttr;
            stRgaAttr.bEnBufPool = RK_TRUE;
            stRgaAttr.u16BufPoolCnt = 2;
            stRgaAttr.u16Rotaion = 0;
            stRgaAttr.stImgIn.u32X = 0;
            stRgaAttr.stImgIn.u32Y = 0;
            stRgaAttr.stImgIn.imgType = IMAGE_TYPE_NV12;
            stRgaAttr.stImgIn.u32Width = u32Width;
            stRgaAttr.stImgIn.u32Height = u32Height;
            stRgaAttr.stImgIn.u32HorStride = u32Width;
            stRgaAttr.stImgIn.u32VirStride = u32Height;
            stRgaAttr.stImgOut.u32X = 0;
            stRgaAttr.stImgOut.u32Y = 0;
            stRgaAttr.stImgOut.imgType = IMAGE_TYPE_BGR888;//IMAGE_TYPE_RGB888;
            stRgaAttr.stImgOut.u32Width =     PD_IMAGE_WIDTH;
            stRgaAttr.stImgOut.u32Height =    PD_IMAGE_HEIGHT;
            stRgaAttr.stImgOut.u32HorStride = PD_IMAGE_WIDTH;
            stRgaAttr.stImgOut.u32VirStride = PD_IMAGE_HEIGHT;
            ret = RK_MPI_RGA_CreateChn(i, &stRgaAttr);
            if (ret) {
               print_level(SV_ERROR, "RK_MPI_RGA_CreateChn[%d] failed. [err=%d]\n",i, s32Ret);
              return -1;
            }

              stSrcChn.enModId = RK_ID_VDEC;
              stSrcChn.s32DevId = 0;
              stSrcChn.s32ChnId = i;
              
              stDestChn.enModId = RK_ID_RGA;
              stDestChn.s32DevId = 0;
              stDestChn.s32ChnId = i;
              ret = RK_MPI_SYS_Bind(&stSrcChn, &stDestChn);    
    }

    return SV_SUCCESS;
}

sint32 mpp_vdec_rebuild(int chn)
{
    sint32 s32Ret = 0 ,ret = 0;
    printf("*******vdec rebuild:CH[%d]*******\n",chn);
    VDEC_CHN_ATTR_S stVdecAttr; 
    stVdecAttr.enCodecType =  RK_CODEC_TYPE_H264 ;
    stVdecAttr.enMode = VIDEO_MODE_FRAME; //VIDEO_MODE_STREAM    VIDEO_MODE_FRAME   JPEG流才用FRAME
    stVdecAttr.enDecodecMode = VIDEO_DECODEC_HADRWARE ;  //VIDEO_DECODEC_HADRWARE    VIDEO_DECODEC_SOFTWARE
    uint32 u32Size;

    MPP_CHN_S stSrcChn;
    MPP_CHN_S stDestChn;

    int i = chn;

    stSrcChn.enModId = RK_ID_VDEC;
    stSrcChn.s32DevId = 0;
    stSrcChn.s32ChnId = i;
    
    stDestChn.enModId = RK_ID_RGA;
    stDestChn.s32DevId = 0;
    stDestChn.s32ChnId = i;

    ret = RK_MPI_SYS_UnBind(&stSrcChn, &stDestChn);
    if(ret != RK_SUCCESS)
    {
        print_level(SV_ERROR, "RK_MPI_SYS_UnBind[%d] failed. [err=%d]\n",i, s32Ret);
    }

    // s32Ret = RK_MPI_RGA_DestroyChn(i);
    // if(s32Ret != RK_SUCCESS)
    // {
    //     print_level(SV_ERROR, "RK_MPI_RGA_DestroyChn[%d] failed. [err=%d]\n",i, s32Ret);
    // }

    s32Ret = RK_MPI_VDEC_DestroyChn(i);
    if(s32Ret != RK_SUCCESS)
    {
        print_level(SV_ERROR, "RK_MPI_VDEC_DestroyChn[%d] failed. [err=%d]\n",i, s32Ret);
    }

    s32Ret = RK_MPI_VDEC_CreateChn(i, &stVdecAttr);
        if(s32Ret != RK_SUCCESS)
        {
            print_level(SV_ERROR, "RK_MPI_VDEC_CreateChn[%d] failed. [err=%#x]\n",i, s32Ret);
        }
            uint32_t u32Width = UVC_CAM_WIDTH;
            uint32_t u32Height = UVC_CAM_HEIGHT;         

            // RGA_ATTR_S stRgaAttr;
            // stRgaAttr.bEnBufPool = RK_TRUE;
            // stRgaAttr.u16BufPoolCnt = 2;
            // stRgaAttr.u16Rotaion = 0;
            // stRgaAttr.stImgIn.u32X = 0;
            // stRgaAttr.stImgIn.u32Y = 0;
            // stRgaAttr.stImgIn.imgType = IMAGE_TYPE_NV12;
            // stRgaAttr.stImgIn.u32Width = u32Width;
            // stRgaAttr.stImgIn.u32Height = u32Height;
            // stRgaAttr.stImgIn.u32HorStride = u32Width;
            // stRgaAttr.stImgIn.u32VirStride = u32Height;
            // stRgaAttr.stImgOut.u32X = 0;
            // stRgaAttr.stImgOut.u32Y = 0;
            // stRgaAttr.stImgOut.imgType = IMAGE_TYPE_BGR888;//IMAGE_TYPE_RGB888;
            // stRgaAttr.stImgOut.u32Width =     PD_IMAGE_WIDTH;
            // stRgaAttr.stImgOut.u32Height =    PD_IMAGE_HEIGHT;
            // stRgaAttr.stImgOut.u32HorStride = PD_IMAGE_WIDTH;
            // stRgaAttr.stImgOut.u32VirStride = PD_IMAGE_HEIGHT;
            // ret = RK_MPI_RGA_CreateChn(i, &stRgaAttr);
            // if (ret) {
            //   printf("ERROR: Create rga[%d] falied! ret=%d\n",i, ret);
            //   return -1;
            // }

              stSrcChn.enModId = RK_ID_VDEC;
              stSrcChn.s32DevId = 0;
              stSrcChn.s32ChnId = i;
              
              stDestChn.enModId = RK_ID_RGA;
              stDestChn.s32DevId = 0;
              stDestChn.s32ChnId = i;
              ret = RK_MPI_SYS_Bind(&stSrcChn, &stDestChn);
    
    return SV_SUCCESS;
}



sint32 mpp_vpss_Vdec()
{
    sint32 s32Ret = 0 ,ret = 0;

    sint32 s32Fd = m_stVpssAlgFrame.s32AlgChnFd;

    int counter = 0;

    uint32 u32BufNum = 0;
    uint32 u32DstSize = 0; 

    static struct timespec tvNow = {0, 0};
    static struct timespec tvLast = {0, 0};
    int Getframe_CH = 0;
    int Detect_en_flag = 0;    
    int DisplayMode_flag = 0;
    int VDEC_CH = 0;

    uint32_t  u32StepTimeMs = 0;
    uint32_t  u32cnt = 0;
    uint32_t  u32sum = 0;

    uint32_t tvcnt = 0;

    uint16_t u16cnt = 0;

    mpp_vdec_init();

    uint8_t Reset_Frame_done[4] = {0};

    uint8_t u8ResetFrameBuf[4] = {0};
    u8ResetFrameBuf[0] = 0Xff;
    u8ResetFrameBuf[1] = 0Xbb;

    uint8_t Reset_cnt = 0;
    while(1)   
    {
        // clock_gettime(CLOCK_MONOTONIC, &tvNow);
        // u32StepTimeMs = ((tvNow.tv_sec*1000 + tvNow.tv_nsec/1000000) - (tvLast.tv_sec*1000 + tvLast.tv_nsec/1000000));
        // tvLast = tvNow;
       // printf("****delay ms__dec pic:%d ***** \n",u32StepTimeMs);
        // u32sum += u32StepTimeMs;
        // u32cnt ++;
        // if(u32cnt >10) //100*50ms = 5s
        // {
        //    printf("****delay ms__dec pic:%d ***** \n",u32sum / u32cnt );
        //    u32cnt =  0;
        //    u32sum =  0;
        // }

        int chn = 0;
        int32_t timeout = 500000; 
        int index_vdec = -1;

        index_vdec = ringque_VDEC.Read(chn, timeout, [/*UtilizeResource*/&ringque_VDEC](int &current){
            //cout << "    --- reader" << " --- " << GetMtimestamp() << " rpos:" << ringque.rpos_[0] << " index:" << current << " value:" << *ringbuf[current] << endl;
            //printf("    --- reader --- %llu, rpos: %d, index: %d, value: %#x\n", GetMtimestamp(), ringque.rpos_[0], current, (uint32_t)(*ringbuf[current]));
            // usleep(10000); // 10ms
             //sleep_ms(10);
            return current; //0;
        });

        //printf("&& vdec read[%d] :%d \n",index_vdec, *(MptrData[index_vdec] +7));

#if 1   //从解码到送进队列 ： 10ms

       // clock_gettime(CLOCK_MONOTONIC, &tvNow);
       // u32StepTimeMs = ((tvNow.tv_sec*1000 + tvNow.tv_nsec/1000000) - (tvLast.tv_sec*1000 + tvLast.tv_nsec/1000000));
       // tvLast = tvNow;

    Getframe_CH = *(MptrData[index_vdec] + 32); 
    Detect_en_flag = *(MptrData[index_vdec] + 38);
    DisplayMode_flag = *(MptrData[index_vdec] + 8);

    if(! (*(MptrData[index_vdec] +34)&0x80) )//== RESOL_720P
        VDEC_CH = Getframe_CH + 4;
    else
        VDEC_CH = Getframe_CH ; 

    // #if 1
    // if(Reset_Frame_done[Getframe_CH] == 0)
    // {
    //     if( *(MptrData[index_vdec] +6) == 0)
    //         {   
    //         Reset_Frame_done[Getframe_CH] = 1;
    //         printf("Reset_Frame_done \n");
    //         }
    //     else
    //         {
    //         printf("wait_IFrame \n");
    //         u8ResetFrameBuf[2] = Getframe_CH;
    //         sint32 s32WriteLen = write(m_stPdInfo.s32SerialFd[0], u8ResetFrameBuf, 4);
    //         continue;
    //         }
    // }
    // #endif

    //printf("***frame: %d***\n",*(MptrData[index_vdec] +6));

    // if( *(MptrData[index_vdec] + 7) == 30)
    // {

    //         printf("wait_IFrame \n");
    //         u8ResetFrameBuf[2] = Getframe_CH;
    //         sint32 s32WriteLen = write(m_stPdInfo.s32SerialFd[0], u8ResetFrameBuf, 4);
    //         continue;
            
    // }

    MEDIA_BUFFER m_mb = NULL;

    if(H264_RESOL == RESOL_480P)
    {
        m_mb = RK_MPI_MB_CreateBuffer(RINGBUF_VDEC_SIZE , RK_TRUE, 0);  //yuv 420
        if(!m_mb)
        {
          print_level(SV_ERROR, "RK_MPI_MB_CreateBuffer failed\n");
        }
        RK_MPI_MB_SetSize(m_mb, RINGBUF_VDEC_SIZE );
        memcpy(RK_MPI_MB_GetPtr(m_mb), MptrData[index_vdec] , RINGBUF_VDEC_SIZE );
    }
    else if(H264_RESOL == RESOL_720P)
    {
        m_mb = RK_MPI_MB_CreateBuffer(RINGBUF_VDEC_SIZE_2 , RK_TRUE, 0);  //yuv 420
        if(!m_mb)
        {
          print_level(SV_ERROR, "RK_MPI_MB_CreateBuffer failed\n");
        }
        RK_MPI_MB_SetSize(m_mb, RINGBUF_VDEC_SIZE_2);
        memcpy(RK_MPI_MB_GetPtr(m_mb), MptrData[index_vdec] , RINGBUF_VDEC_SIZE_2);    
    }
    else
    {
        m_mb = RK_MPI_MB_CreateBuffer(RINGBUF_VDEC_SIZE , RK_TRUE, 0);  //yuv 420
        if(!m_mb)
        {
          print_level(SV_ERROR, "RK_MPI_MB_CreateBuffer failed\n");
        }
        RK_MPI_MB_SetSize(m_mb, RINGBUF_VDEC_SIZE );
        memcpy(RK_MPI_MB_GetPtr(m_mb), MptrData[index_vdec] , RINGBUF_VDEC_SIZE );   
    }

 


  
        s32Ret =  RK_MPI_SYS_SendMediaBuffer(RK_ID_VDEC, VDEC_CH , m_mb);
        if(s32Ret != RK_SUCCESS)
        {
            print_level(SV_ERROR, "RK_MPI_SYS_SendMediaBuffer failed. [err=%#x]\n", s32Ret);
        }
        //usleep(10 * 1000);
        RK_MPI_MB_ReleaseBuffer(m_mb);
        //RK_MPI_SYS_StopGetMediaBuffer(RK_ID_VDEC,0);
       // mpp_vdec_rebuild(0);


        MEDIA_BUFFER RGA_m_mb = RK_MPI_SYS_GetMediaBuffer(RK_ID_RGA, VDEC_CH, 500); //解码数据直接送到RGA
            if(!RGA_m_mb )
            {
                 RK_MPI_MB_ReleaseBuffer(RGA_m_mb);
                 print_level(SV_INFO, "***err:RK_MPI_SYS_GetMediaBuffer%d failed.***\n",VDEC_CH);

                remove("root/err.264");
                FILE *fp = fopen("root/err.264", "wb+");
                if (NULL != fp)
                {   
                    if(H264_RESOL == RESOL_480P)
                        fwrite(MptrData[index_vdec], 1, RINGBUF_VDEC_SIZE, fp);
                    else if(H264_RESOL == RESOL_720P)
                        fwrite(MptrData[index_vdec], 1, RINGBUF_VDEC_SIZE_2, fp);
                }
                fclose(fp);
                // mpp_vpss_Deinitialize();
                 system("pkill -9  alg");
     
                 continue;  
            }
        //usleep(10 * 1000);
     //    printf("******RK_MPI_MB_ReleaseBuffer*******\n");
         // int index = (counter[Getframe_CH] % (RINGBUF_COUNT/4)) + (Getframe_CH * (4-1));  //每3个buf放一个通道的帧，溢出就丢
            int index = counter % RINGBUF_COUNT;

            memcpy(dstData[index],RK_MPI_MB_GetPtr(RGA_m_mb),RINGBUF_SIZE);

            RK_MPI_MB_ReleaseBuffer(RGA_m_mb);

            dstData[index][RINGBUF_SIZE-extern_num]   =  Getframe_CH;
            dstData[index][RINGBUF_SIZE-extern_num+1] =  Detect_en_flag;
            dstData[index][RINGBUF_SIZE-extern_num+2] =  DisplayMode_flag;

        //clock_gettime(CLOCK_MONOTONIC, &tvNow);
        //u32StepTimeMs = ((tvNow.tv_sec*1000 + tvNow.tv_nsec/1000000) - (tvLast.tv_sec*1000 + tvLast.tv_nsec/1000000));


        // u32sum += u32StepTimeMs;
        // u32cnt ++;
        // if(u32cnt >60) //100*50ms = 5s
        // {

        //    printf("dec delay: %d\n",u32sum / u32cnt );
        //    u32cnt =  0;
        //    u32sum =  0; 
        // }

        // if(u16cnt ++ > 100)
        // {
        // system("cat /sys/class/thermal/thermal_zone0/temp");
        // u16cnt= 0;
        // }

            if(TEST_RINGCOUNT < RINGBUF_COUNT)
               TEST_RINGCOUNT ++;


                // *** write resource
                int rel = ringque.Write(index, [](int &release){ 
                    return ++release; 
                });

                if (rel > 0) { // release dequeued resource
                   // printf("        --- relese dequeued resource: %d\n", --rel);
                }
            counter ++;
     #endif
    }

    return SV_SUCCESS;
}



sint32 mpp_vpss_Monitor()
{

    while(1)
    {
    s32time_out ++;
    if(s32time_out > 600) //60s没有进入算法，则重启
    {
         print_level(SV_ERROR, "wait alg timeout\n");
         mpp_vpss_Deinitialize();
         system("pkill -9  alg"); // 超时重启 6s
    }


    //断电上电 test
    

    sleep_ms(100);
    }
}




sint32 pd_Alarm_Out_Enable(PD_ALARM_TYPE enAlarmType)
{
    sint32 s32Ret;
    uint8 u8Value = 1;

#if 0 //USING_LED
    LED_ALARMOUT_MODE enMode;
    if (BOARD_IsCustomer(BOARD_C_ADA32V2_FTC))
    {
        enMode = LED_ALARMOUT_NO_DETECTION_PWM;
        switch (enAlarmType)
        {
            case PD_ALARM_TYPE_GREEN:
                enMode = LED_ALARMOUT_GREEN_PWM;
                break;
            case PD_ALARM_TYPE_YELLOW:
                enMode = LED_ALARMOUT_YELLOW_PWM;
                break;
            case PD_ALARM_TYPE_RED:
                enMode = LED_ALARMOUT_RED_PWM;
                break;
            case PD_ALARM_TYPE_NULL:
                enMode = LED_ALARMOUT_NO_DETECTION_PWM;
                break;
        }
        return LED_ALARMOUT_setMode(enMode);
    }
#endif

#if (defined(BOARD_ADA32V2) || defined(BOARD_ADA32V3) || defined(BOARD_ADA32IR) || defined(BOARD_ADA32N1) || defined(BOARD_ADA47V1) || defined(BOARD_ADA32C4) || defined(BOARD_ADA32C4))
    s32Ret = BOARD_SetAlarmOut(m_stPdInfo.stCfgParam.enAlgTrigger ==  TRIGGER_UP ? 1 : 0);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "BOARD_SetAlarmOut fail![err=%#x]\n", s32Ret);
        return s32Ret;
    }
#endif

    return SV_SUCCESS;
}

sint32 pd_Alarm_Out_Reset()
{
    sint32 s32Ret;
    uint8 u8Value = 0;
#if 0 //USING_LED
#if (BOARD_ADA32V2)
    if (BOARD_IsCustomer(BOARD_C_ADA32V2_FTC))
    {
        return LED_ALARMOUT_setMode(LED_ALARMOUT_NO_DETECTION_PWM);
    }
#endif
#endif

#if (defined(BOARD_ADA32V2) || defined(BOARD_ADA32V3) || defined(BOARD_ADA32IR) || defined(BOARD_ADA32N1) || defined(BOARD_ADA47V1) || defined(BOARD_ADA32C4))
    s32Ret = BOARD_SetAlarmOut(m_stPdInfo.stCfgParam.enAlgTrigger ==  TRIGGER_UP ? 0 : 1);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "BOARD_SetAlarmOut fail![err=%#x]\n", s32Ret);
        return s32Ret;
    }
#endif

    return SV_SUCCESS;
}

/* AlarmOut 初始化 */
sint32 pd_Alarm_Out_Init()
{
    sint32 s32Ret;

#if 1
    /* FTC客户需要上电2s探测性触发输出 */
    if (BOARD_IsCustomer(BOARD_C_ADA32V2_FTC))
    {
        pd_Alarm_Out_Enable(PD_ALARM_TYPE_RED);
        sleep(2);
    }
#endif

    s32Ret = pd_Alarm_Out_Reset();
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "pd_Alarm_Out_Reset fail![err=%#x]\n", s32Ret);
        return s32Ret;
    }

    return SV_SUCCESS;
}

/* 配置看门狗 */
static inline sint32 pd_LED_watchdog_Setting(LED_RGB_STATE enState)
{
    sint32 s32Ret = SV_SUCCESS;
#if USING_LED
    s32Ret |= LED_RGB_setStateAndWatchDog(enState);
    s32Ret |= LED_ALARMOUT_setWatchDog();
#endif
    return s32Ret;
}

/* 配置触发方向 */
static inline sint32 pd_LED_AlarmOUt_Direction(TRIGGER_TYPE_S enDirection)
{
    sint32 s32Ret = SV_SUCCESS;
#if USING_LED
    s32Ret = LED_ALARMOUT_setDirection((LED_DIRECTION_TYPE)enDirection);
#endif
    return s32Ret;
}

/* 配置LED灯的颜色 */
static inline sint32 pd_LED_RGB_color_setting()
{
    sint32 s32Ret = SV_SUCCESS;
#if USING_LED
    s32Ret = LED_RGB_setColor(LED_RGB_YELLOW);
#endif
    return s32Ret;
}


/* 播放报警音频 */
void pd_Audio_Play(void *pvArg)
{
    PD_ALARM_NOTICE_S stAlarmNotice = *((PD_ALARM_NOTICE_S*)pvArg);
    sint32 s32Ret;
    static int mode = -1;
    static int bPlay = 0;
    ALARM_EVENT_S stAlarmEvent = {0};
    MSG_PACKET_S stMsgPkt = {0};
    struct timeval tvAlarm;
    struct timezone tz;
#ifdef  REMOVE_ALARM_OUT
    s32Ret = pthread_detach(pthread_self());
    if(s32Ret!=0)
    {
        pthread_exit(NULL); // 显式退出
    }

#endif  //REMOVE_ALARM_OUT

    //print_level(SV_DEBUG, "bPlay:%d, mode:%d, new_mode:%d\n", bPlay, mode, new_mode);

    if(stAlarmNotice.enMode == PD_ALARM_TYPE_NULL)
    {
        return;
    }
    
    if(bPlay && mode >= stAlarmNotice.enMode)
    {
        //print_level(SV_WARN, "still playing alarm%d\n", mode);
        return;
    }

    bPlay = 1;

    memset(&stAlarmEvent, 0, sizeof(stAlarmEvent));
    gettimeofday(&tvAlarm, &tz);
    stAlarmEvent.enAlarmEvent = ALARM_EVENT_PD;
    stAlarmEvent.enAlarmType = ALARM_PD_ROI1;
    if(stAlarmNotice.enMode == 1) stAlarmEvent.enAlarmType = ALARM_PD_ROI1;
    if(stAlarmNotice.enMode == 2) stAlarmEvent.enAlarmType = ALARM_PD_ROI2;
    if(stAlarmNotice.enMode == 3) stAlarmEvent.enAlarmType = ALARM_PD_ROI3;
    stAlarmEvent.s32TimeStamp = (sint32)tvAlarm.tv_sec;
    stAlarmEvent.s32Chn = stAlarmNotice.s32Chn;

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

#if (defined(BOARD_ADA32V2) || defined(BOARD_ADA32C4))
    memset(&stMsgPkt, 0, sizeof(stMsgPkt));
    stMsgPkt.stMsg.u16OpCode = OP_EVENT_ALG_ALARM;
    stMsgPkt.stMsg.u8Param = stAlarmNotice.enMode;
    stMsgPkt.u32Size = sizeof(stAlarmEvent);
    s32Ret = Msg_submitEvent(EP_UVCSERVER, OP_EVENT_ALG_ALARM, &stMsgPkt);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
    }
#endif

    mode = stAlarmNotice.enMode;
    //print_level(SV_DEBUG, "mode:%d\n", mode);
#ifndef REMOVE_ALARM_OUT
    if (BOARD_IsCustomer(BOARD_C_ADA32V2_LUIS) && stAlarmNotice.bCar == SV_TRUE)
    {
        ALARM_PlayAlarm(stAlarmNotice.s32Chn, mode, SV_FALSE);  // 播放车辆的报警音
    }
    else
    {
        ALARM_PlayAlarm(stAlarmNotice.s32Chn, mode, SV_TRUE);   // 播放行人的报警音
    }
#endif  //REMOVE_ALARM_OUT


    //sleep_ms(2000);     // 等待音频播放完成

    if(mode == stAlarmNotice.enMode)
    {
        bPlay = 0;
    }
    
    return;
}



#if 1
uint32_t serial_init(const sint8* port, uint32_t baud,uint8_t CH)
{
    uint32_t s32FdUpdate;
    struct termios stAttr;
    uint32_t s32Ret = -1;
    s32FdUpdate = open(port, O_RDWR|O_NOCTTY|O_NDELAY);
    if (s32FdUpdate < 0)
    {        
        print_level(SV_ERROR, "serial_init: %s Open failed\n", port);
        close(s32FdUpdate);
        return SV_FALSE;
    }
    print_level(SV_INFO, "serial_init: %s Open successful\n", port);
    tcgetattr(s32FdUpdate, &stAttr);
    bzero(&stAttr, sizeof(stAttr));

    switch (baud) {
        case 9600:
            cfsetispeed(&stAttr, B9600);
            cfsetospeed(&stAttr, B9600);
            break;
        case 19200:
            cfsetispeed(&stAttr, B19200);
            cfsetospeed(&stAttr, B19200);
            break;
        case 38400:
            cfsetispeed(&stAttr, B38400);
            cfsetospeed(&stAttr, B38400);
            break;
        case 115200:
            cfsetispeed(&stAttr, B115200);
            cfsetospeed(&stAttr, B115200);
            break;
        default:
            fprintf(stderr, "Warning: Baudrate not supported!\n");
            close(s32FdUpdate);
            return -1;
    }

    stAttr.c_cflag |= CLOCAL | CREAD;
    stAttr.c_cflag &= ~CSIZE;
    stAttr.c_cflag |= CS8;
    stAttr.c_cflag &= ~PARENB;
    stAttr.c_cflag &= ~CSTOPB;
    stAttr.c_cflag &= ~CRTSCTS;
    stAttr.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    stAttr.c_oflag &= ~OPOST;
    s32Ret = tcflush(s32FdUpdate, TCIFLUSH);
    if (s32Ret < 0)
    {
        print_level(SV_ERROR, "tcflush failed. [err=%#x]\n", errno);
    }
    stAttr.c_cc[VTIME] = 255;
    stAttr.c_cc[VMIN] = 0;
    s32Ret = tcsetattr(s32FdUpdate, TCSANOW, &stAttr);
    if (s32Ret < 0)
    {
        print_level(SV_ERROR, "tcsetattr failed. [err=%#x]\n", errno);
    }
    m_stPdInfo.s32SerialFd[CH] = s32FdUpdate;
  
    /* TCIFLUSH: flushes data received but not read.
     * TCOFLUSH: flushes data written but not transmitted
     * TCIOFLUSH: flushes both data received but not read, and data written but not transmitted
     */
    tcflush(m_stPdInfo.s32SerialFd[CH], TCIOFLUSH);  /* 清空串口1的读写缓冲 */

    printf("serial init success. FD=%d\n", m_stPdInfo.s32SerialFd[CH]);
    return SV_TRUE;
}

int uart_recieve()
{
    sint32 s32Ret = 0;
    fd_set rfds;
    int rxlen = 0;
    char buf[255] = {0};
    char ch = 0;
    while(1)
    {
    FD_ZERO(&rfds);
    FD_SET(m_stPdInfo.s32SerialFd[0], &rfds);
    s32Ret = select(m_stPdInfo.s32SerialFd[0] +1,&rfds, NULL, NULL, NULL); //block mode

    if(s32Ret > 0)
       {
        if(FD_ISSET(m_stPdInfo.s32SerialFd[0],&rfds))
            {
               rxlen = read(m_stPdInfo.s32SerialFd[0] , buf, 255);
               if(rxlen > 0)
               {
                 for(int i = 0; i < rxlen ; i++)
                 {   
                    sprintf(&ch,"%x",buf[i]);
                    write(m_stPdInfo.s32SerialFd[1], &ch, 1);  //控制433
                 }
               }
           }
       }
    }
    return SV_TRUE; 
}

#endif
/* 触发线使能 */
void pd_Alarm_Out(void *pvArg)
{
    sint32 s32Ret, pwm, i = 0;
    static int bTrigger = 0;
    static struct timespec tvStart = {0, 0};
    static struct timespec tvNow = {0, 0};
    static PD_ALARM_TYPE lmode;   /* 实时保存相应的触发模式 */
    static SV_BOOL st_bStop;
    PD_ALARM_NOTICE_S stAlarmNotice = *((PD_ALARM_NOTICE_S*)pvArg);
    sint32 s32Interval = 2000;  /* 默认休眠时间为2秒 */
    CFG_PDS_PARAM *pstPdsParam = NULL;
    
   #ifdef REMOVE_ALARM_OUT
       s32Ret = pthread_detach(pthread_self());
    if(s32Ret!=0)
    {
        pthread_exit(NULL); // 显式退出
    }
   #endif
    if (0 == stAlarmNotice.s32Chn)    
        pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
    else
        pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh1.stPdsParam;

    if(pstPdsParam->s32PdAlarmOutInterval == 0)    //持续时间为0ms直接退出
    {
        goto skip_alg_twice;
    }

    if(pstPdsParam->s32PdAlarmOutInterval > 0)
    {
        s32Interval = pstPdsParam->s32PdAlarmOutInterval;
    }

    if(stAlarmNotice.enMode == PD_ALARM_TYPE_GREEN && pstPdsParam->bPdAlarmOutGreen == SV_FALSE)
    {
        goto skip_alg_twice;
    }
    else if(stAlarmNotice.enMode == PD_ALARM_TYPE_YELLOW && pstPdsParam->bPdAlarmOutYellow == SV_FALSE)
    {
        goto skip_alg_twice;
    }
    else if(stAlarmNotice.enMode == PD_ALARM_TYPE_RED && pstPdsParam->bPdAlarmOutRed == SV_FALSE)
    {
        goto skip_alg_twice;
    }
    else if(stAlarmNotice.enMode == PD_ALARM_TYPE_NULL)
    {
        goto skip_alg_twice;
    }

    st_bStop = SV_FALSE;
    lmode = stAlarmNotice.enMode;   /* 更新报警模式 */
    if (lmode != stAlarmNotice.enMode)
    {
        lmode = stAlarmNotice.enMode;
        if (BOARD_IsCustomer(BOARD_C_ADA32V2_FTC))
        {
            pd_Alarm_Out_Enable(lmode);
        }
    }
    
    if(bTrigger)
    {
        clock_gettime(CLOCK_MONOTONIC, &tvStart);   //重置时间
        goto skip_alg;
    }

    bTrigger = 1;
    clock_gettime(CLOCK_MONOTONIC, &tvStart);
    tvNow = tvStart;

    pd_Alarm_Out_Enable(lmode);
#if 0
    while(((1000*tvNow.tv_sec + tvNow.tv_nsec/1000000) - (1000*tvStart.tv_sec + tvStart.tv_nsec/1000000) \
          < s32Interval) || 
          (st_bStop != SV_TRUE)
          )
    {
        sleep_ms(10);
        clock_gettime(CLOCK_MONOTONIC, &tvNow);
    }
#else
    while(((1000*tvNow.tv_sec + tvNow.tv_nsec/1000000) - (1000*tvStart.tv_sec + tvStart.tv_nsec/1000000) \
          < s32Interval) || 
          (st_bStop != SV_TRUE)
          )
    {
        i = (i + 1) % 10;
        /* 设置PWM占用比      绿色占空比为30% 黄色占空比为60% 红色占空比为100%*/
        if(BOARD_IsCustomer(BOARD_C_ADA32V2_FTC))
        {
            switch(lmode)
            {
                case PD_ALARM_TYPE_GREEN:
                    pwm = 3;
                    break;
                case PD_ALARM_TYPE_YELLOW:
                    pwm = 6;
                    break;
                case PD_ALARM_TYPE_RED:
                default:
                    pwm = 10;
                    break;
            }
    
            if(i < pwm)
            {
                print_level(SV_INFO, "i:%d enable!\n", i);
                pd_Alarm_Out_Enable(lmode);
            }
            else
            {
                print_level(SV_INFO, "i:%d disable!\n", i);
                pd_Alarm_Out_Reset();
            }
        }
        sleep_ms(10);
        clock_gettime(CLOCK_MONOTONIC, &tvNow);
    }
#endif
    pd_Alarm_Out_Reset();
    bTrigger = 0;

skip_alg:
    return;

skip_alg_twice:
    st_bStop = SV_TRUE;
    return;
}

/* 播放声光报警音频 */
void pd_avAlarmer_Play(void *pvArg)
{
    PD_ALARM_NOTICE_S stAlarmNotice = *((PD_ALARM_NOTICE_S*)pvArg);
    sint32 s32Ret;
    static int mode = -1;
    static int bPlay = 0;
    static struct timespec tvStart = {0, 0};
    static struct timespec tvNow = {0, 0};
    CFG_PDS_PARAM *pstPdsParam = NULL;
    sint32 s32Interval = 2000;  /* 默认持续时间为2秒 */
    
    if (0 == stAlarmNotice.s32Chn)    
        pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
    else
        pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh1.stPdsParam;

    avALARMER_MSG_S stavAlarmerMsg = {0};
    MSG_PACKET_S stMsgPkt = {0};

    if(stAlarmNotice.enMode == PD_ALARM_TYPE_NULL)
    {
        return;
    }

    if(pstPdsParam->s32PdAlarmOutInterval == 0)    //持续时间为0ms直接退出
    {
        return;
    }

    if(pstPdsParam->s32PdAlarmOutInterval > 0)
    {
        s32Interval = pstPdsParam->s32PdAlarmOutInterval;
    }

    if(bPlay && mode >= stAlarmNotice.enMode)
    {
        if(mode == stAlarmNotice.enMode)
        {
            clock_gettime(CLOCK_MONOTONIC, &tvStart);   //重置时间
        }
        return;
    }

    bPlay = 1;
    mode = stAlarmNotice.enMode;
    clock_gettime(CLOCK_MONOTONIC, &tvStart);

    stavAlarmerMsg.s32Play = avAlarm_Play_Start;
    if(mode == 1)
        stavAlarmerMsg.s32Freq = avAlarm_Freq_1HZ;
    else if(mode == 2)
        stavAlarmerMsg.s32Freq = avAlarm_Freq_3HZ;
    else if(mode == 3)
        stavAlarmerMsg.s32Freq = avAlarm_Freq_5HZ;
    
    stavAlarmerMsg.s32Volume = m_stPdInfo.stCfgParam.s32PdsAlarmerVolume;
    stavAlarmerMsg.s32Times = avAlarm_Play_Always;
    stavAlarmerMsg.s32Seq = (m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam.enAudioType - 1) * 3 + (mode - 1) + 1;
    stMsgPkt.pu8Data = (uint8*)&stavAlarmerMsg;
    stMsgPkt.u32Size = sizeof(stavAlarmerMsg);
    s32Ret = Msg_submitEvent(EP_AVALARMER, OP_EVENT_AVALARMER_SEND, &stMsgPkt);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
    }

    // 等待声光报警器播放完成
    while((1000*tvNow.tv_sec + tvNow.tv_nsec/1000000) - (1000*tvStart.tv_sec + tvStart.tv_nsec/1000000) \
          < s32Interval)
    {
        if(mode < stAlarmNotice.enMode)
        {
            return;
        }

        sleep_ms(10);
        clock_gettime(CLOCK_MONOTONIC, &tvNow);
    }

    if(mode >= stAlarmNotice.enMode)
    {
        memset(&stavAlarmerMsg, 0x00, sizeof(stavAlarmerMsg));
        stMsgPkt.pu8Data = (uint8*)&stavAlarmerMsg;
        stMsgPkt.u32Size = sizeof(stavAlarmerMsg);
        s32Ret = Msg_submitEvent(EP_AVALARMER, OP_EVENT_AVALARMER_SEND, &stMsgPkt);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
        }
        bPlay = 0;
    }

    return;
}

sint32 pd_Alarm_Post(PD_ALARM_NOTICE_S stAlarmNotice)
{
    sint32 s32Ret;
    pthread_t thread_sound, thread_trigger, thread_alarmer;
    static PD_ALARM_NOTICE_S stAlarmNoticeLocal = {0};
#if (defined(BOARD_ADA32SDK))
    if (!m_stPdInfo.bKeyAuth)
    {
        return SV_SUCCESS;
    }
#endif

#if (defined(BOARD_ADA47V1))
    return SV_SUCCESS;
#endif
    stAlarmNoticeLocal.bPerson  = stAlarmNotice.bPerson;
    stAlarmNoticeLocal.bCar     = stAlarmNotice.bCar;
    stAlarmNoticeLocal.enMode   = stAlarmNotice.enMode;
    stAlarmNoticeLocal.enSplit  = stAlarmNotice.enSplit;
    stAlarmNoticeLocal.s32Chn   = stAlarmNotice.s32Chn;


    thpool_add_work(m_stPdInfo.thpool, pd_Audio_Play, &stAlarmNoticeLocal);
    thpool_add_work(m_stPdInfo.thpool, pd_Alarm_Out, &stAlarmNoticeLocal);
    thpool_add_work(m_stPdInfo.thpool, pd_avAlarmer_Play, &stAlarmNoticeLocal);

    return SV_SUCCESS;
}

/* 播放遮挡检测音频 */
void pd_Audio_Shelter_Play(void *pvArg)
{
    PD_ALARM_NOTICE_S stAlarmNotice = *((PD_ALARM_NOTICE_S*)pvArg);
    sint32 s32Ret;
    static int mode = -1;
    static int bPlay = 0;

    struct timeval tvAlarm;
    struct timezone tz;

    //print_level(SV_DEBUG, "bPlay:%d, mode:%d, new_mode:%d\n", bPlay, mode, new_mode);
    if(stAlarmNotice.enMode == PD_ALARM_TYPE_NULL)
    {
        return;
    }
    
    if(bPlay && mode >= stAlarmNotice.enMode)
    {
        return;
    }

    bPlay = 1;
    mode = stAlarmNotice.enMode;
    //print_level(SV_DEBUG, "mode:%d\n", mode);

    ALARM_PlayAudio(ALARM_SHELTER);
    //sleep_ms(2000);     // 等待音频播放完成

    if(mode == stAlarmNotice.enMode)
    {
        bPlay = 0;
    }
    
    return;
}


sint32 pd_Shelter_Post(PD_ALARM_NOTICE_S stAlarmNotice)
{
    sint32 s32Ret;
    pthread_t thread_sound, thread_trigger, thread_alarmer;
    static PD_ALARM_NOTICE_S stAlarmNoticeLocal = {0};
#if (defined(BOARD_ADA32SDK))
    if (!m_stPdInfo.bKeyAuth)
    {
        return SV_SUCCESS;
    }
#endif

    stAlarmNoticeLocal.bPerson = stAlarmNotice.bPerson;
    stAlarmNoticeLocal.enMode = stAlarmNotice.enMode;
    stAlarmNoticeLocal.enSplit = stAlarmNotice.enSplit;
    stAlarmNoticeLocal.s32Chn = stAlarmNotice.s32Chn;

    thpool_add_work(m_stPdInfo.thpool, pd_Audio_Shelter_Play, &stAlarmNoticeLocal);
    thpool_add_work(m_stPdInfo.thpool, pd_Alarm_Out, &stAlarmNoticeLocal);

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




sint32 pd_DumpInfo(sint32 s32Chn, PD_DUMP_INFO_S *pstPdDumpInfo)
{
    sint32 s32Ret = 0, i;
    uint32 u32ChnNum = 1;
    sint32 fd = -1;
    cJSON *pstJson = NULL, *pstList = NULL, *pstItem = NULL;
    cJSON *pstTimeStamp = NULL, *pstPdWorkMode = NULL, *pstGreenRoiNum = NULL, *pstYellowRoiNum = NULL, *pstRedRoiNum = NULL;
    char szTimeStamp[1024];
    char szBuf[1024];
    char echo_szBuf[2048];
    CFG_PDS_PARAM *apstPdsParam[ALG_MAX_CHN] = {0};
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

#if (defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1))
    u32ChnNum = 3;
    apstPdsParam[0] = &m_stPdInfo.stCfgParam.stAlgCh1.stPdsParam;
    apstPdsParam[1] = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
    apstPdsParam[2] = &m_stPdInfo.stCfgParam.stAlgCh3.stPdsParam;
#elif (defined(BOARD_ADA32IR))
    u32ChnNum = 2;
    apstPdsParam[0] = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
    apstPdsParam[1] = &m_stPdInfo.stCfgParam.stAlgCh1.stPdsParam;
#else
    u32ChnNum = 1;
    apstPdsParam[0] = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
#endif
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
        sprintf(szTimeStamp, "%lld", (astDumpInfo[i].s64TimeStamp));
        pstTimeStamp = cJSON_CreateRaw(szTimeStamp);
        if(NULL == pstTimeStamp)
        {
            print_level(SV_ERROR, "cJSON_CreateNumber failed.\n");
            goto exit;
        }
        cJSON_AddItemToObject(pstItem, "TimeStamp", pstTimeStamp);

        pstPdWorkMode = cJSON_CreateNumber(apstPdsParam[i]->enPDWorkMode);
        if(NULL == pstPdWorkMode)
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

        if(apstPdsParam[i]->enPdsModel == E_PDS_SH)
        {
            cJSON_AddItemToObject(pstItem, "RedHelmetNum", cJSON_CreateNumber(astDumpInfo[i].s32RedHelmetNum));
            cJSON_AddItemToObject(pstItem, "YellowHelmetNum", cJSON_CreateNumber(astDumpInfo[i].s32YellowHelmetNum));
            cJSON_AddItemToObject(pstItem, "WhiteHelmetNum", cJSON_CreateNumber(astDumpInfo[i].s32WhiteHelmetNum));
            cJSON_AddItemToObject(pstItem, "BlueHelmetNum", cJSON_CreateNumber(astDumpInfo[i].s32BlueHelmetNum));
            cJSON_AddItemToObject(pstItem, "NoHelmetNum", cJSON_CreateNumber(astDumpInfo[i].s32NoHelmetNum));
        }

        //print_level(SV_INFO, "time: %lld, red: %d yellow: %d green: %d\n", astDumpInfo[i].s64TimeStamp, astDumpInfo[i].s32RedRoiNum, astDumpInfo[i].s32YellowRoiNum, astDumpInfo[i].s32GreenRoiNum);
    }

#if 0
    if(pstPdDumpInfo->s32DistanceNum > 0)
    {
        cJSON_AddItemToObject(pstJson, "DistanceXY", cJSON_CreateIntArray((int*)pstPdDumpInfo->s32DistanceXY, pstPdDumpInfo->s32DistanceNum*2));
    }
#endif

    memset(szBuf, 0, 1024);
    cJSON_PrintPreallocated(pstJson, szBuf, 1024, 0);
    
    string_to_file("/var/info/pd-tmp", szBuf);
    rename("/var/info/pd-tmp", "/var/info/pd");

exit:
    cJSON_Delete(pstJson);
    return SV_SUCCESS;
}

/******************************************************************************
 * 函数功能: 200889客户DUMP信息矫正，MCU屏蔽行人绿色区域报警，车辆报警使用绿色区域
 * 输入参数: pstPdDumpInfo - dump 信息
             SV_BOOL --- bCar
 * 输出参数: 无
 * 返回值  : 无
 *****************************************************************************/
sint32 pd_DumpInfo_200889_Correct(PD_DUMP_INFO_S * pstPdDumpInfo, pdsa32::EAlgObjectClass oclass, PD_ROI_E enRoi)
{
    if (BOARD_IsNotCustomer(BOARD_C_ADA32V2_200889))
        return SV_SUCCESS;

    if (oclass == pdsa32::E_CLS_PERSON || oclass == pdsa32::E_CLS_PERSON_HAT)
    {
        if (enRoi == PD_ROI_GREEN)
        {
            pstPdDumpInfo->s32GreenRoiNum -= 1;
        }
    }
    else if (oclass == pdsa32::E_CLS_CAR)
    {
        switch(enRoi)
        {
            case PD_ROI_GREEN:
                break;
            case PD_ROI_YELLOW:
                pstPdDumpInfo->s32YellowRoiNum -= 1;
                pstPdDumpInfo->s32GreenRoiNum += 1;
                break;
            case PD_ROI_RED:
                pstPdDumpInfo->s32RedRoiNum -= 1;
                pstPdDumpInfo->s32GreenRoiNum += 1;
                break;
        }
    }
    
    return SV_SUCCESS;
}

/******************************************************************************
 * 函数功能: 200055客户A需求DUMP信息矫正，检测区域有一个人,使用绿色触发;检测区域有两个人,使用黄色触发;检测区域大于两个人,使用红色触发
 * 输入参数: pstPdDumpInfo - dump 信息
             SV_BOOL --- bCar
 * 输出参数: 无
 * 返回值  : 无
 *****************************************************************************/
sint32 pd_DumpInfo_200055A_Correct(PD_DUMP_INFO_S * pstPdDumpInfo, int rectNum)
{
    if (BOARD_IsNotCustomer(BOARD_C_ADA32V2_VT_A))
        return SV_SUCCESS;

    switch(rectNum)
    {
        case 0:
            pstPdDumpInfo->s32GreenRoiNum = 0;
            pstPdDumpInfo->s32YellowRoiNum = 0;
            pstPdDumpInfo->s32RedRoiNum = 0;
            break;
        case 1:
            pstPdDumpInfo->s32GreenRoiNum = 1;
            pstPdDumpInfo->s32YellowRoiNum = 0;
            pstPdDumpInfo->s32RedRoiNum = 0;
            break;
        case 2:
            pstPdDumpInfo->s32GreenRoiNum = 0;
            pstPdDumpInfo->s32YellowRoiNum = 1;
            pstPdDumpInfo->s32RedRoiNum = 0;
            break;
        default:
            pstPdDumpInfo->s32GreenRoiNum = 0;
            pstPdDumpInfo->s32YellowRoiNum = 0;
            pstPdDumpInfo->s32RedRoiNum = 1;
            break;
    }
    
    return SV_SUCCESS;
}


/******************************************************************************
 * 函数功能: 读取JPEG图片
 * 输入参数: pstPdGuiImg --- 位图结构体
             filename --- 图片名
 * 输出参数: 无
 * 返回值  : 无
 *****************************************************************************/
sint32 pd_Gui_LoadJpeg(PD_GUI_IMG_S *pstPdGuiImg, char *filename)
{
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
    FILE* pFile;
    uint32 i, j;
    uint32 w_src, h_src, byte, stride;
    char *pbmp, *pbmpTmp, *pjpegbmp, *pstart;

    if(pstPdGuiImg == NULL)
    {
        return ERR_NULL_PTR;
    }
    
    if ((pFile = fopen(filename, "rb")) == NULL)
    {
        printf("open %s failed\n", filename);
        return SV_FAILURE;
    }

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);
    jpeg_stdio_src(&cinfo, pFile);
    jpeg_read_header(&cinfo, TRUE);
    w_src = cinfo.image_width;
    h_src = cinfo.image_height;
    byte = 3;
    stride = w_src * byte;

    jpeg_start_decompress(&cinfo);
    pjpegbmp = malloc(cinfo.output_width * cinfo.output_components);
    pbmp = malloc(w_src * h_src * byte);
    pbmpTmp = pbmp;
    for(i = 0; i < h_src; i++)
    {
        pstart = pjpegbmp;
        jpeg_read_scanlines(&cinfo, &pstart, 1);
        for(j = 0; j < w_src; j++)
        {
            *(pbmpTmp + 2) = *(pstart++);  //r
            *(pbmpTmp + 1) = *(pstart++);  //g
            *(pbmpTmp + 0) = *(pstart++);  //b
            pbmpTmp+=3;
        }
    }
    
    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    fclose(pFile);
    free(pjpegbmp);

    pstPdGuiImg->pbmp = pbmp;
    pstPdGuiImg->s32Width = w_src;
    pstPdGuiImg->s32Height = h_src;
    
    return SV_SUCCESS;
}

/******************************************************************************
 * 函数功能: 缩放图片
 * 输入参数: pstPdImgDst --- 目标位图数据
             pstPdImgSrc --- 源位图数据
 * 输出参数: 无
 * 返回值  : 无
 *****************************************************************************/
sint32 pd_Gui_Zoom(PD_GUI_IMG_S *pstPdImgDst, PD_GUI_IMG_S *pstPdImgSrc)
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


sint32 pd_QRCodeCaliberation(sint32 s32Chn, SV_POINT2_S *pQRCode1, SV_POINT2_S *pQRCode2)
{
    sint32 s32Ret = SV_SUCCESS;
    
#if !defined(BOARD_ADA32V3)
    STImgPoint stImgMasks[4] = {0};
    STCaliParams stCaliParams = {0};
    sint32 s32Width = 1920, s32Height = 1080;
    PD_GUI_IMG_S stPdImgSrc, stPdImgDst;
	SV_POINT2_S stPointTmp;
	CFG_PDS_PARAM *pstPdsParam = NULL;
	char szFilePath[256] = {0};
    FILE *fp = NULL;

    if (s32Chn >= ALG_MAX_CHN)
    {
        return ERR_ILLEGAL_PARAM;
    }

    if (NULL == pQRCode1 || NULL == pQRCode2)
    {
        return ERR_NULL_PTR;
    }
    
    print_level(SV_INFO, "QRCode1:(%lf, %lf),(%lf, %lf)\n", pQRCode1[0].dX, pQRCode1[0].dY, pQRCode1[1].dX, pQRCode1[1].dY);
    print_level(SV_INFO, "QRCode2:(%lf, %lf),(%lf, %lf)\n", pQRCode2[0].dX, pQRCode2[0].dY, pQRCode2[1].dX, pQRCode2[1].dY);

    if((pQRCode1[1].dX - pQRCode1[0].dX)*(pQRCode1[1].dY - pQRCode1[0].dY) < 0.01*0.01)
        return SV_FAILURE;
    
    if((pQRCode2[1].dX - pQRCode2[0].dX)*(pQRCode2[1].dY - pQRCode2[0].dY) < 0.01*0.01)
        return SV_FAILURE;
	
	if((pQRCode1[1].dX - pQRCode1[0].dX) < 0)
	{
		stPointTmp = pQRCode1[1];
		pQRCode1[1] = pQRCode1[0];
		pQRCode1[0] = stPointTmp;
	}

	if((pQRCode2[1].dX - pQRCode2[0].dX) < 0)
	{
		stPointTmp = pQRCode2[1];
		pQRCode2[1] = pQRCode2[0];
		pQRCode2[0] = stPointTmp;
	}
	
    sprintf(szFilePath, "/var/snap/snap%d.jpeg", s32Chn);
    s32Ret = pd_Gui_LoadJpeg(&stPdImgSrc, szFilePath);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "pd_LoadJpeg fail!\n");
        return s32Ret;
    }


    stPdImgDst.s32Width = s32Width;
    stPdImgDst.s32Height = s32Height;
    stPdImgDst.pbmp = malloc(stPdImgDst.s32Width * stPdImgDst.s32Height * 3);


    s32Ret = pd_Gui_Zoom(&stPdImgDst, &stPdImgSrc);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "pd_Gui_Zoom fail!\n");
        free(stPdImgDst.pbmp);
        free(stPdImgSrc.pbmp);
        return s32Ret;
    }

    stImgMasks[0].fX = int(pQRCode1[0].dX * s32Width);
    stImgMasks[0].fY = int(pQRCode1[0].dY * s32Height);
    stImgMasks[1].fX = int(pQRCode1[1].dX * s32Width);
    stImgMasks[1].fY = int(pQRCode1[1].dY * s32Height);
    
    stImgMasks[2].fX = int(pQRCode2[0].dX * s32Width);
    stImgMasks[2].fY = int(pQRCode2[0].dY * s32Height);
    stImgMasks[3].fX = int(pQRCode2[1].dX * s32Width);
    stImgMasks[3].fY = int(pQRCode2[1].dY * s32Height);

    s32Ret = Cali_run((unsigned char *)stPdImgDst.pbmp, stCaliParams, stImgMasks, 0);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "Cali_run fail!\n");
        free(stPdImgDst.pbmp);
        free(stPdImgSrc.pbmp);
        return s32Ret;
    }


    s32Ret = Cali_import(stCaliParams, 0);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "Cali_import fail!\n");
        free(stPdImgDst.pbmp);
        free(stPdImgSrc.pbmp);
        return s32Ret;
    }

    s32Ret = CONFIG_ReloadFile();
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "CONFIG_ReloadFile failed. [err=%#x]\n", s32Ret);
    }


    s32Ret = CONFIG_GetAlgParam(&m_stPdInfo.stCfgParam);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "CONFIG_GetAlgParam failed. [err=%#x]\n", s32Ret);
    }

#if (defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1))
    switch (s32Chn)
    {
        case 0:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh1.stPdsParam;
            break;
        case 1:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
            break;
        case 2:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh3.stPdsParam;
            break;
    }
#elif (defined(BOARD_ADA32IR))
    switch (s32Chn)
    {
        case 0:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
            break;
        case 1:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh1.stPdsParam;
            break;
        case 2:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh3.stPdsParam;
            break;
        default:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
            break;
    }
#else
    pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
#endif
    pstPdsParam->bCalibrated = SV_TRUE;
    memcpy(pstPdsParam->astCalibrationInterParams, stCaliParams.fK, sizeof(stCaliParams.fK));
    memcpy(pstPdsParam->astCalibrationRotateVector, stCaliParams.fRotation, sizeof(stCaliParams.fRotation));
    memcpy(pstPdsParam->astCalibrationTranslateVector, stCaliParams.fTranslation, sizeof(stCaliParams.fTranslation));
    memcpy(pstPdsParam->astCalibrationDistortionFactor, stCaliParams.fDistortion, sizeof(stCaliParams.fDistortion));
    memcpy(pstPdsParam->astCalibrationCamPos, stCaliParams.fOldCamPos, sizeof(stCaliParams.fOldCamPos));
    memcpy(pstPdsParam->astCalibrationEulerAngles, stCaliParams.fEulerAngles, sizeof(stCaliParams.fEulerAngles));
    memcpy(pstPdsParam->astCalibrationPrincipal, stCaliParams.fPrincipal, sizeof(stCaliParams.fPrincipal));
    s32Ret = CONFIG_SetAlgParam(&m_stPdInfo.stCfgParam);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "CONFIG_SetAlgParam failed. [err=%#x]\n", s32Ret);
    }

    
    free(stPdImgDst.pbmp);
    free(stPdImgSrc.pbmp);

#endif
    return s32Ret;
}

sint32 pd_PointRealToImage(SV_3DPOINT2_S *pstPointReal, SV_POINT2_S *pstPointImage, sint32 s32Chn)
{
    sint32 s32Ret = 0;
    
#if !defined(BOARD_ADA32V3)
    STImgPoint stImg = {0};
    STWorldPoint stWorld = {0};

    stWorld.fX = pstPointReal->dX;
    stWorld.fY = pstPointReal->dY;
    stWorld.fZ = pstPointReal->dZ;
    s32Ret = Cali_projection(stWorld, stImg, s32Chn);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "Cali_projection failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }

    pstPointImage->dX = stImg.fX / 1920;
    pstPointImage->dY = stImg.fY / 1080;
#endif
    return s32Ret;
}


sint32 pd_PointImageToReal(SV_POINT2_S *pstPointImage, SV_3DPOINT2_S *pstPointReal, sint32 s32Chn)
{
    sint32 s32Ret = 0;
    
#if !defined(BOARD_ADA32V3)
    STImgPoint stImg = {0};
    STWorldPoint stWorld = {0};

    stImg.fX = pstPointImage->dX * 1920;
    stImg.fY = pstPointImage->dY * 1080;

    s32Ret = Cali_distance(stImg, stWorld, s32Chn);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "Cali_projection failed. [err=%#x]\n", s32Ret);
        return s32Ret;
    }

    pstPointReal->dX = stWorld.fX;
    pstPointReal->dY = stWorld.fY;
    pstPointReal->dZ = stWorld.fZ;

#endif
    return s32Ret;
}

/******************************************************************************
 * 函数功能: 对行人算法识别的矩形框进行滤波防抖
 * 输入参数: pstNewResult --- 最新帧识别的结果
             u32Chn --- 通道号
             fThresholdX --- X轴方向跨度阈值系数
             fThresholdY --- Y轴方向跨度阈值系数
 * 输出参数: 无
 * 返回值  : 无
 * 注意    : 该函数是不可重入函数，只允许一个线程调用
 *****************************************************************************/
void pd_RectAntiShakeFilter(pdsa32::STAlgInfo *pstNewResult, uint32 u32Chn, float fThresholdX, float fThresholdY)
{
    sint32 i, j, k;
    pdsa32::STAlgResult stNewRect, stOldRect;
    static pdsa32::STAlgInfo astOldResult[4];
    float fThrDistWidth, fThrDistHeight, fMoveDist;

    if (NULL == pstNewResult || u32Chn >= 4)
    {
        return;
    }

    for (i = 0; i < pstNewResult->u32Nums; i++)
    {
        stNewRect.fX1 = pstNewResult->stResults[i].fX1 * m_stPdInfo.u32Width;
        stNewRect.fY1 = pstNewResult->stResults[i].fY1 * m_stPdInfo.u32Height;
        stNewRect.fX2 = pstNewResult->stResults[i].fX2 * m_stPdInfo.u32Width;
        stNewRect.fY2 = pstNewResult->stResults[i].fY2 * m_stPdInfo.u32Height;
        for (j = 0; j < astOldResult[u32Chn].u32Nums; j++)
        {
            stOldRect.fX1 = astOldResult[u32Chn].stResults[j].fX1 * m_stPdInfo.u32Width;
            stOldRect.fY1 = astOldResult[u32Chn].stResults[j].fY1 * m_stPdInfo.u32Height;
            stOldRect.fX2 = astOldResult[u32Chn].stResults[j].fX2 * m_stPdInfo.u32Width;
            stOldRect.fY2 = astOldResult[u32Chn].stResults[j].fY2 * m_stPdInfo.u32Height;
            fThrDistWidth = fThresholdX * (stNewRect.fX2 - stNewRect.fX1);
            fThrDistHeight = fThresholdY * (stNewRect.fY2 - stNewRect.fY1);
            fMoveDist = sqrtf(powf((stOldRect.fX1 - stNewRect.fX1),2) + powf((stOldRect.fY1 - stNewRect.fY1),2));
            if (fMoveDist < fThrDistWidth)
            {
                fMoveDist = sqrtf(powf((stOldRect.fX2 - stNewRect.fX2),2) + powf((stOldRect.fY2 - stNewRect.fY2),2));
                if (fMoveDist < fThrDistHeight)
                {
                    pstNewResult->stResults[i].fX1 = astOldResult[u32Chn].stResults[j].fX1;
                    pstNewResult->stResults[i].fY1 = astOldResult[u32Chn].stResults[j].fY1;
                    pstNewResult->stResults[i].fX2 = astOldResult[u32Chn].stResults[j].fX2;
                    pstNewResult->stResults[i].fY2 = astOldResult[u32Chn].stResults[j].fY2;

                    /* 排队该旧帧矩形框减少下个新矩形的匹配遍历数 */
                    for (k = j; k < astOldResult[u32Chn].u32Nums - 1; k++)
                    {
                        astOldResult[u32Chn].stResults[k] = astOldResult[u32Chn].stResults[k+1];
                    }
                    astOldResult[u32Chn].u32Nums--;
                    break;
                }
            }
        }
    }

    astOldResult[u32Chn] = *pstNewResult;
}

/******************************************************************************
 * 函数功能: 检查指定坐标所属的标定区域
 * 输入参数: pstRect --- 行人矩形
 * 输出参数: 无
 * 返回值  : 区域类型
 * 说明    : 无
 *****************************************************************************/
PD_ROI_E pd_CheckRoiPoint(pdsa32::STAlgResult *pstRect)
{
    PD_ROI_E enRoi = PD_ROI_BUTT;
    CFG_PDROI_E enRoiStyle = m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam.enRoiStyle;
    return enRoi;
}

/******************************************************************************
 * 函数功能: 判断两个矩形是否相交
 * 输入参数: lt1 --- 矩形1左上角坐标(按屏幕坐标，左上角为原点)
             rb1 --- 矩形1右下角坐标
             lt2 --- 矩形2左上角坐标
             rb2 --- 矩形2右下角坐标
 * 输出参数: 无
 * 返回值  : 是否相交
 * 说明    :  矩形A的宽 Wa = Xa2-Xa1 高 Ha = Ya2-Ya1
            矩形B的宽 Wb = Xb2-Xb1 高 Hb = Yb2-Yb1
            矩形A的中心坐标 (Xa3,Ya3) = （ (Xa2+Xa1)/2 ，(Ya2+Ya1)/2 ）
            矩形B的中心坐标 (Xb3,Yb3) = （ (Xb2+Xb1)/2 ，(Yb2+Yb1)/2 ）
            只要同时满足下面两个式子，就可以说明两个矩形相交
            1） | Xb3-Xa3 | <= Wa/2 + Wb/2
            2） | Yb3-Ya3 | <= Ha/2 + Hb/2
            即：
            | Xb2+Xb1-Xa2-Xa1 | <= Xa2-Xa1 + Xb2-Xb1
            | Yb2+Yb1-Ya2-Ya1 | <= Ya2-Ya1 + Yb2-Yb1
           
 *****************************************************************************/
SV_BOOL pd_Rect2Intersect(SV_POINT2_S lt1, SV_POINT2_S rb1, SV_POINT2_S lt2, SV_POINT2_S rb2)
{
    SV_POINT2_S c1, c2;     // 两个矩形的中心点坐标
    double w1, h1, w2, h2;  // 两个矩形的宽高

    c1.dX = (lt1.dX + rb1.dX) / 2.0;
    c1.dY = (lt1.dY + rb1.dY) / 2.0;
    c2.dX = (lt2.dX + rb2.dX) / 2.0;
    c2.dY = (lt2.dY + rb2.dY) / 2.0;
    w1 = rb1.dX - lt1.dX;
    h1 = rb1.dY - lt1.dY;
    w2 = rb2.dX - lt2.dX;
    h2 = rb2.dY - lt2.dY;

    //print_level(SV_DEBUG, "c1(%fx%f), c2(%fx%f), w1:%f, h1:%f, w2:%f, h2:%f\n", c1.dX, c1.dY, c2.dX, c2.dY, w1, h1, w2, h2);
    if (fabs(c1.dX - c2.dX) <= (w1/2.0 + w2/2.0) && fabs(c1.dY - c2.dY) <= (h1/2.0 + h2/2.0))
    {
        return SV_TRUE;
    }

    return SV_FALSE;
}

/*
 * 判断区间[x1, x2]和区间[x3, x4](x4可能小于x3)是否有交集，有返回1，无0
 */
SV_BOOL pd_IntervalOverlap(double x1, double x2, double x3, double x4)
{
    double tmp;

    if (x3 > x4)
    {
        tmp = x3;
        x3 = x4;
        x4 = tmp;
    }

    if (x3 > x2 || x4 < x1)
    {
        return SV_FALSE;
    }
    else
    {
        return SV_TRUE;
    }
}

/******************************************************************************
 * 函数功能: 判断一个矩形是否与一条线段相交
 * 输入参数: lt --- 矩形左上角坐标(按屏幕坐标，左上角为原点)
             rb --- 矩形右下角坐标
             pa --- 线段顶点A
             pb --- 线段顶点B
 * 输出参数: 无
 * 返回值  : 是否相交
 * 说明    :  
           ^ y轴               N
           :    |-------------| --> C
           :    |             |    B
           :    |             |   /
           :    |             |  /
           :    |-------------| /D
           :    M              /
           :                  /
           :                 /                   
           :                A
           :    E-------------F  G-H
           ++++++++++++++++++++++++++++++++++++++++++++++++++> x轴

           通过M和N点的y坐标计算直线AB上的D和C点，B和C点中取y值小的点B，A和D点中取y值大的点D
           最后确定了线段BD在x轴上的投影GH，矩形在x轴上的投影EF，判断EF和GH是否有交集
 *****************************************************************************/
SV_BOOL pd_RectLineIntersect(SV_POINT2_S lt, SV_POINT2_S rb, SV_POINT2_S pa, SV_POINT2_S pb)
{
    SV_POINT2_S tmp, pc, pd;
    
    if (pa.dY == pb.dY) // 线段平行于x轴
    {
        return pd_IntervalOverlap(lt.dX, rb.dX, pa.dX, pb.dX);
    }

    if (pa.dY > pb.dY)  // AB两点交换，让B点的y坐标最大
    {
        tmp = pa;
        pa = pb;
        pb = tmp;
    }

    /* 在线段AB上确定点C和D */
    double k = (pb.dX - pa.dX) / (pb.dY - pa.dY); // 算出斜率, 两点确定一条直线: (x-x1)/(x2-x1)=(y-y1)/(y2-y1)
    if (pa.dY < lt.dY)
    {
        pc.dY = lt.dY;
        pc.dX = k * (pc.dY - pa.dY) + pa.dX;
    }
    else
    {
        pc = pa;
    }

    if (pb.dY > rb.dY)
    {
        pd.dY = rb.dY;
        pd.dX = k * (pd.dY - pa.dY) + pa.dX;
    }
    else
    {
        pd = pb;
    }

    //print_level(SV_DEBUG, "r(%fx%f %fx%f) pa(%fx%f), pb(%fx%f), pc(%fx%f), pd(%fx%f)\n", lt.dX, lt.dY, rb.dX, rb.dY, pa.dX, pa.dY, pb.dX, pb.dY, pc.dX, pc.dY);
    if (pd.dY >= pc.dY) // y维上有交集
    {
        return pd_IntervalOverlap(lt.dX, rb.dX, pc.dX, pd.dX);
    }

    return SV_FALSE;
}

/******************************************************************************
 * 函数功能: 判断两个线段是否相交
 * 输入参数: pa --- 线段AB的A点坐标
             pb --- 线段AB的B点坐标
             pc --- 线段CD的C点坐标
             pd --- 线段CD的D点坐标
 * 输出参数: 无
*****************************************************************************/
SV_BOOL pd_Line2Intersect(SV_POINT2_S pa, SV_POINT2_S pb, SV_POINT2_S pc, SV_POINT2_S pd)
{
    if((max(pa.dX, pb.dX) < min(pc.dX, pd.dX)) ||
       (max(pc.dX, pd.dX) < min(pa.dX, pb.dX)) ||
       (max(pa.dY, pb.dY) < min(pc.dY, pd.dY)) ||
       (max(pc.dY, pd.dY) < min(pa.dY, pb.dY)))
       return SV_FALSE;

    if((((pb.dX-pa.dX)*(pc.dY-pa.dY)-(pb.dY-pa.dY)*(pc.dX-pa.dX)) * ((pb.dX-pa.dX)*(pd.dY-pa.dY)-(pb.dY-pa.dY)*(pd.dX-pa.dX)) >= 0) ||
       (((pd.dX-pc.dX)*(pa.dY-pc.dY)-(pd.dY-pc.dY)*(pa.dX-pc.dX)) * ((pd.dX-pc.dX)*(pb.dY-pc.dY)-(pd.dY-pc.dY)*(pb.dX-pc.dX)) >= 0) )
        return SV_FALSE;
    
    return SV_TRUE;
}


/******************************************************************************
 * 函数功能: 判断一个坐标点是否位于多边形内(边上也算)
 * 输入参数: poy --- 多边形坐标数组
             nb --- 多边形坐标数量
             pot  --- 该坐标点的坐标
 * 输出参数: 无
 * 返回值  : 是否位于多边形内
 * 说明    :  通过计算该坐标点到多边形顶点的线段和该点邻边的向量积来判断
            坐标方向 —————————————→ X
                    |⊕ ⊕ ⊕ ⊕
                    |⊕ ⊕ ⊕ ⊕
                    |⊕ ⊕ ⊕ ⊕
                    |⊕ ⊕ ⊕ ⊕
                    |⊕ ⊕ ⊕ ⊕
                    ↓           Z
                    Y
                    
 * 注意  :    多边形坐标需要逆时针排列
 *****************************************************************************/
SV_BOOL pd_PolygonPointIntersect(SV_POINT2_S *poy, int nb, SV_POINT2_S pot)
{
    int i;
    
    for(i = 0; i < nb-1; i++)
    {
        if(((poy[i].dX - pot.dX)*(poy[i+1].dY - poy[i].dY) - (poy[i].dY - pot.dY)*(poy[i+1].dX - poy[i].dX)) > 0)
        {
            return SV_FALSE;
        }
    }

    
    if(((poy[i].dX - pot.dX)*(poy[0].dY - poy[i].dY) - (poy[i].dY - pot.dY)*(poy[0].dX - poy[i].dX)) > 0)
    {
        return SV_FALSE;
    }

    return SV_TRUE;
}



/******************************************************************************
 * 函数功能: 判断两个多边形是否有交集
 * 输入参数: po1 --- 多边形1坐标数组
             nb1 --- 多边形1坐标数量
             po2 --- 多边形2坐标数组
             nb2 --- 多边形2坐标数量
 * 输出参数: 无
 * 返回值  : 是否有交集
 * 说明    :  循环判断多边形1的所有坐标点是否位于多边形2(边上也算),
            循环判断多边形2的所有坐标点是否位于多边形1(边上也算),
            如果有任意一个坐标点满足条件, 说明两个多边形有交集
 * 注意  :    多边形坐标需要逆时针排列
 *****************************************************************************/
SV_BOOL pd_Polygon2Intersect(SV_POINT2_S *po1, int nb1, SV_POINT2_S *po2, int nb2)
{
    int i, j;

    for(i = 0; i < nb2; i++)
    {
        if(pd_PolygonPointIntersect(po1 , nb1, po2[i]))
        {
            return SV_TRUE;
        }
    }

    for(i = 0; i < nb1; i++)
    {
        if(pd_PolygonPointIntersect(po2, nb2, po1[i]))
        {
            return SV_TRUE;
        }
    }
#if 1
    for(i = 0; i < nb1; i++)
    {
        for(j = 0; j < nb2; j++)
        {
            if(pd_Line2Intersect(po1[i], po1[(i+1)%nb1], po2[j], po2[(j+1)%nb2]))
                return SV_TRUE;
        }
    }
#endif
    return SV_FALSE;
}

/******************************************************************************
 * 函数功能: 判断椭圆多边形和多边形是否有交集(每个多边形有四个顶点)
 * 输入参数: po1 --- 多边形1坐标数组
             po2 --- 多边形2坐标数组
             B1  --- 椭圆1 Y轴长度(1)
             B2  --- 椭圆2 X轴长度(1)
             
 * 输出参数: 无
 * 返回值  : 是否有交集
 * 说明    :  循环判断多边形1的所有坐标点是否位于多边形2(边上也算),
            循环判断多边形2的所有坐标点是否位于多边形1(边上也算),
            如果有任意一个坐标点满足条件, 说明两个多边形有交集
 * 注意  :    多边形坐标需要逆时针排列
 *****************************************************************************/
SV_BOOL pd_EllipsePolygon2Intersect(SV_POINT2_S *po1, SV_POINT2_S *po2, float B1, float B2)
{
    int i, j;
    float A1, A2;
    float X1, Y1, X2, Y2;
    float A1_2, A2_2, B1_2, B2_2;
    SV_BOOL bIn1, bIn2, bReturn;
    
    A1 = (po1[3].dX - po1[0].dX) / 2;
    A2 = (po1[2].dX - po1[1].dX) / 2;
    A1_2 = A1 * A1;
    A2_2 = A2 * A2;
    B1_2 = B1 * B1;
    B2_2 = B2 * B2;
    
    X1 = (po1[3].dX + po1[0].dX) / 2;
    Y1 = (po1[3].dY + po1[0].dY) / 2;
    
    X2 = (po1[2].dX + po1[1].dX) / 2;
    Y2 = (po1[2].dY + po1[1].dY) / 2;

    for(i = 0; i < 4; i++)
    {
        if(pd_PolygonPointIntersect(po2, 4, po1[i]))
        {
            return SV_TRUE;
        }
    }

    for(i = 0; i < 4; i++)
    {
        bIn1 = SV_FALSE;
        bIn2 = SV_FALSE;
        
        if ((po2[i].dX - X1) * (po2[i].dX - X1) * B1_2 + (po2[i].dY - Y1) * (po2[i].dY - Y1) * A1_2 < A1_2 * B1_2 )
            bIn1 = SV_TRUE;
        
        if ((po2[i].dX - X2) * (po2[i].dX - X2) * B2_2 + (po2[i].dY - Y2) * (po2[i].dY - Y2) * A2_2 < A2_2 * B2_2 )
            bIn2 = SV_TRUE;


        if(B1 < 0 && bIn1 && po2[i].dY < Y1)
        {
            return SV_TRUE;
        }

        if(B2 > 0 && bIn2 && po2[i].dY > Y2)
        {
            return SV_TRUE;
        }

        if(pd_PolygonPointIntersect(po1 , 4, po2[i]))
        {
            bReturn = SV_FALSE;
            
            if (B1 > 0 && !bIn1)
            {
                bReturn = SV_TRUE;
            }

            if (B2 < 0 && !bIn2)
            {
                bReturn = SV_TRUE;
            }

            if(bReturn)
                return SV_TRUE;
        }

    }


#if 0
    for(i = 0; i < 4; i++)
    {
        for(j = 0; j < 4; j++)
        {
            if(pd_Line2Intersect(po1[i], po1[(i+1)%4], po2[j], po2[(j+1)%4]))
                return SV_TRUE;
        }
    }
#endif
    return SV_FALSE;
}


/******************************************************************************
 * 函数功能: 判断多边形B是否在多边形A内部
 * 输入参数: f32XMin1, f32YMin1---镂空区域左上角坐标
             f32XMax1, f32YMax1---镂空区域右下角坐标
             f32XMin2, f32YMin2---行人检测框左上角坐标
             f32XMax2, f32YMax2---行人检测框左下角坐标
 * 输出参数: 无
 * 返回值  : 是否在内部
 * 说明    :   两个框相交的面积/行人检测框的面积         
 * 注意  :    多边形坐标需要逆时针排列
 *****************************************************************************/
SV_BOOL pd_Polygon2Interior(float f32XMin1, float f32YMin1, float f32XMax1, float f32YMax1,
									float f32XMin2, float f32YMin2, float f32XMax2, float f32YMax2)
{
		/*** Check the input, and change the Return value  ***/
		float f32Inter = 0;
		float f32Total = 0;
		float f32XMin = 0;
		float f32YMin = 0;
		float f32XMax = 0;
		float f32YMax = 0;
		float f32Area1 = 0;
		float f32Area2 = 0;
		float f32InterWidth = 0;
		float f32InterHeight = 0;
	
		f32XMin = ALG_MAX(f32XMin1, f32XMin2);
		f32YMin = ALG_MAX(f32YMin1, f32YMin2);
		f32XMax = ALG_MIN(f32XMax1, f32XMax2);
		f32YMax = ALG_MIN(f32YMax1, f32YMax2);
	
		f32InterWidth = f32XMax - f32XMin + 1e-6;
		f32InterHeight = f32YMax - f32YMin + 1e-6;
	
		f32InterWidth = ( f32InterWidth >= 0 ) ? f32InterWidth : 0;
		f32InterHeight = ( f32InterHeight >= 0 ) ? f32InterHeight : 0;
	
		f32Inter = f32InterWidth * f32InterHeight;
		f32Area1 = (f32XMax1 - f32XMin1 + 1e-6) * (f32YMax1 - f32YMin1 + 1e-6);
		f32Area2 = (f32XMax2 - f32XMin2 + 1e-6) * (f32YMax2 - f32YMin2 + 1e-6);
	
		f32Total = f32Area1 + f32Area2 - f32Inter;
	
		if(f32Inter>0.75*f32Area2)
		{
			//print_level(SV_DEBUG,"f32Inter:%f  f32Area1:%f   f32Area2:%f Ratio:%f\n",f32Inter,f32Area1,f32Area2,f32Inter/f32Area2);
			return SV_TRUE;
		}
		
		return SV_FALSE;

}



/******************************************************************************
 * 函数功能: 判断矩形框和半圆是否有交集
 * 输入参数: center --- 圆心
             radius --- 半径
             po1 --- 矩形框坐标数组
 * 输出参数: 无
 * 返回值  : 是否有交集
 * 说明    :  详见代码内说明
 * 注意  :    矩形的点分别为左上角，左下角，右下角，右上角
*****************************************************************************/
SV_BOOL pd_SemiCircleRectIntersect(SV_POINT2_S center, double radius, SV_POINT2_S *pRect)
{
    int i;
    double dx, dy;
    double radius_2 = radius * radius;
    SV_POINT2_S rcenter;    /* 矩形中心 */
    double width, height;

    /* 检测矩形框四个点是否位于半圆中 */
    for(i = 0; i < 4; i++)
    {
        dx = pRect[i].dX - center.dX;
        dy = pRect[i].dY - center.dY;
        if(dy > 0)
        {
            continue;
        }
        
        if(dx * dx + dy * dy <= radius_2)
        {
            return SV_TRUE;
        }
    }

    rcenter.dX = (pRect[0].dX + pRect[2].dX) / 2;
    rcenter.dY = (pRect[0].dY + pRect[2].dY) / 2;
    width = pRect[2].dX - pRect[0].dX;
    height = pRect[2].dY - pRect[0].dY;
    
    /* 检测圆是否包含在矩形相交 */
    if(center.dY >= pRect[0].dY) /* 当且仅当圆心比矩形左上角点的坐标大时才会相交 */
    {
        if((abs(center.dY - rcenter.dY) < height / 2 + radius) && (abs(center.dX - rcenter.dX) < width / 2))
        {
            return SV_TRUE;
        }

        if((abs(center.dX - rcenter.dX) < width / 2 + radius) && (abs(center.dY - rcenter.dY) < height / 2))
            return SV_TRUE;
    }

    return SV_FALSE;
}




/******************************************************************************
 * 函数功能: 检查行人矩形所属纵向排布的标定区域
 * 输入参数: s32Chn --- 通道号
             pstRect --- 行人矩形
 * 输出参数: 无
 * 返回值  : 区域类型
 * 说明    : *     1.1 #--# 2.1      *
           *         /绿\           *
           *    1.2 ------ 2.2     *
           *       /| 黄 |\         *
           *   1.3 -------- 2.3    *
           *     /|   红    |\      *
           *  1.4 ---#--#--- 2.4   *
   
 *****************************************************************************/
PD_ROI_E pd_CheckRoiVerticalLayout(sint32 s32Chn, pdsa32::STAlgResult *pstRect)
{
    CFG_PDS_PARAM *pstPdsParam = NULL;

    if (s32Chn >= ALG_MAX_CHN)
    {
        return ERR_ILLEGAL_PARAM;
    }

    if (NULL == pstRect)
    {
        return ERR_NULL_PTR;
    }
    
#if (defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1))
    switch (s32Chn)
    {
        case 0:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh1.stPdsParam;
            break;
        case 1:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
            break;
        case 2:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh3.stPdsParam;
            break;
    }
#elif (defined(BOARD_ADA32IR))
    switch (s32Chn)
    {
        case 0:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
            break;
        case 1:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh1.stPdsParam;
            break;
        case 2:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh3.stPdsParam;
            break;
        default:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
    }
#else
    pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
#endif

    PD_ROI_E enRoi = pstPdsParam->bPdTestMode ? PD_ROI_BLUE : PD_ROI_BUTT;

    SV_POINT2_S stP11 =  pstPdsParam->astPdCalibrationPoints[0];
    SV_POINT2_S stP12 =  pstPdsParam->astPdCalibrationPoints[1];
    SV_POINT2_S stP13 =  pstPdsParam->astPdCalibrationPoints[2];
    SV_POINT2_S stP14 =  pstPdsParam->astPdCalibrationPoints[3];
    SV_POINT2_S stP21 =  pstPdsParam->astPdCalibrationPoints[4];
    SV_POINT2_S stP22 =  pstPdsParam->astPdCalibrationPoints[5];
    SV_POINT2_S stP23 =  pstPdsParam->astPdCalibrationPoints[6];
    SV_POINT2_S stP24 =  pstPdsParam->astPdCalibrationPoints[7];

#if 0
    SV_POINT2_S stTopLeft = pstPdsParam->astPdCalibrationPoints[0];
    SV_POINT2_S stTopRigth = pstPdsParam->astPdCalibrationPoints[4];
    SV_POINT2_S stBottomLeft = pstPdsParam->astPdCalibrationPoints[3];
    SV_POINT2_S stBottomRight = pstPdsParam->astPdCalibrationPoints[7];
    SV_POINT2_S stRectGreenLt, stRectGreenRb;   // 各区域内侧完整矩形部分的左上角和右下角的点
    SV_POINT2_S stRectYellowLt, stRectYellowRb;
    SV_POINT2_S stRectRedLt, stRectRedRb;
    SV_POINT2_S stDestLt, stDestRb;

    if (NULL == pstRect)
    {
        return PD_ROI_BUTT;
    }



    stDestLt.dX = pstRect->fX1;
    stDestLt.dY = pstRect->fY1;
    stDestRb.dX = pstRect->fX2;
    stDestRb.dY = pstRect->fY2;
    if (stP11.dX > stP14.dX)    // 左边往内侧倾斜
    {
        stRectGreenLt = stP11;
        stRectYellowLt = stP12;
        stRectRedLt = stP13;
    }
    else
    {
        stRectGreenLt.dX = stP12.dX;
        stRectGreenLt.dY = stP11.dY;
        stRectYellowLt.dX = stP13.dX;
        stRectYellowLt.dY = stP12.dY;
        stRectRedLt.dX = stP14.dX;
        stRectRedLt.dY = stP13.dY;
    }

    if (stP21.dX > stP24.dX)    // 右边往外侧倾斜
    {
        stRectGreenRb = stP22;
        stRectYellowRb = stP23;
        stRectRedRb = stP24; 
    }
    else
    {
        stRectGreenRb.dX = stP21.dX;
        stRectGreenRb.dY = stP22.dY;
        stRectYellowRb.dX = stP22.dX;
        stRectYellowRb.dY = stP23.dY;
        stRectRedRb.dX = stP23.dX;
        stRectRedRb.dY = stP24.dY;
    }

    if (pd_Rect2Intersect(stDestLt, stDestRb, stRectRedLt, stRectRedRb)     // 判断行人矩形与区域的矩形部分是否有相交
        || pd_RectLineIntersect(stDestLt, stDestRb, stP13, stP14)           // 判断行人矩形与区域的左边是否有相交
        || pd_RectLineIntersect(stDestLt, stDestRb, stP23, stP24))          // 判断行人矩形与区域的右边是否有相交
    {
        enRoi =  PD_ROI_RED;
    }
    else if (pd_Rect2Intersect(stDestLt, stDestRb, stRectYellowLt, stRectYellowRb)
            || pd_RectLineIntersect(stDestLt, stDestRb, stP12, stP13)
            || pd_RectLineIntersect(stDestLt, stDestRb, stP22, stP23))        
    {
        enRoi = PD_ROI_YELLOW;
    }
    else if (pd_Rect2Intersect(stDestLt, stDestRb, stRectGreenLt, stRectGreenRb)
            || pd_RectLineIntersect(stDestLt, stDestRb, stP11, stP12)
            || pd_RectLineIntersect(stDestLt, stDestRb, stP21, stP22))
    {
        enRoi = PD_ROI_GREEN;
    }
#else
    SV_POINT2_S stRectGreen[4], stRectYellow[4], stRectRed[4];
    SV_POINT2_S stRectDest[4];
    if (NULL == pstRect)
    {
        return PD_ROI_BUTT;
    }

    stRectGreen[0] = stP11;
    stRectGreen[1] = stP12;
    stRectGreen[2] = stP22;
    stRectGreen[3] = stP21;

    stRectYellow[0] = stP12;
    stRectYellow[1] = stP13;
    stRectYellow[2] = stP23;
    stRectYellow[3] = stP22;

    stRectRed[0] = stP13;
    stRectRed[1] = stP14;
    stRectRed[2] = stP24;
    stRectRed[3] = stP23;

    stRectDest[0].dX = pstRect->fX1;
    stRectDest[0].dY = pstRect->fY1;
    stRectDest[1].dX = pstRect->fX1;
    stRectDest[1].dY = pstRect->fY2;
    stRectDest[2].dX = pstRect->fX2;
    stRectDest[2].dY = pstRect->fY2;
    stRectDest[3].dX = pstRect->fX2;
    stRectDest[3].dY = pstRect->fY1;

    if(pstPdsParam->bPdRoiRed && pd_Polygon2Intersect(stRectRed, 4, stRectDest, 4))
    {
        enRoi =  PD_ROI_RED;
    }
    else if(pstPdsParam->bPdRoiYellow && pd_Polygon2Intersect(stRectYellow, 4, stRectDest, 4))
    {
        enRoi = PD_ROI_YELLOW;
    }
    else if(pstPdsParam->bPdRoiGreen && pd_Polygon2Intersect(stRectGreen, 4, stRectDest, 4))
    {
        enRoi = PD_ROI_GREEN;
    }

	
#endif

    return enRoi;
}

/******************************************************************************
 * 函数功能: 检查行人矩形所属横向排布的标定区域
 * 输入参数: pstRect --- 行人矩形
             bLeftRed --- 是否红区在左边
 * 输出参数: 无
 * 返回值  : 区域类型
 * 说明    : * 1.1   1.2   1.3   1.4   *
           *  |     |     |     |    *
           *  |     |     |     |    *
           *  |红/绿|    黄  |红/绿|      *
           *  |     |     |     |    *
           *  |     |     |     |    *
           * 2.1   2.2   2.3   2.4   *
   
 *****************************************************************************/
PD_ROI_E pd_CheckRoiHorizontalLayout(sint32 s32Chn, pdsa32::STAlgResult *pstRect, SV_BOOL bLeftRed)
{
    CFG_PDS_PARAM *pstPdsParam = NULL;

    if (s32Chn >= ALG_MAX_CHN)
    {
        return ERR_ILLEGAL_PARAM;
    }

    if (NULL == pstRect)
    {
        return ERR_NULL_PTR;
    }
    
#if (defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1))
    switch (s32Chn)
    {
        case 0:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh1.stPdsParam;
            break;
        case 1:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
            break;
        case 2:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh3.stPdsParam;
            break;
    }
#elif (defined(BOARD_ADA32IR))
    switch (s32Chn)
    {
        case 0:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
            break;
        case 1:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh1.stPdsParam;
            break;
        case 2:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh3.stPdsParam;
            break;
        default:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
    }
#else
    pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
#endif
    PD_ROI_E enRoi = pstPdsParam->bPdTestMode ? PD_ROI_BLUE : PD_ROI_BUTT;

    SV_POINT2_S stP11 =  pstPdsParam->astPdCalibrationPoints[0];
    SV_POINT2_S stP12 =  pstPdsParam->astPdCalibrationPoints[1];
    SV_POINT2_S stP13 =  pstPdsParam->astPdCalibrationPoints[2];
    SV_POINT2_S stP14 =  pstPdsParam->astPdCalibrationPoints[3];
    SV_POINT2_S stP21 =  pstPdsParam->astPdCalibrationPoints[4];
    SV_POINT2_S stP22 =  pstPdsParam->astPdCalibrationPoints[5];
    SV_POINT2_S stP23 =  pstPdsParam->astPdCalibrationPoints[6];
    SV_POINT2_S stP24 =  pstPdsParam->astPdCalibrationPoints[7];
    SV_POINT2_S stRectGreen[4], stRectYellow[4], stRectRed[4];
    SV_POINT2_S stRectDest[4];

    if (NULL == pstRect)
    {
        return PD_ROI_BUTT;
    }

    stRectGreen[0] = (bLeftRed == SV_TRUE)? stP13: stP11;
    stRectGreen[1] = (bLeftRed == SV_TRUE)? stP23: stP21;
    stRectGreen[2] = (bLeftRed == SV_TRUE)? stP24: stP22;
    stRectGreen[3] = (bLeftRed == SV_TRUE)? stP14: stP12;

    stRectYellow[0] = stP12;
    stRectYellow[1] = stP22;
    stRectYellow[2] = stP23;
    stRectYellow[3] = stP13;

    stRectRed[0] = (bLeftRed == SV_FALSE)? stP13: stP11;
    stRectRed[1] = (bLeftRed == SV_FALSE)? stP23: stP21;
    stRectRed[2] = (bLeftRed == SV_FALSE)? stP24: stP22;
    stRectRed[3] = (bLeftRed == SV_FALSE)? stP14: stP12;

    stRectDest[0].dX = pstRect->fX1;
    stRectDest[0].dY = pstRect->fY1;
    stRectDest[1].dX = pstRect->fX1;
    stRectDest[1].dY = pstRect->fY2;
    stRectDest[2].dX = pstRect->fX2;
    stRectDest[2].dY = pstRect->fY2;
    stRectDest[3].dX = pstRect->fX2;
    stRectDest[3].dY = pstRect->fY1;

    if(pstPdsParam->bPdRoiRed && pd_Polygon2Intersect(stRectRed, 4, stRectDest, 4))
    {
        enRoi =  PD_ROI_RED;
    }
    else if(pstPdsParam->bPdRoiYellow && pd_Polygon2Intersect(stRectYellow, 4, stRectDest, 4))
    {
        enRoi = PD_ROI_YELLOW;
    }
    else if(pstPdsParam->bPdRoiGreen && pd_Polygon2Intersect(stRectGreen, 4, stRectDest, 4))
    {
        enRoi = PD_ROI_GREEN;
    }

    return enRoi;
}

/******************************************************************************
 * 函数功能: 检查行人矩形所属横向排布的标定区域
 * 输入参数: pstRect --- 行人矩形
 * 输出参数: 无
 * 返回值  : 区域类型
 * 说明    :   *          --------           *
             *        ---++++++---         *
             *      ---+++++*++++---       *
             *     ---+++++***++++---      *
             *    ---+++++*****+++++---    *
             *   ---+++++*******+++++---   *
             *  1.2 1.3 1.4 1.1            *
             *  |绿|黄|红|圆心|                 *
   
 *****************************************************************************/

PD_ROI_E pd_CheckRoiSemiCircleLayout(sint32 s32Chn, pdsa32::STAlgResult *pstRect)
{
    CFG_PDS_PARAM *pstPdsParam = NULL;

    if (s32Chn >= ALG_MAX_CHN)
    {
        return ERR_ILLEGAL_PARAM;
    }

    if (NULL == pstRect)
    {
        return ERR_NULL_PTR;
    }

#if (defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1))
    switch (s32Chn)
    {
        case 0:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh1.stPdsParam;
            break;
        case 1:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
            break;
        case 2:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh3.stPdsParam;
            break;
    }
#elif (defined(BOARD_ADA32IR))
    switch (s32Chn)
    {
        case 0:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
            break;
        case 1:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh1.stPdsParam;
            break;
        case 2:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh3.stPdsParam;
            break;
        default:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
    }
#else
    pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
#endif

    PD_ROI_E enRoi = pstPdsParam->bPdTestMode ? PD_ROI_BLUE : PD_ROI_BUTT;
    SV_POINT2_S stP11 =  pstPdsParam->astPdCalibrationPoints[0];
    SV_POINT2_S stP12 =  pstPdsParam->astPdCalibrationPoints[1];
    SV_POINT2_S stP13 =  pstPdsParam->astPdCalibrationPoints[2];
    SV_POINT2_S stP14 =  pstPdsParam->astPdCalibrationPoints[3];
    SV_POINT2_S stP21 =  pstPdsParam->astPdCalibrationPoints[4];
    SV_POINT2_S stP22 =  pstPdsParam->astPdCalibrationPoints[5];
    SV_POINT2_S stP23 =  pstPdsParam->astPdCalibrationPoints[6];
    SV_POINT2_S stP24 =  pstPdsParam->astPdCalibrationPoints[7];
    double eps = 1080.0 / 1920.0; // 对应圆形判断，需要进行比例转换

    SV_POINT2_S stRectDest[4];
    SV_POINT2_S stCenter;
    double radius[3];
    
    if (NULL == pstRect)
    {
        return PD_ROI_BUTT;
    }

    stRectDest[0].dX = pstRect->fX1;
    stRectDest[0].dY = pstRect->fY1 * eps;
    stRectDest[1].dX = pstRect->fX1;
    stRectDest[1].dY = pstRect->fY2 * eps;
    stRectDest[2].dX = pstRect->fX2;
    stRectDest[2].dY = pstRect->fY2 * eps;
    stRectDest[3].dX = pstRect->fX2;
    stRectDest[3].dY = pstRect->fY1 * eps;

    stCenter.dX = stP11.dX;
    stCenter.dY = stP11.dY * eps;

    radius[0] = stP11.dX - stP12.dX;
    radius[1] = stP11.dX - stP13.dX;
    radius[2] = stP11.dX - stP14.dX;


    if(pstPdsParam->bPdRoiRed && pd_SemiCircleRectIntersect(stCenter, radius[2], stRectDest))
    {
        enRoi =  PD_ROI_RED;
    }
    else if(pstPdsParam->bPdRoiYellow && pd_SemiCircleRectIntersect(stCenter, radius[1], stRectDest))
    {
        enRoi = PD_ROI_YELLOW;
    }
    else if(pstPdsParam->bPdRoiGreen && pd_SemiCircleRectIntersect(stCenter, radius[0], stRectDest))
    {
        enRoi = PD_ROI_GREEN;
    }

    return enRoi;
}

/******************************************************************************
 * 函数功能: 检查行人矩形是否在椭圆区域内
 * 输入参数: s32Chn --- 通道号
             pstRect --- 行人矩形
 * 输出参数: 无
 * 返回值  : 区域类型
 * 说明    : *     1.1 #--# 2.1      *
           *         /绿\           *
           *    1.2 ------ 2.2     *
           *       /| 黄 |\         *
           *   1.3 -------- 2.3    *
           *     /|   红    |\      *
           *  1.4 ---#--#--- 2.4   *

           3.1 空白区Y轴长度
           3.2 绿区Y轴长度
           3.3 黄区Y轴长度
 *****************************************************************************/
PD_ROI_E pd_CheckRoiEllipseLayout(sint32 s32Chn, pdsa32::STAlgResult *pstRect)
{
    CFG_PDS_PARAM *pstPdsParam = NULL;

    if (s32Chn >= ALG_MAX_CHN)
    {
        return ERR_ILLEGAL_PARAM;
    }

    if (NULL == pstRect)
    {
        return ERR_NULL_PTR;
    }
    
#if (defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1))
    switch (s32Chn)
    {
        case 0:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh1.stPdsParam;
            break;
        case 1:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
            break;
        case 2:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh3.stPdsParam;
            break;
    }
#elif (defined(BOARD_ADA32IR))
    switch (s32Chn)
    {
        case 0:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
            break;
        case 1:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh1.stPdsParam;
            break;
        case 2:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh3.stPdsParam;
            break;
        default:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
    }
#else
    pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
#endif

    PD_ROI_E enRoi = pstPdsParam->bPdTestMode ? PD_ROI_BLUE : PD_ROI_BUTT;

    SV_POINT2_S stP11 =  pstPdsParam->astPdCalibrationPoints[0];
    SV_POINT2_S stP12 =  pstPdsParam->astPdCalibrationPoints[1];
    SV_POINT2_S stP13 =  pstPdsParam->astPdCalibrationPoints[2];
    SV_POINT2_S stP14 =  pstPdsParam->astPdCalibrationPoints[3];
    SV_POINT2_S stP21 =  pstPdsParam->astPdCalibrationPoints[4];
    SV_POINT2_S stP22 =  pstPdsParam->astPdCalibrationPoints[5];
    SV_POINT2_S stP23 =  pstPdsParam->astPdCalibrationPoints[6];
    SV_POINT2_S stP24 =  pstPdsParam->astPdCalibrationPoints[7];


    SV_POINT2_S stRectGreen[4], stRectYellow[4], stRectRed[4];
    SV_POINT2_S stRectDest[4];
    float fGreenB[2], fYellowB[2], fRedB[2];
    
    if (NULL == pstRect)
    {
        return PD_ROI_BUTT;
    }

    stRectGreen[0] = stP11;
    stRectGreen[1] = stP12;
    stRectGreen[2] = stP22;
    stRectGreen[3] = stP21;
    fGreenB[0] = pstPdsParam->fEllipseB[0];
    fGreenB[1] = pstPdsParam->fEllipseB[1];


    stRectYellow[0] = stP12;
    stRectYellow[1] = stP13;
    stRectYellow[2] = stP23;
    stRectYellow[3] = stP22;
    fYellowB[0] = pstPdsParam->fEllipseB[1];
    fYellowB[1] = pstPdsParam->fEllipseB[2];

    stRectRed[0] = stP13;
    stRectRed[1] = stP14;
    stRectRed[2] = stP24;
    stRectRed[3] = stP23;
    fRedB[0] = pstPdsParam->fEllipseB[2];
    fRedB[1] = 0;

    stRectDest[0].dX = pstRect->fX1;
    stRectDest[0].dY = pstRect->fY1;
    stRectDest[1].dX = pstRect->fX1;
    stRectDest[1].dY = pstRect->fY2;
    stRectDest[2].dX = pstRect->fX2;
    stRectDest[2].dY = pstRect->fY2;
    stRectDest[3].dX = pstRect->fX2;
    stRectDest[3].dY = pstRect->fY1;

    if(pstPdsParam->bPdRoiRed && pd_EllipsePolygon2Intersect(stRectRed, stRectDest, fRedB[0], fRedB[1]))
    {
        enRoi =  PD_ROI_RED;
    }
    else if(pstPdsParam->bPdRoiYellow && pd_EllipsePolygon2Intersect(stRectYellow, stRectDest, fYellowB[0], fYellowB[1]))
    {
        enRoi = PD_ROI_YELLOW;
    }
    else if(pstPdsParam->bPdRoiGreen && pd_EllipsePolygon2Intersect(stRectGreen, stRectDest, fGreenB[0], fGreenB[1]))
    {
        enRoi = PD_ROI_GREEN;
    }

    return enRoi;
}

/******************************************************************************
 * 函数功能: 检查行人矩形是否在画板区域内
 * 输入参数: s32Chn --- 通道号
             pstRect --- 行人矩形
 * 输出参数: 无
 * 返回值  : 区域类型
 * 说明    : *         ————          *
           *       /      \        *
           *      /        \       *
           *     /           ——    *
           *    |     绿        |   *
           *    |          ————    *
           *     \        |        *
           *      \       |        *
           *        ——————         *
           ROI 区域的点按照逆时针排列，点可以任意组成各种形状
           一共有红，黄，绿三种ROI区域
 *****************************************************************************/
PD_ROI_E pd_CheckRoiDrawBoardLayout(sint32 s32Chn, pdsa32::STAlgResult *pstRect)
{
#if (!defined(BOARD_ADA47V1))
    CFG_PDS_PARAM *pstPdsParam = NULL;
    SV_POINT2_S stRectL, stRectR;
    PD_ROI_E enRoi ;
    sint32 s32Result;

    if (s32Chn >= ALG_MAX_CHN)
    {
        return ERR_ILLEGAL_PARAM;
    }

    if (NULL == pstRect)
    {
        return ERR_NULL_PTR;
    }
    

#if (defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1))
    switch (s32Chn)
    {
        case 0:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh1.stPdsParam;
            break;
        case 1:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
            break;
        case 2:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh3.stPdsParam;
            break;
    }
#elif (defined(BOARD_ADA32IR))
    switch (s32Chn)
    {
        case 0:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
            break;
        case 1:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh1.stPdsParam;
            break;
        case 2:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh3.stPdsParam;
            break;
        default:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
    }
#else
    pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
#endif


    enRoi = pstPdsParam->bPdTestMode ? PD_ROI_BLUE : PD_ROI_BUTT;
    stRectL.dX = pstRect->fX1;
    stRectL.dY = pstRect->fY1;
    stRectR.dX = pstRect->fX2;
    stRectR.dY = pstRect->fY2;

    s32Result = ROI_BOARD_Detect(stRectL, stRectR);
    switch(s32Result)
    {
        case 1:
            enRoi = PD_ROI_GREEN;
            break;
        case 2:
            enRoi = PD_ROI_YELLOW;
            break;
        case 3:
            enRoi = PD_ROI_RED;
            break;
        case 0:
        default:
            break;
    }

    return enRoi;
#endif
}


/******************************************************************************
 * 函数功能: 检查行人矩形所属纵向镂空排布的标定区域
 * 输入参数: s32Chn --- 通道号
             pstRect --- 行人矩形
 * 输出参数: 无
 * 返回值  : 区域类型
 * 说明    : *     1.1 #--#--#--#--#--#--2.1      *
           *         /绿         		 \	  	  *
           *    1.2 ------   ——————		 2.2      *
           *       /| 黄 |	|镂空    |	   \      *
           *   1.3 --------	|不检测|	   2.3    *
           *     /|   红    | ——————		\     *
           *  1.4 ---#--#---#----#--- #-----2.4   *
   
 *****************************************************************************/
PD_ROI_E  pd_CheckRoiHollow(sint32 s32Chn, pdsa32::STAlgResult *pstRect,PD_ROI_E enInRoi)
{
    CFG_PDS_PARAM *pstPdsParam = NULL;
    sint32 s32Ret;
    PD_ROI_E enOutRoi = enInRoi;

    if (s32Chn >= ALG_MAX_CHN)
    {
        return ERR_ILLEGAL_PARAM;
    }

    if (NULL == pstRect)
    {
        return ERR_NULL_PTR;
    }
    
#if (defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1))
    switch (s32Chn)
    {
        case 0:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh1.stPdsParam;
            break;
        case 1:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
            break;
        case 2:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh3.stPdsParam;
            break;
    }
#elif (defined(BOARD_ADA32IR))
    switch (s32Chn)
    {
        case 0:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
            break;
        case 1:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh1.stPdsParam;
            break;
        case 2:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh3.stPdsParam;
            break;
        default:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
    }
#else
    pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
#endif

    float xmin, xmax, ymin, ymax;

    SV_POINT2_S stP31 =  pstPdsParam->astPdCalibrationPoints[8];
    SV_POINT2_S stP32 =  pstPdsParam->astPdCalibrationPoints[9];
    SV_POINT2_S stP33 =  pstPdsParam->astPdCalibrationPoints[10];
    SV_POINT2_S stP34 =  pstPdsParam->astPdCalibrationPoints[11];


    SV_POINT2_S stRectHollow[4];
    SV_POINT2_S stRectDest[4];
    if (NULL == pstRect)
    {
        return PD_ROI_BUTT;
    }

    //stRectHollow[0] = stP31;
    //stRectHollow[1] = stP32;
    //stRectHollow[2] = stP33;
    //stRectHollow[3] = stP34;

    xmin = pstPdsParam->astPdCalibrationPoints[8].dX;
    xmax = pstPdsParam->astPdCalibrationPoints[8].dX;;
    ymin = pstPdsParam->astPdCalibrationPoints[8].dY;
    ymax = pstPdsParam->astPdCalibrationPoints[8].dY;
    for(int i = 0; i < 4; i++)
    {
        xmin = min(xmin, pstPdsParam->astPdCalibrationPoints[8+i].dX);
        xmax = max(xmax, pstPdsParam->astPdCalibrationPoints[8+i].dX);
        ymin = min(ymin, pstPdsParam->astPdCalibrationPoints[8+i].dY);
        ymax = max(ymax, pstPdsParam->astPdCalibrationPoints[8+i].dY);
    }
    stRectHollow[0].dX = xmin;
    stRectHollow[0].dY = ymin;
    stRectHollow[3].dX = xmax;
    stRectHollow[3].dY = ymax;



    stRectDest[0].dX = pstRect->fX1;
    stRectDest[0].dY = pstRect->fY1;
    stRectDest[1].dX = pstRect->fX1;
    stRectDest[1].dY = pstRect->fY2;
    stRectDest[2].dX = pstRect->fX2;
    stRectDest[2].dY = pstRect->fY2;
    stRectDest[3].dX = pstRect->fX2;
    stRectDest[3].dY = pstRect->fY1;


    s32Ret = pd_Polygon2Interior(stRectHollow[0].dX,stRectHollow[0].dY,stRectHollow[3].dX,stRectHollow[3].dY,\
                    stRectDest[0].dX,stRectDest[0].dY,stRectDest[2].dX,stRectDest[2].dY);

    if( SV_TRUE == s32Ret)
    {
        enOutRoi = PD_ROI_BUTT;
    }

    return enOutRoi;
}


/******************************************************************************
 * 函数功能: 检查行人矩形所属的标定区域
 * 输入参数: pstRect --- 行人矩形
 * 输出参数: 无
 * 返回值  : 区域类型
 * 说明    : *     1.1 #--# 2.1      *   1.1   1.2   1.3   1.4   *
           *         |绿|           *    |     |     |     |    *
           *    1.2 ------ 2.2     *    |     |     |     |    *
           *        A|黄|B          *    |红/绿|    黄  |红/绿|      *
           *   1.3 -------- 2.3    *    |     |     |     |    *
           *         |红|           *    |     |     |     |    *
           *  1.4 ---#--#--- 2.4   *   2.1   2.2   2.3   2.4   *

           *          --------              1.1 #--#--#--#--#--#2.1  		*    
           *        ---++++++---         	  |绿|							*
           *      ---+++++*++++---         1.2 -----**********---2.2		*
           *     ---+++++***++++---      	  |黄|	*镂空      *				*
           *    ---+++++*****+++++---     1.3 ------* 不检测 *----2.3			*
           *   ---+++++*******+++++---  	  |红|	**********				*
           *  1.2 1.3 1.4 1.1            1.4 ---#--#--#---#----#---2.4		*
           *  |绿|黄|红|圆心             						    *
 *****************************************************************************/
PD_ROI_E pd_CheckRoiRect(sint32 s32Chn, pdsa32::STAlgResult *pstRect)
{
    PD_ROI_E enRoi = PD_ROI_BUTT;
    CFG_PDS_PARAM *pstPdsParam = NULL;
    
    if (s32Chn >= ALG_MAX_CHN)
    {
        return ERR_ILLEGAL_PARAM;
    }
    
    if (NULL == pstRect)
    {
        return PD_ROI_BUTT;
    }

#if (defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1))
    switch (s32Chn)
    {
        case 0:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh1.stPdsParam;
            break;
        case 1:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
            break;
        case 2:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh3.stPdsParam;
            break;
    }
#elif (defined(BOARD_ADA32IR))
    switch (s32Chn)
    {
        case 0:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
            break;
        case 1:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh1.stPdsParam;
            break;
        case 2:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh3.stPdsParam;
            break;
        default:
            pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
    }
#else
    pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
#endif
    CFG_PDROI_E enRoiStyle = pstPdsParam->enRoiStyle;

#if (defined(BOARD_ADA32V2))
    /* 更换ROI区域的判断策略 */
    pdsa32::STAlgResult stRectTmp;
    switch(pstPdsParam->enDetectPart)
    {
        case E_PDS_DETECTION_TOTAL:
            break;
        case E_PDS_DETECTION_BOTTOM:
            memcpy(&stRectTmp, pstRect, sizeof(stRectTmp));
            pstRect = &stRectTmp;
            pstRect->fY1 = pstRect->fY2 - 0.0040;   // 只判断底边是否与ROI区域相交
            break;
        default:
            break;
    }
#endif

    switch (enRoiStyle)
    {
        case CFG_PDROI_BOTTOM:
            enRoi = pd_CheckRoiVerticalLayout(s32Chn, pstRect);
            break;

        case CFG_PDROI_LEFT:
            enRoi = pd_CheckRoiHorizontalLayout(s32Chn, pstRect, SV_TRUE);
            break;

        case CFG_PDROI_RIGHT:
            enRoi = pd_CheckRoiHorizontalLayout(s32Chn, pstRect, SV_FALSE);
            break;
        case CFG_PDROI_SEMICIRCLE:
            enRoi = pd_CheckRoiSemiCircleLayout(s32Chn, pstRect);
            break;
        case CFG_PDROI_ELLIPSE:
            enRoi = pd_CheckRoiEllipseLayout(s32Chn, pstRect);
            break;
        case CFG_PDROI_DRAWBOARD:
            enRoi = pd_CheckRoiDrawBoardLayout(s32Chn, pstRect);
            break;
    }

    if (enRoiStyle != CFG_PDROI_DRAWBOARD)
    {
        if(1 == m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam.enPdHollow)
        {
            enRoi = pd_CheckRoiHollow(s32Chn, pstRect,enRoi);
        }
    }
    
    return enRoi;
}

/******************************************************************************
 * 函数功能: 获取GPS信息数据
 * 输入参数: 无
 * 输出参数: pszMcuPort -- mcu对应的设备文件
 * 返回值  : SV_FAILURE -- 失败
 			 SV_SUCCESS -- 成功
 * 注意      : 无
 *****************************************************************************/
sint32 pd_GetGpsResults(sint32 &s32status, sint32 &s32speed)
{
    sint32 s32Ret;
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
 * 函数功能: 获取MCU信息数据
 * 输入参数: 无
 * 输出参数: pszMcuPort -- mcu对应的设备文件
 * 返回值  : SV_FAILURE -- 失败
             SV_SUCCESS -- 成功
 * 注意      : 无
 *****************************************************************************/
sint32 pd_GetMcuResults(uint8 &u8AlarmIn)
{
    sint32 s32Ret;
    char szJsonRet[1024] = {0};
    cJSON *pstJson = NULL, *pstTmp = NULL;
	
    s32Ret = cJSON_GetJson(DUMP_INFO_MCU, szJsonRet);
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

    pstTmp = cJSON_GetObjectItemCaseSensitive(pstJson, "bAlarmIn");
    if (NULL == pstTmp)
    {
        print_level(SV_ERROR, "keyword Status is not exist.\n");
        return SV_FAILURE;
    }

    u8AlarmIn = pstTmp->valueint;

    cJSON_Delete(pstJson);

    return SV_SUCCESS;
}


/* SPEED ZONE 发送限速车标LOGO */
sint32 pd_SpeedZoneLogoPost(SV_BOOL bLimit)
{
     return SV_SUCCESS;
    sint32 s32Ret = 0;
    uint16 u16mask;
    MSG_PACKET_S stMsgPkt = {0};
    MEDIA_GUI_DRAW_S stMediaGuiDraw = {0};
    MEDIA_GUI_NULL_S stGuiNull;
    MEDIA_GUI_SPEED_ZONE_S stGuiSpeedZone = {0};

    memset(&stMsgPkt, 0, sizeof(stMsgPkt));
    stMsgPkt.pu8Data = (uint8*)&stMediaGuiDraw;
    stMsgPkt.u32Size = sizeof(MEDIA_GUI_DRAW_S);
    //stMsgPkt.u32Size = MEDIA_GUI_SIZE(stMediaGuiDraw);
    memset(&stMediaGuiDraw, 0x00, sizeof(stMediaGuiDraw));

    stGuiSpeedZone.bLimit = bLimit;


    u16mask = MEDIA_GUI_GET_MASK(0, MEDIA_GUI_PLANE_EXTERN1, MEDIA_GUI_OP_CLEAR);
    s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiNull);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
    }

    /* 粘贴SPEEDZONE */
    u16mask = MEDIA_GUI_GET_MASK(0, MEDIA_GUI_PLANE_EXTERN1, MEDIA_GUI_OP_SPEED_ZONE);
    s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiSpeedZone);
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

/* SPEED ZONE 类别解析 */
SV_BOOL pd_SpeedZoneIsClassParse(SV_BOOL bSpeedZone, SV_BOOL bEndSpeedZone)
{
    sint32 s32Ret;    
    static sint32 s32Cnt = 0;
    static pdsa32::EAlgObjectClass eLastClass = pdsa32::E_CLS_RESERVE1;
    pdsa32::EAlgObjectClass eNowClass = pdsa32::E_CLS_RESERVE1;
    SV_BOOL IsSpeedZone = SV_FALSE;
    //SV_BOOL bLimit;

    if (bSpeedZone && bEndSpeedZone)
    {
        eNowClass = pdsa32::E_SPEED_ZONE;
    }
    else if (bSpeedZone && !bEndSpeedZone)
    {
        eNowClass = pdsa32::E_SPEED_ZONE;
    }
    else if (!bSpeedZone && bEndSpeedZone)
    {
        eNowClass = pdsa32::E_END_SPEED_ZONE;
    }
    
    if(eNowClass != pdsa32::E_SPEED_ZONE && eNowClass != pdsa32::E_END_SPEED_ZONE)
    {
        return IsSpeedZone;
    }
    
    IsSpeedZone = SV_TRUE;

    /* 连续一定数量的帧是相同的限速类型才认为有效 */
    if (s32Cnt == 0)
    {
        eLastClass = eNowClass;
        s32Cnt++;
        return IsSpeedZone;
    }
    else if (eLastClass != eNowClass)
    {
        s32Cnt = 0;
        eLastClass = eNowClass;
        return IsSpeedZone;
    }
    else if (s32Cnt < 9)
    {
        s32Cnt++;
        return IsSpeedZone;
    }
    
    s32Cnt = 0;
    
    m_stPdInfo.bLimit = pdsa32::E_SPEED_ZONE == eNowClass ? SV_TRUE : SV_FALSE;

    s32Ret = ALG_AlarmOut_Switch(m_stPdInfo.bLimit);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "ALG_AlarmOut_Switch failed! [err=%d]\n", s32Ret);
        return IsSpeedZone;
    }

    s32Ret = pd_SpeedZoneLogoPost(m_stPdInfo.bLimit);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "pd_SpeedZoneLogoPost failed! [err=%d]\n", s32Ret);
        return IsSpeedZone;
    }

    return IsSpeedZone;
}

/* SPEED ZONE 初始化 */
sint32 pd_SpeedZoneInit(PD_CFG_PARAM_S *pstInitParam)
{
    return SV_SUCCESS;
    sint32 s32Ret;
    m_stPdInfo.bLimit = SV_TRUE;
    if(pstInitParam->stAlgParam.stAlgCh2.stPdsParam.bPdAlarmOutRed == SV_FALSE &&
       pstInitParam->stAlgParam.stAlgCh2.stPdsParam.bPdAlarmOutYellow == SV_FALSE &&
       pstInitParam->stAlgParam.stAlgCh2.stPdsParam.bPdAlarmOutGreen == SV_FALSE)
    {
        m_stPdInfo.bLimit = SV_FALSE;
    }

    s32Ret = pd_SpeedZoneLogoPost(m_stPdInfo.bLimit);
    if (s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "pd_SpeedZoneLogoPost failed! [err=%d]\n", s32Ret);
        return s32Ret;
    }

    return SV_SUCCESS;
}

/* 计算A因子,1为帽子检测框,2为行人矩形框 */
float pd_SafetyHelmet_AFactor(float fX1L, float fY1L, float fX1R, float fY1R,
                     float fX2L, float fY2L, float fX2R, float fY2R)
{
    float w = min(fX1R, fX2R) - max(fX1L, fX2L);
    float h = min(fY1R, fY2R) - max(fY1L, fY2L);

    if (w < 0)
        return 0;

    if (h < 0)
        return 0;

    return w * h / ((fY1R - fY1L) * (fX1R - fX1L));
}

/* 计算B因子,1为帽子检测框,2为行人矩形框 */
float pd_SafetyHelmet_BFactor(float fX1L, float fY1L, float fX1R, float fY1R,
                     float fX2L, float fY2L, float fX2R, float fY2R)
{
    float ybias = fY1L - fY2L;
    float yheight = fY2R - fY2L;

    //if (ybias < 0)
    //    return 0;

    return 1 - abs(ybias) / yheight;
}

/******************************************************************************
 * 函数功能: 解析带帽子的信息
 * 输入参数: pstResult --- 算法返回的数据
             s32Num --- 检测数量
 * 输出参数: 无
 * 返回值  : 无
 * 说明    : 无
 *****************************************************************************/
PD_PERSON_HAT_S * pd_SafetyHelmet_Parse(pdsa32::STAlgResult *pstResult, sint32 s32Num)
{
    sint32 s32Ret, i, j, i1, i2;
    static PD_PERSON_HAT_S stPdPersonHat;
    sint32 s32PersonNoHelmet[20], s32PersonHat[20], s32Hat[20];
    sint32 s32PNoHelmetNum = 0, s32PHatNum = 0, s32HatNum = 0;
    float *pfscore = NULL;
    float fBetterScore;
    sint32 s32BetterIdx;
    float threshold = 0.7;

    memset(&stPdPersonHat, 0x00, sizeof(stPdPersonHat));
    for (i = 0; i < s32Num; i++)
    {
        if (pstResult[i].classes == pdsa32::E_CLS_PERSON)
        {
            s32PersonNoHelmet[s32PNoHelmetNum++] = i;
        }

        if (pstResult[i].classes == pdsa32::E_CLS_PERSON_HAT)
        {
            s32PersonHat[s32PHatNum++] = i;
        }

        /* 安全帽 */
        if (pstResult[i].classes >= pdsa32::E_HAT_RED && pstResult[i].classes <= pdsa32::E_HAT_BLUE)
        {
            s32Hat[s32HatNum++] = i;
        }
    }

    for (i = 0; i < s32PHatNum; i++)
    {
        i1 = s32PersonHat[i];
        stPdPersonHat.x1[stPdPersonHat.s32Num] = pstResult[i1].fX1;
        stPdPersonHat.y1[stPdPersonHat.s32Num] = pstResult[i1].fY1;
        stPdPersonHat.x2[stPdPersonHat.s32Num] = pstResult[i1].fX2;
        stPdPersonHat.y2[stPdPersonHat.s32Num] = pstResult[i1].fY2;
        stPdPersonHat.s32Num++;
    }
    
    for (i = 0; i < s32PNoHelmetNum; i++)
    {
        i1 = s32PersonNoHelmet[i];
        stPdPersonHat.x1[stPdPersonHat.s32Num] = pstResult[i1].fX1;
        stPdPersonHat.y1[stPdPersonHat.s32Num] = pstResult[i1].fY1;
        stPdPersonHat.x2[stPdPersonHat.s32Num] = pstResult[i1].fX2;
        stPdPersonHat.y2[stPdPersonHat.s32Num] = pstResult[i1].fY2;
        stPdPersonHat.s32Num++;
    }


    pfscore = malloc((s32PHatNum * s32HatNum + 1) * sizeof(float));
    for(i = 0; i < s32HatNum; i++)
    {
        for(j = 0; j < s32PHatNum; j++)
        {
            float fA, fB;
            i1 = s32Hat[i];
            i2 = s32PersonHat[j];

            fA = pd_SafetyHelmet_AFactor(pstResult[i1].fX1, pstResult[i1].fY1, pstResult[i1].fX2, pstResult[i1].fY2,
                                pstResult[i2].fX1, pstResult[i2].fY1, pstResult[i2].fX2, pstResult[i2].fY2);

            fB = pd_SafetyHelmet_BFactor(pstResult[i1].fX1, pstResult[i1].fY1, pstResult[i1].fX2, pstResult[i1].fY2,
                                pstResult[i2].fX1, pstResult[i2].fY1, pstResult[i2].fX2, pstResult[i2].fY2);

            if (fA <= 0 || fB <= 0)
            {

                fA = fB = 0;
            }
            pfscore[i*s32PHatNum + j] = (fA + fB) / 2;
        }
    }

    for(i = 0; i < s32HatNum; i++)
    {
        fBetterScore = pfscore[i*s32PHatNum];
        s32BetterIdx = 0;
        for(j = 0; j < s32PHatNum; j++)
        {
            if (pfscore[i*s32PHatNum+j] > fBetterScore)
            {
                fBetterScore = pfscore[i*s32PHatNum+j];
                s32BetterIdx = j;
            }
        }

        if (fBetterScore > threshold)
        {
            i1 = s32Hat[i];
            i2 = s32PersonHat[s32BetterIdx];

            stPdPersonHat.bHat[s32BetterIdx] = SV_TRUE;
            stPdPersonHat.hx1[s32BetterIdx] = pstResult[i1].fX1;
            stPdPersonHat.hy1[s32BetterIdx] = pstResult[i1].fY1;
            stPdPersonHat.hx2[s32BetterIdx] = pstResult[i1].fX2;
            stPdPersonHat.hy2[s32BetterIdx] = pstResult[i1].fY2;
            switch(pstResult[i1].classes)
            {
                case pdsa32::E_HAT_RED:
                    stPdPersonHat.color[s32BetterIdx] = HAT_RED;
                    pstResult[i2].classes = pdsa32::E_CLS_PERSON_HAT_RED;
                    break;
                case pdsa32::E_HAT_YELLOW:
                    stPdPersonHat.color[s32BetterIdx] = HAT_YELLOW;
                    pstResult[i2].classes = pdsa32::E_CLS_PERSON_HAT_YELLOW;
                    break;
                case pdsa32::E_HAT_WHITE:
                    stPdPersonHat.color[s32BetterIdx] = HAT_WHITE;
                    pstResult[i2].classes = pdsa32::E_CLS_PERSON_HAT_WHITE;
                    break;
                case pdsa32::E_HAT_BLUE:
                    stPdPersonHat.color[s32BetterIdx] = HAT_BLUE;
                    pstResult[i2].classes = pdsa32::E_CLS_PERSON_HAT_BLUE;
                    break;
                default:
                    stPdPersonHat.color[s32BetterIdx] = HAT_BUTT;
                    break;
            }

            /* 把其它得分全部清空 */
            for(i1 = 0; i1 < s32HatNum; i1++)
            {
                pfscore[i1*s32PHatNum+s32BetterIdx] = 0;
            }
        }
    }


    free(pfscore);
    return &stPdPersonHat;
}

SV_BOOL bNeedAlarm(pdsa32::EAlgObjectClass classes)
{
    PD_ROI_E enRoi = PD_ROI_BUTT;
    CFG_PDS_PARAM *pstPdsParam = &m_stPdInfo.stCfgParam.stAlgCh2.stPdsParam;
    SV_BOOL bNeed = SV_TRUE;

    if (pstPdsParam->enPdsModel != E_PDS_SH)
    {
        return SV_TRUE;
    }

    switch(classes)
    {
        case pdsa32::E_CLS_PERSON_HAT_RED:
            bNeed = pstPdsParam->stPdSafetyHelmet.bSkipRedHelmet == SV_TRUE ? SV_FALSE : SV_TRUE;
            break;
        case pdsa32::E_CLS_PERSON_HAT_YELLOW:
            bNeed = pstPdsParam->stPdSafetyHelmet.bSkipYellowHelmet == SV_TRUE ? SV_FALSE : SV_TRUE;
            break;
        case pdsa32::E_CLS_PERSON_HAT_WHITE:
            bNeed = pstPdsParam->stPdSafetyHelmet.bSkipWhiteHelmet == SV_TRUE ? SV_FALSE : SV_TRUE;
            break;
        case pdsa32::E_CLS_PERSON_HAT_BLUE:
            bNeed = pstPdsParam->stPdSafetyHelmet.bSkipBlueHelmet == SV_TRUE ? SV_FALSE : SV_TRUE;
            break;
        default:
            bNeed = SV_TRUE;
            break;
    }

    return bNeed;
}

/* 检测AlarmIn是否输入，来判断是否启动算法检测 */
SV_BOOL bAlarmInDetection(SV_BOOL bPdAlarmIn, TRIGGER_TYPE_S enTrigger)
{
    if (SV_FALSE == bPdAlarmIn)
    {
        return SV_TRUE;
    }

    sint32 s32Ret;
    uint8 u8IoIn1 = SV_TRUE, u8IoIn2 = SV_FALSE;
    SV_BOOL bHighLevelInput = SV_FALSE; /* 有高电平输入 */
    SV_BOOL bDetection = SV_TRUE;       /* 是否执行检测 */

    s32Ret = BOARD_RK_GetGPIO(PD_ALARM_IN_BIND, PD_ALARM_IN_PIN, &u8IoIn1);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_WARN, "BOARD_RK_GetGPIO fail.\n");
    }
    
    s32Ret = pd_GetMcuResults(u8IoIn2);
    if (SV_SUCCESS != s32Ret)
    {
        //print_level(SV_WARN, "BOARD_RK_GetGPIO fail.\n");
    }

    //u8IoIn1 IO口反向
    //print_level(SV_INFO, "get alarmin: %d, mcuAlarmIn: %d\n", u8IoIn1, u8IoIn2);
    /* 两个都没有高电平输入 */
    if(u8IoIn1 == SV_TRUE && u8IoIn2 == SV_FALSE) bHighLevelInput = SV_FALSE;
    else bHighLevelInput = SV_TRUE;
    
    if (TRIGGER_UP == enTrigger && bHighLevelInput == SV_FALSE) bDetection = SV_FALSE;
    if (TRIGGER_DOWN == enTrigger && bHighLevelInput == SV_TRUE) bDetection = SV_FALSE;

    return bDetection;
}


PD_ALARM_TYPE enRoiToAlarm(PD_ROI_E enRoi)
{
    switch(enRoi)
    {
        case PD_ROI_RED:
            return PD_ALARM_TYPE_RED;
        case PD_ROI_YELLOW:
            return PD_ALARM_TYPE_YELLOW;
        case PD_ROI_GREEN:
            return PD_ALARM_TYPE_GREEN;
        default:
            return PD_ALARM_TYPE_NULL;
    }
   // return PD_ALARM_TYPE_NULL; //去掉报警播放
}

int save_buffer(char *buffer, int size)
{
    static int i = 0;
    int fd;
    if(i ++ > 0)
        return 0;
    fd = open("/var/buffer", O_RDWR | O_CREAT);
    write(fd, buffer, size);
    close(fd);
    return 0;
}


sint32 pd_get_readIdx(sint32 s32Chn)
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

sint32 pd_release_readIdx(sint32 s32Chn, sint32 s32Idx)
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


/************************************************************
协议：（2bytes，0xffaa）    	 （1bytes）					（2bytes CRC校验除了同步头外的数据）
       同步头       +     数据长度   +    数据    +             CRC
************************************************************/


static const unsigned short crc16Table[256] = {
0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241,
0xc601, 0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440,
0xcc01, 0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40,
0x0a00, 0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841,
0xd801, 0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40,
0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41,
0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641,
0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040,
0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240,
0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441,
0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41,
0xfa01, 0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840,
0x2800, 0xe8c1, 0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41,
0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40,
0xe401, 0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640,
0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041,
0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240,
0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441,
0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41,
0xaa01, 0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840,
0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41,
0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40,
0xb401, 0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1, 0xb681, 0x7640,
0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041,
0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0, 0x5280, 0x9241,
0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440,
0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40,
0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841,
0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40,
0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41,
0x4400, 0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641,
0x8201, 0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040,
};

 uint32_t u32PrintTimeMs = 0;
 uint32_t u32RunCnt = 0;
 uint32_t u32PerDelayMs = 0;    
uint8 u8SerialData[1024] = {0};  
uint8 u8SerialData_Rev[100] = {0};

 Algo_Result stAlgResult[50] = {0};
 uint8 u8LenAlgData = 0;
 uint32_t s32WriteLen = 0;
 uint16 u16Crc16 = 0;

static uint16_t crc16_Gen(unsigned char *pu8Data, int u32Len)
{
	uint16_t uwCrc = 0, i;
    uint8_t ubByte;
    
    if (NULL == pu8Data || 0 == u32Len)
    {
        return 0;
    }

    for (i = 0; i < u32Len; i++)
    {
        ubByte = *pu8Data;
        uwCrc = (uwCrc >> 8) ^ crc16Table[(uwCrc ^ ubByte) & 0xff]; 
        pu8Data++;
    }

    return (((uwCrc >> 8) | (uwCrc << 8)) & 0xFFFF);
}

/* PD算法驱动线程 */

void * pd_alg_Body(void *pvArg)
{
    sint32 s32Ret = 0, i, j;
    sint32 s32CurChn = 0, s32Idx;
    sint32 s32AlarmMode = -1;
    SV_BOOL bPersonAlarm[3], bCarAlarm[3], bTmpAlarm[3];
    
    sint32 s32AlarmPoly = 0;
    uint32 u32RectCnt = 0;
    uint8  u8IoIn1 = 0, u8IoIn2 = 0;
    PD_ROI_E enRoi;
    PD_INFO_S *pstPdInfo = (PD_INFO_S *)pvArg;
    PD_DUMP_INFO_S stPdDumpInfo;
    uint32 u32StepTimeMs = 0;
    struct timespec tvLast = {0, 0};
    struct timespec tvNow = {0, 0};
    struct timespec tvBegin, tvEnd;
    sint32 s32level = 0;
    float  stRedSensitivity = 0.0;
    float  stYellowSensitivity = 0.0;
    float  stGreenSensitivity = 0.0;
    sint32 s32RedTimeMs[ALG_MAX_CHN] = {0}; // 检测倒计时(ms)
    sint32 s32YellowTimeMs[ALG_MAX_CHN] = {0};
    sint32 s32GreenTimeMs[ALG_MAX_CHN] = {0};
    pdsa32::STAlgInfo stPdResult = {0};
    MSG_PACKET_S stMsgPkt = {0};
    MEDIA_GUI_DRAW_S stMediaGuiDraw = {0};
    MEDIA_GUI_PERSON_S stGuiRect = {0};
    MEDIA_GUI_NULL_S stGuiNull;
    MSG_MCU_PERSON_DIS stMcuPersonDis = {0};
    PD_RESULT_S stPdResultNum = {0};
    CHN_ALG_E *apenChnAlg[ALG_MAX_CHN] = {0};
    CFG_PDS_PARAM *apstPdsParam[ALG_MAX_CHN] = {0};
    sint32 s32GpsStatus = 0, s32GpsSpeed = 0;     /* GPS状态, GPS速度 */
    PD_ALARM_NOTICE_S stAlarmNotic = {0};
    pdsa32::CPdsAlg *apcsPdsAlg = NULL;
    pdsa32::EAlgObjectClass eClass = pdsa32::E_CLS_RESERVE1;
    SV_BOOL bSpeedZone = SV_FALSE;
    SV_BOOL bEndSpeedZone = SV_FALSE;
    sint32 s32GreenRoiNum;              /* 绿色ROI区域检测数量 */
    sint32 s32YellowRoiNum;             /* 黄色ROI区域检测数量 */
    sint32 s32RedRoiNum;                /* 红色ROI区域检测数量 */
    sint32 s32TotalNum;                 /* 三个ROI区域总共的检测数量 */
    sint32 s32Total = 0;
    sint32 s32Cnt = 0;
    SV_BOOL bDetection = SV_FALSE;  /* 是否执行检测 或者对于200019客户是否检测车辆 */
    SV_BOOL bRunAlg = SV_TRUE;  /* 是否跑算法 */
    uint32 u32Width, u32Height;
    SV_BOOL bBlankClass = SV_FALSE;
    pdsa32::STPostParam stPostParam;
    stPostParam.stOverTakeParam.s32Delay = 5;

    

    uint16 u16mask;
#if 0
    void *apvBuf[4][3] = {NULL};
#else
	void *apvBuf[4] = {NULL};
#endif

    char apvBuf00[3][RINGBUF_SIZE];

    //uint32 u32BufLen = PD_IMAGE_BUF_SIZE;
    uint32 u32BufLen = m_stPdInfo.u32Width*m_stPdInfo.u32Height*3;
    uint32 u32IrBufLen = PD_IR_BUF_SIZE;
    uint32 u32Cnt = 0;
    PD_PERSON_HAT_S *pstPdPersonHat = NULL;


    s32Ret = prctl(PR_SET_NAME, "pd_body");
    if (0 != s32Ret)
    {
        print_level(SV_ERROR, "prctl PR_SET_NAME failed! [err:%s]\n", strerror(errno));
    }

    apenChnAlg[0] = &pstPdInfo->stCfgParam.stAlgCh1.enAlgType;
    apenChnAlg[1] = &pstPdInfo->stCfgParam.stAlgCh2.enAlgType;
    apenChnAlg[2] = &pstPdInfo->stCfgParam.stAlgCh3.enAlgType;
#if (defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1))    
    apstPdsParam[0] = &pstPdInfo->stCfgParam.stAlgCh1.stPdsParam;
    apstPdsParam[1] = &pstPdInfo->stCfgParam.stAlgCh2.stPdsParam;
    apstPdsParam[2] = &pstPdInfo->stCfgParam.stAlgCh3.stPdsParam;
#elif (defined(BOARD_ADA32IR))
    apstPdsParam[0] = &pstPdInfo->stCfgParam.stAlgCh2.stPdsParam;
    apstPdsParam[1] = &pstPdInfo->stCfgParam.stAlgCh1.stPdsParam;
#else
    apstPdsParam[0] = &pstPdInfo->stCfgParam.stAlgCh2.stPdsParam;
#endif
    for (i = 0; i < pstPdInfo->u32ChnNum; i++)
    {
#if (defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1))
        if (*(apenChnAlg[i]) != ALG_PDS)
        {
            continue;
        }
#endif

// #if 0
// /*******************************/
//         for(j = 0; j < 3; j++)
//         {
// #if (defined(BOARD_ADA32IR))
//             u32IrBufLen = (0 == i) ? u32BufLen : PD_IR_BUF_SIZE;
//             apvBuf[i][j] = mmap(NULL, u32IrBufLen, PROT_READ, MAP_SHARED, pstPdInfo->as32MediaBufFd[i][j], 0);
// #else
//             apvBuf[i][j] = mmap(NULL, u32BufLen, PROT_READ, MAP_SHARED, pstPdInfo->as32MediaBufFd[i][j], 0);
// #endif
//             if (MAP_FAILED == apvBuf[i][j])
//             {
//                 print_level(SV_ERROR, "mmap[%d] failed.\n", i);
//                 return NULL;
//             }
//         }
// /*******************************/
// #else
// /*******************************/
// #if (defined(BOARD_ADA32IR))
//         u32IrBufLen = (0 == i) ? u32BufLen : PD_IR_BUF_SIZE;
//         apvBuf[i] = mmap(NULL, u32IrBufLen, PROT_READ, MAP_SHARED, pstPdInfo->as32MediaBufFd[i], 0);
// #else
//         apvBuf[i] = mmap(NULL, u32BufLen, PROT_READ, MAP_SHARED, pstPdInfo->as32MediaBufFd[i], 0);
// #endif
//         if (MAP_FAILED == apvBuf[i])
//         {
//             print_level(SV_ERROR, "mmap[%d] failed.\n", i);
//             return NULL;
//         }
// /*******************************/
// #endif
    }

#if (defined(BOARD_ADA32V2) || defined(BOARD_ADA32V3) || defined(BOARD_ADA32IR) || defined(BOARD_ADA32C4))
    s32Ret = postModelListMessage(pstPdInfo->modelMessageList);
    if (s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "postModelListMessage failed! [err=0x%x]\n", s32Ret);
    }
#endif

    print_level(SV_INFO, "enter PD detection:%d.\n",pstPdInfo->bRunning);
    clock_gettime(CLOCK_MONOTONIC, &tvLast);
    clock_gettime(CLOCK_MONOTONIC, &tvBegin);

    sint32 last_alarm_mode = PD_ALARM_TYPE_NULL;

    static struct timespec tvNow_test = {0, 0};
    static struct timespec tvLast_test = {0, 0};
    uint32_t u32sum;
    uint32_t u32cnt;
    uint32_t u32StepTimeMs_test;

    //uint32_t testcnt[4] = {0};  

    while (pstPdInfo->bRunning)
    {

        // clock_gettime(CLOCK_MONOTONIC, &tvNow_test);
        // u32StepTimeMs_test = ((tvNow_test.tv_sec*1000 + tvNow_test.tv_nsec/1000000) - (tvLast_test.tv_sec*1000 + tvLast_test.tv_nsec/1000000));
        // tvLast_test = tvNow_test;

        // u32sum += u32StepTimeMs_test;
        // u32cnt ++;
        // if(u32cnt >99)
        // {
        //    printf("****delay ms__alg:%d***** \n",u32sum / u32cnt);
        //    u32cnt =  0;
        //    u32sum =  0;
        // }

        //四路平均性测试
        // u32cnt ++;
        // if(u32cnt >99)
        // {
        //    printf("****[0]:%d,[1]:%d,[2]:%d.[3]:%d***** \n",testcnt[0],testcnt[1],testcnt[2],testcnt[3]);
        //    testcnt[0] = 0;
        //    testcnt[1] = 0;
        //    testcnt[2] = 0;
        //    testcnt[3] = 0;
        //    u32cnt = 0;
        // }

        s32CurChn = 0;
        //print_level(SV_DEBUG, "pd_alg_Body running %d ...\n",s32CurChn);
        //sleep_ms(1000);
        clock_gettime(CLOCK_MONOTONIC, &tvNow);
        u32StepTimeMs = ((tvNow.tv_sec*1000 + tvNow.tv_nsec/1000000) - (tvLast.tv_sec*1000 + tvLast.tv_nsec/1000000));
        tvLast = tvNow;
        memset(&stPdDumpInfo, 0x00, sizeof(PD_DUMP_INFO_S));
        s32AlarmMode = 0;
        memset(bPersonAlarm, 0x00, sizeof(bPersonAlarm));
        memset(bCarAlarm,    0x00, sizeof(bCarAlarm));
        memset(bTmpAlarm,    0x00, sizeof(bTmpAlarm));
        memset(&stAlarmNotic, 0x00, sizeof(stAlarmNotic));
        stPdDumpInfo.s64TimeStamp = tvNow.tv_sec * 1000 + tvNow.tv_nsec /1000000;
        s32CurChn++;
        if (s32CurChn >= pstPdInfo->u32ChnNum)
        {
            s32CurChn = 0;
        }

        #if 1// Calculate the delay time per 100 times
        // s32Cnt++;
        // s32Total += u32StepTimeMs;
    //    if (s32Cnt >= 100)
    //    {
    //        print_level(SV_DEBUG, "pd_alg_Body delay: %d...\n", s32Total / s32Cnt);
    //         s32Cnt = 0;
    //         s32Total = 0;
    //    }
       // print_level(SV_DEBUG, "pd_alg_Body delay: %d...\n",stPdDumpInfo.s64TimeStamp);


        #endif

#if (defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1))
        // if (*(apenChnAlg[s32CurChn]) != ALG_PDS)
        // {
        //     continue;
        // }
#endif


#if (defined (BOARD_ADA32IR))
        if(BOARD_IsCustomer(BOARD_C_ADA32IR_100393))
        {
            sleep_ms(1000);
            continue;
        }
        //s32CurChn:0 可见光 1:红外光
        if (((0 == s32CurChn) && (MEDIA_SPLIT_IR_ONE == pstPdInfo->enSplitMode))
            || ((1 == s32CurChn) && (MEDIA_SPLIT_ONE == pstPdInfo->enSplitMode)))
        {
            for (i=0; i<pstPdInfo->u32ChnNum; i++)
            {
                if(apstPdsParam[i]->s32PdRedInterval >= 0 && s32RedTimeMs[i] > 0)
                {
                    s32RedTimeMs[i] -= u32StepTimeMs;
                }

                if(apstPdsParam[i]->s32PdYellowInterval >= 0 && s32YellowTimeMs[i] > 0)
                {
                    s32YellowTimeMs[i] -= u32StepTimeMs;
                }

                if(apstPdsParam[i]->s32PdGreenInterval >= 0 && s32GreenTimeMs[i] > 0)
                {
                    s32GreenTimeMs[i] -= u32StepTimeMs;
                }
            } 
            sleep_ms(5);
            continue;
        }
#endif
        memset(&stPdResult, 0x00, sizeof(pdsa32::STAlgInfo));
        memset(&stMediaGuiDraw, 0x00, sizeof(stMediaGuiDraw));
        memset(&stMcuPersonDis, 0x00, sizeof(stMcuPersonDis));
        memset(&stGuiRect, 0x00, sizeof(stGuiRect));
        u32RectCnt = 0;

#if 0
        /* 清空原来的画板 */
        u16mask = MEDIA_GUI_GET_MASK(s32CurChn, 0, MEDIA_GUI_OP_CLEAR);
        s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiNull);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
        }

        bRunAlg = SV_TRUE;
#if (!defined (BOARD_ADA32IR))
        bRunAlg &= (m_stPdInfo.stCfgParam.bAlgEnable == SV_TRUE);                        /* 算法开关 */
#else
        bRunAlg &= (
            ((s32CurChn == 0 && SV_TRUE == pstPdInfo->stCfgParam.bAlgEnable) 
            || (s32CurChn == 1 && SV_TRUE == pstPdInfo->stCfgParam.stAlgCh1.bAlgEnable))
        );
#endif
       
#if (!defined (BOARD_ADA32N1))
        // s32Ret = pd_GetGpsResults(s32GpsStatus, s32GpsSpeed);
        // if (SV_SUCCESS != s32Ret)
        // {
        //     print_level(SV_WARN, "dmm_GetGpsResults fail.\n");
        // }

        // if(s32GpsStatus == 3)
        // {
        //     if(s32GpsSpeed > apstPdsParam[s32CurChn]->s32PdWorkspeed_maxSpeed ||
        //        s32GpsSpeed < apstPdsParam[s32CurChn]->s32PdWorkspeed_minSpeed)
        //     {
        //         bRunAlg &= SV_FALSE;
        //     }
        // }
#endif

        bDetection = bAlarmInDetection(apstPdsParam[s32CurChn]->bPdAlarmIn, apstPdsParam[s32CurChn]->enPdAlarmInTrigger);
        if (BOARD_IsNotCustomer(BOARD_C_ADA32V2_200019))  bRunAlg &= bDetection;


        if (!bRunAlg)
        {
            pd_LED_watchdog_Setting(LED_RGB_FLASH);
            sleep_ms(100);
            goto skip_alg;
        }

        for (i=0; i<pstPdInfo->u32ChnNum; i++)
        {
            if(apstPdsParam[i]->s32PdRedInterval >= 0 && s32RedTimeMs[i] > 0)
            {
                s32RedTimeMs[i] -= u32StepTimeMs;
            }

            if(apstPdsParam[i]->s32PdYellowInterval >= 0 && s32YellowTimeMs[i] > 0)
            {
                s32YellowTimeMs[i] -= u32StepTimeMs;
            }

            if(apstPdsParam[i]->s32PdGreenInterval >= 0 && s32GreenTimeMs[i] > 0)
            {
                s32GreenTimeMs[i] -= u32StepTimeMs;
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
#if 0
        s32Idx = pd_get_readIdx(s32CurChn);
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

    #endif
        // if(pstPdInfo->bRotate && pstPdInfo->apcsPdsAlg_90 != NULL)
        // {
        //     u32Width = m_stPdInfo.u32Width_90;
        //     u32Height = m_stPdInfo.u32Height_90;
        //     apcsPdsAlg = pstPdInfo->apcsPdsAlg_90;
        // }

#if 0 // for degbug
        remove("/mnt/nfs/alg.rgb");
        FILE *fp = fopen("/mnt/nfs/alg.rgb", "wb+");
        if (NULL != fp)
        {   
            fwrite((char *)apvBuf[s32CurChn], 1, m_stPdInfo.u32Width*m_stPdInfo.u32Height*3, fp);
            fclose(fp);
        }
        print_level(SV_INFO, "write finish!\n");        
        MS_V(s32CurChn);
        exit(1);

#endif
        // MS_P(0);
        // u8BufferNum --;

        // printf(" ______ u8BufferNum :%d \n",u8BufferNum);


        u32Width  = PD_IMAGE_WIDTH;
        u32Height =  PD_IMAGE_HEIGHT;
        apcsPdsAlg = pstPdInfo->apcsPdsAlg[s32CurChn];

       // apvBuf[s32CurChn] = dstData;
#if 1
        int chn = 0;
        int32_t timeout = 500000; 
        int index = -1;

        if(TEST_RINGCOUNT > 0)
        TEST_RINGCOUNT --;

        index = ringque.Read(chn, timeout, [/*UtilizeResource*/&ringque](int &current){
            //cout << "    --- reader" << " --- " << GetMtimestamp() << " rpos:" << ringque.rpos_[0] << " index:" << current << " value:" << *ringbuf[current] << endl;
            //printf("    --- reader --- %llu, rpos: %d, index: %d, value: %#x\n", GetMtimestamp(), ringque.rpos_[0], current, (uint32_t)(*ringbuf[current]));
            // usleep(10000); // 10ms
             //sleep_ms(10);
            return current; //0;
        });
      //  memcpy( apvBuf00[index] , dstData[index], u32Width*u32Height*3);
#endif
        // remove("./alg.rgb");
        // FILE *fp = fopen("./alg.rgb", "wb+");
        // fwrite(apvBuf00[index], 1, 608*352*3, fp);
        // fclose(fp);

#if 0
        SOURCE_ID =  *((uint8 *)apvBuf[s32CurChn][s32Idx]) ; //帧的第一个字节是source ID    
        s32Ret = apcsPdsAlg->AlgForward((char *)apvBuf[s32CurChn][s32Idx], u32Width, u32Height, 0 , s32CurChn);
#else
       // SOURCE_ID =  *((uint8 *)apvBuf[s32CurChn]) ; //帧的第一个字节是source ID   

        SOURCE_ID      =  dstData[index][RINGBUF_SIZE-extern_num]; 
        DETECT_AREA_EN =  dstData[index][RINGBUF_SIZE-extern_num+1];
        DisplayMode    =  dstData[index][RINGBUF_SIZE-extern_num+2];

        s32Ret = apcsPdsAlg->AlgForward(dstData[index] , u32Width, u32Height, 0, 0);
#endif

        if (0 != s32Ret)
        {
            print_level(SV_ERROR, "ALGPDS_forward failed. [err=%d]\n", s32Ret);

// #if 0
//             pd_release_readIdx(s32CurChn, s32Idx);
// #else
//             MS_V(s32CurChn);
// #endif
            ALG_Calculate_unLock();
            sleep_ms(10);
            continue;
        }


// #if 0
// 		pd_release_readIdx(s32CurChn, s32Idx);
// #else
//         MS_V(s32CurChn);
// #endif
        ALG_Calculate_unLock();
        //clock_gettime(CLOCK_MONOTONIC, &tvEnd);
        //long long int eclipse_time = (tvEnd.tv_sec*1000 + tvEnd.tv_nsec/1000000) - (tvBegin.tv_sec*1000 + tvBegin.tv_nsec/1000000); // (ms)
        //s32Cnt++;
        //print_level(SV_INFO, "framerate:%lf(fps)\n", 1.0 * s32Cnt / eclipse_time * 1000);

        //apcsPdsAlg->AlgResult(&stPdResult, pdsa32::E_POST_TRACK, pstPdInfo->stTrackParam, s32CurChn);

        apcsPdsAlg->AlgResult(&stPdResult, pdsa32::E_POST_NONE, pstPdInfo->stTrackParam, s32CurChn);

        if (0 != s32Ret)
        {
            print_level(SV_ERROR, "AlgResult failed. [err=%d]\n", s32Ret);
            sleep_ms(10);
            continue;
        }

       // MS_V(1);
#if PD_OVERTAKE_ALARM
        // 行人靠近处理
        apcsPdsAlg->AlgPostProcess(&stPdResult, pdsa32::E_POST_OVER_ALARM, stPostParam, s32CurChn);
#endif

        pd_RectAntiShakeFilter(&stPdResult, s32CurChn, 0.1, 0.2);



        s32level = min(2, max(0, apstPdsParam[s32CurChn]->s32PdSensitivity));
        stRedSensitivity = apstPdsParam[s32CurChn]->astPdRedSensitivity[2 - s32level];
        stYellowSensitivity = apstPdsParam[s32CurChn]->astPdYellowSensitivity[2 - s32level];
        stGreenSensitivity = apstPdsParam[s32CurChn]->astPdGreenSensitivity[2 - s32level];
        
        /* 默认不使用车辆报警音频 */
        if(BOARD_IsNotCustomer(BOARD_C_ADA32V2_LUIS))
        {
            stAlarmNotic.bPerson = SV_TRUE;
        }
        else
        {
            stAlarmNotic.bPerson = SV_FALSE;
        }

        if (E_PDS_SH == pstPdInfo->stCfgParam.stAlgCh2.stPdsParam.enPdsModel)
        {
            pstPdPersonHat = pd_SafetyHelmet_Parse(stPdResult.stResults, stPdResult.u32Nums);
        }

        pd_LED_watchdog_Setting(LED_RGB_ON);

        memset(u8SerialData, 0, sizeof(u8SerialData));
        memset(stAlgResult, 0, sizeof(stAlgResult));
        u8LenAlgData = 0;

        u8SerialData[0] = 0xff;
        u8SerialData[1] = 0xaa;
        
        uint8 u8car_cnt = 0;
        uint8 u8person_cnt = 0;

        /* 将算法结果 stPdResult.u32Nums = 0 作为零发送 */
      //  printf("detect num[%d]:%d \n",SOURCE_ID ,stPdResult.u32Nums);

skip_alg:  
        for (i = 0; i < stPdResult.u32Nums; i++)
        { 
            ALG_RANGE_LIMIT(stPdResult.stResults[i].fX1, 0.0, 1.0);
            ALG_RANGE_LIMIT(stPdResult.stResults[i].fY1, 0.0, 1.0);
            ALG_RANGE_LIMIT(stPdResult.stResults[i].fX2, 0.0, 1.0);
            ALG_RANGE_LIMIT(stPdResult.stResults[i].fY2, 0.0, 1.0);


            Algo_Result_f stResults_f;
            /* 添加到串口数据中 */
            stResults_f.fx1 = stPdResult.stResults[i].fX1 ;
            stResults_f.fy1 = stPdResult.stResults[i].fY1 ;
            stResults_f.fx2 = stPdResult.stResults[i].fX2 ;
            stResults_f.fy2 = stPdResult.stResults[i].fY2 ;

            // stAlgResult.x1 = (short)(stPdResult_t.fx1 * 1024);
            // stAlgResult.y1 = (short)(stPdResult_t.fy1 * 600);
            // stAlgResult.x2 = (short)(stPdResult_t.fx2 * 1024);
            // stAlgResult.y2 = (short)(stPdResult_t.fy2 * 600);

            // s32AlarmMode = area_detect(stAlgResult[i],Area_Info_Get); //获取报警程度
            // stAlgResult[u8LenAlgData].alarmType = s32AlarmMode;

            coordinate_remap(&stAlgResult[u8LenAlgData],stResults_f,DisplayMode,1024,600);

            if (stAlgResult[u8LenAlgData].x1 < 20)
            {
            	stAlgResult[u8LenAlgData].x1 = 20;
            }
            if (stAlgResult[u8LenAlgData].y1 < 20)
            {
            	stAlgResult[u8LenAlgData].y1 = 20;
            }
            
            
            if (stAlgResult[u8LenAlgData].x2 > 1024-20)
            {
            	stAlgResult[u8LenAlgData].x2 = 1024-20;
            }
            if (stAlgResult[u8LenAlgData].y2 > 600-20)
            {
            	stAlgResult[u8LenAlgData].y2 = 600-20;
            }

            // if ((stAlgResult[u8LenAlgData].x2 - stAlgResult[u8LenAlgData].x1 < 30) || (stAlgResult[u8LenAlgData].y2 - stAlgResult[u8LenAlgData].y1 < 30))
            // {
            //     s32AlarmMode = PD_ALARM_TYPE_NULL;
            // 	continue;
            // }
            
            // if ((stAlgResult[u8LenAlgData].x1 > 1024) || (stAlgResult[u8LenAlgData].y1 > 600))
            // {
            //     s32AlarmMode = PD_ALARM_TYPE_NULL;
            // 	continue;
            // }

            u8LenAlgData++;
        /*
            PD_ALARM_TYPE_RED;
            PD_ALARM_TYPE_YELLOW;
            PD_ALARM_TYPE_GREEN;
            PD_ALARM_TYPE_NULL;
        */

        //如果把梯形检测区域关掉了，则返回全部

        if(DETECT_AREA_EN)
        {
            if(s32AlarmMode == PD_ALARM_TYPE_NULL)
            {
                u8LenAlgData --;
                continue;
            }
        }


           if(last_alarm_mode < s32AlarmMode)
           {
                last_alarm_mode = s32AlarmMode; //获取报警最严重的
           }

/************sh: can't create /sys/class/gpio_stonkam/RGB/wtd: nonexistent directory
sh: can't create /sys/class/gpio_stonkam/alarmout/wtd: nonexistent directory
sh: can't create /sys/class/gpio_stonkam/RGB/rgbstate: nonexistent directory***********/
         last_alarm_mode =   PD_ALARM_TYPE_NULL;
         s32AlarmMode = PD_ALARM_TYPE_NULL;

            if(stPdResult.stResults[i].classes == pdsa32::E_CLS_PERSON)
                    u8person_cnt++;

            if(stPdResult.stResults[i].classes == pdsa32::E_CLS_CAR)
                    u8car_cnt++;

        

            /* SkipPerson 使能, 则跳过行人检测 */
            if(pstPdInfo->bSkipPerson[s32CurChn] && stPdResult.stResults[i].classes == pdsa32::E_CLS_PERSON)
            {
                continue;
            }
            
            if(pstPdInfo->bSkipCar[s32CurChn] && stPdResult.stResults[i].classes == pdsa32::E_CLS_CAR)
            {
                continue;
            }

            if(!bDetection && stPdResult.stResults[i].classes == pdsa32::E_CLS_CAR)
            {
                continue;
            }
#if PD_OVERTAKE_ALARM
            if(!stPdResult.stResults[i].bOverTake)
            {
                continue;
            }
#endif


            //print_level(SV_DEBUG, "enRoi:%d, classes: %d, confidence:%f, (%f,%f) (%f,%f)\n", enRoi, stPdResult.stResults[i].classes, stPdResult.stResults[i].fConfidence, stPdResult.stResults[i].fX1, stPdResult.stResults[i].fY1, stPdResult.stResults[i].fX2, stPdResult.stResults[i].fY2);

            if(u32RectCnt >= 20)
            {
                break;
            }

            bBlankClass = SV_FALSE;
            pdsa32::EAlgObjectClass aclass = stPdResult.stResults[i].classes;
            switch (enRoi)
            {
                case PD_ROI_RED:
                    stPdDumpInfo.s32RedRoiNum++;
                    stGuiRect.astPersonsRect[u32RectCnt].color = GUI_COLOR_RED;
                    if (bNeedAlarm(aclass)) bTmpAlarm[enRoi] = SV_TRUE;
                    break;
                case PD_ROI_YELLOW:
                    stPdDumpInfo.s32YellowRoiNum++;
                    stGuiRect.astPersonsRect[u32RectCnt].color = GUI_COLOR_L_YELLOW;
                    if (bNeedAlarm(aclass)) bTmpAlarm[enRoi] = SV_TRUE;
                    break;
                case PD_ROI_GREEN:
                    stPdDumpInfo.s32GreenRoiNum++;
                    stGuiRect.astPersonsRect[u32RectCnt].color = GUI_COLOR_GREEN;
                    if (bNeedAlarm(aclass)) bTmpAlarm[enRoi] = SV_TRUE;
                    break;
                case PD_ROI_BLUE:
                    bBlankClass = SV_TRUE;
                    stGuiRect.astPersonsRect[u32RectCnt].color = GUI_COLOR_L_BLUE;
                    break;
                case PD_ROI_BUTT:
                    bBlankClass = SV_TRUE;
                    if(BOARD_IsCustomer(BOARD_C_ADA32V2_LUIS))
                    {
                        stGuiRect.astPersonsRect[u32RectCnt].color = GUI_COLOR_GREEN;
                    }
                    break;
            }
            
            
            u32RectCnt++;
        }

        s32AlarmMode = last_alarm_mode;

        stGuiRect.u32PersonNum = u32RectCnt;

//串口组包
#if 1
        memcpy(&u8SerialData[2], &SOURCE_ID, 1);
        memcpy(&u8SerialData[3], &u8LenAlgData, 1);
        memcpy(&u8SerialData[4], &stAlgResult[0], sizeof(Algo_Result) * u8LenAlgData);
        //u16Crc16 = crc16_Gen(&u8SerialData[2], sizeof(Algo_Result) * u8LenAlgData + 1);
        u16Crc16 = crc16_Gen(&u8SerialData[2], sizeof(Algo_Result) * u8LenAlgData + 2); //加一位表示当前通道

        //memcpy(&u8SerialData[sizeof(Algo_Result) * u8LenAlgData + 3], &u16Crc16, 2);
        memcpy(&u8SerialData[sizeof(Algo_Result) * u8LenAlgData + 4], &u16Crc16, 2);  //向后移一位

        if(SOURCE_ID < 4)
        s32WriteLen = write(m_stPdInfo.s32SerialFd[0], u8SerialData, sizeof(Algo_Result) * u8LenAlgData + 6);
#else
        uint8 pc_flag = 0;
        memcpy(&u8SerialData[2], &SOURCE_ID, 1);

        if(u8person_cnt > 0 )
            pc_flag += 1;
        if(u8car_cnt > 0)
            pc_flag += 2;

        memcpy(&u8SerialData[3], &pc_flag, 1);
        memcpy(&u8SerialData[4], &u8LenAlgData, 1);
        memcpy(&u8SerialData[5], &stAlgResult[0], sizeof(Algo_Result) * u8LenAlgData);
        u16Crc16 = crc16_Gen(&u8SerialData[2], sizeof(Algo_Result) * u8LenAlgData + 3); //加一位表示当前通道
        memcpy(&u8SerialData[sizeof(Algo_Result) * u8LenAlgData + 5], &u16Crc16, 2);  //向后移一位
        if(SOURCE_ID < 4)
        s32WriteLen = write(m_stPdInfo.s32SerialFd[0], u8SerialData, sizeof(Algo_Result) * u8LenAlgData + 7);     
#endif

        s32time_out = 0;

        last_alarm_mode = PD_ALARM_TYPE_NULL;

        clock_gettime(CLOCK_MONOTONIC, &tvLast);

        // printf("send:");
        // for(int i = 0; i< (sizeof(Algo_Result) * u8LenAlgData + 6) ;i++)
        // {
        //     printf("%02x",u8SerialData[i]);
        // }
        // printf("\n");

#if 0
        if (BOARD_IsCustomer(BOARD_C_ADA32V2_201165))
        {               
            pd_SpeedZoneIsClassParse(bSpeedZone, bEndSpeedZone);
            bSpeedZone = SV_FALSE;
            bEndSpeedZone = SV_FALSE;    
        }

        /* 如果未使能行人矩形框则只发送左上角一个点 */
        if (stGuiRect.u32PersonNum != 0 && !apstPdsParam[s32CurChn]->bRectPerson)
        {
            stGuiRect.u32PersonNum = 1;
            stGuiRect.astPersonsRect[0].x1 = 0;
            stGuiRect.astPersonsRect[0].y1 = 0;
            stGuiRect.astPersonsRect[0].x2 = 0.0005;
            stGuiRect.astPersonsRect[0].y2 = 0.0009;
            stGuiRect.astPersonsRect[0].color = GUI_COLOR_L_BLUE;
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
        //stMsgPkt.u32Size = sizeof(stMediaGuiDraw);
        stMsgPkt.u32Size = MEDIA_GUI_SIZE(stMediaGuiDraw);
        
        s32Ret = Msg_submitEvent(EP_CONTROL, OP_EVENT_MEDIA_GUI, &stMsgPkt);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
        }

        if(stMcuPersonDis.s32Num > 0)
        {
            memset(&stMsgPkt, 0, sizeof(stMsgPkt));
            stMsgPkt.pu8Data = (uint8*)&stMcuPersonDis;
            stMsgPkt.u32Size = sizeof(stMcuPersonDis);
            s32Ret = Msg_submitEvent(EP_MCU, OP_EVENT_PD_PERSON_DIS, &stMsgPkt);
            if (SV_SUCCESS != s32Ret)
            {
                print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
            }

            s32Ret = Msg_submitEvent(EP_CAN, OP_EVENT_PD_PERSON_DIS, &stMsgPkt);
            if (SV_SUCCESS != s32Ret)
            {
                print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
            }
        }

        #endif

        s32TotalNum = stPdDumpInfo.s32GreenRoiNum + stPdDumpInfo.s32YellowRoiNum + stPdDumpInfo.s32RedRoiNum;
        pd_DumpInfo_200055A_Correct(&stPdDumpInfo, s32TotalNum);
        s32RedRoiNum = stPdDumpInfo.s32RedRoiNum;
        s32YellowRoiNum = stPdDumpInfo.s32YellowRoiNum;
        s32GreenRoiNum = stPdDumpInfo.s32GreenRoiNum;
        /* dump 给MCU的信息 */
        if(apstPdsParam[s32CurChn]->bPdAlarmOutRed == SV_FALSE || apstPdsParam[s32CurChn]->s32PdRedInterval < 0 || s32RedTimeMs[s32CurChn] > 0)
        {
            stPdDumpInfo.s32RedRoiNum = 0;
        }

        if(apstPdsParam[s32CurChn]->bPdAlarmOutYellow == SV_FALSE || apstPdsParam[s32CurChn]->s32PdYellowInterval < 0 || s32YellowTimeMs[s32CurChn] > 0)
        {
            stPdDumpInfo.s32YellowRoiNum = 0;
        }

        if(apstPdsParam[s32CurChn]->bPdAlarmOutGreen == SV_FALSE || apstPdsParam[s32CurChn]->s32PdGreenInterval < 0 || s32GreenTimeMs[s32CurChn] > 0)
        {
            stPdDumpInfo.s32GreenRoiNum = 0;
        }

        pd_DumpInfo(s32CurChn, &stPdDumpInfo);
        
        stAlarmNotic.s32Chn = s32CurChn;
        stAlarmNotic.enSplit = pstPdInfo->enSplitMode;

        // if(BOARD_IsCustomer(BOARD_C_ADA32V2_EXHIBITION_A) ||
        //    BOARD_IsCustomer(BOARD_C_ADA32V2_EXHIBITION_B) ||
        //    BOARD_IsCustomer(BOARD_C_ADA32V2_EXHIBITION_C) ||
        //    BOARD_IsCustomer(BOARD_C_ADA32V2_EXHIBITION_D))
        // {
        //     s32AlarmMode = s32AlarmMode == 3 ? 3 : 0;
        //     bCarAlarm[PD_ROI_YELLOW] = bCarAlarm[PD_ROI_GREEN] = SV_FALSE;
        //     bPersonAlarm[PD_ROI_YELLOW] = bPersonAlarm[PD_ROI_GREEN] = SV_FALSE;
        // }


        /* 更新报警模式 */
        // for(i = PD_ROI_RED; i < PD_ROI_BLUE; i++)
        // {
        //     enRoi = i;
        //     if (!bCarAlarm[enRoi] && !bPersonAlarm[enRoi])
        //         continue;

        //     /* ROI区域优先，行人报警优先 */
        //     if (bPersonAlarm[enRoi])
        //         stAlarmNotic.bPerson = SV_TRUE;
        //     else
        //         stAlarmNotic.bCar = SV_TRUE;

        //     s32AlarmMode = enRoiToAlarm(enRoi);
        //     break;
        // }


        /* 根据报警间隔清空数据 */
        s32GreenRoiNum = s32GreenTimeMs[s32CurChn]   <= 0 ? s32GreenRoiNum :  0;
        s32YellowRoiNum = s32YellowTimeMs[s32CurChn] <= 0 ? s32YellowRoiNum : 0;
        s32RedRoiNum = s32RedTimeMs[s32CurChn]       <= 0 ? s32RedRoiNum :    0;

        SV_BOOL bSendEvent = SV_FALSE;
        switch (s32AlarmMode)
        {
            case PD_ALARM_TYPE_RED:
                stAlarmNotic.enMode = s32AlarmMode;
                if(apstPdsParam[s32CurChn]->s32PdRedInterval < 0 || s32RedTimeMs[s32CurChn] > 0)
                {
                    stAlarmNotic.enMode = 0;
                    break;
                }
                s32RedTimeMs[s32CurChn] = apstPdsParam[s32CurChn]->s32PdRedInterval * 1000;
                if (apstPdsParam[s32CurChn]->s32PdRedInterval > 0) bSendEvent = SV_TRUE;
                break;
            case PD_ALARM_TYPE_YELLOW:
                stAlarmNotic.enMode = s32AlarmMode;
                if (apstPdsParam[s32CurChn]->s32PdYellowInterval < 0 || s32YellowTimeMs[s32CurChn] > 0)
                {
                    stAlarmNotic.enMode = 0;
                    break;
                }
                s32YellowTimeMs[s32CurChn] = apstPdsParam[s32CurChn]->s32PdYellowInterval * 1000;
                if (apstPdsParam[s32CurChn]->s32PdYellowInterval > 0) bSendEvent = SV_TRUE;
                
                if (s32RedTimeMs[s32CurChn] > 0) stAlarmNotic.enMode = 0;   // 红色报警的间隔优先
                break;
            case PD_ALARM_TYPE_GREEN:
                stAlarmNotic.enMode = s32AlarmMode;
                if(apstPdsParam[s32CurChn]->s32PdGreenInterval < 0 || s32GreenTimeMs[s32CurChn] > 0)
                {
                    stAlarmNotic.enMode = 0;
                    break;
                }
                s32GreenTimeMs[s32CurChn] = apstPdsParam[s32CurChn]->s32PdGreenInterval * 1000;
                if (apstPdsParam[s32CurChn]->s32PdGreenInterval > 0) bSendEvent = SV_TRUE;
                
                if (s32RedTimeMs[s32CurChn] > 0) stAlarmNotic.enMode = 0;       // 红色报警的间隔优先
                if (s32YellowTimeMs[s32CurChn] > 0) stAlarmNotic.enMode = 0;    // 黄色报警的间隔优先
                break;
            case PD_ALARM_TYPE_BUTT:
            default:
                stAlarmNotic.enMode = 0;
                break;
        }
        pd_Alarm_Post(stAlarmNotic);

        // if (bSendEvent)
        // {
        //     stPdResultNum.s64TimeStamp = stPdDumpInfo.s64TimeStamp;
        //     stPdResultNum.s32RedRoiNum = stPdDumpInfo.s32RedRoiNum;
        //     stPdResultNum.s32YellowRoiNum = s32YellowRoiNum;
        //     stPdResultNum.s32GreenRoiNum = s32GreenRoiNum;
        //     memset(&stMsgPkt, 0, sizeof(stMsgPkt));
        //     stMsgPkt.pu8Data = (uint8*)&stPdResultNum;
        //     stMsgPkt.u32Size = sizeof(stPdResultNum);
            
            
            
        //     s32Ret = Msg_submitEvent(EP_MCU, OP_EVENT_PD_PERSON_NUM, &stMsgPkt);
        //     if (SV_SUCCESS != s32Ret)
        //     {
        //         print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
        //     }
            
        //     s32Ret = Msg_submitEvent(EP_CAN, OP_EVENT_PD_PERSON_NUM, &stMsgPkt);
        //     if (SV_SUCCESS != s32Ret)
        //     {
        //         print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
        //     }
        // }
    
    }
    for (i = 0; i < pstPdInfo->u32ChnNum; i++)
    {
// #if (defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1))
//         if (*(apenChnAlg[s32CurChn]) != ALG_PDS)
//         {
//             continue;
//         }
// #endif
#if 0
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

sint32 pd_PostShelterResult()
{
    sint32 s32Ret = 0;
    uint16 u16mask;
    MSG_PACKET_S stMsgPkt = {0};
    MEDIA_GUI_DRAW_S stMediaGuiDraw = {0};
    MEDIA_GUI_NULL_S stGuiNull;
    MEDIA_GUI_ALARM_DMM_S stGuiAlarmDmm;
    PD_ALARM_NOTICE_S stPdAlarmNotice;
    const int sleeptime = 0;

    stPdAlarmNotice.enMode = PD_ALARM_TYPE_RED; // 遮挡触发沿用报警

    stGuiAlarmDmm.contime = sleeptime + 1;
    stGuiAlarmDmm.enAlarmLevel = ALARM_LEVEL_HIGH;
    stGuiAlarmDmm.enAlarmType = ALARM_SHELTER;

    memset(&stMsgPkt, 0, sizeof(stMsgPkt));
    stMsgPkt.pu8Data = (uint8*)&stMediaGuiDraw;
    stMsgPkt.u32Size = sizeof(MEDIA_GUI_DRAW_S);
    u16mask = MEDIA_GUI_GET_MASK(0, MEDIA_GUI_PLANE_EXTERN1, MEDIA_GUI_OP_CLEAR);
    s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiNull);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
    }

    u16mask = MEDIA_GUI_GET_MASK(0, MEDIA_GUI_PLANE_EXTERN1, MEDIA_GUI_OP_ALARM_DMM);
    s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiAlarmDmm);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
    }

    s32Ret = Msg_submitEvent(EP_CONTROL, OP_EVENT_MEDIA_GUI, &stMsgPkt);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
    }

    s32Ret = pd_Shelter_Post(stPdAlarmNotice);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "pd_Alarm_Out_Post failed. [err=%#x]\n", s32Ret);
    }

    //sleep(sleeptime);   /* 休眠3秒钟 */

    return SV_SUCCESS;
}

int *loaddata()
{
    FILE *fp;
    char StrLine[1024];
    static int *p = malloc(sizeof(int)*7797*2);
    int n;
    int x, y;
    
    n = 0;
    fp = fopen("/var/point_deal.txt", "r");
    
    while(!feof(fp))
    {
        fgets(StrLine, 1024, fp);
        sscanf(StrLine, "[%d, %d]", &x, &y);
        p[n++] = x;
        p[n++] = y;
    }
    return p;
}

/* 绘制10万个点 */
sint32 pd_Post_1MillionPoint()
{
    sint32 s32Ret = 0;
    uint16 u16mask;
    MEDIA_GUI_DRAW_S stMediaGuiDraw;
    MEDIA_GUI_POINT_S stPoint = {0};
    MEDIA_GUI_NULL_S stGuiNull;
    int i = 0, j = 0;
    MSG_PACKET_S stMsgPkt = {0};
    struct timeval stTm1;
    struct timeval stTm2;
    int real_sleep;
    static int *pointdata = loaddata();


    memset(&stMsgPkt, 0, sizeof(stMsgPkt));
    stMsgPkt.pu8Data = (uint8*)&stMediaGuiDraw;
    stMsgPkt.u32Size = sizeof(MEDIA_GUI_DRAW_S);

    memset(&stMediaGuiDraw, 0x00, sizeof(stMediaGuiDraw));
    u16mask = MEDIA_GUI_GET_MASK(0, MEDIA_GUI_PLANE_EXTERN1, MEDIA_GUI_OP_CLEAR);
    s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiNull);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
    }

    gettimeofday(&stTm1, NULL);

    u16mask = MEDIA_GUI_GET_MASK(0, MEDIA_GUI_PLANE_EXTERN1, MEDIA_GUI_OP_DRAW_POINT);
    for(i = 0; i < 7797; i++)
    {
        stPoint.x = (1.0 * pointdata[2*i] / 960);
        stPoint.y = (1.0 * pointdata[2*i+1] / 608);
        s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stPoint);
        
        if (SV_SUCCESS != s32Ret)   /* 说明buffer已经满了 */
        {
            /* 提交事件 */
            print_level(SV_INFO, "(%d %d) Msg_submitEvent\n", i, j);
            s32Ret = Msg_submitEvent(EP_CONTROL, OP_EVENT_MEDIA_GUI, &stMsgPkt);
            if (SV_SUCCESS != s32Ret)
            {
                print_level(SV_ERROR, "Msg_submitEvent failed(i%d j%d). [err=%#x]\n", i, j, s32Ret);
            }
        
            /* 重置数据 */
            memset(&stMediaGuiDraw, 0x00, sizeof(stMediaGuiDraw));  /* 重置数据 */
            i--;   /* 由于i填充失败,因此回退 */
            //usleep(10*1000);
        }
    }

    /* 剩余事件提交 */
    s32Ret = Msg_submitEvent(EP_CONTROL, OP_EVENT_MEDIA_GUI, &stMsgPkt);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
    }
    gettimeofday(&stTm2, NULL);

    real_sleep = (stTm2.tv_sec - stTm1.tv_sec) * 1000 + (stTm2.tv_usec - stTm1.tv_usec) / 1000;
    print_level(SV_INFO, "real_sleep:%d\n", real_sleep);

    return SV_SUCCESS;
}

sint32 pd_tool_occlusion_Body(void *pvArg)
{
    PD_INFO_S *pstPdInfo = (PD_INFO_S *)pvArg;
    sint32 s32Ret;

    if (!pstPdInfo->stCfgParam.stAlgCh2.stPdsParam.bshelterEnable)
    {
        return SV_SUCCESS;
    }

    s32Ret = system("occlusion_detect /var/snap/snap0.jpeg 2.5 2>1 > /dev/null");
    s32Ret = (s32Ret >> 8); /* 高8位才是命令的返回值,低8位是系统调用的返回值 */
    if (s32Ret != SV_TRUE)
    {
        return SV_SUCCESS;
    }

    s32Ret = pd_PostShelterResult();
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "pd_PostShelterResult failed. [err=%#x]\n", s32Ret);
    }

    return SV_SUCCESS;
}

sint32 pd_tool_Init(void *pvArg)
{
    PD_INFO_S *pstPdInfo = (PD_INFO_S *)pvArg;
    sint32 s32Ret;
    if (!pstPdInfo->stCfgParam.stAlgCh2.stPdsParam.bshelterEnable)
    {
        return SV_SUCCESS;
    }

    s32Ret = ALG_Hide_Picstream();
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "ALG_Hide_Picstream failed! [err=%d]\n", s32Ret);
    }

    return SV_SUCCESS;
}

typedef struct tagpdtoolstruct
{
    sint32 (*pd_tool_func)(void*);  /* 执行函数 */
    sint32 sleep_time;              /* 休眠时间 */
} PD_TOOL_STRCUT;

/* 工具线程,包括遮挡检测，二维码检测等功能 */
void pd_tool_Body(void *pvArg)
{
    sint32 s32Ret, i;
    PD_INFO_S *pstPdInfo = (PD_INFO_S *)pvArg;
    struct timeval stTm1;
    struct timeval stTm2;
    uint32 sleep_count = 0;
    const int cycle_time = 100;  // 休眠100毫秒
    int real_sleep;

    const PD_TOOL_STRCUT stToolList[] = {
        {.pd_tool_func = pd_tool_occlusion_Body, .sleep_time = 1000}
    };


    s32Ret = pd_tool_Init(pstPdInfo);
    if (s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "pd_tool_Init failed! [err=%d]\n", s32Ret);
    }

    while (pstPdInfo->bRunning)
    {
        gettimeofday(&stTm1, NULL);

        for(i = 0; i < sizeof(stToolList) / sizeof(stToolList[0]); i++)
        {
            if(stToolList[i].pd_tool_func != NULL && sleep_count % stToolList[i].sleep_time == 0)
            {
                s32Ret = stToolList[i].pd_tool_func(pvArg);
                if(s32Ret != SV_SUCCESS)
                {
                    print_level(SV_ERROR, "stToolList[%d].pd_tool_func failed! [err=%d]\n", i, s32Ret);
                    continue;
                }
            }
        }
        
        gettimeofday(&stTm2, NULL);
        real_sleep = (stTm2.tv_sec - stTm1.tv_sec) * 1000 + (stTm2.tv_usec - stTm1.tv_usec) / 1000;
        real_sleep = (real_sleep < cycle_time ? cycle_time - real_sleep : 0);   // 防止为负数
        real_sleep = (real_sleep < 3000 ? real_sleep : 3000);                   // 防止休眠时间过长
        sleep_ms(real_sleep);
    }
}


void * pd_Test_Body(void *pvArg)
{
    sint32 s32Ret;
    MSG_CAN_DATA_S stMsgCanData = {0};
    PD_INFO_S *pstPdInfo = (PD_INFO_S *)pvArg;

    char szTestData1[128] = "hello!";
    char szTestData2[128] = "hello, haungabcdefghijk!";
    char szTestData3[MCU_CAN_VALID_DATA_LEN] = {0};
    char szTestData4[MCU_CAN_VALID_DATA_LEN+1] = {0};
    
    for (int i = 0; i < MCU_CAN_VALID_DATA_LEN; i++)
        szTestData3[i] = 'a';

    for (int i = 0; i < MCU_CAN_VALID_DATA_LEN+1; i++)
        szTestData4[i] = 0x61;

    while (pstPdInfo->bRunning)
    {
        sleep_ms(1000);
        printf("send data1, str len: %d\n", strlen(szTestData1));
        memset(&stMsgCanData, 0, sizeof(stMsgCanData));
        stMsgCanData.bSingleFrame = SV_TRUE;
        stMsgCanData.s32DataLen = strlen(szTestData1);
        strncpy(stMsgCanData.szCanData, szTestData1, stMsgCanData.s32DataLen);
        s32Ret = PD_SendCanData(&stMsgCanData);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
        }
        
        sleep_ms(1000);
        printf("send data2, str len: %d\n", strlen(szTestData2));
        memset(&stMsgCanData, 0, sizeof(stMsgCanData));
        stMsgCanData.bSingleFrame = SV_FALSE;
        stMsgCanData.s32DataLen = strlen(szTestData2);
        strncpy(stMsgCanData.szCanData, szTestData2, stMsgCanData.s32DataLen);
        s32Ret = PD_SendCanData(&stMsgCanData);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
        }

        sleep_ms(1000);
        printf("send data3, str len: %d\n", strlen(szTestData3));
        memset(&stMsgCanData, 0, sizeof(stMsgCanData));
        stMsgCanData.bSingleFrame = SV_FALSE;
        stMsgCanData.s32DataLen = MCU_CAN_VALID_DATA_LEN;
        strncpy(stMsgCanData.szCanData, szTestData3, stMsgCanData.s32DataLen);
        s32Ret = PD_SendCanData(&stMsgCanData);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
        }

        sleep_ms(1000);
        printf("send data4, str len: %d\n", strlen(szTestData4));
        memset(&stMsgCanData, 0, sizeof(stMsgCanData));
        stMsgCanData.bSingleFrame = SV_FALSE;
        stMsgCanData.s32DataLen = MCU_CAN_VALID_DATA_LEN+1;
        strncpy(stMsgCanData.szCanData, szTestData4, stMsgCanData.s32DataLen);
        s32Ret = PD_SendCanData(&stMsgCanData);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
        }
    }

    return NULL;
}

sint32 pd_CheckCallback(uint8_t* au8SourceAddr, uint8_t* au8ResultAddr)
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

    m_stPdInfo.bKeyAuth = stRetPkt.stMsg.s32Param;
    memcpy(au8ResultAddr, stAuthDes.au8Buf, 16);
    print_level(SV_DEBUG, "bKeyAuth:%d\n", m_stPdInfo.bKeyAuth);
    
    return 0;
}


sint32 PD_Init(PD_CFG_PARAM_S *pstInitParam)
{
    sint32 s32Ret = 0, i, j;
    sint32 s32CenterGaze = 0;
    float afThresholds[4];
    uint32 u32ThrNum = 4;
    SV_BOOL bValidFd = SV_FALSE;
    CHN_ALG_E *apenChnAlg[ALG_MAX_CHN] = {0};
    CFG_PDS_PARAM *apstPdsParam[ALG_MAX_CHN] = {0};
    STCaliParams stCaliParams = {0};
    char *pszModelFile = NULL;
    char *pszModelFileList[3];
    pdsa32::EAlgType enAlgType = pdsa32::E_PDSALG_TYPE_RGB_P;
    pdsa32::CPdsAlg *apcsPdsAlgRgbP = NULL;
    pdsa32::CPdsAlg *apcsPdsAlgRgbPC = NULL;
    pdsa32::CPdsAlg *apcsPdsAlgRgbPC_90 = NULL;
    pdsa32::CPdsAlg *apcsPdsAlgRgbPOW = NULL;
    pdsa32::CPdsAlg *apcsPdsAlgRgbPCOW = NULL;
    pdsa32::CPdsAlg *apcsPdsAlgRgbIrPC = NULL;
    pdsa32::CPdsAlg *apcsPdsAlgRgbMANHOLE = NULL;
    pdsa32::CPdsAlg *apcsPdsAlgRgbBEAR = NULL;
    pdsa32::CPdsAlg *apcsPdsAlgRgbNIrPC = NULL;
    pdsa32::CPdsAlg *apcsPdsAlgRgbSH = NULL;
    pdsa32::CPdsAlg *apcsPdsAlgRgbSZ = NULL;
    pdsa32::CPdsAlg *apcsPdsAlgRgbSSR = NULL;


    s32Ret = MS_Init();
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "MS_Init failed. [err=%#x]\n", s32Ret);
        return ERR_SYS_NOTREADY;
    }
    pthread_attr_t 	attr;
    printf("**************************\n pd init\n ******************************\n");

   uint32 u32Tid_v4l2 = 0;
   uint32 u32Tid_monitor = 0;
   uint32 u32Tid_vdec = 0; 
   s32Ret = mpp_vpss_InitAlgChn();   //打开264的句柄  m_stVpssAlgFrame.s32AlgChnFd

    if (SV_SUCCESS != s32Ret)
    {
        printf("video node open failed\n");
        for(int i = 0 ; i <30  ; i++)  //3s后重启
            sleep_ms(100);
        system("pkill -9  alg");
    }
    else
    {

    // if(0 == access("/root/err_exit", F_OK) )
    //     {
    //         printf("err exit, reset alg");
    //         system("rm  /root/err_exit");
    //         system("pkill -9  alg");
    //     }

    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);       //设置为分离线程

    s32Ret = pthread_create(&u32Tid_v4l2, &attr, mpp_vpss_GetAlgFrame,NULL);//v4l2抓图

   s32Ret = pthread_create(&u32Tid_vdec, &attr, mpp_vpss_Vdec,NULL); //264解码

    s32Ret = pthread_create(&u32Tid_monitor, &attr, mpp_vpss_Monitor,NULL);

    }


    if (NULL == pstInitParam)
    {
        return ERR_NULL_PTR;
    }

    for (i = 0; i < 4; i++)
    {
#if 0
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

    // if (!bValidFd)
    // {
    //     return ERR_ILLEGAL_PARAM;
    // }

    memset(&m_stPdInfo, 0, sizeof(PD_INFO_S));
    s32Ret = pthread_mutex_init(&m_stPdInfo.mutexRunStat, NULL);
    if (0 != s32Ret)
    {
        print_level(SV_ERROR, "pthread_mutex_init failed! [err=%#x]\n", s32Ret);
        return ERR_SYS_NOTREADY;
    }

#if 0
    s32Ret = MH_Init();
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "MH_Init failed. [err=%#x]\n", s32Ret);
        return ERR_SYS_NOTREADY;
    }

    MH_PrintState(0);
#endif

    for(i = 0; i < sizeof(m_stPdInfo.modelMessageList)/sizeof(m_stPdInfo.modelMessageList[0]); i++)
    {
        m_stPdInfo.modelMessageList[i] = calloc(128, 1);   /* 信息使用128个字节 */
    }

#if (defined(BOARD_ADA42V1) || defined(BOARD_ADA42PTZV1))   
    apenChnAlg[0] = &pstInitParam->stAlgParam.stAlgCh1.enAlgType;
    apenChnAlg[1] = &pstInitParam->stAlgParam.stAlgCh2.enAlgType;
    apenChnAlg[2] = &pstInitParam->stAlgParam.stAlgCh3.enAlgType;
    apstPdsParam[0] = &pstInitParam->stAlgParam.stAlgCh1.stPdsParam;
    apstPdsParam[1] = &pstInitParam->stAlgParam.stAlgCh2.stPdsParam;
    apstPdsParam[2] = &pstInitParam->stAlgParam.stAlgCh3.stPdsParam;
#elif (defined(BOARD_ADA32IR))
    apenChnAlg[0] = &pstInitParam->stAlgParam.stAlgCh2.enAlgType;
    apstPdsParam[0] = &pstInitParam->stAlgParam.stAlgCh2.stPdsParam;
    apenChnAlg[1] = &pstInitParam->stAlgParam.stAlgCh1.enAlgType;
    apstPdsParam[1] = &pstInitParam->stAlgParam.stAlgCh1.stPdsParam;    
#else
    apenChnAlg[0] = &pstInitParam->stAlgParam.stAlgCh2.enAlgType;
    apstPdsParam[0] = &pstInitParam->stAlgParam.stAlgCh2.stPdsParam;
#endif

    m_stPdInfo.u32ChnNum = pstInitParam->u32ChnNum;
    for (i = 0; i < 4; i++)
    {
#if 0
        for (j = 0; j < 3; j++)
        {
            m_stPdInfo.as32MediaBufFd[i][j] = pstInitParam->as32MediaBufFd[i][j];
        }
#else
		m_stPdInfo.as32MediaBufFd[i] = pstInitParam->as32MediaBufFd[i];
#endif
    }

    if (0 != pstInitParam->u32Width && 0 != pstInitParam->u32Height)
    {
        m_stPdInfo.u32Width = pstInitParam->u32Width;
        m_stPdInfo.u32Height = pstInitParam->u32Height;
        m_stPdInfo.u32Width_90 = pstInitParam->u32Height;
        m_stPdInfo.u32Height_90 = pstInitParam->u32Width;
    }
    else
    {
        m_stPdInfo.u32Width = PD_IMAGE_WIDTH;
        m_stPdInfo.u32Height = PD_IMAGE_HEIGHT;
        m_stPdInfo.u32Width_90 = PD_IMAGE_WIDTH_90;
        m_stPdInfo.u32Height_90 = PD_IMAGE_HEIGHT_90;
    }
    
#if !defined(BOARD_ADA32V3)
    for (i = 0; i < m_stPdInfo.u32ChnNum; i++)
    {
        if (*(apenChnAlg[i]) != ALG_PDS)
        {
            continue;
        }
        
        if (BOARD_IsSVersion(BOARD_S_H_6M) || BOARD_IsSVersion(BOARD_S_V_6M))
        {
            s32Ret = Cali_init(780, 6.0, 0.0028, i);
        }
        else
        {
            s32Ret = Cali_init(780, 2.8, 0.0028, i);
        }
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "MS_Init failed. [err=%#x]\n", s32Ret);
            return ERR_SYS_NOTREADY;
        }
    
        if(apstPdsParam[i]->bCalibrated == SV_TRUE)
        {
            memcpy(stCaliParams.fK, apstPdsParam[i]->astCalibrationInterParams, sizeof(stCaliParams.fK));
            memcpy(stCaliParams.fRotation, apstPdsParam[i]->astCalibrationRotateVector, sizeof(stCaliParams.fRotation));
            memcpy(stCaliParams.fTranslation, apstPdsParam[i]->astCalibrationTranslateVector, sizeof(stCaliParams.fTranslation));
            memcpy(stCaliParams.fDistortion, apstPdsParam[i]->astCalibrationDistortionFactor, sizeof(stCaliParams.fDistortion));
            memcpy(stCaliParams.fOldCamPos, apstPdsParam[i]->astCalibrationCamPos, sizeof(stCaliParams.fOldCamPos));
            memcpy(stCaliParams.fEulerAngles, apstPdsParam[i]->astCalibrationEulerAngles, sizeof(stCaliParams.fEulerAngles));
            memcpy(stCaliParams.fPrincipal, apstPdsParam[i]->astCalibrationPrincipal, sizeof(stCaliParams.fPrincipal));

            s32Ret = Cali_import(stCaliParams, i);
            if(s32Ret != SV_SUCCESS)
            {
                print_level(SV_ERROR, "Cali_import fail!\n");
                return s32Ret;
            }
        }
    }
#endif

    pdsa32::STAlgParam stAlgParam(0.52, 0.45);              // 设置算法参数

    for (i = 0; i < m_stPdInfo.u32ChnNum; i++)
    {
        if (*(apenChnAlg[i]) != ALG_PDS)
        {
            continue;
        }
        
        apstPdsParam[i]->enPdsModel = pd_model_renew(apstPdsParam[i]->enPdsModel); // 由于模型合并,去除OW和NIR硬件类型

            apstPdsParam[i]->enPdsModel = E_PDS_PC;

        switch (apstPdsParam[i]->enPdsModel)
        {
            case E_PDS_P:
            case E_PDS_SH:
            case E_PDS_IR_P:
                m_stPdInfo.bSkipCar[i] = SV_TRUE;
                break;
            
            case E_PDS_C:
            case E_PDS_IR_C:
                m_stPdInfo.bSkipPerson[i] = SV_TRUE;
                break;
            default:
                break;
        }

        /* 获取模型文件位置 */
        memset(pszModelFileList, 0x00, sizeof(pszModelFileList));
        s32Ret = pd_model_file(apstPdsParam[i]->enPdsModel, pszModelFileList);
        if(s32Ret != SV_SUCCESS)
        {
            print_level(SV_ERROR, "pd_model_file failed!\n");
            return s32Ret;
        }

        for(j = 0; j < (sizeof(pszModelFileList) / sizeof(pszModelFileList[0])); j++)
        {
            if(pszModelFileList[j] == NULL)
                continue;
            print_level(SV_INFO, "need to load modefile[%d]:%s\n", j, pszModelFileList[j]);
        }

        s32Ret = getModelListMessage(pszModelFileList, m_stPdInfo.modelMessageList);
        if(s32Ret != SV_SUCCESS)
        {
            print_level(SV_ERROR, "getModelListMessage failed! [err=0x%x]\n", s32Ret);
        }

        print_level(SV_INFO, "chn[%d] ready to load model[%d]\n", i, apstPdsParam[i]->enPdsModel);
        switch (apstPdsParam[i]->enPdsModel)
        {
            case E_PDS_P:
                if (BOARD_IsCustomer(BOARD_C_ADA32V2_200889) || BOARD_IsCustomer(BOARD_C_ADA32V2_201623))
                {
                    goto toPcModel;
                }
                if (NULL == apcsPdsAlgRgbP)
                {
                    apcsPdsAlgRgbP = new pdsa32::CPdsAlg(pdsa32::E_PDSALG_TYPE_RGB_P);
                    if (NULL == apcsPdsAlgRgbP)
                    {
                        print_level(SV_ERROR, "new pdsa32::CPdsAlg failed. [err=%d]\n", s32Ret);
                        return SV_FAILURE;
                    }

                    s32Ret = apcsPdsAlgRgbP->AlgInit(pszModelFileList, stAlgParam, pd_CheckCallback);
                    if (0 != s32Ret)
                    {
                        print_level(SV_ERROR, "csPdsAlg.AlgInit failed. [err=%d]\n", s32Ret);
                        return SV_FAILURE;
                    }
                }
                m_stPdInfo.apcsPdsAlg[i] = apcsPdsAlgRgbP;
                break;
            case E_PDS_C:
            case E_PDS_PC:
toPcModel:
                if (NULL == apcsPdsAlgRgbPC)
                {
                    apcsPdsAlgRgbPC = new pdsa32::CPdsAlg(pdsa32::E_PDSALG_TYPE_RGB_PC);
                    if (NULL == apcsPdsAlgRgbPC)
                    {
                        print_level(SV_ERROR, "new pdsa32::CPdsAlg failed. [err=%d]\n", s32Ret);
                        return SV_FAILURE;
                    }

                    s32Ret = apcsPdsAlgRgbPC->AlgInit(pszModelFileList, stAlgParam, pd_CheckCallback);
                    if (0 != s32Ret)
                    {
                        print_level(SV_ERROR, "csPdsAlg.AlgInit failed. [err=%d]\n", s32Ret);
                        return SV_FAILURE;
                    }
                }
                m_stPdInfo.apcsPdsAlg[i] = apcsPdsAlgRgbPC;
                break;
            case E_PDS_MANHOLE:
                if (NULL == apcsPdsAlgRgbMANHOLE)
                {
                    apcsPdsAlgRgbMANHOLE = new pdsa32::CPdsAlg(pdsa32::E_PDSALG_TYPE_RGB_MANHOLE);
                    if (NULL == apcsPdsAlgRgbMANHOLE)
                    {
                        print_level(SV_ERROR, "new pdsa32::CPdsAlg failed. [err=%d]\n", s32Ret);
                        return SV_FAILURE;
                    }
                
                    s32Ret = apcsPdsAlgRgbMANHOLE->AlgInit(pszModelFileList, stAlgParam, pd_CheckCallback);
                    if (0 != s32Ret)
                    {
                        print_level(SV_ERROR, "csPdsAlg.AlgInit failed. [err=%d]\n", s32Ret);
                        return SV_FAILURE;
                    }
                }
                m_stPdInfo.apcsPdsAlg[i] = apcsPdsAlgRgbMANHOLE;
                break;
            case E_PDS_BEAR:
                if (NULL == apcsPdsAlgRgbBEAR)
                {
                    apcsPdsAlgRgbBEAR = new pdsa32::CPdsAlg(pdsa32::E_PDSALG_TYPE_RGB_LYQ_BEAR_201207);
                    if (NULL == apcsPdsAlgRgbBEAR)
                    {
                        print_level(SV_ERROR, "new pdsa32::CPdsAlg failed. [err=%d]\n", s32Ret);
                        return SV_FAILURE;
                    }
                
                    s32Ret = apcsPdsAlgRgbBEAR->AlgInit(pszModelFileList, stAlgParam, pd_CheckCallback);
                    if (0 != s32Ret)
                    {
                        print_level(SV_ERROR, "csPdsAlg.AlgInit failed. [err=%d]\n", s32Ret);
                        return SV_FAILURE;
                    }
                }
                m_stPdInfo.apcsPdsAlg[i] = apcsPdsAlgRgbBEAR;
                break;
            case E_PDS_IR_C:
            case E_PDS_IR_PC:
            case E_PDS_IR_P:
                if (NULL == apcsPdsAlgRgbIrPC)
                {
                    apcsPdsAlgRgbIrPC = new pdsa32::CPdsAlg(pdsa32::E_PDSALG_TYPE_RED_PC);
                    if (NULL == apcsPdsAlgRgbIrPC)
                    {
                        print_level(SV_ERROR, "new pdsa32::CPdsAlg failed. [err=%d]\n", s32Ret);
                        return SV_FAILURE;
                    }

                    s32Ret = apcsPdsAlgRgbIrPC->AlgInit(pszModelFileList, stAlgParam, pd_CheckCallback);
                    if (0 != s32Ret)
                    {
                        print_level(SV_ERROR, "csPdsAlg.AlgInit failed. [err=%d]\n", s32Ret);
                        return SV_FAILURE;
                    }
                }
                m_stPdInfo.apcsPdsAlg[i] = apcsPdsAlgRgbIrPC;
                break;
            case E_PDS_SH:
                if (NULL == apcsPdsAlgRgbSH)
                {
                    apcsPdsAlgRgbSH = new pdsa32::CPdsAlg(pdsa32::E_PDSALG_TYPE_RGB_SH);
                    if (NULL == apcsPdsAlgRgbSH)
                    {
                        print_level(SV_ERROR, "new pdsa32::CPdsAlg failed. [err=%d]\n", s32Ret);
                        return SV_FAILURE;
                    }

                    s32Ret = apcsPdsAlgRgbSH->AlgInit(pszModelFileList, stAlgParam, pd_CheckCallback);
                    if (0 != s32Ret)
                    {
                        print_level(SV_ERROR, "csPdsAlg.AlgInit failed. [err=%d]\n", s32Ret);
                        return SV_FAILURE;
                    }
                }
                m_stPdInfo.apcsPdsAlg[i] = apcsPdsAlgRgbSH;
                break;
            case E_PDS_SZ:
                apcsPdsAlgRgbSZ = new pdsa32::CPdsAlg(pdsa32::E_PDSALG_TYPE_RGB_CQQ_SIGN_201165);
                if (NULL == apcsPdsAlgRgbSZ)
                {
                    print_level(SV_ERROR, "new pdsa32::CPdsAlg failed. [err=%d]\n", s32Ret);
                    return SV_FAILURE;
                }
                
                s32Ret = apcsPdsAlgRgbSZ->AlgInit(pszModelFileList, stAlgParam, pd_CheckCallback);
                if (0 != s32Ret)
                {
                    print_level(SV_ERROR, "csPdsAlg.AlgInit failed. [err=%d]\n", s32Ret);
                    return SV_FAILURE;
                }
    
                m_stPdInfo.apcsPdsAlg[i] = apcsPdsAlgRgbSZ;
                break;
            case E_PDS_SSR:
                apcsPdsAlgRgbSSR = new pdsa32::CPdsAlg(pdsa32::E_PDSALG_TYPE_RGB_YMH_SSR_201338);
                if (NULL == apcsPdsAlgRgbSSR)
                {
                    print_level(SV_ERROR, "new pdsa32::CPdsAlg failed. [err=%d]\n", s32Ret);
                    return SV_FAILURE;
                }
                
                s32Ret = apcsPdsAlgRgbSSR->AlgInit(pszModelFileList, stAlgParam, pd_CheckCallback);
                if (0 != s32Ret)
                {
                    print_level(SV_ERROR, "csPdsAlg.AlgInit failed. [err=%d]\n", s32Ret);
                    return SV_FAILURE;
                }

                m_stPdInfo.apcsPdsAlg[i] = apcsPdsAlgRgbSSR;
                break;
            default:
                print_level(SV_ERROR, "invalid modelType[%d]=%d\n", i, apstPdsParam[i]->enPdsModel);
                return SV_FAILURE;
        }
        print_level(SV_INFO, "chn[%d] load model[%d] success!\n", i, apstPdsParam[i]->enPdsModel);
    }

// #if (defined(BOARD_ADA32V2)|| defined(BOARD_ADA32C4))
//     if (pstInitParam->bRotate)
//     {
//         /* 为旋转配置,去除原来所有模型信息 */
//         for(i = 0; i < sizeof(m_stPdInfo.modelMessageList) / sizeof(m_stPdInfo.modelMessageList[0]); i++)
//         {
//             memset(m_stPdInfo.modelMessageList[0], 0x00, sizeof(m_stPdInfo.modelMessageList[0]));
//         }

//         memset(pszModelFileList, 0x00, sizeof(pszModelFileList));
//         pszModelFileList[0] = PD_MODEL_RGB_PC_90;
        
//         s32Ret = getModelListMessage(pszModelFileList, m_stPdInfo.modelMessageList);
//         if(s32Ret != SV_SUCCESS)
//         {
//             print_level(SV_ERROR, "getModelListMessage failed! [err=0x%x]\n", s32Ret);
//         }
//     }

//     if (NULL == apcsPdsAlgRgbPC_90)
//     {
//         apcsPdsAlgRgbPC_90 = new pdsa32::CPdsAlg(pdsa32::E_PDSALG_TYPE_AIBOX_RGB_PC);
//         if (NULL == apcsPdsAlgRgbPC_90)
//         {
//             print_level(SV_WARN, "new pdsa32::CPdsAlg failed. [err=%d]\n", s32Ret);
//             goto skip_90;
//         }
    
//         s32Ret = apcsPdsAlgRgbPC_90->AlgInit(PD_MODEL_RGB_PC_90, stAlgParam, pd_CheckCallback);
//         if (0 != s32Ret)
//         {
//             print_level(SV_WARN, "csPdsAlg.AlgInit failed. [err=%d]\n", s32Ret);
//             goto skip_90;
//         }
//         m_stPdInfo.apcsPdsAlg_90 = apcsPdsAlgRgbPC_90;
//     }
// #endif    
skip_90:

    //if (BOARD_IsCustomer(BOARD_C_ADA32V2_201165))
    //{
    //    s32Ret = pd_SpeedZoneInit(pstInitParam);
    //    if(s32Ret != SV_SUCCESS)
    //    {
    //        print_level(SV_ERROR, "pd_SpeedZoneInit failed! [err=%d]\n");
    //    }
    //}

// #if (defined(BOARD_ADA32V2) || defined(BOARD_ADA32N1) || defined(BOARD_ADA32C4))
//     if (pstInitParam->stAlgParam.stAlgCh2.stPdsParam.enRoiStyle == CFG_PDROI_DRAWBOARD)
//     {
//         ROI_BOARD_CFG_S stRoiBoardCfg;
//         stRoiBoardCfg.bGreenEnable = pstInitParam->stAlgParam.stAlgCh2.stPdsParam.bPdRoiGreen;
//         stRoiBoardCfg.bYellowEnable = pstInitParam->stAlgParam.stAlgCh2.stPdsParam.bPdRoiYellow;
//         stRoiBoardCfg.bRedEnable = pstInitParam->stAlgParam.stAlgCh2.stPdsParam.bPdRoiRed;
//         stRoiBoardCfg.fGreenScale = pstInitParam->stAlgParam.stAlgCh2.stPdsParam.stPdRoiBoard.fGreenScale;
//         stRoiBoardCfg.fYellowScale = pstInitParam->stAlgParam.stAlgCh2.stPdsParam.stPdRoiBoard.fYellowScale;
//         stRoiBoardCfg.fRedScale = pstInitParam->stAlgParam.stAlgCh2.stPdsParam.stPdRoiBoard.fRedScale;
        
//         memcpy(stRoiBoardCfg.fGreenPoint, pstInitParam->stAlgParam.stAlgCh2.stPdsParam.stPdRoiBoard.fGreenPoint, sizeof(stRoiBoardCfg.fGreenPoint));
//         memcpy(stRoiBoardCfg.fYellowPoint, pstInitParam->stAlgParam.stAlgCh2.stPdsParam.stPdRoiBoard.fYellowPoint, sizeof(stRoiBoardCfg.fYellowPoint));
//         memcpy(stRoiBoardCfg.fRedPoint, pstInitParam->stAlgParam.stAlgCh2.stPdsParam.stPdRoiBoard.fRedPoint, sizeof(stRoiBoardCfg.fRedPoint));
        
//         s32Ret = ROI_BOARD_Init(&stRoiBoardCfg);
//         if (s32Ret != SV_SUCCESS)
//         {
//             print_level(SV_ERROR, "ROI_BOARD_Init failed![err=%d]\n", s32Ret);
//         }
//     }
// #endif

    m_stPdInfo.stCfgParam = pstInitParam->stAlgParam;
    m_stPdInfo.fConfidenceThr = 0.3f;
    m_stPdInfo.fNmsThreshold = 0.45f;
    m_stPdInfo.fTrackThreshold = 0.6f;
    m_stPdInfo.fTrackMaxAge = 4;
    afThresholds[0] = m_stPdInfo.fConfidenceThr;
    afThresholds[1] = m_stPdInfo.fNmsThreshold = 0.45f;
    afThresholds[2] = m_stPdInfo.fTrackThreshold = 0.6f;
    afThresholds[3] = m_stPdInfo.fTrackMaxAge = 4;
    m_stPdInfo.enSplitMode = pstInitParam->s32SplitMode;
    m_stPdInfo.bRotate     = pstInitParam->bRotate;
    pdsa32::STTrackParam stTacrkParam;    // 跟踪参数(用默认构造函数)
    m_stPdInfo.stTrackParam = stTacrkParam;

#if 0
    s32Ret = pd_LED_AlarmOUt_Direction(pstInitParam->stAlgParam.enAlgTrigger);
    if (s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "pd_LED_AlarmOUt_Direction failed! [err=%d]\n", s32Ret);
    }
    
    s32Ret = pd_LED_RGB_color_setting();
    if (s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "pd_LED_RGB_color_setting failed! [err=%d]\n", s32Ret);
    }
#endif

    pd_Alarm_Out_Init();

    sint8 *ps8Port =  "/dev/ttyS3";
    s32Ret = serial_init(ps8Port, 115200,0);

    sint8 *ps8Port1 = "/dev/ttyS4";
    s32Ret = serial_init(ps8Port1, 115200,1);
 

    //pthread_attr_t  attr;
    uint32 u32Tid_uartRead = 0;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);       //设置为分离线程
    s32Ret = pthread_create(&u32Tid_uartRead, &attr, uart_recieve,NULL);
    

    //request_threaded_irq();

    if (SV_TRUE == s32Ret)
    {
        printf( "serial_init SUCCESS!\n");
    }

    return SV_SUCCESS;
}

sint32 PD_Fini()
{
    sint32 i;

    for(i = 0; i < sizeof(m_stPdInfo.modelMessageList)/sizeof(m_stPdInfo.modelMessageList[0]); i++)
    {
        free(m_stPdInfo.modelMessageList[i]);
    }

    for (i = 0; i < ALG_MAX_CHN; i++)
    {
        if (NULL != m_stPdInfo.apcsPdsAlg[i])
        {
            delete m_stPdInfo.apcsPdsAlg[i];
        }
    }
    pthread_mutex_destroy(&m_stPdInfo.mutexRunStat);

    return SV_SUCCESS;
}

sint32 PD_Start()
{
    sint32 s32Ret = 0;
    pthread_t thread, thread1, thread2;

    m_stPdInfo.bRunning = SV_TRUE;
    s32Ret = pthread_create(&thread, NULL, pd_alg_Body, &m_stPdInfo);
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


    s32Ret = pthread_create(&thread1, NULL, pd_tool_Body, &m_stPdInfo);
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

    m_stPdInfo.thpool  = thpool_init(10);    /* 创建线程池, 数量为 10 */

    m_stPdInfo.u32TidAlg = thread;
    m_stPdInfo.u32TidTool = thread1;
#if 0
    s32Ret = pthread_create(&thread2, NULL, pd_Test_Body, &m_stPdInfo);
    if (0 != s32Ret)
    {
        print_level(SV_ERROR, "pthread_create failed! [err: %s]\n", strerror(errno));
        return ERR_SYS_NOTREADY;
    }
#endif

    return SV_SUCCESS;
}

sint32 PD_Stop()
{
    sint32 s32Ret = 0;
    pthread_t thread = m_stPdInfo.u32TidAlg;
    void *pvRetval = NULL;

    m_stPdInfo.bRunning = SV_FALSE;
    s32Ret = pthread_join(thread, &pvRetval);
    if (0 != s32Ret)
    {
        print_level(SV_ERROR, "pthread_join failed. [err: %s]\n", strerror(errno));
        return SV_FAILURE;
    }

    thpool_wait(m_stPdInfo.thpool);
    thpool_destroy(m_stPdInfo.thpool);

    return SV_SUCCESS;
}

sint32 PD_ConfigSet(CFG_ALG_PARAM *pstCfgParam)
{
    sint32 s32Ret;
    if (NULL == pstCfgParam)
    {
        return ERR_NULL_PTR;
    }

    pthread_mutex_lock(&m_stPdInfo.mutexRunStat);
    m_stPdInfo.stCfgParam = *pstCfgParam;
    pthread_mutex_unlock(&m_stPdInfo.mutexRunStat);
    

#if (defined(BOARD_ADA32V2) || defined(BOARD_ADA32N1) || defined(BOARD_ADA32C4))
    if (pstCfgParam->stAlgCh2.stPdsParam.enRoiStyle == CFG_PDROI_DRAWBOARD)
    {
        ROI_BOARD_CFG_S stRoiBoardCfg;
        stRoiBoardCfg.bGreenEnable = pstCfgParam->stAlgCh2.stPdsParam.bPdRoiGreen;
        stRoiBoardCfg.bYellowEnable = pstCfgParam->stAlgCh2.stPdsParam.bPdRoiYellow;
        stRoiBoardCfg.bRedEnable = pstCfgParam->stAlgCh2.stPdsParam.bPdRoiRed;
        stRoiBoardCfg.fGreenScale = pstCfgParam->stAlgCh2.stPdsParam.stPdRoiBoard.fGreenScale;
        stRoiBoardCfg.fYellowScale = pstCfgParam->stAlgCh2.stPdsParam.stPdRoiBoard.fYellowScale;
        stRoiBoardCfg.fRedScale = pstCfgParam->stAlgCh2.stPdsParam.stPdRoiBoard.fRedScale;
        
        memcpy(stRoiBoardCfg.fGreenPoint, pstCfgParam->stAlgCh2.stPdsParam.stPdRoiBoard.fGreenPoint, sizeof(stRoiBoardCfg.fGreenPoint));
        memcpy(stRoiBoardCfg.fYellowPoint, pstCfgParam->stAlgCh2.stPdsParam.stPdRoiBoard.fYellowPoint, sizeof(stRoiBoardCfg.fYellowPoint));
        memcpy(stRoiBoardCfg.fRedPoint, pstCfgParam->stAlgCh2.stPdsParam.stPdRoiBoard.fRedPoint, sizeof(stRoiBoardCfg.fRedPoint));
        
        s32Ret = ROI_BOARD_Init(&stRoiBoardCfg);
        if (s32Ret != SV_SUCCESS)
        {
            print_level(SV_ERROR, "ROI_BOARD_Init failed![err=%d]\n", s32Ret);
        }
    }
#endif

#if (defined(BOARD_ADA32V2) || defined(BOARD_ADA32C4))
    s32Ret = pd_LED_AlarmOUt_Direction(pstCfgParam->enAlgTrigger);
    if (s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "pd_LED_AlarmOUt_Direction failed! [err=%d]\n", s32Ret);
    }
#endif
    
    pd_Alarm_Out_Reset();
    return SV_SUCCESS;
}

sint32 PD_SplitSet(sint32 enSplitMode)
{
    if ((MEDIA_SPLIT_THREE == enSplitMode)
        || (MEDIA_SPLIT_FOUR == enSplitMode)
        || (MEDIA_SPLIT_SIX == enSplitMode)
        || (enSplitMode > MEDIA_SPLIT_BUTT))
    {
        return ERR_ILLEGAL_PARAM;
    }
    /* 释放热插拔设备红外摄像头的buffer */
    if (MEDIA_SPLIT_ONE == enSplitMode)
    {
        MS_V(1);
    }
    m_stPdInfo.enSplitMode = enSplitMode;
    return SV_SUCCESS;
}

sint32 PD_RotateSet(SV_BOOL bRotate)
{
    m_stPdInfo.bRotate = bRotate;
    return SV_SUCCESS;
}

sint32 PD_SendCanData(MSG_CAN_DATA_S *pstCanData)
{
    sint32 s32Ret;
    MSG_PACKET_S stMsgPkt = {0};
    MSG_CAN_DATA_S stMsgCanData = {0};

    memset(&stMsgCanData, 0, sizeof(stMsgCanData));
    stMsgCanData.bSingleFrame = pstCanData->bSingleFrame;
    stMsgCanData.s32DataLen = pstCanData->s32DataLen;
    strncpy(stMsgCanData.szCanData, pstCanData->szCanData, stMsgCanData.s32DataLen);
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

    return SV_SUCCESS;
}

/* 获取重置后的新模型 */
EPdsModel pd_model_renew(EPdsModel pdsmodel)
{
    EPdsModel model = pdsmodel;

    switch(pdsmodel)
    {
        case E_PDS_P:
        case E_PDS_OWP:
        case E_PDS_NIR_P:
            model = E_PDS_P;
            break;
        case E_PDS_PC:
        case E_PDS_OWPC:
        case E_PDS_NIR_PC:
            model = E_PDS_PC;
            break;
        case E_PDS_C:
        case E_PDS_OWC:
        case E_PDS_NIR_C:
            model = E_PDS_C;
            break;
        default:
            break;
    }

    if (BOARD_IsCustomer(BOARD_C_ADA32V2_201165))
    {
        model = E_PDS_SZ;
    }
    
    if (BOARD_IsCustomer(BOARD_C_ADA32V2_201338))
    {
        model = E_PDS_SSR;
    }

    return model;
}


/* 获取模型路径,已经过滤掉OW和NIR硬件平台 */
sint32 pd_model_file(EPdsModel pdsmodel, char **modefilelist)
{
    char *pModelFile = NULL;
    
    switch(pdsmodel)
    {
        case E_PDS_P:
            // modefilelist[0] = PD_MODEL_RGB_P;
            
            // if (BOARD_IsCustomer(BOARD_C_ADA32V2_FTC))
            //     modefilelist[0] = PD_MODEL_RGB_P_FTC;
            
            // if (BOARD_IsCustomer(BOARD_C_ADA32V2_201266B) || BOARD_IsCustomer(BOARD_C_ADA32V2_201266C))
            //     modefilelist[0] = PD_MODEL_RGB_P_201266B;
            
            modefilelist[0] = PD_MODEL_RGB_TEST;

            break;
        case E_PDS_PC:
        case E_PDS_C:

            modefilelist[0] = PD_MODEL_RGB_TEST;    
            // modefilelist[0] = PD_MODEL_RGB_PC;
        
            // if (BOARD_IsCustomer(BOARD_C_ADA32V2_202406))
            //     modefilelist[0] = PD_MODEL_RGB_PC_202406;
            
            break;
        case E_PDS_MANHOLE:
            modefilelist[0] = PD_MODEL_RGB_MANHOLE;
            break;
        case E_PDS_BEAR:
            modefilelist[0] = PD_MODEL_RGB_BEAR;
            break;
        case E_PDS_SH:
            modefilelist[0] = PD_MODEL_RGB_SH;
            break;
        case E_PDS_SSR:
            modefilelist[0] = PD_MODEL_RGB_SSR;
            modefilelist[1] = PD_MODEL_TRAFFIC;
            break;
        case E_PDS_SZ:
            modefilelist[0] = PD_MODEL_RGB_CQQ_SIGN;
            break;
        case E_PDS_IR_P:
        case E_PDS_IR_C:
        case E_PDS_IR_PC:
            modefilelist[0] = PD_MODEL_IR_PC;
            break;
        default:
            print_level(SV_ERROR, "cannot parse pdsmodel:%d\n", pdsmodel);
            return SV_FAILURE;
            break;
    }

    return SV_SUCCESS;
}


sint32 swap_endian(sint32 value)
{
    uint8 *p = (uint8 *)&value;
    return (sint32)((sint32)p[0] << 24 | (sint32)p[1] << 16 | (sint32)p[2] << 8 | (sint32)p[3]);
}


/* 获取模型信息 */
sint32 getModelMessage(char *modelfile, char *string)
{
    sint32 s32Ret;
    sint32 fd;
    char buf[1024];
    char *jsonHead;
    sint32 jsonlen;
    int readn;

    if (modelfile == NULL || string == NULL)
    {
        return ERR_NULL_PTR;
    }

    if (access(modelfile, F_OK) != 0)
    {
        return ERR_INVALID_DATA;
    }

    fd = open(modelfile, O_RDONLY);
    if(fd < 0)
    {
        print_level(SV_ERROR, "open %s failed!\n", modelfile);
        return SV_FAILURE;
    }

    readn = read(fd, buf, sizeof(buf));
    if(readn != sizeof(buf))
    {
        close(fd);
        print_level(SV_ERROR, "read %s failed!\n", modelfile);
        return SV_FAILURE;
    }

    /* 非JSON版本,通过MD5来实现 */
    if (strncmp(buf, "JSON", 4) != 0) 
    {
        char cmdStr[128];
        close(fd);
        sprintf(cmdStr, "md5sum %s | awk -F ' ' '{printf $1}'", modelfile);
        s32Ret = SAFE_System_Recv(cmdStr, buf, sizeof(buf));
        if(s32Ret < 0)
        {
            print_level(SV_ERROR, "SAFE_System_Recv cmdStr:%s failed!\n", cmdStr);
            return SV_FAILURE;
        }

        //memmove(buf, buf + strlen(buf) - 8, 8+1);
        print_level(SV_INFO, "buf:%s\n", buf);
        sprintf(string, "M:%.8s", buf + strlen(buf) - 8);
        //print_level(SV_INFO, "string:%s\n", string);
        return SV_SUCCESS;
    }

    cJSON *pstJson = NULL, *pstJsonTmp = NULL;
    /* JSON版本 */
    memcpy(&jsonlen, buf + strlen("JSON"), sizeof(jsonlen));   /* 拷贝json长度大小 */
    jsonlen = swap_endian(jsonlen); /* 大小端转化 */
    jsonHead = buf + strlen("JSON") + sizeof(jsonlen);
    jsonHead[jsonlen] = '\0';

    pstJson = cJSON_Parse(jsonHead);
    if (NULL == pstJson)
    {
        close(fd);
        print_level(SV_ERROR, "cJSON_Parse failed!\n");
        return SV_FAILURE;
    }

    pstJsonTmp = cJSON_GetObjectItemCaseSensitive(pstJson, "MD5");
    if(pstJsonTmp != NULL)
    {
        //print_level(SV_INFO, "md5sum %s %s\n", modelfile, pstJsonTmp->string);
        sprintf(string, "M:%.8s", pstJsonTmp->valuestring);
        //print_level(SV_INFO, "string:%s\n", string);
    }
    
    cJSON_Delete(pstJson);
    close(fd);

    return SV_SUCCESS;
}

/* 获取模型列表信息 */
sint32 getModelListMessage(char **modelFileList, char **stringList)
{
    sint32 s32Ret, i, j;
    if (modelFileList == NULL || stringList == NULL)
    {
        return ERR_NULL_PTR;
    }

    /* 找到第一个空白的指针 */
    while(strlen(*stringList)) stringList++;

    for(i = 0; i < 3; i++)
    {
        print_level(SV_INFO, "modelFileList[%d]:%d\n", i, modelFileList[i]);
        if (modelFileList[i] == NULL)
        {
            break;
        }

        s32Ret = getModelMessage(modelFileList[i], *(stringList++));
        if(s32Ret != SV_SUCCESS)
        {
            print_level(SV_ERROR, "getModelMessage failed![err=0x%x]\n", s32Ret);
            continue;
            //return s32Ret;
        }
    }
    
    return SV_SUCCESS;
}

/* 将算法信息发送给ipsys */
sint32 postModelListMessage(char **stringList)
{
    sint32 s32Ret = 0, i;
    uint16 u16mask;
    MSG_PACKET_S stMsgPkt = {0};
    MEDIA_GUI_DRAW_S stMediaGuiDraw = {0};
    MEDIA_GUI_NULL_S stGuiNull;
    MEDIA_GUI_STRING_S stGuiString;
    float xstart = 1;
    float ystart = 0.9;
    float ystep = 0.05;
    
    const int contime = 2;  /* 持续时间 */

    memset(&stMsgPkt, 0, sizeof(stMsgPkt));
    stMsgPkt.pu8Data = (uint8*)&stMediaGuiDraw;
    stMsgPkt.u32Size = sizeof(MEDIA_GUI_DRAW_S);
    u16mask = MEDIA_GUI_GET_MASK(0, MEDIA_GUI_PLANE_EXTERN2, MEDIA_GUI_OP_CLEAR);
    s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiNull);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
    }

    if(BOARD_IsCustomer(BOARD_C_ADA32V2_WXKY))
    {
        ystart = 0.85;
    }

    for (i = 0; i < 5; i++)
    {
        if (strlen(stringList[i]) == 0)
        {
            break;
        }
        stGuiString.x = xstart - strlen(stringList[i]) * 0.018;
        stGuiString.y = ystart - i * ystep;
        stGuiString.fontsize = 3;
        stGuiString.color = GUI_COLOR_BLUE;
        stGuiString.contime = 3; // 持续5秒钟 
        stGuiString.bShowAllTime = SV_TRUE;
        strcpy(stGuiString.string, stringList[i]);
        u16mask = MEDIA_GUI_GET_MASK(0, MEDIA_GUI_PLANE_EXTERN2, MEDIA_GUI_OP_DRAW_STRING);
        s32Ret = MEDIA_GUI_INSERT(stMediaGuiDraw, u16mask, stGuiString);
        if (SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "MEDIA_GUI_INSERT failed. [err=%#x]\n", s32Ret);
        }
    }

    s32Ret = Msg_submitEvent(EP_CONTROL, OP_EVENT_MEDIA_GUI, &stMsgPkt);
    if (SV_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "Msg_submitEvent failed. [err=%#x]\n", s32Ret);
    }

    return SV_SUCCESS;
}



