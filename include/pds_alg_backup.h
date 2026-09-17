//
// Created by root on 4/23/20.
//

#ifndef _ADAS32_PDS_ALGLIB_H_
#define _ADAS32_PDS_ALGLIB_H_

#include "stdint.h"
#include "stdlib.h"
#include "stdio.h"
#include "rknn_api.h"
#include "vector"
#include <string>
// #include "drmrga.h"
// #include "RockchipRga.h"

using namespace std;
namespace pdsa32{

#define SV_MAX_OUTPUT_PLATE_DET_NUM 10 // 最多能输出的车牌检测结果数量

/**********************************************************
 * 回调函数: 算法库认证
 * 输入：
 *     au8SourceAddr: 16个字节, 应用层拿到的数据
 *     au8ResultAddr: 16个字节, 应用层处理完后, 给算法库的数据
 * 
 * 返回:
 *     
**********************************************************/
typedef int32_t (*ALG_CHECK_CALLBACK)(uint8_t* au8SourceAddr, uint8_t* au8ResultAddr);



    /****************************************
     * 返回值, 部分与 rknn_api.h 保持同步
     ****************************************/
    typedef enum PDSALG_PROCESS_CODE_E {
        /********************************RKNN的错误码***********************************************/
        E_ALG_SUCCESS                                     = 0,    //  成功
        E_ALG_ERR_FAIL                                    = -1,   //  执行失败
        E_ALG_ERR_TIMEOUT                                 = -2,   //  线性库优化失败
        E_ALG_ERR_DEVICE_UNAVAILABLE                      = -3,   //  设备不可访问
        E_ALG_ERR_MALLOC_FAIL                             = -4,   //  内存创建失败
        E_ALG_ERR_PARAM_INVALID                           = -5,   //  参数无效
        E_ALG_ERR_MODEL_INVALI                            = -6,   //  模型无效
        E_ALG_ERR_CTX_INVALID                             = -7,   //  context无效
        E_ALG_ERR_INPUT_INVALID                           = -8,   //  输入无效
        E_ALG_ERR_OUTPUT_INVALID                          = -9,   //  输出无效
        E_ALG_ERR_DEVICE_UNMATCH                          = -10,  //  设备不匹配, 需要更新RKNN的sdk和npu的驱动和固件
        E_ALG_ERR_INCOMPATILE_PRE_COMPILE_MODEL           = -11,  //  加载的RKNN模型是预编译的, 当前驱动不支持这种模式
        E_ALG_ERR_INCOMPATILE_OPTIMIZATION_LEVEL_VERSION  = -12,  //  加载的RKNN模型设置了优化级别, 当前驱动不支持这种模式
        E_ALG_ERR_TARGET_PLATFORM_UNMATCH                 = -13,  //  加载的RKNN模型和当前运行平台不符合
        E_ALG_ERR_NON_PRE_COMPILED_MODEL_ON_MINI_DRIVE    = -14,  //  加载的RKNN模型不是预编译的, 当前用的mini driver跑不了
        /********************************RKNN的错误码***********************************************/


        /********************************本算法库的错误码***********************************************/
        E_ALG_ERR_RE_INIT                                 = -20,  //  请不要重复初始化
        E_ALG_ERR_MODEL_NULL                              = -21,  //  初始化时, 模型文件的指针为空, 请正确设置参数
        E_ALG_ERR_LOAD_MODEL                              = -22,  //  初始化时, 加载模型文件失败
        E_ALG_ERR_MUST_INIT                               = -23,  //  需要先进行初始化
        E_ALG_ERR_TYPE_UNMATCH                            = -24,  //  加载的模型与设置的算法类型不符合, 例如设置的是只检人的模型, 却加载了检人检车的模型 
        E_ALG_ERR_TYPE_UNSUPPORT                          = -25,  //  不支持的算法模型 
        E_ALG_ERR_ALG_VERSION                             = -26,  //  算法版本问题, 不知道算法库对应的是YOLOv5还是YOLOx, 需要联系开发者进行联调,确认,Debug
        E_ALG_ERR_DMA_BUFFER                              = -27,  //  申请DMAbuffer错误
        E_ALG_ERR_ZERO_COPY_NO_FD                         = -28,  //  零拷贝模式(加载模型后识别到是零拷贝的模型), 但是没有DMAbuffer的文件描述符, 无法获取硬件地址和使用零拷贝, 要么换个不是零拷贝的模型
        E_ALG_ERR_CALLBACK_NULL_PTR                       = -29,  //  回调函数为空
        E_ALG_ERR_AUTHENTICATE                            = -30,  //  算法认证失败
        E_ALG_ERR_CHANNEL_LIMIT                           = -31,  //  通道号超出上限了, 最多支持4通道
        E_ALG_ERR_PARSE_JSON                              = -32,  //  解析模型JSON失败
        E_ALG_ERR_NO_IMPLEMENT                            = -33,  //  功能未实现
        /********************************本算法库的错误码***********************************************/

    }EAlgProcessCode;


    /***********************
     * 算法类型
     ***********************/
    typedef enum PDS_ALG_TYPE_E {
        E_PDSALG_TYPE_RGB_P = 0,       /* 可见光(RGB)检测, 行人(P) */
        E_PDSALG_TYPE_RGB_PC,          /* 可见光(RGB)检测, 行人(P) + 车辆(C) */
        E_PDSALG_TYPE_AIBOX_RGB_P,     /* 可见光(RGB)检测, 行人(P) */              /*AiBox用的, 尺寸小一点, 速度快一点  20230717弃用: 加了JSON可以直接替, 改输入尺寸就行了*/
        E_PDSALG_TYPE_AIBOX_RGB_PC,    /* 可见光(RGB)检测, 行人(P) + 车辆(C) */    /*AiBox用的, 尺寸小一点, 速度快一点  20230717弃用: 加了JSON可以直接替, 改输入尺寸就行了*/
        E_PDSALG_TYPE_RED_PC,          /*   红外(RED)检测, 行人(P) + 车辆(C) */
        E_PDSALG_TYPE_NIR_PC,          /* 近红外(near Infrared)检测, 行人(P) + 车辆(C) */
        E_PDSALG_TYPE_RGB_P_OW,        /* 可见光(RGB)检测, 大角度A37, 行人(P) */
        E_PDSALG_TYPE_RGB_PC_OW,       /* 可见光(RGB)检测, 大角度A37, 行人(P) + 车辆(C) */
        E_PDSALG_TYPE_RGB_ADAS,        /* 可见光(RGB)ADAS算法, 480x288, 包括行人,车辆,轨道,交通标志 */
        E_PDSALG_TYPE_RGB_ADAS2,       /* 可见光(RGB)(DV435)ADAS算法, 608x352, 包括 绿灯,黄灯,红灯,限速标志,人,车,轨道 */
        E_PDSALG_TYPE_RGB_ADAS3,       /* 可见光(RGB)(DashCam)ADAS算法, 576x416, 包括 绿灯,黄灯,红灯,人,车,轨道 */
        E_PDSALG_TYPE_RGB_ADAS4,       /* 可见光(RGB)ADAS算法, 608x352, 包括 绿灯,黄灯,红灯,限速标志,人,车,轨道, 用OCR识别交通标志 */
        E_PDSALG_TYPE_RGB_PCZL,        /* 可见光(RGB)ADAS算法, 416x224, 行人(P)+车辆(C)+斑马线(Z)+车道线(L) */
        E_PDSALG_TYPE_RGB_HEAD,        /* 可见光(RGB)检测, 人头(head)计数, */
        E_PDSALG_TYPE_RGB_MANHOLE,     /* 可见光(RGB)检测, 检沙井盖(Manhole), */
        E_PDSALG_TYPE_RGB_PF,          /* 可见光(RGB)检测, 行人(Person) + 叉车(Forklift), */
        E_PDSALG_TYPE_RED_SR,          /*   红外(RED), 超分辨率算法(Super Resolution) */
        E_PDSALG_TYPE_RGB_SH,          /* 可见光(RGB)检测, 安全帽(Safety Hat) */
        E_PDSALG_TYPE_RGB_PLATE_DET,   /* 可见光(RGB)检测, 车牌检测（plate det） */
        E_PDSALG_TYPE_RGB_PLATE_REC,   /* 可见光(RGB)字符识别, 车牌字符检测（plate rec） */
        E_PDSALG_TYPE_RGB_UNI_SIGN ,   /* 可见光(RGB)检测 ,六种通用标志识别 */
        E_PDSALG_TYPE_RGB_PC_UNISIGN,  /* 可见光(RGB)检测 ,人车+六种通用标志识别 */
        E_PDSALG_TYPE_RGB_PC_90,       /* 可见光(RGB)检测, 行人(P) + 车辆(C) 90' */




        /**********************************************************
         * 一些客户定制的.... 
         * 命名方法: 光源(RGB:可见光; RED:红外)
         *          + 业务员(CQQ:陈芹勤; YMH:易美华)
         *          + 功能与任务(SIGN:各种标志; SSR:Speed Sign recognize)
         *          + 客户号
         * 如果只写客户号, 将导致代码可读性变差, 写程序时不清楚含义和逻辑
         * 如果只写业务员和功能, 则不具有唯一性, 因为相同的业务员会有不同的功能需求, 不同的业务员可能有相同的功能需求, 同一个业务员和需求可能是不同的客户
         **********************************************************/
        E_PDSALG_TYPE_RGB_CQQ_SIGN_201165 = 300,      /* 可见光(RGB)检测, 陈勤芹的红蓝标志检测, 客户号201165 */
        E_PDSALG_TYPE_RGB_YMH_SSR_201338,     /* 可见光(RGB)检测, 易美华的SSR功能, 客户号201338, 检人,车,限速标志*/
        E_PDSALG_TYPE_RGB_LYQ_BEAR_201207,     /* 可见光(RGB)检测, 李颖琪 熊模型, 客户号201207, 检熊*/

        E_PDSALG_TYPE_REVERSE    /* 暂未开发.... */

    } EAlgType;


    /***********************
     * 目标类型
     ***********************/
    typedef enum PDS_ALG_OBJ_CLS {
        E_CLS_PERSON      = 0,     /* 人 */
        E_CLS_HEAD        = 1,     /* 人头 */
        E_CLS_CAR         = 2,     /* 车 */
        E_CLS_ZEBRA       = 3,     /* 斑马线 */
        E_CLS_MANHOLE     = 4,     /* 沙井盖 */
        E_CLS_FORKLIFT    = 5,     /* 叉车 */

        E_LIGHT_RED       = 10,    /* 红灯 */
        E_LIGHT_YELLOW    = 11,    /* 黄灯 */
        E_LIGHT_GREEN     = 12,    /* 绿灯 */
        
        E_CLS_PERSON_HAT  = 20,    /* 戴着安全帽的人 */
        E_HAT_RED         = 21,    /* 红色安全帽*/
        E_HAT_YELLOW      = 22,    /* 黄色安全帽*/
        E_HAT_WHITE       = 23,    /* 白色安全帽*/
        E_HAT_BLUE        = 24,    /* 蓝色安全帽*/
        
        // 20230328: 去掉了, 用int变量来表示限速值
        // E_SIGN_NO         = 100,   /* 限速0 */
        // E_SIGN_10         = 101,   /* 限速10 */
        // E_SIGN_100        = 102,   /* 限速100 */
        // E_SIGN_110        = 103,   /* 限速110 */
        // E_SIGN_120        = 104,   /* 限速120 */
        // E_SIGN_130        = 105,   /* 限速130 */
        // E_SIGN_140        = 106,   /* 限速140 */
        // E_SIGN_15         = 107,   /* 限速15 */
        // E_SIGN_20         = 108,   /* 限速20 */
        // E_SIGN_30         = 109,   /* 限速30 */
        // E_SIGN_40         = 110,   /* 限速40 */
        // E_SIGN_5          = 111,   /* 限速5 */
        // E_SIGN_50         = 112,   /* 限速50 */
        // E_SIGN_60         = 113,   /* 限速60 */
        // E_SIGN_70         = 114,   /* 限速70 */
        // E_SIGN_80         = 115,   /* 限速80 */
        // E_SIGN_90         = 116,   /* 限速90 */
        
        E_CLS_SIGN_US,                 /* 美国的各种标志, 美国的标志比较复杂, 详情见结构体 STAlgUSSign */
        E_CLS_SIGN_SPEED_CHN,          /* 中国限速标志, 具体限速多少见 STAlgResult.u32SpeedLimit */
        E_CLS_SIGN_HIGHT,              /* 限高标志, 具体限高多少见 STAlgResult.fHightLimit */
        E_CLS_SIGN_STOP,               /* STOP/停 标志 */

        E_CLS_PLATE_SINGLE,            /* 单行车牌 标志 */
        E_CLS_PLATE_DOUBLE,            /* 双行车牌 标志 */

        E_CLS_UNI_SIGN_PIC1,            /* 六种通用标志识别 标志1 */
        E_CLS_UNI_SIGN_PIC2,            /* 六种通用标志识别 标志2 */
        E_CLS_UNI_SIGN_PIC3,            /* 六种通用标志识别 标志3 */
        E_CLS_UNI_SIGN_PIC4,            /* 六种通用标志识别 标志4 */
        E_CLS_UNI_SIGN_PIC5,            /* 六种通用标志识别 标志5 */
        E_CLS_UNI_SIGN_PIC6,            /* 六种通用标志识别 标志6 */

        E_SPEED_ZONE      = 300,   /* 红色 SPEED ZONE (陈勤芹客户仓库限速标志)*/
        E_END_SPEED_ZONE  = 301,   /* 蓝色 END SPEED ZONE (陈勤芹客户仓库限速标志)*/

        E_CLS_PERSON_HAT_RED,       /* 带红色安全帽的人 */
        E_CLS_PERSON_HAT_YELLOW,    /* 带黄色安全帽的人 */
        E_CLS_PERSON_HAT_WHITE,     /* 带白色安全帽的人 */
        E_CLS_PERSON_HAT_BLUE,      /* 带蓝色安全帽的人 */

        E_CLS_BEAR,                /* 熊*/   
        E_CLS_RESERVE1,            /* 保留类型, 未知 */ 
        E_CLS_RESERVE2,            /* 保留类型, 未知 */  
        E_CLS_RESERVE3,            /* 保留类型, 未知 */ 
    } EAlgObjectClass;

    /***********************
     * 美国的各种路标标志
     ***********************/
    typedef enum PDS_ALG_US_SIGN_TYPE {
        E_SIGN_US_SPEED_LIMIT,            /* 最高限速 */
        E_SIGN_US_MINIMUM_SPEED,          /* 最低限速 */
        E_SIGN_US_MAX_MIN,                /* 同时限了最高和最低 */
        E_SIGN_US_TRUCKS,                 /* 卡车限速 */
        E_SIGN_US_NIGHT,                  /* 夜间限速 */
        E_SIGN_US_END_MPH,                /*    */
        E_SIGN_US_STATE_SPEED_LIMIT,      /*    */
        E_SIGN_US_END_MPH_LIMIT,          /*    */
        E_SIGN_US_END_MILE_SPEED          /*    */
    } EAlgUSSignType;

    typedef struct PDS_ALG_US_SIGN
    {
        EAlgUSSignType eUSSignType;  // 美国标志类型
        uint32_t u32SpeedLimitHigh;  // 限速上限
        uint32_t u32SpeedLimitLow;  // 限速下限
    }STAlgUSSign;

    /***********************
     * 车道线类型
     ***********************/
    typedef enum PDS_ALG_LANE_TYPE {
        E_LANE_LEFT_SOLID    = 0,     /* 左实线 */
        E_LANE_LEFT_DASH     = 1,     /* 左虚线 */
        E_LANE_RIGHT_SOLID   = 2,     /* 右实线 */
        E_LANE_RIGHT_DASH    = 3,     /* 右虚线 */
        E_LANE_MIDDLE        = 4,     /* 中线 */
    } EAlgLaneType;

    /***********************
     * 相机视角
     ***********************/
    typedef enum PDS_ALG_CAM_VIEW_TYPE {
        E_CAM_VIEW_FORWARD    = 0,     /* 前视镜头0 */
        E_CAM_VIEW_FORWARD_1  = 1,     /* 前视镜头1 */
        E_CAM_VIEW_BACK       = 2,     /* 后视镜头0 */
        E_CAM_VIEW_BACK_1     = 3,     /* 后视镜头1 */
        E_CAM_VIEW_LEFT       = 4,     /* 左视镜头0 */
        E_CAM_VIEW_LEFT_1     = 5,     /* 左视镜头1 */
        E_CAM_VIEW_RIGHT      = 6,     /* 右视镜头0 */
        E_CAM_VIEW_RIGHT_1    = 7,     /* 右视镜头1 */
    } EAlgCamViewType;

    /***********************
     * 人头计数的界限的类型
     ***********************/
    typedef enum PDS_ALG_HEAD_COUNT_LINE_TYPE {
        E_COUNT_HEAD_Y     = 0,     /* 判断Y轴 (上下移动的情况) */
        E_COUNT_HEAD_X     = 1,     /* 判断X轴 (左右移动的情况) */
        E_COUNT_HEAD_UP    = 2,     /* 判断Y轴, 向上为上车方向 */
        E_COUNT_HEAD_DOWN  = 3,     /* 判断Y轴, 向下为上车方向 */
        E_COUNT_HEAD_LEFT  = 4,     /* 判断X轴, 向左为上车方向 */
        E_COUNT_HEAD_RIGHT = 5,     /* 判断X轴, 向右为上车方向 */
    } EAlgHeadLineType;

    /***********************
     * 人头计数的跟踪区域类型
     ***********************/
    typedef enum PDS_ALG_HEAD_TRACK_TYPE_CODE_E {
        E_COUNT_TRACK_FULL_SCREEN           = 0,     /* 全屏计数, 对整个屏幕内检测到的人头框进行跟踪 */
        E_COUNT_TRACK_CENTER_IN_RECTANGLE   = 1,     /* 对于检测到的人头框, 中心点落在计数区域内的, 进行跟踪和计数*/
        E_COUNT_TRACK_ENTIRE_IN_RECTANGLE   = 2,     /* 对于检测到的人头框, 整个框都在计数区域内的, 进行跟踪和计数*/
    } EAlgHeadTrackType;

    /***********************
     * 人头计数的计数方式
     ***********************/
    typedef enum PDS_ALG_HEAD_COUNT_TPYE_CODE_E {
        E_COUNT_ONCE                = 0,     /* 人头经过计数线之后, 计1次, 后续反复横跳不再计数, 完全离开技术区域后才能再次计数 */
        E_COUNT_REPEATEDLY          = 1,     /* 可以重复计数, 只要在计数线上徘徊, 每次都计 */
    } EAlgHeadCountType;


    /***********************
     * 后处理选项
     ***********************/
    typedef enum PDS_POSTPROCESS_CODE_E {
        E_POST_NONE = 0,         /* 啥也不干 */
        E_POST_TRACK,            /* 启用跟踪, 消除一闪而过的误检 */
        E_POST_OVER_ALARM,       /* 超车判断 */
        E_POST_TRACK_COUNT_HEAD,  /* 人头计数 */
        E_POST_TRACK_COUNT_HEAD_CLOSE_DOOR,  /* 人头计数: 关门信号(把站在门口但是没上车的人算上去) */
    } EAlgPostCode;


    /***********************
     * 车道线的点, 或者其它的点
     ***********************/
    struct STAlgPoint
    {
        float fX;
        float fY;
        
        STAlgPoint():fX(0),fY(0){}  // 默认值
        STAlgPoint(float x, float y):fX(x),fY(y){}
    };

    /***********************
     * 一条车道线
     ***********************/
    struct STAlgLane
    {
        EAlgLaneType eType;        // 车道线的属性
        uint32_t u32PointNums;     // 这条线有几个点
        STAlgPoint aPoint[100];
    };

    /***********************
     * 检测算法参数
     ***********************/
    struct STAlgParam
    {
        float fScore;  // (默认0.52) 得分阈值
        float fNMS;  // (默认0.45) nms阈值

        uint32_t u32Width;  // (返回值)算法要求的图像的宽
        uint32_t u32Height;  // (返回值)算法要求的图像的高
        uint32_t u32Channel;  // (返回值)算法要求的图像的通道数

        // 默认值
        STAlgParam():fScore(0.52),fNMS(0.45){}
        STAlgParam(float set_score, float set_nms):fScore(set_score),fNMS(set_nms){}
    };



    /***********************
     * 跟踪算法参数
     ***********************/
    struct STTrackParam
    {
        float fScore;  // (默认0.52) 检测得分大于此得分才跟踪
        uint32_t u32Streak;  // (默认3) 连续检到几帧, 才会被认为是真实的目标, 才会进行跟踪
        uint32_t u32Age;  // (默认3) 连续多少帧没匹配到, 就从跟踪集合里踢出
        uint32_t u32Window;  // (默认4) 时间窗口, 在最近的几帧内判定命中次数
        uint32_t u32Hits;  // (默认2) 时间窗口内的命中次数, 大于这个次数才显示
        uint32_t u32MaxTrackNum;  // (默认2)最多跟踪几个目标

        // 默认值
        STTrackParam():fScore(0.52),u32Streak(3),u32Age(3),u32Window(5),u32Hits(3),u32MaxTrackNum(5){}

        // 用户指定
        STTrackParam(float score, uint32_t s, uint32_t a, uint32_t w, uint32_t h, uint32_t n):\
                          fScore(score),u32Streak(s),u32Age(a),u32Window(w),u32Hits(h),u32MaxTrackNum(n){}
    };


    /***********************
     * 超车报警的参数
     ***********************/
    struct STOverTakeParam
    {
        EAlgCamViewType eViewType;  // 哪个视角的相机
        int32_t s32Delay;  // 延时帧数,默认5. (卡尔曼滤波需要累积几帧才准确, 早期输出结果不准确且不稳定, 为了减少误判, 延迟几帧较好)
        float fChangingTHR;  // 变化率阈值, 大于此才报警
        float fMinX;  // 0~1之间的值, (在此范围内的框才执行策略)
        float fMaxX;  // 0~1之间的值, (在此范围内的框才执行策略)
        float fMinY;  // 0~1之间的值, (在此范围内的框才执行策略)
        float fMaxY;  // 0~1之间的值, (在此范围内的框才执行策略)
        float fdistance; // 0.05  边界线
        int32_t s32frameNum; // 超车报警判断帧数
        float fspeed;     // gps speed

        // 默认值
        STOverTakeParam():eViewType(E_CAM_VIEW_FORWARD),s32Delay(5),fChangingTHR(-0.0008),fMinX(0),fMaxX(1),fMinY(0),fMaxY(1),fdistance(0.05),s32frameNum(7),fspeed(0){}

        // 用户指定
        STOverTakeParam(EAlgCamViewType eType, int32_t s32d, float fth, float fx1, float fx2, float fy1, float fy2,float fdistance_in,int fframe_in,float fspeed_in):\
                            eViewType(eType), s32Delay(s32d), fChangingTHR(fth), fMinX(fx1), fMaxX(fx2), fMinY(fy1), fMaxY(fy2), fdistance(fdistance_in),\
                            s32frameNum(fframe_in), fspeed(fspeed_in){}
    };

    /***********************
     * 人头计数的参数
     * 基本假设: 单看画面：从下往上就叫上车, 从上往下就叫下车
     ***********************/
    struct STCountHeadParam
    {
        bool bClean;  // 是否清零
        float fTrackScore;  // 大于此得分的框才计数, 默认 0.
        unsigned int u32CountFrames;  // 统计最近?帧的平均值(越多越准确, 但是越不灵敏, 最大值为4, 建议设为3, 默认为2)
        EAlgHeadLineType eMode;  // 模式: [0]横装, 超过Y轴横线就计数(人是上下移动). [1]竖装, 超过X轴竖线就计数(人是左右移动)
        EAlgHeadTrackType eTrackMode;  // 跟踪模式, 见结构体注释
        EAlgHeadCountType eCountMode;  // 计数方式, 见结构体注释
        STAlgPoint fRectScope1;  // 矩形计数区域的左上角坐标 (相对于原图, 归一化的, 0~1 之间)
        STAlgPoint fRectScope2;  // 矩形计数区域的右下角坐标 (相对于原图, 归一化的, 0~1 之间)
        float fLinePosition;  // 线的位置, 超过这条线的人头就计数 (相对于原图, 归一化的, 0~1 之间)
        uint32_t u32InNums;  // 总上车人数
        uint32_t u32OutNums;  // 总下车人数
        uint32_t u32ThisTimeIn;  // 本次上了几个人 (根据当前帧的跟踪结果算出来的)
        uint32_t u32ThisTimeOut;  // 本次下了几个人 (根据当前帧的跟踪结果算出来的)
        bool bReverse;  // 是否反转, 默认是False, 也就是不反转, 也就是正装, 也就是往上移动的为上车, 往下移动的为下车。
                        // 这个主要影响关门信号对上车的人的判断, 反转后应用层要交换 u32InNums 和 u32OutNums 的值

        // 默认值
        STCountHeadParam():bClean(false),fTrackScore(0.5),u32CountFrames(2),eMode(E_COUNT_HEAD_Y),eTrackMode(E_COUNT_TRACK_FULL_SCREEN),eCountMode(E_COUNT_ONCE),\
                           fRectScope1(STAlgPoint(0.2,0.2)),fRectScope2(STAlgPoint(0.8,0.8)),\
                           fLinePosition(0.5),u32InNums(0),u32OutNums(0),bReverse(false){}

        // 用户指定
        STCountHeadParam(bool c, float s, uint32_t f, EAlgHeadLineType m, EAlgHeadTrackType tm, EAlgHeadCountType ct, \
                         STAlgPoint p1, STAlgPoint p2, float x, uint32_t in, uint32_t out, bool bR):\
                         bClean(c), fTrackScore(s), u32CountFrames(f), eMode(m), eTrackMode(tm), eCountMode(ct), fRectScope1(p1),\
                         fRectScope2(p2), fLinePosition(x), u32InNums(in), u32OutNums(out), bReverse(bR){}
    };

    /***********************
     * 后处理的的参数
     ***********************/
    struct STPostParam
    {
        STTrackParam stTrackParam;  // 跟踪参数
        STOverTakeParam stOverTakeParam;  // 超车报警的参数
        STCountHeadParam stCountHeadParam;  // 人头计数的参数
    };

    /***********************
     * 检测结果 (注意:改了这里记得改跟踪的头文件)
     ***********************/
    typedef struct STAlgResult_
    {
        EAlgObjectClass classes;  // 类别, [0]人; [1]车;

        // 该结构体的坐标值, 是基于长宽为1的框, 再乘以实际图像的宽或者高才是实际坐标
        float  fX1;	   // 左上角x坐标, 区间[0,1]
        float  fY1;    // 左上角y坐标, 区间[0,1]
        float  fX2;    // 右下角x坐标, 区间[0,1]
        float  fY2;    // 右下角y坐标, 区间[0,1]
        float  fConfidence;   // 目标的置信度
        float  fDistance;     // 距离, 单位mm, 被检测的目标与相机之间的距离
        float  fArea;  // 框的面积, 也是用归一化坐标算出来的, 要乘以原图的宽和高才是实际面积
        uint32_t u32ID;  // 跟踪的id号
        
        bool bOverTake;  // 是否超车. 危险, 请远离

        // 交通标志相关数据
        STAlgUSSign stUSSign;  // 美国标志相关数据, 当 EAlgObjectClass 为 E_SIGN_US 时有效
        uint32_t u32SpeedLimit;  // 限速值, 当 EAlgObjectClass 为 E_SIGN_SPEED_CHN 时有效
        float fHightLimit;  // 限高值, 当 EAlgObjectClass 为 E_SIGN_HIGHT 时有效

        // 保留
        float   reserve1;
        float   reserve2;
        int32_t reserve3;
        int32_t reserve4;

    }STAlgResult;

    /***********************
     * 超分辨率的结果
     ***********************/
    struct STAlgSRResult
    {
        unsigned char *pResultData;  // 指向超分辨率的结果, 数据排列是RBGRGBRGBRGB, 内存是算法库里面分配的
        int32_t s32OutImgWidth;
        int32_t s32OutImgHeight;
        int32_t s32OutImgChannel;
    };

    /***********************
 * 车牌检测的结果
 ***********************/
    typedef struct  STAlgPLATEDETResult_
    {
        EAlgObjectClass classes;  // 类别, [29]单行的车牌; [30]双行的车牌;

        // 该结构体的坐标值, 是基于长宽为1的框, 再乘以实际图像的宽或者高才是实际坐标
        float  fX1;	   // 左上角x坐标, 区间[0,1]
        float  fY1;    // 左上角y坐标, 区间[0,1]
        float  fX2;    // 右下角x坐标, 区间[0,1]
        float  fY2;    // 右下角y坐标, 区间[0,1]
        float  fConfidence;   // 目标的置信度
        float  fDistance;     // 距离, 单位mm, 被检测的目标与相机之间的距离
        float  fArea;  // 框的面积, 也是用归一化坐标算出来的, 要乘以原图的宽和高才是实际面积
        uint32_t u32ID;  // 跟踪的id号

//        bool bOverTake;  // 是否超车. 危险, 请远离

        // 车牌的四个角的角点
        float  fCorn1X;
        float  fCorn1Y;
        float  fCorn2X;
        float  fCorn2Y;
        float  fCorn3X;
        float  fCorn3Y;
        float  fCorn4X;
        float  fCorn4Y;

        // 保留
        float   reserve1;
        float   reserve2;
        int32_t reserve3;
        int32_t reserve4;
    }STAlgPLATEDETResult;
    /***********************
 * 车牌识别的结果
 ***********************/
    typedef struct  STAlgPLATERECResult_
    {
        //std::string strOCRRet;
        char strOCRRet[100];
    }STAlgPLATERECResult;


    // #define MAXTARGET 50
    typedef struct STAlgInfo_
    {
        uint32_t u32Nums;			// 检测到的目标数量
        STAlgResult stResults[100];	// 目标信息
        
        uint32_t u32LaneNums;   // 车道线的数量
        STAlgLane aLane[10];    // 车道线

        STAlgSRResult stSRResult;  // 超分辨率的结果

        STAlgPLATEDETResult stPlateDetResults[SV_MAX_OUTPUT_PLATE_DET_NUM];// 车牌的结果
        STAlgPLATERECResult stPlateRecResult[SV_MAX_OUTPUT_PLATE_DET_NUM];// 车牌识别的结果

    }STAlgInfo;


    class CPdsAlg {
    public:
        /********************************************************************************
         * 构造函数
         * 
         * 输入: 
         *     type: 算法类型, 见结构体[EAlgType]
         ********************************************************************************/
        CPdsAlg(EAlgType type, bool bDebug=false);
        ~CPdsAlg();

        /********************************************************************************
         * 初始化函数
         * 输入: 
         *     pModelFile: 模型文件的路径, 注意要和'算法类型'对应
         *     parameter_InOut: 得分阈值, nms阈值
         *     pfCheckCallback: 某些给客户的版本需要验证 
         *     pModelFile2: ADAS算法有时候需要多个模型, 例如交通灯, 没有就填null
         *     pModelFile3: ADAS算法可能要用3个模型, 例如要另外用OCR对交通标志做字符识别, 没有就填null
         * 
         * 输出:
         *     parameter_InOut: 算法图片宽高
         *     
         * 返回值:
         *     见错误码    
         ********************************************************************************/
        int32_t AlgInit(char* pModelFile1, STAlgParam &parameter_InOut);

        int32_t AlgInit(char* pModelFile1, STAlgParam &parameter_InOut, ALG_CHECK_CALLBACK pfCheckCallback);

        int32_t AlgInit(char* pModelFile1, STAlgParam &parameter_InOut, ALG_CHECK_CALLBACK pfCheckCallback, char* pModelFile2);
                        
        int32_t AlgInit(char* pModelFile1, STAlgParam &parameter_InOut, ALG_CHECK_CALLBACK pfCheckCallback, char* pModelFile2, char* pModelFile3);

        /********************************************************************************
         * 郑南城封装的函数, 暂时不改
         ********************************************************************************/
        int32_t AlgInit(char** pModeFileList, STAlgParam &parameter_InOut, ALG_CHECK_CALLBACK pfCheckCallback)
        {
            int i, modeltotal;
            if(pModeFileList == NULL)
                return -1;

            for(i = 0, modeltotal = 0; i < 3; i++)
            {
                if(pModeFileList[i] == NULL)
                    break;
                modeltotal++;
            }

            switch(modeltotal)
            {
                case 1:
                    return AlgInit(pModeFileList[0], parameter_InOut, pfCheckCallback);
                    break;
                case 2:
                    return AlgInit(pModeFileList[0], parameter_InOut, pfCheckCallback, pModeFileList[1]);
                    break;
                case 3:
                    return AlgInit(pModeFileList[0], parameter_InOut, pfCheckCallback, pModeFileList[1], pModeFileList[2]);
                    break;
                default:            /* 模型文件均为空时,直接退出 */
                    return -1;
                    break;
            }
            return 0;
        }

        /********************************************************************************
         * 跑神经网络
         * 输入: 
         *     p_inputdata: RGB888图像数据的地址
         *                  输入尺寸: [608x352x3] 可见光, 检人
         *                            [608x352x3] 可见光, 检人车
         *                            [512x384x3] 红外, 检人车
         * 
         *     s32InputWidth: DMABuff的FD对应的图片的宽
         *     s32InputHeight: DMABuff的FD对应的图片的高
         *     s32DmaBuffFd: DMA Buff的FD, 用于0拷贝
         *     u32Channel: 第几路算法, 默认0(AIbox这种可能构造1次算法,用这个算法类跑多个通道,在这里标识是第几路)
         *                                  AlgForward和AlgResult是一对原子操作, 同一路算法必须及时取结果, 否则会被覆盖
         * 输出:
         *     无
         *     
         * 返回值:
         *     见错误码          
         ********************************************************************************/
        int32_t AlgForward(char* p_inputdata, int32_t s32InputWidth, int32_t s32InputHeight, int32_t s32DmaBuffFd=0, uint32_t u32Channel=0);
        //int AlgForward(int dmaFD);

        /********************************************************************************
         * 取结果, 在 AlgForward 成功后调用
         * 输入:
         *     ePostCode:  后处理选项(是否开启跟踪, 用于消除误检)
         *     trk_param:  跟踪的参数
         *     channel:    摄像头通道号, [0,4], 用于区分跟踪
         *     
         * 
         * 输出:
         *     ptsResult: 检测框的信息
         *     
         * 返回值:
         *     见错误码           
         ********************************************************************************/
        int32_t AlgResult(STAlgInfo *pstResult, EAlgPostCode ePostCode=E_POST_NONE, STTrackParam trk_param=STTrackParam(), uint32_t u32Channel=0);

        /********************************************************************************
         * 后处理, 在获取检测结果后使用, 主要是一些策略, 例如:
         * 1. 消除一闪而过的误检; 
         * 2. 超车报警的策略;
         * 3. 人头计数;
         * 4. 通过跟踪给目标分配一个ID, 以便持续标识某个物体
         * 5. 人数统计的巴士车关门了
         * 
         * 输入:
         *     ptsResult:   检测框的信息
         *     ePostCode:   后处理选项, 选择哪个后处理
         *     stPostParam: 后处理参数
         * 
         * 输出:
         *     ptsResult: 检测框的信息
         *     
         * 返回值:
         *     见错误码           
         ********************************************************************************/
        int32_t AlgPostProcess(STAlgInfo *pstResult, EAlgPostCode ePostCode, STPostParam &stPostParam, uint32_t u32Channel=0);

        /********************************************************************************
         * 获取当前算法库版本
         * 输入:
         *     无
         * 
         * 输出:
         *     无
         *     
         * 返回值:
         *     版本字符串指针           
         ********************************************************************************/
        const char* AlgLibVersion();

        /********************************************************************************
         * 获取当前算法模型版本
         * 输入:
         *     无
         * 
         * 输出:
         *     无
         *     
         * 返回值:
         *     版本字符串指针           
         ********************************************************************************/
        const char* AlgModelVersion();

    private:
        
        class m_cInner;
        class m_cInner* m_pcInner=nullptr;   

        void releaseMem();
    };




    /******************************************************
     * 其它功能函数
     *****************************************************/

    /**********************************************************
     * 计算碰撞时间TTC
     * 输入：
     *     fInstallHeight: 相机安装高度(米)
     *     fGPSSpeed:      GPS速度(Km/h)
     *     fALGHeight:     当前算法输入图像的高(多少像素)
     *     fY2:            检测框底边, 与检测算法返回的 STAlgResult.fY2 的值相同, 是0~1之间的小数, 相对坐标
     * 
     * 返回:
     *     碰撞时间(s)  取小数点后两位
    **********************************************************/
    float getTTC(float fInstallHeight, float fGPSSpeed, float fALGHeight, float fY2);

    /**********************************************************************
    此函数丢弃
    功能： 对ADAS相机ROI区域内的地面目标求距离，由安装高度、目标框Y像素、消失线Y像素等值和几何关系求得近似值，现暂时此功能用于ISAAC2.8mm相机，以后其他相机待实验和版本更新;
    输入参数： 
    fImgYsize: （0-1080之间的数） 
                图像的高度，程序内部输入，目标是判断中心点Y值，由于现产品光心存在浮动，所以采用理想值高度的一半，因此需要输入高度;
    fInstallH: 相机的安装高度，客户外部输入，对误差的影响次于消失线像素真值；若消失线像素完全对齐，±10cm高度误差造成的测距误差可接受，但是考虑到不可能完全对齐，因此希望保持到
            ±4cm范围，取值±4是因为ISAAC相机壳体高度7cm.
    fVanishY:  消失线像素Y值，为了减少客户操作失误，已建议改取线上边值为取小宽度奇数中心值，初步选定5宽度；
            实际使用时，应为客户限制上下调节极限，目标为了使得客户的安装工况不过于上仰或下俯，过于上仰会盲区大天空占比大，过于下俯会导致地面占比过大；
            暂定上下限为中心线上下1/8，放在应用层外部以方便调节；后续可增添自动标定以减少手动标定的误差。
    fTargetY:  检测算法检测框下边沿的Y值，实际会有浮动等像素级的误差，仅在测距中影响小;注意此值应该比fVanishY大，否则不应测距。
    fYPixelVauleofFocalLength: 相机内参值f/dy，标定得5个相机此值偏差小，对于ISAAC2.8mm相机使用用平均值1059，放在外部应用层输入已方便更换相机时外部调节。           
    输出参数：
    fTargetDistance: ROI区域内的目标与相机的距离，目标需满足地平线假设等;
    应用： 外部函数，调用以获得目标距离，可用于后续的车距监测和碰撞预警。
    注：关于参数个数检查由于内部使用不做，参数数值合法性问题如下，fImgYsize图像像素高度；fInstallH的输入单位与输出的距离单位相等；fVanishY由标定获得，其上下限放在外面以易调节；
    fTargetY应大于fVanishY，否则不应传参；fYPixelVauleofFocalLength对于ISSAC2.8mm相机固定值1059，放在外部输入以便于以后更换相机.
    ***********************************************************************/
    float GetTargetDistance(float fImgYSize, float fInstallH, float fVanishY, float fTargetY, float fYPixelValueofFocalLength);

    /**********************************************************************
    * 此函数丢弃
    反算，GetTargetDistance的反函数，用于剪裁摄像头到车前保险杆之间的ROI 
    
    ***********************************************************************/
    float GetTargetY(float fImgYSize, float fInstallH, float fVanishY, float fTargetDistance, float fYPixelValueofFocalLength);

    /**********************************************************************
     * 此函数丢弃
     
     * 计算两个多边形的相交的面积, 其实里面就是调用了 OpenCV 的 cv::intersectConvexConvex
     * 输入:
     *     vecPoints1: 第一个多边形上的点
     *     vecPoints2: 第二个多边形上面的点
     * 输出:
     *     两个多边形的交面积(失败或不相交则返回0)
    ***********************************************************************/
    float InterArea(std::vector<STAlgPoint> vecPoints1, std::vector<STAlgPoint> vecPoints2);

}








#endif //ADAS32_ALGLIB_H
