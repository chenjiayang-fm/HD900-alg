/******************************************************************************
  File Name     : OpticalFlowCommon.h
  Version       : 1.0.0
  Author        : Xie jun
  Created       : 2025-02-19
  Last Modified : 2025-03-26
  Description   : 存放光流库各类结构体，枚举等
  Function List :
  History       : 
  日期，修改内容：  2025-03-26：增加栈板检测等参数的结构体
  日期，修改内容：  2025-04-30: 增加判断出叉臂上下后，光流暂停时间和运行时间
******************************************************************************/
#ifndef OPTICALFLOW_COMMON_H
#define OPTICALFLOW_COMMON_H
#include <iostream>
#include <stdio.h>
#include <stdint.h>

namespace opticalflowanalyzeralg
{
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
    typedef enum ALG_PROCESS_CODE_E {
        /********************************RKNN的错误码***********************************************/
        E_ALG_SUCCESS                                     = 0,    // 成功
        E_ALG_ERR_FAIL                                    = -1,   // 执行失败
        E_ALG_ERR_TIMEOUT                                 = -2,   // 线性库优化失败
        E_ALG_ERR_DEVICE_UNAVAILABLE                      = -3,   // 设备不可访问
        E_ALG_ERR_MALLOC_FAIL                             = -4,   // 内存创建失败
        E_ALG_ERR_PARAM_INVALID                           = -5,   // 参数无效
        E_ALG_ERR_MODEL_INVALI                            = -6,   // 模型无效
        E_ALG_ERR_CTX_INVALID                             = -7,   // context无效
        E_ALG_ERR_INPUT_INVALID                           = -8,   // 输入无效
        E_ALG_ERR_OUTPUT_INVALID                          = -9,   // 输出无效
        E_ALG_ERR_DEVICE_UNMATCH                          = -10,  // 设备不匹配, 需要更新RKNN的sdk和npu的驱动和固件
        E_ALG_ERR_INCOMPATILE_PRE_COMPILE_MODEL           = -11,  // 加载的RKNN模型是预编译的, 当前驱动不支持这种模式
        E_ALG_ERR_INCOMPATILE_OPTIMIZATION_LEVEL_VERSION  = -12,  // 加载的RKNN模型设置了优化级别, 当前驱动不支持这种模式
        E_ALG_ERR_TARGET_PLATFORM_UNMATCH                 = -13,  // 加载的RKNN模型和当前运行平台不符合
        E_ALG_ERR_NON_PRE_COMPILED_MODEL_ON_MINI_DRIVE    = -14,  // 加载的RKNN模型不是预编译的, 当前用的mini driver跑不了
        /********************************RKNN的错误码***********************************************/


        /********************************本算法库的错误码***********************************************/
        E_ALG_ERR_RE_INIT                                 = -20,  // 请不要重复初始化
        E_ALG_ERR_MODEL_NULL                              = -21,  // 初始化时, 模型文件的指针为空, 请正确设置参数
        E_ALG_ERR_LOAD_MODEL                              = -22,  // 初始化时, 加载模型文件失败
        E_ALG_ERR_MUST_INIT                               = -23,  // 需要先进行初始化
        E_ALG_ERR_TYPE_UNMATCH                            = -24,  // 加载的模型与设置的算法类型不符合, 例如设置的是只检人的模型, 却加载了检人检车的模型 
        E_ALG_ERR_TYPE_UNSUPPORT                          = -25,  // 不支持的算法模型 
        E_ALG_ERR_ALG_VERSION                             = -26,  // 算法版本问题, 需要联系开发者进行联调,确认,Debug
        E_ALG_ERR_DMA_BUFFER                              = -27,  // 申请DMAbuffer错误
        E_ALG_ERR_ZERO_COPY_NO_FD                         = -28,  // 零拷贝模式(加载模型后识别到是零拷贝的模型), 但是没有DMAbuffer的文件描述符, 无法获取硬件地址和使用零拷贝, 要么换个不是零拷贝的模型
        E_ALG_ERR_CALLBACK_NULL_PTR                       = -29,  // 回调函数为空
        E_ALG_ERR_AUTHENTICATE                            = -30,  // 算法认证失败
        E_ALG_ERR_FILE_NOT_EXIST                          = -31,  // 模型文件不存在
        E_ALG_ERR_ALG_INIT_FLOW                           = -32,  // 光流模型初始化失败
        /********************************本算法库的错误码***********************************************/

        E_ALG_ERR_CFG_PATH                                = -40,  //  配置文件不存在
        E_ALG_ERR_MUST_IMPORT                             = -41,  //  需要先导入文件

    }EAlgProcessCode;


    /***********************
     * 算法参数
     ***********************/
    struct STAlgParam // 定义模型参数的结构体
    {
        uint32_t u32Width;    // (返回值)算法要求的图像的宽
        uint32_t u32Height;   // (返回值)算法要求的图像的高
        uint32_t u32Channel;  // (返回值)算法要求的图像的通道数
    };

    /***********************
     * 算法类型
     ***********************/
    typedef enum ALG_TYPE_E {

        E_ALG_TYPE_DIRECTION_JUDGE   = 0,         // 方向判断 
        E_ALG_TYPE_PALLET_DETECT     = 1,         // 栈板检测
        // E_ALG_TYPE_REVERSE                  /* 暂未开发.... */

    } EAlgType;
    

    /****************************************
     * 算法模型路径
     ****************************************/
    typedef struct ST_ALG_MODEL_PATH {

        char * pModelPath;      //  模型的路径

        //  默认值
        ST_ALG_MODEL_PATH():pModelPath(nullptr){}
        ST_ALG_MODEL_PATH(char* p1): pModelPath(p1){}

    }STAlgModelPath;


    /****************************************
     * 算法模型版本
     ****************************************/
    typedef struct ST_ALG_MODEL_VERSION {
        char* pModelVersion;      //  模型版本
        
        //  默认值
        ST_ALG_MODEL_VERSION():pModelVersion(nullptr){}
        ST_ALG_MODEL_VERSION(char* p1): pModelVersion(p1){}
        
    }STAlgModelVersion;


    /****************************************
     * 光流结果指针
     ****************************************/
    typedef struct ST_OPTICAL_FLOW_RESULT 
    {
        float * pfOpticalFlowData;   // 光流数据指针
        ST_OPTICAL_FLOW_RESULT():pfOpticalFlowData(nullptr){}
        ST_OPTICAL_FLOW_RESULT(float *p1):pfOpticalFlowData(p1){}

    } STOpticalFlowResult;


    /****************************************
     * 车辆状态编码
     ****************************************/
    typedef enum E_VEHICLE_STATE_CODE 
    {
        E_STATE_STATIONARY             = 0,       // 静止
        E_STATE_MOVE_FORWARD_STRAIGHT  = 1,       // 笔直前进
        E_STATE_MOVE_FORWARD_LEFT      = 2,       // 左前
        E_STATE_MOVE_FORWARD_RIGHT     = 3,       // 右前
        E_STATE_MOVE_BACKWARD_STRAIGHT = 4,       // 笔直后退
        E_STATE_MOVE_BACKWARD_LEFT     = 5,       // 左后
        E_STATE_MOVE_BACKWARD_RIGHT    = 6,       // 右后
        E_STATE_ROTATE_LEFT            = 7,       // 左旋 
        E_STATE_ROTATE_RIGHT           = 8,       // 右旋
        E_STATE_FORKARM_RAISE          = 9,       // 叉臂上升
        E_STATE_FORKARM_LOWER          = 10,      // 叉臂下降
        E_STATE_WRONG                  = 11,      // 光流错误
    } EVehicleStateCode;


    /****************************************
     * 车辆类型编码，包括常规车辆和叉车（暂时），用于避免常规车辆出现叉臂上下的误检
     ****************************************/
    typedef enum E_VEHICLE_TYPE
    {
        E_VEHICLE_TYPE_NORMAL = 0,   // 除了叉车外的常规车款
        E_VEHICLE_TYPE_FORKLIFT,     // 叉车
    } EVehicleTypeCode;


    /****************************************
     * 相机安装方向，包括前装，后装，侧装(左右)
     ****************************************/
    typedef enum E_CAMERA_MOUNT_CODE
    {
        E_CAMERA_MOUNT_FRONT      = 0,     // 前装
        E_CAMERA_MOUNT_SIDE_LEFT  = 1,     // 侧装(左)
        E_CAMERA_MOUNT_SIDE_RIGHT = 2,     // 侧装(右)
        E_CAMERA_MOUNT_REAR       = 3,     // 后装
    } ECameraMountCode;


    /****************************************
     * 角点，在画面中的位置，单位是像素
     ****************************************/
    typedef struct ST_POINT_2F{
        float fX;
        float fY;
        ST_POINT_2F():fX(0), fY(0){}
        ST_POINT_2F(float _fX, float _fY): fX(_fX), fY(_fY){}
    } STPoint2f;


    /****************************************
     * 五档报警的范围，单位像素
     ****************************************/
    typedef struct ST_ALARM_RANGE
    {
        uint32_t u32Range1 = 10;    // 范围最小，等级最高
        uint32_t u32Range2 = 20;    // 范围中间，二级报警
        uint32_t u32Range3 = 30;    // 范围最大，等级最低


        // 默认值
        ST_ALARM_RANGE():u32Range1(10), u32Range2(20), u32Range3(30){}
        ST_ALARM_RANGE(uint32_t r1,uint32_t r2,uint32_t r3,uint32_t r4, uint32_t r5):u32Range1(r1), u32Range2(r2),u32Range3(r3){}

    }STAlarmRange;


    /****************************************
     * 报警程度
     ****************************************/
    typedef enum E_ALARM_LEVEL
    {
        E_ALARM_RANGE_0 = 0,       //  不报警 
        E_ALARM_RANGE_1 = 1,       //  一级（最低）
        E_ALARM_RANGE_2 = 2,
        E_ALARM_RANGE_3 = 3,       //  三级（最高）

    }EAlarmLevel;


    /****************************************
     * 栈板检测算法结构体，用于参数导入和导出
     ****************************************/
    typedef struct ST_ALG_PARAMS
    {
        bool bDebug = false;                      // 是否显示中间输出
        
        /********************************光流参数***********************************************/
        STAlgModelPath stOpticalFlowModelPath;    // 光流模型路径

        uint32_t u32Imgheight;                    // 图像高
        uint32_t u32Imgwidth;                     // 图像宽

        uint32_t u32Interval;                     // 光流点间隔
        uint32_t u32FlowFrameInterval;            // 光流间隔帧数
        float fFlowStationThreshold;              // 光流静止阈值
        float fFlowWrongThreshold;                // 光流错误阈值
        float fVanishPointX;                      // 光流消失点x
        float fVanishPointY;                      // 光流消失点y
        
        EVehicleTypeCode eVehicleTypeCode;        // 车辆类型枚举
        ECameraMountCode eCameraMountCode;        // 相机安装方位枚举
        /********************************光流参数***********************************************/


        /********************************栈板检测参数*******************************************/
        STAlgModelPath stPalletDetectModelPath;   // 栈板检测模型路径
        float fPalletDetectSensity;               // 栈板检测灵敏度
        float fNMSThreshold;                      // NMS阈值
        float fFLowPauseDuration;                 // 检测到栈板上下后，光流暂停间隔, 默认3s
        float fFlowRunDuration;                   // 检测到栈板上下后，光流持续运行时间，默认1s
        uint32_t u32CaliLine;                     // 栈板标定线
        uint32_t u32TurnoffTime;                  // 延迟关灯时间，20s
        STAlarmRange stAlarmRang;                 // 五级报警的范围
        /********************************栈板检测参数*******************************************/

        // 默认值
        ST_ALG_PARAMS():u32Interval(8), u32FlowFrameInterval(1), fFlowStationThreshold(1.0), fFlowWrongThreshold(40.0), eVehicleTypeCode(E_VEHICLE_TYPE_NORMAL), eCameraMountCode(E_CAMERA_MOUNT_FRONT){}


    }STAlgParams;

    /****************************************
     * 检测目标枚举
     ****************************************/
    typedef enum E_TARGET_CLASS
    {
        E_CLS_PERSON           = 0,   // 行人
        E_CLS_CAR              = 1,   // 车辆
        E_CLS_PALLET           = 2,   // 栈板

    }EAlgCommonObjectClass;


    /****************************************
     * 栈板检测算法结构体，用于参数导入和导出
     ****************************************/
    typedef struct ST_PALLET_DETECT_INFO
    {
        EAlgCommonObjectClass cls;           // 检测到的目标类别
        float fConfidence;                   // 置信度
        float fX1;                           // 左上角x坐标，区间[0,1]
        float fY1;                           // 左上角y坐标，区间[0,1]
        float fX2;                           // 右下角x坐标, 区间[0,1]
        float fY2;                           // 右下角y坐标, 区间[0,1]
        EAlarmLevel eAlarmLevel;             // 新增报警等级

    }STPalletDetectInfo;


    /****************************************
     * 每次运行光流算法返回的参数
     ****************************************/
    typedef struct ST_PALLET_DETECT_RESULT
    {
        bool bLampSwitch;                            // 是否开灯，false为关灯，true为开灯
        bool bDetectPallet;                          // 是否在检测栈板，false为没有检测，true为正在检测
        EAlarmLevel eAlarmLevel;                     // 报警档位
        uint32_t u32NumofObj;                        // 检测到的目标数量
        STPalletDetectInfo stPalletDetectInfo[100];  // 检测的目标信息

        // 默认值
        ST_PALLET_DETECT_RESULT():bLampSwitch(false), bDetectPallet(false), eAlarmLevel(E_ALARM_RANGE_0), u32NumofObj(0){}

    }STPalletDetectResult;

}

#endif