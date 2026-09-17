/******************************************************************************
Copyright (C) 2021-2023 广州敏视数码科技有限公司版权所有.

文件名：targettrack.h

日期: 2022-02-08

文件功能描述: 定义跟踪算法功能接口

其他: // 其他内容说明

版本: v1.0.0(最新版本号)

*******************************************************************************/
#ifndef _TARGET_TRACK_H_
#define _TARGET_TRACK_H_
#include "../../../include/common.h"
#include "config.h"
#include "SiameseFCpp.h"

#define MAX_TARGET 50

/* 模块配置参数 */
typedef struct tag_TargetTrackCfg_S
{
    uint32          u32ChnNum;              /* 算法通道数目 */
    sint32          s32MediaBufFd;      	/* 媒体通道Media Buffer的文件描述符 */
    sint32          s32MediaBufChn;     	/* 媒体通道Media Buffer的对应通道 */
    CFG_ALG_PARAM   stAlgParam;             /* 算法配置参数 */
} TARGET_TRACK_CFG_PARAM_S;

typedef struct tag_TargetTrackAlgRect_S
{
    /* 该结构体的坐标值, 是基于长宽为1的框, 再乘以实际图像的宽或者高才是实际坐标 */
    float  fX1;         /* 左上角x坐标, 区间[0,1] */    
    float  fY1;         /* 左上角y坐标, 区间[0,1]        */ 
    float  fX2;         /* 右下角x坐标, 区间[0,1] */    
    float  fY2;         /* 右下角y坐标, 区间[0,1] */   
} TARGET_TRACK_Alg_S;

/* 算法结果参数 */
typedef struct tag_TargetTrackRect_S
{
    sint32 				s32Num;					/* 检测到的目标数量 */
    TARGET_TRACK_Alg_S 	stRect[MAX_TARGET];		/* 目标信息     */ 
} TARGET_TRACK_RECT_S;

/******************************************************************************
 * 函数功能: 初始化跟踪算法模块
 * 输入参数: pstInitParam --- 初始化配置参数
 * 输出参数: 无
 * 返回值  : SV_SUCCESS - 成功
             ERR_NULL_PTR - 传入参数指针为NULL
             SV_FAILURE - 失败
 * 注意    : 无
 *****************************************************************************/
extern sint32 TARGET_TRACK_Init(TARGET_TRACK_CFG_PARAM_S *pstInitParam);

/******************************************************************************
 * 函数功能: 去初始化跟踪算法模块
 * 输入参数: 无
 * 输出参数: 无
 * 返回值  : SV_SUCCESS - 成功
             SV_FAILURE - 其它错误
 * 注意    : 无
 *****************************************************************************/
extern sint32 TARGET_TRACK_Fini();

/******************************************************************************
 * 函数功能: 启动跟踪算法模块
 * 输入参数: 无
 * 输出参数: 无
 * 返回值  : SV_SUCCESS - 成功
             SV_FAILURE - 其它错误
 * 注意    : 
 *****************************************************************************/
extern sint32 TARGET_TRACK_Start();

/******************************************************************************
 * 函数功能: 停止跟踪算法模块
 * 输入参数: 无
 * 输出参数: 无
 * 返回值  : SV_SUCCESS - 成功
             SV_FAILURE - 其它错误
 * 注意    : 
 *****************************************************************************/
extern sint32 TARGET_TRACK_Stop();

/******************************************************************************
 * 函数功能: 手动更新目标位置
 * 输入参数: stTrackPos -- 初始跟踪目标位置
 * 输出参数: 无
 * 返回值  : SV_SUCCESS - 成功
             SV_FAILURE - 其它错误
 * 注意    : 
 *****************************************************************************/
extern sint32 TrackPosUpdate(PTZ_POS_S stTrackPos);

#endif  /* _TARGET_TRACK_H_ */