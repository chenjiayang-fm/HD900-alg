#ifndef COMMON_H_
#define  COMMON_H_
#include <opencv2/opencv.hpp>

#include <fstream>
#include <malloc.h>
#include <iostream>

#include<stdio.h>
#include<stdlib.h>
#include <string.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>

#include "rknn_api.h"
#include <system_error>
#include <map>

#include <omp.h>
#include "algopcode.h"
#include <print.h>
#define pi 3.141592654

struct faceParam
{
    cv::Rect cFaceRoi; // 人脸区域
    cv::Rect cRPhoneRoi; // 右边打电话区域
    cv::Rect cLPhoneRoi; // 左边打电话区域
    cv::Rect cDrinkEatRoi; // 吃喝东西区域
    cv::Rect cMaskRoi;  //口罩区域
    cv::Rect cSmokeRoi;  //香烟检测区域
    cv::Rect cSmokeRoiT2; //抽烟手势检测区域
    cv::Rect cEyeRoi;    //眼睛检测区域（暂时没有用到）
    cv::Rect cSeatbeltRoi;  //安全带检测区域
    cv::Rect cHelmetRoi; //安全帽检测区域
    float fScaling;    //人脸对齐缩放比例
    float Ppoints[66];	  //人脸特征点坐标（局部坐标）
    float pfPose[3];      //头部三维姿态（经过矫正）
    int fXoffset;		//人脸区域x坐标
    int fYoffset;        //人脸区域y坐标
    float fAngle;     //人脸旋转角度
};

struct dmmParam
{
    bool detectFace; //是否检测到人脸
    float Ppoints[66]; //人脸特征点坐标（全局坐标）
    float ECR; //眼睛闭合度（暂时没用到）
    float ShowECR;   //眼睛闭合度
    float HeadPose[3]; //头部三维姿态，分别是俯仰角pitch，水平角yaw，旋转角roll
    float Smoke; //抽烟检测分数
    float Phone; //打电话检测分数
    float fDrinkEat; //吃喝东西检测分数
    bool Yawn;//是否打哈欠
    bool NoMask;//是否带口罩
    cv::Rect showFaceRoi; //人脸区域
    float pfEyeScore[2]; //眼睛状态检测分数
    float pfGlassScore[2];//眼镜类型检测分数
    float seatbeltScore;//安全带检测分数
    float fHelmetScore;//安全帽检测分数
    bool shelter;//是否遮挡
    float pfGaze2d[2]; //视线方向，分别是yaw，pitch
};


struct frParam
{
    cv::Mat* faceImgPtr;
    float HeadPose[3];
    float feature[512];
    bool feature_val;
    char userPath[512];
};

enum {
    RUNNING_STATE_INIT=0,
    RUNNING_STATE_FREE,
    RUNNING_STATE_DETECTION,
    RUNNING_STATE_CALIBRATION_PRE,
    RUNNING_STATE_CALIBRATION,
    RUNNING_STATE_REGISTER,
    RUNNING_STATE_RECOGNITION,
    RUNNING_STATE_MAX
};
typedef enum {
    E_DMS31 = 0,
    E_ADA42 = 1

} EApplyType;

typedef enum {
    E_GENERAL = 0,
    E_EXHIBITION = 1
} EVersionType;

struct STYOLORegion
{
    int32_t s32OffsetX;   /*YOLO补边的X偏置*/
    int32_t s32OffsetY;   /*YOLO补边的Y偏置*/
    float fScale;         /*缩放比例*/
    cv::Mat cRoi;         /*预处理完的区域*/

    /*构造函数*/
    STYOLORegion():s32OffsetX(0),s32OffsetY(0),fScale(1.0){}
    STYOLORegion(int32_t s32Width, int32_t s32Height):s32OffsetX(0),s32OffsetY(0),fScale(1.0)
    {
        cRoi = cv::Mat(s32Height, s32Width, CV_8UC1,cv::Scalar(0));
    }
};
#endif
