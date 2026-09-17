
#ifndef _ADAS32_ALG_PDS_H_
#define _ADAS32_ALG_PDS_H_

#include "stdlib.h"
#include "stdio.h"


typedef struct ALGPDS_RECT_
{
    // 该结构体的坐标值, 是基于长宽为1的框, 再乘以实际图像的宽或者高才是实际坐标
    /* The coordinate value of the structure is based on a box with a length and width of 1, 
     *  and then multiplied by the width or height of the actual image to get the actual coordinates 
     */
    float  x1;	// 左上角x坐标, 区间[0,1]    //The x coordinate of the upper left corner, interval [0,1] 
    float  y1; 	// 左上角y坐标, 区间[0,1]    //The y coordinate of the upper left corner, interval [0,1] 
    float  x2; 	// 右下角x坐标, 区间[0,1]    //The x coordinate of the lower right corner, interval [0,1] 
    float  y2; 	// 右下角y坐标, 区间[0,1]    //The y coordinate of the lower right corner, interval [0,1] 
    float  confidence;   // 目标的置信度  //Target confidence 
    float  distance;     // 距离, 单位mm, 被检测的目标与相机之间的距离    //Distance, in mm, the distance between the detected target and the camera 
    float  reserve1;  // 保留位
    unsigned short classes;		// 类别, [0]人; [1]车;  // Category, [0] people; [1] cars; 
    // int id;
}ALGPDS_RECT_S;

#define MAXTARGET 50

typedef struct ALG_PDS_INFO_S
{
    int num;			// 检测到的目标数量     // Number of detected targets 
    ALGPDS_RECT_S Rect_t[MAXTARGET];	// 目标信息     // Target information 
}ALGPDS_INFO_S;


/*******************************************************
 * 获取当前算法的版本      * Get the version of the current algorithm 
********************************************************/
const char* ALGPDS_LibVersion();  // 库的版本   // The version of the library 
const char* ALGPDS_ModelVersion();  // 模型的版本  // Model version 

/*******************************************************
 * 初始化函数，包括nnie及相关资源与配置 * Initialization function, including nnie and related resources and configuration 
 * 输入：
 *    threshold				： 行人检测相关的若干阈值,  // Several thresholds related to pedestrian detection
 *                             [得分阈值, NMS阈值, 跟踪阈值, 跟踪帧数]    // Score threshold, NMS threshold, tracking threshold, tracking frame number 
 *    numberOfThresholds	:  传入阈值的个数，
 *    ptModelFile           :  模型文件路径   // Model file path 
********************************************************/
int ALGPDS_init(float *thresholds, int numberOfThresholds, char* ptModelFile);

/*******************************************************
 * 该函数为算法向前推断函数，阻塞，直到推断过程结束才返回。 * This function is a function for the algorithm to infer forward, blocking, and returning until the end of the inference process 
 * 输入:  RGBRGBRGB...RGB图片的指针, U8C3类型, 当前尺寸为 608x352x3 (宽x高x通道) 
 * 
********************************************************/
int ALGPDS_forward(char* p_inputdata);

/*******************************************************
 * 获取算法前推结果 * Get algorithm forward results 
 * 输入&输出: 
 *           ptsResult:算法检测结果
 * 
 *           bTrack: 是否使用跟踪模块
********************************************************/
int ALGPDS_get_result(ALGPDS_INFO_S *ptsResult, bool bTrack=false);

/*******************************************************
 * 释放算法库的资源
********************************************************/
int ALGPDS_release();

/*------------------ 打印算法库的信息-------------------
int flag  设置1，打印算法库的调试信息；默认是0
*/
void ALGPDS_debug_print(bool flag);
#endif // _ADAS32_ALG_PDS_H_
