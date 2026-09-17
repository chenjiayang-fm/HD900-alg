/******************************************************************************
Copyright (C) 广州敏视数码科技有限公司版权所有.

文件名：camera_map.c

作者: 郑南城    版本: v1.0.0(初始版本号)   日期: 2020-08-26

文件功能描述: 真实物体三维坐标与相机像素坐标的映射

*******************************************************************************/
#include "camera_map.h"
#include "trace_common.h"
#include <math.h>

#define abs(x) ((x)<0? -(x) : (x))
#define sign(x) ( (x)<0? -1 : ( (x) > 0 ? 1 : 0 ) )

//摄像头内参
#define Camera_Fx   1.08062989e+03
#define Camera_Fy   1.08503068e+03
#define Camera_U0   1.02306962e+03
#define Camera_V0   5.16517689e+02

//摄像头外参
#define Camera_K1   -0.41056371
#define Camera_K2   0.2306753
#define Camera_K3   -0.08057446
#define Camera_P1   -0.00069162
#define Camera_P2   0.00054153

//相机两侧和前方位置偏移(x_bias_p, y_bias_n), (x_bias_n, y_bias_n)
double x_bias_p = 1.0;
double x_bias_n = -1.0;
double y_bias = 1.0;

//相机的俯仰角和偏航角
double roll_x = 0;
double pitch_y = 0;

//相机高度
double High = 1.0;

//画布信息
int canvas_offset_x = 0;
int canvas_offset_y = 0;


/********************************************************************************************
 * 功能：设置相机角度
 * 实现方法：
 * 输入参数：相机的俯仰角(弧度)和翻滚角(弧度)
 * 输出参数：
 * *******************************************************************************************
 */
void Camera_Angle_Set(double roll, double pitch)
{
    roll_x = -roll;
    pitch_y = -pitch;
}
/********************************************************************************************
 * 功能：设置画布信息
 * 实现方法：
 * 输入参数：画布的实际坐标 x, y
 * 输出参数：
 * *******************************************************************************************
 */
void Camera_Canvas_Set(int canvas_x, int canvas_y)
{
    canvas_offset_x = -canvas_x;
    canvas_offset_y = -canvas_y;
}
/********************************************************************************************
 * 功能：设置相机映射位置信息
 * 实现方法：
 * 输入参数：相机高度，相机定点x负偏，相机定点x正偏，相机定点y偏移
 * 输出参数：
 * *******************************************************************************************
 */
void Camera_Aim_Set(double _High, double _x_p, double _x_n, double _y)
{
    High = _High;
    x_bias_n = _x_p;
    x_bias_p = _x_p;
    y_bias = _y;
}

/********************************************************************************************
 * 功能：真实物理坐标到相机图片坐标系的映射，结果为像素坐标
 * 实现方法：旋转变换，投影，畸变变化和映射
 * 输入参数：相机的俯仰角和翻滚角
 * 输出参数：
 * *******************************************************************************************
 */
void Camera_Point_Map(pixel_point *point, double axis_x, double axis_y, double axis_z)
{
    double x_temp, y_temp, z_temp;
    double x_temp1,y_temp1;
    double r_2,r_4;

    x_temp = axis_x;
    y_temp = axis_y;
    z_temp = axis_z;

    /*
    //旋转变换: 由对地坐标系转为对摄像头坐标系，不考虑偏航角
    x_temp = cos(pitch_y)*axis_x-sin(pitch_y)*axis_z;
    y_temp = sin(roll_x)*sin(pitch_y)*axis_x + cos(roll_x)*axis_y + sin(roll_x)*cos(pitch_y)*axis_z;
    z_temp = cos(roll_x)*sin(pitch_y)*axis_x - sin(roll_x)*axis_y + cos(roll_x)*cos(pitch_y)*axis_z;
    */
    //2D投影
    x_temp = x_temp / y_temp;
    y_temp = z_temp / y_temp;

    //距离计算
    r_2 = x_temp*x_temp + y_temp*y_temp;
    r_4 = r_2 * r_2;

    //对坐标畸变换(因为拍摄的相机存在畸变，所以映射后的坐标也需要是畸变的坐标)
    x_temp1 = x_temp*(1+Camera_K1*r_2+Camera_K2*r_4) + 2*Camera_P1*x_temp*y_temp + Camera_P2*(r_2+2*x_temp*x_temp);
    y_temp1 = y_temp*(1+Camera_K1*r_2+Camera_K2*r_4) + Camera_P1*(r_2+2*y_temp*y_temp) + 2*Camera_P2*x_temp*y_temp;
    x_temp1 = x_temp;
    y_temp1 = y_temp;
    //映射为图片像素
    point->x_aixs = (int)(x_temp1*Camera_Fx + Camera_U0) + canvas_offset_x;
    point->y_axis = (int)(y_temp1*Camera_Fy + Camera_V0) + canvas_offset_y;
    return;
}

/********************************************************************************************
 * 功能：绘制在对地坐标系下的轨迹 到 在相机采集图片像素坐标系的映射
 * 实现方法：
 * 输入参数：像素的数组，像素的长度，起始角度(弧度)，终止角度(弧度)，
 *          轨迹半径，相机高度，x轴偏移，y轴偏移
 * 输出参数：
 * *******************************************************************************************
 */

void Camera_Trace_Map(pixel_point *points, int lens, double theta_start, double theta_end, double radius, double High, double x_bias, double y_bias)
{
    //轨迹半径角度以及步长
    double theta_step;
    double theta = theta_start;
    double x_temp,y_temp,z_temp;

    //防止半径过大，浪费资源
    if(radius>100)
    {
        theta_end = theta_start + M_PI/30;
    }
    
    theta_step = (theta_end-theta_start) / (lens-1);
    for(int i=0;i<lens;i++)
    {
        x_temp = radius*(1-cos(theta)) + x_bias;
        y_temp = abs(radius)*sin(theta) + y_bias;
        z_temp = High;
        Camera_Point_Map(&points[i], x_temp, y_temp, z_temp);
        theta += theta_step;
        
    }
    return;
}

/********************************************************************************************
 * 功能：绘制在对地坐标系车辆左侧倒车轨迹线 到 在相机采集图片像素坐标系的映射
 * 实现方法：
 * 输入参数：像素的数组，像素的长度，起始角度(弧度)，终止角度(弧度)，轨迹半径
 *          
 * 输出参数：
 * *******************************************************************************************
 */
void Camera_LeftTrace_Map(pixel_point *points, int lens, double theta_start, double theta_end, double radius)
{
    Camera_Trace_Map(points, lens, theta_start, theta_end, radius, High, x_bias_n,y_bias);
}

/********************************************************************************************
 * 功能：绘制在对地坐标系车辆右侧倒车轨迹线 到 在相机采集图片像素坐标系的映射
 * 实现方法：
 * 输入参数：像素的数组，像素的长度，起始角度(弧度)，终止角度(弧度)，轨迹半径
 *          
 * 输出参数：
 * *******************************************************************************************
 */
void Camera_RightTrace_Map(pixel_point *points, int lens, double theta_start, double theta_end, double radius)
{
    Camera_Trace_Map(points, lens, theta_start, theta_end, radius, High, x_bias_p,y_bias);
}


