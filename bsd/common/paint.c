/******************************************************************************
Copyright (C) 广州敏视数码科技有限公司版权所有.

文件名：paint.c

作者: 郑南城    版本: v1.0.0(初始版本号)   日期: 2020-08-25

文件功能描述: 直线绘制和折线绘制

*******************************************************************************/

#include "paint.h"
//#include <math.h>
//#include <linux/fb.h>

//绝对值和符号函数
#define abs(x) ((x)<0? -(x) : (x))
#define sign(x) ( (x)<0? -1 : ( (x) > 0 ? 1 : 0 ) )


/********************************************************************************************
 * 功能：绘制直线图
 * 实现方法：通用Bresenham算法快速绘制直线
 * 输入参数：图像的一维数组(pbmp)，图像宽度(width)，图像高度(height)，
 * 输入参数：直线起始坐标(x_start, y_start), 直线终止坐标(x_end, y_end) 备注：坐标从零开始
 * 输入参数：直线颜色(color), 直线宽度(stick)
 * 输入参数：绘制成功标志,成功返回0，失败返回1
 * *******************************************************************************************
 */
int paint_Line_uint16(uint16*pbmp, int width,int height, pixel_point point_start, pixel_point point_end, uint16 color, int stick)
{
    int MAX_PIXEL = width * height - 1;         //总像素点
    int deltaX, deltaY;                         //直线的x,y坐标占宽
    int hatE;                                   //误差判断
    int sign1,sign2;                            //deltaX, deltaY正负判断
    int Interchange;                            //deltaX 和 deltaY调换判断
    int SV_SCUESS = 0;
    int half = stick / 2, hhalf = stick;     //绘制宽度，以(half<<1,hhalf<<1)绘制直线坐标点
    int i,j,k,temp,index;                       //中间变量

    int x_start, x_end, y_start, y_end;         //起点和终点值
    int output;
    
    x_start = point_start.x_aixs;
    y_start = point_start.y_axis;
    x_end = point_end.x_aixs;
    y_end = point_end.y_axis;

    deltaX = abs(x_end - x_start), deltaY = abs(y_end - y_start);
    sign1 = sign(x_end - x_start), sign2 = sign(y_end - y_start);

    
    //--------------------------------------------------------------------------------------------------------------------------------
    //绘制步长判断，选最小步长
    if(deltaY > deltaX)
    {
        temp = deltaX;
        deltaX = deltaY;
        deltaY = temp;
        Interchange = 1;
    }
    else{
        Interchange = 0;
    }
    hatE = 2*deltaY - deltaX;
    //--------------------------------------------------------------------------------------------------------------------------------


    //--------------------------------------------------------------------------------------------------------------------------------
    //通用Bresenham
    for(i=0;i<deltaX;i++)
    {
        //绘制以(x_start,y_start)为中心，长宽为(half<<1, hhalf<<1)的矩形
        for(j=0;j<hhalf;j++)
        {
            for(k=-half;k<half;k++)
            {   
                if(x_start+k >= width || x_start+k < 0)
                    continue;
                if(y_start + j >= height || y_start + j <0)
                    continue;
                index = x_start+k + (y_start + j)*width;
                pbmp[index] = color;
            }
            
        }
        
        //循环判断，生成坐标值(x_start,y_start)
        while(hatE > 0)
        {
            if(Interchange){
                x_start += sign1;
            }
            else{
                y_start += sign2;
            }
            hatE -= 2*deltaX;
        }
        if(Interchange)
        {
            y_start += sign2;
        }
        else
        {
            x_start += sign1;
        }
        hatE += 2*deltaY;

    }
    //--------------------------------------------------------------------------------------------------------------------------------

    return SV_SCUESS;
}


/********************************************************************************************
 * 功能：绘制虚线图
 * 实现方法：通用Bresenham算法快速绘制虚线
 * 输入参数：图像的一维数组(pbmp)，图像宽度(width)，图像高度(height)，
 * 输入参数：直线起始坐标(x_start, y_start), 直线终止坐标(x_end, y_end) 备注：坐标从零开始
 * 输入参数：直线颜色(color), 直线宽度(stick)
 * 输入参数：绘制成功标志,成功返回0，失败返回1
 * *******************************************************************************************
 */
int paint_DottedLine_uint16(uint16*pbmp, int width,int height, pixel_point point_start, pixel_point point_end, uint16 color, int stick)
{
    int MAX_PIXEL = width * height - 1;         //总像素点
    int deltaX, deltaY;                         //直线的x,y坐标占宽
    int hatE;                                   //误差判断
    int sign1,sign2;                            //deltaX, deltaY正负判断
    int Interchange;                            //deltaX 和 deltaY调换判断
    int SV_SCUESS = 0;
    int half = stick / 2, hhalf = stick;     //绘制宽度，以(half<<1,hhalf<<1)绘制直线坐标点
    int i,j,k,temp,index;                       //中间变量

    int x_start, x_end, y_start, y_end;         //起点和终点值
    int output;
    
    x_start = point_start.x_aixs;
    y_start = point_start.y_axis;
    x_end = point_end.x_aixs;
    y_end = point_end.y_axis;

    deltaX = abs(x_end - x_start), deltaY = abs(y_end - y_start);
    sign1 = sign(x_end - x_start), sign2 = sign(y_end - y_start);

    
    //--------------------------------------------------------------------------------------------------------------------------------
    //绘制步长判断，选最小步长
    if(deltaY > deltaX)
    {
        temp = deltaX;
        deltaX = deltaY;
        deltaY = temp;
        Interchange = 1;
    }
    else{
        Interchange = 0;
    }
    hatE = 2*deltaY - deltaX;
    //--------------------------------------------------------------------------------------------------------------------------------


    //--------------------------------------------------------------------------------------------------------------------------------
    //通用Bresenham
    for(i=0;i<deltaX;i++)
    {
        //绘制以(x_start,y_start)为中心，长宽为(half<<1, hhalf<<1)的矩形
        if((i%40) <= 20)
            goto skip;
        
        for(j=0;j<hhalf;j++)
        {
            for(k=-half;k<half;k++)
            {   
                if(x_start+k >= width || x_start+k < 0)
                    continue;
                if(y_start + j >= height || y_start + j <0)
                    continue;
                index = x_start+k + (y_start + j)*width;
                pbmp[index] = color;
            }
            
        }
skip:
        //循环判断，生成坐标值(x_start,y_start)
        while(hatE > 0)
        {
            if(Interchange){
                x_start += sign1;
            }
            else{
                y_start += sign2;
            }
            hatE -= 2*deltaX;
        }
        if(Interchange)
        {
            y_start += sign2;
        }
        else
        {
            x_start += sign1;
        }
        hatE += 2*deltaY;

    }
    //--------------------------------------------------------------------------------------------------------------------------------

    return SV_SCUESS;
}

/********************************************************************************************
 * 功能    ：绘制直线图
 * 实现方法：通用Bresenham算法快速绘制直线
 * 输入参数：图像的一维数组(pbmp)，图像宽度(width)，图像高度(height)，
 * 输入参数：直线起始坐标(x_start, y_start), 直线终止坐标(x_end, y_end) 备注：坐标从零开始
 * 输入参数：直线颜色(color), 直线宽度(stick)
 * 输入参数：绘制成功标志,成功返回0，失败返回1
 * 注意    ：输入的坐标并非真实坐标，而是按照图片的宽高的比例scale，原点坐标为左上角
 * *******************************************************************************************
 */
int paint_Line_scale_uint16(uint16*pbmp, int width,int height, pixel_point_scale point_start_scale, \
                            pixel_point_scale point_end_scale, uint16 color, int stick)
{
    pixel_point point_start, point_end;
    point_start.x_aixs      = (int)(point_start_scale.x_scale * width);
    point_start.y_axis      = (int)(point_start_scale.y_scale * height);
    point_end.x_aixs        = (int)(point_end_scale.x_scale * width);
    point_end.y_axis        = (int)(point_end_scale.y_scale * height);
    
    return paint_DottedLine_uint16(pbmp, width, height, point_start, point_end, color, stick);
}

/********************************************************************************************
 * 功能：绘制折线图 
 * 实现方法：多次调用 paint_MultiLine_uint16 绘制直线实现折线图
 * 输入参数：图像的一维数组(pbmp)，图像宽度(width)，图像高度(height)，
 * 输入参数：直线起始坐标(x_start, y_start), 直线终止坐标(x_end, y_end) 备注：坐标从零开始
 * 输入参数：直线颜色(color), 直线宽度(stick)
 * 输入参数：绘制成功标志,成功返回0，失败返回1
 * 输出参数：true
 * *******************************************************************************************
 */
int paint_MultiLine_uint16(uint16*pbmp, int width,int height, pixel_point* point_paint, int lens, uint16 color, int stick)
{
    int i;
    for(i=0;i<lens-1;i++)
    {
        paint_Line_uint16(pbmp, width, height, point_paint[i],point_paint[i+1], color, stick);
    }
    return 1;
}

/********************************************************************************************
 * 功能    ：按照图片比例绘制折线图
 * 实现方法：多次调用 paint_MultiLine_uint16 绘制直线实现折线图
 * 输入参数：图像的一维数组(pbmp)，图像宽度(width)，图像高度(height)，
 * 输入参数：直线起始坐标(x_start, y_start), 直线终止坐标(x_end, y_end) 备注：坐标从零开始
 * 输入参数：直线颜色(color), 直线宽度(stick)
 * 输入参数：绘制成功标志,成功返回0，失败返回1
 * 输出参数：true
 * 说明    ：输入的坐标并非真实坐标，而是按照图片的宽高的比例scale，原点坐标为左上角
 * *******************************************************************************************
 */

int paint_MultiLine_scale_uint16(uint16*pbmp, int width,int height, pixel_point_scale* point_scale_paint, int lens, uint16 color, int stick)
{
    int i;
    for(i=0;i<lens-1;i++)
    {
        paint_Line_scale_uint16(pbmp, width, height, point_scale_paint[i], point_scale_paint[i+1], color, stick);
    }
    return 1;
}

void print_point(pixel_point point_paint)
{
    printf("(%d,%d)\n",point_paint.x_aixs,point_paint.y_axis);
}
void print_point_scale(pixel_point_scale point_paint)
{
    printf("(%.3f,%.3f)\n",point_paint.x_scale,point_paint.y_scale);
}