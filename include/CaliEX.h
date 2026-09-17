#ifndef _CALI_EX_HPP_
#define _CALI_EX_HPP_
#include <stdint.h>


typedef struct _STCaliParams_ {
    float fK[9];  // 内参矩阵
    float fRotation[3];  // 旋转向量
    float fTranslation[3];  // 平移向量
    float fDistortion[5];  // 畸变系数
    float fOldCamPos[3];  // 标定好后, "相机坐标系的原点"在世界坐标系的位置 --- (x,y,z)
    float fEulerAngles[3];  // 欧拉角, 3个参数[a,b,c], 分别代表将相机坐标系: 绕z轴旋转c弧度, 绕y轴转b弧度, 绕x轴转a弧度. 最后与世界坐标系方向一致
    float fPrincipal[2];  // 相机光心在像素上的坐标
}STCaliParams;


// 世界坐标点, 单位是毫米
typedef struct _STWorldPoint_{
    float fX;
    float fY;
    float fZ;
}STWorldPoint;


// 像素坐标点, 单位是像素
typedef struct _IMG_POINT_{
    float fX;  // x就是u, 原点在图片左上角
    float fY;  // y就是v, 原点在图片左上角
}STImgPoint;




/**********************************************************************
 * 
 * 标定初始化1 (这个是标定的时候用)
 * 
 * [输入参数]
 * PatSize: 二维码标定板的宽, 单位毫米(mm)
 * FocalLength: 镜头焦距, 单位毫米(mm), 目前支持设置为 6.0 和 2.8
 * PixelPitch: 像素尺度(每个像素点多少毫米), 查sensor规格书获得, 目前海思A32用的是0.0028 
 * ChannelIndex: 是哪路相机在初始化, 默认请传0, 只有1台相机
 * 
 **********************************************************************/
int32_t Cali_init(float fPatSize, float fFocalLength, float fPixelPitch, uint32_t s32ChannelIndex);

/**********************************************************************
 * 
 * 导入标定好的参数 (这个是标定好后, 应用里面用, 例如重新启动了机器, 或者新建了类对象)
 * 
 * [输入参数]
 * outerParam: 之前标定的参数
 * ChannelIndex: 是哪路相机在导入参数, 默认请传0, 只有1台相机
 * 
 **********************************************************************/
int32_t Cali_import(STCaliParams stOuterParam, uint32_t s32ChannelIndex);

/***********************************************************************
 * 
 * 开始标定
 * 
 * [输入参数]
 * src:  8bit单通道1920x1080图片的数据地址
 * masks:  传入4个数组进来, 表示4个点坐标
 *       masks[0]: 左边二维码矩形框的左上角坐标
 *       masks[1]: 左边二维码矩形框的右下角坐标
 *       masks[2]: 右边二维码矩形框的左上角坐标
 *       masks[3]: 右边二维码矩形框的右下角坐标
 * ChannelIndex: 是哪路相机在标定, 默认请传0, 只有1台相机
 * 
 * [输出参数]
 * outerParam: 各种相机参数
 * 
 * 注: 标定完后, 需要应用层保存outerParam, 
 *     后面测距的时候通过Cali_init(),Cali_import() 把参数载入进来, 再进行测距
 ***********************************************************************/
int32_t Cali_run(unsigned char *src, STCaliParams &outerParam, uint32_t s32ChannelIndex);
int32_t Cali_run(unsigned char *src, STCaliParams &outerParam, STImgPoint* masks, uint32_t s32ChannelIndex);

/******************************************************************
 * 测距, 根据图像上的坐标, 算出它在真实世界的位置
 * 
 * [输入参数]
 * imgPoint: 像素坐标; 以图像左上角为原点, 单位是像素
 * ChannelIndex: 是哪路相机在测距
 * 
 * [输出参数]
 * worldPoint: 世界坐标; 该像素点对应的实际位置, 以摄像头为原点, 摄像头前面的方向是y, 左右的方向是x, 单位毫米
 ******************************************************************/
int32_t Cali_distance(STImgPoint imgPoint, STWorldPoint &worldPoint, uint32_t ChannelIndex);

/******************************************************************
 * 测距, 根据世界坐标, 算出它在图像的位置
 * 
 * [输入参数] 
 * worldPoint: 世界坐标; 单位毫米.
 *             前方y毫米, 左(负数)右(正数)x毫米, 高度z毫米(地面为z=0)
 * ChannelIndex: 是哪路相机在测距
 * 
 * [输出参数]
 * imgPoint: 像素坐标
 ******************************************************************/
int32_t Cali_projection(STWorldPoint worldPoint, STImgPoint &imgPoint, uint32_t ChannelIndex);
#endif