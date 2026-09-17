#ifndef __BSD__H__
#define __BSD__H__

extern "C" { 
#include "./common/paint.h"
}

//各通道图片分辨率
#define CHIND_WIDTH_x1  1280
#define CHIND_HEIGHT_x1 720
#define CHIND_WIDTH_x2  352
#define CHIND_HEIGHT_x2 240
#define CHIND_WIDTH_x3  1920
#define CHIND_HEIGHT_x3 1080

//OSD图片叠加信息
#define BSD_POINTX_x1   0
#define BSD_POINTY_x1   464
#define BSD_POINTX_x2   0
#define BSD_POINTY_x2   170
#define BSD_POINTX_x3   0
#define BSD_POINTY_x3   696
#define BSD_HEIGHT_x1   164
#define BSD_WIDTH_x1    1280
#define BSD_HEIGHT_x2   45
#define BSD_WIDTH_x2    352
#define BSD_HEIGHT_x3   246
#define BSD_WIDTH_x3    1920

//子图叠加信息
#define BSD_SUB_HEIGHT_x1   BSD_HEIGHT_x1
#define BSD_SUB_WIDTH_x1    (BSD_WIDTH_x1 / 5)
#define BSD_SUB_HEIGHT_x2   BSD_HEIGHT_x2
#define BSD_SUB_WIDTH_x2    (BSD_WIDTH_x2 / 5)
#define BSD_SUB_HEIGHT_x3   BSD_HEIGHT_x3
#define BSD_SUB_WIDTH_x3    (BSD_WIDTH_x3 / 5)
#define BSD_SIZE_x1         (BSD_WIDTH_x1 * BSD_HEIGHT_x1)
#define BSD_SIZE_x2         (BSD_WIDTH_x2 * BSD_HEIGHT_x2)
#define BSD_SIZE_x3         (BSD_WIDTH_x3 * BSD_HEIGHT_x3)

//识别区域 左上点x,左上点y,左下点x,左下点y,右下点x,右下点y,右上点x,右上点y
#define BSD_ROI_x1 0.02
#define BSD_ROI_y1 0.35
#define BSD_ROI_x2 0.02
#define BSD_ROI_y2 0.95
#define BSD_ROI_x3 0.98
#define BSD_ROI_y3 0.95
#define BSD_ROI_x4 0.98
#define BSD_ROI_y4 0.35

//通道ID
#define BSD_CHNID_x1        0
#define BSD_CHNID_x2        1
#define BSD_CHNID_x3        2

//位图句柄
#define BSD_HANDEL_x1       10
#define BSD_HANDEL_x2       11
#define BSD_HANDEL_x3       12

//信息句柄
#define BSD_HANDEL_MESSAGE_x1 13
#define BSD_HANDEL_MESSAGE_x2 14
#define BSD_HANDEL_MESSAGE_x3 15

//区域句柄
#define BSD_HANDEL_RGN_x1       16
#define BSD_HANDEL_RGN_x2       17
#define BSD_HANDEL_RGN_x3       18

//图片位置
#define BSD_FILE_NAME       "overtake"
#define BSD_FILE_EXT_NAME   ".tar.gz"
#define BSD_FILE_ELEMENT    "/%dx%d.png"
#define BSD_ZIP_PATH        "/root/" BSD_FILE_NAME BSD_FILE_EXT_NAME
#define BSD_PATH            "/var/" BSD_FILE_NAME BSD_FILE_ELEMENT
#define BSD_MESSAGE_ENABLE  1   //显示TH信息数据
#define BSD_RGN_ENABLE      1   //显示追踪区域
#define BSD_CHIND_NUM       3   //使用OSD通道最大数量

//BSD算法参数
#define BSD_TH              8   //敏感度


/* BSD 控制信息 */
typedef struct tagBsdControl_S
{
    
    uint32          u32Tid;             /* 处理线程ID */
    SV_BOOL         bRunning;           /* 线程是否正在运行 */
} BSD_CTL_S;

/* OSD叠加区域信息 */
typedef struct tagBsdRegion_S
{
    SV_BOOL     bCreated;
    SV_BOOL     bAttached;
    SV_BOOL     bShow;
    /* 其中三个属于位图图样，三个属于信息图样，三个属于区域图样 */
    RGN_HANDLE  pu32Handle[9];          
    HI_S32      ps32ChnId[9];
    SV_RECT_S   pstRect[9];
} BSD_RGN_S;

/* BSD 运行数据信息 */
typedef struct tagBSDState_s
{
    SV_BOOL     bUpdate;                /* 更新标志 */
    SV_BOOL     state[5];               /* 追踪状态信息 */


    uint32      pheight[3];             /* 三通道位图图样高度 */
    uint32      pwidth[3];              /* 三通道位图图样宽度 */
    uint32      pyaxis[3];              /* 三通道位图y轴坐标 */
    uint32      pheight_sub[3];         /* 三通道位图子图图样高度 */
    uint32      pwidth_sub[3];          /* 三通道位图子图图样宽度 */

    HI_VOID    *pvir_bsd[3];            /* 三通道位图图样数据 */
    HI_VOID    *pvir_sub_bsd[3];        /* 三通道位图子图图样数据 */
    HI_VOID    *pvir_sub_bsd_zip[15];   /* 三通道位图子图图样的压缩数据 */
    uint32      psub_zip_len[15];       /* 三通道位图子图图样的压缩数据长度 */

    pixel_point_scale   point_paint[5]; /* 用于绘制区域图样的点 */

    uint32      bsd_th;                 /* BSD 灵敏度信息 */
    char        pstring[20];            /* 用于设置信息图样的字符数据 */

} BSD_STA_S;

/* BSD 算法数据信息 */
typedef struct tagBSDAlgorithm_S
{
    float fPoint[8];                    /* 识别区域信息 */
    int   bsd_th;                       /* 敏感度设置 */
    
    int   nBSDRe;                       /* 当前算法结果 */
    int   preBSDRe;                     /* 前一帧算法结果 */
    
} BSD_ALG_S;

/* BSD 全局信息 */
typedef struct tagBSDINFO_S
{
    BSD_CTL_S       stCTL;
    BSD_RGN_S       stRGN;
    BSD_STA_S       stSTA;
    BSD_ALG_S       stALG;
} BSD_INFO_S;

sint32 img_memcopy(char *img_dst, int width_dst, int height_dst, 
                  char *img_src, int width_src, int height_src, 
                  int x_insert, int y_insert, int byte);

sint32 BSD_Init(BSD_INFO_S* pBsdInfo);              //BSD 初始化
sint32 BSD_PARAM_Init(BSD_INFO_S* pBsdInfo);         //BSD 参数初始化
sint32 BSD_OSD_Init(BSD_INFO_S* pBsdInfo);          //BSD OSD叠加信息初始化
sint32 BSD_Image_Init(BSD_INFO_S* pBsdInfo);        //BSD 位图读取初始化
sint32 BSD_Font_Init(BSD_INFO_S* pBsdInfo);         //BSD 字体信息初始化
sint32 BSD_ALG_Init(BSD_INFO_S* pBsdInfo);          //BSD 算法初始化


sint32 BSD_Handle_Body(void *arg);                 //BSD 入口程序
sint32 BSD_OSD_Update(BSD_INFO_S* pBsdInfo);        //BSD OSD更新
sint32 BSD_Image_Update(BSD_INFO_S* pBsdInfo);      //BSD 位图更新
sint32 BSD_Font_Update(BSD_INFO_S* pBsdInfo);       //BSD 字体更新
sint32 BSD_Rect_Update(BSD_INFO_S* pBsdInfo);        //BSD 区域更新
sint32 BSD_ALG_Update(BSD_INFO_S* pBsdInfo);        //BSD 算法结果更新


sint32 BSD_Fini(BSD_INFO_S* pBsdInfo);              //BSD去初始化
sint32 BSD_PARAM_Fini(BSD_INFO_S* pBsdInfo);
sint32 BSD_OSD_Fini(BSD_INFO_S* pBsdInfo);          //BSD OSD叠加信息去初始化
sint32 BSD_Image_Fini(BSD_INFO_S* pBsdInfo);        //BSD 位图信息去初始化
sint32 BSD_Font_Fini(BSD_INFO_S* pBsdInfo);         //BSD 字体信息去初始化
sint32 BSD_ALG_Fini(BSD_INFO_S* pBsdInfo);          //BSD 算法去初始化

sint32 BSD_Start(BSD_INFO_S* pBsdInfo);             //启动线程
sint32 BSD_Stop(BSD_INFO_S* pBsdInfo);              //关闭线程

#endif