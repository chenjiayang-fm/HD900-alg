/********************************************************
 * 论文：《SiamFC++: Towards Robust and Accurate Visual Tracking with Target Estimation Guidelines》
 * 文章地址：https://arxiv.org/abs/1911.06188
 * github地址：https://github.com/MegviiDetection/video_analyst
 * 
 * 缩写: SiameseFC++  -->>  SiameseFCpp  -->>  SFCP
 * 
 * 
 * 
 ********************************************************/
#include "rknn_api.h"
#include <opencv2/core.hpp>


typedef struct _SFCP_BOX_{
    int cx;  // 中心点的x坐标
    int cy;  // 中心点的y坐标
    int w;   // 宽
    int h;   // 高
    friend std::ostream& operator<<(std::ostream &os, const _SFCP_BOX_ &box){
        os << "cx[" << box.cx << "] cy[" << box.cy << "] w[" << box.w << "] h[" << box.h << "]";
        return os;
    }
}SFCP_BOX;


typedef struct _SFCP_BOX_1T_{
    int x1;   // 左上角的x坐标
    int y1;   // 左上角的y坐标
    int x2;   // 右下角的x坐标
    int y2;   // 右下角的y坐标
}SFCP_BOX1_T;


typedef struct _SFCP_BOX_2T_{
    int x1;  // 左上角的x坐标
    int y1;  // 左上角的y坐标
    int w;   // 宽
    int h;   // 高
}SFCP_BOX2_T;


/****************************************
 * 返回值, 部分与 rknn_api.h 保持同步
 ****************************************/
enum SFCP_PROCESS_TYPE {
    /********************************RKNN的错误码***********************************************/
    SFCP_SUCCESS                                     = 0,    //  成功
    SFCP_ERR_FAIL                                    = -1,   //  执行失败
    SFCP_ERR_TIMEOUT                                 = -2,   //  线性库优化失败
    SFCP_ERR_DEVICE_UNAVAILABLE                      = -3,   //  设备不可访问
    SFCP_ERR_MALLOC_FAIL                             = -4,   //  内存创建失败
    SFCP_ERR_PARAM_INVALID                           = -5,   //  参数无效
    SFCP_ERR_MODEL_INVALI                            = -6,   //  模型无效
    SFCP_ERR_CTX_INVALID                             = -7,   //  context无效
    SFCP_ERR_INPUT_INVALID                           = -8,   //  输入无效
    SFCP_ERR_OUTPUT_INVALID                          = -9,   //  输出无效
    SFCP_ERR_DEVICE_UNMATCH                          = -10,  //  设备不匹配, 需要更新RKNN的sdk和npu的驱动和固件
    SFCP_ERR_INCOMPATILE_PRE_COMPILE_MODEL           = -11,  //  加载的RKNN模型是预编译的, 当前驱动不支持这种模式
    SFCP_ERR_INCOMPATILE_OPTIMIZATION_LEVEL_VERSION  = -12,  //  加载的RKNN模型设置了优化级别, 当前驱动不支持这种模式
    SFCP_ERR_TARGET_PLATFORM_UNMATCH                 = -13,  //  加载的RKNN模型和当前运行平台不符合
    SFCP_ERR_NON_PRE_COMPILED_MODEL_ON_MINI_DRIVE    = -14,  //  加载的RKNN模型不是预编译的, 当前用的mini driver跑不了
    /********************************RKNN的错误码***********************************************/


    /********************************本算法库的错误码***********************************************/
    SFCP_ERR_RE_INIT                                 = -20,  //  请不要重复初始化
    SFCP_ERR_MODEL_NULL                              = -21,  //  初始化时, 模型文件的指针为空, 请正确设置参数
    SFCP_ERR_LOAD_MODEL                              = -22,  //  初始化时, 加载模型文件失败
    SFCP_ERR_MUST_INIT                               = -23,  //  需要先进行初始化
    SFCP_ERR_MUST_PREPARE                            = -24,   //  需要先准备跟踪对象
    SFCP_ERR_MODEL_UNMATCH                           = -25,  //  模型与算法库的预设不匹配, 可能是零拷贝的模型, 或者其它模型
    SFCP_ERR_DMF_FD_INVALID                          = -26,  //  用了零拷贝的模型, 但是没有正确的DMA BUFFER FD
    SFCP_ERR_IMG_MUST_16ALIGN                        = -27,  //  用了零拷贝的模型, 输入图像必须16字节对齐
    SFCP_ERR_DRM_ERROR                               = -28,  //  DRM初始化错误
    SFCP_ERR_RGB_BLIT                                = -29,  //  RGA错误
    /********************************本算法库的错误码***********************************************/

};

/***********************************************
 * 
 * 建议输入1280x720
 * 
 ***********************************************/
class SFCP {
public:

    SFCP();

    ~SFCP();

    /******************************************
     * 算法初始化, 包括加载模型和参数配置
     * 
     * modelFile:  模型文件
     * 
     * debug:      是否开启调试信息
     *             true:  打印
     *             false: 不打印
     *
     ******************************************/ 
    int init(char* modelFile, bool debug);

     /******************************************
     * 准备跟踪, 根据要跟踪的区域(targetBox), 做一些初始化, 
     * 在第一帧时调用
     * 后续调用update即可对targetBox进行跟踪
     * 
     * 如果要改变或者更新跟踪对象, 重新调用此接口, 传入新的targetBox
     * 
     * 图片尺寸设好了就不要更改了, 建议 1280x720 or 960x540
     * 
     * imgRGBdata: RGB888的图片地址
     * s32DmaBuffFD: 输入图像的 dma buffer 的文件描述符
     * imgWidth:   图片的宽
     * imgHeight:  图片的高
     * targetBox:  要跟踪的区域
     ******************************************/  
    int prepare(char* imgRGBdata, int32_t s32DmaBuffFD, int imgWidth, int imgHeight, SFCP_BOX targetBox);

    
    /******************************************
     * 开始跟踪, 返回被跟踪的目标在当前图片的位置
     * 
     * 图片尺寸设好了就不要更改了, 建议 1280x720 or 960x540
     * 
     * [输入]
     * imgRGBdata: RGB888的图片地址, 尺寸与 prepare 时设置的一致
     * s32DmaBuffFD: 输入图像的 dma buffer 的文件描述符
     * fAmount: 搜索区域扩展多少, 0.5~0.8之间, 默认0.5
     * 
     * [输出]
     * resultBox: 跟踪结果
     * scoreOutput: 本次跟踪框的得分(置信度)
     ******************************************/  
    int update(char* imgRGBdata, int32_t s32DmaBuffFD, SFCP_BOX& resultBox, float& scoreOutput, float fAmount=0.5);


private:
    static const int zSize = 128;  // 匹配模板的尺寸, 正方形
    static const int xSize = 304;  // 搜索图的尺寸, 正方形
    static const int scoreSize = 17;
    static const int totalStride = 8;
    static const int scoreOffset = 87;  // ((xSize-1) - (scoreSize-1)*totalStride) // 2
    static constexpr float contextAmount = 0.5f;
    static constexpr float penaltyK = 0.05413758904760692f; 
    static constexpr float windowInfluence = 0.23153228172839774; 
    static constexpr float testLR = 0.5249642198880932; 


    struct{
        int imgHeight;  // 原图实际尺寸, 推荐1280x720
        int imgWidth;  // 原图实际尺寸, 推荐1280x720
        float scale_x;
        cv::Mat window;
        cv::Mat trackTarget;  // 被跟踪的目标(像素数据)
        cv::Mat trackSearch;  // 被搜索的图片(像素数据)
        cv::Scalar avgChans;  // 颜色均值
        SFCP_BOX currentBox;  // 当前目标在哪
        float currentScore;
    }state;  // 跟踪器状态

    rknn_context RKctx;  // RK接口的句柄
    unsigned char* pModelData = nullptr;  // 模型文件的数据指针, 加载模型后获得 (记得释放)
    rknn_input_output_num RKioNum;  // 输入输出的数量, 加载模型查询获得
    rknn_tensor_attr* pInputAttr = nullptr;  // 模型输入向量属性, 初始化后获得 (记得释放)
    rknn_tensor_attr* pOutputAttr = nullptr;  // 模型输出向量属性, 初始化后获得 (记得释放)

    rknn_input* pRKinputs = nullptr;  // 模型输入, 初始化后获得 (记得释放)
    rknn_output* pRKoutputs = nullptr;  // 模型输出, 初始化后获得 (记得释放)

    float* pAnchorXctr = nullptr;  // 先验框中心点
    float* pAnchorYctr = nullptr;  // 先验框中心点
    float* pDecodeX0 = nullptr;  // 
    float* pDecodeY0 = nullptr;  // 
    float* pDecodeX1 = nullptr;  // 
    float* pDecodeY1 = nullptr;  // 
    float* pDecodeScore = nullptr;  // 

    bool m_bZeroCopy;  // 是不是用的0拷贝
    bool bDebug;  // 是否开启debug
    bool bInitSuccess;  // 是否初始化成功
    bool bPrepareSuccess;  // 是否准备成功(模板图)

    void printRKVersion();
    void cropZ(cv::Mat inImage, SFCP_BOX target, cv::Mat& outImage);
    void cropX(cv::Mat inImage, SFCP_BOX target, cv::Mat& outImage, float fAmount=0.5);
    void cropXm(cv::Mat inImage, SFCP_BOX target, cv::Mat& outImage, float fAmount=0.5);
    void initAnchorCtr();
    void assignMemory();  // 为处理结果分配空间
    void postProcess();
};
