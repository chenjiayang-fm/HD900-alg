//
// Created by root on 21-11-18.
//
/****************************************************************
 File Name : DSM.h
 Version: 1.0.0
 Author: lin jiaping
 Created: 2022-6-30
 Last Modified: 2022-6-30
 Description: DMS头文件
 Function list:
 History :
 2022-08-15,按代码审查意见修改代码
****************************************************************/
#ifndef DSM_H
#define DSM_H

#include "Thread.h"


#define NMS_UNION 1
#define NMS_MIN  2

#define THREDNUM 1
#define TASKNUM 1


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

typedef struct STFaceObject
{
    cv::Rect_<float> cfRect;
    float fProb;
};

typedef struct STModelGroup{
    rknn_context FANCtx,FDCtx,FRCtx,SmCtx,PhCtx,DECtx,GlassCtx,MaskCtx,Smoket1Ctx,SeatbeltCtx,EyeCtx,GazeCtx,HelmetCtx;
	//模型大小
    uint32_t u32FANModelLen,u32FDModelLen,u32FRModelLen,u32SmModelLen,u32PhModelLen,u32DEModelLen,u32GlassModelLen,u32MaskModelLen,u32Smoket1ModelLen,u32SeatbeltModelLen,u32EyeModelLen,u32GazeModelLen,u32HelmetModelLen;

	//模型指针头
    unsigned char* pu8FANModel;
    unsigned char* pu8FDModel;
    unsigned char* pu8FRModel;
    unsigned char* pu8SmModel;
    unsigned char* pu8PhModel;
    unsigned char* pu8DEModel;
    unsigned char* pu8GlassModel;
    unsigned char* pu8MaskModel;
    unsigned char* pu8Smoket1Model;
	unsigned char* pu8SeatbeltModel;
    unsigned char* pu8EyeModel;
    unsigned char* pu8GazeModel;
    unsigned char* pu8HelmetModel;

	//模型路径
    const char* ps8FANModelPath;
    const char* ps8FDModelPath;
    const char* ps8FRModelPath;
    const char* ps8SmModelPath;
    const char* ps8PhModelPath;
    const char* ps8DEModelPath;
    const char* ps8GlassModelPath;
    const char* ps8MaskModelPath;
    const char* ps8Smoket1ModelPath;
    const char* ps8SeatbeltModelPath;
    const char* ps8EyeModelPath;
    const char* ps8GazeModelPath;
    const char* ps8HelmetModelPath;

	//模型输入输出数量
    rknn_input_output_num FANIoNum;
    rknn_input_output_num FDIoNum;
    rknn_input_output_num FRIoNum;
    rknn_input_output_num SmIoNum;
    rknn_input_output_num PhIoNum;
    rknn_input_output_num DEIoNum;
    rknn_input_output_num GlassIoNum;
    rknn_input_output_num MaskIoNum;
    rknn_input_output_num Smoket1IoNum;
    rknn_input_output_num SeatbeltIoNum;
    rknn_input_output_num EyeIoNum;
    rknn_input_output_num GazeIoNum;
    rknn_input_output_num HelmetIoNum;

	//模型输入
    rknn_input FANInputs[1];
    rknn_input FDInputs[1];
    rknn_input FRInputs[1];
    rknn_input SmInputs[1];
    rknn_input PhInputs[1];
    rknn_input DEInputs[1];
    rknn_input GlassInputs[1];
    rknn_input MaskInputs[2];
    rknn_input Smoket1Inputs[1];
    rknn_input SeatbeltInputs[1];
    rknn_input EyeInputs[1];
    rknn_input pstGazeInputs[1];
    rknn_input pstHelmetInputs[1];

	//模型输出
    rknn_output FDOutputs[6];
    rknn_output FANOutputs[4];
    rknn_output FROutputs[1];
    rknn_output SmOutputs[1];
    rknn_output PhOutputs[3];
    rknn_output DEOutputs[3];
    rknn_output GlassOutputs[1];
    rknn_output MaskOutputs[1];
    rknn_output Smoket1Outputs[1];
    rknn_output SeatbeltOutputs[3];
    rknn_output EyeOutputs[1];
    rknn_output pstGazeOutputs[1];
    rknn_output pstHelmetOutputs[3];
    rknn_tensor_attr PhOutputAttrs[3];
    rknn_tensor_attr DEOutputAttrs[3];



    //模型MD5码
    const char* ps8FANModelMd5[2];
    const char* ps8FDModelMd5[2];
    const char* ps8FRModelMd5[2];
    const char* ps8SmModelMd5[2];
    const char* ps8PhModelMd5[2];
    const char* ps8DEModelMd5[2];
    const char* ps8GlassModelMd5[2];
    const char* ps8MaskModelMd5[2];
    const char* ps8Smoket1ModelMd5[2];
    const char* ps8SeatbeltModelMd5[2];
    const char* ps8EyeModelMd5[2];
    const char* ps8GazeModelMd5[2];
    const char* ps8HelmetModelMd5[2];
};//模型组

class CDSM{
    private:

	//人脸检测变量
    cv::Mat m_cScore8;
    cv::Mat m_cBbox8;
    cv::Mat m_cScore16;
    cv::Mat m_cBbox16;
    cv::Mat m_cScore32;
    cv::Mat m_cBbox32;
    std::vector<STFaceObject> m_vec_stFaceobjects;

	//眼睛闭合度
    float m_fECR;
    float m_fMaxAbsEcr;

	//电话检测类型（左边或右边）
    uint8_t m_u8PhoneType;

	//电话检测分数
    float m_fPhone;

	//吃喝东西检测分数
    float m_fDrinkEat;

	//抽烟手势检测结果
    bool m_bSmokeorNot;

	//抽烟检测分数
    float m_fSmoke;
	cv::Mat m_cSmokeHeatMap;

	//人脸检测输入大小
    float m_fWscale;
    float m_fHscale;

	//人脸对齐（跟踪）帧数
    uint16_t m_u16Count;

	//口罩检测结果
    bool m_bNoMask;

    
	//线程池
    CThreadPool m_cThreadPool;
    CMyTask m_pcTaskObj[TASKNUM];

	//安全带检测结果
    float m_fSeatbeltScore;
    float m_fSeatBeltThr;

    //安全帽检测效果
    float m_fHelmetScore;
    float m_fHelmetThr;

    //shelter
    uint8_t m_u8KSize;
    float m_fSigma;
    float m_fK;
    float m_fBeta;
    uint8_t m_u8ImShelTh;
    uint8_t m_u8ICamShelTh;
    cv::Mat m_cKernelH;
    cv::Mat m_cKernelV;
    cv::Mat m_cKernel1H;
    cv::Mat m_cKernel1V;
    bool m_bImgOcclusion;
    //平台类型
    EApplyType m_eApplyType;
	//是否加载人脸识别模型
	bool m_bFrOrNot;
public:
        STModelGroup m_stRKModels;
		/*********************************************************************
		*该函数获取算法版本号；
		*输入：
		*   无；
		*输出：
		*   无
		*********************************************************************/
        void DSMVersion(){print_level(SV_INFO,"DSM Version: 2.1.0.45\n");}
		/*********************************************************************
		*该函数初始化DSM类，导入模型，开辟模型输入输出内存；
		*输入：
		*   无；
		*输出：
		*   0,：成功
		*	-1：失败
		*********************************************************************/
        int32_t DSMInit(ALG_CHECK_CALLBACK pfCheckCallback, bool bEncrypt = true);
		/*********************************************************************
		*该函数创建DSM检测类，初始化变量；
		*输入：
		*   EApplyType_ 平台类型。默认为DMS31；
		*输出：
		*   无
		*********************************************************************/
        CDSM(EApplyType m_eApplyType_ = E_DMS31, bool bFrOrNot = true);
        ~CDSM();
		/*********************************************************************
		*该函数开辟模型输入输出内存；
		*输入：
		*   无；
		*输出：
		*   0,：成功
		*	-1：失败
		*********************************************************************/
        int32_t BlobsMemDSM();
		/*********************************************************************
		*该函数为DSM模型组前推；
		*输入：
		*   pu8ImgsrcPtr,：图像指针头
		*	dmmP：DMS检测结果
		*输出：
		*   0,：成功
		*	-1：失败
		*********************************************************************/
        int32_t ForwardGroupDSM(uchar* pu8ImgsrcPtr,dmmParam& dmmP,bool bPhoneSW,bool bSmokeSW,bool bNoMaskSW,bool bSmokeT2,bool bDrinkEatSW);
		/*********************************************************************
		*该函数为人脸识别模型组前推；
		*输入：
		*   pu8ImgsrcPtr,：图像指针头
		*	frP：人脸识别检测结果
		*输出：
		*   0,：成功
		*	-1：失败
		*********************************************************************/
        int32_t ForwardGroupFR(uchar* pu8ImgsrcPtr,frParam& frP);
		/*********************************************************************
		*该函数打断人脸对齐，重新人脸检测；
		*输入：
		*   无
		*输出：
		*   无
		*********************************************************************/
        void FpInit();
		/*********************************************************************
		*该函数获取人脸方框；
		*输入：
		*   无
		*输出：
		*   人脸方框坐标
		*********************************************************************/
        cv::Rect GetAppearanceRoi();
		/*********************************************************************
		*该函数剪切抽烟、打电话、口罩、安全带的检测区域；
		*输入：
		*   cRp：右边打电话区域
		*	Lp：左边打电话区域
		*	Sm：香烟检测区域
		*	cSmT2：抽烟手势检测区域
		*	Mask：口罩检测区域
		*输出：
		*   无
		*********************************************************************/
		void detectRoi(cv::Rect cRp,cv::Rect cLp,cv::Rect cSm,cv::Rect cSmT2,cv::Rect cMask,cv::Rect cSb,cv::Rect cDE,cv::Rect cHelmet);
		/*********************************************************************
		*该函数为香烟检测后处理；
		*输入：
		*   cSmokeHm:香烟检测热力图
		*输出：
		*   香烟的面积
		*********************************************************************/
		uint32_t LightpointDetect(cv::Mat& cSmokeHm);
		/*********************************************************************
		*该函数为人脸检测后处理；
		*输入：
		*   m_vec_stFaceobjects：人脸检测结果
		*   fPropThr:目标检测阈值
		*   fNmsThr:nms阈值
		*输出：
		*   无
		*********************************************************************/
        void DecodeFd(std::vector<STFaceObject>& m_vec_stFaceobjects,float fPropThr,float fNmsThr);
		/*********************************************************************
		*该函数为图像遮挡检测；
		*输入：
		*   无
		*输出：
		*   True：检测到遮挡
		*   False：检测无遮挡
		*********************************************************************/
        bool DetectShelter();
};
#endif //DSM_H
