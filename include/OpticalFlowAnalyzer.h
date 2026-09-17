/******************************************************************************
  File Name     : OpticalFlowAnalyzer.h
  Version       : 1.0.0
  Author        : Xie jun
  Created       : 2025-02-19
  Last Modified : 2025-05-09
  Description   : 用于外部调用，进行方向判断和栈板报警
  Function List :
  History       : 
  日期，修改内容 ：2025-03-26 整合方向判断以及栈板检测
  日期，修改内容 ：2025-05-09 构造函数增加算法类型输入
******************************************************************************/
#ifndef OPTICALFLOW_ANALYZER_H
#define OPTICALFLOW_ANALYZER_H
#include"OpticalFlowCommon.h"

namespace opticalflowanalyzeralg
{
    /*光流分析类*/
	class COpticalFlowAnalyzer
	{
		private:
			class m_cInnerAnalysis;
			class m_cInnerAnalysis* m_pcInnerAnalysis=nullptr;
			void releaseMem();

		public:
			/******************************************************************************
			功能: 类COpticalFlowAnalyzer构造函数；
			参数: 
			-eAlgType:             算法类型
			返值: 无；
			应用: 初始化类别成员；
			******************************************************************************/
			COpticalFlowAnalyzer(EAlgType eAlgType);

			/******************************************************************************
			功能: 类CCameraAnomalyAnalyzer默认析构函数；
			参数: 无；
			返值: 无；
			应用: 释放类别实例；
			******************************************************************************/
			~COpticalFlowAnalyzer();

			/******************************************************************************
			功能: 车辆状态分析函数，返回方向
			参数: 
			-p_inputdata：         图像的数据指针，RGB图像数据的地址
			-eVehicleStateCode:    返回的车辆状态
			返值:                  错误编码，详见OpticalFlowCommon.h
			应用: 用于车辆状态分析
			******************************************************************************/
			int32_t VehicleStateAnalysis(uint8_t* p_inputdata, EVehicleStateCode& eVehicleStateCode); 

			/******************************************************************************
			功能: 栈板分析函数，分析栈板状态
			参数: 
			-p_inputdata：         图像的数据指针，RGB图像数据的地址
			-stPalletDetectResult: 返回的结果结构体
			返值:                  错误编码，详见OpticalFlowCommon.h
			应用: 用于叉车栈板检测
			******************************************************************************/
			int32_t PalletStateAnalysis(uint8_t* p_inputdata, STPalletDetectResult& stPalletDetectResult); 

			/******************************************************************************
			功能: 修改并保存栈板检测参数到应用层；
			参数: 
			-p_cConfigFilePath:    配置文件路径
			-stPalletDetectResult: 需要保存的配置参数
			返值:                  错误编码
			应用: 用于保存栈板检测参数，每次更改参数时调用
			******************************************************************************/
			int32_t SaveParams(const char* p_cConfigFilePath, const STAlgParams &stPalletDetectParams);

			/******************************************************************************
			功能: 从应用层导入相关参数；
			参数: 
			-p_cConfigFilePath:    配置文件路径
			-stPalletDetectParams: 导出给应用层的参数
			返值:                  错误编码
			应用: 每次机器启动时调用，用于导入相关参数
			******************************************************************************/
			int32_t ImportParams(const char* p_cConfigFilePath, STAlgParams &stPalletDetectParams);

    };

}







#endif