#ifndef lane_h
#define lane_h

#include "lane_common.hpp"


class CLane {
public:
    CLane(uint64_t u64Width, uint64_t u64Height, uint64_t u64anochorNum, uint64_t u64NumLanes,
            std::string ModelImgPath);

    ~CLane();

public:

    int32_t LaneDetect(void* pvRgbData, std::vector<STLaneInfo>& vec_stLaneInfoList1, std::vector<STLaneInfo>& vec_stLaneInfoList2);
    void LaneRelease();
    void Result(float *pfData, std::vector<STLaneInfo> &vec_stLaneInfoList1, std::vector<STLaneInfo>& vec_stLaneInfoList2);
    // void Result(float *pfData, std::vector<STLaneInfo> &vec_stLaneInfoList1);
    
public:
    std::vector<uint64_t> m_vec_RowAnchor {121, 131, 141, 150, 160, 170, 180, 189, 199, 
                                            209, 219, 228, 238, 248, 258, 267, 277, 287};

    uint64_t m_u64ClsNumPerLane {SV_LANEPOINTS};
    uint64_t m_u64AnchorNum {SV_GRIDING};
    uint64_t m_u64NumLanes {SV_NUMLANES};

    uint64_t m_u64ImgWidth {SV_WIDTH};
    uint64_t m_u64ImgHeight {SV_HEIGHT};

    std::vector<uint64_t> m_vecOutAnchor;
    float m_fColSampleW;

    std::vector<STLaneInfo> m_vecLaneList;
    std::vector<uint8_t> m_vecProcessInput;

    char *m_pmem = NULL;

    rknn_context m_Ctx;
    uint8_t *m_pu8Model;
    int32_t m_s32ModelLen {0};
    int32_t m_s32Ret;
    // const std::string model_name = "../../rkmodels_160/800_lane.rknn";
    std::string m_ModelImgPath = "416_224_208_output.rknn";
    rknn_input_output_num m_IoNum;

    //  process
    float m_fOutJ[SV_GRIDING+1][SV_LANEPOINTS][SV_NUMLANES];
    float m_fOutMax[SV_LANEPOINTS][SV_NUMLANES];
    float m_fProb[SV_GRIDING][SV_LANEPOINTS][SV_NUMLANES];
    float m_fIdx[SV_GRIDING];

    uint64_t m_u64OutputMax {0};
    float m_fOutputSum {0.0};
    float m_fSoftMaxsum {0.0};

    float m_fLoc[SV_LANEPOINTS][SV_NUMLANES];
    // uint64_t m_u64LocL[2] {SV_LANEPOINTS,SV_LANEPOINTS};
    // uint64_t m_u64PpNum {0};
    // uint64_t m_u64PpLine {0};
    float m_fOutJTmp {0.0};

    uint64_t m_u64CountNum {0};
    bool m_BRGB {false};
    uint32_t m_u32ChannelID_lane {0};

    //  rk api
    rknn_input m_Inputs[1];
    rknn_output m_Outputs[1];
};

#endif