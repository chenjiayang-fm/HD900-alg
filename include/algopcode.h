/*20190218
1.dmm and fr in the same program



*/
#ifndef _ALG_OPCODE_H_
#define _ALG_OPCODE_H_
#include <stdbool.h>
//#include "ipcopcode.h"
#include "common.h"
#define Face_Max_Point 62
#define Max_persons 10
#define Max_name_length 128
#define Max_error_msg_length 128
#define ALG_USERS_PATH "/root/ID"
#define ALG_USERS_IMG "img.jpg"

//DMM

#define OPC_DMM_CALIBRATION_START (OPC_ALG_START_CODE+0x01)   //server发送给DMM算法应用的标定开始消息标记 对应结构 OpcParm_t server->alg
#define OPC_DMM_ENTER_DETECTION (OPC_ALG_START_CODE+0x02)  //server发送给算法应用进入检测状态 server->alg
#define OPC_DMM_REGISTER       (OPC_ALG_START_CODE+0x03)    //server发送给算法应用进入录入状态 后面跟 FR_RegisterInfo_t server->alg
#define OPC_DMM_RECOGNITION       (OPC_ALG_START_CODE+0x04)    //server发送给算法应用进入登陆状态 server->alg
#define OPC_DMM_WARNING_EVT  ((OPC_ALG_START_CODE+0x05)|OPC_EVENT_FlAG)//发送疲劳检测警报状态 对应结构 OpcParm_t alg->MEDIA
#define OPC_DMM_RECOGNITION_RESULT  ((OPC_ALG_START_CODE+0x06)|OPC_EVENT_FlAG)//发送人脸识别结果 对应结构 OpcParm_t alg->MEDIA
//PDS
#define OPC_PDS_RESULT_EVENT  ((OPC_ALG_START_CODE+0x07)|OPC_EVENT_FlAG)//发送行人检测警报结果 对应结构 OpcParm_t  alg->MEDIA
//ALG
#define OPC_ALG_PARAMS_CHANGE (OPC_ALG_START_CODE+0x13)//参数改变 server->alg
#define OPC_ALG_NET_DISPLAY_EVENT ((OPC_ALG_START_CODE+0x14) | OPC_EVENT_FlAG)	//发送绘图信息给服务器 对应结构 algNetDisplay_t alg->server

typedef enum WarnState_{
    NoWarn=0,
    Fatigue,
    Distractoin,
    No_Driver,
    Smoke,
    Phone,
    Yawn,
    No_Mask,
    SunGlass,
    No_Seatbelt,
    Camera_Occlusion,
    FatigueL2, //creare客户要求
    DrinkEat,
	No_Helmet,
    OutSize,//RED
    AngleTrue,//green
    WarnStateMax
} WarnState;

#define WARN_STATE_STR_LIST {"none","fatigueWarning","distractionWarning","nodriverWarning","smokeWarning","phoneWarning","yawnWarning","nomaskWarning","sunglassWarning","noseatbeltWarning","fatigueWarning"}

#define PDS_WARN (OutSize+2)

typedef enum CaliState_{
    Cali_Free=0,
    Cali_Preparing,
    Cali_Calibrating,
    Cali_Success,
    Cali_Failed,
    Cali_Failed_Right,
    Cali_Failed_Left
} CaliState;

typedef struct imgInfo_
{
    unsigned short width;
    unsigned short height;
}imgInfo_t;

typedef struct Rect_
{
    short  x;
    short  y;
    short  width;
    short  height;
}Rect_t;

typedef struct DMMInfo_
{
    int ESR;
    int deg;
    WarnState warn;
    unsigned int rect_color;
    Rect_t FaceRoi;
    int point_count;
    int point[Face_Max_Point];

} DMMInfo_t;

//Event struct
typedef struct algInfo_
{
    imgInfo_t img;
    DMMInfo_t dmm;
} algInfo_t;


typedef struct CalibInfo_
{
    CaliState state;
} CalibInfo_t;


typedef struct FR_RegisterInfo
{
    char userDir[Max_name_length];					//用户目录
}FR_RegisterInfo_t;

#endif
