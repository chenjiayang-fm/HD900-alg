/****************************************************************
 File Name : stateAnalyzer.hpp
 Version: 1.0.0
 Author: lin jiaping
 Created: 2022-6-30
 Last Modified: 2022-6-30
 Description: DMS状态分析类头文件
 Function list:
 History :
 2022-6-30, 按照代码编写规范重新编写代码
 2022-7-22, 修改类成员命名，加前缀m_
 2022-8-8, 类CStateAnalyzer增加计时器置零API接口
 2022-11-09, 增加打哈欠,红外阻断墨镜,不带口罩警报时间
 2023-01-05, CStateAnalyzer构造函数添加算法类型接口eVerSionType，用于参展模式切换
 2023-01-09, CstateAnalyzer类SetAlgParam成员函数形参去除EVersionType eVerSionType
****************************************************************/
#ifndef _STATEANALYZER_H_
#define _STATEANALYZER_H_

#include "common.h"

#define FACE_DETECT_NUM 100            /*记录人脸检测状态的帧数*/
#define FUNC_STATE_NUM 100             /*记录各功能状态的帧数*/
#define FATIGUE_L2_STATE_NUM 100       /*二级疲劳，记录疲劳状态帧数*/
#define MAX_EYE_ECR_NUM 35             /*统计人眼闭合度阈值帧数*/
#define SORT_EYE_ECR_NUM 75            /*排序人眼闭合度的帧数*/
#define POSE_NUM 200                   /*统计头部姿态的帧数*/
/*定义计时器*/
typedef struct STTimer
{
    bool bTimerState;                  /*计时器状态，开与关*/
    double dTimerBegin;                /*计时器开始时间*/
    double dTimerEnd;                  /*计时器当前时间*/
    double dTimerLength;               /*计时器计时长度*/
    STTimer():bTimerState(false),dTimerBegin(0.0),dTimerEnd(0.0),dTimerLength(0.0){}
};
/*警报模块类*/
class CAlarmModule
{
private:
    int8_t m_ps8State[FUNC_STATE_NUM];    /*记录状态数组*/
    int32_t m_s32AlarmStartCount;         /*触发计时器开始计时的条件*/
    int32_t m_s32AlarmEndCount;           /*计时器停止计时的条件*/
    int32_t m_s32StateCount;              /*统计最新的状态帧数*/
    STTimer m_stAlarmTimer;               /*警报计时器*/
    int32_t m_s32AlarmSleepCount;         /*触发警报后，睡眠的长度*/
    float m_fAlarmTimeLength;             /*触发警报的时间条件*/
    float m_fSens;                        /*功能灵敏度*/
public:
    /**********************************************************
    功能：警报模块类构造函数
    参数：🕒 s32AlarmStart,计时器开始计时的帧数
          s32AlarmEnd,计时器停止计时的帧数
          s32StateCountLength,统计最新状态的帧数
          fAlarmTime,触发警报时长
    输出：无
    应用：创建警报模块类对象
    ************************************************************/
    CAlarmModule(int32_t s32AlarmStart, int32_t s32AlarmEnd,int32_t s32StateCountLength,
                 float fAlarmTime);
    CAlarmModule(){}
    ~CAlarmModule(){}
    /**********************************************************
    功能：记录当前警报状态
    参数：🕒 s8State, 警报状态
    输出：True, 触发警报,False,不触发警报
    应用：用于判断是否触发警报
    ************************************************************/
    bool FeedState(int8_t s8State);
    /**********************************************************
    功能：模块中记录状态数组和计时器归零
    参数：无
    输出：无
    应用：用于将警报模块置零
    ************************************************************/
    void TimerSetZero();
    /**********************************************************
    功能：设置警报时间
    参数：fAlermTime, 警报时间
    输出：无
    应用：用于设置警报时间
    ************************************************************/
    void SetAlarmTime(float fAlermTime){m_fAlarmTimeLength = fAlermTime;}
    /**********************************************************
    功能：设置警报睡眠帧数
    参数：s32Count, 睡眠帧数
    输出：无
    应用：用于设置睡眠帧数
    ************************************************************/
    void SetAlarmSleep(int32_t s32Count){m_s32AlarmSleepCount = s32Count;}
    /**********************************************************
    功能：设置警报灵敏度
    参数：fNewSens, 灵敏度
    输出：无
    应用：用于设置警报灵敏度
    ************************************************************/
    void SetSens(float fNewSens){m_fSens = fNewSens;}
};
/*状态分析类*/
class CStateAnalyzer
{
private:

    float m_fAlgSens;                                        /*算法灵敏度*/
    WarnState m_eState;                                      /*当前状态*/

    bool m_bDetectFaceOrNot;                                 /*是否检测到人脸*/
    bool m_bNoDiverSwitch;                                   /*是否打开离岗检测功能*/
    CAlarmModule m_cNDAlarmModule;                           /*离岗检测警报模块*/
    int32_t m_s32NofaceCount;                                /*连续检测不到人脸的帧数*/

    float m_pfPose[3];                                       /*头部姿态,俯仰角,水平角,旋转角*/
    int32_t m_s32LeftAngleThr;                               /*判断分心的水平角的左侧阈值*/
    int32_t m_s32RightAngleThr;                              /*判断分心的水平角的右侧阈值*/
    int32_t m_s32UpAngleThr;                                 /*判断分心的俯仰角的仰视阈值*/
    int32_t m_s32DownAngleThr;                               /*判断分心的俯仰角的俯视阈值*/
    CAlarmModule m_cDistAlarmModule;                         /*分心检测警报模块*/
    bool m_bDistSwitch;                                      /*是否打开分心检测功能*/
    float m_fGazeYaw;                                        /*视线方向，水平角*/
    float m_fGazePitch;                                      /*视线方向，俯仰角*/

    int32_t m_ps32PoseStatePitch[37];                        /*统计头部姿态的俯仰角*/
    int32_t m_ps32PoseStateYaw[37];                          /*统计头部姿态的水平角*/
    int32_t m_ps32PoseStateRoll[37];                         /*统计头部姿态的旋转角*/
    float m_pfPosePitch[POSE_NUM];                           /*记录各帧的头部姿态俯仰角*/
    float m_pfPoseYaw[POSE_NUM];                             /*记录各帧的头部姿态水平角*/
    float m_pfPoseRoll[POSE_NUM];                            /*记录各帧的头部姿态旋转角*/
    int32_t m_s32PoseCount;                                  /*统计头部姿态的帧数*/
    float m_fNormalPitch;                                    /*常态下头部姿态的俯仰角*/
    float m_fNormalYaw;                                      /*常态下头部姿态的水平角*/
    float m_fNormalRoll;                                     /*常态下头部姿态的旋转角*/
    bool m_bMarkPoseOrNot;                                   /*是否完成头部姿态的记录*/
    STTimer m_stPoseTimer;                                   /*应于计时更新头部姿态*/
    float m_fPoseTimerLength;                                /*头部姿态统计间距时间*/

    float m_fEcr;                                            /*眼睛闭合度*/
    STTimer m_stEcrThrUpdataTimer;                           /*常态下眼睛闭合度更新的计时器*/
    int32_t m_u32EcrNum;                                     /*统计人眼闭合度的帧数*/
    float m_pfSortEcr[SORT_EYE_ECR_NUM];                     /*记录人眼闭合度，用于计算常态下平均眼睛闭合度*/
    float m_fEcrAvg;                                         /*常态下眼睛闭合度*/
    bool m_bFatigueSwitch;                                   /*是否打开疲劳检测功能*/
    CAlarmModule m_cFatigueAlarmModule;                      /*疲劳检测警报模块*/
    float m_fCloseEyeScore;                                  /*闭眼检测分数*/
    float m_fOpenEyeScore;                                   /*开眼检测分数*/

    int32_t m_s32FatigueLevel;                               /*疲劳检测模式，一级or二级*/
    int32_t m_s32FatigueAlarmType;                           /*疲劳警报类型，一级or二级*/
    int8_t m_ps8FatigueL2State[FATIGUE_L2_STATE_NUM];        /*记录二级疲劳状态数组*/
    STTimer m_stFatigueL2AlarmTimer;                         /*二级疲劳警报计时器*/
    STTimer m_stFatigueSleepTimer;                           /*二级疲劳触发警报后睡眠计时器*/
    int32_t m_s32FatigueL2Count;                             /*二级疲劳状态统计帧数*/
    bool m_bFatigueL2Switch;                                 /*是否打开二级疲劳检测功能*/

    float m_fPhoneScore;                                     /*电话检测分数*/
    CAlarmModule m_cPhoneAlarmModule;                        /*电话检测警报模块*/
    bool m_bPhoneSwitch;                                     /*是否打开电话检测功能*/

    float m_fDrinkEatScore;                                     /*吃喝东西检测分数*/
    CAlarmModule m_cDrinkEatAlarmModule;                        /*吃喝东西检测警报模块*/
    bool m_bDrinkEatSwitch;                                     /*是否打开吃喝东西检测功能*/

    float m_fSmokeScore;                                     /*抽烟检测分数*/
    CAlarmModule m_cSmokeAlarmModule;                        /*抽烟检测警报模块*/
    bool m_bSmokeSwitch;                                     /*是否打开抽烟检测功能*/

    bool m_bYawnOrNot;                                       /*是否打哈欠*/
    CAlarmModule m_cYawnAlarmModule;                         /*打哈欠检测警报模块*/
    bool m_bYawnSwitch;                                      /*是否打开打哈欠检测功能*/


    bool m_bNoMaskOrNot;                                     /*是否不带口罩*/
    bool m_bNoMaskSwitch;                                    /*是否打开不带口罩检测功能*/
    CAlarmModule m_cNoMaskAlarmModule;                       /*不带口罩检测警报模块*/

    float m_fNormalglassScore;                               /*普通眼镜检测分数*/
    float m_fSunglassScore;                                  /*红外阻断眼镜检测分数*/
    bool m_bSunglassSwitch;                                  /*是否打开红外阻断眼镜检测*/
    CAlarmModule m_cSunglassAlarmModule;                     /*红外阻断眼镜检测警报模块*/

    float m_fSeatbeltScore;                                  /*安全带检测分数*/
    bool m_bSeatbeltSwitch;                                  /*是否打开安全带检测功能*/
    CAlarmModule m_cSeatbeltAlarmModule;                     /*安全带检测警报模块*/

    float m_fHelmetScore;                                  /*安全帽检测分数*/
    bool m_bHelmetSwitch;                                  /*是否打开安全帽检测功能*/
    CAlarmModule m_cHelmetAlarmModule;                     /*安全帽检测警报模块*/

    bool m_bShelterOrNot;                                    /*是否遮挡摄像头*/
    bool m_bShelterSwitch;                                   /*是否打开遮挡摄像头检测功能*/
    int32_t m_s32ShelterCount;                               /*遮挡摄像头连续帧数*/
    CAlarmModule m_cShelterAlarmModule;                      /*遮挡摄像头检测警报模块*/

    float m_fEcrThr;                                         /*眼睛闭合度阈值*/
    float m_fFatigueTime;                                    /*疲劳警报时间*/
    float m_fDictThrLeft;                                    /*分心水平角左侧阈值*/
    float m_fDictThrRight;                                   /*分心水平角右侧阈值*/
    float m_fDictTime;                                       /*分心警报时间*/
    float m_fNodriverTime;                                   /*离岗警报时间*/
    float m_fSmTime;                                         /*抽烟警报时间*/
    float m_fSmThr;                                          /*抽烟检测阈值*/
    float m_fPhoneTime;                                      /*电话警报时间*/
    float m_fPhoneThr;                                       /*电话检测阈值*/
    float m_fDrinkEatTime;                                   /*吃喝东西警报时间*/
    float m_fDrinkEatThr;                                    /*吃喝东西检测阈值*/
    float m_fCenterAngle;                                    /*标定角度*/
    float m_fSeatbeltTime;                                   /*安全带警报时间*/
    float m_fSeatbeltThr;                                    /*安全带检测阈值*/
    float m_fHelmetTime;                                     /*安全帽警报时间*/
    float m_fHelmetThr;                                      /*安全帽检测阈值*/
    float m_fShelterTime;                                    /*遮挡摄像头警报时间*/
    float m_fYawnTime;                                       /*打哈欠警报时间*/
    float m_fSunGlassTime;                                   /*红外阻断墨镜警报时间*/
    float m_fNoMaskTime;                                     /*不带口罩警报时间*/
    EVersionType m_eVersionType;                             /*版本类型*/
    /***********************************************************
    功能：进行分心检测
    参数：无
    返回值：True, 触发警报,False, 不触发警报
    应用：进行分心检测
    ***********************************************************/
    bool distDetect();
    /***********************************************************
    功能：统计头部姿态，水平角，俯仰角，旋转角
    参数：无
    返回值：无
    应用：进行统计头部姿态
    ***********************************************************/
    void poseStateMark();
    /***********************************************************
    功能：进行离岗检测
    参数：无
    返回值：True, 触发警报,False, 不触发警报
    应用：进行离岗检测
    ***********************************************************/
    bool noDriverDetect();
    /***********************************************************
    功能：进行疲劳检测
    参数：无
    返回值：True, 触发警报,False, 不触发警报
    应用：进行疲劳检测
    ***********************************************************/
    bool fatigueDetect();
    /***********************************************************
    功能：进行疲劳二级检测
    参数：无
    返回值：True, 触发警报,False, 不触发警报
    应用：进行疲劳二级检测
    ***********************************************************/
    bool fatigueDetectLevel2();
    /***********************************************************
    功能：进行抽烟检测
    参数：无
    返回值：True, 触发警报,False, 不触发警报
    应用：进行抽烟检测
    ***********************************************************/
    bool smokeDetect();
    /***********************************************************
    功能：进行电话检测
    参数：无
    返回值：True, 触发警报,False, 不触发警报
    应用：进行电话检测
    ***********************************************************/
    bool phoneDetect();
    /***********************************************************
    功能：进行吃喝东西检测
    参数：无
    返回值：True, 触发警报,False, 不触发警报
    应用：进行吃喝东西检测
    ***********************************************************/
    bool drinkEatDetect();
    /***********************************************************
    功能：进行打哈欠检测
    参数：无
    返回值：True, 触发警报,False, 不触发警报
    应用：进行打哈欠检测
    ***********************************************************/
    bool yawnDetect();
    /***********************************************************
    功能：进行不带口罩检测
    参数：无
    返回值：True, 触发警报,False, 不触发警报
    应用：进行不带口罩检测
    ***********************************************************/
    bool noMaskDetect();
    /***********************************************************
    功能：进行红外阻断眼镜检测
    参数：无
    返回值：True, 触发警报,False, 不触发警报
    应用：进行红外阻断眼镜检测
    ***********************************************************/
    bool sunglassDetect();
    /***********************************************************
    功能：进行安全带检测
    参数：无
    返回值：True, 触发警报,False, 不触发警报
    应用：进行安全带检测
    ***********************************************************/
    bool seatbeltDetect();
    /***********************************************************
    功能：进行摄像头遮挡检测
    参数：无
    返回值：True, 触发警报,False, 不触发警报
    应用：进行摄像头遮挡检测
    ***********************************************************/
    bool shelterDetect();
    /***********************************************************
    功能：进行安全帽检测
    参数：无
    返回值：True, 触发警报,False, 不触发警报
    应用：进行安全帽检测
    ***********************************************************/
    bool helmetDetect();
public:
    /***********************************************************
    功能：状态分析类构造函数
    参数：s32CenterAngle,标定水平角度
         s32GazePitchAngle,标定视线俯仰角
         bNoDriverSW,离岗功能开关
         bFatigueSW,疲劳检测功能开关
         bDistractSW,分心检测功能开关
         bSmokeSW,抽烟检测开关
         bPhoneSW,电话检测开关
         bYawnSW,打哈欠检测开关
         bNoMaskSW,不带口罩检测开关
         bFatigueL2SW,二级疲劳检测开关
         bSunglassSW,红外阻断眼镜检测开关
         bSeatbeltSW,安全带检测开关
         bShelterSW,摄像头遮挡检测开关
         bDrinkEatSW,吃喝东西检测开关
         bHelmetSW,安全帽检测开关
         eVerSionType, 算法模式, 默认为E_GENERAL
    返回值：无
    应用：创建状态分析类对象
    ***********************************************************/
    CStateAnalyzer(int32_t s32CenterAngle, int32_t s32GazePitchAngle, bool bNoDriverSW,bool bFatigueSW,bool bDistractSW,
                   bool bSmokeSW,bool bPhoneSW,bool bYawnSW,bool bNoMaskSW,bool bFatigueL2SW,
                   bool bSunglassSW,bool bSeatbeltSW,bool bShelterSW, bool bDrinkEatSW,bool bHelmetSW, EVersionType eVerSionType = E_GENERAL);
    ~CStateAnalyzer(){}
    /***********************************************************
    功能：各个功能的状态分析
    参数：dmmP,DMS检测信息,thr,灵敏度
    返回值：无
    应用：用于分析各个功能的警报状态
    ***********************************************************/
    void StateAnalysis(dmmParam& dmmP,float thr);
    /***********************************************************
    功能：各个功能参数初始化
    参数：无
    返回值：无
    应用：用于初始化各个功能
    ***********************************************************/
    void InitStateParam();
    /***********************************************************
    功能：标定角度改变,更新分心水平角左右阈值
    参数：s32CenterA,新的标定角度
    返回值：无
    应用：用于更新分心水平角左右阈值
    ***********************************************************/
    void DistractParamChange(int32_t s32CenterAngle, int32_t s32GazePitchAngle)
    {
        m_fCenterAngle = s32CenterAngle;
        m_s32LeftAngleThr = m_fCenterAngle - m_fDictThrLeft;
        m_s32RightAngleThr = m_fCenterAngle + m_fDictThrRight;
        m_s32DownAngleThr = s32GazePitchAngle - 30;
        m_s32UpAngleThr = s32GazePitchAngle + 30;
    }

    /***********************************************************
    功能：获取当前警报状态
    参数：无
    返回值：警报类型
    应用：用于获取当前警报状态
    ***********************************************************/
    WarnState GetState(){return m_eState;}
    /***********************************************************
    功能：切换抽烟检测功能开关
    参数：SwitchState, 开关的状态
    返回值：无
    应用：用于切换抽烟检测功能开关
    ***********************************************************/
    void SetSmokeS(bool SwitchState){m_bSmokeSwitch = SwitchState;}
    /***********************************************************
    功能：切换电话检测功能开关
    参数：SwitchState, 开关的状态
    返回值：无
    应用：用于切换电话检测功能开关
    ***********************************************************/
    void SetTelephoneS(bool SwitchState){m_bPhoneSwitch = SwitchState;}
    /***********************************************************
    功能：切换吃喝东西检测功能开关
    参数：SwitchState, 开关的状态
    返回值：无
    应用：用于切换吃喝东西检测功能开关
    ***********************************************************/
    void SetDrinkEatS(bool SwitchState){m_bDrinkEatSwitch = SwitchState;}
    /***********************************************************
    功能：切换离岗检测功能开关
    参数：SwitchState, 开关的状态
    返回值：无
    应用：用于切换离岗检测功能开关
    ***********************************************************/
    void SetNodriverS(bool SwitchState){m_bNoDiverSwitch = SwitchState;}
    /***********************************************************
    功能：切换疲劳检测功能开关
    参数：SwitchState, 开关的状态
    返回值：无
    应用：用于切换疲劳检测功能开关
    ***********************************************************/
    void SetFatigueS(bool SwitchState){m_bFatigueSwitch = SwitchState;}
    /***********************************************************
    功能：切换分心检测功能开关
    参数：SwitchState, 开关的状态
    返回值：无
    应用：用于切换分心检测功能开关
    ***********************************************************/
    void SetDistractS(bool SwitchState){m_bDistSwitch = SwitchState;}
    /***********************************************************
    功能：切换打哈欠检测功能开关
    参数：SwitchState, 开关的状态
    返回值：无
    应用：用于切换打哈欠检测功能开关
    ***********************************************************/
    void SetYawnS(bool SwitchState){m_bYawnSwitch = SwitchState;}
    /***********************************************************
    功能：切换红外阻断眼镜检测功能开关
    参数：SwitchState, 开关的状态
    返回值：无
    应用：用于切换红外阻断眼镜检测功能开关
    ***********************************************************/
    void SetSunGlassS(bool SwitchState){m_bSunglassSwitch = SwitchState;}
    /***********************************************************
    功能：切换安全带检测功能开关
    参数：SwitchState, 开关的状态
    返回值：无
    应用：用于切换安全带检测功能开关
    ***********************************************************/
    void SetSeatbeltS(bool SwitchState){m_bSeatbeltSwitch = SwitchState;}
    /***********************************************************
    功能：切换摄像头遮挡检测功能开关
    参数：SwitchState, 开关的状态
    返回值：无
    应用：用于切换摄像头遮挡检测功能开关
    ***********************************************************/
    void SetShelterS(bool SwitchState){m_bShelterSwitch = SwitchState;}
    /***********************************************************
    功能：切换安全帽检测功能开关
    参数：SwitchState, 开关的状态
    返回值：无
    应用：用于切换摄像头遮挡检测功能开关
    ***********************************************************/
    void SetHelmetS(bool SwitchState){m_bHelmetSwitch = SwitchState;}
    /***********************************************************
    功能：切换无口罩检测功能开关
    参数：SwitchState, 开关的状态
    返回值：无
    应用：用于切换无口罩检测功能开关
    ***********************************************************/
    void SetnoMaskS(bool SwitchState){m_bNoMaskSwitch = SwitchState;}
    /***********************************************************
    功能：获取电话检测功能开关状态
    参数：无
    返回值：true, 功能打开, false, 功能关闭
    应用：用于获取电话检测功能开关状态
    ***********************************************************/
    bool GetTelephoneS(){return m_bPhoneSwitch;}
    /***********************************************************
    功能：获取吃喝东西检测功能开关状态
    参数：无
    返回值：true, 功能打开, false, 功能关闭
    应用：用于获取电话检测功能开关状态
    ***********************************************************/
    bool GetDrinkEatS(){return m_bDrinkEatSwitch;}
    /***********************************************************
    功能：获取抽烟检测功能开关状态
    参数：无
    返回值：true, 功能打开, false, 功能关闭
    应用：用于获取抽烟检测功能开关状态
    ***********************************************************/
    bool GetSmokeS(){return m_bSmokeSwitch;}
    /***********************************************************
    功能：获取不带口罩检测功能开关状态
    参数：无
    返回值：true, 功能打开, false, 功能关闭
    应用：用于获取不带口罩检测功能开关状态
    ***********************************************************/
    bool GetNoMaskS(){return m_bNoMaskSwitch;}
    /***********************************************************
    功能：获取红外阻断眼镜检测功能开关状态
    参数：无
    返回值：true, 功能打开, false, 功能关闭
    应用：用于获取红外阻断眼镜检测功能开关状态
    ***********************************************************/
    bool GetSunGlassS(){return m_bSunglassSwitch;}
    /***********************************************************
    功能：获取安全带检测功能开关状态
    参数：无
    返回值：true, 功能打开, false, 功能关闭
    应用：用于获取安全带检测功能开关状态
    ***********************************************************/
    bool GetSeatbeltS(){return m_bSeatbeltSwitch;}
    /***********************************************************
    功能：获取摄像头遮挡检测功能开关状态
    参数：无
    返回值：true, 功能打开, false, 功能关闭
    应用：用于获取摄像头遮挡检测功能开关状态
    ***********************************************************/
    bool GetShelterS(){return m_bShelterSwitch;}
    /***********************************************************
    功能：获取安全帽检测功能开关状态
    参数：无
    返回值：true, 功能打开, false, 功能关闭
    应用：用于获取摄像头遮挡检测功能开关状态
    ***********************************************************/
    bool GetHelmetS(){return m_bHelmetSwitch;}
    /***********************************************************
    功能：设置算法参数
    参数：fEcrThrNew,判断闭眼的阈值, 0.0-1.0
         fFatigueTimeNew,疲劳警报时间, 1-6
         fDictThrLeftNew,左边分心偏角阈值, 0-75
         fDictThrRightNew,右边分心偏角阈值, 0-75
         fDictTimeNew,分心警报时间, 1-9
         fNoDriverTimeNew,离岗检测警报时间, 1-30
         fSmThrNew,抽烟检测阈值, 0.0-1.0
         fSmTimeNew,抽烟检测警报时间, 1-5
         fPhoneThrNew,电话检测阈值, 0.0-1.0
         fPhoneTimeNew,电话检测警报时间, 1-5
         fSeatbeltThrNew,安全带检测阈值, 0.0-1.0
         fSeatbeltTimeNew,安全带检测警报时间, 1-30
         fShelterTimeNew, 遮挡摄像头警报时间, 1-30
         fDrinkEatThrNew,吃喝东西检测阈值, 0.0-1.0
         fDrinkEatTimeNew,吃喝东西检测警报时间, 1-5
         fYawnTimeNew, 打哈欠检测警报时间,1-5
         fNoMaskTimeNew, 不带口罩检测警报时间，1-10
         fSunGlassTimeNew, 红外阻断墨镜检测警报时间，1-10
         fHelmetThrNew,安全带检测阈值, 0.0-1.0
         fHelmetTimeNew,安全带检测警报时间, 1-30
         fPoseTimeLength,头部姿态统计间距
    返回值：无
    应用：用于设置算法参数
    ***********************************************************/
    void SetAlgParam(float fEcrThrNew=0.45,float fFatigueTimeNew=2.0,float fDictThrLeftNew=35,
                     float fDictThrRightNew=35, float fDictTimeNew=5.0,float fNoDriverTimeNew=15.0,
                     float fSmThrNew=0.35,float fSmTimeNew=2,float fPhoneThrNew=0.60,float fPhoneTimeNew=3.0,
                     float fSeatbeltThrNew=0.48,float fSeatbeltTimeNew=10.0,float fShelterTimeNew=10.0,float fDrinkEatThrNew=0.40,
                     float fDrinkEatTimeNew=3.0, float fYawnTimeNew = 2.0, float fNoMaskTimeNew=5.0, float fSunGlassTimeNew = 5.0,
                     float fHelmetThrNew=0.5,float fHelmetTimeNew=10.0,float fPoseTimeLength=300);

    /***********************************************************
    功能：将所有计时器置零
    参数：无
    返回值：无
    应用：当速度低于工作阈值，将计时器置零；
    ***********************************************************/
    void SetTimerZero();
    /***********************************************************
    功能：获取获取记录头部姿态角度
    参数：pfPose,头部姿态角度
    返回值：true，角度可用，false，角度不可用；
    应用：获取头部记录姿态
    ***********************************************************/
    bool getAngle(float* pfPose);
};
#endif
