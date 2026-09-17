#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/syscall.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <pthread.h>
#include <ctype.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>
#include <bsd/libbsd.h>

#include "print.h"
#include "common.h"
#include "hi_common.h"
//#include "hi_common_ive.h"
//#include "hi_common_vgs.h"
#include "hi_type.h"
#include "mpi_vgs.h"
#include "mpi_region.h"
//#include "hi_vgs.h"
#include "hi_ive.h"
#include "mpi_ive.h"
#include "mpi_sys.h"
#include "mpi_vpss.h"
#include "config.h"
#include "board.h"
#include "op.h"
#include "msg.h"
#include "media.h"
#include "safefunc.h"
#include "sharefifo.h"
#include "control.h"


#include "bsd.h"
extern "C" { 
#include "./common/png_process.h"
#include "./common/common_zip.h"
#include "mpp_font.h"
#include "./common/paint.h"
}



#define max(x,y) (((x)>(y))?(x):(y))
#define min(x,y) (((x)<(y))?(x):(y))
#define even(x)  (((x)%2==0)?(x):((x)-1))

int ipsys_log_level = SV_DEBUG;
BSD_INFO_S BsdInfo = {0};

/*****************************************************************************
 * 函数功能：拷贝图像
 * 输入参数：目标图像、目标图像宽度、目标图像高度
 *          源图像、源图像宽度、源图像高度
 *          插入坐标x、插入坐标y、像素字节
 * 返回值  : SV_SUCCESS - 成功
             SV_FAILURE - 失败
 * 注意：坐标x对应图像宽度、坐标y对应图像高度
 * ***************************************************************************/
sint32 img_memcopy(char *img_dst, int width_dst, int height_dst, 
                  char *img_src, int width_src, int height_src, 
                  int x_insert, int y_insert, int byte)
{
    int i;
    if(x_insert < 0 || y_insert < 0){
        print_level(SV_ERROR,"img_memcopy error xinsert:%d, y_insert:%d\n",x_insert, y_insert);
        return SV_FAILURE;
    }
    if(x_insert+width_src > width_dst || y_insert+height_src > height_dst)
    {
        print_level(SV_ERROR,"img_memcopy error [%#x]\n",2);
        return SV_FAILURE;
    }
    img_dst += byte*(x_insert + y_insert*width_dst);
    for(i=0;i<height_src;i++)
    {
        memcpy(img_dst, img_src, width_src*byte);
        img_dst += byte*width_dst;
        img_src += byte*width_src;
    }
    return SV_SUCCESS;
}


sint32 BSD_ALG_Init(BSD_INFO_S* pBsdInfo)
{
    sint32 s32Ret;
    float *fPoints;
    int bsd_th;

    pBsdInfo->stALG.fPoint[0] = BSD_ROI_x1;
    pBsdInfo->stALG.fPoint[1] = BSD_ROI_y1;
    pBsdInfo->stALG.fPoint[2] = BSD_ROI_x2;
    pBsdInfo->stALG.fPoint[3] = BSD_ROI_y2;
    pBsdInfo->stALG.fPoint[4] = BSD_ROI_x3;
    pBsdInfo->stALG.fPoint[5] = BSD_ROI_y3;
    pBsdInfo->stALG.fPoint[6] = BSD_ROI_x4;
    pBsdInfo->stALG.fPoint[7] = BSD_ROI_y4;

    fPoints = pBsdInfo->stALG.fPoint;
    bsd_th = pBsdInfo->stALG.bsd_th;

    s32Ret = InitBSD(fPoints, 8);
    if(s32Ret != SV_TRUE)
    {
        print_level(SV_ERROR, "InitBSD fail\n");
        return SV_FAILURE;
    }

    s32Ret = SetTH(bsd_th);
    if(s32Ret != SV_TRUE)
    {
        print_level(SV_ERROR, "SetTH fail\n");
        return SV_FAILURE;
    }

    return SV_SUCCESS;
}

sint32 BSD_Init(BSD_INFO_S* pBsdInfo)
{
    sint32 s32Ret = 0;
    
    s32Ret = BSD_PARAM_Init(pBsdInfo);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "BSD_PARAM_Init fail\n");
        return SV_FAILURE;
    }

    s32Ret = BSD_Image_Init(pBsdInfo);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "BSD_Image_Init fail!\n");
        return SV_FAILURE;
    }

#if BSD_MESSAGE_ENABLE
    s32Ret = BSD_Font_Init(pBsdInfo);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "BSD_Font_Init fail!\n");
        return SV_FAILURE;
    }
#endif

    s32Ret = BSD_OSD_Fini(pBsdInfo);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "BSD_OSD_Fini fail\n");
    }

    s32Ret = BSD_OSD_Init(pBsdInfo);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "BSD_OSD_Init fail\n");
        return SV_FAILURE;
    }
    
    s32Ret = BSD_ALG_Init(pBsdInfo);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "BSD_ALG_Init fail!\n");
        return SV_FAILURE;
    }


    s32Ret = BSD_Image_Update(pBsdInfo);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "BSD_ALG_Update fail\n");
        return SV_FAILURE;
    }

#if BSD_MESSAGE_ENABLE 
    s32Ret = BSD_Font_Update(pBsdInfo);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "BSD_ALG_Update fail\n");
        return SV_FAILURE;
    }
#endif
    s32Ret = BSD_Rect_Update(pBsdInfo);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "BSD_ALG_Update fail\n");
        return SV_FAILURE;
    }

    s32Ret = BSD_OSD_Update(pBsdInfo);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "BSD_OSD_Update fail\n");
        return SV_FAILURE;
    }

    return SV_SUCCESS;
}

sint32 BSD_PARAM_Init(BSD_INFO_S* pBsdInfo)
{
    float fPoints[8] = {BSD_ROI_x1, BSD_ROI_y1, BSD_ROI_x2, BSD_ROI_y2,\
                        BSD_ROI_x3, BSD_ROI_y3, BSD_ROI_x4, BSD_ROI_y4
                        };
    float MAX_RECT[4];  //表示包围RGN的矩形
    float eps = 0.005;  //框的精度，注：不能精确画矩形框，否则在边缘区域会无法显示
    MAX_RECT[0] = 0.0;
    MAX_RECT[1] = min(fPoints[1],fPoints[7]);
    MAX_RECT[2] = 1.0;
    MAX_RECT[3] = max(fPoints[3],fPoints[5]);

#if BSD_RGN_ENABLE
    print_level(SV_INFO, "RECT:(%f,%f),(%f,%f)\n",MAX_RECT[0],MAX_RECT[1],MAX_RECT[2],MAX_RECT[3]);
    pBsdInfo->stSTA.point_paint[0].x_scale = (fPoints[0]-MAX_RECT[0]) / (MAX_RECT[2]-MAX_RECT[0]) + eps;
    pBsdInfo->stSTA.point_paint[0].y_scale = (fPoints[1]-MAX_RECT[1]) / (MAX_RECT[3]-MAX_RECT[1]) + eps;
    pBsdInfo->stSTA.point_paint[1].x_scale = (fPoints[2]-MAX_RECT[0]) / (MAX_RECT[2]-MAX_RECT[0]) + eps;
    pBsdInfo->stSTA.point_paint[1].y_scale = (fPoints[3]-MAX_RECT[1]) / (MAX_RECT[3]-MAX_RECT[1]) - eps;
    pBsdInfo->stSTA.point_paint[2].x_scale = (fPoints[4]-MAX_RECT[0]) / (MAX_RECT[2]-MAX_RECT[0]) - eps;
    pBsdInfo->stSTA.point_paint[2].y_scale = (fPoints[5]-MAX_RECT[1]) / (MAX_RECT[3]-MAX_RECT[1]) - eps;
    pBsdInfo->stSTA.point_paint[3].x_scale = (fPoints[6]-MAX_RECT[0]) / (MAX_RECT[2]-MAX_RECT[0]) - eps;
    pBsdInfo->stSTA.point_paint[3].y_scale = (fPoints[7]-MAX_RECT[1]) / (MAX_RECT[3]-MAX_RECT[1]) + eps;
    pBsdInfo->stSTA.point_paint[4].x_scale = (fPoints[0]-MAX_RECT[0]) / (MAX_RECT[2]-MAX_RECT[0]) + eps;
    pBsdInfo->stSTA.point_paint[4].y_scale = (fPoints[1]-MAX_RECT[1]) / (MAX_RECT[3]-MAX_RECT[1]) + eps;

    print_level(SV_INFO, "point1:(%f,%f), point2:(%f,%f), point3(%f,%f), point4(%f,%f)\n",
                pBsdInfo->stSTA.point_paint[0].x_scale, pBsdInfo->stSTA.point_paint[0].y_scale,
                pBsdInfo->stSTA.point_paint[1].x_scale, pBsdInfo->stSTA.point_paint[1].y_scale,
                pBsdInfo->stSTA.point_paint[2].x_scale, pBsdInfo->stSTA.point_paint[2].y_scale,
                pBsdInfo->stSTA.point_paint[3].x_scale, pBsdInfo->stSTA.point_paint[3].y_scale);

    pBsdInfo->stRGN.pu32Handle[0] = BSD_HANDEL_x1;
    pBsdInfo->stRGN.pu32Handle[1] = BSD_HANDEL_x2;
    pBsdInfo->stRGN.pu32Handle[2] = BSD_HANDEL_x3;

    pBsdInfo->stRGN.ps32ChnId[0]  = BSD_CHNID_x1;
    pBsdInfo->stRGN.ps32ChnId[1]  = BSD_CHNID_x2;
    pBsdInfo->stRGN.ps32ChnId[2]  = BSD_CHNID_x3;

    pBsdInfo->stRGN.pstRect[0].s32X       = even((sint32)(CHIND_WIDTH_x1 *MAX_RECT[0]));
    pBsdInfo->stRGN.pstRect[0].s32Y       = even((sint32)(CHIND_HEIGHT_x1*MAX_RECT[1]));
    pBsdInfo->stRGN.pstRect[0].u32Width   = even((sint32)(CHIND_WIDTH_x1 *(MAX_RECT[2]-MAX_RECT[0])));
    pBsdInfo->stRGN.pstRect[0].u32Height  = even((sint32)(CHIND_HEIGHT_x1*(MAX_RECT[3]-MAX_RECT[1])));

    pBsdInfo->stRGN.pstRect[1].s32X       = even((sint32)(CHIND_WIDTH_x2  *MAX_RECT[0]));
    pBsdInfo->stRGN.pstRect[1].s32Y       = even((sint32)(CHIND_HEIGHT_x2 *MAX_RECT[1]));
    pBsdInfo->stRGN.pstRect[1].u32Width   = even((sint32)(CHIND_WIDTH_x2  *(MAX_RECT[2]-MAX_RECT[0])));
    pBsdInfo->stRGN.pstRect[1].u32Height  = even((sint32)(CHIND_HEIGHT_x2 *(MAX_RECT[3]-MAX_RECT[1])));

    pBsdInfo->stRGN.pstRect[2].s32X       = even((sint32)(CHIND_WIDTH_x3  *MAX_RECT[0]));
    pBsdInfo->stRGN.pstRect[2].s32Y       = even((sint32)(CHIND_HEIGHT_x3 *MAX_RECT[1]));
    pBsdInfo->stRGN.pstRect[2].u32Width   = even((sint32)(CHIND_WIDTH_x3  *(MAX_RECT[2]-MAX_RECT[0])));
    pBsdInfo->stRGN.pstRect[2].u32Height  = even((sint32)(CHIND_HEIGHT_x3 *(MAX_RECT[3]-MAX_RECT[1])));
    
    pBsdInfo->stSTA.pyaxis[0]            = max(0, pBsdInfo->stRGN.pstRect[0].u32Height - BSD_HEIGHT_x1);
    pBsdInfo->stSTA.pheight[0]           = BSD_HEIGHT_x1;
    pBsdInfo->stSTA.pwidth[0]            = BSD_WIDTH_x1;
    pBsdInfo->stSTA.pheight_sub[0]       = BSD_SUB_HEIGHT_x1;
    pBsdInfo->stSTA.pwidth_sub[0]        = BSD_SUB_WIDTH_x1;

    pBsdInfo->stSTA.pyaxis[1]            = max(0, pBsdInfo->stRGN.pstRect[1].u32Height - BSD_HEIGHT_x2);
    pBsdInfo->stSTA.pheight[1]           = BSD_HEIGHT_x2;
    pBsdInfo->stSTA.pwidth[1]            = BSD_WIDTH_x2;
    pBsdInfo->stSTA.pheight_sub[1]       = BSD_SUB_HEIGHT_x2;
    pBsdInfo->stSTA.pwidth_sub[1]        = BSD_SUB_WIDTH_x2;

    pBsdInfo->stSTA.pyaxis[2]            = max(0, pBsdInfo->stRGN.pstRect[2].u32Height - BSD_HEIGHT_x3);
    pBsdInfo->stSTA.pheight[2]           = BSD_HEIGHT_x3;
    pBsdInfo->stSTA.pwidth[2]            = BSD_WIDTH_x3;
    pBsdInfo->stSTA.pheight_sub[2]       = BSD_SUB_HEIGHT_x3;
    pBsdInfo->stSTA.pwidth_sub[2]        = BSD_SUB_WIDTH_x3;
#endif
    
#if BSD_MESSAGE_ENABLE
    pBsdInfo->stRGN.pu32Handle[3] = BSD_HANDEL_MESSAGE_x1;
    pBsdInfo->stRGN.pu32Handle[4] = BSD_HANDEL_MESSAGE_x2;
    pBsdInfo->stRGN.pu32Handle[5] = BSD_HANDEL_MESSAGE_x3;
    pBsdInfo->stRGN.ps32ChnId[3]  = BSD_CHNID_x1;
    pBsdInfo->stRGN.ps32ChnId[4]  = BSD_CHNID_x2;
    pBsdInfo->stRGN.ps32ChnId[5]  = BSD_CHNID_x3;

    pBsdInfo->stRGN.pstRect[3].s32X       = CHIND_WIDTH_x1    / 40 - 8;
    pBsdInfo->stRGN.pstRect[3].s32Y       = CHIND_HEIGHT_x1   / 5 + 20;
    pBsdInfo->stRGN.pstRect[3].u32Height  = 38;
    pBsdInfo->stRGN.pstRect[3].u32Width   = 38*3;
    
    pBsdInfo->stRGN.pstRect[4].s32X       = CHIND_WIDTH_x2    / 40;
    pBsdInfo->stRGN.pstRect[4].s32Y       = CHIND_HEIGHT_x2   / 5 + 10;
    pBsdInfo->stRGN.pstRect[4].u32Height  = 20;
    pBsdInfo->stRGN.pstRect[4].u32Width   = 20*3;

    pBsdInfo->stRGN.pstRect[5].s32X       = CHIND_WIDTH_x3    / 40 - 14;
    pBsdInfo->stRGN.pstRect[5].s32Y       = CHIND_HEIGHT_x3   / 5;
    pBsdInfo->stRGN.pstRect[5].u32Height  = 76;
    pBsdInfo->stRGN.pstRect[5].u32Width   = 76*3;
#endif
    
    for (int i=0; i<BSD_CHIND_NUM; i++)
    {
        pBsdInfo->stSTA.pvir_bsd[i] = malloc(sizeof(s16)* pBsdInfo->stRGN.pstRect[i].u32Height * \
                                                    pBsdInfo->stRGN.pstRect[i].u32Width);
        pBsdInfo->stSTA.pvir_sub_bsd[i] = malloc(sizeof(s16)* pBsdInfo->stSTA.pheight_sub[i] * \
                                                    pBsdInfo->stSTA.pwidth_sub[i]);
        
        memset(pBsdInfo->stSTA.pvir_bsd[i], 0, sizeof(s16)* pBsdInfo->stRGN.pstRect[i].u32Height * \
                                                    pBsdInfo->stRGN.pstRect[i].u32Width);
        memset(pBsdInfo->stSTA.pvir_sub_bsd[i], 0,sizeof(s16)* pBsdInfo->stSTA.pheight_sub[i] * \
                                                    pBsdInfo->stSTA.pwidth_sub[i]);
    }

    return SV_SUCCESS;
}

sint32 BSD_Image_Init(BSD_INFO_S* pBsdInfo)
{
    int i,j;
    char op_string[100];
    char bsd_path[30];
    
    sprintf(op_string, "tar -xzvf %s -C /var", BSD_ZIP_PATH);
    SAFE_System(op_string, NORMAL_WAIT_TIME);
    
    for(i=0; i<BSD_CHIND_NUM; i++)
    {
        for(j=0; j<5; j++)
        {
            sprintf(bsd_path, BSD_PATH, j+1, i+1);
            load_png_image_toARGB1555(\
                                    bsd_path,
                                    (s16*)pBsdInfo->stSTA.pvir_sub_bsd[i],\
                                    (int)pBsdInfo->stSTA.pwidth_sub[i],\
                                    (int)pBsdInfo->stSTA.pheight_sub[i]\
            );
            pBsdInfo->stSTA.psub_zip_len[i*5+j] = \
                                    fastlz_compress_malloc(\
                                    pBsdInfo->stSTA.pvir_sub_bsd[i],\
                                    (sizeof(s16)*(int)pBsdInfo->stSTA.pwidth_sub[i]*\
                                    (int)pBsdInfo->stSTA.pheight_sub[i]),\
                                    &pBsdInfo->stSTA.pvir_sub_bsd_zip[i*5 + j]\
            );
        }
    }

    sprintf(op_string, "rm -rf /var/%s", BSD_FILE_NAME);
    SAFE_System(op_string, NORMAL_WAIT_TIME);
    
    return SV_SUCCESS;
}

sint32 BSD_Font_Init(BSD_INFO_S* pBsdInfo)
{
    sint32 s32Ret = 0;
    MPP_FONT_CONF_S stFontConf = {0};
    sprintf(pBsdInfo->stSTA.pstring, "TH:%2d",pBsdInfo->stALG.bsd_th);
    strcpy(stFontConf.szFontFile, "/root/arial.ttf");
    s32Ret = mpp_font_Init(&stFontConf);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "mpp_font_Init failed [%#x]\n", s32Ret);
        return SV_FAILURE;
    }
    return SV_SUCCESS;
}

sint32 BSD_OSD_Init(BSD_INFO_S* pBsdInfo)
{
    int i;
    sint32 s32Ret = HI_SUCCESS;
    RGN_HANDLE      u32Handle;
    RGN_ATTR_S      stRgnAttr;
    MPP_CHN_S       stChn;
    RGN_CHN_ATTR_S  stChnAttr;

    for(i=0; i<BSD_CHIND_NUM; i++)
    {
        u32Handle = pBsdInfo->stRGN.pu32Handle[i];
        stRgnAttr.enType = OVERLAY_RGN;
        stRgnAttr.unAttr.stOverlay.enPixelFmt = PIXEL_FORMAT_ARGB_1555;
        stRgnAttr.unAttr.stOverlay.stSize.u32Width = pBsdInfo->stRGN.pstRect[i].u32Width;
        stRgnAttr.unAttr.stOverlay.stSize.u32Height = pBsdInfo->stRGN.pstRect[i].u32Height;
        stRgnAttr.unAttr.stOverlay.u32BgColor = 0xf0;
        stRgnAttr.unAttr.stOverlay.u32CanvasNum = 2;

        s32Ret = HI_MPI_RGN_Create(u32Handle, &stRgnAttr);
        if (s32Ret != HI_SUCCESS)
        {
            print_level(SV_ERROR, "HI_MPI_RGN_Create failed. [err=%#x]\n", s32Ret);
            return s32Ret;
        }
        else
        {
            print_level(SV_INFO, "HI_MPI_RGN_Create Handle:%d SUCCESS\n", u32Handle);
        }
        pBsdInfo->stRGN.bCreated     = SV_TRUE;
    
        stChn.enModId = HI_ID_VENC;
        stChn.s32DevId = 0;
        stChn.s32ChnId = pBsdInfo->stRGN.ps32ChnId[i];

        stChnAttr.bShow  = 1;
        stChnAttr.enType = OVERLAY_RGN;
        stChnAttr.unChnAttr.stOverlayChn.stPoint.s32X = pBsdInfo->stRGN.pstRect[i].s32X;
        stChnAttr.unChnAttr.stOverlayChn.stPoint.s32Y = pBsdInfo->stRGN.pstRect[i].s32Y;
        stChnAttr.unChnAttr.stOverlayChn.u32BgAlpha = 0;
        stChnAttr.unChnAttr.stOverlayChn.u32FgAlpha = 64;
        stChnAttr.unChnAttr.stOverlayChn.u32Layer   = 2;
        stChnAttr.unChnAttr.stOverlayChn.stQpInfo.bQpDisable    = HI_FALSE;
        stChnAttr.unChnAttr.stOverlayChn.stQpInfo.bAbsQp        = HI_FALSE;
        stChnAttr.unChnAttr.stOverlayChn.stQpInfo.s32Qp         = 0;
        stChnAttr.unChnAttr.stOverlayChn.stInvertColor.stInvColArea.u32Height = 16;
        stChnAttr.unChnAttr.stOverlayChn.stInvertColor.stInvColArea.u32Width  = 16;
        stChnAttr.unChnAttr.stOverlayChn.stInvertColor.u32LumThresh = 128;
        stChnAttr.unChnAttr.stOverlayChn.stInvertColor.enChgMod     = LESSTHAN_LUM_THRESH;
        stChnAttr.unChnAttr.stOverlayChn.stInvertColor.bInvColEn    = HI_FALSE;
        stChnAttr.unChnAttr.stOverlayChn.enAttachDest = ATTACH_JPEG_MAIN;
        stChnAttr.unChnAttr.stOverlayChn.u16ColorLUT[0] = 0x2abc;
        stChnAttr.unChnAttr.stOverlayChn.u16ColorLUT[1] = 0x7FF0;

        s32Ret = HI_MPI_RGN_AttachToChn(u32Handle, &stChn, &stChnAttr);
        if (s32Ret != HI_SUCCESS)
        {
            print_level(SV_ERROR, "HI_MPI_AttachToChn failed! [err=%#x]\n", s32Ret);
            return s32Ret;
        }
        else
        {
            print_level(SV_INFO, "HI_MPI_RGN_AttachToChn ChnId:%d SUCCESS\n", stChn.s32ChnId);
        }
        pBsdInfo->stRGN.bAttached = SV_TRUE;
    }

#if BSD_MESSAGE_ENABLE
    for(i=0; i<BSD_CHIND_NUM; i++)
    {
        int index = i + 3;
        u32Handle = pBsdInfo->stRGN.pu32Handle[index];
        stRgnAttr.enType = OVERLAY_RGN;
        stRgnAttr.unAttr.stOverlay.enPixelFmt = PIXEL_FORMAT_ARGB_1555;
        stRgnAttr.unAttr.stOverlay.stSize.u32Width = pBsdInfo->stRGN.pstRect[index].u32Width;
        stRgnAttr.unAttr.stOverlay.stSize.u32Height = pBsdInfo->stRGN.pstRect[index].u32Height;
        stRgnAttr.unAttr.stOverlay.u32BgColor = 0x0;
        stRgnAttr.unAttr.stOverlay.u32CanvasNum = 2;
        print_level(SV_INFO, "width:%d, height:%d\n",stRgnAttr.unAttr.stOverlay.stSize.u32Width,stRgnAttr.unAttr.stOverlay.stSize.u32Height);
        s32Ret = HI_MPI_RGN_Create(u32Handle, &stRgnAttr);
        if (s32Ret != HI_SUCCESS)
        {
            print_level(SV_ERROR, "HI_MPI_RGN_Create failed. [err=%#x]\n", s32Ret);
            return s32Ret;
        }
        else
        {
            print_level(SV_INFO, "HI_MPI_RGN_Create Handle:%d SUCCESS\n", u32Handle);
        }
        pBsdInfo->stRGN.bCreated     = SV_TRUE;
    
        stChn.enModId = HI_ID_VENC;
        stChn.s32DevId = 0;
        stChn.s32ChnId = pBsdInfo->stRGN.ps32ChnId[index];

        stChnAttr.bShow  = 1;
        stChnAttr.enType = OVERLAY_RGN;
        stChnAttr.unChnAttr.stOverlayChn.stPoint.s32X = pBsdInfo->stRGN.pstRect[index].s32X;
        stChnAttr.unChnAttr.stOverlayChn.stPoint.s32Y = pBsdInfo->stRGN.pstRect[index].s32Y;
        stChnAttr.unChnAttr.stOverlayChn.u32BgAlpha = 64;
        stChnAttr.unChnAttr.stOverlayChn.u32FgAlpha = 64;
        stChnAttr.unChnAttr.stOverlayChn.u32Layer   = 2;
        stChnAttr.unChnAttr.stOverlayChn.stQpInfo.bQpDisable    = HI_FALSE;
        stChnAttr.unChnAttr.stOverlayChn.stQpInfo.bAbsQp        = HI_FALSE;
        stChnAttr.unChnAttr.stOverlayChn.stQpInfo.s32Qp         = 0;
        stChnAttr.unChnAttr.stOverlayChn.stInvertColor.stInvColArea.u32Height = 16;
        stChnAttr.unChnAttr.stOverlayChn.stInvertColor.stInvColArea.u32Width  = 16;
        stChnAttr.unChnAttr.stOverlayChn.stInvertColor.u32LumThresh = 128;
        stChnAttr.unChnAttr.stOverlayChn.stInvertColor.enChgMod     = LESSTHAN_LUM_THRESH;
        stChnAttr.unChnAttr.stOverlayChn.stInvertColor.bInvColEn    = HI_FALSE;
        stChnAttr.unChnAttr.stOverlayChn.enAttachDest = ATTACH_JPEG_MAIN;
        stChnAttr.unChnAttr.stOverlayChn.u16ColorLUT[0] = 0x2abc;
        stChnAttr.unChnAttr.stOverlayChn.u16ColorLUT[1] = 0x7FF0;

        s32Ret = HI_MPI_RGN_AttachToChn(u32Handle, &stChn, &stChnAttr);
        if (s32Ret != HI_SUCCESS)
        {
            print_level(SV_ERROR, "HI_MPI_AttachToChn failed! [err=%#x]\n", s32Ret);
            return s32Ret;
        }
        else
        {
            print_level(SV_INFO, "HI_MPI_RGN_AttachToChn ChnId:%d SUCCESS\n", stChn.s32ChnId);
        }
    }
#endif

    return SV_SUCCESS;
}


sint32 BSD_Handle_Body(void* args)
{
    sint32 s32Ret;
    BSD_INFO_S* pBsdInfo = (BSD_INFO_S*) args;

    s32Ret = prctl(PR_SET_NAME, "bsd_body");
    if (s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "prctl PR_SET_NAME failed! [err:%s]\n", strerror(errno));
    }

    while(pBsdInfo->stCTL.bRunning)
    {
        s32Ret = BSD_ALG_Update(pBsdInfo);
        if(s32Ret != SV_SUCCESS)
        {
            print_level(SV_ERROR, "BSD_ALG_Update fail\n");
            continue;
        }
        
        if(pBsdInfo->stSTA.bUpdate != SV_TRUE)
        {
            continue;
        }

        s32Ret = BSD_Image_Update(pBsdInfo);
        if(s32Ret != SV_SUCCESS)
        {
            print_level(SV_ERROR, "BSD_ALG_Update fail\n");
            continue;
        }

        s32Ret = BSD_Rect_Update(pBsdInfo);
        if(s32Ret != SV_SUCCESS)
        {
            print_level(SV_ERROR, "BSD_ALG_Update fail\n");
            continue;
        }

        s32Ret = BSD_OSD_Update(pBsdInfo);
        if(s32Ret != SV_SUCCESS)
        {
            print_level(SV_ERROR, "BSD_OSD_Update fail\n");
            continue;
        }
    }

    return SV_SUCCESS;
}

sint32 BSD_ALG_Update(BSD_INFO_S* pBsdInfo)
{
    sint32 s32Ret = 0;
    uint32 u32Cnt = 0;
    sint32 s32Fd = -1;
    VPSS_GRP VpssGrp = 0;
    VPSS_CHN VpssChn = 2;
    sint32 s32GetFrameMilliSec = 4000;
    VGS_TASK_ATTR_S stTask = {0};
    VIDEO_FRAME_INFO_S stVideoFrame;
    VIDEO_FRAME_INFO_S *stVideoFrame_in = &stTask.stImgIn;
    VIDEO_FRAME_INFO_S *stVideoFrame_out = &stTask.stImgOut;

    static int   nBSDRe = 0;
    static bool  bBSDRe;
    static int get_gpio = 1;

    //print_level(SV_INFO, "VpssGrp=%d; VpssChn=%d\n", VpssGrp, VpssChn);
    

    s32Ret = HI_MPI_VPSS_GetChnFrame(VpssGrp, VpssChn, &stVideoFrame, s32GetFrameMilliSec);
    if(s32Ret != HI_SUCCESS)
    {
        print_level(SV_ERROR, "HI_MPI_VPSS_GetChnFrame failed. [err=%#x]\n", s32Ret);
        return SV_FAILURE;
    }

    s32Ret = ProcessBSD(stVideoFrame, get_gpio, bBSDRe, nBSDRe);
    if(s32Ret != true)
    {
        print_level(SV_ERROR, "ProcessBSD failed. [err=%#x]\n", s32Ret);
    }

    s32Ret = HI_MPI_VPSS_ReleaseChnFrame(VpssGrp, VpssChn, &stVideoFrame);
    if (HI_SUCCESS != s32Ret)
    {
        print_level(SV_ERROR, "HI_MPI_VPSS_ReleaseChnFrame failed. [err=%#x]\n", s32Ret);
    }

    pBsdInfo->stALG.preBSDRe = pBsdInfo->stALG.nBSDRe;
    pBsdInfo->stALG.nBSDRe = nBSDRe;

    if(pBsdInfo->stALG.preBSDRe != pBsdInfo->stALG.nBSDRe)
    {
        pBsdInfo->stSTA.bUpdate = SV_TRUE;
        for(int i=0; i<5; i++)
        {
            pBsdInfo->stSTA.state[i] = (nBSDRe >> (i)) & 0x1;
#if 1
        }
#else
            printf("| %d ",state[i]);
        }
        printf("|\n");
#endif
    }


    return SV_SUCCESS;
    
}

sint32 BSD_OSD_Update(BSD_INFO_S* pBsdInfo)
{
    sint32 s32Ret = SV_SUCCESS;
    RGN_HANDLE u32Handle;
    BITMAP_S   stBitmap;
    for(int index=0;index<BSD_CHIND_NUM;index++)
    {
        u32Handle               = pBsdInfo->stRGN.pu32Handle[index];
        stBitmap.enPixelFormat  = PIXEL_FORMAT_ARGB_1555;
        stBitmap.u32Width       = pBsdInfo->stRGN.pstRect[index].u32Width;
        stBitmap.u32Height      = pBsdInfo->stRGN.pstRect[index].u32Height;
        stBitmap.pData          = pBsdInfo->stSTA.pvir_bsd[index];

        s32Ret = HI_MPI_RGN_SetBitMap(u32Handle,&stBitmap);
        if(SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "HI_MPI_RGN_SetBitMap failed! [err=%#x]\n", s32Ret);
            return s32Ret;
        }
    }

    return SV_SUCCESS;
}

sint32 BSD_Image_Update(BSD_INFO_S* pBsdInfo)
{
    int i,j;
    int byte = 2;
    sint32 s32Ret = SV_SUCCESS;
    sint32 width_dst, height_dst, width_src, height_src, yaxis;
    char *img_dst, *img_src, *img_src_blank;
    SV_BOOL *state = pBsdInfo->stSTA.state;

    for(i=0; i<BSD_CHIND_NUM; i++)
    {
        width_dst = pBsdInfo->stRGN.pstRect[i].u32Width;
        height_dst = pBsdInfo->stRGN.pstRect[i].u32Height;
        width_src = pBsdInfo->stSTA.pwidth_sub[i];
        height_src = pBsdInfo->stSTA.pheight_sub[i];
        yaxis = pBsdInfo->stSTA.pyaxis[i];

        img_dst = pBsdInfo->stSTA.pvir_bsd[i];
        img_src = pBsdInfo->stSTA.pvir_sub_bsd[i];
        img_src_blank = (char*)malloc(sizeof(char)*byte*width_src*height_src);
        memset(img_src_blank, 0x0, sizeof(char)*byte*width_src*height_src);

        for(j=0; j<5; j++)
        {
            if(state[j] != SV_TRUE){
                img_memcopy(
                    img_dst, width_dst, height_dst,
                    img_src_blank, width_src, height_src,
                    width_src*j,yaxis,byte
                );
                continue;
            }

            fastlz_decompress(
                pBsdInfo->stSTA.pvir_sub_bsd_zip[5*i + j],
                pBsdInfo->stSTA.psub_zip_len[5*i + j],
                img_src,
                (sizeof(s16)*(int)pBsdInfo->stSTA.pwidth_sub[i]*(int)pBsdInfo->stSTA.pheight_sub[i])\      
            );

            img_memcopy(
                img_dst, width_dst, height_dst,
                img_src, width_src, height_src,
                width_src*j,yaxis,byte
            );

        }

        free(img_src_blank);
    }
    return SV_SUCCESS;
}

sint32 BSD_Font_Update(BSD_INFO_S* pBsdInfo)
{
    static SV_BOOL bUpdate = SV_TRUE;

    int i, index;
    sint32 s32Ret = SV_SUCCESS;
    RGN_HANDLE u32Handle;
    BITMAP_S   stBitmap;

    SV_SIZE_S stBmpSize = {0};
    uint32 u32Stride;
    uint8 font_scale[] = {MPP_FONT_SCALE_2X, MPP_FONT_SCALE_1X, MPP_FONT_SCALE_4X};
    uint32 u32Width, u32Height;
    char *pszStr = pBsdInfo->stSTA.pstring;

    
    if(bUpdate != SV_TRUE)
    {
        return SV_SUCCESS;
    }
    

    bUpdate = SV_FALSE;
    for(int i=0;i<BSD_CHIND_NUM;i++)
    {
        index = i + 3;
        u32Handle = pBsdInfo->stRGN.pu32Handle[index];
        stBitmap.enPixelFormat  = PIXEL_FORMAT_ARGB_1555;
        u32Width = pBsdInfo->stRGN.pstRect[index].u32Width;
        u32Height = pBsdInfo->stRGN.pstRect[index].u32Height;
        stBitmap.pData          = malloc(sizeof(HI_U16)*u32Width*u32Height);

        s32Ret = mpp_font_StringToARGB1555(pszStr,font_scale[i],(uint16*)stBitmap.pData,&stBmpSize,&u32Stride);
        if(SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "mpp_font_StringToARGB1555 failed! [err=%#x]\n", s32Ret);
        }
        print_level(SV_INFO, "width:%d, height:%d, stride:%d\n",stBmpSize.u32Width,stBmpSize.u32Height,u32Stride);

        stBitmap.u32Width       = stBmpSize.u32Width;
        stBitmap.u32Height      = stBmpSize.u32Height;

        s32Ret = HI_MPI_RGN_SetBitMap(u32Handle,&stBitmap);
        if(SV_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "HI_MPI_RGN_SetBitMap failed! [err=%#x]\n", s32Ret);
            //return s32Ret;
        }

        free(stBitmap.pData);
    }
    return s32Ret;
}

sint32 BSD_Rect_Update(BSD_INFO_S* pBsdInfo)
{
    int i, index;
    sint32 s32Ret = SV_SUCCESS;
    RGN_HANDLE u32Handle;
    BITMAP_S   stBitmap;

    pixel_point_scale *point_scale_paint = pBsdInfo->stSTA.point_paint;
    uint16 color = 0x8fe0; //绿色
    int pstick[] = {3,2,5};

    for(i=0; i<BSD_CHIND_NUM; i++)
    {
        index = i;
        u32Handle = pBsdInfo->stRGN.pu32Handle[index];
        stBitmap.enPixelFormat  = PIXEL_FORMAT_ARGB_1555;
        stBitmap.u32Width  = pBsdInfo->stRGN.pstRect[index].u32Width;
        stBitmap.u32Height = pBsdInfo->stRGN.pstRect[index].u32Height;
        stBitmap.pData     = pBsdInfo->stSTA.pvir_bsd[i];

        paint_MultiLine_scale_uint16(
            (HI_U16*)stBitmap.pData,
            stBitmap.u32Width,
            stBitmap.u32Height,
            point_scale_paint, 
            5, color, pstick[i]
        );
    }

    return SV_SUCCESS;
}

sint32 BSD_Fini(BSD_INFO_S* pBsdInfo)
{
    sint32 s32Ret;

    s32Ret = BSD_ALG_Fini(pBsdInfo);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "BSD_ALG_Fini fail\n");
        return SV_FAILURE;
    }

    s32Ret = BSD_Image_Fini(pBsdInfo);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "BSD_Image_Fini fail\n");
        return SV_FAILURE;
    }

    s32Ret = BSD_Font_Fini(pBsdInfo);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "BSD_Font_Fini fail\n");
        return SV_FAILURE;
    }

    s32Ret = BSD_OSD_Fini(pBsdInfo);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "BSD_OSD_Fini fail\n");
        return SV_FAILURE;
    }

    return SV_SUCCESS;
}

sint32 BSD_PARAM_Fini(BSD_INFO_S* pBsdInfo)
{
    for (int i=0; i<BSD_CHIND_NUM; i++)
    {
        free(pBsdInfo->stSTA.pvir_bsd[i]);
        free(pBsdInfo->stSTA.pvir_sub_bsd[i]);
    }

    return SV_SUCCESS;
}

sint32 BSD_OSD_Fini(BSD_INFO_S* pBsdInfo)
{
    sint32 s32Ret;
    RGN_HANDLE  u32Handle;
    MPP_CHN_S   stChn;
    int index;
    for(index=0;index<BSD_CHIND_NUM;index++)
    {
        u32Handle = pBsdInfo->stRGN.pu32Handle[index];
        stChn.enModId = HI_ID_VENC;
        stChn.s32DevId = 0;
        stChn.s32ChnId = pBsdInfo->stRGN.ps32ChnId[index];

        s32Ret = HI_MPI_RGN_DetachFromChn(u32Handle, &stChn);
        if(HI_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "HI_MPI_RGN_DetachFromChn failed! [err=%#x]\n",s32Ret);
        }
        pBsdInfo->stRGN.bAttached = SV_FALSE;

        s32Ret = HI_MPI_RGN_Destroy(u32Handle);
        if(HI_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "HI_MPI_RGN_Destroy failed [err=%#x]\n", s32Ret);
        }
        pBsdInfo->stRGN.bCreated = SV_FALSE;
    }
#if BSD_MESSAGE_ENABLE
    for(index=3;index<6;index++)
    {
        u32Handle = pBsdInfo->stRGN.pu32Handle[index];
        stChn.enModId = HI_ID_VENC;
        stChn.s32DevId = 0;
        stChn.s32ChnId = pBsdInfo->stRGN.ps32ChnId[index];

        s32Ret = HI_MPI_RGN_DetachFromChn(u32Handle, &stChn);
        if(HI_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "HI_MPI_RGN_DetachFromChn failed! [err=%#x]\n",s32Ret);
        }
        pBsdInfo->stRGN.bAttached = SV_FALSE;

        s32Ret = HI_MPI_RGN_Destroy(u32Handle);
        if(HI_SUCCESS != s32Ret)
        {
            print_level(SV_ERROR, "HI_MPI_RGN_Destroy failed [err=%#x]\n", s32Ret);
        }
        pBsdInfo->stRGN.bCreated = SV_FALSE;
    }
#endif
    return SV_SUCCESS;
}

sint32 BSD_Image_Fini(BSD_INFO_S* pBsdInfo)
{
    for (int i=0; i<BSD_CHIND_NUM; i++)
    {
        free(pBsdInfo->stSTA.pvir_sub_bsd_zip[i]);
    }
    return SV_SUCCESS;
}

sint32 BSD_Font_Fini(BSD_INFO_S* pBsdInfo)
{
    sint32 s32Ret = mpp_font_Fini();
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "mpp_font_Fini fail\n");
        return SV_FAILURE;
    }
    return SV_SUCCESS;
}

sint32 BSD_ALG_Fini(BSD_INFO_S* pBsdInfo)
{
    return SV_SUCCESS;
}



sint32 BSD_Start(BSD_INFO_S* pBsdInfo)
{
    sint32 s32Ret = 0;
    pthread_t thread = 0;

    pBsdInfo->stCTL.bRunning = SV_TRUE;
    s32Ret = pthread_create(&thread, NULL, BSD_Handle_Body, pBsdInfo);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "pthread_create failed. [err: %s]\n", strerror(errno));
        return SV_FAILURE;
    }

    pBsdInfo->stCTL.u32Tid = thread;
    return SV_SUCCESS;
}

sint32 BSD_Stop(BSD_INFO_S* pBsdInfo)
{
    sint32 s32Ret = 0;
    pthread_t thread = pBsdInfo->stCTL.u32Tid;
    void *pvRetval = NULL;

    pBsdInfo->stCTL.bRunning = SV_FALSE;
    s32Ret = pthread_join(thread, &pvRetval);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "pthread_join failed. [err: %s]\n", strerror(errno));
        return SV_FAILURE;
    }

    return SV_SUCCESS;
}

/* 中断退出 */
static void exit_handle(int signalnum)
{
    int s32Ret;
    print_level(SV_INFO, "BSD exit\n");
    BsdInfo.stCTL.bRunning = SV_FALSE;
    s32Ret = BSD_Stop(&BsdInfo);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "BSD_Stop fail\n");
        return ;
    }
    
    s32Ret = BSD_Fini(&BsdInfo);
    {
        print_level(SV_ERROR, "BSD_Fini fail\n");
        return ;
    }
    return ;
}

int main(int argc, char **argv)
{
    sint32 s32Ret = 0;
    memset(&BsdInfo, 0, sizeof(BSD_INFO_S));

    if (argc > 1)
    {
        BsdInfo.stALG.bsd_th = atoi(argv[1]);
    }
    else
    {
        BsdInfo.stALG.bsd_th = BSD_TH;
    }

    /*捕获进程退出的系统消息*/
    if (SIG_ERR == signal(SIGTERM, exit_handle))
    {
        printf("catch signal SIGTERM Error: %d, %s\n", errno, strerror(errno));
    }

    if (SIG_ERR == signal(SIGINT, exit_handle))
    {
        printf("catch signal SIGTERM Error: %d, %s\n", errno, strerror(errno));
    }

    /*忽略PIPE消息*/
    if (SIG_ERR == signal(SIGPIPE, SIG_IGN))
    {
        printf("catch signal SIGPIPE Error: %d, %s\n", errno, strerror(errno));
    }


    s32Ret = BSD_Init(&BsdInfo);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "BSD_Init fail!\n");
        return SV_FAILURE;
    }

    s32Ret = BSD_Start(&BsdInfo);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "BSD_Start fail\n");
        return SV_FAILURE;
    }

    while(BsdInfo.stCTL.bRunning == SV_TRUE)
    {
        sleep(1);
    }

    s32Ret = BSD_Stop(&BsdInfo);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "BSD_Stop failed\n");
    }

    s32Ret = BSD_Fini(&BsdInfo);
    if(s32Ret != SV_SUCCESS)
    {
        print_level(SV_ERROR, "BSD_Fini failed\n");
    }

    return 0;
}



















