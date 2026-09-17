#ifndef _ALG_LIB_H_
#define _ALG_LIB_H_

#include "stdlib.h"
#include "stdio.h"


typedef struct ALGPDS_RECT_
{
    float  x1;	// left top, between[0,1]
    float  y1; 	// left top
    float  x2; 	// right bottom
    float  y2; 	// right bottom
    float  confidence;
    float  distance;  // not used now
    float  reserve1;  // not used now
    unsigned short classes; // class ID  
    // int id;
}ALGPDS_RECT_S;

#define MAXTARGET 50

typedef struct ALG_PDS_INFO_S
{
    int num;
    ALGPDS_RECT_S Rect_t[MAXTARGET];
}ALGPDS_INFO_S;


int ALGPDS_init(float *thresholds, int numberOfThresholds, char* ptModelFile);

int ALGPDS_forward(unsigned char* p_inputdata);

int ALGPDS_get_result(ALGPDS_INFO_S *ptsResult);

int ALGPDS_release();

#endif //ADAS32_ALGLIB_H
