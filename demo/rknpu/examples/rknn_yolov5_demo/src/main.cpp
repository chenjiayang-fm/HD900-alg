#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <dlfcn.h>

#define _BASETSD_H

#include "opencv2/core/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include "alglib.h"

/*-------------------------------------------
                  Main Functions
-------------------------------------------*/
int main(int argc, char **argv)
{
    char *model_name = NULL;
    int img_width = 0;
    int img_height = 0;
    int img_channel = 0;
    const float vis_threshold = 0.1;
    const float nms_threshold = 0.5;
    const float conf_threshold = 0.3;
    int ret;

    ALGPDS_INFO_S AlgResutl;

    if (argc != 3){
        printf("Usage: %s <rknn model> <jpg> \n", argv[0]);
        return -1;
    }

    model_name = (char *)argv[1];
    char *image_name = argv[2];

    // [1] get image data
    printf("Read %s ...\n", image_name);
    cv::Mat orig_img = cv::imread(image_name, 1);
    if (!orig_img.data){
        printf("cv::imread %s fail!\n", image_name);
        return -1;
    }
    img_width = orig_img.cols;
    img_height = orig_img.rows;
    printf("img width = %d, img height = %d\n", img_width, img_height);

    cv::Mat resize_img;
    cv::resize(orig_img, resize_img, cv::Size(640, 640));

    float thres[10] = {0};
    thres[0] = conf_threshold;
    thres[1] = nms_threshold;
    thres[2] = vis_threshold;
    
    // [2] initialize the algorithm library
    ret = ALGPDS_init(thres, 3, model_name);

    // [3] run Neural Networks
    ret = ALGPDS_forward(resize_img.data);

    // [4] get process result
    ret = ALGPDS_get_result(&AlgResutl);

    // [5] draw detected boxes on the image
    for (int i = 0; i < AlgResutl.num; i++)
    {
        ALGPDS_RECT_S algBox = AlgResutl.Rect_t[i];
        float x1 =  algBox.x1 * img_width;
        float x2 =  algBox.x2 * img_width;
        float y1 =  algBox.y1 * img_height;
        float y2 =  algBox.y2 * img_height;

        cv::Point leftTop(x1, y1);
        cv::Point rightBottom(x2, y2);

        cv::rectangle(orig_img, leftTop, rightBottom, cv::Scalar(0,0,255),2,8,0);
        printf("[%d], %.2f, %.2f, %.2f, %.2f\n",i, x1,y1,x2,y2);
    }
    
    // save & output
    cv::imwrite("result.png", orig_img);

    ALGPDS_release();

    return 0; 
}