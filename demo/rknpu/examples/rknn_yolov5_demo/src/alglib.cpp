#include "alglib.h"
#include <vector>
#include <string.h>
#include "postprocess.h"
#include "rknn_api.h"

static rknn_context gRKNNctx;

// Netwokr input size (get it from loaded model)
static int gNetInputChannel = 3;  
static int gNetInputWidth = 0;
static int gNetInputHeight = 0;


static rknn_tensor_attr gInputAttrs[1];
static rknn_tensor_attr gOutputAttrs[3];
static rknn_input_output_num gIOnum;


static std::vector<float> gOutScales(0);
static std::vector<uint8_t> gOutZps(0);

static rknn_input gInputs[1];
static rknn_output gOutputs[3];

static float gConfidenceThreshold = 0;
static float gNMSThreshold = 0;
static float gVisThreshold = 0;

// =================================================================


static void printRKNNTensor(rknn_tensor_attr *attr)
{
    printf("index=%d name=%s n_dims=%d dims=[%d %d %d %d] n_elems=%d size=%d "
           "fmt=%d type=%d qnt_type=%d fl=%d zp=%d scale=%f\n",
           attr->index, attr->name, attr->n_dims, attr->dims[3], attr->dims[2],
           attr->dims[1], attr->dims[0], attr->n_elems, attr->size, 0, attr->type,
           attr->qnt_type, attr->fl, attr->zp, attr->scale);
}


static unsigned char *load_data(FILE *fp, size_t ofst, size_t sz)
{
    unsigned char *data;
    int ret;

    data = NULL;

    if (NULL == fp)
    {
        return NULL;
    }

    ret = fseek(fp, ofst, SEEK_SET);
    if (ret != 0)
    {
        printf("blob seek failure.\n");
        return NULL;
    }

    data = (unsigned char *)malloc(sz);
    if (data == NULL)
    {
        printf("buffer malloc failure.\n");
        return NULL;
    }
    ret = fread(data, 1, sz, fp);
    return data;
}

static unsigned char *load_model(const char *filename, int *model_size)
{

    FILE *fp;
    unsigned char *data;

    fp = fopen(filename, "rb");
    if (NULL == fp)
    {
        printf("Open file %s failed.\n", filename);
        return NULL;
    }

    fseek(fp, 0, SEEK_END);
    int size = ftell(fp);

    data = load_data(fp, 0, size);

    fclose(fp);

    *model_size = size;
    return data;
}

int ALGPDS_init(float *thresholds, int numberOfThresholds, char* ptModelFile=NULL)
{
    int ret = 0;

    gConfidenceThreshold = thresholds[0];
    gNMSThreshold = thresholds[1];
    gVisThreshold = thresholds[2];


    printf("Loading mode...\n");
    int model_data_size = 0;
    unsigned char *model_data = load_model(ptModelFile, &model_data_size);
    ret = rknn_init(&gRKNNctx, model_data, model_data_size, 0);
    if (ret < 0){
        printf("rknn_init error ret=%d\n", ret);
        return -1;
    }


    rknn_sdk_version version;
    ret = rknn_query(gRKNNctx, RKNN_QUERY_SDK_VERSION, &version, sizeof(rknn_sdk_version));
    if (ret < 0){
        printf("rknn_init error ret=%d\n", ret);
        return -1;
    }
    printf("sdk version: %s driver version: %s\n", version.api_version, version.drv_version);


    ret = rknn_query(gRKNNctx, RKNN_QUERY_IN_OUT_NUM, &gIOnum, sizeof(gIOnum));
    if (ret < 0){
        printf("rknn_init error ret=%d\n", ret);
        return -1;
    }
    printf("model input num: %d, output num: %d\n", gIOnum.n_input, gIOnum.n_output);


    memset(gInputAttrs, 0, sizeof(gInputAttrs));
    for (int i = 0; i < gIOnum.n_input; i++){
        gInputAttrs[i].index = i;
        ret = rknn_query(gRKNNctx, RKNN_QUERY_INPUT_ATTR, &(gInputAttrs[i]), sizeof(rknn_tensor_attr));
        if (ret < 0){
            printf("rknn_init error ret=%d\n", ret);
            return -1;
        }
        printRKNNTensor(&(gInputAttrs[i]));
    }


    memset(gOutputAttrs, 0, sizeof(gOutputAttrs));
    for (int i = 0; i < gIOnum.n_output; i++){
        gOutputAttrs[i].index = i;
        ret = rknn_query(gRKNNctx, RKNN_QUERY_OUTPUT_ATTR, &(gOutputAttrs[i]), sizeof(rknn_tensor_attr));
        printRKNNTensor(&(gOutputAttrs[i]));
    }


    if (gInputAttrs[0].fmt == RKNN_TENSOR_NCHW){
        printf("model is NCHW input fmt\n");
        gNetInputWidth = gInputAttrs[0].dims[0];
        gNetInputHeight = gInputAttrs[0].dims[1];
        gNetInputChannel = gInputAttrs[0].dims[3];
    }else{
        printf("model is NHWC input fmt\n");
        gNetInputWidth = gInputAttrs[0].dims[1];
        gNetInputHeight = gInputAttrs[0].dims[2];
        gNetInputChannel = gInputAttrs[0].dims[3];
    }

    printf("model input height=%d, width=%d, channel=%d\n", gNetInputHeight, gNetInputWidth, gNetInputChannel);


    memset(gInputs, 0, sizeof(gInputs));
    gInputs[0].index = 0;
    gInputs[0].type = RKNN_TENSOR_UINT8;
    gInputs[0].size = gNetInputWidth * gNetInputHeight * gNetInputChannel;
    gInputs[0].fmt = RKNN_TENSOR_NHWC;
    gInputs[0].pass_through = 0;

    memset(gOutputs, 0, sizeof(gOutputs));
    for (int i = 0; i < gIOnum.n_output; i++){
        gOutputs[i].want_float = 0;
    }

    for (int i = 0; i < gIOnum.n_output; ++i){
        gOutScales.push_back(gOutputAttrs[i].scale);
        gOutZps.push_back(gOutputAttrs[i].zp);
    }

    return ret;
}



int ALGPDS_forward(unsigned char* p_inputdata){
    gInputs[0].buf = p_inputdata;
    int ret = rknn_inputs_set(gRKNNctx, gIOnum.n_input, gInputs);
    if(ret < 0) {
        printf("rknn_input_set fail! ret=%d\n", ret);
        return -1;
    }

    ret = rknn_run(gRKNNctx, nullptr);
    if(ret < 0) {
        printf("rknn_run fail! ret=%d\n", ret);
        return -1;
    }
    
    ret = rknn_outputs_get(gRKNNctx, gIOnum.n_output, gOutputs, NULL);
    if(ret < 0) {
        printf("rknn_outputs_get fail! ret=%d\n", ret);
        return -1;
    }

    printf("forward success... \n");
    return ret;
}


int ALGPDS_get_result(ALGPDS_INFO_S *ptsResult){
    int ret = 0;

    memset(ptsResult, 0, sizeof(ALGPDS_INFO_S));
    detect_result_group_t detect_result_group;
    
    ret = post_process((uint8_t *)gOutputs[0].buf, (uint8_t *)gOutputs[1].buf, (uint8_t *)gOutputs[2].buf, gNetInputHeight, gNetInputWidth,
                 gConfidenceThreshold, gNMSThreshold, gVisThreshold, 1.0, 1.0, gOutZps, gOutScales, &detect_result_group);

    int retNums = std::min(detect_result_group.count, MAXTARGET);
    ptsResult->num = retNums;
    for(int i = 0; i < retNums; i++){
        ptsResult->Rect_t[i].classes = detect_result_group.results[i].id;
        ptsResult->Rect_t[i].confidence = detect_result_group.results[i].prop;
        ptsResult->Rect_t[i].x1 = (float)detect_result_group.results[i].box.left / gNetInputWidth;
        ptsResult->Rect_t[i].x2 = (float)detect_result_group.results[i].box.right / gNetInputWidth;
        ptsResult->Rect_t[i].y1 = (float)detect_result_group.results[i].box.top / gNetInputHeight;
        ptsResult->Rect_t[i].y2 = (float)detect_result_group.results[i].box.bottom / gNetInputHeight;
    }

    ret = rknn_outputs_release(gRKNNctx, gIOnum.n_output, gOutputs);

    return ret;
}

int ALGPDS_release(){

    rknn_outputs_release(gRKNNctx, gIOnum.n_output, gOutputs);

    // Release
    if(gRKNNctx >= 0) {
        rknn_destroy(gRKNNctx);
    }
    
    return 0;
}