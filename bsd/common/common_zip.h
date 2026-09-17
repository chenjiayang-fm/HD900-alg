#ifndef __COMMON_ZIP__
#define __COMMON_ZIP__
#include "common_zip.h"
#include <zlib/zlib.h>
#include <stdlib.h>

typedef char ZLIB_Data;
void Data_Compress(ZLIB_Data *strDst, long *dstLen, ZLIB_Data *strSrc, long srcLen);
void Data_unCompress(ZLIB_Data *strDst, long *dstLen, ZLIB_Data *strSrc, long srcLen);
void Data_Compress_Malloc(ZLIB_Data **sstrDst, long *dstLen, ZLIB_Data *strSrc, long srcLen);

int fastlz_compress_malloc(const void* input, int length, void **output);
int fastlz_decompress(const void* input, int length, void* output, int maxout);
int fastlz_compress(const void* input, int length, void* output);
#endif