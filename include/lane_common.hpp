#ifndef lane_common_h
#define lane_common_h

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>
#include <error.h>
#include <errno.h>
#include <pthread.h>

#include <string.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <sys/time.h>
#include <unistd.h>
#include <sys/mman.h>
#include <dirent.h>
#include <stdexcept>


#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

#include "rknn_api.h"

//#include <stdint-gcc.h>

#define SV_WIDTH 416 
#define SV_HEIGHT 224 
#define SV_GRIDING 208 
#define SV_LANEPOINTS 18 
#define SV_POINTNUM 7 
#define SV_NUMLANES 2 

// state 0:safe 1:alarm
struct STLaneInfo {
    size_t u64x;
    size_t u64y;
    size_t u64state;
};

#endif