#ifndef trace_common
#define trace_common

typedef struct Pixel_Point
{
    int x_aixs;
    int y_axis;
} pixel_point;


void select_sleep(int sec,int usec);
long long int microtime();

#endif