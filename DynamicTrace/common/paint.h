#ifndef paint_h
#define paint_h

#include "trace_common.h"
//#include <linux/fb.h>

typedef unsigned short      uint16;
int paint_Line_uint16(uint16*pbmp, int width,int height, pixel_point point_start, pixel_point point_end, uint16 color, int stick);
int paint_MultiLine_uint16(uint16*pbmp, int width,int height, pixel_point* point_paint, int lens, uint16 color, int stick);


//int paint_Line_uint16(uint16*pbmp, int width,int height, int x_start, int y_start, int x_end, int y_end, uint16 color, int stick);

#endif


