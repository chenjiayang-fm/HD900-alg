#ifndef paint_h
#define paint_h

//#include <linux/fb.h>
typedef struct Pixel_Point
{
    int x_aixs;
    int y_axis;
} pixel_point;

typedef struct Pixel_Point_Scale
{
    float x_scale;
    float y_scale;
} pixel_point_scale;

typedef unsigned short      uint16;
int paint_Line_uint16(uint16*pbmp, int width,int height, pixel_point point_start, pixel_point point_end, uint16 color, int stick);
int paint_MultiLine_uint16(uint16*pbmp, int width,int height, pixel_point* point_paint, int lens, uint16 color, int stick);
int paint_Line_scale_uint16(uint16*pbmp, int width,int height, pixel_point_scale point_start_scale, \
                            pixel_point_scale point_end_scale, uint16 color, int stick);
int paint_MultiLine_scale_uint16(uint16*pbmp, int width,int height, pixel_point_scale* point_scale_paint, int lens, uint16 color, int stick);
void print_point(pixel_point point_paint);
void print_point_scale(pixel_point_scale point_paint);
//int paint_Line_uint16(uint16*pbmp, int width,int height, int x_start, int y_start, int x_end, int y_end, uint16 color, int stick);

#endif


