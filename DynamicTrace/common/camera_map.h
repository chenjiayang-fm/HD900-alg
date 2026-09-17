#include "trace_common.h"
#include <math.h>

#ifndef camera_map
#define camera_map

void Camera_Angle_Set(double roll, double pitch);
void Camera_Canvas_Set(int canvas_x, int canvas_y);
void Camera_Point_Map(pixel_point *point, double axis_x, double axis_y, double axis_z);
void Camera_Trace_Map(pixel_point *points, int lens, double theta_start, double theta_end, double radius, double High, double x_bias, double y_bias);
void Camera_LeftTrace_Map(pixel_point *points, int lens, double theta_start, double theta_end, double radius);
void Camera_RightTrace_Map(pixel_point *points, int lens, double theta_start, double theta_end, double radius);
#endif
