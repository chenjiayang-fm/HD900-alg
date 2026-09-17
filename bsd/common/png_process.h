#ifndef __PNG_PROCESS__
#define __PNG_PROCESS__
#include <setjmp.h>
#include <png/pngconf.h>
#include <png/pnglibconf.h>
#include <png/png.h>
typedef unsigned short s16;
typedef unsigned long DWORD;
int load_png_image_toARGB1555(char *filepath, s16* dest, int width, int height);
#endif