#include <setjmp.h>
#include <png/pngconf.h>
#include <png/pnglibconf.h>
#include <png/png.h>
#include <stdio.h>
#include <string.h>
#include "png_process.h"
#include <print.h>
//typedef short int s16;
typedef unsigned long DWORD;

int load_png_image_toARGB1555(char *filepath, s16* dest, int _width, int _height)
{
    print_level(SV_INFO, "load png %s\n",filepath);
	FILE *fp = fopen(filepath,"rb");
	if(!fp){
        print_level(SV_ERROR, "open %s error!\n",filepath);
        return -1;
    }
    
	png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, 0, 0, 0);
	png_infop info_ptr = png_create_info_struct(png_ptr);

	// Check if the PNG image is valid
	char buf[4];
	setjmp(png_jmpbuf(png_ptr));
	int temp = fread(buf, 1, 4, fp);
	if (temp < 4) {
		fclose(fp);
		png_destroy_read_struct(&png_ptr, &info_ptr, 0);
        print_level(SV_ERROR, "fread error\n");
		return -1;
	}
	temp = png_sig_cmp((png_bytep)buf, (png_size_t)0, 4);
	if (temp != 0) {
		fclose(fp);
        print_level(SV_ERROR,"png_sig_cmp error\n");
		png_destroy_read_struct(&png_ptr, &info_ptr, 0);
		return -1;
	}

	rewind(fp);
	png_init_io(png_ptr, fp);
	png_read_png(png_ptr, info_ptr, PNG_TRANSFORM_EXPAND, 0);
	int color_type = png_get_color_type(png_ptr, info_ptr);
	int w = png_get_image_width(png_ptr, info_ptr);
	int h = png_get_image_height(png_ptr, info_ptr);
    int width=_width, height=_height;
    int deltax, deltay;
    print_level(SV_INFO,"canvas size width: %d, height: %d\n",_width,_height);
    print_level(SV_INFO,"load png size width: %d, height: %d\n", w, h);

    //将位置居中
    deltay = (height-h)/2;
    deltax = (width-w)/2;
	if (dest == NULL) {
		fclose(fp);
		png_destroy_read_struct(&png_ptr, &info_ptr, 0);
        print_level(SV_ERROR,"dest == NULL");
		return -1;
	}
    memset(dest, 0, sizeof(s16)*width*height);
	png_bytep* row_pointers = png_get_rows(png_ptr, info_ptr);
	switch (color_type) {
	case PNG_COLOR_TYPE_RGBA:
        print_level(SV_INFO, "BMP Format: PNG_COLOR_TYPE_RGBA\n");
		for (int y = 0; y<h; ++y) {
			for (int x = 0; x<w; x++) {
                if((x+deltax<0)||(x+deltax)>=width||(y+deltay)<0||(y+deltay)>=height)
                    continue;
                DWORD red = (DWORD)row_pointers[y][4*x+0];
				DWORD green = (DWORD)row_pointers[y][4*x+1];
				DWORD blue = (DWORD)row_pointers[y][4*x+2];
				DWORD alpha = (DWORD)row_pointers[y][4*x+3];
                
                dest[(y+deltay)*width+x+deltax] = (
                    ((alpha >> 7) &0x1) << 15|
                    ((red >> 3) & 0x1f) << 10|
                    ((green >> 3) & 0x1f) << 5|
                    ((blue >> 3) & 0x1f)
                );
			}
		}
		break;
	case PNG_COLOR_TYPE_RGB:
        print_level(SV_INFO, "BMP Format: PNG_COLOR_TYPE_RGB\n");
		for (int y = 0; y<h; ++y) {
			for (int x = 0; x<w; x++) {
                if((x+deltax<0)||(x+deltax)>=width||(y+deltay)<0||(y+deltay)>=height)
                    continue;
                DWORD red = (DWORD)row_pointers[y][3*x+0];
				DWORD green = (DWORD)row_pointers[y][3*x+1];
				DWORD blue = (DWORD)row_pointers[y][3*x+2];
                
                dest[(y+deltay)*width+x+deltax] = (
                    ((0xffff >> 7) &0x1) << 15|
                    ((red >> 3) & 0x1f) << 10|
                    ((green >> 3) & 0x1f) << 5|
                    ((blue >> 3) & 0x1f)
                );
			}
		}
		break;
	default:
		fclose(fp);
		png_destroy_read_struct(&png_ptr, &info_ptr, 0);
		return -1;
	}
	png_destroy_read_struct(&png_ptr, &info_ptr, 0);
    return 0;
}

