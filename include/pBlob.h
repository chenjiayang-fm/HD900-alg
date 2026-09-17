#ifndef PBLOB_H_
#define PBLOB_H_
//#include <iostream>
//#include <opencv2/opencv.hpp>
#include <arm_neon.h>
#include <math.h>
#include "common.h"
//#include <Eigen/Dense>
using namespace std;
using namespace cv;



class Datablob
{
	public:
		float* f32pdata;
		short* s16pdata;
		signed char* s8pdata;
		int width;
		int height;
		int channels;
		float f32ToS16Scale;
		float f32ToS8Scale;
		float maxval;
	public:
		Datablob()
		{
			f32pdata=NULL;
			s16pdata=NULL;
			s8pdata=NULL;
			width=0;
			height=0;
			channels=0;
			f32ToS16Scale=1.f;
			f32ToS8Scale=1.f;
			maxval=0.f;
		}
		~Datablob()
		{
			setNULL();
		}
		void setNULL()
		{
			if(f32pdata)
				free(f32pdata);
			if(s8pdata)
				free(s8pdata);
			if(s16pdata)
				free(s16pdata);
			width = height = channels = 0;
			f32ToS16Scale = 1.f;
			f32ToS8Scale = 1.f;
		}
		Datablob(int w,int h,int c)
		{
			f32pdata=NULL;
			s8pdata=NULL;
			s16pdata=NULL;
			creata(w,h,c);
		}
		bool creata(int w,int h,int c)
		{
			setNULL();
			width=w;
			height=h;
			channels=c;
			f32pdata=(float*)malloc(w*h*c*sizeof(float));
			s16pdata=(short*)malloc(w*h*c*sizeof(short));
			s8pdata=(signed char*)malloc(w*h*c*sizeof(signed char));
			if (f32pdata == NULL)
			{
				cerr << "Cannot alloc memeory for float data blob: " 
					<< width  << "*"
					<< height << "*"
					<< channels << endl;
				return false;
			}

			if (s16pdata == NULL)
			{
				cerr << "Cannot alloc memeory for uint16 data blob: "
					<< width << "*"
					<< height << "*"
					<< channels << endl;
				return false;
			}

			if (s8pdata == NULL)
			{
				cerr << "Cannot alloc memeory for uint8 data blob: "
					<< width << "*"
					<< height << "*"
					<< channels << endl;
				return false;
			}
			return true;
		}
};
class filters
{
	public: 
		short* wgt;
		signed char* wgts8;
		float* bias;
		float* prelu;
		int pad;
		int stridew;
		int strideh;
		int width;
		int height;
		int channels;
		int num;
		float f32ToS16Scale;
		float f32ToS8Scale;
	public:
		filters()
		{
			wgt=NULL;
			wgts8=NULL;
			bias=NULL;
			prelu=NULL;
			pad=0;
			stridew=0;
			strideh=0;
			width=0;
			height=0;
			channels=0;
			num=0;
			f32ToS16Scale=1.f;
			f32ToS8Scale=1.f;
		}
		filters(int p,int sw,int sh,int w,int h,int c,int n,float scale,int wgttype)
		{
			setbull();
			pad=p;
			stridew=sw;
			strideh=sh;
			width=w;
			height=h;
			channels=c;
			num=n;
			if(wgttype==1)
			{
				wgt=(short*)malloc(width*height*channels*num*sizeof(short));
				f32ToS16Scale=scale;
			}
			if(wgttype==2)
			{
				wgts8=(signed char*)malloc(width*height*channels*num*sizeof(signed char));
				f32ToS8Scale=scale;
			}			
			bias=(float*)malloc(num*sizeof(float));
			prelu=(float*)malloc(num*sizeof(float));
		}
		void setbull()
		{
			wgt=NULL;
			wgts8=NULL;
			bias=NULL;
			prelu=NULL;
		}
		~filters()
		{
			if (wgt)
				free(wgt);
			if (wgts8)
				free(wgts8);
			if (bias)
				free(bias);
			if (prelu)
				free(prelu);
		}
};














class Smokenet
{
	private:
		Datablob* blobptr[9];
		filters* wgtptr[9];
		short imgDatatable16s[256];
	public:
		Datablob* result;
		void run(Mat& img);
		Smokenet();
		void conv1();
		void imgToblob(Mat& img);
		Smokenet(int w,int h,char* modelname);
		void create(int w,int h,char* modelname);
		~Smokenet();
		void setnull();

};
class SmokenetT1
{
	private:
		Datablob* blobptr[9];
		filters* wgtptr[9];
		short imgDatatable16s[256];
	public:
		Datablob* result;
		void run(Mat& img);
		SmokenetT1();
		void conv1();
		void imgToblob(Mat& img);
		SmokenetT1(int w,int h,std::string _modelPath);
		void create(int w,int h,std::string _modelPath);
		~SmokenetT1();
		void setnull();

};
class eyeNet
{
	private:
		Datablob* blobptr[9];
		filters* wgtptr[9];
		short imgDatatable16s[256];
		short masktable16s[256];
	public:
		Datablob* result;
		void run(Mat& img,Mat& mask);
		eyeNet();
		void conv1();
		void imgToblob(Mat& img,Mat& mask);
		eyeNet(int w,int h);
		void create(int w,int h);
		~eyeNet();
		void setnull();

};

class poseNet
{
	private:
		Datablob* blobptr[2];
		filters* wgtptr[2];

	public:
		Datablob* result;
		void run(float* posefeat);
		poseNet();
		void imgToblob(float* posefeat);
		poseNet(int c,char* modelname);
		void create(int c,char* modelname);
		~poseNet();
		void setnull();

};
class phonenet
{
	private:
		Datablob* blobptr[9];
		filters* wgtptr[9];
		short imgDatatable16s[256];
	public:
		Datablob* result;
		void run(Mat& img);
		phonenet();
		void conv1();
		void imgToblob(Mat& img);
		phonenet(int w,int h);
		void create(int w,int h);	
		~phonenet();
		void setnull();

};
void blobf32Tos16(Datablob* blobptr,float s16scale);
void blobf32Tos8(Datablob* blobptr,float s16scale);

void Convdw(Datablob* blobptrIn,filters* wgtPtr,Datablob* blobptrOut);
void Convsep(Datablob* blobptrIn,filters* wgtPtr,Datablob* blobptrOut);
void ConvNs16(Datablob* blobptrIn,filters* wgtPtr,Datablob* blobptrOut);

void fullconnect1(Datablob* blobptrIn,filters* wgtPtr,Datablob* blobptrOut);

void scalefloatPrelu(Datablob* blobptr,filters* wgtPtr,float scale);
void scalefloat(Datablob* blobptr,filters* wgtPtr,float scale);
void softmax(Datablob* blobptr);
void bnScaleRelu(Datablob* blobptr,filters* wgtPtr,float scale);
void bnScaleReluMaxpool(Datablob* blobptr,filters* wgtPtr,Datablob* outputblob,float scale);

#endif
