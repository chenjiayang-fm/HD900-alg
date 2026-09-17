/******************************************************************************
Copyright (C) 广州敏视数码科技有限公司版权所有.

文件名：Track.c

作者: 郑南城    版本: v1.0.0(初始版本号)   日期: 2020-08-26

文件功能描述: 获取惯性传感器数据以及调用InertialMeasurement模块进行数据追迹

*******************************************************************************/

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h> // close function
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <fcntl.h>
#include <linux/fb.h>

#include "../ko/bmg160/bmg160-dev.h"
#include "../ko/bmg160/bmg160.h"

#include "../ko/bmi055/bmi055-dev.h"
#include "../ko/bmi055/bma2x2.h"

#include <math.h>
#include <termios.h>
#include <errno.h>
#include "./common/matrix.h"

#include <stdio.h>
#include "InertialMeasurement.h" 
#include "print.h"
#include "Track.h"

#include "common.h"

#include "./common/trace_common.h"

#define BMI_GET_ACCXYZT     	0x00011

typedef struct bmg160_data_t bmg160_data; //gyro
typedef struct bma2x2_accel_data_temp bmi055_data; //accelerator

//惯性传感器数据
typedef struct bmg160_bmi055_data{
	float gyro_x;	//陀螺仪x轴角速度
	float gyro_y;	//陀螺仪y轴角速度
	float gyro_z;	//陀螺仪z轴角速度
	float accelerate_x;	//加速计x轴加速度
	float accelerate_y;	//加速计y轴加速度
	float accelerate_z;	//加速度z轴加速度
}IMU_data; 

//加速计矫正参数
const double adjust_param[] = {1.00016306, -0.11014373,  0.99148386, -0.14815845,  1.00303956, 0.02231303};

//相机三个欧拉角
volatile double roll=0.0,pitch=0.0,yaw=0.0;

//小车转弯半径
volatile double track_radius = 1e3;

//扩展卡尔曼滤波状态，引用IMU模块中的全局变量
extern Matrix * x;

//全局变量bmg160,bmi055句柄
int bmg160_fd = 0, bmi055_fd = 0;

//IMU数据及初始化
bmg160_data track_gyro_data = {0};
bmi055_data track_acc_data = {0};
IMU_data imu_data = {0};

//输出调试信息值
void Track_Printf(int mode);

//将陀螺仪和加速计数据转换为真实值并存储在IMU_data中
void Track_RawGyroAcc_Convert(IMU_data *imu_data,bmg160_data *gyro_data,bmi055_data *acc_data){
	//转换坐标轴，并调整数据范围
	imu_data->gyro_x = -gyro_data->datay*250.0/32768.0*0.0174532925199433;   //range ->(±250°)，单位弧度
	imu_data->gyro_y = gyro_data->dataz*250.0/32768.0*0.0174532925199433;
	imu_data->gyro_z = -gyro_data->datax*250.0/32768.0*0.0174532925199433;  
	imu_data->accelerate_x = -(acc_data->y)*2.0/32768.0*16*9.788 ;                 //range ->(±2g)， 单位m/s^2
	imu_data->accelerate_y = (acc_data->z)*2.0/32768.0*16*9.788;
	imu_data->accelerate_z = -(acc_data->x)*2.0/32768.0*16*9.788;
	
	//加速计数据矫正
	imu_data->accelerate_x = adjust_param[0]*(imu_data->accelerate_x - adjust_param[1]);
	imu_data->accelerate_y = adjust_param[2]*(imu_data->accelerate_y - adjust_param[3]);
	imu_data->accelerate_z = adjust_param[4]*(imu_data->accelerate_z - adjust_param[5]);
}




//陀螺仪和加速计初始化
int Track_GSensor_Init()
{
	bmg160_fd = open("/dev/bmg160",O_RDONLY);
	if (!bmg160_fd)
	{
		perror("open bmg160 fail\n");
		printf("0x%x\n",bmg160_fd);
		return bmg160_fd;
	}
	bmi055_fd = open("/dev/bmi055",O_RDONLY);
	if (!bmi055_fd)
	{
		perror("open bmi055 fail\n");
		return bmi055_fd;
	}
	return SUCCESS;
}


/******************************************************************************
 * 函数功能: IMU模块数据采集和初始化
 * 输入参数: 无
 * 输出参数: 无
 * 返回值  : SV_SUCCESS - 成功
 * 注意    : 无
 *****************************************************************************/
int Track_IMU_init()
{
	//矩阵初始化
	Matrix* imudata = (Matrix*)malloc(sizeof(Matrix));
	Matrix* accel_data_init, *gyro_data_init;
	
	int i=0, ret=0, initnum = 75;

    //配置矩阵
	imudata->data = (double*)malloc(sizeof(double)*6*initnum);
	imudata->row = 6;
	imudata->column = initnum;
	
	//采集初始化
	while(i<initnum){
		if((ret = ioctl(bmg160_fd, BMG_GET_DATAXYZI, &track_gyro_data)) < 0)
		{
			printf("Fail BMG_GET_DATAXYZI %s\n",strerror(errno));
			return(ret);
		}
		if((ret = ioctl(bmi055_fd,BMI_GET_ACCXYZT,&track_acc_data)) < 0)
		{
			printf("Fail BMI_GET_ACCXYZT %s\n",strerror(errno));
			return(ret);
		}

		Track_RawGyroAcc_Convert(&imu_data,&track_gyro_data,&track_acc_data);
		
		imudata->data[initnum*0+i]=imu_data.accelerate_x,imudata->data[initnum*1+i]=imu_data.accelerate_y,imudata->data[initnum*2+i]=imu_data.accelerate_z;
		imudata->data[initnum*3+i]=imu_data.gyro_x,imudata->data[initnum*4+i]=imu_data.gyro_y,imudata->data[initnum*5+i]=imu_data.gyro_z;
		i++;
	}
	
	//切割矩阵赋予加速计和陀螺仪，这里无需free，会自动回收内存
	accel_data_init = M_Cut(imudata,1,3,0,initnum);
	gyro_data_init = M_Cut(imudata,4,6,0,initnum);

	//调用IMU接口，获取初始信息
	IMU_init(accel_data_init,gyro_data_init);
	IMU_getRoll(&roll);
	IMU_getPitch(&pitch);
	IMU_getYaw(&yaw);
	IMU_getRadius(&track_radius);

	printf("finish IMU\n");
	return SUCCESS;
}

/******************************************************************************
 * 函数功能: Track 追迹模块初始化
 * 实现方法：调用Track_GSensor_Init 和 Track_IMU_init 接口
 * 输入参数: 无
 * 输出参数: 无
 * 返回值  : SV_SUCCESS - 成功
 * 注意    : 无
 *****************************************************************************/
//
int Track_Init()
{
	int s32ret;
	s32ret = Track_GSensor_Init();
	if(s32ret == ERROR)
	{
		print_level(SV_ERROR, "Make_Sensor_init faild! [err=%#x]\n", s32ret);
	}
	s32ret = Track_IMU_init();
	if(s32ret == ERROR)
	{
		print_level(SV_ERROR, "Make_IMU_init faild! [err=%#x]\n", s32ret);
	}
	return SUCCESS;
}

/******************************************************************************
 * 函数功能: Track 实时追迹
 * 输入参数: 无
 * 输出参数: 无
 * 返回值  : SV_SUCCESS - 成功
 * 注意    : 无
 *****************************************************************************/
int Track_Realtime_Update()
{
	long long int subend1,substart1,subend2;  //实时记录while循环中，各子模块的时间
    long long int subtime = 0;
    double deltat_pre = 0,  deltat = 0;                 // 采样时间
	int ret, i;

    //long long int starttime,endtime;                  //记录循环总时长，检测采样频率
	//starttime = Track_microtime();
	while(SV_TRUE){
		substart1 = microtime();	
		if ((ret = ioctl(bmg160_fd, BMG_GET_DATAXYZI, &track_gyro_data)) < 0)
		{	
			printf("Fail BMG_GET_DATAXYZI %s\n",strerror(errno));
			return(ret);
		}
		
		if ((ret = ioctl(bmi055_fd,BMI_GET_ACCXYZT,&track_acc_data)) < 0)
		{	
			printf("Fail BMI_GET_ACCXYZT %s\n",strerror(errno));
			return(ret);
		}
		
		Track_RawGyroAcc_Convert(&imu_data,&track_gyro_data,&track_acc_data);
		subend1 = microtime();
		
		
		deltat = ((subend1 - substart1)+subtime) / 1.0e6;
		deltat = (deltat + deltat_pre) / 2.0;
		deltat_pre = ((subend1 - substart1)+subtime) / 1.0e6;
		IMU_setDeltat(deltat);

		IMU_Kalman_Filter(imu_data.gyro_x,imu_data.gyro_y,imu_data.gyro_z,
				imu_data.accelerate_x,imu_data.accelerate_y,imu_data.accelerate_z);
		
		IMU_getRoll(&roll);
		IMU_getPitch(&pitch);
		IMU_getYaw(&yaw);
		IMU_getRadius(&track_radius);
		

		select_sleep(0,1*1000);
		subend2 = microtime();
		subtime = subend2 - substart1;
		//printf("readtime1:%lld\n",subtime);
		//i+=1;
	}
	//endtime = Track_microtime();
	//printf("average take time:%lf\n",1.0*(endtime-starttime)/i/1000000);
	//printf("freq:%lf\n",1000*1000.0/(1.0*(endtime-starttime)/i));
}

/******************************************************************************
 * 函数功能: 输出调试信息
 * 输入参数: mode:	 0 -> print imu data
 * 				    1 -> print orientation
 * 					2 -> print speed
 * 输出参数: 无
 * 返回值  : SV_SUCCESS - 成功
 * 注意    : 无
 *****************************************************************************/
void Track_Printf(int mode)
{
    char senddata[100];
	switch(mode){
		case 0:
			sprintf(senddata,"%s %f %f %f %f %f %f \n","Raw data:",imu_data.accelerate_x,imu_data.accelerate_y,imu_data.accelerate_z,
				imu_data.gyro_x,imu_data.gyro_y,imu_data.gyro_z);
			printf(senddata);
			break;
		case 1:
			sprintf(senddata,"%s %f %f %f \n","Orientation:",roll,pitch,yaw);
			printf(senddata);
			break;
		case 2:
			sprintf(senddata,"%s %f %f %f \n","Speed::",x->data[3],x->data[4],x->data[5]);
			printf(senddata);
			break;
		default:
			break;
	}
	return NULL;
}

/******************************************************************************
 * 函数功能: Track 注销
 * 输入参数: 无
 * 输出参数: 无
 * 返回值  : 无
 * 注意    : 无
 *****************************************************************************/
void Track_Exit()
{
	if(bmg160_fd != 0)
	{
		close(bmg160_fd);
	}

	if(bmi055_fd != 0)
	{
		close(bmi055_fd);
	}
}

/******************************************************************************
 * 函数功能: Track 获取追迹半径
 * 输入参数: 无
 * 输出参数: 无
 * 返回值  : 无
 * 注意    : 无
 *****************************************************************************/
double Track_getRadius(double*_raduis)
{
	*_raduis = track_radius;
	return track_radius;
}

/*
int main()
{
	Track_Init();
	
	

	return 1;
}
*/

