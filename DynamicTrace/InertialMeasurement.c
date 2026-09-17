/******************************************************************************
Copyright (C) 广州敏视数码科技有限公司版权所有.

文件名：InertialMeasurement.c

作者: 郑南城    版本: v1.0.0(初始版本号)   日期: 2020-08-26

文件功能描述: 
	卡尔曼滤波过滤IMU采集数据
		状态估计: 对地位移qx,qy,qz;对地速度vx,vy,vz;对地四元数p0,p1,p2,p3
		预测值  : 载体速度vx`=0,vz`=0, 载体加速度ax1`= ax1 -ag
*******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "./common/matrix.h"
#include "InertialMeasurement.h"

#define KALMAN_ENABLE 1
#define KALMAN_DISABLE 0

#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))

#define ABS(x) ((x)<0? -(x) : (x))
//#define SIGN(x) ( (x)<0? -1 : ( (x) > 0 ? 1 : 0 ) )
#define SIGN(x) ( (x)<0? -1 : 1 )

//参数定义，包括重力值，角速度误差，加速度误差
#define gravity 9.7888
#define noise_accel 0.4//2.83e-2//0.1
#define noise_gyro (2.005524*3.14159265658/180) //0.1*...

//计算相应的协方差
#define cov_zerovel_a (noise_accel*noise_accel)
#define cov_zerovel_w (noise_gyro*noise_gyro)//(0.1*3.14159265658/180)
#define zerov_Wsize 5

//别名
typedef Matrix* pMatrix; 

//实时角速度，加速度数据以及初始化测量的角速度，加速度数据
pMatrix gyro_data, accel_data;

//加速度、角速度偏差
pMatrix bias_a,bias_w;

//状态、协方差、状态转移矩阵、外部不确定性矩阵
pMatrix x,P,F,Q;

//观测量、预测量、预测差值、测量矩阵、观测方差矩阵、卡尔曼增益、
pMatrix Z_observe,Z_prediction,y,H,R,K;

//角速度、加速度、四元数
pMatrix w,a,quat;

//中间变量
pMatrix Ow,Vq,Fc,Gq,Qc,S,R_S_n;

//静止时对地加速度(0,0,g)
pMatrix init_accel;

//对地加速度
pMatrix acc;

//对地坐标系小车速度，对小车坐标系小车速度
pMatrix v_ccs, v_car;

//旋转矩阵
pMatrix rot_matrix, drot_matrix;

pMatrix dquat;


//四元数
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

//旋转半径
double radius=1e3;
double radius_temp = 1e3;
double w_temp = 0.0f;

//重力加速度比例因子，默认为1
double gravity_pro = 1.0;

//时间步长，需要动态调整
volatile double deltat = (1.0/40);

//GSensor 载体坐标系下三轴重力分量
double gravity_axis[3] = {0,0,0};
//GSensor 载体坐标系实际加速度分量
double accelerate_axis[3] = {0,0,0};

//将角度转化为四元数
Matrix *angle_to_quat(double psi,double theta,double phi);

//将四元数转化为旋转矩阵
Matrix *quat_to_rot(Matrix *quat);

//计算对地比力场
Matrix *compVq(Matrix* quat, Matrix* a);

//计算量测矩阵
Matrix* compH(Matrix*quat, Matrix*v_ccs, Matrix* w);

Matrix* compH2(Matrix*dquat,Matrix*drot_matrix, Matrix*v_ccs, Matrix* w);

//计算旋转半径
void compRadius(double *_radius);

//对于加速度传感器采样的数据进行转化，alpha为比例因子
void onSensorChanged(double a0,double a1,double a2,double alpha);


pMatrix init_quat,init_rot;

/******************************************************************************
 * 函数功能: IMU 惯性模块初始化
 * 输入参数: 静态初始加速度，静态初始角度上
 * 输出参数: 角速度偏移，加速度偏移，初始姿态角
 * 返回值  : 无
 * 注意    : 无
 *****************************************************************************/
//IMU初始化
int IMU_init(pMatrix accel_data_init, pMatrix gyro_data_init){
	double init_psi,init_theta,init_phi;
	pMatrix accel_data_norm;
	
	pMatrix bias_a_as;

	accel_data_norm =M_mean(accel_data_init,1);
	accel_data_norm = M_numul(accel_data_norm,1/gravity);
	init_psi = 0;
	init_theta = -asin(accel_data_norm->data[0]);
	init_phi = atan2(accel_data_norm->data[1],accel_data_norm->data[2]);
	init_quat = angle_to_quat(init_psi,init_theta,init_phi);
	init_rot = quat_to_rot(init_quat);

	init_accel = M_Zeros(3,1);
	init_accel->data[2] = gravity;
		
	bias_a_as = M_mul(init_rot,init_accel);
	bias_a = M_mean(accel_data_init,1);
	//gravity_pro = gravity/M_norm(bias_a,2);

	//计算初始加速度偏差	
	bias_a = M_numul(bias_a,gravity_pro);
	bias_a = M_add_sub(1.0,bias_a,1.0,bias_a_as);


	//计算初始角速度偏差
	bias_w = M_mean(gyro_data_init,1);
	
	//拓展算法一阶滤波
	for(int i=0;i<3;i++)
	{
		gravity_axis[i] = init_accel->data[i];
	}
	
	//初始化状态x,以及协方差P,误差R
	x = M_Zeros(10,1);
	x->data[6]=init_quat->data[0],x->data[7]=init_quat->data[1];
	x->data[8]=init_quat->data[2],x->data[9]=init_quat->data[3];
	q0 = x->data[6],q1 = x->data[7],q2 = x->data[8],q3 = x->data[9];
	
	P = M_Zeros(10,10);
	P = M_matFull(P,0,0,M_numul(M_I(6),1e-10));
	P = M_matFull(P,4,4,M_numul(M_I(4), 1e-6));
	
	R = M_numul(M_I(3),pow(noise_accel,2));


	//对所需内容进行备份，防止丢失数据
	init_quat = Matrix_copy(init_quat);
	x = Matrix_copy(x);
	R = Matrix_copy(R);
	P = Matrix_copy(P);
	bias_a = Matrix_copy(bias_a);
	bias_w = Matrix_copy(bias_w);
	init_accel = Matrix_copy(init_accel);
	H = M_Zeros(8,3);
	
	//释放内存,防止内存泄漏
	M_releasememory(8);
	
}


void IMU_Kalman_Filter(double w0,double w1,double w2,double a0,double a1,double a2){
	//矩阵temp和data1作为中间变量
	Matrix *temp;
	double *data1;
	int i=0;

	
	//读取角速度并排除偏差影响
	w = M_Nones(3,1);
	w->data[0]=w0,w->data[1]=w1,w->data[2]=w2; 
	w = M_add_sub(1.0,w,1.0,bias_w);

	//读取加速度
	a = M_Nones(3,1);
	a->data[0]=0,a->data[1]=a1,a->data[2]=a2; 
	a = M_add_sub(1.0,a,1.0,bias_a);

	/*****************************************************************************
	* 使用卡尔曼滤波效果不佳，可能是惯性测量模块本身的精度问题，或是算法本身的缺陷。
	* 因此，可以选用卡尔曼滤波，或者直接规定速度的一般值来衡量轨迹半径，但此时无法更
	* 新摄像头的姿态角
	******************************************************************************/ 
	//
#if KALMAN_DISABLE
	//使用一阶低通滤波，过滤重力加速度并得出惯性系加速度
	onSensorChanged(a0,a1,a2,0.8);
	
	//x->data[6]=init_quat->data[0],x->data[7]=init_quat->data[1];
	//x->data[8]=init_quat->data[2],x->data[9]=init_quat->data[3];

	//获取四元数
	
	quat = M_Cut(x,7,10,1,1);
	
	//Vq为 ∂v/∂q 四元数对速度的偏导矩阵，以计算相应的状态转移矩阵 
	Vq = compVq(quat,a);
	
	//Ow为四元数姿态更新矩阵
	Ow = M_Nones(4,4);
	data1 = Ow->data;
	data1[0]= 0,data1[1]=-w0,data1[2]=-w1,data1[3]=-w2;
	data1[4]=w0,data1[5]=0,data1[6]=w2,data1[7]=-w1;
	data1[8]=w1,data1[9]=-w2,data1[10]=0,data1[11]=w0;
	data1[12]=w2,data1[13]=w1,data1[14]=-w0,data1[15]=0;
	data1 = NULL;
	
	//将偏导矩阵与姿态更新矩阵合并，得到非对角元的临时矩阵
	Fc = M_Zeros(10,10);
	Fc = M_matFull(Fc,0,3,M_I(3));
	
	Fc = M_matFull(Fc,3,6,Vq);
	
	Fc = M_matFull(Fc,6,6,M_numul(Ow,0.5));
	
	//得到状态转移矩阵(使用扩展卡尔曼滤波器)
	F = M_add_sub(1.0,M_I(10),-1.0,M_numul(Fc,deltat));
	
	// 由陀螺仪的误差以及姿态更新矩阵可以推出相应的矩阵Gq,Gq*Gq.T为协方差矩阵
	Gq = M_Nones(4,3);
	data1 = Gq->data;
	data1[0]=-quat->data[1],data1[1]=-quat->data[2],data1[2]=-quat->data[3];
	data1[3]= quat->data[0],data1[4]=-quat->data[3],data1[5]= quat->data[2];
	data1[6]= quat->data[3],data1[7]= quat->data[0],data1[8]=-quat->data[1];
	data1[9]=-quat->data[2],data1[10]= quat->data[1],data1[11]= quat->data[0];
	data1 = NULL;
	Gq = M_numul(Gq,0.5);
	
	//得到相应的协方差矩阵
	Qc = M_Zeros(10,10);
	Qc = M_matFull(Qc,3,3,M_numul(M_I(3),noise_accel*noise_accel));
	Qc = M_matFull(Qc,6,6,M_numul(M_mul(Gq,M_T(Gq)),noise_gyro*noise_gyro));
	Q = M_numul(Qc,deltat);
	
	//---------------------------------------------------------------------------------------------------
	//卡尔曼滤波状态更新

	//将加速度由车载坐标系转化为导航坐标系
	R_S_n = quat_to_rot(quat);
	acc = M_Nones(3,1);
	for(int i=0;i<3;i++)
		acc->data[i] = accelerate_axis[i];
	acc = M_mul(M_T(R_S_n),acc);

	//更新坐标
	temp = M_add_sub(1.0,M_Cut(x,1,3,1,1),-1.0*deltat,M_Cut(x,4,6,1,1));
	temp = M_add_sub(1.0,temp,-0.5*deltat*deltat,acc);
	x = M_matFull(x,0,0,temp);
	
	
	//更新速度
	temp = M_add_sub(1.0,M_Cut(x,4,6,1,1),-1.0*deltat,acc);
	x = M_matFull(x,3,0,temp);

	//更新四元数,同时计算差分式
	dquat = quat;
	drot_matrix = quat_to_rot(quat);
	
	temp = M_numul(Ow,0.5*deltat);
	temp = M_mul(M_add_sub(1.0,M_I(4),-1,temp),quat);
	quat = M_numul(temp,1/M_norm(temp,2)); //归一化
	rot_matrix = quat_to_rot(quat);

	dquat = M_add_sub(1.0,quat,1.0,dquat);
	drot_matrix = M_add_sub(1.0,rot_matrix,1.0,drot_matrix);
	
	x = M_matFull(x,6,0,quat);
	
	//更新协方差
	P = M_mul(M_mul(F,P),M_T(F));
	P = M_add_sub(1.0,P,-1.0,Q);
	//---------------------------------------------------------------------------------------------------
	//卡尔曼滤波状态修正
	
	if(1){
		//使用车辆惯性系中的v_x, a_x, v_z作为观测值，速度vx,加速度ax,速度vz，
		//利用旋转矩阵，得到预测值
		v_ccs = M_Cut(x,4,6,1,1);
		v_car = M_mul(rot_matrix,v_ccs);

		Z_prediction = M_Nones(3,1);
		data1 = Z_prediction->data;
		data1[0] = v_car->data[0],data1[1] = w->data[2]*v_car->data[1];
		data1[2] = v_car->data[2];
		
		
		//利用旋转矩阵，得到预测值
		Z_observe = M_Nones(3,1);
		data1 = Z_observe->data;
		data1[0] = 0,data1[2]=0;
		data1[1] = gravity_axis[0] - M_mul(rot_matrix,init_accel)->data[0];
		data1[1] = data1[1];
		
		//得到观测和预测的差值
		y = M_add_sub(1.0,Z_observe,1.0,Z_prediction);

		//获取观测矩阵H
		H = compH(quat,v_ccs,w);
		
		//观测误差
		R = M_Zeros(3,3);
		R->data[0] = R->data[8] = 1e-2;//1e-4*10;
		R->data[4] = (1e-2)*10;
		
		//获取卡尔曼增益
		S = M_mul(H,M_mul(P,M_T(H)));
		S = M_add_sub(1.0,S,-1.0,R);
		K = M_mul(M_mul(P,M_T(H)),M_Inverse(S));
		
		//修正系统状态
		x = M_add_sub(1.0,x,-1.0,M_mul(K,y));
		temp = M_Cut(x,7,10,1,1);
		temp = M_numul(temp,1/M_norm(temp,2));
		x = M_matFull(x,6,0,temp);
		
		temp = M_add_sub(1.0,M_I(10),1.0,M_mul(K,H));
		P = M_mul(temp,P);
		
	}
	
	x = M_matFull(x,6,0,quat);
	quat = M_Cut(x,7,10,1,1);
	v_ccs = M_Cut(x,4,6,1,1);
	rot_matrix = quat_to_rot(quat);
	v_car = M_mul(M_T(rot_matrix),v_ccs);
	radius = v_car->data[1] / w->data[2];
	//---------------------------------------------------------------------------------------------------
	//结束处理
	
	P = M_add_sub(0.5,P,-0.5,M_T(P));

	q0 = x->data[6],q1 = x->data[7],q2 = x->data[8],q3 = x->data[9];

	init_quat = Matrix_copy(init_quat);
	x = Matrix_copy(x);
	H = Matrix_copy(H);
	R = Matrix_copy(R);
	P = Matrix_copy(P);
	bias_a = Matrix_copy(bias_a);
	bias_w = Matrix_copy(bias_w);
	init_accel = Matrix_copy(init_accel);
	M_releasememory(8);
#else
	w_temp = 0.9*w_temp + 0.1*w->data[2];				//滑动平均值
	radius = - 2*SIGN(w_temp) / (ABS(w_temp) + 1e-5);	//设定速度为2m/s, 计算相应的速度
	radius = ABS(radius)>50? 1e3: radius;
	bias_a = Matrix_copy(bias_a);
	bias_w = Matrix_copy(bias_w);
	M_releasememory(2);
#endif
}


/******************************************************************************
 * 函数功能: 对加速度进行低通滤波
 * 实现方法：使用通用的低通滤波器，该算法在安卓上常用
 * 输入参数: 三轴加速度
 * 输出参数: 三轴所受重力加速度值，三轴的实际加速度值
 * 返回值  : 无
 * 注意    : 无
 *****************************************************************************/
void onSensorChanged(double a0,double a1,double a2,double alpha)
{
	gravity_axis[0] = alpha*gravity_axis[0] + (1-alpha)*a0;
	gravity_axis[1] = alpha*gravity_axis[1] + (1-alpha)*a1;
	gravity_axis[2] = alpha*gravity_axis[2] + (1-alpha)*a2;

	accelerate_axis[0] = a0 -gravity_axis[0];
	accelerate_axis[1] = a1 -gravity_axis[1];
	accelerate_axis[2] = a2 -gravity_axis[2];
}

/******************************************************************************
 * 函数功能: 将欧拉角转换为旋转矩阵
 * 输入参数: 欧拉角
 * 输出参数: 旋转矩阵
 * 返回值  : 无
 * 注意    : 输入的三个角分别为欧拉角𝜓、𝜃、𝜙。利用欧拉角转变为旋转矩阵
 *****************************************************************************/
Matrix *angle_to_quat(double psi,double theta,double phi){
	double angle[3] = {psi,theta,phi};
	double cang[3],sang[3];
	Matrix *quat = M_applymemory();
	double *data = (double*)malloc(4*sizeof(double));
	for(int i=0;i<3;i++){
		cang[i]=cos(angle[i]/2);
		sang[i]=sin(angle[i]/2);
	}
	data[0] = cang[0]*cang[1]*cang[2] + sang[0]*sang[1]*sang[2];
	data[1] = cang[0]*cang[1]*sang[2] - sang[0]*sang[1]*cang[2];
	data[2] = cang[0]*sang[1]*cang[2] + sang[0]*cang[1]*sang[2];
	data[3] = sang[0]*cang[1]*cang[2] - cang[0]*sang[1]*sang[2];

	quat->data = data;
	quat->row = 4;
	quat->column = 1;
	return quat;
}
/******************************************************************************
 * 函数功能: 将四元数转换为旋转矩阵
 * 输入参数: 四元数
 * 输出参数: 旋转矩阵
 * 返回值  : 旋转矩阵
 * 注意    : 无
 *****************************************************************************/
Matrix *quat_to_rot(Matrix *quat){
	Matrix *rot = M_applymemory();
	double *dcm = (double*)malloc(3*3*sizeof(double));
	double *q = quat->data;
	
	dcm[0] = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
	dcm[1] = 2*(q[1]*q[2] + q[0]*q[3]);
	dcm[2] = 2*(q[1]*q[3] - q[0]*q[2]);
	dcm[3] = 2*(q[1]*q[2] - q[0]*q[3]);
	dcm[4] = q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3];
	dcm[5] = 2*(q[2]*q[3] + q[0]*q[1]);
	dcm[6] = 2*(q[1]*q[3] + q[0]*q[2]);
	dcm[7] = 2*(q[2]*q[3] - q[0]*q[1]);
	dcm[8] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
	

	rot->data = dcm;
	rot->row = 3;
	rot->column = 3;
	return rot;
}

/******************************************************************************
 * 函数功能: 计算Vq
 * 输入参数: 四元数，加速度
 * 输出参数: 无
 * 返回值  : 无
 * 注意    : 无
 *****************************************************************************/
Matrix *compVq(Matrix* quat, Matrix* a){
	double q0,q1,q2,q3;
	double ax,ay,az;
	double *data = (double*)malloc(3*4*sizeof(double));
	Matrix *vq =  M_applymemory();
	
	
	q0 = quat->data[0],q1 = quat->data[1];
	q2 = quat->data[2],q3 = quat->data[3];
	ax = a->data[0],ay = a->data[1],az = a->data[2];
	
	//printf("result: %lf|%lf\n",2*(-0.6083*0.0186+0.3401*0.0301-9.6423*0.0006),ax*q1 + ay*q2 + az*q3);
	
	data[0] = 2*(ax*q0 - ay*q3 + az*q2);
	data[4] = 2*(ax*q3 + ay*q0 - az*q1);
	data[8] = 2*(ay*q1 - ax*q2 + az*q0);
	
	data[1] = 2*(ax*q1 + ay*q2 + az*q3);
	data[5] = 2*(ax*q2 - ay*q1 - az*q0);
	data[9] = 2*(ax*q3 + ay*q0 - az*q1);
	
	data[2] = 2*(ay*q1 - ax*q2 + az*q0);
	data[6] = 2*(ax*q1 + ay*q2 + az*q3);
	data[10] = 2*(ay*q3 - ax*q0 - az*q2);

	data[3] = 2*(az*q1 - ay*q0 - ax*q3);
	data[7] = 2*(ax*q0 - ay*q3 + az*q2);
	data[11] = 2*(ax*q1 + ay*q2 + az*q3);

	
	vq->data = data;
	vq->row = 3;
	vq->column = 4;
	return vq;	
}

/******************************************************************************
 * 函数功能: 计算H
 * 输入参数: 四元数，速度, 角速度
 * 输出参数: 无
 * 返回值  : 无
 * 注意    : 无
 *****************************************************************************/

Matrix* compH(Matrix*quat, Matrix*v_ccs, Matrix* w){
	pMatrix _H = M_Zeros(3,10);
	pMatrix rot = quat_to_rot(quat);
	float scale = w->data[2];
	float vx,vy,vz,q0,q1,q2,q3;
	int i;
	_H = M_matFull(_H,0,3,rot);
	vx = v_ccs->data[0],vy = v_ccs->data[1],vz = v_ccs->data[2];
	q0 = quat->data[0], q1=quat->data[1], q2=quat->data[2],q3=quat->data[3];
	
	for(i=3;i<3+3;i++){
		_H->data[i+10] = _H->data[i+10] * scale;
	}

	
	_H->data[6] = 2*(q0*vx+q3*vy-q2*vz);
	_H->data[7] = 2*(q1*vx+q2*vy+q3*vz);
	_H->data[8] = 2*(-q2*vx+q1*vy-q0*vz);
	_H->data[9] = 2*(-q3*vx+q0*vy+q1*vz);
	
	_H->data[16] = 2*(-q3*vx+q0*vy+q1*vz)*scale;
	_H->data[17] = 2*(q2*vx-q1*vy+q0*vz)*scale;
	_H->data[18] = 2*(q1*vx+q2*vy+q3*vz)*scale;
	_H->data[19] = 2*(-q0*vx-q3*vy+q2*vz)*scale;
	
	_H->data[26] = 2*(q2*vx-q1*vy+q0*vz);
	_H->data[27] = 2*(q3*vx-q0*vy-q1*vz);
	_H->data[28] = 2*(q0*vx+q3*vy-q2*vz);
	_H->data[29] = 2*(q1*vx+q2*vy+q3*vz);	
	
	
	return _H;
	
}

//配置IMU时间
void IMU_setDeltat(double t)
{
	deltat = t;
}

//---------------------------------------------------------------------------------------------------------------
//计算roll、pitch、yaw
void IMU_getRoll(double *roll){
	double temp = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * 57.29578;
	*roll = temp;
	//printf("s1:%lf,s2:%lf\n",q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
	//printf("Rool-%f: %f, %f\n", roll,(q0*q1 + q2*q3),(0.5f - q1*q1 - q2*q2) );
	return ;
}
//Calculate Pitch
void IMU_getPitch(double *pitch){
	double temp = asin(-2.0 * (q1*q3 - q0*q2))* 57.29578;
	*pitch = temp;
	//printf("pitch-%f: %lf\n", pitch,(-2.0f * (q1*q3 - q0*q2)));
	return ;
}
//Calculate Yaw
void IMU_getYaw(double *yaw){
	double temp = atan2(q1*q2 + q0*q3, 0.5 - q2*q2 - q3*q3)* 57.29578;
	*yaw = temp;
	//printf("Rool-%f: %f, %f\n", yaw,(double)(0.5f - q2*q2 - q3*q3),(double)(q1*q2 + q0*q3));
	return ;
}
void IMU_getRadius(double *_radius){
	*_radius = radius;
	return ;
}
//---------------------------------------------------------------------------------------------------------------


