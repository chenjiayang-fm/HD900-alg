

## DynamicTrace 项目说明



通过DynamicTrace 调用海思OSD模块和Track模块，实现倒车轨迹线的绘制。 



### DynamicTrace 项目目录结构



├── DynamicTrace.c     // Main 调用Track 和 OSD 实现倒车轨迹线的绘制 

├── Track.h         

├── Track.c         // Track模块 进行IMU数据采集和调用IMU模块追迹 

├── InertialMeasure.h  

├── InertialMeasure.c    // IMU模块 利用IMU数据追迹 

├── common         // common库 

│  ├── state.h       // matrix 矩阵库配置文件 

│  ├── matrix.h  

│  ├── matrix.c      // [matrix](https://github.com/Amoiensis/Matrix_hub) 矩阵库，进行矩阵运算 

│  ├── paint.h 

│  ├── paint.c       // paint 绘制图库，绘制直线和折线图 

│  ├── trace_common.h

│  ├── trace_common.c   // trace_common 常用函数库，包括系统时间和进程休眠 

│  ├── camera_map.h  

│  └── camera_map.c    // camera_map 相机映射库：实现三维物体与相机像素坐标的映射 

├── REAME.md        // 帮助文件 

└── Makefile        // 编译脚本 



### 参数配置

DynamicTrace 项目可以在如下文件中配置相应的参数

-  ./DynamicTrace.c 下可以在main()中配置相应的OSD画布信息

-  ./Track.c 下可以配置 adjust_param 加速计的矫正参数
- ./common/cammera_map.c 下配置摄像头内参、外参以及位置偏移



## 模块主要算法说明

### ./InertialMeasure.c   

以相机的摄像头方向前方，相机的右侧为x轴方向，相机后侧为y轴方向，相机的下方为z轴方向

- int IMU_init(pMatrix accel_data_init, pMatrix gyro_data_init)

> 静止状态时相机的欧拉角满足于 
> $$
> \psi = 0	\\
> sin \theta = \frac{g_x}{g} \\
> tan \phi = \frac{g_y}{g_z} \\
> $$

- void IMU_Kalman_Filter(double w0,double w1,double w2,double a0,double a1,double a2)

> 扩展卡尔曼滤波算法  
>
> 状态估计向量 x = $(p_x,p_y,p_z,v_x,v_y,v_z,q_0,q_1,q_2,q_3)$,   
>
> 其中 $p_x,p_y,p_z$为载体对地坐标，$v_x,v_y,v_z$为载体对地速度，$q_0,q_1,q_2,q_3$为载体对地四元数 
>
> 状态预测向量 z = $(v_x^b, a_x^b,v_z^b)$,    
>
> 其中$v_x^b$为载体对载体坐标系x轴速度，$a_x^b$为载体对载体坐标系x轴加速度除重力的加速度，$v_z^b$为载体对载体坐标系z轴速度
>
> 预测值计算
> $$
> \hat{v}_x^{b} = (q_0^2 + q_1^2 - q_2^2 - q_3^2)v_x+ (2 q_{1} q_{2}+2 q_{0} q_{3}) v_y + (2 q{1} q_{3}-2 q_{0} q_{2}) v_z\\
> 
>   \hat{v}*_z^{b} = (2 q_*{1} q_{3} +2 q_{0} q_{2})v_x + (2 q_{2} q_{3} -2 q_{0} q_{1})v_y + (q_0^2 - q_1^2 - q_2^2 + q_3^2)v_z\\
> 
>   \begin{align*}
> 
>   \hat{a}_x^{b}&= w_z^{b}\hat{v}_y^b \\
> 
>   &= w_z^{b}\left[(2 q_{1} q_{2} -2 q_{0} q_{3})v_x +( q_0^2 - q_1^2 + q_2^2 - q_3^2)v_y + (2 q_{2} q_{3} +2 q_{0} q_{1})v_z\right]
> 
>   \end{align*}
> $$

- Matrix *quat_to_rot(Matrix *quat)

  > 旋转矩阵$R_b^n\quad(n\rightarrow b)$计算为
  > $$
  > \left[\begin{array}{ccc}q_0^2 + q_1^2 - q_2^2 - q_3^2 & 2 q_{1} q_{2}+2 q_{0} q_{3} & 2 q_{1} q_{3}-2 q_{0} q_{2} \\2 q_{1} q_{2} -2 q_{0} q_{3} & q_0^2 - q_1^2 + q_2^2 - q_3^2 & 2 q_{2} q_{3} +2 q_{0} q_{1} \\2 q_{1} q_{3} +2 q_{0} q_{2}  & 2 q_{2} q_{3}  -2 q_{0} q_{1} & q_0^2 - q_1^2 - q_2^2 + q_3^2 \\\end{array}\right]
  > $$
  > 
  >
  > 

 ### ./common/paint.c

- int paint_Line_uint16(uint16*pbmp, int width,int height, pixel_point point_start, pixel_point point_end, uint16 color, int stick)

  >通用Bresenham算法进行直线绘制
  >
  >参考计算机图形学算法基础(机器工业出版社，第二版) page54：通用Bresenham算法



### ./common/camera_map.c

- void Camera_Point_Map(pixel_point *point, double axis_x, double axis_y, double axis_z)

  >相机畸变模型
  >
  >物体三维坐标$(x,y,z)$, 对应在图片坐标$(x^b,y^b)$,转换关系如下：
  >$$
  >\begin{array}{2}
  >&x_1 = \frac{x}{y}\\
  >&y_1 = \frac{z}{y}\\
  >&r^2 = x_1^2 + y_1^2 \\
  >&x_2 = (1+k_1*r^2+k_2*r^2)x_1 + 2p_1 x_1y_1 +p_1(r^2+2x_1^2)\\
  >&y_2 = (1+k_1*r^2+k_2*r^2)y_1 + p_1(r^2+2y_1^2)+ 2p_2 x_1y_1\\
  >&x^b = \lfloor f_x x_2 + u \rfloor\\
  >&y^b = \lfloor y_2 + v\rfloor
  >\end{array}
  >$$
  >其中$(f_x,f_y,u,v)$为相机内参， $(k_1,k_2,p_1,p_2,k_3)$为相机外参