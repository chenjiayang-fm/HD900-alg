#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/syscall.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <pthread.h>
#include <ctype.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>
#include <dirent.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <linux/videodev2.h>
#include <iostream>

#include "../include/itc.h"
#include "../../../include/common.h"

#define UVC_BUF_NUM     12
#define UVC_CAM_WIDTH   102448
#define UVC_CAM_HEIGHT  1

#define RINGBUF_VDEC_COUNT       12
#define RINGBUF_VDEC_SIZE       UVC_CAM_WIDTH*UVC_CAM_HEIGHT //PD_IMAGE_WIDTH *  PD_IMAGE_HEIGHT *3 //   RGB //642048 //
TRWRingQueue<int> ringque_VDEC(RINGBUF_VDEC_COUNT-1);

unsigned char MptrData[RINGBUF_VDEC_COUNT][RINGBUF_VDEC_SIZE];

typedef enum
{
    upgrade_status_init = 0,
    upgrade_status_transfer ,
    upgrade_status_filecompleted ,
    upgrade_status_unzip ,
    upgrade_status_update ,
    upgrade_status_end ,
}upgrade_status_e;

FILE *fp = NULL;
FILE *fp_status = NULL;
uint8_t file_count = 0;
uint64_t rest_file_size = 0;
uint8_t upgrade_status = 0;

typedef struct tagVpssAlgUvcFrame_S
{
    sint32              s32AlgChnFd;            /* UVC摄像头视频节点描述符 */
    unsigned char       *mptr[UVC_BUF_NUM];     /* UVC摄像头映射内存地址 */
} MPP_VPSS_ALG_FRAME_S;

typedef struct upgrade_flag
{
    uint8 md5check;

} upgrade_flag_s;

MPP_VPSS_ALG_FRAME_S m_stVpssAlgFrame = {0};    /* UVC摄像头算法控制信息 */ 
upgrade_flag_s m_upgrade_flag = {0}; /*升级流程*/

sint32 s32SerialFd[4] = {0}; /*串口文件句柄*/

sint32 mpp_vpss_InitAlgChn();
uint32_t serial_init(const sint8* port, uint32_t baud,uint8_t CH);
void uart_recieve();
int SAFE_System_Recv(char* cmdStr,char *recv, int lens);
sint32 mpp_vpss_GetAlgFrame();
sint32 mpp_vpss_file_save();
void mpp_vpss_file_detect();  //一定时间没收到以后认为结束。close file

static void exit_handle(int signalnum)
{
    sint32 s32Ret = 0;
    
    printf("catch signalnum %d!\n", signalnum);
    exit(EXIT_FAILURE);
}

void test_upgrade()
{
    sint32 s32Ret = 0;
    char PackageBuf[100]= {0};
    char cmdStr[128] = "ls /root/upgradefile/testfile";
    SAFE_System_Recv(cmdStr, PackageBuf ,sizeof(PackageBuf));
    printf("local upgrade Package :%s\n", PackageBuf);
 
     //解压压缩包
    memset(cmdStr,0,sizeof(cmdStr));
    sprintf(cmdStr,"tar -xvf %s",PackageBuf);
    s32Ret = system(cmdStr); //阻塞式的，执行完这步才到下步
 
    if(s32Ret != 0)
    {
     printf("depackage err :%d\n", s32Ret);
    }
    else
    {
     printf("depackage success :%d\n");  
    }
 
    s32Ret = system("/root/upgradefile/upgrade.sh");

    if(s32Ret != 0)
    {
     printf("upgrade err :%d\n", s32Ret);
    }
    else
    {
     printf("upgrade success :%d\n");  
     upgrade_status = upgrade_status_end;
     fclose(fp_status);
     system("pkill -9 upgrade");
    }
    //重新启动算法
}

int main(int argc, char * argv[])
{
    sint32 s32Ret = 0;
    if (SIG_ERR == signal(SIGINT, exit_handle))
    {
        printf("catch signal SIGKILL Error: %d, %s\n", errno, strerror(errno));
    }

    system("touch /root/upgradefile/UpdateStatus");  //升级完后删掉，否则下次还进入升级模式

    upgrade_status = upgrade_status_init;

    fp_status = fopen("/root/upgradefile/UpdateStatus","w+");

    sint8 *ps8Port = "/dev/ttyS4";
    s32Ret = serial_init(ps8Port, 115200,0);

    pthread_attr_t 	attr;
    uint32 u32Tid_uartRead = 0;
    uint32 u32Tid_v4l2 = 0;
    uint32 u32Tid_save = 0;
    uint32 u32Tid_detect = 0;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);       //设置为分离线程
    s32Ret = pthread_create(&u32Tid_uartRead, &attr, uart_recieve,NULL); //串口接收

    
    s32Ret = mpp_vpss_InitAlgChn();
    if(s32Ret != SV_SUCCESS)
    {
        printf("video node open failed\n");
    }

    s32Ret = pthread_create(&u32Tid_v4l2, &attr, mpp_vpss_GetAlgFrame,NULL);//v4l2抓图
    s32Ret = pthread_create(&u32Tid_save, &attr, mpp_vpss_file_save,NULL);//把v4l2抓到的数据保存到文件中
    s32Ret = pthread_create(&u32Tid_detect, &attr, mpp_vpss_file_detect,NULL);//每30ms发一段，150ms没有数据认为发送结束

    while(1)
    {
         sleep_ms(100);
        if(upgrade_status == upgrade_status_filecompleted)
        {
            upgrade_status = upgrade_status_update;
            test_upgrade();
        }
    }

    //test_upgrade();
    //system("pkill -9 upgrade");

    return 0;
}




uint32_t serial_init(const sint8* port, uint32_t baud,uint8_t CH)
{
    uint32_t s32FdUpdate;
    struct termios stAttr;
    uint32_t s32Ret = -1;
    s32FdUpdate = open(port, O_RDWR|O_NOCTTY|O_NDELAY);
    if (s32FdUpdate < 0)
    {        
        printf("serial_init: %s Open failed\n", port);
        close(s32FdUpdate);
        return SV_FALSE;
    }
    printf("serial_init: %s Open successful\n", port);
    tcgetattr(s32FdUpdate, &stAttr);
    bzero(&stAttr, sizeof(stAttr));

    switch (baud) {
        case 9600:
            cfsetispeed(&stAttr, B9600);
            cfsetospeed(&stAttr, B9600);
            break;
        case 19200:
            cfsetispeed(&stAttr, B19200);
            cfsetospeed(&stAttr, B19200);
            break;
        case 38400:
            cfsetispeed(&stAttr, B38400);
            cfsetospeed(&stAttr, B38400);
            break;
        case 115200:
            cfsetispeed(&stAttr, B115200);
            cfsetospeed(&stAttr, B115200);
            break;
        default:
            fprintf(stderr, "Warning: Baudrate not supported!\n");
            close(s32FdUpdate);
            return -1;
    }

    stAttr.c_cflag |= CLOCAL | CREAD;
    stAttr.c_cflag &= ~CSIZE;
    stAttr.c_cflag |= CS8;
    stAttr.c_cflag &= ~PARENB;
    stAttr.c_cflag &= ~CSTOPB;
    stAttr.c_cflag &= ~CRTSCTS;
    stAttr.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    stAttr.c_oflag &= ~OPOST;
    s32Ret = tcflush(s32FdUpdate, TCIFLUSH);
    if (s32Ret < 0)
    {
        printf("tcflush failed. [err=%#x]\n", errno);
    }
    stAttr.c_cc[VTIME] = 255;
    stAttr.c_cc[VMIN] = 0;
    s32Ret = tcsetattr(s32FdUpdate, TCSANOW, &stAttr);
    if (s32Ret < 0)
    {
        printf("tcsetattr failed. [err=%#x]\n", errno);
    }
    s32SerialFd[CH] = s32FdUpdate;
  
    /* TCIFLUSH: flushes data received but not read.
     * TCOFLUSH: flushes data written but not transmitted
     * TCIOFLUSH: flushes both data received but not read, and data written but not transmitted
     */
    tcflush(s32SerialFd[CH], TCIOFLUSH);  /* 清空串口1的读写缓冲 */

    printf("serial init success. FD=%d\n", s32SerialFd[CH]);
    return SV_TRUE;
}

sint32 mpp_vpss_InitAlgChn()
{
    sint32 s32Ret = 0;
    uint32 u32BufNum = 0, ret;
    char path[64] = {0};
    //打开设备
    int path_id = 0;
    int fd = 0;
    struct v4l2_capability vcap;

    remove("./testfile");
    fp = fopen("./testfile", "wb+");  

while(1)
{   
    if(path_id > 50)
    {
       return SV_FAILURE;
    }

    sprintf(path, "/dev/video%d",path_id);
    printf("open video node :%s ",path);
    path_id ++;
    fd = open(path, O_RDWR);
    if (fd < 0)
    {
        //printf("open device fail\n");
        continue;
    }
    else
    {
        struct v4l2_capability vcap;
        ioctl(fd, VIDIOC_QUERYCAP, &vcap);
        if (!(V4L2_CAP_VIDEO_CAPTURE & vcap.capabilities)) 
        {
           // printf("vcap.capabilities = %d", vcap.capabilities);
           // print_level(SV_ERROR,"Error: No capture video device!\n");
            continue;
        }
        break;
    }
}  

    //获取摄像头支持的格式
    struct v4l2_fmtdesc v4fmt;
    v4fmt.index = 0;
    v4fmt.type  = V4L2_BUF_TYPE_VIDEO_CAPTURE;
     printf("support format list:\n");
    while (ioctl(fd,VIDIOC_ENUM_FMT, &v4fmt) == 0)
    {
         printf("v4l2_format%d:%s\n",v4fmt.index, v4fmt.description);
        v4fmt.index++;
    }
    
    struct v4l2_frmsizeenum frmsize;
    frmsize.index = 0;
    frmsize.type  = V4L2_BUF_TYPE_VIDEO_CAPTURE;
     printf("support resolution list:\n");
    frmsize.pixel_format = V4L2_PIX_FMT_H264;
    while(ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) == 0)
    {
         printf("frame_size<%d*%d>\n", frmsize.discrete.width, frmsize.discrete.height);
        frmsize.index++;
    }
    
    //设置摄像头支持的格式
    struct v4l2_format vFormat;
    vFormat.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    vFormat.fmt.pix.width       = UVC_CAM_WIDTH;
    vFormat.fmt.pix.height      = UVC_CAM_HEIGHT;
    //vFormat.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
    vFormat.fmt.pix.pixelformat  = V4L2_PIX_FMT_H264;
    ret = ioctl(fd, VIDIOC_S_FMT, &vFormat);
    if(ret < 0)
    {
        printf("set format fail\n");
        return SV_FAILURE;
    }
    
    //申请内核空间
    struct v4l2_requestbuffers vqbuff;
    vqbuff.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    vqbuff.count  = UVC_BUF_NUM;
    vqbuff.memory = V4L2_MEMORY_MMAP;
    ret = ioctl(fd, VIDIOC_REQBUFS, &vqbuff);
    if (ret<0)
    {
        printf("requrey buf fail\n");
        return SV_FAILURE;
    }

    //申请内存空间
    struct v4l2_buffer vbuff;
    for(int i=0; i<UVC_BUF_NUM; i++)
    {
        memset (&vbuff, 0, sizeof (struct v4l2_buffer));
        vbuff.index  = i;        
        vbuff.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        vbuff.memory = V4L2_MEMORY_MMAP;
        ret=ioctl(fd, VIDIOC_QUERYBUF, &vbuff);
        if (ret<0)
        {
            printf("requrey memory fail\n");
            continue;
        }
        
        m_stVpssAlgFrame.mptr[i]= (unsigned char *)mmap(NULL, vbuff.length, PROT_READ, MAP_SHARED, fd, vbuff.m.offset);
        if (m_stVpssAlgFrame.mptr[i] == MAP_FAILED)
        {
            printf("mmap failed. i=%d\n", i);
            continue;
        }

        u32BufNum++;
    }

    if (u32BufNum <= 0)
    {
        printf("v4l2 mmap failed.\n");
        return SV_FAILURE;
    }
    
    for(int i=0; i<u32BufNum; i++)
    {
        vbuff.index  = i;        
        vbuff.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        vbuff.memory = V4L2_MEMORY_MMAP; //缓冲帧放入缓冲队列
        ret=ioctl(fd, VIDIOC_QBUF, &vbuff);
        if(ret<0)
        {
            printf("put fail\n");
            return SV_FAILURE;
        }
    }

    // 开始采集
    enum v4l2_buf_type bufType;
    bufType = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ret = ioctl(fd, VIDIOC_STREAMON, &bufType);
    if(ret<0)
    {
        printf("open fail\n");
        return SV_FAILURE;
    }

    m_stVpssAlgFrame.s32AlgChnFd = fd;

    return SV_SUCCESS;
}


sint32 mpp_vpss_GetAlgFrame()
{
    sint32 s32Ret = 0 ,ret = 0;
    void *pbmp = NULL;
    sint32 s32Fd = m_stVpssAlgFrame.s32AlgChnFd;
    sint32 s32BufIndex = 0;
    int counter = 0;

    uint32 u32BufNum = 0;
    uint32 u32DstSize = 0; 

    static struct timespec tvNow = {0, 0};
    static struct timespec tvLast = {0, 0};
    clock_gettime(CLOCK_MONOTONIC, &tvNow);

    uint32_t  u32StepTimeMs = 0;
    uint32_t  u32cnt = 0;
    uint32_t  u32sum = 0;
    uint32_t  file_data_cnt = 0;

    while(1)   
    {

    //     clock_gettime(CLOCK_MONOTONIC, &tvNow);
    //     u32StepTimeMs = ((tvNow.tv_sec*1000 + tvNow.tv_nsec/1000000) - (tvLast.tv_sec*1000 + tvLast.tv_nsec/1000000));
    //     tvLast = tvNow;
    //    // printf("****delay ms__get pic:%d ***** \n",u32StepTimeMs);

    //     u32sum += u32StepTimeMs;
    //     u32cnt ++;
    //     if(u32cnt >10) //100*50ms = 5s
    //     {

    //        printf("****delay ms__get pic:%d ***** \n",u32sum / u32cnt );
    //        u32cnt =  0;
    //        u32sum =  0;
    //     }

       if (s32Fd <= 0)
       {
        printf("uvc fd is invalid! fd: %d\n", s32Fd);
        return SV_FAILURE;
       }
       
       struct pollfd tFds[1];
       /* poll */
       tFds[0].fd     = s32Fd;
       tFds[0].events = POLLIN;
       s32Ret = poll(tFds, 1, -1);
       if (s32Ret <= 0)
       {
           printf("poll error!\n");
           return SV_FAILURE;
       }
       
       //从队列中取数据
       struct v4l2_buffer readbuff;
       memset(&readbuff, 0, sizeof(struct v4l2_buffer));
       readbuff.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
       readbuff.memory = V4L2_MEMORY_MMAP;
       s32Ret = ioctl(s32Fd, VIDIOC_DQBUF, &readbuff);
       if(s32Ret < 0)
       {
           printf("read fail!\n");
           return SV_FAILURE;
       }
       s32BufIndex = readbuff.index; //12帧缓存

       int index = counter % RINGBUF_VDEC_COUNT; 

       memcpy(MptrData[index],m_stVpssAlgFrame.mptr[s32BufIndex], RINGBUF_VDEC_SIZE);



       if(file_data_cnt == 0)
       {
        for( uint8_t i = 0 ; i<8 ; i++ )
        rest_file_size += ( ((unsigned long long)MptrData[index][i+14]) << (i*8) ) ;
       }

       printf("file data in :%d ,%d\n", file_data_cnt , rest_file_size );

       file_data_cnt ++;

       file_count = 1; //不清时一段时间后认为文件需要保存

       int rel = ringque_VDEC.Write(index, [](int &release){ 
                    return ++release; 
                });
       counter ++;

    
       s32Ret = ioctl(s32Fd, VIDIOC_QBUF, &readbuff);
       if (s32Ret < 0)
       {
           printf("put fail!\n");
           return SV_FAILURE;
       }    

    }

    return SV_SUCCESS;
}


sint32 mpp_vpss_file_save()
{

    int32_t timeout = 500000; 

    uint32 file_save_cnt = 0;

    upgrade_status = upgrade_status_transfer;
    while(1)
    {
    int chn = 0;
    int index = ringque_VDEC.Read(chn, timeout, [/*UtilizeResource*/&ringque_VDEC](int &current){
            //cout << "    --- reader" << " --- " << GetMtimestamp() << " rpos:" << ringque.rpos_[0] << " index:" << current << " value:" << *ringbuf[current] << endl;
            //printf("    --- reader --- %llu, rpos: %d, index: %d, value: %#x\n", GetMtimestamp(), ringque.rpos_[0], current, (uint32_t)(*ringbuf[current]));
            // usleep(10000); // 10ms
             //sleep_ms(10);
            return current; //0;
        });   

    if(rest_file_size  >= (RINGBUF_VDEC_SIZE-48) ) 
    {
        fwrite(MptrData[index]+48, RINGBUF_VDEC_SIZE-48, 1 , fp);  //去掉264的头
        rest_file_size -=  (RINGBUF_VDEC_SIZE-48);
    }
    else
    {
        fwrite(MptrData[index]+48, rest_file_size, 1 , fp);  //去掉264的头
    }

    //fwrite(MptrData[index]+48, RINGBUF_VDEC_SIZE-48, 1 , fp);  //去掉264的头

   //fwrite(MptrData[index], RINGBUF_VDEC_SIZE, 1 , fp);
    //fseek(fp, RINGBUF_VDEC_SIZE, SEEK_CUR);
    }
}


void mpp_vpss_file_detect()
{
    while(1)
    {
        sleep_ms(10);  //每30ms发一段，150ms没有数据认为发送结束
    if(file_count != 0)
    {
        file_count ++;
        if(file_count > 15)
        {
            fclose(fp);
            printf("file saved success\n");
            //****test*****//
            //system("pkill -9 upgrade");
            upgrade_status = upgrade_status_filecompleted;
            pthread_exit(NULL);
        }
    }

    }
}

void uart_recieve()
{
    sint32 s32Ret = 0;
    fd_set rfds;
    int rxlen = 0;
    unsigned char buf[255] = {0};

    while(1)
    {
    FD_ZERO(&rfds);
    FD_SET(s32SerialFd[0], &rfds);
    s32Ret = select(s32SerialFd[0] +1,&rfds, NULL, NULL, NULL); //block mode

    if(s32Ret > 0)
       {
        if(FD_ISSET(s32SerialFd[0],&rfds))
            {
               rxlen = read(s32SerialFd[0] , buf, 255);
               if(rxlen > 0)
               {
                    //接收松翰发过来的md5

                    //与本地md5做比较
                    // if(/*一致*/)
                    // {
                    //     //进入算法
                    // }
                    // else
                    // {
                    //     //开始接收文件
                    // }
               }
           }
       }
    }

}

int SAFE_System_Recv(char* cmdStr,char *recv, int lens)
{
    int fd, rlens;

    if(cmdStr == NULL || recv == NULL || lens <= 0)
    {
        return -1;
    }

#if defined(BOARD_ADA32V3)
    fd = popen2(cmdStr, "r");
    if (fd == NULL)
    {
        //printf("popen idVendor file failed. [err=%#x]\n", errno);
        return -1;
    }

    rlens = fread(recv, sizeof(char), lens-1,  fd);
    if (rlens <= 0)
    {
        if(feof(fd))
        {
            pclose2(fd);
            return 0;
        }
        //printf("pread idVendor file failed. [err=%#x]\n", errno);
        pclose2(fd);
        return -1;
    }
    pclose2(fd);
#else
	fd = popen(cmdStr, "r");
    if (fd == NULL)
    {
        //printf("popen idVendor file failed. [err=%#x]\n", errno);
        return -1;
    }

    rlens = fread(recv, sizeof(char), lens-1,  fd);
    if (rlens <= 0)
    {
        if(feof(fd))
        {
            pclose(fd);
            return 0;
        }
        //printf("pread idVendor file failed. [err=%#x]\n", errno);
        pclose(fd);
        return -1;
    }
    pclose(fd);
#endif

    recv[rlens] = '\0';
    return 0;
}