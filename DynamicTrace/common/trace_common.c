#include "trace_common.h"


#include <unistd.h>
#include <sys/time.h>
/*
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#ifndef __HuaweiLite__ 
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/msg.h>
#include <error.h>
#endif
#include <sys/select.h>
#include <sys/time.h>
#include <sys/prctl.h>
#include <pthread.h>
#include <errno.h>

#include "print.h"
#include "common.h"

#include "hi_common.h"
#include "mpi_audio.h"

#include "media.h"
*/

//阻塞式休眠线程，输入秒、微秒
void select_sleep(int sec, int usec)
{
    struct timeval timeout;
    timeout.tv_sec = sec;
    timeout.tv_usec = usec;
    select(0, NULL, NULL, NULL, &timeout); //阻塞
}


//获取系统微秒时间
long long int microtime(){
	struct timeval time;
	gettimeofday(&time, NULL); //This actually returns a struct that has microsecond precision.
	long long int microsec = ((long long)time.tv_sec * 1000000) + time.tv_usec;
	return microsec;
}



