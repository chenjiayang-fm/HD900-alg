#!/bin/bash
 
#该函数检查关键进程的状态，最终返回进程的个数。111
checkprocess()
{
#       echo "checkprocess...$1"
        if [ "$1" = "" ];
        then
                return 1
        fi
        process_num=`ps -ef|grep "$1" |grep -v "grep" |wc -l`
#       echo "check$1 num=$process_num"
        return $process_num
}
#进程如有退出，则记录相关的信息，并做相关的处理
recordinfo()
{
#       echo "check $1"
        if [ "$1" = "" ];
        then
                return
        fi
#打印进程退出的时间
        date "+++++++++++++++++++ +%Y-%m-%d %H:%M:%S '$1' exit! +++++++++++++++++++"
#重新拉起这个进程
	sleep 2
        "$1" &
        echo "restart $1"
}
 
#检查进程的状态
check()
{
#        echo "####check $1"
        if [ "$1" = "" ];
        then
                return
        fi
        checkprocess "$1"
        check_result=$?
#如果该进程的个数为0，则说明该进程已经退出
        if [ $check_result -eq 0 ];
        then
                recordinfo "$1"
#               exit
        fi
}
#循环检测
./root/alg &
sleep 3
pkill -9 alg
while [ 1 ] ; do
        #关键进程列表，多个进程以逗号分隔，这里以两个进程为例子。
        process_name="/root/alg"
        OLD_IFS="$IFS"
        IFS=","
        arr=($process_name)
        IFS="$OLD_IFS"
 
        for s in ${arr[@]}
        do
          check "$s"
        done
        sleep 2
#       echo "check loop..."
done
