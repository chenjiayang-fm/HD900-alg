#!/bin/bash
directory="/root/upgradefile/"

# 定义一个数组files
#files=()
#results=()



#在升级的目录中找文件
for file in $(find $directory -type f); do
    #files+=("$file")
    #files+=("${file##*/}")
    upgradefile=("${file##*/}")    #待升级的文件
    echo "search file :${upgradefile}"
    for result in $(find / -name "${upgradefile}"); do
        completefile=$directory$upgradefile
	if [[ $result =~ "/root/upgradefile/" ]]; then
            echo "MISS UPGRADE PATH"
	elif [[ $result =~ "/mnt/" ]]; then
	    echo "MISS MNT PATH"
	else
   	    echo "$result Replace"
	    cp "${completefile}" "${result%${upgradefile}*}"
	    
            #rm "${completefile}" #升级完后删掉避免后续占内存
	    #echo "rm ${completefile}"
	    #echo "${completefile}"#更新文件的完整路径
	    #echo "${result%${upgradefile}*}"#把要替换的路径提取出来
	fi
	rm "${completefile}"
    done	
done


#cp ./* ../
#flash_erase /dev/mtd1 0 0 && nandwrite -p /dev/mtd1 ./kernel_adas_2.2.img
#命令的执行结果可以用反引号（`）或美元符号和圆括号（$()）来捕获，然后赋值给变量或作为其他命令的参数
#result=$(find -name "alg") # 使用system()函数调用命令"ls -l"并将结果保存到变量$result中
#echo $result
