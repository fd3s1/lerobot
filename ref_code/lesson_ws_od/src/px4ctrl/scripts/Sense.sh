#!/bin/bash
if [ -z $1 ];
 then echo "请指定操作:（start/stop)"
else 
 case $1 in 
	"start")
		pid=$(ps -ef |grep "python3 Sense.py" |grep -v grep |awk '{print $2}')
		if [ -z $pid ];
		 then 
	     	  echo "---------------------------启动感知----------------------------"
	          nohup echo "qazere1" |sudo -S python3 Sense.py  >/dev/null 2>&1 &
                else
		 echo "感知程序已经启动，无需重复操作"	
		fi
	;;
	"stop")
		pid=$(ps -ef |grep "python3 Sense.py" |grep -v grep |awk '{print $2}')
		if [ -z $pid ];
	         then echo "感知程序已经停止，无需重复操作"
	        else
		 echo "---------------------------停止感知----------------------------"	
		 echo "qazere1" |sudo -S kill $pid
		fi
	;;
	*)
		echo "请重新输入参数：（start/stop)"
	;;
 esac
fi
