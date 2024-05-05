function stop_bridge(){
	pids=`ps -ef | grep bridge | awk '{print $2}'`
	for pid in $pids
	do
		echo $pid
        kill -9 $pid
	done
}

stop_bridge