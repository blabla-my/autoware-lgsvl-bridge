function clean(){
	pids=`ps -ef | grep $1 | awk '{print $2}'`
	for pid in $pids
	do
		echo $pid
        kill -9 $pid
	done
}

clean $1