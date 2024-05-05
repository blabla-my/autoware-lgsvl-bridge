simulatorpath=~/Build_Local
dir=$PWD
function clean(){
	pids=`ps -ef | grep $1 | awk '{print $2}'`
	for pid in $pids
	do
		echo $pid
        kill -9 $pid
	done
}

function restart_simulator(){
	cnt=`ps -ef | grep $simulatorpath/simulator | wc -l`	
	if [ $cnt -le 2 ];then 
		killall -9 $simulatorpath/simulator >& /dev/null
		cd $simulatorpath && $simulatorpath/run.sh &
        cd $dir
		sleep 10
	fi
}

# clean bridge
# clean planner
# rosnode kill -a
# sleep 1
# restart_simulator
python3 $AUTOWARE_FUZZER/run_vse.py $1