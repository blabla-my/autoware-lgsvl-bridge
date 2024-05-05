simulatorpath=$SIMULATOR_PATH
# function restart_simulator(){
# 	pkill -9 simulator
# 	killall -9 simulator
# 	sleep 1
# 	pushd $simulatorpath
# 	DISPLAY=:10.0 $simulatorpath/run.sh &
# 	popd
# 	sleep 10
# }

# function restart_simulator(){
# 	cnt=`ps -ef | grep $simulatorpath/simulator | wc -l`
# 	if [ $cnt -le 2 ];then
# 		killall -9 $simulatorpath/simulator >& /dev/null
# 		pushd $simulatorpath
#         DISPLAY=:10.0 $simulatorpath/run.sh &
#         popd
# 		sleep 10
# 	fi
# }

function restart_simulator() {
	ps -ef | grep $simulator/simulator | awk '{print $2}' | while read pid; do
		kill -9 $pid
	done
	pushd $simulatorpath
	DISPLAY=:10.0 $simulatorpath/run.sh &
	popd
	sleep 10
}

restart_simulator
