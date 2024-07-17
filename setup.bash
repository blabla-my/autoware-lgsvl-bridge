ROS_INSTALLATION_PATH="${ROS_INSTALLATION_PATH:=/opt/ros/melodic}"
AUTOWARE_INSTALLATION_PATH="${AUTOWARE_INSTALLATION_PATH:=$HOME/autoware.ai}"
AUTOWARE_MESSAGE_INSTALLATION_PATH="${AUTOWARE_MESSAGE_INSTALLATION_PATH:=$HOME/autoware_ai_messages}"
SIMULATOR_PATH="${SIMULATOR_PATH:=$HOME/Build_Local}"

source $ROS_INSTALLATION_PATH/setup.zsh
source $AUTOWARE_MESSAGE_INSTALLATION_PATH/devel/setup.zsh
source $AUTOWARE_INSTALLATION_PATH/devel/setup.zsh
#source $HOME/src/repos/autoware.ai/autoware-1.14.0/install/setup.zsh
export AUTOWARE_FUZZER=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)

unalias ftest
unalias resim
alias ftest='python3 $AUTOWARE_FUZZER/tools/run_vse_test.py'
alias resim='bash $AUTOWARE_FUZZER/scripts/restart_simulator.sh'

getmap() {
	jq ".map.name" $1 | xargs echo
}

npc_length() {
	jq ".agents | length" $1

}

clean() {
	pkill "op_*_"
	rosnode kill -a 1>/dev/null 2>/dev/null
}
