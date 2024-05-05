dir=$PWD
seed=$1
wise=$AUTOWARE_FUZZER/tools/wise.py
cluster=vm4
collisions=`ls $dir/$seed/out/collision`

function getMap()
{
	python3 - "$@" <<END
#!/usr/bin/python3
import json
import sys
with open(sys.argv[1]) as f:
	data = json.load(f)
print(data["map"]["name"])
END
}

function run_vse(){
    scenario=$1
    map=`getMap $scenario`
    run_vse_py=$AUTOWARE_FUZZER/tools/run_vse_test.py
    python3 $run_vse_py $scenario $map
}

for collision in $collisions
do
    colpath=$dir/$seed/out/collision/$collision
    if [ -f $colpath ]; then
        echo "Running collision: $collision, path: $colpath"
        rosnode kill -a
        python3 $wise $cluster restart
        run_vse $colpath
    fi
done