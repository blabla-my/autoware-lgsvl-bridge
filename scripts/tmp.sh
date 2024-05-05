#!/bin/bash

dir=$PWD
dirlist=`ls -d */`

blacklist=(RUS_Bicycle-1_1_T-1 DEU_Muc-4_1_T-1)
target=(DEU_Muc-23_2_T-1)

# for scenario in $dirlist
# do
# 	seed_dir=$dir/$scenario
# 	echo $seed_dir
# 
# 	cd $seed_dir/*_out/collision_250
# done

seed_dir=$dir/$1
echo $seed_dir

cd $seed_dir/*_out/collision_250

if [ ! -d "./true_collision/" ]; then
	mkdir true_collision
	mkdir fp_collision
	mkdir not_collision
	mkdir interesting
fi

seed_list=`find . -maxdepth 1 -type f`

# killall -9 simulator >& /dev/null
# DISPLAY=:0 ~/svlsimulator-linux64-2021.2.2/simulator &
# bash -x ~/apollo-fuzzer/tools/restart_simulation.sh

for i in $seed_list; do
	cur=$i

	while true
	do
		echo $cur
		python3 ~/apollo-fuzzer/tools/fp_filter.py $cur
		pos=`find . -type f -name ${i:2}`
		printf "[-] Operation: "
		read -n 2 ans
		
		if [[ $ans = "y" ]]; then 
			echo "[-] save interesting seed"
			mv $pos ./interesting 
			break
		elif [[ $ans = "r" ]]; then
			echo "[-] replaying"
			cur=$pos
			continue
		else
			break
		fi
	done
done
