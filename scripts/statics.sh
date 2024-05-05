#!/bin/bash

dir1=~/src/repos/DONE
dir2=~/src/repos/DONE2
#dir2=~/back/rerun/DONE2
# maplist=`ls -d */`

function calrate(){
    python3 - "$@" <<END
import sys
col = float(sys.argv[1])
queue = float(sys.argv[2])
totaltime = 3*3600
meantime = "%.1f" % (totaltime/col) if col > 0 else "-"
print(meantime)
END
}

function printdata(){
    scenario_name=$1
    queue=$2
    col=$3
    #rate=`calrate $col $queue`
    if [ "$queue" = "17" ] || [ "$queue" = "33" ] ;then
        queue=0
        col=0
    fi
    echo $scenario_name
    echo "#### of Seed Queue "$queue
    echo "#### of Collisions "$col
    #echo "#### rate "$rate
    #echo
    # echo $rate
    echo "$scenario_name,$col,$queue" >> statics.csv
}

function non_fp_collision(){
    seed=$1
    non_fp_cnt=0
    non_fp_cnt=`find $dir/$seed/unique_collisions -type f -not -path "*insufficient_time*" -not -path "*destination_reached*" | wc -l`
    echo $non_fp_cnt
}

function fp_1(){
    seed=$1
    fp_cnt=0
    fp_cnt=`find $dir/$seed/unique_collisions/insufficient_time -type f | wc -l`
    echo $fp_cnt
}

function fp_2(){
    seed=$1
    fp_cnt=0
    fp_cnt=`find $dir/$seed/unique_collisions/destination_reached -type f | wc -l`
    echo $fp_cnt
}

maplist=`cat $1`
a=0
b=0
c=0
d=0

col_cnt=0
queue_cnt=0
col_250_total=0
wp_collision_cnt=0
tmin_collision_cnt=0
non_fp_cnt=0
obj_cnt=0
fp1_cnt=0
fp2_cnt=0
rm -f seeds_not_done.txt
rm -r seeds_done.txt
rm -f statics.csv
for scenario_dir in $maplist
do
    let c++
    scenario_name=$scenario_dir
    #echo $scenario_name
    if [ -d $dir1/$scenario_dir ] || [ -d $dir2/$scenario_dir ]; then
        if [ -d $dir1/$scenario_dir/out/queue ];then
            dir=$dir1
        else
            dir=$dir2
        fi
        if [ ! -f $dir/$scenario_name/fuzzer.log ];then
            continue
        fi
        queue=`ls -l $dir/$scenario_dir/queue_250/ | grep "^-" | wc -l`
        col=`ls -l $dir/$scenario_dir/collision_250/ | grep "^-" | wc -l`
        # printdata $scenario_name $queue $col
        if [ "$queue" = "0" ] || [ "$queue" = "17" ] || [ "$queue" = "33" ];then
            let b++
            echo $scenario_name >> seeds_not_done.txt
            echo 0
            #continue
        fi
        if [ -d $dir/$scenario_dir/wp_collision ];then
            wp_add_cnt=`ls $dir/$scenario_dir/wp_collision | wc -l`
            tmin_add_cnt=`ls $dir/$scenario_dir/tmin_collision | wc -l`
            obj_add_cnt=`ls $dir/$scenario_dir/collision_data | wc -l`
            let wp_collision_cnt=wp_collision_cnt+wp_add_cnt
            let tmin_collision_cnt=tmin_collision_cnt+tmin_add_cnt
            let obj_cnt=obj_cnt+obj_add_cnt

        fi
        
        printdata $scenario_name $queue $col
        add_col_250_cnt=`ls $dir/$scenario_name/collision_250 | wc -l`
        let col_250_cnt=col_250_cnt+add_col_250_cnt
        # echo $scenario_name
        # echo $col_250_cnt
        # echo $col
        let a++
        if [ $queue -ge 250 ]; then
            let d++
        fi
        #let non_fp_cnt=non_fp_cnt+`non_fp_collision $scenario_name`
        #let fp1_cnt=fp1_cnt+`fp_1 $scenario_name`
        #let fp2_cnt=fp2_cnt+`fp_2 $scenario_name`
        let col_cnt=col_cnt+col
        let queue_cnt=queue_cnt+queue
        echo $scenario_name >> seeds_done.txt
    else
         echo 0
    fi   
done

#echo "seeds done: $a/$c"
#echo "seeds not done: $b/$c"
#echo "seeds queue over 200: $d/$c"
echo "of total collision: $col_cnt"
echo "of total queue: $queue_cnt"
#echo "total: $c"
echo "raw collision: $col_cnt"
echo "minimized collisions: $obj_cnt"
let unstable=$col_cnt-$obj_cnt
echo "unstable collisions: $unstable"
echo "non-fp collisions: $non_fp_cnt"
echo "destination reached: $fp2_cnt"
echo "insufficient time: $fp1_cnt"
#echo "done wp collision: $wp_collision_cnt"
#echo "done tmin collision: $tmin_collision_cnt"
#echo "saved obj count: $obj_cnt"