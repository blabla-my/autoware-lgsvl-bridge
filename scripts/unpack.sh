#!/bin/bash

dir=$PWD
maplist=`ls -d */`
apollomapdir=$HOME/apollo/modules/map/data

cd $apollomapdir

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

for scenario_name in $maplist 
do
	map_dir=$dir/$scenario_name
	zipf=$map_dir/environ*
	map_name=`getMap $map_dir/*.json`

	if [ ! -d $map_name ]; then
		mkdir $map_name
	else
		echo dir exist
	fi

	unzip -o -j $zipf hdmaps/apollo50/base_map.bin -d $map_name	
	unzip -o -j $zipf hdmaps/opendrive/*.xodr -d $map_dir/lane
	mkdir -p $map_dir/corpus $map_dir/${scenario_name:0:-1}_out

	if [ -f $map_dir/*.json ]; then
		cp $map_dir/*.json $map_dir/corpus
	fi
done
