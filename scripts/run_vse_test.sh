seed=$1
vse_runner=$AUTOWARE_FUZZER/tools/run_vse_test.py
wise=$AUTOWARE_FUZZER/tools/wise.py

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

# python3 $wise vm4 restart
python3 $vse_runner $@