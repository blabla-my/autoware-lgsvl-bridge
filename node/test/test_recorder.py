import subprocess
from time import sleep
p = subprocess.Popen(["python","/home/vm4/src/repos/autoware_fuzzing/node/wp_recorder.py"], stdin=subprocess.PIPE)
sleep(5)
p.communicate(input='Shut down'.encode())
p.wait()