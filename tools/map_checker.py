import sys
import os
import time
import json
sys.path.append(os.getenv("AUTOWARE_FUZZER"))
from manager import AutowareManager, lgsvl_transform_to_autoware_pose
from geometry_msgs.msg import Pose,PoseStamped,PoseWithCovarianceStamped

class Checker:
    def __init__(self,seed):
        self.autoware_manager = AutowareManager()
        self.autoware_manager.default()
        self.seed = json.load(open(seed,'r'))
        self.map = self.seed["map"]["name"]
        self.goal_pose = lgsvl_transform_to_autoware_pose(self.seed["agents"][0]["destinationPoint"])
        self.initial_pose = lgsvl_transform_to_autoware_pose(self.seed["agents"][0]["transform"])

    def check(self):
        self.autoware_manager.set_args('map',map=self.map)
        self.autoware_manager.start("map")
        self.autoware_manager.start("tf")
        self.autoware_manager.start("op_global_planner")
        self.autoware_manager.start("op_local_planner")
        time.sleep(3)
        self.autoware_manager.sendpose('/initialpose',PoseWithCovarianceStamped,self.initial_pose)
        self.autoware_manager.sendpose('/move_base_simple/goal',PoseStamped,self.goal_pose)
        try:
            self.autoware_manager.wait_rollouts()
            print("Success.")
            return True
        except Exception as e:
            if "Planner Timeout" in str(e):
                print("Fail.")
                return False
        return False

    def clean(self):
        self.autoware_manager.clean()

def _main():
    dir=os.getenv("PWD")
    seedlist=open(sys.argv[1],'r').readlines()
    failed = open("failed.txt",'w')
    for seed in seedlist:
        seed = seed.strip('\n')
        checker = Checker(os.path.join(dir,seed,"corpus",seed+'.json'))
        if checker.check() == False:
            # failed.write(seed+'\n')
            os.system("echo {} >> failed.txt".format(seed))
        checker.clean()
    failed.close()

def main():
    checker = Checker(sys.argv[1])
    res = checker.check()
    print(res)

if __name__=="__main__":
    main()
    