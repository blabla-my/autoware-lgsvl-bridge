import os
import queue
import signal
import subprocess
import math
import psutil
import rospy
import json
from time import sleep
from click import launch
from numpy import binary_repr
from geometry_msgs.msg import Pose,PoseStamped
from scipy.spatial.transform import Rotation
from autoware_msgs.msg import VehicleCmd,LaneArray,Lane

def lgsvl_transform_to_autoware_pose(transform):
    '''
        convert lgsvl transform (in dict) to autoware pose
    '''
    pose = Pose()
    pose.position.x = transform['position']['x']
    pose.position.y = transform['position']['z']
    pose.position.z = transform['position']['y']

    yaw = -transform['rotation']['y'] + 90
    pitch = transform['rotation']['x']
    roll = -transform['rotation']['z']
    
    # if using lgsvl original case, uncomment this
    # pose.position.y = -transform["position"]["x"]
    # pose.position.x =  transform["position"]["z"]
    # pose.position.z =  transform["position"]["x"]
    # yaw = - transform['rotation']['y'] 

    orientation = Rotation.from_euler('zxy',[yaw,pitch,roll],degrees=True).as_quat()
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]

    return pose

    

def rotate_90(pose):
    position = pose.position
    orientation = pose.orientation
    
    x = -position.y
    y = position.x
    z = position.z
    position.x = x
    position.y = y
    position.z = z

    w = float(orientation.w)
    x = float(orientation.x)
    y = float(orientation.y)
    z = float(orientation.z)

    r = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
    p = math.asin(2*(w*y-z*x))
    y = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))

    roll = r
    pitch = p
    yaw = y + 3.0*math.pi/4
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    w = cy * cp * cr + sy * sp * sr 
    x = cy * cp * sr - sy * sp * cr 
    y = sy * cp * sr + cy * sp * cr 
    z = sy * cp * cr - cy * sp * sr 
    orientation.w  = w
    orientation.x = x
    orientation.y = y
    orientation.z = z
    pose.position = position
    pose.orientation = orientation
    return pose


class Module(object):
    launchdir = os.path.join(os.path.dirname(__file__),'../','launch')
    nodedir = os.path.join(os.path.dirname(__file__),'../','node')

    def __init__(self,name,path,**args):
        self.name = name
        self.path = path
        self.process = None
        self.args = args

    def start(self):
        print(f"[+] starting {self.name}")
        if self.path.endswith('.py'):
            arguments = ["{}:={}".format(name,value) for name,value in self.args.items()]
            self.process = subprocess.Popen(['python',self.path,*arguments],stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        elif self.path.endswith('.launch'):
            arguments = ["{}:={}".format(name,value) for name,value in self.args.items()]
            self.process = subprocess.Popen(['roslaunch', self.path, *arguments],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        sleep(0.5)

    def stop(self):
        if self.process != None:
            print(f"[+] terminating {self.name}")
            try:
                for child in psutil.Process(self.process.pid).children(recursive=True):
                    child.kill()
                self.process.kill()
                self.process.wait()
                self.process = None
            except Exception as e:
                rospy.logerr("%s %s",str(e))
                raise
    
    def running(self):
        return self.process!=None and self.process.poll()==None

    def restart(self):
        self.stop()
        sleep(1.5) # this cannot be deleted !
        self.start()


class AutowareManager(object):
    def __init__(self):
        self.module = dict()
        self.publisher = dict()
        self.rollouts_ready = False  
        self.final_waypoints_ready = False
        self.sensors = None
        rospy.init_node(name = "autoware_manager")
        self.rollouts_subscriber = rospy.Subscriber("/local_trajectories", LaneArray, self.rollouts_callback)
        self.final_waypoints_sub = rospy.Subscriber("/final_waypoints", Lane, self.final_waypoints_callback)
        

    def bridge(self):
        if 'bridge' in self.module:
            if not self.module['bridge'].running():
                self.module['bridge'].start()

    def stop_bridge(self):
        if 'bridge' in self.module:
            self.module['bridge'].stop()

    def register(self,name,path,**args):
        '''
            register module to manager
        '''
        if name not in self.module:
            self.module[name] = Module(name,path,**args)

    def unregister(self,name):
        if name in self.module:
            mod = self.module.pop(name)
            mod.stop()

    def set_args(self,name,**args):
        if name in self.module:
            self.module[name].args = args

    def start(self,name):
        if name in self.module:
            self.module[name].start()
    
    def stop(self,name):
        if name in self.module:
            self.module[name].stop()

    def restart(self,name):
        if name in self.module:
            self.module[name].restart()

    def default(self):
        '''
            default configuration:
                - map.launch
                - op_planner.launch
                - waypoint_follower.launch
                - bridge.launch
                - tf.launch
                - ground Truth
                    - localization
                    - detection
        '''
        self.sensors = "367c1a0b-c5a3-4ffc-a6b2-635a762c8894"
        modules = {
            'map':os.path.join(Module.launchdir,'map.launch'),
            'op_global_planner':os.path.join(Module.launchdir,'op_global_planner.launch'),
            'op_local_planner': os.path.join(Module.launchdir,'op_local_planner.launch'),
            'waypoint_follower':os.path.join(Module.launchdir,'waypoint_follower.launch'),
            'bridge':os.path.join(Module.launchdir,'bridge.launch'),
            'tf':os.path.join(Module.launchdir,'tf.launch'),
            'localization_GT':os.path.join(Module.nodedir,'localization_GT.py'),
            'detection_GT': os.path.join(Module.nodedir,'detection_GT.py'),
        }
        for name,path in modules.items():
            self.register(name,path)
    
    def autoware_v14_perceptionGT(self):
        '''
        Autoware version 1.14.0 configuration
        - using perception Ground Truth
        - Using op_global_planner + lane_select + astar_avoid
        '''
        modules = {
            'map': os.path.join(Module.launchdir,'map.launch'),
            'tf':os.path.join(Module.launchdir,'tf.launch'),
            'bridge':os.path.join(Module.launchdir,'bridge.launch'),
            'mission_planning': os.path.join(Module.launchdir, '1.14.0', 'my_mission_planning.launch'),
            'motion_planning': os.path.join(Module.launchdir,'1.14.0', 'my_motion_planning.launch'),
            'waypoint_follower':os.path.join(Module.launchdir, 'waypoint_follower.launch'),
            'localization_GT':os.path.join(Module.nodedir,'localization_GT.py'),
            'detection_GT': os.path.join(Module.nodedir,'detection_GT.py')
        }
        for name,path in modules.items():
            self.register(name,path)

    def prediction_ground_truth(self):
        self.sensors = "367c1a0b-c5a3-4ffc-a6b2-635a762c8894"
        modules = {
            'map':os.path.join(Module.launchdir,'map.launch'),
            'op_global_planner':os.path.join(Module.launchdir,'op_global_planner.launch'),
            'op_local_planner': os.path.join(Module.launchdir,'op_local_planner_no_predictor.launch'),
            'waypoint_follower':os.path.join(Module.launchdir,'waypoint_follower.launch'),
            'bridge':os.path.join(Module.launchdir,'bridge.launch'),
            'tf':os.path.join(Module.launchdir,'tf.launch'),
            'localization_GT':os.path.join(Module.nodedir,'localization_GT.py'),
            'detection_GT': os.path.join(Module.nodedir,'detection_GT.py'),
            'prediction_GT': os.path.join(Module.nodedir,'prediction_GT.py')
        }
        for name,path in modules.items():
            self.register(name,path)

    def control_ground_truth(self):
        self.sensors = "367c1a0b-c5a3-4ffc-a6b2-635a762c8894"
        modules = {
            'map':os.path.join(Module.launchdir,'map.launch'),
            'op_global_planner':os.path.join(Module.launchdir,'op_global_planner.launch'),
            'op_local_planner': os.path.join(Module.launchdir,'op_local_planner.launch'),
            'bridge':os.path.join(Module.launchdir,'bridge.launch'),
            'tf':os.path.join(Module.launchdir,'tf.launch'),
            'detection_GT': os.path.join(Module.nodedir,'detection_GT.py'),
            'control_GT': os.path.join(Module.nodedir,'location_estimation_from_planning.py'),
            'waypoint_follower':os.path.join(Module.launchdir,'waypoint_follower_no_pure_pursuit.launch')
        }
        for name,path in modules.items():
            if name=="detection_GT":
                self.register(name,path,base_link="/base_link_real")
            else:
                self.register(name,path)

    def control_prediction_ground_truth(self):
        self.sensors = "367c1a0b-c5a3-4ffc-a6b2-635a762c8894"
        modules = {
            'map':os.path.join(Module.launchdir,'map.launch'),
            'op_global_planner':os.path.join(Module.launchdir,'op_global_planner.launch'),
            'op_local_planner': os.path.join(Module.launchdir,'op_local_planner_no_predictor.launch'),
            'bridge':os.path.join(Module.launchdir,'bridge.launch'),
            'tf':os.path.join(Module.launchdir,'tf.launch'),
            'detection_GT': os.path.join(Module.nodedir,'detection_GT.py'),
            'control_GT': os.path.join(Module.nodedir,'location_estimation_from_planning.py'),
            'waypoint_follower':os.path.join(Module.launchdir,'waypoint_follower_no_pure_pursuit.launch'),  # only for stopping the ego
            'prediction_GT': os.path.join(Module.nodedir,'prediction_GT.py')
        }
        for name,path in modules.items():
            if name=="detection_GT":
                self.register(name,path,base_link="/base_link_real")
            else:
                self.register(name,path)
            
    def detection_normal(self):
        self.sensors = "05cbb194-d095-4a0e-ae66-ff56c331ca83"
        modules = {
            'map':os.path.join(Module.launchdir,'map.launch'),
            'op_global_planner':os.path.join(Module.launchdir,'op_global_planner.launch'),
            'op_local_planner': os.path.join(Module.launchdir,'op_local_planner.launch'),
            'waypoint_follower':os.path.join(Module.launchdir,'waypoint_follower.launch'),
            'bridge':os.path.join(Module.launchdir,'bridge.launch'),
            'tf':os.path.join(Module.launchdir,'tf.launch'),
            'localization_GT':os.path.join(Module.nodedir,'localization_GT.py'),
            'detection': os.path.join(Module.launchdir,'detection.launch'),
            'points_remap': os.path.join(Module.nodedir, 'points_remap.py')
        }
        for name,path in modules.items():
            self.register(name,path)

    def start_all(self, mask=[]):
        for name,mod in self.module.items():
            if name not in mask:
                mod.restart()
    
    def stop_all(self, mask=[]):
        for name,mod in self.module.items():
            if name not in mask:
                mod.stop()

    def send_goal(self,goalpose):
        print("sending goal .....................................")
        args = ["pub", "/move_base_simple/goal","geometry_msgs/PoseStamped", str(goalpose)]
        binary = "rostopic"
        p = subprocess.Popen([binary,*args])
        self.rollouts_ready = False  
        print("Start waiting ........................")
        while True:
            if self.rollouts_ready == True:
                print("CMD got ! ............................")
                break
            else:
                sleep(1)
        for child in psutil.Process(p.pid).children(recursive=True):
            child.kill()

    def pose2string(self,pose):
        position = pose.position
        orientation = pose.orientation
        pose = json.loads("{\"pose\":{\"position\":{\"x\":0,\"y\":0,\"z\":0},\"orientation\":{\"x\":0,\"y\":0,\"z\":0,\"w\":0}}}")
        pose["pose"]["position"]["x"] = position.x
        pose["pose"]["position"]["y"] = position.y
        pose["pose"]["position"]["z"] = position.z
        pose["pose"]["orientation"]["x"] = orientation.x
        pose["pose"]["orientation"]["y"] = orientation.y
        pose["pose"]["orientation"]["z"] = orientation.z
        pose["pose"]["orientation"]["w"] = orientation.w
        return pose

    def sendpose(self,topic,Messagetype,pose):
        if topic not in self.publisher:
            self.publisher[topic] = rospy.Publisher(topic,Messagetype,queue_size=1)
        message = Messagetype()
        if topic == '/initialpose':
            message.pose.pose = pose
        else:
            message.pose = pose
        cnt = 0
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and cnt < 10:
            self.publisher[topic].publish(message)
            cnt += 1
            rate.sleep()
    
    def sendvel(self,topic,Messagetype,vel):
        if topic not in self.publisher:
            self.publisher[topic] = rospy.Publisher(topic,Messagetype,queue_size=1)
        cnt = 0
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and cnt < 10:
            self.publisher[topic].publish(vel)
            cnt += 1
            rate.sleep()
    
    def wait_rollouts(self,time_out=20):
        self.rollouts_ready = False  
        wait_cnt = 0
        while self.rollouts_ready == False:
            wait_cnt += 1
            sleep(1)
            if wait_cnt >= time_out:
                raise Exception("Planner Timeout !")

    def wait_final_waypoints(self,time_out=20):
        self.final_waypoints_ready = False
        wait_cnt = 0
        while self.final_waypoints_ready == False:
            wait_cnt += 1
            sleep(1)
            if wait_cnt >= time_out:
                raise Exception("Planner Timeout !")
                
    def rollouts_callback(self,rollouts):
        if self.rollouts_ready == False:
            if len(rollouts.lanes) > 0:
                self.rollouts_ready = True

    def final_waypoints_callback(self,lane):
        if self.final_waypoints_ready == False:
            if len(lane.waypoints) > 0:
                self.final_waypoints_ready = True

    def clean(self):
        for name,mod in self.module.items():
            mod.stop()
        self.module = dict()
        # os.system("rosnode kill -a")
        # sleep(2)

def main(): 
    manager = AutowareManager()
    manager.sendpose("/testpose",PoseStamped,Pose())
    

if __name__ == '__main__':
    main()
