import rospy
import tf
import json
import sys
import os
import time
from scipy.spatial.transform import Rotation
from lgsvl_msgs.msg import Detection3D, Detection3DArray
from geometry_msgs.msg import PoseStamped, Point32, PolygonStamped, TwistStamped
from autoware_msgs.msg import Lane

class PlanningRecorder:

    def __init__(self):
        rospy.init_node("planning_recorder")
        self.NPC_dicts = dict()
        self.sub = rospy.Subscriber("/final_waypoints", Lane, self.callback)
        self.planning_traj = []
    

    def callback(self,lane):
        _time = time.time()
        traj = dict(time=_time, waypoints=[])
        for wp in lane.waypoints[:10]:
            json_pose = self.sensor_to_json_pose(wp.pose.pose, time_cost=wp.time_cost, to_map=False)
            traj["waypoints"].append(json_pose)
        self.planning_traj.append(traj)

    def save(self,outpath):
        f = open(outpath,'w')
        json.dump(self.planning_traj,f)
        f.close()

    def sensor_to_json_pose(self,pose,time_cost,to_map=True):
        json_pose = dict(position=dict(),rotation=dict())
        if to_map == True:
            pass
            # /base_link --> /map
            # posestamped = PoseStamped()
            # posestamped.header.frame_id = '/base_link'
            # posestamped.pose = pose
            # new_pose = self.transform_to_new_frame(posestamped,'/map')
            # if new_pose == None:
            #     return None
            # pose_in_map = new_pose.pose
        else:
            pose_in_map = pose
        # Quat --> Euler
        quat = [
            pose_in_map.orientation.x,
            pose_in_map.orientation.y,
            pose_in_map.orientation.z,
            pose_in_map.orientation.w
        ]
        yaw,pitch,roll = Rotation.from_quat(quat).as_euler('zxy',degrees=True)
        # transform to json pose
        json_pose
        json_pose['position']['x'] = pose_in_map.position.x
        json_pose['position']['y'] = pose_in_map.position.z
        json_pose['position']['z'] = pose_in_map.position.y
        json_pose['rotation']['x'] = pitch
        json_pose['rotation']['y'] = 90 - yaw
        json_pose['rotation']['z'] = -roll
        
        # if using lgsvl original case, uncomment this part
        #json_pose['position']['x'] = - pose_in_map.position.y
        #json_pose['position']['y'] = pose_in_map.position.z
        #json_pose['position']['z'] = pose_in_map.position.x
        #json_pose['rotation']['y'] = -yaw
        json_pose["time_cost"] = time_cost
        return json_pose

if __name__=='__main__':
    planningRecorder = PlanningRecorder()
    outpath = sys.argv[1]
    while True:
        cmd = sys.stdin.read().decode()
        if "Shut down" in cmd:
            planningRecorder.save(outpath)
            print("[+] planning trajectory saved to "+outpath)
            break
        else:
            print(cmd)
