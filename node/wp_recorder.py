import rospy
import tf
import json
import sys
import os
import time
from scipy.spatial.transform import Rotation
from lgsvl_msgs.msg import Detection3D, Detection3DArray
from geometry_msgs.msg import PoseStamped, Point32, PolygonStamped, TwistStamped

class WpRecorder:

    def __init__(self):
        rospy.init_node("wp_recorder")
        self.NPC_dicts = dict()
        self.sub = rospy.Subscriber("/simulator/3D_detection",Detection3DArray,self.callback)
        self.tf_listener = tf.TransformListener()
        self.ego_pose_sub = rospy.Subscriber("/current_pose", PoseStamped, self.ego_pose_callback)
        self.ego_vel_sub = rospy.Subscriber("/current_velocity", TwistStamped, self.ego_vel_callback)
    
    def callback(self,objects):
        _time = time.time()
        for obj in objects.detections:
            ID = str(obj.id)
            pose = obj.bbox.position
            sz = obj.bbox.size
            sz = dict(x=sz.x,y=sz.y,z=sz.z)
            if ID not in self.NPC_dicts:
                self.NPC_dicts[ID] = []
            pose = self.sensor_to_json_pose(pose)
            if pose != None:
                self.NPC_dicts[ID].append(dict(position=pose["position"],rotation=pose["rotation"],speed=obj.velocity.linear.x, time=_time, size=sz))

    def ego_pose_callback(self, pose):
        _time = time.time()
        pose = self.sensor_to_json_pose(pose.pose, to_map=False)
        if "ego_pose" not in self.NPC_dicts:
            self.NPC_dicts["ego_pose"] = []
        self.NPC_dicts["ego_pose"].append(dict(position=pose["position"],rotation=pose["rotation"],time=_time))
    
    def ego_vel_callback(self, vel):
        _time = time.time()
        speed = vel.twist.linear.x
        if "ego_speed" not in self.NPC_dicts:
            self.NPC_dicts["ego_speed"] = []
        self.NPC_dicts["ego_speed"].append(dict(speed = vel.twist.linear.x,time=_time))

    def save(self,outpath):
        f = open(outpath,'w')
        json.dump(self.NPC_dicts,f)
        f.close()

    def sensor_to_json_pose(self,pose,to_map=True):
        json_pose = dict(position=dict(),rotation=dict())
        if to_map == True:
            # /base_link --> /map
            posestamped = PoseStamped()
            posestamped.header.frame_id = '/base_link'
            posestamped.pose = pose
            new_pose = self.transform_to_new_frame(posestamped,'/map')
            if new_pose == None:
                return None
            pose_in_map = new_pose.pose
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
        # json_pose['position']['y'] = pose_in_map.position.z
        json_pose['position']['y'] = 0
        json_pose['position']['z'] = pose_in_map.position.y
        json_pose['rotation']['x'] = pitch
        json_pose['rotation']['x'] = 0
        json_pose['rotation']['y'] = 90 - yaw
        # json_pose['rotation']['z'] = -roll
        json_pose['rotation']['z'] = 0

        # if using lgsvl original case, uncomment this part
        #json_pose['position']['x'] = - pose_in_map.position.y
        #json_pose['position']['y'] = pose_in_map.position.z
        #json_pose['position']['z'] = pose_in_map.position.x
        #json_pose['rotation']['y'] = -yaw

        return json_pose

    def transform_to_new_frame(self,pose,target_frame):
        try:    
            return self.tf_listener.transformPose(target_frame,pose)
        except:
            return None

if __name__=='__main__':
    wprecorder = WpRecorder()
    outpath = sys.argv[1]
    while True:
        cmd = sys.stdin.read().decode()
        if "Shut down" in cmd:
            wprecorder.save(outpath)
            print("Trajectory saved to "+outpath)
            break
        else:
            print(cmd)
