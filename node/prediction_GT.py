import math

from numpy import fix
import rospy
import tf
import json
import sys
import os
import copy
from scipy.spatial.transform import Rotation
from lgsvl_msgs.msg import Detection3D, Detection3DArray
from geometry_msgs.msg import PoseStamped, Point32, PolygonStamped,Pose
from autoware_msgs.msg import DetectedObjectArray, DetectedObject, LaneArray, Lane, Waypoint

def fix_density(points,density=0.5):
    if len(points)==0 or density==0:
        return points
    d = 0
    a = 0
    margin = density * 0.01
    remaining = 0
    n_points = 0
    fixed_path = [copy.deepcopy(points[0])]
    si = 0
    ei = 1
    while ei < len(points):
        #d += math.hypot(get(points[ei],'x')-get(points[ei-1],"x"), get(points[ei],'z')-get(points[ei-1],"z"))
        d += distance(points[ei].position,points[ei-1].position)
        a = math.atan2(points[ei].position.y-points[si].position.y, points[ei].position.x -points[si].position.x)
        z = points[ei].position.z

        if d < density - margin:
            ei += 1
            remaining = 0
        elif d > density + margin:
            
            pm = copy.deepcopy(points[si])
            n_points = int(math.floor(d / density)) 
            for k in range(n_points):
                pm.position.x = pm.position.x + density * math.cos(a)
                pm.position.y = pm.position.y + density * math.sin(a)
                pm.position.z = z
                
                closest_index = closest_point_idx(points, pm)
                # pm_with_correct_info = copy.deepcopy(pm)
                if closest_index >= 0 and closest_index < len(points):
                    pm_with_correct_info = copy.deepcopy(points[closest_index])
                    pm_with_correct_info.position = copy.deepcopy(pm.position)
                    fixed_path.append(pm_with_correct_info)

            remaining = d - n_points * density
            si += 1
            points[si].position = copy.deepcopy(pm.position)
            ei += 1
            d = 0
        else:
            d = 0
            remaining = 0
            fixed_path.append(copy.deepcopy(points[ei]))
            ei += 1
            si = ei -1
    
    # handle missing last point
    if len(fixed_path) > 1:
        e_p_0 = fixed_path[len(fixed_path)-2]
        e_p_1 = fixed_path[len(fixed_path)-1]
        e_p = points[len(points)-1]
        #d0 = math.hypot(get(e_p,"x")-get(e_p_0,"x"), get(e_p,"z")-get(e_p_0,"z"))
        #d1 = math.hypot(get(e_p,"x")-get(e_p_1,"x"), get(e_p,"z")-get(e_p_1,"z"))
        d0 = distance(e_p.position, e_p_0.position)
        d1 = distance(e_p.position, e_p_1.position)

        if d0 > density and d1 > density/4.0:
            fixed_path.append(e_p)
        else:
            fixed_path.pop()
            fixed_path.append(e_p)
    else:
        if len(points) > 1:
            fixed_path.append(points[-1])
    return fixed_path


def vecmod(vector):
    mod = 0
    for dim in vector:
        mod += dim**2
    return math.sqrt(mod)

def distance(a, b):
    ax = a.x
    ay = a.y
    bx = b.x
    by = b.y
    return math.sqrt(pow(ax-bx,2)+pow(ay-by,2))    

def closest_point_idx(points, target):
    min_id = -1
    min_distance = float('inf')
    for id,point in enumerate(points):
        dis = distance(point.position, target.position)
        if dis < min_distance:
            min_distance = dis
            min_id = id
    return min_id

class PredictionGT:
    def __init__(self, gt_traj_path, prediction_time = 10, prediction_distance = 20):
        rospy.init_node("prediction_ground_truth")
        self.NPC_dicts = {}
        self.sub = rospy.Subscriber("/prediction/motion_predictor/objects",DetectedObjectArray,self.callback)
        self.pub = rospy.Publisher('/predicted_objects',DetectedObjectArray, queue_size=1)
        self.load_gt_trajectory(gt_traj_path)

        self.prediction_time = prediction_time
        self.prediction_distance = prediction_distance
        pass
    
    def load_gt_trajectory(self,traj_path):
        gt_traj_json = json.load(open(traj_path,'r'))
        self.gt_traj = dict()
        for ID in gt_traj_json:
            if "ego" in ID:
                continue
            self.gt_traj[ID] = []
            for traj_point_json in gt_traj_json[ID]:
                self.gt_traj[ID].append(self.json_to_sensor_pose(traj_point_json))

    def json_to_sensor_pose(self,json_pose):
        sensor_pose = Pose()
        # Euler --> Quat
        yaw,pitch,roll = json_pose['rotation']['y'],json_pose['rotation']['x'],json_pose['rotation']['z']
        yaw = 90 - yaw
        roll = -roll
        quat = Rotation.from_euler('zxy',[yaw,pitch,roll],degrees=True).as_quat()
        # transform to ros sensor pose
        sensor_pose.position.x = json_pose['position']['x']
        sensor_pose.position.y = json_pose['position']['z']
        sensor_pose.position.z = json_pose['position']['y']
        sensor_pose.orientation.x = quat[0]
        sensor_pose.orientation.y = quat[1]
        sensor_pose.orientation.z = quat[2]
        sensor_pose.orientation.w = quat[3]
        return sensor_pose

    def generate_prediction_trajectory(self,id,current_pose,prediction_distance):
        result = LaneArray()
        result.id = id
        lane = Lane()
        ID = str(id)
        # print("prediction distance:",prediction_distance)
        if ID in self.gt_traj:
            start_idx = closest_point_idx(self.gt_traj[ID], current_pose)
            end_idx = start_idx
            dissum = 0
            while end_idx >= 0 and end_idx+1 < len(self.gt_traj[ID]) and dissum < prediction_distance:
                end_idx += 1
                dissum += distance(self.gt_traj[ID][end_idx-1].position, self.gt_traj[ID][end_idx].position)
            # generate lane arry
            if start_idx >= 0:
                # fix density
                fixed_traj = fix_density(self.gt_traj[ID][start_idx: end_idx+1])
                #fixed_traj = copy.deepcopy(self.gt_traj[ID][start_idx: end_idx+1])
                # print("length of fixed_traj:",len(fixed_traj))
                for point in fixed_traj:
                    waypoint = Waypoint()
                    waypoint.pose.pose = point
                    lane.waypoints.append(waypoint)
        result.lanes.append(lane)
        return result

    def callback(self,objects):
        # get current pose
        #predicted_objects = DetectedObjectArray()
        #predicted_objects.header = objects.header
        objects.header.stamp = rospy.Time.now()
        # objects.header.frame_id = 'map'
        for object in objects.objects:
            # generate current prediciton trajectory
            current_pose = object.pose
            id = object.id
            vel = object.velocity.linear.x
            prediction_distance = max(self.prediction_time*vel, self.prediction_distance) 
            object.candidate_trajectories = self.generate_prediction_trajectory(id,current_pose,prediction_distance)
        # publish prediction trajectory to /predicted_objects
        if not rospy.is_shutdown():
            self.pub.publish(objects)

    def run(self):
        print("Prediction ground truth start")
        rospy.spin()

if __name__ == '__main__':
    gt_traj = sys.argv[1].split(":=")[-1]
    predgt = PredictionGT(gt_traj_path=gt_traj)
    predgt.run()