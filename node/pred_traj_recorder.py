import rospy
import tf
import json
import sys
import os
from scipy.spatial.transform import Rotation
from lgsvl_msgs.msg import Detection3D, Detection3DArray
from geometry_msgs.msg import PoseStamped, Point32, PolygonStamped
from autoware_msgs.msg import DetectedObjectArray, DetectedObject
class PredTrajRecorder:

    def __init__(self):
        rospy.init_node("pred_traj_recorder")
        self.NPC_dicts = dict()
        self.sub = rospy.Subscriber("/predicted_objects",DetectedObjectArray,self.callback)
        self.tf_listener = tf.TransformListener()
    
    def callback(self,objects):
        for obj in objects.objects:
            ID = str(obj.id)
            pose = obj.pose
            if ID not in self.NPC_dicts:
                self.NPC_dicts[ID] = []
            pose = self.sensor_to_json_pose(pose)
            traj = []
            # print("[DATA] length of candidate_trajectoreis.lanes: {}".format(len(obj.candidate_trajectories.lanes)))
            if len(obj.candidate_trajectories.lanes) == 0:
                continue
            trajectories = []
            for lane in obj.candidate_trajectories.lanes:
                traj = []
                for waypoint in lane.waypoints:
                    traj_pose = self.sensor_to_json_pose(waypoint.pose.pose)
                    traj.append(traj_pose)
                trajectories.append(traj)
            tmp = dict(pose = pose, trajectories=trajectories)
            #self.NPC_dicts[ID].append(dict(position=pose["position"],rotation=pose["rotation"],speed=obj.velocity.linear.x))
            self.NPC_dicts[ID].append(tmp)

    def save(self,outpath):
        f = open(outpath,'w')
        json.dump(self.NPC_dicts,f)
        f.close()

    def sensor_to_json_pose(self,pose):
        json_pose = dict(position=dict(),rotation=dict())
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
        
        return json_pose

    def transform_to_new_frame(self,pose,target_frame):
        try:    
            return self.tf_listener.transformPose(target_frame,pose)
        except:
            return pose

if __name__=='__main__':
    pred_traj_recorder = PredTrajRecorder()
    outpath = sys.argv[1]
    while True:
        cmd = sys.stdin.read().decode()
        if "Shut down" in cmd:
            #os.system("rm /tmp/out.json")
            #wprecorder.save('/tmp/out.json')
            pred_traj_recorder.save(outpath)
            print("Trajectory saved to "+outpath)
            break
        else:
            print(cmd)
