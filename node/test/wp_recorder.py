import rospy
import tf
import json
from scipy.spatial.transform import Rotation
from lgsvl_msgs.msg import Detection3D, Detection3DArray
from geometry_msgs.msg import PoseStamped, Point32, PolygonStamped

class WpRecorder:

    def __init__(self):
        rospy.init_node("wp_recorder")
        self.NPC_dicts = dict()
        self.sub = rospy.Subscriber("/simulator/3D_detection",Detection3DArray,self.callback)
        self.tf_listener = tf.TransformListener()
    
    def callback(self,objects):
        for obj in objects.detections:
            ID = str(obj.id)
            pose = obj.bbox.position
            if ID not in self.NPC_dicts:
                self.NPC_dicts[ID] = []
            pose = self.sensor_to_json_pose(pose)
            self.NPC_dicts[ID].append(dict(position=pose["position"],rotation=pose["rotation"],speed=obj.velocity.linear.x))

    def save(self):
        f = open("test.json",'w')
        json.dump(self.NPC_dicts,f)
        f.close()

    def sensor_to_json_pose(self,pose):
        json_pose = dict(position=dict(),rotation=dict())
        # /base_link --> /map
        posestamped = PoseStamped()
        posestamped.header.frame_id = '/base_link'
        posestamped.pose = pose
        pose_in_map = self.transform_to_new_frame(posestamped,'/map').pose
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
        return json_pose

    def transform_to_new_frame(self,pose,target_frame):
        try:    
            return self.tf_listener.transformPose(target_frame,pose)
        except:
            return pose

if __name__=='__main__':
    wprecorder = WpRecorder()
    rospy.sleep(30)
    wprecorder.save()