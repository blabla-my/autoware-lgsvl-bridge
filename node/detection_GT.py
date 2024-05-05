import rospy
import tf
import numpy
import os
import sys
from time import sleep
from scipy.spatial.transform import Rotation
from autoware_msgs.msg import DetectedObjectArray, DetectedObject
from lgsvl_msgs.msg import Detection3D, Detection3DArray
from geometry_msgs.msg import PoseStamped, Point32, PolygonStamped

#pub = rospy.Publisher('/ground_truth/objects', DetectedObjectArray, queue_size=1)
pub = rospy.Publisher('/prediction/motion_predictor/objects',DetectedObjectArray, queue_size=1)
pub2 = rospy.Publisher('/detection/contour_tracker/objects',DetectedObjectArray, queue_size=1)
polygon_pub = rospy.Publisher('/test/polygon',PolygonStamped, queue_size=1)
tf_listener = None
BASELINK = "/base_link"
def transform_to_new_frame(tf_listener,pose,src_frame,target_frame):
    try:
        # assert(pose.header.frame_id==src_frame)
        return tf_listener.transformPose(target_frame,pose)
    except:
        return None

def transform_autoware_object(lgsvl_obj,pose_transformed):
    autoware_obj = DetectedObject()
    autoware_obj.header = lgsvl_obj.header
    autoware_obj.header.frame_id = 'map'
    autoware_obj.header.stamp = rospy.Time.now()
    autoware_obj.space_frame = 'map'
    autoware_obj.id = lgsvl_obj.id
    autoware_obj.score = lgsvl_obj.score
    autoware_obj.pose = pose_transformed
    autoware_obj.pose.position.z += lgsvl_obj.bbox.size.z / 2
    autoware_obj.pose_reliable = True
    autoware_obj.dimensions = lgsvl_obj.bbox.size
    autoware_obj.velocity = lgsvl_obj.velocity
    autoware_obj.velocity_reliable = True
    autoware_obj.valid = True
    label_mapping = {
        "Pedestrian": "person",
        "Bicyclist": "bicycle",
        "SchoolBus": "bus",
        "BoxTruck": "truck",
        "Default": "car"
    }
    if label_mapping.has_key(lgsvl_obj.label):
        autoware_obj.label = label_mapping[lgsvl_obj.label]
    else:
        autoware_obj.label = label_mapping['Default']
    # convex hull
    quat = [autoware_obj.pose.orientation.x, autoware_obj.pose.orientation.y, autoware_obj.pose.orientation.z, autoware_obj.pose.orientation.w]
    rotation = Rotation.from_quat(quat)
    # print(rot_matrix)
    autoware_obj.convex_hull.header = autoware_obj.header
    dims = autoware_obj.dimensions
    expand_size = 0
    dims.x += expand_size
    dims.y += expand_size
    dims.z += expand_size
    fl_up = [dims.x/2, dims.y/2, dims.z/2]
    fr_up = [dims.x/2, -dims.y/2, dims.z/2]
    rr_up = [-dims.x/2, -dims.y/2, dims.z/2]
    rl_up = [-dims.x/2, dims.y/2, dims.z/2]
    fl_pt = [dims.x/2, dims.y/2, -dims.z/2]
    fr_pt = [dims.x/2, -dims.y/2, -dims.z/2]
    rr_pt = [-dims.x/2, -dims.y/2, -dims.z/2]
    rl_pt = [-dims.x/2, dims.y/2, -dims.z/2]



    rect = [fl_up,fr_up,rr_up,rl_up,fl_up,fl_pt,fr_pt,rr_pt,rl_pt,fl_pt]
    """
    rect += [
        [0, dims.y/2, dims.z/2],
        [0, dims.y/2, -dims.z/2],
        [0, -dims.y/2, dims.z/2],
        [0, -dims.y/2, -dims.z/2],
        [dims.x/2, 0, dims.z/2],
        [-dims.x/2, 0, dims.z/2],
        [dims.x/2, 0, -dims.z/2],
        [-dims.x/2, 0, -dims.z/2],
        [dims.x/2, dims.y/2, 0],
        [dims.x/2, -dims.y/2, 0],
        [-dims.x/2, dims.y/2, 0],
        [-dims.x/2, -dims.y/2, 0]
    ]
    """
    for vec in rect:
        pt = rotation.apply(vec)
        # print(pt)
        p = Point32()
        p.x = pt[0] + pose_transformed.position.x
        p.y = pt[1] + pose_transformed.position.y
        p.z = pt[2] + pose_transformed.position.z
        autoware_obj.convex_hull.polygon.points.append(p)
    return autoware_obj

def generate_polygon(object):
    longitudinal_distance = object.dimensions.y / 2.0
    lateral_distance = object.dimensions.x / 2.0
    vertial_distance = object.dimensions.z

    p = Point32()

    p.y = object.pose.position.y + longitudinal_distance
    p.x = object.pose.position.x + lateral_distance
    p.z = vertial_distance
    object.convex_hull.polygon.points.append(p)

    p = Point32()
    p.y = object.pose.position.y + longitudinal_distance
    p.x = object.pose.position.x + lateral_distance
    p.z = 0
    object.convex_hull.polygon.points.append(p)

    p = Point32()
    p.y = object.pose.position.y + longitudinal_distance
    p.x = object.pose.position.x - lateral_distance
    p.z = vertial_distance
    object.convex_hull.polygon.points.append(p)
    
    p = Point32()
    p.y = object.pose.position.y + longitudinal_distance
    p.x = object.pose.position.x - lateral_distance
    p.z = 0
    object.convex_hull.polygon.points.append(p)

    p = Point32()
    p.y = object.pose.position.y - longitudinal_distance
    p.x = object.pose.position.x + lateral_distance
    p.z = vertial_distance
    object.convex_hull.polygon.points.append(p)

    p = Point32()
    p.y = object.pose.position.y - longitudinal_distance
    p.x = object.pose.position.x + lateral_distance
    p.z = 0
    object.convex_hull.polygon.points.append(p)

    p = Point32()
    p.y = object.pose.position.y - longitudinal_distance
    p.x = object.pose.position.x - lateral_distance
    p.z = vertial_distance
    object.convex_hull.polygon.points.append(p)

    p = Point32()
    p.y = object.pose.position.y - longitudinal_distance
    p.x = object.pose.position.x - lateral_distance
    p.z = 0
    object.convex_hull.polygon.points.append(p)

    return object.convex_hull.polygon

def callback(data):
    """
    callback for current objects
    """
    objects_msg = DetectedObjectArray()
    objects_msg.header = data.header
    objects_msg.header.stamp = rospy.Time.now()
    objects_msg.header.frame_id = 'map'
    for obj in data.detections:
        '''
            object_msg = DetectedObject()
            object_msg.header = obj.header
            object_msg.header.frame_id = 'map'
            object_msg.id = obj.id
            object_msg.label = obj.label
            object_msg.valid = True
            object_msg.score = obj.score
            object_msg.space_frame = 'map'
            object_msg.pose = obj.bbox.position
            object_msg.dimensions = obj.bbox.size
            object_msg.velocity = obj.velocity
            object_msg.pose_reliable = True
            object_msg.velocity_reliable = True
        '''
        # transform pose to /map 
        pose_transformed = PoseStamped()
        pose_transformed.header.frame_id = BASELINK
        pose_transformed.pose = obj.bbox.position
        global tf_listener
        pose_transformed = transform_to_new_frame(tf_listener=tf_listener,pose=pose_transformed,src_frame=BASELINK,target_frame='/map')
        if pose_transformed == None:
            return
        else:
            pose_transformed = pose_transformed.pose
        object_msg = transform_autoware_object(lgsvl_obj=obj,pose_transformed=pose_transformed)
        objects_msg.objects.append(object_msg)
        polygon_pub.publish(object_msg.convex_hull)
        

    if not rospy.is_shutdown():
        pub.publish(objects_msg)
        pub2.publish(objects_msg)

def convert_objects():
    """
    main loop
    """
    os.system('rosnode kill lgsvl_to_autoware_detected_objects 2>>/dev/null')
    sleep(1)
    rospy.init_node('lgsvl_to_autoware_detected_objects')
    # print("Detection Ground Truth start.")
    global tf_listener
    tf_listener = tf.TransformListener()
    rospy.Subscriber('/simulator/3D_detection', Detection3DArray, callback)
    rospy.spin()

if __name__ == '__main__':
    if len(sys.argv) > 1:
        name,value=sys.argv[1].split(":=")
        if name=="base_link":
            BASELINK = value
            sys.stderr.write("********"+BASELINK)
    convert_objects()
