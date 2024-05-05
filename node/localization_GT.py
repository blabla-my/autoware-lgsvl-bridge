import rospy
import math
import tf

from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped, Pose
from nav_msgs.msg import Odometry
from time import sleep
from scipy.spatial.transform import Rotation
import os
import sys
import math

def initial_pose_callback(pose):
    global initial_pose
    if initial_pose == None:
        initial_pose = PoseStamped()
        initial_pose.pose = pose.pose.pose
        pose.header.frame_id = 'map'

def initial_vel_callback(vel):
    global initial_vel
    if initial_vel == None:
        initial_vel = vel

initial_state = True
initial_pose = None
initial_vel = None
initial_pose_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, initial_pose_callback)
initial_vel_sub = rospy.Subscriber('/initialvel', TwistStamped, initial_vel_callback)
pose_pub = rospy.Publisher('/current_pose', PoseStamped, queue_size=1)
twist_pub = rospy.Publisher('/current_velocity', TwistStamped, queue_size=1)
tf_listener = None

def sensor2autoware(pose,scene_type="commonroad"):
    if scene_type == "commonroad":
        x = -pose.position.y
        y = pose.position.x
        z = pose.position.z
        pose.position.x =  x
        pose.position.y =  y
        pose.position.z =  z
    if "lgsvl" not in scene_type:
        x = pose.orientation.x
        y = pose.orientation.y
        z = pose.orientation.z
        w = pose.orientation.w
        yaw,pitch,roll = Rotation.from_quat([x,y,z,w]).as_euler('zxy',degrees=False)
        yaw += math.pi / 2
        x,y,z,w = Rotation.from_euler('zxy',[yaw,pitch,roll],degrees=False).as_quat()
        pose.orientation.x = x
        pose.orientation.y = y
        pose.orientation.z = z
        pose.orientation.w = w
    if "SanFrancisco" in scene_type:
        pose.position.y = pose.position.y - 204.894546508414
        pose.position.x = pose.position.x - 155.45562744162498

    return pose

def callback(data):
    """
    callback odometry
    """
    global initial_state
    if initial_state == True:
        initial_state = False
    br = tf.TransformBroadcaster()
    data.pose.pose = sensor2autoware(data.pose.pose, scene_type="commonroad")
    origin = data.pose.pose.position
    orientation = data.pose.pose.orientation
    # rotate
    
    # for commonroad-scenes ( if for lgsvl original scene or carla scene, comment this )
    # x = -origin.y
    # y = origin.x
    # z = origin.z

    # origin.x = x
    # origin.y = y
    # origin.z = z

    # # if using lgsvl original map, uncomment this
    # orientation = rotate_90(orientation)
    
    br.sendTransform((origin.x, origin.y, origin.z),
                     (orientation.x,orientation.y,orientation.z,orientation.w), rospy.Time.now(),
                     '/base_link', '/map')
    br.sendTransform((origin.x, origin.y, origin.z),
                    (orientation.x,orientation.y,orientation.z,orientation.w), rospy.Time.now(),
                    '/velodyne', '/map')
    #print(data.pose.pose)
    # Pose
    data.pose.pose.position = origin
    data.pose.pose.orientation = orientation
    pose = PoseStamped()
    pose.header = data.header
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = 'map'
    pose.pose = data.pose.pose

    # Twist
    linear = data.twist.twist.linear
    angular = data.twist.twist.angular

    twist = TwistStamped()
    twist.header = data.header
    twist.header.stamp = rospy.Time.now()
    twist.header.frame_id = 'map'
    twist.twist.linear.x = math.sqrt(linear.x**2 + linear.y**2 + linear.z**2)
    twist.twist.angular = angular

    pose_pub.publish(pose)
    twist_pub.publish(twist)
    # gnss_pub.publish(pose)


def lgsvl_to_autoware_localization():
    """
    main loop
    """
    os.system("rosnode kill lgsvl_to_autoware_localization 2>>/dev/null")
    sleep(1)
    rospy.init_node('lgsvl_to_autoware_localization')
    # print("Localization Ground Truth start.")
    rospy.Subscriber('/odom', Odometry, callback)
    global initial_pose
    while initial_state == True:
        if initial_pose != None and initial_vel != None:
            br = tf.TransformBroadcaster()
            origin = initial_pose.pose.position
            orientation = initial_pose.pose.orientation
            br.sendTransform((origin.x, origin.y, origin.z),
                     (orientation.x,orientation.y,orientation.z,orientation.w), rospy.Time.now(),
                     '/base_link', '/map')
            br.sendTransform((origin.x, origin.y, origin.z),
                     (orientation.x,orientation.y,orientation.z,orientation.w), rospy.Time.now(),
                     '/velodyne', '/map')
            initial_pose.header.stamp = rospy.Time.now()
            initial_vel.header.stamp = rospy.Time.now()
            pose_pub.publish(initial_pose)
            twist_pub.publish(initial_vel)
        else:
            pass
            # print("initial pose or inital vel not ready")
        sleep(0.2)
            
    rospy.spin()


if __name__ == '__main__':
    lgsvl_to_autoware_localization()
