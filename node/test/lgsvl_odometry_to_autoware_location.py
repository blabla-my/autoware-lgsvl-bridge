#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
ground truth localization. Publishes the following topics:
    /ground_truth/current_velocty (geometry_msgs::TwistStamped)
    /ground_truth/current_pose    (geometry_msgs::PoseStamped)
"""
import rospy
import math
import tf

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry

import sys
import math

def rotate_90(orientation):

    w = float(orientation.w)
    x = float(orientation.x)
    y = float(orientation.y)
    z = float(orientation.z)

    r = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
    p = math.asin(2*(w*y-z*x))
    y = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))

    roll = r
    pitch = p
    yaw = y + math.pi/2
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
    #print(x,y,z,w)
    return orientation

#pose_pub = rospy.Publisher('/ground_truth/current_pose', PoseStamped, queue_size=1)
#twist_pub = rospy.Publisher('/ground_truth/current_velocity', TwistStamped, queue_size=1)
pose_pub = rospy.Publisher('/current_pose', PoseStamped, queue_size=1)
twist_pub = rospy.Publisher('/current_velocity', TwistStamped, queue_size=1)
# gnss_pub = rospy.Publisher('/gnss_pose', PoseStamped, queue_size=1)
tf_listener = None

def callback(data):
    """
    callback odometry
    """
    br = tf.TransformBroadcaster()
    origin = data.pose.pose.position
    orientation = data.pose.pose.orientation
    # rotate
    
    x = -origin.y
    y = origin.x
    z = origin.z
    origin.x = x
    origin.y = y
    origin.z = z
    orientation = rotate_90(orientation)
    
    br.sendTransform((origin.x, origin.y, origin.z),
                     (orientation.x,orientation.y,orientation.z,orientation.w), rospy.Time.now(),
                     '/base_link', '/map')
    #br.sendTransform((origin.x, origin.y, origin.z),
    #                 (orientation.x,orientation.y,orientation.z,orientation.w), rospy.Time.now(),
    #                 '/velodyne', '/map')
    #print(data.pose.pose)
    # Pose
    data.pose.pose.position = origin
    data.pose.pose.orientation = orientation
    pose = PoseStamped()
    pose.header = data.header
    # pose.header.frame_id = 'map'
    pose.pose = data.pose.pose

    # Twist
    linear = data.twist.twist.linear
    angular = data.twist.twist.angular

    twist = TwistStamped()
    twist.header = data.header
    # twist.header.frame_id = 'map'
    twist.twist.linear.x = math.sqrt(linear.x**2 + linear.y**2 + linear.z**2)
    twist.twist.angular = angular

    pose_pub.publish(pose)
    twist_pub.publish(twist)
    # gnss_pub.publish(pose)


def lgsvl_to_autoware_localization():
    """
    main loop
    """
    rospy.init_node('lgsvl_to_autoware_localization', anonymous=True)
    tf_listener = tf.TransformListener()
    # role_name = rospy.get_param('/role_name', 'ego_vehicle')
    rospy.Subscriber('/odom', Odometry, callback)
    rospy.spin()


if __name__ == '__main__':
    lgsvl_to_autoware_localization()