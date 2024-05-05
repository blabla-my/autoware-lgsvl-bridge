import rospy
import math
import tf

from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from nav_msgs.msg import Odometry
from time import sleep
from scipy.spatial.transform import Rotation
import os
import sys
import math

class Falling_listener:
    def __init__(self):
        self.fall = False   
        self.no_sensor_data = True
        os.system("rosnode kill falling_listener 2>>/dev/null")
        rospy.init_node('falling_listener')
        #print("falling listner start.")
        rospy.Subscriber('/odom', Odometry, self.callback)

    def callback(self,data):
        if self.no_sensor_data == True:
            self.no_sensor_data = False
        if self.fall == False and data.pose.pose.position.z < -5 :
            self.pose = Point()
            self.pose.z =  data.pose.pose.position.x
            self.pose.x = -data.pose.pose.position.y
            self.pose.y =  data.pose.pose.position.z
            self.fall = True    

if __name__ == '__main__':
    listener = Falling_listener()
    sleep_cnt = 0
    while listener.fall == False:
        if sleep_cnt >= 10 and listener.no_sensor_data == True:
            # print("no sensor data")
            break
        sleep(1)
        sleep_cnt += 1
    #print(listener.pose)
    if listener.no_sensor_data == False:
        trans_str = "x: {}\n".format(listener.pose.x)
        trans_str += "y: {}\n".format(listener.pose.y)
        trans_str += "z: {}".format(listener.pose.z)
        # print(trans_str)