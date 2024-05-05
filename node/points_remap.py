#!/bin/python
import rospy
import time
import tf
from sensor_msgs.msg import PointCloud2

class PointsRemapper:
    def __init__(self, points_topic = "/points_raw_origin"):
        rospy.init_node('points_remapper')
        print("point remapper starts")
        self.points_topic = points_topic
        self.sub = rospy.Subscriber(self.points_topic, PointCloud2, self.callback)
        self.pub = rospy.Publisher('/points_raw',  PointCloud2, queue_size = 1)
        self.listener = tf.TransformListener()

    def callback(self, point):
        point.header.stamp = rospy.Time.now()
        try:
            self.listener.waitForTransform("/base_link", "/map", point.header.stamp, rospy.Duration(2.0))
            self.pub.publish(point)
        except :
            pass
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = PointsRemapper()
    node.run()