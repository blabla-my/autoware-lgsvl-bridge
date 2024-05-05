import rospy
from autoware_msgs.msg import VehicleStatus
from lgsvl_msgs.msg import VehicleOdometry

pub = rospy.Publisher("/vehicle_status", VehicleStatus, queue_size=1)

def callback(vehicle_odometry):
    vehicle_status = VehicleStatus()
    vehicle_status.header = vehicle_odometry.header
    vehicle_status.header.stamp = rospy.Time.now()
    vehicle_status.header.frame_id = 'base_link'
    vehicle_status.speed = vehicle_odometry.velocity / 1000.0
    vehicle_status.angle = vehicle_odometry.front_wheel_angle
    pub.publish(vehicle_status)

def main():
    rospy.init_node("vehicle_status_GT")
    sub = rospy.Subscriber("/lgsvl/vehicle_status", VehicleOdometry, callback)
    rospy.spin()

if __name__ == '__main__':
    main()