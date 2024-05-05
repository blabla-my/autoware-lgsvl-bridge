import traceback
import rospy
import math
import tf
import sys
import json

from autoware_msgs.msg import Lane
from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped, Quaternion 
from nav_msgs.msg import Odometry

from scipy.spatial.transform import Rotation
from localization_GT import sensor2autoware
import time
"""

- Subscribe the selected trajectory topic
- Every time when a trajectory is received, estimate the pose and velocity
    - publish estimated pose and velocity to /current_pose and /current_velocity
    - publish transform of /base_link --> /map
"""

class Estimator:
    def __init__(self):
        rospy.init_node("wp_recorder")
        self.sub = rospy.Subscriber("/final_waypoints", Lane, self.final_waypoints_callback )
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.initial_pose_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.initial_pose_callback)
        self.initial_vel_sub = rospy.Subscriber("/initialvel", TwistStamped, self.initial_vel_callback)
        self.pose_pub = rospy.Publisher('/current_pose', PoseStamped, queue_size=1)
        self.twist_pub = rospy.Publisher('/current_velocity', TwistStamped, queue_size=1)
        br = tf.TransformBroadcaster()
        self.last_vel = None
        self.last_pose = None
        self.initial_state = True
        self.stop_state = False
        self.initial_pose = None
        self.initial_vel = None

        self.trajectory = None

        self.simulation_ready = False

        self.last_pub_time = 0

        self.recorder = dict()
        self.recorder["ego"] = []
        self.file_recorder = open("/tmp/control_gt_traj.txt","w")

        self.max_distance_to_traj = 4
        

    def odom_callback(self,data):
        """
        Maintain the transform betweent /base_link_real and /map
        - Get location groundtruth from lgsvl
            - transform to autoware coordinate
        - broadcast transform
        """
        br = tf.TransformBroadcaster()
        data.pose.pose = sensor2autoware(data.pose.pose, scene_type="commonroad")
        origin = data.pose.pose.position
        orientation = data.pose.pose.orientation
        
        br.sendTransform((origin.x, origin.y, origin.z),
                        (orientation.x,orientation.y,orientation.z,orientation.w), rospy.Time.now(),
                        '/base_link_real', '/map')
        if self.simulation_ready == False:
            # time.sleep(1)
            self.simulation_ready = True

    def initial_pose_callback(self, pose):
        
        self.initial_pose = PoseStamped()
        self.initial_pose.pose = pose.pose.pose
        self.last_pose = self.initial_pose
        
        '''
        if self.initial_state != False:
            br = tf.TransformBroadcaster()
            br.sendTransform(
                (initial_pose.position.x, initial_pose.position.y, initial_pose.position.z),
                (initial_pose.orientation.x, initial_pose.orientation.y, initial_pose.orientation.z,initial_pose.orientation.w), 
                rospy.Time.now(),
                '/base_link', '/map'
            )
        '''

    def initial_vel_callback(self,vel):
        self.initial_vel = vel
        self.last_vel = vel
        # if self.initial_vel.twist.linear.x < 0.001:
            # self.initial_vel.twist.linear.x = 0.5
            #self.last_vel = self.initial_vel

    def find_closest_waypoint(self,position,trajectory):
        Min = float("inf")
        Min_i = -1
        for i in range(len(trajectory.waypoints)):
            #if trajectory.waypoints[i].time_cost > 0.02:
            #    return i+2
            dis = math.hypot(position.x-trajectory.waypoints[i].pose.pose.position.x, position.y-trajectory.waypoints[i].pose.pose.position.y)
            if dis < Min:
                Min_i = i
                Min = dis
        return Min_i 

    def sensor_to_json_pose(self,pose):
        json_pose = dict(position=dict(),angle=dict())
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
        json_pose['angle']['x'] = 0
        json_pose['angle']['y'] = (90 - yaw) % 360
        json_pose['angle']['z'] = 0

        # if using lgsvl original case, uncomment this part
        #json_pose['position']['x'] = - pose_in_map.position.y
        #json_pose['position']['y'] = pose_in_map.position.z
        #json_pose['position']['z'] = pose_in_map.position.x
        #json_pose['rotation']['y'] = -yaw
        
        return json_pose

    def final_waypoints_callback(self,trajectroy):
        if self.simulation_ready == True:
            self.initial_state = False
        self.trajectory = trajectroy
        # sys.stderr.write("final waypoints updated !\n")


    def callback(self, trajectory):
        if self.stop_state:
            sys.stderr.write("Trap into stop state\n")
        if self.simulation_ready == True:
            self.initial_state = False
            #return
        else:
            return
        pose,time_cost = self.estimate_pose(trajectory)
        vel = self.estimate_vel(trajectory)
        try:
            # sleep_gap = time_cost * 0.9
            # sleep(sleep_gap)
            ros_time = rospy.Time.now()

            br = tf.TransformBroadcaster()
            br.sendTransform(
                (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
                (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w), 
                ros_time,
                '/base_link', '/map'
            )
            #if self.stop_state == False:
            sleep_gap = time_cost - (time.time() - self.last_pub_time)
            if sleep_gap > 0:
                time.sleep(sleep_gap)
                # sys.stderr.write("sleep gap: {}; time cost: {}\n".format(sleep_gap,time_cost))
            pose.header.stamp = ros_time
            pose.header.frame_id = 'map'
            vel.header.stamp = ros_time
            vel.header.frame_id = 'map'
            self.last_pose = pose
            self.last_vel = vel
            self.pose_pub.publish(pose)
            self.twist_pub.publish(vel)
            sys.stderr.write("Estimated velocity: {}\n".format(vel.twist.linear.x))
            self.last_pub_time = time.time()

            # Record ego waypoints in lgsvl form
            waypoint = self.sensor_to_json_pose(self.last_pose.pose)
            timestamp = self.last_pub_time
            waypoint["timestamp"] = timestamp
            waypoint["ordinalNumber"] = len(self.recorder["ego"])
            waypoint["speed"] = self.last_vel.twist.linear.x
            self.recorder["ego"].append(waypoint)
            #sys.stderr.write(str(waypoint)+'\n')
            self.file_recorder.write(str(waypoint)+"\n")
            # print("esitmated: ", pose,vel)
            # sys.stderr.write(str(vel))
        except Exception as e:
            sys.stderr.write(str(e)+"\n"+traceback.format_exc()+"\n")
            
    def estimate_pose(self,trajectory):
        start_id = self.find_closest_waypoint(self.last_pose.pose.position,trajectory)
        # sys.stderr.write("selected id: {}\n".format(start_id))
        if len(trajectory.waypoints) > 1 and start_id > 0 and start_id < len(trajectory.waypoints):
            #time_cost = trajectory.waypoints[start_id].time_cost - trajectory.waypoints[start_id-1].time_cost
            dis = math.hypot(
                self.last_pose.pose.position.x-trajectory.waypoints[start_id].pose.pose.position.x,
                self.last_pose.pose.position.y-trajectory.waypoints[start_id].pose.pose.position.y,
            )
            if self.last_vel.twist.linear.x > 0.02:
                time_cost = dis / self.last_vel.twist.linear.x
                return trajectory.waypoints[start_id].pose,time_cost
            else: # trap into stop state
                time_cost = 0.1
                #self.stop_state = True
                sys.stderr.write("Trap in stop state.\n")
                return self.last_pose, time_cost
            # sys.stderr.write("Esitimate time_cost: {}, Distance: {}\n".format(time_cost,dis))
        else:
            self.last_pose
            return self.last_pose,0.1

    def last_waypoint(self,trajectory,pose):
        # for one segment, if pose is on it, return the start point of the segment
        if len(trajectory.waypoints) > 1:
            for i in range(len(trajectory.waypoints)-1):
                segment_distance = math.hypot(
                    trajectory.waypoints[i].pose.pose.position.x-trajectory.waypoints[i+1].pose.pose.position.x,
                    trajectory.waypoints[i].pose.pose.position.y-trajectory.waypoints[i+1].pose.pose.position.y,
                )
                to_start_distance = math.hypot(
                    trajectory.waypoints[i].pose.pose.position.x - self.last_pose.pose.position.x,
                    trajectory.waypoints[i].pose.pose.position.y - self.last_pose.pose.position.y,
                )
                if to_start_distance < segment_distance:
                    return i,to_start_distance,segment_distance
        return (-1,0,0)

    def orientation_to_yaw(self,ori):
        quat = [ori.x, ori.y, ori.z, ori.w]
        yaw,pitch,roll = Rotation.from_quat(quat).as_euler('zxy',degrees=True)
        return yaw

    def yaw_to_orientation(self,yaw):
        quat = Rotation.from_euler('zxy',[yaw,0,0],degrees=True).as_quat()
        ori = Quaternion()
        ori.x, ori.y, ori.z, ori.w = quat[0], quat[1], quat[2], quat[3]
        return ori

    def estimate_pose_v2(self,trajectory):
        """
        Estimate pose from (trajectory,current_pose)
        """
        # Calculate moving direction
        # Calculate estimated position
        if trajectory == None or len(trajectory.waypoints)<=1:
            # sys.stderr.write("Use last pose!\n")
            return self.last_pose
        moving_distance = abs(self.last_vel.twist.linear.x) * 0.02
        # moving_yaw = self.last_vel.twist.angular.y * 0.02
        last_waypoint_id,to_start_distance,segment_distance = self.last_waypoint(trajectory,self.last_pose)
        # sys.stderr.write("Last waypointd id :{}\n".format(last_waypoint_id))
        if last_waypoint_id < 0:
            last_waypoint_id = self.find_closest_waypoint(self.last_pose.pose.position, trajectory)
        if to_start_distance + moving_distance < segment_distance:
            # sys.stderr.write("Moving distance: {}\n".format(moving_distance))
            k = moving_distance / (segment_distance - to_start_distance)
            delta_x = k * ( trajectory.waypoints[last_waypoint_id+1].pose.pose.position.x - trajectory.waypoints[last_waypoint_id].pose.pose.position.x )
            delta_y = k * ( trajectory.waypoints[last_waypoint_id+1].pose.pose.position.y - trajectory.waypoints[last_waypoint_id].pose.pose.position.y )
            delta_yaw = k * ( self.orientation_to_yaw(trajectory.waypoints[last_waypoint_id+1].pose.pose.orientation)-self.orientation_to_yaw(trajectory.waypoints[last_waypoint_id].pose.pose.orientation) )
            cur_pose = PoseStamped()
            cur_pose.pose.position.x = self.last_pose.pose.position.x + delta_x
            cur_pose.pose.position.y = self.last_pose.pose.position.y + delta_y
            cur_pose.pose.orientation = trajectory.waypoints[last_waypoint_id].pose.pose.orientation
            cur_pose_yaw = self.orientation_to_yaw(trajectory.waypoints[last_waypoint_id].pose.pose.orientation) + delta_yaw
            cur_pose.pose.orientation = self.yaw_to_orientation(cur_pose_yaw)
            return cur_pose
        else:
            # move to next segment
            if last_waypoint_id + 2 < len(trajectory.waypoints):
                extra_distance = to_start_distance + moving_distance - segment_distance
                # sys.stderr.write("Extra distance: {}\n".format(extra_distance))
                next_segment_distance = math.hypot(
                        trajectory.waypoints[last_waypoint_id+2].pose.pose.position.x-trajectory.waypoints[last_waypoint_id+1].pose.pose.position.x,
                        trajectory.waypoints[last_waypoint_id+2].pose.pose.position.y-trajectory.waypoints[last_waypoint_id+1].pose.pose.position.y,
                    )
                k = extra_distance / next_segment_distance
                delta_x = k * ( trajectory.waypoints[last_waypoint_id+2].pose.pose.position.x - trajectory.waypoints[last_waypoint_id+1].pose.pose.position.x )
                delta_y = k * ( trajectory.waypoints[last_waypoint_id+2].pose.pose.position.y - trajectory.waypoints[last_waypoint_id+1].pose.pose.position.y )
                cur_pose = PoseStamped()
                cur_pose.pose.position.x = trajectory.waypoints[last_waypoint_id+1].pose.pose.position.x + delta_x
                cur_pose.pose.position.y = trajectory.waypoints[last_waypoint_id+1].pose.pose.position.y + delta_y
                cur_pose.pose.orientation = trajectory.waypoints[last_waypoint_id+1].pose.pose.orientation
                return cur_pose
            else:
                sys.stderr.write("Use last pose!\n")
                return self.last_pose
    
    def estimate_vel_v2(self,trajectory):
        """
        Estimate velocity from (trajectory, current_pose)
        """
        if trajectory == None or len(trajectory.waypoints)<=1:
            return self.last_vel
        moving_distance = abs(self.last_vel.twist.linear.x) * 0.02
        # moving_yaw = self.last_vel.twist.angular.y * 0.02
        last_waypoint_id,to_start_distance,segment_distance = self.last_waypoint(trajectory,self.last_pose)
        target_twist = TwistStamped()
        if last_waypoint_id < 0:
            last_waypoint_id = self.find_closest_waypoint(self.last_pose.pose.position, trajectory)
            to_start_distance = 0
            segment_distance = 0.5
            sys.stderr.write("Using closest waypoint. \n")
        if last_waypoint_id+1 < len(trajectory.waypoints) and to_start_distance + moving_distance < segment_distance:
            k = moving_distance / (segment_distance - to_start_distance) 
            # k = k + (1-k)
            delta_v = k * (trajectory.waypoints[last_waypoint_id+1].twist.twist.linear.x - self.last_vel.twist.linear.x) # sometimes the delta v should be add up to 7 times
            target_twist.twist.linear.x = max(0,self.last_vel.twist.linear.x+delta_v)
            return target_twist
        else:
            if last_waypoint_id + 2 < len(trajectory.waypoints):
                extra_distance = to_start_distance + moving_distance - segment_distance
                next_segment_distance = math.hypot(
                        trajectory.waypoints[last_waypoint_id+2].pose.pose.position.x-trajectory.waypoints[last_waypoint_id+1].pose.pose.position.x,
                        trajectory.waypoints[last_waypoint_id+2].pose.pose.position.y-trajectory.waypoints[last_waypoint_id+1].pose.pose.position.y,
                    )
                k = extra_distance / next_segment_distance
                delta_v = k * (trajectory.waypoints[last_waypoint_id+2].twist.twist.linear.x - trajectory.waypoints[last_waypoint_id+1].twist.twist.linear.x )
                target_twist.twist.linear.x = trajectory.waypoints[last_waypoint_id+2].twist.twist.linear.x + delta_v
                return target_twist
            else:
                sys.stderr.write("Using last velocity\n")
                return self.last_vel
        #if abs(target_twist.twist.linear.x - self.last_vel.twist.linear.x) / 0.02 >= 9.8*4:
        #    sign = -1 if target_twist.twist.linear.x < self.last_vel.twist.linear.x else 1
        #    target_twist.twist.linear.x += sign * (9.8*4) * 0.02
                


    def estimate_vel(self,trajectory):
        start_id = self.find_closest_waypoint(self.last_pose.pose.position,trajectory)
        if len(trajectory.waypoints) >= 1 and len(trajectory.waypoints) > start_id : # and trajectory.waypoints[start_id].twist.twist.linear.x > 0.001:
            #if trajectory.waypoints[start_id].twist.twist.linear.x > 0:
            #    trajectory.waypoints[start_id].twist.twist.linear.x = 0
            return trajectory.waypoints[start_id].twist
        else:
            self.last_vel.twist.linear.x = 0
            self.last_vel.twist.angular.y = 0
            return self.last_vel
        """
        if len(trajectory.waypoints) > start_id+1:
            delta_t = trajectory.waypoints[start_id+1].time_cost - trajectory.waypoints[start_id].time_cost
            if delta_t == 0:
                return self.last_vel
            def to_quat(orientation):
                return [orientation.x,orientation.y,orientation.z,orientation.w]
            quat1 = to_quat(trajectory.waypoints[start_id+1].pose.pose.orientation)
            quat0 = to_quat(trajectory.waypoints[start_id].pose.pose.orientation)
            yaw0,_,_ = Rotation.from_quat(quat0).as_euler('zxy')
            yaw1,_,_ = Rotation.from_quat(quat1).as_euler('zxy')
            delta_yaw = yaw1 - yaw0
            angular_vel = float(delta_yaw) / float(delta_t)
            distance = math.sqrt(
                pow(trajectory.waypoints[start_id+1].pose.pose.position.x-trajectory.waypoints[start_id].pose.pose.position.x,2)+
                pow(trajectory.waypoints[start_id+1].pose.pose.position.y-trajectory.waypoints[start_id].pose.pose.position.y,2)+
                pow(trajectory.waypoints[start_id+1].pose.pose.position.z-trajectory.waypoints[start_id].pose.pose.position.z,2)
            )
            linear_vel = distance / delta_t
            twist = TwistStamped()
            twist.twist.linear.x = linear_vel
            twist.twist.angular.z = angular_vel
            return twist
        else:
            self.last_vel.twist.linear.x = 0
            return self.last_vel
        """

    def run(self):
        while self.initial_state :
            if self.initial_pose != None:
                br = tf.TransformBroadcaster()
                br.sendTransform(
                    (self.initial_pose.pose.position.x, self.initial_pose.pose.position.y, self.initial_pose.pose.position.z),
                    (self.initial_pose.pose.orientation.x, self.initial_pose.pose.orientation.y, self.initial_pose.pose.orientation.z, self.initial_pose.pose.orientation.w), 
                    rospy.Time.now(),
                    '/base_link', '/map'
                )
                self.initial_pose.header.stamp = rospy.Time.now()
                self.initial_pose.header.frame_id = 'map'
                self.pose_pub.publish(self.initial_pose)
                self.last_pose = self.initial_pose
            if self.initial_vel != None:
                self.initial_vel.header.stamp = rospy.Time.now()
                self.twist_pub.publish(self.initial_vel)
                self.last_vel = self.initial_vel
            
            time.sleep(0.02)

        # Main loop of publishing
        while True:
            start_time = time.time()
            cur_pose = self.estimate_pose_v2(self.trajectory)
            cur_vel = self.estimate_vel_v2(self.trajectory)

            ros_time = rospy.Time.now()

            br = tf.TransformBroadcaster()
            br.sendTransform(
                (cur_pose.pose.position.x, cur_pose.pose.position.y, cur_pose.pose.position.z),
                (cur_pose.pose.orientation.x, cur_pose.pose.orientation.y, cur_pose.pose.orientation.z, cur_pose.pose.orientation.w), 
                ros_time,
                '/base_link', '/map'
            )
            cur_pose.header.stamp = ros_time
            cur_vel.header.stamp = ros_time
            if cur_vel.twist.linear.x == 0:
                sys.stderr.write("Trap into stop state\n")
            self.pose_pub.publish(cur_pose)
            #print(cur_pose.pose.position.x ,cur_pose.pose.position.y)
            self.twist_pub.publish(cur_vel)
            self.last_pose = cur_pose
            self.last_vel = cur_vel
            # sys.stderr.write("[CONTROL GT] published\n")
            waypoint = self.sensor_to_json_pose(self.last_pose.pose)
            timestamp = self.last_pub_time
            waypoint["timestamp"] = time.time()
            waypoint["ordinalNumber"] = len(self.recorder["ego"])
            waypoint["speed"] = self.last_vel.twist.linear.x
            self.recorder["ego"].append(waypoint)
            #sys.stderr.write(str(waypoint)+'\n')
            self.file_recorder.write(str(waypoint)+"\n")
            end_time = time.time()
            if 0.02 - (end_time - start_time) > 0.0001:
                #sys.stderr.write("sleep gap: {}\n".format(0.02 - (end_time - start_time)))
                time.sleep(0.02 - (end_time - start_time))
            pass


        rospy.spin()    
        self.file_recorder.close()

def main():
    estimator = Estimator()
    estimator.run()

if __name__ == '__main__':
    main()