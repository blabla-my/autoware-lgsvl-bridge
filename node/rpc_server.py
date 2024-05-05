import subprocess
from time import sleep

from cv2 import GC_INIT_WITH_MASK
from SimpleXMLRPCServer import SimpleXMLRPCServer
from SimpleXMLRPCServer import SimpleXMLRPCRequestHandler
import rospy
import roslaunch
import os
HOST = '127.0.0.1'
PORT = 12345

class Autoware:
    def __init__(self):
        self.goal_sender_launcher = None
        self.goal_sender = None
        self.rviz_launcher = None
        self.rviz_running = False
        self.bridge_launcher = None
        self._bridge_running = False
        self.bridge_process = None
        self.rviz = None
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.launchers = dict()
        self.GT_nodes = dict()
        
    def start_bridge(self):
        if self.bridge_launcher == None:
            self.bridge_launcher = roslaunch.scriptapi.ROSLaunch()
            self.bridge_launcher.start()
        if self._bridge_running == False:
            self._bridge_running = True
            bridge_node = roslaunch.core.Node(package='rosbridge_server',node_type='rosbridge_websocket',name='bridge',required=True,output='screen')
            self.bridge_process = self.bridge_launcher.launch(bridge_node)
        return True

    def start_localization_GT_node(self):
        localization_GT_node_path = '/home/autoware_fuzzing/nodes/lgsvl_odometry_to_autoware_location.py'
        GT_nodes['localization'] = subprocess.Popen(['python',localization_GT_node_path])
        return True

    def start_detection_GT_node(self):
        detection_GT_node_path = '/home/autoware_fuzzing/nodes/lgsvl_to_autoware_detected_objects.py'
        GT_nodes['detection'] = subprocess.Popen(['python',detection_GT_node_path])
        return True

    def stop_GT(name):
        try:
            GT_nodes[name].kill()
            return True
        except Exception as e:
            return False

    def use_sim_time(self):
        os.system("rosparam set use_sim_time true")
        return True

    def stop_bridge(self):
        if self.bridge_process != None:
            self.bridge_process.stop()
            self.bridge_process = None
            self.bridge_launcher.stop()
            self.bridge_launcher = None
            self._bridge_running = False
        return True

    def bridge_running(self):
        return self._bridge_running
    
    def restart_bridge(self):
        self.stop_bridge()
        return self.start_bridge()

    def send_destination(self,destination):
        if self.goal_sender_launcher == None:
            self.goal_sender_launcher = roslaunch.scriptapi.ROSLaunch()
            self.goal_sender_launcher.start()
        args =  "pub /move_base_simple/goal geometry_msgs/PoseStamped \"{}\"".format(destination)
        goal_sender = roslaunch.core.Node(package='rostopic', node_type='rostopic', name = 'goal_sender', args=args, output='screen')
        self.goal_sender = self.goal_sender_launcher.launch(goal_sender)
        return True

    def register_launcher(self, name, args):
        # name: e.g. "my_sensing"
        # args[0]: launch file path
        # args[1:] arguments, 'name:value'
        if self.launchers.has_key(name):
            self.unregister_launcher(name)
        launchfile = roslaunch.rlutil.resolve_launch_arguments(args)[0]
        launcher = roslaunch.scriptapi.ROSLaunch()
        launcher.parent = roslaunch.parent.ROSLaunchParent(self.uuid,[( launchfile, args[1:] )])
        self.launchers[name] = [launcher, False]
        return True

    def unregister_launcher(self,name):
        if self.launchers.has_key(name):
            self.stop_launcher(name)
            launcher,state = self.launchers.pop(name)
            del launcher
            return True
        return False

    def current_modules(self):
        res = []
        for name,launcher in self.launchers.items():
            if launcher[1] == True:
                res.append(name)
        return res

    def start_launcher(self,name):
        if self.launchers.has_key(name):
            if self.launchers[name][1] == False:
                self.launchers[name][0].start()
                self.launchers[name][1] = True
                return True
        return False
    
    def stop_launcher(self,name):
        if self.launchers.has_key(name):
            try:
                launcher = self.launchers[name][0]
                launcher.parent.shutdown()
                # launcher.stop()
                self.launchers[name][0] = launcher
                self.launchers[name][1] = False
                return True
            except Exception as e:
                print(str(e))
                return False
        return False

    def restart_launcher(self,name):
        self.stop_launcher(name)
        return self.start_launcher(name)

    def start_rviz(self):
        if self.rviz_launcher == None:
            self.rviz_launcher = roslaunch.scriptapi.ROSLaunch()
            self.rviz_launcher.start()
        self.rviz_running = True
        rviz_node = roslaunch.core.Node(package='rviz',node_type='rviz',name='rviz')
        self.rviz = self.rviz_launcher.launch(rviz_node)
        return True

    def stop_rviz(self):
        if self.rviz_running == True:
            try:
                self.rviz.stop()
                self.rviz_running = False
            except Exception as e:
                print(str(e))
                return False
        return True
    
    def restart_rviz(self):
        self.stop_rviz()
        return self.start_rviz()

    def test(self):
        print("HELLO")
        return "HEllo!"

class RequestHandler(SimpleXMLRPCRequestHandler):
    rpc_paths = ('/RPC2',)

def main():
    rospy.init_node('autoware_launcher',anonymous=True)
    autoware = Autoware()
    server = SimpleXMLRPCServer((HOST,PORT),requestHandler=RequestHandler)
    server.register_introspection_functions()
    server.register_instance(autoware)
    server.serve_forever()

if __name__ == '__main__':
    main()