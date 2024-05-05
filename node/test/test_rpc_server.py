import xmlrpc.client
from time import sleep
rpcserver = xmlrpc.client.ServerProxy('http://127.0.0.1:12345')

my_map = "/home/autoware/shared_dir/autoware-data/BorregasAve/my_launch/my_map.launch"
my_sensing = "/home/autoware/shared_dir/autoware-data/BorregasAve/my_launch/my_sensing_simulator.launch"
my_detection = "/home/autoware/shared_dir/autoware-data/BorregasAve/my_launch/my_detection.launch"
my_localization = "/home/autoware/shared_dir/autoware-data/BorregasAve/my_launch/my_localization.launch"
my_mission_planning = "/home/autoware/shared_dir/autoware-data/BorregasAve/my_launch/my_mission_planning.launch"
my_motion_planning = "/home/autoware/shared_dir/autoware-data/BorregasAve/my_launch/my_motion_planning.launch"

rpcserver.register_launcher("my_map",[my_map])
rpcserver.register_launcher("my_sensing_simulator",[my_sensing])
rpcserver.register_launcher("my_localization",[my_localization])
rpcserver.register_launcher('my_detection',[my_detection])
rpcserver.register_launcher('my_mission_planning',[my_mission_planning])
rpcserver.register_launcher('my_motion_planning',[my_motion_planning])


#rpcserver.restart_launcher("my_sensing_simulator")
rpcserver.restart_bridge()
sleep(10)
rpcserver.restart_bridge()
#rpcserver.restart_launcher("my_sensing_simulator")