#!/usr/bin/env python3
#
# Copyright (c) 2020-2021 LG Electronics, Inc.
#
# This software contains code licensed as described in LICENSE.
#
# basic script for running a simple vse test
#

import json
import os
import re
import sys
import time
import lgsvl
import math
import traceback
import subprocess
import timeout_decorator
import logger as _Logger
from datetime import datetime
from distutils.filelist import translate_pattern
from dataparser.scenario import Transform
from geometry_msgs.msg import Pose,PoseStamped,PoseWithCovarianceStamped,TwistStamped
from scipy.spatial.transform import Rotation
from time import sleep
from manager import Module,AutowareManager,lgsvl_transform_to_autoware_pose
from threading import Thread

log = _Logger.get_logger("vse-runner")
fuzzer_dir = os.getenv("AUTOWARE_FUZZER")

class MyException(Exception):
    pass

class VSERunner:
    def __init__(self, seed, workdir, feedback='none', _sensor_conf=None,autoware_manager=None,end_after_collision=True,no_ego=False, planning_timeout=30, detection=False):
        self.workdir = workdir
        self.seed = seed
        self.feedback = feedback
        self.VSE_dict = seed.to_json()
        self.is_collision = False
        self.sim = None
        self.ego_agents = []
        self.npc_agents = []
        self.pedestrian_agents = []
        self.collision_object = set()
        self.collision_object_detail = []
        self.maxint = 130
        self.npc_count = 0
        self.sensor_conf = _sensor_conf
        self.autoware_manager = autoware_manager
        self.ego = None
        self.gps = None
        self.ego_fall = False
        self.ego_state = None
        self.start_time = 0
        self.collision_time = 0
        self.fall_time = 0
        self.fall_position = None
        self.falling_listener_thread = None
        self.falling_listener_process = None
        self.no_sensor_data = False
        self.end_after_collision = end_after_collision
        self.no_ego = no_ego
        self.planning_timeout = planning_timeout
        self.detection = detection


    def reset(self):
        log.debug("Reset VSE runner")
        self.ego_agents.clear()
        self.npc_agents.clear()
        self.pedestrian_agents.clear()

    def close(self):
        self.reset()
        self.sim.reset()
        self.sim.close()
        # try:
        #   self.falling_listener_process.kill()
        # except :
        #     pass

    def setup_sim(self, default_host="127.0.0.1", default_port=8181):
        if not self.sim:
            simulator_host = os.getenv('LGSVL__SIMULATOR_HOST', default_host)
            simulator_port = int(os.getenv('LGSVL__SIMULATOR_PORT', default_port))
            log.debug("simulator_host is {}, simulator_port is {}".format(simulator_host, simulator_port))
            self.sim = lgsvl.Simulator(simulator_host, simulator_port)

    def connect_bridge(self, ego_agent, ego_index=0, default_host="127.0.0.1", default_port=9090):
        autopilot_host_env = "LGSVL__AUTOPILOT_{}_HOST".format(ego_index)
        autopilot_port_env = "LGSVL__AUTOPILOT_{}_PORT".format(ego_index)
        bridge_host = os.environ.get(autopilot_host_env, default_host)
        bridge_port = int(os.environ.get(autopilot_port_env, default_port))
        ego_agent.connect_bridge(bridge_host, bridge_port)

        return bridge_host, bridge_port

    def falling_listener(self):
        self.falling_listener_process = subprocess.Popen(["python", os.path.join(os.path.expanduser("~"),fuzzer_dir,
                                                            "node/falling_listener.py")],stdout=subprocess.PIPE)
        trans_str = self.falling_listener_process.stdout.read().decode()
        self.falling_listener_process.wait()
        self.fall_time = time.time() - self.start_time
        # self.ego_fall = True
        # parse trans_str
        try:
            p = trans_str.split("\n")
            if p != None and "no sensor data" in p:
                self.no_sensor_data = True
            elif p != None:
                x = float(p[0].strip("\n").strip("x: "))
                y = float(p[1].strip("\n").strip("y: "))
                z = float(p[2].strip("\n").strip("z: "))
                self.fall_position = lgsvl.Vector(x=x,y=y,z=z)
                self.ego_fall = True
        except Exception as e:
            #self.ego_fall = False
            # print(str(e))
            pass
        # print("[+] Falling state:", trans_str)

    def load_scene(self):
        if "map" not in self.VSE_dict.keys():
            log.error("No map specified in the scenario.")
            sys.exit(1)

        if False and self.VSE_dict["map"]["id"] != "":
            scene = self.VSE_dict["map"]["id"]
        else:
            scene = self.VSE_dict["map"]["name"]
        log.info("Loading {} map.".format(scene))
        if self.sim.current_scene == scene:
            self.sim.reset()
        else:
            self.sim.load(scene,seed=650387)

    def load_agents(self):
        if "agents" not in self.VSE_dict.keys():
            log.warning("No agents specified in the scenario")
            return

        agents_data = self.VSE_dict["agents"]
        for agent_data in agents_data:
            log.debug("Adding agent {}, type: {}".format(agent_data["variant"], agent_data["type"]))
            agent_type_id = agent_data["type"]
            if agent_type_id == lgsvl.AgentType.EGO.value:
                self.ego_agents.append(agent_data)

            elif agent_type_id == lgsvl.AgentType.NPC.value:
                self.npc_agents.append(agent_data)

            elif agent_type_id == lgsvl.AgentType.PEDESTRIAN.value:
                self.pedestrian_agents.append(agent_data)

            else:
                log.warning("Unsupported agent type {}. Skipping agent.".format(agent_data["type"]))

        self.npc_count = len(self.npc_agents)
        log.info("Loaded {} ego agents".format(len(self.ego_agents)))
        log.info("Loaded {} NPC agents".format(len(self.npc_agents)))
        log.info("Loaded {} pedestrian agents".format(len(self.pedestrian_agents)))

    def set_weather(self):
        if "weather" not in self.VSE_dict.keys() or "rain" not in self.VSE_dict["weather"]:
            log.debug("No weather specified in the scenarios")
            return
        weather_data = self.VSE_dict["weather"]
        weather_state = lgsvl.WeatherState(rain=weather_data["rain"],fog=weather_data["fog"],wetness=weather_data["wetness"],cloudiness=weather_data["cloudiness"],damage=weather_data["damage"])
        self.sim.weather = weather_state

    def set_time(self):
        if "time" not in self.VSE_dict.keys() or "year" not in self.VSE_dict["time"]:
            log.debug("No time specified in the scenarios")
            return
        time_data = self.VSE_dict["time"]
        if time_data["hour"] >= 15 or time_data["hour"] <= 9:
            time_data["hour"] = 12
            time_data["minute"] = 0
        dt = datetime(
            year = time_data["year"],
            month = time_data["month"],
            day = time_data["day"],
            hour = time_data["hour"],
            minute = time_data["minute"],
            second = time_data["second"]
        )
        
        self.sim.set_date_time(dt,fixed=False)

    def add_controllables(self):
        if "controllables" not in self.VSE_dict.keys():
            log.debug("No controllables specified in the scenarios")
            return

        controllables_data = self.VSE_dict["controllables"]
        for controllable_data in controllables_data:	
            # Name checking for backwards compability
            spawned = "name" in controllable_data or ("spawned" in controllables_data and controllable_data["spawned"])
            if spawned:
                log.debug("Adding controllable {}".format(controllable_data["name"]))
                controllable_state = lgsvl.ObjectState()
                controllable_state.transform = self.read_transform(controllable_data["transform"])
                try:
                    controllable = self.sim.controllable_add(controllable_data["name"], controllable_state)
                    controllable.attr = controllable_state.transform.position.x
                    policy = controllable_data["policy"]
                    if len(policy) > 0:
                        controllable.control(policy)
                except Exception as e:
                    msg = "Failed to add controllable {}, please make sure you have the correct simulator".format(controllable_data["name"])
                    log.error(msg)
                    log.error("Original exception: " + str(e))
            else:
                uid = controllable_data["uid"]
                log.debug("Setting policy for controllable {}".format(uid))
                controllable = self.sim.get_controllable_by_uid(uid)
                policy = controllable_data["policy"]
                if len(policy) > 0:
                    controllable.control(policy)
                    
    def make_ros_twist(self,vel):
        vel_abs = math.sqrt(
            vel["x"]**2 + vel["y"]**2 + vel["z"]**2
        )
        twist = TwistStamped()
        twist.twist.linear.x = vel_abs
        twist.twist.linear.y = twist.twist.linear.z = 0
        twist.twist.angular.x = twist.twist.angular.y = twist.twist.angular.z = 0
        return twist

    def add_ego(self):
        for i, agent in enumerate(self.ego_agents):
            if "id" in agent:
                agent_name = agent["id"]
            else:
                agent_name = agent["variant"]
            agent_state = lgsvl.AgentState()
            if 'initial_speed' in agent:
                agent_state.velocity = lgsvl.Vector(agent['initial_speed']['x'],agent['initial_speed']['y'],agent['initial_speed']['z'])
            else:
                agent["initial_speed"] = {"x":0,"y":0,"z":0}
            agent_state.transform = self.read_transform(agent["transform"])
            initial_pose = lgsvl_transform_to_autoware_pose(agent["transform"])
            initial_velocity = self.make_ros_twist(agent["initial_speed"])
            # initial_velocity = 
            if "destinationPoint" in agent:
                destination = lgsvl_transform_to_autoware_pose(agent["destinationPoint"])

            def _on_collision(agent1, agent2, contact):
                self.is_collision = True
                self.collision_time = time.time() - self.start_time
                name1 = "STATIC OBSTACLE" if agent1 is None else agent1.name
                name2 = "STATIC OBSTACLE" if agent2 is None else agent2.name
                print("{} collided with {} at {}".format(name1, name2, contact))
                self.seed.store(self.workdir + "/collision/" + str(self.seed.get_hash()))
                if agent1 is None or agent2 is None:
                    pass
                else:
                    # if using tmin
                    self.collision_object.add(agent1.attr)
                    self.collision_object.add(agent2.attr)

                    # If using replay
                    # self.collision_object.add(agent1.name)
                    # self.collision_object.add(agent2.name)
                if name1 == agent["sensorsConfigurationId"]:
                    _ego, _npc = agent1, agent2
                else:
                    _ego, _npc = agent2, agent1
                st1 = _ego.state if _ego is not None else None
                st2 = _npc.state if _npc is not None else None
                # degree = abs(st1.rotation.y - st2.rotation.y) if st1 != None and st2 != None else 0
                # degree = degree if degree < 180 else 360 - degree
                
                # speed1 = st1.speed if st1 is not None else 0
                # speed2 = st2.speed if st2 is not None else 0

                # if speed1 < speed2 and degree <= 90:
                #     log.info("NPC rear-end collision")
                # elif speed1 > speed2 and degree <= 90:
                #     log.info("EGO rear-end collision")
                # else:
                #     log.info("head-on collision")
                self.collision_object_detail = [st1, st2, self.collision_time]

                if self.end_after_collision:
                    sleep(1)
                    log.info("Stopping simulation")
                    self.sim.stop()

            try:
                if self.sensor_conf:
                    ego = self.sim.add_agent(self.sensor_conf, lgsvl.AgentType.EGO, agent_state)
                elif "sensorsConfigurationId" in agent:
                    if self.autoware_manager.sensors != None:
                        agent["sensorsConfigurationId"] = self.autoware_manager.sensors
                    print(f"[+] Adding agent {agent['sensorsConfigurationId']}")
                    ego = self.sim.add_agent(agent["sensorsConfigurationId"], lgsvl.AgentType.EGO, agent_state)
                else:
                    ego = self.sim.add_agent(agent_name, lgsvl.AgentType.EGO, agent_state)
                ego.attr = agent_state.transform.position.x
                ego.on_collision(_on_collision)
                self.ego = ego
            except Exception as e:
                msg = "Failed to add agent {}, please make sure you have the correct simulator".format(agent_name)
                log.error(msg)
                log.error("Original exception: " + str(e))
                # sys.exit(1)
                raise MyException

            try:
                # start bridge if it is not running
                # self.autoware_manager.bridge()
                bridge_host = self.connect_bridge(ego, i)[0]
                # restart op_planner
                # if self.detection:
                #     self.autoware_manager.restart("detection")
                self.autoware_manager.restart('op_global_planner')
                # self.autoware_manager.restart('localization_GT')
                # self.autoware_manager.restart('detection_GT')
                self.autoware_manager.restart('op_local_planner')
                self.autoware_manager.restart('waypoint_follower')
                sleep(3)
                # set desitination
                self.autoware_manager.sendpose('/initialpose',PoseWithCovarianceStamped,initial_pose)
                self.autoware_manager.sendvel('/initialvel',TwistStamped,initial_velocity)
                self.autoware_manager.sendpose('/move_base_simple/goal',PoseStamped,destination)
                # self.autoware_manager.wait_final_waypoints(time_out = 999)
                self.autoware_manager.wait_rollouts(time_out=self.planning_timeout)
                # print("rollouts got !!!!!!!!!!")
            except Exception as e:
                if "Planner Timeout" in str(e):
                    raise
                log.debug(str(e) + traceback.format_exc())

    def add_npc(self):
        for agent in self.npc_agents:
            if "id" in agent:
                agent_name = agent["id"]
            else:
                agent_name = agent["variant"]
            agent_state = lgsvl.AgentState()
            agent_state.transform = self.read_transform(agent["transform"])
            agent_color = lgsvl.Vector(agent["color"]["r"], agent["color"]["g"], agent["color"]["b"]) if "color" in agent else None

            try:
                npc = self.sim.add_agent(agent_name, lgsvl.AgentType.NPC, agent_state, agent_color)
                npc.attr = agent_state.transform.position.x
                if agent["uid"] == "fake-ego" and self.no_ego == True:
                    print("[+] Ego-NPC found.")
                    def fake_ego_on_collision(agent1,agent2,contact):
                        self.is_collision = True
                        self.collision_time = time.time() - self.start_time
                        print("[-] collision!")
                        print("[-] {} collision with {}".format(agent1.name,agent2.name))
                        if agent1 is None or agent2 is None:
                            pass
                        else:
                            self.collision_object.add(agent1.attr)
                            self.collision_object.add(agent2.attr)
                        st1 = agent1.state if agent1 is not None else None
                        st2 = agent2.state if agent2 is not None else None
                        self.collision_object_detail = [st1, st2, self.collision_time]
                        self.sim.stop()
                    npc.on_collision(fake_ego_on_collision)
                    self.ego = npc
            except Exception as e:
                msg = "Failed to add agent {}, please make sure you have the correct simulator".format(agent_name)
                log.error(msg)
                log.error("Original exception: " + str(e))
                # sys.exit(1)
                raise MyException

            if agent["behaviour"]["name"] == "NPCWaypointBehaviour":
                waypoints = self.read_waypoints(agent["waypoints"])
                if waypoints:
                    npc.follow(waypoints)
            elif agent["behaviour"]["name"] == "NPCLaneFollowBehaviour":
                npc.follow_closest_lane(
                    True,
                    agent["behaviour"]["parameters"]["maxSpeed"],
                    agent["behaviour"]["parameters"]["isLaneChange"]
                )

    def add_pedestrian(self):
        for agent in self.pedestrian_agents:
            if "id" in agent:
                agent_name = agent["id"]
            else:
                agent_name = agent["variant"]
            agent_state = lgsvl.AgentState()
            agent_state.transform = self.read_transform(agent["transform"])

            try:
                pedestrian = self.sim.add_agent(agent_name, lgsvl.AgentType.PEDESTRIAN, agent_state)
                pedestrian.attr = agent_state.transform.position.x
            except Exception as e:
                msg = "Failed to add agent {}, please make sure you have the correct simulator".format(agent_name)
                log.error(msg)
                log.error("Original exception: " + str(e))
                raise MyException

            waypoints = self.read_waypoints(agent["waypoints"])
            if waypoints:
                pedestrian.follow(waypoints)

    def read_transform(self, transform_data):
        transform = lgsvl.Transform()
        transform.position = lgsvl.Vector.from_json(transform_data["position"])
        transform.rotation = lgsvl.Vector.from_json(transform_data["rotation"])

        return transform

    def read_waypoints(self, waypoints_data):
        waypoints = []
        for waypoint_data in waypoints_data:
            position = lgsvl.Vector.from_json(waypoint_data["position"])
            speed = waypoint_data["speed"]
            angle = lgsvl.Vector.from_json(waypoint_data["angle"])
            if "wait_time" in waypoint_data:
                wait_time = waypoint_data["wait_time"]
            else:
                wait_time = waypoint_data["waitTime"]
            trigger = self.read_trigger(waypoint_data)
            # waypoint = lgsvl.DriveWaypoint(position, speed, angle=angle, idle=wait_time, trigger=trigger, timestamp=timestamp)
            if "timestamp" in waypoint_data:
                timestamp = waypoint_data["timestamp"]
                waypoint = lgsvl.DriveWaypoint(position, angle=angle, trigger=trigger, timestamp=timestamp)
            else:
                waypoint = lgsvl.DriveWaypoint(position, speed, angle=angle, idle=wait_time, trigger=trigger)
            waypoints.append(waypoint)
            
        return waypoints

    def read_trigger(self, waypoint_data):
        if "trigger" not in waypoint_data:
            return None
        effectors_data = waypoint_data["trigger"]["effectors"]
        if len(effectors_data) == 0:
            return None

        effectors = []
        for effector_data in effectors_data:
            effector = lgsvl.TriggerEffector(effector_data["typeName"], effector_data["parameters"])
            effectors.append(effector)
        trigger = lgsvl.WaypointTrigger(effectors)

        return trigger

    def split_pascal_case(self, s):
        matches = re.finditer('.+?(?:(?<=[a-z])(?=[A-Z\d])|(?<=[A-Z\d])(?=[A-Z][a-z])|$)', s)
        return [m.group(0) for m in matches]

    def calc_distance(self, ego, npc):
        ego_x = ego["x"]
        ego_y = ego["y"]
        ego_z = ego["z"]
        npc_x = npc["x"]
        npc_y = npc["y"]
        npc_z = npc["z"]
        dis = math.pow(npc_x - ego_x, 2) + math.pow(npc_y - ego_y, 2) + math.pow(npc_z - ego_z, 2)
        dis = math.sqrt(dis)
        return dis
    
    def calc_score(self):
        os.system("sleep 1 && cp ~/.config/unity3d/LGElectronics/SVLSimulator-2021.2.2/in.txt.gz /tmp")
        os.system("gunzip -f /tmp/in.txt.gz")

        f = open("/tmp/in.txt", 'r')

        ego_list = []
        npc_list = []

        tmp_buf = f.readline()
        while tmp_buf:
            tmp_obj = json.loads(tmp_buf)

            if type(tmp_obj) is dict:
                ego_list.append(tmp_obj)
            if type(tmp_obj) is list:
                if len(tmp_obj) != 0:
                    npc_list.append(tmp_obj)
            
            tmp_buf = f.readline()

        f.close()
        minD = self.maxint

        assert len(ego_list) == len(npc_list)

        for i in range(len(ego_list)):
            ego = ego_list[i]["Position"]
            npcs = npc_list[i]

            # print("------")
            for _i in npcs:
                npc = _i["Position"]
                curD = self.calc_distance(ego, npc)
                # print(_i["Id"], curD)
                if minD > curD:
                    minD = curD

        log.info(" *** minD is " + str(minD) + " *** ")

        # fitness = -1 * minD
        # score = (fitness + self.maxint) / float(len(self.npc_agents) - 1)
        score = minD / float(self.npc_count)
        return score

    @timeout_decorator.timeout(120)
    def run(self, duration=0.0, force_duration=False, loop=False):
        log.debug("Duration is set to {}.".format(duration))
        self.setup_sim()
        try:
            while True:
                self.load_scene()
                self.load_agents()
                #self.set_weather()
                #self.set_time()
                self.add_npc()
                self.add_pedestrian()
                self.add_controllables()
                print("[+] Before add ego.")
                if self.no_ego == False:
                    self.add_ego()  
                '''
                def _on_agents_traversed_waypoints():
                    log.info("All agents traversed their waypoints.")

                    if not force_duration:
                        log.info("Stopping simulation")
                        self.sim.stop()

                # self.sim.agents_traversed_waypoints(_on_agents_traversed_waypoints)
                '''
                log.info("Starting scenario...")
                self.start_time = time.time()
                # self.falling_listener_thread = Thread(target=self.falling_listener)
                # self.falling_listener_thread.start()
                # sleep(1)
                self.sim.run(duration)
                log.info("Scenario simulation ended.")
                self.ego_state = self.ego.state
                # if self.ego_fall and self.fall_time > 0:
                #     # print("[-] Using listener state.")
                #     self.ego_state.transform = lgsvl.Transform(position = self.fall_position, rotation=lgsvl.Vector(0,0,0))
                if loop:
                    self.reset()
                else:
                    break
        except MyException: 
            log.error("Program exit!")
            print("Cannot run this scene!", file=sys.stderr)
            exit(-1)
        
        self.close()
        # print("[-] Falling listener stoped.")
        # self.falling_listener_process.kill()
        # self.falling_listener_process.wait()
        # calculate feedback score
        if self.feedback == "avfuzzer":
            score = self.calc_score()
        else:
            score = 0

        if self.is_collision:
            score = 100

        return score

    
if __name__ == "__main__":
    import dataparser as _Parser
    import tempfile
    if len(sys.argv) < 2:
        log.error("Input file is not specified, please provide the scenario JSON file.")
        sys.exit(1)
    seed = _Parser.scenario_parser(sys.argv[1])
    print("test stderr", file=sys.stderr)
    autoware_manager = AutowareManager()
    autoware_manager.default()
    autoware_manager.set_args('map',map=seed.map["name"])
    autoware_manager.start_all()
    #autoware_manager.bridge()
    with tempfile.TemporaryDirectory() as tmpdirname:
        os.mkdir(tmpdirname + "/collision")
        vse_runner = VSERunner(seed,tmpdirname,'none',autoware_manager=autoware_manager)
        try:
            vse_runner.run(30)
        except Exception as e:
            # log.info(str(e) + traceback.format_exc())
            print(str(e) + traceback.format_exc())
        finally:
            # vse_runner.close()
            pass
        autoware_manager.clean()
