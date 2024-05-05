#!/usr/bin/env python3
#
# Copyright (c) 2020-2021 LG Electronics, Inc.
#
# This software contains code licensed as described in LICENSE.
#
# basic script for running a simple vse test
#

import json
import sys
import os
sys.path.append(os.getenv("AUTOWARE_FUZZER"))

import logger as _Logger
import re
import lgsvl
from datetime import datetime
from geometry_msgs.msg import Pose,PoseStamped,PoseWithCovarianceStamped,TwistStamped
import math
from scipy.spatial.transform import Rotation
from time import sleep, time
from manager import Module,AutowareManager,lgsvl_transform_to_autoware_pose
import traceback

# log = _Logger.get_logger("vse-runner")
log = _Logger.get_logger("seeds-test")
class VSERunner:
    def __init__(self, json_file, _sensor_conf=None,autoware_manager=None,lane=None,only_ego=False,gt_traj=None,detection=False, ignore_collision = False):
        with open(json_file) as f:
            self.VSE_dict = json.load(f)
        self.jsonfile= json_file
        self.isEgoFault = True
        self.isCollision = False
        self.sim = None
        self.ego_agents = []
        self.npc_agents = []
        self.pedestrian_agents = []
        self.sensor_conf = _sensor_conf
        self.autoware_manager = autoware_manager
        self.mapname=self.VSE_dict["map"]["name"]
        self.mappath=lane
        self.only_ego = only_ego
        self.gt_traj = gt_traj
        self.detection = detection
        self.ignore_collision = ignore_collision

        if self.gt_traj != None:
            print("[+] Runnning prediction ground truth")
            self.autoware_manager.prediction_ground_truth()
            self.autoware_manager.set_args('prediction_GT',gt_trajectory=self.gt_traj)
        elif self.detection == True:
            print("[+] Runnning detection normal")
            self.autoware_manager.detection_normal()
        else:
            print("[+] Runnning default mode")
            self.autoware_manager.default()
            
        self.autoware_manager.set_args('map',map=self.mapname)
        self.autoware_manager.start_all()
        if self.detection:
            print("[+] sleep 10 seconds to make detection work normal")
            sleep(10)   


    def reset(self):
        log.debug("Reset VSE runner")
        self.ego_agents.clear()
        self.npc_agents.clear()
        self.pedestrian_agents.clear()

    def close(self):
        self.reset()
        self.sim.reset()
        self.sim.close()

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
                    policy = controllable_data["policy"]
                    if len(policy) > 0:
                        controllable.control(policy)
                except Exception as e:
                    msg = "Failed to add controllable {}, please make sure you have the correct simulator".format(controllable_data["name"])
                    print(msg, file=sys.stderr)
                    raise
            else:
                uid = controllable_data["uid"]
                log.debug("Setting policy for controllable {}".format(uid))
                controllable = self.sim.get_controllable_by_uid(uid)
                policy = controllable_data["policy"]
                if len(policy) > 0:
                    controllable.control(policy)
                
    def add_ego(self):
        for i, agent in enumerate(self.ego_agents):
            if "id" in agent:
                agent["id"] = "3f4211dc-e5d7-42dc-94c5-c4832b1331bb"
                agent["variant"] = "Jaguar2015XE"
                agent_name = agent["id"]
            else:
                agent["variant"] = "Jaguar2015XE"
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
                self.isCollision = True

                name1 = "STATIC OBSTACLE" if agent1 is None else agent1.name
                name2 = "STATIC OBSTACLE" if agent2 is None else agent2.name
                if name1 == agent["sensorsConfigurationId"]:
                    _ego, _npc = agent1, agent2
                else:
                    _ego, _npc = agent2, agent1

                log.info("{} collided with {} at {}".format(name1, name2, contact))
                ego_speed = _ego.state.speed
                log.info("ego speed {}".format(ego_speed))
                if ego_speed < 0.005:
                    self.isEgoFault = False

                st1 = _ego.state
                st2 = _npc.state if _npc is not None else None
                if st1 is None or st2 is None:
                    pass
                else:
                    log.info(st1.rotation)
                    log.info(st2.rotation)
                    log.info(st1.speed)
                    log.info(st2.speed)
                    degree = abs(st1.rotation.y - st2.rotation.y)
                    degree = degree if degree < 180 else 360 - degree
                    if st1.speed < st2.speed and degree <= 90:
                        log.info("NPC rear-end collision")
                    elif st1.speed > st2.speed and degree <= 90:
                        log.info("EGO rear-end collision")
                    else:
                        log.info("head-on collision")
                
                log.info("Stopping simulation")
                self.sim.stop()

            try:
                if self.sensor_conf:
                    ego = self.sim.add_agent(self.sensor_conf, lgsvl.AgentType.EGO, agent_state)
                elif "sensorsConfigurationId" in agent:
                    agent["sensorsConfigurationId"] = "367c1a0b-c5a3-4ffc-a6b2-635a762c8894" if self.detection==False else \
                                                      "05cbb194-d095-4a0e-ae66-ff56c331ca83"
                    print(f"[+] Adding agent {agent['sensorsConfigurationId']}")
                    ego = self.sim.add_agent(agent["sensorsConfigurationId"], lgsvl.AgentType.EGO, agent_state)
                else:
                    ego = self.sim.add_agent(agent_name, lgsvl.AgentType.EGO, agent_state)
                if self.ignore_collision == False: 
                    ego.on_collision(_on_collision)
                self.ego = ego
            except Exception as e:
                msg = "Failed to add agent {}, please make sure you have the correct simulator".format(agent_name)
                log.error(msg)
                log.error("Original exception: " + str(e) + traceback.format_exc())
                raise
                # sys.exit(1)

            try:
                # start bridge if it is not running
                bridge_host = self.connect_bridge(ego, i)[0]
                self.autoware_manager.restart('op_global_planner')
                # self.autoware_manager.restart('localization_GT')
                # self.autoware_manager.restart('detection_GT')
                self.autoware_manager.restart('op_local_planner')
                self.autoware_manager.restart('waypoint_follower')
                sleep(3)
                self.autoware_manager.sendpose('/current_pose',PoseStamped,initial_pose)
                self.autoware_manager.sendpose('/initialpose',PoseWithCovarianceStamped,initial_pose)
                self.autoware_manager.sendvel('/initialvel',TwistStamped,initial_velocity)
                self.autoware_manager.sendpose('/move_base_simple/goal',PoseStamped,destination)
                self.autoware_manager.wait_rollouts(time_out=20)
                # self.autoware_manager.wait_final_waypoints(time_out = 20)
                pass
            except Exception as e:
                if "Planner Timeout" in str(e):
                    raise
                print(str(e), traceback.format_exc(), file=sys.stderr)
                raise

    def make_ros_twist(self,vel):
        vel_abs = math.sqrt(
            vel["x"]**2 + vel["y"]**2 + vel["z"]**2
        )
        twist = TwistStamped()
        twist.twist.linear.x = vel_abs
        twist.twist.linear.y = twist.twist.linear.z = 0
        twist.twist.angular.x = twist.twist.angular.y = twist.twist.angular.z = 0
        return twist

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
            except Exception as e:
                msg = "Failed to add agent {}, please make sure you have the correct simulator".format(agent_name)
                log.error(msg)
                log.error("Original exception: " + str(e) + traceback.format_exc())
                # sys.exit(1)
                raise

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
            except Exception as e:
                msg = "Failed to add agent {}, please make sure you have the correct simulator".format(agent_name)
                log.error(msg)
                log.error("Original exception: " + str(e) + traceback.format_exc())
                # sys.exit(1)
                raise

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
                waypoint = lgsvl.DriveWaypoint(position, speed, angle=angle, trigger=trigger, timestamp=timestamp)
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

    def run(self, duration=0.0, force_duration=False, loop=False):
        log.debug("Duration is set to {}.".format(duration))
        self.setup_sim()
        #start_time = time()
        while True:
            self.load_scene()
            self.load_agents()
            #self.set_weather()
            #self.set_time()
            if self.only_ego == False:
                self.add_npc()
                self.add_pedestrian()
                self.add_controllables()
            self.add_ego()  
            def _on_agents_traversed_waypoints():
                log.info("All agents traversed their waypoints.")

                if not force_duration:
                    log.info("Stopping simulation")
                    self.sim.stop()

            # self.sim.agents_traversed_waypoints(_on_agents_traversed_waypoints)
            start_time = time()
            log.info("Starting scenario...")
            self.sim.run(duration)
            log.info("Scenario simulation ended.")
            # if self.isCollision:
            #     log.info("Collision happen! IsEgoFault:{}".format(self.isEgoFault))
            #     if self.isEgoFault:
            #         os.system("cp {} {}".format(self.jsonfile,"./true_collision/"))
            #     else:
            #         os.system("cp {} {}".format(self.jsonfile,"./fp_collision"))
            # else:
            #     os.system("cp {} {}".format(self.jsonfile,"./not_collision"))

            transform = self.ego.state.transform
            log.info("Ego transform at final {}".format(transform) )

            if loop:
                self.reset()
            else:
                break
        print(f"[+] running time: {time() - start_time}")

if __name__ == "__main__":

    import getopt

    opts,args = getopt.getopt(sys.argv[1:], "", longopts=["detection","only-ego","gt-trajectory=", "ignore-collision"])

    assert len(args) == 1, "can only run one scenario."

    detection, onlyego, ignore_collision = False, False, False
    gt_traj = None
    for opt,value in opts:
        if opt == "--detection":
            detection = True
        elif opt == "--only-ego":
            onlyego = True
        elif opt == "--gt-trajectory":
            gt_traj = value
        elif opt == "--ignore-collision":
            ignore_collision = True
    autoware_manager = AutowareManager()
    #autoware_manager.bridge()
    vse_runner = VSERunner(args[0],autoware_manager=autoware_manager,only_ego=onlyego,gt_traj=gt_traj, detection=detection, ignore_collision = ignore_collision)
    try:
        vse_runner.run(30)
        # print(vse_runner.ego.transform)
        final_transform = vse_runner.ego.transform
        if final_transform.position.y < -10:
            print("[SeedTest] Ego fell.",file=sys.stderr)
    except Exception as e:
        print("[SeedTest] Failed.",file=sys.stderr)
        print(str(e),file=sys.stderr)
        print(traceback.format_exc())
    autoware_manager.clean()
    
