import math
from numpy.lib.index_tricks import _fill_diagonal_dispatcher
from opendrive2lanelet.opendriveparser import parser as opdParser
from opendrive2lanelet.opendriveparser.elements.junction import Junction
from opendrive2lanelet.opendriveparser.elements.road import Road
from opendrive2lanelet.opendriveparser.elements.roadLanes import Lane
from typing import Tuple, List, Set, Dict

from shapely.geometry import Point, Polygon
from lxml import etree
import numpy as np
from math import floor, pi
import json
import os


class LanePosition:
    lane_position: List[Tuple]

    def __init__(self, _lane_id, _road_id, _lane_position: List[Tuple]):
        self.road_id = _road_id
        self.lane_id = _lane_id
        self.lane_position = _lane_position


class LaneBoundary:
    def __init__(self, _road_id):
        self.road_id = _road_id
        self.boundary_dict = {}
        self.mid_lanes = {}

    def add_boundary(self, lane_id, lane_positions: List[Tuple]):
        self.boundary_dict[lane_id] = lane_positions

    def get_mid_point(self, point1, point2):
        return (
            (point1[0] + point2[0]) / 2,
            (point1[1] + point2[1]) / 2,
            (point1[2] + point2[2]) / 2,
        )

    def get_mid_lane(self, lane_id):
        if lane_id == 0:
            return []
        if lane_id < 0:
            left_lane = self.boundary_dict[lane_id]
            right_lane = self.boundary_dict[lane_id + 1]
        else:
            left_lane = self.boundary_dict[lane_id - 1]
            right_lane = self.boundary_dict[lane_id]
        mid_lane = []
        lane_len = min(len(left_lane), len(right_lane))
        for i in range(lane_len):
            mid_lane.append(self.get_mid_point(left_lane[i], right_lane[i]))
        return mid_lane

    def get_mid_lanes(self):
        ids = sorted(self.boundary_dict.keys())
        left_id = ids[0]
        right_id = ids[-1]
        # print("road id {}".format(self.road_id))
        for i in range(left_id, 0, 1):
            self.mid_lanes["{}#{}".format(self.road_id, i)] = self.get_mid_lane(i)
        for j in range(right_id, 0, -1):
            self.mid_lanes["{}#{}".format(self.road_id, j)] = self.get_mid_lane(j)

    def get_min_lane_dis(self, position):
        closest_lane_id = 0
        min_dis = -1
        for lane_id in self.mid_lanes:
            tmp_min_dis = -1
            for point in self.mid_lanes[lane_id]:
                d = cal_dis(position, point)
                if d < tmp_min_dis or tmp_min_dis == -1:
                    tmp_min_dis = d
            if tmp_min_dis < min_dis or min_dis == -1:
                closest_lane_id = lane_id
                min_dis = tmp_min_dis
        return closest_lane_id, min_dis


def cal_dis(point1, point2):
    d = math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
    return d


def get_roads_and_junctions(path: str) -> Tuple[Set[Road], List[Junction]]:
    parser = etree.XMLParser()
    with open(path, "r") as file:
        root_node = etree.parse(file, parser).getroot()
    road_network = opdParser.parse_opendrive(root_node)
    return road_network.roads, road_network.junctions


def get_lane_waypoint(roads: Set[Road]) -> List[LanePosition]:
    # first loop ———— road, contains one reference line and several lanes
    LanePositionList: list[LanePosition] = []
    for road in roads:
        for lane_section in road.lanes.lane_sections:
            # calculate refence line
            if len(lane_section.leftLanes) > 0:
                lane = lane_section.leftLanes[0]
            elif len(lane_section.rightLanes) > 0:
                lane = lane_section.rightLanes[0]
            else:
                continue
            lane_position: List[Tuple] = []  # lane position
            for width in lane.widths:
                coeffs = width.polynomial_coefficients
                s_pos = width.start_offset
                # avoid s_pos goes beyond road.length
                if s_pos > float(road.planView.length):
                    current_point = road.planView.calc(road.planView.length)[0]
                    heading = road.planView.calc(road.planView.length)[1]
                else:
                    current_point = road.planView.calc(s_pos)[0]
                    heading = road.planView.calc(s_pos)[1]
                res_position = (current_point[0], current_point[1], 0)
                lane_position.append(res_position)
            lane_pos = LanePosition(0, road.id, lane_position)
            LanePositionList.append(lane_pos)
            pre_width = [0] * 1024
            # second loop  —————— lane, use width to calculate position
            # two sides —— left & right
            for lane in lane_section.leftLanes:
                pre_offset = 0
                lane_position: List[Tuple] = []  # lane position
                i = 0
                for width in lane.widths:
                    coeffs = width.polynomial_coefficients
                    s_pos = width.start_offset
                    # avoid s_pos goes beyond road.length
                    if s_pos > float(road.planView.length):
                        current_point = road.planView.calc(road.planView.length)[0]
                        heading = road.planView.calc(road.planView.length)[1]
                    else:
                        current_point = road.planView.calc(s_pos)[0]
                        heading = road.planView.calc(s_pos)[1]
                    delta_s = s_pos - pre_offset

                    # width = a + b * ds + c * ds^2 + d * ds^3
                    width = np.polynomial.polynomial.polyval(delta_s, coeffs)
                    pre_width[i] = pre_width[i] + width
                    t = pre_width[i]
                    i = i + 1

                    # t = pre_width[i] use half width to get the center line of lane
                    srot = delta_s * np.cos(heading) - t * np.sin(heading)
                    trot = delta_s * np.sin(heading) + t * np.cos(heading)

                    # left be plus
                    res_pos = current_point + np.array([srot, trot])

                    # calculate z
                    z = 0
                    # upload this point
                    pre_offset = s_pos
                    res_position = (res_pos[0], res_pos[1], z)
                    lane_position.append(res_position)

                lane_pos = LanePosition(lane.id, road.id, lane_position)
                LanePositionList.append(lane_pos)

            pre_width = [0] * 0x4000
            for lane in lane_section.rightLanes:
                pre_offset = 0
                lane_position: List[Tuple] = []  # lane position
                i = 0
                for width in lane.widths:
                    coeffs = width.polynomial_coefficients
                    s_pos = width.start_offset
                    # avoid s_pos goes beyond road.length
                    if s_pos > float(road.planView.length):
                        current_point = road.planView.calc(road.planView.length)[0]
                        heading = road.planView.calc(road.planView.length)[1]
                    else:
                        current_point = road.planView.calc(s_pos)[0]
                        heading = road.planView.calc(s_pos)[1]
                    delta_s = s_pos - pre_offset

                    # width = a + b * ds + c * ds^2 + d * ds^3
                    width = np.polynomial.polynomial.polyval(delta_s, coeffs)
                    pre_width[i] = pre_width[i] + width
                    t = pre_width[i]
                    i = i + 1
                    # t = pre_width[i] use half width to get the center line of lane
                    srot = delta_s * np.cos(heading) - t * np.sin(heading)
                    trot = delta_s * np.sin(heading) + t * np.cos(heading)
                    # right be minus
                    res_pos = current_point - np.array([srot, trot])
                    # calculate z
                    z = 0
                    # upload this point
                    pre_offset = s_pos
                    res_position = (res_pos[0], res_pos[1], z)
                    lane_position.append(res_position)
                lane_pos = LanePosition(lane.id, road.id, lane_position)
                LanePositionList.append(lane_pos)

    return LanePositionList


def check(xodr_map_path, position):
    roads, junctions = get_roads_and_junctions(xodr_map_path)
    # get waypoint list
    lane_points_list = get_lane_waypoint(roads)
    road_dic = {}
    for lane_points in lane_points_list:
        if lane_points.road_id not in road_dic:
            road_dic[lane_points.road_id] = LaneBoundary(lane_points.road_id)
        road_dic[lane_points.road_id].add_boundary(
            lane_points.lane_id, lane_points.lane_position
        )

    # print(road_dic)
    min_dis = -1
    result_lane_id = ""
    # position = [62.10,-78.67]
    for road_id in road_dic:
        road_dic[road_id].get_mid_lanes()
        lane_id, min_d = road_dic[road_id].get_min_lane_dis(position)
        # print(lane_id,min_d)
        if min_d < min_dis or min_dis == -1:
            min_dis = min_d
            result_lane_id = lane_id
    # print(result_lane_id)
    # print(result_lane_id)
    return result_lane_id  # road_id#lane_id