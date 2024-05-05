"""
dataparser.lane
"""
from opendrive2lanelet.opendriveparser import parser as opdParser
from opendrive2lanelet.opendriveparser.elements.junction import Junction
from opendrive2lanelet.opendriveparser.elements.road import Road
from opendrive2lanelet.opendriveparser.elements.roadLanes import Lane
from typing import Tuple, List, Set, Dict

from shapely.geometry import Point, Polygon
from lxml import etree
import numpy as np
import sys


def get_roads_and_junctions(path: str) -> Tuple[Set[Road], List[Junction]]:
    parser = etree.XMLParser()
    with open(path, 'r') as file:
        root_node = etree.parse(file, parser).getroot()
    road_network = opdParser.parse_opendrive(root_node)
    return road_network.roads, road_network.junctions


class LanePosition():
    _lane_position: List[Tuple]

    def __init__(self, lane: Lane, road: Road, lane_position: List[Tuple]):
        self._parent_Road = road
        self._parent_Road_id = road._id
        self._lane_id = lane.id
        self._type = lane.type
        self._lane_position = lane_position

def get_lane_waypoint(roads: Set[Road]) -> List[LanePosition]:
    #f = open('data_z.txt','w')
    #f.write('road_id,lane_id,x,y,z\n') # y equals z in simultor

    # first loop ———— road, contains one reference line and several lanes
    LanePositionList: list[LanePosition] = []
    for road in roads:
        for lane_section in road.lanes.lane_sections:
            pre_width = [0] * 4096 
            # second loop  —————— lane, use width to calculate position
            
            # two sides —— left & right
            for lane in lane_section.leftLanes:
            #for lane in lane_section.allLanes:
                pre_offset = 0
                lane_position: List[Point] = [] # lane position
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
                    t = pre_width[i] - width / 2 
                    i = i + 1   

                    # t = pre_width[i] use half width to get the center line of lane
                    srot = delta_s / 2 * np.cos(heading) - t * np.sin(heading)
                    trot = delta_s / 2 * np.sin(heading) + t * np.cos(heading)

                    # left be plus
                    res_pos = current_point + np.array([srot, trot])

                    # calculate z
                    for elevationProfile in reversed(road.elevationProfile.elevations):
                        if s_pos >= elevationProfile.start_pos:
                            z_coeffs = elevationProfile.polynomial_coefficients
                            z_delta_s = s_pos - elevationProfile.start_pos
                            z = np.polynomial.polynomial.polyval(z_delta_s, z_coeffs)
                            break

                    # upload this point
                    pre_offset = s_pos
                    res_position = (res_pos[0], res_pos[1], z)
                    lane_position.append(res_position)
                    #f.write(str(road._id)+','+str(lane._id)+','+ str(res_pos[0])+','+str(res_pos[1])+ ','+ str(z)+'\n')
                    #print (str(road._id)+','+str(lane._id)+','+ str(res_pos[0])+','+str(res_pos[1])+','+str(heading)+','+str(t))

                lane_pos = LanePosition(lane, road, lane_position)
                LanePositionList.append(lane_pos)

            pre_width = [0] * 0x4000
            for lane in lane_section.rightLanes:
                pre_offset = 0
                lane_position: List[Point] = [] # lane position
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
                    t = pre_width[i] - width / 2
                    i = i + 1   

                    # t = pre_width[i] use half width to get the center line of lane
                    srot = delta_s / 2 * np.cos(heading) - t * np.sin(heading)
                    trot = delta_s / 2 * np.sin(heading) + t * np.cos(heading)

                    # right be minus
                    res_pos = current_point - np.array([srot, trot])


                    # calculate z
                    for elevationProfile in reversed(road.elevationProfile.elevations):
                        if s_pos >= elevationProfile.start_pos:
                            z_coeffs = elevationProfile.polynomial_coefficients
                            z_delta_s = s_pos - elevationProfile.start_pos
                            z = np.polynomial.polynomial.polyval(z_delta_s, z_coeffs)
                            break
                            

                    # upload this point
                    pre_offset = s_pos
                    res_position = (res_pos[0], res_pos[1], z)
                    lane_position.append(res_position)
                    #f.write(str(road._id)+','+str(lane._id)+','+ str(res_pos[0])+','+str(res_pos[1])+ ','+ str(z)+'\n')
                    #print (str(road._id)+','+str(lane._id)+','+ str(res_pos[0])+','+str(res_pos[1])+','+str(heading)+','+str(t))
                    
                lane_pos = LanePosition(lane, road, lane_position)
                LanePositionList.append(lane_pos)

    return LanePositionList


def lane_parser(_lane):
    """
        Return a list of lane object which contains all waypoints on the road.
        Args:
            _lane (file_path): The json file of the lane.
        Returns:
            list that refers to the lane object (class LanePosition)
    """

    # Parse map roads and junctions
    roads, junctions = get_roads_and_junctions(_lane)
    # get waypoint list
    lane_points_list = get_lane_waypoint(roads)

    return lane_points_list


if __name__ == "__main__":
    # Parse map roads and junctions
    roads, junctions = get_roads_and_junctions(sys.argv[1])
    # get waypoint list
    lane_points_list = get_lane_waypoint(roads)
    # lane_points_list = lane_parser("data_z.txt")

    # print all
    for lane_points in lane_points_list:
        for point in lane_points._lane_position:
            print(str(lane_points._parent_Road_id)+','+str(lane_points._lane_id)+','+str(point))
        print("---")
