from opendrive2lanelet.opendriveparser import parser as opdParser
from opendrive2lanelet.opendriveparser.elements.junction import Junction
from opendrive2lanelet.opendriveparser.elements.road import Road
from opendrive2lanelet.opendriveparser.elements.roadLanes import Lane
from typing import Tuple, List, Set, Dict
from math import floor,asin

from shapely.geometry import Point, Polygon
from lxml import etree
import numpy as np
import pickle
import sys
import os

def get_roads_and_junctions(path : str) -> Tuple[Set[Road], List[Junction]]:
    parser = etree.XMLParser()
    with open(path, 'r') as file:
        root_node = etree.parse(file, parser).getroot()
    road_network = opdParser.parse_opendrive(root_node)
    return road_network.roads, road_network.junctions

class LanePosition():
    _lane_position: List[Tuple]

    def __init__(self, lane: Lane, road: Road, lane_position: List[Tuple] ):
        self._parent_Road = road
        self._parent_Road_id = road._id
        self._lane_id = lane.id
        self._lane_position = lane_position
        self._junction = road._junction

    def __init__(self, lane_id, road: Road, lane_position: List[Tuple] ):
        self._parent_Road = road
        self._parent_Road_id = road._id
        self._lane_id = lane_id
        self._lane_position = lane_position
        self._junction = road._junction

class RoadLane():

    def __init__(self, road: Road, ref_line: List[LanePosition],left_lanes:List[LanePosition],right_lanes:List[LanePosition]):
        self._parent_Road = road
        self._parent_Road_id = road._id
        self._ref_line = ref_line
        self._left_lanes = left_lanes
        self._right_lanes = right_lanes
        self._junction = road._junction

    def maxmin(self) -> Tuple:
        position: List[Point] = []
        for lane in self._left_lanes:
            position.append(lane._lane_position[0])
            position.append(lane._lane_position[-1])
        for lane in self._right_lanes:
            position.append(lane._lane_position[0])
            position.append(lane._lane_position[-1])
        for lane in self._ref_line:
            position.append(lane._lane_position[0])
            position.append(lane._lane_position[-1])
        posx = [xypos[0] for xypos in position]
        posy = [xypos[1] for xypos in position]
        res = (min(posx),max(posx),min(posy),max(posy))
        return res

    def get_point(self) -> List[Tuple]:
        left_pos = []
        right_pos = []
        res = []
        if self._left_lanes:
            left_pos = self._left_lanes[-1]._lane_position
        else:
            left_pos = self._ref_line[0]._lane_position 

        if self._right_lanes:
            right_pos = self._right_lanes[-1]._lane_position
        else:
            right_pos = self._ref_line[0]._lane_position

        left_pos.reverse()
        res = left_pos+right_pos
        left_pos.reverse()
        return res



def get_lane_waypoint(roads: Set[Road]) -> List[RoadLane]:
    #f = open('CHN_Cho-2-2.txt','w')
    #f.write('road_id,lane_id,x,y,z\n') # y equals z in simultor

    # first loop ———— road, contains one reference line and several lanes
    #LanePositionList: list[LanePosition] = []
    #RoadPostion : RoadLane = []
    RoadPositionList: list[RoadLane] = []
    for road in roads:

        RefLanePositionList: list[LanePosition] = []
        road_points: List[Point] = []
        resolution: int = 10 
        for i in range(0, floor(road.planView.length * resolution)):
            current_point = road.planView.calc(i / resolution)[0]
            curpoint = (current_point[0], current_point[1]) 
            road_points.append(curpoint)
            #f.write(str(road._id)+','+str(0)+','+ str(current_point.x)+','+str(current_point.y)+ ','+ str(0)+'\n')
        last_point = (road.planView.calc(road.planView.length)[0][0],road.planView.calc(road.planView.length)[0][1])
        road_points.append(last_point)  
        lane_pos = LanePosition(0, road, road_points)
        RefLanePositionList.append(lane_pos)

        for lane_section in road.lanes.lane_sections:
            
            pre_width = [0] * 4096
            # second loop  —————— lane, use width to calculate position

            LeftLanePositionList: list[LanePosition] = []
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
                    t = pre_width[i] 
                    i = i + 1   

                    # t = pre_width[i] use half width to get the center line of lane
                    srot = - t * np.sin(heading)
                    trot = t * np.cos(heading)

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
                    res_position = (res_pos[0], res_pos[1])
                    lane_position.append(res_position)
                    #f.write(str(road._id)+','+str(lane._id)+','+ str(res_pos[0])+','+str(res_pos[1])+ ','+ str(z)+'\n')
                    #print (str(road._id)+','+str(lane._id)+','+ str(res_pos[0])+','+str(res_pos[1])+','+str(heading)+','+str(t))

                lane_pos = LanePosition(lane, road, lane_position)
                LeftLanePositionList.append(lane_pos)

            RightLanePositionList: list[LanePosition] = []
            pre_width = [0] * 4096
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
                    t = pre_width[i]
                    i = i + 1   

                    # t = pre_width[i] use half width to get the center line of lane
                    srot = - t * np.sin(heading)
                    trot = t * np.cos(heading)

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
                    res_position = (res_pos[0], res_pos[1])
                    lane_position.append(res_position)
                    #f.write(str(road._id)+','+str(lane._id)+','+ str(res_pos[0])+','+str(res_pos[1])+ ','+ str(z)+'\n')
                    #print (str(road._id)+','+str(lane._id)+','+ str(res_pos[0])+','+str(res_pos[1])+','+str(heading)+','+str(t))
                    
                lane_pos = LanePosition(lane, road, lane_position)
                RightLanePositionList.append(lane_pos)

            RoadPostion = RoadLane(road, RefLanePositionList,LeftLanePositionList,RightLanePositionList) 
            RoadPositionList.append(RoadPostion)

    return RoadPositionList

def get_lane_waypoint2(roads: Set[Road]) -> List[RoadLane]:
    #f = open('CHN_Cho-2-2.txt','w')
    #f.write('road_id,lane_id,x,y,z\n') # y equals z in simultor

    # first loop ———— road, contains one reference line and several lanes
    #LanePositionList: list[LanePosition] = []
    #RoadPostion : RoadLane = []
    RoadPositionList: list[RoadLane] = []
    for road in roads:

        RefLanePositionList: list[LanePosition] = []
        road_points: List[Point] = []
        resolution: int = 10 
        for i in range(0, floor(road.planView.length * resolution)):
            current_point = road.planView.calc(i / resolution)[0]
            curpoint = (current_point[0], current_point[1]) 
            road_points.append(curpoint)
            #f.write(str(road._id)+','+str(0)+','+ str(current_point.x)+','+str(current_point.y)+ ','+ str(0)+'\n')
        last_point = (road.planView.calc(road.planView.length)[0][0],road.planView.calc(road.planView.length)[0][1])
        road_points.append(last_point)  
        lane_pos = LanePosition(0, road, road_points)
        RefLanePositionList.append(lane_pos)

        for lane_section in road.lanes.lane_sections:
            
            pre_width = [0] * 1024
            # second loop  —————— lane, use width to calculate position

            LeftLanePositionList: list[LanePosition] = []
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
                    t = pre_width[i] 
                    i = i + 1   

                    # t = pre_width[i] use half width to get the center line of lane
                    #srot = delta_s * np.cos(heading) - t * np.sin(heading)
                    #trot = delta_s * np.sin(heading) + t * np.cos(heading)
                    srot = - t * np.sin(heading)
                    trot = t * np.cos(heading)

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
                    res_position = (res_pos[0], res_pos[1])
                    lane_position.append(res_position)
                    #f.write(str(road._id)+','+str(lane._id)+','+ str(res_pos[0])+','+str(res_pos[1])+ ','+ str(z)+'\n')
                    #print (str(road._id)+','+str(lane._id)+','+ str(res_pos[0])+','+str(res_pos[1])+','+str(heading)+','+str(t))

                lane_pos = LanePosition(lane, road, lane_position)
                LeftLanePositionList.append(lane_pos)

            RightLanePositionList: list[LanePosition] = []
            pre_width = [0] *1024
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
                    t = pre_width[i]
                    i = i + 1   

                    # t = pre_width[i] use half width to get the center line of lane
                    #srot = delta_s * np.cos(heading) - t * np.sin(heading)
                    #trot = delta_s * np.sin(heading) + t * np.cos(heading)
                    srot = - t * np.sin(heading)
                    trot = t * np.cos(heading)

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
                    res_position = (res_pos[0], res_pos[1])
                    lane_position.append(res_position)
                    #f.write(str(road._id)+','+str(lane._id)+','+ str(res_pos[0])+','+str(res_pos[1])+ ','+ str(z)+'\n')
                    #print (str(road._id)+','+str(lane._id)+','+ str(res_pos[0])+','+str(res_pos[1])+','+str(heading)+','+str(t))
                    
                lane_pos = LanePosition(lane, road, lane_position)
                RightLanePositionList.append(lane_pos)

            RoadPostion = RoadLane(road, RefLanePositionList,LeftLanePositionList,RightLanePositionList) 
            RoadPositionList.append(RoadPostion)

    return RoadPositionList
def is_pt_in_poly(pt, poly):#pt是被判断点，poly是多边形
    '''判断点是否在多边形内部的(pnpoly 算法)
    '''
    nvert = len(poly) #围成多边形区域的所有点的数量
    vertx = [] #围成多边形区域的所有点的x坐标
    verty = [] #围成多边形区域的所有点的y坐标
    testx = pt[0] #被判断点的x坐标
    testy = pt[1] #被判断点的y坐标
    for item in poly: #循环poly内的所有点
        vertx.append(item[0]) #将所有点的x坐标都放到vertx中
        verty.append(item[1]) #将所有点的y坐标都放到verty中
 
    j = nvert - 1 # 第一个j是nvert中最后一个的数，本例中第一次j=5-1=4
    res = False #判断结果值
    for i in range(nvert): # i分别是0、1、2、3、4
        if (verty[j] - verty[i]) == 0:
            j = i
            continue
        #**（下图解释）
        x = (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i] 
        #判断 testy不同时小于多边形相邻两点i与j的y坐标 与 testx小于x（下图有解释）是否同时为True
        if ((verty[i] > testy) != (verty[j] > testy)) and (testx < x):
            res = not res #如果在外面一侧，可以否定之否定
        j = i #j分别是4、0、1、2、3，最后还有个4
    return res

def is_point_in_polygon(pointTp, verts):
        """
        - PNPoly算法:判断点是否在不规则区域内（在边界线上也算）
        """
        #is_contains_edge=True
        x,y=pointTp[0],pointTp[1]
        try:
            x, y = float(x), float(y)
        except:
            return False
        vertx = [xyvert[0] for xyvert in verts]
        verty = [xyvert[1] for xyvert in verts]
    
        # N个点中，横坐标和纵坐标的最大值和最小值，判断目标坐标点是否在这个外包四边形之内
        if not verts or not min(vertx) <= x <= max(vertx) or not min(verty) <= y <= max(verty):
            return False
    
        # 上一步通过后，核心算法部分
        nvert = len(verts)
        is_in = False
        for i in range(nvert):
            if i == 0:
                j = nvert - 1
            else:
                j = i-1
            if ((verty[i] > y) != (verty[j] > y)) and (x < (vertx[j] - vertx[i]) * (y - verty[i]) / (verty[j] - verty[i]) + vertx[i]):
                is_in = not is_in
    
        return is_in   


def check_in_road(roadlanes: List[RoadLane], position:Tuple):
    #if position[2] < -5:
    #    return False
    check_p = (position[0],position[1])
    print("+++ roadlanes length:",len(roadlanes))
    for road_line in roadlanes:
        res = road_line.maxmin()
        # 在外包四边形内
        if res[0]<= check_p[0] <= res[1] and res[2]<= check_p[1] <= res[3] :
            shape_points = road_line.get_point()
            if is_pt_in_poly(check_p, shape_points):
                return True,road_line._parent_Road_id
            #if is_point_in_polygon(check_p, shape_points):
                #return True

    return False,None

def get_wheel_position(position:Tuple, rotation:Tuple)->List[Tuple]:
    wheel : List[Tuple] = []
    length = 2.835 / 2
    width = 1.66 / 2
    l2 = length * length + width * width
    l = np.sqrt(l2)
    sina = width / l
    cosa = length / l
    degree = np.radians(rotation[1])
    sinb = np.sin(degree)
    cosb = np.cos(degree)
    wheel1_x = position[0] + l * sina * cosb + l * sinb * cosa
    wheel1_y = position[1] + l * cosa * cosb - l * sina * sinb
    wheel1 = (wheel1_x,wheel1_y,position[2])
    wheel.append(wheel1)
    wheel2_x = position[0] + l * cosb * cosa + l * sinb * sina
    wheel2_y = position[1] + l * sinb * cosa - l * sina * cosb
    wheel2 = (wheel2_x,wheel2_y,position[2])
    wheel.append(wheel2)
    wheel3_x = position[0] - l * sina * cosb - l * sinb * cosa
    wheel3_y = position[1] - l * cosa * cosb + l * sina * sinb
    wheel3 = (wheel3_x,wheel3_y,position[2])
    wheel.append(wheel3)
    wheel4_x = position[0] - l * cosb * cosa - l * sinb * sina
    wheel4_y = position[1] - l * sinb * cosa + l * sina * cosb
    wheel4 = (wheel4_x,wheel4_y,position[2])
    wheel.append(wheel4)

    return wheel

def get_wheel_position_v2(position:Tuple, rotation:Tuple)->List[Tuple]:
    wheel : List[Tuple] = []
    length = 2.835 / 2
    width = 1.66 / 2
    l2 = length * length + width * width
    l = np.sqrt(l2)
    sina = width / l
    degree = np.radians(rotation[1])

    from math import asin,pi
    a = asin(sina)
    b = degree
    x,y,z = tuple(position)

    def to_wheel(x,y,z):
        return (x,y,z)

    w1_x = l * np.sin(a+b) + x
    w1_y = l * np.cos(a+b) + y
    wheel.append(to_wheel(w1_x,w1_y,z))
    w2_x = l * np.sin(b-a) + x
    w2_y = l * np.cos(b-a) + y
    wheel.append(to_wheel(w2_x,w2_y,z))
    w3_x = l * np.sin(b+pi - a) + x
    w3_y = l * np.cos(b+pi - a) + y
    wheel.append(to_wheel(w3_x,w3_y,z))
    w4_x = l * np.sin(b+pi + a) + x
    w4_y = l * np.cos(b+pi + a) + y
    wheel.append(to_wheel(w4_x,w4_y,z))

    return wheel

def get_lane_points_list(mappath):
    roads, junctions = get_roads_and_junctions(mappath)
    lane_points_list = get_lane_waypoint(roads)
    return lane_points_list

def CHECK_IN_ROAD(lane_points_list, transform):
    # roads, junctions = get_roads_and_junctions(mappath)
    # lane_points_list = get_lane_waypoint(roads)
    point = (transform.position.x,transform.position.z,transform.position.y)
    rotation = (transform.rotation.x,transform.rotation.y,transform.rotation.z)
    print(point,rotation)
    flag = True
    if point[2] > -5:
        #wheel_ps = get_wheel_position(point,rotation)
        flag,roadid = check_in_road(lane_points_list,point)
        #if flag == False:
        #    for wheel_p in wheel_ps:
        #        flag = check_in_road(lane_points_list,wheel_p)
        #        if flag == True:
        #            break
    else:
        print("z axis < -5!")
        flag = False
        roadid = None
    return flag,roadid

if __name__ == "__main__":
    map_name = sys.argv[1]
    # load the map
    #dir_map = "/home/vm1/final_pose_check/DONE/"+ map_name + "/lane/"
    #map_file = os.listdir(dir_map)
    #map_file_name = str(dir_map + map_file[0])
    map_file_name = sys.argv[1]
    roads, junctions = get_roads_and_junctions(map_file_name)
    lane_points_list = get_lane_waypoint(roads)
    # result
    #result_name = "/home/vm1/final_pose_check/result_wide/DONE/"+map_name+".csv"
    #f = open(result_name,'w')
    #f.write('id,is_on_lane\n')
    # get all final pose
    #dir = "/home/vm1/final_pose_check/DONE/"+ map_name + "/final_pose"
    #files = os.listdir(dir)
    # open and check
    files = [sys.argv[2]]
    for file in files:
        #check = "/home/vm1/final_pose_check/DONE/"+ map_name + "/final_pose/" + file
        f_check = open(file,"rb")
        obj = pickle.load(f_check)
        #point = (obj.transform.position.x,obj.transform.position.z,obj.transform.position.y)
        #rotation = (obj.transform.rotation.x,obj.transform.rotation.y,obj.transform.rotation.z)
        point = (obj.position.x,obj.position.z,obj.position.y)
        rotation = (obj.rotation.x,obj.rotation.y,obj.rotation.z)
        flag = True
        if point[2] > -5:
            wheel_ps = get_wheel_position(point,rotation)
            flag,_ = check_in_road(lane_points_list,point)
            if flag == False:
                for wheel_p in wheel_ps:
                    flag,_ = check_in_road(lane_points_list,wheel_p)
                    if flag == True:
                        break
        else:
            flag = False
        res = file.replace('.obj','',1) + "," + str(flag) + "\n"
        print(res)
        print(CHECK_IN_ROAD(map_file_name,obj))
        #f.write(res)
            
