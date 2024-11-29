"""
@Project :高精度地图
@Author : YuhaoDu
@Date : 2024/10/12
"""
import time

import numpy as np
import math
from utils.opendrive2discretenet.opendriveparser.parser import parse_opendrive as parse_opendrive_xml
from utils.opendrive2discretenet.network import Network
# from utils.BFS import breadth_first_search
from lxml import etree
import matplotlib.pyplot as plt
from utils.dijistra import dijkstra
import pyproj

wgs84 = pyproj.Proj(proj='latlong', datum='WGS84')
utm   = pyproj.Proj(proj='utm', zone=49, ellps='WGS84')
tmerc = pyproj.Proj(proj="tmerc", lon_0=108.90577060170472, lat_0=34.37650478465651, ellps='WGS84')

utm2tmerc   = pyproj.Transformer.from_proj( utm, tmerc)
tmerc2utm   = pyproj.Transformer.from_proj(tmerc, utm)
wgs842tmerc = pyproj.Transformer.from_proj( wgs84, tmerc)

class RoutePlanner:
    # def __init__(self, start_point,end_point ,path_opendrive='/media/pji/95F4-134F/1030/onsite-structured-test_with_PJPO/changan-v1.2-20241019.xodr'):
    # def __init__(self, start_point,end_point,path_opendrive='/media/wanji/95F4-134F/1030/onsite-structured-test_with_PJPO/changan-v1.2-20241019.xodr'):
    # def __init__(self, start_point,end_point,path_opendrive='/test/onsite-structured-test-master/PandaData/changan-v1.2-20241019.xodr'):
    def __init__(self, start_point,end_point,path_opendrive='PandaData/changan-v1.2-20241019.xodr'):
    # def __init__(self, start_point,end_point,path_opendrive='changan-v1.2-20241019.xodr'):
        self.path_opendrive = path_opendrive
        self.road_info = None
        self.road_hash = {}
        self.road_section = {}
        self.road_graph = {}
        self.last_lane_id = None
        self.init_function()
        self.make_graph()
        self.end_index = self.find_lane_id(end_point)
        self.route_id = np.array([])
        self.InitPlanning(start_point)
        self.LaneChange = 0
        self.RightTurningPath = [47]
        # self.ForbiddenPath = [403]
        self.ForbiddenPath = [56,53,128,219,234 ,6 ,203,209,83,88,113,48,90,239,309]

    def parse_opendrive(self):
        """解析opendrive路网的信息，存储到self.road_info。"""
        with open(self.path_opendrive, 'r', encoding='utf-8') as fh:
            root = etree.parse(fh).getroot()

        openDriveXml = parse_opendrive_xml(root)
        loadedRoadNetwork = Network()
        loadedRoadNetwork.load_opendrive(openDriveXml)

        open_drive_info = loadedRoadNetwork.export_discrete_network(
            filter_types=["driving", "biking", "onRamp", "offRamp", "exit", "entry",
                          "sidewalk"])
        open_drive_info = CoodernateTransform(open_drive_info)
        return open_drive_info

    def plot_lane_center(self, road_info, index, color="black"):
        center = road_info.discretelanes[index].center_vertices
        plt.plot(center[:, 0], center[:, 1], color=color)

    def plot_section(self, index):
        self.plot_lane_center(self.road_info, index)
        lane_id = self.road_info.discretelanes[index].lane_id
        successor_id = self.road_info.discretelanes[index].successor
        presuccessor_id = self.road_info.discretelanes[index].predecessor
        for id in successor_id:
            self.plot_lane_center(self.road_info, self.road_hash[id])
        for id in presuccessor_id:
            self.plot_lane_center(self.road_info, self.road_hash[id])
        plt.show()

    def FindNeighbor(self, road_map, lane_id):
        '''
        function: 寻找车道的相邻车道
        '''
        total_road_id = road_map.discretelanes[lane_id].lane_id.split('.')
        road_id = total_road_id[0]
        lane_ind = total_road_id[2]
        neighbors = []
        for id in road_map.discretelanes[lane_id].successor:
            if id not in self.road_hash:
                continue
            neighbors.append(self.road_hash[id])
        if len(self.road_section[road_id]) == 1:
            return neighbors
        else:
            for info in self.road_section[road_id]:
                gap = abs(int(info[0]) - int(lane_ind))
                if gap == 1:
                    neighbors.append(info[1])
            return neighbors
    def CheckTurning(self, Direction):
        '''
        function: 检查换道是否可行
        '''
        if Direction == -1:
            if self.road_info.discretelanes[self.last_lane_id].leftlane == '': # 如果左侧道路不存在
                return False
            else:
                return True
        else:
            if (self.road_info.discretelanes[self.last_lane_id].rightlane == '' and
                    self.road_info.discretelanes[self.last_lane_id].rightlane != '41.0.-3.-1'): # 如果右侧道路不存在，且右侧道路不为某一道路
                return False
            else:
                return True

    def find_lane_id(self, point):
        '''
        function:通过坐标点寻找最近的lane_id
        '''
        min_distance = float('inf')
        closest_lane_id = None
        point = np.array(point)

        for lane in self.road_info.discretelanes: # 遍历所有道路
            center_vertices = lane.center_vertices # 提取道路中心线
            distances = np.linalg.norm(center_vertices - point, axis=1) # 计算距离
            min_lane_distance = np.min(distances) # 提取最小距离

            if min_lane_distance < min_distance: # 如果比当前最小距离小，则更新道路id
                min_distance = min_lane_distance
                closest_lane_id = lane.lane_id

        return self.road_hash[closest_lane_id]
    def FindWithGuess(self,point):
        '''
        function：已知上一时刻的车道，寻找当前时刻的id
        '''
        min_distance = float('inf')
        closest_lane_id = None
        point = np.array(point)
        neighbor = self.FindNeighbor(self.road_info,self.last_lane_id)
        neighbor.append(self.last_lane_id)
        for lane_id in neighbor:
            center_vertices = self.road_info.discretelanes[lane_id].center_vertices
            distances = np.linalg.norm(center_vertices - point, axis=1)
            min_lane_distance = np.min(distances)

            if min_lane_distance < min_distance:
                min_distance = min_lane_distance
                closest_lane_id = lane_id
        return closest_lane_id, min_distance
    def dist2lane(self,lane_index,point):
        center_vertices = self.road_info.discretelanes[lane_index].center_vertices[::2]
        distances = np.linalg.norm(center_vertices - point, axis=1)
        return np.min(distances)
    def FindWithRoute(self, point):
        '''
        function:通过坐标点寻找最近的lane_id
        '''
        min_distance = float('inf')
        closest_lane_id = None
        point = np.array(point)

        for id in self.route_id:
            # center_vertices = self.road_info.discretelanes[id].center_vertices[::2]
            # distances = np.linalg.norm(center_vertices - point, axis=1)
            # min_lane_distance = np.min(distances)
            min_lane_distance = self.dist2lane(id,point)
            if min_lane_distance < min_distance:
                min_distance = min_lane_distance
                closest_lane_id = id

        # print("min_distance",min_distance)
        # print("current:",self.road_info.discretelanes[self.last_lane_id].lane_id)
        # print("left:",self.road_info.discretelanes[self.last_lane_id].leftlane,self.road_info.discretelanes[self.last_lane_id].leftlane in self.road_hash.keys())
        # print("right:",self.road_info.discretelanes[self.last_lane_id].rightlane,self.road_info.discretelanes[self.last_lane_id].rightlane in self.road_hash.keys())
        if min_distance > 2.5:
            dist2left,dist2right = 1e2, 1e2
            if self.road_info.discretelanes[self.last_lane_id].leftlane in self.road_hash.keys():
                left_index = self.road_hash[self.road_info.discretelanes[self.last_lane_id].leftlane]
                dist2left = self.dist2lane(left_index,point)
            if self.road_info.discretelanes[self.last_lane_id].rightlane in self.road_hash.keys():
                right_index = self.road_hash[self.road_info.discretelanes[self.last_lane_id].rightlane]
                dist2right = self.dist2lane(right_index,point)
            if dist2left < dist2right:
                closest_lane_id = self.road_hash[self.road_info.discretelanes[self.last_lane_id].leftlane]
                min_distance = dist2left
            elif dist2right > dist2left:
                closest_lane_id = self.road_hash[self.road_info.discretelanes[self.last_lane_id].rightlane]
                min_distance = dist2right
            else:
                closest_lane_id = self.find_lane_id(point)
            self.IndexPlanning(closest_lane_id)

        return closest_lane_id,min_distance

    def find_lane(self,point):
        if self.last_lane_id == None:
            self.last_lane_id = self.find_lane_id(point)
            return self.last_lane_id
        else:
            last_lane_id,min_distance = self.FindWithRoute(point)
            # self.last_lane_id,min_distance = self.FindWithGuess(point)
            if(min_distance > 59):
                print("min_distance", min_distance)
            else:
                self.last_lane_id = last_lane_id
                # print("last_lane_id:",self.last_lane_id)
                # self.last_lane_id = self.find_lane_id(point)
            return self.last_lane_id

    def calculate_center_distance(self, vertices1, vertices2):
        '''
        function：计算车道宽度
        '''
        distances = np.linalg.norm(vertices1[:, np.newaxis, :] - vertices2[np.newaxis, :, :], axis=2)
        return np.min(distances)

    def make_graph(self):
        '''
        funciton:构建车道之间的邻接矩阵
        '''
        for i in range(len(self.road_info.discretelanes)):
            neighbors = self.FindNeighbor(self.road_info, i)
            self.road_graph[i] = {}

            current_vertices = self.road_info.discretelanes[i].center_vertices

            for neighbor in neighbors:
                neighbor_vertices = self.road_info.discretelanes[neighbor].center_vertices
                if (self.road_info.discretelanes[neighbor].lane_id in self.road_info.discretelanes[i].successor or
                        self.road_info.discretelanes[neighbor].lane_id in self.road_info.discretelanes[i].predecessor):
                    self.road_graph[i][neighbor] = self.road_info.discretelanes[neighbor].length
                else:
                    center_distance = self.calculate_center_distance(current_vertices, neighbor_vertices)
                    self.road_graph[i][neighbor] = center_distance

        return self.road_graph  # 这行需要在 for 循环外

    def init_function(self):
        '''
        function:初始化地图信息
        '''
        self.road_info = self.parse_opendrive()
        # index 与 road_id的映射关系
        for i in range(len(self.road_info.discretelanes)):
            index = self.road_info.discretelanes[i].lane_id
            self.road_hash[index] = i
        # road_section:同一road下的section
        for i in range(len(self.road_info.discretelanes)):
            index = self.road_info.discretelanes[i].lane_id
            opendrive_str = index.split('.')
            if opendrive_str[0] in self.road_section:
                self.road_section[opendrive_str[0]].append([opendrive_str[2], self.road_hash[index]])
            else:
                self.road_section[opendrive_str[0]] = [[opendrive_str[2], self.road_hash[index]]]

    def DevideDirection(self,A,B,C):
        '''
        判断向量的前进方向，左侧为-1， 右侧为1
        '''
        res = (A[0] - C[0])*(B[1] - C[1]) - (A[1] - C[1])*(B[0] - C[0])
        if(res < 0):
            return 1
        else:
            return -1
    def InitPlanning(self,start):
        start_index = self.find_lane_id(start)
        self.route_id = np.array(dijkstra(self.road_graph, start_index,self.end_index))
    def IndexPlanning(self,start_index):
        self.route_id = np.array(dijkstra(self.road_graph, start_index,self.end_index))
    def DijkstraPlanning(self,start):
        start_index = self.find_lane(start)
        route_id = np.array(dijkstra(self.road_graph, start_index,self.end_index))
        intersect = [x for x in route_id if x in self.route_id]
        print("intersect_num:",len(intersect),"start_index:",start_index)
        if len(intersect) > 7 and abs(len(self.route_id) - len(route_id)) < 2:
            self.route_id = route_id

    def GetRefLaneSuccessor(self, point ,dist2end = 30):
        '''
        function: 计算参考线
        '''
        # self.DijkstraPlanning(point)
        lane_index = self.find_lane(point) # 获取当前道路id
        cur_lane = lane_index
        CurrentLane = self.road_info.discretelanes[lane_index] # 获取当前路段
        RefLane = CurrentLane.center_vertices # 获取当前路段参考线
        PathIds = [lane_index]
        PathLength = self.road_info.discretelanes[lane_index].length  # 使用 length 属性计算车道长度
        while PathLength < 2000 and len(CurrentLane.successor) > 0:
            # 更新 PathLength
            PathLength += self.road_info.discretelanes[lane_index].length
            SuccessorLanes = CurrentLane.successor

            if len(SuccessorLanes) == 1: #如果仅有一个后续路段，则证明仍然处于直线段
                if SuccessorLanes[0] not in self.road_hash.keys():
                    break
                lane_index = self.road_hash[SuccessorLanes[0]]
                CurrentLane = self.road_info.discretelanes[lane_index]
                next_Ref = CurrentLane.center_vertices
                RefLane = np.vstack((RefLane, next_Ref))
            elif len(SuccessorLanes) == 0: # 如果后续段为0，则证明无后续车道
                break
            else: # 如果有多个路段，则判断是否有共同id，如果有路段位于规划路径内，则选取规划路段继续向后沿伸，否则跳出循环
                Successor_id = [self.road_hash[i] for i in SuccessorLanes]
                common_id = list(set(self.route_id) & set(Successor_id))
                if len(common_id) == 1:
                    lane_index = common_id[0]
                    CurrentLane = self.road_info.discretelanes[lane_index]
                    next_Ref = CurrentLane.center_vertices
                    RefLane = np.vstack((RefLane, next_Ref))
                else:
                    break

            if self.road_hash[CurrentLane.lane_id] not in self.route_id:
                break
            PathIds.append(lane_index)
        # print("length of Path id",len(PathIds),"length of path:",PathLength)

        if self.route_id[-1] in PathIds: # 如果到达最后的路段，则无换道行为
            LaneChange = 0
        elif dist2end < 100:
            ind = np.argwhere(PathIds[-1] == self.route_id).flatten()
            if len(ind) == 0:
                LaneChange = self.LaneChange
            else:
                next_lane = self.route_id[ind[0] + 1]
                point_A = RefLane[0]
                point_B = RefLane[-1]
                point_C = self.road_info.discretelanes[next_lane].center_vertices[0]
                LaneChange = self.DevideDirection(point_A, point_B, point_C)
                if LaneChange == -1:
                    if self.road_info.discretelanes[self.route_id[ind][0]].leftlane == '' or cur_lane in self.ForbiddenPath:
                        LaneChange = 0
                elif LaneChange == 1 :
                    if self.road_info.discretelanes[self.route_id[ind][0]].rightlane == '' or cur_lane in self.ForbiddenPath:
                        # print("right_lane_forbidden:",len(self.road_info.discretelanes[self.ForbiddenPath[0]].center_vertices))
                        LaneChange = 0
        else:
            LaneChange = 0
        dist2start = math.hypot(point[0] - RefLane[0][0], point[1] - RefLane[0][1])
        if cur_lane in self.RightTurningPath or (cur_lane == 239 and dist2start > 8):
            LaneChange = 1

        # if LaneChange == -1:
        #     if self.road_info.discretelanes[lane_index].left_type > 2:  # 左车道为实线，禁止换道
        #         LaneChange = 0
        #
        # elif LaneChange == 1:
        #     if self.road_info.discretelanes[lane_index].right_type > 2:  # 右车道为实线，禁止换道
        #         LaneChange = 0

        print("PathLength:", PathLength, "LaneChange:", LaneChange,"cur_lane:",cur_lane,"dist2start",dist2start)
        self.LaneChange = self.CheckTurning(LaneChange)
        return RefLane, LaneChange, PathLength

def tmerc2utm(path): # 坐标系转换函数，由墨卡托坐标系转换至utm坐标系
    transformed_path = pyproj.transform(tmerc, utm, path[:, 0], path[:, 1])
    return np.vstack((transformed_path[0], transformed_path[1])).T

def utm2tmercPoint(point):
    '''
    function: transform the umt point to tmerc point
    '''
    transformed_point = utm2tmerc.transform(point[0],point[1])
    return transformed_point
u = pyproj.Transformer.from_proj(tmerc, utm)
def tmerc2utmPoint(point):
    '''
    function: transform the umt point to tmerc point
    '''
    transformed_point = u.transform(point[0],point[1])
    return transformed_point

def Utm2TmercPath(path):
    '''
        function: transform the umt path to tmerc point
    '''
    transformed_path = utm2tmerc.transform(path[:, 0], path[:, 1])
    return np.vstack((transformed_path[0], transformed_path[1])).T

def WGS842TmercPath(path):
    transformed_path = wgs842tmerc.transform(path[:, 0], path[:, 1])
    return np.vstack((transformed_path[0], transformed_path[1])).T

def CoodernateTransform(road_info):
    for i in range(len(road_info.discretelanes)):
        center_traj = road_info.discretelanes[i].center_vertices
        road_info.discretelanes[i].center_vertices = WGS842TmercPath(center_traj)

        # left_traj = road_info.discretelanes[i].left_vertices
        # road_info.discretelanes[i].left_vertices = WGS842TmercPath(left_traj)
        #
        # right_traj = road_info.discretelanes[i].left_vertices
        # road_info.discretelanes[i].left_vertices = WGS842TmercPath(right_traj)
    return road_info

def ObservationCoodernateTransform(observation):
    '''
    function: change the coodernate of observation during planning
    '''
    ego_point_tmerc = utm2tmercPoint([observation.ego_info.x, observation.ego_info.y])
    observation.ego_info.x, observation.ego_info.y = ego_point_tmerc[0], ego_point_tmerc[1]
    for type,data in observation.object_info.items():
        for id,value in data.items():
            obs_point_tmerc = utm2tmercPoint([value.x, value.y])
            value.x, value.y = obs_point_tmerc[0],obs_point_tmerc[1]
    return observation


if __name__ == '__main__':
    start = [-583.65,-127.68]
    # start = [-1164.54,-452.25]
    # start = [-1152.70,-443.31]
    end = [-562.72871498, -156.63442184]
    RP = RoutePlanner(start,end)

    mid = [-803.58736701, -295.23826449]
    transform_file = r'sparce_traj.npy'
    traj = np.load(transform_file)  # 载入文件坐标（这里是utm坐标或者经纬度）
    # 使用 Transformer 进行经纬度转 UTM
    merc_traj = WGS842TmercPath(traj)[::3]
    RP.DijkstraPlanning(start)
    # for i in range(len(merc_traj)):
    #     RP.DijkstraPlanning(merc_traj[i])
    #     for lane in RP.road_info.discretelanes:
    #         plt.plot(lane.center_vertices[:,0], lane.center_vertices[:,1],color="black")
    #     for id in RP.route_id:
    #         plt.plot(RP.road_info.discretelanes[id].center_vertices[:,0],RP.road_info.discretelanes[id].center_vertices[:,1],color="red")
    #     plt.show()
    # for id in RP.route_id:

    #     # Ref,LaneChange,PathLength = RP.GetRefLaneSuccessor(id)
    #     for lane in RP.road_info.discretelanes:
    #         plt.plot(lane.center_vertices[:,0], lane.center_vertices[:,1],color="black")
    #     Ref = RP.road_info.discretelanes[403].center_vertices
    #     pre = RP.road_info.discretelanes[403].predecessor
    #     next = RP.road_info.discretelanes[403].successor
    #     RefNext = RP.road_info.discretelanes[RP.road_hash[next[0]]].center_vertices
    #     Refpre = RP.road_info.discretelanes[RP.road_hash[pre[0]]].center_vertices
    #     print(len(Ref))
    #     plt.scatter(Ref[:,0],Ref[:,1],color="red")
    #     plt.scatter(Ref[0][0],Ref[0][1],color="blue")
    #     plt.scatter(RefNext[:,0],RefNext[:,1],color="green")
    #     # plt.scatter(RP.road_info.discretelanes[RP.route_id[18]].center_vertices[:,0],RP.road_info.discretelanes[RP.route_id[18]].center_vertices[:,1],color="green")
    #     plt.show()
    for lane in RP.road_info.discretelanes:
        plt.plot(lane.center_vertices[:,0], lane.center_vertices[:,1],color="black")
    # for id in RP.route_id[4:5]:
    #     plt.plot(RP.road_info.discretelanes[id].center_vertices[:, 0], RP.road_info.discretelanes[id].center_vertices[:, 1], color="red")
    plt.plot(RP.road_info.discretelanes[309].center_vertices[:, 0], RP.road_info.discretelanes[309].center_vertices[:, 1], color="red")
    plt.plot(RP.road_info.discretelanes[239].center_vertices[:, 0], RP.road_info.discretelanes[239].center_vertices[:, 1], color="red")
    plt.plot(RP.road_info.discretelanes[47].center_vertices[:, 0], RP.road_info.discretelanes[47].center_vertices[:, 1], color="red")
    # id = RP.route_id[0]
    # plt.plot(RP.road_info.discretelanes[id].center_vertices[:, 0], RP.road_info.discretelanes[id].center_vertices[:, 1],
    #          color="red")
    # plt.scatter(merc_traj[0], merc_traj[1], color='orange', s=10)
    plt.show()


    RP.DijkstraPlanning(start)
    print(RP.route_id)
