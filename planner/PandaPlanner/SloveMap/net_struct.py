'''
@Project :高精度地图
@Author : YuhaoDu
@Date : 2024/10/12 
'''
from utils.opendrive2discretenet.opendriveparser.parser import parse_opendrive as parse_opendrive_xml
from utils.opendrive2discretenet.network import Network
from lxml import etree
import matplotlib.pyplot as plt
from utils.BFS import breadth_first_search

def parse_opendrive(path_opendrive: str) -> None:
    """
    解析opendrive路网的信息，存储到self.replay_info.road_info。
    """
    with open(path_opendrive, 'r', encoding='utf-8') as fh:
        root = etree.parse(fh).getroot()

    # 返回OpenDrive类的实例对象（经过parser.py解析）
    openDriveXml = parse_opendrive_xml(root)

    # 将OpenDrive类对象进一步解析为参数化的Network类对象，以备后续转化为DiscreteNetwork路网并可视化
    loadedRoadNetwork = Network()
    loadedRoadNetwork.load_opendrive(openDriveXml)

    """将解析完成的Network类对象转换为DiscreteNetwork路网，其中使用的只有路网中各车道两侧边界的散点坐标
        车道边界点通过线性插值的方式得到，坐标点储存在<DiscreteNetwork.discretelanes.left_vertices/right_vertices> -> List"""
    open_drive_info = loadedRoadNetwork.export_discrete_network(
        filter_types=["driving", "biking", "onRamp", "offRamp", "exit", "entry",
                      "sidewalk"])  # -> <class> DiscreteNetwork
    return open_drive_info

def plot_lane_center(road_info,index,color="black"):
    center = road_info.discretelanes[index].center_vertices
    plt.plot(center[:,0],center[:,1],color=color)

def plot_section(road_info,index,road_hash):
    plot_lane_center(road_info,index)
    lane_id = road_info.discretelanes[index].lane_id
    successor_id = road_info.discretelanes[index].successor
    presuccessor_id = road_info.discretelanes[index].predecessor
    for id in successor_id:
        plot_lane_center(road_info,road_hash[id])
    for id in presuccessor_id:
        plot_lane_center(road_info,road_hash[id])
    plt.show()

def FindNeighbor(road_map,road_section,road_hash,lane_id):
    total_road_id = road_map.discretelanes[lane_id].lane_id.split('.')
    road_id = total_road_id[0]
    lane_ind = total_road_id[2]
    neighbors = []
    for id in road_map.discretelanes[lane_id].successor:
        if(id not in road_hash.keys()): continue
        neighbors += [road_hash[id]]
    if(len(road_section[road_id]) == 1):
        return neighbors
    else:
        for info in road_section[road_id]:
            gap = abs(int(info[0]) - int(lane_ind))
            if(gap == 1 and gap != 0):
                neighbors.append(info[1])
        return  neighbors

def make_graph(road_info,road_section,road_hash):
    road_graph = {}
    for i in range(len(road_info.discretelanes)):
        road_graph[i] = FindNeighbor(road_info,road_section,road_hash,i)
    return road_graph



if __name__ == '__main__':
    path_opendrive = 'chanan-v0.4-20241008.xodr'
    road_info = parse_opendrive(path_opendrive)
    road_hash = {}
    road_section = {}
    # index 与 road_id的映射关系
    for i in range(len(road_info.discretelanes)):
        index = road_info.discretelanes[i].lane_id
        road_hash[index] = i
    # road_section:同一road下的section
    for i in range(len(road_info.discretelanes)):
        index = road_info.discretelanes[i].lane_id
        opendrive_str = index.split('.')
        if opendrive_str[0] in road_section.keys():
            road_section[opendrive_str[0]].append([opendrive_str[2],road_hash[index]])
        else:
            road_section[opendrive_str[0]] = [[opendrive_str[2],road_hash[index]]]

    road_graph = make_graph(road_info,road_section,road_hash)
    find_id = 151 # to be filled
    find_id = 354
    # find_id = 44
    neighbors = FindNeighbor(road_info, road_section, road_hash, find_id)
    path = breadth_first_search(road_graph,1,find_id)
    print(neighbors)
    for i in range(len(road_info.discretelanes)):
        if (i in path):
            plot_lane_center(road_info, i, color="red")
        else:
            plot_lane_center(road_info, i)
    plt.show()
    print(path)

    # find_index = 200
    # for find_index in range(200,455):
    #     print(find_index)
    #     neighbors = FindNeighbor(road_info,road_section,road_hash,find_index)
    #     for i in range(len(road_info.discretelanes)):
    #         if(i in neighbors or i == find_index):
    #         # if(i in path[:55]):
    #             plot_lane_center(road_info,i,color="red")
    #         else:
    #             plot_lane_center(road_info, i)
    #     plt.show()
    print("parse_success")
    # print(neighbors)


