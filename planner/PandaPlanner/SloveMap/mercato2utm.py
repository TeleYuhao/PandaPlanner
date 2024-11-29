'''
@Project :高精度地图
@Author : YuhaoDu
@Date : 2024/10/16 
'''
import numpy as np

from utils.opendrive2discretenet.opendriveparser.parser import parse_opendrive as parse_opendrive_xml
from utils.opendrive2discretenet.network import Network
from lxml import etree
import matplotlib.pyplot as plt
from net_struct import parse_opendrive
from pyproj import Proj,transform

utm   = Proj(proj='utm', zone=49, ellps='WGS84')
tmerc = Proj(proj="tmerc" ,lon_0=108.90577060170472, lat_0=34.37650478465651 ,ellps='WGS84')


def CoodernateTransform(road_info):
    for i in range(len(road_info.discretelanes)):
        center_traj = road_info.discretelanes[i].center_vertices
        road_info.discretelanes[i].center_vertices = tmerc2utm(center_traj)

        left_traj = road_info.discretelanes[i].left_vertices
        road_info.discretelanes[i].left_vertices = tmerc2utm(left_traj)

        right_traj = road_info.discretelanes[i].left_vertices
        road_info.discretelanes[i].left_vertices = tmerc2utm(right_traj)
    return road_info
def tmerc2utm(path):
    transformed_path = transform(tmerc, utm, path[:, 0], path[:, 1])
    return np.vstack((transformed_path[0],transformed_path[1])).T


if __name__ == '__main__':
    path_opendrive = 'chanan-v0.4-20241008.xodr'
    road_info = parse_opendrive(path_opendrive)
    road_info = CoodernateTransform(road_info)
    utm_traj = np.load("transform.npy")
    wgs84 = Proj(proj='latlong', datum='WGS84')



    for lane in road_info.discretelanes:
        center_traj = lane.center_vertices
        # center = tmerc2utm(center_traj)
        plt.plot(center_traj[:,0], center_traj[:,1],color='black')
    plt.plot(utm_traj[:,0],utm_traj[:,1])
    plt.show()


