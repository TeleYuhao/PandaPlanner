#!/usr/bin/env python
# -*- coding: utf-8 -*-
#import lib
import matplotlib.pyplot as plt
import numpy as np
import math
from utils.observation import Observation
from utils.opendrive2discretenet import parse_opendrive
from pyproj import Proj
import xml.etree.ElementTree as ET
import time

class IDM():
    def __init__(self, a_bound=2.9, exv=40, t=3.0, a=2.22, b=0.5, gama=4, s0=10.0, s1=10.0, aacc_dece = 0.8):
        """跟idm模型有关的模型参数
        :param a_bound: 本车加速度绝对值的上下界
        :param exv: 期望速度
        :param t: 反应时间
        :param a: 起步加速度
        :param b: 舒适减速度
        :param gama: 加速度指数
        :param s0: 静止安全距离
        :param s1: 与速度有关的安全距离选择参数
        """
        self.a_bound = a_bound
        self.exv = exv
        self.t = t
        self.a = a
        self.b = b
        self.gama = gama
        self.s0 = s0
        self.s1 = s1
        self.s_ = 0
        self.index = 0
        self.history_acc_dece = 0   # history_acc_dece
        self.aacc_dece = aacc_dece

        self.all_line = []          # [[x, y, heading(deg), speed(km/h)], []]
        self.ref_line = []          # [[x, y, heading(deg), speed(m/s)], []]
        self.planning_line = []     # [[x, y, heading(deg), speed(m/s)], []]
        self.lonlat2xy = Proj(proj='utm', zone=49, ellps='WGS84', preserve_units='m')  # 西安 zone 49， 北京 zone 50
        self.map = []               # [{'mid' : [lon, lat], 'left' : [lon, lat], 'right' : [lon, lat]}, {}]
        self.all_line_xodr = []     # [[x, y, heading(deg), speed(km/h)], []]
        self.xodr_map = 0           # xodr轨迹有效标志位
        self.use_xodr_flag = 1      # 使用xodr标志位，0-not/1-use
        self.light = 0              # 转向灯，0-nan/1-right/2-left/4-double
        self.use_history_road = 0   # 使用历史轨迹标志
        self.history_line = []      # [[utmx, utmy, heading(deg), speed(km/h)], []]
        self.test_data = []
        self.dev = 0
        self.ego = []               # [utmx, utmy, heading, speed, lon, lat]
        self.stop_time = 0
        self.stop_line_count = 0

        self.road_id_map = {}

    def init(self, scenario_dict):
        print("---------------------------- IDM INIT ----------------------------")
        # print(scenario_dict)
        self.calmap(scenario_dict)
        #
        # if self.use_xodr_flag == 1:
        #     print('load xodr')
        #     self.calmap(scenario_dict['source_file']['xodr'])
        print("----------------------------------------------------------------")


    def act(self, observation: Observation):
        return [1, 2]

    
    


    def calmap(self, path):
        """read xodr to lonlat"""
        # 打开并解析OpenDRIVE文件
        tree = ET.parse(path)
        root = tree.getroot()
        # 找到header元素
        header = root.find('header')
        # 找到geoReference元素
        geo_reference = header.find('geoReference')
        # 获取geoReference元素的文本内容
        proj_string = geo_reference.text
        # 打印投影信息
        proj_string = proj_string.replace(" +geoidgrids=egm96_15.gtx", "")
        # 定义投影坐标系
        projection = Proj(proj_string)
        # 读xodr
        road_info = parse_opendrive(path)  # 局部坐标xy
        temp_id = 0
        self.map = []
        lane = road_info.discretelanes[0]
        for _ in road_info.discretelanes:
            if (_.lane_id == lane.lane_id  ):
                plt.plot(_.center_vertices[:, 0], _.center_vertices[:, 1],label="lane")
            if( _.lane_id in lane.predecessor):
                plt.plot(_.center_vertices[:, 0], _.center_vertices[:, 1], label="predecessor")
            if( _.lane_id in lane.successor):
                plt.plot(_.center_vertices[:, 0], _.center_vertices[:, 1], label="successor")
        plt.legend()
        plt.show()

        for i in road_info.discretelanes:
            temp_id += 1
            temp_dic = {'id' : temp_id, 'mid' : [], 'left' : [], 'right' : [], 'next_id' : []}

            road_id_str = i.lane_id.split('.')
            if road_id_str[0] in self.road_id_map.keys():
                # {road_id} = [lane_id, lane_index]
                self.road_id_map[road_id_str[0]].append([road_id_str[2],temp_id])
            else:
                self.road_id_map[road_id_str[0]] = []
                self.road_id_map[road_id_str[0]].append([road_id_str[2],temp_id])
            # print(self.road_id_map['84'])
            temp_dic['road_id'] = [road_id_str[0],road_id_str[2]]
            for j in i.center_vertices:
                # 输入xy坐标
                x = j[0]
                y = j[1]
                # 转换为经纬度
                lon, lat = projection(x, y, inverse=True)
                temp_dic['mid'].append([lon, lat])
            for j in i.left_vertices:
                # 输入xy坐标
                x = j[0]
                y = j[1]
                # 转换为经纬度
                lon, lat = projection(x, y, inverse=True)
                temp_dic['left'].append([lon, lat])
            for j in i.right_vertices:
                # 输入xy坐标
                x = j[0]
                y = j[1]
                # 转换为经纬度
                lon, lat = projection(x, y, inverse=True)
                temp_dic['right'].append([lon, lat])
            self.map.append(temp_dic)
        for i in self.map:
            if len(i['mid']) == 1:
                i['mid'][0].append(0)
            for xx in range(len(i['mid'])):
                if xx < len(i['mid']) - 1:
                    temp_utm_x1, temp_utm_y1 = self.lonlat2xy(i['mid'][xx][0], i['mid'][xx][1], inverse=False)
                    temp_utm_x2, temp_utm_y2 = self.lonlat2xy(i['mid'][xx+1][0], i['mid'][xx+1][1], inverse=False)
                    # print('temp_utm_x1,temp_utm_x1:',temp_utm_x1, temp_utm_y1)
                    temp_angle = math.atan2((temp_utm_y2 - temp_utm_y1), (temp_utm_x2 - temp_utm_x1)) / math.pi * 180
                    if temp_angle < 0:
                        temp_angle += 360
                    elif temp_angle > 360:
                        temp_angle -= 360
                    i['mid'][xx].append(temp_angle)
                else:
                    i['mid'][xx].append(i['mid'][xx-1][2])
            for j in self.map:
                if j['id'] == i['id']:
                    continue
                temp_dis = self.callonlatdis(i['mid'][-1][0], i['mid'][-1][1], j['mid'][0][0], j['mid'][0][1], 0)
                if temp_dis < 1.0:
                    i['next_id'].append(j['id'])
        for lane in self.map:
            lane['mid']   = np.array(lane['mid'])
            lane['left']  = np.array(lane['left'])
            lane['right'] = np.array(lane['right'])
            # print(i['id'], ', list: ', i['next_id'])
        # print(self.map)

    
    def XodrLane2Utm(self):
        if len(self.all_line_xodr) != 0:
            print('ros lane len: ', len(self.all_line), ', xodr lane len: ', len(self.all_line_xodr))
            self.all_line = []
            for i in self.all_line_xodr:
                temp_x, temp_y = self.lonlat2xy(i[0], i[1], inverse=False)
                self.all_line.append([temp_x, temp_y, i[2], i[3], i[4]])
            print('xodr line ready')
            self.xodr_map = 1
        else:
            print('no xodr lane')
            self.xodr_map = 0

    def callonlatdis(self, lon1, lat1, lon2, lat2, type):
        """cal lonlat dis"""
        temp_x1, temp_y1 = lon1, lat1
        if type == 0:
            temp_x1, temp_y1 = self.lonlat2xy(lon1, lat1, inverse=False)
        temp_x2, temp_y2 = self.lonlat2xy(lon2, lat2, inverse=False)
        temp_dis = math.sqrt((temp_x1 - temp_x2) ** 2 + (temp_y1 - temp_y2) ** 2)
        return temp_dis

    
    def cal_angle(self, _t, controlP_x, controlP_y):
        _dx_1 = 3 * controlP_x[0] * _t * _t
        _dx_2 = 3 * controlP_x[1] * (_t * 2 - 3 * _t * _t)
        _dx_3 = 3 * controlP_x[2] * (1 - 4 * _t + 3 * _t * _t)
        _dx_4 = -3 * controlP_x[3] * (1 - _t) * (1 - _t)
        _dy_1 = 3 * controlP_y[0] * _t * _t
        _dy_2 = 3 * controlP_y[1] * (_t * 2 - 3 * _t * _t)
        _dy_3 = 3 * controlP_y[2] * (1 - 4 * _t + 3 * _t * _t)
        _dy_4 = -3 * controlP_y[3] * (1 - _t) * (1 - _t)
        return math.atan2(_dy_1 + _dy_2 + _dy_3 + _dy_4, _dx_1 + _dx_2 + _dx_3 + _dx_4)

    def XYToLonLat(self, point):
        """car xy to lonlat"""
        R_e = 6378137.0
        R_f = 6356752.314245
        e_1 = math.sqrt(pow(R_e, 2) - pow(R_f, 2)) / R_e

        sin_lat1 = math.sin(self.ego[5] * math.pi / 180.0)
        cos_lat1 = math.cos(self.ego[5] * math.pi / 180.0)
        square_e = e_1 * e_1
        square_sin_lat1 = sin_lat1 * sin_lat1
        R_n = R_e / (math.sqrt(1 - square_e * square_sin_lat1))
        R_m = (R_n * (1 - square_e)) / (1 - square_e * square_sin_lat1)
        temp_angle = self.ego[2] * math.pi / 180
        c = math.cos(temp_angle)
        s = math.sin(temp_angle)
        gx = (point[0] * c + point[1] * s)
        gy = (point[1] * c - point[0] * s)

        lat = gy / math.pi * 180 / R_m + self.ego[5]
        lon = gx / math.pi * 180.0 / R_n / cos_lat1 + self.ego[4]
        return [lon, lat, point[2], point[3], point[4]]



    def CalLight(self):
        """cal light"""
        if len(self.planning_line) == 0:
            return
        index = min(40, len(self.planning_line))
        point_heading_0 = self.planning_line[0][2]
        point_heading_1 = self.planning_line[index - 1][2]
        det_angle = point_heading_1 - point_heading_0
        if det_angle > 180:
            det_angle -= 360
        elif det_angle  < -180:
            det_angle += 360
        if np.abs(det_angle) < 15:
            return
        if det_angle > 0:
            self.light = 2
        else:
            self.light = 1


def FindNeighbor(road_map,road_hash,lane_id):
    neighbor_index = road_map[lane_id]['next_id']
    road_id,lane_index = road_map[lane_id]['road_id'][0],road_map[lane_id]['road_id'][1]
    if(len(road_hash[road_id]) == 1):
        return neighbor_index
    else:
        for info in road_hash[road_id]:
            if(abs(int(info[0]) - int(lane_index)) == 1):
                neighbor_index.append(info[1])
        return  neighbor_index



if __name__ == '__main__':
    planner = IDM()
    path = "chanan-v0.4-20241008.xodr"
    planner.init(path)
    # neighbor  = FindNeighbor(planner.map, planner.road_id_map, 14)
    index = 140
    neighbor = planner.map[index]['next_id'] + planner.map[index]['next_id']
    print(neighbor)
    for i in range(len(planner.map)):
        if(i in neighbor):
            color = 'red'
        else:
            color = 'black'
        road = planner.map[i]
        center = road['mid']
        plt.plot(center[:,0],center[:,1],color = color)
    plt.legend()
    plt.show()
    for index,value in planner.road_id_map.items():
        print(index," :",value)
    for i in range(len(planner.map)):
        print(i,":  next:",planner.map[i]['next_id'])
    print("success_init")