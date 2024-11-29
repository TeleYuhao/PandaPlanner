#!/usr/bin/env python
# -*- coding: utf-8 -*-
#import lib
import math
from utils.opendrive2discretenet import parse_opendrive
from typing import List, Tuple
from pyproj import Proj
import xml.etree.ElementTree as ET
import planner.IDM.pkhx as PH
import time
import json

R_e = 6378137.0           
R_f = 6356752.314245     
e_1 = math.sqrt(pow(R_e, 2) - pow(R_f, 2)) / R_e   
square_e = e_1 * e_1
first = True
all_line = []

class read_map():
    def __init__(self):
        self.first = True
        self.lon0 = 0
        self.lat0 = 0
        self.all_key_id = []
        self.all_uyxu_id = []
        self.all_line = []          # [[x, y, heading(deg), speed(km/h)], []]
        self.map = {}               # [{'mid' : [lon, lat], 'left' : [lon, lat], 'right' : [lon, lat]}, {}]
        self.all_line_xodr = []     # [[x, y, heading(deg), speed(km/h)], []]
        self.test_data = []

    def ll_to_xy(self, lon, lat):
        sin_lat1 = math.sin(math.radians(self.lat0))
        cos_lat1 = math.cos(math.radians(self.lat0))
        square_sin_lat1 = sin_lat1 * sin_lat1
        R_n = R_e / (math.sqrt(1 - square_e * square_sin_lat1))             
        R_m = (R_n * (1 - square_e)) / (1 - square_e * square_sin_lat1)     
        gx = math.radians(lon - self.lon0) * R_n * cos_lat1
        gy = math.radians(lat - self.lat0) * R_m
        return gx, gy

    def Caldis(self, x1, x2, y1, y2):
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

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

        self.map = {}
        for i in road_info.discretelanes:
            temp_dic = {'mid' : [], 'predecessor': i.predecessor, 'successor': i.successor, 'left' : i.leftlane, 'right' : i.rightlane}
            if self.first == True and len(i.center_vertices) > 0:
                self.lon0, self.lat0, self.first = i.center_vertices[0][0], i.center_vertices[0][1], False
                print("lon0,lat0 = ", self.lon0, ", ", self.lat0)
            for j in i.center_vertices:
                gx, gy = self.ll_to_xy(j[0], j[1])
                temp_dic['mid'].append([j[0], j[1], gx, gy])
            # print("id = ", i.lane_id, ", left = ", i.leftlane, ", right = ", i.rightlane, ",predecessor = ", i.predecessor, ", successor = ", i.successor)
            self.map[i.lane_id] = temp_dic
        print("maplen = ", len(self.map))

    def set_left_right(self):
        for id, map_i in self.map.items():
            leftall, rightall = [], []
            # go left
            temp_left_id, temp_right_id = id, id
            while True:
                if temp_left_id in self.map:
                    if self.map[temp_left_id]['left'] == '':
                        break
                    else:
                        temp_left_id = self.map[temp_left_id]['left']
                        leftall.append(temp_left_id)
                else:
                    break
            # go right
            while True:
                if temp_right_id in self.map:
                    if self.map[temp_right_id]['right'] == '':
                        break
                    else:
                        temp_right_id = self.map[temp_right_id]['right']
                        rightall.append(temp_right_id)
                else:
                    break
            map_i['leftall'] = leftall
            map_i['rightall'] = rightall
            for map_i_p in map_i['mid']:
                map_i_p.append(len(leftall))
                map_i_p.append(len(rightall))

    def cal_grjm_id(self, grjm_i_x, grjm_i_y):
        dis_min = 10000
        mubn_id = ''
        # mubn_ludr = {}
        for id, ludr in self.map.items():
            for ludr_dm in ludr['mid']:
                dis = self.Caldis(grjm_i_x, ludr_dm[2], grjm_i_y, ludr_dm[3])
                if dis < 0.1:
                    return id
                if dis < dis_min:
                    mubn_id = id
                    dis_min = dis
        return mubn_id

    def vc_zvjblujk_id(self, all_grjm):
        for grjm_i in all_grjm:
            grjm_i_x, grjm_i_y = self.ll_to_xy(grjm_i[0], grjm_i[1])
            self.all_key_id.append(self.cal_grjm_id(grjm_i_x, grjm_i_y))

    def temp_all_grjm(self):
        a = [
            [ 108.89949923654412 ,  34.37538166124209 ], 
            [ 108.89861532743872 ,  34.37508941128636 ], 
            [ 108.89722156783255 ,  34.37471954820745 ], 
            [ 108.89716094071574 ,  34.37470411071791 ], 
            [ 108.89562800657411 ,  34.374313595347274 ], 
            [ 108.89550678842154 ,  34.37428262473278 ], 
            [ 108.8941382448678 ,  34.37393448336353 ], 
            [ 108.89403430231143 ,  34.37390804971167 ], 
            [ 108.89231595283103 ,  34.37330746541305 ], 
            [ 108.89231448171275 ,  34.37321081406318 ], 
            [ 108.89236497290358 ,  34.37308239946697 ], 
            [ 108.89256745844008 ,  34.372631884883766 ], 
            [ 108.89258323579091 ,  34.372596675180176 ], 
            [ 108.89308286382563 ,  34.37241447264685 ], 
            [ 108.89318326747367 ,  34.37244905113575 ], 
            [ 108.89380242512705 ,  34.372662283326704 ], 
            [ 108.89586007603245 ,  34.373403080118926 ], 
            [ 108.89622866816671 ,  34.373528984111125 ], 
            [ 108.89717444427319 ,  34.37386667602462 ], 
            [ 108.89919331866705 ,  34.37450677680094 ], 
            [ 108.9001994559551 ,  34.3749172270945 ], 
            [ 108.90018879953524 ,  34.374937959704795 ], 
            [ 108.90017814311011 ,  34.3749586923141 ], 
            [ 108.89975014036827 ,  34.37512274450222 ], 
            [ 108.89966522264864 ,  34.37509655315311 ], 
            [ 108.89961429464853 ,  34.37508078807255 ], 
            [ 108.89955011460525 ,  34.37505352690089 ]]
        return a

    def vc_lmjp_grxi(self, start_id, end_id, graph):

        def bfs(start, end, graph):
            queue = [(start, [start])]
            visited = set() 
            while queue:
                current_segment, path = queue.pop(0)
                if current_segment in end_id:
                    return path
                visited.add(current_segment)
                for next_segment in graph[current_segment]['forward']:
                    if next_segment in graph and next_segment not in visited and next_segment not in path: 
                        queue.append((next_segment, path + [next_segment]))
        path = bfs(start_id, end_id, graph)
        return path

    def vc_all_uyxu_id(self):
        # 构建路段图
        road_segments = {}     
        for id, map_i in self.map.items():
            road_segments[id] = {'forward': []}
            road_segments[id]['forward'] += map_i['successor']
            if map_i['left'] != '':
                road_segments[id]['forward'].append(map_i['left'])
            if map_i['right'] != '':
                road_segments[id]['forward'].append(map_i['right'])
        i = 1
        last_id = self.all_key_id[0]
        self.all_uyxu_id.append(last_id)
        # print("all_key = ", self.all_key_id)
        # print("\n\n")
        while i < len(self.all_key_id):
            now_id = self.all_key_id[i]
            # 跟上一个关键点id一样
            if now_id == last_id:
                # print('now_id == last_id \n')
                i += 1
                continue
            # 上一个id首尾相接
            if last_id in self.map[now_id]['predecessor'] or now_id in self.map[last_id]['successor']:
                # print('上一个id首尾相接\n')
                self.all_uyxu_id.append(now_id)
                last_id = now_id
                i += 1
                continue
            # 
            if len(self.map[now_id]['predecessor']) > 0:
                end_id_i = self.map[now_id]['predecessor']
                # print('key = ', now_id, ', key_pre = ', end_id_i, ',  start = ', last_id)
                end_id = []
                end_id += end_id_i
                # for end_id_ii in end_id_i:
                #     end_id += self.map[end_id_ii]['leftall'] + self.map[end_id_ii]['rightall']
                # if 
                path = self.vc_lmjp_grxi(last_id, end_id, road_segments)
                if path is None:
                    self.all_key_id.pop(i)
                    continue
                self.all_uyxu_id.extend(path)
                self.all_uyxu_id.append(now_id)
            else:
                end_id = now_id
                end_id += self.map[now_id]['leftall'] + self.map[now_id]['rightall']
                path = self.vc_lmjp_grxi(last_id, end_id, road_segments)
                self.all_uyxu_id.extend(path)
                if len(path) > 0:
                    if path[-1] != now_id:
                        self.all_uyxu_id.append(now_id)
                else:
                    self.all_uyxu_id.append(now_id)

            last_id = now_id
            i += 1
            # print("\n")
        # 
        if len(self.all_uyxu_id) == 0:
            return 

        j = 1
        while j < len(self.all_uyxu_id):
            if self.all_uyxu_id[j] == self.all_uyxu_id[j - 1]:
                self.all_uyxu_id.pop(j)
            else:
                j += 1

    def hebk_all_road(self):
        for idi in self.all_uyxu_id:
            self.all_line += self.map[idi]['mid']

def a(path, all_grjm0):
    global first, all_line
    if not first:
        return all_line
    RM = read_map()
    all_grjm = all_grjm0
    RM.calmap(path)
    RM.set_left_right()
    RM.vc_zvjblujk_id(all_grjm)
    RM.vc_all_uyxu_id()
    RM.hebk_all_road()
    old_line = []
    all_line = PH.test(RM.all_line, all_grjm, old_line)

    first = False
    return all_line










