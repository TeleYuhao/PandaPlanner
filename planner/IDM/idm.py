#!/usr/bin/env python
# -*- coding: utf-8 -*-
#import lib
import numpy as np
import pandas as pd
import math
from planner.plannerBase import PlannerBase
from utils.observation import Observation
from utils.opendrive2discretenet import parse_opendrive
from typing import List, Tuple
from pyproj import Proj

class IDM(PlannerBase):
    def __init__(self, a_bound=5.0, exv=40, t=1.2, a=2.22, b=2.4, gama=4, s0=1.0, s1=2.0):
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
        self.all_line = []  # [[x, y, heading(rad), speed(km/h)], []]
        self.ref_line = []  # [[x, y, heading(rad), speed(km/h)], []]
        # self.lonlat2xy = Proj('+proj=tmerc +lon_0=108.90575652010739 +lat_0=34.37650478465651 +ellps=WGS84')  # 经纬度转xy，84
        self.lonlat2xy = Proj(proj='utm',zone=49,ellps='WGS84', preserve_units='m')

    def init(self, scenario_dict):
        print("----------------------------IDM INIT----------------------------")
        # print(scenario_dict)
        self.LineLonLat2XY(scenario_dict)
        print("----------------------------------------------------------------")
        # parse_opendrive(scenario_dict['source_file']['xodr'])

    def LineLonLat2XY(self, scenario_dict):
        """全局地图经纬度转全局xy"""
        if len(scenario_dict['task_info']['waypoints']) != 0:
            for one_point in scenario_dict['task_info']['waypoints']:
                temp_x, temp_y = self.lonlat2xy(one_point[0], one_point[1], inverse=False)
                self.all_line.append([temp_x, temp_y, one_point[2], one_point[3]])
                # self.all_line.append(one_point)

    def act(self, observation: Observation):
        # 栅格化地图
        # test
        # if len(self.all_line) >= 10:
        #     observation.ego_info.x = self.all_line[10][0]
        #     observation.ego_info.y = self.all_line[10][1]
        #     observation.ego_info.yaw = self.all_line[10][2]
        #     print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  ')
        #     print('car x: ', observation.ego_info.x, ', y: ', observation.ego_info.y, ', yaw: ', observation.ego_info.yaw)
        
        self.UpdateRefLine(observation.ego_info)
        if len(self.ref_line) == 0:
            print("no ref line")
            # return [0, 0]
        # print("ref line:")
        # print(self.ref_line)
        # if len(self.ref_line) > 0:
        #     print('x = ', self.ref_line[0][0], ', y = ', self.ref_line[0][1], '   ', self.ref_line[0][2])
        # 加载主车信息
        frame = pd.DataFrame(
            vars(observation.ego_info),
            columns=['x', 'y', 'v', 'yaw', 'length', 'width'], 
            index=['ego']
        )
        # 加载背景要素状态信息
        for obj_type in observation.object_info:
            for obj_name, obj_info in observation.object_info[obj_type].items():
                sub_frame = pd.DataFrame(vars(obj_info), columns=['x', 'y', 'v', 'yaw', 'length', 'width'],index=[obj_name])
                frame = pd.concat([frame, sub_frame])
        state = frame.to_numpy()
        result_a = self.deside_acc(state)
        result_r = self.deside_r(observation.ego_info)
        return [result_a, result_r]

    def UpdateRefLine(self, car):
        """更新栅格化地图"""
        self.ref_line = []
        print('!!!  ', len(self.all_line), '     ', self.index)
        if len(self.all_line) != 0:
            min_dis = -1
            break_count = 0
            index = 0
            print('car x = ', car.x, ', y = ', car.y, ', yaw: ', car.yaw)
            for i in range(self.index, len(self.all_line)):
                # print(i, ', x = ', self.all_line[i][0], ', y = ', self.all_line[i][1], '   ', self.all_line[i][2])
                det_x = self.all_line[i][0] - car.x
                det_y = self.all_line[i][1] - car.y
                det_dis = math.sqrt(det_x ** 2 + det_y ** 2)
                # print(i, ', detx = ', det_x, ', dety = ', det_y, ', dis = ', det_dis, ', min_dis = ', min_dis, ', count: ', break_count)
                if min_dis == -1 or det_dis <= min_dis:
                    min_dis = det_dis
                    index = i
                    break_count = 0
                else:
                    break_count = break_count + 1
                    if break_count > 5:
                        break
            self.index = max(0, min(index - 20, len(self.all_line)))
            print("index = ", index)
            for j in range(index, min(index + 200, len(self.all_line))):
                det_x = self.all_line[j][0] - car.x
                det_y = self.all_line[j][1] - car.y
                det_angle = self.all_line[j][2] - car.yaw

                # new_x = det_x * math.cos(det_angle) - det_y * math.sin(det_angle)
                # new_y = det_x * math.sin(det_angle) + det_y * math.cos(det_angle)
                # self.ref_line.append([new_x, new_y, self.all_line[j][2], self.all_line[j][3]])

                distance = math.sqrt(det_x ** 2 + det_y ** 2)
                angle_line = math.atan2(det_x, det_y) / math.pi * 180
                angle = angle_line - car.yaw / math.pi * 180
                new_x = distance * math.sin(angle * math.pi / 180)
                new_y = distance * math.cos(angle * math.pi / 180)
                # print(j, ', angle_line: ', angle_line, ', detangle: ', angle, ', dis: ', distance)
                # print(j, ', utm x: ', self.all_line[j][0], ', y: ', self.all_line[j][1], ', yaw: ', self.all_line[j][2] / math.pi * 180)
                # print(j, ', det x: ', det_x, ', y: ', det_y)
                self.ref_line.append([new_x, new_y, self.all_line[j][2], self.all_line[j][3]])

                # print(j, ', x = ', new_x, ', y = ', new_y)

    def deside_acc(self, state: pd.DataFrame) -> float:
        v, fv, dis_gap, direction = self.getInformFront(state)
        # print(v, fv, dis_gap,direction)
        # print(state)
        if dis_gap < 0:
            a_idm = self.a * (1 - (v / self.exv) ** self.gama)
        else:
            # 求解本车与前车的期望距离
            # print(self.s0,self.s1,self.exv,v,self.t)
            self.s_ = self.s0 + self.s1 * (v / self.exv) ** 0.5 + self.t * v + v * (
                v - fv) / 2 / (self.a * self.b) ** 0.5
            # 求解本车加速度
            a_idm = self.a * (1 - (v / self.exv) ** self.gama - ((self.s_ / (dis_gap+1e-6)) ** 2))
        # 对加速度进行约束
        a_idm = np.clip(a_idm, -self.a_bound, 1e7)
        # print(v,fv,dis_gap,a_idm,self.s_)
        # print(state,v,fv,dis_gap,a_idm)
        return a_idm

    def deside_r(self, car):
        """计算前轮转角"""
        k1 = 1.0
        k2 = 1.0
        wheel_base = 2.49   # 自车轴距
        if len(self.ref_line) == 0:
            return 0
        """ 寻找瞄点 """
        pp_distance = wheel_base + car.v * 0.6 / 3.6
        point_pp = self.ref_line[-1]
        point_front = self.ref_line[-1]
        for point in self.ref_line:
            if point[1] > pp_distance:
                point_pp = point
                break
        for point in self.ref_line:
            if point[1] > wheel_base:
                point_front = point
                break
        """ 计算Pure Pursuit """
        dis_point_pp = math.sqrt(point_pp[0]**2 + point_pp[1]**2)
        sin_alpha = point_pp[0] / dis_point_pp
        r_pp = math.atan(2 * wheel_base * sin_alpha * k1 / (k2 * pp_distance)) * 180.0 / math.pi
        r_delta_h = point_pp[2] - car.yaw
        if r_delta_h > math.pi:
            r_delta_h = r_delta_h - 2 * math.pi
        elif r_delta_h < -math.pi:
            r_delta_h = r_delta_h + 2 * math.pi
        """ 航向角偏差 """
        r_delta_h = r_delta_h * 180.0 / math.pi
        """ 横向位置偏差 """
        r_delta_x = 0.05 * point_front[0]
        """ 加权得到最终角度 """
        r = r_pp * 0.8 + r_delta_h * 0.2 + r_delta_x
        return -r

    def getInformFront(self, state: pd.DataFrame) -> Tuple[float, float, float, float]:
        # direction = np.sign(state[0,2])
        if state[0, 3] < np.pi / 2 or state[0, 3] > np.pi * 3 / 2:
            direction = 1.0
        else:
            direction = -1.0
        state[:,0] = state[:,0]*direction
        # state[:,2] = state[:,2]*direction
        ego = state[0,:]
        v, fv, dis_gap = ego[2], -1, -1
        
        # 在本车前侧
        x_ind = ego[0] < state[:,0]
        y_ind = (np.abs(ego[1] - state[:,1])) < ((ego[5] + state[:,5])/2)
        ind = x_ind & y_ind
        if ind.sum() > 0:
            state_ind = state[ind,:]
            front = state_ind[(state_ind[:,0]-ego[0]).argmin(),:]
            # print(front)
            fv = front[2]
            dis_gap = front[0] - ego[0] - (ego[4] + front[4])/2
        if dis_gap > 100:
            dis_gap = -1
            fv = -1
        return v, fv, dis_gap, direction
