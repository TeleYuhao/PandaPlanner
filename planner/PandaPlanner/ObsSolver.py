'''
@Project :onsite-structured-test_with_PJPO
@Author : YuhaoDu
@Date : 2024/10/29 
'''
import numpy as np
import math
from utils.observation import Observation, ObjectStatus

class OBS:
    def __init__(self):
        self.type_list = {"vehicle", "bicycle", "pedestrian", "stopline"}
        self.obs_type = {"vehicle", "bicycle", "pedestrian", "stopline"}
    def InitParam(self,LaneChange):
        '''
        function: 初始化观测参数
        '''
        self.obs_s_set = [] # 障碍物frenet作标系下 s 位置集合
        self.obs_l_set = [] # 障碍物frenet坐标系下 l 位置集合
        self.obs_length_set = [] # 障碍物长度集合
        self.obs_width_set = [] # 障碍物宽度集合
        self.ReduceForUnknown = False # 避障减速标志位
        self.RePlan = False # 重规划标志位
        self.LeftForWardSpace = 100 # 初始左前空间
        self.LeftBackWardSpace = 100# 初始左后空间

        self.RightForWardSpace = 100 # 初始右前空间
        self.RightBackWardSpace = 100# 初始右后空间
        self.obj_data = []  # 障碍物
        self.LaneChange = LaneChange # 换道指令变量

        self.LeftSpace = 1e2
        self.LeftSpeed = 27 / 3.6
        self.RightSpace = 1e2
        self.RightSpeed = 27 / 3.6
        self.CurSpace = 1e2
        self.CurSpeed = 27 / 3.6

        self.dist2crossing = 100


    def UpdateObs(self,RefSpline,observation:Observation,LaneChange,lane_id,PlanningPath = None):
        '''
        function: 更新观测变量
        Args:
            RefSpline: 地图参考线
            observation: 观测量
            LaneChange: 换道决策变量
        '''
        self.Lane_id = lane_id # 当前车道id
        self.InitParam(LaneChange) # 初始化参数
        self.Spline = RefSpline # 全局参考线(三次样条曲线)
        ego_state = observation.ego_info # 自车状态变量
        self.obs_type = {"construction", "cone","tunnel"}
        self.ConstructionExists  = len(observation.object_info['construction'] )> 0 # 施工区存在标志位

        # 自车Frenet坐标系下的状态
        self.ego_sd = self.Spline.cartesian_to_frenet1D([ego_state.x, ego_state.y])
        for type, data in observation.object_info.items():
            for index, value in data.items():
                obj_sd = self.Spline.cartesian_to_frenet1D([value.x, value.y]) # 计算障碍物frenet坐标系下的位置
                self.UpdateLaneChangeSpace(obj_sd) #计算换道空间
                if type == 'crossing':
                    print("crossing exists")
                    self.dist2crossing = math.hypot(observation.ego_info.x - value.x,
                                                    observation.ego_info.y - value.y)
                if (type == 'others' or type == 'cone' or type == 'construction') and self.ReduceForUnknown == False:
                    # 判断实际障碍物是否存在
                    self.CheckUnknown(obj_sd)
                self.UpdateBound(obj_sd,type,value) # 更新障碍物的frenet位置列表
                if type != 'cone'  and type != 'crossing' and type != 'stopline':
                    self.UpdateBehavior(value,obj_sd) # 更新决策估计变量

                if PlanningPath == None or self.RePlan == True or type != 'cone':
                    continue
                else: # 如果出现锥桶或施工区出现在路径上，则进行重规划
                    obj_planning_sd = PlanningPath.cartesian_to_frenet1D([value.x, value.y])
                    if abs(obj_planning_sd[1]) < 1 and obj_planning_sd[0] < 25:
                        print("*************************    RePlan       **************************")
                        self.RePlan = True

                # print("object_info:",obj_sd,"ego_sd",self.ego_sd)

    def CheckUnknown(self,obj_sd):
        if (abs(obj_sd[1]) < 1.5 and obj_sd[0] < 40):
            print("*************************    Reduce For Object       **************************")
            self.ReduceForUnknown = True
    def UpdateBound(self,obj_sd,Type,value):
        '''
        function: 更新路径规划过程中的障碍物
        Args:
            obj_sd: 障碍物的frenet坐标
            Type: 障碍物类型
            value: 障碍物的具体参数
        '''
        if Type not in self.obs_type : return # 如果障碍物类型不属于需要处理的障碍物，则跳过
        length = max(abs(value.length), abs(value.width))
        width = min(abs(value.width), abs(value.length))
        obj_sd[1] += 1
        if Type == 'tunnel':
            length += 14
            width += 2
        elif Type == 'cone':
            length, width = 0.8, 0.8
        if self.ConstructionExists and Type == 'cone': return
        # 更新列表
        self.obs_s_set.append(obj_sd[0] - self.ego_sd[0])
        self.obs_l_set.append(obj_sd[1])
        self.obs_length_set.append(length)
        self.obs_width_set.append(width)

    def UpdateLaneChangeSpace(self,obj_sd):
        '''
        function: 计算换道可行空间
        Args:
            obj_sd: 障碍物的frenet坐标
        '''

        LeftMaxBound ,LeftMinBound   =   4 , 1
        RightMinBound, RightMaxBound =  -4, -1
        if ( LeftMinBound< obj_sd[1] < LeftMaxBound): # 如果障碍物位于自车左侧，则更新左侧换道可行空间
            if (obj_sd[0] < self.ego_sd[0]):
                self.LeftBackWardSpace = min(self.LeftBackWardSpace, self.ego_sd[0] - obj_sd[0])

            else:
                self.LeftForWardSpace = min(self.LeftForWardSpace, obj_sd[0] - self.ego_sd[0])
        elif ( RightMinBound< obj_sd[1] < RightMaxBound): # 如果障碍物位于自车右侧，则更新右侧换道可行空间
            if (obj_sd[0] < self.ego_sd[0]):
                self.RightBackWardSpace = min(self.RightBackWardSpace, self.ego_sd[0] - obj_sd[0])

            else:
                self.RightForWardSpace = min(self.RightForWardSpace, obj_sd[0] - self.ego_sd[0])


    def UpdateBehavior(self,value,obj_sd):
        '''
        function: 计算用于行为决策的左中右参数
        Args:
            value: 障碍物具体参数
            obj_sd: 障碍物的frenet坐标
        '''
        if obj_sd[0] > self.ego_sd[0] + abs(value.length):
            if 2 < obj_sd[1] < 6 and  obj_sd[0] - self.ego_sd[0] < self.LeftSpace:
                self.LeftSpace = min(obj_sd[0] - self.ego_sd[0], self.LeftSpace)
                self.LeftSpeed = min(value.v, self.CurSpeed)
            elif -2 < obj_sd[1] < 2 and obj_sd[0] - self.ego_sd[0] < self.CurSpace:
                self.CurSpace = min(obj_sd[0] - self.ego_sd[0],self.CurSpace)
                self.CurSpeed = min(value.v, self.CurSpeed)
            elif -6 < obj_sd[1] < -2 and obj_sd[0] - self.ego_sd[0] < self.RightSpace:
                self.RightSpace = min(obj_sd[0] - self.ego_sd[0],self.RightSpace)
                self.RightSpeed = min(value.v, self.RightSpeed)
