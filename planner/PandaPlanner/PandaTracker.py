import matplotlib.pyplot as plt
import numpy as np
from planner.PandaPlanner.Spline2d import Spline2d
# from planner.PandaPlanner.TrajectoryPlanner.CubicSpline import Spline2D
from planner.PandaPlanner.LatController import LatController
from typing import List, Tuple
from planner.plannerBase import PlannerBase
from planner.Controller.LontidudeController import Longitudinal_PID_controller
from planner.PandaPlanner.TrajectoryPlanner.PJSO_IPOPT_use import PJSO
from planner.PandaPlanner.TrajectoryPlanner.Piecewise_jerk_path_optimize import PJPO
from planner.PandaPlanner.TrajectoryPlanner.CubicSpline import Spline
from planner.PandaPlanner.TrajectoryPlanner.ST_GRAPH import ST_GRAPH
import math
import time
from planner.PandaPlanner.TrajectoryPlanner.temp_utils import *
from planner.PandaPlanner.ParkPlanner.reeds_shepp import calc_reeds_shepp
from planner.PandaPlanner.TrajectoryPlanner.get_bound import generate_convex_space, is_lane_change
from planner.PandaPlanner.SloveMap.RoutePlanner import *
from planner.PandaPlanner.behaivor_planner import BehaivorPlanner
from planner.PandaPlanner.ObsSolver import OBS


class PandaPlanner(PlannerBase):

    def init(self, scenario_info: dict) -> None:
        self.LatCon = LatController(np.diag([1.0, 0.01, 1.0, 0.01]), np.diag([400]), 1) # LQR横向控制器
        self.LonCon = Longitudinal_PID_controller() # PID纵向控制器
        self.LonFilter = FirstOrderLowPassFilter() # 纵向低通滤波器
        self.LatFilter = FirstOrderLowPassFilter() # 横向低通滤波器
        self.SpeedPlanner = PJSO(21,0.25) # 速度规划器
        self.PathPlanner = PJPO(50, 1) # 路径规划器
        self.ST_Graph = ST_GRAPH() # ST图
        self.ObsTransformer = OBS() # 观测变量处理器
        self.Total_Global_Traj = scenario_info['task_info']['waypoints']
        self.RoutePlanner = RoutePlanner(self.Total_Global_Traj[0],self.Total_Global_Traj[-1]) # 全局路径规划器
        self.BehaivorPlanner = BehaivorPlanner(self.RoutePlanner) # 行为规划器
        self.time = 0
        self.incurve = False # 自车是否处于弯道标志位
        self.ForwardTurning = False # 前向转向标志位
        self.light = 0 # 车辆灯光标志位
        self.last_acc = 0
        self.PassTime = None # 通行时间（用于停止线识别场景）
        self.StopLine = None # 听直线
        self.WaitTime = None # 等待时间
        self.Pass = False    # 通过性判断(用于停止线识别场景)
        self.ReachEnd = False # 到达终点标志位
        self.ParkingTrajectory = None # 泊车轨迹
        self.ParkLength = None # 泊车轨迹长度
        self.path = []  # 规划轨迹
        self.PlanningPath = None # 规划轨迹 （三次样条: CubicSpline）
        self.TruningPath = False # 轨迹是否为转向/避障轨迹
        self.StopForSecond = False
        self.Solved = True # 求解成功标志位
        self.RefPath = []
        self.MapPath = []
        self.LaneChange = 0
        self.RefSD = np.zeros(50)

    def GetMapTraj(self,observation):
        '''
        function: 从地图获取当前道路的参考线
        '''
        dist2end = math.hypot(observation.ego_info.x - self.MapPath[-1][0],
                              observation.ego_info.y - self.MapPath[-1][1]) if len(self.MapPath ) > 0 else 100
        self.MapPath,self.LaneChange,self.PathLength = self.RoutePlanner.GetRefLaneSuccessor([observation.ego_info.x,observation.ego_info.y] ,dist2end=dist2end)
        self.MapPath = self.MapPath[::12] # 获取参考线并稀疏点
        self.MapPath += np.random.rand(*self.MapPath.shape) / 1e8 # 向参考线添加微小随机扰动，确保问题求解
        self.RefSpline = Spline2d(self.MapPath[:,0],self.MapPath[:,1]) # 将参考线处理为三次样条
        self.ObsTransformer.UpdateObs(self.RefSpline,observation,self.LaneChange,self.RoutePlanner.last_lane_id,self.PlanningPath) # 更新观测变量
        return self.MapPath

    def GetLaneChangeSpaceTest(self):
        '''
        function： 更新换道空间
        '''
        if self.LaneChange == 0:
            self.ForWardSpace = 100
            self.BackWardSpace = 100
            return
        elif self.LaneChange == 1:
            self.ForWardSpace = self.ObsTransformer.RightForWardSpace
            self.BackWardSpace = self.ObsTransformer.RightBackWardSpace
        else:
            self.ForWardSpace = self.ObsTransformer.LeftForWardSpace
            self.BackWardSpace = self.ObsTransformer.LeftBackWardSpace
        print("LaneChangeSpace", self.ForWardSpace, self.BackWardSpace, "LaneChange", self.LaneChange)

    def GetLight(self):
        '''
        function: 计算灯光变量
        '''
        if self.incurve == 0: # 如果车辆位于直线段，灯光收到换道行为控制
            if   self.LaneChange == -1: self.light = 2
            elif self.LaneChange ==  1: self.light = 1
            else: self.light = 0
        else: # 如果为弯道段，根据曲率判断转向方向
            self.light = self.incurve
    def InCurve(self,dist, observation):
        '''
        function：判断车辆是否在弯道内
        '''
        cur = []
        ego_sd = self.RefSpline.cartesian_to_frenet1D([observation.ego_info.x, observation.ego_info.y])
        if(dist < 35):
            for i in np.linspace(ego_sd[0], ego_sd[0] + int(dist), 25):
                cur.append(self.RefSpline.calc_curvature(i))
        else:
            for i in np.linspace(ego_sd[0], ego_sd[0] + 80, 80): # 向前计算80m范围内的曲率值
                cur.append(self.RefSpline.calc_curvature(i))
        if(max(cur[:25]) > 0.04 ): self.incurve = 2
        elif(min(cur[:25]) < -0.04): self.incurve = 1
        else:self.incurve = 0

        self.ForwardTurning = True if (max(cur) > 0.04 or min(cur) < -0.04) else False
        if(self.ReachEnd): self.incurve = 4  # 如果到达终点，则开启双闪

    def CheckLeft(self,observation):
        '''
        function: 检查左侧是否有侧向来车
        '''
        for index, value in observation.object_info['vehicle'].items(): #遍历车辆
            s, d, vs, vd = self.PlanningPath.cartesian_to_frenet2D([value.x, value.y, np.deg2rad(value.yaw), value.v])
            print("veh:",s,d,vs,vd)
            if 10 > d > -3 and vs < -0.3 and   0 < s < 40 and observation.ego_info.v < 3: # 如果车辆位于自车左侧吗且速度小于-0.3
                print("------------ Stop For Left-----------------")
                return True
        return False
    def CalaRefSpeed(self, dist, observation):
        '''
        function: 计算参考速度
        '''
        target = 30 / 3.6
        if self.ObsTransformer.ReduceForUnknown: target = 15 / 3.6 # 如果存在障碍物，则参考速度修改为15
        if len(observation.object_info['crossing']) > 0 and self.CheckLeft(observation): # 如果存在侧向来车且存在人行横道，参考速度修改为-2
            target = -2
        decel = 0
        ego_sd = self.PlanningPath.cartesian_to_frenet1D([observation.ego_info.x, observation.ego_info.y]) # 计算自车位置
        ego_s = ego_sd[0]
        ego_v = observation.ego_info.v
        self.InCurve(dist, observation) # 计算是否在弯道内
        if dist < 50: # 计算曲率速度折减量
            for i in np.arange(max(ego_s - 10, 0), ego_s + int(dist), 1):
                decel += abs(self.PlanningPath.calc_curvature(i))
            decel /= int(dist + 1)
        else:
            for i in np.arange(max(ego_s - 10, 0), ego_s + 50, 1):
                decel += abs(self.PlanningPath.calc_curvature(i))
            decel /= 70
        if 0 <= decel <= 0.005:
            r = 0
        elif 0.005 < decel <= 0.008:
            r = 577 * decel - 2.085
        elif 0.008 < decel <= 0.04:
            r = 41.5 * decel + 2.1
        else:
            r = 4.3

        target -= r
        if self.ObsTransformer.CurSpace < 20 and self.LaneChange == 0 and len(observation.object_info['stopline']) == 0:
            target = min(target, self.ObsTransformer.CurSpeed * 1.1)
        elif (self.ObsTransformer.LeftForWardSpace < 8 or self.ObsTransformer.LeftBackWardSpace < max(12,
                                                                                                      3 * ego_v)) and self.LaneChange == -1:
            # 如果强制左换道存在，且空间不足，则减速等待汇入空间
            target = target * 0.4
        elif (self.ObsTransformer.RightForWardSpace < 8 or self.ObsTransformer.RightBackWardSpace < max(12,
                                                                                                        3 * ego_v)) and self.LaneChange == 1:
            # 如果强制右换道存在，且空间不足，则减速等待汇入空间
            if self.RoutePlanner.last_lane_id == 47:
                target = 13 / 3.6
            else:
                target *= 0.5
        if self.RoutePlanner.last_lane_id in [111, 186, 142, 79, 86, 1, 394 , 309,239]: # 减速危险路段减速
            target *= 0.8
        elif 47 not in self.RoutePlanner.route_id:
            print("come to next circle")
            target *= 0.9
        # if self.RoutePlanner.last_lane_id == 7 : target *= 0.9
        if self.ObsTransformer.dist2crossing < 12 and len(observation.object_info['stopline']) == 0:
            print("Reduce For Crossing")
            target *= 0.8

        return max(target,0)


    def CheckStopForEnd(self,dist,observation):
        '''
        function: 检查是否到达终点
        '''
        if(dist < 20 and observation.ego_info.v < 1):
            self.ReachEnd = True
            self.ParkPlanning(observation)
            self.TrajectoryAnalyzer = TrajectoryAnalyzer(self.ParkingTrajectory)
    @staticmethod
    def ChooseParkLot(observation):
        '''
        function: 车位选择算法
        '''
        ParkLot = None
        distance = 1e2
        for key,value in observation.object_info['parking'].items():
            dist = math.hypot(observation.ego_info.x - value.x,
                              observation.ego_info.y - value.y) # 车辆到车位的距离
            use = True
            for type, data in observation.object_info['vehicle'].items():
                parking2car = math.hypot(value.x - data.x, value.y - data.y) # 车位到障碍物车辆的距离
                if parking2car < 2: # 如果车辆与车位的距离小于2 ，则判断为占用状态
                    use = False
                    break
            if (dist < distance and use): # 如果车位为空，且距离更近，则将车位赋值给parklot
                ParkLot = value
                distance = dist
        return ParkLot
    def ParkPlanning(self,observation):
        '''
        function： 泊车规划函数
        '''
        Start_State = [observation.ego_info.x,observation.ego_info.y,np.deg2rad(observation.ego_info.yaw)] # 自车状态
        TargetParkingLot = self.ChooseParkLot(observation) # 目标车位选择
        ParkYaw = np.deg2rad(TargetParkingLot.yaw ) + np.pi
        EndState = [TargetParkingLot.x ,TargetParkingLot.y ,ParkYaw] # 目标位姿确定
        gap = 6
        EndState[0] -= gap*np.cos(EndState[2])
        EndState[1] -= gap*np.sin(EndState[2])
        print("start_state:",Start_State,"end_state:",EndState,"ego_yaw:",observation.ego_info.yaw)

        RSPath = calc_reeds_shepp(Start_State,EndState,maxc=0.07) #RS曲线计算
        RSPath.x.append(EndState[0] + gap*np.cos(EndState[2]) + 1e-8)
        RSPath.y.append(EndState[1] + gap*np.sin(EndState[2]) - 1e-8)
        self.ParkingTrajectory = Spline2d(RSPath.x,RSPath.y) # 三次样条平滑
        self.ParkLength = RSPath.L + 6

    def CheckStopForStopLine(self,observation):
        '''
        function: 检查是否因为停止线停车
        '''
        ego_sd = self.PlanningPath.cartesian_to_frenet1D([observation.ego_info.x, observation.ego_info.y])
        for index,value in observation.object_info['stopline'].items():
            s, d = self.PlanningPath.cartesian_to_frenet1D([value.x, value.y])
            if(abs(ego_sd[0] - s) < 10):
                return True
        return False

    def PassStopLine(self,observation):
        '''
        function : 是否能够停车线
        '''
        if observation.ego_info.v < 0.5 and self.CheckStopForStopLine(observation) and self.PassTime == None:
            self.PassTime = self.cur_time + 15 # 通行时间计算， 为当前时间之后的15s
            self.Pass = False # 通行权为false
        elif (self.PassTime != None and self.cur_time > self.PassTime and self.cur_time < self.PassTime + 40):
            self.Pass = True # 如果当前时间是通行时间之后，则通行权为true
        elif (self.PassTime != None and self.cur_time > self.PassTime + 15):
            self.PassTime = None
            self.Pass = False

    # @count_time_args()
    def UpdataPartTraj(self,observation):
        '''
        获取局部路径
        '''
        self.MapTraj = self.GetMapTraj(observation)
        if self.PlanningPath == None:#初始路径规划
            self.PathPlanning(observation)
            self.TrajectoryAnalyzer = TrajectoryAnalyzer(self.PlanningPath)

        else:
            # self.GetRefSTest(observation)
            if self.LaneChange == 0:
                LaneChangeVarivable = [self.ObsTransformer.LeftSpace,self.ObsTransformer.LeftSpeed,
                                       self.ObsTransformer.CurSpace, self.ObsTransformer.CurSpeed,
                                       self.ObsTransformer.RightSpace, self.ObsTransformer.RightSpeed] # 换道决策变量整合
                self.LaneChange = self.BehaivorPlanner.CalcBehaivorTest(self.RoutePlanner.last_lane_id,LaneChangeVarivable,observation,self.ForwardTurning) # 换道决策计算
                # self.LaneChange = self.BehaivorPlanner.CalcBehaivor(self.RoutePlanner.last_lane_id,observation,self.RefSpline,self.incurve)
            planning_sd = self.PlanningPath.cartesian_to_frenet1D([observation.ego_info.x, observation.ego_info.y])
            if (self.TruningPath == False and (time.time() - self.PathPlanningTime > 5 or planning_sd[0] > 4)) or self.ObsTransformer.RePlan == True:
                try: # 轨迹规划并传递给横向控制器
                    self.PathPlanning(observation)
                    self.TrajectoryAnalyzer = TrajectoryAnalyzer(self.PlanningPath)
                except:
                    print("error on Path planning")
            elif planning_sd[0] > 27 or time.time() - self.PathPlanningTime > 10:
            # elif planning_sd[0] > 1:
                try:
                    self.PathPlanning(observation)
                    self.TrajectoryAnalyzer = TrajectoryAnalyzer(self.PlanningPath)
                except:
                    print("error on Path planning")

        self.SpeedPlanning(observation)



    def SpeedPlanning(self,observation):
        '''
        function: 速度规划算法
        '''
        # speed_planning
        if self.ReachEnd == False:

            sd = self.PlanningPath.cartesian_to_frenet1D([observation.ego_info.x, observation.ego_info.y]) #计算自车sd
            dist = math.hypot(observation.ego_info.x - self.Total_Global_Traj[-1][0],
                              observation.ego_info.y - self.Total_Global_Traj[-1][1]) # 计算到终点的距离
            target_speed = self.CalaRefSpeed(dist,observation) # 计算参考速度
            if dist < 10:
                target_speed = 0 # 如果到终点，则参考速度为0，缓慢刹停
            init_state = [max(sd[0], 1e-2), observation.ego_info.v, self.last_acc] # 纵向状态 [s,s', s'']
            s_lower_bound, s_upper_bound = self.ST_Graph.UpDate(self.PlanningPath,self.Pass,observation,self.RoutePlanner.last_lane_id) # 计算规划的上下界
            if CheckConvex(s_lower_bound,s_upper_bound) == False: #检查空间是否可行
                target_speed = 0
                s_upper_bound = [sd[0] + 1]*21
                s_lower_bound = [sd[0]]*21
            ref_s = [init_state[0] + target_speed * t for t in np.arange(0, 5.01, 0.25)]
            # print("sd:",sd)
        else: # 如果到达终点，则参考速度为3m/s
            tracking_speed = 3
            target_speed = tracking_speed
            sd = self.ParkingTrajectory.cartesian_to_frenet1D([observation.ego_info.x, observation.ego_info.y])
            if sd[0] > self.ParkLength - 2: target_speed = 0
            # print("parking_sd:",sd,"parking Length:",self.ParkLength,"TargetSpeed:",target_speed)
            if target_speed == 0 and self.ReachEnd == True:
                init_state = [sd[0]+0.1,observation.ego_info.v,0]
                s_lower_bound = [0] * 21
                s_upper_bound = [sd[0] + 0.1] * 21
            else:
                init_state = [max(sd[0], 1e-2), observation.ego_info.v, self.last_acc]
                s_lower_bound = [0]*21
                s_upper_bound = [self.ParkLength - 1]*21
            ref_s = [init_state[0] + tracking_speed * t for t in np.arange(0, 5.01, 0.25)]
        try:
            print("target_speed:",target_speed,"init_state:",init_state,"ReduceForUnknown:",self.ObsTransformer.ReduceForUnknown)
            self.SpeedPlanner.Update(init_state, ref_s, target_speed, s_lower_bound, s_upper_bound)
            self.SpeedPlanner.Solve()
            self.Solved = True
        except:
            print("SpeedPlanning error , try to brake")
            if self.RoutePlanner.last_lane_id in [209,210,56,89,84,45,162,188]:
                print("************************* Reduce In ReduceLane ************************************")
                target_speed = -100
            else: target_speed = 0
            s_lower_bound = [0] * 21
            s_upper_bound = [min(15,self.ObsTransformer.CurSpace)] * 21
            init_state = [0,observation.ego_info.v, 0]
            self.SpeedPlanner.Update(init_state, ref_s, target_speed, s_lower_bound, s_upper_bound)
            self.SpeedPlanner.Solve()
            self.Solved = False
        self.Speed_Curve = Spline(np.arange(0, 5.01, 0.25), self.SpeedPlanner.solution_v.full().flatten())
        self.Acc_Curve   = Spline(np.arange(0, 5.01, 0.25), self.SpeedPlanner.solution_a.full().flatten())
        self.planning_time = time.time()

    def get_bound(self, observation):
        '''
        get the obs bound of path planning
        边界提取算法，获取路径规划凸空间
        '''
        ego_sd = self.RefSpline.cartesian_to_frenet1D([observation.ego_info.x, observation.ego_info.y])
        obs_s_set, obs_l_set, obs_length_set, obs_width_set = [],[],[],[]
        for Type, Object in observation.object_info.items():
            # if Type != 'construction' and 'tunnel':
            #     continue
            if Type != 'cone' and Type != 'construction' : continue
            for index, value in Object.items():
                sd = self.RefSpline.cartesian_to_frenet1D([value.x, value.y])
                length = max(value.length,value.width)
                width  = min(value.width,value.length)
                # if (sd[0] - length/2 - ego_sd[0] > 20 or value.v > 0.2): continue
                obs_s_set.append(sd[0]-ego_sd[0])
                obs_l_set.append(sd[1])
                obs_length_set.append(length)
                obs_width_set.append(width)
        lu_bound, ll_bound,IsConvex = generate_convex_space(obs_s_set, obs_l_set, obs_length_set, obs_width_set, ego_sd[1])
        cartesian_ll_bound, cartesian_lu_bound=[], []
        for i in range(50):
            ll = self.RefSpline.frenet_to_cartesian1D([ego_sd[0]+i, ll_bound[i]])
            lu = self.RefSpline.frenet_to_cartesian1D([ego_sd[0]+i, lu_bound[i]])
            cartesian_ll_bound.append(ll)
            cartesian_lu_bound.append(lu)
        return np.array(cartesian_ll_bound), np.array(cartesian_lu_bound)

    def PathPlanning(self, observation):
        '''
        using Piese wise jerk Path optimization to optimize the init path total 50m
        使用Piece Wise Jerk Path Optimization 方法对前方50m进行规划
        '''
        ego_state = observation.ego_info
        ego_s, ego_d, ego_vs, ego_vd = self.RefSpline.cartesian_to_frenet2D(
            [ego_state.x, ego_state.y, np.deg2rad(ego_state.yaw), ego_state.v]) # 2阶frenet坐标系下状态计算

        ref_yaw = self.RefSpline.calc_yaw(ego_s)
        ego_ad = self.last_acc * np.sin(np.deg2rad(ego_state.yaw) - ref_yaw)
        lu_bound, ll_bound,IsConvex = generate_convex_space(self.ObsTransformer.obs_s_set,
                                                            self.ObsTransformer.obs_l_set,
                                                            self.ObsTransformer.obs_length_set,
                                                            self.ObsTransformer.obs_width_set, ego_d) # 凸空间与上下界生成

        if self.RoutePlanner.last_lane_id == 53 or self.RoutePlanner.last_lane_id == 6 or self.RoutePlanner.last_lane_id == 56 or self.RoutePlanner.last_lane_id == 209:
            lu_bound[:] = 2
            ll_bound[:] = -2

        state = [ego_d, ego_vd * 0.1, ego_ad * 0.1, ego_state.v]
        self.RoutePlanner.CheckTurning(self.LaneChange)# 换道可行性判断
        self.GetLaneChangeSpaceTest() # 换道意图生成
        self.PathPlanner.w_ref = 2
        # 根据左右换道计算参考轨迹，并清空换道意图
        if   (self.LaneChange ==  1 and self.ForWardSpace > 8 and self.BackWardSpace > max(12,3*ego_vs)) :
            ref_s = self.GetRefS(state, -4)
            self.BehaivorPlanner.WaitChangeRight = 0
            self.BehaivorPlanner.WaitChangeLeft = 0
        elif (self.LaneChange == -1 and self.ForWardSpace > 8 and self.BackWardSpace > max(12,3* ego_vs)) :
            ref_s = self.GetRefS(state,4)
            self.BehaivorPlanner.WaitChangeLeft = 0
            self.BehaivorPlanner.WaitChangeRight = 0
        else:

            self.LaneChange = is_lane_change(lu_bound,ll_bound)
            if self.LaneChange  == -1 and self.RoutePlanner.last_lane_id not in self.RoutePlanner.ForbiddenPath:
                self.PathPlanner.w_ref = 2
                ref_s = self.GetRefS(state,4)
            elif self.LaneChange == 1 and self.RoutePlanner.last_lane_id not in self.RoutePlanner.ForbiddenPath:
                self.PathPlanner.w_ref = 2
                ref_s = self.GetRefS(state, -4)
            else:
                self.PathPlanner.w_ref = 0.2
                ref_s = [0] * 50
        try:
            self.PathPlanner.Update(state, ref_s, ll_bound, lu_bound)
            frenet_path=self.PathPlanner.Solve()
        except:
            if len(observation.object_info['construction']) > 0:
                self.LaneChange = -1
                ref_s = self.GetRefS(state, 4)
                lu_bound[:] = 6
                ll_bound[:] = -6
                self.PathPlanner.Update(state, ref_s, ll_bound, lu_bound)
                frenet_path = self.PathPlanner.Solve()
        self.path = []
        for i in range(50):
            p = self.RefSpline.frenet_to_cartesian1D([ego_s+i, frenet_path[i]])
            self.path.append(p)
        self.path = np.array(self.path)
        # 判断是否为避障或换道轨迹
        self.TruningPath = True if ((max(abs(frenet_path[:20])) > abs(ego_d) + 1.25) or (max(abs(frenet_path[:25])) > 3)) else False
        # print("frenet_Path:", frenet_path, "ego_d", ego_d,"TurningPath:",self.TruningPath)
        self.PlanningPath = Spline2d(self.path[:,0],self.path[:,1])
        # 记录规划时间戳
        self.PathPlanningTime = time.time()

    @staticmethod
    def GetRefS(state, offset):
        # gap = 0.3
        ref_s = np.ones(50) * offset
        num = max(int(state[-1] * 3.5), 1)
        n = max(int(state[-1]) + 2, 8)
        gap = abs(offset - state[0]) / num
        for i in range(n): ref_s[i] = state[0]
        for i in range(n, num + n):
            ref_s[i] = state[0] + (i - n) * gap if offset > 0 else state[0] - (i - n) * gap
        return ref_s

    # def CheckStop(self,observation):
    #     if (len(observation.object_info['crossing']) > 0 and
    #             self.RoutePlanner.last_lane_id == 201 and
    #             self.StopForSecond == False and
    #             observation.ego_info.v < 0.3):
    #         print("---------------------stop---------------------")
    #         self.StopForSecond = True
    #         time.sleep(5)


    def act(self, observation: Observation) -> List[float]:
        VehState = VehicleState(observation.ego_info.x,
                                observation.ego_info.y,
                                deg2_pi(observation.ego_info.yaw),
                                observation.ego_info.v,
                                observation.ego_info.a * 9.8) # 车辆状态集合

        planning_gap = 8 # 规划周期间隔
        self.cur_time = time.time() # 当前规划时间
        dist = math.hypot(observation.ego_info.x - self.Total_Global_Traj[-1][0],
                          observation.ego_info.y - self.Total_Global_Traj[-1][1])
        if self.ReachEnd == False: self.CheckStopForEnd(dist,observation) # 检查是否到达终点
        if self.time % planning_gap == 0: # 到达规划周期的整数倍进行规划
            if self.ReachEnd == False:
                self.UpdataPartTraj(observation)
            else:
                self.incurve = 4
                self.SpeedPlanning(observation)

        self.PassStopLine(observation) # 检查是否通过停止线
        time_gap = self.cur_time - self.planning_time

        Target_speed  = self.Speed_Curve.calculate_approximation(time_gap + 0.1) # 目标速度计算
        Target_acc    = self.Acc_Curve.calculate_approximation(time_gap + 0.1) # 目标加速度计算
        self.last_acc = Target_acc

        Lon_pid = self.LonCon.PID_control(VehState, Target_speed) * 0.5
        Lon = min(Target_acc*0.9 + Lon_pid,0.8)
        if -2 < Lon < 0 and self.Solved == True and self.RoutePlanner.last_lane_id in self.ST_Graph.ReduceLane: Lon *= 2
        elif -2 < Lon < 0 and self.Solved == True: Lon *= 1.32
        elif self.Solved == False and self.RoutePlanner.last_lane_id in self.ST_Graph.ReduceLane: Lon *= 2
        elif Lon < -0.5 and VehState.v < 3/3.6: Lon = -0.5




        VehState.v += Lon * 0.1
        delta_opt, self.theta_e, self.e_cg = self.LatCon.ComputeControlCommand(VehState, self.TrajectoryAnalyzer) # 横向控制器输出计算
        # print("theta_e:", self.theta_e, "e_cg:", self.e_cg, "ReachEnd: ", self.ReachEnd)
        self.time += 1
        self.GetLight() # 灯光变量计算
        Lon = self.LonFilter.filter(Lon)
        delta_opt = self.LatFilter.filter(delta_opt)
        # self.CheckStop(observation)
        return [Lon , np.rad2deg(delta_opt)]
