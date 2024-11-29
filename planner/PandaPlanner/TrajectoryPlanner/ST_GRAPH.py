import numpy as np
import matplotlib.pyplot as plt
class ST_GRAPH:
    def __init__(self):
        self.type_list = {"vehicle","bicycle","pedestrian","stopline","cone"}
        self.bound = 9.5
        self.ReduceLane = [209,210,56,89,84,45,162,188]
        self.CircleId = [188,162]
    def UpDate(self,Spline,PassStopline,observation,lane_id):
        self.Spline = Spline # 规划路径 ，由三次样条形式实现
        self.Pass = PassStopline # 通过参考线可行性
        self.lane_id = lane_id # 当前路段id
        self.Filter = True if(lane_id == 239 or lane_id == 309 or lane_id == 47 or lane_id == 48 or lane_id == 127 or lane_id == 128) else False # 过滤路段id
        return self.GenerateBound(observation)
    def GenerateBound(self, observation):
        ego_state = observation.ego_info
        ego_s, ego_d, ego_vs, ego_vd = self.Spline.cartesian_to_frenet2D(
            [ego_state.x, ego_state.y, np.deg2rad(ego_state.yaw), ego_state.v])
        self.s = ego_s
        self.v = ego_vs
        ego_frenet_state = [ego_s, ego_d, ego_vs, ego_vd]
        obj_data = []
        for type, data in observation.object_info.items(): #遍历所有障碍物
            for index, value in data.items():
                s, d, vs, vd = self.Spline.cartesian_to_frenet2D([value.x, value.y, np.deg2rad(value.yaw), value.v])

                vd += 1e-8
                if(type not in self.type_list): continue
                if self.Filter:
                    if ( abs(d - ego_d) > 2):
                        continue
                elif self.lane_id in self.CircleId:
                    if abs(d) > 4 or s < ego_s:
                        continue
                else:
                    if (s < ego_s + 2 or (abs(vd) < 0.3 and abs(d - ego_d) > 2 and type != "pedestrian")):
                        continue
                if (type == "stopline"):
                    if (self.Pass == False):
                        s += 3.5
                    else:
                        continue
                if (type == "pedstrain"):
                    left_buffer = 4 # 左缓冲距离
                    right_buffer = -4  # 右缓冲距离
                elif type == "cone":
                    left_buffer = 0.85
                    right_buffer = -0.85
                elif ((abs(vd) > abs(vs) or abs(vd) > 0.3) and type == "vehicle"):
                    if (vd > 0):
                        left_buffer = -5
                        right_buffer = 3
                    else:
                        left_buffer = 3
                        right_buffer = -5
                else:
                    left_buffer = 2.5
                    right_buffer = -2.5
                # 计算靠近至前进方向中心（d = 0）的时间
                t_zero = -d / vd
                # 计算切入行驶区域缓冲时间(|  |car|  |)  +- 2m
                t_bound_1 = left_buffer / vd + t_zero
                t_bound_2 = right_buffer / vd + t_zero

                if (t_bound_1 > t_bound_2):
                    t_max, t_min = t_bound_1, t_bound_2
                else:
                    t_max, t_min = t_bound_2, t_bound_1
                # 如果切出时间太远，则不考虑
                if (t_max < 0.5 or t_min > 5): continue
                # 如果障碍物已经在缓冲范围内了
                if (t_min < 0 and t_max > 0):
                    obs_s_in = s
                    obs_s_out = s + vs * t_max
                    obs_t_in = 0
                    obs_t_out = t_max
                else:
                    obs_s_in = s + vs * t_min
                    obs_s_out = s + vs * t_max
                    obs_t_in = t_min
                    obs_t_out = t_max
                obj_data.append([obs_s_in, obs_s_out, obs_t_in, obs_t_out, type])
                print("object_sd:", s, d, vs, vd, "ego_sd:", ego_s, ego_d,ego_vs,ego_vd,"obj_data:",obj_data[-1])
        ego_s_list = [ego_s + t * ego_vs for t in np.arange(0, 5.01, 0.25)]
        plt.plot(np.arange(0, 5.01, 0.25), ego_s_list, linestyle='-.', color="green")
        lower_bound, upper_bound = self.DenseData(obj_data, ego_frenet_state)
        return lower_bound, upper_bound
    def DenseData(self,obs_data, ego_state):
        '''
        function ： 稠密化 ST图数据
        '''
        lower_bound = [0] * 21
        upper_bound = [50] * 21
        for data in obs_data:
            # 障碍物ST图 线性拟合 y = kx + b
            k = (data[1] - data[0]) / (data[3] - data[2])
            b = data[0] - k * data[2]
            if(np.isnan(k) or np.isnan(b)): continue # 非空处理
            # 时间分配至0.25s
            t_index = np.arange(max((data[2] // 0.25 - 2), 0), min((data[3] // 0.25) + 1, 21), 1, dtype=int)
            t_new = t_index * 0.25
            s_new = k * t_new + b

            # 如果匀速能够超越,则进行超车，否则进行避让
            if (((ego_state[0] + ego_state[2] * t_new[0] > s_new[0]))
                    and data[-1] != "pedestrian"):
                for i in range(len(t_index)):
                    lower_bound[t_index[i]] = max(lower_bound[t_index[i]], s_new[i] + 4)
            else:
                for i in range(len(t_index)):
                    bound = 11 if (data[-1] == "pedestrian") else self.bound
                    if self.Filter: bound = 8
                    if data[-1] == "cone": bound = 1
                    upper_bound[t_index[i]] = min(upper_bound[t_index[i]], s_new[i] - bound)
                    # if(upper_bound[t_index[i]] < self.s and self.v < 0.3 and data[-1] == "pedestrian"):
                    #     upper_bound[t_index[i]] = self.s + 1e-2
                    if(upper_bound[t_index[i]] < self.s and self.v < 1):
                        upper_bound[t_index[i]] = self.s + 1e-2

            # plt.plot([data[2], data[3]], [data[0], data[1]], linestyle="dashed")
            # plt.plot(t_new, s_new)
            # print("t_new:",t_new1,"t_init:",[data[2], data[3]])
        # plt.plot(np.arange(0, 5.01, 0.25), lower_bound,marker="x",color='blue')
        # plt.plot(np.arange(0, 5.01, 0.25), upper_bound,marker="x",color = 'r')
        # plt.ylim(-10, 60)
        # plt.xlim(-0.5,5.5)
        # plt.pause(0.01)
        # plt.cla()
        return lower_bound,upper_bound