import time

import numpy as np
from utils.observation import Observation
from pyproj import Proj
from planner.PandaPlanner.TrajectoryPlanner.CubicSpline import Spline2D as sp
import matplotlib.pyplot as plt
M_PI = np.pi
class VehicleState:
    def __init__(self, x=0.0, y=0.0, yaw=0.0,
                 v=0.0, a=0.0, e_cg=0, theta_e=0, gear=1):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.a = a
        self.e_cg = e_cg
        self.theta_e = theta_e

        self.gear = gear

        self.light = 0
        self.xodr_map = 0
def deg2_pi(angle):
    """
    regulate theta to -pi ~ pi.
    :param angle: input angle
    :return: regulated angle
    """
    angle = np.deg2rad(angle)

    if angle > M_PI:
        return angle - 2.0 * M_PI

    if angle < -M_PI:
        return angle + 2.0 * M_PI
    return angle


def pi_2_pi(angle):
    """
    regulate theta to -pi ~ pi.
    :param angle: input angle
    :return: regulated angle
    """
    if angle > M_PI:
        return angle - 2.0 * M_PI

    if angle < -M_PI:
        return angle + 2.0 * M_PI
    return angle


def deg_2_deg(angle):
    """
    regulate theta to -pi ~ pi.
    :param angle: input angle
    :return: regulated angle
    """
    if angle > 180:
        return angle - 2.0 * 180

    if angle < -180:
        return angle + 2.0 * 180
    return angle


class TrajectoryAnalyzer:
    def __init__(self, spline):
        self.Spline = spline


    def ToTrajectoryFrame(self, vehicle_state):
        """
        errors to trajectory frame
        theta_e = yaw_vehicle - yaw_ref_path
        e_cg = lateral distance of center of gravity (cg) in frenet frame
        :param vehicle_state: vehicle state (class VehicleState)
        :return: theta_e, e_cg, yaw_ref, k_ref
        """
        yaw = vehicle_state.yaw
        sd = self.Spline.cartesian_to_frenet1D([vehicle_state.x, vehicle_state.y])
        yaw_ref = self.Spline.calc_yaw(sd[0] + vehicle_state.v * 0.125)
        k_ref = self.Spline.calc_curvature(sd[0] + vehicle_state.v * 0.5)
        theta_e = pi_2_pi(yaw - yaw_ref)
        e_cg = sd[1]

        return theta_e, e_cg, yaw_ref, k_ref

def get_indice(traj):
    diff = np.diff(traj, axis=0)
    sum = np.cumsum(np.linalg.norm(diff, axis=1))
    indice = [0]
    d = 0.5
    for i in range(len(sum)):
        if (sum[i] < d * 5):
            continue
        else:
            indice += [i]
            d += 1
    if (indice[-1] != len(traj) - 1):
        indice += [len(traj) - 1]
    return indice
def are_dicts_empty(obs):
    for type, value in obs.object_info.items():
        if len(value) > 0:
            return False
    return True
def count_time_args(msg=None):
    def count_time(func):
        def wrapper(*args, **kwargs):
            nonlocal msg
            if msg is None:
                msg = func.__name__
            t1 = time.time()
            result = func(*args, **kwargs)
            print(f"[{msg}] time is", time.time() - t1)
            return result

        return wrapper

    return count_time

lonlat2xy = Proj(proj='utm', zone=49, ellps='WGS84', preserve_units='m')
def GetInitTraj(traj,observation):
    ego_position = np.array([observation.ego_info.x,observation.ego_info.y])
    # init_traj    = np.array(traj).T
    init_traj    = np.array(traj)
    UTM_Traj     = np.array(lonlat2xy(init_traj[:, 0], init_traj[:, 1])).T
    # UTM_Traj     = init_traj
    UTM_Traj += np.random.rand(*UTM_Traj.shape) / 1e8
    Spline = sp(UTM_Traj[:50, 0], UTM_Traj[:50, 1])
    s,d = Spline.cartesian_to_frenet1D(ego_position)
    if(s < 1e-2):
        plt.plot(UTM_Traj[:,0],UTM_Traj[:,1])
        add_traj = np.linspace(ego_position,UTM_Traj[0])
        plt.plot(add_traj[:,0],add_traj[:,1])
        plt.show()
        UTM_Traj = np.vstack((add_traj,UTM_Traj))

    return UTM_Traj

def CheckConvex(Lower_bound,Upper_bound):
    if Lower_bound <= Upper_bound:
        return True
    return False
import numpy as np


class FirstOrderLowPassFilter:
    def __init__(self, cutoff_freq = 25, sample_freq = 50):
        """
        初始化一阶低通滤波器。
        参数:
        cutoff_freq -- 截止频率（Hz）。
        sample_freq -- 采样频率（Hz）。
        """
        self.alpha = 1 / (1 + 2 * np.pi * cutoff_freq / sample_freq)
        self.prev_output = None

    def filter(self, new_sample):
        """
        对新的样本数据进行滤波。
        参数:
        new_sample -- 新的样本数据。
        返回:
        filtered_sample -- 滤波后的样本数据。
        """
        if self.prev_output is None:
            self.prev_output = new_sample  # 第一个样本直接作为输出
            return new_sample
        else:
            filtered_sample = self.alpha * new_sample + (1 - self.alpha) * self.prev_output
            self.prev_output = filtered_sample
            return filtered_sample

if __name__ == '__main__':
    LowerBound = [0] * 50
    UpperBound = [10] * 50
    CheckConvex(LowerBound,UpperBound)
