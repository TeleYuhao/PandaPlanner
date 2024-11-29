import math
import numpy as np
from pyproj import Proj
from planner.PandaPlanner.smoother import path_smoother
class Global_Path_Analyser:
    def __init__(self):
        # init the lonlat 2 xy frame
        self.lonlat2xy = Proj(proj='utm', zone=49, ellps='WGS84', preserve_units='m')
        # init the reference line smoother
        self.smoother = path_smoother()

    def trajectory_transform(self,traj):
        '''
        transform the init fem trajectory 2 smooth trajectory
        :param traj: trajectory after smoother
        :return:
        '''
        traj_xy = []
        for t in traj:
            xy_t = self.lonlat2xy(t[0], t[1], inverse=False)
            traj_xy.append([xy_t[0],xy_t[1]])
        traj_xy = self.smoother.solve(np.array(traj_xy))
        yaw = np.zeros((len(traj_xy),1))
        for i in range(1,len(traj_xy)):
            dx = traj_xy[i][0] - traj_xy[i-1][0]
            dy = traj_xy[i][1] - traj_xy[i-1][1]
            yaw[i] = math.atan2(dy,dx)
        yaw[0] = yaw[1]
        yaw[-1] = yaw[-2]
        yaw = np.rad2deg(yaw)
        traj_xy = np.hstack((traj_xy,yaw))
        return np.array(traj_xy)
    def calc_curvature(self,x, y, directions = 1):
        '''
        function: calculate the curvature of trajectory
        :param x: the x of trajectory
        :param y: the y of trajectory
        :param directions: the going direction of vehicle
        :return:
        '''
        c, ds = [], []

        for i in range(1, len(x) - 1):
            dxn = x[i] - x[i - 1] + 1e-5
            dxp = x[i + 1] - x[i] + 1e-5
            dyn = y[i] - y[i - 1] + 1e-5
            dyp = y[i + 1] - y[i] + 1e-5
            dn = math.hypot(dxn, dyn)
            dp = math.hypot(dxp, dyp)
            dx = 1.0 / (dn + dp) * (dp / dn * dxn + dn / dp * dxp)
            ddx = 2.0 / (dn + dp) * (dxp / dp - dxn / dn)
            dy = 1.0 / (dn + dp) * (dp / dn * dyn + dn / dp * dyp)
            ddy = 2.0 / (dn + dp) * (dyp / dp - dyn / dn)
            curvature = (ddy * dx - ddx * dy) / (dx ** 2 + dy ** 2)
            d = (dn + dp) / 2.0

            if np.isnan(curvature):
                curvature = 0.0


            if len(c) == 0:
                ds.append(d)
                c.append(curvature)

            ds.append(d)
            c.append(curvature)

        ds.append(ds[-1])
        c.append(c[-1])

        return np.array(c).reshape(len(c),1), ds
    def traj_Solve(self,traj):
        '''
        smooth the init fem trajectory and calculate the param of trajectory
        :param traj:
        :return:
        '''
        traj_xy_frame = self.trajectory_transform(traj)
        c,ds = self.calc_curvature(traj_xy_frame[:,0],traj_xy_frame[:,1])
        traj_xy = np.hstack((traj_xy_frame,c))
        return traj_xy


class VehicleState:
    def __init__(self, x=0.0, y=0.0, yaw=0.0,v=0.0,):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.e_cg = 0.0
        self.theta_e = 0.0
        self.steer = 0.0




