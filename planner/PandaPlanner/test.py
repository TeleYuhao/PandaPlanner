import numpy as np
import matplotlib.pyplot as plt
import math
import rospy
from perception_msgs.msg import PerceptionLocalization
from perception_msgs.msg import PerceptionObjects
from draw import draw_car,draw_bicycle
from utils import *
from CubicSpline import Spline2D


global_traj = np.load("map/transform.npy")
global_traj += np.random.rand(*global_traj.shape) / 1e8
index = get_indice(global_traj)


class replay:
    def __init__(self) -> None:
        self.global_traj = global_traj
        self.sparse_traj = global_traj[index]
        self.traj_x = []
        self.traj_y = []
        self.yaw = 0
        self.obj = None
        self.time = 0
        self.ind_old = 0
        self.follow_id = -1
        rospy.init_node("debug",anonymous=True)
        self.node = rospy.Subscriber("/cicv_location",PerceptionLocalization,self.callback)
        self.perception_node = rospy.Subscriber("/tpperception",PerceptionObjects,self.perception_callback)
    
    def callback(self,data):
        self.traj_x.append(data.position_x)
        self.traj_y.append(data.position_y)
        self.yaw = data.yaw
    def perception_callback(self,data):
        self.obj = data.objs

    def get_trajectory_part(self, traj, index, min_distance):
        total_distance = 0
        start_index, end_index = index, index
        while end_index < len(traj) - 1 and total_distance < min_distance:
            current_distance = np.linalg.norm(traj[end_index + 1] - traj[end_index])
            total_distance += current_distance
            end_index += 1

        return self.sparse_traj[max(0, index - 2):end_index + 1]
    
    def UpdataPartTraj(self, index):
        '''
        获取局部路径
        :return:
        '''
        # use heading point to make following trajectory
        self.traj_part = self.get_trajectory_part(self.sparse_traj, index, min_distance=70)
        self.spline = Spline2D(self.traj_part[:,0],self.traj_part[:,1])
        self.path_x,self.path_y = self.spline.GetPath()



    def planning(self):
        if(self.time%10 == 0):
            x = self.traj_x[-1]
            y = self.traj_y[-1]
            dxy = np.array([[x - ix, y - iy] for ix, iy in
                            self.sparse_traj[self.ind_old: self.ind_old + 50]])
            ind_add = int(np.argmin(np.hypot(dxy[:, 0], dxy[:, 1])))
            self.ind_old += ind_add
            self.UpdataPartTraj(self.ind_old)
        self.time += 1

    def draw_obj(self):
        for ob in (self.obj):
            if(ob.type == 0):
                if(self.follow_id == int(ob.id)):
                    car_color = "orange"
                else:
                    car_color = "red"
                draw_car(ob.xabs,ob.yabs,np.deg2rad(ob.heading),0,color=car_color)
            elif(ob.type == 2):
                plt.scatter(ob.xabs,ob.yabs,color="blue")
            elif(ob.type == 3):
                if(self.follow_id == int(ob.id)):
                    bicycle_color = "orange"
                else:
                    bicycle_color = "green"
                draw_bicycle(ob.xabs,ob.yabs,np.deg2rad(ob.heading),0,color=bicycle_color)

    def get_following_obj(self):
        self.follow_id = -1
        current_s,current_d = self.spline.cartesian_to_frenet1D(self.traj_x[-1], self.traj_y[-1])
        ped_list = []
        obs_list = []
        for ob in (self.obj):
            s,d = self.spline.cartesian_to_frenet1D(ob.xabs,ob.yabs)
            if(s < current_s): continue
            if(ob.type == 2 and d > -1 and s > current_s):
                ped_list.append([ob.id,s,d,ob.heading,0])
            elif((s < current_s or abs(current_d - d) > 2.2)): continue
            else:
                obs_list.append([ob.id,s,d,ob.heading,ob.speed])
        if(len(ped_list)>0):
            self.follow_id = -2
            ped_list = np.array(ped_list)
            return min(ped_list[:,1]),0
        elif(len(obs_list) > 0):
            obs_list = np.array(obs_list)
            index = np.argmin(obs_list[:,1])
            self.follow_id = obs_list[index][0]
            return obs_list[index][1],obs_list[index][-1]
        else:
            return 100,30/3.6

            

    def draw_debug(self):
        plt.plot(self.global_traj[:,0],self.global_traj[:,1],label="global")
        plt.plot(self.traj_x,self.traj_y,label = "tracking")

        plt.plot(self.path_x,self.path_y,label = "planning")

        draw_car(self.traj_x[-1],self.traj_y[-1],np.deg2rad(self.yaw),0)
        plt.xlim(self.traj_x[-1] - 50, self.traj_x[-1]+50)
        plt.ylim(self.traj_y[-1]-50,self.traj_y[-1]+50)
        plt.legend()
        plt.pause(0.01)
        plt.cla()

def main():
    r = replay()
    while True:
        if(len(r.traj_x)>0):
            r.planning()
            r.draw_debug()
            r.draw_obj()
            print(r.get_following_obj())
            
main()