import os
import time
import numpy as np
from matplotlib import pyplot as plt
from pyproj import Proj
from planner.PandaPlanner.PandaTracker import PandaPlanner
from utils.observation import Observation, ObjectStatus
from demo.DemoSenarioDict import DemoSenarioDict
from demo.RosMsg import RosMsg
from planner.PandaPlanner.TrajectoryPlanner.temp_utils import are_dicts_empty
from planner.PandaPlanner.debug import draw,debug_draw
from planner.PandaPlanner.TrajectoryPlanner.temp_utils import count_time_args,GetInitTraj
from planner.PandaPlanner.SloveMap.RoutePlanner import *
from planner.PandaPlanner.PathPublisher import PathPublisher

def UpdateObs(ego, obss):
    """更新每帧背景信息"""
    lonlat2xy = Proj(proj='utm',zone=49,ellps='WGS84', preserve_units='m')  # 西安 zone 49， 北京 zone 50
    observation = Observation()
    ego_x, ego_y = lonlat2xy(ego[1], ego[2], inverse=False)
    observation.ego_info.x = ego_x
    observation.ego_info.y = ego_y
    observation.ego_info.v = ego[3]
    observation.ego_info.a = ego[4]
    observation.ego_info.yaw = ego[5]
    observation.ego_info.width = 2
    observation.ego_info.length = 5
    observation.erase_object_info()
    for type, one_obs in obss.items():
        for id, msg in one_obs.items():
            # time match
            now_time = time.time()
            # if np.abs(now_time - msg['time']) > 2:
            #     print('obs time stamp not match')
            #     print('ros time: ', now_time, ', obs time: ', msg['time'], ', det time: ', np.abs(now_time - msg['time']))
            #     continue
            if type in observation.object_info.keys():
                obj_name = str(id)
                if obj_name not in observation.object_info[type].keys():
                    observation.object_info[type][obj_name] = ObjectStatus()
                observation.object_info[type][obj_name].x = msg['x']
                observation.object_info[type][obj_name].y = msg['y']
                observation.object_info[type][obj_name].v = msg['v']
                observation.object_info[type][obj_name].a = msg['a']
                observation.object_info[type][obj_name].yaw = msg['yaw']
                observation.object_info[type][obj_name].width = msg['width']
                observation.object_info[type][obj_name].length = msg['length']
                observation.object_info[type][obj_name].time = msg['time']
    return observation


lonlat2xy = Proj(proj='utm', zone=49, ellps='WGS84', preserve_units='m')


def start_main():
    # os.environ['ROS_MASTER_URI'] = 'http://192.168.1.104:11311'
    # os.environ['ROS_IP'] = '192.168.1.104'
    # os.environ['ROS_HOSTNAME'] = '192.168.1.105'
    print("start,CurrentWorkSpace:",os.getcwd())
    demo_planner = PandaPlanner()
    ros_msg = RosMsg()  # ros接发msg
    scene_info = DemoSenarioDict()  # 场景数据，空字典
    traj = WGS842TmercPath(np.load("PandaData/global_path.npy"))
    ros_msg.trajectory = traj


    while True:
        if (len(ros_msg.trajectory) > 0):
            print("receive trajectory")
            scene_info.dict['task_info']['waypoints'] = ros_msg.trajectory
            break
    # traj = WGS842TmercPath(np.load("/test/onsite-structured-test-master/PandaData/global_path.npy"))
    traj = WGS842TmercPath(np.load("PandaData/global_path.npy"))
    traj[-1][0] += 3*(traj[-1][0] - traj[-2][0])
    traj[-1][1] += 3*(traj[-1][1] - traj[-2][1])
    ros_msg.trajectory = traj
    scene_info.dict['task_info']['waypoints'] = ros_msg.trajectory
    obs = Observation()
    last_obs = obs.object_info
    loss_num = 0
    traj_x = []
    traj_y = []
    demo_planner.init(scene_info.dict)  # 规控算法初始化，导入地图、起止点、途经点序列等
    # 输入进规控算法的背景数据，主车+从车
    while True:
        if len(ros_msg.ego) != 0:
            try:
                obs = UpdateObs(ros_msg.ego, ros_msg.obs)  # 实时更新每帧背景信息
                if are_dicts_empty(obs) and loss_num < 30:
                    obs.object_info = last_obs
                    loss_num += 1
                else:
                    loss_num = 0
                    last_obs = obs.object_info
            except: continue
            obs_tmerc = ObservationCoodernateTransform(obs)
            # print("ego_info",obs_tmerc.ego_info)
            # plt.clf()
            # plt.subplot(211)
            action = demo_planner.act(obs_tmerc)  # 运行规控算法，输出规控算法结果：[加速度，前轮转角]

            # print("main_stop_line",obs.object_info['stopline'])
            # plt.cla()
            action.append(ros_msg.ego[3])
            action.append(demo_planner.light)
            # action.append(1)
            # print("light_info:",demo_planner.light," ",action[-1])

            # save the historical trajectory
            # traj_x.append(obs.ego_info.x)
            # traj_y.append(obs.ego_info.y)



            # plt.subplot(212)
            # ll_bound, lu_bound = demo_planner.get_bound(obs)
            # plt.plot(ll_bound[:, 0], ll_bound[:, 1], color='red')
            # plt.plot(lu_bound[:, 0], lu_bound[:, 1], color='red')
            # draw(traj_x, traj_y, scene_info.dict['task_info']['waypoints'], obs.ego_info, obs.object_info, demo_planner, p,demo_planner.light)
            # debug_draw(traj_x, traj_y, scene_info.dict['task_info']['waypoints'], obs.ego_info, obs.object_info,demo_planner.LaneChange)
            # if (demo_planner.ParkingTrajectory != None):
            #     ParkTraj = demo_planner.ParkingTrajectory.GetPath()
            #     plt.plot(ParkTraj[:,0],ParkTraj[:,1])
            # if len(demo_planner.path) > 0:
            #     planning_path = demo_planner.PlanningPath.GetPath()
            #     ros_msg.PublishPlanningPath(planning_path)
                # plt.plot(planning_path[:,0],planning_path[:,1], color='green',label="PJPO")
            # if len(demo_planner.RefPath) > 0:
                # RefPath = np.array(demo_planner.RefPath)
            #     plt.plot(RefPath[:,0],RefPath[:,1],color="blue",label="RefPath",linewidth=3)


            # plt.legend()
            # plt.pause(0.01)
            # plt.cla()
            # ros_msg.EgoPublish(obs,demo_planner.LaneChange)
            # ros_msg.obsPub(obs)
            # plt.pause(0.01)
            # plt.ioff()

            # print("action:", action)

            # print("construction:",obs.object_info['construction'])
            ros_msg.PubControl(action)  # 发布，规控算法结果



if __name__ == '__main__':
    start_main()
