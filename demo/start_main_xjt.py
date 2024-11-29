import os 
from pyproj import Proj
from planner.IDM.idm import IDM
from utils.observation import Observation, ObjectStatus
from demo.DemoSenarioDict import DemoSenarioDict
from demo.RosMsg import RosMsg
import time
import numpy as np

target_period = 0.05

def UpdateObs(ego, obss):
    """更新每帧背景信息"""
    lonlat2xy = Proj('+proj=tmerc +lon_0=108.90575652010739 +lat_0=34.37650478465651 +ellps=WGS84')  # 经纬度转xy，84
    observation = Observation()
    # 更新主车
    # use lonlat 2 utm
    ego_x, ego_y = lonlat2xy(ego[1], ego[2], inverse=False)
    observation.ego_info.x = ego_x
    observation.ego_info.y = ego_y
    # use utm
    # observation.ego_info.x = ego[8]
    # observation.ego_info.y = ego[9]

    # det_utm_x = ego_x - ego[8]
    # det_utm_y = ego_y - ego[9]
    # print('utm det x: ', det_utm_x, 'y: ', det_utm_y)

    observation.ego_info.v = ego[3]
    observation.ego_info.a = ego[4]
    observation.ego_info.yaw = ego[5]
    observation.ego_info.width = 2
    observation.ego_info.length = 5
    # 更新从车
    # print('####################')
    observation.erase_object_info()
    for type, one_obs in obss.items():
        # print(type)
        for id, msg in one_obs.items():
            # time match
            now_time = time.time()
            if np.abs(now_time - msg['time']) > 2:
                print('obs time stamp not match')
                print('ros time: ', now_time, ', obs time: ', msg['time'], ', det time: ', np.abs(now_time - msg['time']))
                continue
            # print(msg)
            # temp_x, temp_y = lonlat2xy(msg['x'], msg['y'], inverse=False)
            # msg['x'] = temp_x
            # msg['y'] = temp_y
            # observation.update_object_info(type, id, **msg)
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

def start_main():
    # print("start")
    # os.environ['ROS_MASTER_URI'] = 'http://tegra-ubuntu:11311'
    demo_planner = IDM()                                                # 以planner为基类的IDM跟驰算法
    ros_msg = RosMsg()                                                  # ros接发msg
    scene_info = DemoSenarioDict()                                      # 场景数据，空字典
    scene_info.dict['task_info']['waypoints'] = ros_msg.trajectory      # 更新途经点(全局轨迹)
    demo_planner.init(scene_info.dict)                                  # 规控算法初始化，导入地图、起止点、途经点序列等
    obs = Observation()                                                 # 输入进规控算法的背景数据，主车+从车

    last_time = time.monotonic()
    while True:

        # print('---------------------------- P&C ----------------------------')
        if len(ros_msg.ego) != 0:  
            # print(len(ros_msg.trajectory), ' ==? ', len(scene_info.dict['task_info']['waypoints']), '    ', len(demo_planner.all_line))
            if len(ros_msg.trajectory) != len(demo_planner.all_line) and demo_planner.xodr_map == 0:
                scene_info.dict['task_info']['waypoints'] = ros_msg.trajectory
                demo_planner.init(scene_info.dict)
            obs = UpdateObs(ros_msg.ego, ros_msg.obs)                   # 实时更新每帧背景信息
            # print('ego: ', ros_msg.ego)
            action = demo_planner.act(obs)                              # 运行规控算法，输出规控算法结果：[加速度，前轮转角]
            # print(len(scene_info.dict['task_info']['waypoints']))
            PandaPandaPanda
            action.append(demo_planner.light)
            # print(action)
            ros_msg.PubControl(action)                                  # 发布，规控算法结果
        current_time = time.monotonic()
        detal_time = current_time - last_time
        
        # if detal_time < target_period:
        #     print("detal_time = ", detal_time)
        #     time.sleep(target_period - detal_time)
        # else:
        #     print("warning, loop timeout, detal_time = ", detal_time)
        last_time = time.monotonic()

if __name__ == '__main__':
    start_main()
