import numpy as np
import math
import matplotlib.pyplot as plt
from planner.PandaPlanner.TrajectoryPlanner.temp_utils import count_time_args
# Vehicle config
wheelbase = 2.33  # wheel base: front to rear axle [m]
wheeldist = 1.85  # wheel dist: left to right wheel [m]
v_w = 2.33  # vehicle width [m]
r_b = 0.80  # rear to back [m]
r_f = 3.15  # rear to front [m]
t_r = 0.40  # tire radius [m]
t_w = 0.30  # tire width [m]


def draw(traj_x, traj_y, global_traj, ego_info, object_info, demo_planner, p,direction):
    vel = object_info['vehicle']
    for id, value in vel.items():
        draw_car(value.x, value.y, np.deg2rad(value.yaw), 0)
    ped = object_info['pedestrian']
    for id, value in ped.items():
        plt.scatter(value.x, value.y, s=50, color='blue', marker='o')
    byc = object_info["bicycle"]
    for id, value in byc.items():
        draw_bicycle(value.x, value.y,np.deg2rad(value.yaw),color='green')
    for id,value in object_info['stopline'].items():
        draw_bicycle(value.x, value.y,np.deg2rad(value.yaw),color='red')

    for id,value in object_info['construction'].items():
        draw_construction(value.x,value.y,value.length,value.width, np.deg2rad(value.yaw))
        print("construction_yaw:",value.yaw)
    if direction == 0 :
        color = "black"
    elif direction == -1:
        color = "red"
    else:
        color = "blue"
    draw_car(ego_info.x, ego_info.y, np.deg2rad(ego_info.yaw), 0,color = color)
    plt.plot(global_traj[:, 0], global_traj[:, 1],color='red',label = "global")
    plt.plot(traj_x, traj_y,color='orange',label="ego")
    plt.plot(demo_planner.traj_part[:, 0], demo_planner.traj_part[:, 1],color="black",label="to be intercoperate")
    plt.plot(p[:, 0], p[:, 1],color='green',label="planning")
    plt.xlim(traj_x[-1]-50, traj_x[-1]+50)
    plt.ylim(traj_y[-1]-50, traj_y[-1]+50)
    plt.legend()
    plt.pause(0.01)
    plt.cla()


def debug_draw(traj_x, traj_y, global_traj, ego_info, object_info,LaneChange):
    vel = object_info['vehicle']
    for id, value in vel.items():
        draw_car(value.x, value.y, np.deg2rad(value.yaw), 0)
    ped = object_info['pedestrian']
    for id, value in ped.items():
        plt.scatter(value.x, value.y, s=50, color='blue', marker='o')
    byc = object_info["bicycle"]
    for id, value in byc.items():
        draw_bicycle(value.x, value.y,np.deg2rad(value.yaw),color='green')
    for id,value in object_info['stopline'].items():
        draw_bicycle(value.x, value.y,np.deg2rad(value.yaw),color='red')
    for id,value in object_info['crossing'].items():
        draw_bicycle(value.x, value.y,np.deg2rad(value.yaw),color='blue')
    for id,value in object_info['parking'].items():
        draw_bicycle(value.x, value.y,np.deg2rad(value.yaw),color='red')

    for id,value in object_info['construction'].items():
        draw_construction(value.x,value.y,value.length,value.width, np.deg2rad(value.yaw))
    for id,value in object_info['tunnel'].items():
        draw_construction(value.x,value.y,value.length,value.width, np.deg2rad(value.yaw))
    if LaneChange == 0:
        color = 'black'
    elif LaneChange == 1:
        color = 'blue'
    else:
        color = 'red'
    draw_car(ego_info.x, ego_info.y, np.deg2rad(ego_info.yaw), 0,color = color)
    plt.plot(global_traj[:, 0], global_traj[:, 1],color='red',label = "global")
    plt.plot(traj_x, traj_y,color='orange',label="ego")
    # plt.plot(demo_planner.traj_part[:, 0], demo_planner.traj_part[:, 1],color="black",label="to be intercoperate")
    plt.xlim(traj_x[-1]-50, traj_x[-1]+50)
    plt.ylim(traj_y[-1]-50, traj_y[-1]+50)
    plt.legend()

def draw_car(x, y, yaw, steer, color='black'):
    car = np.array([[-r_b, -r_b, r_f, r_f, -r_b],
                    [v_w / 2, -v_w / 2, -v_w / 2, v_w / 2, v_w / 2]])

    wheel = np.array([[-t_r, -t_r, t_r, t_r, -t_r],
                      [t_w / 2, -t_w / 2, -t_w / 2, t_w / 2, t_w / 2]])

    rlWheel = wheel.copy()
    rrWheel = wheel.copy()
    frWheel = wheel.copy()
    flWheel = wheel.copy()

    Rot1 = np.array([[np.cos(yaw), -np.sin(yaw)],
                     [np.sin(yaw), np.cos(yaw)]])

    Rot2 = np.array([[np.cos(steer), np.sin(steer)],
                     [-np.sin(steer), np.cos(steer)]])

    frWheel = np.dot(Rot2, frWheel)
    flWheel = np.dot(Rot2, flWheel)

    frWheel += np.array([[wheelbase], [-wheeldist / 2]])
    flWheel += np.array([[wheelbase], [wheeldist / 2]])
    rrWheel[1, :] -= wheeldist / 2
    rlWheel[1, :] += wheeldist / 2

    frWheel = np.dot(Rot1, frWheel)
    flWheel = np.dot(Rot1, flWheel)

    rrWheel = np.dot(Rot1, rrWheel)
    rlWheel = np.dot(Rot1, rlWheel)
    car = np.dot(Rot1, car)

    frWheel += np.array([[x], [y]])
    flWheel += np.array([[x], [y]])
    rrWheel += np.array([[x], [y]])
    rlWheel += np.array([[x], [y]])
    car += np.array([[x], [y]])

    plt.plot(car[0, :], car[1, :], color)
    plt.plot(frWheel[0, :], frWheel[1, :], color)
    plt.plot(rrWheel[0, :], rrWheel[1, :], color)
    plt.plot(flWheel[0, :], flWheel[1, :], color)
    plt.plot(rlWheel[0, :], rlWheel[1, :], color)
    plt.axis("equal")
    # plt.show()
def draw_bicycle(x, y, yaw, color='black'):
    car = np.array([[-r_b, -r_b, r_f, r_f, -r_b],
                    [v_w / 2, -v_w / 2, -v_w / 2, v_w / 2, v_w / 2]])


    Rot1 = np.array([[np.cos(yaw), -np.sin(yaw)],
                     [np.sin(yaw), np.cos(yaw)]])
    car = np.dot(Rot1, car)

    car += np.array([[x], [y]])

    plt.plot(car[0, :], car[1, :], color)
    plt.axis("equal")
    # plt.show()

def draw_construction(x,y,length,width,yaw):
    car = np.array([[-length/2, -length/2, length/2, length/2, -length/2],
                    [width / 2, -width / 2, -width / 2, width / 2, width / 2]])

    Rot1 = np.array([[np.cos(yaw), -np.sin(yaw)],
                     [np.sin(yaw), np.cos(yaw)]])
    car = np.dot(Rot1, car)

    car += np.array([[x], [y]])

    plt.plot(car[0, :], car[1, :], color="red")
    plt.axis("equal")