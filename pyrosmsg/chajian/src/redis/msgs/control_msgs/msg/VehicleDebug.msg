Header header
# 蠕行
int16    flag_crawl
float64 crawl_speed_error               # 蠕行速度误差
float64 crawl_veh_accel                 # 蠕行加速度指令
float64 crawl_acc_pel                   # 油门踏板开度
float64 crawl_brk_pel                   # 制动踏板开度

# 纵向控制
# 位置pid
float64 s_matched                       # 匹配点s  
float64 station_preview                 # 预瞄点s     
float64 station_error                   # 预瞄点位置误差      
float64 station_error_limited           # 限幅后的纵向位置误差
float64 speed_offset                    # 位置pid得到的速度补偿
# 速度pid
float64 loc_v                           # 定位车速
float64 ave_acc                         # 平均加速度
float64 speed_preview                   # 预瞄点参考车速
float64 speed_matched                   # 匹配点车速
float64 preview_speed_error             # 预瞄点速度误差   
float64 speed_controller_input_limited  # 限幅后的纵向速度误差
float64 acceleration_cmd_closeloop      # 速度pid得到的加速度补偿 or 匀加速运动公式计算得到的加速度 + 速度误差积分补偿
# 加速度
float64 path_remain                     # 距离停车点的距离
float64 acceleration_lookup             # 纵向控制计算出用于查表的加速度
float64 throttle_cmd                    # 查表得到的油门开度
float64 brake_cmd                       # 查表得到的制动踏板开度
float64 idx_preview                     # 用于计算加减速的预瞄点序号
float64 acceleration_preview            # 使用匀加速运动公式计算得到的加速度

# 横向控制    
int16   lat_target_pointIdx             # 目标点索引序号
float64 lat_target_x                    # 目标点x
float64 lat_target_y                    # 目标点y
float64 cmd_strangle                    # 方向盘转角指令
float64 heading_error                   # 用于方向盘转角补偿的航向角误差
float64 steer_angle                     # 纯跟踪算法得到的方向盘转角指令    
float64 steer_angle_limited             # 根据最大侧向加速度限制后的方向盘转角
float64 strg_delta_limited              # 根据转角增量限制后的方向盘转角

# 状态相关标志位
int16   mode                            # 当前驾驶模式：0紧急制动,1丢失定位或规划，2蠕行模式，3正常行车模式，4档位不匹配停车，5有终点正常停车
int16   idx_trajectory_local_set        # 局部轨迹集合索引序号
int16   idx_matched_point               # 局部轨迹集合索引序号
int16   idx_stop                        # 轨迹终点索引序号
int16   flag_replan                     # 重规划标志位
float64 cur_gear                        # 当前档位:1P2R3N4D
float64 tar_gear                        # 目标档位
int16   flag_trajectory                 # 轨迹正常标志位
int16   flag_loc                        # 定位正常标志位
int16[] start_indices                   # 存储每段轨迹的起始索引  
int16[] end_indices                     # 存储每段轨迹的结束索引 
float64 dist2terminal                   # 到终点的距离
int16   flag_terminal                   # 终点停车标志位
float64 max_diff_time                   # 程序循环运行耗时
int16   flag_deviate                    # 轨迹和定位偏离太远