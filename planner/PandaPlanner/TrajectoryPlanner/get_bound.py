import matplotlib.pyplot as plt
import numpy as np

UPPER_BOUND = 6.0
LOWER_BOUND = -6.0
HALF_WIDTH = 1.75

def generate_convex_space(
        obs_cen_s_set=None,
        obs_cen_l_set=None,
        obs_length_set=None,
        obs_width_set=None,
        ego_l=0,
        dp_path_s_set=np.arange(0, 50, 1),
):
    """
    基于动态规划的粗解以及障碍物信息得到sl空间下的凸空间
    :param dp_path_s_set: 动态规划路径s坐标序列
    :param dp_path_l_set: 动态规划路径l坐标序列
    :param obs_cen_s_set: 障碍物中心s坐标序列
    :param obs_cen_l_set: 障碍物中心l坐标序列
    :param obs_length_set: 障碍物长度序列
    :param obs_width_set: 障碍物宽度序列
    :return:
    """
    # 初始化边界
    l_lower_bound = np.array([LOWER_BOUND] * len(dp_path_s_set))  # 凸空间下界
    l_upper_bound = np.array([UPPER_BOUND] * len(dp_path_s_set))  # 凸空间上界
    conflict_resolved = True  # 初始化 conflict_resolved

    # 遍历每一个障碍物，先按照正常逻辑判断
    for obs_index in range(0, len(obs_cen_s_set)):
        # 找到在 s 轴上离障碍起点、中心点、终点最近的 dp 轨迹序列
        start_index, cen_index, end_index = find_obs_nearest_s(obs_cen_s=obs_cen_s_set[obs_index],
                                                               obs_length=obs_length_set[obs_index],
                                                               dp_path_s_set=dp_path_s_set)

        if start_index is None or cen_index is None or end_index is None:
            continue

        # 根据 ego_l 和障碍物中心 l 值判断是左绕还是右绕
        if ego_l < obs_cen_l_set[obs_index]:  # 右绕
            for j in range(start_index, end_index + 1):
                l_upper_bound[j] = min(l_upper_bound[j], obs_cen_l_set[obs_index] - obs_width_set[obs_index] / 2 - HALF_WIDTH)
        else:  # 左绕
            for j in range(start_index, end_index + 1):
                l_lower_bound[j] = max(l_lower_bound[j], obs_cen_l_set[obs_index] + obs_width_set[obs_index] / 2 + HALF_WIDTH)

    # 检查每个位置的上下界冲突
    conflict_detected_positions = [i for i in range(len(dp_path_s_set)) if l_lower_bound[i] > l_upper_bound[i]]
    # 如果检测到冲突，只对冲突位置附近的障碍物尝试左右绕
    if conflict_detected_positions:
        print('Conflict detected at positions:', conflict_detected_positions)
        print("obs_cen_s_set", obs_cen_s_set)
        print("obs_cen_l_set", obs_cen_l_set)
        print("obs_length_set", obs_length_set)
        print("obs_width_set", obs_width_set)
        conflict_resolved = True  # 重置为 True，以确保下面的逻辑正确设置其值
        for pos in conflict_detected_positions:
            # 找到前后1米范围内的所有障碍物
            nearby_obstacles = []
            for obs_index in range(0, len(obs_cen_s_set)):
                obs_s = obs_cen_s_set[obs_index]
                if abs(dp_path_s_set[pos] - obs_s) <= 10:
                    nearby_obstacles.append(obs_index)

            min_index = conflict_detected_positions[0] - 1
            max_index = min(conflict_detected_positions[-1] + 15,49)
            for j in range(min_index, max_index + 1):
                l_lower_bound[j] = LOWER_BOUND
                l_upper_bound[j] = UPPER_BOUND

            # 计算左右绕的空间，考虑所有附近的障碍物
            left_space = min([l_upper_bound[pos] - (obs_cen_l_set[i] + obs_width_set[i] / 2 + HALF_WIDTH)
                              for i in nearby_obstacles], default=0)
            right_space = min([(obs_cen_l_set[i] - obs_width_set[i] / 2 - HALF_WIDTH) - l_lower_bound[pos]
                               for i in nearby_obstacles], default=0)

            # 选择空间更大的方向绕行
            if left_space >= right_space and left_space >= 0:
                for i in nearby_obstacles:
                    for offset in range(min_index, max_index + 1):  # 对前后0.5米的点也设置bound
                        if 0 <= offset < len(dp_path_s_set):
                            l_lower_bound[offset] = max(l_lower_bound[offset], obs_cen_l_set[i] + obs_width_set[i] / 2 + HALF_WIDTH)
            elif right_space > left_space and right_space >= 0:
                for i in nearby_obstacles:
                    for offset in range(min_index, max_index + 1):  # 对前后0.5米的点也设置bound
                        if 0 <= offset < len(dp_path_s_set):
                            l_upper_bound[offset] = min(l_upper_bound[offset], obs_cen_l_set[i] - obs_width_set[i] / 2 - HALF_WIDTH)
            else:
                print(f"Conflict could not be resolved at position {pos} with nearby obstacles.")
                conflict_resolved = False
                break

        print('Conflict resolved:', conflict_resolved)
    return l_upper_bound, l_lower_bound, conflict_resolved

def is_lane_change(l_upper_bound, l_lower_bound):
    lane_change = 0
    if np.sum(l_lower_bound >= 2.4) >= 3:
        lane_change = -1
    elif np.sum(l_upper_bound <= -2.4) >= 3:
        lane_change = 1
    print("obs_laneChange:",lane_change)
    return lane_change


# 查找障碍物最近的 s 坐标索引
def find_obs_nearest_s(obs_cen_s=None, obs_length=None,
                       dp_path_s_set=None):
    # 如果障碍物在规划起点的后面, 则不识别该障碍物
    if dp_path_s_set[0] > obs_cen_s + obs_length / 2:
        return None, None, None
    # 如果障碍物在规划路径末端的前方较远位置, 则不识别该障碍物
    elif dp_path_s_set[-1] < obs_cen_s - obs_length / 2:
        return None, None, None
    else:
        # 找障碍物起点s坐标的匹配的dp索引
        start_index = find_nearest_s(search_s=obs_cen_s - obs_length / 2, dp_path_s_set=dp_path_s_set)
        # 找障碍物中心点s坐标的匹配的dp索引
        cen_index = find_nearest_s(search_s=obs_cen_s, dp_path_s_set=dp_path_s_set)
        # 找障碍物终点s坐标的匹配的dp索引
        end_index = find_nearest_s(search_s=obs_cen_s + obs_length / 2, dp_path_s_set=dp_path_s_set)
        # 偏保守的估计, 起点匹配索引 - 1, 终点匹配索引 + 1
        if start_index > 0:
            start_index -= 1
        if end_index < len(dp_path_s_set) - 1:
            end_index += 1
        return start_index, cen_index, end_index

# 依据单个 s 坐标在 dp_path 中搜寻匹配点
def find_nearest_s(search_s=None, dp_path_s_set=None):
    index_res = None
    if dp_path_s_set[0] >= search_s:
        return 0
    elif dp_path_s_set[-1] <= search_s:
        return len(dp_path_s_set) - 1
    else:
        for i in range(0, len(dp_path_s_set)):
            if dp_path_s_set[i] < search_s:
                pass
            else:
                index_res = i
                break
        if np.abs(dp_path_s_set[index_res] - search_s) < np.abs(dp_path_s_set[index_res - 1] - search_s):
            return index_res
        else:
            return index_res - 1

if __name__ == '__main__':
    dp_path_s_set = np.arange(0, 50, 1)
    dp_path_l_set = [0] * 50
    obs_cen_s_set = [41.599976, 22.399963, 20.050003, 20.949997, 18.749985, 24.250015]
    obs_cen_l_set = [-0.8791735, 2.244957, -0.100157976, 1.1571515, -2.2677877, 2.76678]
    obs_length_set = [0.8, 0.8, 0.8, 0.8, 0.8, 0.8]
    obs_width_set = [0.8, 0.8, 0.8, 0.8, 0.8, 0.8]

    upper, lower, conflict = generate_convex_space(obs_cen_s_set, obs_cen_l_set, obs_length_set, obs_width_set)
    lane_change = is_lane_change(upper, lower)
    print(lane_change)
    plt.plot(dp_path_s_set, lower, "-*", color="red", label="Lower Bound")
    plt.plot(dp_path_s_set, upper, "-*", color="blue", label="Upper Bound")
    plt.legend()
    plt.show()