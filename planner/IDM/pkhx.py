import copy
import math
import time


space = 0.5                             
bezier_half_order = 20                   
ishedm_dis = 0.02                      
min_hvig = 120
max_v2_degrees_v3 = 30                
pointnum_k = 10                  
max_acc = 1.0            

"""初始化的参数"""
lon0, lat0 = 0, 0
first = True
R_e = 6378137.0           
R_f = 6356752.314245     
e_1 = math.sqrt(pow(R_e, 2) - pow(R_f, 2)) / R_e   


def ll_to_xy(lon, lat):
    sin_lat1 = math.sin(math.radians(lat0))
    cos_lat1 = math.cos(math.radians(lat0))
    square_e = e_1 * e_1
    square_sin_lat1 = sin_lat1 * sin_lat1

    R_n = R_e / (math.sqrt(1 - square_e * square_sin_lat1))            
    R_m = (R_n * (1 - square_e)) / (1 - square_e * square_sin_lat1)     

    gx = math.radians(lon - lon0) * R_n * cos_lat1
    gy = math.radians(lat - lat0) * R_m
    return gx, gy


def xy_to_ll(x, y):
    sin_lat1 = math.sin(math.radians(lat0))
    cos_lat1 = math.cos(math.radians(lat0))
    square_e = e_1 * e_1
    square_sin_lat1 = sin_lat1 * sin_lat1

    R_n = R_e / (math.sqrt(1 - square_e * square_sin_lat1))             
    R_m = (R_n * (1 - square_e)) / (1 - square_e * square_sin_lat1)    

    lat = y / 3.1415927 * 180 / R_m + lat0
    lon = x / 3.1415927 * 180 / R_n / cos_lat1 + lon0

    return lon, lat


def Trans_angle_to_heading(angle):
    heading = 450 - angle
    while heading > 360 or heading < 0:
        if heading > 360:
            heading -= 360
        else:
            heading += 360
    return heading


def Cal_Vector_length(vx, vy):
    return (vx ** 2 + vy ** 2) ** 0.5


def binomialCoefficient(n, k):
    result = 1
    if k > n - k:
        k = n - k
    for i in range(k):
        result *= n - i
        result /= i + 1
    return result


def Vector_doc_product(v1x, v1y, v2x, v2y):
    return v1x * v2x + v1y * v2y


def Xianduan_to_Vector(p1x, p1y, p2x, p2y):
    return p2x - p1x, p2y - p1y


def Caldis(x1, x2, y1, y2):
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


def Get_nearest_index(px, py, maping_spacing):
    dis_near = 100
    nearest_index = -1
    for index, mi in enumerate(maping_spacing):
        dis_temp = Caldis(px, mi[0], py, mi[1])
        if dis_temp < dis_near:
            dis_near = dis_temp
            nearest_index = index
    return nearest_index


def Getpointnum(maping_spacing):
    global space
    dis_straight = 0
    x_last, y_last = maping_spacing[0][0], maping_spacing[0][1]
    for pointi in maping_spacing:
        dis_straight += Caldis(pointi[0], x_last, pointi[1], y_last)
        x_last, y_last = pointi[0], pointi[1]
    # print("dis = {}, space = {}".format(dis_straight, space))
    numf = int(pointnum_k * dis_straight / space) + 1
    # print("起终点折线距离 = {}，初始点数 = {}".format(dis_straight, numf))
    return numf


def Calangle(x1, x2, y1, y2):
    if x1 == x2:
        if y2 >= y1:
            angle = 90
        else:
            angle = -90
    else:
        angle = math.atan((y2 - y1) / (x2 - x1))
        angle = math.degrees(angle)
        if x2 < x1:
            if y2 < y1:
                angle = angle - 180
            else:
                angle = angle + 180

    return angle


def cal_all_heading(points):
    len_points = len(points)
    for i in range(len_points):
        if i == 0:
            angle = Calangle(points[0][0], points[1][0], points[0][1], points[1][1])
        elif i == len_points - 1:
            angle = Calangle(points[i-1][0], points[i][0], points[i-1][1], points[i][1])
        else:
            angle = Calangle(points[i-1][0], points[i+1][0], points[i-1][1], points[i+1][1])
        hi = Trans_angle_to_heading(angle)
        # points[i].append(hi)
        points[i].append(angle)
    return points


def bezier_curve(maping_spacing, point_num):
    points = []
    if len(maping_spacing) == 0:
        return points
    elif len(maping_spacing) == 1:
        points.append([maping_spacing[0][0], maping_spacing[0][1]])
        return points

    n = len(maping_spacing) - 1
    step = 1.0 / (point_num - 1)

    for ti in range(point_num):
        maping_ks = copy.deepcopy(maping_spacing[-1])
        t = ti * step
        x_temp, y_temp = 0, 0
        for i in range(n + 1):
            coefficient = binomialCoefficient(n, i) * pow(1 - t, n - i) * pow(t, i)
            x_temp = x_temp + maping_spacing[i][0] * coefficient
            y_temp = y_temp + maping_spacing[i][1] * coefficient
        maping_ks[0], maping_ks[1] = x_temp, y_temp
        points.append(maping_ks)
    return points


def point3s_to_question(point1, point2, point3):
    question_states = []
    v1x, v1y = Xianduan_to_Vector(point1[0], point1[1], point3[0], point3[1])
    v2x, v2y = Xianduan_to_Vector(point1[0], point1[1], point2[0], point2[1])
    v3x, v3y = Xianduan_to_Vector(point2[0], point2[1], point3[0], point3[1])
    v1_doc_v2 = Vector_doc_product(v1x, v1y, v2x, v2y)
    v2_doc_v3 = Vector_doc_product(v2x, v2y, v3x, v3y)
    if Cal_Vector_length(v2x, v2y) <= ishedm_dis:
        return 0, 0, 0, 0, [1.2]
    elif Cal_Vector_length(v1x, v1y) <= ishedm_dis or Cal_Vector_length(v3x, v3y) <= ishedm_dis:
        return 0, 0, 0, 0, [1.3]
    v1_radians_v2 = math.acos(round(v1_doc_v2 / (Cal_Vector_length(v1x, v1y) * Cal_Vector_length(v2x, v2y)), 8))
    v2_radians_v3 = math.acos(round(v2_doc_v3 / (Cal_Vector_length(v2x, v2y) * Cal_Vector_length(v3x, v3y)), 8))
    v2_degrees_v3 = math.degrees(v2_radians_v3)
    if v2_degrees_v3 > min_hvig:
        question_states.append(2)
    elif v2_degrees_v3 > max_v2_degrees_v3:        # 角度的抖动
        question_states.append(3)
    dis_point2_to_v1 = Cal_Vector_length(v2x, v2y) * math.sin(v1_radians_v2)
    dis_point2_to_point3 = Caldis(point2[0], point3[0], point2[1], point3[1])
    v1_radians_xaxis = math.acos(Vector_doc_product(v1x, v1y, 1, 0) / Cal_Vector_length(v1x, v1y))
    v1_degrees_xaxis = math.degrees(v1_radians_xaxis)
    v1_degrees_xaxis = v1_degrees_xaxis if v1y >= 0 else v1_degrees_xaxis * (-1)
    v1_heading = Trans_angle_to_heading(v1_degrees_xaxis)
    detal_heading_point2h_to_point13h = 0
    return v2_degrees_v3, dis_point2_to_v1, dis_point2_to_point3, detal_heading_point2h_to_point13h, question_states


def cal_start_end_index(index_now, distance, maping):
    global space
    temp_index = index_now
    start_dis_temp, end_dis_temp = 0, 0
    # 先向前找距离当前点distance的index
    while temp_index > 0 and start_dis_temp < distance:
        # 开始点的距离递加
        start_dis_temp += Caldis(maping[temp_index][0], maping[temp_index - 1][0], maping[temp_index][1], maping[temp_index - 1][1])
        # 向起始方向步进索引
        temp_index -= 1
    start_index = temp_index

    # 如果前向距离不够，增加后向距离
    end_distance_target = distance
    temp_index = index_now
    if start_dis_temp < distance:
        end_distance_target += distance - start_dis_temp
    # 向后寻找距离当前点end_distance_target的index
    max_maping_index = len(maping) - 1
    while temp_index < max_maping_index - 1 and end_dis_temp < end_distance_target:
        # 终止点的距离递加
        end_dis_temp += Caldis(maping[temp_index][0], maping[temp_index + 1][0], maping[temp_index][1], maping[temp_index + 1][1])
        # 向终止方向步进索引
        temp_index += 1
    end_index = temp_index

    return start_index, end_index


def cal_start_index(distance, maping):
    global space
    temp_index = len(maping) - 1
    start_dis_temp = 0
    # 先向前找距离当前点distance的index
    while temp_index > 0 and start_dis_temp < distance:
        # 开始点的距离递加
        start_dis_temp += Caldis(maping[temp_index][0], maping[temp_index - 1][0], maping[temp_index][1], maping[temp_index - 1][1])
        # 向起始方向步进索引
        temp_index -= 1
    start_index = temp_index

    return start_index


def cal_end_index(index_now, distance, maping):
    global space
    end_dis_temp = 0
    # 如果前向距离不够，增加后向距离
    end_distance_target = distance
    temp_index = index_now
    # 向后寻找距离当前点end_distance_target的index
    max_maping_index = len(maping) - 1
    while temp_index < max_maping_index and end_dis_temp < end_distance_target:
        # 终止点的距离递加
        end_dis_temp += Caldis(maping[temp_index][0], maping[temp_index + 1][0], maping[temp_index][1], maping[temp_index + 1][1])
        # 向终止方向步进索引
        temp_index += 1
    end_index = temp_index

    return end_index


def Handle(maping1):
    i = 1
    while i < len(maping1) - 2:
        point1 = [maping1[i - 1][0], maping1[i - 1][1]]
        point2 = [maping1[i][0], maping1[i][1]]
        point3 = [maping1[i + 1][0], maping1[i + 1][1]]
        result_temp = point3s_to_question(point1, point2, point3)
        if result_temp[-1]:
            if min(result_temp[-1]) == 1.2:
                maping1.pop(i)
            elif min(result_temp[-1]) == 1.3:
                maping1.pop(i + 1)
            elif min(result_temp[-1]) == 2:
                maping1.pop(i)
                i -= 1
            elif min(result_temp[-1]) == 3:
                maping1.pop(i + 1)
        else:
            i += 1
    return maping1


def test(map_begin, grjm, old_line):
    global lon0, lat0, first
    maping0 = map_begin
    for map0_i in maping0:
        if first == True:
            first = False
            lon0, lat0 = copy.deepcopy(map0_i[0]), copy.deepcopy(map0_i[1])
            map0_i[0], map0_i[1] = 0, 0
        else:
            map0_i[0], map0_i[1] = ll_to_xy(map0_i[0], map0_i[1])

    maping1 = copy.deepcopy(maping0)

    maping1 = Handle(maping1)

    maping2 = cal_all_heading(maping1)

    for grjm_i in grjm:
        grjm_i [0], grjm_i[1] = ll_to_xy(grjm_i[0], grjm_i[1])
    for old_i in old_line:
        old_i [0], old_i[1] = ll_to_xy(old_i[6], old_i[7])

    for map2_i in maping2:
        map2_i[0], map2_i[1] = xy_to_ll(map2_i[0], map2_i[1])
        if len(map2_i) < 7:
            map2_i.append(0)
    return maping2

