int32   ptc_type    # 0:未知 1：行人 2：小客车 3：巴士 4：货车 5：非机动车 100:僵尸车
int32   ptc_id      # 交通参与者id
int32   source      # 0:未知 1：自车 2：v2x 3:视频传感器 4：毫米波雷达 5：地磁线圈传感器  6:激光雷达 7：2类或以上感知数据的融合结果
string  source_id   # 对象来源ID
int32   sec_mark    # 时间戳
float64  pos_lon     # 位置经度
float64  pos_lat     # 位置纬度
float64  pos_latitude# m
float32 speed       # m/s
float32 heading     # 正北是0°，范围0-360
float32 accel       # m/s
float32 accel_angle # 加速度方向
float32 acc4way_lon # 四轴纵向加速度
float32 acc4way_lat # 四轴横向加速度
float32 acc4way_vert# 四轴垂直加速度
float32 acc4way_yaw # 四轴角速度
float32 width       # 单位m
float32 length      # 单位m
float32 height      # 单位m
float32 lon         # 经度
float32 lat         # 纬度
uint8 planlistNum    # 预测轨迹点list
oburoadlist[] roadlist #预测轨迹