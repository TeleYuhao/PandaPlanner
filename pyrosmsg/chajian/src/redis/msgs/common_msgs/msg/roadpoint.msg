float32 x                  #局部横坐标-carbackRFU-cm
float32 y                  #局部纵坐标-carbackRFU-cm
float64 gx                 #经度   
float64 gy                 #纬度
uint8 roadtype           #道路属性                   国汽 1  没有路口
float32 speed            # 速度       m/s    
float32 a                # 加速度     m/s^2
float32 jerk             # 冲击度     m/s^3
uint8 lanetype           #当前车道|总车道(4|4)          国汽   2.5m
uint8 turnlight
uint8 mergelanetype
uint8 sensorlanetype
float32 heading          # heading    deg
float32 curvature        # 道路曲率                   国汽 ---》curvature
float32 dkappa           # 曲率的一阶导数
float32 ddkappa          # 曲率的二阶导数
float32 leftsearchdis
float32 rightsearchdis
float64 s                # frenet坐标 m
float32 sideroadwidth
float32 lanewidth
float32 leftlanewidth
float32 rightlanewidth
float32 relativetime     # 相对时间  ms
uint8 laneswitch
uint8 laneborrow
uint8 lanenum
uint8 lanesite
