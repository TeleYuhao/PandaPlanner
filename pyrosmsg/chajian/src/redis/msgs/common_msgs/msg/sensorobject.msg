uint32  id                     #跟踪的ID
float32 x                      #横坐标
float32 y                      #纵坐标
float32 z                      #Z坐标
float64 longtitude             #经度
float64 latitude               #纬度
float64 altitude               #高度
float32 relspeedy              #纵轴相对速度
float32 relspeedx              #横轴相对速度
float32 rollRad                #横滚角 rad   
float32 pitchRad               #俯仰角 rad        
float32 azimuth                #航向角 rad
float64 pitchrate              #deg/s
float64 rollrate               #deg/s
float64 yawrate                #deg/s
float32 width                  #宽度
float32 length                 #长度
float32 height                 #高度
uint8   classification         #类别
uint8   value                  #Cluster 版本用于速度来源-radar 
float32 confidence             #检测置信度
point3d[] points               #轮廓点数据 
uint8  drivingIntent           #驾驶意图：0-初始，1-切入
uint8  behavior_state          # FORWARD_STATE = 0, STOPPING_STATE = 1, BRANCH_LEFT_STATE = 2, BRANCH_RIGHT_STATE = 3, YIELDING_STATE = 4, ACCELERATING_STATE = 5, SLOWDOWN_STATE = 6
uint8  radarIndex              #相对速度来源
uint8  radarObjectID           #radar跟踪目标ID
float32 s                      #frenet坐标系的s
float32 l                      #frenet坐标系的l
float32 speeds                 #frenet坐标系的s方向速度
float32 speedl                 #frenet坐标系的l方向速度
uint8   objectDecision         #障碍物相关决策
objecthistory[] objectHistory  #历史轨迹信息
objectprediction[] objectPrediction #预测信息
