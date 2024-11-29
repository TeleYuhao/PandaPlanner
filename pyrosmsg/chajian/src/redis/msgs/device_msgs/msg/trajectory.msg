string timestamp
string globalTimeStamp
int32 frameId     
string id              # 
string name            #  参与者名称
string picLicense       #  车牌号
uint8 originalColor      #  原始颜色
uint8 vehicleColor      #  车辆颜色
uint8 vehicletype         # 车辆类型
int32 length              # 长度 cm
int32 width                #  宽度 cm
int32 height                #  高度 cm
uint8 driveType              #    驾驶类型
float64 longitude            #    经度
float64 latitude          #     纬度
float64 courseAngle         #    航向角
int32 speed                 # km/h    
float64 lonAcc              #纵向加速度  m/s2
float64 latAcc                #横向加速度 m/s2
float64 angularVelX           #横摆角速度
uint8 gear                   #档位状态  0：无效  1：R  2：N   3：D   4：P
float64 steerWheelAngle     #方向盘转转角（示例15.1deg）,单位：deg
float64 acceleratorPedal   # 油门踏板开度
float64 braking             # 制动踏板开度
uint8 obuStatus            #  5G/V2X_OBU通信状态；（是否正常：0异常；1正常）
uint8 locationStatus      # 定位状态（是否正常：0异常；1正常)
uint8 chassisStatus       #  底盘线控状态（是 否正常：0异常；1正常）
uint8 autoStatus         #  自动驾驶状态（是否正常：0异常；1正常)
uint8 indicatorStatus     #  转向灯状态（0：关闭 1：左转向 2：右转向）
uint8 blinkerStatus      #  双闪灯状态（0：关闭 1：打开）





