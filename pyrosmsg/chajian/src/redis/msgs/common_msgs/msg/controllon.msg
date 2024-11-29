uint8    espmethod   #esp使能
float32  gpsDis         #输出量
uint8    epbmethod   #手刹使能
uint8    epb         #手刹使能
uint8    geermethod  #档位使能
uint8    gear        #档位
uint8    brakemethod #制动信号使能
float32    brakePedal
float32    gasPedal
uint8    station
uint8    light
uint8    pcmethod    #上位机使能
float32  objdis      #前方最危险目标距离
float32  objrel      #前方最危险目标的相对速度
uint8    mode        #模块模式
uint8    isvalid     #有效位
int64    timestamp   #时间戳   
float32  targetSpeed
float32  apaDis     # 期望加速度（国汽需要的）
uint8    objtype